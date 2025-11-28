#include <board_perception/srv/process_board.hpp>
#include <piper_trajectory/srv/execute_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include "board_perception/msg/board_state.hpp"
#include "move_manager/msg/game_result.hpp"
#include "move_manager/srv/play_move.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std;
using namespace std::chrono_literals;
using Board = array<array<int, 3>, 3>;

class MoveManager : public rclcpp::Node {
  public:
    MoveManager() : Node("ttt_manager") {
        service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        client_callback_group_  = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        play_move_service_ = this->create_service<move_manager::srv::PlayMove>(
            "play_move", std::bind(&MoveManager::handleService, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, service_callback_group_);

        trajectory_client_ = this->create_client<piper_trajectory::srv::ExecuteTrajectory>(
            "execute_trajectory", rmw_qos_profile_services_default, client_callback_group_);

        board_state_subscriber_ = this->create_subscription<board_perception::msg::BoardState>(
            "/board_state", 10, std::bind(&MoveManager::boardStateCallback, this, std::placeholders::_1));

        game_result_publisher_ = this->create_publisher<move_manager::msg::GameResult>("game_result", 10);

        this->declare_parameter<int>("robot_symbol", 0); // 0 for 'O', 1 for 'X'
        robot_symbol_ = this->get_parameter("robot_symbol").as_int();
        this->declare_parameter<bool>("auto_robot_move", true);
        auto_robot_move_ = this->get_parameter("auto_robot_move").as_bool();
        this->declare_parameter<bool>("game_started", true);
        game_started_ = this->get_parameter("game_started").as_bool();

        param_callback_handle_ =
            this->add_on_set_parameters_callback(std::bind(&MoveManager::onSetParameters, this, std::placeholders::_1));

        board_state_          = {{{-1, -1, -1}, {-1, -1, -1}, {-1, -1, -1}}};
        board_state_received_ = false;
        new_state_count_ = 0;

        auto_robot_move_ = true;
        robot_moving_    = false;
        robot_turn_      = false;
    }

  private:
    void boardStateCallback(const board_perception::msg::BoardState::SharedPtr msg) {
        if (msg->board.size() != 9) {
            RCLCPP_WARN(get_logger(), "Received board state of invalid size");
            return;
        }
        if (!msg->success)
            return;
        if (robot_moving_)
            return;

        if (msg->board_detected)
            board_state_received_ = true;

        // Convert flat board to 2D grid
        Board new_board_state;
        for (int i = 0; i < 9; ++i)
            new_board_state[i / 3][i % 3] = msg->board[i];

        // Check if there is any change in the board state
        bool new_state = false;
        bool new_state_confirmed = false;
        for (size_t i = 0; i < 3; ++i)
            for (size_t j = 0; j < 3; ++j)
                if (board_state_[i][j] != new_board_state[i][j] && board_state_[i][j] == -1) {
                    // Require several new states in a row to trigger new move
                    if (new_state_count_ < 5) {
                        new_state_count_++;
                        new_state = true;
                    } else {
                        new_state_count_ = 0;
                        new_state_confirmed = true;
                        RCLCPP_INFO(get_logger(), "New move detected at cell %zu", i * 3 + j + 1);
                        break; // Assume only one move can be made at a time
                    }
                }

        if (!new_state)
            new_state_count_ = 0;
        if (!new_state_confirmed)
            return; // No new move detected

        // Update the stored board state
        board_state_ = new_board_state;

        if (!robot_turn_)
            robot_turn_ = true;

        robotAutoMove();

        // Check for winner or draw
        pair<int, pair<int, int>> winner = checkWinner(new_board_state);
        if (winner.first != -1) {
            RCLCPP_INFO(get_logger(), "Game over! Winner: Player %d", winner.first + 1);
            auto game_result              = move_manager::msg::GameResult();
            game_result.game_over         = true;
            game_result.winner            = winner.first;
            game_result.winner_start_cell = winner.second.first;
            game_result.winner_end_cell   = winner.second.second;
            game_result_publisher_->publish(game_result);
        } else if (isBoardFull(new_board_state)) {
            RCLCPP_INFO(get_logger(), "Game over! It's a draw.");
            auto game_result      = move_manager::msg::GameResult();
            game_result.game_over = true;
            game_result.winner    = -1; // Indicate draw
            game_result_publisher_->publish(game_result);
        }
    }

    void handleService(const std::shared_ptr<move_manager::srv::PlayMove::Request> request,
                       std::shared_ptr<move_manager::srv::PlayMove::Response> response) {
        RCLCPP_INFO(get_logger(), "Move request received (mode=%d)", request->mode);
        if (robot_moving_) {
            serviceFailure(response, "Robot is currently moving, please wait");
            return;
        }

        if (request->mode == 0) {
            // Handle human move: validate cell number and occupancy
            int cell = request->cell_number;
            if (cell < 1 || cell > 9) {
                serviceFailure(response, "Invalid human move: cell number out of range");
                return;
            }
            if (board_state_[(cell - 1) / 3][(cell - 1) % 3] != -1) {
                serviceFailure(response, "Invalid human move: cell already occupied");
                return;
            }
            // Check if requested symbol is opposite of robot's symbol
            if (request->symbol == robot_symbol_) {
                serviceFailure(response, "Invalid human move: symbol matches robot's symbol");
                return;
            }
            // Validate board state
            pair<bool, string> validateResult = validateBoard(request->symbol);
            if (validateResult.first == false) {
                serviceFailure(response, validateResult.second);
                return;
            }
            // Ensure it's human's turn
            if (robot_turn_) {
                serviceFailure(response, "It's not human's turn to move");
                return;
            }

            // Execute human move
            RCLCPP_INFO(get_logger(), "Human move on cell %d", cell);
            auto result = moveRequest(request->symbol, cell);

            if (!result.first) {
                serviceFailure(response, result.second);
                return;
            }

            robot_turn_ = true;
        } else {
            // Ensure it's robot's turn
            if (!robot_turn_) {
                serviceFailure(response, "It's not robot's turn to move");
                return;
            }

            // Handle robot move
            auto result = robotMove();

            if (!result.first) {
                serviceFailure(response, result.second);
                return;
            }
        }

        // Set success message and response
        response->message = "Move executed successfully";
        RCLCPP_INFO(get_logger(), response->message.c_str());
        response->success = true;
    }

    void serviceFailure(std::shared_ptr<move_manager::srv::PlayMove::Response> response, const std::string& msg) {
        response->message = msg;
        RCLCPP_WARN(get_logger(), response->message.c_str());
        response->success = false;
    }

    pair<bool, string> moveRequest(int symbol, int cell) {
        robot_moving_ = true;

        // Prepare and send trajectory execution request
        auto trajectory_request         = std::make_shared<piper_trajectory::srv::ExecuteTrajectory::Request>();
        trajectory_request->type        = symbol;
        trajectory_request->cell_number = cell;
        trajectory_request->return_home = true;

        auto trajectory_future = trajectory_client_->async_send_request(trajectory_request);

        // Wait for trajectory execution and handle timeout/failure
        if (trajectory_future.wait_for(30s) != std::future_status::ready)
            return {false, "Trajectory execution timeout"};

        auto trajectory_response = trajectory_future.get();

        if (!trajectory_response->success)
            return {false, "Trajectory execution failed"};

        robot_moving_ = false;
        return {true, ""};
    }

    pair<bool, string> robotMove() {
        pair<bool, string> validateResult = validateBoard(robot_symbol_);
        if (validateResult.first == false)
            return {false, validateResult.second};

        // Handle robot move: compute best move using minimax
        int cell = bestMove(board_state_, robot_symbol_) + 1;
        if (cell < 1 || cell > 9)
            return {false, "Robot best move not found"};

        RCLCPP_INFO(this->get_logger(), "Robot best move on cell %d", cell);

        robot_turn_ = false;
        return moveRequest(robot_symbol_, cell);
    }

    void robotAutoMove() {
        // Auto-play robot move if it's robot's turn
        RCLCPP_INFO(get_logger(), "auto_robot_move_: %s, robot_moving_: %s, robot_turn_: %s",
                    auto_robot_move_ ? "true" : "false", robot_moving_ ? "true" : "false",
                    robot_turn_ ? "true" : "false");

        if (auto_robot_move_ && !robot_moving_ && robot_turn_) {
            RCLCPP_INFO(get_logger(), "Robot playing move automatically");
            pair<bool, string> robot_move_result = robotMove();
            if (robot_move_result.first == false) {
                RCLCPP_WARN(get_logger(), "Robot move failed -- %s", robot_move_result.second.c_str());
            }
        }
    }

    pair<bool, string> validateBoard(int symbol) {
        // Use the latest board state from the subscriber
        if (!board_state_received_)
            return {false, "No board state received yet"};

        RCLCPP_INFO(get_logger(), "Current board state:");
        printBoard(board_state_);

        // Validate board state
        if (isBoardFull(board_state_))
            return {false, "Board is already full"};
        if (!isBoardValid(board_state_, symbol))
            return {false, "Current board state is invalid for the given symbol"};
        if (checkWinner(board_state_).first != -1)
            return {false, "Game is already over"};

        return {true, ""};
    }

    rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason     = "success";

        for (const auto& param : params) {
            if (param.get_name() == "robot_symbol") {
                int val = param.as_int();
                if (val == 0 || val == 1) {
                    robot_symbol_ = val;
                    RCLCPP_INFO(this->get_logger(), "robot_symbol updated to %d", robot_symbol_);
                } else {
                    result.successful = false;
                    result.reason     = "robot_symbol must be 0 or 1";
                }
            }
            if (param.get_name() == "auto_robot_move") {
                auto_robot_move_ = param.as_bool();
                RCLCPP_INFO(this->get_logger(), "auto_robot_move updated to %s", auto_robot_move_ ? "true" : "false");
            }
            if (param.get_name() == "game_started") {
                game_started_ = param.as_bool();
                RCLCPP_INFO(this->get_logger(), "game_started updated to %s", game_started_ ? "true" : "false");
                if (game_started_) {
                    // Reset board state
                    board_state_ = {{{-1, -1, -1}, {-1, -1, -1}, {-1, -1, -1}}};

                    if (robot_symbol_ == 1)
                        robot_turn_ = true;
                    else
                        robot_turn_ = false;

                    robotAutoMove();
                }
            }
        }
        return result;
    }

    // -------------------------
    // Tic-Tac-Toe Minimax Logic
    // -------------------------

    int bestMove(Board board, int robot_symbol) {
        int best_score = numeric_limits<int>::lowest();
        int best_cell  = -1;

        // Check if the board is empty
        bool empty = true;
        for (int i = 0; i < 3 && empty; ++i)
            for (int j = 0; j < 3 && empty; ++j)
                if (board[i][j] > -1)
                    empty = false;

        if (empty)
            return 4; // return middle cell if empty

        // Try all free cells
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (board[i][j] == -1) {
                    board[i][j] = robot_symbol;

                    int score = minimax(board, 0, false, robot_symbol);

                    board[i][j] = -1; // undo move

                    if (score > best_score) {
                        best_score = score;
                        best_cell  = 3 * i + j; // return cell number
                    }
                }
            }
        }

        return best_cell;
    }

    int minimax(Board& b, int depth, bool is_maximizing, int robot_symbol) {
        int winner       = checkWinner(b).first;
        int human_symbol = (robot_symbol == 0 ? 1 : 0);

        // Depth-aware scoring
        if (winner == robot_symbol)
            return 10 - depth; // prefer quicker wins
        else if (winner == human_symbol)
            return depth - 10; // prefer slower losses (if forced)
        else if (isBoardFull(b))
            return 0; // draw

        if (is_maximizing) {
            int best_score = numeric_limits<int>::lowest();

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    if (b[i][j] == -1) {
                        b[i][j]    = robot_symbol;
                        int score  = minimax(b, depth + 1, false, robot_symbol);
                        b[i][j]    = -1;
                        best_score = max(best_score, score);
                    }
                }
            }

            return best_score;

        } else {
            int best_score = numeric_limits<int>::max();

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    if (b[i][j] == -1) {
                        b[i][j]    = human_symbol;
                        int score  = minimax(b, depth + 1, true, robot_symbol);
                        b[i][j]    = -1;
                        best_score = min(best_score, score);
                    }
                }
            }

            return best_score;
        }
    }

    pair<int, pair<int, int>> checkWinner(const Board& b) {
        // Rows
        for (int i = 0; i < 3; i++)
            if (b[i][0] != -1 && b[i][0] == b[i][1] && b[i][1] == b[i][2])
                return {b[i][0], {3 * i, 3 * i + 2}};

        // Columns
        for (int j = 0; j < 3; j++)
            if (b[0][j] != -1 && b[0][j] == b[1][j] && b[1][j] == b[2][j])
                return {b[0][j], {j, 6 + j}};

        // Diagonal (\)
        if (b[0][0] != -1 && b[0][0] == b[1][1] && b[1][1] == b[2][2])
            return {b[0][0], {0, 8}};
        // Diagonal (/)
        if (b[0][2] != -1 && b[0][2] == b[1][1] && b[1][1] == b[2][0])
            return {b[0][2], {2, 6}};

        return {-1, {-1, -1}}; // no winner
    }

    bool isBoardFull(const Board& board) {
        for (const auto& row : board)
            for (int c : row)
                if (c == -1)
                    return false;
        return true;
    }

    bool isBoardValid(const Board& board, int my_symbol) {
        int other_symbol       = my_symbol == 0 ? 1 : 0;
        int my_symbol_count    = 0;
        int other_symbol_count = 0;

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                if (board[i][j] == my_symbol)
                    my_symbol_count++;
                else if (board[i][j] == other_symbol)
                    other_symbol_count++;

        return (my_symbol_count == other_symbol_count) || (my_symbol_count == other_symbol_count - 1);
    }

    void printBoard(const Board& board) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                char symbol;
                if (board[i][j] == -1)
                    symbol = '.';
                else if (board[i][j] == 0)
                    symbol = 'O';
                else
                    symbol = 'X';

                cout << symbol;
                if (j < 2)
                    cout << " | ";
            }
            cout << endl;
            if (i < 2)
                cout << "--+---+--" << endl;
        }
    }

    rclcpp::CallbackGroup::SharedPtr service_callback_group_;
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;
    rclcpp::Client<piper_trajectory::srv::ExecuteTrajectory>::SharedPtr trajectory_client_;
    rclcpp::Service<move_manager::srv::PlayMove>::SharedPtr play_move_service_;
    rclcpp::Subscription<board_perception::msg::BoardState>::SharedPtr board_state_subscriber_;
    rclcpp::Publisher<move_manager::msg::GameResult>::SharedPtr game_result_publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    Board board_state_;
    bool board_state_received_;
    bool auto_robot_move_;
    bool robot_moving_;
    bool robot_turn_;
    bool game_started_;
    int new_state_count_;
    int robot_symbol_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveManager>();

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}