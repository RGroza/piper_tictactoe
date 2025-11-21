#include <board_perception/srv/process_board.hpp>
#include <piper_trajectory/srv/execute_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include "move_manager/srv/play_move.hpp"

using namespace std;
using namespace std::chrono_literals;

class MoveManager : public rclcpp::Node {
  public:
    MoveManager() : Node("ttt_move_manager") {
        service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        client_callback_group_  = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        play_move_service_ = this->create_service<move_manager::srv::PlayMove>(
            "play_move", std::bind(&MoveManager::handleService, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, service_callback_group_);

        board_processor_client_ = this->create_client<board_perception::srv::ProcessBoard>(
            "process_board", rmw_qos_profile_services_default, client_callback_group_);

        trajectory_client_ = this->create_client<piper_trajectory::srv::ExecuteTrajectory>(
            "execute_trajectory", rmw_qos_profile_services_default, client_callback_group_);
    }

  private:
    void handleService(const std::shared_ptr<move_manager::srv::PlayMove::Request> request,
                       std::shared_ptr<move_manager::srv::PlayMove::Response> response) {
        RCLCPP_INFO(get_logger(), "Move request received (mode=%d)", request->mode);
        // Request current board state
        auto board_future = board_processor_client_->async_send_request(
            std::make_shared<board_perception::srv::ProcessBoard::Request>());

        if (board_future.wait_for(2s) != std::future_status::ready) {
            RCLCPP_ERROR(get_logger(), "process_board timed out");
            serviceFailure(response, "Board processing did not respond");
            return;
        }

        auto board_response = board_future.get();

        if (!board_response->success) {
            serviceFailure(response, "Board processing failed");
            return;
        }

        // Print board state
        RCLCPP_INFO(get_logger(), "Current board state:");
        printBoard(board_response->board);

        int cell;
        if (request->mode == 0) {
            // Check human move is valid
            if (request->cell_number < 1 || request->cell_number > 9) {
                serviceFailure(response, "Invalid human move: cell number out of range");
                return;
            }
            if (board_response->board[request->cell_number - 1] != -1) {
                serviceFailure(response, "Invalid human move: cell already occupied");
                return;
            }
            cell = request->cell_number;
            RCLCPP_INFO(get_logger(), "Human move on cell %d", cell);
        } else {
            // Compute best robot move
            cell = this->bestMove(board_response->board, request->symbol) + 1;
            RCLCPP_INFO(this->get_logger(), "Robot best move on cell %d", cell);
        }

        // Execute trajectory
        auto trajectory_request         = std::make_shared<piper_trajectory::srv::ExecuteTrajectory::Request>();
        trajectory_request->type        = request->symbol;
        trajectory_request->cell_number = cell;
        trajectory_request->return_home = true;

        auto trajectory_future = trajectory_client_->async_send_request(trajectory_request);

        // Print board state after move
        for (int i = 0; i < 9; i++)
            response->board[i] = board_response->board[i];
        response->board[cell - 1] = request->symbol;
        RCLCPP_INFO(get_logger(), "Board state after requested move:");
        printBoard(response->board);

        if (trajectory_future.wait_for(30s) != std::future_status::ready) {
            serviceFailure(response, "Trajectory execution timeout out");
            return;
        }

        auto trajectory_response = trajectory_future.get();

        if (!trajectory_response->success) {
            serviceFailure(response, "Trajectory execution failed");
            return;
        }

        response->message = "Move executed successfully";
        RCLCPP_INFO(get_logger(), response->message.c_str());
        response->success = true;
    }

    void serviceFailure(std::shared_ptr<move_manager::srv::PlayMove::Response> response, const std::string& msg) {
        response->message = msg;
        RCLCPP_ERROR(get_logger(), response->message.c_str());
        response->board.fill(-1);
        response->success = false;
    }

    // -------------------------
    // Tic-Tac-Toe Minimax Logic
    // -------------------------

    int bestMove(array<int, 9> flat_board, int robot_symbol) {
        // Convert 1D array → 3×3 grid
        vector<vector<int>> board(3, vector<int>(3, -1));
        for (int i = 0; i < 9; i++)
            board[i / 3][i % 3] = flat_board[i];

        int best_score = numeric_limits<int>::lowest();
        int best_cell  = -1;

        // Try all free cells
        for (int i = 0; i < 9; i++) {
            if (flat_board[i] == -1) {
                board[i / 3][i % 3] = robot_symbol;

                int score = minimax(board, 0, false, robot_symbol);

                board[i / 3][i % 3] = -1; // undo move

                if (score > best_score) {
                    best_score = score;
                    best_cell  = i; // return cell number
                }
            }
        }

        return best_cell;
    }

    int checkWinner(const vector<vector<int>>& b) {
        // Rows
        for (int i = 0; i < 3; i++)
            if (b[i][0] != -1 && b[i][0] == b[i][1] && b[i][1] == b[i][2])
                return b[i][0];

        // Columns
        for (int j = 0; j < 3; j++)
            if (b[0][j] != -1 && b[0][j] == b[1][j] && b[1][j] == b[2][j])
                return b[0][j];

        // Diagonal (\)
        if (b[0][0] != -1 && b[0][0] == b[1][1] && b[1][1] == b[2][2])
            return b[0][0];

        // Diagonal (/)
        if (b[0][2] != -1 && b[0][2] == b[1][1] && b[1][1] == b[2][0])
            return b[0][2];

        return -1; // no winner
    }

    bool isBoardFull(const vector<vector<int>>& b) {
        for (const auto& row : b)
            for (int c : row)
                if (c == -1)
                    return false;
        return true;
    }

    int minimax(vector<vector<int>>& b, int depth, bool is_maximizing, int robot_symbol) {
        int winner       = checkWinner(b);
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

    void printBoard(array<int, 9> board) {
        for (int i = 0; i < 9; i++) {
            char symbol;
            if (board[i] == -1)
                symbol = '.';
            else if (board[i] == 0)
                symbol = 'O';
            else
                symbol = 'X';

            cout << symbol << " ";
            if ((i + 1) % 3 == 0)
                cout << endl;
            else
                cout << "| ";
        }
    }

    rclcpp::CallbackGroup::SharedPtr service_callback_group_;
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;
    rclcpp::Client<board_perception::srv::ProcessBoard>::SharedPtr board_processor_client_;
    rclcpp::Client<piper_trajectory::srv::ExecuteTrajectory>::SharedPtr trajectory_client_;
    rclcpp::Service<move_manager::srv::PlayMove>::SharedPtr play_move_service_;
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