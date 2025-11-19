#include <board_perception/srv/process_board.hpp>
#include <piper_trajectory/srv/execute_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std;
using namespace std::chrono_literals;

class TicTacToeManager : public rclcpp::Node {
  public:
    TicTacToeManager() : Node("ttt_manager") {
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        board_processor_client_ = this->create_client<board_perception::srv::ProcessBoard>(
            "process_board", rmw_qos_profile_services_default, callback_group_);

        trajectory_client_ = this->create_client<piper_trajectory::srv::ExecuteTrajectory>(
            "execute_trajectory", rmw_qos_profile_services_default, callback_group_);

        timer_ = this->create_wall_timer(2s, std::bind(&TicTacToeManager::timerCallback, this), callback_group_);

        robot_symbol_ = 0; // 0 for O, 1 for X
    }

  private:
    void timerCallback() {
        if (trajectory_in_progress_) {
            RCLCPP_INFO(get_logger(), "Waiting for trajectory to finish...");
            return;
        }

        if (!board_processor_client_->wait_for_service(1s) || !trajectory_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for servers...");
            return;
        }

        auto board_request = std::make_shared<board_perception::srv::ProcessBoard::Request>();

        auto board_future = board_processor_client_->async_send_request(board_request);
        RCLCPP_INFO(this->get_logger(), "Request sent to process board");

        if (board_future.wait_for(1s) != std::future_status::ready) {
            RCLCPP_WARN(this->get_logger(), "Timed out waiting for process_board response");
            return;
        }

        auto board_response = board_future.get();

        if (board_response->success) {
            int cell_number = bestMove(board_response->results, robot_symbol_);

            auto trajectory_request         = std::make_shared<piper_trajectory::srv::ExecuteTrajectory::Request>();
            trajectory_request->type        = robot_symbol_;
            trajectory_request->cell_number = cell_number;
            trajectory_request->return_home = true;

            auto trajectory_future = trajectory_client_->async_send_request(trajectory_request);
            RCLCPP_INFO(this->get_logger(), "Request sent to execute trajectory");
            trajectory_in_progress_ = true;
            trajectory_future.wait();

            if (trajectory_future.wait_for(20s) != std::future_status::ready) {
                RCLCPP_WARN(this->get_logger(), "Timed out waiting for execute_trajectory response");
                return;
            }

            auto trajectory_response = trajectory_future.get();

            if (trajectory_response->success) {
                RCLCPP_INFO(this->get_logger(), "Trajectory executed successfully");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to plan and execute trajectory");
            }

            trajectory_in_progress_ = false;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to process board");
        }
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
                    best_cell  = i + 1; // return cell number
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
        int winner = checkWinner(b);

        // Scoring relative to robot's symbol
        if (winner == robot_symbol)
            return 1;
        else if (winner != -1)
            return -1;

        if (isBoardFull(b))
            return 0;

        int human_symbol = (robot_symbol == 0 ? 1 : 0);

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

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Client<board_perception::srv::ProcessBoard>::SharedPtr board_processor_client_;
    rclcpp::Client<piper_trajectory::srv::ExecuteTrajectory>::SharedPtr trajectory_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    int robot_symbol_;
    bool trajectory_in_progress_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TicTacToeManager>();

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}