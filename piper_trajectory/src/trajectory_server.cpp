#include "piper_trajectory/trajectory_server.hpp"

using std::placeholders::_1;

TrajectoryServer::TrajectoryServer()
    : Node("ttt_trajectory_server"), trajectory_executor_(std::make_shared<rclcpp::Node>("ttt_trajectory_executor")) {
    srv_ = this->create_service<piper_trajectory::srv::ExecuteTrajectory>(
        "execute_trajectory",
        std::bind(&TrajectoryServer::handleService, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "TrajectoryServer node started");
}

void TrajectoryServer::handleService(const std::shared_ptr<piper_trajectory::srv::ExecuteTrajectory::Request> request,
                                     std::shared_ptr<piper_trajectory::srv::ExecuteTrajectory::Response> response) {
    switch (request->type) {
        case 0:
            RCLCPP_INFO(this->get_logger(), "Drawing O in cell %d", request->cell_number);
            trajectory_executor_.drawCircle(request->cell_number);
            break;
        case 1:
            RCLCPP_INFO(this->get_logger(), "Drawing X in cell %d", request->cell_number);
            trajectory_executor_.drawCross(request->cell_number);
            break;
        case 2:
            RCLCPP_INFO(this->get_logger(), "Drawing grid");
            trajectory_executor_.drawGrid();
            break;
        case 3:
            RCLCPP_INFO(this->get_logger(), "Clearing board");
            trajectory_executor_.moveToClear();
            break;
        case 4:
            RCLCPP_INFO(this->get_logger(), "Draw full board");
            trajectory_executor_.drawFullBoard();
            break;
        case 5:
            RCLCPP_INFO(this->get_logger(), "Returning to home position");
            trajectory_executor_.moveToHome();
            response->success = true;
            return;
        case 6:
            RCLCPP_INFO(this->get_logger(), "Drawing contour -- received %zu points, image dimensions: %dx%d",
                        request->contour_points.size() / 2, request->image_dimensions[0], request->image_dimensions[1]);
            trajectory_executor_.drawContour(request->image_dimensions, request->contour_points);
            response->success = true;
            return;
        default:
            RCLCPP_WARN(this->get_logger(), "Invalid type %d received", request->type);
            response->success = false;
            return;
    }

    if (request->return_home) {
        RCLCPP_INFO(this->get_logger(), "Returning to home position");
        trajectory_executor_.moveToHome();
    }

    response->success = true;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryServer>());
    rclcpp::shutdown();
    return 0;
}
