#include "board_perception/board_processor_server.hpp"
#include <cv_bridge/cv_bridge.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

using std::placeholders::_1;

BoardProcessorServer::BoardProcessorServer() : Node("ttt_board_processor_server"), processor_(this, true) {
    image_name_ = this->declare_parameter<std::string>("image_name", "");

    if (!image_name_.empty()) {
        std::string share_dir = ament_index_cpp::get_package_share_directory("board_perception");
        image_path_           = share_dir + "/images/" + image_name_;
        use_image_file_       = true;
        RCLCPP_INFO(this->get_logger(), "Using image file: %s", image_path_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Using image from camera topic");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/D435/color/image_raw", qos, std::bind(&BoardProcessorServer::imageCallback, this, _1));

    srv_ = this->create_service<board_perception::srv::ProcessBoard>(
        "process_board",
        std::bind(&BoardProcessorServer::handleService, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "BoardProcessorServer node started");
}

void BoardProcessorServer::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    latest_image_ = msg;
}

void BoardProcessorServer::handleService(const std::shared_ptr<board_perception::srv::ProcessBoard::Request> request,
                                         std::shared_ptr<board_perception::srv::ProcessBoard::Response> response) {
    for (int i = 0; i < 9; i++)
        response->board[i] = -1;

    response->success = false;

    cv::Mat frame;

    if (use_image_file_) {
        RCLCPP_INFO(this->get_logger(), "Loading image from file: %s", image_path_.c_str());
        frame = cv::imread(image_path_, cv::IMREAD_COLOR);
        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image from file: %s", image_path_.c_str());
            return;
        }
    } else {
        if (!latest_image_) {
            RCLCPP_WARN(this->get_logger(), "No image received yet on /camera/D435/color/image_raw");
            return;
        }

        try {
            frame = cv_bridge::toCvCopy(latest_image_, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion failed: %s", e.what());
            return;
        }
    }

    if (frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Converted image is empty");
        return;
    }

    std::array<int, 9> board;
    board.fill(-1);

    if (!processor_.process(frame, board)) {
        RCLCPP_ERROR(this->get_logger(), "Board processing failed");
        return;
    }

    for (int i = 0; i < 9; i++)
        response->board[i] = board[i];

    response->success = true;

    RCLCPP_INFO(this->get_logger(), "Board processed – result returned.");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoardProcessorServer>());
    rclcpp::shutdown();
    return 0;
}
