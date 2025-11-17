#include "board_perception/board_processor_server.hpp"

using std::placeholders::_1;

BoardProcessorServer::BoardProcessorServer() : Node("board_processor_server"), processor_(true) {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", rclcpp::SensorDataQoS(), std::bind(&BoardProcessorServer::image_callback, this, _1));

    srv_ = this->create_service<board_perception::srv::ProcessBoard>(
        "process_board",
        std::bind(&BoardProcessorServer::handle_service, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "BoardProcessorServer node started.");
}

void BoardProcessorServer::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    latest_image_ = msg;
}

void BoardProcessorServer::handle_service(const std::shared_ptr<board_perception::srv::ProcessBoard::Request>,
                                          std::shared_ptr<board_perception::srv::ProcessBoard::Response> response) {
    for (int i = 0; i < 9; i++)
        response->results[i] = -1;

    if (!latest_image_) {
        RCLCPP_WARN(this->get_logger(), "No image received yet on /camera/color/image_raw");
        return;
    }

    cv::Mat frame;

    try {
        frame = cv_bridge::toCvCopy(latest_image_, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion failed: %s", e.what());
        return;
    }

    if (frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Converted image is empty");
        return;
    }

    auto result = processor_.process(frame);

    for (int i = 0; i < 9; i++)
        response->results[i] = result[i];

    RCLCPP_INFO(this->get_logger(), "Board processed – result returned.");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoardProcessorServer>());
    rclcpp::shutdown();
    return 0;
}
