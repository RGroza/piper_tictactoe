#include "board_perception/board_processor_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

using std::placeholders::_1;

BoardProcessorNode::BoardProcessorNode() : Node("ttt_board_processor_node"), processor_(this) {
    bool debug = this->declare_parameter<bool>("debug", false);
    if (debug) {
        processor_.enableDebug();
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/D435/color/image_raw", qos, std::bind(&BoardProcessorNode::imageCallback, this, _1));

    board_pub_ = this->create_publisher<board_perception::msg::BoardState>("/board_state", 5);

    // Add timer for 10Hz processing
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&BoardProcessorNode::processAndPublish, this));

    RCLCPP_INFO(this->get_logger(), "BoardProcessorNode node started");
}

void BoardProcessorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    latest_image_ = msg;
}

void BoardProcessorNode::processAndPublish() {
    board_perception::msg::BoardState msg;
    msg.board.fill(-1);
    msg.success = false;

    cv::Mat frame;

    if (!latest_image_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "No image received yet on /camera/D435/color/image_raw");
        msg.board_detected = false;
        board_pub_->publish(msg);
        return;
    }

    try {
        frame = cv_bridge::toCvCopy(latest_image_, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "cv_bridge conversion failed: %s",
                              e.what());
        msg.board_detected = false;
        board_pub_->publish(msg);
        return;
    }

    if (frame.empty()) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Converted image is empty");
        msg.board_detected = false;
        board_pub_->publish(msg);
        return;
    }

    std::array<int, 9> board;
    board.fill(-1);

    msg.board_detected = processor_.process(frame, board);

    for (int i = 0; i < 9; i++)
        msg.board[i] = board[i];

    msg.success = true;
    board_pub_->publish(msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoardProcessorNode>());
    rclcpp::shutdown();
    return 0;
}
