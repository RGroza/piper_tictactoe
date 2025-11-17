#pragma once

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "board_perception/image_processor.hpp"
#include "board_perception/srv/process_board.hpp"

class BoardProcessorServer : public rclcpp::Node {
  public:
    BoardProcessorServer();

  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Service<board_perception::srv::ProcessBoard>::SharedPtr srv_;

    sensor_msgs::msg::Image::ConstSharedPtr latest_image_;

    ImageProcessor processor_;

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

    void handle_service(const std::shared_ptr<board_perception::srv::ProcessBoard::Request> request,
                        std::shared_ptr<board_perception::srv::ProcessBoard::Response> response);
};
