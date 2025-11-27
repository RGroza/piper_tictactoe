#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "board_perception/image_processor.hpp"
#include "board_perception/msg/board_state.hpp"

class BoardProcessorNode : public rclcpp::Node {
  public:
    BoardProcessorNode();

  private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void processAndPublish();

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<board_perception::msg::BoardState>::SharedPtr board_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::Image::ConstSharedPtr latest_image_;

    ImageProcessor processor_;
};
