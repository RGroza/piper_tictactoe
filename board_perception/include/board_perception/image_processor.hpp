#pragma once

#include <array>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImageProcessor {
  public:
    ImageProcessor(rclcpp::Node* node);
    bool process(const cv::Mat& frame, std::array<int, 9>& result);
    void enableDebug();
    void enableSaveDebugImages();

  private:
    rclcpp::Node* node_;
    bool debug_;
    bool save_debug_images_;
    std::string debug_output_dir_;

    void saveImage(const std::string& name, const cv::Mat& img);
    void publishImage(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub, const cv::Mat& img);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_board_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_edges_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_cells_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_detection_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_final_;
};
