#pragma once

#include <array>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImageProcessor {
  public:
    ImageProcessor(rclcpp::Node* node, bool debug);
    bool process(const cv::Mat& frame, std::array<int, 9>& result);

  private:
    rclcpp::Node* node_;
    bool debug_;
    std::string debug_output_dir_;

    void orderPoints(std::vector<cv::Point2f>& corner_pts);
    int findClosestEdge(const std::vector<cv::Vec4i>& lines, int starting_coor, int direction, bool vertical);
    void saveDebug(const std::string& name, const cv::Mat& img);
    void publishDebug(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub, const cv::Mat& img);
    double lineLength(const cv::Vec4i& line);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_board_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_edges_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_cells_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_detection_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_final_;
};
