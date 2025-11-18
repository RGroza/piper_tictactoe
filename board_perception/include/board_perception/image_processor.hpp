#pragma once

#include <array>
#include <opencv2/opencv.hpp>

class ImageProcessor {
  public:
    ImageProcessor(bool debug);
    std::array<int, 9> process(const cv::Mat& frame);

  private:
    bool debug_;
    std::string debug_output_dir_;

    void orderPoints(std::vector<cv::Point2f>& corner_pts);
    int findClosestEdge(const std::vector<cv::Vec4i>& lines, int coor, bool vertical);
    void saveDebug(const std::string& name, const cv::Mat& img);
};
