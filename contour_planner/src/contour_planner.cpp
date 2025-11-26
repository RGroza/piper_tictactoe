// OpenCV program to extract contours and output coordinates for robot arm drawing
#include <fstream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "piper_trajectory/srv/execute_trajectory.hpp"

using ExecuteTrajectory = piper_trajectory::srv::ExecuteTrajectory;

class ContourPlannerNode : public rclcpp::Node {
  public:
    ContourPlannerNode() : Node("contour_planner_node") {
        this->declare_parameter<std::string>("image_name", "");
        this->declare_parameter<std::string>("output_csv", "contour_coords.csv");
        this->declare_parameter<bool>("skeletonize", false);
        this->declare_parameter<int>("step_size", 1);

        std::string image_name = this->get_parameter("image_name").as_string();
        std::string output_csv = this->get_parameter("output_csv").as_string();
        skeletonize_           = this->get_parameter("skeletonize").as_bool();
        step_size_             = this->get_parameter("step_size").as_int();

        std::string image_path;
        if (!image_name.empty()) {
            std::string share_dir = ament_index_cpp::get_package_share_directory("contour_planner");
            image_path            = share_dir + "/images/" + image_name;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'image_name' is required.");
            rclcpp::shutdown();
            return;
        }

        client_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        trajectory_client_ = this->create_client<piper_trajectory::srv::ExecuteTrajectory>(
            "execute_trajectory", rmw_qos_profile_services_default, client_callback_group_);

        // Load and process image
        std::vector<std::vector<cv::Point>> contours = process_image(image_path, output_csv);

        // Call service for each contour
        // call_execute_trajectory_for_contours(contours);
    }

  private:
    std::vector<int> image_dimensions_;
    bool skeletonize_;
    int step_size_;
    rclcpp::Client<piper_trajectory::srv::ExecuteTrajectory>::SharedPtr trajectory_client_;
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;

    std::vector<std::vector<cv::Point>> process_image(const std::string& image_path, const std::string& output_csv) {
        cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
        if (image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open or find the image!");
            rclcpp::shutdown();
            return {};
        }

        image_dimensions_ = {image.cols, image.rows};
        RCLCPP_INFO(this->get_logger(), "Processing image of size %dx%d", image_dimensions_[0], image_dimensions_[1]);

        cv::Mat gray, blurred, edges;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        std::vector<std::vector<cv::Point>> contours;
        if (skeletonize_) {
            // Skeletonization (thinning) to get centerline
            cv::Mat binary;
            cv::threshold(gray, binary, 200, 255, cv::THRESH_BINARY);
            cv::imshow("Binary", binary);
            cv::Mat skeleton;
            cv::ximgproc::thinning(binary, skeleton, cv::ximgproc::THINNING_ZHANGSUEN);
            cv::imshow("Skeleton", skeleton);
            // Find contours on skeleton
            cv::findContours(skeleton, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
        } else {
            // Regular edge detection
            cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
            cv::Canny(blurred, edges, 50, 150);
            cv::findContours(edges, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
        }

        // Draw contours image
        cv::Mat contours_image = cv::Mat::zeros(image.size(), CV_8UC3);
        for (size_t i = 0; i < contours.size(); ++i) {
            if (!contours[i].empty())
                cv::drawContours(contours_image, contours, (int)i, cv::Scalar(0, 255, 0), 1);
        }
        cv::imwrite("contours_output.png", contours_image);

        // Output coordinates to CSV
        std::ofstream ofs(output_csv);
        if (!ofs) {
            RCLCPP_ERROR(this->get_logger(), "Could not open output file!");
            rclcpp::shutdown();
            return {};
        }
        ofs << "contour_id,x,y\n";
        double min_contour_length = 10.0;

        std::vector<std::vector<cv::Point>> filtered_contours;

        // Prepare image for points
        cv::Mat points_image = contours_image.clone();

        for (size_t i = 0; i < contours.size(); ++i) {
            const auto& contour = contours[i];
            double length       = cv::arcLength(contour, true);
            if (length < min_contour_length)
                continue;

            std::vector<cv::Point> selected_points;
            int contour_size = static_cast<int>(contour.size());
            for (int j = 0; j < contour_size; j += step_size_) {
                selected_points.push_back(contour[j]);
            }

            // Output to CSV and draw points
            for (const auto& pt : selected_points) {
                ofs << i << "," << pt.x << "," << pt.y << "\n";
                cv::circle(points_image, pt, 2, cv::Scalar(0, 0, 255), -1); // red dot
            }

            filtered_contours.push_back(selected_points);
        }

        ofs.close();
        RCLCPP_INFO(this->get_logger(), "Contour points saved to %s", output_csv.c_str());

        // Save points image
        cv::imwrite("points_output.png", points_image);

        return filtered_contours;
    }

    void call_execute_trajectory_for_contours(const std::vector<std::vector<cv::Point>>& contours) {
        // Wait for the service to be available
        if (!trajectory_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Service /execute_trajectory not available.");
            rclcpp::shutdown();
            return;
        }

        for (size_t i = 0; i < contours.size(); ++i) {
            auto request = std::make_shared<ExecuteTrajectory::Request>();

            request->type = 6;
            if (i == contours.size() - 1) {
                request->return_home = true;
            } else {
                request->return_home = false;
            }
            request->image_dimensions = image_dimensions_;

            for (const auto& pt : contours[i]) {
                request->contour_points.push_back(pt.x);
                request->contour_points.push_back(pt.y);
            }

            RCLCPP_INFO(this->get_logger(), "Calling /execute_trajectory for contour %zu with %zu points", i,
                        contours[i].size());
            auto result = trajectory_client_->async_send_request(request);
            // Wait for the result synchronously for simplicity
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
                rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed for contour %zu", i);
            } else {
                RCLCPP_INFO(this->get_logger(), "Contour %zu executed.", i);
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ContourPlannerNode>();
    rclcpp::shutdown();
    return 0;
}
