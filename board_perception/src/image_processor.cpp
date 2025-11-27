#include "board_perception/image_processor.hpp"
#include <cmath>
#include <filesystem>
#include <limits>

using namespace cv;
using namespace std;

namespace {
constexpr float BOARD_WIDTH        = 0.154f;
constexpr float BOARD_HEIGHT       = 0.203f;
constexpr int BOARD_IMAGE_WIDTH    = 900;
constexpr float BOARD_ASPECT_RATIO = BOARD_HEIGHT / BOARD_WIDTH;
} // namespace

ImageProcessor::ImageProcessor(rclcpp::Node* node, bool debug) : node_(node), debug_(debug) {
    debug_output_dir_ = "/home/user/ros2_ws/src/piper_tictactoe/board_perception/debug";
    // debug_output_dir_ = "/home/robert/ROS/Final/ros2_ws/src/tictactoe/board_perception/debug";

    if (debug_) {
        auto qos       = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().transient_local();
        pub_board_     = node_->create_publisher<sensor_msgs::msg::Image>("/board_processor/board", qos);
        pub_edges_     = node_->create_publisher<sensor_msgs::msg::Image>("/board_processor/edges", qos);
        pub_cells_     = node_->create_publisher<sensor_msgs::msg::Image>("/board_processor/cells", qos);
        pub_detection_ = node_->create_publisher<sensor_msgs::msg::Image>("/board_processor/detection", qos);
        pub_final_     = node_->create_publisher<sensor_msgs::msg::Image>("/board_processor/final", qos);
    }
}

void ImageProcessor::saveDebug(const string& name, const cv::Mat& img) {
    if (!debug_)
        return;
    if (img.empty())
        return;

    filesystem::create_directories(debug_output_dir_);
    string filename = debug_output_dir_ + "/" + name + ".png";
    cv::imwrite(filename, img);
    cout << "Saved debug image: " << filename << endl;
}

void ImageProcessor::publishDebug(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub, const cv::Mat& img) {
    if (!debug_ || !pub)
        return;
    if (img.empty())
        return;

    std::string encoding;
    cv::Mat safe;

    if (img.type() == CV_8UC3) {
        encoding = "bgr8";
        safe     = img;
    } else if (img.type() == CV_8UC1) {
        encoding = "mono8";
        safe     = img;
    } else {
        RCLCPP_WARN(node_->get_logger(), "Unsupported debug image type=%d. Converting to bgr8.", img.type());
        cv::cvtColor(img, safe, cv::COLOR_GRAY2BGR);
        encoding = "bgr8";
    }

    if (!safe.isContinuous()) {
        safe = safe.clone();
    }

    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), encoding, safe).toImageMsg();

    RCLCPP_INFO(node_->get_logger(), "Publishing debug image: encoding=%s step=%u expected=%u", encoding.c_str(),
                msg->step, safe.cols * (safe.channels()));

    pub->publish(*msg);
}

void ImageProcessor::orderPoints(vector<Point2f>& corner_pts) {
    vector<Point2f> pts;
    for (auto& p : corner_pts)
        pts.push_back(Point2f(p.x, p.y));

    auto min_sum  = [&](const Point2f& p1, const Point2f& p2) { return (p1.x + p1.y) < (p2.x + p2.y); };
    auto min_diff = [&](const Point2f& p1, const Point2f& p2) { return (p1.x - p1.y) < (p2.x - p2.y); };

    corner_pts[0] = *min_element(pts.begin(), pts.end(), min_sum);
    corner_pts[1] = *max_element(pts.begin(), pts.end(), min_diff);
    corner_pts[2] = *max_element(pts.begin(), pts.end(), min_sum);
    corner_pts[3] = *min_element(pts.begin(), pts.end(), min_diff);
}

int ImageProcessor::findClosestEdge(const vector<Vec4i>& lines, int starting_coor, int direction, bool vertical) {
    if (lines.empty())
        return starting_coor;

    int min_diff = numeric_limits<int>::max();
    int closest  = starting_coor;

    for (auto& L : lines) {
        int c1 = vertical ? L[0] : L[1];
        int c2 = vertical ? L[2] : L[3];

        int line_coor, diff;
        int diff_c1 = direction * (c1 - starting_coor);
        int diff_c2 = direction * (c2 - starting_coor);

        if (diff_c1 >= 0 && (diff_c1 < diff_c2 || diff_c2 < 0)) {
            line_coor = c1;
            diff      = diff_c1;
        } else {
            line_coor = c2;
            diff      = diff_c2;
        }

        if (diff >= 0 && diff < min_diff) {
            min_diff = diff;
            closest  = line_coor;
        }
    }

    // Add small cropping factor to avoid including grid lines
    return closest + 0.02 * (starting_coor - closest);
}

inline double lineLength(const cv::Vec4i& l) {
    int dx = l[2] - l[0];
    int dy = l[3] - l[1];
    return std::sqrt(dx * dx + dy * dy);
}

int orientation(cv::Point2f p, cv::Point2f q, cv::Point2f r) {
    float val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (fabs(val) < 1e-6)
        return 0;             // colinear
    return (val > 0) ? 1 : 2; // clock or counterclock wise
}

bool onSegment(cv::Point2f p, cv::Point2f q, cv::Point2f r) {
    return q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && q.y <= std::max(p.y, r.y) &&
           q.y >= std::min(p.y, r.y);
}

bool doIntersect(cv::Point2f p1, cv::Point2f q1, cv::Point2f p2, cv::Point2f q2) {
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    if (o1 == 0 && onSegment(p1, p2, q1))
        return true;
    if (o2 == 0 && onSegment(p1, q2, q1))
        return true;
    if (o3 == 0 && onSegment(p2, p1, q2))
        return true;
    if (o4 == 0 && onSegment(p2, q1, q2))
        return true;

    return false;
}

bool ImageProcessor::process(const cv::Mat& frame, array<int, 9>& result) {
    if (frame.empty()) {
        cout << "Empty frame provided to ImageProcessor::process" << endl;
        return false;
    }

    // --------------------------------------------------------
    // 1. Convert to HSV and mask
    // --------------------------------------------------------
    Mat hsv, mask;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    saveDebug("frame", frame);
    inRange(hsv, Scalar(70, 130, 130), Scalar(130, 220, 255), mask);
    saveDebug("mask", mask);

    // --------------------------------------------------------
    // 2. Find all contours and pick smallest large one
    // --------------------------------------------------------
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    int min_contour_area = numeric_limits<int>::max();
    vector<Point> inner;

    for (auto& c : contours) {
        int area   = contourArea(c);
        int length = int(arcLength(c, true));
        RCLCPP_INFO(node_->get_logger(), "Found contour with area, length: %d, %d", area, length);
        if (area > 10000 && area < min_contour_area) {
            RCLCPP_INFO(node_->get_logger(), "Contour area: %d", area);
            min_contour_area = area;
            inner            = c;
        }
    }

    if (inner.empty()) {
        return false;
    }

    Mat board_contour = frame.clone();
    drawContours(board_contour, vector<vector<Point>>{inner}, -1, Scalar(0, 255, 0), 2);

    // --------------------------------------------------------
    // 3. Approximate corner points
    // --------------------------------------------------------
    double peri = arcLength(inner, true);
    vector<Point2f> corner_pts;
    approxPolyDP(inner, corner_pts, 0.01 * peri, true);

    if (corner_pts.size() != 4) {
        saveDebug("corner_points", board_contour);
        return false;
    }

    orderPoints(corner_pts);

    for (auto& p : corner_pts)
        circle(board_contour, p, 5, Scalar(255, 0, 0), -1);

    saveDebug("corner_points", board_contour);
    publishDebug(pub_board_, board_contour);

    // --------------------------------------------------------
    // 4. Perspective transform
    // --------------------------------------------------------
    int width  = BOARD_IMAGE_WIDTH;
    int height = int(BOARD_ASPECT_RATIO * BOARD_IMAGE_WIDTH);

    vector<Point2f> dst_pts = {
        {0, 0}, {float(width - 1), 0}, {float(width - 1), float(height - 1)}, {0, float(height - 1)}};

    Mat warped;
    Mat m = getPerspectiveTransform(corner_pts, dst_pts);
    warpPerspective(frame, warped, m, Size(width, height));
    saveDebug("warped", warped);

    // --------------------------------------------------------
    // 5. Crop warped image
    // --------------------------------------------------------
    float crop_factor = 0.05f;
    int x             = int(crop_factor * width);
    int y             = int(crop_factor * height);
    int w             = int((1 - 2 * crop_factor) * width);
    int h             = int((1 - 2 * crop_factor) * height);

    if (x < 0 || y < 0 || x + w > warped.cols || y + h > warped.rows) {
        cout << "Crop dimensions out of bounds!" << endl;
        return false;
    }

    Mat board = warped(Rect(x, y, w, h)).clone();
    saveDebug("board", board);

    // --------------------------------------------------------
    // 6. Use adaptive threshold and Canny to find edges
    // --------------------------------------------------------
    Mat gray;
    cvtColor(board, gray, COLOR_BGR2GRAY);

    Mat blurred;
    GaussianBlur(gray, blurred, Size(11, 11), 2.0);

    Ptr<CLAHE> clahe = createCLAHE(2.0);
    Mat eq;
    clahe->apply(blurred, eq);

    adaptiveThreshold(eq, mask, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 31, -5);

    Mat edges;
    Canny(mask, edges, 30, 100);
    saveDebug("edges", edges);
    publishDebug(pub_edges_, edges);

    // --------------------------------------------------------
    // 7. Hough lines for grid lines
    // --------------------------------------------------------
    vector<Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, 10, 0.15 * BOARD_IMAGE_WIDTH, 0.05 * BOARD_IMAGE_WIDTH);

    if (lines.empty()) {
        cout << "No lines detected!" << endl;
        return false;
    }

    vector<Vec4i> vertical, horizontal;
    int pixel_threshold = int(0.02 * BOARD_IMAGE_WIDTH);

    for (auto& L : lines) {
        int dx = abs(L[0] - L[2]);
        int dy = abs(L[1] - L[3]);

        if (dx < pixel_threshold && ((L[0] > 0.25 * board.cols && L[0] < 0.40 * board.cols) ||
                                     (L[0] > 0.60 * board.cols && L[0] < 0.75 * board.cols)))
            vertical.push_back(L);
        else if (dy < pixel_threshold && ((L[1] > 0.25 * board.rows && L[1] < 0.40 * board.rows) ||
                                          (L[1] > 0.60 * board.rows && L[1] < 0.75 * board.rows)))
            horizontal.push_back(L);
    }

    Mat line_display = board.clone();
    for (auto& L : vertical)
        line(line_display, Point(L[0], L[1]), Point(L[2], L[3]), Scalar(0, 255, 0), 2);
    for (auto& L : horizontal)
        line(line_display, Point(L[0], L[1]), Point(L[2], L[3]), Scalar(0, 255, 0), 2);

    saveDebug("detected_lines", line_display);

    // --------------------------------------------------------
    // 8. Compute grid edges
    // --------------------------------------------------------
    vector<int> x_edges, y_edges;

    x_edges.push_back(0);
    x_edges.push_back(findClosestEdge(vertical, int(0.25 * board.cols), 1, true));
    x_edges.push_back(findClosestEdge(vertical, int(0.40 * board.cols), -1, true));
    x_edges.push_back(findClosestEdge(vertical, int(0.60 * board.cols), 1, true));
    x_edges.push_back(findClosestEdge(vertical, int(0.75 * board.cols), -1, true));
    x_edges.push_back(board.cols - 1);

    y_edges.push_back(0);
    y_edges.push_back(findClosestEdge(horizontal, int(0.25 * board.rows), 1, false));
    y_edges.push_back(findClosestEdge(horizontal, int(0.40 * board.rows), -1, false));
    y_edges.push_back(findClosestEdge(horizontal, int(0.60 * board.rows), 1, false));
    y_edges.push_back(findClosestEdge(horizontal, int(0.75 * board.rows), -1, false));
    y_edges.push_back(board.rows - 1);

    Mat edge_display = board.clone();
    for (auto& xe : x_edges)
        line(edge_display, Point(xe, 0), Point(xe, board.rows - 1), Scalar(0, 255, 0), 2);
    for (auto& ye : y_edges)
        line(edge_display, Point(0, ye), Point(board.cols - 1, ye), Scalar(0, 255, 0), 2);

    saveDebug("detected_edges", edge_display);

    // --------------------------------------------------------
    // 9. Extract 9 cells
    // --------------------------------------------------------
    vector<Mat> cells;
    vector<array<int, 4>> cell_corners;

    for (int i = 0; i < 6; i += 2) {
        for (int j = 0; j < 6; j += 2) {
            int x1 = x_edges[j];
            int x2 = x_edges[j + 1];
            int y1 = y_edges[i];
            int y2 = y_edges[i + 1];

            Rect r(x1, y1, x2 - x1, y2 - y1);
            if (r.x < 0 || r.y < 0 || r.x + r.width > board.cols || r.y + r.height > board.rows) {
                cout << "Cell extraction out of bounds!" << endl;
                return false;
            }
            if (r.width <= 0 || r.height <= 0) {
                cout << "Invalid cell dimensions!" << endl;
                return false;
            }

            cells.push_back(board(r).clone());
            cell_corners.push_back({x1, y1, x2, y2});
        }
    }

    Mat cell_display = board.clone();
    int idx          = 0;
    for (auto& c : cell_corners) {
        rectangle(cell_display, Point(c[0], c[1]), Point(c[2], c[3]), Scalar(0, 255, 0), 2);
        putText(cell_display, to_string(idx + 1), Point((c[0] + c[2]) / 2 - 10, (c[1] + c[3]) / 2 + 10),
                FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
        idx++;
    }
    saveDebug("cells", cell_display);
    publishDebug(pub_cells_, cell_display);

    if (cells.size() != 9) {
        cout << "Failed to extract 9 cells!" << endl;
        return false;
    }

    // --------------------------------------------------------
    // 10. Detect X or O in each cell
    // --------------------------------------------------------
    // Store final detected lines and circles
    Mat final_image = board.clone();
    vector<Vec4i> final_lines;
    vector<pair<Point2f, float>> final_circles;

    for (int i = 0; i < 9; i++) {
        array<int, 4> corners = cell_corners[i];
        Rect cell_rect(corners[0], corners[1], corners[2] - corners[0], corners[3] - corners[1]);

        if (cell_rect.x < 0 || cell_rect.y < 0 || cell_rect.x + cell_rect.width > edges.cols ||
            cell_rect.y + cell_rect.height > edges.rows) {
            cout << "Cell rect out of bounds for cell " << i << endl;
            continue;
        }

        // --------------------------------------------------------
        // Detect Xs
        // --------------------------------------------------------
        // Detect add lines using Hough lines
        Mat cell_edges = edges(cell_rect).clone();
        vector<Vec4i> lines;
        HoughLinesP(cell_edges, lines, 1, CV_PI / 180, 20, 0.1 * BOARD_IMAGE_WIDTH, 0.05 * BOARD_IMAGE_WIDTH);

        // Display detected lines for debugging
        for (auto& L : lines) {
            line(board, Point(L[0] + corners[0], L[1] + corners[1]), Point(L[2] + corners[0], L[3] + corners[1]),
                 Scalar(255, 0, 0), 2);
        }

        // Find two lines that intersect at ~90 degrees
        bool found_x = false;
        for (size_t m = 0; m < lines.size(); ++m) {
            double angle1 = atan2(lines[m][3] - lines[m][1], lines[m][2] - lines[m][0]) * 180.0 / CV_PI;
            for (size_t n = m + 1; n < lines.size(); ++n) {
                double angle2 = atan2(lines[n][3] - lines[n][1], lines[n][2] - lines[n][0]) * 180.0 / CV_PI;
                double diff   = fabs(angle1 - angle2);

                if (diff > 90)
                    diff = 180 - diff; // handle wrap-around

                if (diff > 60 && diff < 120) {
                    cv::Point2f p1(lines[m][0], lines[m][1]);
                    cv::Point2f q1(lines[m][2], lines[m][3]);
                    cv::Point2f p2(lines[n][0], lines[n][1]);
                    cv::Point2f q2(lines[n][2], lines[n][3]);

                    if (doIntersect(p1, q1, p2, q2)) {
                        Vec4i final_line1 = {lines[m][0] + corners[0], lines[m][1] + corners[1],
                                             lines[m][2] + corners[0], lines[m][3] + corners[1]};
                        Vec4i final_line2 = {lines[n][0] + corners[0], lines[n][1] + corners[1],
                                             lines[n][2] + corners[0], lines[n][3] + corners[1]};
                        final_lines.push_back(final_line1);
                        final_lines.push_back(final_line2);
                        found_x = true;
                        break;
                    }
                }
            }
            if (found_x)
                break;
        }

        if (found_x) {
            result[i] = 1;
            continue;
        }

        // --------------------------------------------------------
        // Detect Os
        // --------------------------------------------------------
        // Dilate then erode (closing) to connect gaps
        Mat morph;
        int morph_size = 2;
        Mat kernel     = getStructuringElement(MORPH_ELLIPSE, Size(2 * morph_size + 1, 2 * morph_size + 1));
        morphologyEx(cell_edges, morph, MORPH_CLOSE, kernel);

        // Blur to further reduce noise
        GaussianBlur(morph, morph, Size(5, 5), 1.5);

        // Threshold to get binary image (if needed)
        threshold(morph, morph, 50, 255, THRESH_BINARY);

        // Find contours in the cell edges to detect circles
        vector<vector<Point>> cell_contours;
        // Use RETR_EXTERNAL and CHAIN_APPROX_NONE for longer, less fragmented contours
        findContours(morph, cell_contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

        // Filter contours based on length and area to find circle-like shapes
        std::vector<std::vector<Point>> filtered_circles;
        for (const auto& contour : cell_contours) {
            double length = arcLength(contour, true);
            double area   = contourArea(contour);
            if (length > 200.0 && area > 200.0) {
                RCLCPP_INFO(node_->get_logger(), "Cell %d contour length: %.2f area: %.2f", i + 1, length, area);
                filtered_circles.push_back(contour);
            }
        }

        if (filtered_circles.empty()) {
            RCLCPP_INFO(node_->get_logger(), "No circle-like contours found in cell %d", i + 1);
            result[i] = -1;
            continue;
        }

        result[i] = 0;

        // Find the contour with the largest area
        double max_area = 0.0;
        std::vector<Point> largest_contour;
        for (const auto& contour : filtered_circles) {
            double area = contourArea(contour);
            if (area > max_area) {
                max_area        = area;
                largest_contour = contour;
            }
        }

        for (auto& contour : filtered_circles) {
            // Draw contour in green on board image
            drawContours(board, vector<vector<Point>>{contour}, -1, Scalar(0, 150, 0), 2, LINE_AA, noArray(), 0,
                         Point(corners[0], corners[1]));
        }

        // Highlight the largest contour in red
        drawContours(board, vector<vector<Point>>{largest_contour}, -1, Scalar(0, 0, 255), 2, LINE_AA, noArray(), 0,
                     Point(corners[0], corners[1]));

        // Fit circle to largest contour
        Point2f center;
        float radius;
        minEnclosingCircle(largest_contour, center, radius);

        // Save circle to be drawn later
        center += Point2f(corners[0], corners[1]);
        final_circles.push_back({center, radius});
    }

    saveDebug("detection", board);
    publishDebug(pub_detection_, board);

    // Display final detected lines and circles
    for (const auto& line_data : final_lines) {
        line(final_image, Point(line_data[0], line_data[1]), Point(line_data[2], line_data[3]), Scalar(255, 255, 0), 2);
    }
    for (const auto& circle_data : final_circles) {
        circle(final_image, Point(int(circle_data.first.x), int(circle_data.first.y)), int(circle_data.second),
               Scalar(255, 255, 0), 2);
    }

    saveDebug("final_detection", final_image);
    publishDebug(pub_final_, final_image);

    return true;
}
