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

ImageProcessor::ImageProcessor(bool debug) : debug_(debug) {
    debug_output_dir_ = "/home/robert/ROS/Final/ros2_ws/src/tictactoe/board_perception/debug";
    std::cout << "ImageProcessor init: debug=" << debug_ << ", dir=" << debug_output_dir_ << std::endl;
}

void ImageProcessor::saveDebug(const std::string& name, const cv::Mat& img) {
    if (!debug_)
        return;
    if (img.empty())
        return;

    // create directory if missing
    std::filesystem::create_directories(debug_output_dir_);

    // zero-padded counter
    // int id = debug_counter_.load();
    // char prefix[16];
    // snprintf(prefix, sizeof(prefix), "%04d_", id);

    std::string filename = debug_output_dir_ + "/" + name + ".png";

    cv::imwrite(filename, img);

    std::cout << "Saved debug image: " << filename << std::endl;
}

void ImageProcessor::orderPoints(vector<Point2f>& cornerPts) {
    vector<Point2f> pts;
    for (auto& p : cornerPts)
        pts.push_back(Point2f(p.x, p.y));

    auto min_sum  = [&](const Point2f& p1, const Point2f& p2) { return (p1.x + p1.y) < (p2.x + p2.y); };
    auto min_diff = [&](const Point2f& p1, const Point2f& p2) { return (p1.x - p1.y) < (p2.x - p2.y); };

    cornerPts[0] = *min_element(pts.begin(), pts.end(), min_sum);
    cornerPts[1] = *max_element(pts.begin(), pts.end(), min_diff);
    cornerPts[2] = *max_element(pts.begin(), pts.end(), min_sum);
    cornerPts[3] = *min_element(pts.begin(), pts.end(), min_diff);
}

int ImageProcessor::findClosestEdge(const vector<Vec4i>& lines, int coor, bool vertical) {
    if (lines.empty())
        return coor;

    int closest;
    if (vertical)
        closest = lines[0][0];
    else
        closest = lines[0][1];

    int min_diff = abs(closest - coor);
    for (auto& L : lines) {
        int c1, c2;
        if (vertical) {
            c1 = L[0];
            c2 = L[2];
        } else {
            c1 = L[1];
            c2 = L[3];
        }

        int line_coor = c1;
        int diff      = abs(c1 - coor);
        if (abs(c2 - coor) < diff) {
            line_coor = c2;
            diff      = abs(c2 - coor);
        }

        if (diff < min_diff) {
            min_diff = diff;
            closest  = line_coor;
        }
    }

    return closest;
}

std::array<int, 9> ImageProcessor::process(const cv::Mat& frame) {
    std::array<int, 9> result;
    result.fill(-1);

    if (frame.empty()) {
        cout << "Empty frame provided to ImageProcessor::process" << endl;
        return result;
    }

    // int current_id = debug_counter_.fetch_add(1);

    Mat hsv, mask;

    // --------------------------------------------------------
    // 1. Convert to HSV and mask for board color
    // --------------------------------------------------------
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(70, 0, 200), Scalar(180, 255, 255), mask);
    saveDebug("mask", mask);

    // --------------------------------------------------------
    // 2. Find all contours and pick the smallest large one
    // --------------------------------------------------------
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    int minContourArea = numeric_limits<int>::max();
    vector<Point> inner;

    for (auto& c : contours) {
        int area = contourArea(c);
        if (area > 10000 && area < minContourArea) {
            cout << "Contour area: " << area << endl;
            minContourArea = area;
            inner          = c;
        }
    }

    if (inner.empty()) {
        return result;
    }

    Mat boardContour = frame.clone();
    drawContours(boardContour, vector<vector<Point>>{inner}, -1, Scalar(0, 255, 0), 3);

    // --------------------------------------------------------
    // 3. Approximate corner points
    // --------------------------------------------------------
    double peri = arcLength(inner, true);
    vector<Point2f> cornerPts;
    approxPolyDP(inner, cornerPts, 0.01 * peri, true);

    if (cornerPts.size() != 4) {
        saveDebug("corner_points", boardContour);
        return result;
    }

    orderPoints(cornerPts);

    for (auto& p : cornerPts)
        circle(boardContour, p, 5, Scalar(0, 0, 255), -1);

    saveDebug("corner_points", boardContour);

    // --------------------------------------------------------
    // 4. Perspective transform
    // --------------------------------------------------------
    int width  = BOARD_IMAGE_WIDTH;
    int height = int(BOARD_ASPECT_RATIO * BOARD_IMAGE_WIDTH);

    vector<Point2f> dstPts = {
        {0, 0}, {float(width - 1), 0}, {float(width - 1), float(height - 1)}, {0, float(height - 1)}};

    Mat warped;
    Mat M = getPerspectiveTransform(cornerPts, dstPts);
    warpPerspective(frame, warped, M, Size(width, height));
    saveDebug("warped", warped);

    // --------------------------------------------------------
    // 5. Crop warped image
    // --------------------------------------------------------
    float cropFactor = 0.05f;
    int x            = int(cropFactor * width);
    int y            = int(cropFactor * height);
    int w            = int((1 - 2 * cropFactor) * width);
    int h            = int((1 - 2 * cropFactor) * height);

    if (x < 0 || y < 0 || x + w > warped.cols || y + h > warped.rows) {
        cout << "Crop dimensions out of bounds!" << endl;
        return result;
    }

    Mat board = warped(Rect(x, y, w, h)).clone();
    saveDebug("board", board);

    // --------------------------------------------------------
    // 6. Detect drawing
    // --------------------------------------------------------
    cvtColor(board, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(30, 0, 130), Scalar(120, 120, 200), mask);
    saveDebug("mask_inside_board", mask);

    Mat edges;
    Canny(mask, edges, 50, 150);
    saveDebug("edges", edges);

    // --------------------------------------------------------
    // 7. Hough Lines
    // --------------------------------------------------------
    vector<Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, 10, 150, 50);

    if (lines.empty()) {
        cout << "No lines detected!" << endl;
        return result;
    }

    vector<Vec4i> vertical, horizontal;
    for (auto& L : lines) {
        int dx = abs(L[0] - L[2]);
        int dy = abs(L[1] - L[3]);

        if (dx < 30)
            vertical.push_back(L);
        else if (dy < 30)
            horizontal.push_back(L);
    }

    Mat lineDisplay = board.clone();
    for (auto& L : vertical)
        line(lineDisplay, Point(L[0], L[1]), Point(L[2], L[3]), Scalar(0, 0, 255), 2);
    for (auto& L : horizontal)
        line(lineDisplay, Point(L[0], L[1]), Point(L[2], L[3]), Scalar(255, 0, 0), 2);

    saveDebug("detected_lines", lineDisplay);

    // --------------------------------------------------------
    // 8. Edge detection (grid)
    // --------------------------------------------------------
    vector<int> xEdges, yEdges;
    xEdges.push_back(0);
    xEdges.push_back(findClosestEdge(vertical, 0, true));
    xEdges.push_back(findClosestEdge(vertical, int(0.4 * board.cols), true));
    xEdges.push_back(findClosestEdge(vertical, int(0.6 * board.cols), true));
    xEdges.push_back(findClosestEdge(vertical, board.cols - 1, true));
    xEdges.push_back(board.cols - 1);

    yEdges.push_back(0);
    yEdges.push_back(findClosestEdge(horizontal, 0, false));
    yEdges.push_back(findClosestEdge(horizontal, int(0.4 * board.rows), false));
    yEdges.push_back(findClosestEdge(horizontal, int(0.6 * board.rows), false));
    yEdges.push_back(findClosestEdge(horizontal, board.rows - 1, false));
    yEdges.push_back(board.rows - 1);

    Mat edgeDisplay = board.clone();
    for (auto& xe : xEdges)
        line(edgeDisplay, Point(xe, 0), Point(xe, board.rows - 1), Scalar(0, 255, 0), 2);
    for (auto& ye : yEdges)
        line(edgeDisplay, Point(0, ye), Point(board.cols - 1, ye), Scalar(0, 255, 0), 2);

    saveDebug("detected_edges", edgeDisplay);

    // --------------------------------------------------------
    // 9. Extract 9 cells
    // --------------------------------------------------------
    vector<Mat> cells;
    vector<array<int, 4>> cellCorners;

    for (int i = 4; i >= 0; i -= 2) {
        for (int j = 0; j < 6; j += 2) {
            int x1 = xEdges[j];
            int x2 = xEdges[j + 1];
            int y1 = yEdges[i];
            int y2 = yEdges[i + 1];

            Rect r(x1, y1, x2 - x1, y2 - y1);
            if (r.x < 0 || r.y < 0 || r.x + r.width > board.cols || r.y + r.height > board.rows) {
                cout << "Cell extraction out of bounds!" << endl;
                return result;
            }
            if (r.width <= 0 || r.height <= 0) {
                cout << "Invalid cell dimensions!" << endl;
                return result;
            }

            cells.push_back(board(r).clone());
            cellCorners.push_back({x1, y1, x2, y2});
        }
    }

    // Draw cell overlay
    Mat cellDisplay = board.clone();
    int idx         = 0;
    for (auto& c : cellCorners) {
        rectangle(cellDisplay, Point(c[0], c[1]), Point(c[2], c[3]), Scalar(255, 0, 0), 2);
        putText(cellDisplay, to_string(idx), Point((c[0] + c[2]) / 2 - 10, (c[1] + c[3]) / 2 + 10),
                FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 2);
        idx++;
    }
    saveDebug("cells", cellDisplay);

    if (cells.size() != 9) {
        cout << "Failed to extract 9 cells!" << endl;
        return result;
    }

    // --------------------------------------------------------
    // 10. Detect X or O in each cell
    // --------------------------------------------------------
    for (int i = 0; i < 9; i++) {
        // X detection
        array<int, 4> corners = cellCorners[i];

        Rect cellRect(corners[0], corners[1], corners[2] - corners[0], corners[3] - corners[1]);
        if (cellRect.x < 0 || cellRect.y < 0 || cellRect.x + cellRect.width > edges.cols ||
            cellRect.y + cellRect.height > edges.rows) {
            cout << "Cell rect out of bounds for cell " << i << endl;
            result[i] = -1;
            continue;
        }

        Mat cellEdges = edges(cellRect).clone();
        vector<Vec4i> lines;
        HoughLinesP(cellEdges, lines, 1, CV_PI / 180, 50, 50, 10);

        bool foundX = false;
        for (auto& L : lines) {
            double angle = abs(atan2(L[3] - L[1], L[2] - L[0])) * 180.0 / CV_PI;
            if ((angle > 25 && angle < 75) || (angle > 105 && angle < 155)) {
                line(board, Point(L[0] + corners[0], L[1] + corners[1]), Point(L[2] + corners[0], L[3] + corners[1]),
                     Scalar(255, 0, 0), 3);
                foundX = true;
                break;
            }
        }

        if (foundX) {
            result[i] = 1;
            continue;
        }

        // O detection
        Mat gray;
        cvtColor(cells[i], gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, gray, Size(5, 5), 0);

        vector<Vec3f> circles;
        HoughCircles(gray, circles, HOUGH_GRADIENT, 1, 200, 10, 50, 50, 300);

        if (!circles.empty()) {
            for (auto& c : circles)
                circle(board, Point(c[0] + corners[0], c[1] + corners[1]), c[2], Scalar(0, 255, 0), 3);
            result[i] = 0;
            continue;
        }

        result[i] = -1;
    }

    saveDebug("final_detection", board);

    return result;
}
