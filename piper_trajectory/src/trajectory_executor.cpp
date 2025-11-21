#include "piper_trajectory/trajectory_executor.hpp"

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <thread>

static const rclcpp::Logger LOGGER            = rclcpp::get_logger("ttt_trajectory_executor");
static const std::string PLANNING_GROUP_ROBOT = "arm";

using namespace std::chrono_literals;

namespace {
constexpr float HOVERING_Z    = 0.204f;
constexpr float DRAW_Z_OFFSET = 0.005f;
constexpr float GRID_SIZE     = 0.045f;
constexpr float CROSS_WIDTH   = 0.025f;
constexpr float CIRCLE_RADIUS = 0.015f;
constexpr float GRID_X_OFFSET = 0.027f;
constexpr float GRID_Y_OFFSET = -0.02f;
} // namespace

TrajectoryExecutor::TrajectoryExecutor(rclcpp::Node::SharedPtr base_node) : base_node_(std::move(base_node)) {
    base_node_->set_parameter(rclcpp::Parameter("use_sim_time", false));

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    move_group_node_ = rclcpp::Node::make_shared("move_group_node", node_options);
    move_group_node_->set_parameter(rclcpp::Parameter("use_sim_time", false));

    executor_.add_node(move_group_node_);
    std::thread([this]() { executor_.spin(); }).detach();

    move_group_robot_        = std::make_shared<MoveGroupInterface>(move_group_node_, PLANNING_GROUP_ROBOT);
    joint_model_group_robot_ = move_group_robot_->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ROBOT);

    RCLCPP_INFO(LOGGER, "Planning Frame: %s", move_group_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Link: %s", move_group_robot_->getEndEffectorLink().c_str());

    std::vector<std::string> group_names = move_group_robot_->getJointModelGroupNames();
    for (size_t i = 0; i < group_names.size(); ++i) {
        RCLCPP_INFO(LOGGER, "Group %lu: %s", i, group_names[i].c_str());
    }

    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_, joint_group_positions_robot_);

    move_group_robot_->setStartStateToCurrentState();

    loadWaypoints();

    RCLCPP_INFO(LOGGER, "TrajectoryExecutor node started");
}

// --------------------------------------------------
// Waypoints: 3x3 tic-tac-toe grid
// --------------------------------------------------

void TrajectoryExecutor::loadWaypoints() {
    auto makePose = [&](double x, double y, double z) {
        Pose pose;
        pose.position.x    = x;
        pose.position.y    = y;
        pose.position.z    = z;
        pose.orientation.x = 0.0f;
        pose.orientation.y = 1.0f;
        pose.orientation.z = 0.0f;
        pose.orientation.w = 0.0f;
        return pose;
    };

    waypoints_.clear();
    waypoints_.reserve(9);

    waypoints_.push_back(makePose(0.245f + GRID_X_OFFSET, -0.025f + GRID_Y_OFFSET, HOVERING_Z));
    waypoints_.push_back(makePose(0.200f + GRID_X_OFFSET, -0.025f + GRID_Y_OFFSET, HOVERING_Z));
    waypoints_.push_back(makePose(0.155f + GRID_X_OFFSET, -0.025f + GRID_Y_OFFSET, HOVERING_Z));
    waypoints_.push_back(makePose(0.245f + GRID_X_OFFSET, 0.020f + GRID_Y_OFFSET, HOVERING_Z));
    waypoints_.push_back(makePose(0.200f + GRID_X_OFFSET, 0.020f + GRID_Y_OFFSET, HOVERING_Z));
    waypoints_.push_back(makePose(0.155f + GRID_X_OFFSET, 0.020f + GRID_Y_OFFSET, HOVERING_Z));
    waypoints_.push_back(makePose(0.245f + GRID_X_OFFSET, 0.065f + GRID_Y_OFFSET, HOVERING_Z));
    waypoints_.push_back(makePose(0.200f + GRID_X_OFFSET, 0.065f + GRID_Y_OFFSET, HOVERING_Z));
    waypoints_.push_back(makePose(0.155f + GRID_X_OFFSET, 0.065f + GRID_Y_OFFSET, HOVERING_Z));
}

// --------------------------------------------------
// Draw grid and fill in all cells
// --------------------------------------------------

void TrajectoryExecutor::drawFullBoard() {
    RCLCPP_INFO(LOGGER, "Drawing grid...");
    drawGrid();

    for (size_t i = 0; i < waypoints_.size(); ++i) {
        if (i % 2 == 0) {
            RCLCPP_INFO(LOGGER, "Drawing circle in cell %zu...", i + 1);
            drawCircle(i + 1);
        } else {
            RCLCPP_INFO(LOGGER, "Drawing cross in cell %zu...", i + 1);
            drawCross(i + 1);
        }
    }
}

// --------------------------------------------------
// Drawing functions
// --------------------------------------------------

void TrajectoryExecutor::drawLine(const Pose& start, float dx, float dy) {
    moveToGoalPose(start);
    moveByDelta(0, 0, -DRAW_Z_OFFSET);
    moveCartesian(dx, dy, 0.0);
    moveByDelta(0, 0, +DRAW_Z_OFFSET);
}

void TrajectoryExecutor::drawGrid() {
    // Vertical line 1
    Pose start = waypoints_[0];
    start.position.x -= GRID_SIZE / 2;
    start.position.y -= GRID_SIZE / 2;
    drawLine(start, 0.0f, 3 * GRID_SIZE);
    // Vertical line 2
    start = waypoints_[1];
    start.position.x -= GRID_SIZE / 2;
    start.position.y -= GRID_SIZE / 2;
    drawLine(start, 0.0f, 3 * GRID_SIZE);
    // Horizontal line 1
    start = waypoints_[0];
    start.position.x += GRID_SIZE / 2;
    start.position.y += GRID_SIZE / 2;
    drawLine(start, -3 * GRID_SIZE, 0.0f);
    // Horizontal line 2
    start = waypoints_[3];
    start.position.x += GRID_SIZE / 2;
    start.position.y += GRID_SIZE / 2;
    drawLine(start, -3 * GRID_SIZE, 0.0f);
}

void TrajectoryExecutor::drawCross(int cell_number) {
    Pose start = waypoints_[cell_number - 1];
    start.position.x += CROSS_WIDTH / 2;
    start.position.y += CROSS_WIDTH / 2;
    drawLine(start, -CROSS_WIDTH, -CROSS_WIDTH);
    start.position.y -= CROSS_WIDTH;
    drawLine(start, -CROSS_WIDTH, +CROSS_WIDTH);
}

void TrajectoryExecutor::drawCircle(int cell_number) {
    Pose center = waypoints_[cell_number - 1];
    center.position.z -= DRAW_Z_OFFSET;

    std::vector<Pose> circle_wps;
    circle_wps.reserve(200);

    for (int i = 0; i < 200; i++) {
        double theta = 2 * M_PI * (i / 200.0);

        Pose p       = center;
        p.position.x = center.position.x + CIRCLE_RADIUS * cos(theta);
        p.position.y = center.position.y + CIRCLE_RADIUS * sin(theta);

        circle_wps.push_back(p);
    }

    Pose first = circle_wps.front();
    first.position.z += DRAW_Z_OFFSET;

    moveToGoalPose(first);

    RobotTrajectory traj;
    double fraction = move_group_robot_->computeCartesianPath(circle_wps, end_effector_step_, jump_threshold_, traj);

    if (fraction < 0.9) {
        RCLCPP_WARN(LOGGER, "Circle trajectory planning failed (%.1f%%)", fraction * 100.0);
        return;
    }

    move_group_robot_->execute(traj);
    moveByDelta(0, 0, DRAW_Z_OFFSET);
}

// --------------------------------------------------
// Motion helpers
// --------------------------------------------------

void TrajectoryExecutor::moveToJointValue(float a0, float a1, float a2, float a3, float a4, float a5) {
    joint_group_positions_robot_[0] = a0;
    joint_group_positions_robot_[1] = a1;
    joint_group_positions_robot_[2] = a2;
    joint_group_positions_robot_[3] = a3;
    joint_group_positions_robot_[4] = a4;
    joint_group_positions_robot_[5] = a5;

    move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
    planAndExecuteKinematic();
}

void TrajectoryExecutor::moveToGoalPose(Pose target) {
    move_group_robot_->setPoseTarget(target);
    planAndExecuteKinematic();
}

void TrajectoryExecutor::moveByDelta(float dx, float dy, float dz) {
    target_pose_robot_ = move_group_robot_->getCurrentPose().pose;

    target_pose_robot_.position.x += dx;
    target_pose_robot_.position.y += dy;
    target_pose_robot_.position.z += dz;

    move_group_robot_->setPoseTarget(target_pose_robot_);
    planAndExecuteKinematic();
}

void TrajectoryExecutor::moveCartesian(float dx, float dy, float dz, float qx, float qy, float qz, float qw) {
    target_pose_robot_ = move_group_robot_->getCurrentPose().pose;
    cartesian_waypoints_.push_back(target_pose_robot_);

    target_pose_robot_.position.x += dx;
    target_pose_robot_.position.y += dy;
    target_pose_robot_.position.z += dz;

    target_pose_robot_.orientation.x += qx;
    target_pose_robot_.orientation.y += qy;
    target_pose_robot_.orientation.z += qz;
    target_pose_robot_.orientation.w += qw;

    cartesian_waypoints_.push_back(target_pose_robot_);
    planAndExecuteCartesian();
}

void TrajectoryExecutor::moveToHome() {
    moveToJointValue(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void TrajectoryExecutor::moveToClear() {
    Pose clear;
    clear.position.x    = 0.227;
    clear.position.y    = -0.112;
    clear.position.z    = 0.212;
    clear.orientation.x = 0.0f;
    clear.orientation.y = 1.0f;
    clear.orientation.z = 0.0f;
    clear.orientation.w = 0.0f;
    moveToGoalPose(clear);
    moveByDelta(0.0f, 0.0f, -0.01f);
    moveByDelta(0.0f, 0.0f, +0.01f);
}

// --------------------------------------------------
// Planning wrappers
// --------------------------------------------------

void TrajectoryExecutor::planAndExecuteKinematic() {
    if (move_group_robot_->plan(kinematics_trajectory_plan_) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group_robot_->execute(kinematics_trajectory_plan_);
        RCLCPP_INFO(LOGGER, "Kinematic trajectory succeeded");
    } else {
        RCLCPP_WARN(LOGGER, "Kinematic trajectory failed");
    }
}

void TrajectoryExecutor::planAndExecuteCartesian() {
    double fraction = move_group_robot_->computeCartesianPath(cartesian_waypoints_, end_effector_step_, jump_threshold_,
                                                              cartesian_trajectory_plan_);

    if (fraction <= 0.9) {
        RCLCPP_WARN(LOGGER, "Cartesian planning failed: %.2f%%", fraction * 100.0);
    } else {
        move_group_robot_->execute(cartesian_trajectory_plan_);
        RCLCPP_INFO(LOGGER, "Cartesian trajectory succeeded");
    }

    cartesian_waypoints_.clear();
}