#ifndef PIPER_TRAJECTORY__TRAJECTORY_EXECUTOR_HPP_
#define PIPER_TRAJECTORY__TRAJECTORY_EXECUTOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <memory>
#include <thread>
#include <vector>

class TrajectoryExecutor {
  public:
    using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
    using JointModelGroup    = moveit::core::JointModelGroup;
    using RobotStatePtr      = moveit::core::RobotStatePtr;
    using Plan               = MoveGroupInterface::Plan;
    using Pose               = geometry_msgs::msg::Pose;
    using RobotTrajectory    = moveit_msgs::msg::RobotTrajectory;

    explicit TrajectoryExecutor(rclcpp::Node::SharedPtr base_node);

    Pose makePose(double x, double y, double z);
    void loadWaypoints();
    void drawFullBoard();
    void drawLine(const Pose& start, float dx, float dy);
    void drawGrid();
    void drawCross(int cell_number);
    void drawCircle(int cell_number);
    void drawContour(const std::vector<int>& image_dimensions, const std::vector<int>& contour_points);
    void moveToHome();
    void moveToClear();

  private:
    rclcpp::Node::SharedPtr base_node_;
    rclcpp::Node::SharedPtr move_group_node_;
    rclcpp::executors::SingleThreadedExecutor executor_;

    std::shared_ptr<MoveGroupInterface> move_group_robot_;
    const JointModelGroup* joint_model_group_robot_;

    std::vector<double> joint_group_positions_robot_;
    RobotStatePtr current_state_robot_;
    Plan kinematics_trajectory_plan_;
    Pose target_pose_robot_;

    std::vector<Pose> cartesian_waypoints_;
    RobotTrajectory cartesian_trajectory_plan_;
    std::vector<Pose> waypoints_;
    Pose board_center_;
    Pose board_image_corner_;

    const double end_effector_step_ = 0.005;
    const double jump_threshold_    = 0.0;

    void moveToJointValue(float a0, float a1, float a2, float a3, float a4, float a5);
    void moveToGoalPose(Pose goal_pose);
    void moveByDelta(float dx, float dy, float dz);
    void moveCartesian(float dx, float dy, float dz, float qx = 0.0, float qy = 0.0, float qz = 0.0, float qw = 0.0);

    void planAndExecuteKinematic();
    void planAndExecuteCartesian();
};

#endif // PIPER_TRAJECTORY__TRAJECTORY_EXECUTOR_HPP_
