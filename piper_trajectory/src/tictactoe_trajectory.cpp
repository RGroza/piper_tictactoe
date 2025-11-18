#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

static const rclcpp::Logger LOGGER            = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "arm";

using namespace std::chrono_literals;

namespace {
constexpr float HOVERING_Z    = 0.18f;
constexpr float DRAW_Z_OFFSET = 0.01f;
constexpr float CROSS_WIDTH   = 0.021f;
constexpr float CIRCLE_RADIUS = 0.015f;
constexpr float BOARD_POS_X   = 0.2f;
constexpr float BOARD_WIDTH   = 0.154f;
constexpr float BOARD_HEIGHT  = 0.203f;
} // namespace

class TicTacToeTrajectory {
  public:
    TicTacToeTrajectory(rclcpp::Node::SharedPtr base_node_) : base_node_(base_node_) {
        base_node_->set_parameter(rclcpp::Parameter("use_sim_time", true));

        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);

        move_group_node_ = rclcpp::Node::make_shared("move_group_node", node_options);
        move_group_node_->set_parameter(rclcpp::Parameter("use_sim_time", true));
        executor_.add_node(move_group_node_);
        std::thread([this]() { this->executor_.spin(); }).detach();

        move_group_robot_ = std::make_shared<MoveGroupInterface>(move_group_node_, PLANNING_GROUP_ROBOT);

        joint_model_group_robot_ = move_group_robot_->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ROBOT);

        RCLCPP_INFO(LOGGER, "Planning Frame: %s", move_group_robot_->getPlanningFrame().c_str());
        RCLCPP_INFO(LOGGER, "End Effector Link: %s", move_group_robot_->getEndEffectorLink().c_str());
        RCLCPP_INFO(LOGGER, "Available Planning Groups:");
        std::vector<std::string> group_names = move_group_robot_->getJointModelGroupNames();
        for (long unsigned int i = 0; i < group_names.size(); i++) {
            RCLCPP_INFO(LOGGER, "Group %ld: %s", i, group_names[i].c_str());
        }

        current_state_robot_ = move_group_robot_->getCurrentState(10);
        current_state_robot_->copyJointGroupPositions(joint_model_group_robot_, joint_group_positions_robot_);

        move_group_robot_->setStartStateToCurrentState();

        loadWaypoints();

        RCLCPP_INFO(LOGGER, "TicTacToeTrajectory initialized");
    }

    void loadWaypoints() {
        auto makePose = [&](double x, double y, double z) {
            Pose pose;
            pose.position.x    = x;
            pose.position.y    = y;
            pose.position.z    = z;
            pose.orientation.x = 0.0;
            pose.orientation.y = 1.0;
            pose.orientation.z = 0.0;
            pose.orientation.w = 0.0;
            return pose;
        };

        waypoints_.push_back(makePose(0.245, 0.065, HOVERING_Z));  // 1
        waypoints_.push_back(makePose(0.200, 0.065, HOVERING_Z));  // 2
        waypoints_.push_back(makePose(0.155, 0.065, HOVERING_Z));  // 3
        waypoints_.push_back(makePose(0.245, 0.020, HOVERING_Z));  // 4
        waypoints_.push_back(makePose(0.200, 0.020, HOVERING_Z));  // 5
        waypoints_.push_back(makePose(0.155, 0.020, HOVERING_Z));  // 6
        waypoints_.push_back(makePose(0.245, -0.025, HOVERING_Z)); // 7
        waypoints_.push_back(makePose(0.200, -0.025, HOVERING_Z)); // 8
        waypoints_.push_back(makePose(0.155, -0.025, HOVERING_Z)); // 9
    }

    void executeTrajectoryPlan() {
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            RCLCPP_INFO(LOGGER, "Moving to waypoint %ld...", i + 1);
            moveToGoalPose(waypoints_[i]);

            if (i % 2 == 0) {
                RCLCPP_INFO(LOGGER, "Drawing circle...");
                drawCircle();
            } else {
                RCLCPP_INFO(LOGGER, "Drawing cross...");
                drawCross();
            }
        }

        RCLCPP_INFO(LOGGER, "Returning to home position...");
        moveToJointValue(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    void drawCross() {
        moveByDelta(+CROSS_WIDTH / 2, +CROSS_WIDTH / 2, 0.0);
        moveByDelta(0.0, 0.0, -DRAW_Z_OFFSET);
        moveCartesian(-CROSS_WIDTH, -CROSS_WIDTH, 0.0);
        moveByDelta(0.0, 0.0, +DRAW_Z_OFFSET);
        moveByDelta(+CROSS_WIDTH, 0.0, 0.0);
        moveByDelta(0.0, 0.0, -DRAW_Z_OFFSET);
        moveCartesian(-CROSS_WIDTH, +CROSS_WIDTH, 0.0);
        moveByDelta(0.0, 0.0, +DRAW_Z_OFFSET);
    }

    void drawCircle() {
        std::vector<Pose> waypoints;
        auto center = move_group_robot_->getCurrentPose().pose;
        center.position.z -= DRAW_Z_OFFSET;

        for (int i = 0; i < 200; i++) {
            double theta = 2 * M_PI * (i / 200.0);
            Pose p       = center;
            p.position.x = center.position.x + CIRCLE_RADIUS * cos(theta);
            p.position.y = center.position.y + CIRCLE_RADIUS * sin(theta);
            waypoints.push_back(p);
        }

        Pose hover_pose = waypoints.front();
        hover_pose.position.z += DRAW_Z_OFFSET;
        moveToGoalPose(hover_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction =
            move_group_robot_->computeCartesianPath(waypoints, end_effector_step_, jump_threshold_, trajectory);

        if (fraction < 0.9) {
            RCLCPP_WARN(LOGGER, "Circle trajectory planning failed with only %.2f%% success", fraction * 100.0);
            return;
        }

        move_group_robot_->execute(trajectory);
        moveByDelta(0.0, 0.0, +DRAW_Z_OFFSET);
    }

  private:
    using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
    using JointModelGroup    = moveit::core::JointModelGroup;
    using RobotStatePtr      = moveit::core::RobotStatePtr;
    using Plan               = MoveGroupInterface::Plan;
    using Pose               = geometry_msgs::msg::Pose;
    using RobotTrajectory    = moveit_msgs::msg::RobotTrajectory;

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
    const double end_effector_step_ = 0.005;
    const double jump_threshold_    = 0.0;

    std::vector<Pose> waypoints_;

    void moveToJointValue(float angle0, float angle1, float angle2, float angle3, float angle4, float angle5) {
        joint_group_positions_robot_[0] = angle0;
        joint_group_positions_robot_[1] = angle1;
        joint_group_positions_robot_[2] = angle2;
        joint_group_positions_robot_[3] = angle3;
        joint_group_positions_robot_[4] = angle4;
        joint_group_positions_robot_[5] = angle5;
        move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
        planAndExecuteKinematic();
    }

    void moveToGoalPose(Pose goal_pose) {
        move_group_robot_->setPoseTarget(goal_pose);
        planAndExecuteKinematic();
    }

    void moveByDelta(float delta_x, float delta_y, float delta_z) {
        target_pose_robot_ = move_group_robot_->getCurrentPose().pose;
        target_pose_robot_.position.x += delta_x;
        target_pose_robot_.position.y += delta_y;
        target_pose_robot_.position.z += delta_z;
        move_group_robot_->setPoseTarget(target_pose_robot_);
        planAndExecuteKinematic();
    }

    void moveCartesian(float x_delta, float y_delta, float z_delta, float quat_x_delta = 0.0, float quat_y_delta = 0.0,
                       float quat_z_delta = 0.0, float quat_w_delta = 0.0) {
        target_pose_robot_ = move_group_robot_->getCurrentPose().pose;
        cartesian_waypoints_.push_back(target_pose_robot_);
        target_pose_robot_.position.x += x_delta;
        target_pose_robot_.position.y += y_delta;
        target_pose_robot_.position.z += z_delta;
        target_pose_robot_.orientation.x += quat_x_delta;
        target_pose_robot_.orientation.y += quat_y_delta;
        target_pose_robot_.orientation.z += quat_z_delta;
        target_pose_robot_.orientation.w += quat_w_delta;
        cartesian_waypoints_.push_back(target_pose_robot_);
        planAndExecuteCartesian();
    }

    void planAndExecuteKinematic() {
        if (move_group_robot_->plan(kinematics_trajectory_plan_) == moveit::core::MoveItErrorCode::SUCCESS) {
            move_group_robot_->execute(kinematics_trajectory_plan_);
            RCLCPP_INFO(LOGGER, "Kinematic trajectory succeeded!");
        } else {
            RCLCPP_INFO(LOGGER, "Kinematic trajectory failed!");
        }
    }

    void planAndExecuteCartesian() {
        double plan_fraction = move_group_robot_->computeCartesianPath(cartesian_waypoints_, end_effector_step_,
                                                                       jump_threshold_, cartesian_trajectory_plan_);
        if (plan_fraction <= 0.9) {
            RCLCPP_WARN(LOGGER, "Trajectory planning failed with only %.2f%% success", plan_fraction * 100.0);
        } else {
            move_group_robot_->execute(cartesian_trajectory_plan_);
            RCLCPP_INFO(LOGGER, "Cartesian trajectory succeeded!");
        }
        cartesian_waypoints_.clear();
    }
}; // class TicTacToeTrajectory

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> base_node = std::make_shared<rclcpp::Node>("ttt_trajectory");
    TicTacToeTrajectory ttt_trajectory_node(base_node);
    ttt_trajectory_node.executeTrajectoryPlan();
    rclcpp::shutdown();
    return 0;
}