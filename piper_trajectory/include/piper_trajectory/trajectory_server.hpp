#ifndef PIPER_TRAJECTORY__TRAJECTORY_SERVER_HPP_
#define PIPER_TRAJECTORY__TRAJECTORY_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include "piper_trajectory/srv/execute_trajectory.hpp"
#include "piper_trajectory/trajectory_executor.hpp"

class TrajectoryServer : public rclcpp::Node {
  public:
    TrajectoryServer();

  private:
    void handleService(const std::shared_ptr<piper_trajectory::srv::ExecuteTrajectory::Request> request,
                       std::shared_ptr<piper_trajectory::srv::ExecuteTrajectory::Response> response);

    TrajectoryExecutor trajectory_executor_;

    rclcpp::Service<piper_trajectory::srv::ExecuteTrajectory>::SharedPtr srv_;
};

#endif // PIPER_TRAJECTORY__TRAJECTORY_SERVER_HPP_
