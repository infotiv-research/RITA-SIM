#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <moveit/local_planner/local_constraint_solver_interface.hpp>
#include <moveit/local_planner/feedback_types.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>

#include <curobo_msgs/srv/mpc_reset.hpp>
#include <curobo_msgs/srv/mpc_step.hpp>

namespace curobo_hybrid_planning_plugins
{
class CuroboMpcLocalSolver : public moveit::hybrid_planning::LocalConstraintSolverInterface
{
public:
  bool initialize(const rclcpp::Node::SharedPtr& node,
                  const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                  const std::string& group_name) override;

  moveit_msgs::action::LocalPlanner::Feedback solve(
      const robot_trajectory::RobotTrajectory& local_trajectory,
      std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> local_goal,
      trajectory_msgs::msg::JointTrajectory& local_solution) override;

  bool reset() override;

private:
  bool buildCurrentState(sensor_msgs::msg::JointState& current_state) const;
  bool buildTargetPoseAndJoints(const robot_trajectory::RobotTrajectory& local_trajectory,
                               geometry_msgs::msg::Pose& target_pose,
                               std::vector<double>& target_joint_positions) const;
  moveit_msgs::action::LocalPlanner::Feedback makeFeedback(const std::string& feedback) const;
  bool callReset(bool restore_trajectory_controller) const;

  template <typename FutureT>
  bool waitForFuture(FutureT& future, const std::chrono::milliseconds& timeout) const
  {
    const auto start = std::chrono::steady_clock::now();
    while ((std::chrono::steady_clock::now() - start) < timeout)
    {
      if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
      {
        return true;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  rclcpp::Node::SharedPtr node_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  const moveit::core::JointModelGroup* joint_model_group_{ nullptr };
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Client<curobo_msgs::srv::MpcStep>::SharedPtr mpc_step_client_;
  rclcpp::Client<curobo_msgs::srv::MpcReset>::SharedPtr mpc_reset_client_;

  std::string group_name_;
  std::string tip_link_;
  double step_dt_{ 0.03 };
  double service_timeout_sec_{ 1.0 };
  bool active_{ false };
  std::mutex solve_mutex_;
};
}  // namespace curobo_hybrid_planning_plugins
