#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/global_planner/global_planner_interface.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit_msgs/action/global_planner.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>

#include <curobo_msgs/srv/set_planner.hpp>
#include <curobo_msgs/srv/trajectory_generation.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace curobo_hybrid_planning_plugins
{
class CuroboGlobalPlanner : public moveit::hybrid_planning::GlobalPlannerInterface
{
public:
  bool initialize(const rclcpp::Node::SharedPtr& node) override;

  moveit_msgs::msg::MotionPlanResponse plan(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>> global_goal_handle)
      override;

  bool reset() noexcept override;

private:
  bool setPlanner(std::uint8_t planner_type) const;
  bool resolvePlanningGroupJointNames(const std::string& group_name,
                                      std::vector<std::string>& ordered_joint_names) const;
  bool buildStartState(const moveit_msgs::msg::MotionPlanRequest& motion_request,
                       const std::vector<std::string>& ordered_joint_names,
                       sensor_msgs::msg::JointState& start_state_msg) const;
  bool extractGoalPose(const moveit_msgs::msg::Constraints& constraints, geometry_msgs::msg::Pose& pose_out) const;
  bool computeGoalJointPositionsFromIk(const std::string& group_name,
                                       const sensor_msgs::msg::JointState& start_state_msg,
                                       const std::vector<std::string>& ordered_joint_names,
                                       const moveit_msgs::msg::Constraints& constraints,
                                       std::vector<double>& joint_positions_out) const;
  bool extractGoalJointPositions(const moveit_msgs::msg::Constraints& constraints,
                                 const std::vector<std::string>& ordered_joint_names,
                                 std::vector<double>& joint_positions_out) const;
  bool buildTrajectoryResponse(const curobo_msgs::srv::TrajectoryGeneration::Response& curobo_response,
                               const sensor_msgs::msg::JointState& start_state_msg,
                               const std::vector<std::string>& ordered_joint_names,
                               const std::string& group_name,
                               double planning_time_sec,
                               moveit_msgs::msg::MotionPlanResponse& response) const;

  static std::string makeServiceName(const std::string& planner_namespace, const std::string& suffix);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<curobo_msgs::srv::SetPlanner>::SharedPtr set_planner_client_;
  rclcpp::Client<curobo_msgs::srv::TrajectoryGeneration>::SharedPtr trajectory_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  mutable std::mutex latest_joint_state_mutex_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  moveit::core::RobotModelConstPtr robot_model_;
  std::string planner_namespace_;
  std::string joint_states_topic_;
  double service_timeout_sec_{ 5.0 };
};
}  // namespace curobo_hybrid_planning_plugins
