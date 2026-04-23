#pragma once

#include <cstddef>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <moveit/local_planner/trajectory_operator_interface.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>

namespace curobo_hybrid_planning_plugins
{
class LookaheadSampler : public moveit::hybrid_planning::TrajectoryOperatorInterface
{
public:
  bool initialize(const rclcpp::Node::SharedPtr& node,
                  const moveit::core::RobotModelConstPtr& robot_model,
                  const std::string& group_name) override;

  moveit_msgs::action::LocalPlanner::Feedback
  addTrajectorySegment(const robot_trajectory::RobotTrajectory& new_trajectory) override;

  moveit_msgs::action::LocalPlanner::Feedback
  getLocalTrajectory(const moveit::core::RobotState& current_state,
                     robot_trajectory::RobotTrajectory& local_trajectory) override;

  double getTrajectoryProgress(const moveit::core::RobotState& current_state) override;

  bool reset() override;

private:
  static constexpr std::size_t kDefaultLookaheadWaypoints = 6;
  static constexpr std::size_t kDefaultNearestSearchWindow = 20;
  static constexpr double kDefaultReachedTolerance = 0.05;
  static constexpr double kDefaultMaxTargetJointDistance = 0.25;

  bool hasReferenceTrajectory() const;
  std::size_t findNearestWaypointIndex(const moveit::core::RobotState& current_state) const;
  std::size_t clampTargetIndexByDistance(const moveit::core::RobotState& current_state,
                                         std::size_t progress_index,
                                         std::size_t target_index) const;
  moveit_msgs::action::LocalPlanner::Feedback makeFeedback(const std::string& feedback) const;

  // The active global reference trajectory is owned by the protected
  // TrajectoryOperatorInterface::reference_trajectory_ member inherited from MoveIt.
  rclcpp::Node::SharedPtr node_;
  const moveit::core::JointModelGroup* joint_group_{ nullptr };

  std::size_t progress_waypoint_index_{ 0 };
  std::size_t lookahead_waypoints_{ kDefaultLookaheadWaypoints };
  std::size_t nearest_search_window_{ kDefaultNearestSearchWindow };
  double reached_tolerance_{ kDefaultReachedTolerance };
  double max_target_joint_distance_{ kDefaultMaxTargetJointDistance };
};
}  // namespace curobo_hybrid_planning_plugins
