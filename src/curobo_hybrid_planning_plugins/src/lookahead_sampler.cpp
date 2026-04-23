#include "curobo_hybrid_planning_plugins/lookahead_sampler.hpp"

#include <algorithm>
#include <limits>
#include <memory>

#include <pluginlib/class_list_macros.hpp>

namespace curobo_hybrid_planning_plugins
{
bool LookaheadSampler::initialize(const rclcpp::Node::SharedPtr& node,
                                  const moveit::core::RobotModelConstPtr& robot_model,
                                  const std::string& group_name)
{
  node_ = node;
  if (!node_)
  {
    return false;
  }

  if (!robot_model)
  {
    RCLCPP_ERROR(node_->get_logger(), "LookaheadSampler received an empty robot model.");
    return false;
  }

  joint_group_ = robot_model->getJointModelGroup(group_name);
  if (joint_group_ == nullptr)
  {
    RCLCPP_ERROR(node_->get_logger(), "Unknown planning group '%s' for LookaheadSampler.", group_name.c_str());
    return false;
  }

  const int lookahead_waypoints =
      node_->declare_parameter<int>("lookahead_waypoints", static_cast<int>(kDefaultLookaheadWaypoints));
  const int nearest_search_window =
      node_->declare_parameter<int>("lookahead_nearest_search_window", static_cast<int>(kDefaultNearestSearchWindow));
  lookahead_waypoints_ = static_cast<std::size_t>(std::max(0, lookahead_waypoints));
  nearest_search_window_ = static_cast<std::size_t>(std::max(1, nearest_search_window));
  reached_tolerance_ =
      std::max(0.0, node_->declare_parameter<double>("lookahead_reached_tolerance", kDefaultReachedTolerance));
  max_target_joint_distance_ = std::max(
      0.0, node_->declare_parameter<double>("lookahead_max_target_joint_distance", kDefaultMaxTargetJointDistance));

  RCLCPP_INFO(node_->get_logger(),
              "LookaheadSampler initialized for group '%s': lookahead_waypoints=%zu, "
              "nearest_search_window=%zu, reached_tolerance=%.3f, max_target_joint_distance=%.3f",
              group_name.c_str(),
              lookahead_waypoints_,
              nearest_search_window_,
              reached_tolerance_,
              max_target_joint_distance_);
  return true;
}

moveit_msgs::action::LocalPlanner::Feedback
LookaheadSampler::addTrajectorySegment(const robot_trajectory::RobotTrajectory& new_trajectory)
{
  if (new_trajectory.getWayPointCount() == 0)
  {
    RCLCPP_WARN(node_->get_logger(),
                "LookaheadSampler received an empty trajectory segment; keeping the previous reference trajectory.");
    return makeFeedback("LookaheadSampler ignored an empty trajectory segment.");
  }

  reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(new_trajectory);
  progress_waypoint_index_ = 0;
  return makeFeedback("");
}

moveit_msgs::action::LocalPlanner::Feedback
LookaheadSampler::getLocalTrajectory(const moveit::core::RobotState& current_state,
                                     robot_trajectory::RobotTrajectory& local_trajectory)
{
  local_trajectory.clear();

  if (!hasReferenceTrajectory())
  {
    return makeFeedback("LookaheadSampler has no reference trajectory.");
  }

  const std::size_t final_index = reference_trajectory_->getWayPointCount() - 1;
  if (final_index == 0)
  {
    progress_waypoint_index_ = 0;
    local_trajectory.addSuffixWayPoint(reference_trajectory_->getWayPoint(0),
                                       reference_trajectory_->getWayPointDurationFromPrevious(0));
    return makeFeedback("");
  }

  const std::size_t nearest_index = findNearestWaypointIndex(current_state);
  progress_waypoint_index_ = std::max(progress_waypoint_index_, nearest_index);

  double progress_distance = reference_trajectory_->getWayPoint(progress_waypoint_index_).distance(current_state,
                                                                                                   joint_group_);
  if (progress_waypoint_index_ < final_index && progress_distance <= reached_tolerance_)
  {
    ++progress_waypoint_index_;
    progress_distance = reference_trajectory_->getWayPoint(progress_waypoint_index_).distance(current_state,
                                                                                              joint_group_);
  }

  const std::size_t requested_target_index = std::min(progress_waypoint_index_ + lookahead_waypoints_, final_index);
  const std::size_t target_index = clampTargetIndexByDistance(current_state,
                                                              progress_waypoint_index_,
                                                              requested_target_index);
  const double target_distance = reference_trajectory_->getWayPoint(target_index).distance(current_state,
                                                                                           joint_group_);
  RCLCPP_DEBUG_THROTTLE(node_->get_logger(),
                        *node_->get_clock(),
                        1000,
                        "LookaheadSampler target: nearest=%zu, progress=%zu, requested=%zu, target=%zu/%zu, "
                        "progress_dist=%.4f, target_dist=%.4f",
                        nearest_index,
                        progress_waypoint_index_,
                        requested_target_index,
                        target_index,
                        final_index,
                        progress_distance,
                        target_distance);
  local_trajectory.addSuffixWayPoint(reference_trajectory_->getWayPoint(target_index),
                                     reference_trajectory_->getWayPointDurationFromPrevious(target_index));
  return makeFeedback("");
}

double LookaheadSampler::getTrajectoryProgress(const moveit::core::RobotState& current_state)
{
  if (!hasReferenceTrajectory())
  {
    return 0.0;
  }

  const std::size_t final_index = reference_trajectory_->getWayPointCount() - 1;
  if (final_index == 0)
  {
    return reference_trajectory_->getWayPoint(0).distance(current_state, joint_group_) <= reached_tolerance_ ? 1.0 : 0.0;
  }

  if (reference_trajectory_->getWayPoint(final_index).distance(current_state, joint_group_) <= reached_tolerance_)
  {
    return 1.0;
  }

  const std::size_t nearest_index = findNearestWaypointIndex(current_state);
  const std::size_t estimated_progress = std::max(progress_waypoint_index_, nearest_index);
  return static_cast<double>(std::min(estimated_progress, final_index)) / static_cast<double>(final_index);
}

bool LookaheadSampler::reset()
{
  reference_trajectory_.reset();
  progress_waypoint_index_ = 0;
  return true;
}

bool LookaheadSampler::hasReferenceTrajectory() const
{
  return reference_trajectory_ && reference_trajectory_->getWayPointCount() > 0;
}

std::size_t LookaheadSampler::findNearestWaypointIndex(const moveit::core::RobotState& current_state) const
{
  if (!hasReferenceTrajectory())
  {
    return 0;
  }

  const std::size_t final_index = reference_trajectory_->getWayPointCount() - 1;
  const std::size_t start_index = std::min(progress_waypoint_index_, final_index);
  const std::size_t end_index = std::min(final_index, start_index + nearest_search_window_);

  std::size_t nearest_index = start_index;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = start_index; index <= end_index; ++index)
  {
    const double distance = reference_trajectory_->getWayPoint(index).distance(current_state, joint_group_);
    if (distance < nearest_distance)
    {
      nearest_distance = distance;
      nearest_index = index;
    }
  }
  return nearest_index;
}

std::size_t LookaheadSampler::clampTargetIndexByDistance(const moveit::core::RobotState& current_state,
                                                         std::size_t progress_index,
                                                         std::size_t target_index) const
{
  if (!hasReferenceTrajectory() || max_target_joint_distance_ <= 0.0)
  {
    return target_index;
  }

  while (target_index > progress_index &&
         reference_trajectory_->getWayPoint(target_index).distance(current_state, joint_group_) >
             max_target_joint_distance_)
  {
    --target_index;
  }
  return target_index;
}

moveit_msgs::action::LocalPlanner::Feedback LookaheadSampler::makeFeedback(const std::string& feedback) const
{
  moveit_msgs::action::LocalPlanner::Feedback message;
  message.feedback = feedback;
  return message;
}
}  // namespace curobo_hybrid_planning_plugins

PLUGINLIB_EXPORT_CLASS(curobo_hybrid_planning_plugins::LookaheadSampler,
                       moveit::hybrid_planning::TrajectoryOperatorInterface)
