#include "curobo_hybrid_planning_plugins/curobo_mpc_local_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace curobo_hybrid_planning_plugins
{
bool CuroboMpcLocalSolver::initialize(const rclcpp::Node::SharedPtr& node,
                                      const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                      const std::string& group_name)
{
  node_ = node;
  planning_scene_monitor_ = planning_scene_monitor;
  group_name_ = group_name;

  joint_model_group_ = planning_scene_monitor_->getRobotModel()->getJointModelGroup(group_name_);
  if (joint_model_group_ == nullptr)
  {
    RCLCPP_ERROR(node_->get_logger(), "Unknown planning group '%s' for cuRobo MPC local solver.", group_name_.c_str());
    return false;
  }

  const auto link_names = joint_model_group_->getLinkModelNames();
  tip_link_ = node_->declare_parameter<std::string>(
      "hybrid_mpc_tip_link",
      link_names.empty() ? std::string() : link_names.back());
  step_dt_ = node_->declare_parameter<double>("hybrid_mpc_step_dt", 0.03);
  service_timeout_sec_ = node_->declare_parameter<double>("hybrid_mpc_service_timeout_sec", 1.0);
  min_target_joint_delta_ = node_->declare_parameter<double>("hybrid_mpc_min_target_joint_delta", 0.01);
  const std::string step_service_name = node_->declare_parameter<std::string>(
      "hybrid_mpc_step_service", "/unified_planner/mpc_step");
  const std::string reset_service_name = node_->declare_parameter<std::string>(
      "hybrid_mpc_reset_service", "/unified_planner/mpc_reset");

  // The local-planner timer callback blocks while waiting for the MPC step response.
  // Keep the service clients in a separate callback group so the response can be processed
  // concurrently by the multithreaded component container.
  service_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  mpc_step_client_ = node_->create_client<curobo_msgs::srv::MpcStep>(step_service_name,
                                                                     rmw_qos_profile_services_default,
                                                                     service_callback_group_);
  mpc_reset_client_ = node_->create_client<curobo_msgs::srv::MpcReset>(reset_service_name,
                                                                       rmw_qos_profile_services_default,
                                                                       service_callback_group_);

  const auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(service_timeout_sec_));
  if (!mpc_step_client_->wait_for_service(timeout))
  {
    RCLCPP_ERROR(node_->get_logger(), "Hybrid MPC step service '%s' is unavailable.", step_service_name.c_str());
    return false;
  }
  if (!mpc_reset_client_->wait_for_service(timeout))
  {
    RCLCPP_ERROR(node_->get_logger(), "Hybrid MPC reset service '%s' is unavailable.", reset_service_name.c_str());
    return false;
  }
  return true;
}

moveit_msgs::action::LocalPlanner::Feedback CuroboMpcLocalSolver::solve(
    const robot_trajectory::RobotTrajectory& local_trajectory,
    std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> local_goal,
    trajectory_msgs::msg::JointTrajectory& local_solution)
{
  (void)local_goal;

  // Prevent overlapping MPC calls when the timer fires faster than the solver runs.
  std::unique_lock<std::mutex> lock(solve_mutex_, std::try_to_lock);
  if (!lock.owns_lock())
  {
    return makeFeedback("");  // skip this tick, previous solve still in progress
  }

  sensor_msgs::msg::JointState current_state;
  if (!buildCurrentState(current_state))
  {
    active_ = false;
    return makeFeedback(std::string(moveit::hybrid_planning::toString(
        moveit::hybrid_planning::LocalFeedbackEnum::LOCAL_PLANNER_STUCK)));
  }

  geometry_msgs::msg::Pose target_pose;
  std::vector<double> target_joint_positions;
  if (!buildTargetPoseAndJoints(local_trajectory, current_state, target_pose, target_joint_positions))
  {
    active_ = false;
    return makeFeedback(std::string(moveit::hybrid_planning::toString(
        moveit::hybrid_planning::LocalFeedbackEnum::LOCAL_PLANNER_STUCK)));
  }

  double target_max_delta = 0.0;
  if (target_joint_positions.size() == current_state.position.size())
  {
    for (std::size_t joint_index = 0; joint_index < target_joint_positions.size(); ++joint_index)
    {
      target_max_delta = std::max(target_max_delta,
                                  std::abs(target_joint_positions[joint_index] - current_state.position[joint_index]));
    }
  }
  RCLCPP_DEBUG_THROTTLE(node_->get_logger(),
                        *node_->get_clock(),
                        1000,
                        "Hybrid MPC local target: max_joint_delta=%.4f, active=%s",
                        target_max_delta,
                        active_ ? "true" : "false");

  auto request = std::make_shared<curobo_msgs::srv::MpcStep::Request>();
  request->current_state = current_state;
  request->target_pose = target_pose;
  request->target_joint_positions = target_joint_positions;
  request->initialize_if_needed = !active_;

  auto future = mpc_step_client_->async_send_request(request);
  const auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(service_timeout_sec_));
  if (!waitForFuture(future, timeout))
  {
    RCLCPP_WARN(node_->get_logger(),
                "Timed out waiting %.3fs for hybrid MPC step response. Reporting local planner stuck.",
                service_timeout_sec_);
    active_ = false;
    return makeFeedback(std::string(moveit::hybrid_planning::toString(
        moveit::hybrid_planning::LocalFeedbackEnum::LOCAL_PLANNER_STUCK)));
  }

  const auto response = future.get();
  if (!response || !response->success)
  {
    RCLCPP_WARN(node_->get_logger(),
                "Hybrid MPC step request failed: %s",
                response ? response->message.c_str() : "no response");
    active_ = false;
    return makeFeedback(std::string(moveit::hybrid_planning::toString(
        moveit::hybrid_planning::LocalFeedbackEnum::LOCAL_PLANNER_STUCK)));
  }

  if (response->path_invalidated)
  {
    RCLCPP_WARN(node_->get_logger(), "Hybrid MPC reported path invalidated.");
    active_ = false;
    return makeFeedback(std::string(moveit::hybrid_planning::toString(
        moveit::hybrid_planning::LocalFeedbackEnum::COLLISION_AHEAD)));
  }

  local_solution.joint_names = response->joint_command.name;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = response->joint_command.position;
  point.velocities = response->joint_command.velocity;
  point.time_from_start = rclcpp::Duration::from_seconds(step_dt_);
  local_solution.points.clear();
  local_solution.points.push_back(point);
  active_ = true;
  return makeFeedback("");
}

bool CuroboMpcLocalSolver::reset()
{
  const bool success = callReset(true);
  active_ = false;
  return success;
}

bool CuroboMpcLocalSolver::buildCurrentState(sensor_msgs::msg::JointState& current_state) const
{
  const moveit::core::RobotState robot_state = [this] {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    return scene->getCurrentState();
  }();

  std::vector<double> positions;
  std::vector<double> velocities;
  robot_state.copyJointGroupPositions(joint_model_group_, positions);
  robot_state.copyJointGroupVelocities(joint_model_group_, velocities);
  current_state.name = joint_model_group_->getVariableNames();
  current_state.position = positions;
  if (velocities.size() == positions.size())
  {
    current_state.velocity = velocities;
  }
  else
  {
    current_state.velocity.assign(positions.size(), 0.0);
  }
  return !current_state.name.empty() && current_state.name.size() == current_state.position.size();
}

bool CuroboMpcLocalSolver::buildTargetPoseAndJoints(const robot_trajectory::RobotTrajectory& local_trajectory,
                                                    const sensor_msgs::msg::JointState& current_state,
                                                    geometry_msgs::msg::Pose& target_pose,
                                                    std::vector<double>& target_joint_positions) const
{
  if (local_trajectory.getWayPointCount() == 0 || tip_link_.empty())
  {
    return false;
  }

  const std::size_t waypoint_count = local_trajectory.getWayPointCount();
  const auto& current_positions = current_state.position;
  std::size_t target_index = waypoint_count - 1;

  for (std::size_t index = 0; index < waypoint_count; ++index)
  {
    std::vector<double> waypoint_positions;
    local_trajectory.getWayPoint(index).copyJointGroupPositions(joint_model_group_, waypoint_positions);
    if (waypoint_positions.size() != current_positions.size())
    {
      continue;
    }

    double max_delta = 0.0;
    for (std::size_t joint_index = 0; joint_index < waypoint_positions.size(); ++joint_index)
    {
      max_delta = std::max(max_delta, std::abs(waypoint_positions[joint_index] - current_positions[joint_index]));
    }

    if (max_delta > min_target_joint_delta_)
    {
      target_index = index;
      break;
    }
  }

  const moveit::core::RobotState& target_state = local_trajectory.getWayPoint(target_index);
  target_state.copyJointGroupPositions(joint_model_group_, target_joint_positions);
  if (target_joint_positions.empty())
  {
    return false;
  }

  const Eigen::Isometry3d& ee_transform = target_state.getGlobalLinkTransform(tip_link_);
  const Eigen::Quaterniond ee_quaternion(ee_transform.rotation());
  target_pose.position.x = ee_transform.translation().x();
  target_pose.position.y = ee_transform.translation().y();
  target_pose.position.z = ee_transform.translation().z();
  target_pose.orientation.x = ee_quaternion.x();
  target_pose.orientation.y = ee_quaternion.y();
  target_pose.orientation.z = ee_quaternion.z();
  target_pose.orientation.w = ee_quaternion.w();
  return true;
}

moveit_msgs::action::LocalPlanner::Feedback CuroboMpcLocalSolver::makeFeedback(const std::string& feedback) const
{
  moveit_msgs::action::LocalPlanner::Feedback message;
  message.feedback = feedback;
  return message;
}

bool CuroboMpcLocalSolver::callReset(bool restore_trajectory_controller) const
{
  auto request = std::make_shared<curobo_msgs::srv::MpcReset::Request>();
  request->restore_trajectory_controller = restore_trajectory_controller;
  auto future = mpc_reset_client_->async_send_request(request);
  const auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(service_timeout_sec_));
  if (!waitForFuture(future, timeout))
  {
    RCLCPP_WARN(node_->get_logger(),
                "Timed out waiting %.3fs for hybrid MPC reset response.",
                service_timeout_sec_);
    return false;
  }
  const auto response = future.get();
  return response && response->success;
}
}  // namespace curobo_hybrid_planning_plugins

PLUGINLIB_EXPORT_CLASS(curobo_hybrid_planning_plugins::CuroboMpcLocalSolver,
                       moveit::hybrid_planning::LocalConstraintSolverInterface)
