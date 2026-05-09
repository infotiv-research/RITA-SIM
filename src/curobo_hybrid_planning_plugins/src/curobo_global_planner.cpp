#include "curobo_hybrid_planning_plugins/curobo_global_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <unordered_map>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace
{
template <typename FutureT>
bool waitForFuture(FutureT& future, const std::chrono::milliseconds& timeout)
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

std::vector<double> reorderPositions(const sensor_msgs::msg::JointState& joint_state,
                                     const std::vector<std::string>& ordered_joint_names)
{
  std::unordered_map<std::string, std::size_t> index_by_name;
  for (std::size_t index = 0; index < joint_state.name.size(); ++index)
  {
    index_by_name.emplace(joint_state.name[index], index);
  }

  std::vector<double> ordered_positions;
  ordered_positions.reserve(ordered_joint_names.size());
  for (const auto& joint_name : ordered_joint_names)
  {
    const auto it = index_by_name.find(joint_name);
    if (it == index_by_name.end() || it->second >= joint_state.position.size())
    {
      throw std::runtime_error("Missing joint '" + joint_name + "' in cuRobo trajectory waypoint.");
    }
    ordered_positions.push_back(joint_state.position[it->second]);
  }
  return ordered_positions;
}
}  // namespace

namespace curobo_hybrid_planning_plugins
{
using moveit_msgs::msg::MoveItErrorCodes;

bool CuroboGlobalPlanner::initialize(const rclcpp::Node::SharedPtr& node)
{
  node_ = node;
  planner_namespace_ = node_->declare_parameter<std::string>("curobo_planner_namespace", "/unified_planner");
  joint_states_topic_ = node_->declare_parameter<std::string>("joint_states_topic", "/moveit_joint_states");
  service_timeout_sec_ = node_->declare_parameter<double>("service_timeout_sec", 5.0);

  set_planner_client_ = node_->create_client<curobo_msgs::srv::SetPlanner>(
      makeServiceName(planner_namespace_, "set_planner"));
  trajectory_client_ = node_->create_client<curobo_msgs::srv::TrajectoryGeneration>(
      makeServiceName(planner_namespace_, "generate_trajectory"));
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::scoped_lock<std::mutex> lock(latest_joint_state_mutex_);
        latest_joint_state_ = msg;
      });

  robot_model_loader::RobotModelLoader model_loader(node_, "robot_description");
  robot_model_ = model_loader.getModel();
  if (!robot_model_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load MoveIt robot model for cuRobo global planner.");
    return false;
  }

  const auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(service_timeout_sec_));
  if (!set_planner_client_->wait_for_service(timeout))
  {
    RCLCPP_ERROR(node_->get_logger(), "cuRobo set_planner service is unavailable.");
    return false;
  }
  if (!trajectory_client_->wait_for_service(timeout))
  {
    RCLCPP_ERROR(node_->get_logger(), "cuRobo generate_trajectory service is unavailable.");
    return false;
  }
  return true;
}

moveit_msgs::msg::MotionPlanResponse CuroboGlobalPlanner::plan(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>> global_goal_handle)
{
  moveit_msgs::msg::MotionPlanResponse response;
  response.error_code.val = MoveItErrorCodes::FAILURE;

  const auto started_at = std::chrono::steady_clock::now();
  const auto goal = global_goal_handle->get_goal();
  if (!goal || goal->motion_sequence.items.empty())
  {
    response.error_code.val = MoveItErrorCodes::INVALID_MOTION_PLAN;
    return response;
  }

  const auto& motion_request = goal->motion_sequence.items.front().req;
  const std::string group_name = motion_request.group_name.empty() ? goal->planning_group : motion_request.group_name;
  std::vector<std::string> ordered_joint_names;
  if (!resolvePlanningGroupJointNames(group_name, ordered_joint_names))
  {
    RCLCPP_ERROR(node_->get_logger(), "Unable to resolve active joints for planning group '%s'.",
                 group_name.c_str());
    response.error_code.val = MoveItErrorCodes::INVALID_GROUP_NAME;
    return response;
  }

  sensor_msgs::msg::JointState start_state_msg;
  if (!buildStartState(motion_request, ordered_joint_names, start_state_msg))
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to build a valid start state for planning group '%s'.",
                 group_name.c_str());
    response.error_code.val = MoveItErrorCodes::START_STATE_INVALID;
    return response;
  }

  if (motion_request.goal_constraints.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt request did not include any goal constraints.");
    response.error_code.val = MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED;
    return response;
  }

  const auto& goal_constraints = motion_request.goal_constraints.front();
  auto request = std::make_shared<curobo_msgs::srv::TrajectoryGeneration::Request>();
  request->start_pose = start_state_msg;
  std::vector<double> goal_joint_positions;
  bool use_joint_space_planner = false;
  bool use_pose_planner = false;
  if (extractGoalJointPositions(goal_constraints, ordered_joint_names, goal_joint_positions))
  {
    request->target_joint_positions = goal_joint_positions;
    use_joint_space_planner = true;
  }
  else if (extractGoalPose(goal_constraints, request->target_pose))
  {
    use_pose_planner = true;
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "MoveIt goal constraints for planning group '%s' could not be translated into a cuRobo goal.",
                 group_name.c_str());
    response.error_code.val = MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED;
    return response;
  }

  const auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(service_timeout_sec_));
  auto send_curobo_request =
      [&](const std::shared_ptr<curobo_msgs::srv::TrajectoryGeneration::Request>& trajectory_request,
          std::uint8_t planner_type, const std::string& planner_label,
          curobo_msgs::srv::TrajectoryGeneration::Response::SharedPtr& service_response_out,
          int& error_code_out) -> bool {
    if (!setPlanner(planner_type))
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to switch cuRobo into %s planning mode.",
                   planner_label.c_str());
      error_code_out = MoveItErrorCodes::FAILURE;
      return false;
    }

    auto future = trajectory_client_->async_send_request(trajectory_request);
    if (!waitForFuture(future, timeout))
    {
      RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for cuRobo %s trajectory generation.",
                   planner_label.c_str());
      error_code_out = MoveItErrorCodes::TIMED_OUT;
      return false;
    }

    service_response_out = future.get();
    if (!service_response_out || !service_response_out->success)
    {
      RCLCPP_ERROR(node_->get_logger(), "cuRobo %s global planning failed: %s",
                   planner_label.c_str(),
                   service_response_out ? service_response_out->message.c_str() : "no response");
      error_code_out = MoveItErrorCodes::PLANNING_FAILED;
      return false;
    }
    return true;
  };

  curobo_msgs::srv::TrajectoryGeneration::Response::SharedPtr service_response;
  int curobo_error_code = MoveItErrorCodes::FAILURE;
  const std::uint8_t planner_type = use_joint_space_planner
                                        ? curobo_msgs::srv::SetPlanner::Request::JOINT_SPACE
                                        : curobo_msgs::srv::SetPlanner::Request::CLASSIC;
  const std::string planner_label = use_joint_space_planner ? "joint-space" : "classic pose";
  bool planned = send_curobo_request(
      request, planner_type, planner_label, service_response, curobo_error_code);

  if (!planned && use_pose_planner &&
      computeGoalJointPositionsFromIk(group_name, start_state_msg, ordered_joint_names,
                                      goal_constraints, goal_joint_positions))
  {
    auto ik_request = std::make_shared<curobo_msgs::srv::TrajectoryGeneration::Request>();
    ik_request->start_pose = start_state_msg;
    ik_request->target_joint_positions = goal_joint_positions;
    RCLCPP_WARN(node_->get_logger(),
                "cuRobo pose planning failed for planning group '%s'; retrying via MoveIt IK joint-space fallback.",
                group_name.c_str());
    planned = send_curobo_request(
        ik_request, curobo_msgs::srv::SetPlanner::Request::JOINT_SPACE,
        "joint-space IK fallback", service_response, curobo_error_code);
  }

  if (!planned)
  {
    response.error_code.val = curobo_error_code;
    return response;
  }

  const double planning_time_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at).count();
  if (!buildTrajectoryResponse(*service_response, start_state_msg, ordered_joint_names, group_name,
                               planning_time_sec, response))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to translate the cuRobo trajectory back into a MoveIt response.");
    response.error_code.val = MoveItErrorCodes::FAILURE;
    return response;
  }

  response.error_code.val = MoveItErrorCodes::SUCCESS;
  return response;
}

bool CuroboGlobalPlanner::reset() noexcept
{
  return true;
}

bool CuroboGlobalPlanner::setPlanner(std::uint8_t planner_type) const
{
  auto request = std::make_shared<curobo_msgs::srv::SetPlanner::Request>();
  request->planner_type = planner_type;
  auto future = set_planner_client_->async_send_request(request);
  const auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(service_timeout_sec_));
  if (!waitForFuture(future, timeout))
  {
    RCLCPP_ERROR(node_->get_logger(), "Timed out while switching cuRobo planner.");
    return false;
  }

  const auto response = future.get();
  if (!response || !response->success)
  {
    RCLCPP_ERROR(node_->get_logger(), "cuRobo planner switch failed.");
    return false;
  }
  return true;
}

bool CuroboGlobalPlanner::resolvePlanningGroupJointNames(const std::string& group_name,
                                                         std::vector<std::string>& ordered_joint_names) const
{
  ordered_joint_names.clear();
  if (!robot_model_)
  {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt robot model is unavailable in the cuRobo global planner.");
    return false;
  }

  const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group)
  {
    RCLCPP_ERROR(node_->get_logger(), "Unknown MoveIt planning group '%s'.", group_name.c_str());
    return false;
  }

  ordered_joint_names = joint_model_group->getVariableNames();
  if (ordered_joint_names.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Planning group '%s' does not expose any active joint variables.",
                 group_name.c_str());
    return false;
  }
  return true;
}

bool CuroboGlobalPlanner::buildStartState(const moveit_msgs::msg::MotionPlanRequest& motion_request,
                                          const std::vector<std::string>& ordered_joint_names,
                                          sensor_msgs::msg::JointState& start_state_msg) const
{
  sensor_msgs::msg::JointState latest_joint_state;
  {
    std::scoped_lock<std::mutex> lock(latest_joint_state_mutex_);
    if (latest_joint_state_)
    {
      latest_joint_state = *latest_joint_state_;
    }
  }

  const auto& request_joint_state = motion_request.start_state.joint_state;
  const bool request_has_state = !request_joint_state.name.empty() &&
                                 request_joint_state.name.size() == request_joint_state.position.size();
  const bool live_has_state = !latest_joint_state.name.empty() &&
                              latest_joint_state.name.size() == latest_joint_state.position.size();
  if (!request_has_state && !live_has_state)
  {
    RCLCPP_ERROR(node_->get_logger(), "No start state is available for cuRobo global planning.");
    return false;
  }

  start_state_msg.name = ordered_joint_names;

  std::unordered_map<std::string, double> position_by_name;
  if (request_has_state)
  {
    for (std::size_t index = 0; index < request_joint_state.name.size(); ++index)
    {
      position_by_name[request_joint_state.name[index]] = request_joint_state.position[index];
    }
  }
  if (live_has_state)
  {
    // Hybrid replanning must start from the measured robot state. The original
    // motion request can still carry the stale start configuration from the
    // first global plan, so let live data override request values.
    for (std::size_t index = 0; index < latest_joint_state.name.size(); ++index)
    {
      position_by_name[latest_joint_state.name[index]] = latest_joint_state.position[index];
    }
  }

  start_state_msg.position.reserve(ordered_joint_names.size());
  for (const auto& joint_name : ordered_joint_names)
  {
    const auto it = position_by_name.find(joint_name);
    if (it == position_by_name.end())
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Start state is missing active planning-group joint '%s'.",
                   joint_name.c_str());
      return false;
    }
    start_state_msg.position.push_back(it->second);
  }
  return true;
}

bool CuroboGlobalPlanner::extractGoalPose(const moveit_msgs::msg::Constraints& constraints,
                                          geometry_msgs::msg::Pose& pose_out) const
{
  if (constraints.position_constraints.empty())
  {
    return false;
  }

  const auto& position_constraint = constraints.position_constraints.front();
  if (position_constraint.constraint_region.primitive_poses.empty())
  {
    return false;
  }

  pose_out = position_constraint.constraint_region.primitive_poses.front();
  if (!constraints.orientation_constraints.empty())
  {
    pose_out.orientation = constraints.orientation_constraints.front().orientation;
  }
  else if (std::abs(pose_out.orientation.w) < 1e-9 && std::abs(pose_out.orientation.x) < 1e-9 &&
           std::abs(pose_out.orientation.y) < 1e-9 && std::abs(pose_out.orientation.z) < 1e-9)
  {
    pose_out.orientation.w = 1.0;
  }
  return true;
}

bool CuroboGlobalPlanner::computeGoalJointPositionsFromIk(
    const std::string& group_name,
    const sensor_msgs::msg::JointState& start_state_msg,
    const std::vector<std::string>& ordered_joint_names,
    const moveit_msgs::msg::Constraints& constraints,
    std::vector<double>& joint_positions_out) const
{
  geometry_msgs::msg::Pose goal_pose;
  if (!extractGoalPose(constraints, goal_pose) || !robot_model_)
  {
    return false;
  }

  const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group)
  {
    return false;
  }

  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  if (!start_state_msg.name.empty() && start_state_msg.name.size() == start_state_msg.position.size())
  {
    robot_state.setVariablePositions(start_state_msg.name, start_state_msg.position);
  }
  robot_state.update();

  std::string ik_link_name;
  std::string goal_frame = robot_model_->getModelFrame();
  if (!constraints.orientation_constraints.empty())
  {
    const auto& orientation_constraint = constraints.orientation_constraints.front();
    if (!orientation_constraint.link_name.empty())
    {
      ik_link_name = orientation_constraint.link_name;
    }
    if (!orientation_constraint.header.frame_id.empty())
    {
      goal_frame = orientation_constraint.header.frame_id;
    }
  }
  if (ik_link_name.empty() && !constraints.position_constraints.empty())
  {
    const auto& position_constraint = constraints.position_constraints.front();
    if (!position_constraint.link_name.empty())
    {
      ik_link_name = position_constraint.link_name;
    }
    if (!position_constraint.header.frame_id.empty())
    {
      goal_frame = position_constraint.header.frame_id;
    }
  }

  if (!goal_frame.empty() && goal_frame != robot_model_->getModelFrame())
  {
    RCLCPP_WARN(node_->get_logger(),
                "RViz goal frame '%s' does not match MoveIt model frame '%s'. "
                "Attempting IK without frame conversion.",
                goal_frame.c_str(), robot_model_->getModelFrame().c_str());
  }

  const double ik_timeout_sec = std::max(0.1, service_timeout_sec_);
  const bool ik_success = ik_link_name.empty()
                              ? robot_state.setFromIK(joint_model_group, goal_pose, ik_timeout_sec)
                              : robot_state.setFromIK(joint_model_group, goal_pose, ik_link_name, ik_timeout_sec);
  if (!ik_success)
  {
    RCLCPP_WARN(node_->get_logger(),
                "MoveIt IK fallback failed for planning group '%s' link '%s' at pose "
                "[%.3f, %.3f, %.3f | %.3f, %.3f, %.3f, %.3f].",
                group_name.c_str(), ik_link_name.c_str(),
                goal_pose.position.x, goal_pose.position.y, goal_pose.position.z,
                goal_pose.orientation.x, goal_pose.orientation.y,
                goal_pose.orientation.z, goal_pose.orientation.w);
    return false;
  }

  joint_positions_out.clear();
  robot_state.copyJointGroupPositions(joint_model_group, joint_positions_out);
  if (joint_positions_out.size() != ordered_joint_names.size())
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "MoveIt IK fallback returned %zu joint values, expected %zu for planning group '%s'.",
                 joint_positions_out.size(), ordered_joint_names.size(), group_name.c_str());
    joint_positions_out.clear();
    return false;
  }

  RCLCPP_INFO(node_->get_logger(),
              "Converted pose goal for planning group '%s' into a joint-space cuRobo fallback request via MoveIt IK.",
              group_name.c_str());
  return true;
}

bool CuroboGlobalPlanner::extractGoalJointPositions(const moveit_msgs::msg::Constraints& constraints,
                                                    const std::vector<std::string>& ordered_joint_names,
                                                    std::vector<double>& joint_positions_out) const
{
  if (constraints.joint_constraints.empty())
  {
    return false;
  }

  std::unordered_map<std::string, double> goal_position_by_name;
  for (const auto& joint_constraint : constraints.joint_constraints)
  {
    goal_position_by_name[joint_constraint.joint_name] = joint_constraint.position;
  }

  joint_positions_out.clear();
  joint_positions_out.reserve(ordered_joint_names.size());
  for (const auto& joint_name : ordered_joint_names)
  {
    const auto it = goal_position_by_name.find(joint_name);
    if (it == goal_position_by_name.end())
    {
      return false;
    }
    joint_positions_out.push_back(it->second);
  }
  return true;
}

bool CuroboGlobalPlanner::buildTrajectoryResponse(
    const curobo_msgs::srv::TrajectoryGeneration::Response& curobo_response,
    const sensor_msgs::msg::JointState& start_state_msg,
    const std::vector<std::string>& ordered_joint_names,
    const std::string& group_name,
    double planning_time_sec,
    moveit_msgs::msg::MotionPlanResponse& response) const
{
  if (curobo_response.trajectory.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "cuRobo did not return any trajectory waypoints.");
    return false;
  }

  response.group_name = group_name;
  response.trajectory_start.joint_state = start_state_msg;
  response.planning_time = planning_time_sec;
  response.trajectory.joint_trajectory.joint_names = ordered_joint_names;

  const double dt = std::max(curobo_response.dt, 1e-3);
  for (std::size_t waypoint_index = 0; waypoint_index < curobo_response.trajectory.size(); ++waypoint_index)
  {
    const auto& waypoint = curobo_response.trajectory[waypoint_index];
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = reorderPositions(waypoint, ordered_joint_names);
    if (!waypoint.velocity.empty())
    {
      if (waypoint.name.empty())
      {
        point.velocities = waypoint.velocity;
      }
      else
      {
        sensor_msgs::msg::JointState velocity_wrapper = waypoint;
        velocity_wrapper.position = waypoint.velocity;
        point.velocities = reorderPositions(velocity_wrapper, ordered_joint_names);
      }
    }
    point.time_from_start = rclcpp::Duration::from_seconds((waypoint_index + 1) * dt);
    response.trajectory.joint_trajectory.points.push_back(point);
  }
  return true;
}

std::string CuroboGlobalPlanner::makeServiceName(const std::string& planner_namespace, const std::string& suffix)
{
  if (planner_namespace.empty() || planner_namespace == "/")
  {
    return "/" + suffix;
  }
  if (planner_namespace.back() == '/')
  {
    return planner_namespace + suffix;
  }
  return planner_namespace + "/" + suffix;
}
}  // namespace curobo_hybrid_planning_plugins

PLUGINLIB_EXPORT_CLASS(curobo_hybrid_planning_plugins::CuroboGlobalPlanner,
                       moveit::hybrid_planning::GlobalPlannerInterface)
