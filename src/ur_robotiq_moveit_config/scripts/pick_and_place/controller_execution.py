"""Helpers for executing curobo-planned trajectories through existing ros2_control actions."""

import time

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .constants import ARM_CONTROLLER_JOINT_NAMES, JOINT_NAMES


def _wait_future_result(node, future, timeout_sec=30.0, poll_period_sec=0.01):
    del node
    deadline = time.monotonic() + float(timeout_sec)
    while time.monotonic() < deadline:
        if future.done():
            try:
                return future.result()
            except Exception:
                return None
        time.sleep(float(poll_period_sec))
    if not future.done():
        return None
    try:
        return future.result()
    except Exception:
        return None


def build_arm_joint_trajectory(waypoints, dt, controller_joint_names=None):
    controller_joint_names = list(controller_joint_names or ARM_CONTROLLER_JOINT_NAMES)
    trajectory = JointTrajectory()
    trajectory.joint_names = controller_joint_names
    final_index = len(waypoints) - 1

    for index, waypoint in enumerate(waypoints):
        point = JointTrajectoryPoint()
        current_names = list(waypoint.name) if waypoint.name else list(JOINT_NAMES)
        name_to_index = {joint_name: idx for idx, joint_name in enumerate(current_names)}
        if not all(joint_name in name_to_index for joint_name in controller_joint_names):
            missing = [
                joint_name for joint_name in controller_joint_names if joint_name not in name_to_index
            ]
            raise RuntimeError(
                "curobo trajectory waypoint is missing expected joints: "
                + ", ".join(missing)
            )

        point.positions = [
            float(waypoint.position[name_to_index[joint_name]])
            for joint_name in controller_joint_names
        ]
        velocity_values = getattr(waypoint, "velocity", None)
        if index == final_index:
            point.velocities = [0.0] * len(controller_joint_names)
            point.accelerations = [0.0] * len(controller_joint_names)
        elif velocity_values and len(velocity_values) >= len(current_names):
            point.velocities = [
                float(velocity_values[name_to_index[joint_name]])
                for joint_name in controller_joint_names
            ]
            point.accelerations = [0.0] * len(controller_joint_names)
        else:
            point.velocities = [0.0] * len(controller_joint_names)
            point.accelerations = [0.0] * len(controller_joint_names)
        time_from_start = max(float(dt), 1e-3) * float(index + 1)
        point.time_from_start.sec = int(time_from_start)
        point.time_from_start.nanosec = int((time_from_start % 1.0) * 1e9)
        trajectory.points.append(point)

    return trajectory


def execute_arm_joint_trajectory(
    node,
    action_client,
    waypoints,
    dt,
    timeout_sec=120.0,
    controller_joint_names=None,
):
    if not action_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("Arm FollowJointTrajectory action server not available.")
        return False

    try:
        trajectory = build_arm_joint_trajectory(
            waypoints,
            dt,
            controller_joint_names=controller_joint_names,
        )
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
        return False

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory
    future = action_client.send_goal_async(goal)
    goal_handle = _wait_future_result(node, future, timeout_sec=5.0)
    if goal_handle is None or not goal_handle.accepted:
        node.get_logger().error("Arm trajectory goal was rejected.")
        return False

    result = _wait_future_result(node, goal_handle.get_result_async(), timeout_sec=timeout_sec)
    if result is None:
        node.get_logger().error("Arm trajectory action returned no result.")
        return False

    raw_error_code = result.result.error_code
    error_code = raw_error_code.val if hasattr(raw_error_code, "val") else int(raw_error_code)
    if error_code == 0:
        return True

    node.get_logger().error(
        f"Arm trajectory action failed with error code {error_code}: "
        f"{result.result.error_string}"
    )
    return False


def execute_arm_joint_target(
    node,
    action_client,
    joint_targets,
    duration_sec=4.0,
    timeout_sec=120.0,
    controller_joint_names=None,
):
    controller_joint_names = list(controller_joint_names or ARM_CONTROLLER_JOINT_NAMES)
    if not action_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("Arm FollowJointTrajectory action server not available.")
        return False

    try:
        positions = [float(joint_targets[joint_name]) for joint_name in controller_joint_names]
    except KeyError as exc:
        node.get_logger().error(
            f"Direct home trajectory is missing joint target for '{exc.args[0]}'."
        )
        return False

    trajectory = JointTrajectory()
    trajectory.joint_names = controller_joint_names
    point = JointTrajectoryPoint()
    point.positions = positions
    safe_duration = max(float(duration_sec), 1e-3)
    point.time_from_start.sec = int(safe_duration)
    point.time_from_start.nanosec = int((safe_duration % 1.0) * 1e9)
    trajectory.points.append(point)

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory
    future = action_client.send_goal_async(goal)
    goal_handle = _wait_future_result(node, future, timeout_sec=5.0)
    if goal_handle is None or not goal_handle.accepted:
        node.get_logger().error("Direct arm trajectory goal was rejected.")
        return False

    result = _wait_future_result(node, goal_handle.get_result_async(), timeout_sec=timeout_sec)
    if result is None:
        node.get_logger().error("Direct arm trajectory action returned no result.")
        return False

    raw_error_code = result.result.error_code
    error_code = raw_error_code.val if hasattr(raw_error_code, "val") else int(raw_error_code)
    if error_code == 0:
        return True

    node.get_logger().error(
        f"Direct arm trajectory failed with error code {error_code}: "
        f"{result.result.error_string}"
    )
    return False
