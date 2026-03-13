#!/usr/bin/env python3
"""Helpers for building and sending controller trajectories from cuRobo states."""

import time

import torch
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def wait_future_result(future, timeout_sec=30.0, poll_period_sec=0.01):
    """Wait for an rclpy future without spinning the current executor thread."""
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


def _tensor_to_rows(values):
    if values is None:
        return None

    if isinstance(values, torch.Tensor):
        tensor = values.detach()
        if tensor.is_cuda:
            tensor = tensor.cpu()
        if tensor.dim() == 1:
            tensor = tensor.unsqueeze(0)
        elif tensor.dim() == 3:
            tensor = tensor[0]
        return tensor.tolist()

    if hasattr(values, "dim"):
        dims = values.dim()
        if dims == 1:
            return [values.tolist()]
        if dims == 3:
            return values[0].tolist()
        return values.tolist()

    if isinstance(values, list):
        if not values:
            return []
        if isinstance(values[0], (int, float)):
            return [list(values)]
        if isinstance(values[0], list):
            return [list(row) for row in values]
    return None


def extract_ordered_waypoints(joint_state, ordered_joint_names, max_points=None):
    """Return trajectory lists reordered into controller joint order."""
    if joint_state is None:
        raise RuntimeError("Joint state is required to extract controller waypoints.")

    current_names = list(getattr(joint_state, "joint_names", []) or [])
    if not current_names:
        raise RuntimeError("Joint state does not expose joint names.")

    positions = _tensor_to_rows(getattr(joint_state, "position", None))
    if not positions:
        raise RuntimeError("Joint state does not contain positions.")

    velocities = _tensor_to_rows(getattr(joint_state, "velocity", None))
    accelerations = _tensor_to_rows(getattr(joint_state, "acceleration", None))

    name_to_index = {joint_name: idx for idx, joint_name in enumerate(current_names)}
    missing = [joint_name for joint_name in ordered_joint_names if joint_name not in name_to_index]
    if missing:
        raise RuntimeError(
            "Joint state is missing controller joints: " + ", ".join(missing)
        )

    if max_points is not None:
        positions = positions[:max_points]
        if velocities is not None:
            velocities = velocities[:max_points]
        if accelerations is not None:
            accelerations = accelerations[:max_points]

    ordered_positions = []
    ordered_velocities = []
    ordered_accelerations = []

    for index, row in enumerate(positions):
        ordered_positions.append(
            [float(row[name_to_index[joint_name]]) for joint_name in ordered_joint_names]
        )

        if velocities is not None and index < len(velocities):
            vel_row = velocities[index]
            ordered_velocities.append(
                [float(vel_row[name_to_index[joint_name]]) for joint_name in ordered_joint_names]
            )
        else:
            ordered_velocities.append([0.0] * len(ordered_joint_names))

        if accelerations is not None and index < len(accelerations):
            acc_row = accelerations[index]
            ordered_accelerations.append(
                [float(acc_row[name_to_index[joint_name]]) for joint_name in ordered_joint_names]
            )
        else:
            ordered_accelerations.append([0.0] * len(ordered_joint_names))

    return ordered_positions, ordered_velocities, ordered_accelerations


def build_joint_trajectory(
    joint_names,
    positions,
    velocities,
    accelerations,
    dt,
):
    """Build a JointTrajectory message from already ordered waypoints."""
    trajectory = JointTrajectory()
    trajectory.joint_names = list(joint_names)

    safe_dt = max(float(dt), 1e-3)
    for index, position_row in enumerate(positions):
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in position_row]
        point.velocities = [float(value) for value in velocities[index]]
        point.accelerations = [float(value) for value in accelerations[index]]
        time_from_start = safe_dt * float(index + 1)
        point.time_from_start.sec = int(time_from_start)
        point.time_from_start.nanosec = int((time_from_start % 1.0) * 1e9)
        trajectory.points.append(point)

    return trajectory
