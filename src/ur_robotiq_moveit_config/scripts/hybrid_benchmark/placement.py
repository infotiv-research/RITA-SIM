"""Choose trajectory-aware obstacle positions and measure TCP clearance."""

from __future__ import annotations

import math
from copy import deepcopy

from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionFK

from .constants import (
    BENCHMARK_OBSTACLE,
    DEFAULT_TRAJECTORY_TARGET_DURATION_FRACTION,
    DEFAULT_TRAJECTORY_TARGET_LEAD_TIME_SEC,
    DEFAULT_TRAJECTORY_WALL_FORWARD_OFFSET_M,
    END_EFFECTOR_LINK,
    JOINT_NAMES,
    WORLD_FRAME,
    vector_norm,
)


# Mark failures that are specific to obstacle placement and FK lookup.
class PlacementError(RuntimeError):
    pass


# Extract XYZ coordinates from a ROS pose message.
def _pose_to_position(pose):
    return [float(pose.position.x), float(pose.position.y), float(pose.position.z)]


# Measure Euclidean distance between two equal-length vectors.
def _distance(a, b):
    return vector_norm(float(ax) - float(bx) for ax, bx in zip(a, b))


# Measure distance from a point to the surface of a yaw-rotated box.
def _point_to_yaw_box_surface_distance(point_xyz, center_xyz, size_xyz, yaw_deg):
    yaw = math.radians(float(yaw_deg))
    dx = float(point_xyz[0]) - float(center_xyz[0])
    dy = float(point_xyz[1]) - float(center_xyz[1])
    dz = float(point_xyz[2]) - float(center_xyz[2])
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    local_x = cy * dx + sy * dy
    local_y = -sy * dx + cy * dy
    half = [max(float(s) * 0.5, 0.0) for s in size_xyz]
    outside_x = max(abs(local_x) - half[0], 0.0)
    outside_y = max(abs(local_y) - half[1], 0.0)
    outside_z = max(abs(dz) - half[2], 0.0)
    return math.sqrt(outside_x * outside_x + outside_y * outside_y + outside_z * outside_z)


# Measure point clearance to an obstacle dictionary from this benchmark.
def point_clearance_to_obstacle(point_xyz, obstacle: dict):
    return _point_to_yaw_box_surface_distance(
        point_xyz,
        [float(value) for value in obstacle["position"]],
        [float(value) for value in obstacle["size"]],
        float(obstacle["rotation_deg"][2]),
    )


# Measure approximate sphere clearance to an obstacle dictionary from this benchmark.
def sphere_clearance_to_obstacle(sphere_center_xyz, sphere_radius, obstacle: dict):
    return point_clearance_to_obstacle(sphere_center_xyz, obstacle) - max(float(sphere_radius), 0.0)


class ObstaclePlacementResolver:
    # Keep the FK service client and async wait helper used for placement queries.
    def __init__(self, fk_client, wait_for_future):
        self._fk_client = fk_client
        self._wait_for_future = wait_for_future

    # Call MoveIt's FK service to convert joint positions into an end-effector pose.
    def _call_fk(self, joint_positions, timeout_sec: float):
        if not self._fk_client.wait_for_service(timeout_sec=min(float(timeout_sec), 0.75)):
            raise PlacementError("compute_fk service unavailable")
        request = GetPositionFK.Request()
        request.header.frame_id = WORLD_FRAME
        request.fk_link_names = [END_EFFECTOR_LINK]
        request.robot_state = RobotState()
        request.robot_state.joint_state.name = list(JOINT_NAMES)
        request.robot_state.joint_state.position = [float(value) for value in joint_positions]
        response = self._wait_for_future(self._fk_client.call_async(request), timeout_sec=float(timeout_sec))
        if response is None or isinstance(response, Exception):
            raise PlacementError(f"compute_fk call failed: {response}")
        error_code = getattr(getattr(response, "error_code", None), "val", None)
        if int(error_code or 0) != MoveItErrorCodes.SUCCESS:
            raise PlacementError(f"compute_fk returned error code {error_code}")
        poses = list(getattr(response, "pose_stamped", []) or [])
        if not poses:
            raise PlacementError("compute_fk returned no poses")
        return poses[0].pose

    # Measure current TCP clearance to the planned obstacle surface.
    def measure_tcp_clearance_to_obstacle(self, joint_positions, obstacle: dict, timeout_sec: float):
        if not joint_positions:
            return None
        try:
            pose = self._call_fk(joint_positions, timeout_sec=float(timeout_sec))
        except PlacementError:
            return None
        return point_clearance_to_obstacle(_pose_to_position(pose), obstacle)

    # Pick an obstacle pose from the initial global trajectory and report selection metadata.
    def resolve_obstacle_placement(self, reference_plan):
        if reference_plan is None:
            raise PlacementError("no initial global plan available")
        points = list(reference_plan.get("points") or [])
        if len(points) < 3:
            raise PlacementError("initial global plan too short")

        duration_sec = float(reference_plan.get("planned_duration_sec") or 0.0)
        desired_time_from_start_sec = max(
            float(DEFAULT_TRAJECTORY_TARGET_LEAD_TIME_SEC),
            duration_sec * float(DEFAULT_TRAJECTORY_TARGET_DURATION_FRACTION),
        )
        anchor_index = min(
            range(1, len(points) - 1),
            key=lambda i: abs(float(points[i].get("time_from_start_sec") or 0.0) - desired_time_from_start_sec),
        )

        prev_position = _pose_to_position(self._call_fk(points[anchor_index - 1]["joint_positions"], 1.0))
        anchor_position = _pose_to_position(self._call_fk(points[anchor_index]["joint_positions"], 1.0))
        next_position = _pose_to_position(self._call_fk(points[anchor_index + 1]["joint_positions"], 1.0))
        start_position = _pose_to_position(self._call_fk(points[0]["joint_positions"], 1.0))
        goal_position = _pose_to_position(self._call_fk(points[-1]["joint_positions"], 1.0))

        tangent_x = next_position[0] - prev_position[0]
        tangent_y = next_position[1] - prev_position[1]
        tangent_norm = math.sqrt(tangent_x * tangent_x + tangent_y * tangent_y)
        if tangent_norm <= 1e-5:
            raise PlacementError("trajectory tangent is degenerate")
        tangent_x /= tangent_norm
        tangent_y /= tangent_norm
        yaw_deg = math.degrees(math.atan2(tangent_y, tangent_x) - math.pi * 0.5)

        forward_offset = float(DEFAULT_TRAJECTORY_WALL_FORWARD_OFFSET_M)
        obstacle = deepcopy(BENCHMARK_OBSTACLE)
        obstacle["position"] = [
            anchor_position[0] + tangent_x * forward_offset,
            anchor_position[1] + tangent_y * forward_offset,
            anchor_position[2],
        ]
        obstacle["rotation_deg"] = [0.0, 0.0, yaw_deg]

        anchor_time = points[anchor_index].get("time_from_start_sec")
        goal_time = points[-1].get("time_from_start_sec")
        remaining_time_to_goal_sec = (
            float(goal_time) - float(anchor_time)
            if goal_time is not None and anchor_time is not None
            else None
        )
        goal_tcp_clearance_to_obstacle_m = _point_to_yaw_box_surface_distance(
            goal_position,
            obstacle["position"],
            obstacle["size"],
            yaw_deg,
        )

        trajectory_selection = {
            "desired_time_from_start_sec": desired_time_from_start_sec,
            "selected_point_index": int(anchor_index),
            "selected_time_from_start_sec": anchor_time,
            "selected_distance_from_start_m": _distance(anchor_position, start_position),
            "selected_distance_to_goal_m": _distance(anchor_position, goal_position),
            "remaining_time_to_goal_sec": remaining_time_to_goal_sec,
            "goal_tcp_clearance_to_obstacle_m": goal_tcp_clearance_to_obstacle_m,
            "wall_yaw_deg": float(yaw_deg),
            "wall_forward_offset_m": forward_offset,
        }
        return obstacle, {"trajectory_selection": trajectory_selection}
