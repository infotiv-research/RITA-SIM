"""Shared constants and small utility helpers for hybrid benchmark runs."""

from __future__ import annotations

import math
import zlib
from pathlib import Path
from typing import Any

from pick_and_place.constants import END_EFFECTOR_LINK, JOINT_NAMES, PLANNING_GROUP

DEFAULT_OUTPUT_DIR = Path("benchmark_results") / "hybrid"
ROBOT_RIGHT_JOINT_POSITIONS = [0.590, 0.016, -0.766, -1.562, 2.343, -1.551, -3.756]
ROBOT_LEFT_JOINT_POSITIONS = [
    -0.645,
    math.radians(2.0),
    math.radians(-65.0),
    math.radians(-73.0),
    math.radians(138.0),
    math.radians(-88.0),
    math.radians(-180.0),
]
BENCHMARK_CASES = {
    "test_1": {
        "description": "Move from robot_right to robot_left.",
        "start_state_name": "robot_right",
        "goal_state_name": "robot_left",
        "start_joint_positions": ROBOT_RIGHT_JOINT_POSITIONS,
        "goal_joint_positions": ROBOT_LEFT_JOINT_POSITIONS,
    },
    "test_2": {
        "description": "Move from robot_left to robot_right.",
        "start_state_name": "robot_left",
        "goal_state_name": "robot_right",
        "start_joint_positions": ROBOT_LEFT_JOINT_POSITIONS,
        "goal_joint_positions": ROBOT_RIGHT_JOINT_POSITIONS,
    },
}
DEFAULT_BENCHMARK_CASE = "test_1"
DEFAULT_INITIAL_PLAN_TIMEOUT_SEC = 8.0
DEFAULT_ACTION_TIMEOUT_SEC = 90.0
DEFAULT_SETTLE_TIME_SEC = 1.0
DEFAULT_PLANNING_TIME_SEC = 8.0
DEFAULT_NUM_ATTEMPTS = 8
SPAWN_TRIGGER_PROFILES_M = {
    "early": 0.60,
    "medium": 0.35,
    "late": 0.20,
    "very_late": 0.10,
}
DEFAULT_SPAWN_TRIGGER_PROFILE = "medium"
DEFAULT_SPAWN_TRIGGER_CLEARANCE_M = SPAWN_TRIGGER_PROFILES_M[DEFAULT_SPAWN_TRIGGER_PROFILE]
DEFAULT_SPAWN_TRIGGER_POLL_PERIOD_SEC = 0.05
DEFAULT_TRAJECTORY_TARGET_LEAD_TIME_SEC = 0.50
DEFAULT_TRAJECTORY_WALL_FORWARD_OFFSET_M = 0.08
DEFAULT_TRAJECTORY_TARGET_DURATION_FRACTION = 0.50
DEFAULT_POST_RUN_IDLE_SEC = 1.0
MOVEIT_FK_SERVICE_NAME = "/compute_fk"
MARKER_TOPIC = "/test_obstacle_markers"
WORLD_FRAME = "world"

BENCHMARK_OBSTACLE = {
    "description": "Wall across the expected transfer corridor; should invalidate the local path and replan.",
    "name": "hybrid_benchmark_obstacle",
    "position": [-1.10, 1.10, 0.90],
    "size": [0.50, 0.02, 0.40],
    "color": [1.0, 0.0, 0.0, 0.85],
    "rotation_deg": [0.0, 0.0, 45.0],
}


# Convert benchmark data into JSON-safe values before writing result files.
def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): json_safe(sub_value) for key, sub_value in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    return str(value)


# Convert obstacle Euler rotation degrees into a ROS quaternion tuple.
def quaternion_from_euler_deg(rotation_deg):
    roll = math.radians(float(rotation_deg[0]))
    pitch = math.radians(float(rotation_deg[1]))
    yaw = math.radians(float(rotation_deg[2]))

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


# Convert ROS duration messages into plain seconds for summaries.
def duration_to_seconds(duration_msg):
    if duration_msg is None:
        return None
    return float(duration_msg.sec) + float(duration_msg.nanosec) * 1e-9


# Convert a vector into its Euclidean magnitude.
def vector_norm(values):
    return math.sqrt(sum(float(value) * float(value) for value in values))


# Derive a repeatable positive RViz marker ID from an obstacle name.
def stable_marker_id(name: str) -> int:
    return zlib.crc32(name.encode("utf-8")) & 0x7FFFFFFF
