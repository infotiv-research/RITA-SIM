"""Case definitions for generic simple-motion benchmarks."""

from __future__ import annotations

import math

from pick_and_place.constants import HOME_JOINT_VALUES, JOINT_NAMES


HOME_JOINT_POSITIONS = [
    HOME_JOINT_VALUES[joint_name]
    for joint_name in JOINT_NAMES
]

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
LOWER_CRATE_A_JOINT_POSITIONS = [
    -0.571,
    math.radians(-1.0),
    math.radians(-84.0),
    math.radians(-103.0),
    math.radians(-171.0),
    math.radians(-92.0),
    math.radians(2.0),
]
LOWER_CRATE_B_JOINT_POSITIONS = [
    0.188,
    math.radians(-2.0),
    math.radians(-84.0),
    math.radians(-107.0),
    math.radians(-174.0),
    math.radians(-91.0),
    math.radians(1.0),
]
LOWER_CRATE_TO_UPPER_START_JOINT_POSITIONS = [
    -0.132,
    math.radians(-0.0),
    math.radians(-86.0),
    math.radians(-117.0),
    math.radians(-160.0),
    math.radians(-90.0),
    math.radians(2.0),
]
UPPER_CRATE_TARGET_POSE = {
    "position": {
        "x": -2.09,
        "y": 0.384,
        "z": 1.096,
    },
    "orientation": {
        "x": 0.000061980668,
        "y": 0.999967674963,
        "z": 0.007242704300,
        "w": 0.003491191910,
    },
}
BETWEEN_LOWER_CRATE_A_JOINT_POSITIONS = [
    0.473,
    math.radians(5.0),
    math.radians(-79.0),
    math.radians(-113.0),
    math.radians(-160.0),
    math.radians(-85.0),
    math.radians(-180.0),
]
BETWEEN_LOWER_CRATE_B_JOINT_POSITIONS = [
    -0.277,
    math.radians(14.0),
    math.radians(-77.0),
    math.radians(-115.0),
    math.radians(-162.0),
    math.radians(-76.0),
    math.radians(-180.0),
]

SIMPLE_MOTION_CASES = {
    "test_1": {
        "description": "Move from robot_right to robot_left.",
        "start_state_name": "robot_right",
        "goal_state_name": "robot_left",
        "start_joint_positions": ROBOT_RIGHT_JOINT_POSITIONS,
        "goal_joint_positions": ROBOT_LEFT_JOINT_POSITIONS,
    },
    "home_to_lower_crate": {
        "description": "Move from the home position to a lower crate interior.",
        "start_state_name": "home",
        "goal_state_name": "lower_crate_a",
        "start_joint_positions": HOME_JOINT_POSITIONS,
        "goal_joint_positions": LOWER_CRATE_A_JOINT_POSITIONS,
    },
    "lower_crate_to_lower_crate": {
        "description": "Move from one lower crate interior to another lower crate interior.",
        "start_state_name": "lower_crate_a",
        "goal_state_name": "lower_crate_b",
        "start_joint_positions": LOWER_CRATE_A_JOINT_POSITIONS,
        "goal_joint_positions": LOWER_CRATE_B_JOINT_POSITIONS,
    },
    "lower_crate_to_upper_crate": {
        "description": "Move from a lower crate interior to an upper crate interior.",
        "start_state_name": "lower_crate",
        "goal_state_name": "upper_crate",
        "start_joint_positions": LOWER_CRATE_TO_UPPER_START_JOINT_POSITIONS,
        "goal_pose": UPPER_CRATE_TARGET_POSE,
        "goal_position_tolerance_m": 0.02,
        "goal_orientation_tolerance_rad": 0.35,
    },
    "between_lower_crate_to_between_lower_crate": {
        "description": "Move with the gripper gap between two lower crates to the same gap between two other lower crates.",
        "start_state_name": "between_lower_crate_a",
        "goal_state_name": "between_lower_crate_b",
        "start_joint_positions": BETWEEN_LOWER_CRATE_A_JOINT_POSITIONS,
        "goal_joint_positions": BETWEEN_LOWER_CRATE_B_JOINT_POSITIONS,
    },
}

DEFAULT_SIMPLE_MOTION_CASE = "test_1"


def resolve_simple_motion_case(case_name: str) -> dict:
    case = SIMPLE_MOTION_CASES[case_name]
    resolved = {
        "case_name": case_name,
        "description": str(case["description"]),
        "start_state_name": str(case["start_state_name"]),
        "goal_state_name": str(case["goal_state_name"]),
        "joint_names": list(JOINT_NAMES),
        "start_joint_positions": [float(value) for value in case["start_joint_positions"]],
    }
    if "goal_joint_positions" in case:
        resolved["goal_joint_positions"] = [
            float(value) for value in case["goal_joint_positions"]
        ]
    if "goal_pose_from_joint_positions" in case:
        resolved["goal_pose_from_joint_positions"] = [
            float(value) for value in case["goal_pose_from_joint_positions"]
        ]
    if "goal_pose" in case:
        goal_pose = case["goal_pose"]
        resolved["goal_pose"] = {
            "position": {
                axis: float(goal_pose["position"][axis])
                for axis in ("x", "y", "z")
            },
            "orientation": {
                axis: float(goal_pose["orientation"][axis])
                for axis in ("x", "y", "z", "w")
            },
        }
    if "goal_position_tolerance_m" in case:
        resolved["goal_position_tolerance_m"] = float(case["goal_position_tolerance_m"])
    if "goal_orientation_tolerance_rad" in case:
        resolved["goal_orientation_tolerance_rad"] = float(case["goal_orientation_tolerance_rad"])
    return resolved


def joint_position_map(joint_positions) -> dict[str, float]:
    return {
        joint_name: float(joint_positions[index])
        for index, joint_name in enumerate(JOINT_NAMES)
    }
