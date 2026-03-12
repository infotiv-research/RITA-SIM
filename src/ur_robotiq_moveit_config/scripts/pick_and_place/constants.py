"""
Shared constants for target-object pick-and-place modules.

Responsibilities:
- Define gripper action and joint naming constants.
- Define hardcoded home joint targets for the manipulator.
- Define MoveIt planning group/end-effector identifiers.
"""

GRIPPER_JOINT = "finger_joint"
ARM_TRAJECTORY_ACTION = "/joint_trajectory_controller/follow_joint_trajectory"
GRIPPER_TRAJECTORY_ACTION = (
    "/robotiq_gripper_joint_trajectory_controller/follow_joint_trajectory"
)

JOINT_NAMES = [
    "gantry_joint",
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

ARM_CONTROLLER_JOINT_NAMES = list(JOINT_NAMES)

# Home target hardcoded from ur_robotiq_gantry_macro.srdf.xacro group_state "home".
HOME_JOINT_VALUES = {
    "gantry_joint": 0.0,
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -0.872665,  # -50 degrees
    "elbow_joint": -1.308997,  # -75 degrees
    "wrist_1_joint": 2.199115,  # 126 degrees
    "wrist_2_joint": -1.5708,
    "wrist_3_joint": -3.14159,
}

PLANNING_GROUP = "ur_manipulator"
END_EFFECTOR_LINK = "TCP_point"
