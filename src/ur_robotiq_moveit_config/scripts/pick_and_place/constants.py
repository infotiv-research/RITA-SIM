"""
Shared constants for target-object pick-and-place modules.

Responsibilities:
- Define gripper action and joint naming constants.
- Define hardcoded home joint targets for the manipulator.
- Define MoveIt planning group/end-effector identifiers.
"""

GRIPPER_JOINT = "finger_joint"
GRIPPER_TRAJECTORY_ACTION = (
    "/robotiq_gripper_joint_trajectory_controller/follow_joint_trajectory"
)

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Home target hardcoded from ur_robotiq_macro.srdf.xacro group_state "home".
HOME_JOINT_VALUES = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -1.22173,
    "elbow_joint": -0.837758,
    "wrist_1_joint": -1.06465,
    "wrist_2_joint": 1.5708,
    "wrist_3_joint": 0.0,
}

PLANNING_GROUP = "ur_manipulator"
END_EFFECTOR_LINK = "TCP_point"
