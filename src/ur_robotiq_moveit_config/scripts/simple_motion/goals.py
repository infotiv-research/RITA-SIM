"""MoveGroup goal builders for simple-motion benchmark cases."""

from __future__ import annotations

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive

from pick_and_place.constants import END_EFFECTOR_LINK, JOINT_NAMES, PLANNING_GROUP


DEFAULT_ALLOWED_PLANNING_TIME_SEC = 10.0
DEFAULT_NUM_PLANNING_ATTEMPTS = 10
DEFAULT_MAX_VELOCITY_SCALING = 0.3
DEFAULT_MAX_ACCELERATION_SCALING = 0.3


def _configure_pipeline(request: MotionPlanRequest, planning_pipeline: str | None) -> None:
    pipeline = str(planning_pipeline or "").strip().lower()
    pipeline_map = {
        "cumotion": "isaac_ros_cumotion",
        "ompl": "ompl",
    }
    pipeline_id = pipeline_map.get(pipeline)
    if pipeline_id and hasattr(request, "pipeline_id"):
        request.pipeline_id = pipeline_id


def build_joint_goal(
    joint_targets: dict[str, float],
    *,
    current_joint_positions: dict[str, float] | None = None,
    planning_pipeline: str | None = None,
    planner_id: str | None = None,
    joint_tolerance: float = 0.02,
    replan_attempts: int = 1,
) -> MoveGroup.Goal:
    request = MotionPlanRequest()
    request.group_name = PLANNING_GROUP
    request.allowed_planning_time = DEFAULT_ALLOWED_PLANNING_TIME_SEC
    request.num_planning_attempts = DEFAULT_NUM_PLANNING_ATTEMPTS
    request.max_velocity_scaling_factor = DEFAULT_MAX_VELOCITY_SCALING
    request.max_acceleration_scaling_factor = DEFAULT_MAX_ACCELERATION_SCALING
    _configure_pipeline(request, planning_pipeline)
    if planner_id:
        request.planner_id = str(planner_id)

    if current_joint_positions:
        names = [
            joint_name
            for joint_name in JOINT_NAMES
            if joint_name in current_joint_positions
        ]
        request.start_state.joint_state.name = names
        request.start_state.joint_state.position = [
            float(current_joint_positions[joint_name]) for joint_name in names
        ]

    request.workspace_parameters.header.frame_id = "world"
    request.workspace_parameters.min_corner.x = -3.0
    request.workspace_parameters.min_corner.y = -3.0
    request.workspace_parameters.min_corner.z = -0.5
    request.workspace_parameters.max_corner.x = 3.0
    request.workspace_parameters.max_corner.y = 3.0
    request.workspace_parameters.max_corner.z = 3.0

    constraints = Constraints()
    for joint_name in JOINT_NAMES:
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = joint_name
        joint_constraint.position = float(joint_targets[joint_name])
        joint_constraint.tolerance_above = float(joint_tolerance)
        joint_constraint.tolerance_below = float(joint_tolerance)
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)
    request.goal_constraints.append(constraints)

    goal = MoveGroup.Goal()
    goal.request = request
    goal.planning_options = PlanningOptions()
    goal.planning_options.plan_only = False
    goal.planning_options.look_around = False
    goal.planning_options.replan = int(replan_attempts) > 1
    goal.planning_options.replan_attempts = max(int(replan_attempts), 1)
    return goal


def build_pose_goal(
    target_pose,
    *,
    current_joint_positions: dict[str, float] | None = None,
    planning_pipeline: str | None = None,
    planner_id: str | None = None,
    position_tolerance_m: float = 0.02,
    orientation_tolerance_rad: float = 0.35,
    replan_attempts: int = 1,
) -> MoveGroup.Goal:
    request = MotionPlanRequest()
    request.group_name = PLANNING_GROUP
    request.allowed_planning_time = DEFAULT_ALLOWED_PLANNING_TIME_SEC
    request.num_planning_attempts = DEFAULT_NUM_PLANNING_ATTEMPTS
    request.max_velocity_scaling_factor = DEFAULT_MAX_VELOCITY_SCALING
    request.max_acceleration_scaling_factor = DEFAULT_MAX_ACCELERATION_SCALING
    _configure_pipeline(request, planning_pipeline)
    if planner_id:
        request.planner_id = str(planner_id)

    if current_joint_positions:
        names = [
            joint_name
            for joint_name in JOINT_NAMES
            if joint_name in current_joint_positions
        ]
        request.start_state.joint_state.name = names
        request.start_state.joint_state.position = [
            float(current_joint_positions[joint_name]) for joint_name in names
        ]

    request.workspace_parameters.header.frame_id = "world"
    request.workspace_parameters.min_corner.x = -3.0
    request.workspace_parameters.min_corner.y = -3.0
    request.workspace_parameters.min_corner.z = -0.5
    request.workspace_parameters.max_corner.x = 3.0
    request.workspace_parameters.max_corner.y = 3.0
    request.workspace_parameters.max_corner.z = 3.0

    constraints = Constraints()

    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = "world"
    position_constraint.link_name = END_EFFECTOR_LINK
    position_constraint.target_point_offset.x = 0.0
    position_constraint.target_point_offset.y = 0.0
    position_constraint.target_point_offset.z = 0.0

    bounding_sphere = SolidPrimitive()
    bounding_sphere.type = SolidPrimitive.SPHERE
    bounding_sphere.dimensions = [float(position_tolerance_m)]
    position_constraint.constraint_region.primitives.append(bounding_sphere)
    position_constraint.constraint_region.primitive_poses.append(target_pose)
    position_constraint.weight = 1.0
    constraints.position_constraints.append(position_constraint)

    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = "world"
    orientation_constraint.link_name = END_EFFECTOR_LINK
    orientation_constraint.orientation = target_pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = float(orientation_tolerance_rad)
    orientation_constraint.absolute_y_axis_tolerance = float(orientation_tolerance_rad)
    orientation_constraint.absolute_z_axis_tolerance = float(orientation_tolerance_rad)
    orientation_constraint.weight = 1.0
    constraints.orientation_constraints.append(orientation_constraint)

    request.goal_constraints.append(constraints)

    goal = MoveGroup.Goal()
    goal.request = request
    goal.planning_options = PlanningOptions()
    goal.planning_options.plan_only = False
    goal.planning_options.look_around = False
    goal.planning_options.replan = int(replan_attempts) > 1
    goal.planning_options.replan_attempts = max(int(replan_attempts), 1)
    return goal
