"""Build MoveGroup action goals for the predefined hybrid benchmark cases."""

from __future__ import annotations

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
)

from .constants import (
    BENCHMARK_CASES,
    DEFAULT_NUM_ATTEMPTS,
    DEFAULT_PLANNING_TIME_SEC,
    JOINT_NAMES,
    PLANNING_GROUP,
    WORLD_FRAME,
)


# Create the shared MoveIt planning request fields used by each benchmark goal.
def build_request_template(planning_time_sec: float, num_attempts: int):
    request = MotionPlanRequest()
    request.group_name = PLANNING_GROUP
    request.num_planning_attempts = int(num_attempts)
    request.allowed_planning_time = float(planning_time_sec)
    request.workspace_parameters.header.frame_id = WORLD_FRAME
    request.workspace_parameters.min_corner.x = -3.0
    request.workspace_parameters.min_corner.y = -3.0
    request.workspace_parameters.min_corner.z = -0.5
    request.workspace_parameters.max_corner.x = 3.0
    request.workspace_parameters.max_corner.y = 3.0
    request.workspace_parameters.max_corner.z = 3.0
    return request


# Wrap a planning request in a MoveGroup action goal that executes the plan.
def wrap_move_group_goal(request: MotionPlanRequest):
    goal = MoveGroup.Goal()
    goal.request = request
    goal.planning_options = PlanningOptions()
    goal.planning_options.plan_only = False
    goal.planning_options.look_around = False
    goal.planning_options.replan = False
    goal.planning_options.replan_attempts = 1
    return goal


# Build a joint-space MoveGroup goal from target joint positions.
def build_joint_goal(joint_targets, planning_time_sec: float, num_attempts: int):
    request = build_request_template(planning_time_sec, num_attempts)
    constraints = Constraints()
    tolerance = 0.02
    for index, joint_name in enumerate(JOINT_NAMES):
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = joint_name
        joint_constraint.position = float(joint_targets[index])
        joint_constraint.tolerance_above = tolerance
        joint_constraint.tolerance_below = tolerance
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)
    request.goal_constraints.append(constraints)
    return wrap_move_group_goal(request)


# Resolve a named benchmark case into metadata plus start and goal actions.
def resolve_benchmark_case(case_name: str):
    case = BENCHMARK_CASES[case_name]
    start_joint_positions = [float(value) for value in case["start_joint_positions"]]
    goal_joint_positions = [float(value) for value in case["goal_joint_positions"]]
    case_spec = {
        "mode": "joint",
        "case_name": case_name,
        "description": str(case["description"]),
        "start_state_name": str(case["start_state_name"]),
        "goal_state_name": str(case["goal_state_name"]),
        "joint_names": list(JOINT_NAMES),
        "start_joint_positions": start_joint_positions,
        "goal_joint_positions": goal_joint_positions,
    }
    return (
        case_spec,
        build_joint_goal(start_joint_positions, DEFAULT_PLANNING_TIME_SEC, DEFAULT_NUM_ATTEMPTS),
        build_joint_goal(goal_joint_positions, DEFAULT_PLANNING_TIME_SEC, DEFAULT_NUM_ATTEMPTS),
    )
