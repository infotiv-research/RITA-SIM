"""Collect benchmark events and summarize them into JSON-friendly results."""

from __future__ import annotations

from datetime import datetime

from action_msgs.msg import GoalStatus
from moveit_msgs.msg import MoveItErrorCodes

from .constants import JOINT_NAMES, duration_to_seconds, vector_norm


# Reorder a JointState message into the benchmark's configured joint order.
def extract_joint_positions(msg):
    if not msg.name or len(msg.name) != len(msg.position):
        return None
    name_to_position = {name: float(position) for name, position in zip(msg.name, msg.position)}
    if any(joint_name not in name_to_position for joint_name in JOINT_NAMES):
        return None
    return [name_to_position[joint_name] for joint_name in JOINT_NAMES]


# Extract the final joint target from a planned MoveIt trajectory.
def extract_final_joint_target(robot_trajectory):
    joint_trajectory = getattr(robot_trajectory, "joint_trajectory", None)
    if joint_trajectory is None or not joint_trajectory.points:
        return None
    final_point = joint_trajectory.points[-1]
    return {
        "joint_names": list(joint_trajectory.joint_names),
        "positions": [float(value) for value in final_point.positions],
        "time_from_start_sec": duration_to_seconds(final_point.time_from_start),
    }


# Create the mutable record that accumulates one benchmark run.
def create_run_record(run_index: int, goal_spec: dict):
    return {
        "run_index": int(run_index),
        "goal_spec": dict(goal_spec),
        "started_at_wall": datetime.now().astimezone().isoformat(timespec="seconds"),
        "start_monotonic": 0.0,
        "global_plan_events": [],
        "initial_global_plan_reference": None,
        "hybrid_mpc_summary": {
            "path_invalidated_count": 0,
            "first_invalidation_t_rel_sec": None,
        },
        "joint_state_summary": {"last_positions": None},
        "obstacle": {
            "planned_obstacle": None,
            "spawned": False,
            "spawn_t_rel_sec": None,
            "spawn_tcp_to_obstacle_surface_distance_m": None,
            "min_tcp_clearance_m": None,
            "min_robot_clearance_m": None,
            "distance_sampling_active": True,
            "service_message": None,
            "wait_reason": None,
        },
    }


# Append a global plan event and capture the first plan as the obstacle reference.
def append_global_solution(run_data: dict, msg, t_rel_sec: float):
    trajectory = getattr(msg, "trajectory", None)
    joint_trajectory = getattr(trajectory, "joint_trajectory", None)
    point_count = len(joint_trajectory.points) if joint_trajectory and joint_trajectory.points else 0
    duration_sec = (
        duration_to_seconds(joint_trajectory.points[-1].time_from_start) if point_count > 0 else None
    )

    reference_plan = None
    if point_count > 0 and run_data.get("initial_global_plan_reference") is None:
        name_to_index = {name: index for index, name in enumerate(joint_trajectory.joint_names)}
        point_records = []
        for point_index, point in enumerate(joint_trajectory.points):
            ordered_positions = []
            for joint_name in JOINT_NAMES:
                source_index = name_to_index.get(joint_name)
                if source_index is None or source_index >= len(point.positions):
                    ordered_positions = []
                    break
                ordered_positions.append(float(point.positions[source_index]))
            if not ordered_positions:
                continue
            point_records.append(
                {
                    "point_index": int(point_index),
                    "joint_positions": ordered_positions,
                    "time_from_start_sec": duration_to_seconds(point.time_from_start),
                }
            )
        if point_records:
            reference_plan = {
                "received_t_rel_sec": t_rel_sec,
                "point_count": int(point_count),
                "planned_duration_sec": duration_sec,
                "points": point_records,
            }

    run_data["global_plan_events"].append(
        {
            "t_rel_sec": t_rel_sec,
            "planning_time_sec": float(msg.planning_time),
            "planned_duration_sec": duration_sec,
        }
    )
    return reference_plan


# Store the latest ordered joint positions for final error and clearance checks.
def append_joint_state(run_data: dict, positions: list[float]):
    run_data["joint_state_summary"]["last_positions"] = list(positions)


# Accumulate hybrid MPC invalidation telemetry for the current run.
def accumulate_hybrid_mpc_telemetry(run_data: dict, payload: dict, t_rel_sec: float):
    summary = run_data["hybrid_mpc_summary"]
    if payload.get("path_invalidated"):
        summary["path_invalidated_count"] += 1
        if summary["first_invalidation_t_rel_sec"] is None:
            summary["first_invalidation_t_rel_sec"] = t_rel_sec


# Decide whether a MoveGroup result represents a successful completed motion.
def is_successful_move_group_result(result_data: dict):
    return (
        bool(result_data.get("accepted"))
        and not bool(result_data.get("timed_out"))
        and int(result_data.get("status") or 0) == GoalStatus.STATUS_SUCCEEDED
        and int(result_data.get("error_code") or 0) == MoveItErrorCodes.SUCCESS
    )


# Convert obstacle tracking state into the compact output schema.
def _summarize_obstacle(obstacle_state: dict):
    planned = obstacle_state.get("planned_obstacle") or {}
    tcp_available = obstacle_state.get("min_tcp_clearance_m") is not None
    robot_available = obstacle_state.get("min_robot_clearance_m") is not None
    if tcp_available and robot_available:
        distance_sampling_status = "ok"
    elif tcp_available:
        distance_sampling_status = "robot_collision_spheres_unavailable"
    elif robot_available:
        distance_sampling_status = "tcp_unavailable"
    elif planned:
        distance_sampling_status = "tcp_and_robot_unavailable"
    else:
        distance_sampling_status = "obstacle_not_planned"

    obstacle = {
        "spawned": bool(obstacle_state.get("spawned")),
        "spawn_time_after_goal_sent_sec": obstacle_state.get("spawn_t_rel_sec"),
        "position": list(planned.get("position") or []),
        "rotation_deg": list(planned.get("rotation_deg") or []),
        "size": list(planned.get("size") or []),
        "tcp_clearance_at_spawn_m": obstacle_state.get("spawn_tcp_to_obstacle_surface_distance_m"),
        "min_tcp_clearance_m": obstacle_state.get("min_tcp_clearance_m"),
        "min_robot_clearance_m": obstacle_state.get("min_robot_clearance_m"),
        "distance_sampling_status": distance_sampling_status,
    }
    if obstacle_state.get("wait_reason"):
        obstacle["wait_reason"] = obstacle_state.get("wait_reason")
    if obstacle_state.get("service_message") and not obstacle_state.get("spawned"):
        obstacle["message"] = obstacle_state.get("service_message")
    return obstacle


# Build the final JSON-friendly summary for one benchmark run.
def summarize_run(run_data: dict, move_group_result: dict | None):
    move_group_result = move_group_result or {}
    precondition = run_data.get("precondition_start_state") or {}
    goal_spec = run_data.get("goal_spec") or {}
    obstacle_state = run_data.get("obstacle") or {}
    hybrid_mpc = run_data.get("hybrid_mpc_summary") or {}
    joint_states = run_data.get("joint_state_summary") or {}
    global_plan_events = run_data.get("global_plan_events") or []

    obstacle_spawn_t = obstacle_state.get("spawn_t_rel_sec")
    first_invalidation_t = hybrid_mpc.get("first_invalidation_t_rel_sec")
    next_global_after_obstacle = (
        next(
            (event["t_rel_sec"] for event in global_plan_events if event["t_rel_sec"] >= obstacle_spawn_t),
            None,
        )
        if obstacle_spawn_t is not None
        else None
    )

    joint_error_norm = None
    final_target = move_group_result.get("final_joint_target")
    final_positions = joint_states.get("last_positions")
    if final_target and final_positions:
        target_positions = list(final_target.get("positions") or [])
        if len(target_positions) == len(final_positions):
            joint_error_norm = vector_norm(
                current - target for current, target in zip(final_positions, target_positions)
            )

    precondition_success = is_successful_move_group_result(precondition) if precondition else None
    benchmark_success = is_successful_move_group_result(move_group_result) if move_group_result else False
    if precondition_success is False:
        failure_stage = "precondition_start_state"
        success = False
    elif not move_group_result:
        failure_stage = None
        success = False
    elif not benchmark_success:
        failure_stage = "benchmark_goal"
        success = False
    else:
        failure_stage = None
        success = True

    global_plans = [
        {
            "time_sec": event.get("t_rel_sec"),
            "planning_time_sec": event.get("planning_time_sec"),
            "planned_duration_sec": event.get("planned_duration_sec"),
        }
        for event in global_plan_events
    ]

    return {
        "run_index": int(run_data.get("run_index", 0)),
        "started_at_wall": run_data.get("started_at_wall"),
        "case": goal_spec.get("case_name"),
        "start_state": goal_spec.get("start_state_name"),
        "goal_state": goal_spec.get("goal_state_name"),
        "success": bool(success),
        "failure_stage": failure_stage,
        "timed_out": bool(move_group_result.get("timed_out")),
        "benchmark_goal_wall_time_sec": move_group_result.get("elapsed_sec"),
        "obstacle": _summarize_obstacle(obstacle_state),
        "plans": global_plans,
        "replanning": {
            "global_plan_count": len(global_plan_events),
            "replan_count": max(len(global_plan_events) - 1, 0),
            "path_invalidated_count": hybrid_mpc.get("path_invalidated_count", 0),
            "time_from_obstacle_spawn_to_path_invalidation_sec": (
                first_invalidation_t - obstacle_spawn_t
                if obstacle_spawn_t is not None and first_invalidation_t is not None
                else None
            ),
            "time_from_obstacle_spawn_to_next_global_plan_sec": (
                next_global_after_obstacle - obstacle_spawn_t
                if obstacle_spawn_t is not None and next_global_after_obstacle is not None
                else None
            ),
        },
        "final_joint_error_norm": joint_error_norm,
    }
