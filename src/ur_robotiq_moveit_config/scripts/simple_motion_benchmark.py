#!/usr/bin/env python3
"""Run one generic simple-motion case and emit recorder events."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetMotionPlan, GetPositionFK
from moveit_msgs.action import HybridPlanner, MoveGroup
from moveit_msgs.msg import MotionSequenceItem, MotionSequenceRequest
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from pick_and_place.backend_curobo import CuroboMotionBackend
from pick_and_place.constants import ARM_TRAJECTORY_ACTION, END_EFFECTOR_LINK, JOINT_NAMES
from simple_motion.cases import (
    DEFAULT_SIMPLE_MOTION_CASE,
    SIMPLE_MOTION_CASES,
    joint_position_map,
    resolve_simple_motion_case,
)
from simple_motion.goals import build_joint_goal, build_pose_goal


METRICS_EVENT_TOPIC = "/simple_motion/metrics_events"
PLANNER_CHOICES = ("curobo", "cumotion", "ompl", "hybrid")
METRICS_PUBLISH_SETTLE_SEC = 0.2


@dataclass(frozen=True)
class MotionResult:
    success: bool
    planning_time_s: float | None = None
    execution_time_s: float | None = None
    total_time_s: float | None = None
    failure_reason: str = ""
    failure_detail: str = ""
    moveit_error_code: int | None = None
    moveit_error_name: str = ""
    action_status_code: int | None = None
    action_status_name: str = ""
    backend_name: str = ""
    backend_failure_reason: str = ""
    backend_failure_detail: str = ""
    backend_error_code: int | None = None
    backend_status_name: str = ""


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run a simple one-move benchmark case.")
    parser.add_argument("--case", default=DEFAULT_SIMPLE_MOTION_CASE, choices=sorted(SIMPLE_MOTION_CASES))
    parser.add_argument("--planner", default="curobo", choices=PLANNER_CHOICES)
    parser.add_argument(
        "--ompl-planner-id",
        choices=("RRTstarkConfigDefault", "RRTConnectkConfigDefault"),
        help="Optional OMPL planner config for MoveGroup requests.",
    )
    parser.add_argument("--joint-tolerance", type=float, default=0.02)
    parser.add_argument("--setup-settle-time", type=float, default=1.0)
    parser.add_argument(
        "--hybrid-retry-timeout",
        type=float,
        default=60.0,
        help="Seconds to keep retrying transient hybrid planning failures before failing.",
    )
    parser.add_argument(
        "--hybrid-retry-delay",
        type=float,
        default=2.0,
        help="Seconds to wait between hybrid planning retries.",
    )
    parser.add_argument(
        "--hybrid-retry-observe-timeout",
        type=float,
        default=30.0,
        help="Seconds to watch joint motion after a hybrid abort before retrying.",
    )
    parser.add_argument(
        "--hybrid-retry-initial-grace",
        type=float,
        default=2.0,
        help="Seconds to allow a delayed hybrid replan/execution to start after an abort.",
    )
    parser.add_argument(
        "--hybrid-retry-stationary-time",
        type=float,
        default=1.5,
        help="Seconds of no joint motion before considering the robot blocked and retrying.",
    )
    parser.add_argument(
        "--hybrid-retry-motion-epsilon",
        type=float,
        default=0.001,
        help="Largest joint delta treated as no motion while observing hybrid retries.",
    )
    return parser


class SimpleMotionBenchmark(Node):
    def __init__(self, args) -> None:
        super().__init__("simple_motion_benchmark")
        self.args = args
        self.planner = str(args.planner).strip().lower()
        self.ompl_planner_id = (
            str(args.ompl_planner_id).strip() if args.ompl_planner_id else None
        )
        self.joint_tolerance = float(args.joint_tolerance)
        self.curobo_planner_type = "joint_space"
        self._last_joint_positions = {}
        self._last_joint_positions_msg_time = 0.0
        self._moveit_error_name_map = None

        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self.hybrid_planner_client = ActionClient(self, HybridPlanner, "/run_hybrid_planning")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.motion_plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.arm_traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            ARM_TRAJECTORY_ACTION,
        )
        self.metrics_event_pub = self.create_publisher(String, METRICS_EVENT_TOPIC, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 20)

        self._curobo_backend = (
            CuroboMotionBackend(self) if self.planner == "curobo" else None
        )

    def _on_joint_states(self, msg: JointState) -> None:
        if not msg.name or len(msg.name) != len(msg.position):
            return
        self._last_joint_positions = {
            name: float(position)
            for name, position in zip(msg.name, msg.position)
        }
        self._last_joint_positions_msg_time = time.monotonic()

    def _wait_future_result(self, future, timeout_sec=30.0, poll_period_sec=0.01):
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            if future.done():
                try:
                    return future.result()
                except Exception:
                    return None
            time.sleep(float(poll_period_sec))
        return None

    def _wait_for_recent_joint_positions(self, timeout_sec=1.0, max_age_sec=0.5):
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            latest_time = float(self._last_joint_positions_msg_time)
            age = time.monotonic() - latest_time
            if self._last_joint_positions and age <= float(max_age_sec):
                return dict(self._last_joint_positions)
            time.sleep(0.05)
        if self._last_joint_positions:
            return dict(self._last_joint_positions)
        return None

    def _current_joints_match(self, joint_targets: dict[str, float]) -> bool:
        current = self._wait_for_recent_joint_positions(timeout_sec=1.0)
        if not current:
            return False
        for joint_name in JOINT_NAMES:
            if joint_name not in current:
                return False
            if abs(float(current[joint_name]) - float(joint_targets[joint_name])) > self.joint_tolerance:
                return False
        return True

    def _max_joint_delta(self, lhs: dict[str, float], rhs: dict[str, float]) -> float | None:
        deltas = []
        for joint_name in JOINT_NAMES:
            if joint_name not in lhs or joint_name not in rhs:
                return None
            deltas.append(abs(float(lhs[joint_name]) - float(rhs[joint_name])))
        return max(deltas) if deltas else None

    def _wait_for_hybrid_retry_observation(
        self,
        joint_targets: dict[str, float],
        label: str,
        started_at: float,
        retry_timeout_sec: float,
    ) -> bool:
        remaining_retry_time = max(float(retry_timeout_sec) - (time.monotonic() - float(started_at)), 0.0)
        observe_timeout_sec = min(
            max(float(self.args.hybrid_retry_observe_timeout), 0.0),
            remaining_retry_time,
        )
        if observe_timeout_sec <= 0.0:
            return False

        initial_grace_sec = max(float(self.args.hybrid_retry_initial_grace), 0.0)
        stationary_time_sec = max(float(self.args.hybrid_retry_stationary_time), 0.0)
        motion_epsilon = max(float(self.args.hybrid_retry_motion_epsilon), 0.0)
        observed_at = time.monotonic()
        deadline = observed_at + observe_timeout_sec
        last_positions = None
        last_motion_time = observed_at

        self.get_logger().info(
            f"Observing hybrid goal [{label}] after aborted action before retrying."
        )

        while time.monotonic() < deadline:
            if self._current_joints_match(joint_targets):
                self.get_logger().warn(
                    f"Hybrid goal [{label}] reached the target after an aborted action. Treating it as success."
                )
                return True

            current = self._wait_for_recent_joint_positions(timeout_sec=0.2, max_age_sec=0.5)
            now = time.monotonic()
            if current:
                if last_positions is not None:
                    max_delta = self._max_joint_delta(current, last_positions)
                    if max_delta is not None and max_delta > motion_epsilon:
                        last_motion_time = now

                last_positions = current

            if (
                now - observed_at >= initial_grace_sec
                and now - last_motion_time >= stationary_time_sec
            ):
                return False

            time.sleep(0.1)

        return self._current_joints_match(joint_targets)

    def wait_until_ready(self) -> bool:
        if self._curobo_backend is not None:
            return bool(self._curobo_backend.wait_until_ready())

        if self.planner == "hybrid":
            self.get_logger().info("Waiting for HybridPlanner action server (/run_hybrid_planning)...")
            if not self.hybrid_planner_client.wait_for_server(timeout_sec=30.0):
                self.get_logger().error("HybridPlanner action server not available after 30s.")
                return False

            return True

        self.get_logger().info("Waiting for MoveGroup action server (/move_action)...")
        if not self.move_group_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("MoveGroup action server not available after 30s.")
            return False

        return True

    def _moveit_error_name(self, error_code: int) -> str:
        if self._moveit_error_name_map is None:
            mapping = {}
            for name in dir(MoveItErrorCodes):
                if not name.isupper():
                    continue
                value = getattr(MoveItErrorCodes, name)
                if isinstance(value, int):
                    mapping[int(value)] = name
            self._moveit_error_name_map = mapping
        return self._moveit_error_name_map.get(int(error_code), "UNKNOWN_ERROR_CODE")

    @staticmethod
    def _goal_status_name(status_code: int) -> str:
        mapping = {
            GoalStatus.STATUS_UNKNOWN: "STATUS_UNKNOWN",
            GoalStatus.STATUS_ACCEPTED: "STATUS_ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "STATUS_EXECUTING",
            GoalStatus.STATUS_CANCELING: "STATUS_CANCELING",
            GoalStatus.STATUS_SUCCEEDED: "STATUS_SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "STATUS_CANCELED",
            GoalStatus.STATUS_ABORTED: "STATUS_ABORTED",
        }
        return mapping.get(int(status_code), "STATUS_INVALID")

    @staticmethod
    def _moveit_failure_reason(error_name: str, status_name: str) -> str:
        if error_name == "PLANNING_FAILED":
            return "moveit_planning_failed"
        if error_name == "CONTROL_FAILED":
            return "moveit_control_failed"
        if error_name == "TIMED_OUT":
            return "moveit_timed_out"
        if status_name == "STATUS_ABORTED":
            return "moveit_goal_aborted"
        if status_name == "STATUS_CANCELED":
            return "moveit_goal_canceled"
        return "moveit_goal_failed"

    def _planning_pipeline(self) -> str | None:
        if self.planner == "cumotion":
            return "cumotion"
        if self.planner == "ompl":
            return "ompl"
        return None

    def _moveit_planner_id(self) -> str | None:
        if self.planner == "ompl":
            return self.ompl_planner_id
        return None

    def _send_move_group_joint_goal(
        self,
        joint_targets: dict[str, float],
        label: str,
    ) -> MotionResult:
        current = self._wait_for_recent_joint_positions(timeout_sec=1.0)
        goal = build_joint_goal(
            joint_targets,
            current_joint_positions=current,
            planning_pipeline=self._planning_pipeline(),
            planner_id=self._moveit_planner_id(),
            joint_tolerance=self.joint_tolerance,
            normalize_periodic_targets=self.planner == "ompl",
        )

        self.get_logger().info(f"Sending MoveGroup joint goal [{label}]...")
        movement_start_time = time.monotonic()
        goal_handle = self._wait_future_result(
            self.move_group_client.send_goal_async(goal),
            timeout_sec=30.0,
        )
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"MoveGroup joint goal [{label}] was rejected.")
            return MotionResult(
                False,
                total_time_s=time.monotonic() - movement_start_time,
                failure_reason="move_group_joint_goal_rejected",
                failure_detail=f"MoveGroup joint goal [{label}] was rejected.",
            )

        wrapped = self._wait_future_result(
            goal_handle.get_result_async(),
            timeout_sec=120.0,
        )
        total_time_s = time.monotonic() - movement_start_time
        if wrapped is None:
            self.get_logger().error(f"MoveGroup returned no result for [{label}].")
            return MotionResult(
                False,
                total_time_s=total_time_s,
                failure_reason="move_group_joint_goal_no_result",
                failure_detail=f"MoveGroup returned no result for joint goal [{label}].",
            )

        error_code = int(wrapped.result.error_code.val)
        status = int(getattr(wrapped, "status", GoalStatus.STATUS_UNKNOWN))
        error_name = self._moveit_error_name(error_code)
        status_name = self._goal_status_name(status)
        if error_code == MoveItErrorCodes.SUCCESS and status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"MoveGroup joint goal [{label}] succeeded.")
            return MotionResult(True, total_time_s=total_time_s)

        if self._current_joints_match(joint_targets):
            self.get_logger().warn(
                f"MoveGroup joint goal [{label}] reported {error_name}({error_code}) / "
                f"{status_name}, but the final joints match the target. Treating it as success."
            )
            return MotionResult(True, total_time_s=total_time_s, backend_name=self.planner)

        self.get_logger().error(
            f"MoveGroup joint goal [{label}] failed: "
            f"error={error_name}({error_code}), "
            f"status={status_name}."
        )
        return MotionResult(
            False,
            total_time_s=total_time_s,
            failure_reason=self._moveit_failure_reason(error_name, status_name),
            failure_detail=(
                f"MoveGroup joint goal [{label}] failed: "
                f"error={error_name}({error_code}), status={status_name}."
            ),
            moveit_error_code=error_code,
            moveit_error_name=error_name,
            action_status_code=status,
            action_status_name=status_name,
            backend_name=self.planner,
            backend_failure_reason=self._moveit_failure_reason(error_name, status_name),
            backend_failure_detail=(
                f"MoveGroup joint goal [{label}] failed: "
                f"error={error_name}({error_code}), status={status_name}."
            ),
            backend_error_code=error_code,
            backend_status_name=status_name,
        )

    def _send_move_group_pose_goal(
        self,
        target_pose,
        case_spec: dict,
        label: str,
    ) -> MotionResult:
        current = self._wait_for_recent_joint_positions(timeout_sec=1.0)
        goal = build_pose_goal(
            target_pose,
            current_joint_positions=current,
            planning_pipeline=self._planning_pipeline(),
            planner_id=self._moveit_planner_id(),
            position_tolerance_m=float(case_spec.get("goal_position_tolerance_m", 0.02)),
            orientation_tolerance_rad=float(case_spec.get("goal_orientation_tolerance_rad", 0.35)),
        )

        self.get_logger().info(f"Sending MoveGroup pose goal [{label}]...")
        movement_start_time = time.monotonic()
        goal_handle = self._wait_future_result(
            self.move_group_client.send_goal_async(goal),
            timeout_sec=30.0,
        )
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"MoveGroup pose goal [{label}] was rejected.")
            return MotionResult(
                False,
                total_time_s=time.monotonic() - movement_start_time,
                failure_reason="move_group_pose_goal_rejected",
                failure_detail=f"MoveGroup pose goal [{label}] was rejected.",
            )

        wrapped = self._wait_future_result(
            goal_handle.get_result_async(),
            timeout_sec=120.0,
        )
        total_time_s = time.monotonic() - movement_start_time
        if wrapped is None:
            self.get_logger().error(f"MoveGroup returned no result for [{label}].")
            return MotionResult(
                False,
                total_time_s=total_time_s,
                failure_reason="move_group_pose_goal_no_result",
                failure_detail=f"MoveGroup returned no result for pose goal [{label}].",
            )

        error_code = int(wrapped.result.error_code.val)
        status = int(getattr(wrapped, "status", GoalStatus.STATUS_UNKNOWN))
        error_name = self._moveit_error_name(error_code)
        status_name = self._goal_status_name(status)
        if error_code == MoveItErrorCodes.SUCCESS and status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"MoveGroup pose goal [{label}] succeeded.")
            return MotionResult(True, total_time_s=total_time_s)

        self.get_logger().error(
            f"MoveGroup pose goal [{label}] failed: "
            f"error={error_name}({error_code}), "
            f"status={status_name}."
        )
        return MotionResult(
            False,
            total_time_s=total_time_s,
            failure_reason=self._moveit_failure_reason(error_name, status_name),
            failure_detail=(
                f"MoveGroup pose goal [{label}] failed: "
                f"error={error_name}({error_code}), status={status_name}."
            ),
            moveit_error_code=error_code,
            moveit_error_name=error_name,
            action_status_code=status,
            action_status_name=status_name,
            backend_name=self.planner,
            backend_failure_reason=self._moveit_failure_reason(error_name, status_name),
            backend_failure_detail=(
                f"MoveGroup pose goal [{label}] failed: "
                f"error={error_name}({error_code}), status={status_name}."
            ),
            backend_error_code=error_code,
            backend_status_name=status_name,
        )

    def _curobo_failure_fields(self) -> dict:
        failure = dict(getattr(self._curobo_backend, "last_failure", {}) or {})
        return {
            "backend_name": str(failure.get("backend_name") or "curobo_ros"),
            "backend_failure_reason": str(
                failure.get("backend_failure_reason") or "curobo_failure"
            ),
            "backend_failure_detail": str(failure.get("backend_failure_detail") or ""),
            "backend_error_code": failure.get("backend_error_code"),
            "backend_status_name": str(failure.get("backend_status_name") or ""),
        }

    def _curobo_failure_result(
        self,
        *,
        planning_time_s: float | None,
        execution_time_s: float | None,
        total_time_s: float | None,
        fallback_reason: str,
        fallback_detail: str,
    ) -> MotionResult:
        fields = self._curobo_failure_fields()
        if not fields["backend_failure_detail"]:
            fields["backend_failure_detail"] = fallback_detail
        if fields["backend_failure_reason"] == "curobo_failure":
            fields["backend_failure_reason"] = fallback_reason
        return MotionResult(
            False,
            planning_time_s=planning_time_s,
            execution_time_s=execution_time_s,
            total_time_s=total_time_s,
            failure_reason=fields["backend_failure_reason"],
            failure_detail=fields["backend_failure_detail"],
            **fields,
        )

    def _send_curobo_joint_goal(
        self,
        joint_targets: dict[str, float],
        label: str,
    ) -> MotionResult:
        self.get_logger().info(f"Sending cuRobo joint-space goal [{label}]...")
        planning_start_time = time.monotonic()
        response = self._curobo_backend._plan_joint_target(
            joint_targets,
            planner_type="joint_space",
        )
        planning_time_s = time.monotonic() - planning_start_time
        execution_start_time = time.monotonic()
        success = bool(self._curobo_backend._execute_response(response))
        execution_time_s = time.monotonic() - execution_start_time
        total_time_s = planning_time_s + execution_time_s
        if not success:
            return self._curobo_failure_result(
                planning_time_s=planning_time_s,
                execution_time_s=execution_time_s,
                total_time_s=total_time_s,
                fallback_reason="curobo_joint_goal_failed",
                fallback_detail=f"cuRobo joint-space goal [{label}] failed.",
            )
        return MotionResult(
            success,
            planning_time_s=planning_time_s,
            execution_time_s=execution_time_s,
            total_time_s=total_time_s,
            backend_name="curobo_ros",
        )

    def _send_hybrid_goal_once(self, move_group_goal: MoveGroup.Goal, label: str, joint_targets: dict[str, float] | None = None) -> MotionResult:
        hybrid_goal = HybridPlanner.Goal()
        hybrid_goal.planning_group = move_group_goal.request.group_name
        sequence_item = MotionSequenceItem()
        sequence_item.req = move_group_goal.request
        sequence_item.blend_radius = 0.0
        sequence_request = MotionSequenceRequest()
        sequence_request.items = [sequence_item]
        hybrid_goal.motion_sequence = sequence_request

        self.get_logger().info(f"Sending direct hybrid goal [{label}]...")
        movement_start_time = time.monotonic()
        goal_handle = self._wait_future_result(
            self.hybrid_planner_client.send_goal_async(hybrid_goal),
            timeout_sec=30.0,
        )
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"Hybrid goal [{label}] was rejected.")
            return MotionResult(
                False,
                total_time_s=time.monotonic() - movement_start_time,
                failure_reason="hybrid_goal_rejected",
                failure_detail=f"Hybrid goal [{label}] was rejected.",
            )

        wrapped = self._wait_future_result(
            goal_handle.get_result_async(),
            timeout_sec=120.0,
        )
        total_time_s = time.monotonic() - movement_start_time
        if wrapped is None or getattr(wrapped, 'result', None) is None:
            self.get_logger().error(f"Hybrid planner returned no result for [{label}].")
            return MotionResult(
                False,
                total_time_s=total_time_s,
                failure_reason="hybrid_goal_no_result",
                failure_detail=f"Hybrid planner returned no result for goal [{label}].",
            )

        hybrid_result = wrapped.result
        error_code = int(hybrid_result.error_code.val)
        status = int(getattr(wrapped, "status", GoalStatus.STATUS_UNKNOWN))
        error_name = self._moveit_error_name(error_code)
        status_name = self._goal_status_name(status)
        if error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f"Hybrid goal [{label}] succeeded.")
            return MotionResult(True, total_time_s=total_time_s, backend_name=self.planner)

        if joint_targets is not None and self._current_joints_match(joint_targets):
            self.get_logger().warn(
                f"Hybrid goal [{label}] reported {error_name}({error_code}) / {status_name}, but the final joints match the target. Treating it as success."
            )
            return MotionResult(True, total_time_s=total_time_s, backend_name=self.planner)

        self.get_logger().error(
            f"Hybrid goal [{label}] failed: error={error_name}({error_code}), status={status_name}."
        )
        return MotionResult(
            False,
            total_time_s=total_time_s,
            failure_reason=self._moveit_failure_reason(error_name, status_name),
            failure_detail=(
                f"Hybrid goal [{label}] failed: error={error_name}({error_code}), status={status_name}."
            ),
            moveit_error_code=error_code,
            moveit_error_name=error_name,
            action_status_code=status,
            action_status_name=status_name,
            backend_name=self.planner,
            backend_failure_reason=self._moveit_failure_reason(error_name, status_name),
            backend_failure_detail=(
                f"Hybrid goal [{label}] failed: error={error_name}({error_code}), status={status_name}."
            ),
            backend_error_code=error_code,
            backend_status_name=status_name,
        )

    def _is_retryable_hybrid_failure(self, result: MotionResult) -> bool:
        return (
            self.planner == "hybrid"
            and not result.success
            and result.moveit_error_name == "PLANNING_FAILED"
            and result.action_status_name == "STATUS_ABORTED"
        )

    def _send_hybrid_goal(
        self,
        move_group_goal: MoveGroup.Goal,
        label: str,
        joint_targets: dict[str, float] | None = None,
        retry_goal_builder=None,
    ) -> MotionResult:
        retry_timeout_sec = max(float(self.args.hybrid_retry_timeout), 0.0)
        retry_delay_sec = max(float(self.args.hybrid_retry_delay), 0.0)
        started_at = time.monotonic()
        attempt = 1

        while True:
            goal = move_group_goal if attempt == 1 or retry_goal_builder is None else retry_goal_builder()
            result = self._send_hybrid_goal_once(goal, label, joint_targets=joint_targets)
            result = MotionResult(
                success=result.success,
                planning_time_s=result.planning_time_s,
                execution_time_s=result.execution_time_s,
                total_time_s=time.monotonic() - started_at,
                failure_reason=result.failure_reason,
                failure_detail=result.failure_detail,
                moveit_error_code=result.moveit_error_code,
                moveit_error_name=result.moveit_error_name,
                action_status_code=result.action_status_code,
                action_status_name=result.action_status_name,
                backend_name=result.backend_name,
                backend_failure_reason=result.backend_failure_reason,
                backend_failure_detail=result.backend_failure_detail,
                backend_error_code=result.backend_error_code,
                backend_status_name=result.backend_status_name,
            )
            if result.success:
                if attempt > 1:
                    self.get_logger().info(
                        f"Hybrid goal [{label}] succeeded after {attempt} attempts."
                    )
                return result

            elapsed = time.monotonic() - started_at
            if (
                retry_goal_builder is None
                or not self._is_retryable_hybrid_failure(result)
                or elapsed >= retry_timeout_sec
            ):
                return result

            if joint_targets is not None and self._wait_for_hybrid_retry_observation(
                joint_targets,
                label,
                started_at,
                retry_timeout_sec,
            ):
                return MotionResult(
                    True,
                    total_time_s=time.monotonic() - started_at,
                    backend_name=self.planner,
                )

            elapsed = time.monotonic() - started_at
            if elapsed >= retry_timeout_sec:
                return result

            attempt += 1
            self.get_logger().warn(
                f"Hybrid goal [{label}] planning failed while blocked; retrying "
                f"attempt {attempt} for up to {retry_timeout_sec:.1f}s."
            )
            time.sleep(retry_delay_sec)

    def _send_curobo_pose_goal(self, target_pose, label: str) -> MotionResult:
        position = target_pose.position
        orientation = target_pose.orientation
        self.get_logger().info(
            f"Sending cuRobo pose goal [{label}] "
            f"xyz=({position.x:.3f}, {position.y:.3f}, {position.z:.3f})..."
        )
        planning_start_time = time.monotonic()
        response = self._curobo_backend._plan_pose(
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
            planner_type="classic",
        )
        planning_time_s = time.monotonic() - planning_start_time
        execution_start_time = time.monotonic()
        success = bool(self._curobo_backend._execute_response(response))
        execution_time_s = time.monotonic() - execution_start_time
        total_time_s = planning_time_s + execution_time_s
        if not success:
            return self._curobo_failure_result(
                planning_time_s=planning_time_s,
                execution_time_s=execution_time_s,
                total_time_s=total_time_s,
                fallback_reason="curobo_pose_goal_failed",
                fallback_detail=f"cuRobo pose goal [{label}] failed.",
            )
        return MotionResult(
            success,
            planning_time_s=planning_time_s,
            execution_time_s=execution_time_s,
            total_time_s=total_time_s,
            backend_name="curobo_ros",
        )

    def move_to_joint_positions(self, joint_positions, label: str) -> MotionResult:
        joint_targets = joint_position_map(joint_positions)
        missing = [joint_name for joint_name in JOINT_NAMES if joint_name not in joint_targets]
        if missing:
            self.get_logger().error(f"Joint target [{label}] is missing: {', '.join(missing)}")
            return MotionResult(False)
        if self._current_joints_match(joint_targets):
            self.get_logger().info(f"Joint target [{label}] already reached; skipping motion.")
            return MotionResult(True, total_time_s=0.0)
        if self.planner == "hybrid":
            def build_hybrid_joint_goal():
                return build_joint_goal(
                    joint_targets,
                    current_joint_positions=self._wait_for_recent_joint_positions(timeout_sec=1.0),
                    planning_pipeline=self._planning_pipeline(),
                    planner_id=self._moveit_planner_id(),
                    joint_tolerance=self.joint_tolerance,
                    normalize_periodic_targets=self.planner == "ompl",
                )

            return self._send_hybrid_goal(
                build_hybrid_joint_goal(),
                label,
                joint_targets=joint_targets,
                retry_goal_builder=build_hybrid_joint_goal,
            )
        if self._curobo_backend is not None:
            return self._send_curobo_joint_goal(joint_targets, label)
        return self._send_move_group_joint_goal(joint_targets, label)

    def pose_from_joint_positions(self, joint_positions, label: str):
        if not self.fk_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveIt FK service unavailable: /compute_fk")
            return None

        request = GetPositionFK.Request()
        request.header.frame_id = "world"
        request.fk_link_names = [END_EFFECTOR_LINK]
        request.robot_state = RobotState()
        request.robot_state.joint_state.name = list(JOINT_NAMES)
        request.robot_state.joint_state.position = [float(value) for value in joint_positions]

        response = self._wait_future_result(self.fk_client.call_async(request), timeout_sec=5.0)
        if response is None:
            self.get_logger().error(f"FK request [{label}] returned no response.")
            return None

        error_code = int(getattr(response.error_code, "val", 0))
        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"FK request [{label}] failed: {self._moveit_error_name(error_code)}({error_code})."
            )
            return None

        poses = list(getattr(response, "pose_stamped", []) or [])
        if not poses:
            self.get_logger().error(f"FK request [{label}] returned no poses.")
            return None
        return poses[0].pose

    def move_to_goal(self, case_spec: dict, label: str) -> MotionResult:
        if "goal_joint_positions" in case_spec:
            return self.move_to_joint_positions(case_spec["goal_joint_positions"], label=label)

        if "goal_pose" in case_spec:
            target_pose = self.pose_from_case_spec(case_spec["goal_pose"])
            if self.planner == "hybrid":
                goal = build_pose_goal(
                    target_pose,
                    current_joint_positions=self._wait_for_recent_joint_positions(timeout_sec=1.0),
                    planning_pipeline=self._planning_pipeline(),
                    planner_id=self._moveit_planner_id(),
                    position_tolerance_m=float(case_spec.get("goal_position_tolerance_m", 0.02)),
                    orientation_tolerance_rad=float(case_spec.get("goal_orientation_tolerance_rad", 0.35)),
                )
                return self._send_hybrid_goal(goal, label)
            if self._curobo_backend is not None:
                return self._send_curobo_pose_goal(target_pose, label)
            return self._send_move_group_pose_goal(target_pose, case_spec, label)

        if "goal_pose_from_joint_positions" in case_spec:
            target_pose = self.pose_from_joint_positions(
                case_spec["goal_pose_from_joint_positions"],
                label=f"{label}_fk",
            )
            if target_pose is None:
                return MotionResult(False)
            if self.planner == "hybrid":
                goal = build_pose_goal(
                    target_pose,
                    current_joint_positions=self._wait_for_recent_joint_positions(timeout_sec=1.0),
                    planning_pipeline=self._planning_pipeline(),
                    planner_id=self._moveit_planner_id(),
                    position_tolerance_m=float(case_spec.get("goal_position_tolerance_m", 0.02)),
                    orientation_tolerance_rad=float(case_spec.get("goal_orientation_tolerance_rad", 0.35)),
                )
                return self._send_hybrid_goal(goal, label)
            if self._curobo_backend is not None:
                return self._send_curobo_pose_goal(target_pose, label)
            return self._send_move_group_pose_goal(target_pose, case_spec, label)

        self.get_logger().error(f"Case '{case_spec['case_name']}' has no supported goal target.")
        return MotionResult(False)

    @staticmethod
    def pose_from_case_spec(pose_spec: dict) -> Pose:
        pose = Pose()
        pose.position.x = float(pose_spec["position"]["x"])
        pose.position.y = float(pose_spec["position"]["y"])
        pose.position.z = float(pose_spec["position"]["z"])
        pose.orientation.x = float(pose_spec["orientation"]["x"])
        pose.orientation.y = float(pose_spec["orientation"]["y"])
        pose.orientation.z = float(pose_spec["orientation"]["z"])
        pose.orientation.w = float(pose_spec["orientation"]["w"])
        return pose

    def publish_metrics_event(
        self,
        event: str,
        case_spec: dict,
        settle_after: bool = False,
        **fields,
    ) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "event": event,
                "case": case_spec["case_name"],
                "start_state": case_spec["start_state_name"],
                "goal_state": case_spec["goal_state_name"],
                **fields,
            }
        )
        self.metrics_event_pub.publish(msg)
        if settle_after:
            time.sleep(METRICS_PUBLISH_SETTLE_SEC)

    def run_case(self, case_spec: dict) -> bool:
        if not self.wait_until_ready():
            self.publish_metrics_event(
                "setup_failed",
                case_spec,
                settle_after=True,
                failure_reason=f"{self.planner}_planner_not_ready",
            )
            return False

        self.get_logger().info(
            f"Moving to unmeasured start state '{case_spec['start_state_name']}'."
        )
        setup_result = self.move_to_joint_positions(
            case_spec["start_joint_positions"],
            label=f"{case_spec['case_name']}_setup_start",
        )
        if not setup_result.success:
            self.get_logger().error("Failed to reach simple-motion start state.")
            self.publish_metrics_event(
                "setup_failed",
                case_spec,
                settle_after=True,
                failure_reason=(
                    setup_result.failure_reason or "failed_to_reach_start_state"
                ),
                failure_detail=setup_result.failure_detail,
                moveit_error_code=setup_result.moveit_error_code,
                moveit_error_name=setup_result.moveit_error_name,
                action_status_code=setup_result.action_status_code,
                action_status_name=setup_result.action_status_name,
                backend_name=setup_result.backend_name,
                backend_failure_reason=setup_result.backend_failure_reason,
                backend_failure_detail=setup_result.backend_failure_detail,
                backend_error_code=setup_result.backend_error_code,
                backend_status_name=setup_result.backend_status_name,
            )
            return False

        time.sleep(max(float(self.args.setup_settle_time), 0.0))

        self.publish_metrics_event("movement_start", case_spec, settle_after=True)
        result = self.move_to_goal(case_spec, label=f"{case_spec['case_name']}_measured_goal")
        self.publish_metrics_event(
            "movement_end",
            case_spec,
            settle_after=True,
            success=bool(result.success),
            planning_time_s=result.planning_time_s,
            execution_time_s=result.execution_time_s,
            total_time_s=result.total_time_s,
            failure_reason=(
                "" if result.success else result.failure_reason or "failed_to_move_to_goal_state"
            ),
            failure_detail="" if result.success else result.failure_detail,
            moveit_error_code=result.moveit_error_code,
            moveit_error_name=result.moveit_error_name,
            action_status_code=result.action_status_code,
            action_status_name=result.action_status_name,
            backend_name=result.backend_name,
            backend_failure_reason=result.backend_failure_reason,
            backend_failure_detail=result.backend_failure_detail,
            backend_error_code=result.backend_error_code,
            backend_status_name=result.backend_status_name,
        )
        return bool(result.success)

    def restore(self) -> None:
        if self._curobo_backend is not None:
            self._curobo_backend.restore_previous_planner()


def main() -> int:
    parser = build_arg_parser()
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = SimpleMotionBenchmark(args)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        case_spec = resolve_simple_motion_case(args.case)
        success = node.run_case(case_spec)
        return 0 if success else 1
    finally:
        node.restore()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
