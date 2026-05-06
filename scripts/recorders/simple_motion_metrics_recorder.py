#!/usr/bin/env python3
"""Record metrics for one generic simple-motion movement."""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatusArray
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


METRICS_EVENT_TOPIC = "/simple_motion/metrics_events"
ACTION_EXECUTING = 2
ACTION_FINISHED = {4, 5, 6}


class SimpleMotionMetricsRecorder(Node):
    def __init__(self):
        super().__init__("simple_motion_metrics_recorder")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.current_movement = None
        self.movement = None
        self.case_name = ""
        self.failure_reason = ""
        self.failure_detail = ""
        self.moveit_error_code = None
        self.moveit_error_name = ""
        self.action_status_code = None
        self.action_status_name = ""
        self.backend_name = ""
        self.backend_failure_reason = ""
        self.backend_failure_detail = ""
        self.backend_error_code = None
        self.backend_status_name = ""

        self.create_timer(0.05, self.sample_tcp_position)
        self.create_subscription(
            GoalStatusArray,
            "/joint_trajectory_controller/follow_joint_trajectory/_action/status",
            self.on_action_status,
            10,
        )
        self.create_subscription(
            GoalStatusArray,
            "/unified_planner/execute_trajectory/_action/status",
            self.on_action_status,
            10,
        )
        self.create_subscription(String, METRICS_EVENT_TOPIC, self.on_metrics_event, 10)

    def on_metrics_event(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        case_name = str(event.get("case") or "")
        if case_name:
            self.case_name = case_name

        event_type = event.get("event")
        if event_type == "movement_start":
            self.start_movement(case_name)
        elif event_type == "movement_end":
            self.finish_movement(
                bool(event.get("success", False)),
                planning_time_s=event.get("planning_time_s"),
                execution_time_s=event.get("execution_time_s"),
                total_time_s=event.get("total_time_s"),
                failure_reason=str(event.get("failure_reason") or ""),
                failure_detail=str(event.get("failure_detail") or ""),
                moveit_error_code=event.get("moveit_error_code"),
                moveit_error_name=str(event.get("moveit_error_name") or ""),
                action_status_code=event.get("action_status_code"),
                action_status_name=str(event.get("action_status_name") or ""),
                backend_name=str(event.get("backend_name") or ""),
                backend_failure_reason=str(event.get("backend_failure_reason") or ""),
                backend_failure_detail=str(event.get("backend_failure_detail") or ""),
                backend_error_code=event.get("backend_error_code"),
                backend_status_name=str(event.get("backend_status_name") or ""),
            )
        elif event_type == "setup_failed":
            self.failure_reason = str(
                event.get("failure_reason") or "failed_to_reach_start_state"
            )
            self.failure_detail = str(event.get("failure_detail") or "")
            self.moveit_error_code = event.get("moveit_error_code")
            self.moveit_error_name = str(event.get("moveit_error_name") or "")
            self.action_status_code = event.get("action_status_code")
            self.action_status_name = str(event.get("action_status_name") or "")
            self.backend_name = str(event.get("backend_name") or "")
            self.backend_failure_reason = str(event.get("backend_failure_reason") or "")
            self.backend_failure_detail = str(event.get("backend_failure_detail") or "")
            self.backend_error_code = event.get("backend_error_code")
            self.backend_status_name = str(event.get("backend_status_name") or "")

    def start_movement(self, case_name: str) -> None:
        if self.current_movement is not None:
            self.finish_movement(False, failure_reason="interrupted_by_new_movement")

        self.current_movement = {
            "case_name": case_name,
            "start_time": time.monotonic(),
            "first_execution_start_time": None,
            "execution_start_time": None,
            "execution_goal_id": None,
            "execution_time_s": 0.0,
            "reported_planning_time_s": None,
            "last_position": None,
            "movement_m": 0.0,
        }
        self.sample_tcp_position()

    def finish_movement(
        self,
        success: bool,
        planning_time_s=None,
        execution_time_s=None,
        total_time_s=None,
        failure_reason: str = "",
        failure_detail: str = "",
        moveit_error_code=None,
        moveit_error_name: str = "",
        action_status_code=None,
        action_status_name: str = "",
        backend_name: str = "",
        backend_failure_reason: str = "",
        backend_failure_detail: str = "",
        backend_error_code=None,
        backend_status_name: str = "",
    ) -> None:
        if self.current_movement is None:
            return
        if planning_time_s is not None:
            try:
                self.current_movement["reported_planning_time_s"] = float(planning_time_s)
            except (TypeError, ValueError):
                pass
        reported_execution_time_s = self._optional_float(execution_time_s)
        reported_total_time_s = self._optional_float(total_time_s)

        self.sample_tcp_position()
        self.stop_execution_timer()
        movement = self.current_movement
        self.current_movement = None

        elapsed_time_s = max(0.0, time.monotonic() - movement["start_time"])
        if reported_total_time_s is not None:
            elapsed_time_s = reported_total_time_s
        if movement["reported_planning_time_s"] is not None:
            planning_time_s = movement["reported_planning_time_s"]
        elif movement["first_execution_start_time"] is not None:
            planning_time_s = max(
                0.0,
                movement["first_execution_start_time"] - movement["start_time"],
            )
        else:
            planning_time_s = None

        execution_time_s = (
            reported_execution_time_s
            if reported_execution_time_s is not None
            else movement["execution_time_s"]
        )
        if execution_time_s <= 0.0:
            if planning_time_s is None:
                execution_time_s = elapsed_time_s
            else:
                execution_time_s = max(0.0, elapsed_time_s - planning_time_s)
        if (
            movement["reported_planning_time_s"] is None
            and reported_total_time_s is not None
            and execution_time_s is not None
        ):
            planning_time_s = max(0.0, reported_total_time_s - execution_time_s)

        self.movement = {
            "case_name": movement["case_name"],
            "planning_time_s": planning_time_s,
            "execution_time_s": execution_time_s,
            "elapsed_time_s": elapsed_time_s,
            "tcp_movement_cm": movement["movement_m"] * 100.0,
            "success": bool(success),
            "failure_reason": failure_reason,
            "failure_detail": failure_detail,
            "moveit_error_code": moveit_error_code,
            "moveit_error_name": moveit_error_name,
            "action_status_code": action_status_code,
            "action_status_name": action_status_name,
            "backend_name": backend_name,
            "backend_failure_reason": backend_failure_reason,
            "backend_failure_detail": backend_failure_detail,
            "backend_error_code": backend_error_code,
            "backend_status_name": backend_status_name,
        }
        if not success:
            self.failure_reason = failure_reason

    def stop_execution_timer(self) -> None:
        if self.current_movement is None:
            return
        if self.current_movement["execution_start_time"] is None:
            return
        self.current_movement["execution_time_s"] += max(
            0.0,
            time.monotonic() - self.current_movement["execution_start_time"],
        )
        self.current_movement["execution_start_time"] = None
        self.current_movement["execution_goal_id"] = None

    def on_action_status(self, msg: GoalStatusArray) -> None:
        if self.current_movement is None:
            return

        executing_ids = {
            self.goal_id_text(status)
            for status in msg.status_list
            if int(status.status) == ACTION_EXECUTING
        }
        finished_ids = {
            self.goal_id_text(status)
            for status in msg.status_list
            if int(status.status) in ACTION_FINISHED
        }
        now = time.monotonic()

        if self.current_movement["execution_goal_id"] is None and executing_ids:
            goal_id = sorted(executing_ids)[0]
            self.current_movement["execution_goal_id"] = goal_id
            self.current_movement["execution_start_time"] = now
            if self.current_movement["first_execution_start_time"] is None:
                self.current_movement["first_execution_start_time"] = now

        goal_id = self.current_movement["execution_goal_id"]
        if goal_id is not None and goal_id in finished_ids:
            self.stop_execution_timer()

    @staticmethod
    def goal_id_text(status) -> str:
        return bytes(status.goal_info.goal_id.uuid).hex()

    @staticmethod
    def _optional_float(value):
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def sample_tcp_position(self) -> None:
        if self.current_movement is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform("world", "TCP_point", Time())
        except TransformException:
            return

        translation = transform.transform.translation
        position = (translation.x, translation.y, translation.z)
        last_position = self.current_movement["last_position"]
        if last_position is not None:
            dx = position[0] - last_position[0]
            dy = position[1] - last_position[1]
            dz = position[2] - last_position[2]
            self.current_movement["movement_m"] += math.sqrt(dx * dx + dy * dy + dz * dz)
        self.current_movement["last_position"] = position

    def result(self) -> dict:
        if self.movement is None:
            return {
                "case_name": self.case_name,
                "movement": {},
                "totals": {
                    "planning_time_s": None,
                    "execution_time_s": None,
                    "tcp_movement_cm": None,
                },
                "success": False,
                "failure_reason": self.failure_reason or "movement_not_recorded",
                "failure_detail": self.failure_detail,
                "moveit_error_code": self.moveit_error_code,
                "moveit_error_name": self.moveit_error_name,
                "action_status_code": self.action_status_code,
                "action_status_name": self.action_status_name,
                "backend_name": self.backend_name,
                "backend_failure_reason": self.backend_failure_reason,
                "backend_failure_detail": self.backend_failure_detail,
                "backend_error_code": self.backend_error_code,
                "backend_status_name": self.backend_status_name,
            }

        return {
            "case_name": self.case_name or self.movement.get("case_name", ""),
            "movement": self.movement,
            "totals": {
                "planning_time_s": self.movement.get("planning_time_s"),
                "execution_time_s": self.movement.get("execution_time_s"),
                "tcp_movement_cm": self.movement.get("tcp_movement_cm"),
            },
            "success": bool(self.movement.get("success")),
            "failure_reason": self.failure_reason or self.movement.get("failure_reason", ""),
            "failure_detail": self.movement.get("failure_detail", ""),
            "moveit_error_code": self.movement.get("moveit_error_code"),
            "moveit_error_name": self.movement.get("moveit_error_name", ""),
            "action_status_code": self.movement.get("action_status_code"),
            "action_status_name": self.movement.get("action_status_name", ""),
            "backend_name": self.movement.get("backend_name", ""),
            "backend_failure_reason": self.movement.get("backend_failure_reason", ""),
            "backend_failure_detail": self.movement.get("backend_failure_detail", ""),
            "backend_error_code": self.movement.get("backend_error_code"),
            "backend_status_name": self.movement.get("backend_status_name", ""),
        }


def main() -> int:
    stop_file = Path(sys.argv[1])

    rclpy.init()
    node = SimpleMotionMetricsRecorder()
    try:
        while rclpy.ok() and not stop_file.exists():
            rclpy.spin_once(node, timeout_sec=0.05)

        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)

        if node.current_movement is not None:
            node.finish_movement(False, failure_reason="recorder_stopped_with_active_movement")
        print(json.dumps(node.result(), sort_keys=True), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
