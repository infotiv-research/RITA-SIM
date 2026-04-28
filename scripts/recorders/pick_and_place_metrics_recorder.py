#!/usr/bin/env python3
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


TRAJECTORY_PHASES = (
    "pre_grasp",
    "grasp",
    "post_grasp_lift",
    "pre_drop",
    "release",
    "post_release_retreat",
    "return_home",
)

METRICS_EVENT_TOPIC = "/pick_and_place/metrics_events"
ACTION_EXECUTING = 2
ACTION_FINISHED = {4, 5, 6}


class PickAndPlaceMetricsRecorder(Node):
    def __init__(self):
        super().__init__("pick_and_place_metrics_recorder")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.current_phase = None
        self.segments = {}
        self.failed_phase = None
        self.failure_reason = ""
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
        self.create_subscription(
            String,
            METRICS_EVENT_TOPIC,
            self.on_metrics_event,
            10,
        )

    def on_metrics_event(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        phase = event.get("phase")
        if phase not in TRAJECTORY_PHASES:
            return

        if event.get("event") == "phase_start":
            self.start_phase(phase)
        elif event.get("event") == "phase_end":
            planning_time_s = event.get("planning_time_s")
            failure_reason = str(event.get("failure_reason") or "")
            self.finish_phase(
                bool(event.get("success", False)),
                phase=phase,
                planning_time_s=planning_time_s,
                failure_reason=failure_reason,
            )

    def start_phase(self, phase: str) -> None:
        if self.current_phase is not None:
            if self.current_phase["phase"] == phase:
                return
            self.finish_phase(False, failure_reason=f"interrupted_by_{phase}")

        self.current_phase = {
            "phase": phase,
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

    def finish_phase(
        self,
        success: bool,
        phase: str | None = None,
        planning_time_s=None,
        failure_reason: str = "",
    ) -> None:
        if self.current_phase is None:
            return
        if phase is not None and self.current_phase["phase"] != phase:
            return
        if planning_time_s is not None:
            try:
                self.current_phase["reported_planning_time_s"] = float(planning_time_s)
            except (TypeError, ValueError):
                pass

        self.sample_tcp_position()
        self.stop_execution_timer()
        phase_data = self.current_phase
        self.current_phase = None

        elapsed_time_s = max(0.0, time.monotonic() - phase_data["start_time"])
        if phase_data["reported_planning_time_s"] is not None:
            planning_time_s = phase_data["reported_planning_time_s"]
        elif phase_data["first_execution_start_time"] is not None:
            planning_time_s = max(
                0.0, phase_data["first_execution_start_time"] - phase_data["start_time"]
            )
        else:
            planning_time_s = None

        execution_time_s = phase_data["execution_time_s"]
        if execution_time_s <= 0.0:
            if planning_time_s is None:
                execution_time_s = elapsed_time_s
            else:
                execution_time_s = max(0.0, elapsed_time_s - planning_time_s)

        self.segments[phase_data["phase"]] = {
            "phase": phase_data["phase"],
            "planning_time_s": planning_time_s,
            "execution_time_s": execution_time_s,
            "elapsed_time_s": elapsed_time_s,
            "tcp_movement_cm": phase_data["movement_m"] * 100.0,
            "success": bool(success),
            "failure_reason": failure_reason,
        }
        if not success and self.failed_phase is None:
            self.failed_phase = phase_data["phase"]
            self.failure_reason = failure_reason

    def stop_execution_timer(self) -> None:
        if self.current_phase is None:
            return
        if self.current_phase["execution_start_time"] is None:
            return
        self.current_phase["execution_time_s"] += max(
            0.0, time.monotonic() - self.current_phase["execution_start_time"]
        )
        self.current_phase["execution_start_time"] = None
        self.current_phase["execution_goal_id"] = None

    def on_action_status(self, msg: GoalStatusArray) -> None:
        if self.current_phase is None:
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

        if self.current_phase["execution_goal_id"] is None and executing_ids:
            goal_id = sorted(executing_ids)[0]
            self.current_phase["execution_goal_id"] = goal_id
            self.current_phase["execution_start_time"] = now
            if self.current_phase["first_execution_start_time"] is None:
                self.current_phase["first_execution_start_time"] = now

        goal_id = self.current_phase["execution_goal_id"]
        if goal_id is not None and goal_id in finished_ids:
            self.stop_execution_timer()

    @staticmethod
    def goal_id_text(status) -> str:
        return bytes(status.goal_info.goal_id.uuid).hex()

    def sample_tcp_position(self) -> None:
        if self.current_phase is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform("world", "TCP_point", Time())
        except TransformException:
            return

        translation = transform.transform.translation
        position = (translation.x, translation.y, translation.z)
        last_position = self.current_phase["last_position"]
        if last_position is not None:
            dx = position[0] - last_position[0]
            dy = position[1] - last_position[1]
            dz = position[2] - last_position[2]
            self.current_phase["movement_m"] += math.sqrt(dx * dx + dy * dy + dz * dz)
        self.current_phase["last_position"] = position

    def result(self) -> dict:
        success = all(
            self.segments.get(phase, {}).get("success") is True
            for phase in TRAJECTORY_PHASES
        )
        totals = {
            "planning_time_s": self.sum_metric("planning_time_s"),
            "execution_time_s": self.sum_metric("execution_time_s"),
            "tcp_movement_cm": self.sum_metric("tcp_movement_cm"),
        }
        return {
            "phases": list(TRAJECTORY_PHASES),
            "segments": self.segments,
            "totals": totals,
            "success": success,
            "failed_phase": self.failed_phase,
            "failure_reason": self.failure_reason,
        }

    def sum_metric(self, key: str) -> float:
        return sum(
            float(segment[key])
            for segment in self.segments.values()
            if segment.get(key) is not None
        )


def main() -> int:
    stop_file = Path(sys.argv[1])

    rclpy.init()
    node = PickAndPlaceMetricsRecorder()
    try:
        while rclpy.ok() and not stop_file.exists():
            rclpy.spin_once(node, timeout_sec=0.05)

        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)

        if node.current_phase is not None:
            node.finish_phase(False, failure_reason="recorder_stopped_with_active_phase")
        print(json.dumps(node.result(), sort_keys=True), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
