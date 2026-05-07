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


DEFAULT_METRICS_EVENT_TOPIC = "/metrics_events"
DEFAULT_START_EVENT = "phase_start"
DEFAULT_END_EVENT = "phase_end"
DEFAULT_PHASE_FIELD = "phase"
ACTION_EXECUTING = 2
ACTION_FINISHED = {4, 5, 6}
EVENT_METADATA_FIELDS = (
    "failure_detail",
    "moveit_error_code",
    "moveit_error_name",
    "action_status_code",
    "action_status_name",
    "backend_name",
    "backend_failure_reason",
    "backend_failure_detail",
    "backend_error_code",
    "backend_status_name",
)


class TrajectoryMetricsRecorder(Node):
    def __init__(
        self,
        metrics_event_topic: str = DEFAULT_METRICS_EVENT_TOPIC,
        world_frame: str = "world",
        tcp_frame: str = "TCP_point",
        start_event: str = DEFAULT_START_EVENT,
        end_event: str = DEFAULT_END_EVENT,
        phase_field: str = DEFAULT_PHASE_FIELD,
        default_phase: str = "",
    ):
        super().__init__("trajectory_metrics_recorder")
        self.metrics_event_topic = metrics_event_topic
        self.world_frame = world_frame
        self.tcp_frame = tcp_frame
        self.start_event = start_event
        self.end_event = end_event
        self.phase_field = phase_field
        self.default_phase = default_phase
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.current_phase = None
        self.phase_order = []
        self.segments = {}
        self.failed_phase = None
        self.failure_reason = ""
        self.metadata = {}
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
            self.metrics_event_topic,
            self.on_metrics_event,
            10,
        )

    def on_metrics_event(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        event_type = event.get("event")
        if event_type == "setup_failed":
            self.record_setup_failure(event)
            return

        phase = self.phase_from_event(event)
        if not phase:
            return

        if event_type == self.start_event:
            self.start_phase(phase)
        elif event_type == self.end_event:
            planning_time_s = event.get("planning_time_s")
            failure_reason = str(event.get("failure_reason") or "")
            self.finish_phase(
                bool(event.get("success", False)),
                phase=phase,
                planning_time_s=planning_time_s,
                execution_time_s=event.get("execution_time_s"),
                total_time_s=event.get("total_time_s"),
                failure_reason=failure_reason,
                metadata=self.metadata_from_event(event),
            )

    def phase_from_event(self, event: dict) -> str:
        phase = str(event.get(self.phase_field) or "")
        if phase:
            return phase
        return self.default_phase

    def metadata_from_event(self, event: dict) -> dict:
        return {field: event.get(field) for field in EVENT_METADATA_FIELDS}

    def record_setup_failure(self, event: dict) -> None:
        phase = self.phase_from_event(event)
        if phase and phase not in self.phase_order:
            self.phase_order.append(phase)
        self.failed_phase = phase or self.failed_phase
        self.failure_reason = str(event.get("failure_reason") or "setup_failed")
        self.metadata = self.metadata_from_event(event)

    def start_phase(self, phase: str) -> None:
        if self.current_phase is not None:
            if self.current_phase["phase"] == phase:
                return
            self.finish_phase(False, failure_reason=f"interrupted_by_{phase}")

        if phase not in self.phase_order:
            self.phase_order.append(phase)

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
        execution_time_s=None,
        total_time_s=None,
        failure_reason: str = "",
        metadata: dict | None = None,
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

        reported_execution_time_s = self.optional_float(execution_time_s)
        reported_total_time_s = self.optional_float(total_time_s)

        elapsed_time_s = max(0.0, time.monotonic() - phase_data["start_time"])
        if reported_total_time_s is not None:
            elapsed_time_s = reported_total_time_s

        if phase_data["reported_planning_time_s"] is not None:
            planning_time_s = phase_data["reported_planning_time_s"]
        elif phase_data["first_execution_start_time"] is not None:
            planning_time_s = max(
                0.0, phase_data["first_execution_start_time"] - phase_data["start_time"]
            )
        else:
            planning_time_s = None

        execution_time_s = (
            reported_execution_time_s
            if reported_execution_time_s is not None
            else phase_data["execution_time_s"]
        )
        if execution_time_s <= 0.0:
            if planning_time_s is None:
                execution_time_s = elapsed_time_s
            else:
                execution_time_s = max(0.0, elapsed_time_s - planning_time_s)
        if (
            phase_data["reported_planning_time_s"] is None
            and reported_total_time_s is not None
            and execution_time_s is not None
        ):
            planning_time_s = max(0.0, reported_total_time_s - execution_time_s)

        self.segments[phase_data["phase"]] = {
            "phase": phase_data["phase"],
            "planning_time_s": planning_time_s,
            "execution_time_s": execution_time_s,
            "elapsed_time_s": elapsed_time_s,
            "tcp_movement_cm": phase_data["movement_m"] * 100.0,
            "success": bool(success),
            "failure_reason": failure_reason,
        }
        if metadata:
            self.segments[phase_data["phase"]].update(metadata)
            self.metadata = metadata
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

    @staticmethod
    def optional_float(value):
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def sample_tcp_position(self) -> None:
        if self.current_phase is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.tcp_frame,
                Time(),
            )
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
        success = bool(self.segments) and all(
            segment.get("success") is True for segment in self.segments.values()
        )
        if self.segments:
            totals = {
                "planning_time_s": self.sum_metric("planning_time_s"),
                "execution_time_s": self.sum_metric("execution_time_s"),
                "tcp_movement_cm": self.sum_metric("tcp_movement_cm"),
            }
        else:
            totals = {
                "planning_time_s": None,
                "execution_time_s": None,
                "tcp_movement_cm": None,
            }
        return {
            "phases": list(self.phase_order),
            "segments": self.segments,
            "totals": totals,
            "success": success,
            "failed_phase": self.failed_phase,
            "failure_reason": self.failure_reason,
            **self.metadata,
        }

    def sum_metric(self, key: str) -> float:
        return sum(
            float(segment[key])
            for segment in self.segments.values()
            if segment.get(key) is not None
        )


def main() -> int:
    stop_file = Path(sys.argv[1])
    metrics_event_topic = (
        sys.argv[2] if len(sys.argv) > 2 else DEFAULT_METRICS_EVENT_TOPIC
    )
    world_frame = sys.argv[3] if len(sys.argv) > 3 else "world"
    tcp_frame = sys.argv[4] if len(sys.argv) > 4 else "TCP_point"
    start_event = sys.argv[5] if len(sys.argv) > 5 else DEFAULT_START_EVENT
    end_event = sys.argv[6] if len(sys.argv) > 6 else DEFAULT_END_EVENT
    phase_field = sys.argv[7] if len(sys.argv) > 7 else DEFAULT_PHASE_FIELD
    default_phase = sys.argv[8] if len(sys.argv) > 8 else ""

    rclpy.init()
    node = TrajectoryMetricsRecorder(
        metrics_event_topic,
        world_frame,
        tcp_frame,
        start_event,
        end_event,
        phase_field,
        default_phase,
    )
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
