#!/usr/bin/env python3
"""Record per-joint movement for phase events."""

import json
import math
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


DEFAULT_METRICS_EVENT_TOPIC = "/metrics_events"
DEFAULT_JOINT_STATES_TOPIC = "/joint_states"
DEFAULT_START_EVENT = "phase_start"
DEFAULT_END_EVENT = "phase_end"
DEFAULT_PHASE_FIELD = "phase"


class JointMovementRecorder(Node):
    def __init__(
        self,
        metrics_event_topic: str,
        joint_states_topic: str,
        rotational_joints: list[str],
        linear_joints: list[str],
        start_event: str = DEFAULT_START_EVENT,
        end_event: str = DEFAULT_END_EVENT,
        phase_field: str = DEFAULT_PHASE_FIELD,
        default_phase: str = "",
    ):
        super().__init__("joint_movement_recorder")
        self.rotational_joints = rotational_joints
        self.linear_joints = linear_joints
        self.tracked_joints = rotational_joints + linear_joints
        self.start_event = start_event
        self.end_event = end_event
        self.phase_field = phase_field
        self.default_phase = default_phase
        self.latest_positions = {}
        self.current_phase = None
        self.phase_order = []
        self.segments = {}

        self.create_subscription(String, metrics_event_topic, self.on_metrics_event, 10)
        self.create_subscription(JointState, joint_states_topic, self.on_joint_state, 20)

    def on_metrics_event(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        event_type = event.get("event")
        phase = self.phase_from_event(event)
        if not phase:
            return

        if event_type == self.start_event:
            self.start_phase(phase)
        elif event_type == self.end_event:
            self.finish_phase(phase)

    def phase_from_event(self, event: dict) -> str:
        phase = str(event.get(self.phase_field) or "")
        if phase:
            return phase
        return self.default_phase

    def start_phase(self, phase: str) -> None:
        if self.current_phase is not None:
            self.finish_phase(self.current_phase["phase"])

        if phase not in self.phase_order:
            self.phase_order.append(phase)

        self.current_phase = {
            "phase": phase,
            "last_positions": dict(self.latest_positions),
            "movement": {joint: 0.0 for joint in self.tracked_joints},
        }

    def finish_phase(self, phase: str) -> None:
        if self.current_phase is None:
            return
        if self.current_phase["phase"] != phase:
            return

        movement = self.current_phase["movement"]
        self.current_phase = None
        self.segments[phase] = {
            "joints": self.format_joint_movement(movement),
        }

    def on_joint_state(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        self.update_latest_positions(positions)

        if self.current_phase is None:
            return

        self.accumulate_current_phase_movement(positions)

    def update_latest_positions(self, positions: dict) -> None:
        for joint in self.tracked_joints:
            if joint in positions:
                self.latest_positions[joint] = float(positions[joint])

    def accumulate_current_phase_movement(self, positions: dict) -> None:
        last_positions = self.current_phase["last_positions"]
        movement = self.current_phase["movement"]

        for joint in self.tracked_joints:
            if joint not in positions or joint not in last_positions:
                continue

            current_position = float(positions[joint])
            movement[joint] += abs(current_position - last_positions[joint])
            last_positions[joint] = current_position

    def format_joint_movement(self, movement: dict) -> dict:
        joints = {}

        for joint in self.rotational_joints:
            joints[joint] = {
                "movement_deg": math.degrees(movement.get(joint, 0.0)),
            }

        for joint in self.linear_joints:
            joints[joint] = {
                "movement_m": movement.get(joint, 0.0),
            }

        return joints

    def result(self) -> dict:
        return {
            "phases": list(self.phase_order),
            "segments": self.segments,
        }


def csv_list(value: str) -> list[str]:
    return [item for item in value.split(",") if item]


def main() -> int:
    stop_file = Path(sys.argv[1])
    metrics_event_topic = (
        sys.argv[2] if len(sys.argv) > 2 else DEFAULT_METRICS_EVENT_TOPIC
    )
    joint_states_topic = sys.argv[3] if len(sys.argv) > 3 else DEFAULT_JOINT_STATES_TOPIC
    rotational_joints = csv_list(sys.argv[4]) if len(sys.argv) > 4 else []
    linear_joints = csv_list(sys.argv[5]) if len(sys.argv) > 5 else []
    start_event = sys.argv[6] if len(sys.argv) > 6 else DEFAULT_START_EVENT
    end_event = sys.argv[7] if len(sys.argv) > 7 else DEFAULT_END_EVENT
    phase_field = sys.argv[8] if len(sys.argv) > 8 else DEFAULT_PHASE_FIELD
    default_phase = sys.argv[9] if len(sys.argv) > 9 else ""

    rclpy.init()
    node = JointMovementRecorder(
        metrics_event_topic,
        joint_states_topic,
        rotational_joints,
        linear_joints,
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
            node.finish_phase(node.current_phase["phase"])
        print(json.dumps(node.result(), sort_keys=True), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
