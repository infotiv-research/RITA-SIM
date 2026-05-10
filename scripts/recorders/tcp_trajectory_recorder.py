#!/usr/bin/env python3
import json
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


DEFAULT_METRICS_EVENT_TOPIC = "/metrics_events"
DEFAULT_START_EVENT = "movement_start"
DEFAULT_END_EVENT = "movement_end"
DEFAULT_SAMPLING_RATE_HZ = 20.0


class TcpTrajectoryRecorder(Node):
    def __init__(
        self,
        metrics_event_topic: str = DEFAULT_METRICS_EVENT_TOPIC,
        world_frame: str = "world",
        tcp_frame: str = "TCP_point",
        start_event: str = DEFAULT_START_EVENT,
        end_event: str = DEFAULT_END_EVENT,
        sampling_rate: float = DEFAULT_SAMPLING_RATE_HZ,
    ):
        super().__init__("tcp_trajectory_recorder")
        self.world_frame = world_frame
        self.tcp_frame = tcp_frame
        self.start_event = start_event
        self.end_event = end_event
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.recording = False
        self.points = []
        self.start_time_monotonic = None
        
        self.create_subscription(
            String,
            metrics_event_topic,
            self.on_metrics_event,
            10,
        )
        
        self.timer = self.create_timer(1.0 / sampling_rate, self.sample_tcp)

    def on_metrics_event(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        event_type = event.get("event")
        if event_type == self.start_event:
            self.get_logger().info(f"Starting TCP recording (event: {self.start_event})")
            self.points = []
            self.start_time_monotonic = time.monotonic()
            self.recording = True
        elif event_type == self.end_event:
            self.get_logger().info(f"Stopping TCP recording (event: {self.end_event})")
            self.recording = False

    def sample_tcp(self) -> None:
        if not self.recording:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.tcp_frame,
                Time(),
            )
        except TransformException:
            return

        now_monotonic = time.monotonic()
        elapsed_s = now_monotonic - self.start_time_monotonic
        
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        
        self.points.append({
            "time_s": round(elapsed_s, 4),
            "x": round(translation.x, 6),
            "y": round(translation.y, 6),
            "z": round(translation.z, 6),
            "qx": round(rotation.x, 6),
            "qy": round(rotation.y, 6),
            "qz": round(rotation.z, 6),
            "qw": round(rotation.w, 6),
        })

    def get_result(self) -> list:
        return self.points


def main() -> int:
    stop_file = Path(sys.argv[1])
    metrics_topic = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_METRICS_EVENT_TOPIC
    world_frame = sys.argv[3] if len(sys.argv) > 3 else "world"
    tcp_frame = sys.argv[4] if len(sys.argv) > 4 else "TCP_point"
    start_event = sys.argv[5] if len(sys.argv) > 5 else DEFAULT_START_EVENT
    end_event = sys.argv[6] if len(sys.argv) > 6 else DEFAULT_END_EVENT

    rclpy.init()
    node = TcpTrajectoryRecorder(
        metrics_event_topic=metrics_topic,
        world_frame=world_frame,
        tcp_frame=tcp_frame,
        start_event=start_event,
        end_event=end_event
    )
    
    try:
        while rclpy.ok() and not stop_file.exists():
            rclpy.spin_once(node, timeout_sec=0.05)

        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.01)

        print(json.dumps(node.get_result()))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
