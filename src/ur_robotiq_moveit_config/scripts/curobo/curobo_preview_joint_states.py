#!/usr/bin/env python3

import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class CuroboPreviewJointStates(Node):
    def __init__(self):
        super().__init__("curobo_preview_joint_states")

        self.trajectory_topic = str(
            self.declare_parameter("trajectory_topic", "/trajectory").value
        )
        self.preview_joint_states_topic = str(
            self.declare_parameter("preview_joint_states_topic", "/trajectory/joint_states").value
        )
        self.publish_final_state = bool(
            self.declare_parameter("publish_final_state", True).value
        )

        self._publisher = self.create_publisher(
            JointState,
            self.preview_joint_states_topic,
            10,
        )
        self._subscription = self.create_subscription(
            JointTrajectory,
            self.trajectory_topic,
            self._on_trajectory,
            10,
        )

        self._executor_thread = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

    def _on_trajectory(self, msg: JointTrajectory):
        if not msg.points:
            return

        with self._lock:
            self._stop_active_worker_locked()
            self._stop_event = threading.Event()
            self._executor_thread = threading.Thread(
                target=self._publish_preview,
                args=(msg, self._stop_event),
                daemon=True,
                name="curobo_preview_joint_states",
            )
            self._executor_thread.start()

    def _stop_active_worker_locked(self):
        if self._executor_thread is None:
            return

        self._stop_event.set()
        if self._executor_thread.is_alive():
            self._executor_thread.join(timeout=0.5)
        self._executor_thread = None

    def _publish_preview(self, msg: JointTrajectory, stop_event: threading.Event):
        start_time = time.monotonic()
        previous_time = 0.0

        for point in msg.points:
            if stop_event.is_set():
                return

            target_time = float(point.time_from_start.sec) + (
                float(point.time_from_start.nanosec) * 1e-9
            )
            sleep_duration = max(0.0, target_time - previous_time)
            if sleep_duration > 0.0:
                deadline = start_time + target_time
                while not stop_event.is_set():
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        break
                    time.sleep(min(remaining, 0.01))

            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = list(msg.joint_names)
            joint_state.position = [float(v) for v in point.positions]
            joint_state.velocity = [float(v) for v in point.velocities]
            joint_state.effort = []
            self._publisher.publish(joint_state)
            previous_time = target_time

        if self.publish_final_state and not stop_event.is_set() and msg.points:
            final_point = msg.points[-1]
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = list(msg.joint_names)
            joint_state.position = [float(v) for v in final_point.positions]
            joint_state.velocity = [0.0] * len(final_point.positions)
            joint_state.effort = []
            self._publisher.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = CuroboPreviewJointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        with node._lock:
            node._stop_active_worker_locked()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
