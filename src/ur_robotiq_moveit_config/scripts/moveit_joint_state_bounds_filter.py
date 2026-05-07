#!/usr/bin/env python3

# Republishes /joint_states onto a MoveIt-facing topic, nudging arm joint
# positions that sit a hair outside their URDF limits back inside. Prevents
# MoveIt from rejecting the start state as out-of-bounds when sim/controller
# values drift by tiny floating-point amounts past the joint limits.

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


ARM_JOINT_LIMITS = {
    "gantry_joint": (-0.75, 0.75),
    "shoulder_pan_joint": (-2.0 * math.pi, 2.0 * math.pi),
    "shoulder_lift_joint": (-2.0 * math.pi, 2.0 * math.pi),
    "elbow_joint": (-math.pi, math.pi),
    "wrist_1_joint": (-2.0 * math.pi, 2.0 * math.pi),
    "wrist_2_joint": (-2.0 * math.pi, 2.0 * math.pi),
    "wrist_3_joint": (-2.0 * math.pi, 2.0 * math.pi),
}


class MoveItJointStateBoundsFilter(Node):
    def __init__(self):
        super().__init__("moveit_joint_state_bounds_filter")
        self.input_topic = str(
            self.declare_parameter("input_topic", "/joint_states").value
        )
        self.output_topic = str(
            self.declare_parameter("output_topic", "/moveit_bounded_joint_states").value
        )
        self.limit_epsilon = float(
            self.declare_parameter("limit_epsilon", 1e-5).value
        )
        self.max_correction = float(
            self.declare_parameter("max_correction", 1e-2).value
        )
        self._correction_count = 0

        self.pub = self.create_publisher(JointState, self.output_topic, 20)
        self.sub = self.create_subscription(
            JointState,
            self.input_topic,
            self._on_joint_state,
            20,
        )
        self.get_logger().info(
            "Filtering joint states for MoveIt bounds: "
            f"{self.input_topic} -> {self.output_topic}, "
            f"limit_epsilon={self.limit_epsilon:g}, "
            f"max_correction={self.max_correction:g}"
        )

    def _bounded_position(self, joint_name, position):
        limits = ARM_JOINT_LIMITS.get(joint_name)
        if limits is None:
            return float(position)

        lower, upper = limits
        value = float(position)
        low_target = lower + self.limit_epsilon
        high_target = upper - self.limit_epsilon

        if value < low_target and abs(value - lower) <= self.max_correction:
            return low_target
        if value > high_target and abs(value - upper) <= self.max_correction:
            return high_target
        return value

    def _on_joint_state(self, msg):
        out = JointState()
        out.header = msg.header
        out.name = list(msg.name)
        out.position = [
            self._bounded_position(name, position)
            for name, position in zip(msg.name, msg.position)
        ]
        out.velocity = list(msg.velocity)
        out.effort = list(msg.effort)

        for name, before, after in zip(msg.name, msg.position, out.position):
            if abs(float(before) - float(after)) > 0.0:
                self._correction_count += 1
                if self._correction_count <= 5 or self._correction_count % 100 == 0:
                    self.get_logger().warn(
                        "Clamped joint state for MoveIt bounds: "
                        f"{name} {float(before):.8f} -> {float(after):.8f} "
                        f"(correction #{self._correction_count})"
                    )

        self.pub.publish(out)


def main():
    rclpy.init()
    node = MoveItJointStateBoundsFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
