#!/usr/bin/env python3

"""Ensure the arm is at the configured home pose during stack startup."""

import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pick_and_place.constants import (
    ARM_CONTROLLER_JOINT_NAMES,
    ARM_TRAJECTORY_ACTION,
    HOME_JOINT_VALUES,
)


class HomeBootstrap(Node):
    def __init__(self):
        super().__init__("home_bootstrap")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("controller_action_name", ARM_TRAJECTORY_ACTION)
        self.declare_parameter("home_tolerance", 0.03)
        self.declare_parameter("joint_state_wait_sec", 15.0)
        self.declare_parameter("controller_wait_sec", 15.0)
        self.declare_parameter("trajectory_duration_sec", 4.0)
        self.declare_parameter("result_wait_sec", 30.0)

        self._latest_joint_state = None
        self._joint_state_sub = self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._joint_state_callback,
            10,
        )
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("controller_action_name").value),
        )

    def _joint_state_callback(self, msg):
        self._latest_joint_state = msg

    def _wait_future_result(self, future, timeout_sec=30.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=float(timeout_sec))
        if not future.done():
            return None
        try:
            return future.result()
        except Exception:
            return None

    def _wait_for_joint_state(self, timeout_sec):
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            if self._latest_joint_state is not None:
                return self._latest_joint_state
            rclpy.spin_once(self, timeout_sec=0.1)
        return None

    @staticmethod
    def _positions_by_name(msg):
        return {name: position for name, position in zip(msg.name, msg.position)}

    def _home_state_errors(self, positions_by_name):
        errors = []
        missing = []
        for joint_name in ARM_CONTROLLER_JOINT_NAMES:
            if joint_name not in positions_by_name:
                missing.append(joint_name)
                continue
            current = float(positions_by_name[joint_name])
            target = float(HOME_JOINT_VALUES[joint_name])
            errors.append((joint_name, current, target, abs(current - target)))
        return missing, errors

    def _is_home_state(self, positions_by_name, tolerance):
        missing, errors = self._home_state_errors(positions_by_name)
        if missing:
            return False, missing, errors
        return all(error <= tolerance for _, _, _, error in errors), missing, errors

    def _build_home_goal(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = list(ARM_CONTROLLER_JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [
            float(HOME_JOINT_VALUES[joint_name])
            for joint_name in ARM_CONTROLLER_JOINT_NAMES
        ]
        duration = float(self.get_parameter("trajectory_duration_sec").value)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        trajectory.points.append(point)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        return goal

    def run(self):
        joint_state_wait_sec = float(self.get_parameter("joint_state_wait_sec").value)
        controller_wait_sec = float(self.get_parameter("controller_wait_sec").value)
        result_wait_sec = float(self.get_parameter("result_wait_sec").value)
        home_tolerance = float(self.get_parameter("home_tolerance").value)

        msg = self._wait_for_joint_state(joint_state_wait_sec)
        if msg is None:
            self.get_logger().warn(
                f"No /joint_states received within {joint_state_wait_sec:.1f}s; "
                "skipping home bootstrap."
            )
            return 0

        positions_by_name = self._positions_by_name(msg)
        is_home, missing, errors = self._is_home_state(
            positions_by_name, home_tolerance
        )
        if is_home:
            max_error = max((error for _, _, _, error in errors), default=0.0)
            self.get_logger().info(
                "Robot is already at HOME_JOINT_VALUES; "
                f"skipping home bootstrap. max_error={max_error:.4f}"
            )
            return 0
        if missing:
            self.get_logger().warn(
                "Cannot verify HOME_JOINT_VALUES for missing joints: "
                f"{', '.join(missing)}. Sending home trajectory."
            )

        self.get_logger().warn(
            "Sending arm to HOME_JOINT_VALUES before planner startup."
        )

        if not self._action_client.wait_for_server(timeout_sec=controller_wait_sec):
            self.get_logger().warn(
                "Arm FollowJointTrajectory action server is unavailable; "
                "skipping home bootstrap."
            )
            return 0

        future = self._action_client.send_goal_async(self._build_home_goal())
        goal_handle = self._wait_future_result(future, timeout_sec=5.0)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Home bootstrap trajectory goal was rejected.")
            return 0

        result = self._wait_future_result(
            goal_handle.get_result_async(), timeout_sec=result_wait_sec
        )
        if result is None:
            self.get_logger().warn("Home bootstrap trajectory returned no result.")
            return 0

        raw_error_code = result.result.error_code
        error_code = (
            raw_error_code.val if hasattr(raw_error_code, "val") else int(raw_error_code)
        )
        if error_code != 0:
            self.get_logger().warn(
                f"Home bootstrap trajectory failed with error code {error_code}: "
                f"{result.result.error_string}"
            )
            return 0

        self.get_logger().info("Home bootstrap trajectory completed successfully.")
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = HomeBootstrap()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
