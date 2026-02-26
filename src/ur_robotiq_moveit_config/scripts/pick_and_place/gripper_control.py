"""
Gripper action and finger-state helper mixin.

Responsibilities:
- Track latest finger joint samples from /joint_states.
- Execute gripper FollowJointTrajectory goals.
- Apply blocked-close heuristics to detect likely successful grasps.
"""

import time

from control_msgs.action import FollowJointTrajectory
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .constants import GRIPPER_JOINT


class GripperControlMixin:
    @staticmethod
    def _wait_future_result(future, timeout_sec=30.0, poll_period_sec=0.01):
        """Wait for an async future without spinning a second executor."""
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            if future.done():
                try:
                    return future.result()
                except Exception:
                    return None
            time.sleep(float(poll_period_sec))
        return None

    def _on_joint_states(self, msg):
        """Track latest finger joint state for grasp-close fallback logic."""
        try:
            idx = msg.name.index(GRIPPER_JOINT)
        except ValueError:
            return
        if idx >= len(msg.position):
            return

        now = time.monotonic()
        finger_pos = float(msg.position[idx])
        self._last_finger_joint_position = finger_pos
        self._last_finger_joint_msg_time = now
        self._finger_joint_history.append((now, finger_pos))

    def _wait_for_finger_joint_state(self, timeout_sec=1.0):
        """Wait briefly for a recent finger_joint sample and return its position."""
        end_time = time.monotonic() + float(timeout_sec)
        while time.monotonic() < end_time:
            pos = self._last_finger_joint_position
            age = time.monotonic() - self._last_finger_joint_msg_time
            if pos is not None and age <= 0.5:
                return float(pos)
            time.sleep(0.05)
        return self._last_finger_joint_position

    def _finger_motion_last_window(self, window_sec=0.5):
        """Return absolute finger motion over the recent time window."""
        if len(self._finger_joint_history) < 2:
            return None

        t_min = time.monotonic() - float(window_sec)
        samples = [pos for ts, pos in self._finger_joint_history if ts >= t_min]
        if len(samples) < 2:
            return None
        return abs(float(samples[-1]) - float(samples[0]))

    def _is_grasp_likely_from_positions(self, start_position, end_position):
        """Heuristic for blocked-close success based on finger-joint motion."""
        if end_position is None:
            return False

        min_pos = float(self.close_success_min_position)
        # `close_success_min_delta` now represents maximum motion (rad) in the
        # last 0.5s to consider the finger "stalled" against the object.
        max_recent_motion = float(self.close_success_min_delta)
        recent_motion = self._finger_motion_last_window(window_sec=0.5)
        closed_enough = float(end_position) >= min_pos
        stalled_recently = (
            recent_motion is not None and float(recent_motion) <= max_recent_motion
        )
        return closed_enough and stalled_recently

    def _close_motion_reached_grasp(self, start_position):
        """Heuristic: treat blocked close as success if finger moved/closed enough."""
        end_position = self._wait_for_finger_joint_state(timeout_sec=0.5)
        if not self._is_grasp_likely_from_positions(start_position, end_position):
            return False
        recent_motion = self._finger_motion_last_window(window_sec=0.5)
        self.get_logger().warn(
            "Gripper close action did not finish cleanly, but finger_joint indicates "
            "grasp likely succeeded "
            f"(start={start_position}, end={float(end_position):.3f}, "
            f"recent_motion_0.5s={recent_motion})."
        )
        return True

    def _move_gripper(self, position, allow_stall_success=False):
        """Move gripper through ros2_control trajectory action.

        If allow_stall_success=True and this is a closing command, consider the
        command successful when the finger joint clearly moved/closed even when
        the action server does not return a terminal result.
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = [GRIPPER_JOINT]
        start_position = self._wait_for_finger_joint_state(timeout_sec=0.5)
        is_close_command = (
            start_position is None or float(position) >= float(start_position) + 1e-4
        )

        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = Duration(seconds=1.5).to_msg()
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(
            f"Sending gripper trajectory goal ({GRIPPER_JOINT}={position})..."
        )
        future = self.gripper_traj_client.send_goal_async(goal)
        goal_handle = self._wait_future_result(future, timeout_sec=30.0)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Gripper trajectory goal was rejected!")
            return False

        result_future = goal_handle.get_result_async()

        if allow_stall_success and is_close_command:
            early_deadline = time.monotonic() + float(self.close_early_success_wait_s)
            while time.monotonic() < early_deadline:
                if result_future.done():
                    break
                end_position = self._last_finger_joint_position
                if self._is_grasp_likely_from_positions(start_position, end_position):
                    self.get_logger().warn(
                        "Gripper close appears successful from finger_joint feedback; "
                        "continuing without waiting for full action timeout."
                    )
                    cancel_future = goal_handle.cancel_goal_async()
                    self._wait_future_result(cancel_future, timeout_sec=0.5)
                    return True
                time.sleep(0.05)

        result = self._wait_future_result(
            result_future, timeout_sec=float(self.gripper_result_timeout_s)
        )
        if result is None:
            if allow_stall_success and is_close_command and self._close_motion_reached_grasp(
                start_position
            ):
                return True
            self.get_logger().error("Gripper trajectory action returned no result!")
            return False

        raw_error_code = result.result.error_code
        error_code = (
            raw_error_code.val if hasattr(raw_error_code, "val") else int(raw_error_code)
        )
        if error_code == 0:
            return True

        if allow_stall_success and is_close_command and self._close_motion_reached_grasp(
            start_position
        ):
            return True

        self.get_logger().error(
            "Gripper trajectory action failed with error code: "
            f"{error_code}, message: {result.result.error_string}"
        )
        return False
