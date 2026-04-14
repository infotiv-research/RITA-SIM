#!/usr/bin/env python3

from __future__ import annotations

import copy
import threading
import time

import rclpy
from moveit_msgs.action import HybridPlanner
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanResponse
from moveit_msgs.msg import MotionSequenceItem
from moveit_msgs.msg import MotionSequenceRequest
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class HybridMoveActionBridge(Node):
    """Bridge RViz MoveGroup action requests into MoveIt Hybrid Planning."""

    def __init__(self) -> None:
        super().__init__("hybrid_move_action_bridge")

        self.declare_parameter("move_action_name", "/move_action")
        self.declare_parameter("hybrid_action_name", "/run_hybrid_planning")
        self.declare_parameter("motion_plan_service_name", "/plan_kinematic_path")
        self.declare_parameter("global_solution_topic", "/global_trajectory")
        self.declare_parameter("default_planning_group", "ur_manipulator")
        self.declare_parameter("server_wait_timeout_sec", 5.0)

        self._move_action_name = self.get_parameter("move_action_name").get_parameter_value().string_value
        self._hybrid_action_name = self.get_parameter("hybrid_action_name").get_parameter_value().string_value
        self._motion_plan_service_name = (
            self.get_parameter("motion_plan_service_name").get_parameter_value().string_value
        )
        self._global_solution_topic = (
            self.get_parameter("global_solution_topic").get_parameter_value().string_value
        )
        self._default_planning_group = (
            self.get_parameter("default_planning_group").get_parameter_value().string_value
            or "ur_manipulator"
        )
        self._server_wait_timeout_sec = (
            self.get_parameter("server_wait_timeout_sec").get_parameter_value().double_value
        )

        self._cb_group = ReentrantCallbackGroup()
        self._hybrid_client = ActionClient(
            self,
            HybridPlanner,
            self._hybrid_action_name,
            callback_group=self._cb_group,
        )
        self._motion_plan_client = self.create_client(
            GetMotionPlan,
            self._motion_plan_service_name,
            callback_group=self._cb_group,
        )
        self._action_server = ActionServer(
            self,
            MoveGroup,
            self._move_action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )
        self._latest_global_solution: MotionPlanResponse | None = None
        self._latest_global_solution_time_ns = 0
        self._global_solution_lock = threading.Lock()
        self._global_solution_sub = self.create_subscription(
            MotionPlanResponse,
            self._global_solution_topic,
            self._on_global_solution,
            10,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f"Hybrid MoveGroup bridge listening on '{self._move_action_name}' "
            f"and forwarding execution to '{self._hybrid_action_name}'."
        )

    def _on_global_solution(self, msg: MotionPlanResponse) -> None:
        with self._global_solution_lock:
            self._latest_global_solution = copy.deepcopy(msg)
            self._latest_global_solution_time_ns = self.get_clock().now().nanoseconds

    def goal_callback(self, _goal_request: MoveGroup.Goal) -> GoalResponse:
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        request = goal_handle.request
        planning_group = request.request.group_name or self._default_planning_group
        start_time_ns = self.get_clock().now().nanoseconds

        if not planning_group:
            result = self._make_failure_result(
                MoveItErrorCodes.INVALID_GROUP_NAME,
                "MoveGroup request did not include a planning group.",
            )
            goal_handle.abort()
            return result

        if request.planning_options.plan_only:
            return self._execute_plan_only(goal_handle)

        return self._execute_hybrid(goal_handle, planning_group, start_time_ns)

    def _execute_plan_only(self, goal_handle):
        self._publish_feedback(goal_handle, "PLAN_ONLY_FALLBACK")
        if not self._motion_plan_client.wait_for_service(timeout_sec=self._server_wait_timeout_sec):
            result = self._make_failure_result(
                MoveItErrorCodes.COMMUNICATION_FAILURE,
                f"MoveIt planning service '{self._motion_plan_service_name}' is unavailable.",
            )
            goal_handle.abort()
            return result

        request = GetMotionPlan.Request()
        request.motion_plan_request = copy.deepcopy(goal_handle.request.request)
        future = self._motion_plan_client.call_async(request)
        self._wait_for_future(future, goal_handle)

        if goal_handle.is_cancel_requested:
            result = self._make_failure_result(
                MoveItErrorCodes.PREEMPTED,
                "Plan-only request canceled by client.",
            )
            goal_handle.canceled()
            return result

        if not future.done() or future.exception() is not None:
            result = self._make_failure_result(
                MoveItErrorCodes.COMMUNICATION_FAILURE,
                f"Plan-only request to '{self._motion_plan_service_name}' failed.",
            )
            goal_handle.abort()
            return result

        response = future.result()
        if response is None:
            result = self._make_failure_result(
                MoveItErrorCodes.FAILURE,
                "Plan-only request returned no response.",
            )
            goal_handle.abort()
            return result

        result = MoveGroup.Result()
        motion_plan_response = response.motion_plan_response
        result.error_code = motion_plan_response.error_code
        result.trajectory_start = motion_plan_response.trajectory_start
        result.planned_trajectory = motion_plan_response.trajectory
        result.planning_time = motion_plan_response.planning_time
        if motion_plan_response.error_code.val == MoveItErrorCodes.SUCCESS:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _execute_hybrid(self, goal_handle, planning_group: str, start_time_ns: int):
        self._publish_feedback(goal_handle, "HYBRID_PLANNING")
        if not self._hybrid_client.wait_for_server(timeout_sec=self._server_wait_timeout_sec):
            result = self._make_failure_result(
                MoveItErrorCodes.COMMUNICATION_FAILURE,
                f"Hybrid planner action '{self._hybrid_action_name}' is unavailable.",
            )
            goal_handle.abort()
            return result

        hybrid_goal = HybridPlanner.Goal()
        hybrid_goal.planning_group = planning_group
        sequence_item = MotionSequenceItem()
        sequence_item.req = copy.deepcopy(goal_handle.request.request)
        sequence_item.blend_radius = 0.0
        sequence_request = MotionSequenceRequest()
        sequence_request.items = [sequence_item]
        hybrid_goal.motion_sequence = sequence_request

        send_goal_future = self._hybrid_client.send_goal_async(
            hybrid_goal,
            feedback_callback=lambda feedback: self._on_hybrid_feedback(goal_handle, feedback),
        )
        self._wait_for_future(send_goal_future, goal_handle)

        if goal_handle.is_cancel_requested:
            result = self._make_failure_result(
                MoveItErrorCodes.PREEMPTED,
                "Hybrid planning request canceled before dispatch.",
            )
            goal_handle.canceled()
            return result

        if not send_goal_future.done() or send_goal_future.exception() is not None:
            result = self._make_failure_result(
                MoveItErrorCodes.COMMUNICATION_FAILURE,
                f"Failed to send goal to '{self._hybrid_action_name}'.",
            )
            goal_handle.abort()
            return result

        hybrid_goal_handle = send_goal_future.result()
        if hybrid_goal_handle is None or not hybrid_goal_handle.accepted:
            result = self._make_failure_result(
                MoveItErrorCodes.PLANNING_FAILED,
                "Hybrid planning manager rejected the MoveGroup request.",
            )
            goal_handle.abort()
            return result

        result_future = hybrid_goal_handle.get_result_async()
        cancel_future = None
        cancel_requested = False
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested and not cancel_requested:
                cancel_requested = True
                self._publish_feedback(goal_handle, "HYBRID_CANCELING")
                cancel_future = hybrid_goal_handle.cancel_goal_async()
            time.sleep(0.05)

        if cancel_future is not None:
            self._wait_for_future(cancel_future, goal_handle)

        if goal_handle.is_cancel_requested:
            result = self._make_failure_result(
                MoveItErrorCodes.PREEMPTED,
                "Hybrid planning request canceled by client.",
            )
            goal_handle.canceled()
            return result

        if not result_future.done() or result_future.exception() is not None:
            result = self._make_failure_result(
                MoveItErrorCodes.COMMUNICATION_FAILURE,
                f"Hybrid planner action '{self._hybrid_action_name}' failed to return a result.",
            )
            goal_handle.abort()
            return result

        wrapped_result = result_future.result()
        if wrapped_result is None or wrapped_result.result is None:
            result = self._make_failure_result(
                MoveItErrorCodes.FAILURE,
                "Hybrid planner returned an empty result.",
            )
            goal_handle.abort()
            return result

        hybrid_result = wrapped_result.result
        move_group_result = MoveGroup.Result()
        move_group_result.error_code = hybrid_result.error_code
        cached_solution = self._latest_global_solution_after(start_time_ns)
        if cached_solution is not None:
            move_group_result.trajectory_start = cached_solution.trajectory_start
            move_group_result.planned_trajectory = cached_solution.trajectory
            move_group_result.planning_time = cached_solution.planning_time
        else:
            move_group_result.trajectory_start = copy.deepcopy(goal_handle.request.request.start_state)
            move_group_result.planning_time = (self.get_clock().now().nanoseconds - start_time_ns) / 1e9

        if hybrid_result.error_code.val == MoveItErrorCodes.SUCCESS:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return move_group_result

    def _latest_global_solution_after(self, start_time_ns: int) -> MotionPlanResponse | None:
        with self._global_solution_lock:
            if self._latest_global_solution is None:
                return None
            if self._latest_global_solution_time_ns < start_time_ns:
                return None
            return copy.deepcopy(self._latest_global_solution)

    def _on_hybrid_feedback(self, goal_handle, feedback_msg) -> None:
        feedback_text = feedback_msg.feedback.feedback or "HYBRID_RUNNING"
        self._publish_feedback(goal_handle, feedback_text)

    def _make_failure_result(self, error_code_val: int, message: str) -> MoveGroup.Result:
        self.get_logger().warn(f"[hybrid_move_action_bridge] {message} (code={error_code_val})")
        result = MoveGroup.Result()
        result.error_code.val = error_code_val
        return result

    @staticmethod
    def _publish_feedback(goal_handle, state: str) -> None:
        feedback = MoveGroup.Feedback()
        feedback.state = state
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _wait_for_future(future, goal_handle) -> None:
        while rclpy.ok() and not future.done() and not goal_handle.is_cancel_requested:
            time.sleep(0.05)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HybridMoveActionBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
