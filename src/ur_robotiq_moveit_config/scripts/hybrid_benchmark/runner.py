"""Coordinate MoveIt goals, obstacle spawning, telemetry, and run summaries."""

from __future__ import annotations

import json
import math
import threading
import time
from copy import deepcopy

from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanResponse
from moveit_msgs.srv import GetPositionFK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from curobo_msgs.srv import AddObject, RemoveObject

from .constants import (
    BENCHMARK_OBSTACLE,
    DEFAULT_ACTION_TIMEOUT_SEC,
    DEFAULT_INITIAL_PLAN_TIMEOUT_SEC,
    DEFAULT_POST_RUN_IDLE_SEC,
    DEFAULT_SETTLE_TIME_SEC,
    DEFAULT_SPAWN_TRIGGER_CLEARANCE_M,
    DEFAULT_SPAWN_TRIGGER_POLL_PERIOD_SEC,
    END_EFFECTOR_LINK,
    MARKER_TOPIC,
    MOVEIT_FK_SERVICE_NAME,
    WORLD_FRAME,
    quaternion_from_euler_deg,
)
from .move_group_goals import resolve_benchmark_case
from .markers import ObstacleMarkerManager
from .placement import (
    ObstaclePlacementResolver,
    PlacementError,
    point_clearance_to_obstacle,
    sphere_clearance_to_obstacle,
)
from .recording import (
    append_global_solution,
    append_joint_state,
    accumulate_hybrid_mpc_telemetry,
    create_run_record,
    extract_final_joint_target,
    extract_joint_positions,
    is_successful_move_group_result,
    summarize_run,
)


# Create a normalized failure record for MoveGroup action attempts.
def _move_group_failure(accepted: bool, timed_out: bool, status=GoalStatus.STATUS_ABORTED):
    return {"accepted": accepted, "timed_out": timed_out, "status": status, "error_code": None}


class HybridObstacleBenchmark(Node):
    # Set up ROS clients, subscriptions, marker publishing, and shared run state.
    def __init__(self) -> None:
        super().__init__("hybrid_obstacle_benchmark")

        self._cb_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._active_run = None
        self._global_plan_ready_event = threading.Event()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.move_group_client = ActionClient(
            self, MoveGroup, "/move_action", callback_group=self._cb_group
        )
        self.add_object_client = self.create_client(
            AddObject, "/unified_planner/add_object", callback_group=self._cb_group
        )
        self.remove_object_client = self.create_client(
            RemoveObject, "/unified_planner/remove_object", callback_group=self._cb_group
        )
        self.moveit_fk_client = self.create_client(
            GetPositionFK, MOVEIT_FK_SERVICE_NAME, callback_group=self._cb_group
        )

        marker_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._markers = ObstacleMarkerManager(
            self, self.create_publisher(MarkerArray, MARKER_TOPIC, marker_qos)
        )
        self._placement = ObstaclePlacementResolver(self.moveit_fk_client, self._wait_for_future)

        for msg_type, topic, callback, depth in (
            (MotionPlanResponse, "/global_trajectory", self._on_global_solution, 10),
            (JointState, "/moveit_joint_states", self._on_joint_state, 20),
            (MarkerArray, "/curobo_live_collision_spheres", self._on_collision_spheres, 10),
            (String, "/unified_planner/hybrid_mpc_telemetry", self._on_hybrid_mpc_telemetry, 50),
        ):
            self.create_subscription(msg_type, topic, callback, depth, callback_group=self._cb_group)

    # Convert monotonic time into seconds relative to the active run start.
    @staticmethod
    def _relative_time(run_data: dict, now_monotonic: float) -> float:
        return float(now_monotonic - float(run_data["start_monotonic"]))

    # Wait until the action and obstacle services required by the benchmark are available.
    def wait_until_ready(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + float(timeout_sec)
        if not self.move_group_client.wait_for_server(timeout_sec=max(deadline - time.monotonic(), 0.0)):
            self.get_logger().error("Timed out waiting for MoveGroup action server.")
            return False
        if not self.add_object_client.wait_for_service(timeout_sec=max(deadline - time.monotonic(), 0.0)):
            self.get_logger().error("Timed out waiting for add_object service.")
            return False
        if not self.remove_object_client.wait_for_service(timeout_sec=max(deadline - time.monotonic(), 0.0)):
            self.get_logger().error("Timed out waiting for remove_object service.")
            return False
        return True

    # Wait for a ROS future with a timeout and return None on timeout.
    def _wait_for_future(self, future, timeout_sec: float):
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            if future.done():
                try:
                    return future.result()
                except Exception as exc:
                    return exc
            time.sleep(0.005)
        return None

    # Start recording data for one benchmark run.
    def _start_run_record(self, run_index: int, goal_spec: dict):
        with self._lock:
            self._global_plan_ready_event.clear()
            self._active_run = create_run_record(run_index, goal_spec)
            self._active_run["start_monotonic"] = time.monotonic()

    # Stop recording the active run and return its collected data.
    def _finish_run_record(self):
        with self._lock:
            finished = self._active_run
            self._active_run = None
            self._global_plan_ready_event.clear()
        return finished

    # Update the obstacle section of the active run record.
    def _update_obstacle_state(self, **fields) -> None:
        with self._lock:
            if self._active_run is not None:
                self._active_run["obstacle"].update(fields)

    # Return the active planned obstacle when sampling is enabled.
    def _distance_sampling_obstacle(self):
        with self._lock:
            if self._active_run is None:
                return None
            obstacle_state = self._active_run.get("obstacle") or {}
            if not bool(obstacle_state.get("distance_sampling_active", True)):
                return None
            planned = obstacle_state.get("planned_obstacle")
            if not planned:
                return None
            return deepcopy(planned)

    # Store a new obstacle clearance minimum if it improves on the current one.
    def _update_clearance_minimum(self, value_key: str, clearance_m: float):
        with self._lock:
            if self._active_run is not None:
                obstacle = self._active_run["obstacle"]
                current = obstacle.get(value_key)
                clearance = float(clearance_m)
                if current is None or clearance < float(current):
                    obstacle[value_key] = clearance

    # Remove the benchmark obstacle from the planner and RViz.
    def _clear_obstacle(self, name: str) -> None:
        self._call_remove_obstacle(name, timeout_sec=5.0)
        self._markers.hide_obstacle_marker(name)

    # Wait for the first global trajectory that can guide obstacle placement.
    def _wait_for_initial_global_plan(self, timeout_sec: float, stop_event: threading.Event):
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline and not stop_event.is_set():
            if self._global_plan_ready_event.wait(timeout=0.1):
                with self._lock:
                    plan = self._active_run and self._active_run.get("initial_global_plan_reference")
                return deepcopy(plan) if plan is not None else None
        return None

    # Record global plan messages and expose the first plan to the spawn thread.
    def _on_global_solution(self, msg: MotionPlanResponse):
        now = time.monotonic()
        with self._lock:
            if self._active_run is None:
                return
            t_rel_sec = self._relative_time(self._active_run, now)
            reference_plan = append_global_solution(self._active_run, msg, t_rel_sec)
            if reference_plan is not None and self._active_run.get("initial_global_plan_reference") is None:
                self._active_run["initial_global_plan_reference"] = reference_plan
                self._global_plan_ready_event.set()

    # Record the latest joint state while a benchmark run is active.
    def _on_joint_state(self, msg: JointState):
        positions = extract_joint_positions(msg)
        if positions is None:
            return
        obstacle = self._distance_sampling_obstacle()
        with self._lock:
            if self._active_run is not None:
                append_joint_state(self._active_run, positions)
        if obstacle is None:
            return
        tcp_position = self._point_in_world(END_EFFECTOR_LINK, [0.0, 0.0, 0.0])
        if tcp_position is not None:
            self._update_clearance_minimum(
                "min_tcp_clearance_m",
                point_clearance_to_obstacle(tcp_position, obstacle),
            )

    # Rotate a vector by a normalized quaternion.
    @staticmethod
    def _rotate_vector_by_quaternion(vector_xyz, quaternion_xyzw):
        x, y, z = (float(value) for value in vector_xyz)
        qx, qy, qz, qw = (float(value) for value in quaternion_xyzw)
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 0.0:
            return [x, y, z]
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm
        tx = 2.0 * (qy * z - qz * y)
        ty = 2.0 * (qz * x - qx * z)
        tz = 2.0 * (qx * y - qy * x)
        return [
            x + qw * tx + (qy * tz - qz * ty),
            y + qw * ty + (qz * tx - qx * tz),
            z + qw * tz + (qx * ty - qy * tx),
        ]

    # Transform a point into the benchmark world frame.
    def _point_in_world(self, frame_id: str, point_xyz):
        frame_id = str(frame_id or WORLD_FRAME).strip()
        if frame_id.startswith("/"):
            frame_id = frame_id[1:]
        point = [float(value) for value in point_xyz]
        if frame_id == WORLD_FRAME:
            return point
        try:
            transform = self._tf_buffer.lookup_transform(WORLD_FRAME, frame_id, Time())
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rotated = self._rotate_vector_by_quaternion(
            point,
            [rotation.x, rotation.y, rotation.z, rotation.w],
        )
        return [
            float(translation.x) + rotated[0],
            float(translation.y) + rotated[1],
            float(translation.z) + rotated[2],
        ]

    # Transform a marker position into the benchmark world frame.
    def _marker_position_in_world(self, marker):
        return self._point_in_world(
            getattr(marker.header, "frame_id", "") or WORLD_FRAME,
            [
                float(marker.pose.position.x),
                float(marker.pose.position.y),
                float(marker.pose.position.z),
            ],
        )

    # Record the minimum robot collision-sphere clearance for live cuRobo markers.
    def _on_collision_spheres(self, msg: MarkerArray):
        obstacle = self._distance_sampling_obstacle()
        if obstacle is None:
            return

        min_clearance = None
        for marker in msg.markers:
            if int(marker.action) != Marker.ADD or int(marker.type) != Marker.SPHERE:
                continue
            radius = max(
                float(marker.scale.x),
                float(marker.scale.y),
                float(marker.scale.z),
            ) * 0.5
            if radius <= 0.0:
                continue
            center = self._marker_position_in_world(marker)
            if center is None:
                continue
            clearance = sphere_clearance_to_obstacle(center, radius, obstacle)
            if min_clearance is None or clearance < min_clearance:
                min_clearance = float(clearance)

        if min_clearance is not None:
            self._update_clearance_minimum(
                "min_robot_clearance_m",
                min_clearance,
            )

    # Record hybrid MPC telemetry such as path invalidation events.
    def _on_hybrid_mpc_telemetry(self, msg: String):
        payload = json.loads(msg.data)
        now = time.monotonic()
        with self._lock:
            if self._active_run is not None:
                accumulate_hybrid_mpc_telemetry(
                    self._active_run,
                    payload,
                    self._relative_time(self._active_run, now),
                )

    # Request removal of an obstacle from the unified planner.
    def _call_remove_obstacle(self, name: str, timeout_sec: float):
        request = RemoveObject.Request()
        request.name = str(name)
        result = self._wait_for_future(self.remove_object_client.call_async(request), timeout_sec)
        if isinstance(result, Exception) or result is None:
            return False, "remove_object request timed out or failed"
        return bool(result.success), str(result.message)

    # Request insertion of a cuboid obstacle into the unified planner.
    def _call_add_obstacle(self, obstacle: dict, timeout_sec: float):
        request = AddObject.Request()
        request.type = AddObject.Request.CUBOID
        request.name = str(obstacle["name"])
        request.pose.position.x, request.pose.position.y, request.pose.position.z = (
            float(v) for v in obstacle["position"]
        )
        (
            request.pose.orientation.x,
            request.pose.orientation.y,
            request.pose.orientation.z,
            request.pose.orientation.w,
        ) = quaternion_from_euler_deg(obstacle["rotation_deg"])
        request.dimensions.x, request.dimensions.y, request.dimensions.z = (
            float(v) for v in obstacle["size"]
        )
        request.color.r, request.color.g, request.color.b, request.color.a = (
            float(v) for v in obstacle["color"]
        )
        result = self._wait_for_future(self.add_object_client.call_async(request), timeout_sec)
        if isinstance(result, Exception) or result is None:
            return False, "add_object request timed out or failed"
        return bool(result.success), str(result.message)

    # Measure the current TCP distance to the candidate obstacle surface.
    def _measure_current_tcp_clearance(self, obstacle: dict, timeout_sec: float):
        with self._lock:
            joint_positions = (
                list(self._active_run["joint_state_summary"].get("last_positions") or [])
                if self._active_run is not None
                else []
            )
        if not joint_positions:
            return None, None
        clearance = self._placement.measure_tcp_clearance_to_obstacle(
            joint_positions, obstacle, timeout_sec=timeout_sec
        )
        if clearance is None:
            return None, None
        with self._lock:
            if self._active_run is None:
                return None, None
            t_rel_sec = self._relative_time(self._active_run, time.monotonic())
            obstacle_state = self._active_run.get("obstacle") or {}
            if bool(obstacle_state.get("distance_sampling_active", True)):
                current = obstacle_state.get("min_tcp_clearance_m")
                if current is None or float(clearance) < float(current):
                    obstacle_state["min_tcp_clearance_m"] = float(clearance)
            return t_rel_sec, float(clearance)

    # Spawn the benchmark obstacle once the TCP reaches the configured clearance trigger.
    def _spawn_obstacle_on_clearance_trigger(self, run_index: int, stop_event: threading.Event):
        reference_plan = self._wait_for_initial_global_plan(
            timeout_sec=DEFAULT_INITIAL_PLAN_TIMEOUT_SEC,
            stop_event=stop_event,
        )
        try:
            obstacle, metadata = self._placement.resolve_obstacle_placement(reference_plan)
        except PlacementError as exc:
            self._update_obstacle_state(wait_reason="placement_failed", service_message=str(exc))
            print(f"[run {run_index}] obstacle placement failed: {exc}")
            return

        trajectory_selection = metadata.get("trajectory_selection") or {}
        target_time = trajectory_selection.get("selected_time_from_start_sec")
        self._update_obstacle_state(
            planned_obstacle={
                "name": str(obstacle["name"]),
                "position": [float(v) for v in obstacle["position"]],
                "size": [float(v) for v in obstacle["size"]],
                "rotation_deg": [float(v) for v in obstacle["rotation_deg"]],
            },
        )
        position = obstacle["position"]
        print(
            f"[run {run_index}] trajectory-aware obstacle target "
            f"point={trajectory_selection.get('selected_point_index')} "
            f"t={target_time if target_time is not None else 'n/a'}s "
            f"spawn=({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})"
        )

        trigger_clearance = None
        while not stop_event.is_set():
            _, trigger_clearance = self._measure_current_tcp_clearance(obstacle, timeout_sec=0.25)
            if trigger_clearance is not None and trigger_clearance <= DEFAULT_SPAWN_TRIGGER_CLEARANCE_M:
                break
            stop_event.wait(DEFAULT_SPAWN_TRIGGER_POLL_PERIOD_SEC)
        else:
            self._update_obstacle_state(wait_reason="spawn_trigger_not_reached")
            print(f"[run {run_index}] obstacle spawn trigger not reached before run ended")
            return

        success, message = self._call_add_obstacle(obstacle, timeout_sec=10.0)
        with self._lock:
            spawn_t = (
                self._relative_time(self._active_run, time.monotonic())
                if self._active_run is not None
                else None
            )
        updates = {
            "service_message": str(message),
            "spawned": bool(success),
        }
        if success:
            updates["spawn_t_rel_sec"] = spawn_t
            _, clearance = self._measure_current_tcp_clearance(obstacle, timeout_sec=0.5)
            updates["spawn_tcp_to_obstacle_surface_distance_m"] = clearance
        self._update_obstacle_state(**updates)

        if success:
            self._markers.show_obstacle_marker(obstacle)
            print(f"[run {run_index}] obstacle spawned on {MARKER_TOPIC} (planner + RViz marker)")
        else:
            self._markers.hide_obstacle_marker(str(obstacle["name"]))
            print(f"[run {run_index}] obstacle spawn failed: {message}")

    # Send a MoveGroup goal and normalize the action result for recording.
    def _send_move_group_goal(self, goal):
        started_at = time.monotonic()
        goal_handle = self._wait_for_future(self.move_group_client.send_goal_async(goal), timeout_sec=10.0)
        if not goal_handle or isinstance(goal_handle, Exception):
            return _move_group_failure(accepted=False, timed_out=False, status=None)
        if not goal_handle.accepted:
            return _move_group_failure(accepted=False, timed_out=False)

        wrapped = self._wait_for_future(goal_handle.get_result_async(), timeout_sec=DEFAULT_ACTION_TIMEOUT_SEC)
        if wrapped is None:
            goal_handle.cancel_goal_async()
            return _move_group_failure(accepted=True, timed_out=True)
        if isinstance(wrapped, Exception):
            return _move_group_failure(accepted=True, timed_out=False)

        result = wrapped.result
        error_code = getattr(getattr(result, "error_code", None), "val", None)
        return {
            "accepted": True,
            "timed_out": False,
            "status": int(wrapped.status),
            "error_code": int(error_code) if error_code is not None else None,
            "elapsed_sec": time.monotonic() - started_at,
            "final_joint_target": extract_final_joint_target(getattr(result, "planned_trajectory", None)),
        }

    # Run one benchmark iteration from precondition motion through cleanup and summary.
    def run_single_benchmark(self, args, run_index: int):
        case_spec, start_goal, benchmark_goal = resolve_benchmark_case(args.case)

        obstacle_name = str(BENCHMARK_OBSTACLE["name"])
        self._clear_obstacle(obstacle_name)

        print(f"[run {run_index}] moving to benchmark start state for case {case_spec['case_name']}...")
        precondition = self._send_move_group_goal(start_goal)
        if not is_successful_move_group_result(precondition):
            self._clear_obstacle(obstacle_name)
            run_data = create_run_record(run_index, case_spec)
            run_data["precondition_start_state"] = precondition
            run_data["obstacle"]["wait_reason"] = "precondition_failed"
            run_data["obstacle"]["service_message"] = "Run aborted because the start-state precondition failed."
            return summarize_run(run_data, None)
        time.sleep(DEFAULT_SETTLE_TIME_SEC)

        print(
            f"[run {run_index}] sending benchmark goal "
            f"(mode={case_spec['mode']}, case={case_spec['case_name']})..."
        )
        self._start_run_record(run_index, case_spec)
        stop_event = threading.Event()
        spawn_thread = threading.Thread(
            target=self._spawn_obstacle_on_clearance_trigger,
            args=(run_index, stop_event),
            daemon=True,
        )
        spawn_thread.start()

        move_group_result = self._send_move_group_goal(benchmark_goal)
        self._update_obstacle_state(distance_sampling_active=False)

        stop_event.set()
        spawn_thread.join(timeout=12.0)

        time.sleep(DEFAULT_POST_RUN_IDLE_SEC)
        self._clear_obstacle(obstacle_name)

        finished = self._finish_run_record()
        finished["precondition_start_state"] = precondition
        return summarize_run(finished, move_group_result)
