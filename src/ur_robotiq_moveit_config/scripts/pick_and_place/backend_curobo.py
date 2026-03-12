"""curobo_ros backend adapter for pick-and-place arm motion."""

import math
import time

from geometry_msgs.msg import Pose
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

try:
    from rclpy.parameter_client import AsyncParameterClient
except ImportError:
    AsyncParameterClient = None

try:
    from curobo_msgs.srv import GetPlanners, SetPlanner, TrajectoryGeneration
except ImportError:
    GetPlanners = None
    SetPlanner = None
    TrajectoryGeneration = None

from .backend_interface import MotionBackendInterface
from .constants import END_EFFECTOR_LINK, HOME_JOINT_VALUES, JOINT_NAMES
from .controller_execution import execute_arm_joint_trajectory


class CuroboMotionBackend(MotionBackendInterface):
    name = "curobo_ros"
    _continuous_joint_limits = {
        "shoulder_pan_joint": (-2.0 * math.pi, 2.0 * math.pi),
        "wrist_1_joint": (-2.0 * math.pi, 2.0 * math.pi),
        "wrist_2_joint": (-2.0 * math.pi, 2.0 * math.pi),
        "wrist_3_joint": (-2.0 * math.pi, 2.0 * math.pi),
    }
    _periodic_home_joints = {
        "shoulder_pan_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    }

    _planner_type_map = {
        "classic": getattr(SetPlanner.Request, "CLASSIC", 0) if SetPlanner else 0,
        "mpc": getattr(SetPlanner.Request, "MPC", 1) if SetPlanner else 1,
        "multipoint": getattr(SetPlanner.Request, "MULTIPOINT", 4) if SetPlanner else 4,
        "joint_space": getattr(SetPlanner.Request, "JOINT_SPACE", 5) if SetPlanner else 5,
    }

    def __init__(self, node):
        super().__init__(node)
        if TrajectoryGeneration is None or SetPlanner is None:
            raise RuntimeError("curobo_msgs is not available in this environment.")

        self.trajectory_client = node.create_client(
            TrajectoryGeneration, "/unified_planner/generate_trajectory"
        )
        self.set_planner_client = node.create_client(SetPlanner, "/unified_planner/set_planner")
        self.get_planners_client = (
            node.create_client(GetPlanners, "/unified_planner/get_planners")
            if GetPlanners is not None
            else None
        )
        self.parameter_client = (
            AsyncParameterClient(node, "/unified_planner")
            if AsyncParameterClient is not None
            else None
        )
        self._active_planner_type = None
        self._active_planner_id = None
        self._original_planner_type = None
        self._original_planner_id = None

    def _wait_future_result(self, future, timeout_sec=10.0, poll_period_sec=0.01):
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            if future.done():
                try:
                    return future.result()
                except Exception:
                    return None
            time.sleep(float(poll_period_sec))
        if not future.done():
            return None
        try:
            return future.result()
        except Exception:
            return None

    def wait_until_ready(self):
        self.node.get_logger().info("Waiting for curobo trajectory service...")
        if not self.trajectory_client.wait_for_service(timeout_sec=30.0):
            self.node.get_logger().error(
                "curobo trajectory service unavailable after 30s: /unified_planner/generate_trajectory"
            )
            return False
        if not self.set_planner_client.wait_for_service(timeout_sec=30.0):
            self.node.get_logger().error(
                "curobo set_planner service unavailable after 30s: /unified_planner/set_planner"
            )
            return False
        if not self.node.arm_traj_client.wait_for_server(timeout_sec=30.0):
            self.node.get_logger().error(
                "Arm FollowJointTrajectory action server unavailable after 30s: "
                "/joint_trajectory_controller/follow_joint_trajectory"
            )
            return False
        self._capture_original_planner_state()
        return True

    def _capture_original_planner_state(self):
        if self._original_planner_id is not None:
            return True
        if self.get_planners_client is None:
            self.node.get_logger().warn(
                "curobo get_planners service unavailable; pick-and-place will not restore the previous planner."
            )
            return False
        if not self.get_planners_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn(
                "curobo get_planners service unavailable; pick-and-place will not restore the previous planner."
            )
            return False
        future = self.get_planners_client.call_async(GetPlanners.Request())
        response = self._wait_future_result(future, timeout_sec=5.0)
        if response is None or not bool(response.success):
            self.node.get_logger().warn(
                "Failed to query curobo active planner; pick-and-place will not restore the previous planner."
            )
            return False

        self._original_planner_id = int(response.current_planner_id)
        self._active_planner_id = self._original_planner_id
        self._original_planner_type = self._planner_key_for_id(self._original_planner_id)
        self._active_planner_type = self._original_planner_type
        return True

    def _planner_key_for_id(self, planner_id):
        for planner_name, mapped_id in self._planner_type_map.items():
            if int(mapped_id) == int(planner_id):
                return planner_name
        return None

    def _ensure_planner_type(self, requested=None):
        if requested is None:
            requested = str(getattr(self.node, "curobo_planner_type", "classic")).strip().lower()
        else:
            requested = str(requested).strip().lower()
        planner_type = self._planner_type_map.get(requested)
        if planner_type is None:
            self.node.get_logger().warn(
                f"Unknown curobo planner type '{requested}', using classic."
            )
            planner_type = self._planner_type_map["classic"]
            requested = "classic"

        if self._active_planner_type == requested:
            return True

        request = SetPlanner.Request()
        request.planner_type = int(planner_type)
        future = self.set_planner_client.call_async(request)
        response = self._wait_future_result(future, timeout_sec=5.0)
        if response is None or not response.success:
            self.node.get_logger().error(
                f"Failed to switch curobo planner to '{requested}'."
            )
            return False
        self._active_planner_type = requested
        self._active_planner_id = int(planner_type)
        return True

    def restore_previous_planner(self):
        if self._original_planner_id is None:
            return True
        if self._active_planner_id == self._original_planner_id:
            return True
        if not self.set_planner_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn(
                "curobo set_planner service unavailable; cannot restore the previous planner."
            )
            return False

        request = SetPlanner.Request()
        request.planner_type = int(self._original_planner_id)
        future = self.set_planner_client.call_async(request)
        response = self._wait_future_result(future, timeout_sec=5.0)
        if response is None or not response.success:
            self.node.get_logger().warn(
                "Failed to restore the previous curobo planner after pick-and-place."
            )
            return False

        self._active_planner_id = self._original_planner_id
        self._active_planner_type = self._original_planner_type
        if self._original_planner_type is not None:
            self.node.get_logger().info(
                f"Restored curobo planner to '{self._original_planner_type}'."
            )
        else:
            self.node.get_logger().info(
                f"Restored curobo planner to id={int(self._original_planner_id)}."
            )
        return True

    def _apply_runtime_parameters(self, planning_time=None, num_attempts=None):
        if self.parameter_client is None:
            return False
        if not self.parameter_client.wait_for_service(timeout_sec=2.0):
            return False
        parameters = []
        if planning_time is not None:
            parameters.append(Parameter("timeout", Parameter.Type.DOUBLE, float(planning_time)))
        if num_attempts is not None:
            parameters.append(Parameter("max_attempts", Parameter.Type.INTEGER, int(num_attempts)))
        if not parameters:
            return True
        future = self.parameter_client.set_parameters(parameters)
        response = self._wait_future_result(future, timeout_sec=5.0)
        return response is not None and all(result.successful for result in response)

    @staticmethod
    def _nearest_equivalent_angle(target_rad, current_rad):
        target = float(target_rad)
        current = float(current_rad)
        best = target
        best_distance = abs(target - current)
        for wrap in (-2.0 * math.pi, 2.0 * math.pi):
            candidate = target + wrap
            distance = abs(candidate - current)
            if distance < best_distance:
                best = candidate
                best_distance = distance
        return best

    def _build_home_joint_target(self):
        target = {joint_name: float(HOME_JOINT_VALUES[joint_name]) for joint_name in JOINT_NAMES}
        latest = self.node._wait_for_recent_joint_positions(timeout_sec=1.0, max_age_sec=0.5)
        if latest is None:
            return target

        for joint_name in self._periodic_home_joints:
            if joint_name not in latest or joint_name not in target:
                continue
            target[joint_name] = self._nearest_equivalent_angle(
                target[joint_name],
                latest[joint_name],
            )
        return target

    def _equivalent_joint_candidates(self, joint_name, target_rad, current_rad):
        lower, upper = self._continuous_joint_limits.get(
            joint_name,
            (-2.0 * math.pi, 2.0 * math.pi),
        )
        candidates = []
        for wrap_multiplier in (0, -1, 1):
            candidate = float(target_rad) + wrap_multiplier * (2.0 * math.pi)
            if candidate < lower - 1e-6 or candidate > upper + 1e-6:
                continue
            if any(abs(existing - candidate) < 1e-6 for existing in candidates):
                continue
            candidates.append(candidate)
        candidates.sort(key=lambda value: abs(value - float(current_rad)))
        return candidates or [float(target_rad)]

    def _build_home_joint_target_candidates(self):
        base_target = {joint_name: float(HOME_JOINT_VALUES[joint_name]) for joint_name in JOINT_NAMES}
        latest = self.node._wait_for_recent_joint_positions(timeout_sec=1.0, max_age_sec=0.5)
        if latest is None:
            return [base_target]

        per_joint_candidates = {}
        for joint_name in self._periodic_home_joints:
            if joint_name not in latest or joint_name not in base_target:
                continue
            per_joint_candidates[joint_name] = self._equivalent_joint_candidates(
                joint_name,
                base_target[joint_name],
                latest[joint_name],
            )

        ranked_candidates = []
        seen_signatures = set()

        def add_candidate(candidate):
            signature = tuple(round(float(candidate[joint_name]), 6) for joint_name in JOINT_NAMES)
            if signature in seen_signatures:
                return
            seen_signatures.add(signature)
            distance = 0.0
            for joint_name in JOINT_NAMES:
                current_value = float(latest.get(joint_name, candidate[joint_name]))
                distance += abs(float(candidate[joint_name]) - current_value)
            ranked_candidates.append((distance, candidate))

        nearest_target = dict(base_target)
        for joint_name, candidates in per_joint_candidates.items():
            nearest_target[joint_name] = float(candidates[0])
        add_candidate(nearest_target)
        add_candidate(dict(base_target))

        joint_names = sorted(per_joint_candidates.keys())
        for joint_name in joint_names:
            for candidate_value in per_joint_candidates[joint_name][1:]:
                candidate = dict(nearest_target)
                candidate[joint_name] = float(candidate_value)
                add_candidate(candidate)

        if joint_names:
            combo_limit = 12
            generated = 0
            first_joint = joint_names[0]
            remaining = joint_names[1:]
            for first_value in per_joint_candidates[first_joint]:
                partials = [({first_joint: float(first_value)})]
                for joint_name in remaining:
                    next_partials = []
                    for partial in partials:
                        for candidate_value in per_joint_candidates[joint_name]:
                            updated = dict(partial)
                            updated[joint_name] = float(candidate_value)
                            next_partials.append(updated)
                    partials = next_partials
                for partial in partials:
                    candidate = dict(base_target)
                    candidate.update(partial)
                    add_candidate(candidate)
                    generated += 1
                    if generated >= combo_limit:
                        break
                if generated >= combo_limit:
                    break

        ranked_candidates.sort(key=lambda item: item[0])
        return [candidate for _, candidate in ranked_candidates]

    def _current_joint_state(self):
        latest = self.node._wait_for_recent_joint_positions(timeout_sec=1.0, max_age_sec=0.5)
        if latest is None:
            self.node.get_logger().warn(
                "No recent /joint_states available, falling back to HOME_JOINT_VALUES for curobo start state."
            )
            latest = HOME_JOINT_VALUES
        msg = JointState()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(latest[joint_name]) for joint_name in JOINT_NAMES]
        return msg

    def _plan_pose(
        self,
        x,
        y,
        z,
        qx,
        qy,
        qz,
        qw,
        planning_time=None,
        num_attempts=None,
        planner_type=None,
    ):
        if not self._ensure_planner_type(requested=planner_type):
            return None
        if not self._apply_runtime_parameters(
            planning_time=planning_time,
            num_attempts=num_attempts,
        ):
            self.node.get_logger().warn(
                "Could not apply curobo runtime planner parameters before pose planning; "
                "planner will use its current timeout/max_attempts values."
            )

        request = TrajectoryGeneration.Request()
        request.start_pose = self._current_joint_state()
        request.target_pose = Pose()
        request.target_pose.position.x = float(x)
        request.target_pose.position.y = float(y)
        request.target_pose.position.z = float(z)
        request.target_pose.orientation.x = float(qx)
        request.target_pose.orientation.y = float(qy)
        request.target_pose.orientation.z = float(qz)
        request.target_pose.orientation.w = float(qw)

        future = self.trajectory_client.call_async(request)
        response = self._wait_future_result(future, timeout_sec=30.0)
        if response is None:
            self.node.get_logger().error("Timed out waiting for curobo trajectory response.")
            return None
        if not response.success or not response.trajectory:
            self.node.get_logger().warn(
                f"curobo trajectory request failed: {response.message}"
            )
            return None
        return response

    def _plan_joint_target(
        self,
        joint_targets,
        planning_time=None,
        num_attempts=None,
        planner_type="joint_space",
    ):
        if not self._ensure_planner_type(requested=planner_type):
            return None
        if not self._apply_runtime_parameters(
            planning_time=planning_time,
            num_attempts=num_attempts,
        ):
            self.node.get_logger().warn(
                "Could not apply curobo runtime planner parameters before joint-space planning; "
                "planner will use its current timeout/max_attempts values."
            )

        request = TrajectoryGeneration.Request()
        request.start_pose = self._current_joint_state()
        request.target_joint_positions = [float(joint_targets[joint_name]) for joint_name in JOINT_NAMES]
        future = self.trajectory_client.call_async(request)
        response = self._wait_future_result(future, timeout_sec=30.0)
        if response is None:
            self.node.get_logger().error("Timed out waiting for curobo joint-space response.")
            return None
        if not response.success or not response.trajectory:
            self.node.get_logger().warn(
                f"curobo joint-space request failed: {response.message}"
            )
            return None
        return response

    def _execute_response(self, response):
        return execute_arm_joint_trajectory(
            self.node,
            self.node.arm_traj_client,
            response.trajectory,
            response.dt if response.dt > 0.0 else 0.03,
        )

    def move_to_pose(self, x, y, z, qx, qy, qz, qw, planning_time=None, num_attempts=None):
        orientation_attempts = [("configured", (qx, qy, qz, qw))]
        current_orientation = self.node._lookup_end_effector_orientation()
        if current_orientation is not None:
            orientation_attempts.append(("current_ee", current_orientation))

        seen = []
        for label, orientation in orientation_attempts:
            if any(self.node._quaternions_equivalent(orientation, existing) for existing in seen):
                continue
            seen.append(orientation)
            aqx, aqy, aqz, aqw = orientation
            response = self._plan_pose(
                x,
                y,
                z,
                aqx,
                aqy,
                aqz,
                aqw,
                planning_time=planning_time,
                num_attempts=num_attempts,
            )
            if response is None:
                continue
            if self._execute_response(response):
                return True
            self.node.get_logger().warn(
                f"curobo execution failed for orientation attempt '{label}'."
            )
        return False

    def move_to_home(self):
        self.node.get_logger().info(
            "Planning collision-aware HOME motion with curobo joint-space planner."
        )
        home_target_candidates = self._build_home_joint_target_candidates()
        planning_time = max(float(getattr(self.node, "planning_time", 10.0)), 15.0)
        num_attempts = max(int(getattr(self.node, "num_planning_attempts", 10)), 12)

        for attempt_index, home_joint_targets in enumerate(home_target_candidates, start=1):
            self.node.get_logger().info(
                f"HOME joint target for planning (attempt {attempt_index}/{len(home_target_candidates)}): "
                + ", ".join(
                    f"{joint_name}={float(home_joint_targets[joint_name]):.3f}"
                    for joint_name in JOINT_NAMES
                )
            )
            response = self._plan_joint_target(
                home_joint_targets,
                planning_time=planning_time,
                num_attempts=num_attempts,
                planner_type="joint_space",
            )
            if response is None:
                continue
            return self._execute_response(response)

        self.node.get_logger().error(
            "Failed to generate curobo joint-space HOME trajectory."
        )
        return False

    def move_to_release_pose_with_retries(self):
        release_orientation = self.node._resolve_release_orientation()
        if release_orientation is None:
            self.node.get_logger().error("Configured release orientation is invalid.")
            return False

        attempt_orientations = [release_orientation]
        current_orientation = self.node._lookup_end_effector_orientation()
        if current_orientation is not None and not self.node._quaternions_equivalent(
            current_orientation, release_orientation
        ):
            attempt_orientations.append(current_orientation)

        for orientation in attempt_orientations[: max(1, int(self.node.dropoff_max_pose_retries))]:
            oqx, oqy, oqz, oqw = orientation
            response = self._plan_pose(
                float(self.node.release_pose_x),
                float(self.node.release_pose_y),
                float(self.node.release_pose_z),
                oqx,
                oqy,
                oqz,
                oqw,
                planning_time=float(self.node.dropoff_planning_time_s),
                num_attempts=int(self.node.dropoff_num_planning_attempts),
            )
            if response is None:
                continue
            if self._execute_response(response):
                return True
        return False
