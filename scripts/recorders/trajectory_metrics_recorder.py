#!/usr/bin/env python3
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatusArray
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_METRICS_EVENT_TOPIC = "/metrics_events"
DEFAULT_START_EVENT = "phase_start"
DEFAULT_END_EVENT = "phase_end"
DEFAULT_PHASE_FIELD = "phase"
DEFAULT_COLLISION_SPHERES_TOPIC = "/curobo_live_collision_spheres"
DEFAULT_PLANNING_SCENE_TOPIC = "/planning_scene"
DEFAULT_OBSTACLE_MARKER_TOPIC = "/curobo_world_collision_markers"
DEFAULT_OBSTACLE_MARKER_NAMESPACE = ""
DEFAULT_TRACKED_OBSTACLE_NAMES = ""
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
        collision_spheres_topic: str = DEFAULT_COLLISION_SPHERES_TOPIC,
        planning_scene_topic: str = DEFAULT_PLANNING_SCENE_TOPIC,
        obstacle_marker_topic: str = DEFAULT_OBSTACLE_MARKER_TOPIC,
        obstacle_marker_namespace: str = DEFAULT_OBSTACLE_MARKER_NAMESPACE,
        tracked_obstacle_names: str = DEFAULT_TRACKED_OBSTACLE_NAMES,
    ):
        super().__init__("trajectory_metrics_recorder")
        self.metrics_event_topic = metrics_event_topic
        self.world_frame = world_frame
        self.tcp_frame = tcp_frame
        self.start_event = start_event
        self.end_event = end_event
        self.phase_field = phase_field
        self.default_phase = default_phase
        self.collision_spheres_topic = collision_spheres_topic
        self.planning_scene_topic = planning_scene_topic
        self.obstacle_marker_topic = obstacle_marker_topic
        self.obstacle_marker_namespace = obstacle_marker_namespace
        self.tracked_obstacle_names = {
            name.strip()
            for name in str(tracked_obstacle_names).split(",")
            if name.strip()
        }
        self.marker_obstacles = {}
        self.scene_objects = {}
        self.scene_obstacles = {}
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
        self.create_subscription(
            MarkerArray,
            self.collision_spheres_topic,
            self.on_collision_spheres,
            10,
        )
        self.create_subscription(
            PlanningScene,
            self.planning_scene_topic,
            self.on_planning_scene,
            10,
        )
        self.create_subscription(
            MarkerArray,
            self.obstacle_marker_topic,
            self.on_obstacle_markers,
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
            "saw_collision_spheres": False,
            "min_collision_clearance_m": None,
            "closest_collision_object": None,
            "closest_robot_sphere": None,
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
            self.record_end_without_start(
                bool(success),
                phase=phase,
                planning_time_s=planning_time_s,
                execution_time_s=execution_time_s,
                total_time_s=total_time_s,
                failure_reason=failure_reason,
                metadata=metadata,
            )
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

        collision_status = self.collision_distance_status(phase_data)
        self.segments[phase_data["phase"]] = {
            "phase": phase_data["phase"],
            "planning_time_s": planning_time_s,
            "execution_time_s": execution_time_s,
            "elapsed_time_s": elapsed_time_s,
            "tcp_movement_cm": phase_data["movement_m"] * 100.0,
            "success": bool(success),
            "failure_reason": failure_reason,
            "min_collision_clearance_m": phase_data["min_collision_clearance_m"],
            "collision_distance_status": collision_status,
            "closest_collision_object": phase_data["closest_collision_object"],
            "closest_robot_sphere": phase_data["closest_robot_sphere"],
        }
        if metadata:
            self.segments[phase_data["phase"]].update(metadata)
            self.metadata = metadata
        if not success and self.failed_phase is None:
            self.failed_phase = phase_data["phase"]
            self.failure_reason = failure_reason

    def record_end_without_start(
        self,
        success: bool,
        phase: str | None = None,
        planning_time_s=None,
        execution_time_s=None,
        total_time_s=None,
        failure_reason: str = "",
        metadata: dict | None = None,
    ) -> None:
        phase_name = str(phase or self.default_phase or "")
        if not phase_name:
            return
        if phase_name not in self.phase_order:
            self.phase_order.append(phase_name)

        reported_planning_time_s = self.optional_float(planning_time_s)
        reported_execution_time_s = self.optional_float(execution_time_s)
        reported_total_time_s = self.optional_float(total_time_s)
        elapsed_time_s = reported_total_time_s
        if elapsed_time_s is None:
            elapsed_time_s = sum(
                value
                for value in (
                    reported_planning_time_s,
                    reported_execution_time_s,
                )
                if value is not None
            )

        segment = {
            "phase": phase_name,
            "planning_time_s": reported_planning_time_s,
            "execution_time_s": reported_execution_time_s,
            "elapsed_time_s": elapsed_time_s,
            "tcp_movement_cm": None,
            "success": bool(success),
            "failure_reason": failure_reason,
            "min_collision_clearance_m": None,
            "collision_distance_status": "not_recorded",
            "closest_collision_object": None,
            "closest_robot_sphere": None,
        }
        if metadata:
            segment.update(metadata)
            self.metadata = metadata
        self.segments[phase_name] = segment

        if not success and self.failed_phase is None:
            self.failed_phase = phase_name
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

    def on_obstacle_markers(self, msg: MarkerArray) -> None:
        for marker in msg.markers:
            if int(marker.action) == Marker.DELETEALL:
                self.marker_obstacles.clear()
                continue

            key = (str(marker.ns), int(marker.id))
            if int(marker.action) == Marker.DELETE:
                self.marker_obstacles.pop(key, None)
                continue

            if int(marker.action) != Marker.ADD or int(marker.type) != Marker.CUBE:
                continue
            if self.obstacle_marker_namespace and str(marker.ns) != self.obstacle_marker_namespace:
                continue

            name = str(marker.text or f"{marker.ns}:{marker.id}")
            if self.tracked_obstacle_names and name not in self.tracked_obstacle_names:
                continue

            center = self.point_in_world(
                getattr(marker.header, "frame_id", "") or self.world_frame,
                [
                    float(marker.pose.position.x),
                    float(marker.pose.position.y),
                    float(marker.pose.position.z),
                ],
            )
            if center is None:
                continue

            self.marker_obstacles[key] = {
                "name": name,
                "shape": "box",
                "center": center,
                "size": [
                    max(float(marker.scale.x), 0.0),
                    max(float(marker.scale.y), 0.0),
                    max(float(marker.scale.z), 0.0),
                ],
                "orientation": self.normalized_quaternion(
                    [
                        float(marker.pose.orientation.x),
                        float(marker.pose.orientation.y),
                        float(marker.pose.orientation.z),
                        float(marker.pose.orientation.w),
                    ]
                ),
            }

    def on_planning_scene(self, msg: PlanningScene) -> None:
        for collision_object in msg.world.collision_objects:
            self.update_scene_object(collision_object)

    def update_scene_object(self, collision_object: CollisionObject) -> None:
        object_id = str(collision_object.id or "").strip()
        if not object_id:
            return

        operation = self.uint8_value(collision_object.operation)
        if operation == self.uint8_value(CollisionObject.REMOVE):
            self.scene_objects.pop(object_id, None)
            self.remove_scene_obstacles(object_id)
            return

        if operation == self.uint8_value(CollisionObject.MOVE):
            entry = self.scene_objects.get(object_id)
            if entry is None:
                return
            entry["pose"] = self.pose_to_dict(collision_object.pose)
            entry["frame_id"] = str(collision_object.header.frame_id or self.world_frame)
            self.rebuild_scene_object_obstacles(object_id)
            return

        if operation != self.uint8_value(CollisionObject.ADD):
            return

        geometries = []
        for primitive, primitive_pose in zip(
            collision_object.primitives,
            collision_object.primitive_poses,
        ):
            geometry = self.primitive_geometry(primitive)
            if geometry is None:
                continue
            geometry["pose"] = self.pose_to_dict(primitive_pose)
            geometries.append(geometry)

        for mesh, mesh_pose in zip(collision_object.meshes, collision_object.mesh_poses):
            vertices = [
                [float(vertex.x), float(vertex.y), float(vertex.z)]
                for vertex in mesh.vertices
            ]
            if not vertices:
                continue
            geometries.append(
                {
                    "shape": "mesh",
                    "pose": self.pose_to_dict(mesh_pose),
                    "vertices": vertices,
                }
            )

        if not geometries:
            return

        self.scene_objects[object_id] = {
            "frame_id": str(collision_object.header.frame_id or self.world_frame),
            "pose": self.pose_to_dict(collision_object.pose),
            "geometries": geometries,
        }
        self.rebuild_scene_object_obstacles(object_id)

    def remove_scene_obstacles(self, object_id: str) -> None:
        prefix = (str(object_id),)
        for key in list(self.scene_obstacles):
            if key[:1] == prefix:
                self.scene_obstacles.pop(key, None)

    def rebuild_scene_object_obstacles(self, object_id: str) -> None:
        entry = self.scene_objects.get(object_id)
        self.remove_scene_obstacles(object_id)
        if entry is None:
            return

        object_pose = self.pose_in_world(entry["frame_id"], entry["pose"])
        if object_pose is None:
            return

        for index, geometry in enumerate(entry["geometries"]):
            world_pose = self.compose_pose(object_pose, geometry["pose"])
            key = (object_id, index)
            name = object_id if len(entry["geometries"]) == 1 else f"{object_id}:{index}"
            shape = geometry["shape"]
            if shape == "box":
                self.scene_obstacles[key] = {
                    "name": name,
                    "shape": "box",
                    "center": world_pose["position"],
                    "size": list(geometry["size"]),
                    "orientation": world_pose["orientation"],
                }
            elif shape == "sphere":
                self.scene_obstacles[key] = {
                    "name": name,
                    "shape": "sphere",
                    "center": world_pose["position"],
                    "radius": float(geometry["radius"]),
                }
            elif shape == "cylinder":
                radius = float(geometry["radius"])
                length = float(geometry["length"])
                self.scene_obstacles[key] = {
                    "name": name,
                    "shape": "box",
                    "center": world_pose["position"],
                    "size": [2.0 * radius, 2.0 * radius, length],
                    "orientation": world_pose["orientation"],
                }
            elif shape == "mesh":
                world_vertices = [
                    self.transform_point(world_pose, vertex)
                    for vertex in geometry["vertices"]
                ]
                if not world_vertices:
                    continue
                mins = [min(vertex[axis] for vertex in world_vertices) for axis in range(3)]
                maxs = [max(vertex[axis] for vertex in world_vertices) for axis in range(3)]
                self.scene_obstacles[key] = {
                    "name": name,
                    "shape": "box",
                    "center": [
                        0.5 * (mins[axis] + maxs[axis]) for axis in range(3)
                    ],
                    "size": [
                        maxs[axis] - mins[axis] for axis in range(3)
                    ],
                    "orientation": [0.0, 0.0, 0.0, 1.0],
                }

    @staticmethod
    def uint8_value(value) -> int:
        if isinstance(value, (bytes, bytearray)):
            return int(value[0]) if value else 0
        return int(value)

    @staticmethod
    def primitive_geometry(primitive: SolidPrimitive):
        primitive_type = int(primitive.type)
        dimensions = [float(value) for value in primitive.dimensions]
        if primitive_type == SolidPrimitive.BOX and len(dimensions) >= 3:
            return {
                "shape": "box",
                "size": [
                    max(dimensions[SolidPrimitive.BOX_X], 0.0),
                    max(dimensions[SolidPrimitive.BOX_Y], 0.0),
                    max(dimensions[SolidPrimitive.BOX_Z], 0.0),
                ],
            }
        if primitive_type == SolidPrimitive.SPHERE and dimensions:
            return {
                "shape": "sphere",
                "radius": max(dimensions[SolidPrimitive.SPHERE_RADIUS], 0.0),
            }
        if primitive_type == SolidPrimitive.CYLINDER and len(dimensions) >= 2:
            return {
                "shape": "cylinder",
                "length": max(dimensions[SolidPrimitive.CYLINDER_HEIGHT], 0.0),
                "radius": max(dimensions[SolidPrimitive.CYLINDER_RADIUS], 0.0),
            }
        return None

    def on_collision_spheres(self, msg: MarkerArray) -> None:
        if self.current_phase is None:
            return
        self.current_phase["saw_collision_spheres"] = True
        obstacles = self.collision_obstacles()
        if not obstacles:
            return

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
            center = self.point_in_world(
                getattr(marker.header, "frame_id", "") or self.world_frame,
                [
                    float(marker.pose.position.x),
                    float(marker.pose.position.y),
                    float(marker.pose.position.z),
                ],
            )
            if center is None:
                continue

            for obstacle in obstacles:
                clearance = self.sphere_clearance_to_obstacle(center, radius, obstacle)
                current = self.current_phase["min_collision_clearance_m"]
                if current is not None and clearance >= float(current):
                    continue

                self.current_phase["min_collision_clearance_m"] = float(clearance)
                self.current_phase["closest_collision_object"] = str(obstacle["name"])
                self.current_phase["closest_robot_sphere"] = {
                    "center": [float(value) for value in center],
                    "radius": float(radius),
                    "frame_id": self.world_frame,
                }

    def collision_distance_status(self, phase_data: dict) -> str:
        if phase_data.get("min_collision_clearance_m") is not None:
            return "ok"
        if not self.collision_obstacles():
            return "no_collision_objects"
        if not bool(phase_data.get("saw_collision_spheres")):
            return "robot_collision_spheres_unavailable"
        return "not_sampled"

    def collision_obstacles(self):
        return [
            *self.scene_obstacles.values(),
            *self.marker_obstacles.values(),
        ]

    def point_in_world(self, frame_id: str, point_xyz):
        frame_id = str(frame_id or self.world_frame).strip()
        if frame_id.startswith("/"):
            frame_id = frame_id[1:]
        point = [float(value) for value in point_xyz]
        if frame_id == self.world_frame:
            return point
        try:
            transform = self.tf_buffer.lookup_transform(self.world_frame, frame_id, Time())
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rotated = self.rotate_vector_by_quaternion(
            point,
            [rotation.x, rotation.y, rotation.z, rotation.w],
        )
        return [
            float(translation.x) + rotated[0],
            float(translation.y) + rotated[1],
            float(translation.z) + rotated[2],
        ]

    @staticmethod
    def rotate_vector_by_quaternion(vector_xyz, quaternion_xyzw):
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

    @staticmethod
    def normalized_quaternion(quaternion_xyzw):
        qx, qy, qz, qw = (float(value) for value in quaternion_xyzw)
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 0.0:
            return [0.0, 0.0, 0.0, 1.0]
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm
        return [qx, qy, qz, qw]

    @staticmethod
    def pose_to_dict(pose):
        return {
            "position": [
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            ],
            "orientation": TrajectoryMetricsRecorder.normalized_quaternion(
                [
                    float(pose.orientation.x),
                    float(pose.orientation.y),
                    float(pose.orientation.z),
                    float(pose.orientation.w),
                ]
            ),
        }

    def pose_in_world(self, frame_id: str, pose: dict):
        position = self.point_in_world(frame_id, pose["position"])
        if position is None:
            return None
        frame_id = str(frame_id or self.world_frame).strip()
        if frame_id.startswith("/"):
            frame_id = frame_id[1:]
        orientation = pose["orientation"]
        if frame_id != self.world_frame:
            try:
                transform = self.tf_buffer.lookup_transform(self.world_frame, frame_id, Time())
            except TransformException:
                return None
            tf_q = [
                float(transform.transform.rotation.x),
                float(transform.transform.rotation.y),
                float(transform.transform.rotation.z),
                float(transform.transform.rotation.w),
            ]
            orientation = self.multiply_quaternions(tf_q, orientation)
        return {
            "position": position,
            "orientation": self.normalized_quaternion(orientation),
        }

    def compose_pose(self, parent: dict, child: dict) -> dict:
        rotated_offset = self.rotate_vector_by_quaternion(
            child["position"],
            parent["orientation"],
        )
        return {
            "position": [
                float(parent["position"][axis]) + rotated_offset[axis]
                for axis in range(3)
            ],
            "orientation": self.multiply_quaternions(
                parent["orientation"],
                child["orientation"],
            ),
        }

    def transform_point(self, pose: dict, point_xyz):
        rotated = self.rotate_vector_by_quaternion(point_xyz, pose["orientation"])
        return [
            float(pose["position"][axis]) + rotated[axis]
            for axis in range(3)
        ]

    @staticmethod
    def multiply_quaternions(lhs_xyzw, rhs_xyzw):
        lx, ly, lz, lw = TrajectoryMetricsRecorder.normalized_quaternion(lhs_xyzw)
        rx, ry, rz, rw = TrajectoryMetricsRecorder.normalized_quaternion(rhs_xyzw)
        return TrajectoryMetricsRecorder.normalized_quaternion(
            [
                lw * rx + lx * rw + ly * rz - lz * ry,
                lw * ry - lx * rz + ly * rw + lz * rx,
                lw * rz + lx * ry - ly * rx + lz * rw,
                lw * rw - lx * rx - ly * ry - lz * rz,
            ]
        )

    @staticmethod
    def point_clearance_to_obstacle(point_xyz, obstacle: dict) -> float:
        orientation = obstacle.get("orientation", [0.0, 0.0, 0.0, 1.0])
        dx = float(point_xyz[0]) - float(obstacle["center"][0])
        dy = float(point_xyz[1]) - float(obstacle["center"][1])
        dz = float(point_xyz[2]) - float(obstacle["center"][2])
        inv_orientation = [
            -float(orientation[0]),
            -float(orientation[1]),
            -float(orientation[2]),
            float(orientation[3]),
        ]
        local_x, local_y, local_z = TrajectoryMetricsRecorder.rotate_vector_by_quaternion(
            [dx, dy, dz],
            inv_orientation,
        )
        half = [max(float(s) * 0.5, 0.0) for s in obstacle["size"]]
        qx = abs(local_x) - half[0]
        qy = abs(local_y) - half[1]
        qz = abs(local_z) - half[2]
        outside_x = max(qx, 0.0)
        outside_y = max(qy, 0.0)
        outside_z = max(qz, 0.0)
        outside_distance = math.sqrt(
            outside_x * outside_x + outside_y * outside_y + outside_z * outside_z
        )
        inside_distance = min(max(qx, qy, qz), 0.0)
        return outside_distance + inside_distance

    def sphere_clearance_to_obstacle(self, center_xyz, radius: float, obstacle: dict) -> float:
        robot_radius = max(float(radius), 0.0)
        if obstacle.get("shape") == "sphere":
            dx = float(center_xyz[0]) - float(obstacle["center"][0])
            dy = float(center_xyz[1]) - float(obstacle["center"][1])
            dz = float(center_xyz[2]) - float(obstacle["center"][2])
            center_distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            return center_distance - robot_radius - max(float(obstacle["radius"]), 0.0)
        return self.point_clearance_to_obstacle(center_xyz, obstacle) - robot_radius

    def result(self) -> dict:
        success = bool(self.segments) and all(
            segment.get("success") is True for segment in self.segments.values()
        )
        if self.segments:
            totals = {
                "planning_time_s": self.sum_metric("planning_time_s"),
                "execution_time_s": self.sum_metric("execution_time_s"),
                "tcp_movement_cm": self.sum_metric("tcp_movement_cm"),
                "min_collision_clearance_m": self.min_metric("min_collision_clearance_m"),
                "collision_distance_status": self.total_collision_distance_status(),
            }
        else:
            totals = {
                "planning_time_s": None,
                "execution_time_s": None,
                "tcp_movement_cm": None,
                "min_collision_clearance_m": None,
                "collision_distance_status": "not_recorded",
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

    def min_metric(self, key: str):
        values = [
            float(segment[key])
            for segment in self.segments.values()
            if segment.get(key) is not None
        ]
        if not values:
            return None
        return min(values)

    def total_collision_distance_status(self) -> str:
        statuses = [
            str(segment.get("collision_distance_status") or "")
            for segment in self.segments.values()
        ]
        if any(status == "ok" for status in statuses):
            return "ok"
        if any(status == "robot_collision_spheres_unavailable" for status in statuses):
            return "robot_collision_spheres_unavailable"
        if any(status == "no_collision_objects" for status in statuses):
            return "no_collision_objects"
        if statuses:
            return statuses[0] or "not_sampled"
        return "not_recorded"


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
    collision_spheres_topic = (
        sys.argv[9] if len(sys.argv) > 9 else DEFAULT_COLLISION_SPHERES_TOPIC
    )
    obstacle_marker_topic = (
        sys.argv[10] if len(sys.argv) > 10 else DEFAULT_OBSTACLE_MARKER_TOPIC
    )
    obstacle_marker_namespace = (
        sys.argv[11] if len(sys.argv) > 11 else DEFAULT_OBSTACLE_MARKER_NAMESPACE
    )
    tracked_obstacle_names = (
        sys.argv[12] if len(sys.argv) > 12 else DEFAULT_TRACKED_OBSTACLE_NAMES
    )

    rclpy.init()
    node = TrajectoryMetricsRecorder(
        metrics_event_topic,
        world_frame,
        tcp_frame,
        start_event,
        end_event,
        phase_field,
        default_phase,
        collision_spheres_topic,
        DEFAULT_PLANNING_SCENE_TOPIC,
        obstacle_marker_topic,
        obstacle_marker_namespace,
        tracked_obstacle_names,
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
