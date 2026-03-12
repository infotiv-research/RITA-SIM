#!/usr/bin/env python3

import os
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

try:
    import trimesh
except ImportError:
    trimesh = None

try:
    from curobo_msgs.srv import AddObject, RemoveObject, UpdateObjectPoses
except ImportError:
    AddObject = None
    RemoveObject = None
    UpdateObjectPoses = None


def _parse_float_list(raw, expected_len=None):
    parts = str(raw).split()
    values = []
    for token in parts:
        try:
            values.append(float(token))
        except ValueError:
            continue
    if expected_len is not None and len(values) < expected_len:
        return None
    return values


def _parse_bool(raw):
    if isinstance(raw, bool):
        return raw
    return str(raw).strip().lower() in {"1", "true", "yes", "on"}


def _parse_origin(origin_tag):
    if origin_tag is None:
        pose = Pose()
        pose.orientation.w = 1.0
        return pose

    xyz = _parse_float_list(origin_tag.get("xyz", "0 0 0"), expected_len=3) or [0.0, 0.0, 0.0]
    rpy = _parse_float_list(origin_tag.get("rpy", "0 0 0"), expected_len=3) or [0.0, 0.0, 0.0]
    quat = Rotation.from_euler("xyz", rpy).as_quat()
    pose = Pose()
    pose.position.x = float(xyz[0])
    pose.position.y = float(xyz[1])
    pose.position.z = float(xyz[2])
    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])
    return pose


class IsaacCuroboWorldBridge(Node):
    def __init__(self):
        super().__init__("curobo_world_bridge")

        self.assets_root = os.path.abspath(
            str(self.declare_parameter("assets_root", "assets/isaac_urdf_exports").value)
        )
        self.world_frame = str(self.declare_parameter("world_frame", "world").value)
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 2.0).value)
        self.startup_delay_sec = float(self.declare_parameter("startup_delay_sec", 3.0).value)
        self.environment_collision_padding = float(
            self.declare_parameter("environment_collision_padding", 0.0).value
        )
        self.mesh_as_cuboid = _parse_bool(
            self.declare_parameter("mesh_as_cuboid", True).value
        )
        self.max_mesh_geometries_per_object = max(
            1, int(self.declare_parameter("max_mesh_geometries_per_object", 5).value)
        )
        exclude_raw = self.declare_parameter("exclude_collision_objects", "").value
        self.control_topic = str(
            self.declare_parameter(
                "environment_collision_control_topic",
                "/environment_collision/control",
            ).value
        )
        self.publish_collision_markers = _parse_bool(
            self.declare_parameter("publish_collision_markers", True).value
        )
        self.collision_marker_topic = str(
            self.declare_parameter(
                "collision_marker_topic",
                "/curobo_world_collision_markers",
            ).value
        )
        self.collision_marker_namespace = str(
            self.declare_parameter(
                "collision_marker_namespace",
                "curobo_world_collision",
            ).value
        )
        self.collision_marker_alpha = float(
            self.declare_parameter("collision_marker_alpha", 0.22).value
        )
        self.pose_position_tolerance_m = float(
            self.declare_parameter("pose_position_tolerance_m", 0.002).value
        )
        self.pose_orientation_tolerance_rad = float(
            self.declare_parameter("pose_orientation_tolerance_rad", 0.02).value
        )
        self.dimensions_tolerance_m = float(
            self.declare_parameter("dimensions_tolerance_m", 0.001).value
        )
        self.max_concurrent_adds = max(
            1, int(self.declare_parameter("max_concurrent_adds", 8).value)
        )
        self.use_move_updates = _parse_bool(
            self.declare_parameter("use_move_updates", True).value
        )
        self.max_move_batch_size = max(
            1, int(self.declare_parameter("max_move_batch_size", 128).value)
        )

        self.excluded_ids = self._parse_excluded_ids(exclude_raw)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.control_sub = self.create_subscription(
            String, self.control_topic, self._on_control_command, 10
        )
        self.marker_pub = None
        if self.publish_collision_markers:
            self.marker_pub = self.create_publisher(
                MarkerArray,
                self.collision_marker_topic,
                10,
            )

        self.add_client = self.create_client(AddObject, "/unified_planner/add_object")
        self.remove_client = self.create_client(RemoveObject, "/unified_planner/remove_object")
        self.move_client = None
        if UpdateObjectPoses is not None:
            self.move_client = self.create_client(
                UpdateObjectPoses,
                "/unified_planner/update_object_poses",
            )

        self.objects = {}
        self.alias_map = {}
        self.published_signatures = {}
        self.suppressed_frames = set()
        self._mesh_bbox_cache = {}
        self._warned_missing_frames = set()
        self._loaded_spec_count = 0
        self._ready_at_monotonic = time.monotonic() + max(0.0, self.startup_delay_sec)
        self._inflight_operations = []
        self._published_marker_specs = {}
        self._warned_missing_move_service = False
        self.load_collision_geometry()
        self.get_logger().info(
            f"Loaded curobo environment bridge assets: {len(self.objects)} frames, "
            f"{self._loaded_spec_count} collision geometries from '{self.assets_root}'."
        )

        clamped_rate = max(0.1, self.publish_rate_hz)
        self.create_timer(1.0 / clamped_rate, self.publish_objects)
        if self.marker_pub is not None:
            self.create_timer(1.0 / clamped_rate, self.publish_collision_markers_markers)

    @staticmethod
    def _parse_excluded_ids(raw):
        if isinstance(raw, (list, tuple)):
            return {str(item).strip() for item in raw if str(item).strip()}
        return {part.strip() for part in str(raw).split(",") if part.strip()}

    @staticmethod
    def _compose_pose(parent_pose, child_pose):
        parent_rot = Rotation.from_quat(
            [
                parent_pose.orientation.x,
                parent_pose.orientation.y,
                parent_pose.orientation.z,
                parent_pose.orientation.w,
            ]
        )
        child_rot = Rotation.from_quat(
            [
                child_pose.orientation.x,
                child_pose.orientation.y,
                child_pose.orientation.z,
                child_pose.orientation.w,
            ]
        )
        rotated_offset = parent_rot.apply(
            [child_pose.position.x, child_pose.position.y, child_pose.position.z]
        )
        pose = Pose()
        pose.position.x = float(parent_pose.position.x + rotated_offset[0])
        pose.position.y = float(parent_pose.position.y + rotated_offset[1])
        pose.position.z = float(parent_pose.position.z + rotated_offset[2])
        quat = (parent_rot * child_rot).as_quat()
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
        return pose

    def _build_primitive_spec(self, geometry_tag, origin_pose, folder_path):
        box_tag = geometry_tag.find("box")
        if box_tag is not None:
            size = _parse_float_list(box_tag.get("size", ""), expected_len=3)
            if size is None:
                return None
            pad = max(0.0, float(self.environment_collision_padding))
            return {
                "type": AddObject.Request.CUBOID,
                "pose": origin_pose,
                "dimensions": (
                    max(1e-6, float(size[0]) + 2.0 * pad),
                    max(1e-6, float(size[1]) + 2.0 * pad),
                    max(1e-6, float(size[2]) + 2.0 * pad),
                ),
                "mesh_file_path": "",
            }

        cylinder_tag = geometry_tag.find("cylinder")
        if cylinder_tag is not None:
            try:
                radius = float(cylinder_tag.get("radius", "0"))
                length = float(cylinder_tag.get("length", "0"))
            except ValueError:
                return None
            return {
                "type": AddObject.Request.CYLINDER,
                "pose": origin_pose,
                "dimensions": (radius, length, length),
                "mesh_file_path": "",
            }

        sphere_tag = geometry_tag.find("sphere")
        if sphere_tag is not None:
            try:
                radius = float(sphere_tag.get("radius", "0"))
            except ValueError:
                return None
            return {
                "type": AddObject.Request.SPHERE,
                "pose": origin_pose,
                "dimensions": (radius, radius, radius),
                "mesh_file_path": "",
            }

        mesh_tag = geometry_tag.find("mesh")
        if mesh_tag is None:
            return None
        mesh_filename = mesh_tag.get("filename", "").split("/")[-1]
        mesh_path = os.path.join(folder_path, "meshes", mesh_filename)
        if not os.path.exists(mesh_path):
            return None
        scale = _parse_float_list(mesh_tag.get("scale", "1 1 1"), expected_len=3)
        if scale is None:
            scale = [1.0, 1.0, 1.0]
        mesh_cuboid_spec = self._build_mesh_cuboid_spec(mesh_path, origin_pose, scale)
        if mesh_cuboid_spec is not None:
            return mesh_cuboid_spec
        return {
            "type": AddObject.Request.MESH,
            "pose": origin_pose,
            "dimensions": (float(scale[0]), float(scale[1]), float(scale[2])),
            "mesh_file_path": mesh_path,
        }

    @staticmethod
    def _compute_oriented_bounding_box(mesh):
        """Compute a tight oriented bounding box in the mesh local frame.

        Using trimesh's OBB primitive avoids the PCA ambiguity that can
        oversize near-symmetric meshes like cubes.
        """
        obb = mesh.bounding_box_oriented
        extents = np.array(obb.primitive.extents, dtype=float)
        transform = np.array(obb.primitive.transform, dtype=float)
        center = transform[:3, 3]
        quat = Rotation.from_matrix(transform[:3, :3]).as_quat()
        return (
            tuple(float(v) for v in center),
            tuple(float(v) for v in extents),
            tuple(float(v) for v in quat),
        )

    def _build_mesh_cuboid_spec(self, mesh_path, origin_pose, scale):
        if not self.mesh_as_cuboid or trimesh is None:
            return None

        scale_key = tuple(float(value) for value in scale)
        cache_key = (mesh_path, scale_key)
        cached = self._mesh_bbox_cache.get(cache_key)
        if cached is None:
            try:
                mesh = trimesh.load(mesh_path, force="mesh", process=False)
                if isinstance(mesh, trimesh.Scene):
                    mesh = trimesh.util.concatenate(tuple(mesh.geometry.values()))
                mesh = mesh.copy()
                mesh.apply_scale(scale_key)
                centroid, extents, quat_xyzw = self._compute_oriented_bounding_box(mesh)
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to approximate mesh '{mesh_path}' as cuboid: {exc}"
                )
                self._mesh_bbox_cache[cache_key] = None
                return None
            cached = (centroid, extents, quat_xyzw)
            self._mesh_bbox_cache[cache_key] = cached

        if cached is None:
            return None

        centroid, extents, quat_xyzw = cached
        pad = max(0.0, float(self.environment_collision_padding))
        centroid_pose = Pose()
        centroid_pose.position.x = centroid[0]
        centroid_pose.position.y = centroid[1]
        centroid_pose.position.z = centroid[2]
        centroid_pose.orientation.x = quat_xyzw[0]
        centroid_pose.orientation.y = quat_xyzw[1]
        centroid_pose.orientation.z = quat_xyzw[2]
        centroid_pose.orientation.w = quat_xyzw[3]
        return {
            "type": AddObject.Request.CUBOID,
            "pose": self._compose_pose(origin_pose, centroid_pose),
            "dimensions": (
                max(1e-6, extents[0] + 2.0 * pad),
                max(1e-6, extents[1] + 2.0 * pad),
                max(1e-6, extents[2] + 2.0 * pad),
            ),
            "mesh_file_path": "",
        }

    def load_collision_geometry(self):
        if not os.path.isdir(self.assets_root):
            self.get_logger().warn(
                f"Assets root does not exist or is not a directory: {self.assets_root}"
            )
            return

        for obj_folder in sorted(os.listdir(self.assets_root)):
            if obj_folder in self.excluded_ids:
                continue

            folder_path = os.path.join(self.assets_root, obj_folder)
            if not os.path.isdir(folder_path):
                continue

            urdf_path = os.path.join(folder_path, f"{obj_folder}.urdf")
            if not os.path.exists(urdf_path):
                continue

            try:
                root = ET.parse(urdf_path).getroot()
            except Exception as exc:
                self.get_logger().warn(f"Failed to parse URDF '{urdf_path}': {exc}")
                continue

            specs = []
            collision_specs_added = 0
            for index, collision_tag in enumerate(root.findall(".//link/collision")):
                origin_pose = _parse_origin(collision_tag.find("origin"))
                geometry_tag = collision_tag.find("geometry")
                if geometry_tag is None:
                    continue
                spec = self._build_primitive_spec(geometry_tag, origin_pose, folder_path)
                if spec is None:
                    continue
                collision_specs_added += 1
                if collision_specs_added > self.max_mesh_geometries_per_object:
                    continue
                if index == 0:
                    object_name = obj_folder
                else:
                    object_name = f"{obj_folder}__geom_{index}"
                spec["name"] = object_name
                specs.append(spec)
            if specs:
                self.objects[obj_folder] = specs
                self.alias_map[obj_folder] = [spec["name"] for spec in specs]
                self._loaded_spec_count += len(specs)

    def _remove_object(self, object_name):
        if RemoveObject is None:
            return False
        request = RemoveObject.Request()
        request.name = str(object_name)
        return self.remove_client.call_async(request)

    def _add_object(self, spec, frame_pose):
        if AddObject is None:
            return False
        world_pose = self._compose_pose(frame_pose, spec["pose"])
        request = AddObject.Request()
        request.type = int(spec["type"])
        request.name = str(spec["name"])
        request.pose = world_pose
        request.dimensions.x = float(spec["dimensions"][0])
        request.dimensions.y = float(spec["dimensions"][1])
        request.dimensions.z = float(spec["dimensions"][2])
        request.mesh_file_path = str(spec["mesh_file_path"])
        request.color.r = 0.0
        request.color.g = 1.0
        request.color.b = 1.0
        request.color.a = 1.0
        return self.add_client.call_async(request)

    def _poll_inflight_operations(self):
        pending_operations = []
        for entry in self._inflight_operations:
            future = entry.get("future")
            if future is None or not future.done():
                pending_operations.append(entry)
                continue

            operation = entry.get("operation") or {}
            object_name = operation.get("name", "<unknown>")
            signature = operation.get("signature")
            op_type = operation.get("type")

            success = False
            message = ""
            try:
                response = future.result()
                success = bool(getattr(response, "success", False))
                message = str(getattr(response, "message", ""))
            except Exception as exc:
                message = str(exc)

            duplicate_add = (
                op_type == "add"
                and not success
                and "already exists" in (message or "").lower()
            )

            if success:
                if op_type == "add":
                    self.published_signatures[object_name] = signature
                    self._published_marker_specs[object_name] = {
                        "spec": operation.get("spec"),
                        "world_pose": operation.get("world_pose"),
                    }
                elif op_type == "remove":
                    self.published_signatures.pop(object_name, None)
                    self._published_marker_specs.pop(object_name, None)
                elif op_type == "move":
                    updated_objects = operation.get("updated_objects", {})
                    for updated_name, updated_entry in updated_objects.items():
                        self.published_signatures[updated_name] = updated_entry["state"]
                        self._published_marker_specs[updated_name] = {
                            "spec": updated_entry["spec"],
                            "world_pose": updated_entry["world_pose"],
                        }
            elif duplicate_add:
                # A stale object can survive across planner restarts. Treat this as a
                # reconcile signal: mark it present with an unknown signature so the
                # next publish cycle removes it once and then adds the current spec.
                self.published_signatures[object_name] = None
                self.get_logger().info(
                    f"Curobo world bridge reconciling stale object '{object_name}' "
                    "that already exists in planner state."
                )
            else:
                self.get_logger().warn(
                    f"Curobo world bridge {op_type or 'operation'} failed for '{object_name}': "
                    f"{message or 'unknown error'}"
                )

        self._inflight_operations = pending_operations

    def _has_inflight_remove(self):
        return any(
            (entry.get("operation") or {}).get("type") == "remove"
            for entry in self._inflight_operations
        )

    def _has_inflight_move(self):
        return any(
            (entry.get("operation") or {}).get("type") == "move"
            for entry in self._inflight_operations
        )

    def _inflight_add_names(self):
        return {
            (entry.get("operation") or {}).get("name")
            for entry in self._inflight_operations
            if (entry.get("operation") or {}).get("type") == "add"
        }

    def _update_object_poses(self, update_batch):
        if UpdateObjectPoses is None or self.move_client is None:
            return None

        request = UpdateObjectPoses.Request()
        for entry in update_batch:
            request.names.append(str(entry["name"]))
            request.poses.append(entry["world_pose"])
        return self.move_client.call_async(request)

    def _on_control_command(self, msg):
        raw = str(msg.data).strip()
        if not raw or raw.lower() == "clear":
            if raw.lower() == "clear":
                self.suppressed_frames.clear()
            return

        if ":" not in raw:
            self.get_logger().warn(
                f"Invalid control command '{raw}'. Expected suppress:<id>, unsuppress:<id>, or clear."
            )
            return

        action, object_id = raw.split(":", 1)
        action = action.strip().lower()
        object_id = object_id.strip()
        if not object_id:
            return

        frame_name = object_id
        if action == "suppress":
            self.suppressed_frames.add(frame_name)
            return

        if action == "unsuppress":
            self.suppressed_frames.discard(frame_name)
            return

    def _marker_color(self, spec_type):
        alpha = max(0.01, min(1.0, self.collision_marker_alpha))
        if spec_type == AddObject.Request.CUBOID:
            return (0.05, 0.85, 1.0, alpha)
        if spec_type == AddObject.Request.CYLINDER:
            return (1.0, 0.72, 0.15, alpha)
        if spec_type == AddObject.Request.SPHERE:
            return (0.35, 1.0, 0.35, alpha)
        return (1.0, 0.45, 0.2, alpha)

    def _make_marker(self, object_name, spec, world_pose, marker_id):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self.collision_marker_namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose = world_pose
        marker.frame_locked = False

        r, g, b, a = self._marker_color(spec["type"])
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = float(a)

        if spec["type"] == AddObject.Request.CUBOID:
            marker.type = Marker.CUBE
            marker.scale.x = float(spec["dimensions"][0])
            marker.scale.y = float(spec["dimensions"][1])
            marker.scale.z = float(spec["dimensions"][2])
        elif spec["type"] == AddObject.Request.CYLINDER:
            marker.type = Marker.CYLINDER
            radius = float(spec["dimensions"][0])
            length = float(spec["dimensions"][1])
            marker.scale.x = 2.0 * radius
            marker.scale.y = 2.0 * radius
            marker.scale.z = length
        elif spec["type"] == AddObject.Request.SPHERE:
            marker.type = Marker.SPHERE
            diameter = 2.0 * float(spec["dimensions"][0])
            marker.scale.x = diameter
            marker.scale.y = diameter
            marker.scale.z = diameter
        elif spec["type"] == AddObject.Request.MESH and spec["mesh_file_path"]:
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = Path(spec["mesh_file_path"]).resolve().as_uri()
            marker.mesh_use_embedded_materials = False
            marker.scale.x = float(spec["dimensions"][0])
            marker.scale.y = float(spec["dimensions"][1])
            marker.scale.z = float(spec["dimensions"][2])
        else:
            return None

        marker.text = object_name
        return marker

    @staticmethod
    def _build_object_state(spec, world_pose):
        return {
            "position": (
                float(world_pose.position.x),
                float(world_pose.position.y),
                float(world_pose.position.z),
            ),
            "orientation": (
                float(world_pose.orientation.x),
                float(world_pose.orientation.y),
                float(world_pose.orientation.z),
                float(world_pose.orientation.w),
            ),
            "type": int(spec["type"]),
            "dimensions": tuple(float(value) for value in spec["dimensions"]),
            "mesh_file_path": str(spec["mesh_file_path"]),
        }

    @staticmethod
    def _orientation_distance_rad(lhs_xyzw, rhs_xyzw):
        lhs = Rotation.from_quat(lhs_xyzw)
        rhs = Rotation.from_quat(rhs_xyzw)
        return float((lhs.inv() * rhs).magnitude())

    def _states_match(self, previous_state, current_state):
        if previous_state is None or current_state is None:
            return False
        if previous_state["type"] != current_state["type"]:
            return False
        if previous_state["mesh_file_path"] != current_state["mesh_file_path"]:
            return False

        if any(
            abs(current - previous) > self.pose_position_tolerance_m
            for previous, current in zip(
                previous_state["position"], current_state["position"]
            )
        ):
            return False

        if (
            self._orientation_distance_rad(
                previous_state["orientation"],
                current_state["orientation"],
            )
            > self.pose_orientation_tolerance_rad
        ):
            return False

        if any(
            abs(current - previous) > self.dimensions_tolerance_m
            for previous, current in zip(
                previous_state["dimensions"], current_state["dimensions"]
            )
        ):
            return False

        return True

    def publish_collision_markers_markers(self):
        if self.marker_pub is None:
            return

        marker_array = MarkerArray()
        delete_all = Marker()
        delete_all.header.frame_id = self.world_frame
        delete_all.header.stamp = self.get_clock().now().to_msg()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        for marker_id, object_name in enumerate(sorted(self._published_marker_specs)):
            entry = self._published_marker_specs[object_name]
            marker = self._make_marker(
                object_name,
                entry["spec"],
                entry["world_pose"],
                marker_id,
            )
            if marker is not None:
                marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def publish_objects(self):
        if AddObject is None or RemoveObject is None:
            return
        if time.monotonic() < self._ready_at_monotonic:
            return
        self._poll_inflight_operations()

        if not self.add_client.service_is_ready():
            if not self.add_client.wait_for_service(timeout_sec=0.0):
                return
        if not self.remove_client.service_is_ready():
            if not self.remove_client.wait_for_service(timeout_sec=0.0):
                return
        move_updates_available = False
        if self.use_move_updates and UpdateObjectPoses is not None and self.move_client is not None:
            if self.move_client.service_is_ready() or self.move_client.wait_for_service(timeout_sec=0.0):
                move_updates_available = True
                self._warned_missing_move_service = False
            elif not self._warned_missing_move_service:
                self.get_logger().warn(
                    "cuRobo MOVE updates are enabled, but /unified_planner/update_object_poses "
                    "is not available yet. Falling back to remove/add reconciliation until it appears."
                )
                self._warned_missing_move_service = True

        desired = {}
        for frame_name, specs in self.objects.items():
            if frame_name in self.suppressed_frames:
                continue

            try:
                tf = self.tf_buffer.lookup_transform(self.world_frame, frame_name, Time())
            except Exception:
                if frame_name not in self._warned_missing_frames:
                    self.get_logger().warn(
                        f"Skipping environment collision frame '{frame_name}' because no TF "
                        f"transform to '{self.world_frame}' is currently available."
                    )
                    self._warned_missing_frames.add(frame_name)
                continue

            self._warned_missing_frames.discard(frame_name)

            frame_pose = Pose()
            frame_pose.position.x = tf.transform.translation.x
            frame_pose.position.y = tf.transform.translation.y
            frame_pose.position.z = tf.transform.translation.z
            frame_pose.orientation = tf.transform.rotation

            for spec in specs:
                world_pose = self._compose_pose(frame_pose, spec["pose"])
                state = self._build_object_state(spec, world_pose)
                desired[spec["name"]] = (spec, frame_pose, state)

        stale_names = set(self.published_signatures) - set(desired)
        if stale_names:
            if self._inflight_operations:
                return
            object_name = sorted(stale_names)[0]
            future = self._remove_object(object_name)
            if future:
                self._inflight_operations.append(
                    {
                        "future": future,
                        "operation": {
                            "type": "remove",
                            "name": object_name,
                            "signature": None,
                        },
                    }
                )
            return

        if self._has_inflight_remove():
            return

        inflight_add_names = self._inflight_add_names()
        available_add_slots = max(0, self.max_concurrent_adds - len(inflight_add_names))
        fallback_changed_name = None
        for object_name, (spec, frame_pose, state) in desired.items():
            if self._states_match(self.published_signatures.get(object_name), state):
                continue
            if object_name in self.published_signatures:
                if move_updates_available:
                    continue
                fallback_changed_name = object_name
                break
            if object_name in inflight_add_names:
                continue
            if available_add_slots <= 0:
                break
            future = self._add_object(spec, frame_pose)
            if future:
                world_pose = self._compose_pose(frame_pose, spec["pose"])
                self._inflight_operations.append(
                    {
                        "future": future,
                        "operation": {
                            "type": "add",
                            "name": object_name,
                            "signature": state,
                            "spec": spec,
                            "world_pose": world_pose,
                        },
                    }
                )
                inflight_add_names.add(object_name)
                available_add_slots -= 1

        if move_updates_available and not self._has_inflight_move():
            move_batch = []
            for object_name, (spec, frame_pose, state) in desired.items():
                if object_name not in self.published_signatures:
                    continue
                if object_name in inflight_add_names:
                    continue
                if self._states_match(self.published_signatures.get(object_name), state):
                    continue
                world_pose = self._compose_pose(frame_pose, spec["pose"])
                move_batch.append(
                    {
                        "name": object_name,
                        "spec": spec,
                        "world_pose": world_pose,
                        "state": state,
                    }
                )
                if len(move_batch) >= self.max_move_batch_size:
                    break

            if move_batch:
                future = self._update_object_poses(move_batch)
                if future:
                    self._inflight_operations.append(
                        {
                            "future": future,
                            "operation": {
                                "type": "move",
                                "name": "__batch__",
                                "updated_objects": {
                                    entry["name"]: {
                                        "state": entry["state"],
                                        "spec": entry["spec"],
                                        "world_pose": entry["world_pose"],
                                    }
                                    for entry in move_batch
                                },
                            },
                        }
                    )
                return

        if fallback_changed_name is not None:
            if self._inflight_operations:
                return
            future = self._remove_object(fallback_changed_name)
            if future:
                self._inflight_operations.append(
                    {
                        "future": future,
                        "operation": {
                            "type": "remove",
                            "name": fallback_changed_name,
                            "signature": None,
                        },
                    }
                )


def main(args=None):
    rclpy.init(args=args)
    node = IsaacCuroboWorldBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
