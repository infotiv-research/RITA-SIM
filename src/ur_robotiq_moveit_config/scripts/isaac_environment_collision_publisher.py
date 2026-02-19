#!/usr/bin/env python3

import math
import os
import re
import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


@dataclass(frozen=True)
class Pose3:
    xyz: Vector3
    quat: Quaternion


@dataclass(frozen=True)
class BoxCollision:
    name: str
    size: Vector3
    pose: Pose3


def _parse_vec3(text: str, fallback: Vector3 = (0.0, 0.0, 0.0)) -> Vector3:
    values = text.split()
    if len(values) < 3:
        return fallback
    return (float(values[0]), float(values[1]), float(values[2]))


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return (qx, qy, qz, qw)


def _quat_multiply(q1: Quaternion, q2: Quaternion) -> Quaternion:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _quat_conjugate(q: Quaternion) -> Quaternion:
    x, y, z, w = q
    return (-x, -y, -z, w)


def _quat_normalize(q: Quaternion) -> Quaternion:
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def _rotate_vec(q: Quaternion, v: Vector3) -> Vector3:
    qv: Quaternion = (v[0], v[1], v[2], 0.0)
    rotated = _quat_multiply(_quat_multiply(q, qv), _quat_conjugate(q))
    return (rotated[0], rotated[1], rotated[2])


def _compose_pose(parent: Pose3, child: Pose3) -> Pose3:
    rotated = _rotate_vec(parent.quat, child.xyz)
    xyz = (
        parent.xyz[0] + rotated[0],
        parent.xyz[1] + rotated[1],
        parent.xyz[2] + rotated[2],
    )
    quat = _quat_normalize(_quat_multiply(parent.quat, child.quat))
    return Pose3(xyz=xyz, quat=quat)


def _pose_from_sdf_text(text: str) -> Pose3:
    values = text.split()
    if len(values) < 6:
        return Pose3((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    x, y, z, roll, pitch, yaw = [float(v) for v in values[:6]]
    return Pose3((x, y, z), _quat_from_rpy(roll, pitch, yaw))


def _sanitize_id(token: str) -> str:
    out = re.sub(r"[^0-9A-Za-z_]+", "_", token).strip("_")
    return out if out else "instance"


def _vector_add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vector_scale(a: Vector3, s) -> Vector3:
    if isinstance(s, (int, float)):
        return (a[0] * s, a[1] * s, a[2] * s)
    return (a[0] * s[0], a[1] * s[1], a[2] * s[2])


def _vector_mid(min_xyz: Vector3, max_xyz: Vector3) -> Vector3:
    return (
        0.5 * (min_xyz[0] + max_xyz[0]),
        0.5 * (min_xyz[1] + max_xyz[1]),
        0.5 * (min_xyz[2] + max_xyz[2]),
    )


def _vector_sub(max_xyz: Vector3, min_xyz: Vector3) -> Vector3:
    return (
        max_xyz[0] - min_xyz[0],
        max_xyz[1] - min_xyz[1],
        max_xyz[2] - min_xyz[2],
    )


def _dot(a: Vector3, b: Vector3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vector3, b: Vector3) -> Vector3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _normalize(v: Vector3) -> Vector3:
    n = math.sqrt(_dot(v, v))
    if n <= 1e-12:
        return (0.0, 0.0, 1.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def _quat_from_basis(x_axis: Vector3, y_axis: Vector3, z_axis: Vector3) -> Quaternion:
    # Rotation matrix with basis vectors as columns.
    m00, m01, m02 = x_axis[0], y_axis[0], z_axis[0]
    m10, m11, m12 = x_axis[1], y_axis[1], z_axis[1]
    m20, m21, m22 = x_axis[2], y_axis[2], z_axis[2]

    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s

    return _quat_normalize((qx, qy, qz, qw))


def _maybe_binary_stl(blob: bytes) -> bool:
    if len(blob) < 84:
        return False
    tri_count = struct.unpack("<I", blob[80:84])[0]
    return len(blob) == 84 + tri_count * 50


def _stl_points_normals(path: str) -> Tuple[List[Vector3], List[Vector3]]:
    with open(path, "rb") as f:
        blob = f.read()

    if _maybe_binary_stl(blob):
        tri_count = struct.unpack("<I", blob[80:84])[0]
        offset = 84
        xs: List[float] = []
        ys: List[float] = []
        zs: List[float] = []
        normals: List[Vector3] = []
        points: List[Vector3] = []
        for _ in range(tri_count):
            data = blob[offset: offset + 50]
            if len(data) < 50:
                break
            vals = struct.unpack("<12fH", data)
            n = (vals[0], vals[1], vals[2])
            v1 = (vals[3], vals[4], vals[5])
            v2 = (vals[6], vals[7], vals[8])
            v3 = (vals[9], vals[10], vals[11])

            if math.sqrt(_dot(n, n)) < 1e-9:
                n = _cross(_vector_sub(v2, v1), _vector_sub(v3, v1))
            n = _normalize(n)
            normals.append(n)
            points.extend([v1, v2, v3])

            for v in (v1, v2, v3):
                xs.append(v[0])
                ys.append(v[1])
                zs.append(v[2])
            offset += 50
        if not points:
            raise RuntimeError(f"Could not parse STL vertices from: {path}")
        return points, normals

    points = []
    normals = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        current_normal = (0.0, 0.0, 1.0)
        for line in f:
            line = line.strip().lower()
            if line.startswith("facet normal"):
                parts = line.split()
                if len(parts) >= 5:
                    current_normal = _normalize(
                        (float(parts[2]), float(parts[3]), float(parts[4]))
                    )
                continue
            if not line.startswith("vertex "):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
            points.append((x, y, z))
            normals.append(current_normal)

    if not points:
        raise RuntimeError(f"Could not parse STL vertices from: {path}")
    return points, normals


def _stl_bounds(path: str) -> Tuple[Vector3, Vector3]:
    points, _ = _stl_points_normals(path)
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def _stl_box_geometry(path: str) -> Tuple[Vector3, Vector3, Quaternion]:
    points, normals = _stl_points_normals(path)

    # Build representative face directions (sign-agnostic clustering).
    axes: List[Vector3] = []
    counts: List[int] = []
    for n in normals:
        n = _normalize(n)
        matched = False
        for i, axis in enumerate(axes):
            d = _dot(n, axis)
            if abs(d) > 0.999:
                sign = 1.0 if d >= 0.0 else -1.0
                axis_new = _normalize(
                    (
                        axis[0] + sign * n[0],
                        axis[1] + sign * n[1],
                        axis[2] + sign * n[2],
                    )
                )
                axes[i] = axis_new
                counts[i] += 1
                matched = True
                break
        if not matched:
            axes.append(n)
            counts.append(1)

    # Fallback to AABB for degenerate cases.
    if len(axes) < 2:
        min_xyz, max_xyz = _stl_bounds(path)
        center = _vector_mid(min_xyz, max_xyz)
        size = _vector_sub(max_xyz, min_xyz)
        return center, size, (0.0, 0.0, 0.0, 1.0)

    order = sorted(range(len(axes)), key=lambda i: counts[i], reverse=True)
    axes = [axes[i] for i in order]

    x_axis = axes[0]
    y_axis = min(axes[1:], key=lambda v: abs(_dot(x_axis, v)))
    z_axis = _cross(x_axis, y_axis)
    if math.sqrt(_dot(z_axis, z_axis)) < 1e-6:
        min_xyz, max_xyz = _stl_bounds(path)
        center = _vector_mid(min_xyz, max_xyz)
        size = _vector_sub(max_xyz, min_xyz)
        return center, size, (0.0, 0.0, 0.0, 1.0)

    z_axis = _normalize(z_axis)
    y_axis = _normalize(_cross(z_axis, x_axis))
    x_axis = _normalize(x_axis)

    basis = (x_axis, y_axis, z_axis)
    mins = [float("inf"), float("inf"), float("inf")]
    maxs = [float("-inf"), float("-inf"), float("-inf")]
    for p in points:
        for i in range(3):
            proj = _dot(p, basis[i])
            mins[i] = min(mins[i], proj)
            maxs[i] = max(maxs[i], proj)

    center = (0.0, 0.0, 0.0)
    for i in range(3):
        center = _vector_add(center, _vector_scale(basis[i], 0.5 * (mins[i] + maxs[i])))
    size = (
        maxs[0] - mins[0],
        maxs[1] - mins[1],
        maxs[2] - mins[2],
    )
    quat = _quat_from_basis(x_axis, y_axis, z_axis)
    return center, size, quat


def _dae_bounds(path: str) -> Tuple[Vector3, Vector3]:
    tree = ET.parse(path)
    root = tree.getroot()
    ns = ""
    if root.tag.startswith("{"):
        ns = root.tag.split("}")[0] + "}"

    unit_meter = 1.0
    unit = root.find(f"{ns}asset/{ns}unit")
    if unit is not None:
        unit_meter = float(unit.attrib.get("meter", "1.0"))

    xs: List[float] = []
    ys: List[float] = []
    zs: List[float] = []

    for src in root.findall(f".//{ns}source"):
        source_id = src.attrib.get("id", "")
        if "positions" not in source_id:
            continue
        float_array = src.find(f"{ns}float_array")
        if float_array is None or float_array.text is None:
            continue
        values = [float(v) * unit_meter for v in float_array.text.split()]
        for i in range(0, len(values), 3):
            if i + 2 >= len(values):
                break
            xs.append(values[i])
            ys.append(values[i + 1])
            zs.append(values[i + 2])

    if not xs:
        raise RuntimeError(f"Could not parse DAE positions from: {path}")
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def _dae_box_geometry(path: str) -> Tuple[Vector3, Vector3, Quaternion]:
    min_xyz, max_xyz = _dae_bounds(path)
    center = _vector_mid(min_xyz, max_xyz)
    size = _vector_sub(max_xyz, min_xyz)
    return center, size, (0.0, 0.0, 0.0, 1.0)


class IsaacEnvironmentCollisionPublisher(Node):
    def __init__(self) -> None:
        super().__init__("isaac_environment_collision_publisher")

        self.declare_parameter("assets_root", "")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("stacking_crate_frame_prefix", "stacking_crate_isaac")
        self.declare_parameter("flow_rack_frame_hint", "flow_rack")
        self.declare_parameter("robot_arm_beam_frame_hint", "robot_arm_beam")
        self.declare_parameter("enable_stacking_crates", True)
        self.declare_parameter("enable_flow_rack", True)
        self.declare_parameter("enable_robot_arm_beam", True)
        self.declare_parameter("collision_padding", 0.0)
        self.declare_parameter("exclude_collision_objects", "")

        assets_root = self.get_parameter("assets_root").value
        if not assets_root:
            cwd_guess = os.path.abspath(
                os.path.join(os.getcwd(), "assets", "simlan_environment")
            )
            assets_root = cwd_guess
        self.assets_root = os.path.abspath(assets_root)
        self.world_frame = str(self.get_parameter("world_frame").value)

        self.enable_stacking_crates = bool(
            self.get_parameter("enable_stacking_crates").value
        )
        self.enable_flow_rack = bool(self.get_parameter("enable_flow_rack").value)
        self.enable_robot_arm_beam = bool(
            self.get_parameter("enable_robot_arm_beam").value
        )
        self.collision_padding = float(self.get_parameter("collision_padding").value)

        self.stacking_crate_prefix = str(
            self.get_parameter("stacking_crate_frame_prefix").value
        )
        self.flow_rack_hint = str(self.get_parameter("flow_rack_frame_hint").value)
        self.robot_arm_beam_hint = str(
            self.get_parameter("robot_arm_beam_frame_hint").value
        )
        self.exclude_collision_objects = self._parse_csv_set(
            str(self.get_parameter("exclude_collision_objects").value)
        )

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        publish_rate_hz = max(0.2, publish_rate_hz)

        self._mesh_box_cache: Dict[str, Tuple[Vector3, Vector3, Quaternion]] = {}
        self._known_frames: set[str] = set()
        self._published_object_ids: set[str] = set()
        self._warned_missing: Dict[str, int] = {}

        if not os.path.isdir(self.assets_root):
            raise RuntimeError(
                f"Assets root does not exist: {self.assets_root}. "
                "Set parameter `assets_root` to your assets/simlan_environment path."
            )

        self.model_boxes = {
            "stacking_crate_isaac": self._load_model_boxes("stacking_crate_isaac"),
            "flow_rack": self._load_model_boxes("flow_rack"),
            "robot_arm_beam": self._load_model_boxes("robot_arm_beam"),
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(TFMessage, "/tf", self._tf_callback, 100)
        self.create_subscription(TFMessage, "/tf_static", self._tf_callback, 100)

        self.planning_scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_scene)

        self.get_logger().info(
            "Loaded environment collisions: "
            f"stacking_crate_isaac={len(self.model_boxes['stacking_crate_isaac'])} boxes, "
            f"flow_rack={len(self.model_boxes['flow_rack'])} boxes, "
            f"robot_arm_beam={len(self.model_boxes['robot_arm_beam'])} boxes. "
            f"assets_root={self.assets_root}, world_frame={self.world_frame}, "
            f"collision_padding={self.collision_padding:.6f} m"
        )
        if self.exclude_collision_objects:
            self.get_logger().info(
                "Excluding collision objects (exact match): "
                + ", ".join(sorted(self.exclude_collision_objects))
            )

    def _tf_callback(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            if t.child_frame_id:
                self._known_frames.add(t.child_frame_id)
            if t.header.frame_id:
                self._known_frames.add(t.header.frame_id)

    def _mesh_box_geometry(self, path: str) -> Tuple[Vector3, Vector3, Quaternion]:
        cached = self._mesh_box_cache.get(path)
        if cached is not None:
            return cached

        ext = os.path.splitext(path)[1].lower()
        if ext == ".stl":
            box = _stl_box_geometry(path)
        elif ext == ".dae":
            box = _dae_box_geometry(path)
        else:
            raise RuntimeError(f"Unsupported collision mesh format: {path}")

        self._mesh_box_cache[path] = box
        return box

    def _resolve_mesh_path(self, model_name: str, uri: str) -> str:
        mesh_rel = ""
        prefix = f"model://{model_name}/"
        if uri.startswith(prefix):
            mesh_rel = uri[len(prefix):]
        else:
            parts = uri.split("/", 3)
            mesh_rel = parts[-1] if parts else uri
            if "meshes/" not in mesh_rel:
                mesh_rel = os.path.join("meshes", mesh_rel)

        mesh_path = os.path.join(self.assets_root, model_name, "source", mesh_rel)
        return os.path.abspath(mesh_path)

    def _load_model_boxes(self, model_name: str) -> List[BoxCollision]:
        sdf_path = os.path.join(self.assets_root, model_name, "source", "model.sdf")
        if not os.path.isfile(sdf_path):
            raise RuntimeError(f"SDF not found: {sdf_path}")

        tree = ET.parse(sdf_path)
        root = tree.getroot()
        model = root.find("model")
        if model is None:
            raise RuntimeError(f"No <model> tag found in: {sdf_path}")

        model_boxes: List[BoxCollision] = []
        for link in model.findall("link"):
            link_pose = _pose_from_sdf_text(link.findtext("pose", "0 0 0 0 0 0"))
            for collision in link.findall("collision"):
                coll_name = collision.attrib.get("name", "collision")
                coll_pose = _pose_from_sdf_text(collision.findtext("pose", "0 0 0 0 0 0"))

                geometry = collision.find("geometry")
                if geometry is None:
                    continue

                center_local = (0.0, 0.0, 0.0)
                size = None

                box_tag = geometry.find("box")
                mesh_tag = geometry.find("mesh")

                if box_tag is not None:
                    size = _parse_vec3(box_tag.findtext("size", "0 0 0"))
                    center_local = (0.0, 0.0, 0.0)
                elif mesh_tag is not None:
                    uri = mesh_tag.findtext("uri", "")
                    scale = _parse_vec3(mesh_tag.findtext("scale", "1 1 1"), (1.0, 1.0, 1.0))
                    mesh_path = self._resolve_mesh_path(model_name, uri)
                    if not os.path.isfile(mesh_path):
                        raise RuntimeError(f"Collision mesh not found: {mesh_path}")
                    if model_name == "robot_arm_beam":
                        # Beam should be represented as two legs + top beam, not one big box.
                        beam_parts = [
                            ("left_leg", (0.2, 0.2, 1.6), (0.1, -0.1, -0.8)),
                            ("right_leg", (0.2, 0.2, 1.6), (2.5, -0.1, -0.8)),
                            ("top_beam", (2.2, 0.2, 0.2), (1.3, -0.1, -0.1)),
                        ]
                        for part_name, part_size, part_center in beam_parts:
                            part_size_scaled = _vector_scale(part_size, scale)
                            part_center_scaled = _vector_scale(part_center, scale)
                            if min(part_size_scaled) <= 0.0:
                                continue
                            center_rot = _rotate_vec(coll_pose.quat, part_center_scaled)
                            coll_pose_centered = Pose3(
                                xyz=_vector_add(coll_pose.xyz, center_rot),
                                quat=coll_pose.quat,
                            )
                            box_pose = _compose_pose(link_pose, coll_pose_centered)
                            model_boxes.append(
                                BoxCollision(
                                    name=f"{coll_name}_{part_name}",
                                    size=part_size_scaled,
                                    pose=box_pose,
                                )
                            )
                        continue
                    else:
                        mesh_center, mesh_size, mesh_quat = self._mesh_box_geometry(mesh_path)
                        center_local = _vector_scale(mesh_center, scale)
                        size = _vector_scale(mesh_size, scale)
                        coll_pose = Pose3(
                            xyz=coll_pose.xyz,
                            quat=_quat_normalize(_quat_multiply(coll_pose.quat, mesh_quat)),
                        )

                if size is None:
                    continue

                if min(size) <= 0.0:
                    self.get_logger().warn(
                        f"Skipping non-positive collision size for {model_name}/{coll_name}: {size}"
                    )
                    continue

                center_rot = _rotate_vec(coll_pose.quat, center_local)
                coll_pose_centered = Pose3(
                    xyz=_vector_add(coll_pose.xyz, center_rot),
                    quat=coll_pose.quat,
                )
                box_pose = _compose_pose(link_pose, coll_pose_centered)

                model_boxes.append(
                    BoxCollision(
                        name=coll_name,
                        size=size,
                        pose=box_pose,
                    )
                )

        if not model_boxes:
            raise RuntimeError(f"No usable collision boxes parsed from: {sdf_path}")
        return model_boxes

    @staticmethod
    def _canonical_frame(frame: str) -> str:
        for suffix in ("::new_link", "/new_link", ".new_link", "_new_link"):
            if frame.endswith(suffix):
                return frame[: -len(suffix)]
        return frame

    @staticmethod
    def _frame_sort_key(frame: str) -> Tuple[int, int, str]:
        lower = frame.lower()
        is_link = lower.endswith("new_link")
        # Prefer link frames over model/root frames.
        return (0 if is_link else 1, len(frame), frame)

    @staticmethod
    def _parse_csv_set(text: str) -> set[str]:
        items = set()
        for raw in text.split(","):
            token = raw.strip().lower()
            if token:
                items.add(token)
                if token.endswith("_aabb"):
                    items.add(token[: -len("_aabb")])
        return items

    def _is_excluded_collision_object(
        self, frame: str, canonical_frame: str, object_id: str
    ) -> bool:
        if not self.exclude_collision_objects:
            return False
        candidates = {
            frame.lower(),
            canonical_frame.lower(),
            _sanitize_id(frame).lower(),
            _sanitize_id(canonical_frame).lower(),
            object_id.lower(),
            (object_id + "_aabb").lower(),
        }
        return not self.exclude_collision_objects.isdisjoint(candidates)

    def _frames_for_hint(self, hint: str) -> List[str]:
        if not hint:
            return []
        hint_l = hint.lower()
        matches = [f for f in self._known_frames if hint_l in f.lower()]
        grouped: Dict[str, str] = {}
        for frame in matches:
            key = self._canonical_frame(frame)
            prev = grouped.get(key)
            if prev is None or self._frame_sort_key(frame) < self._frame_sort_key(prev):
                grouped[key] = frame
        return sorted(grouped.values(), key=self._frame_sort_key)

    def _lookup_world_pose(self, frame: str) -> Optional[Pose3]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None

        q = (
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        )
        q = _quat_normalize(q)
        xyz = (
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        )
        return Pose3(xyz=xyz, quat=q)

    @staticmethod
    def _to_ros_pose(p: Pose3) -> Pose:
        msg = Pose()
        msg.position.x = p.xyz[0]
        msg.position.y = p.xyz[1]
        msg.position.z = p.xyz[2]
        msg.orientation.x = p.quat[0]
        msg.orientation.y = p.quat[1]
        msg.orientation.z = p.quat[2]
        msg.orientation.w = p.quat[3]
        return msg

    def _build_collision_object(
        self, model_name: str, frame: str, boxes: Sequence[BoxCollision]
    ) -> Optional[CollisionObject]:
        world_from_frame = self._lookup_world_pose(frame)
        if world_from_frame is None:
            return None

        canonical_frame = self._canonical_frame(frame)
        object_id = f"simlan_{model_name}_{_sanitize_id(canonical_frame)}"
        if self._is_excluded_collision_object(frame, canonical_frame, object_id):
            return None

        msg = CollisionObject()
        msg.id = object_id
        msg.header.frame_id = self.world_frame
        msg.operation = CollisionObject.ADD

        pad2 = 2.0 * self.collision_padding
        for box in boxes:
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [
                box.size[0] + pad2,
                box.size[1] + pad2,
                box.size[2] + pad2,
            ]

            world_pose = _compose_pose(world_from_frame, box.pose)
            msg.primitives.append(primitive)
            msg.primitive_poses.append(self._to_ros_pose(world_pose))

        return msg

    def _publish_scene(self) -> None:
        collision_msgs: List[CollisionObject] = []
        current_ids: set[str] = set()

        if self.enable_flow_rack:
            flow_frames = self._frames_for_hint(self.flow_rack_hint)
            if flow_frames:
                msg = self._build_collision_object(
                    "flow_rack", flow_frames[0], self.model_boxes["flow_rack"]
                )
                if msg is not None:
                    collision_msgs.append(msg)
                    current_ids.add(msg.id)

        if self.enable_robot_arm_beam:
            beam_frames = self._frames_for_hint(self.robot_arm_beam_hint)
            if beam_frames:
                msg = self._build_collision_object(
                    "robot_arm_beam", beam_frames[0], self.model_boxes["robot_arm_beam"]
                )
                if msg is not None:
                    collision_msgs.append(msg)
                    current_ids.add(msg.id)

        if self.enable_stacking_crates:
            crate_frames = self._frames_for_hint(self.stacking_crate_prefix)
            if not crate_frames and self.stacking_crate_prefix != "stacking_crate":
                crate_frames = self._frames_for_hint("stacking_crate")
            if not crate_frames:
                crate_frames = self._frames_for_hint("crate")
            for frame in crate_frames:
                msg = self._build_collision_object(
                    "stacking_crate_isaac",
                    frame,
                    self.model_boxes["stacking_crate_isaac"],
                )
                if msg is not None:
                    collision_msgs.append(msg)
                    current_ids.add(msg.id)

        to_remove = self._published_object_ids - current_ids
        for object_id in sorted(to_remove):
            msg = CollisionObject()
            msg.id = object_id
            msg.header.frame_id = self.world_frame
            msg.operation = CollisionObject.REMOVE
            collision_msgs.append(msg)

        if not collision_msgs:
            now_ns = self.get_clock().now().nanoseconds
            last = self._warned_missing.get("no_collision_msgs", 0)
            if now_ns - last > 5_000_000_000:
                self._warned_missing["no_collision_msgs"] = now_ns
                self.get_logger().warn(
                    "No environment collision objects published yet. "
                    "Waiting for matching TF frames."
                )
            return

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = collision_msgs
        self.planning_scene_pub.publish(scene)
        self._published_object_ids = current_ids


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = IsaacEnvironmentCollisionPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
