#!/usr/bin/env python3

import os
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformListener


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


def _parse_origin(origin_tag):
    if origin_tag is None:
        return Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(w=1.0))

    xyz = _parse_float_list(origin_tag.get("xyz", "0 0 0"), expected_len=3) or [
        0.0,
        0.0,
        0.0,
    ]
    rpy = _parse_float_list(origin_tag.get("rpy", "0 0 0"), expected_len=3) or [
        0.0,
        0.0,
        0.0,
    ]
    quat = Rotation.from_euler("xyz", rpy).as_quat()
    return Pose(
        position=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
        orientation=Quaternion(
            x=float(quat[0]),
            y=float(quat[1]),
            z=float(quat[2]),
            w=float(quat[3]),
        ),
    )


def parse_obj(filepath, scale):
    """Parse OBJ file into a ROS Mesh."""
    verts, tris = [], []
    with open(filepath, "r", errors="ignore") as f:
        for line in f:
            parts = line.split()
            if not parts:
                continue

            if parts[0] == "v":
                verts.append(
                    Point(
                        x=float(parts[1]) * scale[0],
                        y=float(parts[2]) * scale[1],
                        z=float(parts[3]) * scale[2],
                    )
                )
            elif parts[0] == "f":
                try:
                    idx = []
                    for value in parts[1:]:
                        vertex_idx = int(value.split("/")[0])
                        vertex_idx = (
                            vertex_idx - 1 if vertex_idx > 0 else len(verts) + vertex_idx
                        )
                        idx.append(vertex_idx)

                    for i in range(1, len(idx) - 1):
                        tris.append(
                            MeshTriangle(vertex_indices=[idx[0], idx[i], idx[i + 1]])
                        )
                except ValueError:
                    continue

    if not verts or not tris:
        return None
    return Mesh(vertices=verts, triangles=tris)


class IsaacMoveItPublisher(Node):
    def __init__(self):
        super().__init__("isaac_moveit_publisher")

        assets = self.declare_parameter("assets_root", "assets/isaac_urdf_exports").value
        self.world_frame = self.declare_parameter("world_frame", "world").value
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 30.0).value)
        self.environment_collision_padding = float(
            self.declare_parameter("environment_collision_padding", 0.0).value
        )
        exclude_raw = self.declare_parameter("exclude_collision_objects", "").value
        control_topic = self.declare_parameter(
            "environment_collision_control_topic",
            "/environment_collision/control",
        ).value

        self.assets_root = os.path.abspath(assets)
        self.excluded_ids = self._parse_excluded_ids(exclude_raw)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.control_sub = self.create_subscription(
            String, str(control_topic), self._on_control_command, 10
        )

        # frame_id -> list[("mesh"|"primitive", Mesh|SolidPrimitive, Pose)]
        self.objects = {}
        self.published = set()
        self.suppressed_ids = set()

        self.load_collision_geometry()
        clamped_rate = max(0.1, self.publish_rate_hz)
        self.get_logger().info(
            f"Loaded {len(self.objects)} collision objects from '{self.assets_root}'. "
            f"Publishing at {clamped_rate:.2f} Hz in frame '{self.world_frame}'."
        )
        if self.excluded_ids:
            self.get_logger().info(
                "Excluded collision IDs: " + ", ".join(sorted(self.excluded_ids))
            )

        self.create_timer(1.0 / clamped_rate, self.publish_poses)

    @staticmethod
    def _parse_excluded_ids(raw):
        if isinstance(raw, (list, tuple)):
            return {str(item).strip() for item in raw if str(item).strip()}
        return {part.strip() for part in str(raw).split(",") if part.strip()}

    def _build_primitive(self, geometry_tag):
        box_tag = geometry_tag.find("box")
        if box_tag is not None:
            size = _parse_float_list(box_tag.get("size", ""), expected_len=3)
            if size is None:
                return None
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            pad = max(0.0, float(self.environment_collision_padding))
            primitive.dimensions = [
                max(1e-6, float(size[0]) + 2.0 * pad),
                max(1e-6, float(size[1]) + 2.0 * pad),
                max(1e-6, float(size[2]) + 2.0 * pad),
            ]
            return primitive

        cylinder_tag = geometry_tag.find("cylinder")
        if cylinder_tag is not None:
            try:
                radius = float(cylinder_tag.get("radius", "0"))
                length = float(cylinder_tag.get("length", "0"))
            except ValueError:
                return None
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [length, radius]
            return primitive

        sphere_tag = geometry_tag.find("sphere")
        if sphere_tag is not None:
            try:
                radius = float(sphere_tag.get("radius", "0"))
            except ValueError:
                return None
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.SPHERE
            primitive.dimensions = [radius]
            return primitive

        return None

    def load_collision_geometry(self):
        """Load collision geometry from exported URDF folders."""
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

            frame_name = obj_folder
            urdf_path = os.path.join(folder_path, f"{frame_name}.urdf")
            if not os.path.exists(urdf_path):
                continue

            try:
                root = ET.parse(urdf_path).getroot()
            except Exception as exc:
                self.get_logger().warn(f"Failed to parse URDF '{urdf_path}': {exc}")
                continue

            geometry_entries = []
            for collision_tag in root.findall(".//link/collision"):
                origin_pose = _parse_origin(collision_tag.find("origin"))
                geometry_tag = collision_tag.find("geometry")
                if geometry_tag is None:
                    continue

                primitive = self._build_primitive(geometry_tag)
                if primitive is not None:
                    geometry_entries.append(("primitive", primitive, origin_pose))
                    continue

                mesh_tag = geometry_tag.find("mesh")
                if mesh_tag is None:
                    continue

                mesh_filename = mesh_tag.get("filename", "").split("/")[-1]
                mesh_path = os.path.join(folder_path, "meshes", mesh_filename)
                if not os.path.exists(mesh_path):
                    self.get_logger().warn(f"Missing mesh file: {mesh_path}")
                    continue

                scale = _parse_float_list(mesh_tag.get("scale", "1 1 1"), expected_len=3)
                if scale is None:
                    scale = [1.0, 1.0, 1.0]

                mesh = parse_obj(mesh_path, scale)
                if mesh is not None:
                    geometry_entries.append(("mesh", mesh, origin_pose))

            if geometry_entries:
                self.objects[frame_name] = geometry_entries

    def _publish_remove(self, object_id):
        scene = PlanningScene(is_diff=True)
        remove_object = CollisionObject()
        remove_object.id = str(object_id)
        remove_object.header.frame_id = self.world_frame
        remove_object.operation = CollisionObject.REMOVE
        scene.world.collision_objects = [remove_object]
        self.scene_pub.publish(scene)

    def _on_control_command(self, msg):
        raw = str(msg.data).strip()
        if not raw:
            return

        if raw.lower() == "clear":
            if self.suppressed_ids:
                previously_suppressed = set(self.suppressed_ids)
                self.suppressed_ids.clear()
                for object_id in previously_suppressed:
                    self.published.discard(object_id)
                self.get_logger().info(
                    "Cleared suppressed environment collision IDs."
                )
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
            self.get_logger().warn(f"Ignoring empty collision object ID in command '{raw}'.")
            return

        if action == "suppress":
            if object_id not in self.suppressed_ids:
                self.suppressed_ids.add(object_id)
                self.published.discard(object_id)
                self._publish_remove(object_id)
                self.get_logger().info(f"Suppressed environment collision object '{object_id}'.")
            return

        if action == "unsuppress":
            if object_id in self.suppressed_ids:
                self.suppressed_ids.remove(object_id)
                self.published.discard(object_id)
                self.get_logger().info(
                    f"Unsuppressed environment collision object '{object_id}'."
                )
            return

        self.get_logger().warn(
            f"Unknown control action '{action}' in command '{raw}'."
        )

    def publish_poses(self):
        """Publish current object poses as world collision objects."""
        scene = PlanningScene(is_diff=True)

        for frame_name, collision_data in self.objects.items():
            if frame_name in self.suppressed_ids:
                continue

            try:
                tf = self.tf_buffer.lookup_transform(self.world_frame, frame_name, Time())
            except Exception:
                continue

            is_new = frame_name not in self.published
            obj = CollisionObject()
            obj.id = frame_name
            obj.header.frame_id = self.world_frame
            obj.operation = CollisionObject.ADD if is_new else CollisionObject.MOVE
            obj.pose = Pose(
                position=Point(
                    x=tf.transform.translation.x,
                    y=tf.transform.translation.y,
                    z=tf.transform.translation.z,
                ),
                orientation=tf.transform.rotation,
            )

            if is_new:
                for geom_type, geom_value, geom_pose in collision_data:
                    if geom_type == "mesh":
                        obj.meshes.append(geom_value)
                        obj.mesh_poses.append(geom_pose)
                    else:
                        obj.primitives.append(geom_value)
                        obj.primitive_poses.append(geom_pose)

                self.published.add(frame_name)
                self.get_logger().info(f"Adding [{frame_name}] to planning scene.")

                obj_color = ObjectColor()
                obj_color.id = frame_name
                obj_color.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
                scene.object_colors.append(obj_color)

            scene.world.collision_objects.append(obj)

        if scene.world.collision_objects:
            self.scene_pub.publish(scene)


def main():
    rclpy.init()
    node = IsaacMoveItPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
