#!/usr/bin/env python3

"""
Upstream cuMotion action server with collision-object frame correction.
"""

from __future__ import annotations

import copy
import logging
import math
import xml.etree.ElementTree as ET

from curobo.types.math import Pose
from isaac_ros_cumotion.cumotion_planner import CumotionActionServer
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import MoveItErrorCodes
import rclpy
from rclpy.executors import MultiThreadedExecutor
import tf2_ros


class _CuroboWarpCacheWarningFilter(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:
        return "Object already in warp cache, using existing instance for:" not in (
            record.getMessage()
        )


class _DeferredGoalHandle:
    """Proxy goal handle that defers terminal result publication to the wrapper."""

    def __init__(self, goal_handle):
        self._goal_handle = goal_handle

    def __getattr__(self, name):
        return getattr(self._goal_handle, name)

    def succeed(self, response=None):
        del response

    def abort(self, response=None):
        del response


class UpstreamFrameFixCumotionActionServer(CumotionActionServer):
    """Upstream action server with only world-object frame transform override."""

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter("suppress_curobo_warp_cache_warnings", True)
        self.declare_parameter("planning_scene_world_frame", "world")

        suppress_warp_cache_warnings = (
            self.get_parameter("suppress_curobo_warp_cache_warnings")
            .get_parameter_value()
            .bool_value
        )
        if suppress_warp_cache_warnings:
            self._install_curobo_warp_cache_warning_filter()

        planning_scene_world_frame = (
            self.get_parameter("planning_scene_world_frame")
            .get_parameter_value()
            .string_value
        )
        self._planning_scene_world_frame = (
            planning_scene_world_frame if planning_scene_world_frame else "world"
        )
        self._robot_base_frame = getattr(
            self, "_CumotionActionServer__robot_base_frame", "base_link"
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._warned_tf_lookup = False
        self._static_world_to_base_pose = self._parse_urdf_world_to_base()

    @staticmethod
    def _install_curobo_warp_cache_warning_filter() -> None:
        curobo_logger = logging.getLogger("curobo")
        if not any(
            isinstance(existing_filter, _CuroboWarpCacheWarningFilter)
            for existing_filter in curobo_logger.filters
        ):
            curobo_logger.addFilter(_CuroboWarpCacheWarningFilter())

    @staticmethod
    def _rpy_to_quaternion(roll: float, pitch: float, yaw: float):
        cr = math.cos(roll / 2.0)
        sr = math.sin(roll / 2.0)
        cp = math.cos(pitch / 2.0)
        sp = math.sin(pitch / 2.0)
        cy = math.cos(yaw / 2.0)
        sy = math.sin(yaw / 2.0)
        return [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]

    def _parse_urdf_world_to_base(self):
        try:
            urdf_path = self.get_parameter("urdf_path").get_parameter_value().string_value
        except Exception:
            return None

        if urdf_path == "":
            return None

        try:
            root = ET.parse(urdf_path).getroot()
        except Exception as exc:
            self.get_logger().warn(f"Failed to parse URDF at {urdf_path}: {exc}")
            return None

        for joint in root.findall("joint"):
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is None or child is None:
                continue
            if parent.get("link") != self._planning_scene_world_frame:
                continue
            if child.get("link") != self._robot_base_frame:
                continue

            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
            origin = joint.find("origin")
            if origin is not None:
                xyz_str = origin.get("xyz", "").strip()
                rpy_str = origin.get("rpy", "").strip()
                if xyz_str:
                    xyz = [float(v) for v in xyz_str.split()]
                if rpy_str:
                    rpy = [float(v) for v in rpy_str.split()]

            qw, qx, qy, qz = self._rpy_to_quaternion(rpy[0], rpy[1], rpy[2])
            pose_world_base = Pose.from_list([xyz[0], xyz[1], xyz[2], qw, qx, qy, qz])
            return pose_world_base.inverse()

        return None

    def _transform_collision_object_to_base_frame(
        self, obj: CollisionObject
    ) -> CollisionObject:
        frame_id = obj.header.frame_id if obj.header.frame_id else self._planning_scene_world_frame
        if frame_id == self._robot_base_frame:
            return obj

        t_base_source = None
        if (
            frame_id == self._planning_scene_world_frame
            and self._static_world_to_base_pose is not None
        ):
            t_base_source = self._static_world_to_base_pose

        if t_base_source is None:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._robot_base_frame,
                    frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
                t = transform.transform
                t_base_source = Pose.from_list(
                    [
                        t.translation.x,
                        t.translation.y,
                        t.translation.z,
                        t.rotation.w,
                        t.rotation.x,
                        t.rotation.y,
                        t.rotation.z,
                    ]
                )
                self._warned_tf_lookup = False
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as exc:
                if not self._warned_tf_lookup:
                    self.get_logger().error(
                        f"TF lookup '{frame_id}' -> '{self._robot_base_frame}' failed: {exc}"
                    )
                    self._warned_tf_lookup = True
                return obj

        obj_pose = Pose.from_list(
            [
                obj.pose.position.x,
                obj.pose.position.y,
                obj.pose.position.z,
                obj.pose.orientation.w,
                obj.pose.orientation.x,
                obj.pose.orientation.y,
                obj.pose.orientation.z,
            ]
        )
        new_pose = t_base_source.multiply(obj_pose).tolist()

        transformed = copy.deepcopy(obj)
        transformed.pose.position.x = new_pose[0]
        transformed.pose.position.y = new_pose[1]
        transformed.pose.position.z = new_pose[2]
        transformed.pose.orientation.w = new_pose[3]
        transformed.pose.orientation.x = new_pose[4]
        transformed.pose.orientation.y = new_pose[5]
        transformed.pose.orientation.z = new_pose[6]
        transformed.header.frame_id = self._robot_base_frame
        return transformed

    def update_world_objects(self, moveit_objects):
        transformed_objects = [
            self._transform_collision_object_to_base_frame(obj) for obj in moveit_objects
        ]
        return super().update_world_objects(transformed_objects)

    @staticmethod
    def _trajectory_point_count(result) -> int:
        return len(result.planned_trajectory.joint_trajectory.points)

    def _finalize_goal(self, goal_handle, result):
        trajectory_points = self._trajectory_point_count(result)
        if result.error_code.val == MoveItErrorCodes.SUCCESS and trajectory_points > 0:
            goal_handle.succeed(result)
        else:
            self.get_logger().warn(
                "Aborting goal because cuMotion result is incomplete: "
                f"error_code={result.error_code.val}, points={trajectory_points}"
            )
            goal_handle.abort(result)
        return result

    def execute_callback(self, goal_handle):
        deferred_goal_handle = _DeferredGoalHandle(goal_handle)
        result = super().execute_callback(deferred_goal_handle)
        return self._finalize_goal(goal_handle, result)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UpstreamFrameFixCumotionActionServer()
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
