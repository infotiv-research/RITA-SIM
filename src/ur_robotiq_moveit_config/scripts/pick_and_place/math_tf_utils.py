"""TF and quaternion helper for pose handling.

Responsibilities:
- Query target-object/tool transforms from TF.
- Transform poses between frames.
- Normalize and compare quaternions.
- Provide target-object-in-tool fallback estimates when TF trees are disconnected.
"""

import math

from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import TransformException

from .constants import END_EFFECTOR_LINK


class MathTfMixin:
    def _lookup_target_object_pose(self):
        """Look up target-object pose in world frame. Returns (x, y, z, qx, qy, qz, qw)."""
        target_frame = str(getattr(self, "target_object_frame", ""))
        if not target_frame:
            self.get_logger().warn(
                "No target object frame configured; cannot perform TF lookup."
            )
            return None

        for attempt in range(10):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    target_frame,
                    Time(),
                    timeout=Duration(seconds=2.0),
                )
                return (
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z,
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z,
                    tf.transform.rotation.w,
                )
            except TransformException as e:
                self.get_logger().warn(f"TF lookup attempt {attempt + 1}/10 failed: {e}")
        return None

    def _lookup_end_effector_orientation(self):
        """Look up current end-effector orientation in world frame."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                END_EFFECTOR_LINK,
                Time(),
                timeout=Duration(seconds=0.5),
            )
            q = tf.transform.rotation
            return (q.x, q.y, q.z, q.w)
        except TransformException as e:
            self.get_logger().warn(
                f"Could not get current {END_EFFECTOR_LINK} orientation for fallback: {e}"
            )
            return None

    @staticmethod
    def _quaternions_equivalent(q1, q2, tol=1e-3):
        """Return True if two quaternions represent the same rotation (q and -q equivalent)."""
        dot = abs(sum(a * b for a, b in zip(q1, q2)))
        return abs(1.0 - dot) < tol

    @staticmethod
    def _normalize_quaternion(qx, qy, qz, qw):
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 1e-9:
            return (0.0, 0.0, 0.0, 1.0)
        inv = 1.0 / norm
        return (qx * inv, qy * inv, qz * inv, qw * inv)

    def _transform_pose_between_frames(
        self, x, y, z, qx, qy, qz, qw, source_frame, target_frame
    ):
        """Transform pose from source_frame to target_frame. Returns tuple or None on failure."""
        nq_x, nq_y, nq_z, nq_w = self._normalize_quaternion(qx, qy, qz, qw)
        if source_frame == target_frame:
            return (x, y, z, nq_x, nq_y, nq_z, nq_w)

        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException as e:
            self.get_logger().warn(
                f"Could not transform pose from '{source_frame}' to '{target_frame}': {e}"
            )
            return None

        input_pose = Pose()
        input_pose.position.x = float(x)
        input_pose.position.y = float(y)
        input_pose.position.z = float(z)
        input_pose.orientation.x = nq_x
        input_pose.orientation.y = nq_y
        input_pose.orientation.z = nq_z
        input_pose.orientation.w = nq_w

        try:
            transformed_pose = do_transform_pose(input_pose, tf)
        except Exception as e:
            self.get_logger().warn(
                f"Pose transform failed from '{source_frame}' to '{target_frame}': {e}"
            )
            return None

        out_qx, out_qy, out_qz, out_qw = self._normalize_quaternion(
            transformed_pose.orientation.x,
            transformed_pose.orientation.y,
            transformed_pose.orientation.z,
            transformed_pose.orientation.w,
        )
        return (
            transformed_pose.position.x,
            transformed_pose.position.y,
            transformed_pose.position.z,
            out_qx,
            out_qy,
            out_qz,
            out_qw,
        )

    def _lookup_target_object_pose_in_end_effector(
        self,
        target_object_world_pose=None,
        grasp_offset_z=None,
    ):
        """Get target-object pose in end-effector frame as (x, y, z, qx, qy, qz, qw)."""
        target_frame = str(getattr(self, "target_object_frame", ""))
        if not target_frame:
            self.get_logger().warn(
                "No target object frame configured; cannot transform pose into end-effector frame."
            )
            return None

        for _ in range(3):
            try:
                tf = self.tf_buffer.lookup_transform(
                    END_EFFECTOR_LINK,
                    target_frame,
                    Time(),
                    timeout=Duration(seconds=0.5),
                )
                return (
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z,
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z,
                    tf.transform.rotation.w,
                )
            except TransformException:
                continue

        if target_object_world_pose is not None:
            transformed = self._transform_pose_between_frames(
                target_object_world_pose[0],
                target_object_world_pose[1],
                target_object_world_pose[2],
                target_object_world_pose[3],
                target_object_world_pose[4],
                target_object_world_pose[5],
                target_object_world_pose[6],
                self.world_frame,
                END_EFFECTOR_LINK,
            )
            if transformed is not None:
                return transformed

        if grasp_offset_z is not None:
            # Fallback when world and robot TF trees are disconnected:
            # assume the executed grasp kept the end-effector aligned with target frame
            # and place the target object along EE -Z at the commanded grasp offset.
            estimated_z = -float(grasp_offset_z)
            self.get_logger().warn(
                "Using grasp-geometry fallback for target-object attachment because TF "
                f"between '{self.world_frame}' and '{END_EFFECTOR_LINK}' is unavailable."
            )
            return (0.0, 0.0, estimated_z, 0.0, 0.0, 0.0, 1.0)

        self.get_logger().warn(
            f"Could not determine target-object pose in '{END_EFFECTOR_LINK}' frame for attachment."
        )
        return None
