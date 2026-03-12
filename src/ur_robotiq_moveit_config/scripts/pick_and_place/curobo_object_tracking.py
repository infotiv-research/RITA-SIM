"""Kinematic object attachment tracking for the curobo_ros backend.

Uses cuRobo's native kinematic attachment via the AttachObject service
so carried-object collision spheres move with the robot link.
"""

import os
import time

from visualization_msgs.msg import Marker, MarkerArray

try:
    from curobo_msgs.srv import AttachObject, SetLinkCollision
except ImportError:
    AttachObject = None
    SetLinkCollision = None

from . import sphere_builder


class CuroboObjectTracker:
    def __init__(self, node):
        self.node = node
        self._attached = False
        self._collisions_disabled = False
        self._marker_topic = "/curobo_attached_object_collision_spheres"
        self._marker_namespace = "curobo_attached_object_collision_spheres"
        self._marker_pub = self.node.create_publisher(
            MarkerArray,
            self._marker_topic,
            10,
        )

        self._attach_client = (
            self.node.create_client(AttachObject, "/unified_planner/attach_object")
            if AttachObject is not None
            else None
        )

    def _publish_delete_markers(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        self._marker_pub.publish(marker_array)

    def _publish_attachment_markers(self, sphere_data):
        marker_array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        now = self.node.get_clock().now().to_msg()
        sphere_values = list(sphere_data or [])
        marker_id = 0
        for index in range(0, len(sphere_values), 4):
            if index + 3 >= len(sphere_values):
                break
            x = float(sphere_values[index])
            y = float(sphere_values[index + 1])
            z = float(sphere_values[index + 2])
            radius = float(sphere_values[index + 3])
            if radius <= 0.0:
                continue

            marker = Marker()
            marker.header.frame_id = "TCP_point"
            marker.header.stamp = now
            marker.ns = self._marker_namespace
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.frame_locked = True
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.scale.x = radius * 2.0
            marker.scale.y = radius * 2.0
            marker.scale.z = radius * 2.0
            marker.color.a = 0.65
            marker.color.r = 0.1
            marker.color.g = 1.0
            marker.color.b = 0.25
            marker_array.markers.append(marker)
            marker_id += 1

        self._marker_pub.publish(marker_array)

    @staticmethod
    def _wait_future_result(future, timeout_sec=5.0, poll_period_sec=0.01):
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            if future.done():
                try:
                    return future.result()
                except Exception:
                    return None
            time.sleep(float(poll_period_sec))
        return None

    def _set_gripper_link_collisions(self, enabled):
        if SetLinkCollision is None:
            return False
        if not self.node.curobo_set_link_collision_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn("curobo set_link_collision service unavailable.")
            return False

        request = SetLinkCollision.Request()
        request.link_names = list(self.node.gripper_touch_links)
        request.enabled = bool(enabled)
        future = self.node.curobo_set_link_collision_client.call_async(request)
        response = self._wait_future_result(future, timeout_sec=5.0)
        if response is None or not response.success:
            self.node.get_logger().warn(
                f"Failed to set gripper link collisions enabled={bool(enabled)}."
            )
            return False
        return True

    def _load_collision_specs(self):
        """Load collision specs from target object URDF."""
        assets_root = getattr(self.node, "_curobo_attachment_assets_root", None)
        if assets_root is None:
            # Fall back to the world bridge assets_root parameter if available
            try:
                assets_root = str(
                    self.node.get_parameter("assets_root").get_parameter_value().string_value
                )
            except Exception:
                pass

        if not assets_root:
            self.node.get_logger().warn(
                "No assets_root configured for curobo object attachment URDF lookup."
            )
            return []

        object_id = str(self.node.target_object_id)
        urdf_path = os.path.join(assets_root, object_id, f"{object_id}.urdf")
        if not os.path.exists(urdf_path):
            self.node.get_logger().warn(
                f"No URDF found at '{urdf_path}' for kinematic attachment. "
                "Falling back to simple sphere."
            )
            return []

        specs = sphere_builder.load_collision_specs_from_urdf(urdf_path)
        if not specs:
            self.node.get_logger().warn(
                f"URDF '{urdf_path}' has no parseable collision geometry."
            )
        return specs

    def _build_sphere_data(self, object_pose_in_ee):
        """Build flattened sphere data for the AttachObject service."""
        collision_specs = self._load_collision_specs()

        target_diameter = float(
            getattr(self.node, "cumotion_attachment_target_sphere_diameter_m", 0.03)
        )
        inflation = float(
            getattr(self.node, "cumotion_object_collision_inflation_m", 0.008)
        )
        min_spheres = int(
            getattr(self.node, "cumotion_attachment_min_spheres", 8)
        )
        max_spheres = int(
            getattr(self.node, "cumotion_attachment_max_spheres", 32)
        )

        if collision_specs:
            spheres = sphere_builder.build_attachment_spheres(
                collision_specs=collision_specs,
                object_pose_in_attach_frame=object_pose_in_ee,
                target_diameter=target_diameter,
                inflation=inflation,
                min_spheres=min_spheres,
                max_spheres=max_spheres,
            )
            if spheres:
                flattened = []
                for x, y, z, r in spheres:
                    flattened.extend([float(x), float(y), float(z), float(r)])
                return flattened

        # Fallback: single sphere at object center
        self.node.get_logger().info(
            "Using single-sphere fallback for kinematic attachment."
        )
        radius = 0.5 * target_diameter + inflation
        x, y, z = float(object_pose_in_ee[0]), float(object_pose_in_ee[1]), float(object_pose_in_ee[2])
        return [x, y, z, radius]

    def attach(self, object_pose_world, object_pose_in_ee=None):
        """Attach target object as kinematic collision spheres on the robot link.

        Args:
            object_pose_world: 7-tuple (x, y, z, qx, qy, qz, qw) in world frame.
            object_pose_in_ee: Optional 7-tuple in end-effector frame. If None,
                computed via TF lookup.
        """
        if AttachObject is None:
            self.node.get_logger().error(
                "curobo_msgs.srv.AttachObject unavailable; cannot attach object."
            )
            return False
        if self._attach_client is None:
            self.node.get_logger().error("AttachObject client not initialized.")
            return False
        if not self._attach_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error("AttachObject service unavailable.")
            return False

        # Compute object pose in end-effector frame if not provided
        if object_pose_in_ee is None:
            object_pose_in_ee = self.node._lookup_target_object_pose_in_end_effector(
                target_object_world_pose=object_pose_world,
            )
            if object_pose_in_ee is None:
                self.node.get_logger().error(
                    "Cannot determine object pose in EE frame for kinematic attachment."
                )
                return False

        # Disable gripper link collisions to avoid self-collision with carried object
        if not self._collisions_disabled:
            self._collisions_disabled = self._set_gripper_link_collisions(enabled=False)

        # Build sphere data
        sphere_data = self._build_sphere_data(object_pose_in_ee)

        # Call AttachObject service
        request = AttachObject.Request()
        request.link_name = "attached_object"
        request.attach = True
        request.sphere_data = [float(v) for v in sphere_data]

        future = self._attach_client.call_async(request)
        response = self._wait_future_result(future, timeout_sec=5.0)
        if response is None or not response.success:
            msg = getattr(response, "message", "no response") if response else "timeout"
            self.node.get_logger().error(
                f"AttachObject service call failed: {msg}"
            )
            return False

        sphere_count = response.sphere_count
        self._attached = True
        self._publish_attachment_markers(sphere_data)
        self.node.get_logger().info(
            f"Kinematic attachment active: {sphere_count} spheres on 'attached_object' link."
        )
        return True

    def refresh(self, target_object_world_pose):
        """No-op: kinematic spheres move automatically with the robot link."""
        return True

    def detach(self, best_effort=False, restore_collisions=True):
        """Detach kinematic collision spheres from the robot link."""
        if not self._attached:
            return True

        if AttachObject is None or self._attach_client is None:
            if not best_effort:
                self.node.get_logger().error("AttachObject client unavailable for detach.")
            return best_effort

        if not self._attach_client.wait_for_service(timeout_sec=2.0):
            if not best_effort:
                self.node.get_logger().error("AttachObject service unavailable for detach.")
            return best_effort

        request = AttachObject.Request()
        request.link_name = "attached_object"
        request.attach = False
        request.sphere_data = []

        future = self._attach_client.call_async(request)
        response = self._wait_future_result(future, timeout_sec=5.0)
        success = response is not None and response.success

        if success:
            self._attached = False
            self._publish_delete_markers()
            self.node.get_logger().info("Kinematic attachment detached.")
        elif not best_effort:
            self.node.get_logger().error("Failed to detach kinematic attachment.")

        if restore_collisions and self._collisions_disabled:
            self._set_gripper_link_collisions(enabled=True)
            self._collisions_disabled = False

        if best_effort and not success:
            self._publish_delete_markers()

        return success or best_effort
