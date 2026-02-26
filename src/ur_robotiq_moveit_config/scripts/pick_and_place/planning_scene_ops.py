"""
Planning-scene helper for grasped-object collision handling.

Responsibilities:
- Control environment collision suppression for grasped world objects.
- Attach/detach grasped collision geometry to/from the end effector.
- Verify object state via /get_planning_scene.
"""

import copy
import time

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import GetPlanningScene
from std_msgs.msg import Bool, String

from .constants import END_EFFECTOR_LINK


def _identity_pose():
    pose = Pose()
    pose.orientation.w = 1.0
    return pose


class PlanningSceneOpsMixin:
    @staticmethod
    def _wait_future_result(future, timeout_sec=0.5, poll_period_sec=0.01):
        """Wait for an async future without spinning a second executor."""
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            if future.done():
                try:
                    return future.result()
                except Exception:
                    return None
            time.sleep(float(poll_period_sec))
        return None

    def _publish_surface_gripper_command(self, close):
        """Publish a Bool command to the Isaac Sim surface gripper action graph."""
        if not hasattr(self, "gripper_command_pub") or self.gripper_command_pub is None:
            self.get_logger().warn(
                "Surface gripper publisher not configured. Skipping /gripper_command publish."
            )
            return

        msg = Bool()
        msg.data = bool(close)
        self.gripper_command_pub.publish(msg)
        self.get_logger().info(
            f"Published /gripper_command={str(msg.data).lower()} for Surface Gripper."
        )

    @staticmethod
    def _quaternion_multiply(q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )

    @staticmethod
    def _quaternion_conjugate(q):
        qx, qy, qz, qw = q
        return (-qx, -qy, -qz, qw)

    def _rotate_vector(self, vec3, quat):
        vx, vy, vz = vec3
        v_quat = (vx, vy, vz, 0.0)
        rotated = self._quaternion_multiply(
            self._quaternion_multiply(quat, v_quat),
            self._quaternion_conjugate(quat),
        )
        return (rotated[0], rotated[1], rotated[2])

    def _compose_pose(self, parent_pose, child_pose):
        p = parent_pose if parent_pose is not None else _identity_pose()
        c = child_pose if child_pose is not None else _identity_pose()

        pqx, pqy, pqz, pqw = self._normalize_quaternion(
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
        )
        cqx, cqy, cqz, cqw = self._normalize_quaternion(
            c.orientation.x, c.orientation.y, c.orientation.z, c.orientation.w
        )

        rotated_child = self._rotate_vector(
            (c.position.x, c.position.y, c.position.z),
            (pqx, pqy, pqz, pqw),
        )
        tx = float(p.position.x) + rotated_child[0]
        ty = float(p.position.y) + rotated_child[1]
        tz = float(p.position.z) + rotated_child[2]
        oqx, oqy, oqz, oqw = self._normalize_quaternion(
            *self._quaternion_multiply((pqx, pqy, pqz, pqw), (cqx, cqy, cqz, cqw))
        )

        pose = Pose()
        pose.position.x = tx
        pose.position.y = ty
        pose.position.z = tz
        pose.orientation.x = oqx
        pose.orientation.y = oqy
        pose.orientation.z = oqz
        pose.orientation.w = oqw
        return pose

    def _query_planning_scene(self, components_mask, timeout_sec=0.5):
        if not self.get_planning_scene_client.wait_for_service(timeout_sec=timeout_sec):
            return None

        req = GetPlanningScene.Request()
        req.components.components = int(components_mask)
        future = self.get_planning_scene_client.call_async(req)
        response = self._wait_future_result(future, timeout_sec=timeout_sec)
        if response is None:
            return None
        return response.scene

    def _get_world_collision_object(self, object_id):
        scene = self._query_planning_scene(
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY, timeout_sec=0.5
        )
        if scene is None:
            return None

        for obj in scene.world.collision_objects:
            if obj.id == object_id and obj.operation != CollisionObject.REMOVE:
                return copy.deepcopy(obj)
        return None

    def _wait_for_world_object_state(self, object_id, present, timeout_sec=1.0):
        """Wait until object appears or disappears in world collision objects."""
        if not self.get_planning_scene_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(
                "Cannot verify world object state: /get_planning_scene is unavailable."
            )
            return True

        end_time = time.monotonic() + float(timeout_sec)
        while time.monotonic() < end_time:
            obj = self._get_world_collision_object(object_id)
            observed_present = obj is not None
            if observed_present == bool(present):
                return True
            self._sleep(0.05)
        return False

    def _set_environment_object_suppressed(
        self, object_id, suppress=True, wait_timeout_sec=1.0
    ):
        """Send suppress/unsuppress command to environment collision publisher."""
        if not hasattr(self, "environment_collision_control_pub"):
            self.get_logger().warn(
                "No environment collision control publisher configured."
            )
            return False

        msg = String()
        msg.data = f"{'suppress' if suppress else 'unsuppress'}:{str(object_id)}"
        self.environment_collision_control_pub.publish(msg)
        self.get_logger().info(
            f"Published environment collision control command: {msg.data}"
        )
        expected_present = not bool(suppress)
        verified = self._wait_for_world_object_state(
            object_id, present=expected_present, timeout_sec=wait_timeout_sec
        )
        if not verified:
            self.get_logger().warn(
                f"Timed out waiting for world object '{object_id}' present={expected_present}."
            )
        return verified

    def _snapshot_world_object_geometry(self, object_id, timeout_sec=1.5):
        """Fetch the latest world collision object geometry for object_id."""
        end_time = time.monotonic() + float(timeout_sec)
        while time.monotonic() < end_time:
            obj = self._get_world_collision_object(object_id)
            if obj is not None and (obj.primitives or obj.meshes):
                return obj
            self._sleep(0.05)
        return None

    def _object_geometry_in_end_effector(self, world_object_snapshot):
        """Transform world-object geometry poses into END_EFFECTOR_LINK frame."""
        if world_object_snapshot is None:
            return None

        source_frame = (
            world_object_snapshot.header.frame_id
            if world_object_snapshot.header.frame_id
            else self.world_frame
        )
        object_pose = world_object_snapshot.pose
        primitives = []
        primitive_poses = []
        meshes = []
        mesh_poses = []

        for idx, primitive in enumerate(world_object_snapshot.primitives):
            local_pose = (
                world_object_snapshot.primitive_poses[idx]
                if idx < len(world_object_snapshot.primitive_poses)
                else _identity_pose()
            )
            world_shape_pose = self._compose_pose(object_pose, local_pose)
            transformed = self._transform_pose_between_frames(
                world_shape_pose.position.x,
                world_shape_pose.position.y,
                world_shape_pose.position.z,
                world_shape_pose.orientation.x,
                world_shape_pose.orientation.y,
                world_shape_pose.orientation.z,
                world_shape_pose.orientation.w,
                source_frame,
                END_EFFECTOR_LINK,
            )
            if transformed is None:
                return None

            tx, ty, tz, tqx, tqy, tqz, tqw = transformed
            pose = Pose()
            pose.position.x = float(tx)
            pose.position.y = float(ty)
            pose.position.z = float(tz)
            pose.orientation.x = float(tqx)
            pose.orientation.y = float(tqy)
            pose.orientation.z = float(tqz)
            pose.orientation.w = float(tqw)

            primitives.append(copy.deepcopy(primitive))
            primitive_poses.append(pose)

        for idx, mesh in enumerate(world_object_snapshot.meshes):
            local_pose = (
                world_object_snapshot.mesh_poses[idx]
                if idx < len(world_object_snapshot.mesh_poses)
                else _identity_pose()
            )
            world_shape_pose = self._compose_pose(object_pose, local_pose)
            transformed = self._transform_pose_between_frames(
                world_shape_pose.position.x,
                world_shape_pose.position.y,
                world_shape_pose.position.z,
                world_shape_pose.orientation.x,
                world_shape_pose.orientation.y,
                world_shape_pose.orientation.z,
                world_shape_pose.orientation.w,
                source_frame,
                END_EFFECTOR_LINK,
            )
            if transformed is None:
                return None

            tx, ty, tz, tqx, tqy, tqz, tqw = transformed
            pose = Pose()
            pose.position.x = float(tx)
            pose.position.y = float(ty)
            pose.position.z = float(tz)
            pose.orientation.x = float(tqx)
            pose.orientation.y = float(tqy)
            pose.orientation.z = float(tqz)
            pose.orientation.w = float(tqw)

            meshes.append(copy.deepcopy(mesh))
            mesh_poses.append(pose)

        return primitives, primitive_poses, meshes, mesh_poses

    def _attach_object_collision(
        self,
        object_id,
        object_world_pose=None,
        world_object_snapshot=None,
        grasp_offset_z=None,
        publish_surface_gripper_command=True,
    ):
        """Attach object_id to the end-effector in MoveIt's planning scene."""
        if self._attached_object_id == object_id:
            return True

        attached_object = CollisionObject()
        attached_object.id = str(object_id)
        attached_object.header.frame_id = END_EFFECTOR_LINK
        attached_object.operation = CollisionObject.ADD

        geometry_added = False
        snapshot = world_object_snapshot
        if snapshot is None:
            snapshot = self._snapshot_world_object_geometry(object_id, timeout_sec=0.5)

        transformed_geometry = self._object_geometry_in_end_effector(snapshot)
        if transformed_geometry is not None:
            primitives, primitive_poses, meshes, mesh_poses = transformed_geometry
            if primitives or meshes:
                attached_object.primitives = primitives
                attached_object.primitive_poses = primitive_poses
                attached_object.meshes = meshes
                attached_object.mesh_poses = mesh_poses
                geometry_added = True

        if not geometry_added:
            self.get_logger().error(
                f"No world geometry snapshot available for '{object_id}'. "
                "Cannot attach object collision geometry."
            )
            return False

        aco = AttachedCollisionObject()
        aco.link_name = END_EFFECTOR_LINK
        aco.object = attached_object
        aco.touch_links = [str(link_name) for link_name in self.gripper_touch_links]

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [aco]

        self.planning_scene_pub.publish(scene)
        if publish_surface_gripper_command:
            self._publish_surface_gripper_command(True)
        self._attached_object_id = str(object_id)
        self._sleep(0.2)
        return True

    def _detach_object_collision(self, object_id=None, publish_surface_gripper_command=True):
        """Detach the current object (or provided object_id) from end-effector."""
        detach_id = str(object_id) if object_id is not None else self._attached_object_id
        if not detach_id:
            return True

        detach_object = CollisionObject()
        detach_object.id = detach_id
        detach_object.operation = CollisionObject.REMOVE

        detach_aco = AttachedCollisionObject()
        detach_aco.link_name = END_EFFECTOR_LINK
        detach_aco.object = detach_object

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [detach_aco]
        self.planning_scene_pub.publish(scene)
        if publish_surface_gripper_command:
            self._publish_surface_gripper_command(False)

        if self._attached_object_id == detach_id:
            self._attached_object_id = None
        self._sleep(0.2)
        return True

    def _wait_for_attached_object_in_planning_scene(self, object_id, timeout_sec=2.0):
        """Check MoveIt's monitored planning scene for an attached object ID."""
        if not self.get_planning_scene_client.wait_for_service(timeout_sec=0.5):
            return False

        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.ROBOT_STATE
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        end_time = time.monotonic() + float(timeout_sec)
        while time.monotonic() < end_time:
            future = self.get_planning_scene_client.call_async(req)
            response = self._wait_future_result(future, timeout_sec=0.5)
            if response is not None:
                attached = response.scene.robot_state.attached_collision_objects
                if any(obj.object.id == object_id for obj in attached):
                    return True
            self._sleep(0.1)
        return False
