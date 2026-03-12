"""
cuMotion object-attachment helpers for carried-object collision safety.

Responsibilities:
- Attach a picked target object to cuMotion as dynamic link spheres.
- Detach target-object spheres from cuMotion on release/abort paths.
- Build adaptive sphere geometry from runtime object collision shapes.
"""

import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive

from . import sphere_builder
from .constants import END_EFFECTOR_LINK

try:
    from isaac_ros_cumotion_interfaces.action import UpdateLinkSpheres
except ImportError:
    UpdateLinkSpheres = None


class CumotionAttachmentOpsMixin:
    def _init_cumotion_attachment_client(self):
        """Initialize optional cuMotion link-sphere attachment client."""
        self._cumotion_object_attached = False
        self._cumotion_attach_client = None

        if not getattr(self, "_use_cumotion_object_attachment", False):
            return

        if UpdateLinkSpheres is None:
            self.get_logger().error(
                "cuMotion object attachment requested, but "
                "'isaac_ros_cumotion_interfaces' is unavailable in this environment."
            )
            self._use_cumotion_object_attachment = False
            return

        self._cumotion_attach_client = ActionClient(
            self,
            UpdateLinkSpheres,
            str(self.cumotion_attach_action_name),
            callback_group=self.cb_group,
        )

    def _wait_for_cumotion_attachment_server(self, timeout_sec):
        """Wait for planner_attach_object action server when feature is enabled."""
        if not getattr(self, "_use_cumotion_object_attachment", False):
            return True

        if self._cumotion_attach_client is None:
            self.get_logger().error(
                "cuMotion object attachment client is not initialized."
            )
            return False

        return self._cumotion_attach_client.wait_for_server(
            timeout_sec=float(timeout_sec)
        )

    @staticmethod
    def _identity_pose_tuple():
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

    @staticmethod
    def _pose_msg_to_tuple(pose):
        if pose is None:
            return CumotionAttachmentOpsMixin._identity_pose_tuple()
        return (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )

    @staticmethod
    def _mesh_aabb(mesh_msg):
        if mesh_msg is None or not mesh_msg.vertices:
            return None
        xs = [float(v.x) for v in mesh_msg.vertices]
        ys = [float(v.y) for v in mesh_msg.vertices]
        zs = [float(v.z) for v in mesh_msg.vertices]
        min_x = min(xs)
        max_x = max(xs)
        min_y = min(ys)
        max_y = max(ys)
        min_z = min(zs)
        max_z = max(zs)
        return (
            (max_x - min_x, max_y - min_y, max_z - min_z),
            (
                0.5 * (min_x + max_x),
                0.5 * (min_y + max_y),
                0.5 * (min_z + max_z),
            ),
        )

    def _collision_specs_from_world_snapshot(self, world_object_snapshot):
        """Convert runtime world geometry into sphere_builder collision specs."""
        specs = []
        primitive_poses = world_object_snapshot.primitive_poses
        for idx, primitive in enumerate(world_object_snapshot.primitives):
            local_pose_msg = (
                primitive_poses[idx] if idx < len(primitive_poses) else Pose()
            )
            if idx >= len(primitive_poses):
                local_pose_msg.orientation.w = 1.0
            local_pose = self._pose_msg_to_tuple(local_pose_msg)
            if primitive.type == SolidPrimitive.BOX:
                if len(primitive.dimensions) >= 3:
                    specs.append(
                        {
                            "type": "box",
                            "dimensions": (
                                float(primitive.dimensions[SolidPrimitive.BOX_X]),
                                float(primitive.dimensions[SolidPrimitive.BOX_Y]),
                                float(primitive.dimensions[SolidPrimitive.BOX_Z]),
                            ),
                            "pose": local_pose,
                        }
                    )
            elif primitive.type == SolidPrimitive.SPHERE:
                if len(primitive.dimensions) >= 1:
                    specs.append(
                        {
                            "type": "sphere",
                            "dimensions": (
                                float(primitive.dimensions[SolidPrimitive.SPHERE_RADIUS]),
                            ),
                            "pose": local_pose,
                        }
                    )
            elif primitive.type == SolidPrimitive.CYLINDER:
                if len(primitive.dimensions) >= 2:
                    specs.append(
                        {
                            "type": "cylinder",
                            "dimensions": (
                                float(primitive.dimensions[SolidPrimitive.CYLINDER_HEIGHT]),
                                float(primitive.dimensions[SolidPrimitive.CYLINDER_RADIUS]),
                            ),
                            "pose": local_pose,
                        }
                    )

        mesh_poses = world_object_snapshot.mesh_poses
        for idx, mesh in enumerate(world_object_snapshot.meshes):
            mesh_aabb = self._mesh_aabb(mesh)
            if mesh_aabb is None:
                continue
            mesh_size, mesh_center = mesh_aabb
            local_pose_msg = mesh_poses[idx] if idx < len(mesh_poses) else Pose()
            if idx >= len(mesh_poses):
                local_pose_msg.orientation.w = 1.0
            local_pose = self._pose_msg_to_tuple(local_pose_msg)
            mesh_center_pose = (
                float(mesh_center[0]),
                float(mesh_center[1]),
                float(mesh_center[2]),
                0.0,
                0.0,
                0.0,
                1.0,
            )
            specs.append(
                {
                    "type": "box",
                    "dimensions": tuple(float(v) for v in mesh_size),
                    "pose": sphere_builder.compose_pose_tuples(
                        local_pose,
                        mesh_center_pose,
                    ),
                }
            )

        return specs

    def _target_object_pose_in_cumotion_attach_frame(
        self, target_object_world_pose=None, grasp_offset_z=None
    ):
        """Compute target-object pose in the link frame used for cuMotion attachment spheres."""
        object_pose_ee = self._lookup_target_object_pose_in_end_effector(
            target_object_world_pose=target_object_world_pose,
            grasp_offset_z=grasp_offset_z,
        )
        if object_pose_ee is None:
            return None

        target_link = str(self.cumotion_attach_object_link_name)
        if target_link in (END_EFFECTOR_LINK, "attached_object"):
            return object_pose_ee

        ex, ey, ez, eqx, eqy, eqz, eqw = object_pose_ee
        transformed = self._transform_pose_between_frames(
            ex,
            ey,
            ez,
            eqx,
            eqy,
            eqz,
            eqw,
            END_EFFECTOR_LINK,
            target_link,
        )
        if transformed is None:
            self.get_logger().warn(
                f"Could not transform target-object pose from '{END_EFFECTOR_LINK}' to "
                f"'{target_link}' for cuMotion attachment."
            )
        return transformed

    def _build_target_object_attachment_spheres(
        self, object_pose_attach_frame, world_object_snapshot=None
    ):
        """Build dynamic attachment spheres from object geometry snapshot."""
        has_geometry = (
            world_object_snapshot is not None
            and (
                bool(world_object_snapshot.primitives)
                or bool(world_object_snapshot.meshes)
            )
        )
        if not has_geometry:
            self.get_logger().error(
                "No world collision geometry snapshot available for cuMotion attachment."
            )
            return None

        collision_specs = self._collision_specs_from_world_snapshot(
            world_object_snapshot
        )
        if not collision_specs:
            self.get_logger().error(
                "Object geometry exists but produced no valid cuMotion attachment specs."
            )
            return None

        target_diameter = max(
            1e-4, float(self.cumotion_attachment_target_sphere_diameter_m)
        )
        min_spheres = max(1, int(self.cumotion_attachment_min_spheres))
        max_spheres = max(min_spheres, int(self.cumotion_attachment_max_spheres))
        inflation = max(0.0, float(self.cumotion_object_collision_inflation_m))
        spheres = sphere_builder.build_attachment_spheres(
            collision_specs=collision_specs,
            object_pose_in_attach_frame=object_pose_attach_frame,
            target_diameter=target_diameter,
            inflation=inflation,
            min_spheres=min_spheres,
            max_spheres=max_spheres,
        )

        if not spheres:
            self.get_logger().error(
                "Object geometry exists but produced no valid cuMotion attachment spheres."
            )
            return None

        if len(spheres) < min_spheres:
            self.get_logger().warn(
                f"Generated {len(spheres)} cuMotion attachment spheres, below configured minimum "
                f"{min_spheres}. Continuing with available spheres."
            )

        flattened = []
        for x, y, z, radius in spheres:
            flattened.extend([float(x), float(y), float(z), float(radius)])
        return flattened

    def _send_cumotion_attachment_goal(self, attach_object, flattened_sphere_arr, timeout_sec):
        """Call UpdateLinkSpheres action and wait for completion."""
        if self._cumotion_attach_client is None or UpdateLinkSpheres is None:
            return False

        goal = UpdateLinkSpheres.Goal()
        goal.attach_object = bool(attach_object)
        goal.flattened_sphere_arr = [float(v) for v in flattened_sphere_arr]
        goal.object_link_name = str(self.cumotion_attach_object_link_name)

        send_future = self._cumotion_attach_client.send_goal_async(goal)
        goal_handle = self._wait_future_result(send_future, timeout_sec=timeout_sec)
        if goal_handle is None:
            self.get_logger().error(
                "Timed out waiting for cuMotion attachment goal acceptance."
            )
            return False
        if not goal_handle.accepted:
            self.get_logger().error("cuMotion attachment goal was rejected.")
            return False

        result_future = goal_handle.get_result_async()
        result = self._wait_future_result(result_future, timeout_sec=timeout_sec)
        if result is None:
            self.get_logger().error(
                "Timed out waiting for cuMotion attachment action result."
            )
            return False

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"cuMotion attachment action finished with status={int(result.status)}."
            )
            return False

        outcome = ""
        if result.result is not None:
            outcome = str(result.result.outcome)
        if outcome and ("fail" in outcome.lower() or "error" in outcome.lower()):
            self.get_logger().error(
                f"cuMotion attachment action reported failure outcome: {outcome}"
            )
            return False
        return True

    def _attach_object_to_cumotion(
        self,
        target_object_world_pose=None,
        grasp_offset_z=None,
        world_object_snapshot=None,
    ):
        """Attach current target object as dynamic spheres in cuMotion robot model."""
        if not getattr(self, "_use_cumotion_object_attachment", False):
            return True
        if self._cumotion_object_attached:
            return True

        object_pose_attach = self._target_object_pose_in_cumotion_attach_frame(
            target_object_world_pose=target_object_world_pose,
            grasp_offset_z=grasp_offset_z,
        )
        if object_pose_attach is None:
            return False

        flattened = self._build_target_object_attachment_spheres(
            object_pose_attach_frame=object_pose_attach,
            world_object_snapshot=world_object_snapshot,
        )
        if flattened is None:
            return False
        sphere_count = len(flattened) // 4
        object_id = str(getattr(self, "target_object_id", "target_object"))
        self.get_logger().info(
            f"Attaching target object '{object_id}' to cuMotion as "
            f"{sphere_count} spheres on link "
            f"'{self.cumotion_attach_object_link_name}'."
        )
        success = self._send_cumotion_attachment_goal(
            attach_object=True,
            flattened_sphere_arr=flattened,
            timeout_sec=float(self.cumotion_attach_timeout_s),
        )
        if success:
            self._cumotion_object_attached = True
        return success

    def _detach_object_from_cumotion(self, retry_once=False, best_effort=False):
        """Detach dynamic target-object spheres from cuMotion robot model."""
        if not getattr(self, "_use_cumotion_object_attachment", False):
            return True
        if not self._cumotion_object_attached:
            return True

        max_attempts = 2 if bool(retry_once) else 1
        timeout_s = float(self.cumotion_detach_timeout_s)
        for attempt in range(max_attempts):
            success = self._send_cumotion_attachment_goal(
                attach_object=False,
                flattened_sphere_arr=[],
                timeout_sec=timeout_s,
            )
            if success:
                self._cumotion_object_attached = False
                return True
            if attempt + 1 < max_attempts:
                self.get_logger().warn(
                    "cuMotion detach failed, retrying once..."
                )
                time.sleep(0.1)

        if best_effort:
            self.get_logger().warn(
                "Best-effort cuMotion detach failed; continuing abort path."
            )
        else:
            self.get_logger().error("Failed to detach target object from cuMotion.")
        return False
