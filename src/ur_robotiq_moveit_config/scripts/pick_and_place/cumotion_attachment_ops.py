"""
cuMotion object-attachment helpers for carried-object collision safety.

Responsibilities:
- Attach a picked target object to cuMotion as dynamic link spheres.
- Detach target-object spheres from cuMotion on release/abort paths.
- Build adaptive sphere geometry from runtime object collision shapes.
"""

import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive

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
    def _quaternion_multiply_local(q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )

    @staticmethod
    def _quaternion_conjugate_local(q):
        qx, qy, qz, qw = q
        return (-qx, -qy, -qz, qw)

    def _rotate_vector_local(self, vec3, quat):
        vx, vy, vz = vec3
        v_quat = (vx, vy, vz, 0.0)
        rotated = self._quaternion_multiply_local(
            self._quaternion_multiply_local(quat, v_quat),
            self._quaternion_conjugate_local(quat),
        )
        return (rotated[0], rotated[1], rotated[2])

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

    def _compose_pose_tuples(self, parent_pose, child_pose):
        px, py, pz, pqx, pqy, pqz, pqw = parent_pose
        cx, cy, cz, cqx, cqy, cqz, cqw = child_pose
        pqx, pqy, pqz, pqw = self._normalize_quaternion(pqx, pqy, pqz, pqw)
        cqx, cqy, cqz, cqw = self._normalize_quaternion(cqx, cqy, cqz, cqw)
        rx, ry, rz = self._rotate_vector_local((cx, cy, cz), (pqx, pqy, pqz, pqw))
        oqx, oqy, oqz, oqw = self._normalize_quaternion(
            *self._quaternion_multiply_local(
                (pqx, pqy, pqz, pqw),
                (cqx, cqy, cqz, cqw),
            )
        )
        return (
            float(px + rx),
            float(py + ry),
            float(pz + rz),
            float(oqx),
            float(oqy),
            float(oqz),
            float(oqw),
        )

    @staticmethod
    def _axis_centers(extent, divisions, center_offset=0.0):
        safe_extent = max(1e-6, float(extent))
        safe_divisions = max(1, int(divisions))
        if safe_divisions == 1:
            return [float(center_offset)], safe_extent
        step = safe_extent / float(safe_divisions)
        start = (-0.5 * safe_extent) + (0.5 * step) + float(center_offset)
        return [float(start + i * step) for i in range(safe_divisions)], float(step)

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

    @staticmethod
    def _cap_grid_counts(nx, ny, nz, max_cells):
        cx = max(1, int(nx))
        cy = max(1, int(ny))
        cz = max(1, int(nz))
        cap = max(1, int(max_cells))
        while (cx * cy * cz) > cap:
            if cx >= cy and cx >= cz and cx > 1:
                cx -= 1
                continue
            if cy >= cx and cy >= cz and cy > 1:
                cy -= 1
                continue
            if cz > 1:
                cz -= 1
                continue
            break
        return cx, cy, cz

    def _attachment_preclamp_budget(self):
        # Build slightly denser than max output so deterministic downsampling still has choices.
        max_spheres = max(1, int(getattr(self, "cumotion_attachment_max_spheres", 32)))
        return max(64, max_spheres * 6)

    def _build_box_local_spheres(
        self, size_xyz, target_diameter, inflation, center_offset=(0.0, 0.0, 0.0)
    ):
        sx = max(1e-6, float(size_xyz[0]))
        sy = max(1e-6, float(size_xyz[1]))
        sz = max(1e-6, float(size_xyz[2]))
        target = max(1e-4, float(target_diameter))
        nx = max(1, int(math.ceil(sx / target)))
        ny = max(1, int(math.ceil(sy / target)))
        nz = max(1, int(math.ceil(sz / target)))
        nx, ny, nz = self._cap_grid_counts(
            nx, ny, nz, self._attachment_preclamp_budget()
        )

        cx, cy, cz = center_offset
        xs, dx = self._axis_centers(sx, nx, center_offset=cx)
        ys, dy = self._axis_centers(sy, ny, center_offset=cy)
        zs, dz = self._axis_centers(sz, nz, center_offset=cz)
        radius = (
            math.sqrt((0.5 * dx) ** 2 + (0.5 * dy) ** 2 + (0.5 * dz) ** 2)
            + float(inflation)
        )
        spheres = []
        for x in xs:
            for y in ys:
                for z in zs:
                    spheres.append((float(x), float(y), float(z), float(radius)))
        return spheres

    def _build_cylinder_local_spheres(
        self, height, radius, target_diameter, inflation
    ):
        h = max(1e-6, float(height))
        r = max(1e-6, float(radius))
        target = max(1e-4, float(target_diameter))
        dx_extent = 2.0 * r
        dy_extent = 2.0 * r
        dz_extent = h
        nx = max(1, int(math.ceil(dx_extent / target)))
        ny = max(1, int(math.ceil(dy_extent / target)))
        nz = max(1, int(math.ceil(dz_extent / target)))
        nx, ny, nz = self._cap_grid_counts(
            nx, ny, nz, self._attachment_preclamp_budget()
        )
        xs, dx = self._axis_centers(dx_extent, nx, center_offset=0.0)
        ys, dy = self._axis_centers(dy_extent, ny, center_offset=0.0)
        zs, dz = self._axis_centers(dz_extent, nz, center_offset=0.0)
        cell_half_diag_xy = math.sqrt((0.5 * dx) ** 2 + (0.5 * dy) ** 2)
        radial_limit = max(0.0, r - cell_half_diag_xy)
        radius_out = (
            math.sqrt((0.5 * dx) ** 2 + (0.5 * dy) ** 2 + (0.5 * dz) ** 2)
            + float(inflation)
        )
        spheres = []
        for x in xs:
            for y in ys:
                if (x * x + y * y) > (radial_limit * radial_limit):
                    continue
                for z in zs:
                    spheres.append((float(x), float(y), float(z), float(radius_out)))
        if not spheres:
            spheres.append((0.0, 0.0, 0.0, float(max(r, 0.5 * h) + float(inflation))))
        return spheres

    def _build_sphere_local_spheres(self, radius, target_diameter, inflation):
        r = max(1e-6, float(radius))
        target = max(1e-4, float(target_diameter))
        diameter = 2.0 * r
        n = max(1, int(math.ceil(diameter / target)))
        n, _, _ = self._cap_grid_counts(n, n, n, self._attachment_preclamp_budget())
        xs, dx = self._axis_centers(diameter, n, center_offset=0.0)
        ys, dy = self._axis_centers(diameter, n, center_offset=0.0)
        zs, dz = self._axis_centers(diameter, n, center_offset=0.0)
        cell_half_diag = math.sqrt((0.5 * dx) ** 2 + (0.5 * dy) ** 2 + (0.5 * dz) ** 2)
        if n == 1 or cell_half_diag >= r:
            return [(0.0, 0.0, 0.0, float(r + float(inflation)))]
        inscribed_limit = max(0.0, r - cell_half_diag)
        radius_out = float(cell_half_diag + float(inflation))
        spheres = []
        for x in xs:
            for y in ys:
                for z in zs:
                    if (x * x + y * y + z * z) <= (inscribed_limit * inscribed_limit):
                        spheres.append((float(x), float(y), float(z), radius_out))
        if not spheres:
            spheres.append((0.0, 0.0, 0.0, float(r + float(inflation))))
        return spheres

    def _transform_local_spheres_to_attach(self, shape_pose_attach, local_spheres):
        px, py, pz, qx, qy, qz, qw = shape_pose_attach
        qx, qy, qz, qw = self._normalize_quaternion(qx, qy, qz, qw)
        transformed = []
        for lx, ly, lz, radius in local_spheres:
            rx, ry, rz = self._rotate_vector_local((lx, ly, lz), (qx, qy, qz, qw))
            transformed.append(
                (
                    float(px + rx),
                    float(py + ry),
                    float(pz + rz),
                    float(radius),
                )
            )
        return transformed

    def _downsample_spheres_deterministic(self, spheres, target_count):
        if len(spheres) <= target_count:
            return spheres
        keep = []
        step = float(len(spheres)) / float(target_count)
        for i in range(int(target_count)):
            keep.append(spheres[int(i * step)])
        return keep

    def _build_target_object_attachment_spheres_from_snapshot_once(
        self, world_object_snapshot, object_pose_attach_frame, target_diameter
    ):
        inflation = max(0.0, float(self.cumotion_object_collision_inflation_m))
        object_pose_attach = tuple(float(v) for v in object_pose_attach_frame)
        spheres = []

        primitive_poses = world_object_snapshot.primitive_poses
        for idx, primitive in enumerate(world_object_snapshot.primitives):
            local_pose_msg = (
                primitive_poses[idx] if idx < len(primitive_poses) else Pose()
            )
            if idx >= len(primitive_poses):
                local_pose_msg.orientation.w = 1.0
            local_pose = self._pose_msg_to_tuple(local_pose_msg)
            primitive_pose_attach = self._compose_pose_tuples(object_pose_attach, local_pose)
            local_spheres = []
            if primitive.type == SolidPrimitive.BOX:
                if len(primitive.dimensions) >= 3:
                    local_spheres = self._build_box_local_spheres(
                        (
                            float(primitive.dimensions[SolidPrimitive.BOX_X]),
                            float(primitive.dimensions[SolidPrimitive.BOX_Y]),
                            float(primitive.dimensions[SolidPrimitive.BOX_Z]),
                        ),
                        target_diameter=target_diameter,
                        inflation=inflation,
                    )
            elif primitive.type == SolidPrimitive.SPHERE:
                if len(primitive.dimensions) >= 1:
                    local_spheres = self._build_sphere_local_spheres(
                        float(primitive.dimensions[SolidPrimitive.SPHERE_RADIUS]),
                        target_diameter=target_diameter,
                        inflation=inflation,
                    )
            elif primitive.type == SolidPrimitive.CYLINDER:
                if len(primitive.dimensions) >= 2:
                    local_spheres = self._build_cylinder_local_spheres(
                        float(primitive.dimensions[SolidPrimitive.CYLINDER_HEIGHT]),
                        float(primitive.dimensions[SolidPrimitive.CYLINDER_RADIUS]),
                        target_diameter=target_diameter,
                        inflation=inflation,
                    )
            if local_spheres:
                spheres.extend(
                    self._transform_local_spheres_to_attach(
                        primitive_pose_attach, local_spheres
                    )
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
            mesh_pose_attach = self._compose_pose_tuples(object_pose_attach, local_pose)
            local_spheres = self._build_box_local_spheres(
                mesh_size,
                target_diameter=target_diameter,
                inflation=inflation,
                center_offset=mesh_center,
            )
            spheres.extend(
                self._transform_local_spheres_to_attach(mesh_pose_attach, local_spheres)
            )

        return spheres

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

        target_diameter = max(
            1e-4, float(self.cumotion_attachment_target_sphere_diameter_m)
        )
        min_spheres = max(1, int(self.cumotion_attachment_min_spheres))
        max_spheres = max(min_spheres, int(self.cumotion_attachment_max_spheres))
        diameter_floor = min(0.005, target_diameter)
        spheres = []

        for _ in range(6):
            spheres = self._build_target_object_attachment_spheres_from_snapshot_once(
                world_object_snapshot=world_object_snapshot,
                object_pose_attach_frame=object_pose_attach_frame,
                target_diameter=target_diameter,
            )
            if len(spheres) >= min_spheres or target_diameter <= diameter_floor:
                break
            target_diameter *= 0.7

        if not spheres:
            self.get_logger().error(
                "Object geometry exists but produced no valid cuMotion attachment spheres."
            )
            return None

        if len(spheres) > max_spheres:
            spheres = self._downsample_spheres_deterministic(spheres, max_spheres)
        elif len(spheres) < min_spheres:
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
