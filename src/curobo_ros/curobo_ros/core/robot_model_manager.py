import torch
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.types.robot import RobotConfig
from curobo.types.state import JointState
from curobo_msgs.srv import SetLinkCollision
from typing import List, Tuple


class RobotModelManager:
    """
    Manages the robot model, kinematics, and collision geometry.
    Responsible for:
    - Managing CudaRobotModel for kinematics computations
    - Computing robot collision spheres for visualization and masking
    - Providing joint state management
    - Enabling/disabling link collision spheres (shared with all solvers)
    """

    def __init__(self, robot_cfg: RobotConfig, robot, base_link: str, node=None):
        """
        Initialize the robot model manager.

        Args:
            robot_cfg: Robot configuration from cuRobo
            robot: Robot interface object for accessing joint states
            base_link: Robot base frame name
            node: ROS2 node (for logging and service registration)
        """
        self.robot_cfg = robot_cfg
        self.robot = robot
        self.base_link = base_link
        self.node = node

        # Create CUDA robot model for kinematics
        self.kin_model = CudaRobotModel(robot_cfg.kinematics)

        # Device configuration for operations
        self._ops_dtype = torch.float32
        self._device = torch.device('cuda')

    def get_kinematics_state(self, joint_positions):
        """
        Compute the kinematics state from joint positions.

        Args:
            joint_positions: Tensor of joint positions

        Returns:
            Kinematics state containing link poses, spheres, etc.
        """
        return self.kin_model.get_state(joint_positions)

    def get_collision_spheres(self) -> List[List[float]]:
        """
        Get the robot's collision spheres in current configuration.
        Useful for visualization and point cloud masking.

        Returns:
            List of spheres, each as [x, y, z, radius]
        """
        q_js = JointState(
            position=torch.tensor(
                self.robot.get_joint_pose(),
                dtype=self._ops_dtype,
                device=self._device
            ),
            joint_names=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        )

        kinematics_state = self.kin_model.get_state(q_js.position)

        robot_spheres = kinematics_state.link_spheres_tensor.view(-1, 4)
        robot_spheres = robot_spheres.cpu().numpy().tolist()

        return robot_spheres

    def set_link_collision(
        self, link_names: List[str], enabled: bool
    ) -> Tuple[List[str], List[str]]:
        """
        Enable or disable collision spheres for a list of links.

        Modifies link_spheres in-place on the shared KinematicsTensorConfig,
        so the change is immediately visible to all solvers (MotionGen, MPC).

        Returns:
            Tuple[List[str], List[str]]: (applied_links, unknown_links)
        """
        kc = self.kin_model.kinematics_config
        applied, unknown = [], []
        for link in link_names:
            if link not in kc.link_name_to_idx_map:
                unknown.append(link)
            else:
                if enabled:
                    kc.enable_link_spheres(link)
                else:
                    kc.disable_link_spheres(link)
                applied.append(link)
        return applied, unknown

    def set_link_collision_callback(
        self,
        request: SetLinkCollision.Request,
        response: SetLinkCollision.Response,
    ) -> SetLinkCollision.Response:
        """
        ROS service callback — enable or disable collision spheres for a list of links.

        All solvers (MotionGen, MPC) share the same KinematicsTensorConfig, so a
        single in-place modification affects every active solver simultaneously.
        The state persists until an explicit call with the opposite value.
        """
        applied, unknown = self.set_link_collision(list(request.link_names), request.enabled)

        response.applied_links = applied
        response.unknown_links = unknown

        if unknown:
            kc = self.kin_model.kinematics_config
            if self.node:
                self.node.get_logger().warn(
                    f"set_link_collision: unknown links {unknown}. "
                    f"Available: {list(kc.link_name_to_idx_map.keys())}"
                )

        if applied:
            state = "enabled" if request.enabled else "disabled"
            if self.node:
                self.node.get_logger().info(f"Collision spheres {state} for: {applied}")
            response.success = True
            response.message = f"Collision {state} for {applied}"
        else:
            response.success = False
            response.message = f"No valid links found in: {list(request.link_names)}"

        return response

    def get_joint_state(self) -> JointState:
        """
        Get current joint state from robot interface.

        Returns:
            JointState object with current joint positions
        """
        return JointState(
            position=torch.tensor(
                self.robot.get_joint_pose(),
                dtype=self._ops_dtype,
                device=self._device
            ),
            joint_names=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        )

    def attach_object(self, link_name, sphere_data):
        """Attach collision spheres to a robot link for kinematic object tracking.

        Args:
            link_name: Name of the link (e.g. "attached_object")
            sphere_data: Flat list [x,y,z,r, x,y,z,r, ...] in link-local coords

        Returns:
            (success: bool, message: str, sphere_count: int)
        """
        kc = self.kin_model.kinematics_config
        if link_name not in kc.link_name_to_idx_map:
            return False, f"Unknown link: {link_name}", 0

        n_spheres = len(sphere_data) // 4
        if n_spheres == 0:
            return False, "No sphere data provided", 0

        # Get the current sphere slots for this link
        current = kc.get_link_spheres(link_name)
        max_slots = current.shape[0]

        # Build [N, 4] tensor, padding unused slots with radius=-100
        import torch
        sphere_tensor = torch.full(
            (max_slots, 4), -100.0, dtype=self._ops_dtype, device=self._device
        )
        sphere_tensor[:, :3] = 0.0
        actual = min(n_spheres, max_slots)
        for i in range(actual):
            base = i * 4
            sphere_tensor[i, 0] = float(sphere_data[base])
            sphere_tensor[i, 1] = float(sphere_data[base + 1])
            sphere_tensor[i, 2] = float(sphere_data[base + 2])
            sphere_tensor[i, 3] = float(sphere_data[base + 3])

        success = kc.attach_object(sphere_tensor=sphere_tensor, link_name=link_name)
        if success:
            msg = f"Attached {actual} spheres to {link_name}"
            if n_spheres > max_slots:
                msg += f" (truncated from {n_spheres}, max slots={max_slots})"
        else:
            msg = f"cuRobo attach_object failed for {link_name}"
        return success, msg, actual

    def detach_object(self, link_name):
        """Detach collision spheres from a robot link.

        Args:
            link_name: Name of the link (e.g. "attached_object")

        Returns:
            (success: bool, message: str)
        """
        kc = self.kin_model.kinematics_config
        if link_name not in kc.link_name_to_idx_map:
            return False, f"Unknown link: {link_name}"

        success = kc.detach_object(link_name=link_name)
        if success:
            return True, f"Detached spheres from {link_name}"
        return False, f"cuRobo detach_object failed for {link_name}"

    def attach_object_callback(self, request, response):
        """ROS service callback for AttachObject."""
        link_name = request.link_name or "attached_object"
        if request.attach:
            success, message, sphere_count = self.attach_object(
                link_name, list(request.sphere_data)
            )
            response.success = success
            response.message = message
            response.sphere_count = sphere_count
            if self.node:
                if success:
                    self.node.get_logger().info(message)
                else:
                    self.node.get_logger().error(message)
        else:
            success, message = self.detach_object(link_name)
            response.success = success
            response.message = message
            response.sphere_count = 0
            if self.node:
                if success:
                    self.node.get_logger().info(message)
                else:
                    self.node.get_logger().error(message)
        return response
