import importlib.util
import os
import shutil
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def _load_module_from_file(module_name, file_path):
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load launch file: {file_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_base_launch_module():
    base_launch_path = os.path.join(
        get_package_share_directory("ur_robotiq_moveit_config"),
        "launch",
        "ur_robotiq_isaac_moveit.launch.py",
    )
    return _load_module_from_file("ur_robotiq_isaac_moveit_base_launch", base_launch_path)


def _load_curobo_launch_module():
    curobo_launch_path = os.path.join(
        get_package_share_directory("ur_robotiq_curobo_config"),
        "launch",
        "ur_robotiq_curobo.launch.py",
    )
    return _load_module_from_file("ur_robotiq_curobo_launch", curobo_launch_path)


def _normalize_prefix_value(prefix_value):
    if prefix_value in {'""', "''"}:
        return ""
    if len(prefix_value) >= 2 and prefix_value[0] == prefix_value[-1] and prefix_value[0] in {'"', "'"}:
        return prefix_value[1:-1]
    return prefix_value


def _render_semantic_description(moveit_config_package, moveit_config_file, prefix_value):
    xacro_executable = shutil.which("xacro")
    if xacro_executable is None:
        raise RuntimeError("xacro executable not found in PATH")

    srdf_path = os.path.join(
        get_package_share_directory(moveit_config_package),
        "srdf",
        moveit_config_file,
    )
    return subprocess.check_output(
        [
            xacro_executable,
            srdf_path,
            "name:=ur",
            f"prefix:={prefix_value}",
        ],
        text=True,
    )


def _build_curobo_planner_nodes(context):
    """Generate the cuRobo trajectory planner node and world bridge.

    These provide the /unified_planner/* services that the hybrid planning
    C++ plugins depend on.
    """
    curobo_launch_module = _load_curobo_launch_module()

    planner_params_file = LaunchConfiguration("planner_params_file").perform(context)
    planner_defaults = curobo_launch_module._load_ros_parameters(planner_params_file)

    xrdf_path = os.path.join(
        get_package_share_directory("ur_robotiq_moveit_config"),
        "config",
        "ur10e_robotiq_2f_140_gantry.xrdf",
    )
    override_path = os.path.join(
        get_package_share_directory("ur_robotiq_curobo_config"),
        "config",
        "ur10e_robotiq_gantry_curobo.overrides.yaml",
    )
    runtime_urdf_path = curobo_launch_module._generate_runtime_urdf(context)
    gripper_collision_buffer = float(
        LaunchConfiguration("gripper_collision_buffer").perform(context)
    )
    runtime_yaml, arm_joint_names = curobo_launch_module._generate_runtime_robot_config(
        xrdf_path, runtime_urdf_path, override_path, gripper_collision_buffer,
    )

    environment_publish_rate_hz = float(planner_defaults.get("environment_publish_rate_hz", 30.0))
    environment_startup_delay_sec = float(planner_defaults.get("environment_startup_delay_sec", 3.0))
    environment_collision_padding = float(planner_defaults.get("environment_collision_padding", 0.0))
    environment_pose_position_tolerance_m = float(
        planner_defaults.get("environment_pose_position_tolerance_m", 0.002)
    )
    environment_pose_orientation_tolerance_rad = float(
        planner_defaults.get("environment_pose_orientation_tolerance_rad", 0.02)
    )
    environment_dimensions_tolerance_m = float(
        planner_defaults.get("environment_dimensions_tolerance_m", 0.001)
    )
    collision_marker_alpha = float(planner_defaults.get("collision_marker_alpha", 0.22))
    curobo_world_mesh_as_cuboid = bool(planner_defaults.get("curobo_world_mesh_as_cuboid", True))
    curobo_world_max_mesh_geometries_per_object = int(
        planner_defaults.get("curobo_world_max_mesh_geometries_per_object", 5)
    )
    curobo_world_max_concurrent_adds = int(
        planner_defaults.get("curobo_world_max_concurrent_adds", 8)
    )
    curobo_world_use_move_updates = bool(
        planner_defaults.get("curobo_world_use_move_updates", True)
    )
    curobo_world_max_move_batch_size = int(
        planner_defaults.get("curobo_world_max_move_batch_size", 128)
    )
    visualize_collision_world = bool(planner_defaults.get("visualize_collision_world", True))
    robot_collision_sphere_alpha = float(planner_defaults.get("robot_collision_sphere_alpha", 0.5))
    robot_collision_sphere_rate_hz = float(planner_defaults.get("robot_collision_sphere_rate_hz", 10.0))

    curobo_planner = Node(
        package="curobo_ros",
        executable="curobo_trajectory_planner",
        output="screen",
        parameters=[
            planner_params_file,
            {
                "robot_config_file": runtime_yaml,
                "cameras_config_file": "",
                "base_link": "gantry_base_link",
                "world_file": "",
                "controller_action_name": "/joint_trajectory_controller/follow_joint_trajectory",
                "controller_joint_names": list(arm_joint_names),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
    )

    world_bridge = Node(
        package="ur_robotiq_moveit_config",
        executable="curobo_world_bridge.py",
        output="screen",
        parameters=[
            {
                "assets_root": LaunchConfiguration("assets_root"),
                "static_assets_root": LaunchConfiguration("static_assets_root"),
                "world_frame": LaunchConfiguration("world_frame"),
                "publish_rate_hz": environment_publish_rate_hz,
                "startup_delay_sec": environment_startup_delay_sec,
                "environment_collision_padding": environment_collision_padding,
                "exclude_collision_objects": LaunchConfiguration("exclude_collision_objects"),
                "mesh_as_cuboid": curobo_world_mesh_as_cuboid,
                "max_mesh_geometries_per_object": curobo_world_max_mesh_geometries_per_object,
                "max_concurrent_adds": curobo_world_max_concurrent_adds,
                "use_move_updates": curobo_world_use_move_updates,
                "max_move_batch_size": curobo_world_max_move_batch_size,
                "publish_collision_markers": visualize_collision_world,
                "collision_marker_topic": "/curobo_world_collision_markers",
                "collision_marker_alpha": collision_marker_alpha,
                "pose_position_tolerance_m": environment_pose_position_tolerance_m,
                "pose_orientation_tolerance_rad": environment_pose_orientation_tolerance_rad,
                "dimensions_tolerance_m": environment_dimensions_tolerance_m,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
    )

    live_collision_spheres = Node(
        package="ur_robotiq_moveit_config",
        executable="curobo_live_collision_spheres.py",
        output="screen",
        parameters=[
            {
                "robot_config_file": runtime_yaml,
                "joint_state_topic": "/joint_states",
                "joint_names": arm_joint_names,
                "base_link": "gantry_base_link",
                "marker_topic": "/curobo_live_collision_spheres",
                "marker_alpha": robot_collision_sphere_alpha,
                "publish_rate_hz": robot_collision_sphere_rate_hz,
            }
        ],
    )

    return [curobo_planner, world_bridge, live_collision_spheres]


def _build_hybrid_container(context, *_args, **_kwargs):
    base_launch_module = _load_base_launch_module()

    moveit_config_package = "ur_robotiq_moveit_config"
    ur_description_package = "ur_description"
    ur_robotiq_description_package = "ur_robotiq_description"

    ur_type_value = LaunchConfiguration("ur_type").perform(context)
    safety_limits_value = LaunchConfiguration("safety_limits").perform(context)
    safety_pos_margin_value = LaunchConfiguration("safety_pos_margin").perform(context)
    safety_k_position_value = LaunchConfiguration("safety_k_position").perform(context)
    prefix_value = _normalize_prefix_value(LaunchConfiguration("prefix").perform(context))
    isaac_arm_topic_value = LaunchConfiguration("isaac_arm_topic").perform(context)
    isaac_gripper_topic_value = LaunchConfiguration("isaac_gripper_topic").perform(context)
    raw_joint_states_topic_value = LaunchConfiguration("raw_joint_states_topic").perform(context)
    use_sim_time_value = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"

    ur_description_share = get_package_share_directory(ur_description_package)
    joint_limit_params_path = os.path.join(
        ur_description_share,
        "config",
        ur_type_value,
        "joint_limits.yaml",
    )
    kinematics_params_path = os.path.join(
        ur_description_share,
        "config",
        ur_type_value,
        "default_kinematics.yaml",
    )
    physical_params_path = os.path.join(
        ur_description_share,
        "config",
        ur_type_value,
        "physical_parameters.yaml",
    )
    visual_params_path = os.path.join(
        ur_description_share,
        "config",
        ur_type_value,
        "visual_parameters.yaml",
    )
    urdf_xacro_path = os.path.join(
        get_package_share_directory(ur_robotiq_description_package),
        "urdf",
        base_launch_module.GANTRY_DESCRIPTION_FILE,
    )

    robot_description_content, _runtime_urdf_path = base_launch_module.generate_runtime_urdf(
        xacro_file=urdf_xacro_path,
        ur_type=ur_type_value,
        joint_limit_params=joint_limit_params_path,
        kinematics_params=kinematics_params_path,
        physical_params=physical_params_path,
        visual_params=visual_params_path,
        safety_limits=safety_limits_value,
        safety_pos_margin=safety_pos_margin_value,
        safety_k_position=safety_k_position_value,
        prefix=prefix_value,
        isaac_arm_topic=isaac_arm_topic_value,
        isaac_gripper_topic=isaac_gripper_topic_value,
        isaac_joint_states_topic=raw_joint_states_topic_value,
    )
    robot_description = {"robot_description": robot_description_content}
    robot_description_semantic = {
        "robot_description_semantic": _render_semantic_description(
            moveit_config_package,
            base_launch_module.GANTRY_SRDF_FILE,
            prefix_value,
        )
    }
    robot_description_kinematics = {
        "robot_description_kinematics": base_launch_module.load_yaml(
            moveit_config_package,
            "config/kinematics.yaml",
        )
    }
    joint_limits_config = base_launch_module.load_yaml(
        moveit_config_package,
        base_launch_module.GANTRY_MOVEIT_JOINT_LIMITS_FILE,
    )
    if joint_limits_config is None:
        joint_limits_config = {}
    robot_description_planning = {
        "robot_description_planning": base_launch_module.normalize_numeric_param_types(joint_limits_config)
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    shared_moveit_parameters = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        planning_scene_monitor_parameters,
        {"use_sim_time": use_sim_time_value},
    ]

    global_planner_parameters = base_launch_module.load_yaml(
        "curobo_hybrid_planning_plugins",
        "config/global_planner.yaml",
    )
    local_planner_parameters = base_launch_module.load_yaml(
        "curobo_hybrid_planning_plugins",
        "config/local_planner.yaml",
    )
    manager_parameters = base_launch_module.load_yaml(
        "curobo_hybrid_planning_plugins",
        "config/hybrid_planning_manager.yaml",
    )

    hybrid_container = ComposableNodeContainer(
        name="curobo_hybrid_planning_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::GlobalPlannerComponent",
                name="global_planner_component",
                parameters=[
                    global_planner_parameters,
                    *shared_moveit_parameters,
                ],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::LocalPlannerComponent",
                name="local_planner_component",
                parameters=[
                    local_planner_parameters,
                    *shared_moveit_parameters,
                ],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::HybridPlanningManager",
                name="hybrid_planning_manager",
                parameters=[
                    manager_parameters,
                ],
            ),
        ],
    )

    curobo_nodes = _build_curobo_planner_nodes(context)
    return curobo_nodes + [hybrid_container]


def generate_launch_description():
    default_assets_root = os.path.abspath(
        os.path.join(
            get_package_share_directory("ur_robotiq_moveit_config"),
            "..", "..", "..", "..",
            "assets", "isaac_urdf_exports",
        )
    )
    default_static_assets_root = os.path.abspath(
        os.path.join(
            get_package_share_directory("ur_robotiq_moveit_config"),
            "..", "..", "..", "..",
            "assets", "static_collisions",
        )
    )
    default_planner_params = os.path.join(
        get_package_share_directory("ur_robotiq_curobo_config"),
        "config",
        "curobo_hybrid_planner_params.yaml",
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robotiq_moveit_config"), "launch", "ur_robotiq_isaac_moveit_human.launch.py"]
            )
        ),
        launch_arguments={
            "launch_move_group": LaunchConfiguration("launch_move_group"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_servo": LaunchConfiguration("launch_servo"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "planning_pipeline": LaunchConfiguration("planning_pipeline"),
            "publish_environment_collisions": LaunchConfiguration("publish_environment_collisions"),
            "publish_human_collisions": LaunchConfiguration("publish_human_collisions"),
            "ur_type": LaunchConfiguration("ur_type"),
            "safety_limits": LaunchConfiguration("safety_limits"),
            "safety_pos_margin": LaunchConfiguration("safety_pos_margin"),
            "safety_k_position": LaunchConfiguration("safety_k_position"),
            "prefix": LaunchConfiguration("prefix"),
            "isaac_arm_topic": LaunchConfiguration("isaac_arm_topic"),
            "isaac_gripper_topic": LaunchConfiguration("isaac_gripper_topic"),
            "raw_joint_states_topic": LaunchConfiguration("raw_joint_states_topic"),
            "moveit_joint_states_topic": LaunchConfiguration("moveit_joint_states_topic"),
            "disable_capabilities": "move_group/MoveGroupMoveAction",
        }.items(),
    )
    hybrid_move_action_bridge = Node(
        package="ur_robotiq_moveit_config",
        executable="hybrid_move_action_bridge.py",
        name="hybrid_move_action_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "move_action_name": "/move_action",
                "hybrid_action_name": "/run_hybrid_planning",
                "motion_plan_service_name": "/plan_kinematic_path",
                "global_solution_topic": "/global_trajectory",
                "default_planning_group": "ur_manipulator",
                "server_wait_timeout_sec": 5.0,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_move_group", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("launch_servo", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("planning_pipeline", default_value="ompl"),
            DeclareLaunchArgument("publish_environment_collisions", default_value="true"),
            DeclareLaunchArgument("publish_human_collisions", default_value="true"),
            DeclareLaunchArgument("ur_type", default_value="ur10e"),
            DeclareLaunchArgument("safety_limits", default_value="true"),
            DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
            DeclareLaunchArgument("safety_k_position", default_value="20"),
            DeclareLaunchArgument("prefix", default_value='""'),
            DeclareLaunchArgument("isaac_arm_topic", default_value="/isaac_joint_commands"),
            DeclareLaunchArgument("isaac_gripper_topic", default_value="/isaac_joint_gripper"),
            DeclareLaunchArgument("raw_joint_states_topic", default_value="/joint_states"),
            DeclareLaunchArgument("moveit_joint_states_topic", default_value="/moveit_joint_states"),
            # cuRobo planner arguments
            DeclareLaunchArgument("planner_params_file", default_value=default_planner_params),
            DeclareLaunchArgument("world_frame", default_value="world"),
            DeclareLaunchArgument("assets_root", default_value=default_assets_root),
            DeclareLaunchArgument("static_assets_root", default_value=default_static_assets_root),
            DeclareLaunchArgument("gripper_collision_buffer", default_value="0.0"),
            DeclareLaunchArgument("exclude_collision_objects", default_value="robot_arm_beam"),
            moveit_launch,
            hybrid_move_action_bridge,
            OpaqueFunction(function=_build_hybrid_container),
        ]
    )
