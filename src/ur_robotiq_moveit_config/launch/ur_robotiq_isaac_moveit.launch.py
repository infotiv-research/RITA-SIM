import os
import hashlib
import shutil
import subprocess
import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def normalize_numeric_param_types(value):
    if isinstance(value, dict):
        return {k: normalize_numeric_param_types(v) for k, v in value.items()}
    if isinstance(value, list):
        return [normalize_numeric_param_types(v) for v in value]
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return float(value)
    return value


GANTRY_DESCRIPTION_FILE = "ur_robotiq_gantry.urdf.xacro"
GANTRY_SRDF_FILE = "ur_robotiq_gantry.srdf.xacro"
GANTRY_MOVEIT_CONTROLLERS_FILE = "config/moveit_controllers_isaac_gantry.yaml"
GANTRY_ROS2_CONTROLLERS_FILE = "config/ur_robotiq_controllers_isaac_gantry.yaml"
GANTRY_MOVEIT_JOINT_LIMITS_FILE = "config/joint_limits_gantry.yaml"
GANTRY_CUMOTION_XRDF_FILE = "config/ur10e_robotiq_2f_140_gantry.xrdf"


def parse_csv_list(raw_value):
    return [token.strip() for token in str(raw_value).split(",") if token.strip()]


def merge_csv_values(raw_value, required_values):
    merged_values = parse_csv_list(raw_value)
    for value in required_values:
        if value not in merged_values:
            merged_values.append(value)
    return ",".join(merged_values)


def generate_runtime_urdf(
    xacro_file,
    ur_type,
    joint_limit_params,
    kinematics_params,
    physical_params,
    visual_params,
    safety_limits,
    safety_pos_margin,
    safety_k_position,
    prefix,
    isaac_arm_topic,
    isaac_gripper_topic,
    isaac_joint_states_topic,
):
    xacro_executable = shutil.which("xacro")
    if xacro_executable is None:
        raise RuntimeError("xacro executable not found in PATH")

    xacro_cmd = [
        xacro_executable,
        xacro_file,
        "robot_ip:=xxx.yyy.zzz.www",
        f"joint_limit_params:={joint_limit_params}",
        f"kinematics_params:={kinematics_params}",
        f"physical_params:={physical_params}",
        f"visual_params:={visual_params}",
        f"safety_limits:={safety_limits}",
        f"safety_pos_margin:={safety_pos_margin}",
        f"safety_k_position:={safety_k_position}",
        "name:=ur",
        f"ur_type:={ur_type}",
        "sim_isaac:=true",
        f"isaac_joint_commands:={isaac_arm_topic}",
        f"isaac_gripper_joint_commands:={isaac_gripper_topic}",
        f"isaac_joint_states:={isaac_joint_states_topic}",
        "script_filename:=ros_control.urscript",
        "input_recipe_filename:=rtde_input_recipe.txt",
        "output_recipe_filename:=rtde_output_recipe.txt",
        f"prefix:={prefix}",
    ]
    urdf_content = subprocess.check_output(xacro_cmd, text=True)

    cmd_fingerprint = hashlib.sha256(" ".join(xacro_cmd).encode("utf-8")).hexdigest()[:16]
    urdf_path = os.path.join(tempfile.gettempdir(), f"ur_robotiq_runtime_{cmd_fingerprint}.urdf")
    with open(urdf_path, "w", encoding="utf-8") as f:
        f.write(urdf_content)

    return urdf_content, urdf_path


def launch_setup(context, *args, **kwargs):
    # Args
    ur_type = LaunchConfiguration("ur_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")

    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")
    planning_pipeline = LaunchConfiguration("planning_pipeline")
    launch_move_group = LaunchConfiguration("launch_move_group")
    capabilities = LaunchConfiguration("capabilities")
    disable_capabilities = LaunchConfiguration("disable_capabilities")
    launch_cumotion_planner = LaunchConfiguration("launch_cumotion_planner")
    cumotion_use_patched_node = LaunchConfiguration("cumotion_use_patched_node")
    collision_cache_cuboid = LaunchConfiguration("collision_cache_cuboid")
    collision_cache_mesh = LaunchConfiguration("collision_cache_mesh")
    cumotion_time_dilation_factor = LaunchConfiguration("cumotion_time_dilation_factor")
    cumotion_override_moveit_scaling_factors = LaunchConfiguration(
        "cumotion_override_moveit_scaling_factors"
    )
    cumotion_max_attempts = LaunchConfiguration("cumotion_max_attempts")
    cumotion_num_graph_seeds = LaunchConfiguration("cumotion_num_graph_seeds")
    cumotion_num_trajopt_seeds = LaunchConfiguration("cumotion_num_trajopt_seeds")
    cumotion_num_trajopt_time_steps = LaunchConfiguration("cumotion_num_trajopt_time_steps")
    cumotion_trajopt_finetune_iters = LaunchConfiguration("cumotion_trajopt_finetune_iters")
    cumotion_interpolation_dt = LaunchConfiguration("cumotion_interpolation_dt")
    cumotion_voxel_size = LaunchConfiguration("cumotion_voxel_size")
    cumotion_publish_curobo_world_as_voxels = LaunchConfiguration(
        "cumotion_publish_curobo_world_as_voxels"
    )
    cumotion_publish_voxel_size = LaunchConfiguration("cumotion_publish_voxel_size")

    joy_dev = LaunchConfiguration("joy_dev")
    joy_deadzone = LaunchConfiguration("joy_deadzone")

    teleop_pkg = LaunchConfiguration("teleop_pkg")
    teleop_exe = LaunchConfiguration("teleop_exe")
    isaac_arm_topic = LaunchConfiguration("isaac_arm_topic")
    isaac_gripper_topic = LaunchConfiguration("isaac_gripper_topic")
    servo_out_topic = LaunchConfiguration("servo_out_topic")
    enable_joint_state_filter = LaunchConfiguration("enable_joint_state_filter")
    raw_joint_states_topic = LaunchConfiguration("raw_joint_states_topic")
    moveit_joint_states_topic = LaunchConfiguration("moveit_joint_states_topic")
    publish_environment_collisions = LaunchConfiguration("publish_environment_collisions")
    environment_collision_world_frame = LaunchConfiguration("environment_collision_world_frame")
    environment_collision_publish_rate_hz = LaunchConfiguration(
        "environment_collision_publish_rate_hz"
    )
    environment_collision_padding = LaunchConfiguration("environment_collision_padding")
    exclude_collision_objects = LaunchConfiguration("exclude_collision_objects")
    simlan_assets_root = LaunchConfiguration("simlan_assets_root")
    static_assets_root = LaunchConfiguration("static_assets_root")

    # Packages / files
    ur_description_package = "ur_description"
    ur_robotiq_description_package = "ur_robotiq_description"
    moveit_config_package = "ur_robotiq_moveit_config"

    ur_type_value = ur_type.perform(context)
    safety_limits_value = safety_limits.perform(context)
    safety_pos_margin_value = safety_pos_margin.perform(context)
    safety_k_position_value = safety_k_position.perform(context)
    prefix_value = prefix.perform(context)
    ur_robotiq_description_file = GANTRY_DESCRIPTION_FILE
    moveit_config_file = GANTRY_SRDF_FILE
    moveit_controllers_file = GANTRY_MOVEIT_CONTROLLERS_FILE
    ros2_controllers_file = GANTRY_ROS2_CONTROLLERS_FILE
    moveit_joint_limits_file = GANTRY_MOVEIT_JOINT_LIMITS_FILE
    selected_cumotion_robot_xrdf = os.path.join(
        get_package_share_directory(moveit_config_package),
        GANTRY_CUMOTION_XRDF_FILE,
    )

    ur_description_share = get_package_share_directory(ur_description_package)
    joint_limit_params_path = os.path.join(
        ur_description_share, "config", ur_type_value, "joint_limits.yaml"
    )
    kinematics_params_path = os.path.join(
        ur_description_share, "config", ur_type_value, "default_kinematics.yaml"
    )
    physical_params_path = os.path.join(
        ur_description_share, "config", ur_type_value, "physical_parameters.yaml"
    )
    visual_params_path = os.path.join(
        ur_description_share, "config", ur_type_value, "visual_parameters.yaml"
    )
    ur_robotiq_description_file_path = os.path.join(
        get_package_share_directory(ur_robotiq_description_package),
        "urdf",
        ur_robotiq_description_file,
    )

    # Generate one URDF from xacro and reuse it across nodes to avoid model drift.
    robot_description_content, runtime_urdf_path = generate_runtime_urdf(
        xacro_file=ur_robotiq_description_file_path,
        ur_type=ur_type_value,
        joint_limit_params=joint_limit_params_path,
        kinematics_params=kinematics_params_path,
        physical_params=physical_params_path,
        visual_params=visual_params_path,
        safety_limits=safety_limits_value,
        safety_pos_margin=safety_pos_margin_value,
        safety_k_position=safety_k_position_value,
        prefix=prefix_value,
        isaac_arm_topic=isaac_arm_topic.perform(context),
        isaac_gripper_topic=isaac_gripper_topic.perform(context),
        isaac_joint_states_topic=raw_joint_states_topic.perform(context),
    )

    robot_description = {"robot_description": robot_description_content}

    # MoveIt semantic + kinematics
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(moveit_config_package), "srdf", moveit_config_file]),
            " ",
            "name:=ur", " ",
            "prefix:=", prefix, " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(moveit_config_package, "config/kinematics.yaml")
    }
    joint_limits_config = load_yaml(moveit_config_package, moveit_joint_limits_file)
    if joint_limits_config is None:
        joint_limits_config = {}
    robot_description_planning = {
        "robot_description_planning": normalize_numeric_param_types(joint_limits_config)
    }

    # Planning pipeline
    ompl_pipeline_config = load_yaml(moveit_config_package, "config/ompl_planning.yaml")
    if ompl_pipeline_config is None:
        raise RuntimeError("Failed to load config/ompl_planning.yaml")
    cumotion_planning_config = load_yaml(
        moveit_config_package, "config/isaac_ros_cumotion_planning.yaml"
    )
    if cumotion_planning_config is None:
        raise RuntimeError("Failed to load config/isaac_ros_cumotion_planning.yaml")
    selected_planning_pipeline = planning_pipeline.perform(context)
    if selected_planning_pipeline == "cumotion":
        planning_pipelines = ["ompl", "isaac_ros_cumotion"]
        default_planning_pipeline = "isaac_ros_cumotion"
    else:
        planning_pipelines = ["ompl"]
        default_planning_pipeline = "ompl"
    planning_pipeline_parameters = {
        "planning_pipelines": planning_pipelines,
        "default_planning_pipeline": default_planning_pipeline,
        "ompl": ompl_pipeline_config,
    }
    if selected_planning_pipeline == "cumotion":
        planning_pipeline_parameters["isaac_ros_cumotion"] = cumotion_planning_config

    # Controllers config (for planning execution; not used by Isaac teleop)
    controllers_yaml = load_yaml(moveit_config_package, moveit_controllers_file)
    isaac_ros2_control_yaml = load_yaml(
        ur_robotiq_description_package, ros2_controllers_file
    )
    available_ros2_controllers = set()
    if isaac_ros2_control_yaml is not None:
        cm_params = (
            isaac_ros2_control_yaml.get("controller_manager", {}).get("ros__parameters", {})
        )
        for controller_name, controller_cfg in cm_params.items():
            if isinstance(controller_cfg, dict) and "type" in controller_cfg:
                available_ros2_controllers.add(controller_name)

    # Keep MoveIt default controller selection aligned with available ros2_control controllers.
    if (
        "joint_trajectory_controller" in available_ros2_controllers
        and "scaled_joint_trajectory_controller" not in available_ros2_controllers
    ):
        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["joint_trajectory_controller"]["default"] = True
    if "robotiq_gripper_joint_trajectory_controller" in available_ros2_controllers:
        controllers_yaml["robotiq_gripper_joint_trajectory_controller"]["default"] = True

    if use_fake_hardware.perform(context) == "true":
        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["joint_trajectory_controller"]["default"] = True
        controllers_yaml["robotiq_gripper_joint_trajectory_controller"]["default"] = True
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.05,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }
    bounded_joint_states_topic = "/moveit_bounded_joint_states"

    # Clamp tiny Isaac joint-state overshoots before MoveIt consumes them.
    # This keeps states like wrist_3=-6.28320 inside the URDF +/-2*pi bounds
    # without wrapping by a full revolution and confusing the trajectory controller.
    moveit_joint_state_bounds_filter_node = Node(
        package="ur_robotiq_moveit_config",
        executable="moveit_joint_state_bounds_filter.py",
        name="moveit_joint_state_bounds_filter",
        condition=IfCondition(enable_joint_state_filter),
        output="screen",
        parameters=[
            {
                "input_topic": raw_joint_states_topic,
                "output_topic": bounded_joint_states_topic,
                "limit_epsilon": 1e-5,
                "max_correction": 1e-2,
            },
            {"use_sim_time": False},
        ],
    )

    # Filter raw Isaac joint states so mimic joints are reconstructed from finger_joint.
    # This avoids malformed gripper poses in MoveIt/RViz when Isaac publishes explicit mimic joints.
    moveit_joint_state_filter_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="moveit_joint_state_filter",
        condition=IfCondition(enable_joint_state_filter),
        arguments=[
            runtime_urdf_path
        ],
        output="screen",
        parameters=[
            {
                "source_list": ["raw_joint_states"],
                "rate": 100.0,
                "publish_default_positions": False,
                # cuMotion rejects empty velocity arrays when it falls back to the
                # current JointState on /moveit_joint_states.
                "publish_default_velocities": True,
                "publish_default_efforts": False,
                "use_mimic_tags": True,
            },
            {"use_sim_time": False},
        ],
        remappings=[
            ("raw_joint_states", bounded_joint_states_topic),
            ("joint_states", moveit_joint_states_topic),
        ],
    )

    # move_group uses filtered joint states for consistent gripper kinematics
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        condition=IfCondition(launch_move_group),
        output="screen",
        remappings=[
            ("joint_states", moveit_joint_states_topic),
            ("/joint_states", moveit_joint_states_topic),
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipeline_parameters,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"capabilities": capabilities},
            {"disable_capabilities": disable_capabilities},
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
        ],
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package), "rviz", "view_robot.rviz"])
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "error"],
        remappings=[
            ("joint_states", moveit_joint_states_topic),
            ("/joint_states", moveit_joint_states_topic),
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_pipeline_parameters,
            robot_description_kinematics,
            robot_description_planning,
            warehouse_ros_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    exclude_collision_objects_value = exclude_collision_objects.perform(context)
    effective_exclude_collision_objects = merge_csv_values(
        exclude_collision_objects_value, ["robot_arm_beam"]
    )

    environment_collision_node = Node(
        package=moveit_config_package,
        executable="isaac_urdf_collision_publisher.py",
        name="isaac_urdf_collision_publisher",
        condition=IfCondition(publish_environment_collisions),
        output="screen",
        parameters=[
            {
                "assets_root": simlan_assets_root,
                "static_assets_root": static_assets_root,
                "world_frame": environment_collision_world_frame,
                "publish_rate_hz": environment_collision_publish_rate_hz,
                "environment_collision_padding": environment_collision_padding,
                "exclude_collision_objects": effective_exclude_collision_objects,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    nodes = [
        moveit_joint_state_bounds_filter_node,
        moveit_joint_state_filter_node,
        move_group_node,
        environment_collision_node,
        rviz_node,
    ]
    launch_cumotion_enabled = launch_cumotion_planner.perform(context).lower() == "true"
    use_patched_cumotion = cumotion_use_patched_node.perform(context).lower() == "true"
    if selected_planning_pipeline == "cumotion" and launch_cumotion_enabled:
        cumotion_common_parameters = [
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "robot": selected_cumotion_robot_xrdf,
                "urdf_path": runtime_urdf_path,
                "collision_cache_cuboid": collision_cache_cuboid,
                "collision_cache_mesh": collision_cache_mesh,
                "joint_states_topic": moveit_joint_states_topic,
                "tool_frame": "TCP_point",
                "time_dilation_factor": cumotion_time_dilation_factor,
                "override_moveit_scaling_factors": cumotion_override_moveit_scaling_factors,
                "max_attempts": cumotion_max_attempts,
                "num_graph_seeds": cumotion_num_graph_seeds,
                "num_trajopt_seeds": cumotion_num_trajopt_seeds,
                "num_trajopt_time_steps": cumotion_num_trajopt_time_steps,
                "trajopt_finetune_iters": cumotion_trajopt_finetune_iters,
                "interpolation_dt": cumotion_interpolation_dt,
                "voxel_size": cumotion_voxel_size,
                "publish_curobo_world_as_voxels": cumotion_publish_curobo_world_as_voxels,
                "publish_voxel_size": cumotion_publish_voxel_size,
            },
            {"use_sim_time": use_sim_time},
        ]
        static_planning_scene_node = Node(
            package="isaac_ros_cumotion",
            executable="static_planning_scene",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
            ],
        )
        cumotion_planner_node_upstream = Node(
            package="isaac_ros_cumotion",
            executable="cumotion_planner_node",
            output="screen",
            arguments=[
                "--ros-args",
                "--log-level",
                "curobo:=error",
            ],
            parameters=cumotion_common_parameters,
        )
        cumotion_planner_node_patched = Node(
            package=moveit_config_package,
            executable="cumotion_planner_upstream_framefix.py",
            output="screen",
            arguments=[
                "--ros-args",
                "--log-level",
                "curobo:=error",
            ],
            parameters=cumotion_common_parameters,
        )
        nodes.append(static_planning_scene_node)
        if use_patched_cumotion:
            nodes.append(cumotion_planner_node_patched)
        else:
            nodes.append(cumotion_planner_node_upstream)
    return nodes


def generate_launch_description():
    default_simlan_assets_root = os.path.abspath(
        os.path.join(
            get_package_share_directory("ur_robotiq_moveit_config"),
            "..",
            "..",
            "..",
            "..",
            "assets",
            "isaac_urdf_exports",
        )
    )

    default_static_assets_root = os.path.abspath(
        os.path.join(
            get_package_share_directory("ur_robotiq_moveit_config"),
            "..",
            "..",
            "..",
            "..",
            "assets",
            "static_collisions",
        )
    )

    declared_arguments = [
        # UR
        DeclareLaunchArgument(
            "ur_type", default_value="ur10e",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20"],
            description="Type/series of used UR robot."
        ),
        DeclareLaunchArgument(
            "use_fake_hardware", default_value="true",
            description="If true, mirror commands to states (fake hardware)."
        ),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        # Default 0 in sim: the URDF <safety_controller> soft limits otherwise
        # shrink wrist joints from ±2π to ±(2π−margin), causing START_STATE_INVALID
        # when cuMotion winds a wrist past ±6.13319 rad. Set >0 only against real hardware.
        DeclareLaunchArgument("safety_pos_margin", default_value="0.0"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),

        # General
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path for warehouse_ros SQLite DB."
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("prefix", default_value='""'),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("launch_servo", default_value="true"),
        DeclareLaunchArgument(
            "planning_pipeline",
            default_value="cumotion",
            choices=["ompl", "cumotion"],
            description="Select MoveIt planning pipeline. Use cumotion in the cumotion container.",
        ),
        DeclareLaunchArgument(
            "launch_move_group",
            default_value="true",
            description="Launch the MoveIt move_group node.",
        ),
        DeclareLaunchArgument(
            "capabilities",
            default_value="",
            description="Space-separated extra move_group capabilities to load.",
        ),
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value="",
            description="Space-separated move_group capabilities to disable.",
        ),
        DeclareLaunchArgument(
            "launch_cumotion_planner",
            default_value="true",
            description="Launch cumotion_planner_node when planning_pipeline:=cumotion.",
        ),
        DeclareLaunchArgument(
            "cumotion_use_patched_node",
            default_value="true",
            description="Use local patched cuMotion action server for better stability under repeated goals.",
        ),
        DeclareLaunchArgument(
            "collision_cache_cuboid",
            default_value="100",
            description="cuMotion cuboid obstacle cache size. Increase if OBB cache errors appear.",
        ),
        DeclareLaunchArgument(
            "collision_cache_mesh",
            default_value="100",
            description="cuMotion mesh obstacle cache size.",
        ),
        DeclareLaunchArgument(
            "cumotion_time_dilation_factor",
            default_value="0.3",
            description="Fallback time dilation for cuMotion when MoveIt scaling is overridden.",
        ),
        DeclareLaunchArgument(
            "cumotion_override_moveit_scaling_factors",
            default_value="false",
            description="Use fixed cuMotion time_dilation_factor instead of MoveIt scaling sliders.",
        ),
        DeclareLaunchArgument(
            "cumotion_max_attempts",
            default_value="28",
            description="Maximum cuMotion planning attempts per query.",
        ),
        DeclareLaunchArgument(
            "cumotion_num_graph_seeds",
            default_value="14",
            description="Number of graph seeds for cuMotion planning.",
        ),
        DeclareLaunchArgument(
            "cumotion_num_trajopt_seeds",
            default_value="10",
            description="Number of trajopt seeds for cuMotion planning.",
        ),
        DeclareLaunchArgument(
            "cumotion_num_trajopt_time_steps",
            default_value="48",
            description="Number of trajectory optimization time steps for cuMotion.",
        ),
        DeclareLaunchArgument(
            "cumotion_trajopt_finetune_iters",
            default_value="360",
            description="Finetuning iterations for cuMotion trajectory optimization.",
        ),
        DeclareLaunchArgument(
            "cumotion_interpolation_dt",
            default_value="0.02",
            description="Interpolation dt for generated trajectories.",
        ),
        DeclareLaunchArgument(
            "cumotion_voxel_size",
            default_value="0.05",
            description="cuMotion voxel collision grid size in meters. Smaller is more accurate but slower.",
        ),
        DeclareLaunchArgument(
            "cumotion_publish_curobo_world_as_voxels",
            default_value="false",
            description="Publish cuMotion's internal occupied voxels as Marker on /curobo/voxels.",
        ),
        DeclareLaunchArgument(
            "cumotion_publish_voxel_size",
            default_value="0.05",
            description="Published debug voxel size for /curobo/voxels Marker topic.",
        ),
        DeclareLaunchArgument(
            "enable_joint_state_filter",
            default_value="true",
            description="Enable filtering/reconstruction of mimic joints for MoveIt/RViz.",
        ),
        DeclareLaunchArgument(
            "raw_joint_states_topic",
            default_value="/joint_states",
            description="Raw joint state stream (typically published by Isaac Sim).",
        ),
        DeclareLaunchArgument(
            "moveit_joint_states_topic",
            default_value="/moveit_joint_states",
            description="Filtered joint state stream consumed by move_group and RViz.",
        ),
        DeclareLaunchArgument(
            "publish_environment_collisions",
            default_value="true",
            description="Publish Isaac environment collisions to MoveIt planning scene.",
        ),
        DeclareLaunchArgument(
            "environment_collision_world_frame",
            default_value="world",
            description="World frame used for planning-scene collision objects.",
        ),
        DeclareLaunchArgument(
            "environment_collision_publish_rate_hz",
            default_value="30.0",
            description="Update rate for environment collision objects.",
        ),
        DeclareLaunchArgument(
            "environment_collision_padding",
            default_value="0.000",
            description="Per-face padding (meters) added to published environment collision boxes.",
        ),
        DeclareLaunchArgument(
            "exclude_collision_objects",
            default_value="",
            description=(
                "Comma-separated exact object/frame names to suppress in environment collisions "
                "(e.g. stacking_crate_upper_1 or simlan_stacking_crate_isaac_stacking_crate_upper_1)."
            ),
        ),
        DeclareLaunchArgument(
            "simlan_assets_root",
            default_value=default_simlan_assets_root,
            description="Path to assets/isaac_urdf_exports directory containing exported URDF models.",
        ),
        DeclareLaunchArgument(
            "static_assets_root",
            default_value=default_static_assets_root,
            description="Path to assets/static_collisions directory containing hand-maintained URDF collision models.",
        ),

        # Xbox / joy
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
        DeclareLaunchArgument("joy_deadzone", default_value="0.1"),

        # Teleop + adapter (override these with your actual package/executable if different)
        DeclareLaunchArgument("teleop_pkg", default_value="ur_robotiq_servo",
                              description="Package that contains the xbox teleop node."),
        DeclareLaunchArgument("teleop_exe", default_value="xbox_control",
                              description="Executable/script name for the telop xbox node."),

        # Topics (match your Isaac Action Graph)
        DeclareLaunchArgument("isaac_arm_topic", default_value="/isaac_joint_commands"),
        DeclareLaunchArgument("isaac_gripper_topic", default_value="/isaac_joint_gripper"),
        DeclareLaunchArgument("servo_out_topic", default_value="/servo_node/joint_position_cmds"),
        DeclareLaunchArgument(
            "servo_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_robotiq_moveit_config"), "config", "ur_robotiq_servo.yaml"]
            ),
            description="Path to Servo YAML (can point to src or install).",
        ),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
