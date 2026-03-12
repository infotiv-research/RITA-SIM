import hashlib
import os
import shutil
import subprocess
import tempfile
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from curobo.types.file_path import ContentPath
from curobo.util.xrdf_utils import convert_xrdf_to_curobo
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _load_yaml_file(path):
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    return data or {}


def _deep_update(dst, src):
    for key, value in src.items():
        if isinstance(value, dict) and isinstance(dst.get(key), dict):
            _deep_update(dst[key], value)
        else:
            dst[key] = value
    return dst


def _replace_package_uris(urdf_content):
    package_map = {
        "ur_description": get_package_share_directory("ur_description"),
        "ur_robotiq_description": get_package_share_directory("ur_robotiq_description"),
        "robotiq_description": get_package_share_directory("robotiq_description"),
    }
    resolved = urdf_content
    for package_name, package_path in package_map.items():
        package_uri = Path(package_path).resolve().as_uri().rstrip("/") + "/"
        resolved = resolved.replace(
            f"package://{package_name}/",
            package_uri,
        )
    return resolved


def _generate_runtime_urdf(context):
    xacro_file = os.path.join(
        get_package_share_directory("ur_robotiq_description"),
        "urdf",
        "ur_robotiq_gantry.urdf.xacro",
    )
    ur_type = LaunchConfiguration("ur_type").perform(context)
    joint_limit_params = os.path.join(
        get_package_share_directory("ur_description"),
        "config",
        ur_type,
        "joint_limits.yaml",
    )
    kinematics_params = os.path.join(
        get_package_share_directory("ur_description"),
        "config",
        ur_type,
        "default_kinematics.yaml",
    )
    physical_params = os.path.join(
        get_package_share_directory("ur_description"),
        "config",
        ur_type,
        "physical_parameters.yaml",
    )
    visual_params = os.path.join(
        get_package_share_directory("ur_description"),
        "config",
        ur_type,
        "visual_parameters.yaml",
    )

    xacro_executable = shutil.which("xacro")
    if xacro_executable is None:
        raise RuntimeError("xacro executable not found in PATH")

    xacro_cmd = [
        xacro_executable,
        xacro_file,
        "robot_ip:=0.0.0.0",
        f"joint_limit_params:={joint_limit_params}",
        f"kinematics_params:={kinematics_params}",
        f"physical_params:={physical_params}",
        f"visual_params:={visual_params}",
        "safety_limits:=true",
        "safety_pos_margin:=0.15",
        "safety_k_position:=20",
        "name:=ur",
        f"ur_type:={ur_type}",
        "sim_isaac:=true",
        "isaac_joint_commands:=/isaac_joint_commands",
        "isaac_gripper_joint_commands:=/isaac_joint_gripper",
        "isaac_joint_states:=/joint_states",
        "script_filename:=ros_control.urscript",
        "input_recipe_filename:=rtde_input_recipe.txt",
        "output_recipe_filename:=rtde_output_recipe.txt",
        "prefix:=",
    ]
    urdf_content = subprocess.check_output(xacro_cmd, text=True)
    urdf_content = _replace_package_uris(urdf_content)

    cmd_fingerprint = hashlib.sha256(" ".join(xacro_cmd).encode("utf-8")).hexdigest()[:16]
    urdf_path = os.path.join(tempfile.gettempdir(), f"ur_robotiq_curobo_{cmd_fingerprint}.urdf")
    with open(urdf_path, "w", encoding="utf-8") as handle:
        handle.write(urdf_content)
    return urdf_path


def _generate_runtime_robot_config(xrdf_path, urdf_path, override_path, gripper_collision_buffer):
    content_path = ContentPath(
        robot_xrdf_absolute_path=xrdf_path,
        robot_urdf_absolute_path=urdf_path,
        robot_asset_absolute_path="/",
    )
    config = convert_xrdf_to_curobo(content_path=content_path)
    override_config = _load_yaml_file(override_path)
    _deep_update(config, override_config)

    kinematics = config["robot_cfg"]["kinematics"]
    cspace = kinematics.get("cspace") or {}
    joint_names = list(cspace.get("joint_names") or [])
    if not joint_names:
        raise RuntimeError(
            f"Missing robot_cfg.kinematics.cspace.joint_names in override config: {override_path}"
        )

    # Keep the collision model in sync with the repo XRDF/URDF while applying
    # checked-in controller tuning from the override YAML.
    kinematics["use_usd_kinematics"] = False
    kinematics["asset_root_path"] = "/"
    kinematics["urdf_path"] = urdf_path
    kinematics["extra_links"] = {
        "attached_object": {
            "parent_link_name": "TCP_point",
            "link_name": "attached_object",
            "fixed_transform": [0, 0, 0, 1, 0, 0, 0],
            "joint_type": "FIXED",
            "joint_name": "attach_joint",
        }
    }
    kinematics["extra_collision_spheres"] = {"attached_object": 32}

    # Add attached_object to collision_link_names
    collision_link_names = kinematics.get("collision_link_names") or []
    if "attached_object" not in collision_link_names:
        collision_link_names.append("attached_object")
    kinematics["collision_link_names"] = collision_link_names

    # Merge attached_object self-collision ignore pairs
    self_collision_ignore = dict(kinematics.get("self_collision_ignore") or {})
    robotiq_base_links = [
        "robotiq_base_link",
        "robotiq_140_base_link",
    ]
    attached_ignore_links = [
        "wrist_3_link",
        "tool0",
        *robotiq_base_links,
        "left_outer_knuckle",
        "left_inner_knuckle",
        "left_outer_finger",
        "left_inner_finger",
        "left_inner_finger_pad",
        "right_outer_knuckle",
        "right_inner_knuckle",
        "right_outer_finger",
        "right_inner_finger",
        "right_inner_finger_pad",
    ]
    for link in attached_ignore_links:
        existing = self_collision_ignore.get(link) or []
        if isinstance(existing, str):
            existing = [existing]
        else:
            existing = list(existing)
        if "attached_object" not in existing:
            existing.append("attached_object")
        self_collision_ignore[link] = existing
    kinematics["self_collision_ignore"] = self_collision_ignore

    # Add attached_object self-collision buffer
    self_collision_buffer = dict(kinematics.get("self_collision_buffer") or {})
    self_collision_buffer["attached_object"] = 0.0
    kinematics["self_collision_buffer"] = self_collision_buffer

    kinematics["mesh_link_names"] = [
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
        *robotiq_base_links,
    ]
    gripper_links = [
        *robotiq_base_links,
        "left_outer_finger",
        "left_inner_finger_pad",
        "left_inner_knuckle",
        "left_inner_finger",
        "left_outer_knuckle",
        "right_outer_finger",
        "right_inner_finger_pad",
        "right_inner_knuckle",
        "right_inner_finger",
        "right_outer_knuckle",
    ]
    collision_sphere_buffer = dict(kinematics.get("collision_sphere_buffer", {}) or {})
    for link_name in gripper_links:
        collision_sphere_buffer[link_name] = float(gripper_collision_buffer)
    kinematics["collision_sphere_buffer"] = collision_sphere_buffer

    fingerprint = hashlib.sha256(
        f"{xrdf_path}:{urdf_path}:{override_path}".encode("utf-8")
    ).hexdigest()[:16]
    runtime_yaml = os.path.join(tempfile.gettempdir(), f"ur_robotiq_curobo_{fingerprint}.yaml")
    with open(runtime_yaml, "w", encoding="utf-8") as handle:
        yaml.safe_dump(config, handle, sort_keys=False)
    return runtime_yaml, joint_names


def launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory("ur_robotiq_curobo_config")
    xrdf_path = os.path.join(
        get_package_share_directory("ur_robotiq_moveit_config"),
        "config",
        "ur10e_robotiq_2f_140_gantry.xrdf",
    )
    override_path = os.path.join(
        package_share,
        "config",
        "ur10e_robotiq_gantry_curobo.overrides.yaml",
    )
    runtime_urdf_path = _generate_runtime_urdf(context)
    runtime_yaml, arm_joint_names = _generate_runtime_robot_config(
        xrdf_path,
        runtime_urdf_path,
        override_path,
        float(LaunchConfiguration("gripper_collision_buffer").perform(context)),
    )

    with open(runtime_urdf_path, "r", encoding="utf-8") as handle:
        runtime_urdf_content = handle.read()

    live_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": runtime_urdf_content,
            }
        ],
    )

    curobo_planner = Node(
        package="curobo_ros",
        executable="curobo_trajectory_planner",
        output="screen",
        parameters=[
            {
                "robot_config_file": runtime_yaml,
                "cameras_config_file": "",
                "base_link": "gantry_base_link",
                "world_file": "",
                "max_attempts": int(LaunchConfiguration("max_attempts").perform(context)),
                "timeout": float(LaunchConfiguration("timeout").perform(context)),
                "time_dilation_factor": float(
                    LaunchConfiguration("time_dilation_factor").perform(context)
                ),
                "voxel_size": float(LaunchConfiguration("voxel_size").perform(context)),
                "collision_activation_distance": float(
                    LaunchConfiguration("collision_activation_distance").perform(context)
                ),
            }
        ],
    )

    preview_joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        namespace="preview",
        output="screen",
        parameters=[
            {
                "source_list": ["/trajectory/joint_states"],
            }
        ],
    )

    preview_trajectory_joint_state_source = Node(
        package="ur_robotiq_moveit_config",
        executable="curobo_preview_joint_states.py",
        output="screen",
        parameters=[
            {
                "trajectory_topic": "/trajectory",
                "preview_joint_states_topic": "/trajectory/joint_states",
                "publish_final_state": True,
            }
        ],
    )

    preview_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="preview",
        output="screen",
        parameters=[
            {
                "robot_description": runtime_urdf_content,
                "frame_prefix": "preview/",
            }
        ],
    )

    preview_static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace="preview",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "preview/world"],
    )

    rviz_config = os.path.join(package_share, "rviz", "curobo_minimal.rviz")
    curobo_rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            {
                "max_attempts": LaunchConfiguration("max_attempts").perform(context),
                "timeout": LaunchConfiguration("timeout").perform(context),
                "time_dilation_factor": LaunchConfiguration("time_dilation_factor").perform(context),
                "voxel_size": LaunchConfiguration("voxel_size").perform(context),
                "collision_activation_distance": LaunchConfiguration(
                    "collision_activation_distance"
                ).perform(context),
                "base_link": "gantry_base_link",
                "tool_frame": "TCP_point",
                "controller_action_name": "/joint_trajectory_controller/follow_joint_trajectory",
                "controller_joint_names": list(arm_joint_names),
            }
        ],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    world_bridge = Node(
        package="ur_robotiq_moveit_config",
        executable="curobo_world_bridge.py",
        output="screen",
        parameters=[
            {
                "assets_root": LaunchConfiguration("assets_root"),
                "world_frame": LaunchConfiguration("world_frame"),
                "publish_rate_hz": LaunchConfiguration("environment_publish_rate_hz"),
                "startup_delay_sec": LaunchConfiguration("environment_startup_delay_sec"),
                "environment_collision_padding": LaunchConfiguration("environment_collision_padding"),
                "exclude_collision_objects": LaunchConfiguration("exclude_collision_objects"),
                "mesh_as_cuboid": LaunchConfiguration("curobo_world_mesh_as_cuboid"),
                "max_mesh_geometries_per_object": LaunchConfiguration(
                    "curobo_world_max_mesh_geometries_per_object"
                ),
                "max_concurrent_adds": LaunchConfiguration(
                    "curobo_world_max_concurrent_adds"
                ),
                "use_move_updates": LaunchConfiguration("curobo_world_use_move_updates"),
                "max_move_batch_size": LaunchConfiguration(
                    "curobo_world_max_move_batch_size"
                ),
                "publish_collision_markers": LaunchConfiguration(
                    "visualize_collision_world"
                ),
                "collision_marker_topic": LaunchConfiguration("collision_marker_topic"),
                "collision_marker_alpha": LaunchConfiguration("collision_marker_alpha"),
                "pose_position_tolerance_m": LaunchConfiguration(
                    "environment_pose_position_tolerance_m"
                ),
                "pose_orientation_tolerance_rad": LaunchConfiguration(
                    "environment_pose_orientation_tolerance_rad"
                ),
                "dimensions_tolerance_m": LaunchConfiguration(
                    "environment_dimensions_tolerance_m"
                ),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    collision_setup = Node(
        package="ur_robotiq_moveit_config",
        executable="curobo_collision_setup.py",
        output="screen",
        parameters=[
            {
                "obb_cache": LaunchConfiguration("curobo_collision_cache_obb"),
                "mesh_cache": LaunchConfiguration("curobo_collision_cache_mesh"),
                "blox_cache": LaunchConfiguration("curobo_collision_cache_blox"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("curobo_configure_collision_cache")),
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
                "marker_topic": LaunchConfiguration("robot_collision_sphere_topic"),
                "marker_alpha": LaunchConfiguration("robot_collision_sphere_alpha"),
                "publish_rate_hz": LaunchConfiguration("robot_collision_sphere_rate_hz"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("visualize_robot_collision_spheres")),
    )

    return [
        live_robot_state_publisher,
        curobo_planner,
        preview_trajectory_joint_state_source,
        preview_joint_state_publisher,
        preview_robot_state_publisher,
        preview_static_transform,
        collision_setup,
        live_collision_spheres,
        world_bridge,
        curobo_rviz,
    ]


def generate_launch_description():
    default_assets_root = os.path.abspath(
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur10e"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("world_frame", default_value="world"),
            DeclareLaunchArgument("assets_root", default_value=default_assets_root),
            DeclareLaunchArgument("environment_publish_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("environment_startup_delay_sec", default_value="3.0"),
            DeclareLaunchArgument("environment_collision_padding", default_value="0.000"),
            DeclareLaunchArgument("environment_pose_position_tolerance_m", default_value="0.002"),
            DeclareLaunchArgument(
                "environment_pose_orientation_tolerance_rad", default_value="0.02"
            ),
            DeclareLaunchArgument("environment_dimensions_tolerance_m", default_value="0.001"),
            DeclareLaunchArgument("gripper_collision_buffer", default_value="0.0"),
            DeclareLaunchArgument("exclude_collision_objects", default_value="robot_arm_beam"),
            DeclareLaunchArgument("visualize_collision_world", default_value="true"),
            DeclareLaunchArgument("visualize_robot_collision_spheres", default_value="true"),
            DeclareLaunchArgument(
                "robot_collision_sphere_topic",
                default_value="/curobo_live_collision_spheres",
            ),
            DeclareLaunchArgument("robot_collision_sphere_alpha", default_value="0.5"),
            DeclareLaunchArgument("robot_collision_sphere_rate_hz", default_value="10.0"),
            DeclareLaunchArgument(
                "collision_marker_topic",
                default_value="/curobo_world_collision_markers",
            ),
            DeclareLaunchArgument("collision_marker_alpha", default_value="0.22"),
            DeclareLaunchArgument("curobo_world_mesh_as_cuboid", default_value="true"),
            DeclareLaunchArgument(
                "curobo_world_max_mesh_geometries_per_object", default_value="5"
            ),
            DeclareLaunchArgument("curobo_world_max_concurrent_adds", default_value="8"),
            DeclareLaunchArgument("curobo_world_use_move_updates", default_value="true"),
            DeclareLaunchArgument("curobo_world_max_move_batch_size", default_value="128"),
            DeclareLaunchArgument("curobo_configure_collision_cache", default_value="false"),
            DeclareLaunchArgument("curobo_collision_cache_obb", default_value="100"),
            DeclareLaunchArgument("curobo_collision_cache_mesh", default_value="10"),
            DeclareLaunchArgument("curobo_collision_cache_blox", default_value="10"),
            DeclareLaunchArgument("max_attempts", default_value="12"),
            DeclareLaunchArgument("timeout", default_value="15.0"),
            DeclareLaunchArgument("time_dilation_factor", default_value="0.4"),
            DeclareLaunchArgument("voxel_size", default_value="0.05"),
            DeclareLaunchArgument("collision_activation_distance", default_value="0.005"),
            OpaqueFunction(function=launch_setup),
        ]
    )
