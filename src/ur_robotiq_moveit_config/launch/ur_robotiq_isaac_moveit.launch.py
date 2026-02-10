import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


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

    joy_dev = LaunchConfiguration("joy_dev")
    joy_deadzone = LaunchConfiguration("joy_deadzone")

    teleop_pkg = LaunchConfiguration("teleop_pkg")
    teleop_exe = LaunchConfiguration("teleop_exe")
    isaac_arm_topic = LaunchConfiguration("isaac_arm_topic")
    isaac_gripper_topic = LaunchConfiguration("isaac_gripper_topic")
    servo_out_topic = LaunchConfiguration("servo_out_topic")

    # Packages / files
    ur_description_package = "ur_description"
    ur_robotiq_description_package = "ur_robotiq_description"
    ur_robotiq_description_file = "ur_robotiq.urdf.xacro"
    moveit_config_package = "ur_robotiq_moveit_config"
    moveit_config_file = "ur_robotiq.srdf.xacro"

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "visual_parameters.yaml"]
    )

    # Robot description (UR + Robotiq)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(ur_robotiq_description_package), "urdf", ur_robotiq_description_file]),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=", joint_limit_params, " ",
            "kinematics_params:=", kinematics_params, " ",
            "physical_params:=", physical_params, " ",
            "visual_params:=", visual_params, " ",
            "safety_limits:=", safety_limits, " ",
            "safety_pos_margin:=", safety_pos_margin, " ",
            "safety_k_position:=", safety_k_position, " ",
            "name:=ur", " ",
            "ur_type:=", ur_type, " ",
            "script_filename:=ros_control.urscript", " ",
            "input_recipe_filename:=rtde_input_recipe.txt", " ",
            "output_recipe_filename:=rtde_output_recipe.txt", " ",
            "prefix:=", prefix, " ",
        ]
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

    # Planning pipeline
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(moveit_config_package, "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Controllers config (for planning execution; not used by Isaac teleop)
    controllers_yaml = load_yaml(moveit_config_package, "config/moveit_controllers_isaac.yaml")
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
        "trajectory_execution.allowed_start_tolerance": 0.01,
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

    # move_group (remap to Isaac joint states so TF follows sim)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
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
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            warehouse_ros_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    
    return [move_group_node, rviz_node]


def generate_launch_description():
    declared_arguments = [
        # UR
        DeclareLaunchArgument(
            "ur_type", default_value="ur10e",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20"],
            description="Type/series of used UR robot."
        ),
        DeclareLaunchArgument(
            "use_fake_hardware", default_value="false",
            description="If true, mirror commands to states (fake hardware)."
        ),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
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
        DeclareLaunchArgument("isaac_gripper_topic", default_value="/isaac_joint_commands_robotiq"),
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
