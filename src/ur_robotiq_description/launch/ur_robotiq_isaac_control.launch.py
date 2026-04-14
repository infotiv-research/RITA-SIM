from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

GANTRY_DESCRIPTION_FILE = "ur_robotiq_gantry.urdf.xacro"
GANTRY_CONTROLLERS_FILE = "ur_robotiq_controllers_isaac_gantry.yaml"


def controller_spawner(name, active=True):
    args = [
        name,
        "--controller-manager",
        "/controller_manager",
        "--controller-manager-timeout",
        LaunchConfiguration("controller_spawner_timeout"),
    ]
    if not active:
        args.append("--inactive")
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=args,
        output="screen",
    )


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    description_package = LaunchConfiguration("description_package")
    ur_description_package = LaunchConfiguration("ur_description_package")
    tf_prefix = LaunchConfiguration("tf_prefix")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    controller_manager_update_rate = LaunchConfiguration("controller_manager_update_rate")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    isaac_arm_topic = LaunchConfiguration("isaac_arm_topic")
    isaac_gripper_topic = LaunchConfiguration("isaac_gripper_topic")
    isaac_joint_states_topic = LaunchConfiguration("isaac_joint_states_topic")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "ur_robotiq_calibration.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", GANTRY_DESCRIPTION_FILE]
            ),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            ur_type,
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "isaac_joint_commands:=",
            isaac_arm_topic,
            " ",
            "isaac_gripper_joint_commands:=",
            isaac_gripper_topic,
            " ",
            "isaac_joint_states:=",
            isaac_joint_states_topic,
            " ",
            "use_fake_hardware:=false ",
            "sim_isaac:=true ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    initial_controllers = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", GANTRY_CONTROLLERS_FILE]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            {
                # The Isaac topic bridge is less deterministic than direct robot I/O,
                # so run the controller manager at a lower rate to avoid overruns.
                "update_rate": ParameterValue(controller_manager_update_rate, value_type=int)
            },
            ParameterFile(initial_controllers, allow_substs=True),
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="both",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "error"],
        condition=IfCondition(launch_rviz),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    if activate_joint_controller.perform(context).lower() in ["true", "1"]:
        joint_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                initial_joint_controller,
                "-c",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ],
        )
    else:
        joint_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                initial_joint_controller,
                "-c",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
                "--inactive",
            ],
        )

    controller_spawner_nodes = [
        controller_spawner("robotiq_gripper_joint_trajectory_controller", active=True),
        controller_spawner("joint_state_broadcaster", active=False),
        controller_spawner("forward_position_controller", active=False),
        controller_spawner("forward_gripper_position_controller", active=False),
        controller_spawner("robotiq_activation_controller", active=False),
    ]

    return [
        control_node,
        robot_state_publisher,
        rviz_node,
        joint_controller_spawner,
        *controller_spawner_nodes,
    ]



def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur10e", description="Type of UR robot"),
        DeclareLaunchArgument("robot_ip", default_value="0.0.0.0", description="Robot IP"),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument("description_package", default_value="ur_robotiq_description"),
        DeclareLaunchArgument("ur_description_package", default_value="ur_description"),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("controller_spawner_timeout", default_value="10"),
        DeclareLaunchArgument("controller_manager_update_rate", default_value="250"),
        DeclareLaunchArgument("initial_joint_controller", default_value="joint_trajectory_controller"),
        DeclareLaunchArgument("activate_joint_controller", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("isaac_arm_topic", default_value="/isaac_joint_commands"),
        DeclareLaunchArgument("isaac_gripper_topic", default_value="/isaac_joint_gripper"),
        DeclareLaunchArgument("isaac_joint_states_topic", default_value="/joint_states"),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
