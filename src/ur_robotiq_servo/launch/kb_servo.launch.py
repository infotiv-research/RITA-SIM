import os

from launch import LaunchDescription
from launch_ros.actions import Node
import ament_index_python.packages
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('ur_robotiq_servo'),
        'config')
    joy_params = os.path.join(config_directory, 'joy-params.yaml')
    ps5_params = os.path.join(config_directory, 'ps5-params.yaml')

    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .to_moveit_configs()
    )
    # Get parameters for the Servo node
    servo_params = (
        ParameterBuilder("moveit_servo")
        .yaml(
            parameter_namespace="moveit_servo",
            file_path="config/panda_simulated_config.yaml",
        )
        .to_dict()
    )

    # The servo cpp interface demo
    # Creates the Servo node and publishes commands to it
    servo_node = Node(
        package="moveit2_tutorials",
        executable="servo_cpp_interface_demo",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    return LaunchDescription([
        servo_node
    ])