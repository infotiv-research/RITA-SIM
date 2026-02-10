import os

from launch import LaunchDescription
from launch_ros.actions import Node
import ament_index_python.packages

def generate_launch_description():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('ur_robotiq_servo'),
        'config')
    joy_params = os.path.join(config_directory, 'joy-params.yaml')
    ps5_params = os.path.join(config_directory, 'ps5-params.yaml')
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[joy_params],
        ),
        Node(
            package='ur_robotiq_servo',
            executable='ps5_servo',
            name='ps5_servo_node',
            output='screen',
            parameters=[ps5_params],
            arguments=[],
        ),
    ])