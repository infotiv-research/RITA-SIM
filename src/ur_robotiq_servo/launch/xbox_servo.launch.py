import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_directory = os.path.join(
        get_package_share_directory('ur_robotiq_servo'),
        'config'
    )
    
    joy_params = os.path.join(config_directory, 'joy-params.yaml')  # facultatif si séparé
    xbox_params = os.path.join(config_directory, 'xbox-params.yaml')

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[joy_params],  # ou directement dans xbox-params.yaml si tout est dedans
        ),
        Node(
            package='ur_robotiq_servo',
            executable='xbox_servo',
            name='xbox_servo_node',
            output='screen',
            parameters=[xbox_params],
        ),
    ])
