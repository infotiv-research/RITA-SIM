from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    config_path = os.path.join(get_package_share_directory('servo_keyboard'), 'config', 'servo_keyboard_params.yaml')

    return LaunchDescription([
        Node(
            package='servo_keyboard',
            executable='servo_keyboard_input',
            name='servo_keyboard_input',
            output='screen',
            parameters=[config_path],
        )
    ])