
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='m4_firmware',
            executable='m4_controller_node',
            name='m4_controller',
            output='screen'
        )
    ])