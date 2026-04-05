from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    mavros_launch_dir = os.path.join(
        get_package_share_directory('mavros'),
        'launch'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/pixhawk:57600'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mavros_launch_dir, 'px4.launch')
            ),
            launch_arguments={
                'fcu_url': LaunchConfiguration('fcu_url')
            }.items()
        )
    ])