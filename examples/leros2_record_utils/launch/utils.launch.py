from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('leros2_record_utils')
    config_file = os.path.join(pkg_share, 'config', 'record_utils.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to the config file'
        ),
        Node(
            package='leros2_record_utils',
            executable='leros2_record_utils',
            name='leros2_record_utils',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        )
    ])
