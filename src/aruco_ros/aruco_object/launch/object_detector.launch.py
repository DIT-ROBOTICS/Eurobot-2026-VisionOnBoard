from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('aruco_object')
    config_file = os.path.join(pkg_share, 'config', 'combined_params.yaml')

    logger_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level'
    )

    object_detector_node = Node(
        package='aruco_object',
        executable='aruco_detector_node',
        name='object_detector',
        parameters=[config_file],
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    return LaunchDescription([
        logger_arg,
        object_detector_node
    ])
