from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'aruco_object'
    
    config = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'combined_params.yaml'
    )

    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='aruco_row_scanner',
            name='aruco_row_scanner',
            output='screen',
            parameters=[config]
        )
    ])
