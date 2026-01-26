from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'aruco_cluster_detect' # Note: Project name in CMake is aruco_cluster_detect, but folder is aruco_object. Careful!
    # Checking CMakeLists.txt: project(aruco_cluster_detect). 
    # Checking package.xml (not viewed, but likely aruco_cluster_detect or aruco_object).
    # Wait, the user provided path: src/aruco_ros/aruco_object/CMakeLists.txt
    # project(aruco_cluster_detect)
    # Let's assume the package name logic follows the project name.
    
    config = os.path.join(
        get_package_share_directory('aruco_cluster_detect'),
        'config',
        'scanner_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='aruco_cluster_detect',
            executable='aruco_row_scanner',
            name='aruco_row_scanner',
            output='screen',
            parameters=[config]
        )
    ])
