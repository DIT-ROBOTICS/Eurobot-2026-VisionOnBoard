from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import os
from ament_index_python.packages import get_package_share_directory


def launch_setup(context: LaunchContext):
    """Launch 4 aruco_row_scanner nodes"""
    camera_positions = ['front', 'back', 'left', 'right']
    
    pkg_share = get_package_share_directory('aruco_object')
    config_file = os.path.join(pkg_share, 'config', 'combined_params.yaml')
    log_level = LaunchConfiguration('log_level').perform(context)
    team_color = LaunchConfiguration('team_color').perform(context)
    
    valid_team_colors = ['blue', 'yellow']
    if team_color not in valid_team_colors:
        raise Exception(f"[ERROR] Invalid team_color: '{team_color}'")
    
    nodes = []
    
    for position in camera_positions:
        node = Node(
            package='aruco_object',
            executable='aruco_row_scanner',
            name=f'aruco_row_scanner_{position}',
            parameters=[config_file, {'camera_position': position, 'team_color': team_color}],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level]
        )
        nodes.append(node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('team_color', default_value='blue'),
        OpaqueFunction(function=launch_setup)
    ])
