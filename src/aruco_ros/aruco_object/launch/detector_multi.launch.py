from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
import os
from ament_index_python.packages import get_package_share_directory


def launch_setup(context: LaunchContext):
    """Launch 4 object_detector nodes"""
    # Side mapping: 0=front, 1=right, 2=back, 3=left
    camera_positions = ['front', 'back', 'left', 'right']
    default_side = int(LaunchConfiguration('default_dock_side').perform(context))

    if default_side not in [0, 1, 2, 3]:
        raise Exception(f"[ERROR] Invalid dock_side: '{default_side}'. Must be 0-3.")
    
    pkg_share = get_package_share_directory('aruco_object')
    config_file = os.path.join(pkg_share, 'config', 'combined_params.yaml')
    log_level = LaunchConfiguration('log_level').perform(context)
    
    nodes = []
    
    for position in camera_positions:
        node = Node(
            package='aruco_object',
            executable='aruco_detector_node',
            name=f'object_detector_{position}',
            parameters=[config_file, {'camera_position': position}],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level]
        )
        nodes.append(node)
    
    # Publish initial dock_side
    publish_cmd = ExecuteProcess(
        cmd=['bash', '-c', 
             f'sleep 2 && ros2 topic pub /robot/dock_side std_msgs/msg/Int16 "data: {default_side}" -1'],
        output='screen'
    )
    nodes.append(publish_cmd)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('default_dock_side', default_value='1'),  # 0=front, 1=right, 2=back, 3=left
        OpaqueFunction(function=launch_setup)
    ])
