from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
import os
from ament_index_python.packages import get_package_share_directory


def launch_setup(context: LaunchContext):
    """Launch 4 object_detector nodes"""
    camera_positions = ['front', 'back', 'left', 'right']
    default_camera = LaunchConfiguration('default_active_camera').perform(context)

    if default_camera and default_camera not in camera_positions:
        raise Exception(f"[ERROR] Invalid camera position: '{default_camera}'")
    
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
    
    # Publish initial active camera
    if default_camera:
        publish_cmd = ExecuteProcess(
            cmd=['bash', '-c', 
                 f'sleep 2 && ros2 topic pub /active_camera std_msgs/msg/String "data: \'{default_camera}\'" -1'],
            output='screen'
        )
        nodes.append(publish_cmd)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('default_active_camera', default_value='back'),
        OpaqueFunction(function=launch_setup)
    ])
