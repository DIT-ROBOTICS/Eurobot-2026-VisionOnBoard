from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def launch_setup(context: LaunchContext):
    """Called after launch arguments are resolved"""

    # Define valid camera positions
    camera_positions = ['front', 'back', 'left', 'right']
    
    # 1. Grab the value
    default_camera = LaunchConfiguration('default_active_camera').perform(context)

    # 2. VALIDATION LOGIC
    # Allow empty string if 'dormant' is intended, otherwise check the list
    if default_camera and default_camera not in camera_positions:
        raise Exception(f"\n\n[ERROR] Invalid camera position: '{default_camera}'. \n"
                        f"Valid options are: {camera_positions} or an empty string.\n")
    pkg_share = get_package_share_directory('aruco_object')
    pkg_realsense = get_package_share_directory('realsense2_camera')
    config_file = os.path.join(pkg_share, 'config', 'combined_params.yaml')
    
    log_level = LaunchConfiguration('log_level').perform(context)
    team_color = LaunchConfiguration('team_color').perform(context)
    
    # Validate team_color
    valid_team_colors = ['blue', 'yellow']
    if team_color not in valid_team_colors:
        raise Exception(f"\n\n[ERROR] Invalid team_color: '{team_color}'. \n"
                        f"Valid options are: {valid_team_colors}\n")
    
    actions = []
    
    # ========== PHASE 1: Launch 4 cameras with bandwidth-safe settings ==========
    for position in camera_positions:
        camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_realsense, 'launch', 'rs_launch.py')
            ),
            launch_arguments={
                'camera_namespace': position,
                'camera_name': position,
                'rgb_camera.color_profile': '640,480,15',  # Low bandwidth: 640x480 @ 15fps
                'enable_depth': 'false',  # Disable depth to save USB bandwidth
                'enable_infra': 'false',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
            }.items()
        )
        actions.append(camera)
    
    # ========== PHASE 2: Delayed detector/scanner nodes (wait for cameras) ==========
    detector_scanner_nodes = []
    
    # Launch 4 object_detector nodes
    for position in camera_positions:
        node = Node(
            package='aruco_object',
            executable='aruco_detector_node',
            name=f'object_detector_{position}',
            parameters=[
                config_file,
                {'camera_position': position}
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level]
        )
        detector_scanner_nodes.append(node)
    
    # Launch 4 aruco_row_scanner nodes
    for position in camera_positions:
        node = Node(
            package='aruco_object',
            executable='aruco_row_scanner',
            name=f'aruco_row_scanner_{position}',
            parameters=[
                config_file,
                {'camera_position': position, 'team_color': team_color}
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level]
        )
        detector_scanner_nodes.append(node)
    
    # Wrap detector/scanner nodes in TimerAction to delay 5 seconds after cameras start
    delayed_nodes = TimerAction(
        period=5.0,
        actions=detector_scanner_nodes
    )
    actions.append(delayed_nodes)
    
    # ========== PHASE 3: Publish initial active camera ==========
    if default_camera:
        # Additional delay to ensure detectors are ready before publishing active camera
        publish_cmd = ExecuteProcess(
            cmd=['bash', '-c', 
                 f'sleep 7 && ros2 topic pub /active_camera std_msgs/msg/String "data: \'{default_camera}\'" -1'],
            output='screen'
        )
        actions.append(publish_cmd)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level'
        ),
        DeclareLaunchArgument(
            'default_active_camera',
            default_value='right',
            description='Camera to activate at startup (front/back/left/right, empty for dormant)'
        ),
        DeclareLaunchArgument(
            'team_color',
            default_value='blue',
            description='Team color (blue/yellow) - determines which ArUco IDs are targets'
        ),
        OpaqueFunction(function=launch_setup)
    ])
