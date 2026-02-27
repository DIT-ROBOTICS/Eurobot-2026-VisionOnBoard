from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    depth_topic_arg = DeclareLaunchArgument(
        "depth_topic",
        default_value="/camera/camera/depth/image_rect_raw",
        description="Depth image topic subscribed by object_feedback_node",
    )

    dist_topic_arg = DeclareLaunchArgument(
        "dist_topic",
        default_value="/camera/camera/obj_distance",
        description="Distance topic published by object_feedback_node",
    )

    # Launch single side feedback node
    launch_fb = Node(
        package="object_feedback",
        executable="object_feedback_node",
        name="object_feedback_node",
        parameters=[
            {"depth_topic": LaunchConfiguration("depth_topic")},
            {"dist_topic": LaunchConfiguration("dist_topic")},
        ],
    )

    ld = LaunchDescription()
    ld.add_action(depth_topic_arg)
    ld.add_action(dist_topic_arg)
    ld.add_action(launch_fb)

    return ld