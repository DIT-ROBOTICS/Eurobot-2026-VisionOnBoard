
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
	# Front feedback arguments
    DeclareLaunchArgument(
        "front_depth_topic",
        default_value="/front/front/depth/image_rect_raw",
        description="Front depth image topic subscribed by object_feedback_front",
    ),
	DeclareLaunchArgument(
        "front_dist_topic",
        default_value="/front/front/obj_distance",
        description="Front distance topic published by object_feedback_front",
    ),
	
    # Right feedback arguments
    DeclareLaunchArgument(
        "right_depth_topic",
        default_value="/right/right/depth/image_rect_raw",
        description="Right depth image topic subscribed by object_feedback_right",
    ),
    DeclareLaunchArgument(
        "right_dist_topic",
        default_value="/right/right/obj_distance",
        description="Right distance topic published by object_feedback_right",
    ),
	
    # Back feedback arguments
    DeclareLaunchArgument(
        "back_depth_topic",
        default_value="/back/back/depth/image_rect_raw",
        description="Back depth image topic subscribed by object_feedback_back",
    ),
	DeclareLaunchArgument(
        "back_dist_topic",
        default_value="/back/back/obj_distance",
        description="Back distance topic published by object_feedback_back",
    ),

    # Left feedback arguments
    DeclareLaunchArgument(
        "left_depth_topic",
        default_value="/left/left/depth/image_rect_raw",
        description="Left depth image topic subscribed by object_feedback_left",
    ),
    DeclareLaunchArgument(
        "left_dist_topic",
        default_value="/left/left/obj_distance",
        description="Left distance topic published by object_feedback_left",
    ),
	
    # Feedback all node arguments
    DeclareLaunchArgument(
        "success_topic",
        default_value="/robot/vision/onTakeSuccess",
        description="Success topic published by feedback_all_node",
    ),
	DeclareLaunchArgument(
        "distance_threshold_mm",
        default_value="190",
        description="Distance threshold in mm for feedback_all_node to determine success",
    ),
]


def generate_launch_description():

	object_feedback_front = Node(
		package="object_feedback",
		executable="object_feedback_node",
		name="object_feedback_front",
		parameters=[
			{"depth_topic": LaunchConfiguration("front_depth_topic")},
			{"dist_topic": LaunchConfiguration("front_dist_topic")},
		],
	)

	object_feedback_right = Node(
		package="object_feedback",
		executable="object_feedback_node",
		name="object_feedback_right",
		parameters=[
			{"depth_topic": LaunchConfiguration("right_depth_topic")},
			{"dist_topic": LaunchConfiguration("right_dist_topic")},
		],
	)

	object_feedback_back = Node(
		package="object_feedback",
		executable="object_feedback_node",
		name="object_feedback_back",
		parameters=[
			{"depth_topic": LaunchConfiguration("back_depth_topic")},
			{"dist_topic": LaunchConfiguration("back_dist_topic")},
		],
	)

	object_feedback_left = Node(
		package="object_feedback",
		executable="object_feedback_node",
		name="object_feedback_left",
		parameters=[
			{"depth_topic": LaunchConfiguration("left_depth_topic")},
			{"dist_topic": LaunchConfiguration("left_dist_topic")},
		],
	)

	feedback_all = Node(
		package="object_feedback",
		executable="feedback_all_node",
		name="feedback_all_node",
		parameters=[
			{"front_dist_topic": LaunchConfiguration("front_dist_topic")},
			{"right_dist_topic": LaunchConfiguration("right_dist_topic")},
			{"back_dist_topic": LaunchConfiguration("back_dist_topic")},
			{"left_dist_topic": LaunchConfiguration("left_dist_topic")},
			{"distance_threshold_mm": LaunchConfiguration("distance_threshold_mm")},
			{"success_topic": LaunchConfiguration("success_topic")},
		],
	)

	ld = LaunchDescription(ARGUMENTS)

	ld.add_action(object_feedback_front)
	ld.add_action(object_feedback_right)
	ld.add_action(object_feedback_back)
	ld.add_action(object_feedback_left)
	ld.add_action(feedback_all)

	return ld
