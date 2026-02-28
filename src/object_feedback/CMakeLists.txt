cmake_minimum_required(VERSION 3.8)
project(object_feedback)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(object_feedback_node
  src/object_feedback.cpp
)

add_executable(feedback_all_node
  src/feedback_all.cpp
)

ament_target_dependencies(object_feedback_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  geometry_msgs
  visualization_msgs
)

ament_target_dependencies(feedback_all_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  tf2_ros
)

target_link_libraries(object_feedback_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  object_feedback_node
  feedback_all_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
# install(DIRECTORY include/
#   DESTINATION include
# )

# Install launch and config
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
