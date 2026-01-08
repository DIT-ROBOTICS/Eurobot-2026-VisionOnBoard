cmake_minimum_required(VERSION 3.8)
project(aruco_object)

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
find_package(visualization_msgs REQUIRED)


include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(object_detector src/object_detector.cpp)

ament_target_dependencies(object_detector
  rclcpp
  sensor_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  geometry_msgs
  visualization_msgs
)

target_link_libraries(object_detector
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  object_detector
  DESTINATION lib/${PROJECT_NAME})

ament_package()
