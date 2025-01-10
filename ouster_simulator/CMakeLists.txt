cmake_minimum_required(VERSION 3.8)
project(ouster_simulator)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ouster_publisher_node src/ouster_publisher_node.cpp)
ament_target_dependencies(
  ouster_publisher_node
  rclcpp
  sensor_msgs
)

add_executable(ouster_subscriber_node src/ouster_subscriber_node.cpp)
ament_target_dependencies(
  ouster_subscriber_node
  rclcpp
  sensor_msgs
)

install(
  TARGETS ouster_publisher_node ouster_subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

