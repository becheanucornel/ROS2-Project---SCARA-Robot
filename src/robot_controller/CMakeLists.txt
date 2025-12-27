cmake_minimum_required(VERSION 3.15)
project(robot_controller)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nlohmann_json)

add_executable(robot_driver src/robot_driver.cpp)

ament_target_dependencies(robot_driver
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  tf2
)

add_executable(motion_publisher_node src/motion_publisher.cpp)

ament_target_dependencies(motion_publisher_node 
  rclcpp
  geometry_msgs
  nlohmann_json
  sensor_msgs
)

install(TARGETS 
  robot_driver
  motion_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
