cmake_minimum_required(VERSION 3.8)
project(autoware_practice_lidar_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(simple_lidar_simulator
  src/simple_lidar_simulator.cpp
)

ament_target_dependencies(simple_lidar_simulator
  rclcpp
  geometry_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
)

install(TARGETS
  simple_lidar_simulator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()