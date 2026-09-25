cmake_minimum_required(VERSION 3.14)
project(railbreak_backup_odometry CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tram_vehicle_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(backup_odometry_node src/backup_odometry_node.cpp)
target_include_directories(backup_odometry_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(backup_odometry_node rclcpp nav_msgs sensor_msgs diagnostic_msgs
  tram_vehicle_msgs ament_index_cpp)

add_executable(replay_events tools/replay_events.cpp)
target_include_directories(replay_events PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS backup_odometry_node replay_events DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
# Offline track map from the issued train GNSS. In the package so a clone
# scores in the map frame. Without the directory the node still runs.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/assets)
  install(DIRECTORY assets DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
