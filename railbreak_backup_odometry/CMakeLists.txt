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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # ROS executor stress. The filter, FaultScore and integrity regressions are
  # test_core, registered here so a package review does not see only this gtest.
  # The same binary is the CTest target of tools/CMakeLists.txt on a host without ROS.
  ament_add_gtest(test_gnss_stress test/test_gnss_stress.cpp)
  if(TARGET test_gnss_stress)
    target_include_directories(test_gnss_stress PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_compile_definitions(test_gnss_stress PRIVATE
      RAILBREAK_ASSETS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets")
    ament_target_dependencies(test_gnss_stress rclcpp nav_msgs sensor_msgs diagnostic_msgs
      tram_vehicle_msgs ament_index_cpp)
  endif()
  add_executable(test_core tools/test_core.cpp)
  target_include_directories(test_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME test_core COMMAND $<TARGET_FILE:test_core>)
endif()

ament_package()
