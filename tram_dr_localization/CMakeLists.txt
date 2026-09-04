cmake_minimum_required(VERSION 3.16)
project(tram_dr_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(tram_dr_core
  src/lib/plant.cpp
  src/lib/sca.cpp
  src/lib/ukf.cpp
  src/lib/modes.cpp
  src/lib/map.cpp
)
target_include_directories(tram_dr_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
if(NOT MSVC)
  target_compile_options(tram_dr_core PRIVATE -fno-fast-math)
endif()

add_executable(state_estimator_node src/nodes/state_estimator_node.cpp)
ament_target_dependencies(state_estimator_node
  rclcpp nav_msgs std_msgs diagnostic_msgs sensor_msgs geometry_msgs)
target_link_libraries(state_estimator_node tram_dr_core)

add_executable(fault_monitor_node src/nodes/fault_monitor_node.cpp)
ament_target_dependencies(fault_monitor_node rclcpp diagnostic_msgs)

add_executable(map_projector_node src/nodes/map_projector_node.cpp)
ament_target_dependencies(map_projector_node rclcpp sensor_msgs nav_msgs)
target_link_libraries(map_projector_node tram_dr_core)

add_executable(topic_adapter_node src/nodes/topic_adapter_node.cpp)
ament_target_dependencies(topic_adapter_node
  rclcpp std_msgs sensor_msgs geometry_msgs)
target_link_libraries(topic_adapter_node tram_dr_core)

install(TARGETS tram_dr_core state_estimator_node fault_monitor_node map_projector_node
  topic_adapter_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_types test/test_types.cpp)
  target_link_libraries(test_types tram_dr_core)
  ament_add_gtest(test_plant test/test_plant.cpp)
  target_link_libraries(test_plant tram_dr_core)
  ament_add_gtest(test_sca test/test_sca.cpp)
  target_link_libraries(test_sca tram_dr_core)
  ament_add_gtest(test_ukf test/test_ukf.cpp)
  target_link_libraries(test_ukf tram_dr_core)
endif()

ament_package()
