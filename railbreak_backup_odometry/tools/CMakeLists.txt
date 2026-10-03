# Standalone build of the core (no ROS): lockstep and timing checks on any host.
#   cmake -S railbreak_backup_odometry/tools -B build/rbo && cmake --build build/rbo --config Release
cmake_minimum_required(VERSION 3.14)
project(railbreak_backup_odometry_tools CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_executable(replay_events replay_events.cpp)
target_include_directories(replay_events PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)
add_executable(replay_velocity replay_velocity.cpp)
target_include_directories(replay_velocity PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)
add_executable(test_core test_core.cpp)
target_include_directories(test_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)
enable_testing()
add_test(NAME test_core COMMAND test_core)
