cmake_minimum_required(VERSION 3.16)
project(tram_dr_core_standalone LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PKG "${CMAKE_CURRENT_SOURCE_DIR}/../tram_dr_localization")

add_executable(test_core
  test_core.cpp
  ${PKG}/src/lib/plant.cpp
  ${PKG}/src/lib/sca.cpp
  ${PKG}/src/lib/ukf.cpp
  ${PKG}/src/lib/modes.cpp
  ${PKG}/src/lib/map.cpp
)
target_include_directories(test_core PRIVATE ${PKG}/include)

add_executable(replay_ukf
  replay_ukf.cpp
  ${PKG}/src/lib/plant.cpp
  ${PKG}/src/lib/sca.cpp
  ${PKG}/src/lib/ukf.cpp
  ${PKG}/src/lib/modes.cpp
  ${PKG}/src/lib/map.cpp
)
target_include_directories(replay_ukf PRIVATE ${PKG}/include)

enable_testing()
add_test(NAME test_core COMMAND test_core)

if(MSVC)
  set(_tram_dr_warn /W4)
else()
  set(_tram_dr_warn -Wall -Wextra -fno-fast-math)
endif()
target_compile_options(test_core PRIVATE ${_tram_dr_warn})
target_compile_options(replay_ukf PRIVATE ${_tram_dr_warn})
