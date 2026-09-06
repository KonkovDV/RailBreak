cmake_minimum_required(VERSION 3.16)
project(tram_dr_core_standalone LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PKG "${CMAKE_CURRENT_SOURCE_DIR}/../tram_dr_localization")

set(TRAM_DR_CORE_SRC
  ${PKG}/src/lib/plant.cpp
  ${PKG}/src/lib/sca.cpp
  ${PKG}/src/lib/ukf.cpp
  ${PKG}/src/lib/modes.cpp
  ${PKG}/src/lib/map.cpp
)

add_executable(test_core test_core.cpp ${TRAM_DR_CORE_SRC})
target_include_directories(test_core PRIVATE ${PKG}/include)

# Integrity contracts: freshness, standstill evidence, per-channel confidence,
# time/control validation and covariance numerics. Runs under NDEBUG too.
add_executable(test_integrity_contracts test_integrity_contracts.cpp ${TRAM_DR_CORE_SRC})
target_include_directories(test_integrity_contracts PRIVATE ${PKG}/include)

# Scaled-UT weight algebra and the joint (alpha, beta, kappa) admissibility
# window from docs/math.md section 5. Deliberately standalone: it links no
# core sources and includes no repo headers, so it stays an independent check
# on the algebra rather than a tautology against ukf.cpp.
add_executable(test_ut_weights_psd test_ut_weights_psd.cpp)

# The dependent counterpart of the above: cross-checks the shipped
# ut_weights.hpp against the same closed-form window and against hand-derived
# values, and pins the F-08 hazard (alpha = 0.58 is admissible at beta = 2 but
# not at beta = 1 or beta = 0). Header-only, so no core sources are linked.
add_executable(test_ut_weights_header test_ut_weights_header.cpp)
target_include_directories(test_ut_weights_header PRIVATE ${PKG}/include)

add_executable(replay_ukf replay_ukf.cpp ${TRAM_DR_CORE_SRC})
target_include_directories(replay_ukf PRIVATE ${PKG}/include)

enable_testing()
add_test(NAME test_core COMMAND test_core)
add_test(NAME test_integrity_contracts COMMAND test_integrity_contracts)
add_test(NAME test_ut_weights_psd COMMAND test_ut_weights_psd)
add_test(NAME test_ut_weights_header COMMAND test_ut_weights_header)

if(MSVC)
  set(_tram_dr_warn /W4)
else()
  set(_tram_dr_warn -Wall -Wextra -fno-fast-math)
endif()
target_compile_options(test_core PRIVATE ${_tram_dr_warn})
target_compile_options(test_integrity_contracts PRIVATE ${_tram_dr_warn})
target_compile_options(test_ut_weights_psd PRIVATE ${_tram_dr_warn})
target_compile_options(test_ut_weights_header PRIVATE ${_tram_dr_warn})
target_compile_options(replay_ukf PRIVATE ${_tram_dr_warn})
