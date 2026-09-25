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
# window described in docs/math.md. Deliberately standalone: it links no
# core sources and includes no repo headers, so it stays an independent check
# on the algebra rather than a tautology against ukf.cpp.
add_executable(test_ut_weights_psd test_ut_weights_psd.cpp)

# The dependent counterpart of the above: cross-checks the shipped
# ut_weights.hpp against the same closed-form window and against hand-derived
# values, and pins the F-08 hazard (alpha = 0.58 is admissible at beta = 2 but
# not at beta = 1 or beta = 0). Header-only, so no core sources are linked.
add_executable(test_ut_weights_header test_ut_weights_header.cpp)
target_include_directories(test_ut_weights_header PRIVATE ${PKG}/include)

# F-34 and F-35 witnesses: the Gauss-Markov (Ornstein-Uhlenbeck) stationary
# prior that replaces the repeatedly re-fused mass pseudo-measurement, and the
# saturation of a NIS evaluated against a Huber-inflated S. Header-only like
# the target above - it includes prior.hpp and links no core sources, so it
# also runs under the sanitizers.
add_executable(test_prior_and_nis test_prior_and_nis.cpp)
target_include_directories(test_prior_and_nis PRIVATE ${PKG}/include)

# Numerical edge contracts, including an independent long-double LDL check.
add_executable(test_numerical_edges test_numerical_edges.cpp)
target_include_directories(test_numerical_edges PRIVATE ${PKG}/include)

# SI force laws, drive-state contract and passive zero-crossing events.
add_executable(test_plant_contracts test_plant_contracts.cpp ${PKG}/src/lib/plant.cpp)
target_include_directories(test_plant_contracts PRIVATE ${PKG}/include)

# Process priors must also run during wheel silence and rest/ZUPT packets.
add_executable(test_prior_scheduling test_prior_scheduling.cpp ${TRAM_DR_CORE_SRC})
target_include_directories(test_prior_scheduling PRIVATE ${PKG}/include)

add_executable(replay_ukf replay_ukf.cpp ${TRAM_DR_CORE_SRC})
target_include_directories(replay_ukf PRIVATE ${PKG}/include)

add_executable(test_hackathon_phase0 test_hackathon_phase0.cpp ${TRAM_DR_CORE_SRC})
target_include_directories(test_hackathon_phase0 PRIVATE ${PKG}/include)

enable_testing()
add_test(NAME test_core COMMAND test_core)
add_test(NAME test_integrity_contracts COMMAND test_integrity_contracts)
add_test(NAME test_ut_weights_psd COMMAND test_ut_weights_psd)
add_test(NAME test_ut_weights_header COMMAND test_ut_weights_header)
add_test(NAME test_prior_and_nis COMMAND test_prior_and_nis)
add_test(NAME test_numerical_edges COMMAND test_numerical_edges)
add_test(NAME test_plant_contracts COMMAND test_plant_contracts)
add_test(NAME test_prior_scheduling COMMAND test_prior_scheduling)
add_test(NAME test_hackathon_phase0 COMMAND test_hackathon_phase0)

if(MSVC)
  set(_tram_dr_warn /W4)
else()
  set(_tram_dr_warn -Wall -Wextra -fno-fast-math)
endif()
target_compile_options(test_core PRIVATE ${_tram_dr_warn})
target_compile_options(test_integrity_contracts PRIVATE ${_tram_dr_warn})
target_compile_options(test_ut_weights_psd PRIVATE ${_tram_dr_warn})
target_compile_options(test_ut_weights_header PRIVATE ${_tram_dr_warn})
target_compile_options(test_prior_and_nis PRIVATE ${_tram_dr_warn})
target_compile_options(test_numerical_edges PRIVATE ${_tram_dr_warn})
target_compile_options(test_plant_contracts PRIVATE ${_tram_dr_warn})
target_compile_options(test_prior_scheduling PRIVATE ${_tram_dr_warn})
target_compile_options(test_hackathon_phase0 PRIVATE ${_tram_dr_warn})
target_compile_options(replay_ukf PRIVATE ${_tram_dr_warn})
