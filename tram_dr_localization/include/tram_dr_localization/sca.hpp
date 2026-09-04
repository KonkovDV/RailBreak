#pragma once
#include "tram_dr_localization/types.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace tram_dr {

constexpr int kAxleMotor = 0;
constexpr int kAxleTrailer = 1;

struct ScaParams {
  double r0_m{0.35};
  double sigma_v_mps{0.15};  // 1-sigma on r*omega before inflation
  // Extra 1-sigma as a fraction of |v|. 0 until inspect_bag shows coarse ω.
  double sigma_v_rel{0.0};
  double z_thresh{2.5};
  double inflate_max{100.0};
  // Average L/R on each bogie (0-1, 2-3, and 4-5 if n≥6) before the median.
  // Street curves at R~25 m give ~6% L/R split on independently rotating wheels.
  bool pair_lr{true};
  // 0 = motor, 1 = trailer. Under traction the consensus is trailers-only
  // when at least one trailer is live (classical unpowered-axle odometry).
  std::array<int, kNWheels> axle_role{};
  bool traction{false};  // set by the UKF from notch before sca_analyze
};

struct ScaResult {
  std::array<double, kNWheels> r_omega{};  // variance on omega_i, (rad/s)^2
  std::array<double, kNWheels> inflate{};   // >= 1
  int n_inflated{0};
  int n_inflated_motor{0};
  int n_inflated_trailer{0};
  int n_trailer_ok{0};
  bool used_trailer_consensus{false};
  double v_consensus_mps{0.0};
  // Set by the UKF, not sca_analyze: wheels agree with each other but not the body.
  bool common_mode{false};
};

// Median + z-inflate of inconsistent wheel speeds (not a hard-delete).
ScaResult sca_analyze(const double* omega, std::size_t n, const double* d_scale,
                      const ScaParams& p);

void sca_inflate(const std::vector<double>& omega, std::vector<double>& r_diag);

}  // namespace tram_dr
