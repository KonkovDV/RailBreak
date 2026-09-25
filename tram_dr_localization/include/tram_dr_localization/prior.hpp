#pragma once
// Stationary priors for weakly observable states, and the algebra of a
// robustified innovation. Header-only, ROS-free, depends only on <cmath>.
//
// ---------------------------------------------------------------------------
// F-23: a prior is a fixed amount of information, not a measurement stream
// ---------------------------------------------------------------------------
// Ukf::apply_mass_prior() used to fuse the SAME pseudo-measurement
// z = log(m0) once per second through an ordinary Kalman update. A scalar
// update adds 1/R to the information every time it runs, so after N
// applications
//
//     1 / P_N = 1 / P_0 + N / R
//
// and the variance collapses far below the prior it was derived from. With
// the shipped numbers (sigma_prior = 0.3 on log m, q_logm = (80/m0)^2 per
// second, m0 = 28 t) the fixed point of the alternating
// "grow by Q, then fuse the prior" recursion
//
//     P <- P R / (P + R) + Q
//
// is P = (Q + sqrt(Q^2 + 4 Q R)) / 2 = 8.6123e-4, i.e. sigma_log_m = 0.02935.
// The estimator therefore claims +/- 822 kg on a vehicle whose declared
// loading range is +/- 12500 kg (71-911EM "Lvenok", 15-40 t). That is a
// 10.3x overconfidence in the single parameter that scales the entire
// traction model, and it is manufactured out of nothing: no new information
// enters the filter between applications.
//
// It matters most in exactly the case the integrity layer exists for. On an
// unaided coast down the longest gap of route 10 (2385.6 m, 142.85 s from
// 16.7 m/s) a 1-sigma mass error moves the integrated path by
//
//     +/- 0.0293 -> +17.8 / -18.7 m       (what the filter believes)
//     +/- 0.3019 -> +152.5 / -245.6 m     (what the loading range implies)
//
// against an alert limit of 124.28 m. The pinned prior hides a dominant
// error source from P_ss, and P_ss is what the protection level is built on.
//
// The correct model for "slowly varying quantity with a known stationary
// spread" is a first-order Gauss-Markov (Ornstein-Uhlenbeck) process - the
// standard treatment of bias states in inertial navigation:
//
//     x_{k+1} = z + phi (x_k - z),          phi = exp(-dt / tau)
//     P_{k+1} = phi^2 P_k + (1 - phi^2) R
//
// The mean still reverts to the prior, so the anchor that the repeated
// update was there to provide is preserved: log m cannot wander off to the
// clip bounds to explain a force error. But the stationary variance is
// EXACTLY R for every tau and every dt, so certainty can no longer be
// fabricated by running the filter for longer.
//
// This is the linear-Gaussian propagation of the state transition
// A = diag(1, ..., phi, ..., 1) with Q = diag(0, ..., (1-phi^2) R, ..., 0),
// so the cross-covariances of the prior state with every other state must be
// scaled by phi as well - see gauss_markov_cross().

#include <cmath>

namespace tram_dr {
namespace prior {

// Reversion factor of a first-order Gauss-Markov process over dt with time
// constant tau. Returns 1.0 (no reversion) for non-positive or non-finite
// dt, and 0.0 (instant reversion to the prior mean) as tau -> 0.
inline double gauss_markov_phi(double dt_s, double tau_s) {
  if (!std::isfinite(dt_s) || !std::isfinite(tau_s) || dt_s <= 0.0) {
    return 1.0;
  }
  if (tau_s <= 0.0) {
    return 0.0;
  }
  return std::exp(-dt_s / tau_s);
}

// Mean reversion toward the prior value z. phi = 1 leaves x untouched.
inline double gauss_markov_mean(double x, double z, double phi) {
  return z + phi * (x - z);
}

// Variance propagation with stationary variance r_stationary. The fixed
// point is r_stationary for any phi in [0, 1), which is the whole point:
// P == R is a stable equilibrium instead of a decaying transient.
inline double gauss_markov_variance(double p, double phi, double r_stationary) {
  const double phi2 = phi * phi;
  return phi2 * p + (1.0 - phi2) * r_stationary;
}

// Cross-covariance propagation for the same transition. Off-diagonal terms
// of the reverting state carry a single factor of phi, not phi^2.
inline double gauss_markov_cross(double p_cross, double phi) {
  return phi * p_cross;
}

// Fixed point of the DEFECTIVE recursion P <- P R / (P + R) + Q, i.e. what a
// repeatedly fused constant prior actually converges to. Kept in the header
// because it is the quantity the regression test pins: if someone reinstates
// the repeated update, the test says by how much it lies.
inline double repeated_update_fixed_point(double r, double q) {
  if (!std::isfinite(r) || !std::isfinite(q) || r <= 0.0 || q < 0.0) {
    return std::nan("");
  }
  if (q == 0.0) {
    return 0.0;  // information grows without bound; variance -> 0
  }
  return 0.5 * (q + std::sqrt(q * q + 4.0 * q * r));
}

// How many independent fusions of the same prior the fixed point corresponds
// to: from 1/P = (1 + n)/R we get n = R/P - 1. Reported by the test so the
// defect has an interpretable size rather than just a small number.
inline double equivalent_fusions(double r, double p) {
  if (!std::isfinite(r) || !std::isfinite(p) || p <= 0.0) {
    return std::nan("");
  }
  return r / p - 1.0;
}

// ---------------------------------------------------------------------------
// F-24: a robustified innovation covariance is not a consistency statistic
// ---------------------------------------------------------------------------
// update_wheels() caps the information of an outlying wheel by inflating the
// innovation variance,
//
//     scale  = max(1, nu^2 / (c^2 S_jj)),      S'_jj = scale * S_jj
//
// which is the right thing to do to the GAIN: a locked wheel at full slide
// must not drag v_hat. The defect was computing the reported NIS from the
// inflated S. For a single channel that gives
//
//     NIS' = nu^2 / (scale * S) = min(nu^2 / S, c^2)
//
// so the consistency statistic SATURATES at c^2 = 9 no matter how gross the
// outlier is - nu = 5, 50 and 1000 all report exactly 9. The monitor is
// blinded by the very mechanism that suppressed the fault, and nis_cusum
// inherits the blindness. This is the same failure mode recently formalised
// for gated filters, where the post-gate innovation follows a truncated
// chi-square rather than chi^2_m (Or, "Selection-Induced Contraction of
// Innovation Statistics in Gated Kalman Filters", arXiv:2512.18508).
//
// Rule: robustify the gain, report the raw statistic. NIS must be evaluated
// against the pre-inflation S, and the number of capped channels must be
// published so an operator can tell suppression from agreement.

// Huber/DCS variance inflation factor for one channel. Always >= 1.
inline double huber_scale(double nu, double s_jj, double c) {
  if (!(c > 0.0) || !std::isfinite(c) || !std::isfinite(nu) ||
      !std::isfinite(s_jj) || !(s_jj > 0.0)) {
    return 1.0;
  }
  const double ratio = (nu * nu) / (c * c * s_jj);
  return (ratio > 1.0) ? ratio : 1.0;
}

// The NIS a single channel would report if it were evaluated against the
// inflated variance. Saturates at c^2; provided so the regression test can
// assert the saturation instead of trusting a comment about it.
inline double huber_capped_nis(double nu, double s_jj, double c) {
  if (!std::isfinite(nu) || !std::isfinite(s_jj) || !(s_jj > 0.0)) {
    return std::nan("");
  }
  const double raw = (nu * nu) / s_jj;
  if (!(c > 0.0) || !std::isfinite(c)) {
    return raw;
  }
  const double c2 = c * c;
  return (raw < c2) ? raw : c2;
}

}  // namespace prior
}  // namespace tram_dr
