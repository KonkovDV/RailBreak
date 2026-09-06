#pragma once
// Scaled unscented-transform weights and the JOINT (alpha, beta, kappa)
// admissibility window. Header-only, ROS-free, depends only on <cmath>.
//
// Why this file exists (F-08)
// ---------------------------
// alpha, beta and kappa_ut are three independent ROS parameters, but the
// positive-semidefiniteness of the propagated covariance depends on them
// *jointly*. Validating each one against its own interval - which is what
// validate_ukf() did - accepts combinations that drive the zeroth covariance
// weight negative. P then stops being PSD, la::project_pd() repairs it on
// every cycle, and the operator is shown a filter that appears to work while
// its reported uncertainty is fiction. The repair is what makes this
// dangerous: it removes the crash that would otherwise expose the
// misconfiguration.
//
// Algebra (docs/math.md section 5)
// --------------------------------
//   lambda = alpha^2 * (L + kappa) - L
//   c      = L + lambda = alpha^2 * (L + kappa)
//   W_m0   = lambda / c
//   W_i    = 0.5 / c                             (i = 1 .. 2L)
//   W_c0   = W_m0 + (1 - alpha^2 + beta)
//
// Sanity identities: sum(W_m) = 1 and sum(W_c) = 1 + (1 - alpha^2 + beta).
//
// At kappa = 0 the scaling collapses to W_m0 = 1 - 1/alpha^2, hence
//
//   W_c0 = 2 + beta - alpha^2 - 1/alpha^2
//
// which is INDEPENDENT of the state dimension L. Requiring W_c0 >= 0 is a
// quadratic in alpha^2 with roots ((2+beta) +/- sqrt((2+beta)^2 - 4)) / 2:
//
//   beta = 2 -> alpha in [sqrt(2-sqrt3), sqrt(2+sqrt3)] = [0.5176381, 1.9318517]
//   beta = 1 -> alpha in [1/phi, phi]                   = [0.6180340, 1.6180340]
//   beta = 0 -> alpha = 1 exactly (the window degenerates to a single point)
//
// The shipped default (alpha = 0.58, beta = 2, kappa_ut = 0, L = 12) clears
// the lower edge by 12.05 % and clears the Luo-Moroz sufficient bound
// 1/sqrt(1+beta) = 0.5773503 by only 0.459 %. That margin is thin, and more
// importantly it MOVES the moment beta is edited: the very same alpha = 0.58
// is inadmissible at beta = 1 and at beta = 0. A per-parameter range check
// cannot see that; this predicate can.
//
// standalone/test_ut_weights_psd.cpp re-derives the same window from scratch
// without including this header, so the algebra has two independent witnesses.

#include <cmath>

namespace tram_dr {
namespace ut {

struct Weights {
  double lambda{0.0};
  double c{0.0};
  double wm0{0.0};
  double wc0{0.0};
  double wi{0.0};
};

// Unguarded evaluation of the weight set. May return non-finite members for
// inadmissible inputs; use weights_psd_ok() to decide admissibility.
inline Weights weights(double alpha, double beta, double kappa, int L) {
  Weights w;
  const double Ld = static_cast<double>(L);
  const double a2 = alpha * alpha;
  w.lambda = a2 * (Ld + kappa) - Ld;
  w.c = Ld + w.lambda;
  w.wm0 = w.lambda / w.c;
  w.wc0 = w.wm0 + (1.0 - a2 + beta);
  w.wi = 0.5 / w.c;
  return w;
}

// The joint admissibility predicate. True only when the triple (alpha, beta,
// kappa) yields a finite, usable weight set with W_c0 >= 0 at dimension L,
// i.e. only when the unscented covariance is guaranteed PSD before any
// project_pd() repair.
inline bool weights_psd_ok(double alpha, double beta, double kappa, int L) {
  if (!std::isfinite(alpha) || !std::isfinite(beta) || !std::isfinite(kappa)) {
    return false;
  }
  if (L <= 0 || beta < 0.0) {
    return false;
  }
  const Weights w = weights(alpha, beta, kappa, L);
  // c = L + lambda is the common denominator of every weight. c <= 0 makes the
  // sigma-point spread imaginary, so reject before dividing.
  if (!std::isfinite(w.lambda) || !std::isfinite(w.c) || !(w.c > 0.0)) {
    return false;
  }
  if (!std::isfinite(w.wm0) || !std::isfinite(w.wc0) || !std::isfinite(w.wi)) {
    return false;
  }
  return w.wc0 >= 0.0;
}

// Closed-form admissible alpha interval at kappa = 0, from W_c0 >= 0. Writes
// [lo, hi] and returns true for any finite beta >= 0. Independent of L.
inline bool alpha_window(double beta, double* lo, double* hi) {
  if (!lo || !hi || !std::isfinite(beta) || beta < 0.0) {
    return false;
  }
  const double b = 2.0 + beta;
  const double disc = b * b - 4.0;  // = beta * (beta + 4) >= 0 for beta >= 0
  if (!(disc >= 0.0)) {
    return false;
  }
  const double root = std::sqrt(disc);
  *lo = std::sqrt(0.5 * (b - root));
  *hi = std::sqrt(0.5 * (b + root));
  return true;
}

// Luo-Moroz sufficient lower bound. For alpha in [1/sqrt(1+beta), 1] the
// weight simplifies to W_c0 = beta / (1 + beta) >= 0, so admissibility holds
// without solving the quadratic. Sufficient, not necessary: the true window
// extends below this bound.
inline double sufficient_alpha_lower(double beta) {
  if (!std::isfinite(beta) || beta < 0.0) {
    return std::nan("");
  }
  return 1.0 / std::sqrt(1.0 + beta);
}

}  // namespace ut
}  // namespace tram_dr
