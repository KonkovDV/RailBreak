// Cross-check of tram_dr_localization/include/tram_dr_localization/ut_weights.hpp
// against the closed-form scaled-UT algebra of docs/math.md section 5.
//
// This file is the DEPENDENT half of the UT algebra check: it includes the
// shipped header and asserts that the header agrees with values derived by
// hand. test_ut_weights_psd.cpp is the INDEPENDENT half: it includes no repo
// header and re-derives the same window from scratch. Keeping both means the
// same algebra mistake has to be made twice, in two different forms, to pass
// CI - which is the point of the exercise, not redundancy.
//
// Runs under NDEBUG: failures are counted and reported through the exit code
// rather than asserted away.

#include <cmath>
#include <cstdio>

#include "tram_dr_localization/ut_weights.hpp"

namespace {

int g_failures = 0;

void check(bool ok, const char* what) {
  if (!ok) {
    std::printf("FAIL %s\n", what);
    ++g_failures;
  }
}

void check_near(double got, double want, double tol, const char* what) {
  if (!std::isfinite(got) || std::fabs(got - want) > tol) {
    std::printf("FAIL %s: got %.15g want %.15g tol %.3g\n", what, got, want, tol);
    ++g_failures;
  }
}

}  // namespace

int main() {
  using tram_dr::ut::Weights;
  using tram_dr::ut::alpha_window;
  using tram_dr::ut::sufficient_alpha_lower;
  using tram_dr::ut::weights;
  using tram_dr::ut::weights_psd_ok;

  // kStateDim = 6 + kNWheels = 6 + 6 = 12. Hard-coded on purpose: this test
  // must not include types.hpp, or it would inherit the value it is checking.
  constexpr int kL = 12;
  constexpr double kAlpha = 0.58;  // shipped default
  constexpr double kBeta = 2.0;
  constexpr double kKappa = 0.0;

  // ---- 1. shipped operating point, exact hand-derived values --------------
  // alpha^2 = 0.3364; lambda = 0.3364*12 - 12 = -7.9632; c = 4.0368
  // W_m0 = -7.9632/4.0368 = -1659/841 = -1.972651605231...
  // W_c0 = W_m0 + (1 - 0.3364 + 2) = -1.972651605231 + 2.6636 = 0.690948394769
  // W_i  = 0.5/4.0368 = 625/5046 = 0.123860483551...
  const Weights w = weights(kAlpha, kBeta, kKappa, kL);
  check_near(w.lambda, -7.9632, 1e-12, "lambda at shipped defaults");
  check_near(w.c, 4.0368, 1e-12, "c at shipped defaults");
  check_near(w.wm0, -1659.0 / 841.0, 1e-12, "W_m0 at shipped defaults");
  check_near(w.wm0, -1.972651605231, 1e-9, "W_m0 decimal at shipped defaults");
  check_near(w.wc0, 0.690948394769, 1e-9, "W_c0 at shipped defaults");
  check_near(w.wi, 625.0 / 5046.0, 1e-12, "W_i at shipped defaults");
  check_near(w.wi, 0.123860483551, 1e-9, "W_i decimal at shipped defaults");
  check(w.wc0 >= 0.0, "W_c0 >= 0 at shipped defaults");
  check(weights_psd_ok(kAlpha, kBeta, kKappa, kL), "shipped defaults admissible");

  // ---- 2. weight normalisation identities --------------------------------
  const double sum_m = w.wm0 + 2.0 * kL * w.wi;
  const double sum_c = w.wc0 + 2.0 * kL * w.wi;
  check_near(sum_m, 1.0, 1e-12, "sum(W_m) == 1");
  check_near(sum_c, 1.0 + (1.0 - kAlpha * kAlpha + kBeta), 1e-12,
             "sum(W_c) == 1 + (1 - alpha^2 + beta)");
  check_near(sum_c, 3.6636, 1e-12, "sum(W_c) == 3.6636");

  // ---- 3. W_c0 is L-independent at kappa = 0 -----------------------------
  // W_m0 = (alpha^2 L - L) / (alpha^2 L) = 1 - 1/alpha^2, so L cancels.
  const double wc0_ref = 2.0 + kBeta - kAlpha * kAlpha - 1.0 / (kAlpha * kAlpha);
  check_near(w.wc0, wc0_ref, 1e-12, "W_c0 == 2 + beta - a^2 - 1/a^2");
  for (const int L : {1, 2, 4, 12, 13, 25, 64}) {
    const Weights wl = weights(kAlpha, kBeta, kKappa, L);
    check_near(wl.wc0, wc0_ref, 1e-12, "W_c0 independent of L at kappa=0");
    check_near(wl.wm0, 1.0 - 1.0 / (kAlpha * kAlpha), 1e-12,
               "W_m0 independent of L at kappa=0");
  }

  // ---- 4. closed-form windows -------------------------------------------
  double lo = 0.0, hi = 0.0;
  check(alpha_window(2.0, &lo, &hi), "alpha_window(beta=2) succeeds");
  check_near(lo, std::sqrt(2.0 - std::sqrt(3.0)), 1e-15, "beta=2 lower edge");
  check_near(hi, std::sqrt(2.0 + std::sqrt(3.0)), 1e-15, "beta=2 upper edge");
  check_near(lo, 0.517638090205041, 1e-12, "beta=2 lower edge decimal");
  check_near(hi, 1.931851652578137, 1e-12, "beta=2 upper edge decimal");

  check(alpha_window(1.0, &lo, &hi), "alpha_window(beta=1) succeeds");
  check_near(lo, (std::sqrt(5.0) - 1.0) / 2.0, 1e-12, "beta=1 lower edge is 1/phi");
  check_near(hi, (std::sqrt(5.0) + 1.0) / 2.0, 1e-12, "beta=1 upper edge is phi");

  check(alpha_window(0.0, &lo, &hi), "alpha_window(beta=0) succeeds");
  check_near(lo, 1.0, 1e-12, "beta=0 window collapses to alpha=1 (lower)");
  check_near(hi, 1.0, 1e-12, "beta=0 window collapses to alpha=1 (upper)");
  check(weights_psd_ok(1.0, 0.0, 0.0, kL), "alpha=1, beta=0 admissible");

  check(!alpha_window(-1.0, &lo, &hi), "alpha_window rejects beta < 0");
  check(!alpha_window(std::nan(""), &lo, &hi), "alpha_window rejects NaN beta");

  // ---- 5. THE hazard F-08 exists to catch --------------------------------
  // The shipped alpha is fine at beta = 2 and inadmissible at beta = 1 and
  // beta = 0. Editing beta alone, with alpha untouched and each parameter
  // still inside its own documented range, silently breaks PSD-ness.
  check(weights_psd_ok(0.58, 2.0, 0.0, kL), "alpha=0.58 admissible at beta=2");
  check(!weights_psd_ok(0.58, 1.0, 0.0, kL), "alpha=0.58 INADMISSIBLE at beta=1");
  check(!weights_psd_ok(0.58, 0.0, 0.0, kL), "alpha=0.58 INADMISSIBLE at beta=0");
  check_near(weights(0.58, 1.0, 0.0, kL).wc0, -0.309051605231, 1e-9,
             "W_c0 at alpha=0.58, beta=1");
  check_near(weights(0.58, 0.0, 0.0, kL).wc0, -1.309051605231, 1e-9,
             "W_c0 at alpha=0.58, beta=0");

  // ---- 6. rejections ----------------------------------------------------
  check(!weights_psd_ok(0.50, 2.0, 0.0, kL), "alpha below beta=2 lower edge");
  check(!weights_psd_ok(2.00, 2.0, 0.0, kL), "alpha above beta=2 upper edge");
  check(!weights_psd_ok(0.0, 2.0, 0.0, kL), "alpha=0 gives c=0");
  check(!weights_psd_ok(0.58, 2.0, -static_cast<double>(kL), kL),
        "kappa = -L gives c=0");
  check(!weights_psd_ok(0.58, 2.0, -static_cast<double>(kL) - 1.0, kL),
        "kappa < -L gives c<0");
  check(!weights_psd_ok(std::nan(""), 2.0, 0.0, kL), "NaN alpha rejected");
  check(!weights_psd_ok(0.58, std::nan(""), 0.0, kL), "NaN beta rejected");
  check(!weights_psd_ok(0.58, 2.0, std::nan(""), kL), "NaN kappa rejected");
  check(!weights_psd_ok(0.58, -1.0, 0.0, kL), "negative beta rejected");
  check(!weights_psd_ok(0.58, 2.0, 0.0, 0), "L=0 rejected");
  check(!weights_psd_ok(0.58, 2.0, 0.0, -3), "negative L rejected");

  // ---- 7. Luo-Moroz sufficient bound ------------------------------------
  for (const double beta : {0.0, 0.5, 1.0, 2.0, 3.0, 7.0}) {
    const double a_suf = sufficient_alpha_lower(beta);
    check(std::isfinite(a_suf), "sufficient bound finite");
    check(weights_psd_ok(a_suf, beta, 0.0, kL), "sufficient bound is admissible");
    // At the bound the weight is exactly beta/(1+beta).
    check_near(weights(a_suf, beta, 0.0, kL).wc0, beta / (1.0 + beta), 1e-12,
               "W_c0 == beta/(1+beta) at the sufficient bound");
    // It is sufficient, not necessary: the true edge sits at or below it.
    double wlo = 0.0, whi = 0.0;
    check(alpha_window(beta, &wlo, &whi), "window for sufficiency comparison");
    check(wlo <= a_suf + 1e-15, "true lower edge <= sufficient bound");
  }
  check_near(sufficient_alpha_lower(2.0), 1.0 / std::sqrt(3.0), 1e-15,
             "sufficient bound at beta=2 is 1/sqrt(3)");
  // Shipped margin over the sufficient bound: 0.58/0.5773502691896258 - 1.
  check_near(kAlpha / sufficient_alpha_lower(kBeta) - 1.0, 0.00459, 5e-6,
             "shipped alpha clears Luo-Moroz bound by 0.459 %");
  // Shipped margin over the true lower edge.
  check_near(kAlpha / std::sqrt(2.0 - std::sqrt(3.0)) - 1.0, 0.12053, 5e-5,
             "shipped alpha clears the true beta=2 edge by 12.05 %");

  // ---- 8. predicate agrees with the closed form on a fine grid ----------
  // The predicate and the window are two different computations; they must
  // classify every grid point identically away from the boundary.
  for (const double beta : {0.0, 0.25, 0.5, 1.0, 2.0, 3.0, 5.0}) {
    double wlo = 0.0, whi = 0.0;
    if (!alpha_window(beta, &wlo, &whi)) {
      check(false, "grid: alpha_window failed");
      continue;
    }
    for (int k = 1; k <= 4000; ++k) {
      const double alpha = 0.0005 * static_cast<double>(k);  // 0.0005 .. 2.0
      const bool inside = (alpha >= wlo) && (alpha <= whi);
      const bool predicate = weights_psd_ok(alpha, beta, 0.0, kL);
      // Skip a narrow collar around each edge where double rounding can
      // legitimately disagree about a boundary point.
      const double collar = 1e-9;
      if (std::fabs(alpha - wlo) < collar || std::fabs(alpha - whi) < collar) {
        continue;
      }
      if (inside != predicate) {
        std::printf("FAIL grid: beta=%.4g alpha=%.6f window=%d predicate=%d\n",
                    beta, alpha, static_cast<int>(inside),
                    static_cast<int>(predicate));
        ++g_failures;
      }
    }
  }

  if (g_failures == 0) {
    std::printf("test_ut_weights_header: all checks passed\n");
    return 0;
  }
  std::printf("test_ut_weights_header: %d failure(s)\n", g_failures);
  return 1;
}
