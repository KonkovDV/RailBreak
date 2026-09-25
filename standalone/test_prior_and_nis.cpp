// Regression witnesses for F-23 (repeated prior) and F-24 (blinded NIS).
//
// Header-only target: includes prior.hpp and nothing else from the package,
// so it links without the core library and runs under the sanitizers.
// Every reference number here was recomputed independently before being
// written down; the shipped operating point is asserted, not described.

#include "tram_dr_localization/prior.hpp"

#include <cmath>
#include <cstddef>
#include <cstdio>

namespace {

int g_failures = 0;

void check(bool ok, const char* what) {
  if (!ok) {
    std::printf("FAIL %s\n", what);
    ++g_failures;
  }
}

void check_near(double got, double want, double tol, const char* what) {
  if (!(std::fabs(got - want) <= tol)) {
    std::printf("FAIL %s: got %.15g want %.15g tol %g\n", what, got, want, tol);
    ++g_failures;
  }
}

// Shipped operating point of the mass prior.
constexpr double kM0 = 28000.0;
constexpr double kSigmaPrior = 0.3;
constexpr double kMassRwPerStep = (80.0 / kM0) * (80.0 / kM0) * 0.02;

}  // namespace

int main() {
  using namespace tram_dr::prior;

  const double R = kSigmaPrior * kSigmaPrior;
  check_near(R, 0.09, 1e-15, "prior variance on log m");

  // ---------------------------------------------------------------------
  // 1. The defect, pinned numerically. 50 Hz random walk, 1 Hz refusion.
  // ---------------------------------------------------------------------
  const double q_per_sec = kMassRwPerStep * 50.0;
  check_near(q_per_sec, 8.16326530612245e-06, 1e-18, "q on log m per second");

  const double p_bad = repeated_update_fixed_point(R, q_per_sec);
  check_near(p_bad, 8.61234207913811e-04, 1e-14,
             "fixed point of the repeated update");
  check_near(std::sqrt(p_bad), 0.0293468, 1e-7,
             "sigma_log_m the repeated prior converges to");

  // Reached by iteration as well as by the closed form: the closed form is
  // not a fit, it is the fixed point.
  double p_iter = R;
  for (int i = 0; i < 200000; ++i) {
    p_iter = p_iter * R / (p_iter + R) + q_per_sec;
  }
  check_near(p_iter, p_bad, 1e-12, "iteration reaches the closed-form fixed point");

  // Interpretable size of the defect.
  const double n_eff = equivalent_fusions(R, p_bad);
  check(n_eff > 100.0 && n_eff < 106.0,
        "repeated prior is worth ~104 independent fusions");

  // The honest number: prior once, then random walk for the longest
  // unaided gap of route 10.
  const double p_honest = R + q_per_sec * 142.8503;
  check_near(std::sqrt(p_honest), 0.30193, 1e-5, "sigma_log_m after one fusion");
  check(std::sqrt(p_honest) / std::sqrt(p_bad) > 10.0,
        "the repeated prior is >10x overconfident");

  // In kilograms, against the declared Lvenok loading range.
  check_near(std::sqrt(p_bad) * kM0, 821.7, 0.5, "pinned 1-sigma mass, kg");
  check_near(std::sqrt(p_honest) * kM0, 8454.0, 1.0, "honest 1-sigma mass, kg");

  // ---------------------------------------------------------------------
  // 2. The fix: stationary variance is EXACTLY R, for every tau and dt.
  // ---------------------------------------------------------------------
  static const double kTaus[] = {1.0, 30.0, 300.0, 3600.0};
  static const double kDts[] = {0.001, 0.02, 0.1, 0.2};
  for (std::size_t a = 0; a < sizeof(kTaus) / sizeof(kTaus[0]); ++a) {
    for (std::size_t b = 0; b < sizeof(kDts) / sizeof(kDts[0]); ++b) {
      const double phi = gauss_markov_phi(kDts[b], kTaus[a]);
      check(phi > 0.0 && phi < 1.0, "phi is a strict contraction");

      // The variance map is an exact affine contraction toward R with factor
      // phi^2:  P_next - R = phi^2 (P - R).  Asserting the identity is
      // stronger than watching a loop converge and it holds for every
      // starting variance, including 0 and 1e6. It is also the reason the
      // stationary variance cannot be manufactured: R is the only fixed
      // point, approached from both sides, never crossed.
      static const double kStarts[] = {0.0, 1e-9, 1e-6, 0.09, 1.0, 100.0, 1e6};
      for (std::size_t s = 0; s < sizeof(kStarts) / sizeof(kStarts[0]); ++s) {
        const double p0 = kStarts[s];
        const double p1 = gauss_markov_variance(p0, phi, R);
        check_near(p1 - R, phi * phi * (p0 - R),
                   1e-12 * (1.0 + std::fabs(p0)),
                   "variance map contracts toward the prior by phi^2");
        if (p0 < R) {
          check(p1 > p0 - 1e-15 && p1 < R + 1e-15, "grows toward R from below");
        } else if (p0 > R) {
          check(p1 < p0 + 1e-15 && p1 > R - 1e-15, "decays toward R from above");
        }
      }

      // A bounded run must reduce the gap by exactly the predicted factor,
      // for every (tau, dt) pair in the grid.
      {
        const int steps = 20000;
        double p = 1e-6;
        for (int i = 0; i < steps; ++i) {
          p = gauss_markov_variance(p, phi, R);
        }
        const double predicted =
            R + std::pow(phi * phi, static_cast<double>(steps)) * (1e-6 - R);
        check_near(p, predicted, 1e-9, "gap decays by phi^(2n) after n steps");
      }

      // R is a fixed point exactly.
      check_near(gauss_markov_variance(R, phi, R), R, 1e-15,
                 "the prior variance is a fixed point");
    }
  }

  // Mean reversion: monotone, never overshoots, converges to the prior.
  {
    const double phi = gauss_markov_phi(0.02, 30.0);
    const double z = std::log(kM0);
    double x = std::log(15000.0);  // pinned at the low Lvenok clip
    for (int i = 0; i < 50000; ++i) {
      const double next = gauss_markov_mean(x, z, phi);
      check(next >= x - 1e-15, "mean reverts upward monotonically");
      check(next <= z + 1e-12, "mean never overshoots the prior");
      // Exact affine contraction of the gap: (x_next - z) = phi (x - z).
      // Tolerance is set by the cancellation in (z + phi (x - z)) - z, i.e.
      // one ulp of z, not by the contraction itself.
      check_near(next - z, phi * (x - z), 1e-14, "mean contracts by phi");
      x = next;
    }
    // The iteration cannot converge past the resolution of the addition
    // z + phi (x - z). Progress stops once the reversion step (1 - phi)|x-z|
    // falls below half an ulp of z, i.e. at
    //
    //     |x - z| = ulp(z) / (2 (1 - phi))
    //
    // which for z = log 28000 and tau/dt = 1500 is 1.3327e-12. Asserting the
    // closed-form floor instead of a magic tolerance keeps the test honest
    // about what double precision can deliver. In the estimator this floor
    // is ~1e-11 on log m against a prior sigma of 0.3, i.e. 10 orders of
    // magnitude below anything that matters.
    const double ulp_z = std::nextafter(z, 1.0e9) - z;
    const double floor_gap = ulp_z / (2.0 * (1.0 - phi));
    check_near(floor_gap, 1.33271176810331e-12, 1e-18,
               "closed-form convergence floor");
    check(std::fabs(x - z) <= 1.05 * floor_gap,
          "mean converges to the ulp-limited floor");
    check(std::fabs(x - z) >= 0.5 * ulp_z,
          "and does not pretend to be exact");
    check_near(gauss_markov_mean(z, z, phi), z, 1e-15,
               "the prior mean is a fixed point");
  }

  // Cross terms take one factor of phi, not two. A(P)A^T with
  // A = diag(1, .., phi, .., 1) is the only propagation consistent with the
  // variance rule above.
  {
    const double phi = gauss_markov_phi(0.02, 300.0);
    check_near(gauss_markov_cross(1.0, phi), phi, 1e-15,
               "cross-covariance scales by phi");
    check(gauss_markov_cross(1.0, phi) > gauss_markov_variance(0.0, phi, 0.0),
          "cross term is not squared");
  }

  // Degenerate inputs are inert rather than poisonous.
  check_near(gauss_markov_phi(0.0, 300.0), 1.0, 1e-15, "dt = 0 does not revert");
  check_near(gauss_markov_phi(-1.0, 300.0), 1.0, 1e-15, "negative dt does not revert");
  check_near(gauss_markov_phi(0.02, 0.0), 0.0, 1e-15, "tau = 0 reverts fully");
  check(!std::isfinite(repeated_update_fixed_point(-1.0, 1.0)),
        "negative prior variance is rejected");

  // ---------------------------------------------------------------------
  // 3. F-24: the Huber cap saturates the consistency statistic.
  // ---------------------------------------------------------------------
  const double c = 3.0;  // shipped huber_c
  const double c2 = c * c;

  // Inside the cap the statistic is untouched.
  check_near(huber_scale(1.0, 1.0, c), 1.0, 1e-15, "no inflation inside the cap");
  check_near(huber_capped_nis(1.0, 1.0, c), 1.0, 1e-15, "raw NIS inside the cap");
  check_near(huber_scale(3.0, 1.0, c), 1.0, 1e-15, "cap is not active exactly at c");
  check_near(huber_capped_nis(3.0, 1.0, c), c2, 1e-15, "NIS equals c^2 at the edge");

  // Outside the cap it is pinned to c^2 for ANY outlier magnitude. This is
  // the whole finding: a locked wheel and a 200x absurd reading are
  // indistinguishable in the reported statistic.
  static const double kOutliers[] = {5.0, 10.0, 50.0, 1000.0, 1e6};
  for (std::size_t i = 0; i < sizeof(kOutliers) / sizeof(kOutliers[0]); ++i) {
    const double nu = kOutliers[i];
    const double raw = nu * nu;  // S_jj = 1
    check(raw > c2, "outlier really is outside the cap");
    check_near(huber_capped_nis(nu, 1.0, c), c2, 1e-12,
               "capped NIS saturates at c^2");
    check_near(huber_scale(nu, 1.0, c) * c2, raw, 1e-6 * raw,
               "scale times c^2 recovers the raw statistic");
  }

  // Consequence for the CUSUM: with m = 4 live channels the per-epoch
  // increment is bounded even under an arbitrarily large fault.
  {
    const int m = 4;
    const double expected = m + 0.5 * std::sqrt(static_cast<double>(m));
    check_near(expected, 5.0, 1e-15, "CUSUM reference at m = 4");
    const double max_increment = m * c2 - expected;
    check_near(max_increment, 31.0, 1e-12, "bounded CUSUM increment when all capped");
    // The reference sits BELOW mean + 0.5 sd of chi^2_m, so the monitor is
    // deliberately more sensitive than the textbook choice. That part is
    // fail-safe and is kept.
    const double textbook = m + 0.5 * std::sqrt(2.0 * m);
    check(expected < textbook, "shipped CUSUM reference is the conservative one");
    check_near(textbook, 5.41421356237309, 1e-12, "textbook reference m + sd/2");
  }

  // Turning the robustifier off must return the raw statistic untouched.
  check_near(huber_capped_nis(1000.0, 1.0, 0.0), 1.0e6, 1e-3,
             "huber_c = 0 reports the raw NIS");
  check_near(huber_scale(1000.0, 1.0, 0.0), 1.0, 1e-15, "huber_c = 0 does not inflate");

  if (g_failures == 0) {
    std::printf("test_prior_and_nis: all checks passed\n");
    return 0;
  }
  std::printf("test_prior_and_nis: %d failure(s)\n", g_failures);
  return 1;
}
