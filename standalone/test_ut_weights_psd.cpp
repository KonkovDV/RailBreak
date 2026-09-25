// External review artifact - hackathon triage, .
// Standalone by design: no repo headers, no CMake wiring, no dependencies.
//   g++ -O2 -std=c++17 -o /tmp/t standalone/test_ut_weights_psd.cpp && /tmp/t
//
// Checks the scaled-UT weight algebra used by tram_dr::Ukf and, above all, the
// *joint* (alpha, beta, kappa) admissibility condition that docs/math.md states
// in prose but that no runtime validation enforces. alpha and beta are both
// reachable from configuration, so the pair can be driven outside the window
// without any diagnostic.
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <string>

namespace {

int g_failed = 0;
int g_total = 0;

void check(bool ok, const std::string& name) {
  ++g_total;
  if (ok) {
    std::printf("ok   %s\n", name.c_str());
  } else {
    ++g_failed;
    std::printf("FAIL %s\n", name.c_str());
  }
}

struct UtWeights {
  double lambda{0.0};
  double wm0{0.0};
  double wc0{0.0};
  double wi{0.0};
};

UtWeights ut_weights(double alpha, double beta, double kappa, int L) {
  const double Ld = static_cast<double>(L);
  UtWeights w;
  w.lambda = alpha * alpha * (Ld + kappa) - Ld;
  const double c = Ld + w.lambda;
  w.wm0 = w.lambda / c;
  w.wc0 = w.wm0 + (1.0 - alpha * alpha + beta);
  w.wi = 0.5 / c;
  return w;
}

// Proposed guard for validate_ukf(): reject parameter sets whose zeroth
// covariance weight is negative. P = sum_i Wc_i (X_i - x)(X_i - x)^T is then not
// guaranteed positive semi-definite, and project_pd() is forced to repair the
// covariance on every single cycle instead of only on genuine numerical noise.
bool ut_weights_psd_ok(double alpha, double beta, double kappa, int L) {
  const UtWeights w = ut_weights(alpha, beta, kappa, L);
  const double c = static_cast<double>(L) + w.lambda;
  return std::isfinite(w.lambda) && c > 0.0 && w.wc0 >= 0.0;
}

}  // namespace

int main() {
  const double alpha = 0.58;
  const double beta = 2.0;
  const double kappa = 0.0;
  const int L = 12;
  const UtWeights w = ut_weights(alpha, beta, kappa, L);

  // 1. Shipped weights reproduce docs/math.md section 5 to the printed digits.
  check(std::fabs(w.lambda - (-7.9632)) < 1e-4, "lambda == -7.963 (docs/math.md)");
  check(std::fabs(w.wm0 - (-1.972652)) < 1e-6, "Wm0 == -1.973 (docs/math.md)");
  check(std::fabs(w.wc0 - 0.690948) < 1e-6, "Wc0 == +0.691 (docs/math.md)");
  check(std::fabs(w.wi - 0.123860) < 1e-6, "Wi == 0.124 (docs/math.md)");

  double sum_m = w.wm0;
  double sum_c = w.wc0;
  for (int i = 0; i < 2 * L; ++i) {
    sum_m += w.wi;
    sum_c += w.wi;
  }
  check(std::fabs(sum_m - 1.0) < 1e-12, "sum(Wm) == 1");
  check(std::fabs(sum_c - (1.0 + 1.0 - alpha * alpha + beta)) < 1e-12,
        "sum(Wc) == 1 + (1 - alpha^2 + beta) == 3.6636 (expected for scaled UT)");

  // 2. Admissible window at kappa = 0, beta = 2 is [sqrt(2-sqrt3), sqrt(2+sqrt3)].
  const double lo = std::sqrt(2.0 - std::sqrt(3.0));
  const double hi = std::sqrt(2.0 + std::sqrt(3.0));
  check(std::fabs(lo - 0.5176380902) < 1e-9, "lower edge == sqrt(2-sqrt(3)) == 0.5176");
  check(std::fabs(hi - 1.9318516526) < 1e-9, "upper edge == sqrt(2+sqrt(3)) == 1.9319");
  check(ut_weights_psd_ok(lo + 1e-9, beta, kappa, L), "just inside lower edge accepted");
  check(!ut_weights_psd_ok(lo - 1e-6, beta, kappa, L), "just below lower edge rejected");
  check(!ut_weights_psd_ok(hi + 1e-6, beta, kappa, L), "just above upper edge rejected");
  for (int Lx : {4, 8, 10, 12, 25}) {
    check(ut_weights_psd_ok(lo + 1e-9, beta, kappa, Lx),
          "window is L-independent at kappa=0, L=" + std::to_string(Lx));
  }

  // 3. Shipped configuration is admissible, with the margins docs/math.md claims.
  check(ut_weights_psd_ok(alpha, beta, kappa, L), "shipped alpha=0.58, beta=2 admissible");
  check(alpha > lo, "alpha=0.58 above policy edge 0.518");
  check(alpha > 1.0 / std::sqrt(1.0 + beta), "alpha=0.58 above Luo-Moroz 1/sqrt(1+beta)=0.5774");
  check(ut_weights_psd_ok(alpha, beta, 0.25, L), "kappa_cut=0.25 variant admissible");
  check(ut_weights_psd_ok(1.0, beta, kappa, L), "alpha=1 cubature limit admissible");

  // 4. THE GAP. Both alpha and beta are configurable; the joint condition is
  //    not validated anywhere in the core, not even after the parameter
  //    validation added in fix/core-integrity-contracts.
  //    Margin to the Luo-Moroz bound is only 0.46 %, so small edits bite.
  check(!ut_weights_psd_ok(alpha, 1.0, kappa, L),
        "alpha=0.58 with beta=1 must be REJECTED (Wc0 = -0.309)");
  check(!ut_weights_psd_ok(alpha, 0.0, kappa, L),
        "alpha=0.58 with beta=0 must be REJECTED (Wc0 = -1.309)");
  check(!ut_weights_psd_ok(0.1, beta, kappa, L), "alpha=0.1 must be REJECTED");
  check(!ut_weights_psd_ok(0.5, beta, kappa, L), "alpha=0.5 must be REJECTED (Wc0 = -0.25)");

  // RB08-12: W_c0 >= 0 is a sufficient policy for nonnegative-weight P, not a
  // necessary condition for PSD. The guard still rejects the triple.
  {
    const UtWeights wneg = ut_weights(0.5, 2.0, 0.0, 1);
    check(wneg.wc0 < 0.0, "alpha=0.5 beta=2 has Wc0<0");
    check(!ut_weights_psd_ok(0.5, 2.0, 0.0, 1), "policy still rejects Wc0<0");
    const double wi = wneg.wi;
    const double p = wneg.wc0 * 1.0 + 2.0 * wi * 1.0;
    check(p > 0.0, "negative Wc0 can still yield P>0 (sufficient, not necessary)");
  }

  std::printf("\n%d/%d checks passed\n", g_total - g_failed, g_total);
  return g_failed == 0 ? 0 : 1;
}
