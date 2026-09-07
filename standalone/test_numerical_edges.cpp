// Independent edge contracts for numerical helpers; no ROS, no assert/NDEBUG.
#include "tram_dr_localization/lin_alg.hpp"
#include "tram_dr_localization/ut_weights.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>

namespace {
int checks = 0;
int failures = 0;
void check(bool ok, const char* label) {
  ++checks;
  if (!ok) {
    ++failures;
    std::cerr << "FAIL: " << label << '\n';
  }
}

// Independent long-double LDL^T of A - floor I: no production Cholesky.
bool above_floor(const double* a, int n, double floor) {
  long double l[16][16]{};
  long double d[16]{};
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < i; ++j) {
      long double s = a[i * n + j];
      for (int k = 0; k < j; ++k) s -= l[i][k] * d[k] * l[j][k];
      l[i][j] = s / d[j];
    }
    long double s = static_cast<long double>(a[i * n + i]) - floor;
    for (int k = 0; k < i; ++k) s -= l[i][k] * l[i][k] * d[k];
    if (!(s > 0) || !std::isfinite(s)) return false;
    d[i] = s;
    l[i][i] = 1;
  }
  return true;
}

void weight_edges() {
  using namespace tram_dr::ut;
  check(!weights_psd_ok(-0.58, 2.0, 0.0, 12), "negative alpha rejected by public guard");
  check(!weights_psd_ok(0.0, 2.0, 0.0, 12), "zero alpha rejected");
  check(weights_psd_ok(0.58, 2.0, 0.0, 12), "default UT still accepted");
  check(!weights_psd_ok(0.58, 1.0, 0.0, 12), "joint beta guard retained");
  for (double beta : {0.0, 1e-16, 1e-12, 0.1, 1.0, 2.0, 1e8, 1e12,
                       1e154, std::numeric_limits<double>::max()}) {
    double lo = -1.0, hi = -1.0;
    const bool ok = alpha_window(beta, &lo, &hi);
    check(ok, "finite nonnegative beta has a window");
    check(std::isfinite(lo) && std::isfinite(hi) && lo > 0 && hi >= lo,
          "window endpoints finite, positive and ordered");
    check(std::isfinite(lo * hi) && std::fabs(lo * hi - 1.0) < 1e-14,
          "window endpoints reciprocal (no subtractive cancellation)");
    if (beta > 0.0 && beta < 1e-10) {
      check(lo < 1.0 && hi > 1.0, "small positive beta is not collapsed to beta=0");
    }
    if (beta >= 0.1) {
      // Derive the boundary from alpha - 1/alpha = sqrt(beta).
      const long double a = hi;
      const long double residual = (a - 1.0L / a) / std::sqrt(static_cast<long double>(beta));
      check(std::fabs(residual - 1.0L) < 1e-14L, "window satisfies independent boundary identity");
    }
  }
  double lo = 7.0, hi = 8.0;
  check(!alpha_window(-1.0, &lo, &hi) && lo == 7.0 && hi == 8.0,
        "invalid window request leaves outputs unchanged");
  check(!alpha_window(2.0, nullptr, &hi), "null window output rejected");
}

void covariance_edges() {
  using namespace tram_dr::la;
  constexpr double floor = 1e-12;
  for (int n = 1; n <= 16; ++n) {
    double a[256]{};
    for (int i = 0; i < n; ++i) a[i * n + i] = (i == 0) ? -1.0 : 2.0 + i;
    check(project_pd(a, n, floor), "indefinite diagonal is repaired, not rolled back at exact floor");
    check(above_floor(a, n, floor), "independent LDL validates repaired spectrum");
    double b[256]{};
    check(project_pd(b, n, floor), "zero covariance is repairable");
    check(above_floor(b, n, floor), "repaired zero covariance clears requested floor");
  }
  double rank_one[4]{1.0, 1.0, 1.0, 1.0};
  check(project_pd(rank_one, 2, floor), "rank-deficient correlated matrix is repaired");
  check(above_floor(rank_one, 2, floor), "rank-one repair verified independently");
  double rotated[4]{499999.9995, 500000.0005, 500000.0005, 499999.9995};
  check(project_pd(rotated, 2, floor), "ill-conditioned rotated covariance is repaired");
  check(above_floor(rotated, 2, floor), "scale-aware repair verified independently");
  double pd[4]{4.0, 1.0, 1.0, 3.0};
  double original[4];
  std::memcpy(original, pd, sizeof(pd));
  check(project_pd(pd, 2, floor) && std::memcmp(pd, original, sizeof(pd)) == 0,
        "healthy symmetric PD matrix preserved byte-for-byte");
  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity()}) {
    double a[4]{1.0, bad, bad, 2.0};
    double saved[4];
    std::memcpy(saved, a, sizeof(a));
    check(!project_pd(a, 2, floor) && std::memcmp(a, saved, sizeof(a)) == 0,
          "nonfinite covariance refused atomically");
  }
  check(!project_pd(nullptr, 2, floor), "null covariance refused");
  check(!project_pd(pd, 17, floor), "oversized covariance refused before access");
  check(!project_pd(pd, 2, -1.0), "negative eigenvalue floor refused");
}
}  // namespace

int main() {
  weight_edges();
  covariance_edges();
  std::cout << checks << " checks, " << failures << " failures\n";
  return failures == 0 ? 0 : 1;
}
