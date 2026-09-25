#include "tram_dr_localization/sca.hpp"

#include <algorithm>
#include <cmath>

namespace tram_dr {

namespace {

// Street IRW L/R at R≈20–25 m, gauge 1524 mm is ~6–8 %. Above this, treat
// the bogie as a fault and keep the wheels as independent SCA channels.
constexpr double kPairAgreeRel = 0.12;
// Extra 1-sigma so a geometric split does not trip z_thresh.
constexpr double kCurveSigmaRel = 0.08;

double median_copy(std::array<double, kNWheels> v, int n) {
  n = std::min(n, kNWheels);
  std::sort(v.begin(), v.begin() + n);
  if (n <= 0) {
    return 0.0;
  }
  if (n % 2 == 1) {
    return v[n / 2];
  }
  return 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

double radius_m(double d, double r0) { return std::clamp(d, kDMin, kDMax) * r0; }

bool agree_lr(double a, double b) {
  const double mean = 0.5 * (a + b);
  const double scale = std::max(std::fabs(mean), 1.0);
  return std::fabs(a - b) <= kPairAgreeRel * scale;
}

bool is_trailer(const ScaParams& p, int i) {
  return p.axle_role[static_cast<std::size_t>(i)] == kAxleTrailer;
}

}  // namespace

ScaResult sca_analyze(const double* omega, std::size_t n, const double* d_scale,
                      const ScaParams& p) {
  ScaResult out;
  const int m = static_cast<int>(std::min(n, static_cast<std::size_t>(kNWheels)));
  if (m <= 0 || omega == nullptr) {
    return out;
  }
  std::array<double, kNWheels> v_raw{};
  std::array<char, kNWheels> ok{};
  int n_ok = 0;
  for (int i = 0; i < m; ++i) {
    const double d = (d_scale != nullptr) ? d_scale[i] : 1.0;
    const bool finite =
        std::isfinite(omega[i]) && std::fabs(omega[i]) <= kOmegaAbsMax;
    ok[static_cast<std::size_t>(i)] = finite ? 1 : 0;
    v_raw[static_cast<std::size_t>(i)] =
        finite ? omega[i] * radius_m(d, p.r0_m) : 0.0;
    out.inflate[static_cast<std::size_t>(i)] = 1.0;
    if (finite) {
      ++n_ok;
      if (is_trailer(p, i)) {
        ++out.n_trailer_ok;
      }
    }
  }
  if (n_ok == 0) {
    for (int i = 0; i < m; ++i) {
      out.inflate[static_cast<std::size_t>(i)] = p.inflate_max;
      out.r_omega[static_cast<std::size_t>(i)] = 1e6;
      ++out.n_inflated;
      if (is_trailer(p, i)) {
        ++out.n_inflated_trailer;
      } else {
        ++out.n_inflated_motor;
      }
    }
    return out;
  }

  std::array<double, kNWheels> v_work = v_raw;
  if (m >= 4 && p.pair_lr) {
    const auto maybe_pair = [&](int a, int b) {
      if (a >= m || b >= m) {
        return;
      }
      if (!(ok[static_cast<std::size_t>(a)] && ok[static_cast<std::size_t>(b)])) {
        return;
      }
      const double va = v_raw[static_cast<std::size_t>(a)];
      const double vb = v_raw[static_cast<std::size_t>(b)];
      if (!agree_lr(va, vb)) {
        return;
      }
      const double mean = 0.5 * (va + vb);
      v_work[static_cast<std::size_t>(a)] = mean;
      v_work[static_cast<std::size_t>(b)] = mean;
    };
    maybe_pair(0, 1);
    maybe_pair(2, 3);
    if (m >= 6) {
      maybe_pair(4, 5);
    }
  }

  out.used_trailer_consensus = p.traction && out.n_trailer_ok >= 1;
  std::array<double, kNWheels> fin{};
  int nf = 0;
  for (int i = 0; i < m; ++i) {
    if (!ok[static_cast<std::size_t>(i)]) {
      continue;
    }
    if (out.used_trailer_consensus && !is_trailer(p, i)) {
      continue;
    }
    fin[static_cast<std::size_t>(nf++)] = v_work[static_cast<std::size_t>(i)];
  }
  if (nf == 0) {
    for (int i = 0; i < m; ++i) {
      if (ok[static_cast<std::size_t>(i)]) {
        fin[static_cast<std::size_t>(nf++)] = v_work[static_cast<std::size_t>(i)];
      }
    }
    out.used_trailer_consensus = false;
  }
  if (p.consensus == 1 && nf >= 3 && (p.traction || p.braking)) {
    std::array<double, kNWheels> ordered = fin;
    std::sort(ordered.begin(), ordered.begin() + nf);
    // Traction: second-smallest rejects one fast (slipping) axle.
    // Braking: second-largest rejects one slow (sliding) axle.
    // For an odd count and a single outlier these match the median; an
    // asymmetric sample does not. n==3 with one outlier is the median.
    out.v_consensus_mps = (p.braking && !p.traction) ? ordered[static_cast<std::size_t>(nf - 2)]
                                                     : ordered[1];
  } else {
    out.v_consensus_mps = (nf == 1) ? fin[0] : median_copy(fin, nf);
  }

  const double sig0 = std::max(p.sigma_v_mps, 1e-4);
  double sig =
      (m >= 4 && p.pair_lr)
          ? std::max(sig0, kCurveSigmaRel * std::fabs(out.v_consensus_mps))
          : sig0;
  if (p.sigma_v_rel > 0.0) {
    sig = std::max(sig, p.sigma_v_rel * std::fabs(out.v_consensus_mps));
  }

  for (int i = 0; i < m; ++i) {
    const double d = (d_scale != nullptr) ? d_scale[i] : 1.0;
    const double r = radius_m(d, p.r0_m);
    const bool trailer = is_trailer(p, i);
    if (!ok[static_cast<std::size_t>(i)]) {
      out.inflate[static_cast<std::size_t>(i)] = p.inflate_max;
      out.r_omega[static_cast<std::size_t>(i)] = 1e6;
      ++out.n_inflated;
      if (trailer) {
        ++out.n_inflated_trailer;
      } else {
        ++out.n_inflated_motor;
      }
      continue;
    }
    double sigma = sig;
    const double resid =
        std::fabs(v_raw[static_cast<std::size_t>(i)] - out.v_consensus_mps);
    double z = resid / sigma;
    double inf = 1.0;
    while (z > p.z_thresh && inf < p.inflate_max) {
      inf *= 1.5;
      sigma = sig * inf;
      z = resid / sigma;
    }
    if (inf > 1.0 + 1e-9) {
      ++out.n_inflated;
      if (trailer) {
        ++out.n_inflated_trailer;
      } else {
        ++out.n_inflated_motor;
      }
    }
    out.inflate[static_cast<std::size_t>(i)] = inf;
    const double sigma_omega = sigma / std::max(r, 1e-4);
    out.r_omega[static_cast<std::size_t>(i)] = sigma_omega * sigma_omega;
  }
  for (int i = m; i < kNWheels; ++i) {
    out.r_omega[static_cast<std::size_t>(i)] = 1e6;
    out.inflate[static_cast<std::size_t>(i)] = p.inflate_max;
  }
  return out;
}

void sca_inflate(const std::vector<double>& omega, std::vector<double>& r_diag) {
  std::array<double, kNWheels> d{};
  d.fill(1.0);
  const ScaResult s = sca_analyze(omega.data(), omega.size(), d.data(), ScaParams{});
  r_diag.resize(omega.size());
  for (std::size_t i = 0; i < omega.size(); ++i) {
    r_diag[i] = (i < static_cast<std::size_t>(kNWheels)) ? s.r_omega[i] : 1e6;
  }
}

}  // namespace tram_dr
