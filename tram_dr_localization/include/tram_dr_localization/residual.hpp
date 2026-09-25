#pragma once
// Linear-in-parameters residual force φ(v, n, b)ᵀθ. Default-off.
// Cubic clamped B-splines in speed (knots 0, 2, 5, 10, 15, 20 m/s)
// tensored with {1, n+, n−, b}, plus one lag of n+ and of b.
// 8 × 4 + 2 = 34 coefficients. Outside [0, 20] m/s the residual is off.

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

namespace tram_dr {

inline constexpr int kResidualCoeff = 34;
inline constexpr int kResidualSpline = 8;

struct ResidualModel {
  bool enabled{false};
  double h_max{0.5};
  double sigma2{1.0};
  int n{kResidualCoeff};
  std::array<double, kResidualCoeff> theta{};
  bool have_info{false};
  std::array<double, kResidualCoeff * kResidualCoeff> info_inv{};
  double n_lag{0.0};
  double b_lag{0.0};
};

inline void residual_splines(double v_mps, double* b) {
  // Clamped knot vector, degree 3. Partition of unity on [0, 20].
  constexpr double kT[] = {0, 0, 0, 0, 2, 5, 10, 15, 20, 20, 20, 20};
  constexpr int kP = 3;
  const double x = std::clamp(v_mps, 0.0, 20.0);
  double n0[12]{};
  for (int i = 0; i < 11; ++i) {
    if (kT[i] <= x && x < kT[i + 1]) {
      n0[i] = 1.0;
    }
  }
  if (x >= 20.0) {
    n0[7] = 1.0;
  }
  double cur[12];
  double nxt[12];
  for (int i = 0; i < 12; ++i) {
    cur[i] = n0[i];
  }
  for (int p = 1; p <= kP; ++p) {
    for (int i = 0; i < 12; ++i) {
      nxt[i] = 0.0;
    }
    const int nbasis = 12 - p - 1;
    for (int i = 0; i < nbasis; ++i) {
      const double d1 = kT[i + p] - kT[i];
      const double d2 = kT[i + p + 1] - kT[i + 1];
      double a = 0.0;
      double c = 0.0;
      if (d1 > 0.0) {
        a = (x - kT[i]) / d1 * cur[i];
      }
      if (d2 > 0.0) {
        c = (kT[i + p + 1] - x) / d2 * cur[i + 1];
      }
      nxt[i] = a + c;
    }
    for (int i = 0; i < 12; ++i) {
      cur[i] = nxt[i];
    }
  }
  for (int i = 0; i < kResidualSpline; ++i) {
    b[i] = cur[i];
  }
  if (x >= 20.0) {
    for (int i = 0; i < kResidualSpline; ++i) {
      b[i] = 0.0;
    }
    b[kResidualSpline - 1] = 1.0;
  }
}

inline void residual_basis(double v_mps, double notch, double brake, double n_lag,
                           double b_lag, double* phi) {
  double spl[kResidualSpline];
  residual_splines(v_mps, spl);
  const double n_pos = std::max(notch, 0.0);
  const double n_neg = std::max(-notch, 0.0);
  const double br = std::clamp(brake, 0.0, 1.0);
  const double gate[4] = {1.0, n_pos, n_neg, br};
  int k = 0;
  for (int g = 0; g < 4; ++g) {
    for (int i = 0; i < kResidualSpline; ++i) {
      phi[k++] = spl[i] * gate[g];
    }
  }
  phi[k++] = n_lag;
  phi[k++] = b_lag;
}

struct ResidualForce {
  double force_n{0.0};
  double var_n2{0.0};
  bool applied{false};
};

inline ResidualForce residual_force(const ResidualModel& m, double v_mps, double notch,
                                    double brake) {
  ResidualForce out;
  if (!m.enabled || m.n <= 0) {
    return out;
  }
  if (!std::isfinite(v_mps) || v_mps < 0.0 || v_mps > 20.0) {
    return out;
  }
  double phi[kResidualCoeff]{};
  const int n = std::min(m.n, kResidualCoeff);
  residual_basis(v_mps, notch, brake, m.n_lag, m.b_lag, phi);
  if (m.have_info && m.h_max > 0.0) {
    double h = 0.0;
    for (int i = 0; i < n; ++i) {
      double acc = 0.0;
      for (int j = 0; j < n; ++j) {
        acc += m.info_inv[static_cast<std::size_t>(i * kResidualCoeff + j)] * phi[j];
      }
      h += phi[i] * acc;
    }
    if (!(h <= m.h_max)) {
      return out;
    }
    out.var_n2 = std::max(m.sigma2, 0.0) * std::max(h, 0.0);
  } else {
    out.var_n2 = std::max(m.sigma2, 0.0);
  }
  double f = 0.0;
  for (int i = 0; i < n; ++i) {
    f += phi[i] * m.theta[static_cast<std::size_t>(i)];
  }
  if (!std::isfinite(f)) {
    return ResidualForce{};
  }
  out.force_n = f;
  out.applied = true;
  return out;
}

inline std::string residual_yaml_trim(std::string s) {
  const auto hash = s.find('#');
  if (hash != std::string::npos) {
    s = s.substr(0, hash);
  }
  std::size_t a = 0;
  std::size_t b = s.size();
  while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) {
    ++a;
  }
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) {
    --b;
  }
  return s.substr(a, b - a);
}

// Minimal YAML for fit_residual.py: enabled, sigma2, h_max, n, theta: […].
inline bool parse_residual_yaml_text(const std::string& text, ResidualModel* m) {
  if (m == nullptr || text.find("theta:") == std::string::npos) {
    return false;
  }
  std::string flat;
  int depth = 0;
  for (char c : text) {
    if (c == '[') {
      ++depth;
    }
    if (depth > 0 && (c == '\n' || c == '\r')) {
      flat.push_back(' ');
    } else {
      flat.push_back(c);
    }
    if (c == ']') {
      --depth;
    }
  }
  ResidualModel out;
  bool have_theta = false;
  std::istringstream in(flat);
  std::string line;
  while (std::getline(in, line)) {
    const std::string t = residual_yaml_trim(line);
    if (t.empty()) {
      continue;
    }
    const auto col = t.find(':');
    if (col == std::string::npos) {
      continue;
    }
    const std::string key = residual_yaml_trim(t.substr(0, col));
    const std::string val = residual_yaml_trim(t.substr(col + 1));
    if (key == "enabled") {
      out.enabled = (val == "true" || val == "True" || val == "yes" || val == "1");
    } else if (key == "sigma2") {
      out.sigma2 = std::strtod(val.c_str(), nullptr);
    } else if (key == "h_max") {
      out.h_max = std::strtod(val.c_str(), nullptr);
    } else if (key == "n") {
      out.n = std::clamp(static_cast<int>(std::strtol(val.c_str(), nullptr, 10)), 1,
                         kResidualCoeff);
    } else if (key == "theta") {
      const auto lb = val.find('[');
      const auto rb = val.rfind(']');
      if (lb == std::string::npos || rb == std::string::npos || rb <= lb) {
        continue;
      }
      std::istringstream nums(val.substr(lb + 1, rb - lb - 1));
      std::string tok;
      int i = 0;
      while (std::getline(nums, tok, ',') && i < kResidualCoeff) {
        const std::string n = residual_yaml_trim(tok);
        if (n.empty()) {
          continue;
        }
        out.theta[static_cast<std::size_t>(i++)] = std::strtod(n.c_str(), nullptr);
      }
      have_theta = i > 0;
      if (i > 0) {
        out.n = std::max(out.n, i);
      }
    }
  }
  if (!have_theta) {
    return false;
  }
  *m = out;
  return true;
}

inline bool load_residual_yaml(const char* path, ResidualModel* m) {
  if (path == nullptr || m == nullptr) {
    return false;
  }
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  return parse_residual_yaml_text(ss.str(), m);
}

}  // namespace tram_dr
