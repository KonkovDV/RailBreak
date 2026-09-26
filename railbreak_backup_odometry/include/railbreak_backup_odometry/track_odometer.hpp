// Along-track backup odometer. No ROS, no GNSS after init, no allocation per step.
// Mirrors tools/organizer/odometer.py; tools/organizer/lockstep.py compares the two.
//
// State x = [s, v, k, ba]
//   s   coordinate on the closed track ring (m)
//   v   speed (m/s)
//   k   common wheel scale: true speed = k * corrected bogie speed
//   ba  bias of the notch acceleration model (m/s^2)
//
// Three time scales keep calibration, noise and slip apart:
//   rho  rear/front ratio, sign-step median tracker over minutes, off heavy notch;
//        both bogies are corrected half-way to their geometric mean
//   r    measurement noise from the corrected front-rear difference; it grows only
//        while that difference is sign-balanced, a one-signed run freezes it
//   slip normalised innovation gate and a two-bogie consensus gate; a flagged
//        bogie is down-weighted to r_bad, with both flagged the model carries v
// Bogie updates treat k as a consider state (Schmidt-Kalman): wheels and model
// cannot separate v from k, only station anchors move k.
//
//   predict  s += v dt,  v += (a_tab(n, v) - g i(s) + ba) dt
//   update   u_bogie = v / k + e
//   ZUPT     both bogies still for zupt_hold_s  ->  v = 0
//   anchor   once per dwell, unique station within the gate:  s = s_stop + e
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace railbreak {

constexpr double kG = 9.80665;
constexpr double kEarthR = 6378137.0;
constexpr int kNotchMin = -15;
constexpr int kNotchMax = 15;

struct Params {
  double unit = 1.0 / 3.6;  // bogie field -> m/s (recordings are km/h)
  double k0 = 1.0027;
  double sigma_k0 = 0.004;
  double q_k = 1e-9;
  double q_v = 0.05;
  double q_s = 1e-4;
  double q_ba = 1e-5;
  double sigma_ba0 = 0.05;
  double r0 = 0.05 * 0.05;
  double r_min = 0.02 * 0.02;
  double r_max = 1.0;
  double r_alpha = 0.01;
  double r_alpha_grow = 0.0005;
  double bias_alpha = 0.03;
  double rho_step = 2e-5;
  int rho_notch_max = 6;
  double rho_max = 0.08;
  double rho_v_min = 3.0;
  double r_bad = 25.0;
  double nis_gate = 16.0;
  double fr_sigma_gate = 4.0;
  double fr_floor = 0.3;
  double zupt_v = 0.05;
  double zupt_hold_s = 1.0;
  double zupt_noise_mult = 3.0;
  double stop_sd_max = 3.0;
  double stop_sigma_floor = 1.0;
  double stop_gate = 3.0;
  double wheel_stale_s = 0.35;
  double dt_max = 0.2;
  double max_gap_s = 30.0;  // longer gaps: reset the time base, do not integrate
  double v_max = 30.0;      // plausibility bound on a bogie sample (m/s)
  double recover_s = 3.0;   // bogies agree but the model does not: trust bogies after this
  double load_factor = 1.0;  // scales a_tab only; 1 leaves the identified table
  double davis_a = 0.0;      // extra resistance, m/s^2, 1/s, 1/m; 0 is already in a_tab
  double davis_b = 0.0;
  double davis_c = 0.0;
};


struct Stop {
  double s_m;
  double sd_m;
  int n;
};

// Ring map sampled every ~1 m. s is strictly increasing.
struct TrackMap {
  std::vector<double> s, x, y, h, grade;
  double ring_len = 0.0;
  double lat0 = 0.0, lon0 = 0.0;
  double off_ks = 0.0;

  bool empty() const { return s.size() < 2; }

  double wrap(double v) const {
    if (ring_len <= 0.0) return v;
    double w = std::fmod(v, ring_len);
    return w < 0.0 ? w + ring_len : w;
  }

  // Linear interpolation of column c at wrapped s. Empty map (assets missing): 0.
  double at(const std::vector<double>& c, double sv) const {
    if (s.size() < 2 || c.size() != s.size()) return 0.0;
    const double q = wrap(sv);
    if (q <= s.front()) return c.front();
    if (q >= s.back()) return c.back();
    const auto it = std::upper_bound(s.begin(), s.end(), q);
    const std::size_t j = static_cast<std::size_t>(it - s.begin());
    const double t = (q - s[j - 1]) / std::max(s[j] - s[j - 1], 1e-9);
    return c[j - 1] + t * (c[j] - c[j - 1]);
  }

  void enu(double lat, double lon, double& e, double& n) const {
    const double d2r = 3.14159265358979323846 / 180.0;
    e = (lon - lon0) * d2r * kEarthR * std::cos(lat0 * d2r);
    n = (lat - lat0) * d2r * kEarthR;
  }

  // Exact inverse of enu(): map metres back to degrees.
  void latlon(double e, double n, double& lat, double& lon) const {
    const double r2d = 180.0 / 3.14159265358979323846;
    lat = lat0 + n / kEarthR * r2d;
    lon = lon0 + e / (kEarthR * std::cos(lat0 / r2d)) * r2d;
  }
};

struct NotchTable {
  std::vector<double> v_centre;                 // m/s
  std::array<std::vector<double>, 31> a;        // per notch, same length as v_centre

  double lookup(int notch, double v) const {
    const int i = std::clamp(notch, kNotchMin, kNotchMax) - kNotchMin;
    const auto& row = a[static_cast<std::size_t>(i)];
    if (row.empty() || v_centre.empty()) return 0.0;
    const double q = std::max(v, 0.0);
    if (q <= v_centre.front()) return row.front();
    if (q >= v_centre.back()) return row.back();
    const auto it = std::upper_bound(v_centre.begin(), v_centre.end(), q);
    const std::size_t j = static_cast<std::size_t>(it - v_centre.begin());
    const double t = (q - v_centre[j - 1]) / (v_centre[j] - v_centre[j - 1]);
    return row[j - 1] + t * (row[j] - row[j - 1]);
  }
};

namespace detail {
inline std::vector<std::vector<double>> read_csv(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open " + path);
  std::string line;
  std::getline(f, line);  // header
  std::vector<std::vector<double>> rows;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::vector<double> row;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) row.push_back(std::stod(cell));
    rows.push_back(std::move(row));
  }
  return rows;
}

inline double yaml_number(const std::string& path, const std::string& key) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open " + path);
  std::string line;
  while (std::getline(f, line)) {
    const auto p = line.find(':');
    if (p == std::string::npos) continue;
    if (line.substr(0, p) == key) return std::stod(line.substr(p + 1));
  }
  throw std::runtime_error("missing " + key + " in " + path);
}
}  // namespace detail

struct Assets {
  TrackMap map;
  NotchTable table;
  std::vector<Stop> stops;
  double k0 = 1.0;
};

inline Assets load_assets(const std::string& dir) {
  Assets a;
  for (const auto& r : detail::read_csv(dir + "/ring.csv")) {
    if (r.size() < 5) throw std::runtime_error("ring.csv row");
    a.map.s.push_back(r[0]);
    a.map.x.push_back(r[1]);
    a.map.y.push_back(r[2]);
    a.map.h.push_back(r[3]);
    a.map.grade.push_back(r[4]);
  }
  const std::string meta = dir + "/meta.yaml";
  a.map.lat0 = detail::yaml_number(meta, "lat0_deg");
  a.map.lon0 = detail::yaml_number(meta, "lon0_deg");
  a.map.ring_len = detail::yaml_number(meta, "ring_len_m");
  a.map.off_ks = detail::yaml_number(meta, "off_ks_m");
  a.k0 = detail::yaml_number(meta, "k0");
  const auto notch = detail::read_csv(dir + "/notch.csv");
  if (notch.size() != 31) throw std::runtime_error("notch.csv needs 31 rows");
  const std::size_t nv = notch.front().size() - 1;
  for (std::size_t j = 0; j < nv; ++j) a.table.v_centre.push_back(static_cast<double>(j) + 0.5);
  for (const auto& r : notch) {
    const int n = static_cast<int>(std::lround(r[0]));
    if (n < kNotchMin || n > kNotchMax || r.size() != nv + 1) throw std::runtime_error("notch.csv row");
    a.table.a[static_cast<std::size_t>(n - kNotchMin)].assign(r.begin() + 1, r.end());
  }
  for (const auto& r : detail::read_csv(dir + "/stops.csv")) {
    if (r.size() < 3) throw std::runtime_error("stops.csv row");
    a.stops.push_back({r[0], r[1], static_cast<int>(r[2])});
  }
  if (a.map.empty()) throw std::runtime_error("empty ring");
  return a;
}

enum class Mode : std::uint8_t { kUninit = 0, kWheels = 1, kModel = 2, kZupt = 3 };

inline const char* mode_name(Mode m) {
  switch (m) {
    case Mode::kUninit: return "UNINIT";
    case Mode::kWheels: return "WHEELS";
    case Mode::kModel: return "MODEL";
    case Mode::kZupt: return "ZUPT";
  }
  return "?";
}

struct Bogie {
  bool have = false;
  double t = 0.0;
  double u = 0.0;  // m/s
};


class TrackOdometer {
 public:
  static constexpr int N = 4;
  static constexpr int IS = 0, IV = 1, IK = 2, IBA = 3;
  using Vec = std::array<double, N>;
  using Mat = std::array<std::array<double, N>, N>;

  TrackOdometer(const Assets* assets, Params p) : a_(assets), p_(p) {
    p_.k0 = assets->k0;
    reset();
  }

  void reset() {
    x_ = {0.0, 0.0, p_.k0, 0.0};
    P_ = {};
    P_[IS][IS] = 1.0;
    P_[IV][IV] = 0.01;
    P_[IK][IK] = p_.sigma_k0 * p_.sigma_k0;
    P_[IBA][IBA] = p_.sigma_ba0 * p_.sigma_ba0;
    have_t_ = false;
    notch_ = 0;
    front_ = rear_ = Bogie{};
    r_ = p_.r0;
    log_rho_ = 0.0;
    d_bias_ = 0.0;
    still_since_ = -1.0;
    anchored_ = false;
    slip_ = false;
    mode_ = Mode::kWheels;
    n_anchor_ = n_rejected_ = n_gap_reset_ = 0;
    have_v_ = false;
    disagree_since_ = -1.0;
  }

  void init(double s0, double sigma_s0) {
    x_[IS] = s0;
    P_[IS][IS] = sigma_s0 * sigma_s0;
  }

  void set_time(double t) {
    t_ = t;
    have_t_ = true;
  }

  void on_cmd(double t, int position) {
    if (!stamp_ok(t)) return;
    predict(t);
    notch_ = std::clamp(position, kNotchMin, kNotchMax);
  }

  void on_bogie(double t, bool is_front, double raw) {
    // A stamp behind the filter is another subscription, not a step backward.
    // Equal stamps still apply: the two bogies can share a header stamp.
    if (!stamp_ok(t)) return;
    predict(t);
    if (!std::isfinite(raw)) {
      ++n_rejected_;
      return;
    }
    const double u_raw = raw * p_.unit;
    if (std::fabs(u_raw) > p_.v_max) {
      ++n_rejected_;
      return;
    }
    Bogie& me = is_front ? front_ : rear_;
    const Bogie other = is_front ? rear_ : front_;
    me = {true, t, u_raw};
    const bool fresh_other = other.have && t >= other.t && t - other.t < p_.wheel_stale_s && std::isfinite(other.u);
    const double u = corrected(is_front, u_raw);
    if (!have_v_) {
      // The first reading sets the speed; there is nothing to gate it against.
      x_[IV] = std::max(0.0, u * x_[IK]);
      have_v_ = true;
    }
    const double v = x_[IV], k = x_[IK];
    const double pred = v / k;
    Vec h{};
    h[IV] = 1.0 / k;
    h[IK] = -v / (k * k);
    const double r = r_;  // noise level before this pair is learned
    const double S = quad(h) + r;
    const double innov = u - pred;
    bool slip = innov * innov / S > p_.nis_gate;
    bool agree = false;
    if (fresh_other) {
      const double uo = corrected(!is_front, other.u);
      const double gate = std::max(p_.fr_floor, p_.fr_sigma_gate * std::sqrt(2.0 * r));
      agree = std::fabs(u - uo) <= gate;
      if (!agree && std::fabs(u - pred) > std::fabs(uo - pred)) slip = true;
      const double uf = is_front ? u_raw : other.u;
      const double ur = is_front ? other.u : u_raw;
      learn_pair(uf, ur);
    }
    // Both bogies agree with each other but not with the model: a short run is
    // a slide or spin of both, a long one means the model state is wrong (after
    // a dropout). Re-acquire from the bogies.
    if (slip && agree) {
      if (disagree_since_ < 0.0) {
        disagree_since_ = t;
      } else if (t - disagree_since_ >= p_.recover_s) {
        x_[IV] = std::max(0.0, u * k);
        for (int i = 0; i < N; ++i) P_[IV][i] = P_[i][IV] = 0.0;
        P_[IV][IV] = r * k * k;
        disagree_since_ = -1.0;
        slip_ = false;
        zupt(t);
        if (mode_ != Mode::kZupt) mode_ = Mode::kWheels;
        return;
      }
    } else {
      disagree_since_ = -1.0;
    }
    slip_ = slip;
    update_scalar(h, innov, slip ? p_.r_bad : r, true);
    x_[IV] = std::max(0.0, x_[IV]);
    zupt(t);
    if (mode_ != Mode::kZupt) mode_ = slip ? Mode::kModel : Mode::kWheels;
  }

  double s() const { return a_->map.wrap(x_[IS]); }
  double v() const { return x_[IV]; }
  double k() const { return x_[IK]; }
  double model_bias() const { return x_[IBA]; }
  double rear_front_ratio() const { return std::exp(log_rho_); }
  double noise_sd() const { return std::sqrt(r_); }
  double sigma_s() const { return std::sqrt(std::max(P_[IS][IS], 0.0)); }
  double sigma_v() const { return std::sqrt(std::max(P_[IV][IV], 0.0)); }
  bool slip() const { return slip_; }
  Mode mode() const { return mode_; }
  int notch() const { return notch_; }
  int n_anchor() const { return n_anchor_; }
  bool stamp_ok(double t) {
    if (!std::isfinite(t) || (have_t_ && t < t_)) {
      ++n_rejected_;
      return false;
    }
    return true;
  }

  int n_rejected() const { return n_rejected_; }
  int n_gap_reset() const { return n_gap_reset_; }
  double a_model_now() const { return a_model(x_[IV], x_[IS]) + x_[IBA]; }

 private:
  double corrected(bool is_front, double u) const {
    const double half = 0.5 * log_rho_;
    return is_front ? u * std::exp(half) : u * std::exp(-half);
  }

  double a_model(double v, double s) const {
    const double q = std::max(v, 0.0);
    const double tab = a_->table.lookup(notch_, q);
    return p_.load_factor * tab - kG * a_->map.at(a_->map.grade, s) -
           (p_.davis_a + p_.davis_b * q + p_.davis_c * q * q);
  }

  double quad(const Vec& h) const {
    double q = 0.0;
    for (int i = 0; i < N; ++i)
      for (int j = 0; j < N; ++j) q += h[i] * P_[i][j] * h[j];
    return q;
  }

  void learn_pair(double uf, double ur) {
    const double d = corrected(true, uf) - corrected(false, ur);
    const double sd = std::sqrt(2.0 * r_);
    const double e = d - d_bias_;
    d_bias_ += p_.bias_alpha * (d - d_bias_);
    const bool biased = std::fabs(d_bias_) > 2.0 * sd + 0.05;
    if (std::fabs(e) <= p_.fr_sigma_gate * sd) {
      r_ += p_.r_alpha * (0.5 * e * e - r_);
    } else if (!biased) {
      const double ec = p_.fr_sigma_gate * sd;
      r_ += p_.r_alpha_grow * (0.5 * ec * ec - r_);
    }
    r_ = std::clamp(r_, p_.r_min, p_.r_max);
    if (uf > p_.rho_v_min && ur > p_.rho_v_min && std::abs(notch_) <= p_.rho_notch_max) {
      const double lr = std::log(ur / uf);
      if (std::fabs(lr) < p_.rho_max + 0.02) {
        log_rho_ += lr > log_rho_ ? p_.rho_step : -p_.rho_step;
        log_rho_ = std::clamp(log_rho_, -p_.rho_max, p_.rho_max);
      }
    }
  }

  void predict(double t) {
    if (!have_t_) {
      set_time(t);
      return;
    }
    double dt_all = t - t_;
    if (!(dt_all > 0.0)) return;  // regressed or equal stamp: no time passes
    if (dt_all > p_.max_gap_s) {
      ++n_gap_reset_;
      t_ = t;
      return;
    }
    while (dt_all > 1e-9) {
      const double dt = std::min(dt_all, p_.dt_max);
      const double s = x_[IS], v = x_[IV];
      const bool zupt = mode_ == Mode::kZupt;
      const double a = zupt ? 0.0 : a_model(v, s) + x_[IBA];
      const double v_new = zupt ? 0.0 : std::min(p_.v_max, std::max(0.0, v + a * dt));
      x_[IS] = s + 0.5 * (v + v_new) * dt;
      x_[IV] = v_new;
      // P = F P F' + Q, F = I + dt e_s e_v' + (zupt ? 0 : dt) e_v e_ba'
      Mat F{};
      for (int i = 0; i < N; ++i) F[i][i] = 1.0;
      F[IS][IV] = dt;
      F[IV][IBA] = zupt ? 0.0 : dt;
      Mat FP{}, out{};
      for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
          for (int m = 0; m < N; ++m) FP[i][j] += F[i][m] * P_[m][j];
      for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
          for (int m = 0; m < N; ++m) out[i][j] += FP[i][m] * F[j][m];
      out[IS][IS] += p_.q_s * dt;
      out[IV][IV] += p_.q_v * dt;
      out[IK][IK] += p_.q_k * dt;
      out[IBA][IBA] += p_.q_ba * dt;
      P_ = out;
      dt_all -= dt;
    }
    t_ = t;
  }

  void update_scalar(const Vec& h, double innov, double r, bool consider_k) {
    Vec Ph{};
    for (int i = 0; i < N; ++i)
      for (int j = 0; j < N; ++j) Ph[i] += P_[i][j] * h[j];
    double S = r;
    for (int i = 0; i < N; ++i) S += h[i] * Ph[i];
    if (!(S > 0.0) || !std::isfinite(S) || !std::isfinite(innov)) return;
    Vec K{};
    for (int i = 0; i < N; ++i) K[i] = Ph[i] / S;
    if (consider_k) K[IK] = 0.0;
    for (int i = 0; i < N; ++i) x_[i] += K[i] * innov;
    // Joseph form, valid for the suboptimal (consider) gain.
    Mat A{}, AP{}, out{};
    for (int i = 0; i < N; ++i)
      for (int j = 0; j < N; ++j) A[i][j] = (i == j ? 1.0 : 0.0) - K[i] * h[j];
    for (int i = 0; i < N; ++i)
      for (int j = 0; j < N; ++j)
        for (int m = 0; m < N; ++m) AP[i][j] += A[i][m] * P_[m][j];
    for (int i = 0; i < N; ++i)
      for (int j = 0; j < N; ++j) {
        for (int m = 0; m < N; ++m) out[i][j] += AP[i][m] * A[j][m];
        out[i][j] += r * K[i] * K[j];
      }
    P_ = out;
  }

  void zupt(double t) {
    const double thr = std::max(p_.zupt_v, p_.zupt_noise_mult * std::sqrt(r_));
    const bool both_still = front_.have && rear_.have && std::fabs(front_.u) < thr &&
                            std::fabs(rear_.u) < thr &&
                            std::fabs(front_.t - rear_.t) < p_.wheel_stale_s;
    if (!both_still) {
      still_since_ = -1.0;
      anchored_ = false;
      if (mode_ == Mode::kZupt) mode_ = Mode::kWheels;
      return;
    }
    if (still_since_ < 0.0) still_since_ = t;
    if (t - still_since_ >= p_.zupt_hold_s) {
      mode_ = Mode::kZupt;
      x_[IV] = 0.0;
      for (int i = 0; i < N; ++i) P_[IV][i] = P_[i][IV] = 0.0;
      P_[IV][IV] = 1e-6;
      if (!anchored_) {
        anchor();
        anchored_ = true;
      }
    }
  }

  double ring_delta(double a, double b) const {
    double d = a - b;
    const double L = a_->map.ring_len;
    if (L > 0.0) {
      d = std::fmod(d + 0.5 * L, L);
      if (d < 0.0) d += L;
      d -= 0.5 * L;
    }
    return d;
  }

  void anchor() {
    const double s = a_->map.wrap(x_[IS]);
    const double sig = std::sqrt(std::max(P_[IS][IS], 0.0));
    int n_cand = 0;
    double best_d = 0.0, best_r = 0.0;
    for (const auto& st : a_->stops) {
      if (st.sd_m > p_.stop_sd_max) continue;
      const double r_sd = std::max(st.sd_m, p_.stop_sigma_floor);
      const double gate = p_.stop_gate * std::sqrt(sig * sig + r_sd * r_sd);
      const double d = ring_delta(st.s_m, s);
      if (std::fabs(d) <= gate) {
        ++n_cand;
        best_d = d;
        best_r = r_sd;
      }
    }
    if (n_cand != 1) return;  // none, or ambiguous: do not guess
    Vec h{};
    h[IS] = 1.0;
    update_scalar(h, best_d, best_r * best_r, false);
    ++n_anchor_;
  }

  const Assets* a_;
  Params p_;
  Vec x_{};
  Mat P_{};
  double t_ = 0.0;
  bool have_t_ = false;
  int notch_ = 0;
  Bogie front_, rear_;
  double r_ = 0.0, log_rho_ = 0.0, d_bias_ = 0.0;
  double still_since_ = -1.0;
  bool anchored_ = false;
  bool slip_ = false;
  bool have_v_ = false;
  double disagree_since_ = -1.0;
  Mode mode_ = Mode::kWheels;
  int n_anchor_ = 0;
  int n_rejected_ = 0;
  int n_gap_reset_ = 0;
};


inline void wgs84_ecef(double lat_deg, double lon_deg, double h, double& X, double& Y, double& Z) {
  const double d2r = 3.14159265358979323846 / 180.0;
  const double f = 1.0 / 298.257223563;
  const double e2 = f * (2.0 - f);
  const double sl = std::sin(lat_deg * d2r), cl = std::cos(lat_deg * d2r);
  const double N = kEarthR / std::sqrt(1.0 - e2 * sl * sl);
  X = (N + h) * cl * std::cos(lon_deg * d2r);
  Y = (N + h) * cl * std::sin(lon_deg * d2r);
  Z = (N * (1.0 - e2) + h) * sl;
}

// Transverse Mercator on WGS84, origin on the equator, Krueger series to n^6.
// lon0_deg is the central meridian. Northing is from the equator.
inline void tmerc_wgs84(double lat_deg, double lon_deg, double lon0_deg, double k0,
                        double& e, double& n) {
  const double pi = 3.14159265358979323846;
  const double d2r = pi / 180.0;
  const double f = 1.0 / 298.257223563;
  const double nn = f / (2.0 - f);
  const double n2 = nn * nn, n3 = n2 * nn, n4 = n3 * nn, n5 = n4 * nn, n6 = n5 * nn;
  const double A = kEarthR / (1.0 + nn) * (1.0 + n2 / 4.0 + n4 / 64.0 + n6 / 256.0);
  const double al[6] = {
      nn / 2.0 - 2.0 * n2 / 3.0 + 5.0 * n3 / 16.0 + 41.0 * n4 / 180.0 - 127.0 * n5 / 288.0 +
          7891.0 * n6 / 37800.0,
      13.0 * n2 / 48.0 - 3.0 * n3 / 5.0 + 557.0 * n4 / 1440.0 + 281.0 * n5 / 630.0 -
          1983433.0 * n6 / 1935360.0,
      61.0 * n3 / 240.0 - 103.0 * n4 / 140.0 + 15061.0 * n5 / 26880.0 + 167603.0 * n6 / 181440.0,
      49561.0 * n4 / 161280.0 - 179.0 * n5 / 168.0 + 6601661.0 * n6 / 7257600.0,
      34729.0 * n5 / 80640.0 - 3418889.0 * n6 / 1995840.0,
      212378941.0 * n6 / 319334400.0};
  const double phi = lat_deg * d2r;
  const double lam = (lon_deg - lon0_deg) * d2r;
  const double c = 2.0 * std::sqrt(nn) / (1.0 + nn);
  const double t = std::sinh(std::atanh(std::sin(phi)) - c * std::atanh(c * std::sin(phi)));
  const double xi = std::atan2(t, std::cos(lam));
  const double eta = std::atanh(std::sin(lam) / std::sqrt(1.0 + t * t));
  double se = eta, sx = xi;
  for (int j = 1; j <= 6; ++j) {
    se += al[j - 1] * std::cos(2.0 * j * xi) * std::sinh(2.0 * j * eta);
    sx += al[j - 1] * std::sin(2.0 * j * xi) * std::cosh(2.0 * j * eta);
  }
  e = k0 * A * se;
  n = k0 * A * sx;
}

// UTM zone on the same series. False easting 500 km, scale 0.9996.
inline void utm_forward(double lat_deg, double lon_deg, int zone, double& e, double& n) {
  tmerc_wgs84(lat_deg, lon_deg, 6.0 * zone - 183.0, 0.9996, e, n);
  e += 500000.0;
}

// Moscow city grid, GKINP 01-268-02 angles, on WGS84 (the issued GNSS ellipsoid).
// Official MKRS is Bessel 1841. The datum shift is not in the open table.
inline void mkrs_forward(double lat_deg, double lon_deg, double& east, double& north) {
  constexpr double kLat0 = 55.0 + 40.0 / 60.0;
  constexpr double kLon0 = 37.5;
  double e0, n0, e, n;
  tmerc_wgs84(kLat0, kLon0, kLon0, 1.0, e0, n0);
  tmerc_wgs84(lat_deg, lon_deg, kLon0, 1.0, e, n);
  east = e - e0;
  north = n - n0;
}

// kMkrsStart: Moscow grid minus the start point (the spoken rule for a grid
// the checker did not define numerically). kMkrs: the same grid, absolute.
// kMgrs / kGridStart: UTM 37N. kEnu: WGS84 tangent at the start.
enum class FrameMode { kMkrsStart, kMkrs, kMgrs, kGridStart, kEnu };

struct OutputFrame {
  FrameMode mode = FrameMode::kMkrsStart;
  int zone = 37;
  double e_off = 300000.0, n_off = 6100000.0;
  bool x_is_north = false;  // false: x east, y north (ROS). true: Russian X north, Y east
  double lat0 = 0.0, lon0 = 0.0, h0 = 0.0;
  double e0 = 0.0, n0 = 0.0;
  double me0 = 0.0, mn0 = 0.0;

  void set_start(double lat, double lon, double h) {
    lat0 = lat;
    lon0 = lon;
    h0 = h;
    utm_forward(lat, lon, zone, e0, n0);
    mkrs_forward(lat, lon, me0, mn0);
  }

  void to_out(double lat, double lon, double h, double& x, double& y, double& z) const {
    if (mode == FrameMode::kEnu) {
      const double d2r = 3.14159265358979323846 / 180.0;
      double X, Y, Z, X0, Y0, Z0;
      wgs84_ecef(lat, lon, h, X, Y, Z);
      wgs84_ecef(lat0, lon0, h0, X0, Y0, Z0);
      const double dx = X - X0, dy = Y - Y0, dz = Z - Z0;
      const double sl = std::sin(lat0 * d2r), cl = std::cos(lat0 * d2r);
      const double so = std::sin(lon0 * d2r), co = std::cos(lon0 * d2r);
      x = -so * dx + co * dy;
      y = -sl * co * dx - sl * so * dy + cl * dz;
      z = cl * co * dx + cl * so * dy + sl * dz;
      return;
    }
    double e, n;
    if (mode == FrameMode::kMkrs || mode == FrameMode::kMkrsStart) {
      mkrs_forward(lat, lon, e, n);
      if (mode == FrameMode::kMkrsStart) {
        e -= me0;
        n -= mn0;
        z = h - h0;
      } else {
        z = h;
      }
    } else {
      utm_forward(lat, lon, zone, e, n);
      if (mode == FrameMode::kMgrs) {
        e -= e_off;
        n -= n_off;
        z = h;
      } else {
        e -= e0;
        n -= n0;
        z = h - h0;
      }
    }
    if (x_is_north) {
      x = n;
      y = e;
    } else {
      x = e;
      y = n;
    }
  }
};

// Start hypothesis from the GNSS start window: nearest ring points within
// max(3 m, d_min + 0.5 m); master->rover azimuth rejects the opposite track.
struct InitResult {
  bool ok = false;
  double s0 = 0.0;
  double d0 = 0.0;
  bool used_heading = false;
};

inline InitResult init_on_ring(const TrackMap& m, double lat, double lon, bool have_rover,
                               double rlat, double rlon) {
  InitResult r;
  if (m.empty()) return r;
  double e = 0, n = 0;
  m.enu(lat, lon, e, n);
  double he = 0, hn = 0;
  if (have_rover) {
    double re = 0, rn = 0;
    m.enu(rlat, rlon, re, rn);
    he = re - e;
    hn = rn - n;
    r.used_heading = std::hypot(he, hn) > 1.0;
  }
  const std::size_t N = m.s.size();
  double dmin = std::numeric_limits<double>::infinity();
  for (std::size_t j = 0; j < N; ++j) dmin = std::min(dmin, std::hypot(m.x[j] - e, m.y[j] - n));
  const double lim = std::max(3.0, dmin + 0.5);
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t j = 0; j < N; ++j) {
    const double d = std::hypot(m.x[j] - e, m.y[j] - n);
    if (d > lim) continue;
    double penalty = 0.0;
    if (r.used_heading) {
      const std::size_t a = j >= 5 ? j - 5 : 0, b = std::min(j + 5, N - 1);
      const double tx = m.x[b] - m.x[a], ty = m.y[b] - m.y[a];
      const double c = (tx * he + ty * hn) / (std::hypot(tx, ty) * std::hypot(he, hn) + 1e-9);
      penalty = c >= 0.0 ? 0.0 : 100.0;
    }
    if (d + penalty < best) {
      best = d + penalty;
      r.s0 = m.s[j];
      r.d0 = d;
      r.ok = true;
    }
  }
  return r;
}

}  // namespace railbreak
