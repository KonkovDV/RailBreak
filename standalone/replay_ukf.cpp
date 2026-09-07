// Replay scaled UKF on filter.csv (no GT columns). Offline only.
#include "tram_dr_localization/ukf.hpp"
#include "tram_dr_localization/vehicle_yaml.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct Row {
  double t{0.0};
  tram_dr::Input u{};
  double omega[tram_dr::kNWheels]{};
  int n{0};
};

std::string trim_csv(const std::string& s) {
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

// missing_nan: empty / unparsed → NaN (wheels). Else 0 (notch/brake/t).
double parse_csv_double(const std::string& raw, bool missing_nan) {
  const std::string s = trim_csv(raw);
  if (s.empty()) {
    return missing_nan ? std::numeric_limits<double>::quiet_NaN() : 0.0;
  }
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str()) {
    return missing_nan ? std::numeric_limits<double>::quiet_NaN() : 0.0;
  }
  while (end != nullptr && *end != '\0' &&
         std::isspace(static_cast<unsigned char>(*end))) {
    ++end;
  }
  if (end != nullptr && *end != '\0') {
    return missing_nan ? std::numeric_limits<double>::quiet_NaN() : 0.0;
  }
  return v;
}

void json_num(std::ostream& o, const char* key, double v) {
  o << "\"" << key << "\":";
  if (std::isfinite(v)) {
    o << std::defaultfloat << std::setprecision(std::numeric_limits<double>::max_digits10)
      << v;
  } else {
    o << "null";
  }
}

int wheel_index(const std::string& h) {
  if (h.size() != 2 || (h[0] != 'w' && h[0] != 'W')) {
    return -1;
  }
  if (h[1] < '0' || h[1] > '5') {
    return -1;
  }
  return h[1] - '0';
}

std::vector<std::string> split_csv(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    out.push_back(cell);
  }
  return out;
}

bool load_csv(const char* path, std::vector<Row>& rows, bool legacy) {
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  std::string line;
  if (!std::getline(in, line)) {
    return false;
  }
  const std::vector<std::string> header = split_csv(line);
  int col_t = -1;
  int col_notch = -1;
  int col_brake = -1;
  std::vector<int> wcols;
  for (int i = 0; i < static_cast<int>(header.size()); ++i) {
    const std::string& h = header[static_cast<std::size_t>(i)];
    if (h == "t_s" || h == "t") {
      col_t = i;
    } else if (h == "notch") {
      col_notch = i;
    } else if (h == "brake") {
      col_brake = i;
    } else if (h == "gt_s" || h == "gt_v" || h == "gt") {
      continue;
    } else {
      const int wi = wheel_index(h);
      if (wi >= 0 && wi < tram_dr::kNWheels) {
        if (static_cast<int>(wcols.size()) <= wi) {
          wcols.resize(static_cast<std::size_t>(wi + 1), -1);
        }
        wcols[static_cast<std::size_t>(wi)] = i;
      }
    }
  }
  int n_w = static_cast<int>(wcols.size());
  if (legacy && (col_t < 0 || n_w <= 0)) {
    col_t = (col_t < 0) ? 0 : col_t;
    col_notch = (col_notch < 0) ? 1 : col_notch;
    col_brake = (col_brake < 0) ? 2 : col_brake;
    n_w = 4;
    wcols = {3, 4, 5, 6};
  }
  if (col_t < 0 || n_w <= 0) {
    std::cerr << "replay_ukf: CSV needs t_s/t and w0..wN columns (or --legacy-csv)\n";
    return false;
  }
  if (col_notch < 0) {
    col_notch = -1;
  }
  if (col_brake < 0) {
    col_brake = -1;
  }
  while (std::getline(in, line)) {
    if (line.empty()) {
      continue;
    }
    const std::vector<std::string> cells = split_csv(line);
    if (cells.empty()) {
      continue;
    }
    Row r;
    const auto at = [&](int col, bool missing_nan) -> double {
      if (col < 0 || col >= static_cast<int>(cells.size())) {
        return missing_nan ? std::numeric_limits<double>::quiet_NaN() : 0.0;
      }
      return parse_csv_double(cells[static_cast<std::size_t>(col)], missing_nan);
    };
    r.t = at(col_t, true);
    if (!std::isfinite(r.t)) {
      continue;
    }
    r.u.notch = at(col_notch, true);
    r.u.notch_valid = std::isfinite(r.u.notch);
    if (!r.u.notch_valid) {
      r.u.notch = 0.0;
    }
    r.u.brake = at(col_brake, true);
    r.u.brake_valid = std::isfinite(r.u.brake);
    if (!r.u.brake_valid) {
      r.u.brake = 0.0;
    }
    r.n = n_w;
    for (int i = 0; i < n_w && i < tram_dr::kNWheels; ++i) {
      const int col = (i < static_cast<int>(wcols.size())) ? wcols[static_cast<std::size_t>(i)] : -1;
      r.omega[i] = at(col, true);
    }
    rows.push_back(r);
  }
  return !rows.empty();
}

}  // namespace

int main(int argc, char** argv) {
  const char* csv_path = nullptr;
  const char* out_path = nullptr;
  const char* vehicle_path = nullptr;
  const char* route_path = nullptr;
  bool legacy_csv = false;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--vehicle") == 0 && i + 1 < argc) {
      vehicle_path = argv[++i];
    } else if (std::strcmp(argv[i], "--route") == 0 && i + 1 < argc) {
      route_path = argv[++i];
    } else if (std::strcmp(argv[i], "--legacy-csv") == 0) {
      legacy_csv = true;
    } else if (csv_path == nullptr) {
      csv_path = argv[i];
    } else if (out_path == nullptr) {
      out_path = argv[i];
    }
  }
  if (csv_path == nullptr || out_path == nullptr) {
    std::cerr << "usage: replay_ukf filter.csv out.jsonl [--vehicle vehicle.yaml] "
                 "[--route route_10.yaml] [--legacy-csv]\n";
    return 1;
  }
  std::vector<Row> rows;
  if (!load_csv(csv_path, rows, legacy_csv)) {
    std::cerr << "cannot read " << csv_path << "\n";
    return 1;
  }
  std::ofstream out(out_path);
  if (!out) {
    std::cerr << "cannot write " << out_path << "\n";
    return 1;
  }
  tram_dr::Ukf ukf;
  if (vehicle_path != nullptr) {
    tram_dr::VehicleOverlay ov;
    if (!tram_dr::load_vehicle_yaml(vehicle_path, &ov)) {
      std::cerr << "cannot read vehicle yaml " << vehicle_path << "\n";
      return 1;
    }
    tram_dr::PlantParams plant = tram_dr::default_plant_params();
    tram_dr::ScaParams sca{};
    tram_dr::apply_vehicle_overlay(ov, &plant, &sca);
    ukf.set_plant(plant);
    ukf.set_sca(sca);
    tram_dr::UkfParams cfg = ukf.params();
    bool cfg_dirty = false;
    if (ov.n_wheels >= 1 && ov.n_wheels <= tram_dr::kNWheels) {
      cfg.n_wheels = ov.n_wheels;
      cfg_dirty = true;
    }
    if (ov.has_mass_door) {
      cfg.mass_door_kg = ov.mass_door_kg;
      cfg_dirty = true;
    }
    if (ov.has_r0_uncalibrated) {
      cfg.r0_uncalibrated = ov.r0_uncalibrated;
      cfg_dirty = true;
    }
    if (cfg_dirty) {
      ukf.set_params(cfg);
    }
  } else {
    std::cerr << "replay_ukf: Combino NF100 twin plant (pass --vehicle for "
                 "lvenok_moscow / vityaz_m)\n";
  }
  if (route_path != nullptr) {
    tram_dr::UkfParams cfg = ukf.params();
    if (!tram_dr::load_route_s_m(route_path, cfg.stop_s_m.data(), &cfg.n_stops,
                                 tram_dr::kMaxStops, &cfg.stop_gate_m)) {
      std::cerr << "cannot read route_s_m from " << route_path << "\n";
      return 1;
    }
    ukf.set_params(cfg);
  }
  double t_prev = rows.front().t;
  std::vector<double> tick_us;
  tick_us.reserve(rows.size());
  constexpr double kDtMax = 0.20;
  constexpr int kMaxCatchup = 18000;  // 1 h at kDtMax; then clamp leftover
  bool first = true;
  tram_dr::Input hold{};
  bool have_hold = false;
  for (const Row& r : rows) {
    double dt = r.t - t_prev;
    if (first) {
      dt = 0.02;
      first = false;
    } else if (!std::isfinite(dt) || dt <= 0.0) {
      // RT10-01: duplicate / regressing stamps must not invent kDtMin.
      continue;
    }
    t_prev = r.t;
    int catchup = 0;
    const tram_dr::Input gap_u = have_hold ? hold : r.u;
    while (dt > kDtMax && catchup < kMaxCatchup) {
      ukf.predict_and_update(gap_u, nullptr, 0, kDtMax);
      dt -= kDtMax;
      ++catchup;
    }
    if (dt > kDtMax) {
      dt = kDtMax;
    }
    if (dt <= 0.0) {
      continue;
    }
    const auto t0 = std::chrono::steady_clock::now();
    const tram_dr::UkfEstimate e =
        ukf.predict_and_update(r.u, r.omega, static_cast<std::size_t>(r.n), dt);
    const auto t1 = std::chrono::steady_clock::now();
    tick_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    hold = r.u;
    have_hold = true;
    out << "{\"kind\":\"est\",";
    json_num(out, "t", r.t);
    out << ",";
    json_num(out, "s", e.x.s_m);
    out << ",";
    json_num(out, "v", e.x.v_mps);
    out << ",";
    json_num(out, "p_ss", e.p_ss);
    out << ",";
    json_num(out, "p_vv", e.p_vv);
    out << ",\"confidence\":\"" << tram_dr::confidence_label(e.confidence) << "\",";
    json_num(out, "nis", e.nis);
    out << ",\"nis_valid\":" << (e.nis_valid ? "true" : "false") << ",";
    json_num(out, "nis_cusum", e.nis_cusum);
    out << ",\"n_frozen\":" << e.n_frozen
        << ",\"s_unbounded\":" << (e.s_unbounded ? "true" : "false")
        << ",\"zupt_forced\":" << (e.zupt_forced ? "true" : "false")
        << ",\"sca_current\":" << (e.sca_current ? "true" : "false") << ",";
    if (e.s_unbounded) {
      out << "\"over_m\":null,";
      json_num(out, "under_m", e.under_m);
      out << ",\"b_s\":null,\"pl_s\":null,";
    } else {
      json_num(out, "over_m", e.over_m);
      out << ",";
      json_num(out, "under_m", e.under_m);
      out << ",";
      json_num(out, "b_s", e.b_s_m);
      out << ",";
      json_num(out, "pl_s", e.pl_s_m);
      out << ",";
    }
    json_num(out, "pl_v", e.pl_v_mps);
    out << ",";
    json_num(out, "al_s", e.al_s_m);
    out << ",";
    json_num(out, "a_kin", e.a_kin_mps2);
    out << ",\"a_unphysical\":" << (e.a_unphysical ? "true" : "false")
        << ",\"confidence_v\":\"" << tram_dr::confidence_label(e.confidence_v)
        << "\",\"confidence_s\":\"" << tram_dr::confidence_label(e.confidence_s)
        << "\",\"chol_fail\":" << e.chol_fail << ",";
    json_num(out, "s_unobserved_s", e.s_unobserved_s);
    out << ",\"n_omega_used\":" << e.n_omega_used
        << ",\"n_slip_axles\":" << e.n_slip_axles << ",";
    json_num(out, "relative_wheel_slide", e.relative_wheel_slide);
    out << ",";
    json_num(out, "path_disagree_m", e.path_disagree_m);
    out << ",\"path_disagree_latched\":" << (e.path_disagree_latched ? "true" : "false") << ",";
    json_num(out, "v_chan_a", e.v_chan_a_mps);
    out << ",\"zupt_at_stop\":" << (e.zupt_at_stop ? "true" : "false")
        << ",\"n_wheels\":" << r.n << "}\n";
  }
  if (!tick_us.empty()) {
    std::sort(tick_us.begin(), tick_us.end());
    const auto pct = [&](double p) {
      const std::size_t i = static_cast<std::size_t>(
          std::clamp(p * static_cast<double>(tick_us.size() - 1), 0.0,
                     static_cast<double>(tick_us.size() - 1)));
      return tick_us[i];
    };
    std::cerr << "replay_ukf tick_us p50=" << pct(0.50) << " p99=" << pct(0.99)
              << " n=" << tick_us.size() << "\n";
  }
  return 0;
}
