// Replay scaled UKF on filter.csv (no GT columns). Offline only.
#include "tram_dr_localization/ukf.hpp"
#include "tram_dr_localization/vehicle_yaml.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
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

int wheel_index(const std::string& h) {
  if (h.size() < 2 || (h[0] != 'w' && h[0] != 'W')) {
    return -1;
  }
  if (!std::isdigit(static_cast<unsigned char>(h[1]))) {
    return -1;
  }
  return std::atoi(h.c_str() + 1);
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

bool load_csv(const char* path, std::vector<Row>& rows) {
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  std::string line;
  if (!std::getline(in, line)) {
    return false;
  }
  const std::vector<std::string> header = split_csv(line);
  int col_t = 0;
  int col_notch = 1;
  int col_brake = 2;
  std::vector<int> wcols;
  for (int i = 0; i < static_cast<int>(header.size()); ++i) {
    const std::string& h = header[static_cast<std::size_t>(i)];
    if (h == "t_s" || h == "t") {
      col_t = i;
    } else if (h == "notch") {
      col_notch = i;
    } else if (h == "brake") {
      col_brake = i;
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
  if (n_w <= 0) {
    n_w = 4;
    wcols = {3, 4, 5, 6};
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
    const auto at = [&](int col) -> double {
      if (col < 0 || col >= static_cast<int>(cells.size())) {
        return 0.0;
      }
      return std::strtod(cells[static_cast<std::size_t>(col)].c_str(), nullptr);
    };
    r.t = at(col_t);
    r.u.notch = at(col_notch);
    r.u.notch_valid = true;
    r.u.brake = at(col_brake);
    r.n = n_w;
    for (int i = 0; i < n_w && i < tram_dr::kNWheels; ++i) {
      const int col = (i < static_cast<int>(wcols.size())) ? wcols[static_cast<std::size_t>(i)] : -1;
      r.omega[i] = at(col);
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
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--vehicle") == 0 && i + 1 < argc) {
      vehicle_path = argv[++i];
    } else if (std::strcmp(argv[i], "--route") == 0 && i + 1 < argc) {
      route_path = argv[++i];
    } else if (csv_path == nullptr) {
      csv_path = argv[i];
    } else if (out_path == nullptr) {
      out_path = argv[i];
    }
  }
  if (csv_path == nullptr || out_path == nullptr) {
    std::cerr << "usage: replay_ukf filter.csv out.jsonl [--vehicle vehicle.yaml] "
                 "[--route route_10.yaml]\n";
    return 1;
  }
  std::vector<Row> rows;
  if (!load_csv(csv_path, rows)) {
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
    if (ov.has_mass_door) {
      tram_dr::UkfParams cfg = ukf.params();
      cfg.mass_door_kg = ov.mass_door_kg;
      ukf.set_params(cfg);
    }
    if (ov.has_r0_uncalibrated) {
      tram_dr::UkfParams cfg = ukf.params();
      cfg.r0_uncalibrated = ov.r0_uncalibrated;
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
  for (const Row& r : rows) {
    double dt = r.t - t_prev;
    if (dt <= 0.0) {
      dt = 0.02;
    }
    dt = std::clamp(dt, 0.005, 0.20);
    t_prev = r.t;
    const auto t0 = std::chrono::steady_clock::now();
    const tram_dr::UkfEstimate e =
        ukf.predict_and_update(r.u, r.omega, static_cast<std::size_t>(r.n), dt);
    const auto t1 = std::chrono::steady_clock::now();
    tick_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    out << "{\"kind\":\"est\",\"t\":" << r.t << ",\"s\":" << e.x.s_m << ",\"v\":" << e.x.v_mps
        << ",\"p_ss\":" << e.p_ss << ",\"p_vv\":" << e.p_vv << ",\"confidence\":\""
        << tram_dr::confidence_label(e.confidence) << "\",\"nis\":" << e.nis
        << ",\"nis_valid\":" << (e.nis_valid ? "true" : "false")
        << ",\"nis_cusum\":" << e.nis_cusum
        << ",\"n_frozen\":" << e.n_frozen
        << ",\"over_m\":" << e.over_m << ",\"under_m\":" << e.under_m
        << ",\"b_s\":" << e.b_s_m << ",\"pl_s\":" << e.pl_s_m
        << ",\"pl_v\":" << e.pl_v_mps << ",\"al_s\":" << e.al_s_m
        << ",\"a_kin\":" << e.a_kin_mps2
        << ",\"a_unphysical\":" << (e.a_unphysical ? "true" : "false")
        << ",\"confidence_v\":\"" << tram_dr::confidence_label(e.confidence_v)
        << "\",\"confidence_s\":\"" << tram_dr::confidence_label(e.confidence_s)
        << "\",\"chol_fail\":" << e.chol_fail
        << ",\"s_unobserved_s\":" << e.s_unobserved_s
        << ",\"n_omega_used\":" << e.n_omega_used
        << ",\"n_slip_axles\":" << e.n_slip_axles
        << ",\"relative_wheel_slide\":" << e.relative_wheel_slide
        << ",\"path_disagree_m\":" << e.path_disagree_m
        << ",\"path_disagree_latched\":" << (e.path_disagree_latched ? "true" : "false")
        << ",\"v_chan_a\":" << e.v_chan_a_mps
        << ",\"zupt_at_stop\":" << (e.zupt_at_stop ? "true" : "false")
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
