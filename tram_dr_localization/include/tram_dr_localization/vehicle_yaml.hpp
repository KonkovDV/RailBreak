#pragma once
// Minimal ROS-parameter YAML overlay for standalone replay_ukf.
// Not a general YAML parser. Unknown keys are ignored.

#include "tram_dr_localization/plant.hpp"
#include "tram_dr_localization/sca.hpp"
#include "tram_dr_localization/types.hpp"

#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

namespace tram_dr {

struct VehicleOverlay {
  std::string profile;
  int n_wheels{0};  // 0 = leave runtime n to the CSV / node
  bool has_mass{false};
  double mass_kg{28000.0};
  bool has_r0{false};
  double r0_m{0.35};
  bool has_a_trac{false};
  double a_trac_max{1.3};
  bool has_a_svc{false};
  double a_svc{1.2};
  bool has_A{false};
  double A_d{800.0};
  bool has_B{false};
  double B_d{40.0};
  bool has_C{false};
  double C_d{6.0};
  bool has_tau{false};
  double tau_drv_s{0.0};
  bool has_gamma{false};
  double gamma_rot{0.0};
  bool has_rail{false};
  double brake_nonadhesive_frac{0.0};
  bool has_notch_accel{false};
  bool notch_as_accel{false};
  bool has_grade{false};
  // Optional scalar map prior. Shipped YAML must leave this unset / 0.
  // Do not load a Strogino OSINT estimate here — i(s) lives in F_bias.
  double i_grade{0.0};
  bool has_jerk{false};
  double j_max_mps3{0.0};
  bool has_v_base{false};
  double v_base_mps{6.0};
  bool has_mass_min{false};
  double mass_min_kg{20000.0};
  bool has_mass_max{false};
  double mass_max_kg{70000.0};
  bool has_mass_door{false};
  double mass_door_kg{3000.0};
  bool has_pair_lr{false};
  bool pair_lr{true};
  bool has_axle_role{false};
  std::array<int, kNWheels> axle_role{};
  bool has_sigma_v_rel{false};
  double sigma_v_rel{0.0};
  bool has_r0_uncalibrated{false};
  bool r0_uncalibrated{false};
};

inline std::string yaml_trim(std::string s) {
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

inline bool yaml_bool(const std::string& v, bool* out) {
  if (v == "true" || v == "True" || v == "yes") {
    *out = true;
    return true;
  }
  if (v == "false" || v == "False" || v == "no") {
    *out = false;
    return true;
  }
  return false;
}

inline bool parse_vehicle_yaml_text(const std::string& text, VehicleOverlay* o) {
  if (o == nullptr) {
    return false;
  }
  *o = VehicleOverlay{};
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    const std::string t = yaml_trim(line);
    if (t.empty() || t == "/**:" || t == "ros__parameters:") {
      continue;
    }
    const auto col = t.find(':');
    if (col == std::string::npos) {
      continue;
    }
    const std::string key = yaml_trim(t.substr(0, col));
    std::string val = yaml_trim(t.substr(col + 1));
    if (key.empty() || val.empty()) {
      continue;
    }
    if (key == "vehicle_profile") {
      o->profile = val;
    } else if (key == "n_wheels") {
      o->n_wheels = std::atoi(val.c_str());
    } else if (key == "mass_kg") {
      o->has_mass = true;
      o->mass_kg = std::strtod(val.c_str(), nullptr);
    } else if (key == "wheel_radius_m") {
      o->has_r0 = true;
      o->r0_m = std::strtod(val.c_str(), nullptr);
    } else if (key == "a_trac_max") {
      o->has_a_trac = true;
      o->a_trac_max = std::strtod(val.c_str(), nullptr);
    } else if (key == "a_svc") {
      o->has_a_svc = true;
      o->a_svc = std::strtod(val.c_str(), nullptr);
    } else if (key == "A_d") {
      o->has_A = true;
      o->A_d = std::strtod(val.c_str(), nullptr);
    } else if (key == "B_d") {
      o->has_B = true;
      o->B_d = std::strtod(val.c_str(), nullptr);
    } else if (key == "C_d") {
      o->has_C = true;
      o->C_d = std::strtod(val.c_str(), nullptr);
    } else if (key == "tau_drv_s") {
      o->has_tau = true;
      o->tau_drv_s = std::strtod(val.c_str(), nullptr);
    } else if (key == "gamma_rot") {
      o->has_gamma = true;
      o->gamma_rot = std::strtod(val.c_str(), nullptr);
    } else if (key == "brake_nonadhesive_frac") {
      o->has_rail = true;
      o->brake_nonadhesive_frac = std::strtod(val.c_str(), nullptr);
    } else if (key == "i_grade") {
      o->has_grade = true;
      o->i_grade = std::strtod(val.c_str(), nullptr);
    } else if (key == "j_max_mps3") {
      o->has_jerk = true;
      o->j_max_mps3 = std::strtod(val.c_str(), nullptr);
    } else if (key == "mass_min_kg") {
      o->has_mass_min = true;
      o->mass_min_kg = std::strtod(val.c_str(), nullptr);
    } else if (key == "mass_max_kg") {
      o->has_mass_max = true;
      o->mass_max_kg = std::strtod(val.c_str(), nullptr);
    } else if (key == "mass_door_kg") {
      o->has_mass_door = true;
      o->mass_door_kg = std::strtod(val.c_str(), nullptr);
    } else if (key == "v_base_mps") {
      o->has_v_base = true;
      o->v_base_mps = std::strtod(val.c_str(), nullptr);
    } else if (key == "sca_sigma_v_rel") {
      o->has_sigma_v_rel = true;
      o->sigma_v_rel = std::strtod(val.c_str(), nullptr);
    } else if (key == "r0_uncalibrated") {
      bool b = false;
      if (yaml_bool(val, &b)) {
        o->has_r0_uncalibrated = true;
        o->r0_uncalibrated = b;
      }
    } else if (key == "notch_as_accel") {
      bool b = false;
      if (yaml_bool(val, &b)) {
        o->has_notch_accel = true;
        o->notch_as_accel = b;
      }
    } else if (key == "sca_pair_lr") {
      bool b = false;
      if (yaml_bool(val, &b)) {
        o->has_pair_lr = true;
        o->pair_lr = b;
      }
    } else if (key == "axle_role") {
      o->has_axle_role = true;
      o->axle_role.fill(0);
      int i = 0;
      std::string num;
      for (char c : val) {
        if (std::isdigit(static_cast<unsigned char>(c)) || c == '-') {
          num.push_back(c);
        } else if (!num.empty()) {
          if (i < kNWheels) {
            o->axle_role[static_cast<std::size_t>(i++)] = std::atoi(num.c_str());
          }
          num.clear();
        }
      }
      if (!num.empty() && i < kNWheels) {
        o->axle_role[static_cast<std::size_t>(i)] = std::atoi(num.c_str());
      }
    }
  }
  return true;
}

inline bool load_vehicle_yaml(const char* path, VehicleOverlay* o) {
  if (path == nullptr || o == nullptr) {
    return false;
  }
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  return parse_vehicle_yaml_text(ss.str(), o);
}

// True if pos sits on a `# …` comment line (not a live YAML key).
inline bool yaml_key_on_comment_line(const std::string& text, std::size_t pos) {
  std::size_t i = pos;
  while (i > 0 && text[i - 1] != '\n' && text[i - 1] != '\r') {
    --i;
  }
  while (i < pos && (text[i] == ' ' || text[i] == '\t')) {
    ++i;
  }
  return i < text.size() && text[i] == '#';
}

inline std::size_t find_live_yaml_key(const std::string& text, const char* key) {
  const std::string k(key);
  std::size_t from = 0;
  while (from < text.size()) {
    const auto p = text.find(k, from);
    if (p == std::string::npos) {
      return std::string::npos;
    }
    if (!yaml_key_on_comment_line(text, p)) {
      return p;
    }
    from = p + k.size();
  }
  return std::string::npos;
}

// Stop abscissae from route_10.yaml. First live route_s_m: [...] wins. Not i(s).
inline bool parse_route_s_m(const std::string& text, double* s, int* n, int cap) {
  if (s == nullptr || n == nullptr || cap <= 0) {
    return false;
  }
  const auto key = find_live_yaml_key(text, "route_s_m:");
  if (key == std::string::npos) {
    return false;
  }
  const auto lb = text.find('[', key);
  const auto rb = text.find(']', lb);
  if (lb == std::string::npos || rb == std::string::npos || rb <= lb) {
    return false;
  }
  *n = 0;
  std::string num;
  const auto take = [&]() {
    if (num.empty() || *n >= cap) {
      num.clear();
      return;
    }
    const double v = std::strtod(num.c_str(), nullptr);
    if (std::isfinite(v)) {
      s[(*n)++] = v;
    }
    num.clear();
  };
  for (std::size_t i = lb + 1; i < rb; ++i) {
    const unsigned char c = static_cast<unsigned char>(text[i]);
    if (std::isdigit(c) || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E') {
      num.push_back(static_cast<char>(c));
    } else if (!num.empty()) {
      take();
    }
  }
  take();
  return *n > 0;
}

inline bool parse_stop_gate_m(const std::string& text, double* gate) {
  if (gate == nullptr) {
    return false;
  }
  const auto key = find_live_yaml_key(text, "stop_gate_m:");
  if (key == std::string::npos) {
    return false;
  }
  const char* p = text.c_str() + key + 12;  // "stop_gate_m:"
  char* end = nullptr;
  const double v = std::strtod(p, &end);
  if (end == p || !std::isfinite(v) || v < 0.0) {
    return false;
  }
  *gate = v;
  return true;
}

inline bool load_route_s_m(const char* path, double* s, int* n, int cap,
                           double* stop_gate = nullptr) {
  if (path == nullptr) {
    return false;
  }
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  const std::string text = ss.str();
  if (!parse_route_s_m(text, s, n, cap)) {
    return false;
  }
  if (stop_gate != nullptr) {
    parse_stop_gate_m(text, stop_gate);
  }
  return true;
}

inline void apply_vehicle_overlay(const VehicleOverlay& o, PlantParams* p, ScaParams* s) {
  if (p == nullptr || s == nullptr) {
    return;
  }
  if (o.has_mass) {
    p->m0_kg = o.mass_kg;
  }
  if (o.has_r0) {
    p->r0_m = o.r0_m;
    s->r0_m = o.r0_m;
  }
  if (o.has_a_trac) {
    p->a_trac_max = o.a_trac_max;
  }
  if (o.has_a_svc) {
    p->a_svc = o.a_svc;
  }
  if (o.has_A) {
    p->A_d = o.A_d;
  }
  if (o.has_B) {
    p->B_d = o.B_d;
  }
  if (o.has_C) {
    p->C_d = o.C_d;
  }
  if (o.has_tau) {
    p->tau_drv_s = o.tau_drv_s;
  }
  if (o.has_gamma) {
    p->gamma_rot = o.gamma_rot;
  }
  if (o.has_rail) {
    p->brake_nonadhesive_frac = o.brake_nonadhesive_frac;
  }
  if (o.has_notch_accel) {
    p->notch_as_accel = o.notch_as_accel;
  }
  if (o.has_grade) {
    p->i_grade = o.i_grade;
  }
  if (o.has_jerk) {
    p->j_max_mps3 = o.j_max_mps3;
  }
  if (o.has_mass_min) {
    p->mass_min_kg = o.mass_min_kg;
  }
  if (o.has_mass_max) {
    p->mass_max_kg = o.mass_max_kg;
  }
  if (o.has_v_base) {
    p->v_base_mps = o.v_base_mps;
  }
  if (o.has_pair_lr) {
    s->pair_lr = o.pair_lr;
  }
  if (o.has_axle_role) {
    s->axle_role = o.axle_role;
  }
  if (o.has_sigma_v_rel) {
    s->sigma_v_rel = o.sigma_v_rel;
  }
}

}  // namespace tram_dr
