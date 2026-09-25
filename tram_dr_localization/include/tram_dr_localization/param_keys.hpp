#pragma once
// Shared parameter table for the ROS node and standalone replay_ukf.
// Adding a tunable: one row here, one declare_parameter in the node, one
// apply_key case. Unknown keys stay unknown rather than silently changing
// the plant.

#include "tram_dr_localization/plant.hpp"
#include "tram_dr_localization/sca.hpp"
#include "tram_dr_localization/types.hpp"
#include "tram_dr_localization/ukf.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace tram_dr {

enum class ParamKind { kDouble, kInt, kBool, kString };

struct ParamKey {
  const char* name;
  ParamKind kind;
};

// ROS node keys plus overlay keys that replay_ukf must honour. Order is
// the node's declare_parameter order so a missing row is easy to see.
inline constexpr ParamKey kParamKeys[] = {
    {"rate_hz", ParamKind::kDouble},
    {"ukf_alpha", ParamKind::kDouble},
    {"ukf_beta", ParamKind::kDouble},
    {"ukf_kappa", ParamKind::kDouble},
    {"kappa_cut", ParamKind::kDouble},
    {"r_common_mode", ParamKind::kDouble},
    {"p_ss_init", ParamKind::kDouble},
    {"k_sigma", ParamKind::kDouble},
    {"k_over", ParamKind::kDouble},
    {"freeze_s", ParamKind::kDouble},
    {"ukf_cubature", ParamKind::kBool},
    {"kappa_hold_s", ParamKind::kDouble},
    {"k_lost", ParamKind::kDouble},
    {"notch_lost_s", ParamKind::kDouble},
    {"allow_reverse", ParamKind::kBool},
    {"mass_door_kg", ParamKind::kDouble},
    {"q_v", ParamKind::kDouble},
    {"zupt_hold_s", ParamKind::kDouble},
    {"encoder_pulses_per_rev", ParamKind::kInt},
    {"huber_c", ParamKind::kDouble},
    {"slide_grade_lost_s", ParamKind::kDouble},
    {"a_kin_downhill", ParamKind::kDouble},
    {"mass_prior_log_sigma", ParamKind::kDouble},
    {"mass_prior_tau_s", ParamKind::kDouble},
    {"q_fb_wheels_n", ParamKind::kDouble},
    {"stop_gate_m", ParamKind::kDouble},
    {"r0_uncalibrated", ParamKind::kBool},
    {"age_degraded_s", ParamKind::kDouble},
    {"age_lost_s", ParamKind::kDouble},
    {"zupt_omega_only_s", ParamKind::kDouble},
    {"n_wheels", ParamKind::kInt},
    {"wheel_radius_m", ParamKind::kDouble},
    {"mass_kg", ParamKind::kDouble},
    {"a_trac_max", ParamKind::kDouble},
    {"a_svc", ParamKind::kDouble},
    {"v_base_mps", ParamKind::kDouble},
    {"v2_mps", ParamKind::kDouble},
    {"A_d", ParamKind::kDouble},
    {"B_d", ParamKind::kDouble},
    {"C_d", ParamKind::kDouble},
    {"tau_drv_s", ParamKind::kDouble},
    {"gamma_rot", ParamKind::kDouble},
    {"brake_nonadhesive_frac", ParamKind::kDouble},
    {"notch_as_accel", ParamKind::kBool},
    {"i_grade", ParamKind::kDouble},
    {"j_max_mps3", ParamKind::kDouble},
    {"mass_min_kg", ParamKind::kDouble},
    {"mass_max_kg", ParamKind::kDouble},
    {"mu_min", ParamKind::kDouble},
    {"mu_max", ParamKind::kDouble},
    {"emergency_nonadhesive_frac", ParamKind::kDouble},
    {"vehicle_profile", ParamKind::kString},
    {"wheel_latency_s", ParamKind::kDouble},
    {"sca_z_thresh", ParamKind::kDouble},
    {"sca_pair_lr", ParamKind::kBool},
    {"sca_sigma_v_rel", ParamKind::kDouble},
    {"sca_sigma_v_mps", ParamKind::kDouble},
    {"notch_max_abs", ParamKind::kDouble},
    {"twist_is_omega", ParamKind::kBool},
    {"notch_stale_s", ParamKind::kDouble},
    {"brake_stale_s", ParamKind::kDouble},
    {"hold_last_command", ParamKind::kBool},
    {"default_dt_s", ParamKind::kDouble},
    {"dt_min_s", ParamKind::kDouble},
    {"dt_max_s", ParamKind::kDouble},
    {"stamp_regression_tol_s", ParamKind::kDouble},
    {"stamp_skew_warn_s", ParamKind::kDouble},
    {"max_catchup_steps", ParamKind::kInt},
    {"notch_type", ParamKind::kString},
    {"wheels_type", ParamKind::kString},
    {"notch_encoding", ParamKind::kString},
    {"brake_source", ParamKind::kString},
    {"radius_common_sigma", ParamKind::kDouble},
    {"radius_indiv_sigma", ParamKind::kDouble},
    {"interval_mode", ParamKind::kString},
    {"v_design_mps", ParamKind::kDouble},
    {"interval_sigma_omega", ParamKind::kDouble},
    {"interval_r_lo", ParamKind::kDouble},
    {"interval_r_hi", ParamKind::kDouble},
    {"sca_consensus", ParamKind::kString},
    {"r_adapt", ParamKind::kBool},
    {"r_adapt_rho", ParamKind::kDouble},
    {"r_adapt_r_min", ParamKind::kDouble},
    {"r_adapt_r_max", ParamKind::kDouble},
    {"stop_update", ParamKind::kBool},
    {"stop_sigma_m", ParamKind::kDouble},
    {"stop_map_sigma_m", ParamKind::kDouble},
    {"stop_assoc_g", ParamKind::kDouble},
};

inline constexpr int kParamKeyCount = static_cast<int>(sizeof(kParamKeys) / sizeof(kParamKeys[0]));

inline bool known_param_key(const char* name) {
  if (name == nullptr) {
    return false;
  }
  for (int i = 0; i < kParamKeyCount; ++i) {
    if (std::strcmp(kParamKeys[i].name, name) == 0) {
      return true;
    }
  }
  return false;
}

inline const ParamKey* find_param_key(const char* name) {
  if (name == nullptr) {
    return nullptr;
  }
  for (int i = 0; i < kParamKeyCount; ++i) {
    if (std::strcmp(kParamKeys[i].name, name) == 0) {
      return &kParamKeys[i];
    }
  }
  return nullptr;
}

struct EstimatorBundle {
  UkfParams ukf{};
  PlantParams plant{};
  ScaParams sca{};
  double notch_max_abs{8.0};
  double notch_stale_s{0.25};
  double brake_stale_s{0.25};
  bool hold_last_command{false};
  double default_dt_s{0.02};
  double dt_min_s{0.001};
  double dt_max_s{0.20};
  double stamp_regression_tol_s{0.001};
  double stamp_skew_warn_s{5.0};
  int max_catchup_steps{18000};
  double wheel_latency_s{0.0};
  bool twist_is_omega{false};
  double rate_hz{50.0};
  std::string vehicle_profile{"combino_nf100"};
  std::string notch_type{"float32"};
  std::string wheels_type{"float64_array"};
  std::string notch_encoding{"auto"};
};

inline int parse_interval_mode(const char* s) {
  if (s == nullptr) {
    return 0;
  }
  if (std::strcmp(s, "monitor") == 0) {
    return 1;
  }
  if (std::strcmp(s, "truncate") == 0) {
    return 2;
  }
  if (std::strcmp(s, "integrity") == 0) {
    return 3;
  }
  return 0;
}

inline const char* interval_mode_label(int m) {
  if (m == 1) {
    return "monitor";
  }
  if (m == 2) {
    return "truncate";
  }
  if (m == 3) {
    return "integrity";
  }
  return "off";
}

inline int parse_sca_consensus(const char* s) {
  if (s != nullptr && std::strcmp(s, "extremal") == 0) {
    return 1;
  }
  return 0;
}

inline bool apply_param_key(EstimatorBundle* b, const char* key, const char* val) {
  if (b == nullptr || key == nullptr || val == nullptr) {
    return false;
  }
  const ParamKey* meta = find_param_key(key);
  if (meta == nullptr) {
    return false;
  }
  auto as_d = [&]() { return std::strtod(val, nullptr); };
  auto as_i = [&]() { return std::atoi(val); };
  auto as_b = [&]() {
    return std::strcmp(val, "true") == 0 || std::strcmp(val, "True") == 0 ||
           std::strcmp(val, "1") == 0 || std::strcmp(val, "yes") == 0;
  };
  if (std::strcmp(key, "ukf_alpha") == 0) {
    b->ukf.alpha = as_d();
  } else if (std::strcmp(key, "ukf_beta") == 0) {
    b->ukf.beta = as_d();
  } else if (std::strcmp(key, "ukf_kappa") == 0) {
    b->ukf.kappa_ut = as_d();
  } else if (std::strcmp(key, "kappa_cut") == 0) {
    b->ukf.kappa_cut = as_d();
  } else if (std::strcmp(key, "r_common_mode") == 0) {
    b->ukf.r_common_mode = as_d();
  } else if (std::strcmp(key, "p_ss_init") == 0) {
    b->ukf.p_ss_init = as_d();
  } else if (std::strcmp(key, "k_sigma") == 0) {
    b->ukf.k_sigma = as_d();
  } else if (std::strcmp(key, "k_over") == 0) {
    b->ukf.k_over = as_d();
  } else if (std::strcmp(key, "freeze_s") == 0) {
    b->ukf.freeze_s = as_d();
  } else if (std::strcmp(key, "ukf_cubature") == 0) {
    b->ukf.cubature = as_b();
  } else if (std::strcmp(key, "kappa_hold_s") == 0) {
    b->ukf.kappa_hold_s = as_d();
  } else if (std::strcmp(key, "k_lost") == 0) {
    b->ukf.k_lost = as_d();
  } else if (std::strcmp(key, "notch_lost_s") == 0) {
    b->ukf.notch_lost_s = as_d();
  } else if (std::strcmp(key, "allow_reverse") == 0) {
    b->ukf.allow_reverse = as_b();
  } else if (std::strcmp(key, "mass_door_kg") == 0) {
    b->ukf.mass_door_kg = as_d();
  } else if (std::strcmp(key, "q_v") == 0) {
    b->ukf.q_v = as_d();
  } else if (std::strcmp(key, "zupt_hold_s") == 0) {
    b->ukf.zupt_hold_s = as_d();
  } else if (std::strcmp(key, "encoder_pulses_per_rev") == 0) {
    b->ukf.encoder_pulses_per_rev = as_i();
  } else if (std::strcmp(key, "huber_c") == 0) {
    b->ukf.huber_c = as_d();
  } else if (std::strcmp(key, "slide_grade_lost_s") == 0) {
    b->ukf.slide_grade_lost_s = as_d();
  } else if (std::strcmp(key, "a_kin_downhill") == 0) {
    b->ukf.a_kin_downhill = as_d();
  } else if (std::strcmp(key, "mass_prior_log_sigma") == 0) {
    b->ukf.mass_prior_log_sigma = as_d();
  } else if (std::strcmp(key, "mass_prior_tau_s") == 0) {
    b->ukf.mass_prior_tau_s = as_d();
  } else if (std::strcmp(key, "q_fb_wheels_n") == 0) {
    b->ukf.q_fb_wheels_n = as_d();
  } else if (std::strcmp(key, "stop_gate_m") == 0) {
    b->ukf.stop_gate_m = as_d();
  } else if (std::strcmp(key, "r0_uncalibrated") == 0) {
    b->ukf.r0_uncalibrated = as_b();
  } else if (std::strcmp(key, "age_degraded_s") == 0) {
    b->ukf.age_degraded_s = as_d();
  } else if (std::strcmp(key, "age_lost_s") == 0) {
    b->ukf.age_lost_s = as_d();
  } else if (std::strcmp(key, "zupt_omega_only_s") == 0) {
    b->ukf.zupt_omega_only_s = as_d();
  } else if (std::strcmp(key, "n_wheels") == 0) {
    b->ukf.n_wheels = std::clamp(as_i(), 1, kNWheels);
  } else if (std::strcmp(key, "wheel_radius_m") == 0) {
    b->plant.r0_m = as_d();
    b->sca.r0_m = b->plant.r0_m;
  } else if (std::strcmp(key, "mass_kg") == 0) {
    b->plant.m0_kg = as_d();
  } else if (std::strcmp(key, "a_trac_max") == 0) {
    b->plant.a_trac_max = as_d();
  } else if (std::strcmp(key, "a_svc") == 0) {
    b->plant.a_svc = as_d();
  } else if (std::strcmp(key, "v_base_mps") == 0) {
    b->plant.v_base_mps = as_d();
  } else if (std::strcmp(key, "v2_mps") == 0) {
    b->plant.v2_mps = as_d();
  } else if (std::strcmp(key, "A_d") == 0) {
    b->plant.A_d = as_d();
  } else if (std::strcmp(key, "B_d") == 0) {
    b->plant.B_d = as_d();
  } else if (std::strcmp(key, "C_d") == 0) {
    b->plant.C_d = as_d();
  } else if (std::strcmp(key, "tau_drv_s") == 0) {
    b->plant.tau_drv_s = as_d();
  } else if (std::strcmp(key, "gamma_rot") == 0) {
    b->plant.gamma_rot = as_d();
  } else if (std::strcmp(key, "brake_nonadhesive_frac") == 0) {
    b->plant.brake_nonadhesive_frac = as_d();
  } else if (std::strcmp(key, "notch_as_accel") == 0) {
    b->plant.notch_as_accel = as_b();
  } else if (std::strcmp(key, "i_grade") == 0) {
    b->plant.i_grade = as_d();
  } else if (std::strcmp(key, "j_max_mps3") == 0) {
    b->plant.j_max_mps3 = as_d();
  } else if (std::strcmp(key, "mass_min_kg") == 0) {
    b->plant.mass_min_kg = as_d();
  } else if (std::strcmp(key, "mass_max_kg") == 0) {
    b->plant.mass_max_kg = as_d();
  } else if (std::strcmp(key, "mu_min") == 0) {
    b->plant.mu_min = as_d();
  } else if (std::strcmp(key, "mu_max") == 0) {
    b->plant.mu_max = as_d();
  } else if (std::strcmp(key, "emergency_nonadhesive_frac") == 0) {
    b->plant.emergency_nonadhesive_frac = as_d();
  } else if (std::strcmp(key, "vehicle_profile") == 0) {
    b->vehicle_profile = val;
  } else if (std::strcmp(key, "wheel_latency_s") == 0) {
    b->wheel_latency_s = as_d();
  } else if (std::strcmp(key, "sca_z_thresh") == 0) {
    b->sca.z_thresh = as_d();
  } else if (std::strcmp(key, "sca_pair_lr") == 0) {
    b->sca.pair_lr = as_b();
  } else if (std::strcmp(key, "sca_sigma_v_rel") == 0) {
    b->sca.sigma_v_rel = as_d();
  } else if (std::strcmp(key, "sca_sigma_v_mps") == 0) {
    b->sca.sigma_v_mps = as_d();
  } else if (std::strcmp(key, "notch_max_abs") == 0) {
    b->notch_max_abs = as_d();
  } else if (std::strcmp(key, "twist_is_omega") == 0) {
    b->twist_is_omega = as_b();
  } else if (std::strcmp(key, "notch_stale_s") == 0) {
    b->notch_stale_s = as_d();
  } else if (std::strcmp(key, "brake_stale_s") == 0) {
    b->brake_stale_s = as_d();
  } else if (std::strcmp(key, "hold_last_command") == 0) {
    b->hold_last_command = as_b();
  } else if (std::strcmp(key, "default_dt_s") == 0) {
    b->default_dt_s = as_d();
  } else if (std::strcmp(key, "dt_min_s") == 0) {
    b->dt_min_s = as_d();
  } else if (std::strcmp(key, "dt_max_s") == 0) {
    b->dt_max_s = as_d();
  } else if (std::strcmp(key, "stamp_regression_tol_s") == 0) {
    b->stamp_regression_tol_s = as_d();
  } else if (std::strcmp(key, "stamp_skew_warn_s") == 0) {
    b->stamp_skew_warn_s = as_d();
  } else if (std::strcmp(key, "max_catchup_steps") == 0) {
    b->max_catchup_steps = as_i();
  } else if (std::strcmp(key, "rate_hz") == 0) {
    b->rate_hz = as_d();
  } else if (std::strcmp(key, "notch_type") == 0) {
    b->notch_type = val;
  } else if (std::strcmp(key, "wheels_type") == 0) {
    b->wheels_type = val;
  } else if (std::strcmp(key, "notch_encoding") == 0) {
    b->notch_encoding = val;
  } else if (std::strcmp(key, "brake_source") == 0) {
    b->ukf.brake_source = parse_brake_source(val);
  } else if (std::strcmp(key, "radius_common_sigma") == 0) {
    b->ukf.radius_common_sigma = as_d();
  } else if (std::strcmp(key, "radius_indiv_sigma") == 0) {
    b->ukf.radius_indiv_sigma = as_d();
  } else if (std::strcmp(key, "interval_mode") == 0) {
    b->ukf.interval_mode = parse_interval_mode(val);
  } else if (std::strcmp(key, "v_design_mps") == 0) {
    b->ukf.v_design_mps = as_d();
  } else if (std::strcmp(key, "interval_sigma_omega") == 0) {
    b->ukf.interval_sigma_omega = as_d();
  } else if (std::strcmp(key, "interval_r_lo") == 0) {
    b->ukf.interval_r_lo = as_d();
  } else if (std::strcmp(key, "interval_r_hi") == 0) {
    b->ukf.interval_r_hi = as_d();
  } else if (std::strcmp(key, "sca_consensus") == 0) {
    b->ukf.sca_consensus = parse_sca_consensus(val);
    b->sca.consensus = b->ukf.sca_consensus;
  } else if (std::strcmp(key, "r_adapt") == 0) {
    b->ukf.r_adapt = as_b();
  } else if (std::strcmp(key, "r_adapt_rho") == 0) {
    b->ukf.r_adapt_rho = as_d();
  } else if (std::strcmp(key, "r_adapt_r_min") == 0) {
    b->ukf.r_adapt_r_min = as_d();
  } else if (std::strcmp(key, "r_adapt_r_max") == 0) {
    b->ukf.r_adapt_r_max = as_d();
  } else if (std::strcmp(key, "stop_update") == 0) {
    b->ukf.stop_update = as_b();
  } else if (std::strcmp(key, "stop_sigma_m") == 0) {
    b->ukf.stop_sigma_m = as_d();
  } else if (std::strcmp(key, "stop_map_sigma_m") == 0) {
    b->ukf.stop_map_sigma_m = as_d();
  } else if (std::strcmp(key, "stop_assoc_g") == 0) {
    b->ukf.stop_assoc_g = as_d();
  } else {
    return false;
  }
  return true;
}

}  // namespace tram_dr
