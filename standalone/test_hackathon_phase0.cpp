// Phase-0 contracts: brake_source, common-radius prior, interval monitor,
// SCA extremal consensus, residual spline partition of unity.
#include "tram_dr_localization/param_keys.hpp"
#include "tram_dr_localization/residual.hpp"
#include "tram_dr_localization/sca.hpp"
#include "tram_dr_localization/ukf.hpp"

#include <cmath>
#include <iostream>
#include <string>

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

void spline_partition() {
  using namespace tram_dr;
  for (double v : {0.0, 1.0, 5.0, 12.0, 20.0}) {
    double b[kResidualSpline];
    residual_splines(v, b);
    double s = 0.0;
    for (int i = 0; i < kResidualSpline; ++i) {
      s += b[i];
      check(b[i] >= -1e-12, "B-spline is non-negative");
    }
    check(std::fabs(s - 1.0) < 1e-9, "B-splines partition unity on [0,20]");
  }
}

void param_table_defaults() {
  using namespace tram_dr;
  EstimatorBundle a;
  EstimatorBundle b;
  UkfParams u;
  PlantParams p;
  ScaParams s;
  check(a.ukf.alpha == u.alpha && a.ukf.q_v == u.q_v, "UKF defaults match constructor");
  check(a.plant.r0_m == p.r0_m && a.plant.m0_kg == p.m0_kg, "plant defaults match");
  check(a.sca.sigma_v_mps == s.sigma_v_mps, "SCA defaults match");
  check(known_param_key("q_v") && known_param_key("brake_source"), "core keys registered");
  check(apply_param_key(&b, "q_v", "0.01") && std::fabs(b.ukf.q_v - 0.01) < 1e-15,
        "--set q_v applies");
  check(!apply_param_key(&b, "not_a_key", "1"), "unknown keys rejected");
  check(!a.ukf.r_adapt && !a.ukf.stop_update, "M5/M11 default off");
  check(known_param_key("r_adapt") && known_param_key("stop_update"),
        "M5/M11 keys registered");
  check(apply_param_key(&b, "r_adapt", "true") && b.ukf.r_adapt, "--set r_adapt");
  check(apply_param_key(&b, "stop_update", "true") && b.ukf.stop_update,
        "--set stop_update");
}

void combined_notch_standstill() {
  using namespace tram_dr;
  UkfParams cfg;
  cfg.brake_source = BrakeSource::kNotch;
  Ukf ukf(cfg);
  Input u;
  u.notch = -0.5;
  u.brake = 0.0;
  u.brake_valid = true;
  double w[4] = {0.0, 0.0, 0.0, 0.0};
  UkfEstimate e{};
  for (int i = 0; i < 80; ++i) {
    e = ukf.predict_and_update(u, w, 4, 0.02);
  }
  check(std::fabs(e.x.v_mps) < 0.05, "n=-0.5 at rest stays at rest");
  check(e.mode == Mode::kStandstill, "n=-0.5 at rest reaches ZUPT");
}

void combined_notch_slide_latch() {
  using namespace tram_dr;
  UkfParams cfg;
  cfg.brake_source = BrakeSource::kNotch;
  cfg.kappa_hold_s = 0.0;
  cfg.kappa_cut = 0.15;
  Ukf ukf(cfg);
  Input u;
  u.notch = -0.8;
  u.brake_valid = true;
  double w[4] = {0.0, 0.0, 0.0, 0.0};
  UkfEstimate e{};
  for (int i = 0; i < 80; ++i) {
    const double omega = 8.0 / 0.35;
    w[0] = w[1] = w[2] = w[3] = omega;
    e = ukf.predict_and_update(u, w, 4, 0.02);
  }
  for (int i = 0; i < 80; ++i) {
    w[0] = w[1] = w[2] = w[3] = 0.0;
    e = ukf.predict_and_update(u, w, 4, 0.02);
  }
  check(e.mode != Mode::kStandstill, "locked wheels under brake are not ZUPT");
  check(std::fabs(e.x.v_mps) > 0.2, "false ZUPT does not zero body speed under brake");
  check(e.mode == Mode::kSlide || e.slip_latched || e.sca.common_mode ||
            e.path_disagree_latched || e.confidence != Confidence::kOk,
        "slide under negative combined notch latches integrity");
}

void brake_none_not_lost() {
  using namespace tram_dr;
  UkfParams cfg;
  cfg.brake_source = BrakeSource::kNone;
  Ukf ukf(cfg);
  Input u;
  u.notch = 0.3;
  u.brake_valid = false;
  double w[4] = {5.0, 5.0, 5.0, 5.0};
  UkfEstimate e{};
  for (int i = 0; i < 200; ++i) {
    e = ukf.predict_and_update(u, w, 4, 0.02);
  }
  check(e.confidence != Confidence::kLost, "structurally absent brake is not LOST");
}

void radius_prior_rank_one() {
  using namespace tram_dr;
  UkfParams common;
  common.radius_common_sigma = 0.05;
  common.radius_indiv_sigma = 0.005;
  Ukf ukf_c(common);
  UkfParams indep;
  indep.radius_common_sigma = 0.0;
  indep.radius_indiv_sigma = 0.05;
  Ukf ukf_i(indep);
  Input u;
  u.notch = 0.3;
  double w[4];
  UkfEstimate ec{};
  UkfEstimate ei{};
  for (int k = 0; k < 40; ++k) {
    w[0] = 10.0;
    w[1] = 10.4;
    w[2] = 9.6;
    w[3] = 10.1;
    ec = ukf_c.predict_and_update(u, w, 4, 0.02);
    ei = ukf_i.predict_and_update(u, w, 4, 0.02);
  }
  auto spread = [](const State& x) {
    double m = 0.0;
    for (int i = 0; i < 4; ++i) {
      m += x.d[static_cast<std::size_t>(i)];
    }
    m /= 4.0;
    double s = 0.0;
    for (int i = 0; i < 4; ++i) {
      const double e = x.d[static_cast<std::size_t>(i)] - m;
      s += e * e;
    }
    return s;
  };
  check(ec.initialized && ei.initialized, "radius prior still initialises");
  check(spread(ec.x) < spread(ei.x),
        "rank-one common-radius prior shrinks axle-scale spread vs independent");
}

void interval_monitor_contains_twin() {
  using namespace tram_dr;
  UkfParams cfg;
  cfg.interval_mode = 1;
  Ukf ukf(cfg);
  PlantParams plant;
  State gt;
  gt.v_mps = 1.0;
  Input u;
  u.notch = 0.4;
  bool ok = true;
  bool saw = false;
  for (int i = 0; i < 250; ++i) {
    plant_step(gt, u, 0.02, plant);
    const double omega = std::max(gt.v_mps, 0.0) / plant.r0_m;
    double w[4] = {omega, omega, omega, omega};
    const UkfEstimate e = ukf.predict_and_update(u, w, 4, 0.02);
    if (e.interval_active) {
      saw = true;
      if (gt.v_mps < e.v_lo_mps - 0.25 || gt.v_mps > e.v_hi_mps + 0.25) {
        ok = false;
      }
    }
  }
  check(saw && ok, "plant-twin speed stays inside monitor bounds");
}

void residual_python_grid() {
  using namespace tram_dr;
  ResidualModel m;
  m.enabled = true;
  m.sigma2 = 4.0;
  for (int i = 0; i < kResidualCoeff; ++i) {
    m.theta[static_cast<std::size_t>(i)] = static_cast<double>(i + 1);
  }
  struct Pt {
    double v, n, b, f;
  };
  // Lockstep with ResidualPythonCppGridTests. θ_i = i+1.
  const Pt gold[] = {
      {0.0, 0.0, 0.0, 1.0},
      {0.5, 0.4, 0.0, 5.503875},
      {2.0, 0.0, 0.0, 2.72},
      {3.5, -0.2, 0.1, 9.92153125},
      {5.0, 0.7, 0.0, 12.015865384615385},
      {7.5, 0.0, 0.3, 12.934114583333333},
      {10.0, -0.6, 0.0, 17.558974358974357},
      {13.0, 0.2, 0.5, 23.147810256410256},
      {16.0, 1.0, 0.0, 20.693333333333335},
      {18.5, -1.0, 1.0, 61.671249999999986},
      {20.0, 0.0, 0.0, 8.0},
  };
  for (const Pt& g : gold) {
    const ResidualForce f = residual_force(m, g.v, g.n, g.b);
    const double denom = std::max(1.0, std::fabs(g.f));
    check(f.applied && std::fabs(f.force_n - g.f) / denom < 1e-12,
          "C++ residual_force matches Python eval_force");
    check(std::fabs(f.var_n2 - 4.0) < 1e-15, "residual var_n2 is sigma2 without leverage");
  }
  const ResidualForce off = residual_force(m, 21.0, 0.0, 0.0);
  check(!off.applied && off.force_n == 0.0, "residual off for v > 20 m/s");
  ResidualModel z;
  z.enabled = false;
  z.theta[0] = 1000.0;
  const ResidualForce dis = residual_force(z, 0.0, 0.0, 0.0);
  check(!dis.applied, "disabled residual does not apply");
}

void residual_yaml_roundtrip() {
  using namespace tram_dr;
  std::string yaml = "residual:\n  enabled: true\n  sigma2: 4.0\n  theta: [";
  for (int i = 0; i < kResidualCoeff; ++i) {
    if (i) {
      yaml += ", ";
    }
    yaml += (i == 0) ? "1000" : "0";
  }
  yaml += "]\n";
  ResidualModel m;
  check(parse_residual_yaml_text(yaml, &m), "residual YAML parses");
  check(m.enabled, "residual YAML enables the model");
  check(std::fabs(m.theta[0] - 1000.0) < 1e-12, "residual YAML theta[0]");
  check(std::fabs(m.sigma2 - 4.0) < 1e-12, "residual YAML sigma2");
  m.enabled = true;
  const ResidualForce f = residual_force(m, 5.0, 0.0, 0.0);
  double b[kResidualSpline];
  residual_splines(5.0, b);
  check(f.applied && std::fabs(f.force_n - 1000.0 * b[0]) < 1e-6,
        "C++ residual force matches spline*theta");
  check(f.var_n2 > 0.0, "residual variance enters process intensity");
}

void r_adapt_grows_on_mismatch() {
  using namespace tram_dr;
  UkfParams cfg;
  cfg.r_adapt = true;
  Ukf ukf(cfg);
  Input u;
  u.notch = 0.5;
  u.brake = 0.0;
  u.brake_valid = true;
  double w[4] = {8.0, 8.0, 8.0, 8.0};
  UkfEstimate e{};
  for (int i = 0; i < 40; ++i) {
    e = ukf.predict_and_update(u, w, 4, 0.02);
  }
  const double r0 = e.r_adapt_r_mean;
  w[0] = 24.0;
  for (int i = 0; i < 80; ++i) {
    e = ukf.predict_and_update(u, w, 4, 0.02);
  }
  check(r0 > 0.0 && e.r_adapt_r_mean > r0, "variational R grows on a scaled axle");
  check(e.nis_valid, "NIS remains on nominal R");
}

void extremal_vs_median() {
  using namespace tram_dr;
  ScaParams p;
  p.pair_lr = false;
  p.traction = true;
  p.consensus = 0;
  double w[4] = {10.0, 10.2, 10.1, 20.0};
  double d[4] = {1, 1, 1, 1};
  const ScaResult med = sca_analyze(w, 4, d, p);
  p.consensus = 1;
  const ScaResult ext = sca_analyze(w, 4, d, p);
  check(ext.v_consensus_mps < med.v_consensus_mps,
        "traction extremal rejects the fastest axle");
}
}  // namespace

int main() {
  spline_partition();
  param_table_defaults();
  combined_notch_standstill();
  combined_notch_slide_latch();
  brake_none_not_lost();
  radius_prior_rank_one();
  interval_monitor_contains_twin();
  residual_yaml_roundtrip();
  residual_python_grid();
  r_adapt_grows_on_mismatch();
  extremal_vs_median();
  std::cout << checks << " checks, " << failures << " failures\n";
  return failures == 0 ? 0 : 1;
}
