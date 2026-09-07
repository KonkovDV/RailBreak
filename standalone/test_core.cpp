// ROS-free core tests. Build: cmake -S . -B build && cmake --build build
#include "tram_dr_localization/lin_alg.hpp"
#include "tram_dr_localization/map.hpp"
#include "tram_dr_localization/plant.hpp"
#include "tram_dr_localization/sca.hpp"
#include "tram_dr_localization/timebase.hpp"
#include "tram_dr_localization/ukf.hpp"
#include "tram_dr_localization/vehicle_yaml.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace {

int g_fails = 0;

void expect(bool ok, const char* msg) {
  if (!ok) {
    std::fprintf(stderr, "FAIL %s\n", msg);
    ++g_fails;
  }
}

}  // namespace

int main() {
  {
    tram_dr::State x;
    expect(x.m_eff_kg == 28000.0, "combino tare");
    expect(tram_dr::kStateDim == 12, "L=12 max n=6");
    expect(tram_dr::kNWheels == 6, "compile-time max axles");
    expect(tram_dr::kMu == 11, "mu after six d_i");
    expect(tram_dr::kDMin == 0.85, "d clip lo");
    expect(tram_dr::kDMax == 1.05, "d clip hi");
    expect(tram_dr::kKtracMin == 0.5, "k_trac clip lo");
    expect(tram_dr::kKtracMax == 1.5, "k_trac clip hi");
    expect(std::fabs(tram_dr::map_notch(8.0) - 1.0) < 1e-12, "notch 8 -> 1");
    expect(std::fabs(tram_dr::map_notch(4.0) - 0.5) < 1e-12, "notch 4 -> 0.5");
    expect(std::fabs(tram_dr::map_notch(0.4) - 0.4) < 1e-12, "notch already unit");
    expect(std::fabs(tram_dr::map_notch(1.0, 8.0, tram_dr::NotchEncoding::kDiscrete) - 0.125) < 1e-12,
           "discrete 1 is first step not full traction");
    expect(std::fabs(tram_dr::map_notch(1.001, 8.0, tram_dr::NotchEncoding::kNormalized) - 1.0) < 1e-12,
           "normalized 1.001 clamps, does not divide");
    expect(std::isnan(tram_dr::map_notch(std::numeric_limits<double>::quiet_NaN())),
           "map_notch propagates NaN");
    expect(std::isinf(tram_dr::map_notch(std::numeric_limits<double>::infinity())),
           "map_notch propagates Inf");
  }
  {
    const auto p = tram_dr::default_plant_params();
    expect(std::fabs(tram_dr::davis_resistance_n(0.0, p)) < 1e-9, "davis rest");
    expect(std::fabs(tram_dr::davis_resistance_n(-5.0, p) +
                     tram_dr::davis_resistance_n(5.0, p)) < 1e-9,
           "davis odd");
    tram_dr::State rest;
    tram_dr::Input coast;
    const auto d0 = tram_dr::plant_forces(rest, coast, p);
    expect(std::fabs(d0.a_mps2) < 1e-12, "davis rest does not roll");
    tram_dr::State rev;
    rev.v_mps = -0.5;
    tram_dr::plant_step(rev, coast, 0.02, p);
    expect(rev.v_mps > -0.5, "davis opposes reverse");
    tram_dr::State x;
    x.v_mps = 1.0;
    tram_dr::Input u;
    u.notch = 1.0;
    tram_dr::plant_step(x, u, 0.02);
    expect(x.v_mps > 1.0, "notch accelerates");
    tram_dr::State y;
    tram_dr::Input full;
    full.notch = 1.0;
    tram_dr::plant_step(y, full, 0.02);
    const double a = 1.3;
    expect(std::fabs(y.v_mps - a * 0.02) < 1e-9, "first-step v twin");
    tram_dr::State live;
    tram_dr::State stale;
    live.v_mps = stale.v_mps = 1.0;
    tram_dr::Input cmd;
    cmd.notch = 1.0;
    tram_dr::plant_step(live, cmd, 0.02);
    cmd.notch_valid = false;
    tram_dr::plant_step(stale, cmd, 0.02);
    expect(live.v_mps > stale.v_mps + 1e-4, "invalid notch does not apply traction");
    tram_dr::State br_live;
    tram_dr::State br_stale;
    br_live.v_mps = br_stale.v_mps = 5.0;
    tram_dr::Input br;
    br.brake = 1.0;
    tram_dr::plant_step(br_live, br, 0.02);
    br.brake_valid = false;
    tram_dr::plant_step(br_stale, br, 0.02);
    expect(br_live.v_mps < br_stale.v_mps - 1e-4, "invalid brake does not apply F_brake");
  }
  {
    tram_dr::PlantParams p = tram_dr::default_plant_params();
    p.tau_drv_s = 0.12;
    tram_dr::State x;
    tram_dr::Input u;
    u.notch = 1.0;
    double f_lag = 0.0;
    tram_dr::plant_step(x, u, 0.02, p, &f_lag);
    const double a_alg = 1.3;
    expect(x.v_mps < a_alg * 0.02 - 1e-6, "tau slows first step");
    expect(f_lag > 0.0 && f_lag < 28000.0 * 1.3, "lag between 0 and F*");
  }
  {
    tram_dr::PlantParams p = tram_dr::default_plant_params();
    p.j_max_mps3 = 0.7;
    tram_dr::State x;
    tram_dr::Input u;
    u.notch = 1.0;
    double f_lag = 0.0;
    tram_dr::plant_step(x, u, 0.02, p, &f_lag);
    const double a_alg = 1.3;
    expect(x.v_mps < a_alg * 0.02 - 1e-6, "jerk slows first step");
    expect(std::fabs(f_lag - 28000.0 * 0.7 * 0.02) < 1.0, "jerk df = m j dt");
  }
  {
    tram_dr::PlantParams p = tram_dr::default_plant_params();
    p.gamma_rot = 0.10;
    tram_dr::State x;
    tram_dr::Input u;
    u.notch = 1.0;
    tram_dr::plant_step(x, u, 0.02, p);
    const double a_alg = 1.3;
    expect(x.v_mps < a_alg * 0.02 - 1e-6, "gamma lowers a");
  }
  {
    tram_dr::State x;
    x.v_mps = 5.0;
    x.mu_hat = 0.06;
    tram_dr::Input u;
    u.brake = 1.0;
    const auto p = tram_dr::default_plant_params();
    const tram_dr::PlantDeriv d = tram_dr::plant_forces(x, u, p);
    expect(std::fabs(d.f_trac_n) <= d.f_adh_cap_n + 1e-6, "adhesion caps net contact");
  }
  {
    const double w[] = {10.0, 10.0, 10.0, 40.0};
    const double d[] = {1.0, 1.0, 1.0, 1.0};
    const auto s = tram_dr::sca_analyze(w, 4, d, tram_dr::ScaParams{});
    expect(s.n_inflated >= 1, "sca inflates outlier");
  }
  {
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch = 0.4;
    u.notch_valid = true;
    const double r = 0.35;
    double omega[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 8; ++k) {
      e = ukf.predict_and_update(u, omega, 4, 0.02);
    }
    double fast[4] = {12.0 / r, 12.0 / r, 12.0 / r, 12.0 / r};
    for (int k = 0; k < 15; ++k) {
      e = ukf.predict_and_update(u, fast, 4, 0.02);
    }
    expect(e.sca.n_inflated == 0, "common-mode wheels agree");
    expect(e.sca.common_mode, "common-mode vs body");
  }
  {
    // Slip latch: after a common-mode event the run stays DEGRADED
    // even when wheels and body agree again.
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch = 0.4;
    u.notch_valid = true;
    const double r = 0.35;
    double roll[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 50; ++k) {
      e = ukf.predict_and_update(u, roll, 4, 0.02);
    }
    expect(e.confidence == tram_dr::Confidence::kOk, "healthy run is OK");
    double fast[4] = {12.0 / r, 12.0 / r, 12.0 / r, 12.0 / r};
    for (int k = 0; k < 15; ++k) {
      e = ukf.predict_and_update(u, fast, 4, 0.02);
    }
    expect(e.sca.common_mode, "common-mode detected");
    expect(e.confidence == tram_dr::Confidence::kDegraded, "common-mode degrades");
    for (int k = 0; k < 400; ++k) {
      e = ukf.predict_and_update(u, fast, 4, 0.02);
    }
    expect(!e.sca.common_mode, "common-mode clears once consistent");
    expect(e.confidence == tram_dr::Confidence::kDegraded, "slip latch holds");
    expect(e.confidence_s == tram_dr::Confidence::kDegraded, "s stays degraded");
    expect(e.confidence_v == tram_dr::Confidence::kOk, "v ok once wheels agree");
  }
  {
    double A[4] = {1, 0, 0, 1};
    double L[4];
    expect(tram_dr::la::chol(A, L, 2, 0.0), "chol I");
  }
  {
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    const auto empty = ukf.predict_and_update(u, nullptr, 0);
    expect(!empty.initialized, "uninitialized until wheels");
    const double vel = 5.0;
    const double w = vel / 0.35;
    double omega[4] = {w, w, w, w};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 50; ++k) {
      e = ukf.predict_and_update(u, omega, 4, 0.02);
    }
    expect(e.initialized, "ukf init");
    expect(std::fabs(e.x.v_mps - vel) < 1.5, "ukf tracks v");
    expect(e.p_ss < 1.0e6, "p_ss finite");
    u.notch = 1.0;
    u.notch_valid = true;
    double slow[4] = {1.0 / 0.35, 1.0 / 0.35, 1.0 / 0.35, 1.0 / 0.35};
    e = ukf.predict_and_update(u, slow, 4, 0.02);
    expect(e.x.a_mps2 > 0.5, "diagnostic a uses notch");
  }
  {
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch = 0.0;
    u.notch_valid = true;
    u.brake = 0.3;
    double z[4] = {0.0, 0.0, 0.0, 0.0};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 80; ++k) {
      e = ukf.predict_and_update(u, z, 4, 0.02);
    }
    expect(e.mode == tram_dr::Mode::kStandstill, "zupt with hold brake");
    expect(std::fabs(e.x.v_mps) < 1e-12, "zupt zeros v");
    expect(std::fabs(e.x.f_bias_n) < 250.0, "zupt does not dump Davis into F_bias");
    for (int k = 0; k < 20; ++k) {
      e = ukf.predict_and_update(u, z, 4, 0.02);
    }
    expect(std::fabs(e.x.v_mps) < 1e-9, "zupt holds v after extra ticks");
  }
  {
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch_valid = true;
    const double r = 0.35;
    double roll[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 8; ++k) {
      e = ukf.predict_and_update(u, roll, 4, 0.02);
    }
    u.notch = 0.0;
    u.brake = 0.85;
    double locked[4] = {0.0, 0.0, 0.0, 0.0};
    e = ukf.predict_and_update(u, locked, 4, 0.02);
    expect(e.x.v_mps > 1.0, "no zupt on sliding lock");
  }
  {
    // One frozen encoder: SCA inflates that axle, confidence stays OK.
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch_valid = true;
    const double r = 0.35;
    const double v = 5.0;
    double roll[4] = {v / r, v / r, v / r, v / r};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 40; ++k) {
      e = ukf.predict_and_update(u, roll, 4, 0.02);
    }
    double one_bad[4] = {v / r, v / r, v / r, 0.4 * v / r};
    for (int k = 0; k < 30; ++k) {
      e = ukf.predict_and_update(u, one_bad, 4, 0.02);
    }
    expect(e.sca.n_inflated == 1, "single axle inflated");
    expect(!e.slip_latched, "single axle is not a slip latch");
    expect(e.confidence == tram_dr::Confidence::kOk, "single axle stays OK");
  }
  {
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch_valid = true;
    const double r = 0.35;
    double omega[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 50; ++k) {
      e = ukf.predict_and_update(u, omega, 4, 0.02);
    }
    expect(e.nis_valid, "nis after wheel update");
    expect(e.nis >= 0.0, "nis non-negative");
    expect(e.nis_cusum >= 0.0, "cusum non-negative");
    expect(e.over_m > 0.0, "over_m from P_ss");
    expect(e.over_m > e.under_m - 1e-12, "over_m >= under_m");
    expect(e.over_m > e.under_m + 1e-9, "asymmetric k_over > k_sigma");
    expect(!e.a_unphysical, "healthy a not unphysical");
    e = ukf.predict_and_update(u, nullptr, 0, 0.02);
    expect(!e.nis_valid, "nis invalid on predict-only");
  }
  {
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch_valid = true;
    const double r = 0.35;
    double omega[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 20; ++k) {
      e = ukf.predict_and_update(u, omega, 4, 0.02);
    }
    expect(e.confidence == tram_dr::Confidence::kOk, "pre-outage OK");
    double dead[4] = {std::nan(""), std::nan(""), std::nan(""), std::nan("")};
    e = ukf.predict_and_update(u, dead, 4, 0.02);
    expect(e.confidence == tram_dr::Confidence::kLost, "all-NaN is LOST");
    expect(e.n_omega_used == 0, "n_omega_used 0");
    expect(e.sca.n_inflated >= 4, "all axles inflated");
    for (int k = 0; k < 100; ++k) {
      e = ukf.predict_and_update(u, dead, 4, 0.02);
    }
    expect(e.confidence == tram_dr::Confidence::kLost, "all-NaN stays LOST for 2s");
    expect(e.confidence != tram_dr::Confidence::kOk, "all-NaN never returns OK");
  }
  {
    tram_dr::UkfParams cfg;
    cfg.r0_uncalibrated = true;
    tram_dr::Ukf ukf(cfg);
    tram_dr::Input u;
    u.notch_valid = true;
    const double r = 0.35;
    double omega[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 40; ++k) {
      e = ukf.predict_and_update(u, omega, 4, 0.02);
    }
    expect(e.confidence == tram_dr::Confidence::kDegraded, "uncalibrated r0 DEGRADED");
    expect(e.confidence_s == tram_dr::Confidence::kDegraded, "uncalibrated r0 DEGRADED s");
  }
  {
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch_valid = true;
    const double r = 0.35;
    const double v = 5.0;
    double omega[4] = {v / r, v / r, v / r, v / r};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 40; ++k) {
      e = ukf.predict_and_update(u, omega, 4, 0.02);
    }
    expect(e.n_frozen == 0, "constant speed is not freeze");
    for (int k = 1; k <= 30; ++k) {
      const double v_live = v + 0.08 * static_cast<double>(k);
      omega[0] = v_live / r;
      omega[1] = v_live / r;
      omega[2] = v_live / r;
      omega[3] = v / r;
      e = ukf.predict_and_update(u, omega, 4, 0.02);
    }
    expect(e.n_frozen == 1, "one stuck axle while others move");
    expect(e.confidence == tram_dr::Confidence::kOk, "one freeze stays OK");
  }
  {
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch_valid = true;
    const double r = 0.35;
    const double v = 5.0;
    double omega[4] = {v / r, v / r, v / r, v / r};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 40; ++k) {
      e = ukf.predict_and_update(u, omega, 4, 0.005);
    }
    for (int k = 1; k <= 30; ++k) {
      const double v_live = v + 0.08 * static_cast<double>(k);
      omega[0] = v_live / r;
      omega[1] = v_live / r;
      omega[2] = v_live / r;
      omega[3] = v / r;
      e = ukf.predict_and_update(u, omega, 4, 0.005);
    }
    expect(e.n_frozen == 0, "5 ms x 30 is 0.15 s, not freeze_s");
    for (int k = 1; k <= 90; ++k) {
      const double v_live = v + 0.08 * static_cast<double>(30 + k);
      omega[0] = v_live / r;
      omega[1] = v_live / r;
      omega[2] = v_live / r;
      omega[3] = v / r;
      e = ukf.predict_and_update(u, omega, 4, 0.005);
    }
    expect(e.n_frozen == 1, "freeze_s is elapsed time at 5 ms");
  }
  {
    // Relative kappa floor is 1 m/s; a 0.14 m/s stop tail must not trip common-mode.
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch = 0.0;
    u.notch_valid = true;
    u.brake = 0.0;
    const double r = 0.35;
    const double v = 0.14;
    tram_dr::UkfEstimate e{};
    double match[4] = {v / r, v / r, v / r, v / r};
    for (int k = 0; k < 4; ++k) {
      e = ukf.predict_and_update(u, match, 4, 0.02);
    }
    double off[4] = {(v + 0.20) / r, (v + 0.20) / r, (v + 0.20) / r, (v + 0.20) / r};
    e = ukf.predict_and_update(u, off, 4, 0.02);
    expect(!e.sca.common_mode, "kappa floor ignores stop transient");
    expect(!e.slip_latched, "stop transient is not a slip latch");
  }
  {
    // Plant mismatch, no slip: wheels follow a slower truth plant.
    tram_dr::PlantParams truth = tram_dr::default_plant_params();
    truth.tau_drv_s = 0.3;
    truth.gamma_rot = 0.08;
    truth.A_d *= 1.3;
    truth.B_d *= 1.3;
    truth.C_d *= 1.3;
    truth.a_trac_max *= 0.7;
    tram_dr::State body;
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch = 0.7;
    u.notch_valid = true;
    const double r = 0.35;
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 400; ++k) {
      tram_dr::plant_step(body, u, 0.02, truth);
      const double w = body.v_mps / r;
      double omega[4] = {w, w, w, w};
      e = ukf.predict_and_update(u, omega, 4, 0.02);
    }
    expect(e.confidence != tram_dr::Confidence::kLost, "mismatch is not LOST");
    expect(e.confidence_v != tram_dr::Confidence::kLost, "v channel not lost");
    expect(e.chol_fail == 0, "chol held on mismatch");
  }
  {
    tram_dr::ScaParams on;
    on.pair_lr = true;
    const double r = 0.35;
    const double v = 15.0;
    const double delta = 0.5 * 1.524 / 25.0;
    const double w0 = (v / r) * (1.0 - delta);
    const double w1 = (v / r) * (1.0 + delta);
    const double w[] = {w0, w1, w0, w1};
    const double d[] = {1.0, 1.0, 1.0, 1.0};
    const auto paired = tram_dr::sca_analyze(w, 4, d, on);
    expect(paired.n_inflated == 0, "pair_lr absorbs IRW curve");
    tram_dr::ScaParams off;
    off.pair_lr = false;
    const auto raw = tram_dr::sca_analyze(w, 4, d, off);
    expect(raw.n_inflated >= 1, "unpaired L/R inflates on a tight curve");
  }
  {
    const double w[] = {10.0, std::nan(""), 10.0, 10.0};
    const double d[] = {1.0, 1.0, 1.0, 1.0};
    const auto s = tram_dr::sca_analyze(w, 4, d, tram_dr::ScaParams{});
    expect(s.n_inflated == 1, "NaN axle inflates once");
  }
  {
    tram_dr::UkfParams cfg;
    cfg.n_wheels = 2;
    tram_dr::Ukf ukf(cfg);
    tram_dr::Input u;
    u.notch_valid = true;
    const double r = 0.35;
    double agree[2] = {5.0 / r, 5.0 / r};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 40; ++k) {
      e = ukf.predict_and_update(u, agree, 2, 0.02);
    }
    expect(e.confidence == tram_dr::Confidence::kOk, "two agreeing axles OK");
    double split[2] = {5.0 / r, 12.0 / r};
    for (int k = 0; k < 20; ++k) {
      e = ukf.predict_and_update(u, split, 2, 0.02);
    }
    expect(e.confidence != tram_dr::Confidence::kOk, "n=2 split is not OK");
  }
  {
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch_valid = true;
    const double r = 0.35;
    double omega[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 20; ++k) {
      e = ukf.predict_and_update(u, omega, 4, 0.02);
    }
    expect(e.n_omega_used == 4, "four finite wheels counted");
    omega[1] = std::nan("");
    e = ukf.predict_and_update(u, omega, 4, 0.02);
    expect(std::isfinite(e.x.v_mps), "NaN channel does not NaN the state");
    expect(e.n_omega_used == 3, "NaN channel dropped from count");
  }
  {
    tram_dr::PlantParams mag = tram_dr::default_plant_params();
    mag.brake_nonadhesive_frac = 1.0;
    tram_dr::State x;
    x.v_mps = 8.0;
    x.mu_hat = 0.06;
    tram_dr::Input u;
    u.brake = 1.0;
    const auto capped = tram_dr::plant_forces(x, u, tram_dr::default_plant_params());
    const auto rail = tram_dr::plant_forces(x, u, mag);
    expect(std::fabs(rail.a_mps2) > std::fabs(capped.a_mps2) + 0.2,
           "track magnet bypasses mu cap");
  }
  {
    tram_dr::PlantParams p = tram_dr::default_plant_params();
    p.notch_as_accel = true;
    tram_dr::State heavy;
    heavy.m_eff_kg = 40000.0;
    tram_dr::Input u;
    u.notch = 1.0;
    tram_dr::plant_step(heavy, u, 0.02, p);
    const double a_closed = 1.3;
    expect(std::fabs(heavy.v_mps - a_closed * 0.02) < 1e-9, "accel-notch uses live m");
  }
  {
    tram_dr::PlantParams p = tram_dr::default_plant_params();
    p.i_grade = 0.02;
    tram_dr::State x;
    tram_dr::Input u;
    tram_dr::plant_step(x, u, 0.02, p);
    const double a = -(28000.0 * 9.81 * 0.02) / 28000.0;
    expect(std::fabs(x.v_mps - a * 0.02) < 1e-9, "grade force on coast");
  }
  {
    double A[4] = {1.0, 2.0, 2.0, 1.0};
    expect(tram_dr::la::project_pd(A, 2), "jacob PD reports success");
    double L[4];
    expect(tram_dr::la::chol(A, L, 2, 0.0), "jacob PD then chol");
  }
  {
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch = 0.0;
    u.notch_valid = true;
    const double r = 0.35;
    double roll[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 40; ++k) {
      e = ukf.predict_and_update(u, roll, 4, 0.02);
    }
    double fast[4] = {12.0 / r, 12.0 / r, 12.0 / r, 12.0 / r};
    for (int k = 0; k < 20; ++k) {
      e = ukf.predict_and_update(u, fast, 4, 0.02);
    }
    expect(!e.sca.common_mode, "coast kappa is model error, not latch");
    expect(!e.slip_latched, "coast does not latch s");
  }
  {
    const double alphas[] = {0.58, 0.8, 1.0};
    for (double a : alphas) {
      tram_dr::UkfParams cfg;
      cfg.alpha = a;
      tram_dr::Ukf ukf(cfg);
      tram_dr::Input u;
      u.notch = 0.4;
      u.notch_valid = true;
      const double r = 0.35;
      double omega[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
      tram_dr::UkfEstimate e{};
      for (int k = 0; k < 40; ++k) {
        e = ukf.predict_and_update(u, omega, 4, 0.02);
      }
      expect(e.chol_fail == 0, "A/B alpha chol");
      expect(std::isfinite(e.x.v_mps), "A/B alpha finite v");
      expect(e.x.m_eff_kg > 10000.0 && e.x.m_eff_kg < 60000.0, "A/B mass in range");
    }
    tram_dr::UkfParams ckf;
    ckf.cubature = true;
    tram_dr::Ukf ukf(ckf);
    tram_dr::Input u;
    u.notch = 0.4;
    u.notch_valid = true;
    const double r = 0.35;
    double omega[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 40; ++k) {
      e = ukf.predict_and_update(u, omega, 4, 0.02);
    }
    expect(e.chol_fail == 0, "CKF chol");
    expect(std::isfinite(e.x.v_mps), "CKF finite v");
  }
  {
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch_valid = true;
    const double r = 0.35;
    double omega[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 30; ++k) {
      e = ukf.predict_and_update(u, omega, 4, 0.02);
    }
    u.notch_valid = false;
    for (int k = 0; k < 120; ++k) {
      e = ukf.predict_and_update(u, omega, 4, 0.02);
    }
    expect(e.confidence == tram_dr::Confidence::kLost, "notch age LOST");
  }
  {
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch = 0.8;
    u.notch_valid = true;
    const double r = 0.35;
    double roll[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 40; ++k) {
      e = ukf.predict_and_update(u, roll, 4, 0.02);
    }
    double slow[4] = {2.0 / r, 2.0 / r, 2.0 / r, 2.0 / r};
    for (int k = 0; k < 20; ++k) {
      e = ukf.predict_and_update(u, slow, 4, 0.02);
    }
    expect(!e.sca.common_mode, "ramp sign is not slip");
    expect(!e.slip_latched, "ramp does not latch s");
    expect(e.pl_s_m > e.under_m, "PL includes k sqrt(P)");
    expect(e.b_s_m > 0.0 && e.b_s_m < 2.0, "MDS bound finite before latch");
  }
  {
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch = 0.0;
    u.notch_valid = true;
    u.brake = 0.3;
    double z[4] = {0.0, 0.0, 0.0, 0.0};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 10; ++k) {
      e = ukf.predict_and_update(u, z, 4, 0.02);
    }
    expect(e.mode != tram_dr::Mode::kStandstill, "zupt needs hold window");
  }
  {
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch = 0.3;
    u.notch_valid = true;
    std::uint32_t seed = 1;
    auto rnd = [&seed]() {
      seed = seed * 1664525u + 1013904223u;
      return seed;
    };
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 200; ++k) {
      double omega[4];
      for (int i = 0; i < 4; ++i) {
        const std::uint32_t r = rnd() % 8u;
        if (r == 0) {
          omega[i] = std::numeric_limits<double>::quiet_NaN();
        } else if (r == 1) {
          omega[i] = std::numeric_limits<double>::infinity();
        } else if (r == 2) {
          omega[i] = 1.0e6;
        } else {
          omega[i] = 5.0 / 0.35;
        }
      }
      e = ukf.predict_and_update(u, omega, 4, 0.02);
      expect(std::isfinite(e.x.v_mps), "fuzz v finite");
      expect(std::isfinite(e.x.s_m), "fuzz s finite");
    }
  }
  {
    tram_dr::State x;
    x.m_eff_kg = 80000.0;
    tram_dr::Input u;
    tram_dr::plant_step(x, u, 0.02);
    expect(x.m_eff_kg <= 70000.0 && x.m_eff_kg >= 20000.0, "mass clip 20-70 t");
  }
  {
    tram_dr::ScaParams p;
    p.traction = true;
    p.axle_role[0] = tram_dr::kAxleMotor;
    p.axle_role[1] = tram_dr::kAxleTrailer;
    p.axle_role[2] = tram_dr::kAxleTrailer;
    p.axle_role[3] = tram_dr::kAxleTrailer;
    const double w[] = {40.0, 10.0, 10.0, 10.0};
    const double d[] = {1.0, 1.0, 1.0, 1.0};
    const auto s = tram_dr::sca_analyze(w, 4, d, p);
    expect(s.used_trailer_consensus, "trailer consensus under traction");
    expect(s.n_inflated_motor >= 1, "spinning motor inflated");
    expect(s.n_inflated_trailer == 0, "trailers agree");
  }
  {
    tram_dr::Ukf ukf;
    tram_dr::ScaParams sca;
    sca.axle_role[0] = tram_dr::kAxleMotor;
    sca.axle_role[1] = tram_dr::kAxleTrailer;
    sca.axle_role[2] = tram_dr::kAxleTrailer;
    sca.axle_role[3] = tram_dr::kAxleTrailer;
    ukf.set_sca(sca);
    tram_dr::Input u;
    u.notch = 0.8;
    u.notch_valid = true;
    const double r = 0.35;
    const double v = 5.0;
    tram_dr::UkfEstimate e{};
    double roll[4] = {v / r, v / r, v / r, v / r};
    for (int k = 0; k < 40; ++k) {
      e = ukf.predict_and_update(u, roll, 4, 0.02);
    }
    double spin[4] = {12.0 / r, v / r, v / r, v / r};
    for (int k = 0; k < 25; ++k) {
      e = ukf.predict_and_update(u, spin, 4, 0.02);
    }
    expect(!e.slip_latched, "one motor spin is not a slip latch");
    expect(e.confidence == tram_dr::Confidence::kOk, "trailers keep v OK");
  }
  {
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch_valid = true;
    const double r = 0.35;
    double roll[4] = {10.0 / r, 10.0 / r, 10.0 / r, 10.0 / r};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 20; ++k) {
      e = ukf.predict_and_update(u, roll, 4, 0.02);
    }
    u.notch = 0.0;
    u.brake = 0.85;
    double locked[4] = {0.0, 0.0, 0.0, 0.0};
    for (int k = 0; k < 25; ++k) {
      e = ukf.predict_and_update(u, locked, 4, 0.02);
    }
    expect(std::isfinite(e.x.v_mps), "huber slide v finite");
    expect(e.x.v_mps > 2.0, "huber slide does not collapse v to r omega");
    expect(e.b_s_m > 0.1, "b_s uses |v| Td");
    // All-zero packet at speed is observe_rest_packet, not update_wheels, so
    // n_huber_capped stays 0. DEGRADED must still fire (channel-A kappa).
    expect(e.confidence != tram_dr::Confidence::kOk,
           "locked slide is not silent OK after kappa_hold");
  }
  {
    const char* y =
        "/**:\n"
        "  ros__parameters:\n"
        "    vehicle_profile: lvenok_moscow\n"
        "    n_wheels: 4\n"
        "    sca_pair_lr: false\n"
        "    axle_role: [0, 0, 0, 0]\n"
        "    mass_kg: 37000.0\n";
    tram_dr::VehicleOverlay ov;
    expect(tram_dr::parse_vehicle_yaml_text(y, &ov), "parse vehicle yaml");
    expect(ov.n_wheels == 4, "lvenok n=4");
    expect(ov.has_pair_lr && !ov.pair_lr, "lvenok pair_lr false");
    expect(ov.has_mass && ov.mass_kg > 36000.0, "mass overlay");
    tram_dr::PlantParams p = tram_dr::default_plant_params();
    tram_dr::ScaParams s{};
    tram_dr::apply_vehicle_overlay(ov, &p, &s);
    expect(p.m0_kg > 36000.0, "plant mass from yaml");
    expect(!s.pair_lr, "sca pair_lr from yaml");
  }
  {
    const char* y =
        "/**:\n"
        "  ros__parameters:\n"
        "    vehicle_profile: lvenok_moscow\n"
        "    n_wheels: 4\n"
        "    sca_pair_lr: false\n"
        "    axle_role: [0, 0, 0, 0]\n"
        "    mass_min_kg: 15000.0\n"
        "    mass_max_kg: 40000.0\n"
        "    mass_door_kg: 12000.0\n"
        "    r0_uncalibrated: true\n";
    tram_dr::VehicleOverlay ov;
    expect(tram_dr::parse_vehicle_yaml_text(y, &ov), "parse lvenok clip yaml");
    expect(!ov.has_mass, "lvenok m0 TBD");
    expect(ov.has_mass_min && ov.mass_min_kg == 15000.0, "lvenok mass_min 15 t");
    expect(ov.has_mass_max && ov.mass_max_kg == 40000.0, "lvenok mass_max 40 t");
    expect(ov.has_mass_door && ov.mass_door_kg == 12000.0, "lvenok mass_door");
    expect(ov.has_r0_uncalibrated && ov.r0_uncalibrated, "lvenok r0 uncalibrated");
    tram_dr::PlantParams p = tram_dr::default_plant_params();
    tram_dr::ScaParams s{};
    tram_dr::apply_vehicle_overlay(ov, &p, &s);
    expect(p.mass_min_kg == 15000.0 && p.mass_max_kg == 40000.0, "plant clip from yaml");
    expect(p.m0_kg == 28000.0, "unset mass_kg leaves Combino m0");
  }
  {
    const char* y =
        "map_projector:\n"
        "  ros__parameters:\n"
        "    route_s_m: [0.0, 174.7, 408.4]\n";
    double s[16];
    int n = 0;
    expect(tram_dr::parse_route_s_m(y, s, &n, 16), "parse route_s_m");
    expect(n == 3, "three stops");
    expect(std::fabs(s[1] - 174.7) < 1e-9, "second stop s");
  }
  {
    const char* y =
        "map_projector:\n"
        "  ros__parameters:\n"
        "    route_s_m: [0.0, 174.7]\n"
        "state_estimator:\n"
        "  ros__parameters:\n"
        "    route_s_m: [0.0, 174.7]\n";
    double s[16];
    int n = 0;
    expect(tram_dr::parse_route_s_m(y, s, &n, 16), "parse first route_s_m");
    expect(n == 2, "first list wins");
  }
  {
    const char* y =
        "# route_s_m: [9.9, 9.9]\n"
        "#     stop_gate_m: 1.0\n"
        "map_projector:\n"
        "  ros__parameters:\n"
        "    route_s_m: [0.0, 174.7]\n"
        "state_estimator:\n"
        "  ros__parameters:\n"
        "    stop_gate_m: 40.0\n";
    double s[16];
    int n = 0;
    expect(tram_dr::parse_route_s_m(y, s, &n, 16), "comment does not poison route_s_m");
    expect(n == 2, "live list, not comment");
    expect(std::fabs(s[0]) < 1e-12, "first live stop");
    double gate = -1.0;
    expect(tram_dr::parse_stop_gate_m(y, &gate), "parse stop_gate_m");
    expect(std::fabs(gate - 40.0) < 1e-12, "comment does not poison stop_gate");
  }
  {
    tram_dr::Ukf ukf;
    tram_dr::UkfParams cfg = ukf.params();
    cfg.n_stops = 1;
    cfg.stop_s_m[0] = 0.0;
    cfg.stop_gate_m = 40.0;
    cfg.mass_prior_log_sigma = 0.0;
    ukf.set_params(cfg);
    tram_dr::Input u;
    u.notch_valid = true;
    u.notch = 0.0;
    u.brake = 0.3;
    double z[4] = {0.0, 0.0, 0.0, 0.0};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 40; ++k) {
      e = ukf.predict_and_update(u, z, 4, 0.02);
    }
    expect(e.mode == tram_dr::Mode::kStandstill, "zupt at s=0 is a stop");
    expect(e.zupt_at_stop, "door allowed at origin stop");
    const double p0 = e.p_mm;
    u.brake = 0.0;
    u.notch = 0.06;
    double omega[4] = {0.1, 0.1, 0.1, 0.1};
    e = ukf.predict_and_update(u, omega, 4, 0.02);
    expect(e.p_mm > p0 + 0.008, "mass_door at stop inflates P_mm");
  }
  {
    tram_dr::Ukf ukf;
    tram_dr::UkfParams cfg = ukf.params();
    cfg.n_stops = 1;
    cfg.stop_s_m[0] = 0.0;
    cfg.stop_gate_m = 40.0;
    cfg.mass_prior_log_sigma = 0.0;
    ukf.set_params(cfg);
    tram_dr::Input u;
    u.notch_valid = true;
    u.notch = 0.8;
    const double r = 0.35;
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 800; ++k) {
      const double w = std::max(e.x.v_mps, 1.0) / r;
      double omega[4] = {w, w, w, w};
      e = ukf.predict_and_update(u, omega, 4, 0.02);
    }
    expect(e.x.s_m > 50.0, "driven past stop gate");
    expect(!e.zupt_at_stop, "away from stop vertex");
    // Come to a genuine stand at a switch: release traction, apply the service
    // brake, ramp the wheels down to rest at the commanded rate, then hold.
    // Feeding omega=0 to a body still doing ~12 m/s is the locked-wheel slide
    // case, which "no zupt on sliding lock" above and "false zero wheels cannot
    // erase a moving body" in test_integrity_contracts both forbid: standstill
    // must not be reachable that way. Only the mass door is under test here.
    u.notch = 0.0;
    u.brake = 0.6;
    double v_cmd = std::max(e.x.v_mps, 0.0);
    for (int k = 0; k < 1500; ++k) {
      v_cmd = std::max(0.0, v_cmd - 0.6 * 1.2 * 0.02);
      const double w = v_cmd / r;
      double omega[4] = {w, w, w, w};
      e = ukf.predict_and_update(u, omega, 4, 0.02);
    }
    expect(e.mode == tram_dr::Mode::kStandstill, "zupt off stop after a real stand");
    expect(!e.zupt_at_stop, "switch wait is not a stop");
    const double p_hold = e.p_mm;
    u.brake = 0.0;
    u.notch = 0.06;
    double leave[4] = {0.1, 0.1, 0.1, 0.1};
    e = ukf.predict_and_update(u, leave, 4, 0.02);
    expect(!e.zupt_at_stop, "leave still off stop");
    const double door = (3000.0 / 28000.0) * (3000.0 / 28000.0);
    expect(e.p_mm < p_hold + 0.5 * door, "no mass_door off stop");
  }
  {
    // WSP-held slide: |kappa| vs UKF-v stays below kappa_cut; channel A sees it.
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch_valid = true;
    const double r = 0.35;
    double v_body = 15.0;
    tram_dr::UkfEstimate e{};
    u.notch = 0.0;
    u.brake = 0.0;
    for (int k = 0; k < 40; ++k) {
      const double w = v_body / r;
      double omega[4] = {w, w, w, w};
      e = ukf.predict_and_update(u, omega, 4, 0.02);
    }
    u.brake = 0.85;
    for (int k = 0; k < 700; ++k) {
      v_body = std::max(4.0, v_body - 0.45 * 0.02);
      const double v_w = 0.82 * v_body;
      const double w = v_w / r;
      double omega[4] = {w, w, w, w};
      e = ukf.predict_and_update(u, omega, 4, 0.02);
    }
    expect(std::fabs(e.relative_wheel_slide) < 0.25, "WSP-held kappa below cut");
    expect(e.path_disagree_latched || e.confidence_s == tram_dr::Confidence::kDegraded,
           "path disagree latches WSP-held slide");
    expect(e.confidence != tram_dr::Confidence::kOk, "not silent OK on WSP-held slide");
  }
  {
    tram_dr::PlantParams twin = tram_dr::default_plant_params();
    twin.m0_kg = 22000.0;
    tram_dr::Ukf ukf;
    ukf.set_plant(twin);
    tram_dr::State body;
    body.m_eff_kg = 22000.0;
    tram_dr::Input u;
    u.notch = 0.55;
    u.notch_valid = true;
    const double r = 0.35;
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 500; ++k) {
      const double t = 0.02 * static_cast<double>(k);
      const double i = (t < 5.0) ? 0.03 * (t / 5.0) : 0.03;
      body.f_bias_n = body.m_eff_kg * 9.81 * i;
      tram_dr::plant_step(body, u, 0.02, twin);
      const double w = body.v_mps / r;
      double omega[4] = {w, w, w, w};
      e = ukf.predict_and_update(u, omega, 4, 0.02);
    }
    const double target = 22000.0 * 9.81 * 0.03;
    expect(std::fabs(e.x.f_bias_n - target) < 4000.0, "F_bias tracks 3% grade ramp 22 t");
    expect(e.chol_fail == 0, "grade ramp chol");
  }
  {
    // Estimated Strogino descent 3.2% (not a survey; centre of 25-40 permille).
    // Coast: no i(s) in the filter. F_bias must eat the grade; kappa on coast is
    // model error, not slide.
    tram_dr::PlantParams twin = tram_dr::default_plant_params();
    twin.m0_kg = 22000.0;
    tram_dr::Ukf ukf;
    ukf.set_plant(twin);
    tram_dr::State body;
    body.m_eff_kg = 22000.0;
    tram_dr::Input u;
    u.notch_valid = true;
    const double r = 0.35;
    tram_dr::UkfEstimate e{};
    u.notch = 0.8;
    for (int k = 0; k < 500; ++k) {
      tram_dr::plant_step(body, u, 0.02, twin);
      const double w = body.v_mps / r;
      double omega[4] = {w, w, w, w};
      e = ukf.predict_and_update(u, omega, 4, 0.02);
    }
    u.notch = 0.0;
    u.brake = 0.0;
    for (int k = 0; k < 500; ++k) {
      const double t = 0.02 * static_cast<double>(k);
      const double i = (t < 8.0) ? 0.032 * (t / 8.0) : 0.032;
      body.f_bias_n = -body.m_eff_kg * 9.81 * i;
      tram_dr::plant_step(body, u, 0.02, twin);
      const double w = body.v_mps / r;
      double omega[4] = {w, w, w, w};
      e = ukf.predict_and_update(u, omega, 4, 0.02);
    }
    const double target = -22000.0 * 9.81 * 0.032;
    // Coast q_Fb is 1000 N/sqrt(s): do not demand millinewton match to mgi.
    // Integrity: sign of the ramp, no kappa latch (coast_quiet), v stays OK.
    expect(e.x.f_bias_n < -2000.0, "F_bias sign follows Strogino descent");
    expect(std::fabs(e.x.f_bias_n - target) < 9000.0, "F_bias near Strogino mgi");
    expect(!e.slip_latched, "coast on grade is not a slip latch");
    expect(e.confidence_v == tram_dr::Confidence::kOk, "coast on grade v stays OK");
    expect(e.chol_fail == 0, "Strogino coast chol");
  }
  {
    // Class-order tare 22 t is NOT a passport: kernel (dm, dk) vs +12 t pax
    // (170 x 70 kg estimate). Expect v OK; do not dump the extra mass into F_bias.
    tram_dr::PlantParams twin = tram_dr::default_plant_params();
    twin.m0_kg = 22000.0;
    twin.mass_min_kg = 15000.0;
    twin.mass_max_kg = 40000.0;
    tram_dr::Ukf ukf;
    ukf.set_plant(twin);
    tram_dr::State body;
    body.m_eff_kg = 34000.0;
    tram_dr::Input u;
    u.notch = 0.7;
    u.notch_valid = true;
    const double r = 0.35;
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 600; ++k) {
      tram_dr::plant_step(body, u, 0.02, twin);
      body.m_eff_kg = 34000.0;
      const double w = body.v_mps / r;
      double omega[4] = {w, w, w, w};
      e = ukf.predict_and_update(u, omega, 4, 0.02);
    }
    expect(e.confidence_v == tram_dr::Confidence::kOk, "heavy pax v stays OK");
    expect(std::fabs(e.x.f_bias_n) < 8000.0, "pax mass not dumped into F_bias");
    expect(e.chol_fail == 0, "heavy pax chol");
  }
  {
    tram_dr::UkfParams cfg;
    cfg.mass_prior_log_sigma = 0.3;
    tram_dr::Ukf ukf(cfg);
    tram_dr::Input u;
    u.notch_valid = true;
    const double r = 0.35;
    double omega[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 4000; ++k) {
      e = ukf.predict_and_update(u, omega, 4, 0.02);
    }
    expect(e.chol_fail == 0, "mass prior 80 s chol");
    expect(e.x.m_eff_kg > 15000.0 && e.x.m_eff_kg < 50000.0, "mass stayed in clip");
    // Stationary variance of the Gauss–Markov prior is R=0.09. The defective
    // once-per-second fusion pinned P_mm at 8.61e-04 (σ=0.0293).
    // τ=300 s: after 80 s the gap to R=0.09 has only shrunk by ~41%.
    // Init P_mm=(2000/m0)²≈5e-3, so P≈0.04 — still ≫ the 8.61e-04 pin.
    expect(e.p_mm > 0.02 && e.p_mm < 0.12, "mass prior growing toward R=0.09");
    expect(e.p_mm > 10.0 * 8.61e-04, "mass prior is not the repeated-update pin");
  }
  {
    double lat = 0, lon = 0;
    expect(tram_dr::project_s(100.0, lat, lon), "map");
    expect(lat > 55.0, "lat near Moscow");
    expect(lon > 37.0 && lon < 37.5, "lon in Strogino/Shchukino");
    expect(!tram_dr::project_s(std::numeric_limits<double>::quiet_NaN(), lat, lon),
           "project_s rejects nonfinite s");
  }
  {
    tram_dr::State x;
    x.v_mps = 0.11;
    tram_dr::Input u;
    u.brake = 1.0;
    u.notch = 0.0;
    const double e0 = 0.5 * 28000.0 * x.v_mps * x.v_mps;
    tram_dr::plant_step(x, u, 0.2);
    expect(x.v_mps >= 0.0 && x.v_mps <= 0.11 + 1e-12, "passive brake does not reverse");
    const double e1 = 0.5 * 28000.0 * x.v_mps * x.v_mps;
    expect(e1 <= e0 + 1e-6, "passive brake does not grow kinetic energy");
  }
  {
    double A[4] = {1.0e308, 0.0, 0.0, 1.0e308};
    double orig[4];
    std::memcpy(orig, A, sizeof(orig));
    const bool ok = tram_dr::la::project_pd(A, 2);
    if (!ok) {
      expect(std::memcmp(A, orig, sizeof(orig)) == 0, "project_pd false is byte-stable");
    }
  }
  {
    using tram_dr::MeasDt;
    const auto a = tram_dr::stamped_interval(false, 0.0, 100.0, 0.0, 0.001, 0.001);
    expect(a.kind == MeasDt::Kind::kAnchor && a.advance_stamp, "first stamp anchors");
    const auto r = tram_dr::stamped_interval(true, 100.0, 99.0, 0.0, 0.001, 0.001);
    expect(r.kind == MeasDt::Kind::kReject && !r.advance_stamp, "OOO does not move stamp");
    const auto d = tram_dr::stamped_interval(true, 100.0, 100.0, 0.0, 0.001, 0.001);
    expect(d.kind == MeasDt::Kind::kSkip, "duplicate stamp is skip not floor");
    const auto g = tram_dr::stamped_interval(true, 100.0, 100.04, 0.0, 0.001, 0.001);
    expect(g.kind == MeasDt::Kind::kApply && std::fabs(g.dt - 0.04) < 1e-12, "valid interval");
  }
  {
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch_valid = true;
    const double r = 0.35;
    double roll[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 50; ++k) {
      e = ukf.predict_and_update(u, roll, 4, 0.02);
    }
    const double zero[4] = {0, 0, 0, 0};
    for (int k = 0; k < 500; ++k) {
      e = ukf.predict_and_update(u, zero, 4, 0.02);
    }
    expect(e.path_integrity_latched, "forced ZUPT-at-speed latches path");
    for (int k = 0; k < 40; ++k) {
      e = ukf.predict_and_update(u, roll, 4, 0.02);
    }
    expect(e.path_integrity_latched, "recovered v does not clear path latch");
    expect(e.confidence != tram_dr::Confidence::kOk, "path latch is not HMI-OK");
    expect(e.s_unbounded, "lost metres are unbounded after forced ZUPT");
  }
  {
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch_valid = true;
    const double r = 0.35;
    double roll[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
    const double zero[4] = {0, 0, 0, 0};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 50; ++k) {
      e = ukf.predict_and_update(u, roll, 4, 0.02);
    }
    for (int k = 0; k < 16; ++k) {
      e = ukf.predict_and_update(u, zero, 4, 0.02);
    }
    expect(!e.path_integrity_latched, "0.32 s lock is not forced ZUPT");
    for (int k = 0; k < 20; ++k) {
      e = ukf.predict_and_update(u, roll, 4, 0.02);
    }
    expect(!e.path_integrity_latched, "short lock does not persist a path latch");
  }
  {
    tram_dr::Ukf ukf;
    tram_dr::Input u;
    u.notch_valid = true;
    const double r = 0.35;
    double roll[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
    tram_dr::UkfEstimate e{};
    for (int k = 0; k < 50; ++k) {
      e = ukf.predict_and_update(u, roll, 4, 0.02);
    }
    e = ukf.predict_and_update(u, roll, 4, std::numeric_limits<double>::quiet_NaN());
    expect(e.confidence == tram_dr::Confidence::kLost, "rejected dt is LOST this frame");
    expect(e.path_integrity_latched, "rejected frame latches path");
    for (int k = 0; k < 30; ++k) {
      e = ukf.predict_and_update(u, roll, 4, 0.02);
    }
    expect(e.path_integrity_latched, "valid dt does not unlatch lost path");
    expect(e.confidence != tram_dr::Confidence::kOk, "recovered v is not recovered s");
  }
  if (g_fails) {
    std::fprintf(stderr, "%d failures\n", g_fails);
    return 1;
  }
  std::printf("tram_dr core ok\n");
  return 0;
}
