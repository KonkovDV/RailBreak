// Physical/API regressions. Runs under NDEBUG; no ROS and no ground-truth fit.
#include "tram_dr_localization/plant.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace {
int checks = 0;
int failures = 0;
void check(bool ok, const char* label) {
  ++checks;
  if (!ok) { ++failures; std::cerr << "FAIL: " << label << '\n'; }
}
bool close(double a, double b, double tol = 1e-12) {
  return std::isfinite(a) && std::fabs(a - b) <= tol * std::max(1.0, std::fabs(b));
}

void forces_and_units() {
  using namespace tram_dr;
  PlantParams p;
  const double fmax = p.m0_kg * p.a_trac_max;
  check(close(traction_star_n(1.0, 0.0, p), fmax), "constant-force traction region");
  check(close(traction_star_n(1.0, 2.0 * p.v_base_mps, p), fmax / 2.0),
        "constant-power traction region");
  check(close(traction_star_n(-1.0, 0.0, p), -fmax), "signed traction command");
  for (double v : {0.2, 1.0, 6.0, 20.0}) {
    check(close(davis_resistance_n(v, p), -davis_resistance_n(-v, p)),
          "Davis force is odd in signed speed");
    check(davis_resistance_n(v, p) * v >= 0.0, "Davis resistance removes kinetic energy");
  }
  p.A_d = p.B_d = p.C_d = 0.0;
  State x; x.v_mps = 5.0; x.mu_hat = 0.06;
  Input brake; brake.brake = 1.0;
  const auto limited = plant_forces(x, brake, p);
  check(close(limited.a_mps2, -p.g * x.mu_hat), "wheel brake obeys adhesion cap mu*g");
  p.gamma_rot = 0.1;
  check(close(plant_forces(x, brake, p).a_mps2, limited.a_mps2 / 1.1),
        "rotating inertia changes acceleration, not adhesion force cap");
  check(close(plant_forces(x, brake, p).f_adh_cap_n, x.m_eff_kg * p.g * x.mu_hat),
        "adhesion cap has force units and uses translational mass");
  p.gamma_rot = 0.0;
  p.brake_nonadhesive_frac = 0.5;
  check(close(plant_forces(x, brake, p).a_mps2, -p.g * x.mu_hat - 0.5 * p.a_svc),
        "nonadhesive rail brake is outside wheel adhesion cap");
}

void drive_memory() {
  using namespace tram_dr;
  PlantParams p; p.A_d = p.B_d = p.C_d = 0.0;
  p.tau_drv_s = 1.0;
  Input u; u.notch = 1.0;
  State x;
  plant_step(x, u, 0.2, p);
  check(close(x.v_mps, 0.26), "no-memory overload is algebraic, not repeated cold-start PT1");
  check(close(x.s_m, 0.026), "constant-acceleration position integration");
  State stateful;
  double force = 0.0;
  const double expected_force = p.m0_kg * p.a_trac_max * 0.2 / 1.2;
  plant_step(stateful, u, 0.2, p, &force);
  check(close(force, expected_force), "stateful backward-Euler PT1 first step");
  plant_step(stateful, u, 0.2, p, &force);
  check(close(force, (expected_force + 0.2 * p.m0_kg * p.a_trac_max) / 1.2),
        "stateful PT1 retains memory on second step");
  p.tau_drv_s = 0.0;
  p.j_max_mps3 = 0.5;
  State algebraic;
  plant_step(algebraic, u, 0.2, p);
  check(close(algebraic.v_mps, 0.26), "no-memory overload does not discard jerk state each tick");
  State rate_limited;
  force = 0.0;
  plant_step(rate_limited, u, 0.2, p, &force);
  check(close(force, p.m0_kg * p.j_max_mps3 * 0.2), "stateful jerk limiter bounds delta force");
}

void passive_stop() {
  using namespace tram_dr;
  PlantParams p;
  p.v_eps = 0.001;
  p.B_d = p.C_d = 0.0;
  Input coast;
  for (double sign : {-1.0, 1.0}) {
    State x; x.v_mps = sign * 0.003;
    const double decel = p.A_d / x.m_eff_kg;
    const double expected_s = sign * 0.003 * 0.003 / (2.0 * decel);
    plant_step(x, coast, 0.2, p);
    check(close(x.v_mps, 0.0), "passive coasting resistance cannot reverse the vehicle");
    check(close(x.s_m, expected_s), "coast integrates only until its zero-crossing event");
  }
  // External forces may cause a real reversal: do not confuse it with
  // passive-resistance overshoot, and do not model grade as a passive brake.
  p.A_d = 0.0;
  p.i_grade = 0.2;
  State grade; grade.v_mps = 0.15;
  plant_step(grade, coast, 0.2, p);
  check(grade.v_mps < 0.0, "grade-driven reversal is not suppressed by passive-coast guard");
  p.i_grade = 0.0;
  State bias; bias.v_mps = 0.15; bias.f_bias_n = 28000.0;
  plant_step(bias, coast, 0.2, p);
  check(bias.v_mps < 0.0, "external bias force is not misclassified as passive resistance");
  // Existing passive-brake behavior remains protected, in both directions.
  Input brake; brake.brake = 1.0;
  for (double sign : {-1.0, 1.0}) {
    State x; x.v_mps = sign * 0.15;
    plant_step(x, brake, 0.2, p);
    check(close(x.v_mps, 0.0), "passive brake stops without numerical reversal");
  }
}
}  // namespace

int main() {
  forces_and_units();
  drive_memory();
  passive_stop();
  std::cout << checks << " checks, " << failures << " failures\n";
  return failures == 0 ? 0 : 1;
}
