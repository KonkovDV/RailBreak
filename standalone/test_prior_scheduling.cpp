// Integration test: a process prior runs on filter time, not wheel arrivals.
// The closed-form reference is independent of prior.hpp. No assert/NDEBUG.
#include "tram_dr_localization/ukf.hpp"

#include <cmath>
#include <iostream>
#include <limits>

namespace {
int checks = 0;
int failures = 0;
void check(bool ok, const char* label) {
  ++checks;
  if (!ok) { ++failures; std::cerr << "FAIL: " << label << '\n'; }
}

tram_dr::Ukf make_filter(double sigma = 0.1, double tau = 0.5) {
  tram_dr::UkfParams cfg;
  cfg.mass_prior_log_sigma = sigma;
  cfg.mass_prior_tau_s = tau;
  cfg.mass_door_kg = 0.0;  // isolate the process from passenger-exchange impulses
  cfg.q_v = 0.0;
  tram_dr::Ukf f(cfg);
  tram_dr::PlantParams plant;
  plant.A_d = plant.B_d = plant.C_d = 0.0;
  plant.a_trac_max = plant.a_svc = 0.0;
  f.set_plant(plant);
  return f;
}

// 0 = silence, 1 = all-invalid packet, 2 = full zero/rest packet.
tram_dr::UkfEstimate advance(tram_dr::Ukf& f, int kind, double dt) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double dead[4]{nan, nan, nan, nan};
  const double rest[4]{};
  return f.predict_and_update(tram_dr::Input{},
                              kind == 0 ? nullptr : (kind == 1 ? dead : rest),
                              kind == 0 ? 0 : 4, dt);
}

void elapsed_time_contract(int kind, bool irregular) {
  auto f = make_filter();
  auto e = advance(f, 2, 0.02);  // initialization is separate from this interval
  const double p0 = e.p_mm;
  const double steps[4]{0.001, 0.007, 0.03, 0.2};
  double elapsed = 0.0;
  bool healthy = e.initialized && e.chol_fail == 0;
  for (int i = 0; i < 40; ++i) {
    const double dt = irregular ? steps[i % 4] : 0.02;
    e = advance(f, kind, dt);
    elapsed += dt;
    healthy = healthy && e.chol_fail == 0 && std::isfinite(e.p_mm);
  }
  const double r = 0.1 * 0.1;
  const double expected = r + (p0 - r) * std::exp(-2.0 * elapsed / 0.5);
  // The UKF adds Cholesky jitter each prediction; allow its accumulated
  // numerical contribution, not a percentage large enough to hide no prior.
  check(healthy, "prior scheduling never introduces a numerical rejection");
  check(std::fabs(e.p_mm - expected) < 5e-7,
        "mass variance follows elapsed-time Markov law on every packet kind");
  check(e.p_mm > p0 + 0.002, "missing/rest wheels do not freeze mass uncertainty");
}

void limits_and_rejection() {
  for (int kind : {0, 1, 2}) {
    auto f = make_filter(0.1, 0.0);
    advance(f, 2, 0.02);
    const auto e = advance(f, kind, 0.02);
    check(e.chol_fail == 0 && std::fabs(e.p_mm - 0.01) < 1e-12,
          "tau=0 applies stationary variance on every packet kind");
  }
  auto disabled = make_filter(0.0, 0.5);
  auto e = advance(disabled, 2, 0.02);
  const double p0 = e.p_mm;
  for (int i = 0; i < 25; ++i) e = advance(disabled, 0, 0.02);
  const double q = (80.0 / 28000.0) * (80.0 / 28000.0);
  check(e.chol_fail == 0 && std::fabs(e.p_mm - (p0 + q * 0.5)) < 1e-7,
        "sigma=0 retains the original random walk, without double counting");

  auto f = make_filter();
  e = advance(f, 2, 0.02);
  const double before = e.p_mm;
  tram_dr::Input poisoned;
  poisoned.notch = std::numeric_limits<double>::quiet_NaN();
  e = f.predict_and_update(poisoned, nullptr, 0, 0.02);
  check(e.confidence == tram_dr::Confidence::kLost && e.p_mm == before,
        "rejected input does not advance the mass prior");
  auto waiting = make_filter();
  e = advance(waiting, 0, 0.02);
  check(!e.initialized && e.p_mm == 1e6,
        "no prior propagation before wheel initialization");
}
}  // namespace

int main() {
  for (int kind : {0, 1, 2}) {
    elapsed_time_contract(kind, false);
    elapsed_time_contract(kind, true);
  }
  limits_and_rejection();
  std::cout << checks << " checks, " << failures << " failures\n";
  return failures == 0 ? 0 : 1;
}
