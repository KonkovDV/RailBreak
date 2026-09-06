#include "tram_dr_localization/plant.hpp"
#include "tram_dr_localization/types.hpp"

#include <cmath>
#include <gtest/gtest.h>

TEST(Plant, DefaultMassIsCombinoTare) {
  tram_dr::State x;
  EXPECT_DOUBLE_EQ(x.m_eff_kg, 28000.0);
}

TEST(Plant, DavisGrowsWithSpeed) {
  const auto p = tram_dr::default_plant_params();
  const double r0 = tram_dr::davis_resistance_n(0.0, p);
  const double r10 = tram_dr::davis_resistance_n(10.0, p);
  const double rneg = tram_dr::davis_resistance_n(-10.0, p);
  EXPECT_NEAR(r0, 0.0, 1e-9);
  EXPECT_GT(r10, 0.0);
  EXPECT_NEAR(rneg, -r10, 1e-9);
}

TEST(Plant, DavisOpposesReverseMotion) {
  tram_dr::State x;
  x.v_mps = -0.5;
  tram_dr::Input u;
  tram_dr::plant_step(x, u, 0.02);
  EXPECT_GT(x.v_mps, -0.5);
}

TEST(Plant, NotchAcceleratesForward) {
  tram_dr::State x;
  x.v_mps = 1.0;
  tram_dr::Input u;
  u.notch = 1.0;
  tram_dr::plant_step(x, u, 0.02);
  EXPECT_GT(x.v_mps, 1.0);
  EXPECT_GT(x.s_m, 0.0);
}

TEST(Plant, WetBrakeCappedByAdhesion) {
  tram_dr::State x;
  x.v_mps = 5.0;
  x.mu_hat = 0.06;
  tram_dr::Input u;
  u.brake = 1.0;
  const auto p = tram_dr::default_plant_params();
  const auto d = tram_dr::plant_forces(x, u, p);
  EXPECT_LE(std::fabs(d.f_trac_n), d.f_adh_cap_n + 1e-6);
}

TEST(Plant, StateDimIsSixPlusWheels) {
  EXPECT_EQ(tram_dr::kNWheels, 6);
  EXPECT_EQ(tram_dr::kStateDim, 12);
  EXPECT_EQ(tram_dr::kSigma, 25);
  EXPECT_EQ(tram_dr::kMu, 11);
}

TEST(Plant, FirstStepMatchesPythonTwin) {
  tram_dr::State x;
  tram_dr::Input u;
  u.notch = 1.0;
  tram_dr::plant_step(x, u, 0.02);
  const double a = 28000.0 * 1.3 / 28000.0;
  EXPECT_NEAR(x.v_mps, a * 0.02, 1e-9);
  EXPECT_NEAR(x.s_m, 0.5 * a * 0.02 * 0.02, 1e-12);
}

TEST(Plant, TauDrvSlowsFirstStep) {
  tram_dr::PlantParams p = tram_dr::default_plant_params();
  p.tau_drv_s = 0.12;
  tram_dr::State x;
  tram_dr::Input u;
  u.notch = 1.0;
  double f_lag = 0.0;
  tram_dr::plant_step(x, u, 0.02, p, &f_lag);
  const double a = 1.3;
  EXPECT_LT(x.v_mps, a * 0.02 - 1e-6);
  EXPECT_GT(f_lag, 0.0);
  EXPECT_LT(f_lag, 28000.0 * 1.3);
}

TEST(Plant, GammaRotLowersAccel) {
  tram_dr::PlantParams p = tram_dr::default_plant_params();
  p.gamma_rot = 0.10;
  tram_dr::State x;
  tram_dr::Input u;
  u.notch = 1.0;
  tram_dr::plant_step(x, u, 0.02, p);
  const double a = 1.3;
  EXPECT_LT(x.v_mps, a * 0.02 - 1e-6);
}

TEST(Plant, ClampsMassAfterStep) {
  tram_dr::State x;
  x.m_eff_kg = 80000.0;
  tram_dr::Input u;
  tram_dr::plant_step(x, u, 0.02);
  EXPECT_LE(x.m_eff_kg, 70000.0);
  EXPECT_GE(x.m_eff_kg, 20000.0);
}

TEST(Plant, NotchAsAccelUsesLiveMass) {
  tram_dr::PlantParams p = tram_dr::default_plant_params();
  p.notch_as_accel = true;
  tram_dr::State x;
  x.m_eff_kg = 40000.0;
  tram_dr::Input u;
  u.notch = 1.0;
  tram_dr::plant_step(x, u, 0.02, p);
  const double a = 1.3;
  EXPECT_NEAR(x.v_mps, a * 0.02, 1e-9);
}

TEST(Plant, GradeForceOnCoast) {
  tram_dr::PlantParams p = tram_dr::default_plant_params();
  p.i_grade = 0.02;
  tram_dr::State x;
  tram_dr::Input u;
  tram_dr::plant_step(x, u, 0.02, p);
  const double a = -(28000.0 * 9.81 * 0.02) / 28000.0;
  EXPECT_NEAR(x.v_mps, a * 0.02, 1e-9);
}

TEST(Plant, ClipParamsFalseLeavesMass) {
  tram_dr::State x;
  x.m_eff_kg = 60000.0;
  tram_dr::Input u;
  tram_dr::plant_step(x, u, 0.02, tram_dr::default_plant_params(), nullptr, false);
  EXPECT_NEAR(x.m_eff_kg, 60000.0, 1e-9);
}

TEST(Plant, LvenokMassClip15to40t) {
  tram_dr::PlantParams p = tram_dr::default_plant_params();
  p.mass_min_kg = 15000.0;
  p.mass_max_kg = 40000.0;
  tram_dr::State x;
  x.m_eff_kg = 50000.0;
  tram_dr::Input u;
  tram_dr::plant_step(x, u, 0.02, p);
  EXPECT_LE(x.m_eff_kg, 40000.0);
  x.m_eff_kg = 10000.0;
  tram_dr::plant_step(x, u, 0.02, p);
  EXPECT_GE(x.m_eff_kg, 15000.0);
}

TEST(Plant, JerkLimiterCapsFirstStep) {
  tram_dr::PlantParams p = tram_dr::default_plant_params();
  p.j_max_mps3 = 0.7;
  tram_dr::State x;
  tram_dr::Input u;
  u.notch = 1.0;
  double f_lag = 0.0;
  tram_dr::plant_step(x, u, 0.02, p, &f_lag);
  const double a_alg = 1.3;
  EXPECT_LT(x.v_mps, a_alg * 0.02 - 1e-6);
  EXPECT_NEAR(f_lag, 28000.0 * 0.7 * 0.02, 1.0);
}

TEST(Plant, InvalidNotchAppliesZeroCommand) {
  tram_dr::State live;
  tram_dr::State stale;
  live.v_mps = stale.v_mps = 1.0;
  tram_dr::Input u;
  u.notch = 1.0;
  tram_dr::plant_step(live, u, 0.02);
  u.notch_valid = false;
  tram_dr::plant_step(stale, u, 0.02);
  EXPECT_GT(live.v_mps, stale.v_mps + 1e-4);
}

TEST(Plant, InvalidBrakeAppliesZeroCommand) {
  tram_dr::State live;
  tram_dr::State stale;
  live.v_mps = stale.v_mps = 5.0;
  tram_dr::Input u;
  u.brake = 1.0;
  tram_dr::plant_step(live, u, 0.02);
  u.brake_valid = false;
  tram_dr::plant_step(stale, u, 0.02);
  EXPECT_LT(live.v_mps, stale.v_mps - 1e-4);
}
