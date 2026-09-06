#include "tram_dr_localization/lin_alg.hpp"
#include "tram_dr_localization/ukf.hpp"

#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <stdexcept>

TEST(LinAlg, CholIdentity) {
  double A[4] = {1, 0, 0, 1};
  double L[4];
  ASSERT_TRUE(tram_dr::la::chol(A, L, 2, 0.0));
  EXPECT_NEAR(L[0], 1.0, 1e-9);
  EXPECT_NEAR(L[3], 1.0, 1e-9);
}

TEST(Ukf, UninitializedUntilInputs) {
  tram_dr::Ukf ukf;
  tram_dr::Input u;
  const auto e = ukf.predict_and_update(u, nullptr, 0);
  EXPECT_FALSE(e.initialized);
  EXPECT_EQ(e.confidence, tram_dr::Confidence::kUninitialized);
}

TEST(Ukf, InitializesAndTracksConsistentWheels) {
  tram_dr::Ukf ukf;
  tram_dr::Input u;
  u.notch = 0.0;
  u.notch_valid = true;
  const double v = 5.0;
  const double r = 0.35;
  const double w = v / r;
  double omega[4] = {w, w, w, w};
  tram_dr::UkfEstimate e{};
  for (int k = 0; k < 50; ++k) {
    e = ukf.predict_and_update(u, omega, 4, 0.02);
  }
  EXPECT_TRUE(e.initialized);
  EXPECT_NEAR(e.x.v_mps, v, 1.5);
  EXPECT_LT(e.p_ss, 1.0e6);
  EXPECT_NE(e.confidence, tram_dr::Confidence::kUninitialized);
}

TEST(Ukf, ZuptWithHoldBrake) {
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
  EXPECT_EQ(e.mode, tram_dr::Mode::kStandstill);
  EXPECT_NEAR(e.x.v_mps, 0.0, 1e-12);
  EXPECT_LT(std::fabs(e.x.f_bias_n), 250.0);
  for (int k = 0; k < 20; ++k) {
    e = ukf.predict_and_update(u, z, 4, 0.02);
  }
  EXPECT_NEAR(e.x.v_mps, 0.0, 1e-9);
}

TEST(Ukf, NoZuptOnSlidingLock) {
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
  EXPECT_GT(e.x.v_mps, 1.0);
}

TEST(Ukf, CommonModeInflatesAgreeingWheels) {
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
  EXPECT_EQ(e.sca.n_inflated, 0);
  EXPECT_TRUE(e.sca.common_mode);
}

TEST(Ukf, HealthyRunIsOkNotLost) {
  tram_dr::Ukf ukf;
  tram_dr::Input u;
  u.notch_valid = true;
  const double r = 0.35;
  double omega[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
  tram_dr::UkfEstimate e{};
  for (int k = 0; k < 50; ++k) {
    e = ukf.predict_and_update(u, omega, 4, 0.02);
  }
  EXPECT_EQ(e.confidence, tram_dr::Confidence::kOk);
  EXPECT_LT(e.p_ss, 50.0 * 50.0);
}

TEST(Ukf, CommonModeSlipLatchesDegraded) {
  // Wheels re-agreeing with the body must not restore OK on s within a run.
  // v can return to OK: axles still agree with each other.
  tram_dr::Ukf ukf;
  tram_dr::Input u;
  u.notch = 0.4;
  u.notch_valid = true;
  const double r = 0.35;
  double omega[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
  tram_dr::UkfEstimate e{};
  for (int k = 0; k < 50; ++k) {
    e = ukf.predict_and_update(u, omega, 4, 0.02);
  }
  double fast[4] = {12.0 / r, 12.0 / r, 12.0 / r, 12.0 / r};
  for (int k = 0; k < 15; ++k) {
    e = ukf.predict_and_update(u, fast, 4, 0.02);
  }
  EXPECT_TRUE(e.sca.common_mode);
  EXPECT_EQ(e.confidence, tram_dr::Confidence::kDegraded);
  for (int k = 0; k < 400; ++k) {
    e = ukf.predict_and_update(u, fast, 4, 0.02);
  }
  EXPECT_FALSE(e.sca.common_mode);
  EXPECT_EQ(e.confidence, tram_dr::Confidence::kDegraded);
  EXPECT_EQ(e.confidence_s, tram_dr::Confidence::kDegraded);
  EXPECT_EQ(e.confidence_v, tram_dr::Confidence::kOk);
}

TEST(Ukf, DiagnosticAccelUsesLiveNotch) {
  tram_dr::Ukf ukf;
  tram_dr::Input u;
  u.notch = 1.0;
  u.notch_valid = true;
  const double r = 0.35;
  double omega[4] = {1.0 / r, 1.0 / r, 1.0 / r, 1.0 / r};
  auto e = ukf.predict_and_update(u, omega, 4, 0.02);
  EXPECT_GT(e.x.a_mps2, 0.5);
}

TEST(Ukf, SlipInflatesAndDegrades) {
  tram_dr::Ukf ukf;
  tram_dr::Input u;
  u.notch = 0.4;
  u.notch_valid = true;
  const double v = 4.0;
  const double r = 0.35;
  double omega[4] = {v / r, v / r, v / r, 3.0 * v / r};
  tram_dr::UkfEstimate e{};
  for (int k = 0; k < 20; ++k) {
    e = ukf.predict_and_update(u, omega, 4, 0.02);
  }
  EXPECT_GE(e.sca.n_inflated, 1);
  EXPECT_NE(e.confidence, tram_dr::Confidence::kUninitialized);
}

TEST(Ukf, SingleAxleOutlierStaysOk) {
  // Architecture: one lying encoder → SCA inflates R_ii, status stays OK.
  tram_dr::Ukf ukf;
  tram_dr::Input u;
  u.notch_valid = true;
  const double v = 5.0;
  const double r = 0.35;
  double roll[4] = {v / r, v / r, v / r, v / r};
  tram_dr::UkfEstimate e{};
  for (int k = 0; k < 40; ++k) {
    e = ukf.predict_and_update(u, roll, 4, 0.02);
  }
  double one_bad[4] = {v / r, v / r, v / r, 0.4 * v / r};
  for (int k = 0; k < 30; ++k) {
    e = ukf.predict_and_update(u, one_bad, 4, 0.02);
  }
  EXPECT_EQ(e.sca.n_inflated, 1);
  EXPECT_FALSE(e.sca.common_mode);
  EXPECT_FALSE(e.slip_latched);
  EXPECT_EQ(e.confidence, tram_dr::Confidence::kOk);
}

TEST(Ukf, TwoAxleOutliersDegrade) {
  tram_dr::Ukf ukf;
  tram_dr::Input u;
  u.notch_valid = true;
  const double v = 5.0;
  const double r = 0.35;
  double roll[4] = {v / r, v / r, v / r, v / r};
  tram_dr::UkfEstimate e{};
  for (int k = 0; k < 40; ++k) {
    e = ukf.predict_and_update(u, roll, 4, 0.02);
  }
  // Median of four even samples sits on the two healthy axles; the two
  // frozen/fast ones are both outliers (resid ≫ z σ).
  double two_bad[4] = {v / r, v / r, 0.3 * v / r, 2.5 * v / r};
  for (int k = 0; k < 20; ++k) {
    e = ukf.predict_and_update(u, two_bad, 4, 0.02);
  }
  EXPECT_GE(e.sca.n_inflated, 2);
  EXPECT_NE(e.confidence, tram_dr::Confidence::kOk);
}

TEST(Ukf, NisValidAfterWheelUpdate) {
  tram_dr::Ukf ukf;
  tram_dr::Input u;
  u.notch_valid = true;
  const double r = 0.35;
  double omega[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
  tram_dr::UkfEstimate e{};
  for (int k = 0; k < 50; ++k) {
    e = ukf.predict_and_update(u, omega, 4, 0.02);
  }
  EXPECT_TRUE(e.nis_valid);
  EXPECT_GE(e.nis, 0.0);
  EXPECT_TRUE(std::isfinite(e.nis));
  EXPECT_GE(e.nis_cusum, 0.0);
  EXPECT_GT(e.over_m, e.under_m);
  e = ukf.predict_and_update(u, nullptr, 0, 0.02);
  EXPECT_FALSE(e.nis_valid);
}

TEST(Ukf, FreezeDetectsStuckAxleWhenOthersMove) {
  // Var=0 on a constant-speed synth is normal; freeze only if another axle
  // is still moving. One frozen axle stays OK.
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
  EXPECT_EQ(e.n_frozen, 0);
  for (int k = 1; k <= 30; ++k) {
    const double v_live = v + 0.08 * static_cast<double>(k);
    omega[0] = v_live / r;
    omega[1] = v_live / r;
    omega[2] = v_live / r;
    omega[3] = v / r;
    e = ukf.predict_and_update(u, omega, 4, 0.02);
  }
  EXPECT_EQ(e.n_frozen, 1);
  EXPECT_EQ(e.confidence, tram_dr::Confidence::kOk);
}

TEST(Ukf, CreepageFloorIgnoresStopTransient) {
  // Relative κ is degenerate near standstill. Floor is 1 m/s (ukf.cpp).
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
  EXPECT_FALSE(e.sca.common_mode);
  EXPECT_FALSE(e.slip_latched);
}

TEST(Ukf, TwoFrozenAxlesDegrade) {
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
  for (int k = 1; k <= 30; ++k) {
    const double v_live = v + 0.08 * static_cast<double>(k);
    omega[0] = v_live / r;
    omega[1] = v_live / r;
    omega[2] = v / r;
    omega[3] = v / r;
    e = ukf.predict_and_update(u, omega, 4, 0.02);
  }
  EXPECT_EQ(e.n_frozen, 2);
  EXPECT_EQ(e.confidence, tram_dr::Confidence::kDegraded);
}

TEST(Ukf, TwoWheelDisagreementDegrades) {
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
  EXPECT_EQ(e.confidence, tram_dr::Confidence::kOk);
  double split[2] = {5.0 / r, 12.0 / r};
  for (int k = 0; k < 20; ++k) {
    e = ukf.predict_and_update(u, split, 2, 0.02);
  }
  EXPECT_NE(e.confidence, tram_dr::Confidence::kOk);
}

TEST(Ukf, NanWheelDoesNotPoisonState) {
  tram_dr::Ukf ukf;
  tram_dr::Input u;
  u.notch_valid = true;
  const double r = 0.35;
  double omega[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
  tram_dr::UkfEstimate e{};
  for (int k = 0; k < 20; ++k) {
    e = ukf.predict_and_update(u, omega, 4, 0.02);
  }
  omega[1] = std::nan("");
  e = ukf.predict_and_update(u, omega, 4, 0.02);
  EXPECT_TRUE(std::isfinite(e.x.v_mps));
  EXPECT_TRUE(std::isfinite(e.x.s_m));
  EXPECT_EQ(e.n_omega_used, 3);
}

TEST(Ukf, AllNanWheelsLost) {
  tram_dr::Ukf ukf;
  tram_dr::Input u;
  u.notch_valid = true;
  const double r = 0.35;
  double omega[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
  tram_dr::UkfEstimate e{};
  for (int k = 0; k < 20; ++k) {
    e = ukf.predict_and_update(u, omega, 4, 0.02);
  }
  EXPECT_EQ(e.confidence, tram_dr::Confidence::kOk);
  double dead[4] = {std::nan(""), std::nan(""), std::nan(""), std::nan("")};
  e = ukf.predict_and_update(u, dead, 4, 0.02);
  EXPECT_EQ(e.confidence, tram_dr::Confidence::kLost);
  EXPECT_EQ(e.n_omega_used, 0);
  EXPECT_GE(e.sca.n_inflated, 4);
  for (int k = 0; k < 100; ++k) {
    e = ukf.predict_and_update(u, dead, 4, 0.02);
  }
  EXPECT_EQ(e.confidence, tram_dr::Confidence::kLost);
}

TEST(Ukf, R0UncalibratedDegradesPath) {
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
  EXPECT_EQ(e.confidence, tram_dr::Confidence::kDegraded);
  EXPECT_EQ(e.confidence_s, tram_dr::Confidence::kDegraded);
}

TEST(Ukf, NotchInvalidEscalatesLost) {
  tram_dr::Ukf ukf;
  tram_dr::Input u;
  u.notch_valid = true;
  const double r = 0.35;
  double omega[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
  tram_dr::UkfEstimate e{};
  for (int k = 0; k < 10; ++k) {
    e = ukf.predict_and_update(u, omega, 4, 0.02);
  }
  u.notch_valid = false;
  for (int k = 0; k < 120; ++k) {
    e = ukf.predict_and_update(u, omega, 4, 0.02);
  }
  EXPECT_EQ(e.confidence, tram_dr::Confidence::kLost);
}

TEST(LinAlg, ProjectPdMakesCholWork) {
  double A[4] = {1.0, 2.0, 2.0, 1.0};
  ASSERT_TRUE(tram_dr::la::project_pd(A, 2));
  double L[4];
  ASSERT_TRUE(tram_dr::la::chol(A, L, 2, 0.0));
}

TEST(Ukf, CoastKappaDoesNotLatch) {
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
  EXPECT_FALSE(e.sca.common_mode);
  EXPECT_FALSE(e.slip_latched);
}

TEST(Ukf, CubatureStaysFinite) {
  tram_dr::UkfParams cfg;
  cfg.cubature = true;
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
  EXPECT_EQ(e.chol_fail, 0);
  EXPECT_TRUE(std::isfinite(e.x.v_mps));
}

TEST(Ukf, TractionRampSignDoesNotLatch) {
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
  // Model-ahead-of-wheels: body faster than axles under traction.
  double slow[4] = {2.0 / r, 2.0 / r, 2.0 / r, 2.0 / r};
  for (int k = 0; k < 20; ++k) {
    e = ukf.predict_and_update(u, slow, 4, 0.02);
  }
  EXPECT_FALSE(e.sca.common_mode);
  EXPECT_FALSE(e.slip_latched);
}

TEST(Ukf, ZuptNeedsHoldWindow) {
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
  EXPECT_NE(e.mode, tram_dr::Mode::kStandstill);
  for (int k = 0; k < 20; ++k) {
    e = ukf.predict_and_update(u, z, 4, 0.02);
  }
  EXPECT_EQ(e.mode, tram_dr::Mode::kStandstill);
}

TEST(Ukf, ProtectionLevelHasBiasTerm) {
  tram_dr::Ukf ukf;
  tram_dr::Input u;
  u.notch_valid = true;
  const double r = 0.35;
  double omega[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
  tram_dr::UkfEstimate e{};
  for (int k = 0; k < 30; ++k) {
    e = ukf.predict_and_update(u, omega, 4, 0.02);
  }
  EXPECT_GT(e.b_s_m, 0.0);
  EXPECT_LT(e.b_s_m, 1.0);
  EXPECT_NEAR(e.over_m, e.pl_s_m, 1e-12);
  EXPECT_GT(e.pl_s_m, e.under_m);
  EXPECT_LT(e.pl_s_m, e.al_s_m);
  EXPECT_FALSE(e.s_unbounded);
  EXPECT_TRUE(e.sca_current);
}

TEST(Ukf, AlphaAbStaysFinite) {
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
    EXPECT_EQ(e.chol_fail, 0);
    EXPECT_TRUE(std::isfinite(e.x.v_mps));
  }
}

TEST(Ukf, TrailerAxlesKeepOkUnderMotorSpin) {
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
  double spin[4] = {14.0 / r, v / r, v / r, v / r};
  for (int k = 0; k < 30; ++k) {
    e = ukf.predict_and_update(u, spin, 4, 0.02);
  }
  EXPECT_FALSE(e.slip_latched);
  EXPECT_EQ(e.confidence, tram_dr::Confidence::kOk);
}

TEST(Ukf, HuberSlideKeepsFiniteV) {
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
  EXPECT_TRUE(std::isfinite(e.x.v_mps));
  EXPECT_GT(e.x.v_mps, 2.0);
}

TEST(Ukf, MissedPathScalesWithSpeed) {
  tram_dr::Ukf ukf;
  tram_dr::Input u;
  u.notch_valid = true;
  const double r = 0.35;
  double omega[4] = {15.0 / r, 15.0 / r, 15.0 / r, 15.0 / r};
  tram_dr::UkfEstimate e{};
  for (int k = 0; k < 30; ++k) {
    e = ukf.predict_and_update(u, omega, 4, 0.02);
  }
  EXPECT_NEAR(e.b_s_m, 0.25 * e.x.v_mps * 0.2, 0.15);
}

TEST(Ukf, WspHeldSlideLatchesPathDisagree) {
  tram_dr::Ukf ukf;
  tram_dr::Input u;
  u.notch_valid = true;
  const double r = 0.35;
  double v_body = 15.0;
  tram_dr::UkfEstimate e{};
  for (int k = 0; k < 40; ++k) {
    const double w = v_body / r;
    double omega[4] = {w, w, w, w};
    e = ukf.predict_and_update(u, omega, 4, 0.02);
  }
  u.brake = 0.85;
  for (int k = 0; k < 700; ++k) {
    v_body = std::max(4.0, v_body - 0.45 * 0.02);
    const double w = (0.82 * v_body) / r;
    double omega[4] = {w, w, w, w};
    e = ukf.predict_and_update(u, omega, 4, 0.02);
  }
  EXPECT_LT(std::fabs(e.relative_wheel_slide), 0.25);
  EXPECT_NE(e.confidence, tram_dr::Confidence::kOk);
  EXPECT_TRUE(e.path_disagree_latched || e.confidence_s == tram_dr::Confidence::kDegraded);
}

TEST(Ukf, SetPlantRejectsLockedScaR0Mismatch) {
  tram_dr::Ukf ukf;
  tram_dr::ScaParams sca;
  ukf.set_sca(sca);
  tram_dr::PlantParams plant;
  plant.r0_m = 0.40;
  EXPECT_THROW(ukf.set_plant(plant), std::invalid_argument);
  plant.r0_m = 0.35;
  EXPECT_NO_THROW(ukf.set_plant(plant));
}

TEST(Ukf, SetPlantAloneStillSyncsScaR0) {
  tram_dr::Ukf ukf;
  tram_dr::PlantParams plant;
  plant.r0_m = 0.40;
  EXPECT_NO_THROW(ukf.set_plant(plant));
  tram_dr::ScaParams sca;
  sca.r0_m = 0.40;
  EXPECT_NO_THROW(ukf.set_sca(sca));
}

TEST(Ukf, OmegaOnlyZuptForcedAfterTwoSeconds) {
  tram_dr::Ukf ukf;
  tram_dr::Input u;
  u.notch_valid = true;
  const double r = 0.35;
  double roll[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
  tram_dr::UkfEstimate e{};
  for (int k = 0; k < 50; ++k) {
    e = ukf.predict_and_update(u, roll, 4, 0.02);
  }
  double z[4] = {0.0, 0.0, 0.0, 0.0};
  for (int k = 0; k < 16; ++k) {
    e = ukf.predict_and_update(u, z, 4, 0.02);
  }
  EXPECT_NE(e.mode, tram_dr::Mode::kStandstill);
  EXPECT_FALSE(e.zupt_forced);
  for (int k = 0; k < 120; ++k) {
    e = ukf.predict_and_update(u, z, 4, 0.02);
  }
  EXPECT_EQ(e.mode, tram_dr::Mode::kStandstill);
  EXPECT_TRUE(e.zupt_forced);
  EXPECT_NE(e.confidence, tram_dr::Confidence::kOk);
  EXPECT_NEAR(e.x.v_mps, 0.0, 1e-9);
}

TEST(Ukf, PredictOnlyDoesNotAdvertiseLiveSca) {
  tram_dr::Ukf ukf;
  tram_dr::Input u;
  u.notch_valid = true;
  const double r = 0.35;
  double roll[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
  tram_dr::UkfEstimate e{};
  for (int k = 0; k < 40; ++k) {
    e = ukf.predict_and_update(u, roll, 4, 0.02);
  }
  EXPECT_TRUE(e.sca_current);
  e = ukf.predict_and_update(u, nullptr, 0, 0.02);
  EXPECT_FALSE(e.sca_current);
  EXPECT_EQ(e.n_omega_used, 0);
}

TEST(Ukf, SlipLatchPublishesUnboundedPath) {
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
  double spin[4] = {12.0 / r, 12.0 / r, 12.0 / r, 12.0 / r};
  for (int k = 0; k < 15; ++k) {
    e = ukf.predict_and_update(u, spin, 4, 0.02);
  }
  EXPECT_TRUE(e.s_unbounded);
  EXPECT_EQ(e.over_m, 0.0);
  EXPECT_EQ(e.b_s_m, 0.0);
  EXPECT_FALSE(std::isfinite(e.pl_s_m));
}

TEST(Ukf, CoastingLockDegradesWithoutZupt) {
  tram_dr::Ukf ukf;
  tram_dr::Input u;
  u.notch_valid = true;
  const double r = 0.35;
  double roll[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
  tram_dr::UkfEstimate e{};
  for (int k = 0; k < 50; ++k) {
    e = ukf.predict_and_update(u, roll, 4, 0.02);
  }
  double z[4] = {0.0, 0.0, 0.0, 0.0};
  for (int k = 0; k < 16; ++k) {
    e = ukf.predict_and_update(u, z, 4, 0.02);
  }
  EXPECT_NE(e.mode, tram_dr::Mode::kStandstill);
  EXPECT_GT(e.x.v_mps, 1.0);
  EXPECT_NE(e.confidence, tram_dr::Confidence::kOk);
}

TEST(Ukf, TruncatedPacketCannotProveStandstill) {
  tram_dr::Ukf ukf;
  tram_dr::Input u;
  u.notch_valid = true;
  u.brake = 0.3;
  double one[1] = {0.0};
  tram_dr::UkfEstimate e{};
  for (int k = 0; k < 150; ++k) {
    e = ukf.predict_and_update(u, one, 1, 0.02);
  }
  EXPECT_NE(e.mode, tram_dr::Mode::kStandstill);
}

TEST(Ukf, IncompletePacketDegrades) {
  tram_dr::Ukf ukf;
  tram_dr::Input u;
  u.notch_valid = true;
  const double r = 0.35;
  double roll[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
  tram_dr::UkfEstimate e{};
  for (int k = 0; k < 40; ++k) {
    e = ukf.predict_and_update(u, roll, 4, 0.02);
  }
  EXPECT_EQ(e.confidence, tram_dr::Confidence::kOk);
  double one[1] = {5.0 / r};
  e = ukf.predict_and_update(u, one, 1, 0.02);
  EXPECT_NE(e.confidence, tram_dr::Confidence::kOk);
}

TEST(Ukf, InvalidNotchDoesNotKeepTraction) {
  tram_dr::Ukf live;
  tram_dr::Ukf stale;
  tram_dr::Input u;
  u.notch = 0.8;
  u.notch_valid = true;
  const double r = 0.35;
  double roll[4] = {5.0 / r, 5.0 / r, 5.0 / r, 5.0 / r};
  for (int k = 0; k < 40; ++k) {
    live.predict_and_update(u, roll, 4, 0.02);
    stale.predict_and_update(u, roll, 4, 0.02);
  }
  tram_dr::Input dead = u;
  dead.notch_valid = false;
  tram_dr::UkfEstimate ea{};
  tram_dr::UkfEstimate eb{};
  for (int k = 0; k < 80; ++k) {
    ea = live.predict_and_update(u, nullptr, 0, 0.02);
    eb = stale.predict_and_update(dead, nullptr, 0, 0.02);
  }
  EXPECT_LT(eb.x.v_mps, ea.x.v_mps - 0.2);
}

