#include "tram_dr_localization/types.hpp"
#include <gtest/gtest.h>

TEST(Types, DefaultMassIsCombinoTare) {
  tram_dr::State x;
  EXPECT_DOUBLE_EQ(x.m_eff_kg, 28000.0);
}

TEST(Types, PackedLayout) {
  tram_dr::State x;
  x.s_m = 3.0;
  x.v_mps = 2.0;
  double v[tram_dr::kStateDim];
  tram_dr::pack_state(x, v);
  const tram_dr::State y = tram_dr::unpack_state(v);
  EXPECT_DOUBLE_EQ(y.s_m, 3.0);
  EXPECT_DOUBLE_EQ(y.v_mps, 2.0);
}

TEST(Types, ReparamRoundtrip) {
  double x[tram_dr::kStateDim]{};
  x[tram_dr::kMass] = 28000.0;
  x[tram_dr::kMu] = 0.35;
  for (int i = 0; i < tram_dr::kNWheels; ++i) {
    x[tram_dr::kD0 + i] = 1.0;
  }
  x[tram_dr::kKtrac] = 1.0;
  tram_dr::phys_to_xi(x);
  tram_dr::xi_to_phys(x);
  EXPECT_NEAR(x[tram_dr::kMass], 28000.0, 1e-6);
  EXPECT_NEAR(x[tram_dr::kKtrac], 1.0, 1e-12);
  EXPECT_NEAR(x[tram_dr::kMu], 0.35, 1e-9);
  EXPECT_NEAR(x[tram_dr::kD0], 1.0, 1e-12);
}

TEST(Types, XiToPhysClipsKtracAndD) {
  double x[tram_dr::kStateDim]{};
  x[tram_dr::kMass] = 28000.0;
  x[tram_dr::kKtrac] = 10.0;
  x[tram_dr::kMu] = 0.35;
  for (int i = 0; i < tram_dr::kNWheels; ++i) {
    x[tram_dr::kD0 + i] = 2.0;
  }
  tram_dr::phys_to_xi(x);
  tram_dr::xi_to_phys(x);
  EXPECT_NEAR(x[tram_dr::kKtrac], tram_dr::kKtracMax, 1e-12);
  EXPECT_NEAR(x[tram_dr::kD0], tram_dr::kDMax, 1e-12);

  x[tram_dr::kMass] = 28000.0;
  x[tram_dr::kKtrac] = 0.1;
  x[tram_dr::kMu] = 0.35;
  for (int i = 0; i < tram_dr::kNWheels; ++i) {
    x[tram_dr::kD0 + i] = 0.5;
  }
  tram_dr::phys_to_xi(x);
  tram_dr::xi_to_phys(x);
  EXPECT_NEAR(x[tram_dr::kKtrac], tram_dr::kKtracMin, 1e-12);
  EXPECT_NEAR(x[tram_dr::kD0], tram_dr::kDMin, 1e-12);
}

TEST(Types, CombinedNotchMapsNegativeToBrake) {
  tram_dr::Input u;
  u.notch = -0.4;
  const auto m = tram_dr::apply_brake_source(u, tram_dr::BrakeSource::kNotch);
  EXPECT_NEAR(m.notch, 0.0, 1e-12);
  EXPECT_NEAR(m.brake, 0.4, 1e-12);
  EXPECT_TRUE(m.brake_valid);
  const auto none = tram_dr::apply_brake_source(u, tram_dr::BrakeSource::kNone);
  EXPECT_TRUE(none.brake_valid);
  EXPECT_NEAR(none.brake, 0.0, 1e-12);
}
