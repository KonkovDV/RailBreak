#include "tram_dr_localization/sca.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <vector>

TEST(Sca, ConsensusOnAgreeingWheels) {
  const double w[] = {10.0, 10.05, 9.95, 10.02};
  const double d[] = {1.0, 1.0, 1.0, 1.0};
  const auto r = tram_dr::sca_analyze(w, 4, d, tram_dr::ScaParams{});
  EXPECT_EQ(r.n_inflated, 0);
  EXPECT_NEAR(r.v_consensus_mps, 10.0 * 0.35, 0.05);
}

TEST(Sca, InflatesOutlier) {
  const double w[] = {10.0, 10.0, 10.0, 40.0};
  const double d[] = {1.0, 1.0, 1.0, 1.0};
  const auto r = tram_dr::sca_analyze(w, 4, d, tram_dr::ScaParams{});
  EXPECT_GE(r.n_inflated, 1);
  EXPECT_GT(r.inflate[3], r.inflate[0]);
  EXPECT_GT(r.r_omega[3], r.r_omega[0]);
}

TEST(Sca, PairLrIgnoresStreetCurveSplit) {
  const double r = 0.35;
  const double v = 15.0;
  const double delta = 0.5 * 1.524 / 25.0;
  const double w0 = (v / r) * (1.0 - delta);
  const double w1 = (v / r) * (1.0 + delta);
  const double w[] = {w0, w1, w0, w1};
  const double d[] = {1.0, 1.0, 1.0, 1.0};
  tram_dr::ScaParams on;
  on.pair_lr = true;
  const auto paired = tram_dr::sca_analyze(w, 4, d, on);
  EXPECT_EQ(paired.n_inflated, 0);

  tram_dr::ScaParams off;
  off.pair_lr = false;
  const auto raw = tram_dr::sca_analyze(w, 4, d, off);
  EXPECT_GE(raw.n_inflated, 1);
}

TEST(Sca, NanChannelInflatesOnlyThatAxle) {
  const double w[] = {10.0, std::nan(""), 10.0, 10.02};
  const double d[] = {1.0, 1.0, 1.0, 1.0};
  const auto r = tram_dr::sca_analyze(w, 4, d, tram_dr::ScaParams{});
  EXPECT_EQ(r.n_inflated, 1);
  EXPECT_GT(r.inflate[1], r.inflate[0]);
}

TEST(Sca, AllNanInflatesEveryAxle) {
  const double w[] = {std::nan(""), std::nan(""), std::nan(""), std::nan("")};
  const double d[] = {1.0, 1.0, 1.0, 1.0};
  const auto r = tram_dr::sca_analyze(w, 4, d, tram_dr::ScaParams{});
  EXPECT_EQ(r.n_inflated, 4);
}

TEST(Sca, VectorApiFillsR) {
  std::vector<double> omega{8.0, 8.1, 8.0, 8.05};
  std::vector<double> rdiag;
  tram_dr::sca_inflate(omega, rdiag);
  ASSERT_EQ(rdiag.size(), 4u);
  EXPECT_GT(rdiag[0], 0.0);
}

TEST(Sca, TrailerConsensusIgnoresSpinningMotor) {
  tram_dr::ScaParams p;
  p.traction = true;
  p.axle_role = {0, 1, 1, 1, 0, 0};
  const double w[] = {40.0, 10.0, 10.0, 10.05};
  const double d[] = {1.0, 1.0, 1.0, 1.0};
  const auto r = tram_dr::sca_analyze(w, 4, d, p);
  EXPECT_TRUE(r.used_trailer_consensus);
  EXPECT_GE(r.n_inflated_motor, 1);
  EXPECT_EQ(r.n_inflated_trailer, 0);
  EXPECT_NEAR(r.v_consensus_mps, 10.0 * 0.35, 0.2);
}

TEST(Sca, SixAxlePairLr) {
  const double r0 = 0.35;
  const double v = 12.0;
  const double delta = 0.5 * 1.524 / 25.0;
  const double w0 = (v / r0) * (1.0 - delta);
  const double w1 = (v / r0) * (1.0 + delta);
  const double w[] = {w0, w1, w0, w1, w0, w1};
  const double d[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  tram_dr::ScaParams on;
  on.pair_lr = true;
  const auto paired = tram_dr::sca_analyze(w, 6, d, on);
  EXPECT_EQ(paired.n_inflated, 0);
}

TEST(Sca, ExtremalTractionDropsFastAxle) {
  tram_dr::ScaParams p;
  p.pair_lr = false;
  p.traction = true;
  p.consensus = 0;
  const double w[] = {10.0, 10.1, 10.05, 18.0};
  const double d[] = {1.0, 1.0, 1.0, 1.0};
  const auto med = tram_dr::sca_analyze(w, 4, d, p);
  p.consensus = 1;
  const auto ext = tram_dr::sca_analyze(w, 4, d, p);
  EXPECT_LT(ext.v_consensus_mps, med.v_consensus_mps);
}
