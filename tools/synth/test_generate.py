"""Generator smoke tests. No UKF import."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from generate import SCENARIOS, simulate  # noqa: E402


class GenerateTests(unittest.TestCase):
    def test_all_scenarios_move_or_dwell(self) -> None:
        for name in SCENARIOS:
            rows = simulate(name, duration_s=4.0)
            self.assertGreater(len(rows), 100)
            self.assertIn("w0", rows[0])
            self.assertEqual(rows[1]["t_s"] - rows[0]["t_s"], 0.02)

    def test_seed_repeatable(self) -> None:
        a = simulate("slip_accel", duration_s=2.0)
        b = simulate("slip_accel", duration_s=2.0)
        self.assertEqual(a[-1]["gt_s"], b[-1]["gt_s"])
        self.assertEqual(a[-1]["w3"], b[-1]["w3"])

    def test_axle_fault_freezes(self) -> None:
        rows = simulate("axle_fault", duration_s=12.0)
        late = [r["w3"] for r in rows if r["t_s"] > 9.0]
        self.assertGreater(len(late), 10)
        self.assertAlmostEqual(late[0], late[-1], places=6)

    def test_axle_fault_frozen_lags_good_wheels(self) -> None:
        rows = simulate("axle_fault", duration_s=16.0)
        late = [r for r in rows if r["t_s"] > 12.0]
        self.assertGreater(len(late), 10)
        r = late[len(late) // 2]
        v_frozen = 0.35 * r["w3"]
        v_good = 0.35 * (r["w0"] + r["w1"] + r["w2"]) / 3.0
        self.assertGreater(v_good, v_frozen + 0.5)

    def test_noise_repeatable_physics_clean(self) -> None:
        clean = simulate("coast_no_wire", duration_s=2.0)
        a = simulate("coast_no_wire", duration_s=2.0, noise_sigma=0.05)
        b = simulate("coast_no_wire", duration_s=2.0, noise_sigma=0.05)
        self.assertEqual(a[-1]["w0"], b[-1]["w0"])
        self.assertNotEqual(a[-1]["w0"], clean[-1]["w0"])
        self.assertEqual(a[-1]["gt_s"], clean[-1]["gt_s"])
        self.assertEqual(a[-1]["gt_v"], clean[-1]["gt_v"])

    def test_quantize_hits_measured_omega_only(self) -> None:
        clean = simulate("coast_no_wire", duration_s=2.0)
        q = simulate("coast_no_wire", duration_s=2.0, quantize=256)
        self.assertEqual(q[-1]["gt_v"], clean[-1]["gt_v"])
        step = 2.0 * math.pi / 256.0
        self.assertAlmostEqual(q[-1]["w0"] / step, round(q[-1]["w0"] / step), places=6)

    def test_slip_accel_wheels_outrun_body(self) -> None:
        rows = simulate("slip_accel", duration_s=12.0)
        late = [r for r in rows if r["t_s"] > 8.0]
        self.assertGreater(len(late), 10)
        r = late[len(late) // 2]
        v_wh = 0.35 * (r["w0"] + r["w1"] + r["w2"] + r["w3"]) / 4.0
        self.assertGreater(v_wh, r["gt_v"] + 0.3)

    def test_slide_brake_wheels_lag_body(self) -> None:
        rows = simulate("slide_brake", duration_s=14.0)
        ratios = []
        for r in rows:
            if r["t_s"] <= 8.0 or r["gt_v"] <= 0.4:
                continue
            v_wh = 0.35 * (abs(r["w0"]) + abs(r["w1"]) + abs(r["w2"]) + abs(r["w3"])) / 4.0
            ratios.append(v_wh / r["gt_v"])
        self.assertGreater(len(ratios), 20)
        self.assertLess(min(ratios), 0.4)

    def test_slip_naive_path_overshoots_body(self) -> None:
        rows = simulate("slip_accel", duration_s=12.0)
        s_naive = 0.0
        for r in rows:
            s_naive += 0.35 * (r["w0"] + r["w1"] + r["w2"] + r["w3"]) / 4.0 * 0.02
        self.assertGreater(s_naive, rows[-1]["gt_s"] + 2.0)

    def test_tight_curve_splits_left_right(self) -> None:
        rows = simulate("tight_curve", duration_s=4.0)
        r = rows[-1]
        self.assertLess(r["w0"], r["w1"])
        self.assertAlmostEqual(r["w0"], r["w2"], places=6)
        self.assertAlmostEqual(r["w1"], r["w3"], places=6)

    def test_mismatch_r0_scales_omega(self) -> None:
        rows = simulate("mismatch_r0", duration_s=4.0)
        r = rows[-1]
        v_wh = 0.35 * (r["w0"] + r["w1"] + r["w2"] + r["w3"]) / 4.0
        self.assertAlmostEqual(v_wh * 0.88, r["gt_v"], delta=0.4)

    def test_all_encoders_dead_writes_nan(self) -> None:
        rows = simulate("all_encoders_dead", duration_s=12.0)
        early = [r for r in rows if r["t_s"] < 7.0]
        late = [r for r in rows if r["t_s"] > 8.5]
        self.assertTrue(all(math.isfinite(r["w0"]) for r in early))
        self.assertTrue(all(math.isnan(r["w0"]) for r in late))

    def test_mismatch_jerk_ramps(self) -> None:
        rows = simulate("mismatch_jerk", duration_s=1.0)
        at = [r for r in rows if 0.38 < r["t_s"] < 0.42]
        self.assertGreater(len(at), 0)
        self.assertLess(at[0]["gt_v"], 0.25)

    def test_six_axle_writes_w5(self) -> None:
        rows = simulate("six_axle", duration_s=2.0)
        self.assertIn("w5", rows[0])
        self.assertNotIn("w6", rows[0])
        self.assertGreater(rows[-1]["gt_s"], 0.0)

    def test_slide_brake_wsp_cycles_not_stuck(self) -> None:
        rows = simulate("slide_brake", duration_s=14.0)
        late = [r for r in rows if r["t_s"] > 8.5 and r["gt_v"] > 0.5]
        self.assertGreater(len(late), 20)
        max_zero = 0
        cur = 0
        for r in late:
            wmax = max(abs(r["w0"]), abs(r["w1"]), abs(r["w2"]), abs(r["w3"]))
            if wmax < 0.05:
                cur += 1
                max_zero = max(max_zero, cur)
            else:
                cur = 0
        self.assertLess(max_zero * 0.02, 0.55)

    def test_snow_ice_mu_steps_with_s(self) -> None:
        rows = simulate("snow_ice", duration_s=8.0)
        self.assertGreater(rows[-1]["gt_s"], 1.0)

    def test_wet_clean_is_not_contaminated(self) -> None:
        wet = simulate("wet_clean", duration_s=4.0)
        r = wet[-1]
        v_wh = 0.35 * (r["w0"] + r["w1"] + r["w2"] + r["w3"]) / 4.0
        self.assertAlmostEqual(v_wh, r["gt_v"], delta=0.6)

    def test_strogino_grade_ramp_shape(self) -> None:
        from generate import STROGINO_I_PEAK, STROGINO_I_STEEP, strogino_grade_mag

        self.assertEqual(strogino_grade_mag(0.0), 0.0)
        self.assertAlmostEqual(strogino_grade_mag(60.0 + 50.0), 0.5 * STROGINO_I_PEAK, places=6)
        self.assertAlmostEqual(strogino_grade_mag(250.0), STROGINO_I_PEAK, places=6)
        self.assertAlmostEqual(strogino_grade_mag(60.0 + 100.0 + 250.0 + 50.0), 0.5 * STROGINO_I_PEAK, places=6)
        self.assertEqual(strogino_grade_mag(10_000.0), 0.0)
        self.assertAlmostEqual(strogino_grade_mag(250.0, STROGINO_I_STEEP), STROGINO_I_STEEP, places=6)

    def test_coast_grade_route10_faster_than_flat_coast(self) -> None:
        flat = simulate("coast_no_wire", duration_s=30.0)
        grade = simulate("coast_grade_route10", duration_s=30.0)
        self.assertGreater(grade[-1]["gt_v"], flat[-1]["gt_v"] + 0.5)
        self.assertGreater(grade[-1]["gt_s"], flat[-1]["gt_s"])

    def test_grade_traction_route10_wheels_can_outrun(self) -> None:
        rows = simulate("grade_traction_route10", duration_s=16.0)
        late = [r for r in rows if r["t_s"] > 10.0]
        self.assertGreater(len(late), 10)
        r = late[len(late) // 2]
        v_wh = 0.35 * (r["w0"] + r["w1"] + r["w2"] + r["w3"]) / 4.0
        self.assertGreater(v_wh, r["gt_v"] + 0.2)

    def test_route10_f_bias_sign_matches_direction(self) -> None:
        from generate import scenario_cmd

        down = scenario_cmd("coast_grade_route10", 12.0, 10.0, 250.0)
        self.assertLess(down.f_bias_n, -5000.0)
        steep = scenario_cmd("slide_on_grade_route10_steep", 12.0, 10.0, 250.0)
        self.assertLess(steep.f_bias_n, down.f_bias_n)
        up = scenario_cmd("grade_traction_route10", 12.0, 10.0, 250.0)
        self.assertGreater(up.f_bias_n, 5000.0)


if __name__ == "__main__":
    unittest.main()
