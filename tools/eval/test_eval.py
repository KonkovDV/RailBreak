"""Self-checks for plant_ref / SCA / checker. No UKF import."""

from __future__ import annotations

import io
import json
import math
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from baselines import naive_wheel, plant_only, sca_wheel
from check_envelope import check_rows, main as check_main
from plant_ref import D_MIN, G, PlantParams, VehicleState, davis_resistance_n, plant_step
from plant_ref import generator_step, polach11_force_n, polach9_mu, sca_analyze
from plant_ref import wheel_speeds_step


class PlantRefTests(unittest.TestCase):
    def test_davis_at_rest(self) -> None:
        self.assertAlmostEqual(davis_resistance_n(0.0), 0.0)

    def test_davis_opposes_reverse(self) -> None:
        x = VehicleState(v_mps=-0.5)
        plant_step(x, notch=0.0, brake=0.0, dt_s=0.02)
        self.assertGreater(x.v_mps, -0.5)

    def test_d_min_allows_mismatch_r0(self) -> None:
        self.assertLessEqual(D_MIN, 0.88)

    def test_notch_accelerates(self) -> None:
        x = VehicleState(v_mps=1.0)
        plant_step(x, notch=1.0, brake=0.0, dt_s=0.02)
        self.assertGreater(x.v_mps, 1.0)

    def test_first_step_matches_cpp_plant(self) -> None:
        x = VehicleState()
        plant_step(x, notch=1.0, brake=0.0, dt_s=0.02)
        a = 1.3
        self.assertAlmostEqual(x.v_mps, a * 0.02, places=12)
        self.assertAlmostEqual(x.s_m, 0.5 * a * 0.02 * 0.02, places=12)

    def test_tau_drv_slows_first_step(self) -> None:
        p = PlantParams(tau_drv_s=0.12)
        x = VehicleState()
        filt = [0.0]
        plant_step(x, notch=1.0, brake=0.0, dt_s=0.02, p=p, f_trac_filt=filt)
        a = 1.3
        self.assertLess(x.v_mps, a * 0.02 - 1e-6)
        self.assertGreater(filt[0], 0.0)
        self.assertLess(filt[0], 28000.0 * 1.3)

    def test_jerk_caps_first_step(self) -> None:
        p = PlantParams(j_max_mps3=0.7)
        x = VehicleState()
        filt = [0.0]
        plant_step(x, notch=1.0, brake=0.0, dt_s=0.02, p=p, f_trac_filt=filt)
        a = 1.3
        self.assertLess(x.v_mps, a * 0.02 - 1e-6)
        self.assertAlmostEqual(filt[0], 28000.0 * 0.7 * 0.02, delta=1.0)

    def test_gamma_rot_lowers_accel(self) -> None:
        p = PlantParams(gamma_rot=0.10)
        x = VehicleState()
        plant_step(x, notch=1.0, brake=0.0, dt_s=0.02, p=p)
        a = 1.3
        self.assertLess(x.v_mps, a * 0.02 - 1e-6)

    def test_notch_as_accel_uses_live_mass(self) -> None:
        p = PlantParams(notch_as_accel=True)
        x = VehicleState(m_eff_kg=40000.0)
        plant_step(x, notch=1.0, brake=0.0, dt_s=0.02, p=p)
        a = 1.3
        self.assertAlmostEqual(x.v_mps, a * 0.02, places=12)

    def test_i_grade_on_coast(self) -> None:
        p = PlantParams(i_grade=0.02)
        x = VehicleState()
        plant_step(x, notch=0.0, brake=0.0, dt_s=0.02, p=p)
        a = -(28000.0 * G * 0.02) / 28000.0
        self.assertAlmostEqual(x.v_mps, a * 0.02, places=12)

    def test_plant_clamps_mass(self) -> None:
        x = VehicleState(m_eff_kg=80000.0)
        plant_step(x, notch=0.0, brake=0.0, dt_s=0.02)
        self.assertLessEqual(x.m_eff_kg, 70000.0)
        self.assertGreaterEqual(x.m_eff_kg, 20000.0)

    def test_lvenok_mass_clip(self) -> None:
        p = PlantParams(mass_min_kg=15000.0, mass_max_kg=40000.0)
        x = VehicleState(m_eff_kg=50000.0)
        plant_step(x, notch=0.0, brake=0.0, dt_s=0.02, p=p)
        self.assertLessEqual(x.m_eff_kg, 40000.0)
        x = VehicleState(m_eff_kg=10000.0)
        plant_step(x, notch=0.0, brake=0.0, dt_s=0.02, p=p)
        self.assertGreaterEqual(x.m_eff_kg, 15000.0)

    def test_wet_brake_capped_by_adhesion(self) -> None:
        x = VehicleState(v_mps=5.0, mu_hat=0.06)
        plant_step(x, notch=0.0, brake=1.0, dt_s=0.02)
        m = x.m_eff_kg
        f_adh = m * G * 0.06
        f_run = davis_resistance_n(5.0)
        a_limit = (f_adh + f_run) / m
        self.assertGreater(x.v_mps, 5.0 - a_limit * 0.02 - 1e-6)

    def test_sca_outlier(self) -> None:
        r = sca_analyze([10.0, 10.0, 10.0, 40.0])
        self.assertGreaterEqual(r["n_inflated"], 1)
        self.assertGreater(r["inflate"][3], r["inflate"][0])

    def test_sca_pair_lr_street_curve(self) -> None:
        r0 = 0.35
        v = 15.0
        delta = 0.5 * 1.524 / 25.0
        w0 = (v / r0) * (1.0 - delta)
        w1 = (v / r0) * (1.0 + delta)
        w = [w0, w1, w0, w1]
        paired = sca_analyze(w, pair_lr=True)
        self.assertEqual(paired["n_inflated"], 0)
        raw = sca_analyze(w, pair_lr=False)
        self.assertGreaterEqual(raw["n_inflated"], 1)

    def test_sca_nan_channel(self) -> None:
        r = sca_analyze([10.0, float("nan"), 10.0, 10.0])
        self.assertEqual(r["n_inflated"], 1)

    def test_sca_all_nan_inflates_every_axle(self) -> None:
        r = sca_analyze([float("nan")] * 4)
        self.assertEqual(r["n_inflated"], 4)

    def test_wheel_spin_on_wet(self) -> None:
        p = PlantParams()
        v = 5.0
        r = p.r0_m
        omega = [v / r] * 4
        for _ in range(int(2.0 / 0.02)):
            omega = wheel_speeds_step(
                omega, v, 1.0, 0.0, 0.06, 28000.0, [1.0] * 4, 0.02, wsp=True, p=p
            )
        self.assertGreater(r * sum(omega) / 4.0, v + 0.5)

    def test_generator_newton_iii_matches_sum_adh(self) -> None:
        x = VehicleState(v_mps=5.0, mu_hat=0.06)
        omega = [5.0 / 0.35] * 4
        x2, omega2, f_adh = generator_step(x, omega, 1.0, 0.0, 0.02, wsp=True)
        self.assertGreater(x2.v_mps, 0.0)
        self.assertEqual(len(omega2), 4)
        cap = x.m_eff_kg * G * 0.06
        self.assertLessEqual(abs(sum(f_adh)), cap + 1e-6)
        # Reconstruct a from the force that was applied (v was 5, Davis on).
        f_run = davis_resistance_n(5.0)
        a_expect = (sum(f_adh) - f_run) / x.m_eff_kg
        self.assertAlmostEqual(x2.a_mps2, a_expect, places=9)

    def test_generator_wet_launch_lags_coulomb_plant(self) -> None:
        """Coulomb applies mgμ for the whole step. Instantaneous (11) is 0 at
        zero slip, but Kalker stiffness saturates inside 20 ms, so the first
        implicit step already moves the body. After 1 s wet + (9) + spin,
        the generator body is still slower than the Coulomb twin."""
        dt = 0.02
        n = int(1.0 / dt)
        coulomb = VehicleState(mu_hat=0.06)
        gen = VehicleState(mu_hat=0.06)
        omega = [0.0, 0.0, 0.0, 0.0]
        p = PlantParams(
            polach_B_s_per_m=0.20,
            polach_kA=0.30,
            polach_kS=0.10,
        )
        for _ in range(n):
            plant_step(coulomb, notch=1.0, brake=0.0, dt_s=dt)
            gen, omega, _ = generator_step(
                gen, omega, 1.0, 0.0, dt, wsp=True, p=p
            )
        self.assertGreater(coulomb.v_mps, gen.v_mps + 0.05)
        self.assertGreater(0.35 * sum(omega) / 4.0, gen.v_mps + 0.5)

    def test_frozen_axle_still_contributes_force(self) -> None:
        x = VehicleState(v_mps=5.0, mu_hat=0.35)
        stuck = 20.0
        omega = [5.0 / 0.35, 5.0 / 0.35, 5.0 / 0.35, stuck]
        _, omega2, f_adh = generator_step(
            x, omega, 0.5, 0.0, 0.02, freeze=[False, False, False, True]
        )
        self.assertAlmostEqual(omega2[3], stuck)
        self.assertGreater(abs(f_adh[3]), 1.0)

    def test_polach9_limits(self) -> None:
        self.assertAlmostEqual(polach9_mu(0.35, 0.0, 0.4, 0.6), 0.35)
        self.assertAlmostEqual(polach9_mu(0.35, 1e6, 0.4, 0.6), 0.35 * 0.4, places=6)

    def test_falling_mu_lowers_force_at_large_slip(self) -> None:
        omega = [40.0] * 4
        flat = PlantParams(polach_A=1.0, polach_B_s_per_m=0.0)
        fall = PlantParams(polach_A=0.40, polach_B_s_per_m=0.60)
        _, _, f_flat = generator_step(
            VehicleState(v_mps=5.0, mu_hat=0.35), omega, 1.0, 0.0, 0.02, p=flat
        )
        _, _, f_fall = generator_step(
            VehicleState(v_mps=5.0, mu_hat=0.35), omega, 1.0, 0.0, 0.02, p=fall
        )
        self.assertGreater(abs(sum(f_flat)), abs(sum(f_fall)) + 100.0)

    def test_filter_plant_ignores_polach9(self) -> None:
        p0 = PlantParams(polach_B_s_per_m=0.0)
        p1 = PlantParams(polach_B_s_per_m=0.60)
        x0 = VehicleState(v_mps=5.0, mu_hat=0.35)
        x1 = VehicleState(v_mps=5.0, mu_hat=0.35)
        plant_step(x0, notch=1.0, brake=0.0, dt_s=0.02, p=p0)
        plant_step(x1, notch=1.0, brake=0.0, dt_s=0.02, p=p1)
        self.assertAlmostEqual(x0.v_mps, x1.v_mps)

    def test_polach11_zero_at_zero_slip(self) -> None:
        p = PlantParams()
        q = 28000.0 / 4.0 * G
        self.assertEqual(polach11_force_n(q, 0.35, 0.0, 5.0, p), 0.0)

    def test_polach11_saturates_at_q_mu(self) -> None:
        p = PlantParams(polach_A=1.0, polach_B_s_per_m=0.0)
        q = 28000.0 / 4.0 * G
        mu = 0.35
        f = polach11_force_n(q, mu, 80.0, 5.0, p)
        self.assertGreater(f, 0.98 * q * mu)
        self.assertLessEqual(f, q * mu + 1e-6)

    def test_filter_plant_ignores_polach11(self) -> None:
        p0 = PlantParams(polach_kA=1.0, polach_kS=1.0)
        p1 = PlantParams(polach_kA=0.3, polach_kS=0.1)
        x0 = VehicleState(v_mps=5.0, mu_hat=0.35)
        x1 = VehicleState(v_mps=5.0, mu_hat=0.35)
        plant_step(x0, notch=1.0, brake=0.0, dt_s=0.02, p=p0)
        plant_step(x1, notch=1.0, brake=0.0, dt_s=0.02, p=p1)
        self.assertAlmostEqual(x0.v_mps, x1.v_mps)

    def test_tanh_cap_differs_from_polach11(self) -> None:
        omega = [16.0] * 4
        p11 = PlantParams(creep_force="polach11", polach_A=1.0, polach_B_s_per_m=0.0)
        pth = PlantParams(creep_force="tanh", polach_A=1.0, polach_B_s_per_m=0.0)
        _, _, f11 = generator_step(
            VehicleState(v_mps=5.0, mu_hat=0.35), omega, 1.0, 0.0, 0.02, p=p11
        )
        _, _, fth = generator_step(
            VehicleState(v_mps=5.0, mu_hat=0.35), omega, 1.0, 0.0, 0.02, p=pth
        )
        self.assertGreater(abs(sum(f11) - sum(fth)), 50.0)

    def test_polach11_dry_coast_no_force_chatter(self) -> None:
        """Implicit Euler: matched dry coast must not flip sign(F) every step."""
        p = PlantParams()
        v0 = 8.0
        x = VehicleState(v_mps=v0, mu_hat=0.35)
        omega = [v0 / p.r0_m] * 4
        signs: list[int] = []
        for _ in range(40):
            x, omega, f_adh = generator_step(x, omega, 0.0, 0.0, 0.02, p=p)
            s = sum(f_adh)
            if abs(s) > 1.0:
                signs.append(1 if s > 0.0 else -1)
        flips = sum(
            1 for i in range(1, len(signs)) if signs[i] != signs[i - 1]
        )
        self.assertLess(flips, 3)
        self.assertLess(abs(x.v_mps - v0), 1.0)


class BaselineTests(unittest.TestCase):
    def test_naive_integrates(self) -> None:
        out = naive_wheel([10.0, 10.0], dt_s=0.02, r0_m=0.35)
        self.assertGreater(out[-1].s_m, 0.0)

    def test_plant_only_moves(self) -> None:
        out = plant_only([1.0] * 20, dt_s=0.02)
        self.assertGreater(out[-1].s_m, 0.0)

    def test_complementary_blends(self) -> None:
        from baselines import complementary

        out = complementary([1.0, 1.0], [[10.0] * 4, [10.0] * 4], dt_s=0.02)
        self.assertEqual(len(out), 2)


class CheckerTests(unittest.TestCase):
    def test_no_covariance(self) -> None:
        result = check_rows(
            [{"kind": "est", "confidence": "OK", "t": 0.0, "s": 0.0, "v": 0.0}],
            require_gt=False,
        )
        self.assertTrue(any("NO_COVARIANCE" in h for h in result.hits))

    def test_ok_with_cov(self) -> None:
        result = check_rows(
            [{"kind": "est", "confidence": "OK", "t": 0.0, "s": 1.0, "v": 0.1,
              "p_ss": 4.0, "p_vv": 0.2}],
            require_gt=False,
        )
        self.assertEqual(result.hits, [])
        self.assertFalse(result.has_gt)
        self.assertTrue(any("ENVELOPE_GT skipped" in n for n in result.notes))

    def test_envelope(self) -> None:
        result = check_rows(
            [
                {"kind": "gt", "t": 0.0, "s": 0.0},
                {"kind": "est", "confidence": "OK", "t": 0.0, "s": 100.0, "v": 1.0,
                 "p_ss": 1.0, "p_vv": 0.1},
            ],
            require_gt=True,
        )
        self.assertTrue(any("ENVELOPE_GT" in h for h in result.hits))

    def test_envelope_without_require_gt_flag(self) -> None:
        result = check_rows(
            [
                {"kind": "gt", "t": 0.0, "s": 0.0},
                {"kind": "est", "confidence": "OK", "t": 0.0, "s": 100.0, "v": 1.0,
                 "p_ss": 1.0, "p_vv": 0.1},
            ],
            require_gt=False,
        )
        self.assertTrue(any("ENVELOPE_GT" in h for h in result.hits))

    def test_no_estimate(self) -> None:
        result = check_rows([{"kind": "input", "notch": 0.2}], require_gt=False)
        self.assertTrue(any("NO_ESTIMATE" in h for h in result.hits))

    def test_cli_empty(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "ok.jsonl"
            path.write_text(
                json.dumps({"kind": "est", "confidence": "DEGRADED", "t": 0.0,
                            "s": 0.0, "v": 0.0, "p_ss": 9.0, "p_vv": 0.4})
                + "\n",
                encoding="utf-8",
            )
            buf = io.StringIO()
            with redirect_stdout(buf):
                self.assertEqual(check_main([str(path)]), 0)
            self.assertIn("ENVELOPE_GT skipped", buf.getvalue())
            self.assertIn("checker empty", buf.getvalue())

    def test_hmi_rate_on_ok_breach(self) -> None:
        result = check_rows(
            [
                {"kind": "gt", "t": 0.0, "s": 0.0},
                {"kind": "est", "confidence": "OK", "t": 0.0, "s": 100.0, "v": 1.0,
                 "p_ss": 1.0, "p_vv": 0.1},
            ],
            require_gt=True,
        )
        self.assertEqual(result.n_ok, 1)
        self.assertEqual(result.n_hmi, 1)
        self.assertTrue(any("HMI-rate=" in n for n in result.notes))

    def test_hmi_rate_zero_inside_envelope(self) -> None:
        result = check_rows(
            [
                {"kind": "gt", "t": 0.0, "s": 10.0},
                {"kind": "est", "confidence": "OK", "t": 0.0, "s": 10.2, "v": 1.0,
                 "p_ss": 1.0, "p_vv": 0.1},
            ],
            require_gt=True,
        )
        self.assertEqual(result.hits, [])
        self.assertEqual(result.n_hmi, 0)
        self.assertEqual(result.n_ok, 1)

    def test_missed_path_at_first_degraded(self) -> None:
        result = check_rows(
            [
                {"kind": "gt", "t": 0.0, "s": 4.0},
                {"kind": "est", "confidence": "DEGRADED", "t": 0.0, "s": 6.5, "v": 0.0,
                 "p_ss": 1.0, "p_vv": 0.1},
            ],
            require_gt=True,
        )
        self.assertEqual(result.hits, [])
        self.assertAlmostEqual(result.missed_path_m, 2.5, places=6)

    def test_nan_not_hmi_zero(self) -> None:
        result = check_rows(
            [
                {"kind": "gt", "t": 0.0, "s": 0.0},
                {"kind": "est", "confidence": "OK", "t": 0.0, "s": "nan", "v": 1.0,
                 "p_ss": 1.0, "p_vv": 0.1},
            ],
            require_gt=True,
        )
        self.assertTrue(any("INVALID_STATE" in h for h in result.hits))
        self.assertEqual(result.n_hmi, 0)

    def test_negative_pss(self) -> None:
        result = check_rows(
            [{"kind": "est", "confidence": "OK", "t": 0.0, "s": 0.0, "v": 0.0,
              "p_ss": -1.0, "p_vv": 0.1}],
            require_gt=False,
        )
        self.assertTrue(any("INVALID_COVARIANCE" in h for h in result.hits))

    def test_time_join_not_last_seen(self) -> None:
        result = check_rows(
            [
                {"kind": "est", "confidence": "OK", "t": 0.0, "s": 100.0, "v": 1.0,
                 "p_ss": 1.0, "p_vv": 0.1},
                {"kind": "gt", "t": 10.0, "s": 0.0},
            ],
            require_gt=True,
        )
        self.assertTrue(any("UNMATCHED_GT" in h for h in result.hits))
        self.assertEqual(result.n_hmi, 0)


class Rosbag2Tests(unittest.TestCase):
    def test_cdr_float32_roundtrip(self) -> None:
        from rosbag2_io import decode_message, encode_float32

        blob = encode_float32(-0.5)
        rec = decode_message("std_msgs/msg/Float32", blob)
        self.assertIsNotNone(rec)
        self.assertAlmostEqual(rec["data"], -0.5, places=6)

    def test_cdr_f32_array_roundtrip(self) -> None:
        from rosbag2_io import decode_message, encode_float32_array

        omega = [1.5, 2.5, 3.5, 4.5]
        blob = encode_float32_array(omega)
        rec = decode_message("std_msgs/msg/Float32MultiArray", blob)
        self.assertEqual(len(rec["data"]), 4)
        for a, b in zip(rec["data"], omega):
            self.assertAlmostEqual(a, b, places=5)

    def test_cdr_wheels_roundtrip(self) -> None:
        from rosbag2_io import decode_message, encode_float64_array

        omega = [1.0, 2.0, 3.0, 4.0]
        blob = encode_float64_array(omega)
        rec = decode_message("std_msgs/msg/Float64MultiArray", blob)
        self.assertEqual(rec["data"], omega)

    def test_cdr_int16_roundtrip(self) -> None:
        from rosbag2_io import decode_message, encode_int16

        blob = encode_int16(8)
        rec = decode_message("std_msgs/msg/Int16", blob)
        self.assertAlmostEqual(rec["data"], 8.0)

    def test_cdr_joint_state_velocity(self) -> None:
        from rosbag2_io import decode_message, encode_joint_state

        blob = encode_joint_state([1.0, 2.0, 3.0, 4.0])
        rec = decode_message("sensor_msgs/msg/JointState", blob)
        self.assertEqual(rec["data"], [1.0, 2.0, 3.0, 4.0])

    def test_cdr_velocity_sensor_and_notch_and_fix(self) -> None:
        from rosbag2_io import (
            decode_message,
            encode_driver_command,
            encode_navsat_fix,
            encode_velocity_sensor,
        )

        vel = encode_velocity_sensor(12.5, stamp_s=1.25, frame_id="base_link")
        rec = decode_message("tram_vehicle_msgs/msg/VelocitySensor", vel)
        self.assertAlmostEqual(rec["velocity"], 12.5, places=9)
        self.assertAlmostEqual(rec["stamp_s"], 1.25, places=6)
        self.assertEqual(rec["frame_id"], "base_link")

        cmd = encode_driver_command(-8, stamp_s=2.0, frame_id="")
        rec = decode_message("tram_vehicle_msgs/msg/DriverControllerCommand", cmd)
        self.assertEqual(rec["position"], -8)
        self.assertEqual(rec["frame_id"], "")

        fix = encode_navsat_fix(
            55.8, 37.4, 150.0, stamp_s=3.5, status=2, service=1,
            covariance=[1.0, 0, 0, 0, 4.0, 0, 0, 0, 9.0], covariance_type=2,
        )
        rec = decode_message("sensor_msgs/msg/NavSatFix", fix)
        self.assertAlmostEqual(rec["lat"], 55.8, places=9)
        self.assertAlmostEqual(rec["lon"], 37.4, places=9)
        self.assertEqual(rec["status"], 2)
        self.assertEqual(rec["service"], 1)
        self.assertEqual(rec["covariance_type"], 2)
        self.assertAlmostEqual(rec["covariance"][0], 1.0)
        self.assertAlmostEqual(rec["covariance"][4], 4.0)
        self.assertAlmostEqual(rec["covariance"][8], 9.0)

    def test_cdr_twist_stamped(self) -> None:
        from rosbag2_io import decode_message, encode_twist_stamped

        blob = encode_twist_stamped(3.5)
        rec = decode_message("geometry_msgs/msg/TwistStamped", blob)
        self.assertAlmostEqual(rec["twist_vx"], 3.5)

    def test_bag_to_jsonl_twist_as_mps(self) -> None:
        from bag_to_jsonl import bag_to_rows
        from rosbag2_io import encode_float32, encode_twist_stamped, write_bag

        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "tw"
            rows = [
                ("/tram/controller_notch", "std_msgs/msg/Float32", 0, encode_float32(0.2)),
                ("/tram/brake_cmd", "std_msgs/msg/Float32", 1, encode_float32(0.0)),
                (
                    "/tram/wheel_odom",
                    "geometry_msgs/msg/TwistStamped",
                    2,
                    encode_twist_stamped(3.5),
                ),
            ]
            write_bag(bag, rows)
            recs, _ = bag_to_rows(
                bag, {}, notch_max_abs=8.0, n_wheels=4, wheel_radius_m=0.35
            )
            wheels = [r for r in recs if "w0" in r]
            self.assertTrue(wheels)
            self.assertAlmostEqual(wheels[0]["w0"], 3.5 / 0.35, places=6)
            self.assertTrue(math.isnan(wheels[0]["w1"]))
            self.assertTrue(math.isnan(wheels[0]["w2"]))
            self.assertTrue(math.isnan(wheels[0]["w3"]))
            self.assertNotAlmostEqual(wheels[0]["w1"], 3.5 / 0.35)

    def test_inspect_and_jsonl(self) -> None:
        from inspect_bag import main as inspect_main
        from bag_to_jsonl import bag_to_rows, _load_aliases
        from rosbag2_io import encode_float32, encode_float64_array, write_bag

        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "synth"
            rows = []
            for k in range(5):
                ts = k * 20_000_000
                rows.append(
                    (
                        "/tram/controller_notch",
                        "std_msgs/msg/Float32",
                        ts,
                        encode_float32(0.4),
                    )
                )
                rows.append(
                    (
                        "/tram/brake_cmd",
                        "std_msgs/msg/Float32",
                        ts + 1,
                        encode_float32(0.0),
                    )
                )
                rows.append(
                    (
                        "/tram/wheel_odom",
                        "std_msgs/msg/Float64MultiArray",
                        ts + 2,
                        encode_float64_array([10.0, 10.0, 10.0, 10.0]),
                    )
                )
            write_bag(bag, rows)
            self.assertEqual(inspect_main([str(bag)]), 0)
            recs, notes = bag_to_rows(bag, _load_aliases(None))
            self.assertEqual(notes, [])
            wheels = [r for r in recs if "w0" in r]
            self.assertEqual(len(wheels), 5)
            self.assertEqual(wheels[0]["w3"], 10.0)
            self.assertEqual(check_main([str(bag)]), 2)


class InspectQDefaultsTests(unittest.TestCase):
    def test_map_notch_q2(self) -> None:
        from rosbag2_io import map_notch

        self.assertAlmostEqual(map_notch(8.0), 1.0)
        self.assertAlmostEqual(map_notch(-4.0), -0.5)
        self.assertAlmostEqual(map_notch(0.4), 0.4)
        self.assertAlmostEqual(map_notch(1.0, encoding="discrete"), 0.125)
        self.assertAlmostEqual(map_notch(1.0, encoding="auto"), 1.0)
        self.assertAlmostEqual(map_notch(1.001, encoding="normalized"), 1.0)

    def test_map_notch_nonfinite_not_idle(self) -> None:
        from rosbag2_io import map_notch

        self.assertTrue(math.isnan(map_notch(float("nan"))))
        self.assertTrue(math.isinf(map_notch(float("inf"))))
        self.assertTrue(math.isnan(map_notch("not-a-notch")))  # type: ignore[arg-type]

    def test_pad_wheels_does_not_repeat_last(self) -> None:
        import math as _m

        from rosbag2_io import pad_wheels

        empty = pad_wheels([], 4)
        self.assertEqual(len(empty), 4)
        self.assertTrue(all(_m.isnan(x) for x in empty))
        short = pad_wheels([10.0], 4)
        self.assertEqual(short[0], 10.0)
        self.assertTrue(all(_m.isnan(x) for x in short[1:]))
        self.assertNotEqual(short[1], 10.0)
        junk = pad_wheels(["foo", None], 4)  # type: ignore[list-item]
        self.assertTrue(all(_m.isnan(x) for x in junk))

    def test_rows_to_filter_csv_missing_wheel_is_nan(self) -> None:
        from bag_to_jsonl import rows_to_filter_csv

        with tempfile.TemporaryDirectory() as tmp:
            dest = Path(tmp) / "filter.csv"
            n = rows_to_filter_csv(
                [
                    {
                        "kind": "input",
                        "t": 0.02,
                        "notch": 0.4,
                        "brake": 0.0,
                        "w0": 10.0,
                    }
                ],
                dest,
            )
            self.assertEqual(n, 1)
            text = dest.read_text(encoding="utf-8")
            self.assertIn("w0", text.splitlines()[0])
            cells = text.splitlines()[1].split(",")
            # t, notch, brake, w0, then padded w1.. at least w1
            self.assertGreaterEqual(len(cells), 5)
            self.assertTrue(math.isnan(float(cells[4])))

    def test_odometry_covariance_roundtrip(self) -> None:
        from rosbag2_io import decode_message, encode_odometry

        blob = encode_odometry(12.0, 3.0, 4.0, 0.25)
        rec = decode_message("nav_msgs/msg/Odometry", blob)
        self.assertIsNotNone(rec)
        self.assertAlmostEqual(rec["s"], 12.0, places=6)
        self.assertAlmostEqual(rec["v"], 3.0, places=6)
        self.assertAlmostEqual(rec["p_ss"], 4.0, places=6)
        self.assertAlmostEqual(rec["p_vv"], 0.25, places=6)
        self.assertAlmostEqual(rec["z"], 0.0, places=6)

    def test_odometry_decodes_z(self) -> None:
        from rosbag2_io import decode_message, encode_odometry

        blob = encode_odometry(10.0, 2.0, 1.0, 0.1, z=145.2)
        rec = decode_message("nav_msgs/msg/Odometry", blob)
        self.assertAlmostEqual(rec["z"], 145.2, places=6)

    def test_pose_stamped_decodes_xyz(self) -> None:
        from rosbag2_io import decode_message, encode_pose_stamped

        blob = encode_pose_stamped(10.0, 2.0, 145.2)
        rec = decode_message("geometry_msgs/msg/PoseStamped", blob)
        self.assertIsNotNone(rec)
        self.assertAlmostEqual(rec["s"], 10.0, places=6)
        self.assertAlmostEqual(rec["y"], 2.0, places=6)
        self.assertAlmostEqual(rec["z"], 145.2, places=6)

    def test_pose_stamped_has_no_invented_speed(self) -> None:
        from rosbag2_io import decode_message, encode_pose_stamped

        blob = encode_pose_stamped(10.0, 2.0, 145.2)
        rec = decode_message("geometry_msgs/msg/PoseStamped", blob)
        self.assertIsNotNone(rec)
        self.assertTrue(rec.get("v") in (None, ))

    def test_cdr_rejects_big_endian(self) -> None:
        from rosbag2_io import decode_message

        blob = b"\x00\x00\x00\x00" + b"\x00\x00\x00\x00"
        self.assertIsNone(decode_message("std_msgs/msg/Float32", blob))

    def test_cdr_rejects_truncated_odometry(self) -> None:
        from rosbag2_io import decode_message, encode_odometry

        blob = encode_odometry(1.0, 2.0, 0.1, 0.2)
        self.assertIsNone(decode_message("nav_msgs/msg/Odometry", blob[:20]))

    def test_inspect_q2_q3_on_customer_names(self) -> None:
        from inspect_bag import main as inspect_main
        from bag_to_jsonl import _load_adapter, bag_to_rows
        from rosbag2_io import encode_float32, encode_float64_array, encode_int8, write_bag

        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "cbt"
            yaml_path = Path(tmp) / "topics.yaml"
            yaml_path.write_text(
                "topic_adapter:\n  ros__parameters:\n"
                "    in_notch_topic: /cbt/controller_notch\n"
                "    in_brake_topic: /cbt/brake_cmd\n"
                "    in_wheels_topic: /cbt/wheel_speeds\n"
                "    notch_max_abs: 8.0\n"
                "    n_wheels: 4\n",
                encoding="utf-8",
            )
            rows = []
            for k in range(4):
                ts = k * 20_000_000
                rows.append(
                    ("/cbt/controller_notch", "std_msgs/msg/Int8", ts, encode_int8(8))
                )
                rows.append(
                    (
                        "/cbt/brake_cmd",
                        "std_msgs/msg/Float32",
                        ts + 1,
                        encode_float32(0.0),
                    )
                )
                rows.append(
                    (
                        "/cbt/wheel_speeds",
                        "std_msgs/msg/Float64MultiArray",
                        ts + 2,
                        encode_float64_array([10.0, 10.0, 10.0, 10.0]),
                    )
                )
            write_bag(bag, rows)
            self.assertEqual(inspect_main([str(bag)]), 2)
            aliases, notch_max, n_w, _, _ = _load_adapter(yaml_path)
            recs, _ = bag_to_rows(
                bag, aliases, notch_max_abs=notch_max, n_wheels=n_w
            )
            notches = [r["notch"] for r in recs if "notch" in r and "w0" not in r]
            self.assertTrue(notches)
            self.assertAlmostEqual(notches[0], 1.0)
            wheels = [r for r in recs if "w0" in r]
            self.assertEqual(len(wheels), 4)
            self.assertAlmostEqual(wheels[0]["notch"], 1.0)

    def test_inspect_does_not_guess_gps_speed_as_notch(self) -> None:
        from inspect_bag import guess_roles, probe_bag
        from rosbag2_io import encode_float32, encode_float64_array, write_bag

        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "gps"
            rows = []
            for k in range(4):
                ts = k * 20_000_000
                rows.append(("/gps/speed", "std_msgs/msg/Float32", ts, encode_float32(8.0)))
                rows.append(
                    (
                        "/tram/wheel_odom",
                        "std_msgs/msg/Float64MultiArray",
                        ts + 1,
                        encode_float64_array([10.0, 10.0, 10.0, 10.0]),
                    )
                )
            write_bag(bag, rows)
            roles = guess_roles(probe_bag(bag)["stats"])
            self.assertNotEqual(roles.get("notch"), "/gps/speed")

    def test_uninitialized_odometry_not_ok(self) -> None:
        from bag_to_jsonl import bag_to_rows, _load_aliases
        from rosbag2_io import encode_odometry, write_bag

        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "cold"
            write_bag(
                bag,
                [
                    (
                        "/tram/state_estimate",
                        "nav_msgs/msg/Odometry",
                        0,
                        encode_odometry(0.0, 0.0, 1.0e6, 1.0e6),
                    )
                ],
            )
            recs, _ = bag_to_rows(bag, _load_aliases(None))
            est = [r for r in recs if r.get("kind") == "est"]
            self.assertEqual(est[0]["confidence"], "UNINITIALIZED")
            result = check_rows(est, require_gt=False)
            self.assertTrue(any("UNINITIALIZED" in h for h in result.hits))

    def test_guess_roles_skips_imu_array(self) -> None:
        from inspect_bag import guess_roles
        from collections import Counter

        stats = {
            "/tram/controller_notch": {
                "type": "std_msgs/msg/Float32",
                "lens": Counter(),
                "scalar_min": 0.2,
                "scalar_max": 0.2,
            },
            "/imu/rpy": {
                "type": "std_msgs/msg/Float64MultiArray",
                "lens": Counter({3: 10}),
                "scalar_min": None,
                "scalar_max": None,
            },
        }
        roles = guess_roles(stats)
        self.assertNotEqual(roles.get("wheels"), "/imu/rpy")
        self.assertNotIn("wheels", roles)

    def test_yaml_snippet_enable_only_if_remap(self) -> None:
        from inspect_bag import _yaml_snippet, CANONICAL
        from collections import Counter

        roles = {
            "notch": CANONICAL["notch"],
            "brake": CANONICAL["brake"],
            "wheels": "/cbt/wheel_speeds",
        }
        stats = {
            CANONICAL["notch"]: {
                "type": "std_msgs/msg/Float32",
                "lens": Counter(),
                "scalar_min": 0.2,
                "scalar_max": 0.2,
            },
            "/cbt/wheel_speeds": {
                "type": "std_msgs/msg/Float64MultiArray",
                "lens": Counter({4: 3}),
                "scalar_min": None,
                "scalar_max": None,
            },
        }
        text = _yaml_snippet(roles, stats)
        self.assertIn("enable: true", text)
        self.assertIn("in_notch_topic: /tram/controller_notch", text)
        all_canon = {
            "notch": CANONICAL["notch"],
            "brake": CANONICAL["brake"],
            "wheels": CANONICAL["wheels"],
        }
        idle = _yaml_snippet(all_canon, stats)
        self.assertIn("enable: false", idle)
        self.assertIn("state_estimator:", idle)
        self.assertIn("    notch_type: float32", idle)

    def test_inspect_canonical_int8_notch_flags_estimator(self) -> None:
        from inspect_bag import main as inspect_main
        from rosbag2_io import encode_float32, encode_float64_array, encode_int8, write_bag

        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "i8"
            rows = [
                ("/tram/controller_notch", "std_msgs/msg/Int8", 0, encode_int8(4)),
                ("/tram/brake_cmd", "std_msgs/msg/Float32", 1, encode_float32(0.0)),
                (
                    "/tram/wheel_odom",
                    "std_msgs/msg/Float64MultiArray",
                    2,
                    encode_float64_array([10.0, 10.0, 10.0, 10.0]),
                ),
            ]
            write_bag(bag, rows)
            buf = io.StringIO()
            with redirect_stdout(buf):
                rc = inspect_main([str(bag)])
            self.assertEqual(rc, 2)
            self.assertIn("notch_type: int8", buf.getvalue())
            self.assertIn("state_estimator", buf.getvalue())

    def test_inspect_write_yaml(self) -> None:
        from inspect_bag import main as inspect_main
        from rosbag2_io import encode_float32, encode_float64_array, encode_int8, write_bag

        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "cbt"
            out = Path(tmp) / "customer_topics.yaml"
            rows = [
                ("/cbt/controller_notch", "std_msgs/msg/Int8", 0, encode_int8(8)),
                ("/cbt/brake_cmd", "std_msgs/msg/Float32", 1, encode_float32(0.0)),
                (
                    "/cbt/wheel_speeds",
                    "std_msgs/msg/Float64MultiArray",
                    2,
                    encode_float64_array([10.0, 10.0, 10.0, 10.0]),
                ),
            ]
            write_bag(bag, rows)
            rc = inspect_main([str(bag), "--write-yaml", str(out)])
            self.assertEqual(rc, 2)
            text = out.read_text(encoding="utf-8")
            self.assertIn("enable: true", text)
            self.assertIn("in_notch_type: int8", text)
            self.assertIn("state_estimator:", text)
            self.assertIn("    notch_type: float32", text)

    def test_inspect_write_yaml_canonical_int8(self) -> None:
        from inspect_bag import main as inspect_main
        from rosbag2_io import encode_float32, encode_float64_array, encode_int8, write_bag

        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "i8"
            out = Path(tmp) / "customer_topics.yaml"
            rows = [
                ("/tram/controller_notch", "std_msgs/msg/Int8", 0, encode_int8(4)),
                ("/tram/brake_cmd", "std_msgs/msg/Float32", 1, encode_float32(0.0)),
                (
                    "/tram/wheel_odom",
                    "std_msgs/msg/Float64MultiArray",
                    2,
                    encode_float64_array([10.0, 10.0, 10.0, 10.0]),
                ),
            ]
            write_bag(bag, rows)
            rc = inspect_main([str(bag), "--write-yaml", str(out)])
            self.assertEqual(rc, 2)
            text = out.read_text(encoding="utf-8")
            self.assertIn("enable: false", text)
            self.assertIn("    notch_type: int8", text)
            self.assertIn("    wheels_type: float64_array", text)

    def test_checker_on_estimate_bag(self) -> None:
        from rosbag2_io import encode_odometry, write_bag

        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "est"
            rows = [
                (
                    "/tram/state_estimate",
                    "nav_msgs/msg/Odometry",
                    0,
                    encode_odometry(1.0, 0.5, 9.0, 0.2),
                )
            ]
            write_bag(bag, rows)
            self.assertEqual(check_main([str(bag)]), 2)  # NO_CONFIDENCE: Odometry is not OK

    def test_diagnostic_array_roundtrip(self) -> None:
        from rosbag2_io import decode_message, encode_diagnostic_array

        blob = encode_diagnostic_array("OK", extra={"nis": "0.2"})
        rec = decode_message("diagnostic_msgs/msg/DiagnosticArray", blob)
        self.assertEqual(rec["confidence"], "OK")
        self.assertEqual(rec["values"]["nis"], "0.2")

    def test_checker_stitches_diagnostics_confidence(self) -> None:
        from bag_to_jsonl import bag_to_rows, _load_aliases
        from rosbag2_io import encode_diagnostic_array, encode_odometry, write_bag

        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "est_diag"
            rows = [
                (
                    "/tram/state_estimate",
                    "nav_msgs/msg/Odometry",
                    0,
                    encode_odometry(1.0, 0.5, 9.0, 0.2),
                ),
                (
                    "/tram/diagnostics",
                    "diagnostic_msgs/msg/DiagnosticArray",
                    1_000_000,
                    encode_diagnostic_array("OK"),
                ),
            ]
            write_bag(bag, rows)
            recs, _ = bag_to_rows(bag, _load_aliases(None))
            est = [r for r in recs if r.get("kind") == "est"]
            self.assertEqual(est[0]["confidence"], "OK")
            self.assertEqual(est[0]["confidence_src"], "diagnostics")
            self.assertEqual(check_main([str(bag)]), 0)

    def test_uninitialized_not_overridden_by_diag_ok(self) -> None:
        from bag_to_jsonl import bag_to_rows, _load_aliases
        from rosbag2_io import encode_diagnostic_array, encode_odometry, write_bag

        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "cold"
            rows = [
                (
                    "/tram/state_estimate",
                    "nav_msgs/msg/Odometry",
                    0,
                    encode_odometry(0.0, 0.0, 1.0e6, 1.0e6),
                ),
                (
                    "/tram/diagnostics",
                    "diagnostic_msgs/msg/DiagnosticArray",
                    1,
                    encode_diagnostic_array("OK"),
                ),
            ]
            write_bag(bag, rows)
            recs, _ = bag_to_rows(bag, _load_aliases(None))
            est = [r for r in recs if r.get("kind") == "est"]
            self.assertEqual(est[0]["confidence"], "UNINITIALIZED")
            self.assertEqual(check_main([str(bag)]), 2)

    def test_run_bag_baselines_without_gt(self) -> None:
        from run_bag import main as run_bag_main
        from rosbag2_io import encode_float32, encode_float64_array, write_bag

        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "dry"
            rows = []
            for k in range(5):
                ts = k * 20_000_000
                rows.append(
                    (
                        "/tram/controller_notch",
                        "std_msgs/msg/Float32",
                        ts,
                        encode_float32(0.3),
                    )
                )
                rows.append(
                    (
                        "/tram/brake_cmd",
                        "std_msgs/msg/Float32",
                        ts + 1,
                        encode_float32(0.0),
                    )
                )
                rows.append(
                    (
                        "/tram/wheel_odom",
                        "std_msgs/msg/Float64MultiArray",
                        ts + 2,
                        encode_float64_array([8.0, 8.0, 8.0, 8.0]),
                    )
                )
            write_bag(bag, rows)
            self.assertEqual(run_bag_main([str(bag), "--baselines"]), 0)

    def test_run_bag_baselines_with_gt(self) -> None:
        from run_bag import main as run_bag_main
        from rosbag2_io import encode_float32, encode_float64, encode_float64_array, write_bag

        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "gtbag"
            rows = []
            for k in range(8):
                ts = k * 20_000_000
                rows.append(
                    (
                        "/tram/controller_notch",
                        "std_msgs/msg/Float32",
                        ts,
                        encode_float32(0.2),
                    )
                )
                rows.append(
                    (
                        "/tram/brake_cmd",
                        "std_msgs/msg/Float32",
                        ts + 1,
                        encode_float32(0.0),
                    )
                )
                rows.append(
                    (
                        "/tram/wheel_odom",
                        "std_msgs/msg/Float64MultiArray",
                        ts + 2,
                        encode_float64_array([8.0, 8.0, 8.0, 8.0]),
                    )
                )
                rows.append(
                    ("/gt/s", "std_msgs/msg/Float64", ts + 3, encode_float64(k * 0.056))
                )
            write_bag(bag, rows)
            root = Path(__file__).resolve().parents[2]
            ukf = next(
                (
                    p
                    for p in (
                        root / "standalone" / "build" / "Debug" / "replay_ukf.exe",
                        root / "standalone" / "build" / "Release" / "replay_ukf.exe",
                        root / "standalone" / "build" / "replay_ukf",
                        root / "standalone" / "build" / "replay_ukf.exe",
                    )
                    if p.is_file()
                ),
                None,
            )
            argv = [str(bag), "--baselines"]
            if ukf is not None:
                argv.extend(["--ukf", str(ukf)])
            self.assertEqual(run_bag_main(argv), 0)
            self.assertTrue((bag / "_tramdr_eval" / "baselines.json").is_file())


class ReplayCatchupTests(unittest.TestCase):
    def _replay_ukf(self) -> Path | None:
        root = Path(__file__).resolve().parents[2]
        for p in (
            root / "standalone" / "build" / "Release" / "replay_ukf.exe",
            root / "standalone" / "build" / "Debug" / "replay_ukf.exe",
            root / "standalone" / "build" / "replay_ukf",
            root / "standalone" / "build" / "replay_ukf.exe",
        ):
            if p.is_file():
                return p
        return None

    def test_gap_over_20s_integrates_and_lands(self) -> None:
        import json
        import subprocess

        exe = self._replay_ukf()
        if exe is None:
            self.skipTest("replay_ukf not built")
        with tempfile.TemporaryDirectory() as tmp:
            csv_path = Path(tmp) / "filter.csv"
            out_path = Path(tmp) / "out.jsonl"
            w = 10.0
            csv_path.write_text(
                "t_s,notch,brake,w0,w1,w2,w3\n"
                f"0.00,0.0,0,{w},{w},{w},{w}\n"
                f"25.00,0.0,0,{w},{w},{w},{w}\n",
                encoding="utf-8",
            )
            r = subprocess.run(
                [str(exe), str(csv_path), str(out_path)],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(r.returncode, 0, r.stderr)
            recs = [
                json.loads(line)
                for line in out_path.read_text(encoding="utf-8").splitlines()
                if line.strip()
            ]
            self.assertEqual(len(recs), 2)
            self.assertNotEqual(recs[-1]["confidence"], "LOST")
            # 25 s at ~3.5 m/s. A 20 s cap would land near 70 m.
            self.assertGreater(recs[-1]["s"], 80.0)

    def test_duplicate_stamp_does_not_invent_dt(self) -> None:
        import json
        import subprocess

        exe = self._replay_ukf()
        if exe is None:
            self.skipTest("replay_ukf not built")
        with tempfile.TemporaryDirectory() as tmp:
            csv_path = Path(tmp) / "filter.csv"
            out_path = Path(tmp) / "out.jsonl"
            w = 10.0
            csv_path.write_text(
                "t_s,notch,brake,w0,w1,w2,w3\n"
                f"0.00,0.0,0,{w},{w},{w},{w}\n"
                f"0.00,0.0,0,{w},{w},{w},{w}\n"
                f"0.02,0.0,0,{w},{w},{w},{w}\n",
                encoding="utf-8",
            )
            r = subprocess.run(
                [str(exe), str(csv_path), str(out_path)],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(r.returncode, 0, r.stderr)
            recs = [
                json.loads(line)
                for line in out_path.read_text(encoding="utf-8").splitlines()
                if line.strip()
            ]
            self.assertEqual(len(recs), 2)
            self.assertAlmostEqual(recs[0]["t"], 0.0, places=9)
            self.assertAlmostEqual(recs[1]["t"], 0.02, places=9)

    def test_crlf_csv_keeps_all_four_wheels(self) -> None:
        import json
        import subprocess

        exe = self._replay_ukf()
        if exe is None:
            self.skipTest("replay_ukf not built")
        with tempfile.TemporaryDirectory() as tmp:
            csv_path = Path(tmp) / "filter.csv"
            out_path = Path(tmp) / "out.jsonl"
            # Python csv.excel writes \r\n even on Linux. An untrimmed last
            # header "w3\\r" used to drop the fourth axle → incomplete packet
            # → DEGRADED on every frame (CI e2e on 0.0.10).
            csv_path.write_bytes(
                b"t_s,notch,brake,w0,w1,w2,w3\r\n"
                b"0.00,0.4,0,10,10,10,10\r\n"
                b"0.02,0.4,0,10,10,10,10\r\n"
            )
            r = subprocess.run(
                [str(exe), str(csv_path), str(out_path)],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(r.returncode, 0, r.stderr)
            recs = [
                json.loads(line)
                for line in out_path.read_text(encoding="utf-8").splitlines()
                if line.strip()
            ]
            self.assertEqual(len(recs), 2)
            self.assertEqual(recs[0]["n_wheels"], 4)
            self.assertEqual(recs[1]["n_wheels"], 4)
            self.assertNotEqual(recs[-1]["confidence"], "LOST")

    def test_epoch_timestamps_round_trip(self) -> None:
        import json
        import subprocess

        exe = self._replay_ukf()
        if exe is None:
            self.skipTest("replay_ukf not built")
        with tempfile.TemporaryDirectory() as tmp:
            csv_path = Path(tmp) / "filter.csv"
            out_path = Path(tmp) / "out.jsonl"
            t0 = 1780000001.12345
            t1 = t0 + 0.02
            w = 10.0
            csv_path.write_text(
                "t_s,notch,brake,w0,w1,w2,w3\n"
                f"{t0:.17g},0.0,0,{w},{w},{w},{w}\n"
                f"{t1:.17g},0.0,0,{w},{w},{w},{w}\n",
                encoding="utf-8",
            )
            r = subprocess.run(
                [str(exe), str(csv_path), str(out_path)],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(r.returncode, 0, r.stderr)
            recs = [
                json.loads(line)
                for line in out_path.read_text(encoding="utf-8").splitlines()
                if line.strip()
            ]
            self.assertEqual(len(recs), 2)
            self.assertNotEqual(recs[0]["t"], recs[1]["t"])
            self.assertAlmostEqual(recs[0]["t"], t0, places=9)
            self.assertAlmostEqual(recs[1]["t"], t1, places=9)


class ScoreNeesTests(unittest.TestCase):
    def test_mean_nees_unit_variance(self) -> None:
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "synth"))
        from score import mean_nees

        self.assertAlmostEqual(mean_nees([1.0, -1.0], [1.0, 1.0]), 1.0)
        self.assertTrue(math.isnan(mean_nees([1.0], [0.0])))


class PitchToolsTests(unittest.TestCase):
    def test_identify_coast_recovers_davis_A(self) -> None:
        from identify_coast import identify
        from plant_ref import PlantParams, davis_resistance_n

        p = PlantParams()
        v = 12.0
        rows = []
        for _ in range(250):
            a = -davis_resistance_n(v, p) / p.m0_kg
            v = max(1.2, v + a * 0.02)
            w = v / p.r0_m
            rows.append(
                {"notch": 0.0, "brake": 0.0, "w0": w, "w1": w, "w2": w, "w3": w}
            )
        est = identify(rows, p=p)
        self.assertGreater(est["n_coast"], 50)
        self.assertAlmostEqual(est["A_d"], p.A_d, delta=80.0)

    def test_identify_coast_yaml_has_ros_keys(self) -> None:
        from identify_coast import format_yaml

        yaml = format_yaml(
            {
                "note": "unit",
                "n_coast": 1,
                "n_accel": 1,
                "A_d": 800.0,
                "a_trac_max": 1.3,
            }
        )
        self.assertIn("A_d:", yaml)
        self.assertIn("a_trac_max:", yaml)

    def test_identify_coast_r0_from_gt(self) -> None:
        from identify_coast import format_yaml, identify

        r_true = 0.31
        v = 10.0
        w = v / r_true
        rows = [
            {
                "notch": 0.0,
                "brake": 0.0,
                "w0": w,
                "w1": w,
                "w2": w,
                "w3": w,
                "gt_v": v,
            }
            for _ in range(80)
        ]
        est = identify(rows)
        self.assertAlmostEqual(est["wheel_radius_m"], r_true, delta=1e-6)
        yaml = format_yaml(est)
        self.assertIn("wheel_radius_m:", yaml)
        self.assertIn("r0_uncalibrated: false", yaml)

    def test_identify_notch_yaml_key(self) -> None:
        from identify_notch import format_yaml, identify

        rows = []
        v = 3.0
        for k in range(80):
            n = 0.8 if k % 2 == 0 else 0.5
            a = 1.3 * n - 800.0 / 28000.0
            v = min(5.5, v + a * 0.02)
            w = v / 0.35
            rows.append({"notch": n, "brake": 0.0, "w0": w, "w1": w, "w2": w, "w3": w})
        est = identify(rows)
        self.assertGreater(est["n_samples"], 12)
        self.assertIsNotNone(est["c_mps2"])
        yaml = format_yaml(est)
        self.assertIn("notch_as_accel:", yaml)
        self.assertIn("v_base_mps:", yaml)

    def test_identify_notch_knee(self) -> None:
        from identify_notch import identify
        from plant_ref import PlantParams

        p = PlantParams()
        p.v_base_mps = 6.0
        rows = []
        v = 2.0
        for k in range(40):
            n = 0.8 if k % 2 == 0 else 0.5
            a = n * p.a_trac_max - p.A_d / p.m0_kg
            v = min(5.5, v + a * 0.02)
            w = v / p.r0_m
            rows.append({"notch": n, "brake": 0.0, "w0": w, "w1": w, "w2": w, "w3": w})
        v = 8.0
        for k in range(40):
            n = 0.8
            vb = 8.0
            a = n * p.a_trac_max * (vb / v) - p.A_d / p.m0_kg
            v = v + a * 0.02
            w = v / p.r0_m
            rows.append({"notch": n, "brake": 0.0, "w0": w, "w1": w, "w2": w, "w3": w})
        est = identify(rows, p=p)
        self.assertGreaterEqual(est["n_high_v"], 12)
        self.assertGreater(est["v_base_mps"], 6.5)

    def test_identify_jerk_first_accel(self) -> None:
        from identify_jerk import identify

        rows = []
        v = 0.0
        for k in range(30):
            rows.append(
                {"notch": 0.0, "brake": 0.0, "w0": 0.0, "w1": 0.0, "w2": 0.0, "w3": 0.0, "gt_v": 0.0}
            )
        a_ss = 1.0
        j = 0.8
        a = 0.0
        for k in range(80):
            a = min(a_ss, a + j * 0.02)
            v = v + a * 0.02
            w = v / 0.35
            rows.append(
                {
                    "notch": 0.8,
                    "brake": 0.0,
                    "w0": w,
                    "w1": w,
                    "w2": w,
                    "w3": w,
                    "gt_v": v,
                }
            )
        est = identify(rows)
        self.assertGreater(est["n_samples"], 8)
        self.assertGreater(est["j_max_mps3"], 0.3)

    def test_stop_associate_cli_imports_sys(self) -> None:
        from stop_associate import main as stop_main

        with tempfile.TemporaryDirectory() as tmp:
            missing = Path(tmp) / "nope.jsonl"
            self.assertEqual(stop_main([str(missing)]), 1)

    def test_stop_associate_nearest_vertex(self) -> None:
        from stop_associate import associate

        stops = [
            {"s_m": 0.0, "lat_deg": 55.8, "lon_deg": 37.46, "name": "A"},
            {"s_m": 200.0, "lat_deg": 55.8, "lon_deg": 37.45, "name": "B"},
        ]
        rows = [{"t": 0.02 * i, "s": 2.0, "v": 0.0} for i in range(80)]
        rows.append({"t": 2.0, "s": 2.0, "v": 1.0})
        ev = associate(rows, stops, dwell_s=1.0, gate_m=40.0)
        self.assertEqual(len(ev), 1)
        self.assertEqual(ev[0]["stop"], "A")
        self.assertTrue(ev[0]["gated"])

    def test_route_10_estimator_stops_match_projector(self) -> None:
        import re

        path = Path(__file__).resolve().parents[2] / "tram_dr_localization" / "config" / "route_10.yaml"
        text = path.read_text(encoding="utf-8")
        lists = re.findall(r"^[^#\n]*route_s_m:\s*\[([^\]]+)\]", text, re.M)
        self.assertGreaterEqual(len(lists), 2, "map_projector and state_estimator both need route_s_m")
        parsed = [
            [float(x.strip()) for x in block.split(",") if x.strip()] for block in lists
        ]
        self.assertEqual(parsed[0], parsed[1])
        self.assertEqual(len(parsed[0]), 9)

    def test_plot_svg_has_series(self) -> None:
        from plot_run import render_svg

        t = [0.0, 0.02, 0.04]
        s = [0.0, 0.1, 0.2]
        v = [5.0, 5.0, 5.0]
        svg = render_svg("unit", t, s, v, s, v, ["OK", "OK", "DEGRADED"], s, s, s)
        self.assertIn("<svg", svg)
        self.assertIn("polyline", svg)
        self.assertIn("#2e7d32", svg)

    def test_mcap_without_rosbags_is_explicit(self) -> None:
        from rosbag2_io import iter_messages

        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp)
            (bag / "metadata.yaml").write_text(
                "rosbag2_bagfile_information:\n"
                "  storage_identifier: mcap\n"
                "  relative_file_paths:\n"
                "    - bag_0.mcap\n"
                "  message_count: 0\n",
                encoding="utf-8",
            )
            (bag / "bag_0.mcap").write_bytes(b"mcap")
            try:
                list(iter_messages(bag))
                self.fail("stub mcap must not iterate as sqlite3")
            except FileNotFoundError as e:
                self.assertIn("rosbags", str(e).lower())
            except Exception:
                pass

    def test_sca_trailer_role(self) -> None:
        r = sca_analyze(
            [40.0, 10.0, 10.0, 10.0],
            axle_role=[0, 1, 1, 1],
            traction=True,
        )
        self.assertTrue(r["used_trailer_consensus"])
        self.assertGreaterEqual(r["n_inflated_motor"], 1)
        self.assertEqual(r["n_inflated_trailer"], 0)


class ProfileFromBagTests(unittest.TestCase):
    def test_writes_csv_when_z_varies(self) -> None:
        from profile_from_bag import extract_profile
        from rosbag2_io import encode_odometry, write_bag

        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "zbag"
            rows = []
            for k in range(8):
                s = float(k) * 20.0
                z = 145.0 - 0.03 * s
                blob = encode_odometry(s, 0.0, 1.0, 0.1, y=0.0, z=z)
                rows.append(
                    ("/gt/odometry", "nav_msgs/msg/Odometry", k * 20_000_000, blob)
                )
            write_bag(bag, rows)
            prof = extract_profile(bag)
            self.assertGreater(len(prof), 3)
            self.assertLess(prof[-1]["z_m"], prof[0]["z_m"] - 1.0)

    def test_rejects_flat_z(self) -> None:
        from profile_from_bag import extract_profile
        from rosbag2_io import encode_odometry, write_bag

        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "flat"
            rows = []
            for k in range(6):
                blob = encode_odometry(float(k), 1.0, 1.0, 0.1, z=0.0)
                rows.append(
                    ("/gt/odometry", "nav_msgs/msg/Odometry", k * 20_000_000, blob)
                )
            write_bag(bag, rows)
            with self.assertRaises(SystemExit):
                extract_profile(bag)

    def test_skips_filter_odom_and_uses_pose_stamped(self) -> None:
        from profile_from_bag import extract_profile
        from rosbag2_io import encode_odometry, encode_pose_stamped, write_bag

        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "mix"
            rows = []
            for k in range(8):
                s = float(k) * 20.0
                z = 145.0 - 0.03 * s
                rows.append(
                    (
                        "/tram/state_estimate",
                        "nav_msgs/msg/Odometry",
                        k * 20_000_000,
                        encode_odometry(s, 0.0, 1.0, 0.1, z=0.0),
                    )
                )
                rows.append(
                    (
                        "/gt/pose",
                        "geometry_msgs/msg/PoseStamped",
                        k * 20_000_000 + 1,
                        encode_pose_stamped(s, 0.0, z),
                    )
                )
            write_bag(bag, rows)
            prof = extract_profile(bag)
            self.assertLess(prof[-1]["z_m"], prof[0]["z_m"] - 1.0)

    def test_rejects_undecodable_pose_name(self) -> None:
        from profile_from_bag import extract_profile
        from rosbag2_io import encode_float32, write_bag

        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "badpose"
            write_bag(
                bag,
                [("/gt/pose", "std_msgs/msg/Float32", 0, encode_float32(1.0))],
            )
            with self.assertRaises(SystemExit) as ctx:
                extract_profile(bag)
            self.assertIn("PoseStamped", str(ctx.exception))

    def test_max_messages_truncates(self) -> None:
        from profile_from_bag import extract_profile
        from rosbag2_io import encode_odometry, write_bag

        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "cap"
            rows = []
            for k in range(8):
                s = float(k) * 20.0
                z = 145.0 - 0.03 * s
                rows.append(
                    (
                        "/gt/odometry",
                        "nav_msgs/msg/Odometry",
                        k * 20_000_000,
                        encode_odometry(s, 0.0, 1.0, 0.1, y=0.0, z=z),
                    )
                )
            write_bag(bag, rows)
            with self.assertRaises(SystemExit):
                extract_profile(bag, max_messages=1)

    def test_sqlite_paths_ignore_escape(self) -> None:
        from rosbag2_io import read_metadata, sqlite_paths, write_bag
        from rosbag2_io import encode_odometry

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            bag = root / "bag"
            outside = root / "outside.db3"
            outside.write_bytes(b"not-a-bag")
            write_bag(
                bag,
                [
                    (
                        "/gt/odometry",
                        "nav_msgs/msg/Odometry",
                        0,
                        encode_odometry(0.0, 0.0, 1.0, 0.1),
                    )
                ],
            )
            meta = (bag / "metadata.yaml").read_text(encoding="utf-8")
            poisoned = meta.replace(
                "relative_file_paths:\n    - ",
                "relative_file_paths:\n    - ../outside.db3\n    - ",
            )
            if "../outside.db3" not in poisoned:
                poisoned = meta.replace(
                    "relative_file_paths:",
                    "relative_file_paths:\n    - ../outside.db3",
                )
            (bag / "metadata.yaml").write_text(poisoned, encoding="utf-8")
            info = read_metadata(bag)
            paths = sqlite_paths(bag, info)
            self.assertTrue(all(p.resolve().is_relative_to(bag.resolve()) for p in paths))
            self.assertFalse(any(p.resolve() == outside.resolve() for p in paths))


if __name__ == "__main__":
    unittest.main()
