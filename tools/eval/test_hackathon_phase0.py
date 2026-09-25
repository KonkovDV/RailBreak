"""Unit tests for ingest autodetect, inject isolation, residual basis, identify."""

from __future__ import annotations

import csv
import math
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "eval"))
sys.path.insert(0, str(ROOT / "tools" / "ingest"))
sys.path.insert(0, str(ROOT / "tools" / "ident"))
sys.path.insert(0, str(ROOT / "tools" / "hackathon"))

from ingest import (  # noqa: E402
    apply_omega_scale,
    detect_brake_source,
    detect_notch_encoding,
    detect_omega_scale,
    ingest_csv,
    ingest_parquet,
    map_notch_value,
)
from inject import FAULTS, inject_rows  # noqa: E402
from fit_residual import bsplines, eval_force, fit, to_yaml  # noqa: E402
from identify import identify, identify_radii, to_replay_yaml  # noqa: E402
from plant_ref import PlantParams, VehicleState, plant_step  # noqa: E402


def _covers(true: float, med: float, p05: float, p95: float, rel: float = 0.12) -> bool:
    if math.isfinite(p05) and math.isfinite(p95) and (p95 - p05) > 1e-6:
        pad = 0.05 * max(abs(true), 1.0)
        return (p05 - pad) <= true <= (p95 + pad)
    return abs(med - true) <= rel * max(abs(true), 1.0)


class IngestDetectTests(unittest.TestCase):
    def test_rpm_scale(self) -> None:
        rows = [[200.0, 201.0, 199.0, 200.5]]
        self.assertEqual(detect_omega_scale(rows), "rpm")
        scaled = apply_omega_scale(rows[0], "rpm")
        self.assertLess(max(abs(x) for x in scaled), 80.0)

    def test_percent_notch(self) -> None:
        enc, nmax = detect_notch_encoding([0.0, 50.0, -80.0])
        self.assertEqual(enc, "percent")
        self.assertAlmostEqual(map_notch_value(50.0, enc, nmax), 0.5)

    def test_absent_brake_none(self) -> None:
        self.assertEqual(detect_brake_source(False, [], [0.2, 0.3]), "none")

    def test_negative_notch_is_combined(self) -> None:
        self.assertEqual(detect_brake_source(False, [], [0.2, -0.4]), "notch")

    def test_event_csv_roundtrip(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "in.csv"
            with p.open("w", encoding="utf-8", newline="") as f:
                w = csv.DictWriter(f, fieldnames=["t_s", "notch", "w0"])
                w.writeheader()
                w.writerow({"t_s": 0.0, "notch": 50, "w0": 10})
                w.writerow({"t_s": 0.04, "notch": 50, "w0": 10})
            rows, meta = ingest_csv(p)
            self.assertEqual(meta["n_wheels"], 1)
            self.assertEqual(meta["notch_encoding"], "percent")
            self.assertEqual(meta["brake_source"], "none")
            self.assertAlmostEqual(rows[0]["notch"], 0.5)

    def test_event_notch_hold_last(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "in.csv"
            with p.open("w", encoding="utf-8", newline="") as f:
                w = csv.DictWriter(f, fieldnames=["t_s", "notch", "w0"])
                w.writeheader()
                w.writerow({"t_s": 0.0, "notch": 50, "w0": 10})
                w.writerow({"t_s": 0.5, "notch": "", "w0": 10})
            rows, _meta = ingest_csv(p)
            self.assertAlmostEqual(rows[0]["notch"], 0.5)
            self.assertAlmostEqual(rows[1]["notch"], 0.5)


class ParquetIngestTests(unittest.TestCase):
    def test_roundtrip_or_skip_without_pyarrow(self) -> None:
        try:
            import pyarrow as pa  # type: ignore
            import pyarrow.parquet as pq  # type: ignore
        except ImportError:
            self.skipTest("pyarrow not installed")
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "t.parquet"
            table = pa.table({
                "t_s": [0.0, 0.02],
                "notch": [0.4, 0.4],
                "brake": [0.0, 0.0],
                "w0": [8.0, 8.1],
                "gt_v": [2.8, 2.82],
            })
            pq.write_table(table, path)
            rows, meta = ingest_parquet(path)
            self.assertEqual(len(rows), 2)
            self.assertAlmostEqual(float(rows[0]["notch"]), 0.4)
            self.assertEqual(meta["n_wheels"], 1)


class InjectTests(unittest.TestCase):
    def test_does_not_touch_gt(self) -> None:
        rows = [
            {"t_s": 0.0, "notch": 0.4, "brake": 0.0, "w0": 10.0, "gt_s": 1.0, "gt_v": 3.5},
            {"t_s": 0.02, "notch": 0.4, "brake": 0.0, "w0": 10.2, "gt_s": 1.07, "gt_v": 3.6},
        ]
        out = inject_rows(rows, "slip_ramp_20", seed=42)
        self.assertEqual(out[0]["gt_s"], 1.0)
        self.assertAlmostEqual(float(out[0]["w0"]), 12.0)
        self.assertIn("slip_ramp_20", FAULTS)

    def test_quantize_skips_missing_axles(self) -> None:
        rows = [
            {"t_s": 0.0, "w0": 10.0, "w1": "", "w2": "nan", "gt_v": 3.5},
        ]
        out = inject_rows(rows, "quantize", seed=42)
        self.assertEqual(out[0]["gt_v"], 3.5)
        self.assertAlmostEqual(float(out[0]["w0"]), 10.0)
        self.assertEqual(out[0]["w1"], "")


class ResidualBasisTests(unittest.TestCase):
    def test_partition_of_unity(self) -> None:
        for v in (0.0, 3.0, 10.0, 20.0):
            s = sum(bsplines(v))
            self.assertAlmostEqual(s, 1.0, places=8)

    def test_zero_theta_is_zero_force(self) -> None:
        self.assertAlmostEqual(eval_force([0.0] * 34, 5.0, 0.4, 0.0), 0.0)

    def test_gate_one_matches_first_spline(self) -> None:
        theta = [1000.0] + [0.0] * 33
        b = bsplines(5.0)
        self.assertAlmostEqual(eval_force(theta, 5.0, 0.0, 0.0), 1000.0 * b[0], places=8)


class ResidualPythonCppGridTests(unittest.TestCase):
    """Lockstep with standalone/test_hackathon_phase0.cpp residual_python_grid."""

    GOLD = (
        (0.0, 0.0, 0.0, 1.0),
        (0.5, 0.4, 0.0, 5.503875),
        (2.0, 0.0, 0.0, 2.72),
        (3.5, -0.2, 0.1, 9.92153125),
        (5.0, 0.7, 0.0, 12.015865384615385),
        (7.5, 0.0, 0.3, 12.934114583333333),
        (10.0, -0.6, 0.0, 17.558974358974357),
        (13.0, 0.2, 0.5, 23.147810256410256),
        (16.0, 1.0, 0.0, 20.693333333333335),
        (18.5, -1.0, 1.0, 61.671249999999986),
        (20.0, 0.0, 0.0, 8.0),
    )

    def test_eval_force_matches_gold(self) -> None:
        theta = [float(i + 1) for i in range(34)]
        for v, n, b, f in self.GOLD:
            self.assertAlmostEqual(eval_force(theta, v, n, b), f, places=12, msg=(v, n, b))


class IdentifyRadiiTests(unittest.TestCase):
    def test_recovers_known_radius(self) -> None:
        rows = []
        r0 = 0.31
        for i in range(40):
            v = 5.0
            rows.append({"gt_v": v, "w0": v / r0, "w1": v / r0})
        est = identify_radii(rows)
        self.assertAlmostEqual(est["r0"], r0, places=6)

    def test_replay_yaml_drops_tiny_jerk(self) -> None:
        yaml, skipped = to_replay_yaml({
            "radii": {"r0_mean": 0.31},
            "davis": {"A_d": float("nan")},
            "a_trac_max": 1.2,
            "a_svc": float("nan"),
            "j_max_mps3": 0.046,
        })
        self.assertIn("wheel_radius_m: 0.31000", yaml)
        self.assertNotIn("j_max_mps3", yaml)
        self.assertIn("j_max_mps3", skipped)

    def test_replay_yaml_keeps_tau_in_gate(self) -> None:
        yaml, skipped = to_replay_yaml({
            "radii": {"r0_mean": 0.31},
            "davis": {},
            "lag": {"T_d_s": 0.20},
            "a_trac_max": float("nan"),
            "a_svc": float("nan"),
            "j_max_mps3": float("nan"),
        })
        self.assertIn("tau_drv_s: 0.2000", yaml)
        self.assertNotIn("tau_drv_s", skipped)


class ResidualForecastTests(unittest.TestCase):
    def test_model_mismatch_horizon_improves(self) -> None:
        from m4 import probe_model_mismatch
        with tempfile.TemporaryDirectory() as td:
            rec = probe_model_mismatch(Path(td), duration_s=20.0)
        self.assertGreaterEqual(rec["residual_n"], 20)
        self.assertTrue(rec["e_s_10s_improved"])
        self.assertLess(rec["residual"]["e_s_10s"], rec["physics"]["e_s_10s"])


def _write_ident_csv(path: Path) -> PlantParams:
    p = PlantParams()
    dt = 0.02
    rows: list[dict] = []
    t = 0.0

    def emit(x: VehicleState, notch: float, brake: float) -> None:
        nonlocal t
        w = x.v_mps / p.r0_m
        rows.append({
            "t_s": t,
            "notch": notch,
            "brake": brake,
            "gt_v": x.v_mps,
            "gt_s": x.s_m,
            "a_mps2": x.a_mps2,
            "w0": w, "w1": w, "w2": w, "w3": w,
        })
        t += dt

    x = VehicleState(v_mps=20.0)
    for _ in range(4000):
        plant_step(x, 0.0, 0.0, dt, p)
        emit(x, 0.0, 0.0)
    x = VehicleState(v_mps=12.0)
    for _ in range(4000):
        plant_step(x, 0.0, 0.0, dt, p)
        emit(x, 0.0, 0.0)
    x = VehicleState(v_mps=5.0)
    for _ in range(4000):
        plant_step(x, 0.0, 0.0, dt, p)
        emit(x, 0.0, 0.0)
    x = VehicleState(v_mps=0.3)
    for _ in range(180):
        plant_step(x, 0.7, 0.0, dt, p)
        emit(x, 0.7, 0.0)
    x = VehicleState(v_mps=12.0)
    for _ in range(200):
        plant_step(x, 0.0, 0.6, dt, p)
        emit(x, 0.0, 0.6)
    keys = ["t_s", "notch", "brake", "gt_v", "gt_s", "a_mps2", "w0", "w1", "w2", "w3"]
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    return p


class IdentifyPlantRecoveryTests(unittest.TestCase):
    def test_recovers_plant_params_with_ci(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            csv_path = Path(td) / "ident.csv"
            p = _write_ident_csv(csv_path)
            est = identify(csv_path, mass=p.m0_kg, gamma=p.gamma_rot)
            d = est["davis"]
            self.assertTrue(
                _covers(p.A_d, d["A_d"], d["A_d_p05"], d["A_d_p95"]),
                f"A_d {d}",
            )
            self.assertTrue(
                _covers(p.B_d, d["B_d"], d["B_d_p05"], d["B_d_p95"]),
                f"B_d {d}",
            )
            self.assertTrue(
                _covers(p.C_d, d["C_d"], d["C_d_p05"], d["C_d_p95"]),
                f"C_d {d}",
            )
            self.assertTrue(
                _covers(p.a_trac_max, est["a_trac_max"], est["a_trac_max_p05"],
                        est["a_trac_max_p95"]),
                f"a_trac {est['a_trac_max']}",
            )
            self.assertTrue(
                _covers(p.a_svc, est["a_svc"], est["a_svc_p05"], est["a_svc_p95"], rel=0.20),
                f"a_svc {est['a_svc']}",
            )
            self.assertAlmostEqual(est["radii"]["r0"], p.r0_m, places=4)


class DavisProbeTests(unittest.TestCase):
    def test_multispeed_covers_truth_and_gates_yaml(self) -> None:
        from davis import probe_davis_ident
        with tempfile.TemporaryDirectory() as td:
            rec = probe_davis_ident(Path(td))
        self.assertTrue(rec["covers_truth"], rec.get("covers"))
        self.assertEqual(set(rec["replay_has_davis"]), {"A_d", "B_d", "C_d"})
        self.assertNotIn("A_d", rec["replay_skipped"])
        truth = rec["truth"]
        hat = rec["hat"]
        self.assertLess(abs(hat["A_d"] - truth["A_d"]) / truth["A_d"], 0.01)
        self.assertLess(abs(hat["B_d"] - truth["B_d"]) / max(truth["B_d"], 1e-6), 0.02)
        self.assertLess(abs(hat["C_d"] - truth["C_d"]) / max(truth["C_d"], 1e-6), 0.02)


class ResidualMismatchHorizonTests(unittest.TestCase):
    def test_fitted_residual_beats_physics_on_10s(self) -> None:
        p_true = PlantParams()
        p_true.A_d *= 1.4
        p_true.B_d *= 1.4
        p_true.C_d *= 1.4
        p_filt = PlantParams()
        dt = 0.02
        rows = []
        x = VehicleState(v_mps=8.0)
        t = 0.0
        for _ in range(800):
            plant_step(x, 0.5, 0.0, dt, p_true)
            w = x.v_mps / p_filt.r0_m
            rows.append({
                "t_s": t, "notch": 0.5, "brake": 0.0, "gt_v": x.v_mps, "gt_s": x.s_m,
                "w0": w, "w1": w, "w2": w, "w3": w,
            })
            t += dt
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "mm.csv"
            keys = ["t_s", "notch", "brake", "gt_v", "gt_s", "w0", "w1", "w2", "w3"]
            with path.open("w", encoding="utf-8", newline="") as f:
                wri = csv.DictWriter(f, fieldnames=keys)
                wri.writeheader()
                for r in rows:
                    wri.writerow(r)
            fitted = fit(path, mass=p_filt.m0_kg)
            yaml = to_yaml(fitted)
            self.assertIn("enabled: true", yaml)
            self.assertIn("theta:", yaml)
            theta = fitted["theta"]
            x0 = 8.0
            xt = VehicleState(v_mps=x0)
            xp = VehicleState(v_mps=x0)
            xr = VehicleState(v_mps=x0)
            se_p = 0.0
            se_r = 0.0
            n = int(10.0 / dt)
            for _ in range(n):
                plant_step(xt, 0.5, 0.0, dt, p_true)
                plant_step(xp, 0.5, 0.0, dt, p_filt)
                extra = eval_force(theta, xr.v_mps, 0.5, 0.0)
                xr.f_bias_n = -extra
                plant_step(xr, 0.5, 0.0, dt, p_filt)
                xr.f_bias_n = 0.0
                se_p += (xp.v_mps - xt.v_mps) ** 2
                se_r += (xr.v_mps - xt.v_mps) ** 2
            rmse_p = math.sqrt(se_p / n)
            rmse_r = math.sqrt(se_r / n)
            self.assertLess(rmse_r, rmse_p, f"residual {rmse_r} vs physics {rmse_p}")


if __name__ == "__main__":
    unittest.main()
