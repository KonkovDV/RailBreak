"""Phase-1 dry-run contracts: csv→bag, splits, gramian, overbound, T0 report."""

from __future__ import annotations

import csv
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from statistics import NormalDist

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "eval"))
sys.path.insert(0, str(ROOT / "tools" / "hackathon"))
sys.path.insert(0, str(ROOT / "tools" / "ingest"))

from analysis import (  # noqa: E402
    empirical_gramian,
    kappa_overbound,
    parse_tick_line,
    scale_protection,
    stanford,
)
from figures import heatmap_svg, stanford_svg, timing_svg  # noqa: E402
from inject import inject_on_indices  # noqa: E402
from protocol import csv_to_bag, data_contract_markdown, split_indices  # noqa: E402
from ingest import ingest  # noqa: E402
from t0 import run_t0  # noqa: E402


class SplitTests(unittest.TestCase):
    def test_time_fractions_cover_all(self) -> None:
        rows = [{"t_s": i * 0.02, "notch": 0.2, "w0": 10.0} for i in range(500)]
        sp = split_indices(rows)
        all_i = sorted(sp["train"] + sp["val"] + sp["test"])
        self.assertEqual(all_i, list(range(500)))
        self.assertGreater(len(sp["train"]), len(sp["val"]))
        self.assertTrue(sp["note"].startswith("duration"))


class BagRoundtripTests(unittest.TestCase):
    def test_csv_to_bag_ingest_holds_notch(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            src = Path(td) / "in.csv"
            with src.open("w", encoding="utf-8", newline="") as f:
                w = csv.DictWriter(f, fieldnames=["t_s", "notch", "brake", "w0", "gt_v"])
                w.writeheader()
                w.writerow({"t_s": 0.0, "notch": 0.4, "brake": 0.0, "w0": 8.0, "gt_v": 2.8})
                w.writerow({"t_s": 0.02, "notch": 0.4, "brake": 0.0, "w0": 8.1, "gt_v": 2.82})
                w.writerow({"t_s": 0.04, "notch": "", "brake": 0.0, "w0": 8.2, "gt_v": 2.84})
            rows = []
            with src.open(encoding="utf-8", newline="") as f:
                rows = list(csv.DictReader(f))
            bag = csv_to_bag(rows, Path(td) / "bag")
            self.assertTrue((bag / "metadata.yaml").is_file())
            yaml_text = (bag / "metadata.yaml").read_text(encoding="utf-8")
            self.assertIn("\n  files:\n", yaml_text)
            self.assertIn("path: bag_0.db3", yaml_text)
            out_rows, meta = ingest(bag)
            self.assertGreaterEqual(len(out_rows), 3)
            self.assertAlmostEqual(float(out_rows[0]["notch"]), 0.4, places=5)
            self.assertAlmostEqual(float(out_rows[2]["notch"]), 0.4, places=5)
            self.assertEqual(meta["n_wheels"], 1)
            self.assertAlmostEqual(float(out_rows[0]["gt_v"]), 2.8, places=4)
            import sqlite3
            con = sqlite3.connect(bag / "bag_0.db3")
            qos = con.execute("SELECT offered_qos_profiles FROM topics LIMIT 1").fetchone()[0]
            con.close()
            self.assertTrue(qos)
            self.assertIn("history: 1", qos)
            self.assertIn("reliability: 2", qos)
            self.assertIn("depth: 10", qos)


class GramianTests(unittest.TestCase):
    def test_condition_span_positive(self) -> None:
        g = empirical_gramian(horizon_s=2.0)
        ev = g["eigenvalues_asc"]
        self.assertEqual(len(ev), 5)
        self.assertGreater(g["log10_span"], 1.0)
        self.assertGreaterEqual(g["rank_est"], 2)
        self.assertGreater(ev[-1], ev[0])
        self.assertTrue(g.get("weakest"))

    def test_regimes_each_have_rank(self) -> None:
        from analysis import empirical_gramian_regimes
        g = empirical_gramian_regimes(horizon_s=2.0)
        for name in ("traction", "coast", "brake"):
            self.assertGreaterEqual(g[name]["rank_est"], 1, name)


class OverboundTests(unittest.TestCase):
    def test_unit_gaussian_kappa_near_one(self) -> None:
        nd = NormalDist()
        z = [nd.inv_cdf((i + 0.5) / 400.0) for i in range(400)]
        k = kappa_overbound(z)
        self.assertGreaterEqual(k["kappa_ob"], 1.0)
        self.assertLess(k["kappa_ob"], 1.6)

    def test_stanford_hmi_on_ok_outside_pl(self) -> None:
        est = [{"t": 0.0, "s": 10.0, "p_ss": 0.01, "pl_s": 0.1, "confidence": "OK"}]
        gt = [{"t": 0.0, "s": 0.0}]
        st = stanford(est, gt)
        self.assertEqual(st["n_hmi"], 1)
        self.assertEqual(st["n_ok"], 1)

    def test_scale_protection_multiplies_pl(self) -> None:
        est = [{"t": 0.0, "s": 1.0, "pl_s": 2.0, "over_m": 2.0, "confidence": "OK"}]
        out = scale_protection(est, 3.0)
        self.assertAlmostEqual(out[0]["pl_s"], 6.0)
        self.assertEqual(est[0]["pl_s"], 2.0)


class TickParseTests(unittest.TestCase):
    def test_parses_max(self) -> None:
        t = parse_tick_line("replay_ukf tick_us p50=10.6 p99=16.9 n=1500\n")
        self.assertAlmostEqual(t["p99_us"], 16.9)
        self.assertEqual(t["n"], 1500)
        t2 = parse_tick_line("replay_ukf tick_us p50=10.6 p99=16.9 max=40.0 n=1500\n")
        self.assertAlmostEqual(t2["max_us"], 40.0)


class T0DryRunTests(unittest.TestCase):
    def test_report_without_ukf(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            src = Path(td) / "in.csv"
            with src.open("w", encoding="utf-8", newline="") as f:
                w = csv.DictWriter(f, fieldnames=["t_s", "notch", "brake", "w0", "gt_v", "gt_s"])
                w.writeheader()
                for i in range(80):
                    w.writerow({
                        "t_s": i * 0.02,
                        "notch": 0.5,
                        "brake": 0.0,
                        "w0": 10.0,
                        "gt_v": 3.5,
                        "gt_s": i * 0.07,
                    })
            out = Path(td) / "t0"
            summary = run_t0(src, out, ukf=None)
            self.assertEqual(summary["n"], 80)
            self.assertTrue((out / "REPORT.md").is_file())
            self.assertTrue((out / "data-contract.md").is_file())
            self.assertTrue((out / "synth.bag" / "metadata.yaml").is_file())
            self.assertTrue((out / "inject" / "catalog.json").is_file())
            self.assertTrue((out / "inject" / "val_slip_ramp_20.csv").is_file())
            self.assertTrue((out / "inject" / "full_val_slip_ramp_20.csv").is_file())
            self.assertFalse(list((out / "inject").glob("test_*.csv")))
            with (out / "inject" / "full_val_slip_ramp_20.csv").open(encoding="utf-8", newline="") as fh:
                full = list(csv.DictReader(fh))
            n_train = json.loads((out / "splits" / "splits.json").read_text(encoding="utf-8"))["n_train"]
            self.assertAlmostEqual(float(full[0]["w0"]), 10.0, places=5)
            self.assertAlmostEqual(float(full[n_train]["w0"]), 12.0, places=5)
            self.assertTrue((out / "figures" / "heatmap_fault_method.svg").is_file())
            self.assertTrue((out / "residual.yaml").is_file())
            self.assertTrue((out / "m4_probe.json").is_file())
            self.assertTrue((out / "figures" / "openloop.svg").is_file())
            m4 = json.loads((out / "m4_probe.json").read_text(encoding="utf-8"))
            self.assertGreaterEqual(int(m4["residual_n"]), 20)
            davis = json.loads((out / "davis_probe.json").read_text(encoding="utf-8"))
            self.assertTrue(davis["covers_truth"])
            gram = json.loads((out / "observability.json").read_text(encoding="utf-8"))
            self.assertIn("traction", gram["regimes"])
            self.assertIn("coast", gram["regimes"])
            self.assertIn("brake", gram["regimes"])
            self.assertTrue((out / "provenance.json").is_file())
            text = (out / "data-contract.md").read_text(encoding="utf-8")
            self.assertIn("TBD", text)
            self.assertIn("| gt_columns | yes |", text)
            self.assertNotIn("0.514667", text)
            header = (out / "run" / "filter.csv").read_text(encoding="utf-8").splitlines()[0]
            self.assertIn("w0", header)
            self.assertNotIn("w4", header)

    def test_t0_from_bag_writes_inspect(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            src = Path(td) / "in.csv"
            with src.open("w", encoding="utf-8", newline="") as f:
                w = csv.DictWriter(f, fieldnames=["t_s", "notch", "brake", "w0"])
                w.writeheader()
                for i in range(12):
                    w.writerow({
                        "t_s": i * 0.02, "notch": 0.3, "brake": 0.0, "w0": 7.0,
                    })
            rows = []
            with src.open(encoding="utf-8", newline="") as f:
                rows = list(csv.DictReader(f))
            bag = csv_to_bag(rows, Path(td) / "bag")
            out = Path(td) / "t0"
            summary = run_t0(bag, out, ukf=None)
            self.assertEqual(summary["n"], 12)
            self.assertTrue((out / "inspect.txt").is_file())
            self.assertTrue((out / "inspect.json").is_file())
            inspect = json.loads((out / "inspect.json").read_text(encoding="utf-8"))
            names = {t["name"] for t in inspect["topics"]}
            self.assertIn("/tram/controller_notch", names)
            self.assertIn("/tram/wheel_odom", names)


class InjectSegmentTests(unittest.TestCase):
    def test_train_wheels_untouched(self) -> None:
        rows = [
            {"t_s": 0.0, "w0": 10.0, "gt_v": 3.5},
            {"t_s": 0.02, "w0": 10.0, "gt_v": 3.5},
            {"t_s": 0.04, "w0": 10.0, "gt_v": 3.5},
        ]
        out = inject_on_indices(rows, [1, 2], "slip_ramp_20", seed=42)
        self.assertAlmostEqual(float(out[0]["w0"]), 10.0)
        self.assertAlmostEqual(float(out[1]["w0"]), 12.0)
        self.assertEqual(out[2]["gt_v"], 3.5)


class FigureTests(unittest.TestCase):
    def test_stanford_svg_has_axes(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "st.svg"
            est = [{"t": 0.0, "s": 1.0, "pl_s": 2.0, "confidence": "OK"}]
            gt = [{"t": 0.0, "s": 0.0}]
            stanford_svg(est, gt, p)
            text = p.read_text(encoding="utf-8")
            self.assertIn("<svg", text)
            self.assertIn("Stanford", text)
            heatmap_svg(
                [{"fault": "slip_ramp_20", "method": "ukf", "rmse_s": 1.2}],
                Path(td) / "h.svg",
            )
            self.assertIn("RMSE_s", (Path(td) / "h.svg").read_text(encoding="utf-8"))
            timing_svg({"p50_us": 10.0, "p99_us": 18.0, "max_us": 40.0}, Path(td) / "t.svg")
            self.assertIn("p99", (Path(td) / "t.svg").read_text(encoding="utf-8"))


class RosOdometryBenchTests(unittest.TestCase):
    def test_frozen_stamp_does_not_multiply_trials(self) -> None:
        from bench import score_ros_odometry
        from rosbag2_io import encode_odometry, write_bag

        with tempfile.TemporaryDirectory() as td:
            bag = Path(td) / "bag"
            samples = ((0.0, 0.0), (0.02, 0.01), (0.02, 0.01), (0.02, 9.0))
            msgs = []
            for t, s in samples:
                msgs.append((
                    "/tram/state_estimate",
                    "nav_msgs/msg/Odometry",
                    int(round(t * 1e9)),
                    encode_odometry(s, 0.5, 0.25, 0.01, stamp_s=t),
                ))
            write_bag(bag, msgs)
            gt = [
                {"t_s": "0.0", "gt_s": "0.0", "gt_v": "0.5"},
                {"t_s": "0.02", "gt_s": "0.01", "gt_v": "0.5"},
            ]
            scored = score_ros_odometry(bag, gt)
            self.assertEqual(scored["n_track"], 2)
            self.assertEqual(scored["n_paired"], 2)
            self.assertAlmostEqual(scored["rmse_s"], 0.0, places=6)


class TuneCriterionTests(unittest.TestCase):
    def test_perfect_nees_beats_bad(self) -> None:
        from tune import criterion
        good = criterion({"eps_400": 0.0, "rmse_v": 0.0, "nees_s": 1.0}, n_ok=100, n_total=100)
        bad = criterion({"eps_400": 25.0, "rmse_v": 0.56, "nees_s": math.e}, n_ok=50, n_total=100)
        self.assertLess(good, bad)


if __name__ == "__main__":
    unittest.main()
