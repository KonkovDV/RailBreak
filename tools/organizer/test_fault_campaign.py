"""The campaign catalog is the matrix. Injectors are checked without a bag."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fault_campaign import (  # noqa: E402
    CASES,
    Case,
    apply_streams,
    attack_mask,
    events_of,
    offset_east,
    outside_window,
    regress_events,
    reorder_events,
)


def _ids(group: str) -> set[str]:
    return {c.id for c in CASES if c.group == group}


class CatalogTests(unittest.TestCase):
    def test_ids_are_unique(self) -> None:
        ids = [c.id for c in CASES]
        self.assertEqual(len(ids), len(set(ids)))

    def test_single_bogie_matrix(self) -> None:
        ids = _ids("single")
        scales = ("p1", "p3", "p5", "p10", "m1", "m3", "m5", "m10")
        biases = ("p0_2", "p0_5", "p1", "m0_2", "m0_5", "m1")
        drops = ("0_5", "2", "5", "15", "30")
        delays = ("50ms", "100ms", "200ms", "400ms")
        for which in ("front", "rear"):
            for tag in scales:
                self.assertIn(f"{which}_scale_{tag}", ids)
            for tag in biases:
                self.assertIn(f"{which}_bias_{tag}", ids)
            self.assertIn(f"{which}_frozen", ids)
            self.assertIn(f"{which}_zero", ids)
            self.assertIn(f"{which}_nan", ids)
            self.assertIn(f"{which}_spike", ids)
            for tag in drops:
                self.assertIn(f"{which}_drop_{tag}", ids)
            for tag in delays:
                self.assertIn(f"{which}_delay_{tag}", ids)

    def test_both_bogies(self) -> None:
        ids = _ids("both")
        for tag in ("p1", "p3", "p5", "p10", "m1", "m3", "m5", "m10"):
            self.assertIn(f"both_scale_{tag}", ids)
        for name in (
            "both_slide_m20", "both_spin_p20", "both_frozen", "both_opposite",
            "both_front_then_rear", "both_after_notch",
        ):
            self.assertIn(name, ids)
        for tag in ("0_5", "2", "5", "15", "30"):
            self.assertIn(f"both_drop_{tag}", ids)

    def test_time_gnss_and_map(self) -> None:
        time_ids = _ids("time")
        for name in ("time_reorder", "time_regress", "time_equal_stamp", "time_rate_split", "time_burst"):
            self.assertIn(name, time_ids)
        for tag in ("0_3", "1", "5", "29", "31"):
            self.assertIn(f"time_gap_{tag}", time_ids)
        gnss = _ids("gnss")
        for name in (
            "gnss_master_rover", "gnss_master_only", "gnss_rover_only", "gnss_none",
            "gnss_nofix_after_start", "gnss_window_only", "gnss_master_later",
            "gnss_rover_later", "gnss_chord_80m", "gnss_rover_up_3m", "gnss_late_vs_wheels",
        ):
            self.assertIn(name, gnss)
        maps = _ids("map")
        for name in (
            "map_stop_missed", "map_stop_extra", "map_stop_at_light", "map_wrong_branch",
            "map_shared_terminus", "map_start_moving", "map_last_fix_off_ring",
        ):
            self.assertIn(name, maps)

    def test_inputs_the_twin_does_not_read_are_marked(self) -> None:
        blind = {c.id: c.changes_filter for c in CASES}
        self.assertFalse(blind["gnss_rover_up_3m"])
        self.assertFalse(blind["gnss_window_only"])
        self.assertFalse(blind["map_last_fix_off_ring"])
        self.assertTrue(blind["gnss_chord_80m"])
        self.assertTrue(blind["front_scale_p5"])


class InjectorTests(unittest.TestCase):
    def test_scale_bias_dropout_and_delay(self) -> None:
        t = np.array([0.0, 1.0, 2.0, 3.0, 4.0])
        u = np.array([10.0, 10.0, 10.0, 10.0, 10.0])
        front = (t, u)
        rear = (t.copy(), u.copy())
        scale = Case("front_scale_p5", "single", "scale", "front", 1.05, 0.0)
        (ft, fu), (rt, ru), ok = apply_streams(scale, front, rear, 2.0)
        self.assertTrue(ok)
        self.assertTrue(np.allclose(fu, [10, 10, 10.5, 10.5, 10.5]))
        self.assertTrue(np.allclose(ru, u))
        bias = Case("rear_bias_p0_2", "single", "bias", "rear", 0.2, 0.0)
        (_ft, _fu), (_rt, bru), ok = apply_streams(bias, front, rear, 2.0)
        self.assertTrue(ok)
        self.assertAlmostEqual(bru[2] - 10.0, 0.2 * 3.6)
        drop = Case("front_drop_2", "single", "dropout", "front", dur=2.0)
        (dt, du), _rear, ok = apply_streams(drop, front, rear, 1.0)
        self.assertTrue(ok)
        self.assertTrue(np.array_equal(dt, [0.0, 1.0, 3.0, 4.0]))
        self.assertTrue(np.array_equal(du, [10.0, 10.0, 10.0, 10.0]))
        lag = Case("front_delay_100ms", "single", "delay", "front", 0.1, 0.0)
        (lt, _lu), _r, ok = apply_streams(lag, front, rear, 2.0)
        self.assertTrue(np.allclose(lt, [0.0, 1.0, 2.1, 3.1, 4.1]))

    def test_gap_and_masks(self) -> None:
        t = np.array([0.0, 10.0, 20.0, 41.0])
        self.assertTrue(np.array_equal(outside_window(t, 10.0, 31.0), [True, True, False, True]))
        self.assertTrue(np.array_equal(attack_mask(t, 10.0, 0.0), [False, True, True, True]))

    def test_east_offset_is_80_m(self) -> None:
        lat = 55.8
        row = np.array([[0.0, 0.0, lat, 37.5, 150.0, 2.0]])
        out = offset_east(row, 80.0)
        meters = (out[0, 3] - 37.5) * 111320.0 * np.cos(np.deg2rad(lat))
        self.assertAlmostEqual(meters, 80.0, places=3)

    def test_reorder_and_regress_keep_delivery_order(self) -> None:
        ev = events_of((np.array([1.0, 1.02]), np.array([10.0, 10.0])),
                       (np.array([1.01, 1.03]), np.array([10.0, 10.0])), None)
        swapped, landed = reorder_events(ev, 1.0, 10.0)
        self.assertTrue(landed)
        self.assertGreater(swapped[0][0], swapped[1][0])
        back, landed = regress_events(ev, 1.0)
        self.assertTrue(landed)
        self.assertLess(back[2][0], back[1][0])


if __name__ == "__main__":
    unittest.main()
