"""Run every fault on a split in parallel and write one table.

  python tools/organizer/fault_matrix.py --split val --out local/fault_matrix_val.json
"""

from __future__ import annotations

import argparse
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import faults  # noqa: E402
from eval_odometer import run  # noqa: E402
from odometer import Params  # noqa: E402

ARGS = {}


def one_fault(name: str) -> tuple[str, list[dict]]:
    cl = np.load(ARGS["map"])
    model = dict(np.load(Path(ARGS["model"]) / "model.npz"))
    stops = json.loads((Path(ARGS["model"]) / "stops.json").read_text(encoding="utf-8"))
    names = json.loads(Path(ARGS["splits"]).read_text(encoding="utf-8"))[ARGS["split"]]
    rows = []
    for n in names:
        r = run(Path(ARGS["org"]) / f"{n}.npz", cl, model, stops, 3.0, Params(), name)
        if r:
            r.pop("profile", None)
            rows.append(r)
    return name, rows


def _init(args):
    ARGS.update(args)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--org", default="local/org")
    ap.add_argument("--splits", default="local/splits.json")
    ap.add_argument("--map", default="local/map/july27_arc.npz")
    ap.add_argument("--model", default="local/model_arc")
    ap.add_argument("--split", default="val")
    ap.add_argument("--faults", nargs="*", default=sorted(faults.FAULTS))
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    args = {k: getattr(a, k) for k in ("org", "splits", "map", "model", "split")}
    table = {}
    with ProcessPoolExecutor(initializer=_init, initargs=(args,)) as ex:
        for name, rows in ex.map(one_fault, a.faults):
            med = lambda k: float(np.nanmedian([r.get(k, np.nan) for r in rows]))  # noqa: E731
            p95 = lambda k: float(np.nanpercentile([r.get(k, np.nan) for r in rows], 95))  # noqa: E731
            table[name] = {
                "n": len(rows),
                "all_finite": all(r["finite"] for r in rows),
                "along_rmse_med": med("along_rmse"), "along_rmse_p95": p95("along_rmse"),
                "end_pct_med": med("end_pct"),
                "v_rmse_med": med("v_rmse"),
                "during_max_med": med("along_during_max") if name != "none" else float("nan"),
                "after_max_med": med("along_after_max") if name != "none" else float("nan"),
                "before_max_med": med("along_before_max") if name != "none" else float("nan"),
                "slip_flag_frac": float(np.mean([r.get("slip_flag_in_window", False) for r in rows])) if name != "none" else float("nan"),
            }
            t = table[name]
            print(f"{name:18s} n={t['n']:2d} finite={t['all_finite']} along_rmse {t['along_rmse_med']:6.2f} "
                  f"(p95 {t['along_rmse_p95']:6.2f}) end% {t['end_pct_med']:.3f} v_rmse {t['v_rmse_med']:.3f} "
                  f"max before/during/after {t['before_max_med']:.1f}/{t['during_max_med']:.1f}/{t['after_max_med']:.1f} "
                  f"flag {t['slip_flag_frac']:.2f}", flush=True)
    a.out.write_text(json.dumps(table, indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
