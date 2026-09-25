"""Write the node's runtime assets from the train-only map and model.

Output directory (not in git; derived from organiser data):
  ring.csv     s_m,x_m,y_m,h_m,grade      ENU metres about lat0/lon0
  notch.csv    notch,a_v0,...,a_vN        m/s^2 per 1 m/s speed bin (centre +0.5)
  stops.csv    s_m,sd_m,n
  meta.yaml    lat0_deg, lon0_deg, ring_len_m, k0, off_ks_m
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_model import grade_of  # noqa: E402
from eval_odometer import build_ring  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", type=Path, default=Path("local/map/july27_arc.npz"))
    ap.add_argument("--model", type=Path, default=Path("local/model_arc"))
    ap.add_argument("--k0", type=float, default=1.0027)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    cl = np.load(args.map)
    model = np.load(args.model / "model.npz")
    stops = json.loads((args.model / "stops.json").read_text(encoding="utf-8"))
    ring = build_ring(cl, stops)
    args.out.mkdir(parents=True, exist_ok=True)
    grade = grade_of(ring["s"], ring["h"])
    with open(args.out / "ring.csv", "w", encoding="utf-8") as f:
        f.write("s_m,x_m,y_m,h_m,grade\n")
        for row in zip(ring["s"], ring["x"], ring["y"], ring["h"], grade):
            f.write(",".join(f"{v:.4f}" if i < 4 else f"{v:.6f}" for i, v in enumerate(row)) + "\n")
    v_edges = model["v_edges"]
    with open(args.out / "notch.csv", "w", encoding="utf-8") as f:
        f.write("notch," + ",".join(f"v{v:.1f}" for v in v_edges + 0.5) + "\n")
        for n, row in zip(model["notches"], model["table"]):
            f.write(f"{int(n)}," + ",".join(f"{a:.5f}" for a in row) + "\n")
    with open(args.out / "stops.csv", "w", encoding="utf-8") as f:
        f.write("s_m,sd_m,n\n")
        for st in sorted(ring["stops"], key=lambda q: q["s"]):
            f.write(f"{st['s']:.3f},{st['sd']:.3f},{st['n']}\n")
    (args.out / "meta.yaml").write_text(
        f"lat0_deg: {float(cl['lat0']):.9f}\n"
        f"lon0_deg: {float(cl['lon0']):.9f}\n"
        f"ring_len_m: {ring['L']:.4f}\n"
        f"off_ks_m: {ring['off_ks']:.4f}\n"
        f"k0: {args.k0:.6f}\n",
        encoding="utf-8",
    )
    print("ring points", len(ring["s"]), "length", round(ring["L"], 1), "stops", len(ring["stops"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
