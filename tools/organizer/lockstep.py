"""Python twin vs C++ core on one bag. Compares s and v, not diagnostic counters.

Writes the event CSV, runs replay_events.

  python tools/organizer/lockstep.py <bag> <replay_events.exe> [--assets local/assets]
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eval_odometer as E  # noqa: E402
from build_model import grade_of  # noqa: E402
from odometer import Odometer, Params  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("exe", type=Path)
    ap.add_argument("--assets", type=Path, default=Path("local/assets"))
    ap.add_argument("--map", type=Path, default=Path("local/map/july27_arc.npz"))
    ap.add_argument("--model", type=Path, default=Path("local/model_arc"))
    args = ap.parse_args()
    cl = np.load(args.map)
    model = dict(np.load(args.model / "model.npz"))
    stops = json.loads((args.model / "stops.json").read_text(encoding="utf-8"))
    ring = E.build_ring(cl, stops)
    z = np.load(f"local/org/{args.bag}.npz")
    init = E.ring_init(z, 3.0, ring, float(cl["lat0"]), float(cl["lon0"]))
    ev = []
    for key, kind in ((E.FRONT, 0), (E.REAR, 1), (E.CMD, 2)):
        for row in z[key]:
            ev.append((row[1], kind, row[2]))
    ev.sort(key=lambda e: (e[0], e[1]))
    ev = [e for e in ev if e[0] >= init["t0"]]
    od = Odometer(ring["s"], grade_of(ring["s"], ring["h"]), model["table"], model["notches"],
                  model["v_edges"], ring["stops"], Params(), ring_len=ring["L"])
    sig0 = max(init["d0"], 0.5)
    od.init(init["s0"], sig0)
    od.t = ev[0][0]
    py = []
    for t, kind, val in ev:
        if kind == 2:
            od.on_cmd(t, int(val))
        else:
            od.on_bogie(t, "front" if kind == 0 else "rear", float(val))
        py.append(od.state()[:2])
    py = np.array(py)
    with tempfile.TemporaryDirectory() as tmp:
        evp = Path(tmp) / "ev.csv"
        outp = Path(tmp) / "out.csv"
        with open(evp, "w", encoding="utf-8") as f:
            f.write("t,kind,value,s0,sigma_s0\n")
            for i, (t, kind, val) in enumerate(ev):
                f.write(f"{t:.9f},{kind},{val:.9f},{init['s0'] if i == 0 else 0:.6f},{sig0 if i == 0 else 0:.6f}\n")
        subprocess.run([str(args.exe), str(args.assets), str(evp), str(outp)], check=True)
        cpp = np.loadtxt(outp, delimiter=",", skiprows=1)
    ds = np.abs(cpp[:, 1] - py[:, 0])
    ds = np.minimum(ds, ring["L"] - ds)
    dv = np.abs(cpp[:, 2] - py[:, 1])
    print(f"{args.bag}: events {len(ev)}  max|ds| {ds.max():.3e} m  max|dv| {dv.max():.3e} m/s")
    return 0 if ds.max() < 0.05 and dv.max() < 1e-3 else 1


if __name__ == "__main__":
    raise SystemExit(main())
