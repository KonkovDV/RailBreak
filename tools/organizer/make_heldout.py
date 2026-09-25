"""Held-out style copy of an organiser bag: GNSS only in the first N seconds.

  python tools/organizer/make_heldout.py files/data/<id> local/heldout/<id> --gnss-s 3 [--fault NAME]

GNSS messages (all four topics) are kept while their header stamp is within
N seconds of the first master fix; everything else is copied byte for byte,
except bogie speeds when a fault is injected (re-encoded with the same header).
The scorer still reads the full GNSS from the original bag.
"""

from __future__ import annotations

import argparse
import sqlite3
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "eval"))
import faults  # noqa: E402
from rosbag2_io import decode_message, encode_velocity_sensor, sqlite_paths, read_metadata, write_bag  # noqa: E402

GNSS = ("/sensing/gnss/master/fix", "/sensing/gnss/master/vel", "/sensing/gnss/rover/fix", "/sensing/gnss/rover/vel")
BOGIE = {"/vehicle/front_bogie_velocity": "front", "/vehicle/rear_bogie_velocity": "rear"}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("src", type=Path)
    ap.add_argument("dst", type=Path)
    ap.add_argument("--gnss-s", type=float, default=3.0)
    ap.add_argument("--fault", default="none", choices=sorted(faults.FAULTS))
    ap.add_argument("--max-s", type=float, default=0.0, help="keep only the first N seconds (0 = all)")
    args = ap.parse_args()
    db = sqlite_paths(args.src, read_metadata(args.src))[0]
    con = sqlite3.connect(f"file:{db.as_posix()}?mode=ro", uri=True)
    topics = {r[0]: (r[1], r[2]) for r in con.execute("SELECT id,name,type FROM topics")}
    raw = [(topics[tid][0], topics[tid][1], ts, blob)
           for tid, ts, blob in con.execute("SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp,id")]
    con.close()
    t_first = None
    for name, typ, _ts, blob in raw:
        if name == "/sensing/gnss/master/fix":
            rec = decode_message(typ, blob)
            if rec and rec["status"] >= 0:
                t_first = rec["stamp_s"]
                break
    rows = []
    bogie = {"front": [], "rear": []}
    for name, typ, ts, blob in raw:
        if name in GNSS:
            rec = decode_message(typ, blob)
            st = rec.get("stamp_s") if rec and "stamp_s" in rec else None
            if st is None:
                # TwistStamped decode has no stamp: fall back to bag time.
                st = ts * 1e-9
            if t_first is not None and st <= t_first + args.gnss_s:
                rows.append((name, typ, ts, blob))
            continue
        if name in BOGIE and args.fault != "none":
            rec = decode_message(typ, blob)
            bogie[BOGIE[name]].append((ts, rec["stamp_s"], rec["velocity"], rec["frame_id"], typ, name))
            continue
        rows.append((name, typ, ts, blob))
    t_on = None
    if args.fault != "none":
        f = bogie["front"]
        tf = np.array([r[1] for r in f]); uf = np.array([r[2] for r in f])
        t_on = faults.pick_moving_time(tf, uf, faults.fault_window(args.fault) + 5.0)
        for which, recs in bogie.items():
            t = np.array([r[1] for r in recs]); u = np.array([r[2] for r in recs])
            by_stamp = {round(r[1], 9): r for r in recs}
            t2, u2 = faults.apply(args.fault, which, t, u, t_on)
            for tt, uu in zip(t2, u2):
                r = by_stamp[round(float(tt), 9)]
                rows.append((r[5], r[4], r[0], encode_velocity_sensor(float(uu), stamp_s=float(tt), frame_id=r[3])))
    rows.sort(key=lambda r: r[2])
    if args.max_s > 0 and rows:
        t_end = rows[0][2] + int(args.max_s * 1e9)
        rows = [r for r in rows if r[2] <= t_end]
    write_bag(args.dst, rows)
    print(f"{args.dst}: {len(rows)} messages, GNSS kept {sum(r[0] in GNSS for r in rows)}, fault {args.fault}"
          + (f" at +{t_on - t_first:.1f} s" if t_on and t_first else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
