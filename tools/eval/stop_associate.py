"""Optional stop associator. Reads estimate JSONL + route_10.yaml.

When v≈0, names the nearest OSM stop vertex. Does not write into the UKF.
Map matching is not a measurement.

Usage:
  python tools/eval/stop_associate.py ukf.jsonl
  python tools/eval/stop_associate.py ukf.jsonl --route tram_dr_localization/config/route_10.yaml
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ROUTE = ROOT / "tram_dr_localization" / "config" / "route_10.yaml"

# OSM tram_stop names along route 10 (same order as route_s_m).
# ginfo aliases: Новощукинская ≈ «Детская поликлиника»; order vs Бурназян
# differs OSM vs ginfo — do not reorder this list without GTFS.
STOP_NAMES = [
    "Метро «Щукинская»",
    "Новощукинская улица",
    "Медицинский центр имени Бурназяна",
    "Улица Исаковского, 33",
    "Улица Маршала Катукова",
    "Метро «Строгино»",
    "Школа имени Марины Цветаевой",
    "Таллинская улица, 6",
    "Улица Кулакова",
]


def _parse_route(path: Path) -> list[dict]:
    text = path.read_text(encoding="utf-8")
    try:
        import yaml  # type: ignore
    except ImportError:
        yaml = None  # type: ignore
    s_m: list[float] = []
    lat: list[float] = []
    lon: list[float] = []
    if yaml is not None:
        doc = yaml.safe_load(text) or {}
        params = (doc.get("map_projector") or {}).get("ros__parameters") or {}
        s_m = [float(x) for x in (params.get("route_s_m") or [])]
        lat = [float(x) for x in (params.get("route_lat_deg") or [])]
        lon = [float(x) for x in (params.get("route_lon_deg") or [])]
    else:
        import re

        def _nums(key: str) -> list[float]:
            m = re.search(rf"{key}:\s*\[([^\]]+)\]", text)
            if not m:
                return []
            return [float(x.strip()) for x in m.group(1).split(",") if x.strip()]

        s_m, lat, lon = _nums("route_s_m"), _nums("route_lat_deg"), _nums("route_lon_deg")
    n = min(len(s_m), len(lat), len(lon))
    out = []
    for i in range(n):
        name = STOP_NAMES[i] if i < len(STOP_NAMES) else f"stop_{i}"
        out.append({"s_m": s_m[i], "lat_deg": lat[i], "lon_deg": lon[i], "name": name})
    return out


def associate(
    rows: list[dict],
    stops: list[dict],
    *,
    v_cut: float = 0.15,
    dwell_s: float = 1.0,
    gate_m: float = 40.0,
) -> list[dict]:
    """Emit one event per dwell. Never a UKF update."""
    if not stops:
        return []
    events: list[dict] = []
    t0 = None
    t_last = None
    s_acc = 0.0
    n_acc = 0
    for rec in rows:
        t = rec.get("t")
        v = rec.get("v")
        s = rec.get("s")
        if v is None or s is None or t is None:
            continue
        t = float(t)
        v = float(v)
        s = float(s)
        if not math.isfinite(t) or not math.isfinite(v) or not math.isfinite(s):
            continue
        if abs(v) <= v_cut:
            if t0 is None:
                t0 = t
                s_acc = 0.0
                n_acc = 0
            s_acc += s
            n_acc += 1
            t_last = t
            continue
        if t0 is not None and n_acc > 0 and (t - t0) >= dwell_s:
            s_hat = s_acc / n_acc
            nearest = min(stops, key=lambda st: abs(st["s_m"] - s_hat))
            dist = abs(nearest["s_m"] - s_hat)
            events.append(
                {
                    "t": t0,
                    "s": s_hat,
                    "stop": nearest["name"] if dist <= gate_m else None,
                    "stop_s_m": nearest["s_m"],
                    "residual_m": dist,
                    "gated": dist <= gate_m,
                }
            )
        t0 = None
        n_acc = 0
    if t0 is not None and n_acc > 0:
        t_end = t_last if t_last is not None else t0
        if (t_end - t0) < dwell_s:
            return events
        s_hat = s_acc / n_acc
        nearest = min(stops, key=lambda st: abs(st["s_m"] - s_hat))
        dist = abs(nearest["s_m"] - s_hat)
        events.append(
            {
                "t": t0,
                "s": s_hat,
                "stop": nearest["name"] if dist <= gate_m else None,
                "stop_s_m": nearest["s_m"],
                "residual_m": dist,
                "gated": dist <= gate_m,
            }
        )
    return events


def _load_jsonl(path: Path) -> list[dict]:
    rows = []
    with path.open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            if rec.get("kind") in (None, "est"):
                rows.append(rec)
    return rows


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("jsonl", type=Path, help="ukf.jsonl or bag_to_jsonl estimate rows")
    ap.add_argument("--route", type=Path, default=DEFAULT_ROUTE)
    ap.add_argument("-o", "--out", type=Path, default=None)
    args = ap.parse_args(argv)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if not args.jsonl.is_file():
        sys.stderr.write(f"missing {args.jsonl}\n")
        return 1
    if not args.route.is_file():
        sys.stderr.write(f"missing {args.route}\n")
        return 1
    stops = _parse_route(args.route)
    if len(stops) < 2:
        sys.stderr.write("route has fewer than 2 vertices\n")
        return 1
    events = associate(_load_jsonl(args.jsonl), stops)
    text = "".join(json.dumps(e, ensure_ascii=False) + "\n" for e in events)
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
    sys.stdout.write(text if text else "# no dwells\n")
    sys.stdout.write(f"# n_events={len(events)} (not a UKF measurement)\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
