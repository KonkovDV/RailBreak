"""Inspect a rosbag2 directory. No UKF, no ROS.

Prints topics, Hz, notch range, wheel count, and a YAML snippet for
`customer_topics.yaml` (adapter remap + estimator DDS types). The organiser
bag is not in this repository.
"""

from __future__ import annotations

import argparse
import math
import sys
from collections import Counter
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "eval"))

from rosbag2_io import (  # noqa: E402
    CANONICAL,
    decode_message,
    iter_messages,
    list_topics,
    read_metadata,
)

FILTER_FORBIDDEN_TYPES = (
    "sensor_msgs/msg/NavSatFix",
    "sensor_msgs/msg/Imu",
    "sensor_msgs/msg/PointCloud2",
    "sensor_msgs/msg/Image",
    "sensor_msgs/NavSatFix",
    "sensor_msgs/Imu",
    "sensor_msgs/PointCloud2",
    "sensor_msgs/Image",
)

MAX_MESSAGES = 250_000
FORBIDDEN_HINTS = (
    "imu",
    "gnss",
    "gps",
    "navsat",
    "lidar",
    "camera",
    "image",
    "pointcloud",
    "cloud",
    "rpy",
    "accel",
    "gyro",
    "mag",
)


def _sensor_forbidden(name: str, typ: str) -> bool:
    blob = f"{name} {typ}".lower()
    return any(h in blob for h in FORBIDDEN_HINTS)


def _norm_type(t: str) -> str:
    return t.replace("/msg/", "/")


def probe_bag(bagdir: Path, *, max_messages: int = MAX_MESSAGES) -> dict[str, Any]:
    info = read_metadata(bagdir)
    topics = list_topics(info)
    stats: dict[str, dict[str, Any]] = {}
    n_read = 0
    truncated = False
    for ts_ns, name, typ, blob in iter_messages(bagdir):
        n_read += 1
        if n_read > max_messages:
            truncated = True
            break
        st = stats.get(name)
        if st is None:
            st = {
                "type": typ,
                "count": 0,
                "t0": ts_ns,
                "t1": ts_ns,
                "scalar_min": None,
                "scalar_max": None,
                "uniq": set(),
                "lens": Counter(),
                "absmax_arr": 0.0,
                "decoded": 0,
                "cov_ok": 0,
                "cov_bad": 0,
            }
            stats[name] = st
        st["count"] += 1
        st["t1"] = ts_ns
        decoded = decode_message(typ, blob)
        if decoded is None:
            continue
        st["decoded"] += 1
        if "p_ss" in decoded:
            pss, pvv = decoded.get("p_ss"), decoded.get("p_vv")
            if (
                pss is not None
                and pvv is not None
                and math.isfinite(pss)
                and math.isfinite(pvv)
                and not (pss == 0.0 and pvv == 0.0)
            ):
                st["cov_ok"] += 1
            else:
                st["cov_bad"] += 1
        data = decoded.get("data")
        if data is None and isinstance(decoded.get("velocity"), list):
            data = decoded["velocity"]
        if isinstance(data, list):
            st["lens"][len(data)] += 1
            if data:
                st["absmax_arr"] = max(st["absmax_arr"], max(abs(float(x)) for x in data))
                cur = [float(x) for x in data[:8]]
                prev = st.get("last_arr")
                incs = st.setdefault("omega_inc", [])
                if prev is not None and len(prev) == len(cur) and len(incs) < 8000:
                    for a, b in zip(prev, cur):
                        incs.append(abs(b - a))
                st["last_arr"] = cur
        elif data is not None:
            x = float(data)
            st["scalar_min"] = x if st["scalar_min"] is None else min(st["scalar_min"], x)
            st["scalar_max"] = x if st["scalar_max"] is None else max(st["scalar_max"], x)
            if len(st["uniq"]) < 40:
                st["uniq"].add(round(x, 6) if abs(x) <= 1.5 else round(x, 3))
        elif decoded.get("s") is not None:
            x = float(decoded["s"])
            st["scalar_min"] = x if st["scalar_min"] is None else min(st["scalar_min"], x)
            st["scalar_max"] = x if st["scalar_max"] is None else max(st["scalar_max"], x)
    return {
        "info": info,
        "topics": topics,
        "stats": stats,
        "truncated": truncated,
        "n_read": n_read,
    }


def _hz(st: dict[str, Any]) -> float | None:
    dt = (st["t1"] - st["t0"]) * 1e-9
    if dt <= 0.0 or st["count"] < 2:
        return None
    return (st["count"] - 1) / dt


def _mode_len(st: dict[str, Any]) -> int | None:
    lens: Counter = st["lens"]
    if not lens:
        return None
    return int(lens.most_common(1)[0][0])


def guess_roles(stats: dict[str, dict[str, Any]]) -> dict[str, str]:
    """Fill adapter YAML when names are not /tram/*."""
    roles: dict[str, str] = {}
    for name, st in stats.items():
        if name in CANONICAL.values():
            for role, canon in CANONICAL.items():
                if name == canon:
                    roles[role] = name
    lower = {name: name.lower() for name in stats}
    if "wheels" not in roles:
        cands = []
        for name, st in stats.items():
            if _sensor_forbidden(name, st["type"]):
                continue
            nlen = _mode_len(st)
            typ = _norm_type(st["type"])
            is_js = "JointState" in typ
            is_tw = "TwistStamped" in typ
            if not is_js and not is_tw and (nlen is None or nlen < 2 or nlen > 8):
                continue
            score = 0
            if any(k in lower[name] for k in ("wheel", "odom", "speed", "axle", "joint")):
                score += 2
            if is_js or is_tw:
                score += 2
            if nlen == 4:
                score += 1
            if score == 0:
                continue
            cands.append((score, nlen == 4, name))
        cands.sort(reverse=True)
        if cands:
            roles["wheels"] = cands[0][2]
    if "notch" not in roles:
        cands = []
        for name, st in stats.items():
            if name == roles.get("wheels"):
                continue
            if _sensor_forbidden(name, st["type"]):
                continue
            typ = _norm_type(st["type"])
            if "MultiArray" in typ:
                continue
            score = 0
            if any(k in lower[name] for k in ("notch", "controller", "handle", "throttle", "traction")):
                score += 3
            if "Int8" in typ or "Int16" in typ:
                score += 2
            mx = st["scalar_max"]
            mn = st["scalar_min"]
            if mx is not None and mn is not None:
                a = max(abs(mn), abs(mx))
                if 1.05 < a <= 16.0:
                    score += 2
                if mn < 0.0:
                    score += 1
            if score < 3:
                continue
            cands.append((score, name))
        cands.sort(reverse=True)
        if cands:
            roles["notch"] = cands[0][1]
    if "brake" not in roles:
        cands = []
        for name, st in stats.items():
            if name in {roles.get("wheels"), roles.get("notch")}:
                continue
            if _sensor_forbidden(name, st["type"]):
                continue
            typ = _norm_type(st["type"])
            if "MultiArray" in typ:
                continue
            score = 0
            if "brake" in lower[name] or "tormoz" in lower[name]:
                score += 3
            mn, mx = st["scalar_min"], st["scalar_max"]
            if mn is not None and mx is not None and mn >= -0.05 and mx <= 1.05:
                score += 1
            if score:
                cands.append((score, name))
        cands.sort(reverse=True)
        if cands:
            roles["brake"] = cands[0][1]
    if "estimate" not in roles:
        for name, st in stats.items():
            if _sensor_forbidden(name, st["type"]):
                continue
            if "Odometry" in st["type"] and "gt" not in name.lower():
                roles["estimate"] = name
                break
    return roles


def _estimator_dds_types(stats: dict[str, dict[str, Any]]) -> tuple[str, str]:
    """Estimator subscriptions for messages already on /tram/*. Remapped bags stay canonical."""
    notch_type = "float32"
    wheels_type = "float64_array"
    st_n = stats.get(CANONICAL["notch"])
    if st_n:
        typ = _norm_type(st_n["type"])
        if "Int16" in typ:
            notch_type = "int16"
        elif "Int8" in typ:
            notch_type = "int8"
        elif typ.endswith("Float64") and "MultiArray" not in typ:
            notch_type = "float64"
    st_w = stats.get(CANONICAL["wheels"])
    if st_w:
        wt = _norm_type(st_w["type"])
        if "JointState" in wt:
            wheels_type = "joint_state"
        elif "TwistStamped" in wt:
            wheels_type = "twist_stamped"
        elif "Float32MultiArray" in wt:
            wheels_type = "float32_array"
    return notch_type, wheels_type


def _estimator_type_notes(stats: dict[str, dict[str, Any]]) -> list[str]:
    """Types on /tram/* that the default estimator subscription will miss."""
    notes: list[str] = []
    notch_type, wheels_type = _estimator_dds_types(stats)
    if notch_type == "int8":
        notes.append(
            "canonical notch is Int8 — customer_topics.yaml state_estimator "
            "notch_type: int8 (adapter cannot republish onto the same topic)"
        )
    elif notch_type == "int16":
        notes.append(
            "canonical notch is Int16 — customer_topics.yaml state_estimator "
            "notch_type: int16 (adapter cannot republish onto the same topic)"
        )
    elif notch_type == "float64":
        notes.append(
            "canonical notch is Float64 — customer_topics.yaml state_estimator "
            "notch_type: float64 (launch loads it onto the estimator)"
        )
    if wheels_type == "float32_array":
        notes.append(
            "canonical wheels are Float32MultiArray — customer_topics.yaml "
            "state_estimator wheels_type: float32_array"
        )
    elif wheels_type == "joint_state":
        notes.append(
            "canonical wheels are JointState — customer_topics.yaml "
            "state_estimator wheels_type: joint_state (uses velocity[])"
        )
    elif wheels_type == "twist_stamped":
        notes.append(
            "canonical wheels are TwistStamped — customer_topics.yaml "
            "state_estimator wheels_type: twist_stamped (linear.x → ω via r0 unless twist_is_omega)"
        )
    return notes


def _yaml_snippet(roles: dict[str, str], stats: dict[str, dict[str, Any]]) -> str:
    notch = roles.get("notch", "/cbt/controller_notch")
    brake = roles.get("brake", "/cbt/brake_cmd")
    wheels = roles.get("wheels", "/cbt/wheel_speeds")
    n_type = "float32"
    st_n = stats.get(notch)
    if st_n:
        typ = _norm_type(st_n["type"])
        if "Int16" in typ:
            n_type = "int16"
        elif "Int8" in typ:
            n_type = "int8"
        elif typ.endswith("Float64") and "MultiArray" not in typ:
            n_type = "float64"
    w_type = "float64_array"
    st_w = stats.get(wheels)
    if st_w:
        wt = _norm_type(st_w["type"])
        if "JointState" in wt:
            w_type = "joint_state"
        elif "TwistStamped" in wt:
            w_type = "twist_stamped"
        elif "Float32MultiArray" in wt:
            w_type = "float32_array"
    n_w = _mode_len(st_w) if st_w else 4
    n_w = n_w if n_w else 4
    mx = None
    if st_n and st_n["scalar_max"] is not None and st_n["scalar_min"] is not None:
        mx = max(abs(st_n["scalar_min"]), abs(st_n["scalar_max"]))
    notch_max = 8.0
    if mx is not None and mx > 1.05:
        notch_max = max(8.0, math.ceil(mx))
    remap = (
        notch != CANONICAL["notch"]
        or brake != CANONICAL["brake"]
        or wheels != CANONICAL["wheels"]
    )
    est_notch, est_wheels = _estimator_dds_types(stats)
    return (
        "topic_adapter:\n"
        "  ros__parameters:\n"
        f"    enable: {str(remap).lower()}  # false if in_* already /tram/* (no self-sub loop)\n"
        f"    in_notch_topic: {notch}\n"
        f"    in_brake_topic: {brake}\n"
        f"    in_wheels_topic: {wheels}\n"
        f"    in_notch_type: {n_type}\n"
        f"    in_notch_int8: {str(n_type == 'int8').lower()}\n"
        f"    in_wheels_type: {w_type}\n"
        f"    twist_is_omega: false\n"
        f"    notch_max_abs: {notch_max:.1f}\n"
        f"    n_wheels: {min(n_w, 6)}\n"
        "state_estimator:\n"
        "  ros__parameters:\n"
        f"    notch_type: {est_notch}\n"
        f"    wheels_type: {est_wheels}\n"
    )


def format_report(probe: dict[str, Any]) -> tuple[str, int, str]:
    info = probe["info"]
    topics = probe["topics"]
    stats = probe["stats"]
    names = {t["name"] for t in topics}
    lines: list[str] = []
    lines.append(f"storage: {info.get('storage_identifier')}  messages: {info.get('message_count')}")
    if probe["truncated"]:
        lines.append(f"payload sample truncated at {probe['n_read']} messages")
    lines.append("topics:")
    for t in topics:
        flag = ""
        if t["type"] in FILTER_FORBIDDEN_TYPES:
            flag = "  [NOT for estimator — GT/offline only]"
        extra = ""
        st = stats.get(t["name"])
        if st:
            hz = _hz(st)
            if hz is not None:
                extra += f"  ~{hz:.1f} Hz"
            nlen = _mode_len(st)
            if nlen is not None:
                extra += f"  array_n={nlen}"
            if st["scalar_min"] is not None:
                extra += f"  min={st['scalar_min']:g} max={st['scalar_max']:g}"
            if st["cov_ok"] or st["cov_bad"]:
                extra += f"  cov_ok={st['cov_ok']} cov_bad={st['cov_bad']}"
        lines.append(f"  {t['name']:40} {t['type']:36} n={t['count']}{flag}{extra}")

    gt = [
        t["name"]
        for t in topics
        if t["name"].startswith("/gt/")
        or t["type"] in FILTER_FORBIDDEN_TYPES
        or "NavSatFix" in t["type"]
    ]
    lines.append("Dataset:")
    lines.append("  default: rosbag2 sqlite3 (Humble) or mcap (pip install rosbags); GT offline-only")
    lines.append(f"  GT-like topics: {', '.join(gt) if gt else 'NONE (envelope vs GT skipped)'}")
    lines.append(
        "  Lidar localization / NavSatFix / IMU in the bag = GT for identify_* and the "
        "checker. Never a UKF measurement (no_gnss_scan.py)."
    )

    roles = guess_roles(stats)
    lines.append("Notch (discrete ±8, or already in [-1,1]):")
    notch_name = roles.get("notch")
    if notch_name and notch_name in stats:
        st = stats[notch_name]
        mn, mx = st["scalar_min"], st["scalar_max"]
        nuniq = len(st["uniq"])
        a = None if mx is None else max(abs(mn or 0.0), abs(mx))
        match = "matches default ±8" if a is not None and 4.0 <= a <= 8.5 else (
            "already in [-1,1] — map_notch is no-op" if a is not None and a <= 1.0 + 1e-6 else "check range vs notch_max_abs"
        )
        lines.append(
            f"  {notch_name}  {st['type']}  min={mn} max={mx} unique~{nuniq}  → {match}"
        )
    else:
        lines.append("  no scalar candidate; keep notch_max_abs=8, in_notch_type=float32")

    lines.append("Wheels (default n=4, r0=0.35 m):")
    wheels_name = roles.get("wheels")
    if wheels_name and wheels_name in stats:
        st = stats[wheels_name]
        nlen = _mode_len(st)
        match = "matches default n=4" if nlen == 4 else f"n={nlen} — pad/truncate to 4 in jsonl"
        amax = float(st.get("absmax_arr") or 0.0)
        unit = "filter expects rad/s"
        if amax > 120.0:
            unit = " |max|>120 — likely rpm or pulse rate, not rad/s"
        elif 0.0 < amax < 2.0:
            unit = " |max|<2 — likely m/s on the axle, not rad/s"
        lines.append(
            f"  {wheels_name}  {st['type']}  lens={dict(st['lens'])}  absmax={amax:g}  → {match}; {unit}"
        )
        incs = [x for x in (st.get("omega_inc") or []) if x > 1e-9]
        if len(incs) >= 40:
            uniq = sorted({round(x, 5) for x in incs})
            if 1 < len(uniq) <= 12:
                step = uniq[0]
                n_est = int(round((2.0 * math.pi / 0.02) / step)) if step > 1e-9 else 0
                lines.append(
                    f"  ω increments look quantized ({len(uniq)} levels, min Δω={step:g}). "
                    f"If pulse-count per tick, encoder_pulses_per_rev≈{n_est}; "
                    "set R(ω) and zupt_hold_s≥0.3. Run identify_jerk.py on the first accel."
                )
            else:
                lines.append(
                    f"  ω increments: {len(uniq)} distinct |Δω| (continuous-looking). "
                    "Keep encoder_pulses_per_rev=0 unless the histogram on the bag is stepped."
                )
    else:
        lines.append("  no MultiArray / JointState / TwistStamped candidate; keep n_wheels=4")

    lines.append(
        "Vehicle: route 10 unmanned is 71-911EM Львёнок-Москва (4 axles, Bo-Bo, "
        "all motor — no trailer ATP axle) — vehicle_lvenok_moscow.yaml. "
        "Driverless (ЦБТ, route 10) is not 'autonomous' (battery/catenary-free, route 90). "
        "Do not claim a single tare / kW / r0: sources disagree (22 / ≤24 / 25.2 t) and none is RE. "
        "Strogino bridge: do not invent i(s). June 2026 CWR on the river spans "
        "with a temporary line (mos.ru / AGN / Metro 17.06) — not the 21.05 "
        "Moscow 24 bus-replacement headline. Treat 25.09 as likely single-track "
        "with switches: do not assume 60 km/h. Check ω for two crawls to ~3 m/s "
        "~300–350 m apart. mass_door only at route_10 stop vertices. "
        "Fleet at large is 71-931M; 6-axle YAML is vehicle_vityaz_m.yaml. "
        "Synth twin remains Combino NF100. "
        "Lidar / NavSatFix / ЦБТ odometry in the bag is GT for identify_coast and the "
        "checker ONLY — never a filter measurement. "
        "/tram/brake_cmd semantics (incl. magnetic rail) — from payload, not OSINT."
    )

    lines.append("Canonical /tram/* present:")
    missing = []
    for key, name in CANONICAL.items():
        ok = name in names
        lines.append(f"  {key:10} {name:28} {'yes' if ok else 'NO'}")
        if key != "estimate" and not ok:
            missing.append(name)

    est = stats.get(CANONICAL["estimate"]) or stats.get(roles.get("estimate", ""))
    if est:
        lines.append(
            f"  recorded estimate covariance: ok={est['cov_ok']} bad={est['cov_bad']} "
            "(live ros2 topic echo still needs Humble)"
        )

    rc = 0
    if missing:
        lines.append("canonical inputs missing — adapter must be enabled")
        lines.append("suggested config/customer_topics.yaml (guess from payload, not organiser names):")
        for ln in _yaml_snippet(roles, stats).splitlines():
            lines.append("  " + ln)
        lines.append("names here are a guess from payload, not organiser names")
        rc = 2
    else:
        lines.append("canonical inputs present (adapter can stay disabled)")
        st_n = stats.get(CANONICAL["notch"])
        if st_n and st_n["scalar_max"] is not None:
            a = max(abs(st_n["scalar_min"] or 0.0), abs(st_n["scalar_max"]))
            if a > 1.05:
                lines.append(
                    f"  note: /tram/controller_notch |max|={a:g} > 1; "
                    "estimator map_notch and bag_to_jsonl will scale by notch_max_abs"
                )
        type_notes = _estimator_type_notes(stats)
        for n in type_notes:
            lines.append("  " + n)
        if type_notes:
            rc = 2
    yaml_text = _yaml_snippet(roles, stats)
    return "\n".join(lines) + "\n", rc, yaml_text


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="List rosbag2 topics for tramDR")
    ap.add_argument("bag", type=Path, help="rosbag2 directory (has metadata.yaml)")
    ap.add_argument(
        "--write-yaml",
        type=Path,
        default=None,
        help="write suggested customer_topics.yaml (adapter remap + estimator types)",
    )
    args = ap.parse_args(argv)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    try:
        print(f"bag: {args.bag}")
        probe = probe_bag(args.bag)
    except FileNotFoundError as e:
        sys.stderr.write(f"{e}\n")
        return 1
    text, rc, yaml_text = format_report(probe)
    sys.stdout.write(text)
    if args.write_yaml is not None:
        args.write_yaml.parent.mkdir(parents=True, exist_ok=True)
        args.write_yaml.write_text(yaml_text, encoding="utf-8")
        sys.stdout.write(f"wrote {args.write_yaml}\n")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
