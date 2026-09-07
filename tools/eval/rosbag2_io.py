"""Minimal rosbag2 (sqlite3 + CDR) reader/writer. No ROS, no UKF.

Inspect metadata.yaml, decode std_msgs scalars / MultiArray, Int16,
sensor_msgs/JointState (velocity[]), geometry_msgs/TwistStamped (linear.x),
nav_msgs/Odometry, diagnostic_msgs KeyValue `confidence`.
Unknown types are counted, not decoded.
"""

from __future__ import annotations

import sqlite3
import struct
from pathlib import Path
from typing import Any, Iterable, Iterator

# Untrusted-bag caps. Wheels are ≤6; JointState/diagnostics stay small.
MAX_CDR_SEQ = 256
MAX_CDR_DIM = 16
MAX_CDR_STR = 65_536
MAX_CDR_STAT = 64

CANONICAL = {
    "notch": "/tram/controller_notch",
    "brake": "/tram/brake_cmd",
    "wheels": "/tram/wheel_odom",
    "estimate": "/tram/state_estimate",
}


class CdrIn:
    def __init__(self, blob: bytes) -> None:
        if len(blob) < 4:
            raise ValueError("cdr too short")
        # Encapsulation header is big-endian: CDR_LE is bytes 00 01, not uint16 LE.
        ident = (blob[0] << 8) | blob[1]
        if ident != 0x0001:
            raise ValueError("cdr not little-endian")
        self.buf = blob
        self.i = 4

    def _need(self, n: int) -> None:
        if self.i + n > len(self.buf):
            raise ValueError("cdr truncated")

    def _align(self, n: int) -> None:
        off = self.i - 4
        pad = (n - (off % n)) % n
        if pad:
            self._need(pad)
            self.i += pad

    def u32(self) -> int:
        self._align(4)
        self._need(4)
        v = struct.unpack_from("<I", self.buf, self.i)[0]
        self.i += 4
        return v

    def i32(self) -> int:
        self._align(4)
        self._need(4)
        v = struct.unpack_from("<i", self.buf, self.i)[0]
        self.i += 4
        return v

    def i8(self) -> int:
        self._need(1)
        v = struct.unpack_from("<b", self.buf, self.i)[0]
        self.i += 1
        return v

    def i16(self) -> int:
        self._align(2)
        self._need(2)
        v = struct.unpack_from("<h", self.buf, self.i)[0]
        self.i += 2
        return v

    def u8(self) -> int:
        self._need(1)
        v = self.buf[self.i]
        self.i += 1
        return v

    def f32(self) -> float:
        self._align(4)
        self._need(4)
        v = struct.unpack_from("<f", self.buf, self.i)[0]
        self.i += 4
        return v

    def f64(self) -> float:
        self._align(8)
        self._need(8)
        v = struct.unpack_from("<d", self.buf, self.i)[0]
        self.i += 8
        return v

    def string(self) -> str:
        n = self.u32()
        if n < 0 or n > MAX_CDR_STR:
            raise ValueError("cdr string too long")
        self._need(n)
        raw = self.buf[self.i : self.i + n]
        self.i += n
        if raw.endswith(b"\x00"):
            raw = raw[:-1]
        return raw.decode("utf-8", errors="replace")


def encode_float32(value: float) -> bytes:
    return b"\x00\x01\x00\x00" + struct.pack("<f", float(value))


def encode_int8(value: int) -> bytes:
    return b"\x00\x01\x00\x00" + struct.pack("<b", int(value))


def encode_int16(value: int) -> bytes:
    return b"\x00\x01\x00\x00" + struct.pack("<h", int(value))


def encode_float64_array(values: Iterable[float]) -> bytes:
    data = list(values)
    payload = struct.pack("<I", 0)  # dim length
    payload += struct.pack("<I", 0)  # data_offset
    payload += struct.pack("<I", len(data))
    pad = (8 - (len(payload) % 8)) % 8
    payload += b"\x00" * pad
    if data:
        payload += struct.pack("<" + "d" * len(data), *data)
    return b"\x00\x01\x00\x00" + payload


def encode_float32_array(values: Iterable[float]) -> bytes:
    data = list(values)
    payload = struct.pack("<I", 0)
    payload += struct.pack("<I", 0)
    payload += struct.pack("<I", len(data))
    if data:
        payload += struct.pack("<" + "f" * len(data), *[float(x) for x in data])
    return b"\x00\x01\x00\x00" + payload


def encode_float64(value: float) -> bytes:
    return b"\x00\x01\x00\x00" + struct.pack("<d", float(value))


class CdrOut:
    def __init__(self) -> None:
        self.buf = bytearray()

    def _align(self, n: int) -> None:
        pad = (n - (len(self.buf) % n)) % n
        self.buf += b"\x00" * pad

    def i32(self, v: int) -> None:
        self._align(4)
        self.buf += struct.pack("<i", int(v))

    def u32(self, v: int) -> None:
        self._align(4)
        self.buf += struct.pack("<I", int(v))

    def u8(self, v: int) -> None:
        self.buf.append(int(v) & 0xFF)

    def i16(self, v: int) -> None:
        self._align(2)
        self.buf += struct.pack("<h", int(v))

    def f64(self, v: float) -> None:
        self._align(8)
        self.buf += struct.pack("<d", float(v))

    def string(self, s: str) -> None:
        raw = s.encode("utf-8") + b"\x00"
        self.u32(len(raw))
        self.buf += raw


def encode_pose_stamped(
    x: float,
    y: float,
    z: float,
    *,
    frame_id: str = "map",
) -> bytes:
    w = CdrOut()
    w.i32(0)
    w.u32(0)
    w.string(frame_id)
    w.f64(x)
    w.f64(y)
    w.f64(z)
    for qi in range(4):
        w.f64(0.0 if qi < 3 else 1.0)
    return b"\x00\x01\x00\x00" + bytes(w.buf)


def encode_odometry(
    s: float,
    v: float,
    p_ss: float,
    p_vv: float,
    *,
    frame_id: str = "map",
    child_frame_id: str = "base_link",
    y: float = 0.0,
    z: float = 0.0,
) -> bytes:
    w = CdrOut()
    w.i32(0)
    w.u32(0)
    w.string(frame_id)
    w.string(child_frame_id)
    w.f64(s)
    w.f64(float(y))
    w.f64(float(z))
    for qi in range(4):
        w.f64(0.0 if qi < 3 else 1.0)
    cov_p = [0.0] * 36
    cov_p[0] = float(p_ss)
    for c in cov_p:
        w.f64(c)
    w.f64(v)
    for _ in range(5):
        w.f64(0.0)
    cov_t = [0.0] * 36
    cov_t[0] = float(p_vv)
    for c in cov_t:
        w.f64(c)
    return b"\x00\x01\x00\x00" + bytes(w.buf)


def encode_twist_stamped(vx: float, *, frame_id: str = "") -> bytes:
    w = CdrOut()
    w.i32(0)
    w.u32(0)
    w.string(frame_id)
    w.f64(vx)
    for _ in range(5):
        w.f64(0.0)
    return b"\x00\x01\x00\x00" + bytes(w.buf)


def encode_joint_state(velocity: Iterable[float], names: Iterable[str] | None = None) -> bytes:
    w = CdrOut()
    w.i32(0)
    w.u32(0)
    w.string("")
    name_list = list(names) if names is not None else []
    w.u32(len(name_list))
    for n in name_list:
        w.string(n)
    w.u32(0)  # position
    vel = [float(x) for x in velocity]
    w.u32(len(vel))
    for x in vel:
        w.f64(x)
    w.u32(0)  # effort
    return b"\x00\x01\x00\x00" + bytes(w.buf)


DIAGNOSTICS_TOPIC = "/tram/diagnostics"


def encode_diagnostic_array(
    confidence: str,
    *,
    name: str = "tram_dr",
    message: str = "",
    level: int = 0,
    extra: dict[str, str] | None = None,
) -> bytes:
    """CDR twin of diagnostic_msgs/DiagnosticArray with KeyValue confidence.

    Enough for synthetic bags in test_eval. Not a full Humble encoder.
    """
    w = CdrOut()
    w.i32(0)
    w.u32(0)
    w.string("")
    w.u32(1)
    w.u8(level)
    w.string(name)
    w.string(message or confidence)
    w.string("")
    values = {"confidence": str(confidence)}
    if extra:
        values.update({str(k): str(v) for k, v in extra.items()})
    w.u32(len(values))
    for k, v in values.items():
        w.string(k)
        w.string(v)
    return b"\x00\x01\x00\x00" + bytes(w.buf)


def map_notch(
    raw: float,
    notch_max_abs: float = 8.0,
    encoding: str = "auto",
) -> float:
    """Twin of tram_dr::map_notch. encoding: auto | normalized | discrete."""
    try:
        x = float(raw)
    except (TypeError, ValueError):
        return float("nan")
    if x != x or x in (float("inf"), float("-inf")):
        return x
    m = max(abs(float(notch_max_abs)), 1.0)
    if encoding == "discrete":
        return max(-1.0, min(1.0, x / m))
    if encoding == "normalized":
        return max(-1.0, min(1.0, x))
    if abs(x) <= 1.0:
        return max(-1.0, min(1.0, x))
    return max(-1.0, min(1.0, x / m))


def _as_wheel(x: Any) -> float:
    try:
        return float(x)
    except (TypeError, ValueError):
        return float("nan")


def pad_wheels(omega: list[float], n: int = 4) -> list[float]:
    """Pad a short encoder packet with NaN, never with a repeated live value.

    Repeating the last ω made a 1-axle JointState look like four agreeing
    wheels (false SCA consensus, false ZUPT). Empty → all-NaN, which the
    core treats as data and fails closed.
    """
    n = max(1, min(6, n))
    nan = float("nan")
    if not omega:
        return [nan] * n
    out = [_as_wheel(x) for x in omega[:n]]
    while len(out) < n:
        out.append(nan)
    return out


def _decode_multiarray(r: CdrIn, is_f64: bool) -> dict[str, Any]:
    dim_n = r.u32()
    if dim_n > MAX_CDR_DIM:
        return {"data": []}
    for _ in range(dim_n):
        _ = r.string()
        _ = r.u32()
        _ = r.u32()
    _ = r.u32()  # data_offset
    n = r.u32()
    if n > MAX_CDR_SEQ:
        return {"data": []}
    if is_f64:
        return {"data": [r.f64() for _ in range(n)]}
    return {"data": [float(r.f32()) for _ in range(n)]}


def decode_message(msg_type: str, blob: bytes) -> dict[str, Any] | None:
    try:
        return _decode_message(msg_type, blob)
    except (ValueError, struct.error, IndexError):
        return None


def _decode_message(msg_type: str, blob: bytes) -> dict[str, Any] | None:
    t = msg_type.replace("/msg/", "/")
    r = CdrIn(blob)
    if t in {"std_msgs/Float32", "std_msgs/msg/Float32"}:
        return {"data": r.f32()}
    if t in {"std_msgs/Float64", "std_msgs/msg/Float64"}:
        return {"data": r.f64()}
    if t in {"std_msgs/Int8", "std_msgs/msg/Int8"}:
        return {"data": float(r.i8())}
    if t in {"std_msgs/Int16", "std_msgs/msg/Int16"}:
        return {"data": float(r.i16())}
    if t in {"std_msgs/Float64MultiArray", "std_msgs/msg/Float64MultiArray"}:
        return _decode_multiarray(r, True)
    if t in {"std_msgs/Float32MultiArray", "std_msgs/msg/Float32MultiArray"}:
        return _decode_multiarray(r, False)
    if t in {
        "geometry_msgs/TwistStamped",
        "geometry_msgs/msg/TwistStamped",
    }:
        _ = r.i32()
        _ = r.u32()
        _ = r.string()
        vx = r.f64()
        return {"data": [vx], "twist_vx": vx, "v": vx}
    if t in {
        "sensor_msgs/JointState",
        "sensor_msgs/msg/JointState",
    }:
        _ = r.i32()
        _ = r.u32()
        _ = r.string()
        n_names = r.u32()
        if n_names > MAX_CDR_SEQ:
            return None
        for _ in range(n_names):
            _ = r.string()
        n_pos = r.u32()
        if n_pos > MAX_CDR_SEQ:
            return None
        for _ in range(n_pos):
            _ = r.f64()
        n_vel = r.u32()
        if n_vel > MAX_CDR_SEQ:
            return None
        vel = [r.f64() for _ in range(n_vel)]
        return {"data": vel, "velocity": vel}
    if t in {"nav_msgs/Odometry", "nav_msgs/msg/Odometry"}:
        _ = r.i32()
        _ = r.u32()
        _ = r.string()
        _ = r.string()
        x = r.f64()
        y = r.f64()
        z = r.f64()
        for _ in range(4):
            _ = r.f64()
        cov0 = r.f64()
        for _ in range(35):
            _ = r.f64()
        vx = r.f64()
        for _ in range(5):
            _ = r.f64()
        tw0 = r.f64()
        return {"s": x, "y": y, "z": z, "v": vx, "p_ss": cov0, "p_vv": tw0}
    if t in {
        "geometry_msgs/PoseStamped",
        "geometry_msgs/msg/PoseStamped",
    }:
        _ = r.i32()
        _ = r.u32()
        _ = r.string()
        x = r.f64()
        y = r.f64()
        z = r.f64()
        for _ in range(4):
            _ = r.f64()
        return {"s": x, "y": y, "z": z, "v": None}
    if t in {
        "diagnostic_msgs/DiagnosticArray",
        "diagnostic_msgs/msg/DiagnosticArray",
    }:
        _ = r.i32()
        _ = r.u32()
        _ = r.string()
        nstat = r.u32()
        if nstat > MAX_CDR_STAT:
            return None
        kvs: dict[str, str] = {}
        message = ""
        for _ in range(nstat):
            _level = r.u8()
            name = r.string()
            msg = r.string()
            _hw = r.string()
            nkv = r.u32()
            if nkv > MAX_CDR_SEQ:
                return None
            this_kvs: dict[str, str] = {}
            for _ in range(nkv):
                k = r.string()
                v = r.string()
                this_kvs[k] = v
            if name == "tram_dr" or (not kvs and "confidence" in this_kvs):
                kvs = this_kvs
                message = msg
                if name == "tram_dr":
                    break
        out: dict[str, Any] = {"values": kvs, "message": message}
        if "confidence" in kvs:
            out["confidence"] = kvs["confidence"]
        return out
    return None


def read_metadata(bagdir: Path) -> dict[str, Any]:
    meta = bagdir / "metadata.yaml"
    if not meta.is_file():
        raise FileNotFoundError(f"not a rosbag2 directory: {bagdir}")
    text = meta.read_text(encoding="utf-8")
    try:
        import yaml  # type: ignore
    except ImportError:
        yaml = None  # type: ignore
    if yaml is not None:
        doc = yaml.safe_load(text)
        return doc.get("rosbag2_bagfile_information") or doc
    return _parse_metadata_fallback(text)


def _parse_metadata_fallback(text: str) -> dict[str, Any]:
    """Enough to inspect Humble metadata.yaml if PyYAML is missing."""
    import re

    files = re.findall(r"relative_file_paths:\s*\n(?:\s*-\s*(\S+)\s*\n)+", text)
    simple_files = re.findall(r"-\s+(\S+\.db3)", text)
    ident_m = re.search(r"storage_identifier:\s+(\S+)", text)
    ident = (ident_m.group(1) if ident_m else "sqlite3").strip().strip("'\"")
    names = re.findall(r"name:\s+(\S+)", text)
    types = re.findall(r"type:\s+(\S+)", text)
    counts = [int(x) for x in re.findall(r"message_count:\s+(\d+)", text)]
    topics = []
    # First message_count is the bag total; per-topic counts follow.
    topic_counts = counts[1:] if len(counts) > 1 else counts
    for i, name in enumerate(names):
        typ = types[i] if i < len(types) else ""
        n = topic_counts[i] if i < len(topic_counts) else 0
        topics.append({"topic_metadata": {"name": name, "type": typ}, "message_count": n})
    total = counts[0] if counts else sum(topic_counts)
    return {
        "storage_identifier": ident,
        "message_count": total,
        "topics_with_message_count": topics,
        "relative_file_paths": simple_files or files,
    }


def list_topics(info: dict[str, Any]) -> list[dict[str, Any]]:
    out = []
    for item in info.get("topics_with_message_count") or []:
        tm = item.get("topic_metadata") or {}
        out.append(
            {
                "name": tm.get("name", ""),
                "type": tm.get("type", ""),
                "count": int(item.get("message_count") or 0),
            }
        )
    return out


def sqlite_paths(bagdir: Path, info: dict[str, Any]) -> list[Path]:
    """Resolve .db3 paths that stay inside bagdir. Skip `../` and absolute escapes."""
    root = bagdir.resolve()
    out: list[Path] = []
    for rel in info.get("relative_file_paths") or []:
        raw = Path(str(rel))
        cand = raw if raw.is_absolute() else (bagdir / raw)
        try:
            p = cand.resolve()
        except OSError:
            continue
        if p.is_relative_to(root) and p.is_file():
            out.append(p)
    if out:
        return out
    return [p for p in bagdir.glob("*.db3") if p.resolve().is_relative_to(root) and p.is_file()]


def iter_messages_mcap(bagdir: Path) -> Iterator[tuple[int, str, str, bytes]]:
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as e:
        raise FileNotFoundError(
            f"mcap bag at {bagdir} needs `pip install rosbags` ({e})"
        ) from e
    with AnyReader([bagdir]) as reader:
        for conn, ts, raw in reader.messages():
            yield int(ts), conn.topic, conn.msgtype, raw


def iter_messages(bagdir: Path) -> Iterator[tuple[int, str, str, bytes]]:
    info = read_metadata(bagdir)
    ident = str(info.get("storage_identifier") or "").lower()
    mcaps = list(bagdir.glob("*.mcap"))
    if ident == "mcap" or mcaps:
        yield from iter_messages_mcap(bagdir)
        return
    dbs = sqlite_paths(bagdir, info)
    if not dbs:
        raise FileNotFoundError(
            f"no sqlite3 .db3 in {bagdir}; if this is mcap, pip install rosbags"
        )
    import heapq

    def _one(db: Path) -> Iterator[tuple[int, int, str, str, bytes]]:
        con = sqlite3.connect(f"file:{db.as_posix()}?mode=ro", uri=True)
        try:
            topics = {
                row[0]: (row[1], row[2])
                for row in con.execute("SELECT id, name, type FROM topics")
            }
            for topic_id, ts, mid, blob in con.execute(
                "SELECT topic_id, timestamp, id, data FROM messages ORDER BY timestamp, id"
            ):
                name, typ = topics.get(topic_id, ("", ""))
                yield int(ts), int(mid), name, typ, blob
        finally:
            con.close()

    merged = heapq.merge(*(_one(db) for db in dbs))
    for ts, _mid, name, typ, blob in merged:
        yield ts, name, typ, blob


def write_bag(bagdir: Path, rows: list[tuple[str, str, int, bytes]]) -> None:
    """rows: (topic, type, timestamp_ns, cdr_blob). Humble-like metadata v5."""
    bagdir.mkdir(parents=True, exist_ok=True)
    db = bagdir / "bag_0.db3"
    if db.exists():
        db.unlink()
    con = sqlite3.connect(db)
    con.execute(
        "CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, "
        "type TEXT NOT NULL, serialization_format TEXT NOT NULL, "
        "offered_qos_profiles TEXT NOT NULL)"
    )
    con.execute(
        "CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, "
        "timestamp INTEGER NOT NULL, data BLOB NOT NULL)"
    )
    topic_ids: dict[str, int] = {}
    counts: dict[str, int] = {}
    types: dict[str, str] = {}
    for name, typ, ts, blob in rows:
        if name not in topic_ids:
            cur = con.execute(
                "INSERT INTO topics(name, type, serialization_format, offered_qos_profiles) "
                "VALUES (?,?,?,?)",
                (name, typ, "cdr", ""),
            )
            topic_ids[name] = int(cur.lastrowid)
            types[name] = typ
            counts[name] = 0
        con.execute(
            "INSERT INTO messages(topic_id, timestamp, data) VALUES (?,?,?)",
            (topic_ids[name], int(ts), blob),
        )
        counts[name] += 1
    con.commit()
    con.close()
    stamps = [r[2] for r in rows] or [0]
    duration = max(stamps) - min(stamps)
    lines = [
        "rosbag2_bagfile_information:",
        "  version: 5",
        "  storage_identifier: sqlite3",
        f"  duration:\n    nanoseconds: {int(duration)}",
        f"  starting_time:\n    nanoseconds_since_epoch: {int(min(stamps))}",
        f"  message_count: {len(rows)}",
        "  topics_with_message_count:",
    ]
    for name, tid in topic_ids.items():
        lines.append("    - topic_metadata:")
        lines.append(f"        name: {name}")
        lines.append(f"        type: {types[name]}")
        lines.append("        serialization_format: cdr")
        lines.append('        offered_qos_profiles: ""')
        lines.append(f"      message_count: {counts[name]}")
    lines.append("  compression_format: \"\"")
    lines.append("  compression_mode: \"\"")
    lines.append("  relative_file_paths:")
    lines.append("    - bag_0.db3")
    (bagdir / "metadata.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")
