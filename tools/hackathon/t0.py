"""T0 runbook: inspect → ingest → contract → splits → ident → bench → M6/M9/M10.

Does not import the UKF. replay_ukf is a subprocess. Organiser files stay
out of git. Seed-42 metrics.md numbers are not copied into organiser slots.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "eval"))
sys.path.insert(0, str(ROOT / "tools" / "ingest"))
sys.path.insert(0, str(ROOT / "tools" / "ident"))
sys.path.insert(0, str(ROOT / "tools" / "hackathon"))

from ingest import ingest, ingest_csv, write_run_dir  # noqa: E402
from identify import identify, to_replay_yaml, to_yaml  # noqa: E402
from fit_residual import fit as fit_residual, to_yaml as residual_to_yaml  # noqa: E402
from inject import FAULTS, inject_on_indices, inject_rows  # noqa: E402
from inspect_bag import format_report, guess_roles, probe_bag  # noqa: E402
from check_envelope import check_rows  # noqa: E402
from bench import methods_from_csv, score_method  # noqa: E402
from analysis import (  # noqa: E402
    empirical_gramian,
    empirical_gramian_regimes,
    interval_coverage,
    parse_tick_line,
    physics_qa,
    scale_protection,
    stanford,
    write_json,
)
from figures import (  # noqa: E402
    dropout_svg,
    heatmap_svg,
    interval_svg,
    openloop_svg,
    ride_svg,
    stanford_svg,
    timing_svg,
)
from m4 import probe_model_mismatch  # noqa: E402
from davis import probe_davis_ident  # noqa: E402
from tune import tune_qv  # noqa: E402
from protocol import (  # noqa: E402
    csv_to_bag,
    data_contract_markdown,
    split_indices,
    write_split_csvs,
)


def _load_jsonl(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def _load_csv_rows(path: Path) -> list[dict]:
    with path.open(encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def _write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = list(rows[0].keys()) if rows else ["t_s"]
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for row in rows:
            w.writerow({k: row.get(k, "") for k in keys})


def _replay(ukf: Path, csv_path: Path, jsonl: Path,
            extra: list[str] | None = None) -> tuple[list[dict], dict, str]:
    cmd = [str(ukf), str(csv_path), str(jsonl)]
    if extra:
        cmd.extend(extra)
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise SystemExit(f"replay_ukf rc={proc.returncode}: {proc.stderr}")
    return _load_jsonl(jsonl), parse_tick_line(proc.stderr or ""), proc.stderr or ""


def _score_window(name: str, s: list[float], v: list[float], s_gt: list[float],
                  v_gt: list[float], t: list[float], t0: float, t1: float,
                  pss: list[float] | None = None) -> dict:
    idx = [i for i, ti in enumerate(t) if t0 - 1e-9 <= ti <= t1 + 1e-9]

    def take(xs: list[float]) -> list[float]:
        return [xs[i] for i in idx if i < len(xs)]

    return score_method(
        name, take(s), take(v), take(s_gt), take(v_gt), take(t),
        take(pss) if pss is not None else None,
    )


def _sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _git_head() -> str:
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=ROOT,
            text=True,
            stderr=subprocess.DEVNULL,
        )
        return out.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def provenance(src: Path) -> dict:
    rec: dict = {
        "src": str(src.resolve()),
        "git": _git_head(),
        "command": "python tools/hackathon/t0.py",
    }
    if src.is_file():
        rec["sha256"] = _sha256_file(src)
        return rec
    if not src.is_dir():
        return rec
    hashes = {}
    for name in ("metadata.yaml", "filter.csv", "gt.jsonl", "meta.json"):
        p = src / name
        if p.is_file():
            hashes[name] = _sha256_file(p)
    db3 = sorted(src.glob("*.db3"))
    if db3:
        hashes[db3[0].name] = _sha256_file(db3[0])
    rec["sha256"] = hashes
    return rec


def attach_gt(rows: list[dict], gt: list[dict]) -> list[dict]:
    """Join gt.jsonl onto filter rows by 1 ms stamp. Does not invent GT."""
    if not rows or not gt:
        return rows
    by_t = {round(float(g.get("t", g.get("t_s", 0.0))), 3): g for g in gt}
    for i, row in enumerate(rows):
        try:
            t = round(float(row.get("t_s", row.get("t", i * 0.02))), 3)
        except (TypeError, ValueError):
            continue
        g = by_t.get(t)
        if g is None:
            continue
        if row.get("gt_s") in (None, "") and g.get("s") is not None:
            row["gt_s"] = g["s"]
        if row.get("gt_v") in (None, "") and g.get("v") is not None:
            row["gt_v"] = g["v"]
    return rows


def inspect_bag_to_dir(bag: Path, out: Path) -> dict:
    probe = probe_bag(bag)
    text, rc, yaml_text = format_report(probe)
    (out / "inspect.txt").write_text(text, encoding="utf-8")
    (out / "customer_topics.guess.yaml").write_text(yaml_text, encoding="utf-8")
    slim = {
        "storage": probe["info"].get("storage_identifier"),
        "message_count": probe["info"].get("message_count"),
        "topics": [
            {"name": t["name"], "type": t["type"], "count": t["count"]}
            for t in probe["topics"]
        ],
        "roles": guess_roles(probe["stats"]),
        "format_rc": rc,
    }
    write_json(out / "inspect.json", slim)
    return slim


def write_inject_catalog(rows: list[dict], val_idx: list[int], out: Path) -> dict:
    """Plan §6.4: freeze the fault set. Mutate val indices on a full copy.

    Isolated `val_<fault>.csv` clips are for inspection. Scoring uses
    `full_val_<fault>.csv` so the filter does not cold-start at the val cut.
    Test indices are never mutated.
    """
    inj = out / "inject"
    inj.mkdir(parents=True, exist_ok=True)
    catalog = {
        "seed": 42,
        "applied_to": "val indices on the full recording",
        "test": "unused until freeze",
        "faults": sorted(FAULTS),
        "n_val": len(val_idx),
        "n_full": len(rows),
    }
    write_json(inj / "catalog.json", catalog)
    val_rows = [rows[i] for i in val_idx]
    keys = list(rows[0].keys()) if rows else ["t_s"]
    if val_rows:
        for name in sorted(FAULTS):
            faulted = inject_rows(val_rows, name, seed=42)
            path = inj / f"val_{name}.csv"
            with path.open("w", encoding="utf-8", newline="") as f:
                w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
                w.writeheader()
                for row in faulted:
                    w.writerow({k: row.get(k, "") for k in keys})
            full = inject_on_indices(rows, val_idx, name, seed=42)
            _write_csv(inj / f"full_val_{name}.csv", full)
    return catalog


def run_t0(src: Path, out: Path, ukf: Path | None = None) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    run_dir = out / "run"
    inspect_info: dict = {}
    if src.is_dir() and (src / "metadata.yaml").is_file():
        inspect_info = inspect_bag_to_dir(src, out)

    if src.is_dir() and (src / "filter.csv").is_file():
        rows, detected = ingest_csv(src / "filter.csv")
        meta = dict(detected)
        if (src / "meta.json").is_file():
            file_meta = json.loads((src / "meta.json").read_text(encoding="utf-8"))
            for k, v in file_meta.items():
                if k in meta and meta[k] not in (None, "", "TBD"):
                    continue
                meta[k] = v
        meta["source"] = str(src)
        gt_src = src / "gt.jsonl"
        if gt_src.is_file():
            rows = attach_gt(rows, _load_jsonl(gt_src))
        write_run_dir(run_dir, rows, meta)
        if gt_src.is_file():
            (run_dir / "gt.jsonl").write_text(
                gt_src.read_text(encoding="utf-8"), encoding="utf-8"
            )
    else:
        rows, meta = ingest(src)
        write_run_dir(run_dir, rows, meta)

    # Prefer the persisted run (hold-last already applied) and re-join GT.
    rows = _load_csv_rows(run_dir / "filter.csv")
    rows = attach_gt(rows, _load_jsonl(run_dir / "gt.jsonl"))
    prov = provenance(src)
    write_json(out / "provenance.json", prov)

    contract = data_contract_markdown(rows, meta)
    (out / "data-contract.md").write_text(contract, encoding="utf-8")
    splits = split_indices(rows)
    split_paths = write_split_csvs(rows, splits, out / "splits")
    val_idx = splits.get("val", [])
    inject_meta = write_inject_catalog(rows, val_idx, out)
    qa = physics_qa(rows)
    write_json(out / "physics_qa.json", qa)
    ident = identify(split_paths["train"], mass=28000.0)
    (out / "identify.yaml").write_text(to_yaml(ident), encoding="utf-8")
    replay_yaml, skipped_ident = to_replay_yaml(ident)
    (out / "identify_replay.yaml").write_text(replay_yaml, encoding="utf-8")
    write_json(out / "identify.json", {**ident, "replay_skipped": skipped_ident})
    r0_fit = ident.get("radii", {}).get("r0_mean")
    try:
        r0_fit = float(r0_fit)
    except (TypeError, ValueError):
        r0_fit = 0.35
    if not math.isfinite(r0_fit) or r0_fit < 0.25 or r0_fit > 0.45:
        r0_fit = 0.35
    residual_fit = fit_residual(split_paths["train"], r0_m=r0_fit)
    (out / "residual.yaml").write_text(residual_to_yaml(residual_fit), encoding="utf-8")
    write_json(out / "residual.json", residual_fit)
    gram = empirical_gramian()
    gram["regimes"] = empirical_gramian_regimes()
    write_json(out / "observability.json", gram)
    bag = csv_to_bag(rows, out / "synth.bag")
    fig_dir = out / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    m4 = probe_model_mismatch(out / "m4_probe")
    write_json(out / "m4_probe.json", m4)
    davis = probe_davis_ident(out / "davis_probe")
    write_json(out / "davis_probe.json", davis)
    openloop_svg(
        [
            {"name": "physics", **(m4.get("physics") or {})},
            {"name": "residual", **(m4.get("residual") or {})},
        ],
        fig_dir / "openloop.svg",
    )
    timing = {"p50_us": float("nan"), "p99_us": float("nan"), "max_us": float("nan"), "n": 0}
    table: list[dict] = []
    integrity: dict = {}
    identified: dict = {}
    interval: dict = {}
    tuned: dict = {}
    heat_cells: list[dict] = []
    val_t0 = val_t1 = float("nan")
    if val_idx:
        val_t0 = float(rows[val_idx[0]].get("t_s", 0.0))
        val_t1 = float(rows[val_idx[-1]].get("t_s", 0.0))
    if ukf is not None and ukf.is_file():
        csv_path = run_dir / "filter.csv"
        est, timing, stderr = _replay(ukf, csv_path, run_dir / "ukf.jsonl")
        (out / "replay_stderr.txt").write_text(stderr, encoding="utf-8")
        write_json(out / "timing.json", timing)
        if math.isfinite(float(timing.get("p50_us", "nan"))):
            timing_svg(timing, fig_dir / "timing.svg")
        gt = _load_jsonl(run_dir / "gt.jsonl")
        t = [float(r.get("t_s", i * 0.02)) for i, r in enumerate(rows)]
        s_gt = [float(g.get("s", "nan")) for g in gt] if gt else []
        v_gt = [float(g.get("v", "nan")) for g in gt] if gt else []
        if s_gt and v_gt:
            for name, (s, v) in methods_from_csv(rows).items():
                table.append(score_method(name, s, v, s_gt, v_gt, t))
            if est:
                s_u = [float(e.get("s", "nan")) for e in est]
                v_u = [float(e.get("v", "nan")) for e in est]
                pss = [float(e.get("p_ss", "nan")) for e in est]
                table.append(score_method("ukf", s_u, v_u, s_gt, v_gt, t, pss))
                integrity = stanford(est, gt)
        write_json(out / "bench_v0.json", table)
        env = check_rows(gt + est, require_gt=False)
        envelope = {
            "n_ok": env.n_ok,
            "n_hmi": env.n_hmi,
            "notes": env.notes,
            "n_hits": len(env.hits),
        }
        write_json(out / "envelope.json", envelope)
        kappa = float((integrity.get("overbound") or {}).get("kappa_ob", float("nan")))
        if est and math.isfinite(kappa):
            integrity["after_overbound"] = stanford(scale_protection(est, kappa), gt)
        integrity["envelope"] = envelope
        write_json(out / "integrity.json", integrity)
        ident_yaml = out / "identify_replay.yaml"
        if ident_yaml.is_file() and "wheel_radius_m" in ident_yaml.read_text(encoding="utf-8"):
            est_id, _, _ = _replay(
                ukf, csv_path, run_dir / "ukf_identified.jsonl",
                extra=["--params", str(ident_yaml)],
            )
            env_id = check_rows(gt + est_id, require_gt=False)
            identified = {
                "stanford": stanford(est_id, gt),
                "envelope": {
                    "notes": env_id.notes,
                    "n_ok": env_id.n_ok,
                    "n_hmi": env_id.n_hmi,
                },
            }
            if s_gt and v_gt and est_id:
                identified["rmse_s"] = score_method(
                    "ukf_identified",
                    [float(e.get("s", "nan")) for e in est_id],
                    [float(e.get("v", "nan")) for e in est_id],
                    s_gt, v_gt, t,
                    [float(e.get("p_ss", "nan")) for e in est_id],
                )["rmse_s"]
            write_json(out / "bench_v1.json", identified)
            est_m3, _, _ = _replay(
                ukf, csv_path, run_dir / "ukf_interval.jsonl",
                extra=["--params", str(ident_yaml), "--set", "interval_mode=monitor"],
            )
            interval = interval_coverage(est_m3, gt)
            write_json(out / "interval.json", interval)
            interval_svg(est_m3, gt, fig_dir / "interval.svg")
        res_yaml = out / "residual.yaml"
        ident_yaml = out / "identify_replay.yaml"
        if res_yaml.is_file() and residual_fit.get("n", 0) >= 20:
            extra_r = ["--params", str(res_yaml)]
            if ident_yaml.is_file() and "wheel_radius_m" in ident_yaml.read_text(encoding="utf-8"):
                extra_r = ["--params", str(ident_yaml), "--params", str(res_yaml)]
            est_r, _, _ = _replay(ukf, csv_path, run_dir / "ukf_residual.jsonl", extra=extra_r)
            if s_gt and v_gt and est_r:
                sc_r = score_method(
                    "ukf_residual",
                    [float(e.get("s", "nan")) for e in est_r],
                    [float(e.get("v", "nan")) for e in est_r],
                    s_gt, v_gt, t,
                    [float(e.get("p_ss", "nan")) for e in est_r],
                )
                write_json(out / "bench_v2_residual.json", sc_r)
        if math.isfinite(val_t0) and len(rows) >= 200:
            extra_t = []
            if ident_yaml.is_file() and "wheel_radius_m" in ident_yaml.read_text(encoding="utf-8"):
                extra_t = ["--params", str(ident_yaml)]
            tuned = tune_qv(
                ukf, csv_path, gt, out / "tune", val_t0, val_t1,
                extra=extra_t, budget=6,
            )
            write_json(out / "tune.json", tuned)
        if est:
            stanford_svg(est, gt, fig_dir / "stanford.svg")
            ride_svg(est, gt, fig_dir / "ride.svg")
            if math.isfinite(kappa):
                stanford_svg(
                    scale_protection(est, kappa), gt, fig_dir / "stanford_overbound.svg"
                )
        inj_dir = out / "inject"
        for fault in sorted(FAULTS):
            fcsv = inj_dir / f"full_val_{fault}.csv"
            if not fcsv.is_file() or not math.isfinite(val_t0):
                continue
            frows = _load_csv_rows(fcsv)
            fest, _, _ = _replay(ukf, fcsv, inj_dir / f"ukf_{fault}.jsonl")
            for name, (s, v) in methods_from_csv(frows).items():
                m = _score_window(name, s, v, s_gt, v_gt, t, val_t0, val_t1)
                heat_cells.append({"fault": fault, "method": name, "rmse_s": m.get("rmse_s")})
            if fest and s_gt:
                m_u = _score_window(
                    "ukf",
                    [float(e.get("s", "nan")) for e in fest],
                    [float(e.get("v", "nan")) for e in fest],
                    s_gt, v_gt, t, val_t0, val_t1,
                    [float(e.get("p_ss", "nan")) for e in fest],
                )
                heat_cells.append({"fault": fault, "method": "ukf", "rmse_s": m_u.get("rmse_s")})
            if fault == "dropout_all" and fest and gt:
                pts = []
                by_t = {round(float(g.get("t", 0.0)), 3): g for g in gt}
                for e in fest:
                    te = float(e.get("t", 0.0))
                    if te + 1e-9 < val_t0:
                        continue
                    g = by_t.get(round(te, 3))
                    if g is None:
                        continue
                    try:
                        es = abs(float(e.get("s")) - float(g.get("s", g.get("gt_s"))))
                    except (TypeError, ValueError):
                        continue
                    pts.append({"t": te - val_t0, "e_s": es})
                if pts:
                    dropout_svg(pts, fig_dir / "dropout.svg")
        if heat_cells:
            write_json(out / "inject_val_bench.json", heat_cells)
            heatmap_svg(heat_cells, fig_dir / "heatmap_fault_method.svg")
    elif val_idx:
        # No UKF: still emit baseline heatmap on injected val windows.
        t = [float(r.get("t_s", i * 0.02)) for i, r in enumerate(rows)]
        s_gt = []
        v_gt = []
        for r in rows:
            try:
                s_gt.append(float(r["gt_s"]) if r.get("gt_s") not in (None, "") else float("nan"))
            except (TypeError, ValueError):
                s_gt.append(float("nan"))
            try:
                v_gt.append(float(r["gt_v"]) if r.get("gt_v") not in (None, "") else float("nan"))
            except (TypeError, ValueError):
                v_gt.append(float("nan"))
        if any(math.isfinite(x) for x in s_gt):
            for fault in sorted(FAULTS):
                fcsv = out / "inject" / f"full_val_{fault}.csv"
                if not fcsv.is_file():
                    continue
                frows = _load_csv_rows(fcsv)
                for name, (s, v) in methods_from_csv(frows).items():
                    m = _score_window(name, s, v, s_gt, v_gt, t, val_t0, val_t1)
                    heat_cells.append({"fault": fault, "method": name, "rmse_s": m.get("rmse_s")})
            if heat_cells:
                heatmap_svg(heat_cells, fig_dir / "heatmap_fault_method.svg")
    report = [
        "# T0 report",
        "",
        "Organiser slots that were not observed remain TBD. "
        "Do not paste docs/metrics.md seed-42 numbers here.",
        "",
        f"- source: `{src}`",
        f"- git: `{prov.get('git', 'unknown')}`",
        f"- frames: {len(rows)}",
        f"- split: train {len(splits.get('train', []))} / "
        f"val {len(splits.get('val', []))} / test {len(splits.get('test', []))} "
        f"({splits.get('note', '')})",
        f"- brake_source (ingest): {meta.get('brake_source', 'TBD')}",
        f"- physics n_coast: {qa.get('n_coast')}",
        f"- identify r0_mean: {ident.get('radii', {}).get('r0_mean', 'TBD')}",
        f"- identify A_d: {ident.get('davis', {}).get('A_d', 'TBD')}",
        f"- M1 Davis probe covers_truth: {davis.get('covers_truth')} "
        f"A_d={((davis.get('hat') or {}).get('A_d'))}",
        f"- gramian log10 span: {gram.get('log10_span', 'TBD')} (rank {gram.get('rank_est', 'TBD')})",
        f"- gramian weakest: {gram.get('weakest', 'TBD')}",
        f"- gramian regimes rank: "
        f"T={(gram.get('regimes') or {}).get('traction', {}).get('rank_est', 'TBD')} "
        f"C={(gram.get('regimes') or {}).get('coast', {}).get('rank_est', 'TBD')} "
        f"B={(gram.get('regimes') or {}).get('brake', {}).get('rank_est', 'TBD')}",
        f"- synth bag: `{bag}`",
        f"- inspect topics: {len(inspect_info.get('topics', [])) or 'n/a (not a bag)'}",
        f"- inject faults on val: {len(inject_meta.get('faults', []))}",
        f"- tick p99_us: {timing.get('p99_us', 'TBD')}",
        f"- Stanford HMI-rate (passport): {integrity.get('hmi_rate', 'TBD')}",
        f"- Stanford HMI after κ_ob: "
        f"{(integrity.get('after_overbound') or {}).get('hmi_rate', 'TBD')}",
        f"- envelope notes: {integrity.get('envelope', {}).get('notes', ['TBD'])}",
        f"- identified r0 HMI: {(identified.get('stanford') or {}).get('hmi_rate', 'TBD')}",
        f"- identified n_ok: {(identified.get('stanford') or {}).get('n_ok', 'TBD')}",
        f"- identified rmse_s: {identified.get('rmse_s', 'TBD')}",
        f"- identify replay skipped: {skipped_ident}",
        f"- M3 inside_rate: {interval.get('inside_rate', 'TBD')}",
        f"- M4 e_s_10s physics/residual: "
        f"{(m4.get('physics') or {}).get('e_s_10s', 'TBD')} / "
        f"{(m4.get('residual') or {}).get('e_s_10s', 'TBD')} "
        f"(improved={m4.get('e_s_10s_improved')})",
        f"- residual n: {residual_fit.get('n', 'TBD')}",
        f"- M7 q_v (val, not default): {tuned.get('q_v', 'TBD')} J={tuned.get('J', 'TBD')}",
        f"- kappa_ob: {integrity.get('overbound', {}).get('kappa_ob', 'TBD')}",
        f"- figures: `{fig_dir}`",
        "",
        "Test split was not scored. Next: send docs/organizer-questions.md; "
        "freeze flags on val only; one test pass on .",
        "",
    ]
    (out / "REPORT.md").write_text("\n".join(report), encoding="utf-8")
    summary = {
        "n": len(rows),
        "brake_source": meta.get("brake_source"),
        "timing": timing,
        "integrity": integrity,
        "identified": identified,
        "interval": interval,
        "m4": {"improved": m4.get("e_s_10s_improved"), "n": m4.get("residual_n")},
        "davis": {"covers": davis.get("covers_truth"),
                  "A_d": (davis.get("hat") or {}).get("A_d")},
        "tune": {"q_v": tuned.get("q_v"), "J": tuned.get("J")},
        "git": prov.get("git"),
        "out": str(out),
    }
    write_json(out / "summary.json", summary)
    return summary


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("src", type=Path, help="csv / jsonl / rosbag2 dir / existing run dir")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--ukf", type=Path)
    args = ap.parse_args(argv)
    summary = run_t0(args.src, args.out, args.ukf)
    print(json.dumps(summary, indent=2, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
