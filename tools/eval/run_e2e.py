"""End-to-end gate: replay_ukf on every synth run + independent envelope check.

Runs the standalone UKF binary on each synth/runs/*/filter.csv, merges gt+est
JSONL, and calls check_envelope.py --require-gt. Prints the confidence
distribution per scenario so a vacuous pass (never OK) is visible.

Usage: python tools/eval/run_e2e.py --ukf <path to replay_ukf> [--runs DIR]
Exit 0 = all clean; 2 = checker dirty or replay failed; 1 = usage/missing.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CHECKER = ROOT / "tools" / "eval" / "check_envelope.py"

# Unobservable common-mode scale (wrong r0, d_i agree). Filter stays OK with
# small P_ss — that is the HMI the scenario exists to show. identify_coast.py
# is the mitigation. Do not mix this into model_mismatch.
EXPECTED_HMI = {"mismatch_r0"}

VEHICLE_YAML = {
    "six_axle": ROOT / "tram_dr_localization" / "config" / "vehicle_vityaz_m.yaml",
}

from plot_run import plot_dir  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ukf", type=Path, required=True, help="replay_ukf binary")
    ap.add_argument("--runs", type=Path, default=ROOT / "synth" / "runs")
    args = ap.parse_args(argv)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if not args.ukf.is_file():
        sys.stderr.write(f"missing replay_ukf binary: {args.ukf}\n")
        return 1
    if not args.runs.is_dir():
        sys.stderr.write(f"missing {args.runs}; run tools/synth/generate.py first\n")
        return 1
    rc = 0
    for d in sorted(p for p in args.runs.iterdir() if p.is_dir()):
        if not (d / "filter.csv").is_file():
            continue
        ukf_out = d / "ukf.jsonl"
        cmd = [str(args.ukf), str(d / "filter.csv"), str(ukf_out)]
        veh = VEHICLE_YAML.get(d.name)
        if veh is not None:
            cmd.extend(["--vehicle", str(veh)])
        if "route10" in d.name:
            cmd.extend(
                ["--route", str(ROOT / "tram_dr_localization" / "config" / "route_10.yaml")]
            )
        r = subprocess.run(cmd)
        if r.returncode != 0:
            sys.stderr.write(f"{d.name}: replay_ukf rc={r.returncode}\n")
            rc = 2
            continue
        # Interleave gt/est by t: the checker compares against the last gt
        # record seen, so concatenated files would test est[t] vs gt[end].
        gt_rows = [
            json.loads(line)
            for line in (d / "gt.jsonl").read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
        est_rows = [
            json.loads(line)
            for line in ukf_out.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
        merged_rows = sorted(
            gt_rows + est_rows,
            key=lambda row: (float(row.get("t", 0.0)), 0 if row.get("kind") == "gt" else 1),
        )
        merged = d / "merged.jsonl"
        with merged.open("w", encoding="utf-8", newline="\n") as out:
            for row in merged_rows:
                out.write(json.dumps(row) + "\n")
        c = subprocess.run(
            [sys.executable, str(CHECKER), str(merged), "--require-gt"],
            capture_output=True,
            text=True,
        )
        rows = [
            json.loads(line)
            for line in ukf_out.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
        conf = Counter(str(x.get("confidence", "")) for x in rows)
        print(f"{d.name}: envelope rc={c.returncode} conf={dict(conf)}")
        for line in (c.stdout or "").splitlines():
            if line.startswith("HMI-rate") or line.startswith("missed_path"):
                print(f"  {line}")
        if c.returncode != 0:
            if d.name in EXPECTED_HMI and "ENVELOPE_GT" in (c.stderr or ""):
                print(f"  expected HMI (unobservable d_bar / r0); not a gate fail")
            else:
                sys.stderr.write(c.stderr)
                rc = 2
        try:
            plot_dir(d)
        except Exception as ex:  # pragma: no cover
            sys.stderr.write(f"{d.name}: plot_run {ex}\n")
            rc = 2
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
