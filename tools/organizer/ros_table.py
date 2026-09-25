"""Score every recorded /result/* bag under a directory and print one markdown table.

  python tools/organizer/ros_table.py local/ros_out files/data
Directory names `<id>` or `<id>_<fault>` map to the source bag `<id>`.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


def main() -> int:
    out_dir, src_dir = Path(sys.argv[1]), Path(sys.argv[2])
    rows = []
    for d in sorted(p for p in out_dir.iterdir() if p.is_dir()):
        parts = d.name.split("_")
        src = src_dir / "_".join(parts[:2])
        score = d.with_suffix(".score.json")
        subprocess.run([sys.executable, str(Path(__file__).with_name("score_ros.py")), str(d), str(src),
                        "--out", str(score)], check=True, stdout=subprocess.DEVNULL)
        s = json.loads(score.read_text(encoding="utf-8"))
        res = d.parent / f"{d.name}.res.txt"
        rss = cpu = wall = ""
        if res.is_file():
            kv = dict(x.split("=") for x in res.read_text().split()[1:])
            rss, cpu, wall = kv.get("peak_rss_kb", ""), kv.get("cpu_s", ""), kv.get("wall_s", "")
        rows.append((d.name, s, rss, cpu, wall))
    print("| Запись | 3D RMSE, м | x | y | z | RMSE v, м/с | Покрытие | Гц | Макс. разрыв, с | Дрейф в конце, % | Колбэк max, мкс | Якоря | Пик RSS, МБ | CPU, с |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for name, s, rss, cpu, _wall in rows:
        print(f"| `{name}` | {s['rmse_3d']:.2f} | {s['rmse_x']:.2f} | {s['rmse_y']:.2f} | {s['rmse_z']:.2f} | "
              f"{s['rmse_v_result_velocity']:.3f} | {s['coverage']:.4f} | {s['rate_hz']:.1f} | {s['max_gap_s']:.2f} | "
              f"{s['end_drift_pct']:.3f} | {s['callback_max_us']:.0f} | {s['n_anchor']} | "
              f"{int(rss) / 1024:.1f} | {cpu} |")
    notes = {s["gnss_note"] for _n, s, *_ in rows}
    print(f"\nGNSS: {', '.join(sorted(notes))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
