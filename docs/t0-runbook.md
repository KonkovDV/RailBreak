# T0 runbook

Executable protocol from [hackathon-plan.md](hackathon-plan.md) §7 phase 1.
Clock starts when the organiser bag is on disk (`T0`). Offline T0 does not
need Humble. This Windows host ran Humble in Docker; log:
[ros-rehearsal.md](ros-rehearsal.md).

Drop the bag in `data/bags/<run>/` (gitignored). Then:

```bash
python tools/eval/inspect_bag.py data/bags/<run>
python tools/hackathon/t0.py data/bags/<run> --out reports/t0 ^
  --ukf standalone/build/Release/replay_ukf.exe
```

The same command on a synth run directory (no Humble) is the Phase-0 dry-run:

```bash
python tools/hackathon/t0.py synth/sample/mismatch_r0 --out reports/t0-synth ^
  --ukf standalone/build/Release/replay_ukf.exe
```

Outputs include `inspect.txt` (bags only), `data-contract.md`, `splits/`,
`identify.yaml` from train, `inject/catalog.json` plus `val_<fault>.csv` and
`full_val_<fault>.csv` (wheels mutated on val indices of the **full**
recording; test is not injected), `physics_qa.json`, `observability.json`,
`figures/` (Stanford, heatmap, ride, interval, openloop, timing, dropout), `m4_probe.json`,
`davis_probe.json` (well-posed generator, not the organiser bag),
`tune.json` (val \(q_v\), not a default). With `--ukf` also
`bench_v0.json` / `bench_v1.json` / `integrity.json` / `timing.json`.
Provenance is SHA-256 of the input plus `git rev-parse HEAD`. Do not copy
seed-42 HMI into organiser slots.

On the Humble image the same recording is replayed through the nodes
(`-T` on Windows; `set +u` before sourcing setup.bash):

```bash
docker compose run --rm -T -e BAG=/data/bags/<run> tram_dr \
  bash /opt/tram_dr/tools/hackathon/ros_rehearsal_inner.sh
```

## Clock

| Time | Action | Output |
| --- | --- | --- |
| T0+30 min | Questions in [organizer-questions.md](organizer-questions.md); ingest starts | `reports/t0/run/` |
| T0+2 h | Freeze [data-contract.md](data-contract.md) from `reports/t0/data-contract.md` | observed units only |
| T0+3 h | Splits (local, not git) | `reports/t0/splits/` |
| T0+4 h | Table v0: naive / plant / complementary / UKF, correct `brake_source` | `bench_v0.json` |
| T0+8 h | Identify on train; `identify.yaml`; physics QA; M6 gramian | `identify.yaml`, `physics_qa.json` |
| same day | M9 Stanford on val after flags chosen; one test pass is  | `integrity.json` |

## Physical checks before any UKF tune

Same-day plots live in `physics_qa.json`: \(a\) vs \(n\); coast \(a\)–\(v\);
brake vs deceleration; \(\omega_i/\omega_j\); standstill \(\omega\) noise;
lag of \(\omega\) vs \(v_{gt}\).

## Red lines

- No GNSS / IMU / lidar in the filter (`tools/eval/no_gnss_scan.py`).
- Test is unused until  12:00.
- Organiser files are not committed.
- Humble graph on this Windows host: [ros-rehearsal.md](ros-rehearsal.md).
  Organiser bag still arrives .
