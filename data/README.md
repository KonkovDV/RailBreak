# Organiser rosbag2 drops here. Not in git.

From the repo root:

```bash
python tools/eval/inspect_bag.py data/bags/run01 --write-yaml tram_dr_localization/config/customer_topics.yaml
# writes adapter remap and, if /tram/* is Int8/Float64, estimator DDS types
# launch loads that file onto both topic_adapter and state_estimator
python tools/eval/run_bag.py data/bags/run01 --ukf standalone/build/replay_ukf --baselines
docker compose run --rm -e BAG=/data/bags/run01 tram_dr
```

Without GT the checker prints `ENVELOPE_GT skipped: no GT` (exit 0 is not
an envelope pass). Add `--require-gt` only when the bag actually contains `/gt/*`
or you have mapped lidar localization to GT **offline**. Default vehicle for
`run_bag.py` is Львёнок (4 axles).

If lidar pose has a usable $z$:

```bash
python tools/eval/profile_from_bag.py data/bags/run01 --out data/route_10_profile.csv
```

Offline only — never a filter input.
