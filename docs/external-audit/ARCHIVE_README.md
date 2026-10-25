# RailBreak corrected source archive

This is a corrected source-tree delivery based on RailBreak HEAD
`c6a8c140435abad5b4c3dd2b736e135958c8e716`.

## Quick start

```bash
unzip RailBreak_corrected_source.zip
cd RailBreak_corrected_source

# Host-only core smoke test (no ROS required)
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
  -Irailbreak_backup_odometry/include \
  railbreak_backup_odometry/tools/test_core.cpp \
  -o /tmp/railbreak_test_core
/tmp/railbreak_test_core

python3 -m compileall -q tools scripts
python3 tools/eval/test_docs.py
python3 tools/eval/test_metric_contracts.py
```

## ROS 2 validation

The complete ROS 2 Humble build and rosbag replay require a ROS/Docker environment.
Use the procedure in `AI_WORKPLAN.md`. Do not treat the host-only smoke test as a
replacement for ROS executor, QoS, message-type or latency validation.

## Contents

- `AUDIT_REPORT.md`: independent audit, confirmed corrections and limitations.
- `AI_WORKPLAN.md`: detailed next-step plan for an AI/engineer.
- `railbreak_backup_odometry/`: ROS package, assets, configuration and core tests.
- `tools/`, `scripts/`, `docs/`: existing evaluation and organizer tooling.
- `RELEASE_CHECKSUMS.txt`: release manifest with source-tree digest and archive name. The exact archive SHA-256 is in the adjacent `RailBreak_corrected_source.zip.sha256` file and is repeated in the delivery message.

The archive intentionally contains a source tree, not a bare Git repository.
Git-dependent provenance checks must be run from a normal Git checkout with the
relevant commit objects available. Historical metrics remain labelled as historical;
this delivery does not claim that official rosbag metrics were reproduced.
