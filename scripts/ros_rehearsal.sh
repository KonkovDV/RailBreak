#!/usr/bin/env bash
# Phase 0.11: prove Humble launch starts. Does not need the organiser bag.
# AMENT setup.bash is not `set -u` safe (AMENT_TRACE_SETUP_FILES).
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
docker compose build tram_dr
docker compose run --rm -T \
  -e BAG="${BAG:-/data/bags/synth_rehearsal}" \
  tram_dr bash /opt/tram_dr/tools/hackathon/ros_rehearsal_inner.sh
echo "Log the run in docs/ros-rehearsal.md."
