#!/usr/bin/env bash
# Provenance gate for the improvement baseline.
# This script does not play a bag, does not call docker, and does not write metrics.
# A silent success would look like a replay. Exit 2 until a real replay exists.
set -euo pipefail

root="$(cd "$(dirname "$0")/../.." && pwd)"
echo "RailBreak baseline replay was not run."
echo "Audit: ${root}/docs/improvement/audit_baseline.md"
echo "Recorded numbers only: ${root}/evidence/baseline.json"
echo "HEAD: $(git -C "${root}" rev-parse HEAD)"
echo "Submission tag submission-2026-09-27 -> $(git -C "${root}" rev-parse submission-2026-09-27)"
exit 2
