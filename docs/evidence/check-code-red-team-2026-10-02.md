# Red-team audit of supplied checker bundle and evidence

Date: 2026-10-02 (Europe/Moscow)

This record audits the files supplied in the conversation. It is evidence
about the supplied bundle, not a claim that the bundle is part of the
submission package.

## Inputs and hashes

| Artifact | SHA-256 |
|---|---|
| `check-code-with-bag.zip` | `a48bfb5acfaafbb29fa423164ae0bffda114f810bcd456acbaf9cdd76e626a3f` |
| `30618_88aea4d9_0.db3` | computed during the audit; keep the attachment hash together with this record when archiving the evidence |
| `таллинская - щукинская.json` | `7a402fcd426a607ad2f6cf2fca7dbf38cf2e9a171cd5034c61eb3d71bc0e53ba` |
| `щукинская - таллинская.json` | `036e2d746b87daa6f060485c93b2b8ef942a15f83d357ac252313f84b3ab50c9` |

The archive contains a ROS 2 Humble checker, helper scripts, one SQLite
rosbag2 recording, and a reduced `tram_vehicle_msgs` package. The reduced
message package contains `VelocitySensor.msg`, but not
`DriverControllerCommand.msg`.

## Triage result

### P0 — reference-leaking relay must not be used as a solution

`check-code/scripts/relay_result.py` subscribes to
`/localization/kinematic_state`, which is the reference odometry topic in the
same bag. Its position callback republishes that reference message directly as
`/result/position` and constructs `/result/velocity` from
`reference.twist.twist.linear.x`. The callback for the actual front-bogie
velocity input is a no-op.

This is an oracle/relay, not a model-based estimator. It can produce near-zero
checker error while reading the answer topic. Any score obtained with this
script is invalid evidence for RailBreak and must be excluded from metrics,
benchmarks, and release claims. The script is supplied evidence, not part of
the RailBreak runtime; the corrective action is to quarantine it and require a
causality/input-dependency review for any checker helper.

### P1 — rosbag metadata is internally inconsistent

The SQLite payload and the top-level metadata agree:

- total messages: `117976`;
- topic counts sum to `117976`;
- start timestamp and duration match the SQLite extrema;
- all eight topic streams are timestamp-monotonic in insertion order;
- the offline CDR reader decoded all sampled message types without a parse
  exception.

The per-file metadata does not agree:

- `files[0].message_count` in `metadata.yaml`: `166929`;
- actual `messages` row count in `30618_88aea4d9_0.db3`: `117976`.

This is a bag-integrity defect. It does not change the rows read by a direct
SQLite replay, but it makes `ros2 bag info`/metadata-based provenance
ambiguous and must fail a release evidence gate. The repository now contains
`tools/eval/audit_rosbag_integrity.py`, which detects this class of mismatch
without modifying the bag.

### P1 — official checker is not a complete acceptance gate

The supplied checker computes independently synchronized RMSE/max errors for
velocity and XYZ position. It does not by itself establish:

- output coverage or output rate;
- input-to-output latency p50/p95/p99/max;
- header timestamp monotonicity;
- one-to-one reference pairing or duration-weighted exposure;
- frame/child-frame correctness beyond the selected fields;
- causality, i.e. that the result did not read the reference topic.

Use the checker only as one metric component. Combine it with the RailBreak
fail-closed acceptance checks, raw topic/type inspection, latency probe,
provenance, and an explicit no-reference-leak audit.

## Route JSON audit

Both supplied route files are structurally valid single paths with contiguous
indices and finite point fields:

- Tallinnaya → Shchukinskaya: 4709 points, about 4708.562 m;
- Shchukinskaya → Tallinnaya: 4710 points, about 4708.927 m;
- point spacing is approximately 1 m;
- no large internal tangent discontinuity was observed.

These files were not silently treated as the authoritative RailBreak map.
They need an explicit frame/route provenance decision before being used to
change `ring.csv`, `meta.yaml`, or estimator parameters.

## Reproducible commands

From a checkout with Python and NumPy/PyYAML:

```bash
python3 tools/eval/audit_rosbag_integrity.py /path/to/bag
python3 tools/eval/audit_rosbag_integrity.py /path/to/bag --json
python3 tools/eval/inspect_bag.py /path/to/bag
```

The integrity audit is intentionally fail-closed. It reports mismatches and
never rewrites a supplied recording. The supplied bag is expected to fail only
on its per-file message count until its metadata is corrected by the data
owner or by a reviewed conversion.

## Verification boundary

The sandbox did not have ROS 2 Humble, `rclpy`, `rosbag2_py`, Docker, or CMake.
Therefore no claim is made here about a full ROS build, DDS QoS compatibility,
executor behavior, or live replay. The host-only core regression and Python
contract tests remain separate from this evidence audit.
