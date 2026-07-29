# Frozen telemetry corpus schema

**Version:** 1 · **Frozen:** 28 July 2026

This document is the single source of truth for data recorded after the Week 2
infrastructure gate. Changing a field, order, unit, or timing definition starts a new
corpus version; old and new episodes must not be silently combined.

## Dataset frame

`observation.state` has exactly 30 float32 values, grouped by field. Within every group,
motor order is `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`,
`gripper`.

| slice | suffix | unit and semantics |
|---|---|---|
| `[0:6]` | `.pos` | calibrated normalized joint position, identical to plain SO-101 |
| `[6:12]` | `.load` | raw signed load; ±1000 is ±100% max torque |
| `[12:18]` | `.current` | raw unsigned current, approximately 6.5 mA/LSB |
| `[18:24]` | `.vel` | raw signed servo velocity |
| `[24:30]` | `.volt` | raw supply voltage in decivolts |

`action` remains six calibrated normalized commanded joint positions in the same motor
order. Cameras are `observation.images.wrist` and `observation.images.overhead`, RGB video
at 30 Hz. The standard `timestamp` remains `frame_index / fps`; it is an indexing clock,
not evidence of real capture timing.

Arm 1/2 training and inference must apply `TruncateStateStep(keep=6)`. Arm 3/4 deliberately
consume all 30 dimensions. Telemetry is never commanded.

## Capture-timing sidecar

Real timing is stored separately at
`<dataset-root>/meta/alignment/episode_NNNNNN.jsonl`, one JSON object per dataset frame.
This keeps timing out of the frozen state vector. Each row contains:

- `schema_version`, `episode_index`, `frame_index`;
- `wall_time_ns`, for cross-process/session identification;
- monotonic `observation_start_ns`, `position_read_end_ns`,
  `telemetry_read_end_ns`, and `observation_end_ns`;
- per-camera `capture_ns` from the camera thread and `returned_ns` from the robot read.

Pending rows are committed only for a saved episode. Clearing a re-recorded episode clears
its rows. The alignment audit requires contiguous frame indices, monotonic non-duplicate
camera captures, no capture/control gap over two periods, and camera-to-telemetry distance
within one 30 Hz period (33.3 ms).

## Units and invariants

- CSV diagnostic logs use raw `pos.*` and may use normalized `goal_pos.*`; their headers and
  producing script determine units. Recovery logs use raw ticks for both.
- `Goal_Position_2` is diagnostic-only and is not a corpus feature.
- Camera timing, labels, split metadata, temperatures, and recovery metadata never enter
  `observation.state`.
- Voltage corroborates severe stalls but is not an independent collision trigger.
- Dataset episodes without a passing sidecar audit are quarantined from the corpus until
  the cause is resolved; synthetic timestamps cannot waive this requirement.

Recovery-enabled rollout evidence is a separate relational sidecar governed by
`RECOVERY_EVIDENCE_SCHEMA.md`. Its stable run/episode/attempt identifiers, policy manifest,
physical recovery CSVs, supervisor events, and terminal outcomes never enter the frozen
30-dimensional observation.
