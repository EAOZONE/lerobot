# Completed work

This is the completed-work ledger for the VLA failure-detection study. Detailed evidence
and caveats live in [`WEEK1_REPORT.md`](./WEEK1_REPORT.md),
[`WEEK2_REPORT.md`](./WEEK2_REPORT.md), [`SESSION_2026-07-26.md`](./SESSION_2026-07-26.md),
and [`SESSION_2026-07-27.md`](./SESSION_2026-07-27.md).

## Week 1 feasibility gate — passed 25 July 2026

- SO-101 leader/follower teleoperation verified end to end.
- Feetech position, velocity, load, current, voltage, and temperature registers verified.
- Telemetry block read implemented as one shared-timestamp bus transaction.
- Single-key event markers and live grip confirmation added to the logger.
- Three collision, three slip, and three clean signature runs collected.
- Three matched clear/obstacle trajectory pairs collected.
- Current selected as the primary signal; load retained only as corroboration.
- Latched-alarm diagnosis and safe torque-enable checks implemented.
- Two-camera recording measured at 29.95 Hz.
- Recorded observation schema frozen at 30 dimensions:
  `[pos ×6, load ×6, current ×6, velocity ×6, voltage ×6]`.
- Policy action remains six commanded positions.
- `TruncateStateStep(keep=6)` established as the Arm 1/2 policy switch.

Artifacts:

- `probe_bus.py`
- `scan_registers.py`
- `diagnose.py`
- `feetech_block.py`
- `log_teleop_telemetry.py`
- `plot_signatures.py`
- `gate_analysis.py`
- `so_follower_telemetry.py`
- `record_with_telemetry.py`

## Causal D0 validation — passed 26 July 2026

- Offline smoothing replaced with causal trailing windows.
- First threshold-crossing time implemented for lead-time measurement.
- D0's conditioned current-drop rule retained complete Week 1 slip/clean separation.
- Best tested causal window was 10 frames; useful range fixed at 5–10 frames.
- Command-stationarity interval corrected to cover the smoothing window.
- The upper window bound was traced to grasp duration rather than noise.
- The false +5.11 s `slip_a` lead was identified as an earlier unmarked slip.
- Corpus requirement added: record grasp duration per episode.

Artifact: `causal_eval.py`.

## Free-space current residual, D0r — built 26 July 2026

- Command-conditioned ridge model implemented using a physically motivated basis.
- Model trained only on no-contact free-space runs.
- Warm-up frames made explicitly unscorable.
- Per-joint held-out residual floors stored in the model.
- All three Week 1 matched-pair collisions detected without using the clear twin at
  runtime.
- `pair1_clear` stayed below threshold.
- The known `clean_a` p99 false positive remains visible and documented.
- Model and feature construction require only NumPy/Pandas at scoring time.

Artifacts:

- `freespace_model.py`
- `models/freespace.npz`
- `runs/freespace_01.csv`
- `runs/freespace_payload_01.csv`

The p99 choice was selected after viewing trajectory-A results, so the new-trajectory
evaluation has been pre-registered in
[`D0R_HOLDOUT_PROTOCOL.md`](./D0R_HOLDOUT_PROTOCOL.md).

## Reset/recovery implementation — built 27 July 2026

- Safe halt writes present raw positions as goals before enabling/retaining torque.
- Recorded reset trajectory loads from `reset_home.csv` and ends at home.
- Entry is lift-first: only `shoulder_lift` moves until measured clearance is reached.
- Pan and wrist alignment is feedback-gated, not merely time-gated.
- Clearance timeout aborts before pan motion.
- Default clearance tolerance set to 500 raw ticks after hardware testing.
- Ctrl-C halts at the present pose.
- Optional soak telemetry logging implemented.
- Five-repeat preliminary soak completed.

Artifacts:

- `recovery.py`
- `runs/reset_home.csv`
- `runs/reset_soak.csv`

The full 10-repeat exit test remains unfinished because the preliminary five-repeat soak
predates the final feedback-gated safety sequence. More importantly, later hardware
testing showed that moving `shoulder_lift` alone does not guarantee Cartesian upward
motion from arbitrary poses. The lift-first entry is therefore an implemented experiment,
not a validated general recovery strategy. Path redesign and the exit test remain in
[`NEXT_STEPS.md`](./NEXT_STEPS.md).

## Common detector interface — completed 27 July 2026

- One causal, normalized per-frame score contract implemented; `1.0` is the configured
  operating threshold.
- Duration-only ranking baseline implemented.
- Stateful `OnlineD0.update(...)` implemented for direct 30 Hz use.
- D0 batch scoring verified identical to the established causal rule.
- D0r integrated with warm-up handling and three-frame alert persistence.
- Common parquet score schema established for later D1/D2/D3 detectors.
- Reports now include run/frame AUPRC, false alarms, event-matched recall, lead time,
  recovery-ready fraction, and scoring latency.
- Event attribution prevents unrelated early triggers from becoming true detections.
- D0+ causal feature extraction implemented with 225 columns at window 10.
- Mixed artifact globs skip incompatible CSVs; `--strict` makes mismatches fatal.
- Exact full-directory commands exercised successfully.
- Fourteen focused detector/diagnostic tests pass; Ruff and formatting pass.

Validation totals:

| artifact | result |
|---|---:|
| telemetry runs scored | 17 |
| detector rungs | 3 |
| per-frame score rows | 35,316 |
| D0+ feature rows | 11,772 |
| D0+ columns | 225 |
| focused tests | 14 passed |

Artifacts:

- `detectors.py`
- `test_detectors.py`
- `runs/scores.parquet`
- `D0R_HOLDOUT_PROTOCOL.md`

## Session ritual and invariants established

Run at the start of every robot session and after any stall:

```bash
python research/telemetry/diagnose.py --port /dev/ttyACM0
```

Record session temperature, verify camera alignment, and preserve these invariants:

- never train D0r on contact data;
- never use future frames in detector features;
- never calibrate task difficulty on Arm 3;
- hold episode budgets constant in policy execution time, not wall-clock time;
- retain autonomous successful rollouts as detector negatives;
- do not change the frozen 30-dimensional schema without starting a new corpus version.

## Trajectory-B D0r hold-out — completed, failed specificity 27 July 2026

- New 649-frame trajectory B recorded and replayed into an exactly command-matched
  clear/obstacle pair.
- Locked trajectory-A model and p99 operating point evaluated before fitting B.
- Obstacle response was strong (21.67×) and transferred to `shoulder_lift` and
  `elbow_flex`, not only `shoulder_pan`.
- Clear replay also produced repeated sustained crossings, peaking at 4.32× on
  `wrist_roll`.
- Pre-registered primary pass condition therefore failed.
- Secondary B-fit analysis separated B but became false-positive on A clear, indicating
  trajectory/session-specific free-space coverage.
- Multi-event reporting updated so all markers are retained rather than only the first.

Artifacts:

- `runs/freespace_b_01.csv`
- `runs/pair4_clear.csv`
- `runs/pair4_obstacle.csv`
- `models/freespace_a.npz`
- `models/freespace_b.npz`
- `runs/pair4_scores.parquet`
- `PAIR4_HOLDOUT_RESULT.md`

## `Goal_Position_2` hypothesis eliminated — 28 July 2026

- Two obstacle-free replays of trajectory B recorded with register 71 in the existing block
  transaction; external commands byte-identical across 649 frames.
- Register 71 reads a constant 0 on every motor in every frame of both runs.
- Maximum |Δgoal2| through the wrist-reversal transient that drives the pair4-clear false
  alarm is 0.0 ticks.
- The servo's internal/interpolated setpoint therefore cannot explain repeat-dependent
  current variation, and the remaining candidates (friction, controller state, thermal
  state) are not observable from the bus.
- Confirms rollout-level calibration as the only route, rather than an additional feature.

Artifacts:

- `analyze_goal2.py`
- `runs/clear_goal2_a.csv`
- `runs/clear_goal2_b.csv`

Confirmed directly on hardware the same day with `probe_goal2.py`: at a nonzero standstill
pose (all six motors 710–3692 ticks) register 71 read 0 on every motor, and 292 samples
over 10 s of motion produced one distinct value per motor: 0. This used LeRobot's
per-register `sync_read`, a different code path from the block read, so two independent
reads agree while the same path returns valid `Present_Position` values. The register is
not misread; it is not written. `verify_block_read` could not have caught this — its
30-tick `goal2` tolerance passes a 0-versus-0 agreement silently.

**The hypothesis is closed. No further `goal2` work is outstanding.**

## D0r frozen and trajectory C pre-registered — 28 July 2026

- Model and calibration procedure frozen in
  [`D0R_FROZEN_SPEC.md`](./D0R_FROZEN_SPEC.md) with reference file hashes at commit
  `51494b82`.
- Procedure frozen, fitted model explicitly not: the deployable model is refit on corpus
  training data, and no development `.npz` is eligible.
- Operating point declared as per-rollout conformal at α=0.05 primary and α=0.10 secondary;
  frame percentiles are inadmissible as an operating point.
- Minimum calibration size stated: 19 independent clear rollouts at α=0.05, 9 at α=0.10.
- Corpus annotation fields required for calibration specified, including the rule that a
  "clear" rollout is verified from video and matched telemetry, never inferred from
  detector silence.
- Trajectory C pre-registered in
  [`TRAJECTORY_C_PROTOCOL.md`](./TRAJECTORY_C_PROTOCOL.md): primary pass condition,
  ≥150-tick physical ground truth, abstention accounting, post-hoc rules, and the analysis
  commands written before the data exists.
- C separated from the false-alarm claim: a matched pair is a sensitivity check, and the
  per-rollout false-alarm rate comes from the corpus calibration split only.

Artifacts:

- `D0R_FROZEN_SPEC.md`
- `TRAJECTORY_C_PROTOCOL.md`
- `probe_goal2.py`
