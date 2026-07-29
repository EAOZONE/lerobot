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

## Policy inference latency measured — 28 July 2026

Measured on the real `predict_action` path with the actual pre/post-processor pipelines,
RTX 4090, `lerobot/smolvla_base`, 6-dim state, 2 cameras at 480×640.

- Mean control step at `n_action_steps=50`: **3.8 ms**, a sustained 264 Hz against a 30 Hz
  requirement. Compute is not a constraint on this hardware.
- Single-step prediction costs 21.6× more (81.9 ms, ~12 Hz) and misses the budget on every
  step, confirming §7.1's warning quantitatively.
- Chunk recompute costs ~83–87 ms, about 2.5 control periods, so the loop stalls every
  `n_action_steps`. Recovery must be budgeted against the worst step, not the mean.
- 6-dim and 30-dim state are identical within noise, because SmolVLA pads state to
  `max_state_dim=32`. H5's "Arm 3 pays nothing at runtime" now has runtime evidence.
- In context, D0 and D0r batch-path costs are 0.3% and 0.05% of a mean control step.

Artifact: `bench_inference.py`.

Not yet measured: 20k-step fine-tune wall-clock on this GPU, and D0r's coverage
nearest-neighbour cost at corpus-scale training-set size.

## Recovery and corpus-contract software — completed offline 28 July 2026

- Replaced the accepted lift-first reset interface with bounded reverse-command replay,
  region-selected waypoint routes, monitored return-home replay, and gripper reopen.
- Added fail-closed checks for route validation/ambiguity, joint range, current, following
  error, phase timeout, and command step; violations halt at measured position.
- Added a supervised teleoperation validator that retains commands actually accepted by
  the follower and writes phase-level telemetry logs.
- Added real capture timing without changing the frozen 30-dimensional state: per-episode
  JSONL sidecars carry state/telemetry times and both camera capture timestamps.
- Added a strict alignment auditor and camera reference/session capture utility.
- Froze `DATA_SCHEMA.md`, `LABELING_GUIDE.md`, and `annotation.schema.json` before corpus
  collection, plus recovery/camera protocols and the operator command runbook.
- Recovery, alignment, and existing detector tests: **23 passed offline**.

Artifacts include `RECOVERY_PROTOCOL.md`, `CAMERA_ALIGNMENT_PROTOCOL.md`,
`ROBOT_DATA_RUNBOOK.md`, `validate_recovery.py`, `alignment_sidecar.py`,
`audit_alignment.py`, and `camera_reference.py`.

Not completed: no recovery route is physically validated; camera mounts/reference images
are not complete; no two-camera alignment episode has passed audit; the supervisor is not
integrated into live rollout; the ten-reset exit test has not run.

## Remaining desk-risk reduction — completed 28 July 2026

- Replaced D0r detector-side coverage lookup with a reusable exact nearest-neighbour index.
  SciPy uses `cKDTree`; installations without SciPy retain an exact bounded-memory NumPy
  fallback rather than allocating the full query × reference × feature tensor.
- At a projected 270k reference frames, measured 177 ms one-time tree build, 0.029 ms per
  isolated query, 0.031 ms/frame for a 100-frame batch, and 140 MiB reference storage.
- Confirmed D0r scores, scorable masks, and triggers unchanged on four development runs;
  documented the hash-only runtime amendment in `D0R_FROZEN_SPEC.md`.
- Created the physical procurement checklist and independent-annotator brief. Neither
  external gate is complete until an order is placed and a person accepts.
- Verified CoRL 2026 workshop status against live pages. Identified the strongest target
  and two backups, but all target deadlines remain unpublished/TBA; weekly tracking is in
  `WORKSHOP_TRACKER.md`.

Artifacts: `coverage_index.py`, `bench_coverage.py`, `D0R_COVERAGE_BENCHMARK.md`,
`PROCUREMENT_CHECKLIST.md`, `ANNOTATOR_BRIEF.md`, `WORKSHOP_TRACKER.md`, and
`DATA_BACKUP_PROTOCOL.md`.

## Arm 1/2 pilot training contract — completed offline 28 July 2026

- Added a research-local training entry point that leaves the 30-dimensional dataset
  intact while giving policy construction and normalization a copied six-dimensional
  state view.
- Inserts `TruncateStateStep(keep=6)` before normalization and serializes it with every
  checkpoint; telemetry recording/evaluation registers the custom step before loading.
- Tests prove source metadata/statistics are not mutated, state statistics are sliced,
  the step is first, and it survives a checkpoint save/load round trip.
- Added the pre-registered T1 pilot procedure and exact demonstration, training, and
  autonomous-evaluation commands with reasons and motion warnings.

Artifacts: `telemetry_policy_bridge.py`, `train_positions_only.py`,
`test_telemetry_policy_bridge.py`, and `PILOT_PROTOCOL.md`.

Not completed: no pilot data exists, no policy has been trained, and all physical pilot
gates remain open.

## Corpus structural preflight — completed offline 28 July 2026

- Added one fail-closed command for the dataset acceptance checks that were previously
  spread across manual instructions.
- Reads every stored Parquet state/action row and verifies widths, finiteness, and counts
  against metadata rather than trusting declarations.
- Checks 30/6-dimensional statistics, required wrist/overhead video declarations and
  nonempty MP4 artifacts, exact episode sidecar coverage, camera identities, and strict
  alignment for every frame.
- Keeps semantic checks honest: video clarity, task correctness, object bounds, and
  success still require visual review and labels.

Artifacts: `audit_corpus.py` and `test_audit_corpus.py`.

## Recovery-log gate and autonomous rollout entry point — completed offline 28 July 2026

- Added `audit_recovery.py` to enforce the reviewed route's phase order, timestamp/frame
  continuity, joint/current/following-error/step/timeout limits, and gripper-open endpoint
  over every supervised CSV. It refuses to pass while the route is unvalidated and does
  not claim to measure Cartesian clearance.
- Replaced an invalid pilot command that attempted policy deployment through the current
  teleoperation-only recorder. `rollout_with_telemetry.py` wraps the real episodic rollout
  engine, registers telemetry and the custom checkpoint processor, and retains alignment
  sidecars. The leader is used only for supervised between-episode resets.

Artifacts: `audit_recovery.py`, `test_audit_recovery.py`, and
`rollout_with_telemetry.py`.

Not completed: policy-triggered recovery is not integrated, no route is physically
validated, and the ten-reset exit test has not run.

## Positions-only checkpoint eligibility gate — completed offline 28 July 2026

- Fixed `TruncateStateStep` serialization so `keep` is explicit in checkpoint JSON; a
  non-default round-trip test prevents the six-value default from masking future mistakes.
- Found and closed a pretrained-camera naming gap: the pilot train command now maps frozen
  wrist/overhead keys to SmolVLA camera1/camera2, and the training wrapper refuses this
  corpus when the mapping is absent.
- Added a lightweight checkpoint audit that checks model and normalizer feature widths,
  stored state-statistic tensor widths, first-step truncation, camera mapping, full-chunk
  execution, policy type, and model-weight presence before robot connection.

Artifacts: `audit_positions_checkpoint.py` and
`test_audit_positions_checkpoint.py`.

## Closed-loop recovery supervisor — completed offline 28 July 2026

- Added a fail-closed per-episode state machine around physical recovery: bounded accepted
  commands, trigger metadata, mandatory queue-flush verification, two-recovery cap,
  explicit reinvocation, and aborted/exhausted terminal states.
- Added fsynced JSONL events carrying detector, score, threshold, trigger frame, fault type,
  attempt, route, reverse/completed frames, and error details. Physical phase telemetry
  remains in the recovery CSV rather than being collapsed into supervisor events.
- Tests cover bounded history, successful ordering, missing flush, physical abort,
  reinvocation suppression, retry exhaustion, episode reset, and event persistence.
- Added the concrete rollout lifecycle callbacks: pause before clearing inference,
  interpolation, and cached observation; reject an unflushed resume; publish a fresh start
  observation before resuming; and fail terminally if episode-start reinvocation fails.
- Froze the live adapter and seven remaining integration acceptance checks without enabling
  autonomous recovery on unvalidated routes.

Artifacts: `recovery_supervisor.py`, `test_recovery_supervisor.py`, and
`CLOSED_LOOP_RECOVERY_INTEGRATION.md`.

Not completed: detector-triggered recovery is not wired into live rollout and cannot be
until route validation plus the ten-reset exit test pass.

## Recovery-aware action boundary — completed offline 28 July 2026

- Added a single-step gate that scores the current causal observation before accessing the
  action provider. A threshold crossing enters recovery without popping an RTC action,
  recomputing a sync chunk, or sending anything.
- Non-triggering frames strictly order compute/pop, send, raw accepted-goal readback, then
  command-history retention. Unscorable warm-up remains `None`, never a zero score.
- Added a fixed tick budget: every autonomous observation, including the trigger frame,
  counts once; recovery motion consumes no policy ticks and therefore cannot shorten an
  experimental rollout budget.
- Tests cover trigger suppression, ordinary dispatch order, warm-up, unavailable RTC
  actions, readback failure, stale RTC queue flushing through the real supervisor, and
  budget accounting.
- Added the live D0 observation adapter. It keeps raw current and normalized accepted
  gripper goals in their correct domains, advances only from the follower's possibly
  clipped send result, resets causal history after recovery, and fails closed if accepted
  command state is unavailable.
- Added tests for normalized accepted-goal feedback, explicit first-frame warm-up,
  post-recovery warm-up, and refusal to infer a goal from the proposed action.

Artifacts: `recovery_action_gate.py` and `test_recovery_action_gate.py`.

Not completed: the gate is not wired into a live rollout strategy and the active RTC
strategy composition remains a pre-live acceptance test.

## Recovery-safe RTC generation boundary — completed offline 28 July 2026

- Traced an actual refill race in upstream RTC: non-blocking pause permits an already
  active inference to merge a pre-trigger chunk after queue reset.
- Added a research-local guarded queue and RTC engine. Every producer captures a generation;
  suspend, reset, and resume invalidate outstanding generations, and stale merges are
  rejected while holding the queue lock.
- Added a deterministic threaded test in which inference starts before recovery and
  finishes after resume; the stale merge is rejected and the queue remains empty.
- Made fresh-start detector preparation part of `RolloutRecoveryLifecycle`, before engine
  resume. D0 now refuses a missing normalized start observation and re-enters warm-up from
  the correct held gripper command.

Artifacts: `recovery_rtc_guard.py`, `test_recovery_rtc_guard.py`, plus lifecycle and D0
tests in the existing recovery suites.

Not completed: the guarded engine is not selected by a live strategy factory while routes
remain physically unvalidated.

## Composed closed-loop sync/RTC acceptance — passed offline 28 July 2026

- Added complete mocked loops built from the production D0 scorer, action gate, supervisor,
  rollout lifecycle, interpolation reset, policy-tick budget, and guarded RTC queue.
- The sync case creates an old chunk and pending interpolated action, triggers on a real
  stable-command current drop, proves no trigger-frame compute/pop occurs, clears both
  stale layers, reseeds D0, and recomputes only on the next observation.
- The RTC case starts a producer before the trigger, completes it after recovery resume,
  rejects that stale-generation merge, and sends only a fresh post-recovery action.
- Both cases preserve a three-tick autonomous budget across synchronous recovery motion.

Artifact: `test_closed_loop_recovery_integration.py`.

Not completed: this proves the composition offline but does not register a live recovery
strategy or waive any physical route, ten-reset, timing, or joined-log acceptance gate.

## Recovery joined-evidence contract — completed offline 28 July 2026

- Upgraded supervisor events to schema 2 with a bound run ID and canonical episode/attempt
  IDs shared by trigger, start, terminal transition, and reinvocation.
- Added an exclusive-create immutable run manifest freezing checkpoint/revision, policy,
  sync/RTC mode, full-chunk execution, FPS, and detector/waypoint hashes.
- Added an fsynced outcome ledger and exclusive per-attempt physical CSV writer carrying
  identity, route, monotonic timing, raw goals, and the complete telemetry block.
- Added a relational auditor for missing/orphan/duplicate/conflicting episodes, attempts,
  frames, routes, outcomes, and policy configuration. Synthetic complete and negative
  fixtures produce their declared verdicts.

Artifacts: `recovery_evidence.py`, `audit_recovery_evidence.py`,
`test_recovery_evidence.py`, and `RECOVERY_EVIDENCE_SCHEMA.md`.

Not completed: acceptance item 7 still needs one joined audit PASS from a real
recovery-enabled rollout after the physical gates open.

## Composed retry cap and terminal-frame budget — passed offline 28 July 2026

- Ran the real D0 stable-command current-drop sequence through two successful recoveries;
  each cleared state, reseeded D0, and resumed fresh policy execution.
- Proved the third trigger enters the exhausted state before physical recovery, policy
  compute/pop, or action send.
- Added `RecoveryControlLoop`, whose `finally` boundary counts every observed autonomous
  frame, including a terminal recovery exception that returns no `GatedStepResult`.
- Proved six observations consume six policy ticks while two synchronous recovery motions
  consume none.

Artifact: the retry lifecycle case in `test_closed_loop_recovery_integration.py`.

Not completed: acceptance items 5–6 still require the declared live/hardware demonstration
after route validation; the offline behavior is now fixed and tested.

## Recovery enablement readiness gate — completed offline 28 July 2026

- Froze a reviewed physical-validation record covering every configured route, the three
  visual-clearance ladder stages, ten distinct redesigned recovery logs, zero manual
  interventions, alarm-free pre/post diagnostics, temperatures, and peak current.
- Added a fail-closed auditor that reruns every route/reset log plus the positions-only
  checkpoint audit, checks detector/waypoint hashes and run-manifest identity, and refuses
  `validated:false`, stale, missing, duplicate, or numerically failing evidence.
- On a complete PASS only, it exclusive-creates an immutable enablement token binding the
  run, validation record, configurations, checkpoint revision, chunk settings, and required
  inference engine. RTC tokens name the generation-guarded research engine.
- Tests cover a complete bundle, open physical gates, intervention/fewer-than-ten evidence,
  stale hashes, failed component audits, and token overwrite refusal.

Artifacts: `audit_recovery_readiness.py`, `test_audit_recovery_readiness.py`,
`recovery_validation.schema.json`, and `RECOVERY_READINESS_GATE.md`.

Not completed: the repository has no physical validation record and therefore no token;
the gate records evidence but cannot replace it.

## Recovery token runtime verification — completed offline 28 July 2026

- Added a fail-closed verifier intended as the first recovery-specific live-adapter call,
  before supervisor, executor, or action-gate construction.
- It recomputes physical-record, waypoint, and detector hashes; checks run ID, checkpoint,
  immutable revision, inference mode, and full-chunk settings; and compares the actual
  engine's fully qualified class with the issued token.
- RTC refuses any token or runtime engine other than
  `recovery_rtc_guard.RecoverySafeRTCInferenceEngine`.
- Tests accept exact bound sync and real guarded-RTC class bundles and reject changed
  waypoints, manifest/revision drift, partial chunks, wrong engine types, and unguarded RTC
  declarations. The real-class test allocates no engine resources or hardware connection.

Artifacts: `recovery_enablement.py` and `test_recovery_enablement.py`.

Not completed: the future live strategy must call this verifier; no strategy is registered
while the physical readiness token is unavailable.

The gate integration also corrected D0 warm-up semantics: the first frame cannot contain
a current drop and is now unscorable rather than zero-scored. This enforces the frozen
causal invariant without altering D0r; four D0r development runs retained zero score delta
and identical masks/triggers, and the runtime hash ledger was amended.
