# Next steps

Completed infrastructure has moved to [`DONE.md`](./DONE.md). This file contains only
unfinished work, ordered by dependency. Research framing and accumulated evidence are in
[`Research.md`](./Research.md), [`vla-failure-detection.md`](./vla-failure-detection.md),
and the dated session logs.

## Current gates

| order | gate | requires robot | status |
|---:|---|:---:|---|
| 1 | redesign arbitrary-pose recovery path | partly | **safety blocker** |
| 2 | validate redesigned reset with 10 consecutive repeats | yes | blocked by gate 1 |
| 3 | diagnose D0r cross-trajectory false positives | partly | primary hold-out failed |
| 4 | T1 pilot and difficulty calibration | yes | blocked on task objects |
| 5 | labeling/schema preparation for the rollout corpus | no | start during pilot |

Do not run the unattended reset soak until gate 1 is resolved. The trajectory-B hold-out
is complete; preserve its failed primary result while improving D0r on training and
calibration data only.

---

## 1. Redesign arbitrary-pose recovery before the reset exit test

The current reset enters its recorded path by moving only `shoulder_lift`, then aligning
the other joints. Hardware testing showed why this is not a general safety rule. A single
joint rotation does not mean Cartesian "up": the gripper follows an arc whose direction
depends on the entire arm configuration. From some release or failure poses that arc can
move the gripper sideways, downward, into the table, or through a task object.

**Treat the current lift-first entry as experimental. Do not run it unattended and do not
count it as the final recovery path.** The five-repeat soak only validates the older poses
that happened to be exercised; it does not establish safe recovery from arbitrary rollout
states.

### Required recovery sequence

Build and validate this sequence before repeating the soak:

1. Halt by writing measured present position as the new goal.
2. Flush the policy's queued action chunk.
3. Replay a bounded buffer of recent commanded positions in reverse, returning along the
   path by which the arm entered the task area.
4. From that retracted state, transition through one or more validated clearance
   waypoints.
5. Join and replay the recorded return-home path.
6. Reopen the gripper and reinvoke the policy from the episode start state.
7. Abort on excessive following error, current, timeout, or workspace-limit violation.

Reverse replay is the first safety mechanism, not proof of collision freedom: an object
may have moved, a slip may change what the gripper carries, and the original path may
already include contact. Limit the history, monitor it, and stop before replaying through
the detected fault itself.

### Planning options, cheapest first

1. **Validated waypoint library.** Divide the task workspace into a small number of
   regions and record a safe retract/return path for each. Select by current joint pose or
   end-effector region. This is likely sufficient for the fixed tabletop tasks.
2. **Cartesian clearance path.** Use forward/inverse kinematics to raise the end effector
   to a known collision-free height, then move laterally and descend into the home path.
   Validate every IK solution for joint limits and continuity.
3. **Collision-aware motion planning.** Build a table/robot geometry model and plan from
   the measured state to the reset-path entrance. Use this if waypoint coverage becomes
   brittle or the task geometry changes frequently.

Do not replace the current lift-first rule with another unverified joint-space linear
interpolation. A path is acceptable only after checking the end-effector sweep and link
clearances from the range of poses autonomous failures actually produce.

### Information the runtime must retain

- a timestamped ring buffer of commands actually sent to the follower;
- the action-chunk boundary so queued future actions can be discarded;
- current measured joint pose and, once available, forward-kinematic end-effector pose;
- detector trigger time, score, and likely fault type;
- which recovery path/waypoints were selected;
- current/load/tracking-error telemetry throughout recovery.

### Validation ladder

1. Plot or simulate candidate paths from a grid of representative task poses.
2. Execute at reduced speed with no task objects and an operator at the power switch.
3. Repeat with static task objects in their allowed start regions.
4. Test representative slip, collision, and awkward-release poses individually.
5. Only then run the unattended 10-reset soak.

Pass criteria must include minimum gripper/table clearance, minimum link/object clearance,
joint-limit margin, maximum tracking error/current, and zero manual intervention. Record
these quantities rather than relying only on visual judgment.

### Final reset exit test

The existing soak log has five repeats and predates this redesign. After a safe entry path
has been implemented, the exit criterion is ten consecutive resets with minimal manual
intervention using the redesigned code.

Run the health check first:

```bash
python research/telemetry/diagnose.py --port /dev/ttyACM0
```

Then run the full soak, with the workspace clear and a hand near the power switch for the
first repeat:

```bash
python research/telemetry/recovery.py reset --port /dev/ttyACM0 \
    --home research/telemetry/runs/reset_home.csv --repeat 10 \
    --log research/telemetry/runs/reset_soak.csv
```

Immediately capture post-soak health and temperature:

```bash
python research/telemetry/diagnose.py --port /dev/ttyACM0
```

Record:

- number of completed resets;
- any manual intervention or clearance timeout;
- start/end `shoulder_lift` temperature;
- whether the gripper, wrist, or pan approached the table or an obstacle;
- peak shoulder current in the soak log.

Pass only if all ten complete without unsafe motion or manual repositioning. If it fails,
preserve the log and fix the specific phase before repeating; do not loosen a safety gate
merely to obtain 10/10.

---

## 2. Diagnose D0r cross-trajectory false positives

The pre-registered A→B hold-out is complete and failed specificity. Read
[`PAIR4_HOLDOUT_RESULT.md`](./PAIR4_HOLDOUT_RESULT.md) and
[`D0R_CLEAR_DIAGNOSIS.md`](./D0R_CLEAR_DIAGNOSIS.md) before changing the model.
Trajectory B is now observed development data and can never be presented as an untouched
hold-out again.

What is established:

- the collision residual transfers strongly across trajectories and onto
  `shoulder_lift`/`elbow_flex`;
- the A model fires repeatedly on B clear, peaking at 4.32× p99;
- the B model clears B but fires on A clear;
- pair4 clear remains inside the combined A+B command coverage, so its remaining false
  alerts are not feature extrapolation;
- ordinary independent-replay residuals on pair4 clear reach 31.3 ticks on wrist flex
  and 18.4 on wrist roll, above the frame-p99 floors of 14.9 and 8.3;
- the thresholding unit is therefore wrong: correlated frame percentiles do not control
  false alarms over an independent rollout;
- training and evaluation sessions differed by roughly 5–8 °C, but trajectory and
  temperature shifted together, so causality is unresolved.

Next work, in order:

1. Before bulk collection, record two clear replays with the updated logger. New CSVs
   include `goal2.*`, the servo's read-only internal/interpolated setpoint candidate from
   register 71. Confirm that it moves, is repeatable, and explains the wrist reversal
   current better than the external `goal_pos.*` trace. This costs two bytes in the same
   block transaction and does not alter the 30-dimensional policy schema.
2. Collect at least 19 independent clear calibration rollouts for a finite
   distribution-free 5% per-rollout false-alarm threshold. Vary trajectories, direction,
   payload state, and session; do not record 19 replays of one command trace and call them
   coverage.
3. Include autonomous successful rollouts as hard negatives in that calibration split.
4. Run `freespace_model.py calibrate --alpha 0.05` on those clear files. It now uses one
   maximum causal residual vector per independent rollout and refuses underpowered
   calibration rather than silently using frame percentiles.
5. Keep the commanded-feature coverage diagnostic enabled. It abstains on unsupported
   motion instead of turning model extrapolation into a contact claim.
6. Test session-offset correction using no-contact calibration data only. Temperature may
   be analyzed as a covariate but not claimed causal from A/B alone.
7. Pre-register a new trajectory C after the model and operating procedure are frozen.
   C—not B—becomes the next untouched generalization test.

### Immediate `goal2` experiment

Clear the workspace, then replay trajectory B twice with no obstacle. The updated block
read logs raw `goal2.*` register values alongside current in the same transaction:

```bash
python research/telemetry/log_teleop_telemetry.py \
    --follower-port /dev/ttyACM0 \
    --replay-csv research/telemetry/runs/freespace_b_01.csv \
    --out research/telemetry/runs/clear_goal2_a.csv

python research/telemetry/log_teleop_telemetry.py \
    --follower-port /dev/ttyACM0 \
    --replay-csv research/telemetry/runs/freespace_b_01.csv \
    --out research/telemetry/runs/clear_goal2_b.csv
```

Analyze the matched pair:

```bash
python research/telemetry/analyze_goal2.py \
    research/telemetry/runs/clear_goal2_a.csv \
    research/telemetry/runs/clear_goal2_b.csv
```

The analyzer verifies identical external commands, checks whether register 71 is dynamic,
and compares cross-replay current prediction from external `goal_pos.*` against internal
`goal2.*` motion. If wrist-roll p99 underprediction falls materially with `goal2`, test it
as a development-only commanded/controller-state feature using whole-run splits. If it
does not, retain the current conclusion: unobserved friction/controller-state variability
requires rollout-level calibration rather than another feature fitted to pair4.

Current development diagnostic (not a performance result):

```bash
python research/telemetry/freespace_model.py calibrate \
    --model research/telemetry/models/freespace_ab_coverage.npz --alpha 0.05 \
    research/telemetry/runs/pair1_clear.csv research/telemetry/runs/pair4_clear.csv
```

With only these two clear runs, the command prints their rollout maxima and correctly
declines to write a calibrated model.

Do not solve the failed hold-out by selecting a new percentile on pair4. A post-hoc sweep
may explain sensitivity, but the recorded primary verdict remains failed.

---

## 3. Obtain and lock Week 3 task materials

- Two spare STS3215 servos.
- T1: one 2–3 cm cube and a bowl.
- T2: cylindrical markers in smooth plastic and rubber-gripped finishes; cup with
  adjustable rim height.
- T3: target block plus three distractors at two contrast levels: obvious and
  near-identical.

Before collecting demonstrations:

- tape and photograph both camera mounts;
- define exact object start regions and randomization bounds;
- create a difficulty spec sheet with object identifiers and reference photographs;
- decide where session temperature and camera-alignment checks are logged.

---

## 4. T1 pilot and difficulty lock

Collect 30 randomized T1 demonstrations with the frozen 30-dimensional telemetry schema.
Train the Arm 1 six-dimensional policy with `TruncateStateStep(keep=6)`, then run roughly
30 autonomous evaluations.

Target a 40–60% Arm 1 failure rate. At n=30 the estimate is roughly ±18 percentage points,
so stop after at most three tuning iterations once it lands in band. Prefer physical
difficulty changes—randomization radius and object geometry—over deliberately
undertraining the policy.

During the pilot, compare multi-task-compatible and per-task organization. Prefer one
multi-task policy if failure character remains crisp, because later learned detectors are
policy-specific and splitting failures across three networks weakens D3.

Once Arm 1 difficulty is locked:

- do not retune it for Arm 3;
- do not change objects, bounds, camera pose, or instructions without a new condition;
- record the exact checkpoint and action-chunk configuration;
- measure policy inference latency at the real control-loop settings.

---

## 5. Prepare the rollout corpus contract before collection

Write the labeling guide before labeling any rollout. It must define:

- outcome and fine-grained fault codes `S1–S3`, `E1–E6`, and excluded hardware events
  `H1–H2`;
- collapsed semantic/execution analysis labels;
- onset frame: first frame at which failure becomes inevitable;
- unrecoverable boundary under the fixed scripted recovery;
- uncertainty interval for ambiguous onset;
- recoverable deviations that must remain negatives;
- multiple-event rollouts and which event owns a detector trigger;
- second-annotator procedure for a 15% stratified subset.

Freeze the machine-readable annotation schema at the same time. At minimum record:

- episode/run ID, task, arm, split, session, policy checkpoint, seed;
- success, fine class, collapsed class, onset frame/time, unrecoverable frame/time;
- grasp start/end and duration;
- recovery eligibility and retry count;
- session start/end temperature;
- camera-alignment check;
- disturbance type and measured magnitude when applicable.

Define train/calibration/test splits by trajectory or episode group before fitting D0+.
Near-duplicate trajectories must never cross splits. Threshold selection uses calibration
only; held-out test conditions remain untouched until the detector suite is frozen.

---

## 6. Full demonstration collection

After the T1 gate, collect 60 clean demonstrations per task, 180 total. Interleave tasks
to distribute demonstrator fatigue and drift. Randomize object position on every episode.
Back up each session off-machine before the next collection block.

One corpus trains both policies:

- Arms 1/2: truncate observation state to six positions.
- Arms 3/4: use the full 30-dimensional state.

Train with matching seeds and hyperparameters where possible. Only Arm 1 difficulty is
calibrated; Arm 3's resulting failure rate and failure composition are outcomes.

---

## 7. Detector work after labeled data exists

Use `detectors.py features` to build D0+ inputs, then fit logistic regression before a
small MLP. The trivial duration baseline, D0, D0r, and D0+ must share the same splits and
calibration data.

Later detector order:

1. D1 action-chunk consistency;
2. D0+ telemetry classifier and window-length ablation;
3. D3 supervised latent probe;
4. D2 perturbation disagreement, offline if it is too slow online;
5. fusion of the best proprioceptive and best model-internal detector;
6. oracle labels for the closed-loop upper bound.

Report AUPRC, false alarms per rollout, recall at fixed false-alarm budgets, event-matched
latency, recovery-ready fraction, and measured compute/memory overhead. Keep D0's short
5–10-frame rule separate from D0+'s long-window hypothesis.

---

## 8. Closed-loop requirements to preserve now

- Hold episode budgets constant in **policy execution time**, not wall-clock time.
- Flush queued action chunks on trigger.
- Halt by writing present position as goal; never merely stop sending actions.
- Replay recent commands backward before the return-home path.
- Reopen the gripper and reinvoke the policy from the start pose.
- Cap recovery at two retries.
- Log every trigger, score, detector, threshold, recovery phase, and outcome.
- Report successes recovered, successes disrupted by false alarms, unsafe/failed
  recoveries, and added execution time separately.

The current reset CLI validates halt and return-home mechanics. Integration with policy
chunk flushing, recent-command reversal, gripper reopen, retry accounting, and policy
reinvocation remains future closed-loop work; do not describe those pieces as completed.
