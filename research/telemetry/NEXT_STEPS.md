# Next steps

Completed infrastructure has moved to [`DONE.md`](./DONE.md). Weekly evidence is in
[`WEEK1_REPORT.md`](./WEEK1_REPORT.md) and [`WEEK2_REPORT.md`](./WEEK2_REPORT.md). Research
framing and the schedule are in [`Research.md`](./Research.md) and
[`vla-failure-detection.md`](./vla-failure-detection.md). This file contains only
unfinished work.

The strict desk-completion and external-blocker evidence audit is
[`NEXT_STEPS_BLOCKED_AUDIT_2026-07-28.md`](./NEXT_STEPS_BLOCKED_AUDIT_2026-07-28.md).

## Where this sits against the schedule

`vla-failure-detection.md` §7.1 anchors Week 1 to 3–9 August 2026 and says to shift
uniformly if you start later. Work started **earlier**: the Week 1 gate closed 25 July and
Week 2's software was built 26–28 July. Against the anchor the project is roughly two weeks
ahead, and the nominal Week 3 pilot does not begin until 17 August.

That buffer is real but it is not spendable on analysis. The remaining Week 2 gates now
require physical work, and two later gates have external lead time: procurement and the
second annotator. Spare servos and task objects still gate the pilot absolutely.
**Procurement is the critical path, not D0r.**

The three Week 2 exit criteria that remain unmet are the reset soak, camera rigging, and
telemetry↔frame alignment verification. See `WEEK2_REPORT.md` §1 for the full scorecard.

## Current gates

| order | gate | robot | lead time | status |
|---:|---|:---:|:---:|---|
| 1 | order servos and task materials | no | **external** | **not started — gates everything** |
| 2 | physically validate arbitrary-pose recovery routes | yes | none | **software done; safety gate open** |
| 3 | rig cameras; photograph reference views | yes | none | not started, Week 2 deliverable |
| 4 | verify timing sidecar on a two-camera episode | yes | none | software done; hardware smoke test open |
| 5 | measure policy inference latency | no | none | **done 28 Jul** |
| 6 | validate redesigned reset, 10 consecutive repeats | yes | none | blocked by gate 2 |
| 7 | arrange second annotator | no | external | guide/schema done; person not arranged |
| 8 | T1 pilot and difficulty calibration | yes | none | blocked by gates 1, 3, 6 |

Do not run the unattended reset soak until gate 2 is resolved. The trajectory-B hold-out is
complete and failed; preserve that primary result and improve D0r on training and
calibration data only.

---

## 1. Order the physical materials today

Zero robot time, zero analysis, and everything downstream waits on them. The servo order
has been an open action item since Week 1 and is the stated mitigation for the *active*
servo-overload-alarm risk in `Research.md`.

- Two spare STS3215 servos.
- **T1:** one 2–3 cm cube and a bowl.
- **T2:** cylindrical markers in smooth plastic *and* rubber-gripped finishes; a cup with
  adjustable rim height. T2 is the primary telemetry task — the friction pair is the
  difficulty lever that generates `E2` slips on demand, so both finishes matter.
- **T3:** a target block plus three distractors at two contrast levels, obvious and
  near-identical.

Order the T2 friction pair and the T3 near-identical distractors even if T1 is the only
task piloted. They are the difficulty levers §5.3 depends on, and a second order later
costs another lead time.

Use `PROCUREMENT_CHECKLIST.md` to record exact parts, vendors, dates, and arrivals. The
checklist is ready; the gate remains open until the order is actually placed.

---

## 2. Physically validate arbitrary-pose recovery before the reset exit test

The reverse-replay/waypoint recovery software and supervised validator were implemented
28 July; see `RECOVERY_PROTOCOL.md` and `ROBOT_DATA_RUNBOOK.md`. The runtime now fails
closed on unvalidated/ambiguous routes, current, following error, joint range, and timeout.
The old lift-first path is hidden behind an explicit unsafe-experiment acknowledgement.
`audit_recovery.py` now checks every supervised log against the reviewed configuration and
refuses to pass an unvalidated route; geometric clearance and the ten repetitions remain
physical evidence, not software assertions.

**This is still a safety blocker.** No waypoint has been recorded or physically validated,
the tested queue-flush supervisor is not attached to live rollout, and the ten-reset test
has not run. Do not equate passing offline tests with collision-free motion.

The current reset enters its recorded path by moving only `shoulder_lift`, then aligning
the other joints. Hardware testing showed why this is not a general safety rule. A single
joint rotation does not mean Cartesian "up": the gripper follows an arc whose direction
depends on the entire arm configuration. From some release or failure poses that arc can
move the gripper sideways, downward, into the table, or through a task object.

**Treat the current lift-first entry as experimental. Do not run it unattended and do not
count it as the final recovery path.** The five-repeat soak only validated the older poses
that happened to be exercised; it does not establish safe recovery from arbitrary rollout
states, and it predates the feedback-gated sequence anyway.

### Required recovery sequence

1. Halt by writing measured present position as the new goal.
2. Flush the policy's queued action chunk.
3. Replay a bounded buffer of recent commanded positions in reverse, returning along the
   path by which the arm entered the task area.
4. From that retracted state, transition through one or more validated clearance waypoints.
5. Join and replay the recorded return-home path.
6. Reopen the gripper and reinvoke the policy from the episode start state.
7. Abort on excessive following error, current, timeout, or workspace-limit violation.

Reverse replay is the first safety mechanism, not proof of collision freedom: an object may
have moved, a slip may change what the gripper carries, and the original path may already
include contact. Limit the history, monitor it, and stop before replaying through the
detected fault itself.

### Planning options, cheapest first

1. **Validated waypoint library.** Divide the task workspace into a small number of regions
   and record a safe retract/return path for each. Select by current joint pose or
   end-effector region. Likely sufficient for fixed tabletop tasks.
2. **Cartesian clearance path.** Use forward/inverse kinematics to raise the end effector to
   a known collision-free height, then move laterally and descend into the home path.
   Validate every IK solution for joint limits and continuity. *This is what §7.1's Week 2
   text originally proposed; the joint-space shortcut was the deviation.*
3. **Collision-aware motion planning.** Build a table/robot geometry model and plan from the
   measured state to the reset-path entrance. Use this only if waypoint coverage becomes
   brittle or task geometry changes frequently.

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
these quantities rather than relying on visual judgment.

### Final reset exit test

The existing soak log has five repeats and predates this redesign. The exit criterion is
ten consecutive resets with minimal manual intervention using the redesigned code.
`recovery_validation.schema.json` now freezes the evidence record, and
`audit_recovery_readiness.py` will issue an immutable enablement token only when every
route log, ten distinct redesigned reset logs, pre/post diagnostics, temperature/current
fields, checkpoint, and configuration hash pass. This closes the evidence-format gap; it
does not supply any of the missing physical inputs.
`recovery_enablement.py` now closes the stale-token runtime gap: immediately before
recovery construction it recomputes all bound hashes and compares run, revision, chunk,
mode, checkpoint, and the actual inference-engine class. Changed artifacts or upstream RTC
raise before any recovery object exists.

Health check first:

```bash
python research/telemetry/diagnose.py --port /dev/ttyACM0
```

Then follow the supervised region-by-region commands and validation ladder in
`ROBOT_DATA_RUNBOOK.md`. There is deliberately no unattended soak command until every
workspace route has passed that ladder and the live policy runtime supplies its actual
command ring buffer and action-queue flush.

Immediately capture post-soak health and temperature:

```bash
python research/telemetry/diagnose.py --port /dev/ttyACM0
```

Record: completed resets; any manual intervention or clearance timeout; start/end
`shoulder_lift` temperature; whether the gripper, wrist, or pan approached the table or an
obstacle; peak shoulder current in the soak log.

Pass only if all ten complete without unsafe motion or manual repositioning. If it fails,
preserve the log and fix the specific phase before repeating; do not loosen a safety gate
merely to obtain 10/10.

---

## 3. Rig work and corpus contract, all runnable while blocked

None of this needs a working recovery path, and all of it is on the critical path for the
corpus.

### 3.1 Cameras — a Week 2 deliverable, not started

Mount both cameras, tape the mounts, and photograph the reference views. §5.1 names
viewpoint drift as "the most common silent confound in this literature" and requires the
check to be verified and **logged** at the start of every session. Decide now where that
log lives; retrofitting it is worthless because the drift it detects is already baked in.

### 3.2 Verify the implemented frame-alignment audit trail on hardware

`SOFollowerTelemetry.get_observation()` reads position, the telemetry block, and both
cameras in one call, so telemetry and image share a control step by construction. But the
dataset's `timestamp` column is synthetic (`frame_index / fps`) and always reads as a
perfect 30 Hz, so neither jitter nor a dropped frame is visible in the recorded corpus.
Week 2's exit criterion — alignment verified to within one control step — cannot currently
be checked at all, before or after the fact.

The decision and software are complete: `record_with_telemetry.py` writes per-episode JSONL
sidecars keyed by frame index without changing the 30-dim state. They contain monotonic
state/telemetry read times and each camera thread's actual capture timestamp;
`audit_alignment.py` enforces the one-control-step contract and detects stale, duplicate,
missing, or dropped captures. Offline tests pass.

Still required: record one disposable two-camera episode and obtain a strict audit PASS
before collecting anything for the corpus. Preserve a failing episode and sidecar as a
diagnostic artifact.

### 3.3 Policy inference latency — done 28 July 2026

Measured with `bench_inference.py`; full results in `WEEK2_REPORT.md` §6. Headline: 3.8 ms
mean per control step at `n_action_steps=50` on the RTX 4090, roughly 8× headroom over
30 Hz, and identical for 6-dim and 30-dim state. Compute is not a constraint.

Two results to carry forward:

- **Set `n_action_steps` to the chunk size.** Single-step prediction costs 21.6× more and
  misses the 30 Hz budget on every step. Verify this in the Week 6 checkpoint config.
- **Budget recovery against the worst step, not the mean.** Chunk recompute costs ~85 ms,
  about 2.5 control periods, so the loop hitches every `n_action_steps`. Log which steps
  were recompute steps during closed-loop runs.

```bash
python research/telemetry/bench_inference.py                    # Arms 1/2
python research/telemetry/bench_inference.py --state-dim 30     # Arms 3/4
```

**Still open:** 20k-step fine-tune wall-clock on this GPU from a short timed run. §5.4
budgets ~4 hours on an A100 and "proportionally longer on consumer hardware"; Weeks 4–6
change shape if that number is much worse than assumed. Worth doing on the first pilot
dataset rather than synthetically.

**Coverage scaling measured 28 July.** `D0R_COVERAGE_BENCHMARK.md` projects the stored
reference to 270k frames. The exact reusable SciPy tree costs 177 ms once at model load,
about 0.03 ms per online frame, and 140 MiB; scores and scorable masks remained unchanged.
The exact NumPy fallback is memory-safe but costs about 32 ms for an isolated frame, so the
corpus runtime must include SciPy. Re-measure on the actual corpus model for the final cost
table, but the implementation blocker is closed.

### 3.4 Arrange a second annotator

Week 9 needs an independent annotator on a 15% subsample and a reported Cohen's κ. This is a
person-dependency with its own lead time and nobody has arranged it. §7.1's exit criterion
is κ above roughly 0.7 on class labels, with a re-label if onset agreement is poor — so the
second annotator must be available *during* Week 9, not after.

`ANNOTATOR_BRIEF.md` now fixes the blinded inputs, outputs, time commitment, agreement
report, and outreach text. The gate remains open until a person accepts.

### 3.5 Watch the workshop deadline

Checked 28 July against the live conference and workshop pages. The strongest target is
**Continually Self-Improving Robots** (failure discovery, automatic reset, safe exploration,
reliable evaluation); Human-Centered Robot Learning and Memory for Robot Foundation Models
are backups. All three have calls/pages, but no authoritative calendar deadline is
published yet. `WORKSHOP_TRACKER.md` records the official links, fit, format, and a weekly
Monday check rule. Do not treat the proposal's late-September/early-October estimate as a
deadline.

---

## 4. D0r — what is actually left, and what to stop doing

The pre-registered A→B hold-out is complete and failed specificity. Read
[`PAIR4_HOLDOUT_RESULT.md`](./PAIR4_HOLDOUT_RESULT.md) and
[`D0R_CLEAR_DIAGNOSIS.md`](./D0R_CLEAR_DIAGNOSIS.md) before changing the model. Trajectory B
is now observed development data and can never be presented as an untouched hold-out again.

What is established:

- the collision residual transfers strongly across trajectories and onto `shoulder_lift`
  and `elbow_flex`, not only `shoulder_pan`;
- the A model fires repeatedly on B clear, peaking at 4.32× p99;
- the B model clears B but fires on A clear;
- pair4 clear sits inside combined A+B command coverage and still crosses at 2.2×, so its
  remaining false alerts are not feature extrapolation;
- ordinary independent-replay residuals reach 31.3 ticks on `wrist_flex` and 18.4 on
  `wrist_roll`, above frame-p99 floors of 14.9 and 8.3;
- the thresholding *unit* is therefore wrong: correlated frame percentiles do not control
  false alarms over an independent rollout;
- **`Goal_Position_2` is eliminated.** Register 71 reads a constant 0 on every motor in both
  matched clear replays, including through the wrist-reversal transient. The internal-
  setpoint hypothesis is dead; see `WEEK2_REPORT.md` §3;
- training and evaluation sessions differed by 5–8 °C, but trajectory and temperature
  shifted together, so causality is unresolved.

### Stop trying to fix this with teleoperation sessions

Calibration needs **≥19 independent clear rollouts** for a finite distribution-free 5%
per-rollout threshold. Two exist. The instinct is to book a robot session and record 19 more
clear replays — do not. Nineteen replays of one command trace are not coverage, and
teleoperated clears are the wrong motion distribution anyway: the deployed negatives are
*autonomous policy rollouts*, a third distribution the free-space model has never seen.

Both requirements are satisfied for free by the corpus phase. §5.4's calibration split is
150 rollouts, 50 per task — eight times the conformal minimum, in the right motion
distribution, across sessions and payload states. **D0r's calibration is corpus-phase work.
It is not blocking the pilot and it should not consume robot time before it.**

### Done 28 July 2026

1. **`goal2` eliminated and confirmed on hardware.** Register 71 reads 0 at a nonzero
   standstill pose and through 10 s of motion, via a second independent read path.
   `D0R_CLEAR_DIAGNOSIS.md` carries the result and the internal-setpoint line is struck
   from the candidate causes. Nothing further is outstanding on this.
2. **Model and calibration procedure frozen** in
   [`D0R_FROZEN_SPEC.md`](./D0R_FROZEN_SPEC.md), with reference hashes, the declared α
   values, the ≥19-rollout minimum, and the rule that the procedure is frozen while the
   fitted model is not — the deployable model is refit on corpus training data, and no
   development `.npz` is eligible.
3. **Corpus requirements for calibration** specified in §6 of that spec and frozen in
   `LABELING_GUIDE.md` plus `annotation.schema.json`: verified-clear flag, independence
   group, session, temperature, payload state, and split assignment.
4. **Trajectory C pre-registered** in
   [`TRAJECTORY_C_PROTOCOL.md`](./TRAJECTORY_C_PROTOCOL.md), including how abstention
   counts, what happens if C fails, and the analysis commands written before the data
   exists.

### What remains

1. **Record trajectory C during the pilot** and seal it. It costs almost nothing to capture
   alongside T1 demonstrations and cannot be scored until corpus calibration exists.

2. **Everything else is corpus-phase.** Fit on the corpus training split, calibrate on ≥19
   independent clear rollouts from the calibration split, then open C once.

There is no useful D0r command to run before the corpus exists.

Current development diagnostic, not a performance result:

```bash
python research/telemetry/freespace_model.py calibrate \
    --model research/telemetry/models/freespace_ab_coverage.npz --alpha 0.05 \
    research/telemetry/runs/pair1_clear.csv research/telemetry/runs/pair4_clear.csv
```

With only two clear runs it prints their rollout maxima and correctly declines to write a
calibrated model. That refusal is the intended behaviour.

Do not solve the failed hold-out by selecting a new percentile on pair4. A post-hoc sweep
may explain sensitivity; the recorded primary verdict remains failed.

---

## 5. T1 pilot and difficulty lock

Blocked on materials (§1), cameras (§3.1), and the reset exit test (§2).

Collect 30 randomized T1 demonstrations with the frozen 30-dimensional telemetry schema,
then run roughly 30 autonomous evaluations. The previously missing train-path integration
is now implemented: `train_positions_only.py` narrows copied policy metadata/statistics and
serializes `TruncateStateStep(keep=6)` before normalization without changing the corpus.
The commands and preflight checks are frozen in `PILOT_PROTOCOL.md` and
`ROBOT_DATA_RUNBOOK.md`. No pilot data has been collected and the physical gates above
remain open.

Autonomous evaluation uses `rollout_with_telemetry.py --strategy.type=episodic`. The prior
draft command incorrectly attached `--policy.path` to the teleoperation-only recording
entry point and has been replaced. The rollout wrapper registers the telemetry robot and
positions-only checkpoint processor and retains alignment sidecars.

`audit_corpus.py` is the executable acceptance gate for recorded pilot datasets. It checks
the actual Parquet vector values and counts, normalization-statistic widths, both camera
artifacts, and every alignment sidecar; metadata alone is not accepted as proof. Visual
review and physical task acceptance remain operator judgments.

`audit_positions_checkpoint.py` is the executable Arm 1 checkpoint gate. Training now
refuses the two-camera corpus unless wrist/overhead are explicitly mapped to SmolVLA's
pretrained camera1/camera2 inputs, and the auditor verifies that mapping alongside the
serialized six-position truncation, normalization state, model features, and full action
chunk before autonomous motion.

Target a 40–60% Arm 1 failure rate. At n=30 the estimate carries roughly ±18 percentage
points, so stop after at most three tuning iterations once it lands in band. Prefer physical
difficulty changes — randomization radius and object geometry — over deliberately
undertraining the policy; an undertrained policy fails by flailing and produces mushy,
unrepresentative failures.

Before collecting demonstrations: define exact object start regions and randomization
bounds, and create the difficulty spec sheet with object identifiers and reference
photographs.

During the pilot, compare multi-task-compatible and per-task organization. Prefer one
multi-task policy if failure character stays crisp, because later learned detectors are
policy-specific and splitting failures across three networks weakens D3.

Also during the pilot, at no extra robot cost: record grasp duration per episode, capture
trajectory C for the frozen D0r hold-out, and log session start/end temperature.

Once Arm 1 difficulty is locked:

- do not retune it for Arm 3;
- do not change objects, bounds, camera pose, or instructions without a new condition;
- record the exact checkpoint and action-chunk configuration;
- re-measure the failure rate at the corpus midpoint and log it — drift mid-corpus leaves
  two incomparable halves and you need to know.

---

## 6. Full demonstration collection

After the T1 gate, collect 60 clean demonstrations per task, 180 total. Interleave tasks to
distribute demonstrator fatigue and drift. Randomize object position on every episode.
Verify camera alignment against the §3.1 reference photographs at the start of every session
and log the check. Back up each session off-machine before the next collection block.
Use `DATA_BACKUP_PROTOCOL.md`: upload the complete dataset root, including timing sidecars,
to a private Hub dataset and verify the remote tree before continuing.

One corpus trains both policies:

- Arms 1/2: truncate observation state to six positions.
- Arms 3/4: use the full 30-dimensional state.

Train with matching seeds and hyperparameters where possible. Only Arm 1 difficulty is
calibrated; Arm 3's resulting failure rate and failure composition are outcomes, not
parameters.

---

## 7. Detector work after labeled data exists

Use `detectors.py features` to build D0+ inputs, then fit logistic regression before a small
MLP. The trivial duration baseline, D0, D0r, and D0+ must share the same splits and
calibration data. The duration baseline stays in the comparison — on Week 1 files it ranks
well purely because recording length correlates with positive frames, which is exactly the
confound it exists to expose.

Later detector order:

1. D1 action-chunk consistency;
2. D0+ telemetry classifier and window-length ablation;
3. D3 supervised latent probe;
4. D2 perturbation disagreement, offline if too slow online;
5. fusion of the best proprioceptive and best model-internal detector;
6. oracle labels for the closed-loop upper bound.

Report AUPRC, false alarms per rollout, recall at fixed false-alarm budgets, event-matched
latency, recovery-ready fraction, and measured compute/memory overhead against the §3.3
latency denominator. Keep D0's short 5–10-frame rule separate from D0+'s long-window
hypothesis; the 500-sample industrial result belongs to D0+, not to D0.

---

## 8. Closed-loop requirements to preserve now

- Hold episode budgets constant in **policy execution time**, not wall-clock time.
- Flush queued action chunks on trigger.
- Halt by writing present position as goal; never merely stop sending actions.
- Replay recent commands backward before the return-home path.
- Reopen the gripper and reinvoke the policy from the start pose.
- Cap recovery at two retries.
- Log every trigger, score, detector, threshold, recovery phase, and outcome.
- Report successes recovered, successes disrupted by false alarms, unsafe/failed recoveries,
  and added execution time separately.

The supervised validator exercises halt, an actual-command ring buffer, recent-command
reversal, monitored waypoints/home, and gripper reopen. `RecoverySupervisor` now implements
the offline-testable orchestration contract: structured trigger events, mandatory policy
queue flush verification, a bounded accepted-command history, two-recovery episode cap,
fail-closed states, stale-history clearing, and explicit reinvocation.
`RolloutRecoveryLifecycle` fixes the executable sync/RTC ordering: pause, reset inference
and processors, reset interpolation, invalidate cached observation, publish a fresh start
observation, then resume. Episode start uses the same complete flush/reinvoke pair.
`RecoveryAwareActionGate` now enforces causal score-before-action-provider dispatch, so a
triggering frame cannot pop or compute and send a stale action; only successfully sent and
read-back commands enter history. `PolicyTickBudget` counts each autonomous observation
once while excluding synchronous recovery motion.
`OnlineD0ObservationScorer` now supplies the concrete live D0 boundary: it reads raw
`gripper.current`, conditions on the last normalized `gripper.pos` actually returned by
the follower after safety clipping, exposes first-frame warm-up as unscorable, and clears
history on episode/recovery reset. Raw `Goal_Position` readback remains exclusively for
recovery replay. The gate fails closed rather than guessing when the follower returns no
accepted action.
The RTC trace found that upstream `pause()` alone is racy: a producer already inside
inference may merge a stale chunk after queue reset. `RecoverySafeRTCInferenceEngine` now
uses generation-bearing producer cursors and rejects every merge spanning suspend/reset or
resume; a deterministic threaded test covers the active-producer ordering. The lifecycle
also reseeds D0 from the fresh normalized start observation before action production
resumes. A recovery-enabled RTC adapter must use this guarded engine, not upstream RTC
directly.
The complete mocked sync and guarded-RTC loop acceptance test now passes. It composes D0,
the action gate, supervisor, lifecycle, interpolation reset, policy-tick budget, and RTC
queue across a trigger and successful reinvocation. Sync performs no trigger-frame
compute/pop and drops the old chunk; RTC rejects an in-flight pre-trigger merge that
finishes after recovery and sends only a newly published action.
The joined-evidence software contract is also complete: supervisor event schema 2 carries
canonical run/episode/attempt IDs; immutable manifests freeze checkpoint, revision,
inference mode, chunk settings, and config hashes; each recovery frame carries the same
identity; terminal outcomes are append-only; and `audit_recovery_evidence.py` rejects
missing, orphaned, duplicated, or conflicting joins. A synthetic complete bundle passes.
The composed retry-cap test now runs two real D0-triggered recovery/reinvoke cycles and a
third stable-command current drop. Recoveries one and two resume; trigger three enters the
terminal exhausted state before physical execution, policy access, or action send.
`RecoveryControlLoop` counts that exception frame in `PolicyTickBudget`, closing the hole
in the earlier consume-after-return pattern.
Connecting that gate exposed and fixed D0's sole warm-up frame: it is now explicitly
unscorable (`None` online, `NaN` plus `scorable=false` in batch) rather than a zero anomaly
score. D0r was rechecked independently and remained byte-equivalent in masks/triggers with
zero score delta on four development runs.

**Still open:** these primitives are deliberately not attached to
`rollout_with_telemetry.py` while all
routes remain physically unvalidated. The live adapter must also preserve policy-execution
time budgets in the live strategy, join phase telemetry to episode outcomes on real data, prove no stale chunk
action escapes, select the guarded RTC engine in the live factory, and pass the remaining
hardware and live joined-evidence tests in
`CLOSED_LOOP_RECOVERY_INTEGRATION.md`. Do not describe
detector-triggered hardware recovery as completed.
