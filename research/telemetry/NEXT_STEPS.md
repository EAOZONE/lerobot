# Next steps

Completed infrastructure has moved to [`DONE.md`](./DONE.md). Weekly evidence is in
[`WEEK1_REPORT.md`](./WEEK1_REPORT.md) and [`WEEK2_REPORT.md`](./WEEK2_REPORT.md). Research
framing and the schedule are in [`Research.md`](./Research.md) and
[`vla-failure-detection.md`](./vla-failure-detection.md). This file contains only
unfinished work.

## Where this sits against the schedule

`vla-failure-detection.md` §7.1 anchors Week 1 to 3–9 August 2026 and says to shift
uniformly if you start later. Work started **earlier**: the Week 1 gate closed 25 July and
Week 2's software was built 26–28 July. Against the anchor the project is roughly two weeks
ahead, and the nominal Week 3 pilot does not begin until 17 August.

That buffer is real but it is not spendable on analysis. Everything currently in progress
is desk work that consumes no robot time and no calendar. The two items that *do* have
external lead time — spare servos and task objects — are the two nobody has started, and
they gate the pilot absolutely. **Procurement is the critical path, not D0r.**

The three Week 2 exit criteria that remain unmet are the reset soak, camera rigging, and
telemetry↔frame alignment verification. See `WEEK2_REPORT.md` §1 for the full scorecard.

## Current gates

| order | gate | robot | lead time | status |
|---:|---|:---:|:---:|---|
| 1 | order servos and task materials | no | **external** | **not started — gates everything** |
| 2 | redesign arbitrary-pose recovery path | partly | none | **safety blocker** |
| 3 | rig cameras; photograph reference views | yes | none | not started, Week 2 deliverable |
| 4 | decide the frame-alignment audit trail | no | none | not started, blocks corpus |
| 5 | measure policy inference latency | no | none | **done 28 Jul** |
| 6 | validate redesigned reset, 10 consecutive repeats | yes | none | blocked by gate 2 |
| 7 | write labeling guide and annotation schema | no | none | can start now |
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

---

## 2. Redesign arbitrary-pose recovery before the reset exit test

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

Health check first:

```bash
python research/telemetry/diagnose.py --port /dev/ttyACM0
```

Then the soak, workspace clear, hand near the power switch for the first repeat:

```bash
python research/telemetry/recovery.py reset --port /dev/ttyACM0 \
    --home research/telemetry/runs/reset_home.csv --repeat 10 \
    --log research/telemetry/runs/reset_soak.csv
```

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

### 3.2 Decide the frame-alignment audit trail

`SOFollowerTelemetry.get_observation()` reads position, the telemetry block, and both
cameras in one call, so telemetry and image share a control step by construction. But the
dataset's `timestamp` column is synthetic (`frame_index / fps`) and always reads as a
perfect 30 Hz, so neither jitter nor a dropped frame is visible in the recorded corpus.
Week 2's exit criterion — alignment verified to within one control step — cannot currently
be checked at all, before or after the fact.

Adding a wall-clock column to `observation.state` would break the 30-dim freeze. A
per-episode sidecar keyed by frame index would not. Pick one and implement it before any
corpus episode is recorded, or record an explicit decision that alignment will be argued
from construction rather than measured.

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

**Also open:** D0r's coverage check is a nearest-neighbour search against the whole stored
training reference, so its per-frame cost grows linearly with the free-space training set.
Today it is negligible. Re-measure at corpus scale before quoting D0r's cost as free.

### 3.4 Write the schema document

The 30-dim freeze is real but is described across `WEEK1_REPORT.md` §7, `DONE.md`, and
`README.md`. §7.1 lists "a frozen data schema document" as a Week 2 deliverable and it is
also a released-artifact item. One page, one file.

### 3.5 Write the labeling guide before any labeling begins

Scheduled for Weeks 7–9, but it is zero-robot desk work and §7.1 is explicit that a
taxonomy fitted to data you have already seen is not a taxonomy. Write it now, while
blocked. It must define:

- outcome and fine-grained fault codes `S1–S3`, `E1–E6`, and excluded hardware events
  `H1–H2`;
- collapsed semantic/execution analysis labels;
- onset frame: first frame at which failure becomes inevitable;
- unrecoverable boundary under the fixed scripted recovery;
- uncertainty interval for ambiguous onset;
- recoverable deviations that must remain negatives;
- multiple-event rollouts and which event owns a detector trigger;
- second-annotator procedure for a 15% stratified subset.

Freeze the machine-readable annotation schema at the same time. At minimum record: episode
ID, task, arm, split, session, policy checkpoint, seed; success, fine class, collapsed
class, onset frame/time, unrecoverable frame/time; **grasp start/end and duration**;
recovery eligibility and retry count; session start/end temperature; camera-alignment check;
disturbance type and measured magnitude.

Define train/calibration/test splits by trajectory or episode group before fitting anything.
Near-duplicate trajectories must never cross splits. Threshold selection uses calibration
only; held-out test conditions stay untouched until the detector suite is frozen.

### 3.6 Arrange a second annotator

Week 9 needs an independent annotator on a 15% subsample and a reported Cohen's κ. This is a
person-dependency with its own lead time and nobody has arranged it. §7.1's exit criterion
is κ above roughly 0.7 on class labels, with a re-label if onset agreement is poor — so the
second annotator must be available *during* Week 9, not after.

### 3.7 Watch the workshop deadline

§7.2 targets a CoRL 2026 workshop submission in late September / early October, with the
event on 9 November in Austin. Individual workshop deadlines are set by organizers and are
typically 4–6 weeks before the event. Nobody has looked them up. The submission content —
taxonomy, telemetry signatures, D0/D0r/D0+ preliminary results — is closer to ready than
the schedule assumes, so this is worth pinning to a real date.

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
3. **Corpus requirements for calibration** specified in §6 of that spec: verified-clear
   flag, independence group, session, temperature, payload state, split assignment. These
   must land in the labeling guide (§3.5) before collection, not after.
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

Collect 30 randomized T1 demonstrations with the frozen 30-dimensional telemetry schema.
Train the Arm 1 six-dimensional policy with `TruncateStateStep(keep=6)`, then run roughly 30
autonomous evaluations.

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

The current reset CLI validates halt and return-home mechanics only. Integration with policy
chunk flushing, recent-command reversal, gripper reopen, retry accounting, and policy
reinvocation remains future closed-loop work; do not describe those pieces as completed.
