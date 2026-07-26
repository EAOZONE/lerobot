# Next steps — Week 2

Week 1 passed the gate on both failure modes. Background:
[`WEEK1_REPORT.md`](./WEEK1_REPORT.md) for the gate result,
[`SESSION_2026-07-26.md`](./SESSION_2026-07-26.md) for the causal re-evaluation and the
free-space model, [`README.md`](./README.md) for the tooling reference.

You are running roughly a week and a half ahead of the proposal's 3 Aug anchor. Spend it
on Week 2 infrastructure rather than rushing the pilot — §5.1 is right that retrofitting
telemetry synchronisation later is miserable.

---

## The study was reframed after Week 1 — read this first

The revised proposal and idea doc turn this from a detector benchmark into a **2×2 over
where the free current channel goes**. Two independent choices — does the *policy* see the
telemetry, and does a *monitor* see it — crossed:

| | No detection | With detection |
|---|---|---|
| **Policy sees 6 dims** | **Arm 1** — baseline | **Arm 2** — monitoring |
| **Policy sees 30 dims** | **Arm 3** — conditioning | **Arm 4** — both |

- **Arm 1** — reference success rate, and the source of the labelled failure corpus.
  Difficulty is calibrated here and nowhere else.
- **Arm 2** — the original project. Carries the whole detector ladder and the closed-loop
  experiment.
- **Arm 3** — SmolVLA fine-tuned on the full 30-dim observation, no detectors. End-to-end
  success rate and failure composition only. Marginal cost: one fine-tune plus ~120
  rollouts (~6 robot hours), scheduled Week 11 when the robot is otherwise idle.
- **Arm 4** — the interaction cell. If the conditioned policy already modulates grip from
  current, the monitor may have nothing left to catch. Optional, and the most novel cell.

**This costs almost nothing extra because the schema is already 30 dims and truncates to 6
at training time.** One demonstration corpus trains both policies; the expensive component
— 180 teleoperated episodes — is paid once. `TruncateStateStep(keep=6)` is the **arm
switch**: present in the Arm 1/2 pipeline, deliberately absent in Arm 3's.

### Two rules the factorial imposes

Both are easy to violate by accident and neither is recoverable after the fact.

- **Difficulty is calibrated once, on Arm 1, then frozen.** Arm 3 will land at a different
  failure rate. Do not re-tune it back into band — its rate is a *result*, not a parameter.
- **Episode budget is held constant in policy execution time, not wall-clock time.**
  Recovery burns seconds; a fixed wall-clock cap gives Arm 2 less policy time than Arm 1
  and makes a null result uninterpretable.

### Where the prior-art correction left things

Proprioceptive monitoring of learned policies is established (arXiv 2509.26308), as are
force-conditioned policies (TA-VLA, FACTR 2) and IL failure detection (FAIL-Detect,
Rewind-IL). The contribution is a *cross-literature comparison*, not a discovered signal.
The practical inheritance was FACTR 2's free-space subtraction, now built and measured —
see the session log.

---

## Status

| # | item | status |
|---|---|---|
| — | servo health ritual, schema freeze, 30 fps verification | done 25 Jul |
| — | causal re-evaluation of the Week 1 analysis | done 26 Jul — **passed** |
| — | free-space current model (D0r) | done 26 Jul — **3/3 collisions, no matched control** |
| 1 | reset automation, built as the recovery routine | **todo — do first** |
| 2 | online D0 + duration-only baseline + detector interface | **todo** |
| 3 | trajectory hold-out for D0r | **todo — cheapest confidence gain available** |

Item 2 needs no robot. Items 1 and 3 need the robot but nothing to manipulate. The
ordering keeps the material order off the critical path.

Run everything from the `lerobot` conda env (`conda activate lerobot`) at the repo root.
Scripts marked **(to write)** do not exist yet — the command is the interface to build
against, not something that works today.

| § | script | exists? |
|---|---|---|
| 0 | `diagnose.py` | yes |
| 1 | `recovery.py` | **to write** |
| 2 | `detectors.py` | **to write** |
| 3 | `log_teleop_telemetry.py` + `freespace_model.py` | yes |

## Materials — still outstanding

**Blocked on materials, not setup.** Teleoperation is verified end to end. Week 3 needs
the task objects, so order them now (§5.2/§5.3 imply the list):

- **Two spare STS3215 servos** (§9). Two latched alarms in one week is the signal that
  lead times matter.
- **T1** — a ~2–3 cm cube, and a bowl.
- **T2** — cylindrical markers in **two surface finishes**, smooth plastic *and*
  rubber-gripped, since friction is the slip lever and §5.5 wants a deliberately slippery
  variant to generate `E2` at volume as a separate stratum. Plus a cup with adjustable rim
  height.
- **T3** — a target block and 3 distractors at **two contrast levels**: obviously
  different (red among blue) and near-identical (red among maroon). That pair is what
  dials semantic failure rate without changing anything else.

Buying both variants of T2 and T3 now makes Week 3's difficulty tuning a one-session job
rather than a re-order.

---

## 0. Session ritual — established, run every session

```bash
python research/telemetry/diagnose.py --port /dev/ttyACM0
```

Prints the raw alarm byte per servo (the `scservo` SDK names only bits 1/2/4/8/32, so
anything else surfaces as an empty error string) plus per-servo temperature. A latched
servo rejects every write until the **power supply** is cycled — unplugging USB is not
enough. Run it at session start and after any run that ends in a stall.

Because `Present_Temperature` is deliberately not in the recorded schema, this is also
where session temperature gets captured. Note it in the spec sheet each session — that
series is what explains failure-rate drift in Week 6+ (§5.3), and it is worthless unless
started now while the arm is known-good. It is also the only handle on whether the D0r
floor drifts thermally.

**Frozen schema, for reference.** `observation.state` is 30-dim:
`[0:6] pos · [6:12] load · [12:18] current · [18:24] vel · [24:30] volt`. Verify any new
dataset carries it:

```bash
python -c "
import json, glob
d = sorted(glob.glob('/home/ben/.cache/huggingface/lerobot/ben/telemetry-smoketest_*'))[-1]
i = json.load(open(d + '/meta/info.json'))
print('robot_type:', i['robot_type'], '| frames:', i['total_frames'])
for k in ('observation.state', 'action'):
    print(k, i['features'][k]['shape'])"
```

Expect `observation.state (30,)` and `action (6,)`. Anything else means the wrapper robot
type did not register and the episode is not corpus-compatible.

---

## 1. Reset automation, built as the recovery routine (~half a day)

§7 calls this the highest-leverage engineering investment available, and the arithmetic
supports it: ~1,070 rollouts plus 180 demonstrations at ~1.5 min each is ~28 hours of
robot time before overhead, and manual reset is most of it.

**Build it once and use it twice.** The revised design specifies the closed-loop recovery
routine precisely, and it is nearly the same machinery as a between-episode reset:

1. **Halt by writing present position as goal.** Not by ceasing to send actions — a
   position-controlled servo keeps driving toward its last goal, which is the mechanism
   behind the 12A `pair3` stall. Not by disabling torque, which drops the arm under
   gravity.
2. Flush the queued action chunk.
3. Retract by replaying recent commanded positions in reverse.
4. Return to the episode start pose. Deliberate: the policy was trained on episodes
   beginning near home, so resuming mid-trajectory measures out-of-distribution behaviour
   rather than recovery.
5. Reopen the gripper, re-invoke the policy. At most 2 retries per rollout.

Steps 1, 4 and 5 are exactly what a reset needs. Write them as one module now and the
Week 12–13 closed-loop experiment inherits a tested implementation.

**Run** — record the home pose and one reset trajectory with the existing logger, then
drive them with `recovery.py` **(to write)**:

```bash
# teleoperate to the episode start pose and a short retract path; hold still, then stop
python research/telemetry/log_teleop_telemetry.py \
    --follower-port /dev/ttyACM0 --leader-port /dev/ttyACM1 \
    --out research/telemetry/runs/reset_home.csv
```

```bash
# halt semantics on their own, arm parked wherever it is -- verify current drops and the
# arm neither sags nor keeps driving. Do this before anything that moves.
python research/telemetry/recovery.py halt --port /dev/ttyACM0

# full routine, no policy attached
python research/telemetry/recovery.py reset --port /dev/ttyACM0 \
    --home research/telemetry/runs/reset_home.csv --retract-frames 30

# what Weeks 12-13 will call: 10 consecutive resets, unattended
python research/telemetry/recovery.py reset --port /dev/ttyACM0 \
    --home research/telemetry/runs/reset_home.csv --repeat 10 \
    --log research/telemetry/runs/reset_soak.csv
```

**Exit criterion** (§7 of the proposal): 10 consecutive resets with minimal manual
intervention. The soak log is also the check that repeated resets are not heating the
gravity-loaded `shoulder_lift` — run `diagnose.py` immediately after it, since that joint
is the one that latched an alarm in Week 1.

---

## 2. Detector interface — online D0, duration baseline, D0+ features

The ladder, cheapest first, so the paper can ask what each extra unit of compute buys:

| rung | detector | cost | status |
|---|---|---|---|
| — | **duration-only trivial baseline** | none | todo — ~20 lines |
| **D0** | conditioned current test (grip state + commanded motion + smoothed window) | free | rule validated causally; needs the shared interface |
| **D0r** | free-space current residual | low | done — `freespace_model.py` |
| **D0+** | telemetry classifier — logistic regression / small MLP | near-free | features writable now, needs corpus to train |
| D1 | action-chunk consistency (chunk at *t* vs *t+k*) | cheap, no extra forward passes | Week 10 |
| D2 | perturbation disagreement | *N*× inference | Week 11 |
| D3 | supervised latent probe on SmolVLA's action-expert latents | moderate | Week 11 |
| D_fusion | best proprioceptive ⊕ best model-internal | — | Week 11 |
| D_oracle | human labels, upper bound for the closed loop | — | Week 12–13 |

**The point of this module is one interface.** Every rung emits a per-frame score through
the same API, so Week 11 is purely additive rather than an integration scramble.

**The duration-only baseline is not a joke.** If a latent probe cannot beat a detector
fitted to elapsed time alone, that is worth knowing and reviewers will ask.

**D0 is settled, mechanically.** Window 5–10 frames, measured not guessed; separation
316–365 against clean 0–6; the *command* condition is what carries it. See the session log
for why the window has a hard upper bound.

**D0+ needs labelled data**, so training is Week 7–8 — but feature extraction can be
written now against the Week 1 CSVs. It is also where the long-window hypothesis from the
industrial AD literature belongs, since that result concerns a learned autoencoder over a
window rather than a conditioned rule.

**Run** — `detectors.py` **(to write)**:

```bash
# every implemented rung scores the same runs, one row per detector
python research/telemetry/detectors.py score research/telemetry/runs/*.csv \
    --detectors duration,d0,d0r \
    --model research/telemetry/models/freespace.npz \
    --out research/telemetry/runs/scores.parquet

# AUPRC, false alarms per run, lead time vs the keypress markers
python research/telemetry/detectors.py report research/telemetry/runs/scores.parquet

# D0+ feature extraction -- writable now, trainable in Week 7-8
python research/telemetry/detectors.py features research/telemetry/runs/*.csv \
    --window 10 --out research/telemetry/runs/features.parquet
```

On the Week 1 data `report` will produce numbers with n=3 per class and no honest
confidence interval. That is expected — the job now is to fix the *interface* and confirm
the scores are causal, not to produce results. Treat any AUPRC printed at this stage as a
smoke test.

**Settle the metric before writing evaluation code.** The two revised docs disagree: the
proposal says AUROC plus TPR at fixed false-alarm rate, the idea doc says PR curves, AUPRC,
and "fraction of failures detected early enough to run the recovery routine". At ~50%
failure with sparse onset frames, AUPRC is the right call and the operational metric is the
one that answers RQ4. Pick it once so every rung reports the same thing.

---

## 3. Trajectory hold-out for D0r (one session, no task objects)

The cheapest thing that would materially raise confidence in D0r, and it should ride along
with the next robot session.

All three collision pairs replay the **same** recorded episode — one trajectory, one joint,
one corner of the workspace. So D0r has been asked "can you spot a contact on this one
motion" three times, not "can you spot contacts". Corpus rollouts never repeat a
trajectory.

**Do:** collect free-space data on a *different* sweep (trajectory B), and record a fresh
matched pair — obstacle and clear — on trajectory B as well. Then fit on A, test on B, and
fit on B, test on A. Also vary the joint: every contact so far is `shoulder_pan`.

```bash
# trajectory B free-space, then a matched pair on the same trajectory
python research/telemetry/log_teleop_telemetry.py --follower-port /dev/ttyACM0 \
    --leader-port /dev/ttyACM1 --out research/telemetry/runs/freespace_b_01.csv

python research/telemetry/log_teleop_telemetry.py --follower-port /dev/ttyACM0 \
    --replay-dataset ${HF_USER}/sweep_b --replay-episode 0 \
    --out research/telemetry/runs/pair4_clear.csv
python research/telemetry/log_teleop_telemetry.py --follower-port /dev/ttyACM0 \
    --replay-dataset ${HF_USER}/sweep_b --replay-episode 0 \
    --out research/telemetry/runs/pair4_obstacle.csv

# cross-trajectory: fit on A, score B
python research/telemetry/freespace_model.py fit research/telemetry/runs/freespace_0*.csv \
    --out research/telemetry/models/freespace_a.npz
python research/telemetry/freespace_model.py eval --model research/telemetry/models/freespace_a.npz \
    research/telemetry/runs/pair4_clear.csv research/telemetry/runs/pair4_obstacle.csv
```

**Pre-register the floor percentile before running the eval.** p99 was chosen on the
existing seven runs by sweeping and picking the best answer; repeating that on new data
would make the result meaningless. Write the choice down first.

Keep collisions at `pair1`/`pair2` intensity. The `pair3` exemplar is captured and does not
need repeating — it drew ~12A for 2.3s, which is how these servos burn out.

Two more that need no extra session, if the opportunity arises: a second payload mass (a
heavier object may read as contact), and refitting on a *later* session's free-space data
to see whether the floor drifts thermally.

---

## Then: Week 3 pilot

30 teleoperated demos on T1, fine-tune SmolVLA, measure the failure rate and tune it into
the 40–60% band. Gate: **difficulty locked**.

Three things to carry in:

- At n=30 the failure-rate estimate carries roughly ±18 points (§5.3). Land in band and
  move on; don't burn days chasing a number the sample size can't resolve.
- Prefer physical difficulty levers over checkpoint selection. An undertrained policy fails
  by flailing, and Week 1 showed how easily an unrepresentative failure mode produces a
  signature that doesn't generalise. You want *crisp* failures.
- **Decide multi-task versus per-task here.** The idea doc prefers a single multi-task
  policy, because the learned detectors (D3 especially) are policy-specific and splitting
  the corpus across three networks leaves too few failure instances to fit a latent probe.
  Train both ways on T1 and compare failure rate *and failure character* before committing.

Whatever difficulty is locked here is locked for **Arm 1 only**. Arm 3's failure rate is a
result.

```bash
# Arm 1 policy: TruncateStateStep(keep=6) IN the pipeline
lerobot-train --policy.path=lerobot/smolvla_base \
    --dataset.repo_id=ben/t1-cube-bowl --steps=20000 \
    --output_dir=outputs/train/t1_arm1

# Arm 3 policy: same data, same seed, truncation REMOVED -- Week 6, not now
```

Run both from the same config with one flag between them, not two hand-edited copies.

## Corpus requirements accumulated so far

Things decided since Week 1 that the recording setup must honour:

- **Record grasp duration per episode.** D0's window has a hard upper bound set by grasp
  duration; stating it from data rather than three runs needs this column.
- **Record session temperature** at start and end (§0), for the drift question and for
  D0r's floor stability.
- **Negatives must include autonomous rollouts**, not only teleoperation — that is the
  distribution the detectors actually face.

## Open questions worth one run each

- **Slip precursor.** In `slip_a` cycle 1, gripper current drifted 344→337 over 1.6s before
  letting go. If that's micro-slip rather than thermal drift it would buy far more lead
  time than the 0.3–1.1s measured — which matters, because smoothing already spends some of
  that budget. Grip firmly, wait for `GRIP OK`, hold 2s, then tilt so the object creeps:

  ```bash
  python research/telemetry/log_teleop_telemetry.py \
      --follower-port /dev/ttyACM0 --leader-port /dev/ttyACM1 \
      --out research/telemetry/runs/slip_slow_a.csv
  python research/telemetry/plot_signatures.py \
      research/telemetry/runs/slip_slow_a.csv --motor gripper
  ```

  Thermal drift is monotonic over the whole hold; micro-slip should appear only in the
  seconds before release. Run a matched no-slip hold of the same duration to tell them
  apart — without it, any slow decline is ambiguous.

- **`Goal_Position_2`** (addr 71, read-only) — unknown semantics, possibly the interpolated
  setpoint. If it is, it is a better "commanded position" than `Goal_Position` for both
  D0's command condition and D0r's features:

  ```bash
  python research/telemetry/scan_registers.py --port /dev/ttyACM0 --motor gripper --seconds 6
  ```

- **`Goal_Position`** (addr 42) is already logged by the telemetry logger but is not in the
  30-dim recorded schema. Position error (goal − present) is a named D0+ feature in the
  revised proposal. Widening the block read to 42–70 pulls it into the same transaction for
  free (~210 bytes against a 250-byte packet limit); normalise through `bus._normalize` so
  it shares a scale with `pos`. Note following error is also large during normal fast
  motion — the discriminating pattern is error that *persists while velocity is near zero*.
  Revisit only if D0+ underperforms; changing the schema now would split the corpus.

- **`gate_analysis.py:34`** still hardcodes `SMOOTH_FRAMES = 5` with `center=True` in
  `report_matched_pairs`. Its numbers are sound — the session log confirms centred and
  trailing agree — but leaving a non-causal window in the analysis path invites reuse in a
  detector. Worth a comment at minimum.
