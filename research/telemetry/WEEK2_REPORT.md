# Week 2 — infrastructure: **detectors delivered, hardware gates NOT met**

**Dates:** 26–28 July 2026 · **Platform:** SO-101 follower, Feetech STS3215, 30 Hz
**Proposal week:** §7.1 Week 2 — "Infrastructure"
**Stated exit criterion:** *"You can run 10 consecutive episodes with minimal manual
intervention, and every episode's telemetry aligns with its frames to within one control
step."*

Neither half of that criterion is met. The week nonetheless delivered more than it was
scoped to: the full cheap end of the detector ladder exists, is causal, is tested, and has
already been through one pre-registered hold-out — which it failed, informatively.

The gate table's Week 2 milestone (**schema frozen**) is met and was met in Week 1.

---

## 1. Scorecard against the proposal

| Week 2 requirement (§7.1, §5.1) | Status |
|---|---|
| Telemetry logged on shared timestamps with frames and actions | **done** — one block read inside `get_observation()` |
| Data schema frozen and documented | **done** — 30 dims; documented across three files, no single schema doc |
| Reset routine that runs unattended between episodes | **implemented, not validated** — safety defect found |
| 10 consecutive episodes, minimal intervention | **NOT MET** — blocked on recovery redesign |
| Telemetry↔frame alignment verified to within one control step | **NOT MEASURED** — and not measurable from the corpus as recorded (§5) |
| Mount and tape cameras; photograph reference views | **NOT STARTED** |
| Order two spare STS3215 servos (a Week 1 item) | **NOT STARTED** |

Work ahead of schedule: D0r, the common detector interface, D0+ feature extraction, and a
pre-registered generalization test are all Week 7–8 deliverables in the proposal. They
exist now.

Work behind schedule: everything that touches the physical rig.

**Calendar position.** §7.1 anchors Week 1 to 3–9 August and says to shift uniformly if
you start later. Work started *earlier* — the Week 1 gate closed 25 July, before the
anchor's Week 1 begins. There is roughly a two-week buffer against the nominal Week 3
pilot start (17 August). That buffer is real, but it can only be spent on work that does
not have external lead time, and the two items with external lead time (spare servos, task
objects) are precisely the two nobody has started.

---

## 2. The week's most valuable result is a failed hold-out

D0r — the free-space current residual — was built on 26 July, and by 27 July it had
detected all three Week 1 collisions without the matched-trajectory twin that produced the
Week 1 numbers. That closed the largest gap Week 1 left open, and the proposal's "no
matched control exists at runtime" risk row.

Then it was pre-registered and tested properly, and it failed.

`D0R_HOLDOUT_PROTOCOL.md` locked the p99 floor, 10-frame smoothing, 25-frame warm-up,
3-frame persistence, contact intensity, and a target joint other than `shoulder_pan`
*before* trajectory B was recorded. The primary condition was a sustained crossing on the
contacted joint in `pair4_obstacle` and no crossing in the command-identical
`pair4_clear`.

| run | peak × p99 floor | peak joint | alert episodes | verdict |
|---|---:|---|---:|---|
| `pair4_obstacle` | 21.67 | `shoulder_pan` | 9 | contact |
| `pair4_clear` | **4.32** | `wrist_roll` | **6** | **false positive** |

The obstacle response was strong and — a genuinely new result — transferred to
`shoulder_lift` (7.46×, 8.24×) and `elbow_flex` (18.48×, 12.95×), so the Week 1 finding is
not a `shoulder_pan` artefact. But specificity failed, and no percentile was retuned after
seeing it. Full record: [`PAIR4_HOLDOUT_RESULT.md`](./PAIR4_HOLDOUT_RESULT.md).

**Why this is worth more than a pass would have been.** A pass at n=1 trajectory would have
been indistinguishable from luck, and the project would have carried an unvalidated
detector into a 450-rollout corpus. Instead the diagnosis
([`D0R_CLEAR_DIAGNOSIS.md`](./D0R_CLEAR_DIAGNOSIS.md)) identified a defect that would have
contaminated every downstream false-alarm number:

**The thresholding unit was wrong.** A p99 *frame* floor permits ~1% of ordinary frames to
cross. At 30 Hz that is ~18 exceedances per minute before accounting for temporal
correlation, and three-frame persistence does not fix it because residuals around
direction changes are correlated by construction. The proposal's operational constraint
(§6) is false alarms **per rollout**. The calibration unit must therefore be the maximum
sustained score per independent clear rollout — which is now what
`freespace_model.py calibrate` computes, and it refuses to write a model when
underpowered rather than silently falling back to frame percentiles.

Two further defects, both now instrumented rather than patched:

- **Command-space coverage.** B frames sit far outside A's training support — B→A nearest-
  neighbour distance p95 is 12.76 against an A→A p99 of 0.82, and some gripper features
  land 86–112 training standard deviations out. The model now stores a coverage reference
  and **abstains** on out-of-support motion instead of reporting extrapolation as contact.
- **Validation leakage.** Contiguous-block residual folds leave other frames from the same
  run in the training set. `freespace_model.py` now uses leave-one-whole-run-out validation
  whenever multiple no-contact runs are supplied.

Note what the coverage diagnostic then showed: pair4 clear is scored *in* coverage and
still crosses at 2.2×. Extrapolation is a real problem but it is not the whole problem.
The remainder is ordinary independent-repeat variability, which only rollout-level
calibration addresses.

---

## 3. `Goal_Position_2` — a clean negative, recorded 28 July

The diagnosis left one physical hypothesis open: that repeat-to-repeat current differences
under identical commands come from the servo's internal interpolated setpoint, which the
external `goal_pos` trace cannot see. Feetech exposes a read-only `Goal_Position_2` at
register 71, and it sits inside the existing block read — two extra bytes, no extra bus
transaction, no change to the 30-dim schema.

Two obstacle-free replays of trajectory B were recorded with it (`clear_goal2_a/b.csv`,
649 frames, external commands byte-identical, timestamps within 5 ms) and analyzed with
`analyze_goal2.py`.

**Register 71 reads a constant 0 on every motor in every frame of both runs.**

| joint | goal2 range A/B | frames equal | p99 \|Δgoal2\| | p99 \|Δcurrent\| |
|---|---:|---:|---:|---:|
| `shoulder_pan` | 0/0 | 100.0% | 0.0 | 22.5 |
| `shoulder_lift` | 0/0 | 100.0% | 0.0 | 56.6 |
| `elbow_flex` | 0/0 | 100.0% | 0.0 | 38.6 |
| `wrist_flex` | 0/0 | 100.0% | 0.0 | 17.0 |
| `wrist_roll` | 0/0 | 100.0% | 0.0 | 14.0 |

At the largest paired wrist-current divergence (11.302–11.603 s, trailing-mean Δcurrent
11.5) the maximum |Δgoal2| is 0.0 ticks. The hypothesis is dead.

The read is aimed correctly — LeRobot's own table puts `Goal_Position_2` at (71, 2)
(`src/lerobot/motors/feetech/tables.py:91`) and the block spans 56–72 — so this is a
firmware fact, not an addressing bug. Note that `verify_block_read` will *not* establish
this on its own; its 30-tick tolerance for `goal2` passes a 0-versus-0 agreement silently.

**Confirmed on hardware the same day.** `probe_goal2.py` read register 71 directly at a
nonzero standstill pose — all six motors between 710 and 3692 ticks, register 71 reading 0
on every one — and through 292 samples over 10 s of motion, which produced exactly one
distinct value per motor: 0. This used LeRobot's per-register `sync_read` rather than the
block read's sub-address extraction, so two independent read paths agree while that same
path returns sensible values for `Present_Position`. The register is not being misread; it
is not being written. See `D0R_CLEAR_DIAGNOSIS.md` for the table and for the one limitation
of the standstill condition.

**Consequence.** Strike "internal interpolated setpoint" from the candidate causes in
`D0R_CLEAR_DIAGNOSIS.md`. What remains — friction, controller state, thermal state — is
not observable from the bus at all. That closes off the "find the missing feature" route
and leaves rollout-level calibration as the only path, which is exactly where §5 of that
document already pointed.

The second table `analyze_goal2.py` prints (external vs. goal2 prediction error, +71% to
+200%) is a null control, not a finding: feeding a constant into the motion basis collapses
the ridge fit to the training mean. It confirms the analyzer is wired correctly and nothing
more.

---

## 4. Recovery: the lift-first discovery

`recovery.py` implements halt, a feedback-gated entry sequence, and replay of an
operator-demonstrated return path. Halt writes measured present position back as the goal
and deliberately disconnects *without* disabling torque, so the arm holds rather than
falling or continuing toward a stale goal.

Two things were learned on hardware, in order:

**Time-gating the lift is not enough.** The first implementation moved `shoulder_lift`
toward clearance for a fixed duration before starting pan. The shoulder is gravity-loaded
and did not always physically arrive. The gate now reads `Present_Position` and pan cannot
start until measured lift is within tolerance, with a five-second abort. The initial
20-tick tolerance was also too strict — one run halted 29 ticks short — and the default is
now 500 ticks.

**Then the deeper problem.** Rotating one joint is not Cartesian "up". The gripper follows
an arc whose direction depends on the whole arm configuration, and from some poses that arc
travels sideways, downward, into the table, or through a task object. The five-repeat soak
in `reset_soak.csv` only exercised poses that happened to be safe, and it predates the
feedback gate anyway.

**The lift-first entry is therefore an implemented experiment, not a validated recovery
strategy, and must not be run unattended.** This is the week's safety blocker and the
reason the 10-repeat exit test is not merely unfinished but currently unrunnable.

Worth noting: the proposal's own Week 2 text proposed "an IK-based return-to-start", which
is option 2 of the redesign in `NEXT_STEPS.md` §1. The joint-space shortcut was the
deviation; the plan anticipated this.

---

## 5. Findings not recorded elsewhere

**A desync bug that looked exactly like a real negative result.** The first causal D0
implementation smoothed the current channel but checked goal-stationarity over an
*unsmoothed* interval. The two desynchronize by the window lag, so a deliberate release
passes the filter. Clean runs scored 258–305 against slips at 260–349 — complete overlap,
presenting as "the Week 1 rule does not survive causal evaluation", which is a named High
risk in the proposal. It was caught only because window=1 reproduced the offline numbers
exactly, which is impossible if the rule were genuinely non-causal. Both symptom and fix
are in the function docstring, because in a paper this would be indistinguishable from a
genuine negative.

**A verdict column that was right for the wrong reason.** At p99.9, `pair1` and `pair2`
still printed CONTACT — but on `wrist_flex`, over a second away from the real event.
In a summary table that reads as a clean 3/3. Caught only by cross-checking peak time
*and* joint against the matched-pair ground truth. That cross-check is now a required step,
not a courtesy.

**Warm-up frames manufacture false positives.** Zero-initialised velocity/acceleration/lag
features made `pair1_clear` — a run with no contact — score 2.2× at t=0.13 s, outranking
the genuine contacts. The first 25 frames are now explicitly unscorable at fit and eval
alike. A causal detector cannot score before its buffer fills.

**The duration-only baseline ranks well, and that is the point.** On the Week 1 files,
elapsed time alone is a competitive ranker, because the long collision recording makes time
correlate with positive frames. It stays first-class in the interface precisely so this
confound cannot hide in the eventual held-out comparison.

**Event attribution semantics were wrong and are now fixed.** The first report
implementation counted any trigger anywhere in a failed run as a detection. Crossings are
now matched to onsets only inside an explicit window (default −2.0 s to +1.0 s); a crossing
outside it is a false alarm even if the rollout eventually fails. This is what correctly
demotes `slip_a`'s apparent +5.11 s lead — that crossing belongs to the earlier unmarked
slip. **Do not quote +5.11 s.**

**Multi-marker handling.** `pair4_obstacle` exposed that only the first marker was being
retained. All markers are now kept and matched. The resulting 5/5 match must still not be
quoted as recall: the third keypress has no supporting ≥150-tick position divergence and
must not count as a validated collision merely because a key was pressed.

**Mixed globs.** `runs/` holds analysis tables and recovery logs alongside telemetry.
Recovery logs use raw goal ticks and must never reach command-conditioned detectors.
Incompatible artifacts are now skipped with an explicit reason; `--strict` makes any
mismatch fatal for validating a controlled collection job.

**Following error is deliberately absent from D0+ features.** The logger stores `goal_pos`
normalized and `pos` in raw ticks; subtracting them yields a meaningless quantity. Add it
only when both live in a common calibrated unit.

**Telemetry↔frame alignment is satisfied by construction but has never been measured, and
cannot be audited after the fact.** `SOFollowerTelemetry.get_observation()` reads position,
the telemetry block, and the cameras in one call, so a frame's telemetry and image come
from the same control step by construction. But the dataset's `timestamp` column is
synthetic (`frame_index / fps`) and always reads as a perfect 30 Hz, so neither jitter nor
a dropped frame is visible in the recorded corpus. Adding a wall-clock column to
`observation.state` would break the 30-dim freeze; a per-episode sidecar keyed by frame
index would not. This needs deciding before corpus collection, not after.

---

## 6. Cost measurements, and what they are not

Batch-path scoring cost from `scores.parquet`:

| detector | mean ms/frame |
|---|---:|
| `duration` | 0.00003 |
| `d0` | 0.0116 |
| `d0r` | 0.0018 |

These are offline Python measurements over short CSVs, not control-loop benchmarks. The
denominator they need — end-to-end policy latency per control step — was measured on
28 July with `bench_inference.py`, on the real `predict_action` path with the actual
pre/post-processor pipelines.

**RTX 4090, `lerobot/smolvla_base`, 6-dim state, 2 cameras at 480×640, 150 steps per row:**

| `n_action_steps` | mean ms | recompute ms | worst ms | queue read ms | over 33.3 ms budget |
|---:|---:|---:|---:|---:|---:|
| 1 | 81.9 | 81.9 | 105.0 | — | **150/150** |
| 10 | 11.4 | 85.9 | 101.0 | 3.08 | 16/150 |
| 25 | 5.9 | 85.1 | 88.3 | 2.59 | 6/150 |
| **50** | **3.8** | 82.9 | 83.9 | 2.17 | 3/150 |

Four findings, in descending order of consequence:

**Compute is not a constraint on this hardware.** At the released chunk configuration the
mean control step costs 3.8 ms — a sustained 264 Hz against a 30 Hz requirement, roughly 8×
headroom. Nothing in the cost ladder is threatened by policy latency.

**§7.1's single-step warning is confirmed, quantitatively.** `n_action_steps=1` costs 21.6×
more per step (81.9 ms, ~12 Hz) and misses the 30 Hz budget on *every* step. It is the one
configuration mistake that would silently invalidate the whole runtime-cost analysis.

**The mean hides a stall, and this one matters.** Every 50th step recomputes the chunk at
~83–87 ms — about **2.5 control periods**. The loop hitches on chunk refill. Against a slip
lead-time budget of 0.3–1.1 s that is ~8% of the tightest case consumed by a scheduling
artefact, before recovery does anything. **Budget recovery against the worst step, not the
mean**, and log which steps were recompute steps during closed-loop runs.

**Conditioning is free at runtime, as H5 assumes.** A 30-dim state costs 3.8 ms mean and
85.5 ms recompute against 6-dim's 3.8 ms and 82.9 ms — identical within noise, because
SmolVLA pads state to `max_state_dim=32` regardless. H5's cost-asymmetry premise ("Arm 3
pays at training time and nothing at runtime") now has runtime evidence rather than an
assumption.

In that context the detector costs above are 0.3% (D0) and 0.05% (D0r) of a mean control
step. The "cheap" claim in the cost ladder survives contact with a real denominator — with
one scaling caveat: D0r's coverage check is a nearest-neighbour search against the *entire*
stored training reference, so its per-frame cost grows linearly with the free-space training
set. At corpus scale that reference is far larger than today's 1.2 MB and should be
re-measured, or approximated, before it is quoted as free.

Still unmeasured: 20k-step fine-tune wall-clock on this GPU. §5.4 budgets ~4 hours on an
A100 and "proportionally longer on consumer hardware"; Weeks 4–6 change shape if that
number is much worse.

---

## 7. Implications for the proposal

- **Two High risks retired.** "Week 1 separation does not survive causal evaluation" — did
  not materialise; causal separation is 316–365 against 0–6 at the best window. "No matched
  control exists at runtime" — addressed in principle by D0r, with the calibration caveat
  below.
- **A new risk is now the top one, and it is not in §9's table:** *residual detectors
  calibrated on frame percentiles do not control per-rollout false alarms.* It was found
  here at n=2 trajectories. It would otherwise have been found at n=450 rollouts, after the
  corpus was collected.
- **D0's window is settled** at 5–10 frames from measurement, with a mechanism for the
  upper bound — a trailing mean longer than the grasp dilutes held current below threshold,
  so the rule stops arming entirely rather than degrading. `slip_b` drops out first at a
  ~0.37 s grasp. This inverts the 500-sample assumption taken from the industrial anomaly-
  detection literature, which belongs to D0+, not to D0's conditioned rule.
- **§5.6's D0r description needs one sentence added:** the operating point is a per-rollout
  conformal threshold, not a percentile of the residual distribution.
- **Corpus requirements added:** record grasp duration per episode; retain autonomous
  successful rollouts as hard negatives; keep per-joint and event-level detector outputs
  rather than only a whole-run maximum.
- **Lead time remains the binding constraint on H4** and smoothing spends it: `slip_b`
  leads by +0.07 s unsmoothed, +0.03 s at window 5, and **−0.07 s at window 10**. Choose the
  operating point on lead time, report both.

---

## 8. Artifacts

**New code** — `research/telemetry/`:

| File | Role |
|---|---|
| `causal_eval.py` | trailing-window re-run of the Week 1 analysis; window sweep |
| `freespace_model.py` | D0r: fit / eval / **calibrate**; coverage reference and OOD abstention |
| `recovery.py` | halt and feedback-gated reset trajectory replay |
| `detectors.py` | shared causal score / report / features CLI; `OnlineD0` |
| `test_detectors.py` | 14 focused causal, regression, label, and feature tests — all passing |
| `analyze_goal2.py` | paired-replay diagnostic for register 71 |

**Modified:** `feetech_block.py` (block widened to 56–72 for `goal2`),
`log_teleop_telemetry.py` (CSV replay path, `goal2.*` columns). Nothing in `src/lerobot/`
was changed in Week 2.

**New data** — `research/telemetry/runs/`:

| File | Role |
|---|---|
| `freespace_01.csv`, `freespace_payload_01.csv` | 113 s trajectory-A no-contact training data |
| `freespace_b_01.csv` | 649-frame trajectory-B no-contact source |
| `pair4_clear.csv`, `pair4_obstacle.csv` | command-identical hold-out pair, 5 markers |
| `clear_goal2_a.csv`, `clear_goal2_b.csv` | matched clear replays with register 71 |
| `reset_home.csv`, `reset_soak.csv` | demonstrated reset path; superseded 5-repeat soak |
| `scores.parquet`, `pair4_scores.parquet` | 35,316 corpus score rows; hold-out scores |
| `causal_sweep.csv` | per-run window sweep |

**Models:** `freespace.npz`, `freespace_a.npz`, `freespace_b.npz`, `freespace_ab.npz`,
`freespace_ab_grouped.npz`, `freespace_ab_coverage.npz`.

**Documents:** `SESSION_2026-07-26.md`, `SESSION_2026-07-27.md`,
`D0R_HOLDOUT_PROTOCOL.md`, `PAIR4_HOLDOUT_RESULT.md`, `D0R_CLEAR_DIAGNOSIS.md`,
`DONE.md`.

Corpus validation totals: 17 telemetry runs scored, 3 detector rungs, 35,316 score rows,
11,772 D0+ feature rows at 225 columns, 14/14 tests passing, Ruff clean.

---

## 9. Still open

Ordered as they block each other; the full plan is in
[`NEXT_STEPS.md`](./NEXT_STEPS.md).

1. **Spare STS3215 servos and T1–T3 task objects are not ordered.** These are the only
   items with external lead time and they gate Week 3. A Week 1 action item is still open.
2. **Recovery path redesign.** Safety blocker. Until it is done there is no unattended
   reset, and without unattended reset the Week 7–8 and 12–13 crunches are not survivable —
   §7.1 estimates reset automation at ~9 hours saved across the project.
3. **Camera mounts, tape, reference photographs.** A Week 2 deliverable. §5.1 names
   viewpoint drift as "the most common silent confound in this literature".
4. **Per-episode timestamp sidecar**, or an explicit decision that alignment will not be
   auditable. Must be settled before any corpus is recorded.
5. **Policy inference latency at real control-loop settings.** The denominator for every
   §6 cost claim. Needs nothing that is not already on this machine.
6. **D0r rollout-level calibration** needs ≥19 independent clear rollouts for a 5%
   distribution-free per-rollout budget. Two exist. This should not be solved with a
   dedicated teleoperation session — see `NEXT_STEPS.md` §2.
7. **Trajectory C** must stay untouched until the model and calibration procedure are
   frozen. Trajectory B is now development data and can never serve as a hold-out again.
8. **A second annotator** for the Week 9 15% subsample and Cohen's κ. A person-dependency
   with lead time that nobody has arranged.
9. **The micro-slip question from Week 1** is still open: gripper current drifted 344→337
   over 1.6 s before release in `slip_a`. If that is micro-slip rather than thermal drift it
   materially changes the lead-time budget H4 depends on. One deliberately slow slide
   settles it.
