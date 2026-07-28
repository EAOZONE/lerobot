# Cheap Signals, Real Failures: Evaluating VLA Failure Detection on Low-Cost Manipulation Hardware

**Research proposal — undergraduate thesis / conference paper**
**Platform:** SO-101 (Feetech STS3215) + SmolVLA
**Primary target:** IROS 2027 (deadline 1 March 2027) — workshop paper en route
**Duration:** 14 weeks of experimental work, plus a writing tail

---

## 1. Abstract

Vision-Language-Action (VLA) models fail often enough in real deployment that runtime failure detection has become an active subfield. Existing detectors draw their signal from model internals — latent representations, action-token distributions, or perturbation-induced disagreement — and are validated in simulation or on research-grade manipulators with precise actuation and dedicated force sensing.

This project asks two questions the literature leaves open. First, **do published VLA failure detectors retain their accuracy on low-cost hardware**, where mechanical backlash, position-only control, and poor repeatability add noise that may swamp the signals these methods rely on? Second, **how much of their performance is recoverable from a nearly free proprioceptive baseline** — servo load and current telemetry, which low-cost serial-bus actuators already report and which no existing VLA failure-detection method appears to use?

We fine-tune SmolVLA on three SO-101 manipulation tasks calibrated to fail 40–60% of the time, collect a labeled corpus of ~950 real rollouts with a structured failure taxonomy, and benchmark a cost ladder of detectors from zero-overhead telemetry to latent-space probing. We then close the loop — triggering scripted recovery on detection — and report the net change in task success rate after accounting for false-alarm cost.

Both outcomes are informative. If cheap telemetry matches sophisticated introspection, that is directly actionable for the low-cost robotics community. If it does not, we have characterized when latent signals earn their computational cost.

---

## 2. Motivation

### 2.1 The reliability gap

VLAs post high success rates on standard simulation benchmarks, but recent robustness analyses show those numbers are fragile. Perturbation studies on LIBERO report success collapsing from roughly 95% to under 30% under modest camera-viewpoint and initial-state changes, and find that many models largely disregard the language instruction, behaving closer to vision-action policies. Recovery from mid-execution error — a slipping grasp, an object nudged out of place — remains a widely acknowledged weak point.

This motivates *runtime* detection: rather than making the policy never fail, detect failure early enough to intervene.

### 2.2 The hardware gap

Low-cost arms like the SO-101 have democratized VLA experimentation, and standardized benchmarks for them now exist. But the failure-detection literature developed on a different hardware profile, and the assumptions differ in kind, not just degree:

| Property | Research-grade arm | SO-101 |
|---|---|---|
| Actuation | Torque control, closed-loop | Position control, serial bus |
| Force sensing | Dedicated F/T sensor | None (load telemetry only) |
| Repeatability | Sub-millimeter | Millimeters, with backlash |
| State estimate | High-fidelity | Noisy, gear-lash dependent |

A state-space anomaly detector tuned where "anomalous state" is a clean signal may behave very differently where normal operation already produces several millimeters of unmodeled slop. This is an empirical question nobody has answered.

### 2.3 The unused signal

Feetech STS3215 servos report present load and present current over the bus at control-loop rates. Physical failure modes have direct mechanical signatures: collisions spike current, slipped grasps produce a characteristic gripper-load drop, jams show sustained high load at near-zero velocity. The signal is free — no extra model, no extra inference, no extra hardware.

No VLA failure-detection method appears to use it. The plausible reason is a selection effect: these methods were developed on platforms with proper force sensing, where a crude proxy was unnecessary. On the platforms most people can afford, it is the only force-adjacent signal available.

---

## 3. Positioning against related work

**Policy-internal detection.** Recent VLA failure-detection work clusters into four families:

- *Latent-representation probes.* SAFE (NeurIPS 2026) performs multitask failure detection from internal VLA representations.
- *Calibrated uncertainty.* ReconVLA (arXiv 2604.16677) applies conformal prediction to action outputs, plus a state-space outlier check.
- *Perturbation-based epistemic uncertainty.* arXiv 2606.20754 perturbs hidden activations and treats prediction disagreement as the failure signal.
- *Rollout-interface methods.* Foresight predicts failure from observations and action chunks alone. FAIL-Detect (arXiv 2503.08558) frames detection as sequential OOD detection under conformal prediction, and Rewind-IL (arXiv 2604.16683) pairs a self-consistency signal with an actual recovery mechanism.

**Proprioceptive detection — the literature this project originally missed.** Force/torque anomaly detection for runtime failure monitoring of learned controllers is established, not novel. arXiv 2509.26308 applies autoencoder-based anomaly detection to force/torque time series with operator-provided temporal onset labels, and evaluates transfer across tasks and across control strategies including diffusion policy. It differs from this study in platform (industrial arm with wrench sensing), task domain, and policy family — but the general approach is prior art, and the earlier claim in this proposal that "nobody has checked the obvious baseline" was wrong.

**Force-conditioned policies — a closed adjacent question.** TA-VLA already uses joint current from low-cost arms as a force-related *policy input*, and FACTR 2 (arXiv 2606.12406) critiques and extends it. Feeding telemetry into the policy is therefore not open, which independently justifies §5.6's state-truncation decision. FACTR 2 also observes that joint current conflates free-space actuator effort with contact-induced torque and that separating them requires temporal context — precisely our matched-control problem, with a validated solution path (subtract a learned free-space prediction).

**Adjacent benchmarks.** Two 2026 papers benchmark VLAs on SO-101: arXiv 2606.08881 (π₀.₅, SmolVLA, Wall-X, ACT with a failure taxonomy and recovery-aware metrics) and VLA-REPLICA (arXiv 2605.20774).

**Our delta, restated honestly.** We do not propose a new detection method, and we do not claim to have found an overlooked signal. Both signal families are established. We contribute:

1. **The first direct comparison** of proprioceptive anomaly detection against VLA-internal detectors, under identical data, splits, and operating points. Neither literature has evaluated against the other.
2. **A cross-hardware evaluation** on low-cost, high-backlash hardware where the proprioceptive channel is uncalibrated servo current rather than a wrench sensor.
3. **Cost-normalized results**, including a trivial duration-only baseline, which neither literature currently reports.
4. **Closed-loop evidence** with false-alarm cost subtracted.
5. **An open labeled corpus** of ~950 annotated SO-101 rollouts with frame-level onset labels and synchronized telemetry.

The prior SO-101 benchmark is a **collaborator, not a competitor** — we adopt its task structure and taxonomy where possible so results are directly comparable.

---

## 3b. Study arms

The study is a 2x2 over where the free telemetry channel is used.

| | No detection | With detection |
|---|---|---|
| **Policy sees 6 dims** | **Arm 1** - baseline | **Arm 2** - monitoring |
| **Policy sees 30 dims** | **Arm 3** - conditioning | **Arm 4** - both (optional) |

- **Arm 1.** SmolVLA on joint positions, no intervention. Reference success rate; source of the failure corpus.
- **Arm 2.** Same policy, detectors reading all 30 dims, scripted recovery on trigger. Carries the full detector comparison.
- **Arm 3.** SmolVLA fine-tuned on the 30-dim observation, no intervention. End-to-end success only, no detector suite.
- **Arm 4.** Conditioned policy plus monitor. Tests whether monitoring retains value once the policy already uses the signal. Run if schedule permits.

**Why this is affordable.** The record schema already logs 30 dims and truncates to 6 at training time, so one demonstration corpus trains both policies. No extra teleoperation. Arm 3's marginal cost is one fine-tune plus ~120 evaluation rollouts (~6 robot hours).

**Deliberate asymmetry.** Detector benchmarking happens only on Arm 2. Arm 3 needs a success rate, not a detector suite. This keeps Arm 3 cuttable without damaging the core result.

**Difficulty is calibrated once, on Arm 1, then frozen.** The conditioned policy will land at a different failure rate and must not be re-tuned back into band - that would destroy comparability. Its rate is a result, not a parameter.

**Episode budget is held constant in policy execution time, not wall-clock time.** Recovery consumes seconds; a fixed wall-clock budget would give Arm 2 less policy time than Arm 1 and make a null result uninterpretable.

---

## 4. Research questions and hypotheses

**RQ1 — Transfer.** Do latent- and action-based detectors maintain their reported discriminative power on SO-101?
> *H1:* AUROC degrades relative to published simulation figures, with the largest drop for state-space anomaly methods, which are most exposed to mechanical noise.

**RQ2 — Cheap baseline.** How does servo telemetry compare to model-internal detectors?
> *H2:* Telemetry is competitive or superior for *execution* failures (collision, slip, jam) and substantially worse for *semantic* failures (wrong object, wrong target), which have no mechanical signature.

**RQ3 — Complementarity.** Does fusing telemetry with a model-internal signal beat either alone?
> *H3:* Yes, because H2 predicts the two cover disjoint failure classes.

**RQ4 — Closed loop.** Does acting on detections improve end-to-end success?
> *H4:* Net gain is positive but modest and highly sensitive to false-alarm rate; there exists a threshold beyond which retry cost exceeds recovery benefit.

RQ2 and RQ3 are the core contribution. RQ1 is the framing. RQ4 is the payoff.

---

## 5. Methodology

### 5.1 Platform

- **Robot:** SO-101 follower + leader (teleoperation), Feetech STS3215 servos
- **Cameras:** wrist-mounted RGB + fixed overhead RGB
- **Compute:** consumer GPU (RTX 3090-class or better)
- **Software:** LeRobot, SmolVLA (`lerobot/smolvla_base`, 450M params, flow-matching action expert)
- **Telemetry:** `present_load`, `present_current`, `present_position`, `goal_position` logged synchronously with camera frames and policy outputs

**Two setup requirements that are easy to get wrong:**

*Camera rigidity.* Camera pose must be physically fixed and marked. Viewpoint drift between demonstration and evaluation is the most common silent confound in this literature. Tape the mounts, photograph the reference view, verify against that reference at the start of every session, and log the check.

*Telemetry synchronization.* LeRobot's default dataset schema captures frames, actions and joint positions — **load and current are not included.** The record loop must be extended to log them on shared timestamps. Do this before collecting anything you care about; unsynchronized telemetry is worthless for onset labeling, and retrofitting synchronization is miserable.

### 5.2 Task design

Tasks must fail at **40–60%** under the fine-tuned policy. Too reliable yields too few positive labels; too unreliable leaves no successful comparison class.

| ID | Task | Primary failure modes | Difficulty lever |
|---|---|---|---|
| **T1** | Pick cube from randomized position → place in bowl | grasp miss, placement miss | randomization radius |
| **T2** | Grasp cylindrical marker → insert into cup | slip, drop in transit | object diameter, surface friction, cup rim height |
| **T3** | Pick target block from cluttered tray, 3 distractors | wrong object, collision | distractor similarity |

T2 is primary for telemetry evaluation (slip has the clearest current signature). T3 is primary for semantic failures. T1 is the calibration/sanity task.

### 5.3 Calibrating the failure rate

Three families of lever, and the choice determines *which failure classes* you generate — which matters more than hitting the number.

**Physical difficulty (preferred).** Produces realistic, well-separated failures. Tune each task with the lever aimed at the class it is meant to probe: object diameter and surface friction for slip (a smooth plastic marker versus a rubber-gripped one moves slip rate dramatically with nothing else changed); distractor similarity for semantic errors (one red cube among three blue is easy; among three maroon generates wrong-object failures on demand); randomization radius and proximity to kinematic limits for both.

**Checkpoint selection (calibration only).** Train once, evaluate intermediate checkpoints, keep the one in band. Costs no extra data, but an undertrained policy fails by flailing — you get mushy, unrepresentative failures. Acceptable for T1 sanity checks, poor as the main lever.

**Data starvation (blunt).** Fewer demonstrations per task. Monotonic and reliable, same unrepresentativeness problem.

**Measurement caveat.** At n=30 pilot rollouts, the failure-rate estimate carries roughly ±18 percentage points. You cannot precisely dial in 50%. Land somewhere in band and accept it rather than burning a week chasing a number the sample size cannot resolve.

**Difficulty spec sheet.** Record exact objects, exact randomization bounds, and setup photographs. Failure rate drifts as servos wear and objects scuff. Re-measure every few weeks and log it — if it wanders out of band mid-corpus you have two incomparable datasets and no way to know.

### 5.4 Data collection

**Stage A — Demonstrations.** 60 teleoperated episodes per task (180 total), with object position randomized across the full workspace region every episode. Without randomization the policy learns a fixed trajectory and both the task and the study become meaningless. ~5 hours of teleoperation.

**Stage B — Fine-tuning.** Fine-tune SmolVLA per task (or multi-task with task-specific instructions), 20k steps — roughly 4 hours on an A100, proportionally longer on consumer hardware. Verify the action-chunk configuration for inference; single-step prediction will cripple runtime performance.

**Stage C — Rollout corpus.** Autonomous rollouts recorded with full video, telemetry, and policy internals:

| Split | Per task | Tasks | Rollouts |
|---|---|---|---|
| Detector training | 100 | 3 | 300 |
| Calibration (conformal / thresholds) | 50 | 3 | 150 |
| Held-out detection eval | 60 | 3 | 180 |
| Closed-loop eval (Arm 2) | 40 × 4 conditions | 2 | 320 |
| Arm 3 end-to-end eval | 40 | 3 | 120 |
| **Total** | | | **~1,070** |

### 5.5 Failure taxonomy and labeling

Adapted from the semantic/execution decomposition used in prior SO-101 benchmarking.

**Semantic** — wrong goal pursued
`S1` wrong object · `S2` wrong target location · `S3` no attempt / stalled before engagement

**Execution** — correct intent, poor execution
`E1` grasp miss · `E2` grasp slip · `E3` collision · `E4` drop in transit · `E5` placement miss · `E6` stall / joint limit / kinematic dead-end

**Hardware** — excluded from analysis, logged separately
`H1` servo overload shutdown · `H2` bus communication dropout

**Statistical power caveat — important.** At ~50% failure across 300 training rollouts you get ~150 failures spread over 3 tasks and 9 classes: roughly 10 instances per class. That is far too thin for the per-class analysis H2 depends on. Two mitigations, both needed:

- **Collapse the analysis taxonomy** to semantic-versus-execution binary, plus the two or three most frequent individual classes. Keep fine-grained labels in the released dataset; simply do not claim per-class results the counts cannot support.
- **Deliberately over-sample target modes.** Run a T2 variant with a deliberately slippery object to generate `E2` instances at volume. Report these as a separate stratum, never mixed into the main success-rate numbers.

**Labeling protocol.** Two passes. Pass 1: rollout outcome and failure class from video. Pass 2: **failure onset frame** — first frame at which failure became inevitable. Onset labeling is subjective; mitigate with (a) an explicit written labeling guide with worked examples, authored *before* labeling begins, (b) a second annotator on a 15% subsample, and (c) **reported inter-annotator agreement (Cohen's κ)**. Reviewers will ask. Having the number is the difference between a credible paper and a rejected one.

### 5.6 Detectors

A deliberate cost ladder, cheapest first:

**D0 — Conditioned current test (free).** Current excess evaluated against grip state and commanded motion over a smoothed window. Week 1 established that an absolute magnitude threshold on either channel fails outright — peak current during ordinary teleoperation overlaps completely with peak current during collisions — so this is a conditioned test, not a magnitude test. Load is corroboration only: it saturates at ±500 (gripper) and ±1000 (arm) precisely when the fault is most severe. Zero inference overhead.

**D0r — Free-space residual (low).** Subtract a learned free-space current prediction, conditioned on commanded motion, from the measured value. This is the runtime substitute for Week 1's matched-trajectory control, adapted from FACTR 2. Window length is a tuned hyperparameter: industrial anomaly-detection work found performance dominated by it, with 500-sample windows reaching 0.78 AUROC while short windows fell below 0.4.

**D0+ — Telemetry classifier (near-free).** Logistic regression or small MLP over a proprioceptive window: load, current, position error (goal − present), velocity, and first differences.

**D1 — Action-chunk consistency (cheap).** Compare overlapping timesteps between the chunk predicted at *t* and at *t+k*; disagreement indicates the policy is revising its plan. No architecture changes, no extra forward passes.

**D2 — Perturbation disagreement (expensive).** Perturb hidden activations, sample multiple action predictions, use spread as epistemic uncertainty. Costs *N*× inference — measure and report this, since real-time viability is part of the finding.

**D3 — Supervised latent probe (moderate).** Lightweight classifier on SmolVLA's action-expert latents against onset labels. Represents the SAFE-style family.

**D_fusion — D0+ ⊕ best model-internal detector.** Tests H3.

**D_oracle — human labels.** Upper bound for the closed-loop experiment; isolates how much of the recovery gap is detection error versus recovery-policy inadequacy.

> **Scheduling note:** implement D0 and D0+ *early*, during corpus collection rather than after. They are cheap, they test the project's central premise, and having them working by early October is what makes a workshop submission possible.

### 5.7 Recovery policy

**Keep it scripted.** Learning recovery is a separate paper and will consume the entire semester if allowed to.

On detection: halt → retract to a fixed home-adjacent pose → re-open gripper → re-invoke the policy from the current observation. Cap at 2 retries per rollout. Log every trigger with timestamp and detector score.

The scripted design is a strength: it holds recovery constant so differences in downstream success are attributable to *detection quality*, which is the object of study.

---

## 6. Metrics

**Detection quality**
- Frame-level AUROC and average precision against onset labels
- **TPR at fixed per-rollout false-alarm rate** (report at 5% and 10%) — more decision-relevant than AUROC alone
- **Detection lead time:** seconds between detection and labeled onset. A detector that fires after failure is useless regardless of AUROC
- Breakdown by failure stratum (where H2 lives — expect telemetry to dominate on `E2`/`E3` and collapse on `S1`/`S2`)

**Cost**
- Added inference latency per control step (ms)
- Peak memory overhead
- Implementation complexity (qualitative, but state it)

**Closed-loop**
- Success rate: no-recovery baseline vs. each detector vs. oracle
- **Net gain** = successes recovered − successes lost to false-alarm interruptions
- Mean rollout duration (recovery costs time even when it works)

**Reporting standard.** Every success rate gets a binomial confidence interval. At n=40 per condition the 95% CI is roughly ±15 percentage points — wide. State this plainly rather than reporting bare point estimates, and never claim a difference the intervals do not support. Under-powered real-robot evaluation is the most common reason undergraduate robotics papers are rejected.

---

## 7. Timeline

Robot time: ~950 rollouts plus 180 demonstrations. At ~1.5 min each including manual reset that is ~28 hours of pure robot time; apply a 2× overhead factor for setup, re-runs and maintenance and budget **~60 hours**. Critically, this load is *not* evenly distributed — see the per-week breakdown below. Automating episode resets in Week 2 is the highest-leverage engineering investment available.

### 7.1 Experimental phase (14 weeks)

Anchored to a 3 August 2026 start; shift uniformly if you begin later.

| Week | Dates (2026) | Milestone | Robot hrs | Gate |
|---|---|---|---|---|
| 1 | Aug 3–9 | Teleop running; telemetry readable; **signature test** | 3 | **GO / NO-GO** |
| 2 | Aug 10–16 | Reset automation; telemetry logging integrated; schema frozen | 3 | **Schema frozen** |
| 3 | Aug 17–23 | Pilot: demos on T1, fine-tune, tune failure rate into band | 4 | **Difficulty locked** |
| 4–5 | Aug 24–Sep 6 | Full demonstration collection, all three tasks | 9 | |
| 6 | Sep 7–13 | Fine-tune **both** policies (6-dim and 30-dim); verify latency and chunk config | 2 | **Cut to 2 tasks, or drop Arm 3, if behind** |
| 7–8 | Sep 14–27 | Rollout corpus (train + calibration); D0/D0+ implemented | 22 | |
| 9 | Sep 28–Oct 4 | Labeling; inter-annotator subsample and κ | 1 | **Workshop submission** |
| 10 | Oct 5–11 | D1 implemented; held-out eval collection | 9 | |
| 11 | Oct 12–18 | D2, D3, fusion; full detection results. **Arm 3 evaluation rollouts (~120)** | 8 | **Core result in hand** |
| 12–13 | Oct 19–Nov 1 | Closed-loop experiments | 16 | |
| 14 | Nov 2–8 | Analysis, figures, draft, artifact release | 2 | |

**Robot-time estimate.** 1,250 episodes total (1,070 rollouts + 180 demonstrations) at ~1.5 min each including reset, with a 2× overhead factor, gives **~66 hours** — and it is not evenly distributed. Weeks 7–8, 11, and 12–13 demand 8–11 hours each; everything else sits well under 5. Plan coursework and other commitments around those five weeks specifically, and treat reset automation as the thing that makes them survivable.

---

#### Week 1 — Feasibility

**Goal.** Determine whether the project's central premise is true before investing in it.

**Do.** Install LeRobot; get leader-follower teleoperation working end to end. Separately, poll `present_load` and `present_current` off the Feetech bus in a bare loop and confirm the update rate. Then run the signature test: teleoperate into a deliberate collision, and deliberately slip a grasp (smooth object, lift, let it slide). Log and plot load and current against time. Order two spare STS3215 servos the same day.

**Deliverable.** A plot showing load and current traces for a normal grasp, a collision, and a slip.

**Exit criterion.** Collision produces a visible current spike; slip produces a distinguishable gripper-load drop — **visible by eye, without statistics.** If it needs statistics to see, it is too weak to detect online at useful lead times.

**If it fails.** Stop. Pivot to the language-grounding study, which reuses the platform, the teleoperation setup and most of the pipeline. Losing one week is a good outcome compared to losing ten.

---

#### Week 2 — Infrastructure

**Goal.** Build the two things that determine whether Weeks 7–13 are feasible.

**Do.** Extend LeRobot's record loop so `present_load` and `present_current` are logged on shared timestamps with frames and actions — they are not in the default schema. Then automate episode reset: an IK-based return-to-start plus a scripted gripper open covers most of it, and object repositioning may still need a hand. Mount and tape the cameras; photograph the reference views.

**Deliverable.** A frozen data schema document, and a reset routine that runs unattended between episodes.

**Exit criterion.** You can run 10 consecutive episodes with minimal manual intervention, and every episode's telemetry aligns with its frames to within one control step.

**Why this week matters disproportionately.** Every minute saved per reset is multiplied by 1,130. A reset routine that saves 30 seconds per episode saves roughly 9 hours across the project — more than the entire Week 1 through Week 6 robot budget combined.

---

#### Week 3 — Pilot and difficulty calibration

**Goal.** Lock task difficulty into the 40–60% failure band, and validate the whole pipeline end to end on one task.

**Do.** Collect 30 demonstrations on T1 with full position randomization. Fine-tune. Run ~30 evaluation rollouts and measure the failure rate. Adjust using physical difficulty levers — randomization radius first, then object properties — and re-measure. Expect two or three iterations. Write the difficulty spec sheet as you go: exact objects, exact bounds, photographs.

**Deliverable.** Difficulty spec sheet for all three tasks; one fully working end-to-end pipeline run.

**Exit criterion.** T1 failure rate lands in band, and you have working parameter choices for T2 and T3 based on the same reasoning.

**Watch for.** At n=30 the failure-rate estimate carries roughly ±18 points. Do not iterate more than three times chasing precision the sample size cannot deliver — land in band and move on. Also: if the policy is failing by flailing rather than by clean grasp misses, the task is too hard or the fine-tune undertrained. You want *crisp* failures.

---

#### Weeks 4–5 — Demonstration collection

**Goal.** 180 teleoperated episodes, 60 per task.

**Do.** Collect in blocks with breaks; teleoperation quality degrades measurably with fatigue and inconsistent demonstrations poison the fine-tune. Randomize object position every single episode across the full workspace region. Verify camera alignment against the Week 2 reference photographs at the start of every session and log the check.

**Deliverable.** Three demonstration datasets, versioned and backed up off-machine.

**Exit criterion.** 60 clean episodes per task, with position coverage visibly uniform when plotted.

**Watch for.** Demonstrator drift — your teleoperation style at episode 180 will differ from episode 1. Interleave tasks rather than doing all 60 of T1 then all 60 of T2, so drift distributes evenly rather than correlating with task.

---

#### Week 6 — Fine-tuning and the scope checkpoint

**Goal.** Three trained policies, verified for runtime performance.

**Do.** Fine-tune SmolVLA per task, 20k steps. Verify the action-chunk configuration for inference — single-step prediction will cripple runtime performance. Measure end-to-end inference latency per control step and record it; this becomes the denominator for every cost claim in Section 6. Re-measure failure rate on ~20 rollouts per task and confirm all three are still in band.

**Deliverable.** Checkpoints for both the 6-dim and 30-dim policies, a latency baseline, and confirmed failure rates.

**Both policies are trained here**, from the same demonstrations with identical hyperparameters and seeds where possible. Only the 6-dim policy's failure rate is tuned into band; the 30-dim policy's rate is recorded as a result.

**Exit criterion.** All three tasks in band, inference running at usable control rate.

**This is the scope checkpoint.** If you are behind schedule, cut T1 — it is the sanity task and contributes least. Keep T2 (telemetry evaluation) and T3 (semantic failures); they carry H2. Cutting one task saves roughly 7 hours of robot time downstream and costs you one row in a results table.

---

#### Weeks 7–8 — Corpus collection (crunch)

**Goal.** 450 labeled-ready rollouts: 300 for detector training, 150 for calibration.

**Do.** Run rollouts in long automated blocks, recording video, telemetry, and policy internals. In parallel — during the runs, not after — implement D0 and D0+, and write the labeling guide with worked examples.

**Deliverable.** Corpus (train + calibration splits); working D0/D0+; labeling guide finalized before any labeling begins.

**Exit criterion.** 450 rollouts recorded with complete synchronized data; D0+ producing scores on logged rollouts.

**Watch for.** This is the highest-risk stretch. Two failure modes: hardware wear shifting your failure rate out of band mid-corpus (re-measure at the midpoint and log it — if it has drifted, you have two incomparable halves and need to know), and data loss (back up nightly, off-machine, no exceptions). Also resist the temptation to start labeling before the guide is written; a taxonomy fitted to data you have already seen is not a taxonomy.

---

#### Week 9 — Labeling

**Goal.** Frame-level onset labels on the training and calibration splits.

**Do.** Pass 1 labels rollout outcome and failure class. Pass 2 labels the failure onset frame — first frame at which failure became inevitable. A second annotator independently labels a 15% subsample. Compute Cohen's κ.

**Deliverable.** Labeled splits, κ figure, and a short note on where annotators disagreed.

**Exit criterion.** κ above roughly 0.7 on class labels. If onset-frame agreement is poor, the guide needs sharpening and a re-label of the subsample — do this now, not after building detectors on noisy targets.

**Also this week.** Workshop submission, if targeting CoRL 2026: taxonomy, telemetry signatures, and D0/D0+ preliminary results are a legitimate work-in-progress paper.

---

#### Week 10 — First detectors and held-out data

**Goal.** D1 working; held-out evaluation set collected.

**Do.** Implement action-chunk consistency (D1) — compare overlapping timesteps between the chunk predicted at *t* and at *t+k*. Collect the 180-rollout held-out set. Label it as it comes in rather than batching, so Week 11 is not blocked.

**Deliverable.** D1 scores on the training split; held-out set collected and labeled.

**Exit criterion.** D0, D0+ and D1 all produce per-frame scores on a common interface, so Week 11 is purely additive.

---

#### Week 11 — Full detector suite

**Goal.** All detectors evaluated; the core result exists.

**Do.** Implement D2 (perturbation disagreement) and D3 (supervised latent probe). Build D_fusion. Evaluate everything on held-out data: AUROC, TPR at fixed false-alarm rate, detection lead time, per-stratum breakdown, and measured latency cost for each.

**Deliverable.** The main results table. RQ1, RQ2 and RQ3 answered, plus Arm 3's end-to-end success rate.

**Arm 3's ~120 evaluation rollouts run this week**, since the detector work is compute-bound and leaves robot time free. Annotate them with the same fault taxonomy: the failure-composition comparison against Arm 1 is a result in its own right, independent of any success-rate difference.

**Exit criterion.** You can state, with confidence intervals, whether telemetry is competitive with model-internal detection.

**Note.** If D2 is too slow to run online, that is a finding — report the measured cost and evaluate it offline on logged rollouts. Do not let it consume the week.

---

#### Weeks 12–13 — Closed-loop experiments (crunch)

**Goal.** 320 rollouts across four conditions on two tasks.

**Do.** Conditions: no-recovery baseline, D0+ triggered recovery, best model-internal detector triggered recovery, and oracle (human labels). Recovery stays scripted throughout — halt, retract, re-open, re-invoke, capped at 2 retries. Log every trigger with timestamp and score.

**Deliverable.** Success rates per condition with binomial confidence intervals; net gain accounting for false-alarm cost; mean rollout duration.

**Exit criterion.** RQ4 answered, honestly, including the case where net gain is negative.

**Scheduling note.** This is why the closed-loop phase now spans two weeks rather than one. At 16 hours of robot time it does not fit a single week at any sustainable pace, and it is the most novel component of the paper. If you must compress, drop the third condition (best model-internal detector) rather than the oracle — the oracle is what separates detection error from recovery-policy inadequacy, and without it the negative result is uninterpretable.

---

#### Week 14 — Analysis and release

**Goal.** A complete draft and a released artifact.

**Do.** Figures, ablations, limitations section. Package the dataset with the labeling guide and code. Write the draft.

**Deliverable.** Full draft; dataset ready for Hugging Face; arXiv preprint prepared.

**Exit criterion.** Someone else could reproduce your headline number from the released artifact.

**Realistically,** the draft finishes in the December–February writing tail rather than this week. Week 14's job is to make sure nothing requiring the robot is left outstanding, because your access to it may not survive the semester boundary.

---

**Protect the closed-loop experiment.** It is the most novel component and it is scheduled last, which makes it the most vulnerable. If Week 6 slips, cut a task rather than cutting Weeks 12–13.

### 7.2 Publication timeline

| Date | Action |
|---|---|
| ~Late Sep / early Oct 2026 | **CoRL 2026 workshop submission** (preliminary: taxonomy, telemetry signatures, D0/D0+ results). Individual workshop deadlines are set by organizers, typically 4–6 weeks before the event — watch for them |
| 9 Nov 2026 | CoRL 2026 workshop day, Austin TX |
| ~Mid Nov 2026 | **arXiv preprint** + **Hugging Face dataset release** |
| Dec 2026 | NeurIPS 2026 workshops (alternative or additional venue) |
| Dec 2026 – Feb 2027 | Full write-up, additional ablations, reviewer-anticipating experiments |
| **1 Mar 2027** | **IROS 2027 submission — primary target** |
| Rolling | **RA-L** as alternative or fallback (no deadline) |
| 26 Sep – 1 Oct 2027 | IROS 2027, Florence |

---

## 8. Publication strategy

**Skip ICRA 2027.** Submissions are due 15 September 2026 — you would be submitting a pilot study.

**Workshops first (CoRL 2026 / NeurIPS 2026).** CoRL 2026 workshops are 9 November in Austin, the day before the main conference, and this year the organizers are explicitly steering workshops back toward interactive, problem-oriented formats for new, preliminary and in-progress work. That is exactly what you will have. Workshop papers are usually non-archival — but **verify this for the specific workshop**, since it determines whether the main-venue submission is still available to you.

**RA-L is arguably the best fit.** IEEE Robotics and Automation Letters uses rolling submission with no deadline, which removes the timeline gamble entirely: if Week 9 goes badly you submit in February rather than missing a cycle by a week. It is ~8 pages, respects careful empirical work, and does not demand a novel method. A rigorous comparative study with a released dataset is RA-L-shaped.

**IROS 2027 is the main-conference target.** Draft deadline 1 March 2027, conference 26 September – 1 October 2027 in Florence. Finish experiments in November, write properly through January and February, submit in March. If you want one goal on the wall, make it this.

**RSS 2027 and CoRL 2027** are stretch targets — both excellent, both genuinely hard. Treat them as upside if the results are strong.

**Release the dataset on Hugging Face.** The LeRobot community is where your users are, and a labeled real-robot failure corpus will be picked up there faster than through any conference. This may prove the longest-lived contribution: labeled real-robot failure corpora are scarce, and releasing one gives the paper a reason to be cited regardless of whether the headline result holds.

*Verify every date above against the official sites before planning around it. Deadlines shift, and workshop deadlines are not posted yet.*

---

## 9. Risks and mitigations

| Risk | Severity | Mitigation |
|---|---|---|
| No usable telemetry signature | **Critical** | Week 1 signature test exists precisely to find this out on day 3, not week 10. Fallback: pivot to the language-grounding study, which reuses the same platform and pipeline |
| Failure rate outside 40–60% | High | Week 3 pilot; difficulty levers pre-identified per task; spec sheet and periodic re-measurement |
| Load/current not in LeRobot schema | High | Extend record loop in Week 2, before any corpus collection |
| Too few per-class instances | High | Collapse to semantic/execution binary; deliberately over-sample target modes as a separate stratum |
| Onset labels unreliable | Medium | Written guide, second annotator, report κ |
| Under-powered results | Medium | Pre-register n per condition; report CIs; prefer fewer conditions with more rollouts |
| Servo burnout / hardware failure | Medium | Order two spare STS3215 units in Week 1 — cheap, and lead times are not short |
| D2 too slow for real-time | Low | Itself a reportable finding; run offline on logged rollouts if needed |
| All detectors perform equivalently | Low | Still publishable: "expensive introspection does not beat free telemetry on cheap hardware" is a clean result |
| Scope growth across three arms | **High** | Arm 3 is evaluation-only and the first thing cut; Arm 4 optional; Arm 2 closed-loop protected |
| Arm 3 lands far outside the difficulty band | Medium | Do not re-tune; report the rate and interpret Δ success against Arm 1 with explicit reference to differing base rates |
| Episode budget confound between arms | Medium | Hold policy execution time constant, not wall-clock time |
| Contribution judged incremental vs. industrial AD | **High** | Position explicitly as a cross-literature benchmark; report trivial baseline and cost-normalized results, which neither literature provides |
| Week 1 separation does not survive causal evaluation | **High** | Re-run gate_analysis in trailing-window mode before the corpus phase; Week 1 figures are whole-file and are an upper bound |
| No matched control exists at runtime | **High** | Learn a free-space current model and use the residual, following FACTR 2; validate on held-out trajectories |

**On the last row:** decide *now*, before seeing data, that a null result is acceptable and will be written up honestly. This is the pre-commitment that protects against adding conditions until something looks significant.

---

## 10. Expected contributions

1. First evaluation of VLA failure detectors under low-cost-hardware noise conditions.
2. A free proprioceptive baseline the literature lacks, with per-stratum characterization of where it succeeds and fails.
3. Closed-loop evidence on real hardware, with false-alarm cost accounted for.
4. An open corpus of ~950 labeled SO-101 rollouts with synchronized telemetry, labeling guide, and code.

---

## 11. Paper structure

Eight pages plus appendix.

1. Introduction — reliability gap, hardware gap, unused signal
2. Related work — the four detector families, SO-101 benchmarks
3. Platform and tasks
4. Failure taxonomy and labeling protocol
5. Detectors: the cost ladder
6. Detection results — RQ1, RQ2, RQ3
7. Closed-loop results — RQ4
8. Limitations — single policy family, single arm, three tasks, modest n
9. Conclusion

Write Section 4 and the labeling guide *before* collecting the corpus. A taxonomy invented after seeing the data is a taxonomy fitted to the data.

---

## 12. Week 1, concretely

**Days 1–2.** LeRobot installed, teleoperation running end to end. Separately, confirm you can poll `present_load` and `present_current` off the Feetech bus at a useful rate — in a bare loop, printing. Do not assume.

**Days 2–3.** Extend the record loop so telemetry lands in your logs on shared timestamps with frames and actions.

**Day 3 — the test that decides the project.** Teleoperate into a deliberate collision. Deliberately slip a grasp: grab something smooth, lift, let it slide. Log, then plot load and current against time. You are looking for a visible current spike on collision and a distinguishable gripper-load drop on slip. **Eyeball it first** — if it takes statistics to see, it is probably too weak to detect online at useful lead times.

**Then decide.** Clear signatures → proceed to camera mounting and the Week 3 pilot. Nothing visible → stop and rethink before five weeks are sunk. The language-grounding study is the natural fallback and reuses everything built so far.

**Today, in parallel:** order two spare STS3215 servos. A dead gripper servo in Week 9 with a two-week lead time costs you the closed-loop experiment.

**Ongoing from day one:** keep a dated lab log — setup photographs, configuration changes, anything you tweak. Six weeks in, when the failure rate has drifted, the log is what tells you why.

---

## References

- Liu et al. *LIBERO: Benchmarking Knowledge Transfer for Lifelong Robot Learning.* NeurIPS 2023.
- *LIBERO-plus / In-depth Robustness Analysis for Vision-Language-Action Models.* arXiv:2510.13626.
- Shukor et al. *SmolVLA: A Vision-Language-Action Model for Affordable and Efficient Robotics.* arXiv:2506.01844.
- Yu & Qiu. *Benchmarking Vision-Language-Action Models on SO-101: Failure and Recovery Analysis.* arXiv:2606.08881.
- Huang et al. *VLA-REPLICA: A Low-Cost, Reproducible Benchmark for Real-World Evaluation of Vision-Language-Action Models.* arXiv:2605.20774.
- Gu et al. *SAFE: Multitask Failure Detection for Vision-Language-Action Models.* NeurIPS 2026.
- *ReconVLA: An Uncertainty-Guided and Failure-Aware Vision-Language-Action Framework for Robotic Control.* arXiv:2604.16677.
- *Perturbation-Based Uncertainty for Failure Detection in Vision-Language-Action Models.* arXiv:2606.20754.
- *Anomaly detection for generic failure monitoring in robotic assembly, screwing and manipulation.* arXiv:2509.26308 — nearest neighbour to the proprioceptive arm of this study.
- *FACTR 2: Learning External Force Sensing for Commodity Robot Arms Improves Policy Learning.* arXiv:2606.12406.
- *Can We Detect Failures Without Failure Data?* (FAIL-Detect). arXiv:2503.08558.
- *Rewind-IL: Online Failure Detection and State Respawning for Imitation Learning.* arXiv:2604.16683.
- TA-VLA — torque-aware VLA using low-cost-arm joint current as policy input.
- Cadene et al. *LeRobot: An Open-Source Library for End-to-End Robot Learning.* ICLR 2026.

*Verify all arXiv identifiers and author lists against the live listings before submission.*
