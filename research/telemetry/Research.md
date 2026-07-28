# Where Should Free Proprioception Go? Monitoring versus Conditioning on Low-Cost Manipulation Hardware

**Empirical study for discussion with robotics researchers. Week 1 hardware results are reported; the remainder is proposed.**

Code, data, and analysis: `github.com/EAOZONE/lerobot`, under `research/telemetry/`.

---

## Research question

Low-cost serial-bus servos report drive current at control-loop rates. It is free, uncalibrated, and already present on every arm of this class. Given that channel, where should the signal go?

There are three answers, and they belong to three different literatures:

1. **Nowhere.** The standard configuration: a vision-language-action (VLA) policy conditioned on joint positions, with the telemetry discarded.
2. **Into a runtime monitor.** The signal feeds a fault detector running alongside the policy, which triggers recovery. This is the industrial anomaly-detection approach, transplanted onto hobby-grade actuation.
3. **Into the policy.** The signal is added to the observation vector and the policy learns to use it directly. This is the force-conditioned imitation-learning approach.

Each option has been studied against option 1. Force-conditioned policies beat position-only policies; runtime monitors beat unmonitored execution. **Options 2 and 3 have never been compared against each other**, because the papers that establish them sit in separate literatures with separate baselines, separate metrics, and separate hardware.

For a practitioner with a $150 arm and a free current channel, that is the question that actually matters: given one signal and a limited budget, is it better spent teaching the policy or watching it? This study answers it under identical data, identical tasks, and a common success metric.

---

## Study design

The design is a 2×2 over where the telemetry is used:

| | No detection | With detection |
|---|---|---|
| **Policy sees 6 dims (positions)** | **Arm 1** — baseline | **Arm 2** — monitoring |
| **Policy sees 30 dims (+ load, current, velocity, voltage)** | **Arm 3** — conditioning | **Arm 4** — both |

Three arms are core; the fourth is upside.

**Arm 1 — Baseline.** SmolVLA fine-tuned on joint positions alone, executed without intervention. Provides the reference success rate and the failure corpus that every other arm is measured against.

**Arm 2 — Monitoring.** The same policy, with detectors reading all thirty telemetry dimensions and triggering a scripted recovery. This arm carries the detector comparison: proprioceptive detectors against VLA-internal detectors, under identical data and operating points.

**Arm 3 — Conditioning.** SmolVLA fine-tuned on the full thirty-dimensional observation, executed without intervention. Evaluated end-to-end only; no detector suite.

**Arm 4 — Both.** Conditioned policy plus monitor. The interaction cell, and the most interesting one: if the policy already uses current to avoid slipping, does the monitor still have anything left to catch? Run if schedule permits.

**The design costs less than it appears**, because the recording schema already logs thirty dimensions and truncates to six at training time. One demonstration corpus trains both policies. No additional teleoperation is required — the expensive component is untouched. The marginal cost of Arm 3 is one fine-tune plus roughly 120 evaluation rollouts.

**Deliberate asymmetry.** Detector benchmarking happens only on Arm 2. Arm 3 needs a success rate, not a detector suite. This keeps the study bounded and makes Arm 3 cuttable without damaging the core result.

---

## Why this setting is different

Classical model-based fault detection relies on a reference: a dynamics model, planned trajectory, or commanded setpoint against which observed behavior can be compared. An end-to-end VLA policy offers no comparably interpretable description of its intent. Its objective is encoded implicitly in network activations, and its action sequence may be produced without a planner or reference trajectory that can be checked independently.

VLA-facing work therefore constructs residuals from the policy's internal consistency: probing latent representations, measuring disagreement among action predictions after perturbing hidden activations, or testing whether successive action chunks agree where they overlap.

These methods are technically compelling, but their hardware assumptions may matter. Research-grade arms typically provide accurate state estimation, high repeatability, torque control, and sometimes dedicated force/torque sensing. The SO-101 does not.

| Property | Research-grade arm | SO-101 |
|---|---|---|
| Actuation | Torque control or high-performance position control | Position control over a serial bus |
| Force sensing | Often dedicated force/torque sensing | No dedicated sensor; drive load and current telemetry only |
| Repeatability | Typically submillimeter | Millimeter-scale, with backlash |
| State estimate | High fidelity | Noisy and gear-lash-dependent |

A detector that treats unusual robot states as evidence of impending failure faces a difficult test here: nominal operation already contains substantial unmodeled deviation.

The industrial anomaly-detection literature faces the opposite situation. Its methods are built for exactly this kind of signal but assume a calibrated wrench measurement. On the SO-101 the available channel is servo drive current: uncalibrated, noisy, and confounded by gravity, friction, temperature, and configuration. Both load and current are *drive* measurements rather than force measurements — with torque disabled they read approximately zero regardless of how hard the arm is pushed by hand.

---

## Relationship to adjacent literature

An earlier version of this document claimed nobody had compared learned VLA monitors against a proprioceptive baseline. That was too strong, and the correction determines what this study can assert.

**Proprioceptive failure monitoring of learned policies exists.** Work on generic failure monitoring in robotic assembly applies autoencoder-based anomaly detection to force/torque time series with operator-provided temporal onset labels, and evaluates transfer across tasks and across control strategies including diffusion policy. It differs in platform (industrial arm with wrench sensing), task domain, and policy family — but the approach is prior art.

**Failure detection for imitation-learned policies is likewise established.** FAIL-Detect frames runtime detection as sequential out-of-distribution detection under conformal prediction; Rewind-IL pairs a self-consistency signal with an actual recovery mechanism, overlapping the closed-loop component here.

**Force-conditioned policies are established, and Arm 3 is a replication.** TA-VLA already uses joint current from low-cost arms as a policy input to improve imitation learning, and FACTR 2 critiques and extends it. Arm 3 alone contributes little that is new. Its role in this study is not to establish that conditioning helps — that is known — but to provide a like-for-like comparison against monitoring on the same tasks, the same corpus, and the same metric.

**What is open.** The three literatures have not been evaluated against one another. Neither monitoring nor conditioning has been tested where the proprioceptive channel is uncalibrated servo current on a $150 arm. And the interaction — whether monitoring retains value once the policy is already conditioned on the same signal — appears entirely unexamined.

**A useful inherited result.** FACTR 2 observes that joint current conflates the actuator effort required for free-space motion with that produced by external contact, so separating them reliably requires temporal context. This is precisely the obstacle encountered in Week 1, and their approach — subtracting a learned free-space prediction from the measured value — is a validated path through it.

---

## Preliminary results

The premise required testing before the rest was worth attempting, so Week 1 established whether the signal exists. It does, for both target failure modes, but not as the original design assumed. The findings rest on nine isolated single-event runs and three matched replay pairs. This is a signature test, not an accuracy estimate: no detection rates, ROC characteristics, or lead-time distributions should be inferred from it.

### Slipped grasps

The signature is a sustained gripper current — roughly 200 to 390 in raw units — collapsing below 20 within two samples, while the *commanded* gripper position does not move. Across three slip runs and three clean controls the separation was complete, with roughly an order of magnitude between groups on the onset-drop feature.

The command condition is essential rather than incidental. Evaluated without it, a deliberate release scores higher than any slip and the classes become indistinguishable. The discriminating feature is not the current drop; it is the current drop unaccompanied by a corresponding command.

Lead time from current collapse to the operator's keypress was 0.07–0.80 s. Allowing for reaction lag, true lead on the physical event is plausibly 0.3–1.1 s, or 8–33 samples at 30 Hz. Sufficient to halt the arm; probably insufficient for elaborate recovery.

One run contained two slips. The operator marked only the second; the analysis recovered the first unprompted, where gripper current fell from 337 to 46 with the jaws stationary and no commanded change.

### Collisions

Peak magnitude alone does not work — the most useful negative result of the week. Whole-file peak current during ordinary teleoperation (166–195) overlaps completely with peak current during collisions (172–217); peak load behaves the same way (456–528 against 450–565). Normal motion loads the joints as hard as a collision does. What separates the classes is temporal shape: sustained excess against a matched control.

Establishing this required replaying one recorded trajectory twice — once with an obstacle in the path, once clear — so both executions issue identical commands and align frame for frame. Smoothed peak current excess against the matched baseline was 25×, 64×, and 244× for light, moderate, and hard contact. In the hardest case a joint's current ramped from 281 to 1844 over 0.3 s, after which its position froze for 2.3 s while the commanded goal swept 36° away and velocity held at zero.

A related trap: an obstacle that slides or tips produces no collision at all. The first attempt showed only 5° of positional divergence from the clear run, because nothing had stopped the arm.

### Current is the usable channel; load is not

This was not the working assumption, and it changes the detector design.

Load saturates precisely when the fault is most severe. The gripper clips at ±500 and the arm joints at ±1000; during the hard stall the relevant joint sat pinned at its limit for the entire event, carrying no severity information, while current continued to discriminate up to 1844. The gripper's ±500 ceiling is a software default written by the control stack's configuration routine rather than a hardware limit — invisible unless specifically sought, and not to be raised, since it guards a servo that has already alarmed once.

Load is also noisier across replays. On the light-contact pair, smoothed load excess peaked at 5.3× baseline across three separate episodes, its maximum 2.7 s from the actual contact. Smoothed current peaked at 25× in a single episode, exactly on the contact.

Current is therefore the primary channel; load is corroboration. The conclusion holds for both failure modes from independent evidence.

### Two further constraints

**Smoothing is not optional, and the right window is probably much longer than the one used so far.** Contact is sustained over tenths of a second while replay variance is single-frame noise; unsmoothed, load excess scored 11.6× on a matched pair containing no contact at all. A five-frame (~0.17 s) mean sufficed for visual inspection but is likely insufficient for a learned detector: the industrial anomaly-detection work found performance dominated by window length, with 500-sample windows reaching 0.78 AUROC while short windows fell below 0.4. Window length should be a tuned hyperparameter, and 0.78 AUROC on an industrial platform is a reasonable reference point.

**Bus voltage is specific but insensitive.** Supply sag correlated at +0.99 with current during the hard stall, falling 0.6 V. Light and moderate contacts produced no distinguishable sag, their largest excursions landing 3.5 s and 7.2 s from contact, and clean runs swing 0.3–0.4 V unprompted. Voltage is recorded as a stall-severity indicator, never a trigger.

### Protocol defects worth reporting

Three failed attempts preceded the result, each a protocol defect rather than a hardware limitation.

Current initially read flat zero because torque was disabled — the state inherited between sessions, since the follower disables torque on disconnect. An initial batch used typed event markers landing 3–4 s after the event, a window wider than the events themselves. An initial set of slip runs never established a grasp: gripper current sat at 2–3 throughout, capturing the aftermath rather than the onset. The last was corrected by a live grip-confirmation readout requiring the operator to verify a real grasp before initiating the slip.

### Instrumentation now in place

The recorded observation vector has been widened from 6 to 30 dimensions — position, load, current, velocity, and voltage per joint — and the schema frozen. The record loop was measured at 29.95 Hz with two cameras attached, so no reduction in frame rate or camera count is required. Servo status and temperature registers are read but excluded from the schema; temperature is logged per session, sufficient for tracking the drift that will matter once task difficulty is calibrated.

This schema is what makes the three-arm design affordable: the same recordings serve both the 6-dimensional and 30-dimensional policies, with truncation applied at training time.

### Hardware fragility

Two servos latched overload alarms during the week, one after a joint held approximately 12 A for 2.3 s during the hard-stall run. A latched Feetech servo rejects all subsequent writes until power-cycled, and the SDK does not name most alarm bits, so the failure surfaces as a write error with an empty explanation. Corpus collisions will be held at light and moderate intensity.

---

## Method

### Platform and policies

An SO-101 leader-follower pair for teleoperation, with wrist-mounted and overhead RGB cameras. Both policies are SmolVLA, a 450-million-parameter VLA chosen because it fine-tunes on a single consumer GPU and executes at practical control rates on desktop hardware. The two policies differ only in observation dimensionality (6 versus 30) and are trained on the same demonstrations with identical hyperparameters and seeds where possible.

### Tasks

Three tabletop tasks:

1. a calibration task for validating the data and evaluation pipeline;
2. insertion of a cylindrical object into a cup, intended to produce execution failures — slips, collisions, jams;
3. retrieval of a target from visually similar distractors, intended to produce semantic failures in which the policy confidently manipulates the wrong object.

Each is tuned so the **6-dimensional** policy fails in roughly 40–60% of autonomous trials. This is an experimental design target: enough failures to estimate detector performance, enough successes to characterize false alarms.

**Difficulty is calibrated once, on Arm 1, and then frozen.** The conditioned policy will likely land at a different failure rate. It must not be re-tuned back into band — doing so would destroy comparability across arms. Its rate is a result, not a parameter.

A single multi-task policy is preferred over three single-task policies, since the learned detectors are policy-specific and splitting the corpus across three networks would leave too few failure instances to fit a latent probe. The Week 3 pilot trains both ways on the calibration task and compares failure rate and failure character before committing.

### Data collection and annotation

Approximately 180 teleoperated demonstrations for fine-tuning, then autonomous rollouts recorded with synchronized wrist and overhead video, the 30-dimensional telemetry vector, commanded actions, and the network outputs and internal representations required by the learned detectors.

The annotation protocol, written before collection, distinguishes at least: execution faults (collision, slip, jam, failure to contact); semantic faults (wrong object); recoverable deviations; and the point at which failure becomes unrecoverable under the chosen recovery policy. A second annotator labels a stratified subset; agreement is reported for both fault class and onset time rather than treating labels as unambiguous ground truth.

### Detectors (Arm 2)

A cost ladder spanning both literatures:

| Detector | Signal | Origin | Runtime cost |
|---|---|---|---|
| Duration-only trivial baseline | None | — | None |
| Conditioned current test (grip state, commanded motion, smoothed window) | Proprioception | This work | Negligible |
| Autoencoder or classifier over a telemetry window | Proprioception | Industrial AD | Very low |
| Free-space-subtracted current residual | Proprioception | Adapted from FACTR 2 | Low |
| Action-chunk self-consistency | Policy output | VLA detection | Low |
| Supervised probe on policy latents | Policy internals | VLA detection | Moderate |
| Perturbation-induced prediction spread | Policy internals | VLA detection | Multiple forward passes |
| Fusion of best proprioceptive and best learned | Both | — | Method-dependent |
| Human annotation | Video and telemetry | — | Offline reference |

The trivial baseline is included deliberately: if a latent probe cannot outperform a detector fitted only to elapsed time, that is worth knowing and reviewers will ask.

The conditioned current test is narrower than originally specified — Week 1 showed an absolute threshold on either channel fails outright, so it is a conditioned test, not a magnitude test.

All trained detectors use the same train/validation/test splits, defined at a level that prevents near-duplicate trajectories from leaking across them. Threshold and model selection occur without access to held-out test conditions. All detectors are evaluated **causally**: at time *t*, using only data from the interval up to *t*. The Week 1 analysis is offline and whole-file, so its separation figures are an upper bound on causal performance.

Arm 1's policy input is truncated to six joint positions so that it cannot consume the signal being evaluated as an independent detector. In Arm 3 that truncation is deliberately removed — which is precisely what the arm exists to test.

### Metrics

**Common currency across arms:** change in end-to-end task success relative to Arm 1, with binomial confidence intervals.

**Within Arm 2**, detector quality is reported as precision-recall curves and AUPRC; false alarms per rollout; recall at fixed false-alarm budgets; detection latency relative to both annotated onset and the unrecoverable boundary; the fraction of failures detected with time to execute recovery; and compute, memory, and control-loop overhead. The primary metric is operational: proportion of failures detected early enough to permit the fixed recovery routine, at a specified false-alarm budget.

Week 1's lead-time measurement bears on this. If slip detection affords 0.3–1.1 s, the recovery routine's own execution time is a substantial fraction of the budget, so recovery duration is measured and reported rather than assumed constant.

**Across arms**, cost is reported on a common basis: added inference latency, added training cost, added implementation complexity. Arm 3's cost is paid at training time and is zero at runtime; Arm 2's is paid at runtime. That asymmetry is part of the finding.

**Episode budget must be held constant in policy execution time, not wall-clock time.** Recovery consumes several seconds, so a fixed wall-clock budget would give Arm 2 less policy time than Arm 1 and make a null result uninterpretable.

### Recovery (Arms 2 and 4)

Identical across all detection conditions: halt by writing present position as goal — not by ceasing to send actions, since a position-controlled servo continues driving toward its last goal, the mechanism behind the 12 A stall observed in Week 1, and not by disabling torque, which drops the arm under gravity. Then flush the queued action chunk, retract by replaying recent commanded positions in reverse, return to the episode start pose, reopen the gripper, and reinvoke the policy. At most two retries.

Returning to the start pose is deliberate: the policy was trained on episodes beginning near home, so resuming mid-trajectory would measure out-of-distribution behavior rather than recovery.

Reported separately: failures converted to successes; successful trials disrupted by false alarms; unsuccessful or unsafe recoveries; added execution time. Given the measured lead time, the realistic mechanism is a fast automatic re-attempt rather than fault prevention, and this is stated plainly since "detection enables recovery" implies prevention to most readers.

### Disturbance stratum

A separate, reported-apart stratum introduces external disturbances during reaching — a light push on the links, or a software-injected transient offset on commanded positions.

Its purpose is **hard negatives**. The main corpus contrasts eventful with uneventful rollouts, which makes the false-alarm budget easy to satisfy. A disturbance the policy absorbs and recovers from looks like a fault in telemetry and is not one; that is the case separating a good detector from a jumpy one. Disturbances that do induce failure carry an externally-timed onset, removing label ambiguity for that stratum.

Two requirements. The hand must stay outside both camera frames, or the latent probes may detect a hand rather than a fault — a confound that would inflate their apparent performance for spurious reasons. And since a hand push is not reproducible, the recorded disturbance magnitude is the measured peak positional deviation from commanded, with results stratified by measured rather than intended magnitude.

The software-injected variant is perfectly repeatable and produces following error without external contact force. The contrast is informative: a current-based detector should catch the physical push and may miss the software one entirely.

---

## Hypotheses

**H1: Proprioceptive detection is competitive for mechanically expressed execution faults.** Week 1 offers preliminary support, but under conditions that do not hold in the main study: n=3 per class, teleoperation rather than autonomous control, offline analysis, and a matched trajectory available for collisions.

**H2: Policy-internal detection is more informative for semantic faults.** A smooth grasp of the wrong object is mechanically normal.

**H3: Fusion beats either detector family alone**, if and only if they contribute complementary information after controlling for capacity and supervision.

**H4: Some learned detectors degrade on low-cost hardware**, confusing nominal backlash and tracking error with failure.

**H5: Conditioning and monitoring produce comparable success gains, at different costs.** Arm 3 pays at training time and nothing at runtime; Arm 2 pays per control step. If the gains are similar, the cost asymmetry is the practical result.

**H6: Conditioning changes failure composition, not merely failure rate.** If current-awareness suppresses slips, Arm 3's residual failures shift toward semantic modes. This is testable directly from the annotated corpus and would be reportable independently of any success-rate difference.

**H7 (Arm 4): Monitoring yields diminishing returns on a conditioned policy.** If the policy already exploits the current signal, the monitor has less left to catch. A negative interaction would be the most novel finding available here.

These are deliberately falsifiable. Useful negative results include: telemetry adds little after conditioning on kinematics; learned monitors do not transfer without platform-specific retraining; or conditioning and monitoring are simply redundant.

---

## Intended contribution

The project introduces no new detection architecture and claims no overlooked signal. Both monitoring and conditioning are established. The contribution is a controlled comparison between them, on hardware neither was developed for.

1. **The first like-for-like comparison of monitoring against conditioning** for a single free proprioceptive channel, under identical data, tasks, and success metric.
2. **The first direct comparison of proprioceptive anomaly detection against VLA-internal detectors**, under identical splits and operating points.
3. **Evaluation on low-cost hardware** where the proprioceptive channel is uncalibrated servo current rather than a wrench measurement.
4. **Failure-composition analysis**, testing whether conditioning changes which failures remain rather than how many.
5. **Closed-loop evidence** judged by effect on task success, net of false-alarm cost, with cost reported on a common basis across arms.
6. **A public corpus** of labeled real-robot failures with synchronized video, commands, policy signals, and servo telemetry, including a disturbance stratum.

Two recent papers benchmark VLA policies on the SO-101; where possible their tasks and fault categories are adopted so results extend rather than duplicate that work.

---

## Risks

| Risk | Status | Consequence | Mitigation |
|---|---|---|---|
| Scope growth across three arms | **New, High** | Nothing finishes | Arm 3 is evaluation-only, ~120 rollouts, and is the first thing cut. Arm 4 is optional. Closed-loop on Arm 2 is protected |
| Contribution judged incremental | Open | Reviewers find the comparison unsurprising | Position as a cross-literature benchmark; report trivial baseline, cost normalization, and the interaction cell, which no literature provides |
| Arm 3 falls far outside the difficulty band | **New, Medium** | Arms not comparable | Do not re-tune. Report the rate as a result; interpret Δ success against Arm 1 with explicit reference to differing base rates |
| Week 1 separation does not survive causal evaluation | **Open** | Headline signature figures unreproducible online | Re-run the Week 1 analysis in trailing-window mode before the corpus phase |
| No matched control exists at runtime | **Open** | Collision method does not transfer from Week 1 protocol to deployment | Learn a free-space current model, use the residual, following FACTR 2 |
| Servo telemetry too noisy | Resolved under teleoperation | Baseline carries little signal | Week 1 induced-fault study. Residual risk is transfer to autonomous execution |
| Lead time too short for recovery | Elevated by Week 1 | Closed-loop gain negligible | Measure recovery duration; report detected-in-time fraction; frame recovery as re-attempt, not prevention |
| Hand visible during disturbance runs | **New, Medium** | Latent detectors learn "hand in frame" | Push from outside both camera frames; verify coverage before collection; prefer the software-injected variant |
| Failure onset ambiguous | Open | Lead-time labels unreliable | Observable labeling rules; uncertainty intervals; inter-annotator agreement; disturbance stratum provides externally-timed onsets |
| Too few failures per class | Open | Wide uncertainty | Pilot power analysis; collapse taxonomy to semantic/execution; over-sample target modes as separate strata |
| Split leakage | Open | Results do not reflect generalization | Split by episode; evaluate transfer across a held-out axis |
| Episode budget confound | **New, Medium** | Null closed-loop result uninterpretable | Hold policy execution time constant, not wall-clock time |
| Servo overload alarms | Active | Session loss; attrition | Light and moderate collisions only; per-session diagnostic and temperature log; spare servos |

---

## Questions for discussion

1. **Is the three-arm comparison the right framing?** It reduces to: given one free signal, teach the policy or watch it? Is that a question practitioners want answered, and is it enough for a workshop paper or RA-L?

2. **Is the interaction cell worth protecting?** Arm 4 tests whether monitoring retains value on an already-conditioned policy. It appears unexamined, but it is also the most expensive cell. Would you prioritize it over breadth elsewhere?

3. **Further adjacent work.** Having already missed the industrial anomaly-detection literature by searching with VLA terminology, what other vocabularies should I search — introspective perception, contact-state estimation, execution monitoring, plan-execution verification?

4. **Transfer from matched-pair to online detection.** The collision signature was established by replaying an identical trajectory with and without an obstacle; no such reference exists at runtime. Is a learned free-space current model the right way to close that gap, and does the added machinery undermine the claim that the baseline is cheap?

5. **Comparing arms at different base rates.** If Arm 3's failure rate lands outside the calibrated band, what is the defensible comparison — absolute Δ success, relative risk reduction, or something else?

6. **Experimental unit and power.** Rollout, fault event, or time window? Given correlated frames, what analysis yields defensible confidence intervals?

7. **Operating point.** Given a measured lead time of 0.3–1.1 s for slip, is the recovery budget large enough for "recoverable failures detected at a fixed false-alarm budget" to discriminate between detectors at all?

8. **Scope.** With roughly 66 hours of robot time, which component is cut first if the pilot indicates insufficient coverage?

9. **Telemetry validation.** Week 1 established the signal exists under induced faults in a fixed workspace. What calibration or system-identification experiment — across poses, payloads, temperature — would be needed before interpreting servo current as a general proprioceptive signal rather than a configuration-specific artifact?

10. **A smaller empirical question.** In one slip run, gripper current drifted downward slowly over roughly 1.6 s before release. If that is micro-slip rather than thermal drift, it would extend available lead time considerably. Is that a recognized phenomenon in this class of actuator?

---

## References

**Robustness and benchmarks**

- *In-depth Robustness Analysis for Vision-Language-Action Models.* arXiv:2510.13626.
- Liu et al. *LIBERO: Benchmarking Knowledge Transfer for Lifelong Robot Learning.* NeurIPS 2023.
- Yu & Qiu. *Benchmarking Vision-Language-Action Models on SO-101: Failure and Recovery Analysis.* arXiv:2606.08881.
- Huang et al. *VLA-REPLICA.* arXiv:2605.20774.

**Policy-internal failure detection**

- Gu et al. *SAFE: Multitask Failure Detection for Vision-Language-Action Models.* NeurIPS 2026.
- *ReconVLA.* arXiv:2604.16677.
- *Perturbation-Based Uncertainty for Failure Detection in Vision-Language-Action Models.* arXiv:2606.20754.
- *Can We Detect Failures Without Failure Data?* (FAIL-Detect). arXiv:2503.08558.
- *Rewind-IL: Online Failure Detection and State Respawning for Imitation Learning.* arXiv:2604.16683.

**Proprioceptive monitoring and force-conditioned policies**

- *Anomaly detection for generic failure monitoring in robotic assembly, screwing and manipulation.* arXiv:2509.26308 — nearest neighbour to Arm 2's proprioceptive detectors.
- *FACTR 2: Learning External Force Sensing for Commodity Robot Arms Improves Policy Learning.* arXiv:2606.12406 — critique of raw joint current; free-space torque subtraction.
- TA-VLA — torque-aware VLA using low-cost-arm joint current as policy input; the prior work Arm 3 replicates.

**Platform**

- Shukor et al. *SmolVLA.* arXiv:2506.01844.
- Cadene et al. *LeRobot: An Open-Source Library for End-to-End Robot Learning.* ICLR 2026.

*Bibliographic details and arXiv identifiers should be verified before formal citation. The TA-VLA entry was encountered through citations in other papers rather than read directly.*
