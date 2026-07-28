# Why trajectory-B clear fails D0r

**Diagnosis updated 27 July 2026.** This is development analysis performed after the
pre-registered pair4 hold-out failed. It cannot alter that primary result.

## Short answer

`pair4_clear` fails for two linked reasons:

1. **The A model is extrapolating on trajectory B.** B contains command velocity,
   acceleration, lag, and configuration combinations well outside A's training coverage.
2. **The p99 floor measures within-run frame error, not independent-rollout false-alarm
   behavior.** Rare but ordinary replay variation is larger than the very small distal
   wrist floors, so a few sustained normal transitions trigger the detector.

The result is not caused by CSV replay corruption, mismatched commands, or absence of a
collision signal.

## Evidence 1 — replay is not the cause

The A model was evaluated on both the original `freespace_b_01` teleoperation and its
independent `pair4_clear` replay:

| run | peak × A p99 | joint | time |
|---|---:|---|---:|
| original B teleoperation | 4.59 | `wrist_roll` | 12.01 s |
| B clear replay | 4.32 | `wrist_roll` | 12.01 s |

The false response exists in the original B recording at the same commanded transition.
Replay mechanics cannot be its primary cause. Clear and obstacle goals are also exactly
equal for all motors and all 649 frames.

## Evidence 2 — B command features are outside A coverage

Standardized nearest-neighbor distance was measured in the 68-dimensional commanded
feature space. A frames compared against other A frames have these distances:

| percentile | A→A nearest distance |
|---:|---:|
| p50 | 0.15 |
| p95 | 0.51 |
| p99 | 0.82 |
| max | 1.60 |

B frames measured against A:

| percentile | B→A nearest distance |
|---:|---:|
| p50 | 0.66 |
| p90 | 2.35 |
| p95 | 12.76 |
| p99 | 21.87 |
| max | 28.15 |

At the main 12.01 s wrist-roll false positive, wrist-roll velocity, acceleration, and lag
features are roughly 8–11 A-training standard deviations from their means, with six
features outside A's observed range. Later false windows contain gripper velocity/lag
features 86–112 standard deviations away. Because every joint regressor currently sees
all command features, these gripper extrapolations can drive unrelated wrist-current
predictions downward and enlarge the one-sided residual.

Clipping features to A's observed range does not reliably fix B clear and can distort
predictions further. Coverage must therefore be surfaced as an abstention/diagnostic, not
silently clipped and presented as corrected detection.

## Evidence 3 — independent replay variance exceeds the floor

The original B teleoperation and B clear replay have high current correlation under
identical commands, but rare differences remain:

| joint | current correlation | mean absolute difference | p99 absolute difference |
|---|---:|---:|---:|
| `shoulder_pan` | 0.97 | 3.7 | 23.5 |
| `shoulder_lift` | 0.99 | 6.7 | 55.1 |
| `elbow_flex` | 0.97 | 7.4 | 40.6 |
| `wrist_flex` | 0.97 | 1.6 | 16.5 |
| `wrist_roll` | 0.96 | 1.9 | 23.0 |

A's p99 residual floors are only 14.2 for `wrist_flex` and 9.5 for `wrist_roll`.
Ordinary independent-repeat differences can therefore exceed the operating floor even
though positions remain close. For example, at 12.01 s wrist-roll current is 81 in the
source run and 103 in the clear replay, while measured positions differ by one tick.

This is normal actuator/current variability, not a collision.

### The dominant wrist event is a normal direction-change transient

Frame-level inspection localizes the largest wrist-roll false episode to the fast reversal
around 11.4–12.2 s. The commanded trace accelerates wrist roll in one direction and then
reverses while measured servo velocity reaches roughly ±2,000–2,400 raw units. The final
A+B model predicts a peak current near 121, while normal measured peaks are 176 in the
original B teleoperation and 185 in pair4 clear. Pair4 obstacle reaches 178 in the same
window. The burst is therefore normal servo acceleration/friction behavior present with
or without an obstacle.

The original B, clear replay, and obstacle replay align best at zero frame lag for current,
velocity, and load. Their commanded goals are exact, and wrist positions differ by only
about one tick at the largest current differences. This rules out a shifted CSV replay.
The remaining variation comes from motor-controller dynamics that are not fully determined
by the externally logged goal sequence—velocity profile, friction, and thermal state.
The internal interpolated setpoint was the fourth candidate here and has since been
eliminated; see "Internal setpoint eliminated" below.

The causal ten-frame trailing mean turns a roughly four-frame normal current burst into an
approximately ten-frame threshold episode. Three-frame persistence cannot reject it:
persistence after smoothing is not independent evidence that the underlying disturbance
lasted three frames. Increasing persistence alone is therefore not a principled fix.

The same trajectory transient appears in clear and obstacle. An obstacle-event match near
this interval must be rejected unless obstacle-minus-clear physical divergence independently
supports contact.

### Measured motion confirms transient versus stall, but is not a free D0r feature

At the pair4-clear wrist peak, mean absolute measured wrist velocity over the trailing ten
frames is about 1,740 raw units. At the collision peaks it is about 25 for pair1, zero for
pair2, 10 for pair3, and 160 for pair4; the obstructed joint is stopped or nearly stopped
while drawing much more current. Pair1 clear's smaller wrist alert also occurs near rest,
but its trailing current mean is only 10 versus 220–1,823 for the obstacle peaks.

This confirms that the present examples are separable as normal acceleration transients
versus loaded stalls. It does not justify silently adding measured velocity to D0r:
following error and loss of velocity are themselves contact effects, so conditioning the
free-space predictor on them changes the detector and can introduce target leakage. That
combination belongs as an explicit D0r+stall rule or in D0+, evaluated on the later corpus;
it must not be presented as the commanded-only D0r result.

An offline commanded-only feature ablation added absolute acceleration, squared speed,
square-root acceleration, and jerk terms. It made pair1 clear silent and retained strong
obstacle peaks, but pair4 clear still reached approximately 1.7–2.0× depending on the
variant. The physically sensible nonlinear features may be revisited with more training
runs, but the current two-run post-hoc result is neither sufficient nor a fix.

## Evidence 4 — the validation statistic is mismatched to the reported metric

The original fitter estimates floors from frame residuals held out in contiguous blocks.
With more than one run, those folds still include other frames from the same run in the
training set. That leaks trajectory-specific behavior and underestimates new-run error.

`freespace_model.py` now uses leave-one-whole-run-out validation whenever multiple
no-contact recordings are supplied. A single run retains contiguous blocks only as an
explicit fallback.

Whole-run grouping alone is necessary but insufficient. A p99 **frame** floor permits
roughly 1% of ordinary frames to exceed threshold. At 30 Hz, that is about 18 exceedances
per minute before considering temporal correlation; three-frame persistence does not
guarantee a low per-rollout false-alarm rate because errors around direction changes are
correlated.

The paper's operational constraint is false alarms **per rollout**. The calibration unit
must therefore be the maximum sustained detector score per independent clear rollout,
not a percentile across correlated frames.

## Development-only correction experiment

An A+B model was fit using both no-contact trajectories. It preserved all collision
signals but still scored B clear at 1.8× because its wrist floors remained too optimistic.

Using `pair1_clear` and `pair4_clear` as independent calibration runs, the largest observed
clear residuals required approximate floors of:

| joint | independent-clear maximum floor |
|---|---:|
| `shoulder_pan` | 19.9 |
| `shoulder_lift` | 171.1 |
| `elbow_flex` | 60.6 |
| `wrist_flex` | 24.1 |
| `wrist_roll` | 29.2 |

At those development-only floors, neither clear run crosses, while obstacle peaks remain:

| run | peak × calibrated floor |
|---|---:|
| `pair1_obstacle` | 10.4 |
| `pair2_obstacle` | 12.1 |
| `pair3_obstacle` | 90.0 |
| `pair4_obstacle` | 80.0 |

This demonstrates ample mechanical margin and supports calibration as the immediate
problem. It is not a valid accuracy result: the two clear runs were used to choose the
floors, and two runs cannot estimate a publishable false-alarm budget.

## Internal setpoint eliminated — 28 July 2026

The strongest remaining physical hypothesis was that repeat-dependent current comes from
the servo's internal interpolated setpoint, which the externally logged `goal_pos` trace
cannot observe. Feetech exposes a read-only `Goal_Position_2` at register 71, inside the
existing block read: two extra bytes, no extra bus transaction, no schema change.

Trajectory B was replayed twice with no obstacle and register 71 logged alongside current
in the same transaction (`clear_goal2_a.csv`, `clear_goal2_b.csv`; 649 frames, external
commands byte-identical, timestamps within 5 ms).

**Register 71 reads a constant 0 on every motor in every frame of both runs.**

| joint | goal2 range A/B | frames equal | p99 \|Δgoal2\| | p99 \|Δcurrent\| |
|---|---:|---:|---:|---:|
| `shoulder_pan` | 0/0 | 100.0% | 0.0 | 22.5 |
| `shoulder_lift` | 0/0 | 100.0% | 0.0 | 56.6 |
| `elbow_flex` | 0/0 | 100.0% | 0.0 | 38.6 |
| `wrist_flex` | 0/0 | 100.0% | 0.0 | 17.0 |
| `wrist_roll` | 0/0 | 100.0% | 0.0 | 14.0 |

At the largest paired wrist-current divergence — 11.302–11.603 s, trailing-mean Δcurrent
11.5, inside the same reversal transient that produces the dominant pair4-clear false
alarm — the maximum |Δgoal2| is 0.0 ticks. The register cannot explain the transient even
locally.

The read is aimed correctly: LeRobot's control table places `Goal_Position_2` at (71, 2)
(`src/lerobot/motors/feetech/tables.py:91`) and the block read spans 56–72. This is a
firmware fact, not an addressing error. Note that `verify_block_read` cannot establish
this, because its 30-tick `goal2` tolerance passes a 0-versus-0 agreement silently.

### Confirmed on hardware, 28 July 2026

`probe_goal2.py` read the register directly at a nonzero standstill pose, through
LeRobot's per-register `sync_read` rather than the block read's sub-address extraction:

| motor | `Present_Position` | `Goal_Position` | `Goal_Position_2` |
|---|---:|---:|---:|
| `shoulder_pan` | 2094 | 0 | **0** |
| `shoulder_lift` | 710 | 0 | **0** |
| `elbow_flex` | 3692 | 0 | **0** |
| `wrist_flex` | 2726 | 0 | **0** |
| `wrist_roll` | 2137 | 0 | **0** |
| `gripper` | 1489 | 0 | **0** |

All six motors held clearly nonzero positions and register 71 read 0 on every one. A
further 292 samples over 10 s of motion produced exactly one distinct value per motor: 0.

Two independent read paths therefore agree, and the same `sync_read` path returns sensible
values for `Present_Position`, so the register is not being misread — it is not being
written.

One honest limitation of the standstill probe: `Goal_Position` (register 42) also read 0,
meaning no goal had been written since power-up, so an interpolated setpoint would have had
nothing to track during the watch. That is why the replay evidence above remains the primary
result — in `clear_goal2_a/b.csv` goals were actively written at 30 Hz and the arm was under
load and moving, and register 71 was still flat across all 649 frames. The probe closes the
read-path loophole; the replays close the physics.

`analyze_goal2.py` also prints a second table comparing cross-replay current prediction
from `goal_pos` against `goal2` (+71% to +200% worse). That is a null control, not a
finding: a constant input collapses the ridge fit to the training mean.

**Consequence.** The unexplained variation is friction, controller state, and thermal
state. None is observable from the bus, and no further commanded-side feature can recover
it. This removes the "find the missing feature" route entirely and leaves rollout-level
calibration — the Required correction below — as the only path.

## Temperature is plausible but not identified

A training began around 40–43 °C. B and pair4 began around 44–49 °C. This may shift
friction/current, but trajectory and temperature changed together. Moreover, the false
pattern already appears in B's source teleoperation, tying it directly to command-space
coverage. Temperature remains a candidate secondary contributor, not the established
cause.

## Required correction

1. Train the free-space predictor on multiple trajectories, directions, payload states,
   and sessions.
2. Validate model selection by leaving out complete trajectories/runs.
3. Add a commanded-feature coverage score. Abstain or fall back when the trajectory is
   outside the normality model's support.
4. Calibrate the alert threshold from maximum sustained scores on independent clear
   rollouts, at the desired per-rollout false-alarm budget.
5. Keep training, calibration, and test trajectories disjoint.
6. Require enough calibration runs for the chosen false-alarm claim; two is diagnostic,
   not statistical evidence.
7. Freeze the revised model and calibration procedure before collecting trajectory C.

The detector should not be described as passing until a new untouched clear trajectory
remains silent under a threshold chosen without access to it.

## Implemented diagnostic support

`freespace_model.py` now stores a commanded-feature nearest-neighbour coverage reference
and a held-out p99 coverage floor. Evaluation reports `OOD x` and abstains from a contact
label when the commanded motion exceeds that floor. This separates unsupported model
extrapolation from a calibrated residual crossing.

The A+B coverage model classifies pair4 clear as in coverage (`OOD x` approximately 0,
because its source B trajectory is training data), yet it still crosses at 2.2×. This
confirms that pair4's remaining problem is independent-repeat residual variability and
rollout-level calibration, not command OOD.

The new `calibrate` command computes one maximum causal residual vector per independent
clear rollout. At `--alpha 0.05`, split-conformal calibration needs at least 19 clear
rollouts for a finite distribution-free threshold. With the two available clear runs it
prints empirical maxima—19.9, 171.0, 58.2, 31.3, and 18.4 ticks from shoulder pan through
wrist roll—but deliberately writes no calibrated model. Those two runs explain the false
alerts; they cannot support the desired false-alarm claim.

`detectors.py` now carries both corrections into the common scoring interface: OOD frames
are unscorable abstentions, and `--use-calibrated` selects rollout-calibrated floors once a
sufficient calibration model exists. Frame-p99 remains available for reproducing the
locked failed experiment, but it must not be used for a new deployment claim.
