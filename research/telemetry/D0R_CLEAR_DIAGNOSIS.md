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
by the externally logged goal sequence—likely the internal interpolated setpoint, velocity
profile, friction, and thermal state.

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
