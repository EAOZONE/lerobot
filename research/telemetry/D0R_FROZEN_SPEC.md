# D0r — frozen model and calibration specification

**Frozen 28 July 2026.** Everything in sections 1–5 is fixed before trajectory C is
recorded and before any corpus rollout is collected. Changing any of it invalidates the
hold-out in [`TRAJECTORY_C_PROTOCOL.md`](./TRAJECTORY_C_PROTOCOL.md) unless the change is
recorded as a dated amendment *before* that hold-out is scored.

This freeze exists because the trajectory-B hold-out failed for a reason that only a freeze
prevents: the p99 operating point had been chosen after seeing trajectory-A results, so
there was no honest way to adjust it afterwards, and no way to tell a principled fix from a
fitted one. See [`PAIR4_HOLDOUT_RESULT.md`](./PAIR4_HOLDOUT_RESULT.md) and
[`D0R_CLEAR_DIAGNOSIS.md`](./D0R_CLEAR_DIAGNOSIS.md).

**What is frozen is the procedure, not a fitted model.** Every `.npz` in `models/` is a
development artifact. The deployable model is fit fresh from corpus training data by the
procedure below. In particular `freespace_ab_coverage.npz` must not become the corpus model:
it is trained on trajectory B, which is the trajectory whose clear replay it fails to
explain.

Reference implementation at freeze time:

| file | sha256 |
|---|---|
| `freespace_model.py` | `9fd4d6783ac865c7b3faf0c9b6340130f4c3449c5dde421daa78f591b12f42a4` |
| `detectors.py` | `831a4170d6262da4191a4efd079c8581382c9361c451cbcf533382b57aa99db9` |

Both at commit `51494b82`. Bug fixes that leave the numbers below unchanged are permitted;
anything that moves a score is an amendment.

The hashes identify the implementation, they do not sanctify it. A cosmetic change — a
rename, a comment, a lint fix — breaks the hash without touching a score. When that happens,
note the new hash here and move on. Only a change in behaviour is an amendment. (`ruff` from
the repo root currently reports `N803`/`N806` on `freespace_model.py`'s matrix variables
`X`, `Y`, `W`, `Xs`; that naming is deliberate and pre-dates the freeze.)

---

## 1. Inputs

**Commanded features only.** Every feature derives from `goal_pos.*`. No measured position,
velocity, load, or current enters the predictor.

This is not a detail — it is what makes the residual a detector. Feeding in measured
position hands the model following error, which is itself a contact signal, and it learns
to explain contact away exactly where the residual is needed. Conditioning on the command
also keeps inputs in-distribution during a fault, so the excess stays clean instead of
becoming an extrapolation artefact.

The 68-dimensional basis is the classic rigid-body regressor
`τ ≈ M(q)q̈ + C(q,q̇)q̇ + g(q) + friction`:

| block | count | terms |
|---|---:|---|
| gravity | 12 | `sin`/`cos` per arm joint, plus the `lift+elbow` two-link coupling |
| friction and inertia | 24 | `v`, `|v|`, `sgn(v)`, `a` per joint, gripper included |
| configuration-dependent inertia | 12 | `a × reach`, `|v| × reach` per joint |
| payload proxy | 2 | commanded gripper aperture, and a closed-jaw indicator |
| command history | 18 | commanded velocity at lags 2, 5, 10 |

Targets are the five arm joints' `Present_Current`. The gripper contributes features but is
not a residual target; slip is D0's job.

**`Goal_Position_2` is not an input and will not become one.** Register 71 reads a constant
0 on this firmware — confirmed across 649 frames of two matched clear replays and directly
by `probe_goal2.py`. The internal-setpoint hypothesis is closed.

## 2. Frozen constants

| constant | value | meaning |
|---|---:|---|
| `VEL_SMOOTH` | 5 | trailing frames smoothing commanded velocity before differencing |
| `LAGS` | 2, 5, 10 | frames at which past commanded velocity is offered |
| `RESIDUAL_SMOOTH` | 10 | trailing frames of residual smoothing at scoring time |
| `WARMUP` | 25 | unscorable leading frames, at fit and eval alike |
| `RIDGE_ALPHAS` | 0.1 … 1000 | ridge grid searched per joint |
| `FLOOR_PCTS` | 50, 95, 99, 99.9 | frame-residual percentiles stored for reference |
| `COVERAGE_PCT` | 99 | held-out command-distance percentile defining the coverage floor |
| persistence | 3 frames | consecutive frames above floor required for a contact call |

`WARMUP` is derived (`max(LAGS) + VEL_SMOOTH + RESIDUAL_SMOOTH`), not chosen. It is not
cosmetic: zero-initialised velocity/acceleration/lag features once made `pair1_clear` — a
run with no contact — score 2.2× the floor at t=0.13 s, outranking the genuine contacts.

`RESIDUAL_SMOOTH = 10` matches the window `causal_eval.py` measured as best for D0, so the
two rungs are judged on the same timescale.

## 3. Fit procedure

1. Training data is **no-contact runs only**. `fit` refuses files whose names look faulted.
   This is a normality model; contact is by construction what it cannot explain.
2. Drop `WARMUP` frames from the head of every run.
3. Standardise features on the pooled training set.
4. **Leave-one-whole-run-out validation whenever more than one no-contact run is supplied.**
   Contiguous six-block folds are a single-run fallback only. Within-run folds leak
   trajectory-specific behaviour — adjacent frames at 30 Hz are near-duplicates — and
   underestimate new-run error, which is one of the four defects the B hold-out exposed.
5. Select ridge alpha per joint by mean held-out MAE across those folds.
6. Refit on all training frames at the chosen alpha.
7. Store per-joint held-out residual percentiles as **reference floors only**. They are not
   the operating point (§4).
8. Store the standardised training features as a coverage reference, plus the held-out p99
   nearest-neighbour distance as the coverage floor.

## 4. Operating point — per rollout, never per frame

**The frame percentile is not the operating point and must never be used as one.** A p99
frame floor permits ~1% of ordinary frames to cross; at 30 Hz that is ~18 exceedances per
minute before temporal correlation, and 3-frame persistence does not repair it because
residuals around direction changes are correlated by construction. The operational
constraint is false alarms **per rollout** (`vla-failure-detection.md` §6).

The calibration unit is therefore one independent clear rollout, reduced to one number:

1. For each clear calibration rollout, compute the per-joint maximum smoothed causal
   residual after warm-up (`clear_run_residual_peaks`).
2. Divide by the model's frame-p99 reference floors and take the **maximum across joints**.
   The operational false alarm is *any* joint crossing, so calibrate one joint-max score per
   rollout, not five marginal per-joint rates.
3. Take the split-conformal order statistic `rank = ceil((n+1)(1−α))` of those rollout
   scores. Deployed floors are the frame-p99 floors scaled by that single multiplier.

**Declared α:** 0.05 primary, 0.10 secondary. Both are calibrated and both are reported,
matching §6's requirement to report TPR at 5% and 10% per-rollout false-alarm rates. No
other α may be introduced after seeing results.

**Minimum calibration size.** A finite distribution-free threshold requires
`rank ≤ n`, i.e. **n ≥ 19 at α = 0.05** and n ≥ 9 at α = 0.10. `calibrate` refuses to write
a model below that and prints empirical maxima marked development-only. Do not work around
this by falling back to frame percentiles.

## 5. Detection rule

Per frame, per joint:

- residual is **one-sided**: `clip(measured − predicted, 0, None)`. A joint drawing less
  than predicted is model error, not a fault; folding it in adds only noise.
- trailing 10-frame mean.
- score is normalised so **1.0 is the operating threshold**.
- frames before `WARMUP` are **unscorable**, not zero-scored.
- frames whose commanded-feature nearest-neighbour distance exceeds the coverage floor are
  **unscorable abstentions**, not contact claims. A residual there may be physically real,
  but the model has no calibrated basis for the label, and `detectors.py` excludes those
  frames from AUPRC and false-alarm metrics rather than counting them either way.
- a contact call requires **3 consecutive scorable frames** at or above threshold. Fewer is
  reported as spike-only.

**Reporting requirement carried forward from the B hold-out:** a threshold crossing is
attributed to an onset only inside the declared event window (−2.0 s to +1.0 s). A crossing
outside it is a false alarm even if the rollout eventually fails. Peak time *and* peak joint
must be cross-checked against ground truth before any verdict is believed — at p99.9 both
`pair1` and `pair2` printed CONTACT on the wrong joint over a second from the real event,
which in a summary table reads as a clean 3/3.

---

## 6. What the corpus must supply

Calibration is corpus-phase work (`NEXT_STEPS.md` §4). These fields must be in the
annotation schema *before* collection, or the calibration split cannot be built afterwards.

**Per rollout, required:**

| field | why |
|---|---|
| verified no-contact flag | eligibility for the calibration pool |
| independence group | replays or near-duplicates of one command trace count once |
| session ID and date | session is a shift axis, and must not be confounded with split |
| start/end servo temperature | the one shift D0r has no term for |
| task, policy checkpoint, seed | so the calibration distribution matches deployment |
| payload state | the gripper payload proxy is a feature; its distribution must be covered |
| split assignment | fixed before any fitting; train/calibration/test never cross |
| outcome and fault class | autonomous *successes* are the hard negatives |

**Three rules that are easy to get wrong:**

1. **"Clear" must be verified, not inferred.** A rollout enters the calibration pool because
   video and matched telemetry show no contact — never because the detector stayed silent.
   The latter is circular and would guarantee a passing false-alarm rate.
2. **Replays of one command trace are one calibration unit, not many.** This is precisely
   what the B hold-out got wrong at n=2. Nineteen replays of a single trace are not 19
   rollouts. Independence is over trajectories, sessions, and payload states.
3. **Autonomous successful rollouts must be in the calibration split.** Every negative D0r
   has seen so far is teleoperated. Deployment negatives are autonomous policy rollouts — a
   third motion distribution the free-space model has never been fit on, and the one that
   decides RQ4.

## 7. Deliberately not frozen

- The free-space **training corpus**. The deployed model is refit on corpus training data
  spanning multiple trajectories, directions, payload states, and sessions. Current models
  are development artifacts.
- The **number** of calibration rollouts, beyond the ≥19 minimum.
- Whether a session-offset or temperature covariate is added. If tested, it must be fit on
  no-contact calibration data only, and temperature may be analysed as a covariate but not
  claimed causal from the A/B pair.
- D0+'s window length, which is a separate rung with its own ablation.

## 8. Change control

Amendments are appended below with a date and a reason, and are only valid before the
affected hold-out is scored. An amendment made after seeing hold-out results converts that
hold-out into development data permanently — as happened to trajectory B.

| date | change | reason |
|---|---|---|
| 2026-07-28 | initial freeze | after the trajectory-B specificity failure and the `Goal_Position_2` elimination |

---

## Reproduce

```bash
conda activate lerobot

# fit -- multiple no-contact runs trigger leave-one-run-out validation
python research/telemetry/freespace_model.py fit \
    research/telemetry/runs/freespace_*.csv \
    --out research/telemetry/models/freespace.npz

# calibrate -- one unit per independent clear rollout; refuses if underpowered
python research/telemetry/freespace_model.py calibrate \
    --model research/telemetry/models/freespace.npz --alpha 0.05 \
    --out research/telemetry/models/freespace_calibrated.npz \
    <clear rollout CSVs>

# score at the calibrated per-rollout operating point
python research/telemetry/freespace_model.py eval --use-calibrated \
    --model research/telemetry/models/freespace_calibrated.npz <CSVs>

python research/telemetry/detectors.py score <CSVs> --detectors d0r \
    --model research/telemetry/models/freespace_calibrated.npz --use-calibrated \
    --out research/telemetry/runs/scores.parquet
```

With the two clear runs available at freeze time, `calibrate` prints rollout maxima of
19.9, 171.0, 58.2, 31.3, and 18.4 ticks (pan through wrist roll) and correctly declines to
write a calibrated model. That is the expected output, not a failure.
