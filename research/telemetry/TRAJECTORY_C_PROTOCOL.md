# Trajectory C — pre-registered D0r hold-out

**Registered 28 July 2026, after the freeze in
[`D0R_FROZEN_SPEC.md`](./D0R_FROZEN_SPEC.md) and before trajectory C is recorded.**

Trajectory B is spent. It failed specificity and has since been used for diagnosis, model
fitting, and coverage analysis, so it can never serve as an untouched hold-out again
([`PAIR4_HOLDOUT_RESULT.md`](./PAIR4_HOLDOUT_RESULT.md)). C is the replacement, and this
document exists so that its verdict cannot be negotiated after the fact.

Nothing here may change once C is recorded.

---

## 1. What C tests, and what it does not

The B hold-out conflated two questions and failed on the one it was not really measuring.
C separates them.

| question | evidence | this document |
|---|---|---|
| Does D0r respond to real contact on an unseen trajectory? | matched clear/obstacle pair with physical ground truth | **primary** |
| Does D0r stay quiet on independent clear rollouts? | ≥19 autonomous clear rollouts, corpus calibration split | not here — see below |

A single matched pair cannot estimate a false-alarm rate; that was the arithmetic error
behind the frame-p99 floor. The per-rollout false-alarm claim comes from the corpus
calibration split and nowhere else. C's clear run is a *necessary* check, not a rate.

**C is therefore gated on calibration existing.** It may be *recorded* early — during the
T1 pilot, at nearly no extra robot cost — but it must not be *scored* until a model has
been fit on corpus training data and calibrated on ≥19 independent clear rollouts. Record
it, seal it, and do not open it.

## 2. Locked before recording

**Model.** Fit by the §3 procedure of the frozen spec on corpus training data only.
Trajectory C contributes to neither the training nor the calibration split. No development
`.npz` in `models/` is eligible.

**Operating point.** Per-rollout conformal, α = 0.05 primary and α = 0.10 secondary, both
calibrated before C is opened. No other α is admissible. No frame percentile is admissible.

**Detector settings.** As frozen: 10-frame trailing residual smoothing, 25 unscorable
warm-up frames, 3-frame persistence, one-sided residual, coverage abstention enabled.

**Recording conditions.**

- A new session on a different day from the training and calibration sessions, with start
  and end temperature recorded via `diagnose.py`.
- A trajectory distinct from A and B in path, direction, and workspace region.
- Contact on a joint other than `shoulder_pan` — B established that the residual transfers
  to `shoulder_lift` and `elbow_flex`, and pan alone is the weakest possible test.
- **Light and moderate contacts only.** No `pair3`-style stall. That exemplar is already
  captured and 12 A for 2.3 s is how these servos die.
- The obstacle run and the clear run replay the identical recorded command trace, verified
  byte-identical across all six motors and all frames before scoring.
- Keypress markers recorded, but see §3 — they are not the ground truth.

## 3. Ground truth is physical, not a keypress

A contact counts as a validated event only if matched obstacle-minus-clear position
divergence reaches **≥150 raw ticks** on at least one joint. This threshold is carried
unchanged from the B analysis, where it correctly identified four of five keypresses as
physically consequential and the third as unsupported.

A keypress without matched divergence is not a collision. It is excluded from recall and
does not become a miss. Operator reaction delay also means marker timing bounds the event
window, never the onset.

## 4. Primary pass condition

Declared in full, in advance:

1. On the obstacle run, a sustained crossing (≥3 consecutive scorable frames at or above the
   α = 0.05 calibrated threshold) on a physically validated contacted joint, inside the
   −2.0 s to +1.0 s window around that contact's divergence interval.
2. On the clear run, **no** sustained crossing anywhere.
3. Both runs are within command coverage — see §5.

All three must hold. Anything less is a fail, regardless of how strong the obstacle response
is; B's obstacle peak was 21.67× and it still failed.

## 5. How abstention counts — declared now, not later

The coverage diagnostic can abstain, and there is an obvious temptation to treat abstention
as "not a false alarm" after seeing results. Decided in advance:

- An abstaining frame is neither a true nor a false detection and is excluded from metrics.
- If the **clear** run abstains over more than 10% of its scorable frames, the result is a
  **coverage failure**, reported as such. It is not a pass. It means the training corpus does
  not cover ordinary motion, which is a deployment defect in its own right.
- If the obstacle run abstains across a validated contact's event window, that event counts
  as a **miss**. A detector that declines to answer has not detected anything.

## 6. What may not happen after C is opened

- No percentile, threshold, α, window, persistence, or smoothing change.
- No refit, no feature addition, no session-offset correction fitted with C visible.
- No substitution of a differently-trained model.
- No re-recording of C "because something went wrong in the run" unless the defect is
  mechanical and documented *before* any score is computed — a bus dropout or a mismatched
  command trace qualifies; a disappointing score does not.

Post-hoc analysis is allowed and encouraged, but must be labelled explicitly as post-hoc and
cannot alter the recorded primary verdict. That is how the B result was handled and it is
the reason the B diagnosis is trustworthy.

## 7. If C fails

C becomes development data permanently, exactly as B did. Record the failure in a dated
result file, diagnose it on C and the corpus, then pre-register trajectory D against the
amended spec. Do not attempt to rescue C.

A second consecutive specificity failure would be a substantive finding about the approach,
not merely a setback: it would mean the free-space residual needs a signal the commanded
trace does not contain, and D0r's role in the paper should be reconsidered rather than
retuned. `Goal_Position_2` was the last cheap observable candidate and it is eliminated.

## 8. What may be claimed from C

One trajectory pair, so: **a sensitivity check on unseen motion, not an accuracy estimate.**

Admissible: whether the calibrated detector responds to validated contact on a trajectory it
was not fit on, on which joints, with what lead time relative to divergence onset, and
whether the matched clear run stays silent.

Not admissible from C: AUPRC, false-alarm rate, lead-time distribution, or any per-class
recall. Those come from the corpus, at corpus sample sizes, with binomial confidence
intervals.

## 9. Analysis, written before the data exists

```bash
conda activate lerobot

# 0. integrity: identical commands, expected frame count
python - <<'PY'
import pandas as pd
a = pd.read_csv("research/telemetry/runs/pair5_clear.csv", keep_default_na=False)
b = pd.read_csv("research/telemetry/runs/pair5_obstacle.csv", keep_default_na=False)
cols = [c for c in a.columns if c.startswith("goal_pos.")]
assert len(a) == len(b), (len(a), len(b))
assert all(a[c].equals(b[c]) for c in cols), "command traces differ; not a matched pair"
print(f"matched pair: {len(a)} frames, {a['t'].iloc[-1]:.2f}s")
PY

# 1. physical ground truth BEFORE any detector output is looked at
#    (>=150-tick matched divergence intervals, per joint)

# 2. primary verdict at the calibrated per-rollout operating point
python research/telemetry/freespace_model.py eval --use-calibrated \
    --model research/telemetry/models/freespace_calibrated.npz \
    research/telemetry/runs/pair5_clear.csv \
    research/telemetry/runs/pair5_obstacle.csv

# 3. event-matched scoring through the common interface
python research/telemetry/detectors.py score \
    research/telemetry/runs/pair5_*.csv --detectors duration,d0,d0r \
    --model research/telemetry/models/freespace_calibrated.npz --use-calibrated \
    --out research/telemetry/runs/pair5_scores.parquet
python research/telemetry/detectors.py report research/telemetry/runs/pair5_scores.parquet
```

Step 1 is deliberately ahead of step 2. Establish what physically happened before seeing
what the detector said, or the ≥150-tick boundary becomes negotiable.

Record the outcome in `TRAJECTORY_C_RESULT.md` — including the primary verdict, before any
secondary analysis.
