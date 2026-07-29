# Failure-corpus labeling guide

**Version:** 1 · **Freeze before corpus labeling begins**

## Two-pass procedure

Pass 1 assigns rollout success and a fine class from full video and telemetry. Pass 2 marks
the onset: the first frame at which failure became inevitable under the fixed task and
recovery protocol. Separately mark the unrecoverable boundary: the last frame at which the
fixed scripted recovery could still plausibly convert the rollout to success.

If either boundary is ambiguous, record inclusive earliest/latest frames; do not collapse
uncertainty to a falsely precise point. A deviation that self-corrects and completes the
task is a negative, even if its telemetry looks alarming.

## Fine classes

- `S1`: wrong object; `S2`: wrong target location; `S3`: no attempt or semantic stall
  before engagement.
- `E1`: grasp miss; `E2`: grasp slip; `E3`: collision; `E4`: drop in transit; `E5`:
  placement miss; `E6`: mechanical stall, joint limit, or kinematic dead-end.
- `H1`: servo overload/shutdown; `H2`: bus communication dropout. Hardware events are
  logged but excluded from primary policy/detector analysis.

Analysis collapses `S1–S3` to `semantic` and `E1–E6` to `execution`. Fine labels remain in
the release, but individual-class claims require adequate counts.

## Multiple events and recovery

Record every event in temporal order. The primary fine class is the earliest event that
makes failure inevitable; later consequences remain secondary events. Detector triggers
are matched to events only inside the declared attribution window. An unrelated early
trigger is a false alarm even if failure occurs later.

Record grasp start, grasp end, and duration whenever contact is attempted. Record recovery
eligibility at onset, retry count, trigger detector/score/threshold, and whether recovery
converted failure, disrupted a success, failed safely, or moved unsafely.

## Splits and clear calibration units

Assign train/calibration/test before fitting. Near-duplicate episodes and replays of one
command trace share an `independence_group` and never cross splits. Threshold selection
uses calibration only; held-out test conditions stay sealed.

`verified_clear=true` requires video and telemetry review showing no unintended contact.
Detector silence is never evidence of clear status. Autonomous successes must appear in
calibration, and session, temperature, payload, task, checkpoint, and motion distribution
must cover deployment. Nineteen independent clears are the minimum for a distribution-free
5% per-rollout threshold.

## Second annotator

Independently label a 15% subset stratified by task, outcome, collapsed class, and session.
Do not show first-annotator labels. Report Cohen's kappa for class labels and onset
agreement using exact-frame and within-one/three/five-frame rates plus uncertainty overlap.
If kappa is below roughly 0.7 or onset agreement is poor, adjudicate examples, amend this
guide with dated rationale, and relabel the subset before detector fitting.

