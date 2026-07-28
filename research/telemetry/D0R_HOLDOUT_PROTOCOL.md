# D0r trajectory hold-out protocol

**Locked before trajectory-B data collection: 27 July 2026.**

**Evaluation completed 27 July 2026. Primary result: failed because `pair4_clear`
produced sustained p99 crossings.** The locked protocol below is preserved unchanged;
see [`PAIR4_HOLDOUT_RESULT.md`](./PAIR4_HOLDOUT_RESULT.md) for the recorded result and
allowed secondary analysis.

This file records the evaluation choices before the hold-out result exists. Do not edit
the operating point or acceptance rules after inspecting trajectory B. Any exploratory
alternative belongs in a separately labeled post-hoc analysis.

## Fixed choices

- **Detector:** the free-space current residual in `freespace_model.py`, unchanged.
- **Operating point:** per-joint **p99** held-out free-space residual floor.
- **Residual window:** 10-frame causal trailing mean.
- **Warm-up:** 25 frames, as defined by `freespace_model.WARMUP`.
- **Contact alert:** score at or above 1.0 for at least 3 consecutive frames.
- **Primary joint:** not `shoulder_pan`. All Week 1 contacts used that joint; trajectory B
  must test a different arm joint to count as a joint-generalization result.
- **Contact intensity:** light or moderate, comparable to pair1/pair2. Do not reproduce
  pair3's hard stall.
- **No-contact negative:** replay the identical trajectory with the obstacle absent.
- **No model or threshold changes after seeing the hold-out pair.**

The p99 choice is fixed even though it was selected after inspecting the trajectory-A
runs. That makes trajectory B a genuine test of the chosen operating point. Sweeping p95,
p99, and p99.9 on B may be shown only as post-hoc sensitivity analysis and may not replace
the p99 primary result.

## Collection order

Run the session health check first and record temperatures in the lab log:

```bash
python research/telemetry/diagnose.py --port /dev/ttyACM0
```

Then collect a new free-space sweep and a matched clear/obstacle pair. The commanded
trajectory must be identical within each pair, and trajectory B must differ materially
from the trajectory used by pair1--pair3.

```bash
python research/telemetry/log_teleop_telemetry.py \
    --follower-port /dev/ttyACM0 --leader-port /dev/ttyACM1 \
    --out research/telemetry/runs/freespace_b_01.csv

python research/telemetry/log_teleop_telemetry.py \
    --follower-port /dev/ttyACM0 \
    --replay-csv research/telemetry/runs/freespace_b_01.csv \
    --out research/telemetry/runs/pair4_clear.csv

python research/telemetry/log_teleop_telemetry.py \
    --follower-port /dev/ttyACM0 \
    --replay-csv research/telemetry/runs/freespace_b_01.csv \
    --out research/telemetry/runs/pair4_obstacle.csv
```

`freespace_b_01.csv` itself is the trajectory source: its normalized `goal_pos.*`
columns are replayed directly. No Hugging Face dataset or conversion step is required.

The obstacle must actually stop or materially resist the selected joint. If it slides or
tips, record the attempt as a protocol failure and repeat without treating it as either a
positive or a negative.

## Evaluation

Fit on trajectory A and score B first. Do not inspect a B-fitted model before recording
this result.

```bash
python research/telemetry/freespace_model.py fit \
    research/telemetry/runs/freespace_0*.csv \
    --out research/telemetry/models/freespace_a.npz

python research/telemetry/freespace_model.py eval \
    --model research/telemetry/models/freespace_a.npz --floor-pct 99 \
    research/telemetry/runs/pair4_clear.csv \
    research/telemetry/runs/pair4_obstacle.csv
```

Then reverse the direction as a secondary result: fit only on B free-space data and score
the trajectory-A clear/obstacle pairs. Keep p99 fixed.

## Outcomes to record

For both the clear and obstacle run, record:

- peak normalized residual and joint;
- first sustained threshold crossing and marker-relative latency;
- number of pre-event false alarms;
- session start/end temperature;
- whether contact visibly stopped/resisted the intended joint.

The primary hold-out succeeds only if the obstacle run produces a sustained crossing on
the physically contacted joint and the matched clear run produces none. Any other result
is reported as-is; it does not authorize refitting or choosing a new percentile on B.
