# Week 1 — telemetry go/no-go

Scripts for the Week 1 gate in the VLA failure-detection proposal: prove that the
SO-101's Feetech STS3215 servos report a usable force-adjacent signal before five
weeks are sunk into the study.

## Where the data comes from

LeRobot already knows how to read these registers — they're in the STS/SMS control
table (`src/lerobot/motors/feetech/tables.py`), which `sts3215` maps to:

| Register | Addr | Bytes | Notes |
|---|---|---|---|
| `Present_Position` | 56 | 2 | sign-magnitude, bit 15 |
| `Present_Velocity` | 58 | 2 | sign-magnitude, bit 15 |
| `Present_Load` | 60 | 2 | sign-magnitude, bit 10 → ±1000 = ±100% of max torque |
| `Present_Voltage` | 62 | 1 | |
| `Present_Temperature` | 63 | 1 | °C |
| `Present_Current` | 69 | 2 | unsigned, ~6.5 mA/LSB per Feetech docs — **verify empirically** |

Read them with `bus.sync_read("Present_Load", normalize=False)`. `normalize=False`
matters: only `Present_Position` and `Goal_Position` are in `NORMALIZED_DATA`, and
raw units are what you want for detector features anyway.

The proposal is right that these are **not** in the recorded dataset schema —
`SOFollower.get_observation()` reads `Present_Position` only. Extending that is the
Week 2 job; these scripts are the Week 1 job and bypass the dataset entirely.

## Runbook

Run from the `lerobot` conda env (`conda activate lerobot`).

> **Torque must be ON.** `Present_Load` is the servo's output PWM duty and
> `Present_Current` is its drive current. With torque disabled the driver is idle and
> both read ~0 no matter how hard you push the arm by hand — and `SOFollower`
> disables torque on disconnect, so that's the state you inherit between sessions.
> `probe_bus.py` and `scan_registers.py` now enable it (holding the current pose)
> after a confirmation prompt; `log_teleop_telemetry.py` gets it for free because
> `SOFollower.connect()` → `configure()` leaves torque enabled.

```bash
# Days 1-2 — does the hardware report it at all, and how fast?
python research/telemetry/probe_bus.py --port /dev/ttyACM0

# Holding all six joints makes the shoulder fight gravity for the whole run, which
# can trip overload protection. Hold only what you need:
python research/telemetry/probe_bus.py --port /dev/ttyACM0 --hold gripper

# If a register reads flat zero WITH torque on: find out empirically which SRAM
# addresses respond to load, instead of trusting the control table.
python research/telemetry/scan_registers.py --port /dev/ttyACM0 --motor gripper

# If a write fails with an empty error string -- a latched alarm the SDK can't name.
python research/telemetry/diagnose.py --port /dev/ttyACM0

# Day 3 — signature test. One isolated event per file.
python research/telemetry/log_teleop_telemetry.py \
    --follower-port /dev/ttyACM0 --leader-port /dev/ttyACM1 \
    --out research/telemetry/runs/slip_a.csv --verify   # check block read once
python research/telemetry/log_teleop_telemetry.py \
    --follower-port /dev/ttyACM0 --leader-port /dev/ttyACM1 \
    --out research/telemetry/runs/slip_a.csv

# Matched-trajectory collision: record one sweep, then replay it twice -- obstacle
# in the path for one, clear for the other. Same commanded trajectory sample-for-sample.
python research/telemetry/log_teleop_telemetry.py --follower-port /dev/ttyACM0 \
    --replay-dataset ${HF_USER}/sweep --replay-episode 0 \
    --out research/telemetry/runs/pair1_obstacle.csv
python research/telemetry/log_teleop_telemetry.py --follower-port /dev/ttyACM0 \
    --replay-dataset ${HF_USER}/sweep --replay-episode 0 \
    --out research/telemetry/runs/pair1_clear.csv

# Day 3 — the go/no-go artifacts
python research/telemetry/gate_analysis.py research/telemetry/runs
python research/telemetry/plot_signatures.py research/telemetry/runs/*.csv
```

Name files `collide_*`, `slip_*`, `clean_*` (class is read from the stem) and matched
replay pairs `pairN_obstacle` / `pairN_clear`.

Collect at least: 3 collision runs, 3 slip runs (smooth object, lift, let it slide),
3 clean runs of the same motion as a control. Three of each because one run tells you
nothing about whether a spike was the event or the bus.

## Latched alarms

A Feetech servo that trips overload/overheat protection **latches** the alarm and
rejects every subsequent write until the servo power is cycled — unplug the power
supply itself, not just USB. The scservo SDK names only error bits 1/2/4/8/32, so an
alarm on any other bit surfaces as `RuntimeError: Failed to write ... after 1 tries.`
with an empty explanation. `diagnose.py` prints the raw byte; `probe_bus.py` and
`scan_registers.py` now refuse to enable torque on an alarmed servo and say so.

## Two protocol traps, both already paid for

**Slip runs need a real grasp first.** The first nine-run batch recorded gripper
current of 2–3 throughout every "slip" — the object was never gripped, so there was no
grip force to lose and what got captured was the aftermath, not the onset. The logger
now prints a live `GRIP OK` / `NO GRIP` readout (threshold: gripper current ≥ 150).
Wait for `GRIP OK`, hold 2s, lift, *then* let it slide.

**Collision runs need a matched control.** Peak load during ordinary teleop (456–528)
is indistinguishable from peak load during a collision (450–565). Comparing a collide
run against a differently-executed clean run cannot resolve it. Use the replay pair
above; `gate_analysis.py` diffs them frame-by-frame on `frame_idx`.

**Use current, not load — for both failure modes.** On the first matched pair, smoothed
*load* excess peaked at 5.3× baseline spread over 3 separate episodes, with its maximum
2.7s away from the actual contact: noise. Smoothed *current* excess peaked at 25× in a
single episode exactly on the contact. Same lesson as the gripper, where load clips at
±500 and current does not. `Present_Load` is a coarse PWM-duty estimate; treat
`Present_Current` as the primary channel and load as corroboration.

**Smooth before judging.** Contact is sustained over tenths of a second; replay variance
is single-frame spikes. A 5-frame mean separates them — unsmoothed, raw load excess
scored 11.6× on a pair with no contact at all.

## Recording a corpus with telemetry

`so_follower_telemetry.py` widens `observation.state` from 6 to 24 dims
(pos, load, current, vel — grouped by field, positions first). The `action` column is
unchanged: 6 commanded joint positions.

> **If the detectors underperform, consider adding `Goal_Position` too.** It's the
> servo's target register (addr 42, sign-magnitude bit 15) and sits just outside the
> block read — widening the span to 42–70 pulls it into the same transaction for free,
> ~210 bytes/tick against a 250-byte packet limit. `Present_Position − Goal_Position`
> is following error: how far behind the arm is from where it was commanded, which
> grows when something blocks it. Deliberately left out for now to keep the vector
> small. Two things to know if you add it: normalize it through `bus._normalize` so it
> shares a scale with `pos` (raw ticks can't be differenced against normalized
> positions), and note that following error is also large during normal fast motion —
> the discriminating pattern is error that *persists while velocity is near zero*.

```bash
python research/telemetry/record_with_telemetry.py \
    --robot.type=so101_follower_telemetry --robot.port=/dev/ttyACM0 ...
```

That wrapper exists so `src/lerobot/` stays untouched — importing the module registers
the robot type, and `record()` parses `sys.argv` exactly as `lerobot-record` does.

**Keep telemetry out of the policy.** Put `TruncateStateStep(keep=6)` in the
training/inference pipeline. Because positions occupy `state[:6]`, SmolVLA then sees an
input byte-identical to a plain `so101_follower` recording, while detectors read all 24.
Without it the policy consumes load and current, which confounds RQ2 — the comparison
assumes the policy under test does not already encode the signal being evaluated.

## Passing the gate

Eyeball the plots first. You need a **visible** current/load excursion at the
`collision` marker and a **visible** gripper-load drop at the `slip` marker, absent
from the control runs. The per-marker ratio summary the plot script prints is a
convenience, not the criterion — if you need it to find the event, the signal is too
weak for online detection at useful lead times.
