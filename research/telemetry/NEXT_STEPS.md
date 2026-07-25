# What changed, and what to do next at the robot

`README.md` in this directory is the full reference. This is the short version.

## What changed

**New files**

| File | What it does |
|---|---|
| `feetech_block.py` | One-transaction read of pos/vel/load/volt/temp/current (addr 56–70), shared timestamp. Both the logger and the robot subclass use it. |
| `so_follower_telemetry.py` | SO-101 follower whose `observation.state` is 24-dim (pos, load, current, vel) instead of 6. |
| `record_with_telemetry.py` | `lerobot-record` with that robot type registered. Zero edits to `src/lerobot/`. |
| `truncate_state_step.py` | Slices `observation.state` back to 6 dims for the policy, so SmolVLA's input is unchanged. |
| `gate_analysis.py` | Go/no-go analysis: grip-state features, slip-onset, matched-pair collision diff. |
| `diagnose.py`, `scan_registers.py` | Servo alarm dump and empirical SRAM-register scan. |

**Changed**

- `log_teleop_telemetry.py` — live `GRIP OK` / `NO GRIP` readout; single-keypress markers; `--replay-dataset` mode; block read moved to `feetech_block.py`.
- `probe_bus.py` — enables torque (load/current read ~0 without it); refuses to run on an alarmed servo; `--hold` to energise only some joints.

Nothing in `src/lerobot/` was modified.

## Next steps at the robot

Run everything from the repo root with the `lerobot` conda env active.

### 0. Safety, before you touch anything

`shoulder_lift` latched an overload alarm earlier. If any write fails with an empty
error string, that's it again — unplug the **servo power** (not just USB), wait 5s,
replug. `python research/telemetry/diagnose.py --port /dev/ttyACM0` shows the raw alarm
byte. Move the arm somewhere the shoulder isn't cantilevered before enabling torque.

### 1. Re-record the slip runs (~15 min)

This is the one that failed last time. Watch the readout — it must say `GRIP OK`
**before** you let the object slide, otherwise you're recording the aftermath again.

```bash
python research/telemetry/log_teleop_telemetry.py \
    --follower-port /dev/ttyACM0 --leader-port /dev/ttyACM1 \
    --out research/telemetry/runs/slip_a.csv
```

Per run: still ~3s → close on a smooth object until `GRIP OK` → hold 2s → lift →
let it slide, pressing `s` **as it starts moving** → still ~3s → Ctrl-C.
Do `slip_a`, `slip_b`, `slip_c`.

### 2. Record one sweep episode to replay (~10 min)

Any trajectory that passes through the space where you'll put the obstacle.

```bash
lerobot-record --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=my_follower \
    --teleop.type=so101_leader --teleop.port=/dev/ttyACM1 --teleop.id=my_leader \
    --dataset.repo_id=${HF_USER}/sweep --dataset.single_task="sweep" \
    --dataset.num_episodes=1 --dataset.push_to_hub=false
```

### 3. Replay it twice — obstacle and clear (~15 min)

Same commanded trajectory both times, so the only difference is the collision.

```bash
# obstacle in the path
python research/telemetry/log_teleop_telemetry.py --follower-port /dev/ttyACM0 \
    --replay-dataset ${HF_USER}/sweep --replay-episode 0 \
    --out research/telemetry/runs/pair1_obstacle.csv

# path clear -- do not move anything else
python research/telemetry/log_teleop_telemetry.py --follower-port /dev/ttyACM0 \
    --replay-dataset ${HF_USER}/sweep --replay-episode 0 \
    --out research/telemetry/runs/pair1_clear.csv
```

Repeat for `pair2_*` and `pair3_*`. Keep a hand near the power switch on the first
obstacle run — the arm will push into it open-loop.

### 4. Decide the gate (~5 min)

```bash
python research/telemetry/gate_analysis.py research/telemetry/runs --plot
```

- **Slip passes** if `slip_onset_drop` is large for slip runs and ~0 for clean.
- **Collision passes** if the matched pairs report `CLEAR SIGNATURE`.
- Then look at the plots. The criterion is still *visible by eye* — if it takes
  statistics to find, it won't detect online at useful lead times.

### 5. Only if the gate passes — telemetry recording

```bash
python research/telemetry/record_with_telemetry.py \
    --robot.type=so101_follower_telemetry --robot.port=/dev/ttyACM0 --robot.id=my_follower \
    --teleop.type=so101_leader --teleop.port=/dev/ttyACM1 --teleop.id=my_leader \
    --dataset.repo_id=${HF_USER}/telemetry-test --dataset.single_task="test" \
    --dataset.num_episodes=1 --dataset.push_to_hub=false
```

Watch for "record loop running slower than target FPS". Two bus transactions per tick
plus cameras is the tight spot; if it warns, drop to 2 cameras or 20 fps and **decide
before collecting the real corpus**, not during it.

Then confirm the schema landed:

```bash
python -c "
from lerobot.datasets import LeRobotDataset
d = LeRobotDataset('${HF_USER}/telemetry-test')
print(d.features['observation.state']['shape'], d.features['action']['shape'])
print(d.features['observation.state']['names'][:6])"
```

Expect `(24,) (6,)` and the first six names ending in `.pos`. Layout is
`[0:6] pos · [6:12] load · [12:18] current · [18:24] vel`.

## Still open

- Gripper `Present_Load` clips at ±500 (`Max_Torque_Limit`, set in
  `SOFollower.configure()`). Current doesn't clip and is the better grip channel.
  Document as a platform constraint rather than raising the limit — that guard is what
  protects the servo you've already alarmed once.
