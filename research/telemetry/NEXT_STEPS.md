# Next steps — Week 2

Week 1 is done and the gate passed on both failure modes. See
[`WEEK1_REPORT.md`](./WEEK1_REPORT.md) for what happened and
[`README.md`](./README.md) for the tooling reference.

You are running roughly a week and a half ahead of the proposal's 3 Aug anchor. Spend
some of that on Week 2's schema freeze rather than rushing into the pilot — §5.1 is
right that retrofitting telemetry synchronisation later is miserable.

Week 2's deliverables, per the proposal: **reset automation, telemetry logging
integrated, data schema frozen.**

---

## 0. First, before anything else (~5 min)

`shoulder_pan` held ~12A for 2.3s during the `pair3` stall. Check it survived:

```bash
python research/telemetry/diagnose.py --port /dev/ttyACM0
```

Every servo should report `clean`. If `shoulder_pan` shows an alarm or its temperature
is markedly higher than its neighbours, power-cycle and let it cool before continuing.

Also: order the two spare STS3215 units if that hasn't happened (proposal §9). Two
alarm events in one week is the signal that lead times matter.

---

## 1. Decide what goes in the schema, then freeze it (~30 min)

This is the one genuinely irreversible decision of the week. Everything recorded from
here has to share a schema, or you have two incomparable corpora.

**Currently in `so_follower_telemetry.py`:** `observation.state` is 24-dim —
`[0:6] pos · [6:12] load · [12:18] current · [18:24] vel`.

**Three candidates to add, all already fetched by the block read and discarded:**

| Field | Addr | Case for including |
|---|---|---|
| `Status` | 65 | The servo's alarm byte. Labels taxonomy class `H1` (servo overload shutdown) automatically instead of by inference — and you've now hit it twice, so it will occur in the corpus. |
| `Present_Voltage` | 62 | Supply sag was visible during the `pair3` stall (12.0→11.6V). A whole-arm collision signal that no single joint's load shows. |
| `Present_Temperature` | 63 | Too slow for onset detection, but it's what explains failure-rate drift six weeks in — the §5.3 spec-sheet problem. |

Adding all three takes state to 42 dims. Cost on the wire is zero; they're in the same
transaction. `TruncateStateStep(keep=6)` is unaffected either way, so the policy input
does not change.

My recommendation: **add all three.** They're free, the retrofit cost if you want them
later is a full re-record, and `Status` in particular gives you a labelled hardware
class for nothing.

Once decided, record the layout in the difficulty spec sheet and don't change it again.

## 2. Verify the record loop holds 30 fps (~20 min)

The tight spot: two bus transactions per tick plus camera reads. If it can't hold 30
fps with your camera setup, that's a decision to make **now**, not mid-corpus.

```bash
python research/telemetry/record_with_telemetry.py \
    --robot.type=so101_follower_telemetry --robot.port=/dev/ttyACM0 --robot.id=my_follower \
    --robot.cameras='{ wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, top: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30}}' \
    --teleop.type=so101_leader --teleop.port=/dev/ttyACM1 --teleop.id=my_leader \
    --dataset.repo_id=ben/telemetry-smoketest --dataset.single_task="smoke test" \
    --dataset.num_episodes=1 --dataset.episode_time_s=20 --dataset.push_to_hub=false
```

Note `--dataset.repo_id` needs a real `user/name`. `${HF_USER}` is unset in your shell,
which is what produced the `PermissionError: '/sweep_...'` — an empty user makes the
path absolute and it tries to write to the filesystem root.

Watch for "record loop running slower than target FPS". If it warns, drop to one camera
or 20 fps and note which in the spec sheet. Then confirm the schema landed:

```bash
python -c "
from lerobot.datasets import LeRobotDataset
d = LeRobotDataset('ben/telemetry-smoketest')
print(d.features['observation.state']['shape'], d.features['action']['shape'])
print(d.features['observation.state']['names'])"
```

## 3. Camera rigidity (~30 min)

Deferred from Week 1 and now due. §5.1 calls viewpoint drift the most common silent
confound in this literature.

Mount both cameras so they physically cannot move — tape or clamp the mounts, don't
rely on friction. Photograph the reference view from each. Add a session-start check
against those photographs to the lab log, and log the check every time.

## 4. Reset automation (~half a day)

§7 calls this the highest-leverage engineering investment available, and the arithmetic
supports it: ~950 rollouts plus 180 demonstrations at ~1.5 min each is ~28 hours of
robot time, and manual reset is most of the overhead.

For T1 (cube → bowl) the cheapest useful version is a scripted return-to-home plus a
fixed pick-up-and-replace routine, driven by the same open-loop replay path the
collision pairs used — `log_teleop_telemetry.py --replay-dataset` already proves that
works. Record one reset trajectory, replay it between episodes.

Don't over-build it. If randomised object placement can't be automated, a scripted
arm-reset plus manual object placement still removes most of the per-episode cost.

---

## Then: Week 3 pilot

30 teleoperated demos on T1, fine-tune SmolVLA, measure the failure rate and tune it
into the 40–60% band. Gate: **difficulty locked**.

Two things to carry in from Week 1:

- At n=30 the failure-rate estimate carries roughly ±18 points (§5.3). Land in band and
  move on; don't burn days chasing a number the sample size can't resolve.
- Prefer physical difficulty levers over checkpoint selection. An undertrained policy
  fails by flailing, and Week 1 showed how easily an unrepresentative failure mode
  produces a signature that doesn't generalise.

## Open questions worth one run each

- **Slip precursor.** In `slip_a` cycle 1, gripper current drifted 344→337 over 1.6s
  before letting go. If that's micro-slip rather than thermal drift it would buy far
  more lead time than the 0.3s measured. A deliberately slow slide settles it.
- **`Goal_Position_2`** (addr 71, read-only) — unknown semantics, possibly the
  interpolated setpoint. `scan_registers.py` would reveal whether it carries signal.
