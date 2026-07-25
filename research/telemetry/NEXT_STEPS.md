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

## 0. Servo health after the `pair3` stall — **DONE (25 Jul)**

`shoulder_pan` held ~12A for 2.3s during the `pair3` stall; checked with
`diagnose.py` and cleared to continue.

Re-run it whenever a run ends in a stall, and at the start of each session:

```bash
python research/telemetry/diagnose.py --port /dev/ttyACM0
```

Because `Present_Temperature` is deliberately not in the recorded schema, this is also
where session temperature gets captured. Note the per-servo temperatures in the spec
sheet each session — that series is what explains failure-rate drift in Week 6+ (§5.3),
and it is worthless unless started now while the arm is known-good.

Still outstanding: order the two spare STS3215 units (proposal §9). Two alarm events in
one week is the signal that lead times matter.

---

## 1. Schema — decided, verify it in the smoke test (~5 min)

This was the one genuinely irreversible decision of the week. Everything recorded from
here shares this schema, or you have two incomparable corpora.

**DECIDED.** `observation.state` is 30-dim:
`[0:6] pos · [6:12] load · [12:18] current · [18:24] vel · [24:30] volt`.

Voltage is in as a **stall-severity indicator, not a collision trigger**. Checked
against the Week 1 pairs: it tracked only the hard stall (corr +0.99 with current,
−0.6V), while the light and moderate contacts showed no distinguishable sag and their
largest excursions landed 3.5s and 7.2s from the actual contact. Clean runs swing
0.3–0.4V unprompted. So it is corroboration for current, never a trigger on its own.

`Status` (65) and `Present_Temperature` (63) are **out**. Both are fetched by the block
read and discarded. Two consequences to be aware of:

- Taxonomy class `H1` (servo overload shutdown) now has to be identified by hand rather
  than read off the alarm byte. It is excluded from analysis anyway (§5.5), so a
  session-level note in the lab log is sufficient.
- Temperature drift can still be tracked, just not per-frame — take a reading at the
  start and end of each session with `diagnose.py` and record it in the spec sheet.
  That covers the §5.3 drift question without a schema column.

Record the layout in the difficulty spec sheet now and do not change it again.

## 2. Verify the record loop holds 30 fps — **DONE, passed** (~20 min)

The tight spot was two bus transactions per tick plus camera reads. Measured at
**29.95 Hz** with two cameras; no reduction needed. Re-run this if you change the
camera count, resolution, or add a third camera.

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

**How to actually verify the rate.** The dataset's `timestamp` column is *synthetic* —
`frame_index / fps` — so it always looks like a perfect 30 Hz and proves nothing. The
record loop is wall-clock driven (`lerobot_record.py:282,358`), so the episode occupies
a real `episode_time_s`, and the **frame count** is the measurement:

    actual Hz = total_frames / episode_time_s

A 20s episode yielding 599 frames is 29.95 Hz — kept up. The same episode at 20 Hz
would yield ~400 frames. Also watch the console for
`Record loop is running slower (X Hz) than the target FPS`, which catches individual
slow frames that a healthy average would hide. If either says no, drop to one camera or
20 fps and note which in the spec sheet.

Then confirm the schema landed:

```bash
python -c "
import json, glob
d = sorted(glob.glob('/home/ben/.cache/huggingface/lerobot/ben/telemetry-smoketest_*'))[-1]
i = json.load(open(d + '/meta/info.json'))
print('robot_type:', i['robot_type'], '| frames:', i['total_frames'])
print('actual Hz :', i['total_frames'] / 20)          # / your episode_time_s
for k in ('observation.state', 'action'):
    print(k, i['features'][k]['shape'])"
```

**Result (25 Jul):** 599 frames / 20s = **29.95 Hz** with two cameras and the 30-dim
telemetry schema. Passed — no need to reduce cameras or fps.

## 3. Camera rigidity and stable identity (~30 min)

Deferred from Week 1 and now due. §5.1 calls viewpoint drift the most common silent
confound in this literature.

Mount both cameras so they physically cannot move — tape or clamp the mounts, don't
rely on friction. Photograph the reference view from each. Add a session-start check
against those photographs to the lab log, and log the check every time.

**Address cameras by `/dev/v4l/by-id/` path, never by integer index.** Indices are
assigned at plug time and shift between sessions — on this machine `/dev/video4` and
`video5` appeared hours after `video0`–`video3`. If wrist and overhead silently swap
indices mid-corpus you get viewpoint drift that is undetectable after the fact. The
by-id names are tied to device serials:

| device | by-id (capture node) |
|---|---|
| Logitech C920 | `usb-046d_HD_Pro_Webcam_C920_ECE7923F-video-index0` |
| HHWei USB Camera | `usb-HHWei_Technology_Co.__Ltd._USB_Camera_HHW001-video-index0` |
| icSpring | `usb-icSpring_icspring_camera_202404160005-video-index0` |

Only `-video-index0` nodes are capture devices; `-video-index1` is metadata and will
fail to open. Record which serial is `wrist` and which is `top` in the spec sheet
alongside the reference photographs — that pairing is what makes the session-start
check meaningful.

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
