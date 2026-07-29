# Robot data collection runbook

This is the authoritative, ordered list of Python commands the operator runs to produce
new physical evidence. Run from the repository root after `conda activate lerobot`.
Follower is `/dev/ttyACM0`; leader is `/dev/ttyACM1`.

Commands marked **MOTION** enable or retain torque and may move the arm. Keep the workspace
clear and an operator at the servo power switch. Never run recovery unattended until the
physical validation ladder and ten-reset exit test pass.

## 1. Start every robot session: read health

```bash
python research/telemetry/diagnose.py --port /dev/ttyACM0
```

**Why:** captures alarm bytes, temperatures, voltage, current, position, and protection
settings before motion. This is read-only and explains thermal/session shifts later. Save
the terminal output in the dated session report. Stop if any servo is alarmed or unusually
hot; do not loosen a recovery limit to get past a hardware fault.

## 2. Identify and freeze camera views

```bash
python research/telemetry/camera_reference.py list
```

**Why:** identifies which `/dev/videoN` nodes can capture images. This reads cameras only.
Physically identify the intended wrist and overhead devices.

After mounting and taping both cameras, replace the example device paths if necessary:

```bash
python research/telemetry/camera_reference.py capture \
    --camera wrist=/dev/video0 --camera overhead=/dev/video2 \
    --session rig_v1 --out research/telemetry/camera_reference
```

**Why:** saves the immutable reference views and a manifest containing device identity,
image size, and hashes. No robot motion occurs. Inspect both images before accepting them.

At every later session start:

```bash
python research/telemetry/camera_reference.py check \
    --camera wrist=/dev/video0 --camera overhead=/dev/video2 \
    --session YYYY-MM-DD_a --verdict pass \
    --out research/telemetry/camera_checks
```

**Why:** creates evidence that today’s views still match the taped reference. Use
`--verdict fail` if either view drifted, correct the mount, and repeat with a new session
identifier. Never collect corpus episodes after a failed check.

## 3. Record a return-home demonstration

```bash
python research/telemetry/log_teleop_telemetry.py \
    --follower-port /dev/ttyACM0 --leader-port /dev/ttyACM1 \
    --out research/telemetry/runs/reset_home_v2.csv
```

**MOTION. Why:** records an operator-demonstrated, collision-free path from a validated
clearance pose to the episode home pose, with raw measured positions and telemetry. This is
the final segment of recovery, not proof that arbitrary poses can reach its first frame.
Stop with Ctrl-C only after reaching and holding home. Review/plot the CSV before use.

## 4. Validate each arbitrary-pose recovery region

First copy `recovery_waypoints.example.json` to a session-specific configuration and fill
its limits/routes from supervised measurements. It is intentionally non-executable as
shipped. Keep every new route `validated: false` for its first trials.

The recovery core refuses an unvalidated route by default. After reviewing a route’s
waypoints with the arm unpowered, authorize only that named route for an attended trial,
use conservative step/current/error limits, and run:

```bash
python research/telemetry/validate_recovery.py \
    --follower-port /dev/ttyACM0 --leader-port /dev/ttyACM1 \
    --home research/telemetry/runs/reset_home_v2.csv \
    --waypoints research/telemetry/recovery_waypoints.local.json \
    --supervised-trial-route center \
    --history-seconds 2 \
    --out research/telemetry/runs/recovery_region_center_01.csv
```

**MOTION. Why:** teleoperation fills a two-second ring buffer with commands actually
accepted by the follower. Press `r` at the representative pose to halt, reverse away from
the task area, traverse the selected waypoint route, return home, and open the gripper.
Every recovery frame logs command, measured position, current, load, velocity, and voltage.
Ctrl-C halts at measured position.

Run distinct output names for every region/object/fault pose. Inspect clearances visually
and analyze peak current, tracking error, aborts, and duration. A route is validated only
after the full ladder in `RECOVERY_PROTOCOL.md`; a successful single run is insufficient.

After each trial, audit its recorded values against the exact waypoint configuration:

```bash
python research/telemetry/audit_recovery.py \
    --waypoints research/telemetry/recovery_waypoints.local.json \
    --route center \
    research/telemetry/runs/recovery_region_center_01.csv
```

**Why:** performs no robot I/O. It verifies the complete phase sequence, contiguous frame
indices, monotonic timestamps, joint limits, configured current and following-error
limits, bounded command steps, phase timeouts, and final gripper-open goal, then reports
peak values. It deliberately fails while the route remains `validated: false`. After the
full physical ladder passes and the reviewed configuration is marked validated, rerun it
over every supporting log. This numeric PASS does not measure Cartesian table/object
clearance; record those observations separately as required by `RECOVERY_PROTOCOL.md`.

The old lift-first experiment is deliberately omitted. Do not use `legacy-reset` for data
collection or the exit test.

## 5. Record the first synchronized two-camera test episode

Replace `${HF_USER}`, camera paths, task text, and IDs with the frozen session values:

```bash
python research/telemetry/record_with_telemetry.py \
    --robot.type=so101_follower_telemetry \
    --robot.port=/dev/ttyACM0 --robot.id=follower \
    --robot.cameras='{wrist: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}, overhead: {type: opencv, index_or_path: /dev/video2, width: 640, height: 480, fps: 30}}' \
    --teleop.type=so101_leader --teleop.port=/dev/ttyACM1 --teleop.id=leader \
    --dataset.repo_id=${HF_USER}/telemetry-alignment-smoke \
    --dataset.single_task='Alignment and schema smoke test' \
    --dataset.num_episodes=1 --dataset.fps=30
```

**MOTION. Why:** records one disposable teleoperated episode using the frozen 30-dimensional
state, six-dimensional actions, both videos, and a real-time sidecar under the local dataset
root’s `meta/alignment/`. Do not begin the corpus with this command; it is an infrastructure
smoke test.

Find the emitted sidecar in the dataset root and audit it:

```bash
python research/telemetry/audit_alignment.py \
    /path/to/dataset/root/meta/alignment/episode_000000.jsonl --fps 30
```

**Why:** performs no robot I/O. It verifies every frame has auditable timestamps and that
both camera captures align with telemetry within one 33.3 ms control step. Any failure
blocks corpus recording; preserve the failing sidecar rather than deleting evidence.

## 6. Signature or diagnostic telemetry runs

```bash
python research/telemetry/log_teleop_telemetry.py \
    --follower-port /dev/ttyACM0 --leader-port /dev/ttyACM1 \
    --out research/telemetry/runs/SESSION_DESCRIPTION.csv
```

**MOTION. Why:** produces a raw 30 Hz diagnostic CSV for a deliberately isolated event.
Use the documented single-key markers while the event occurs. This is for targeted
hardware questions, not for accumulating D0r calibration: D0r must wait for independent
autonomous corpus rollouts.

To verify the optimized telemetry read against ordinary register reads before a session:

```bash
python research/telemetry/log_teleop_telemetry.py \
    --follower-port /dev/ttyACM0 --leader-port /dev/ttyACM1 \
    --out research/telemetry/runs/block_verify.csv --verify
```

**Why:** connects to the follower and compares read paths without teleoperating. Run after
bus/register changes, not before every ordinary session.

## 7. End every motion session: capture post-health

```bash
python research/telemetry/diagnose.py --port /dev/ttyACM0
```

**Why:** records post-run alarms and temperature so recovery/corpus logs have a thermal
bracket. Preserve the output even when the session failed.

## Evidence to return after a supervised session

- pre/post diagnostic output and session identifier;
- camera reference/check manifests and both images;
- every recovery CSV, its exact waypoint-config revision, and notes on manual intervention
  or minimum visible clearance;
- the alignment smoke-test dataset path, sidecar, and audit output;
- exact command lines, unexpected messages, power cycles, mount/object changes, and aborts.

After the redesigned ten-reset test exists, create the reviewed record described in
`RECOVERY_READINESS_GATE.md`, then run its `audit_recovery_readiness.py` command. This is a
**NO MOTION** command: it reruns every referenced recovery/checkpoint check and issues an
exclusive enablement token bound to the exact hashes. No token can be issued from the old
five-repeat lift-first soak, missing diagnostics, or operator assertions without ten
distinct passing redesigned logs.

## 8. T1 pilot demonstrations (only after all pilot gates pass)

Do not run this section until procurement, camera/alignment, recovery-route validation,
the 10-reset test, and the readiness token are recorded as passed. Replace every placeholder with the frozen
values in `PILOT_DIFFICULTY_SPEC.local.md`.

```bash
python research/telemetry/record_with_telemetry.py \
    --robot.type=so101_follower_telemetry \
    --robot.port=/dev/ttyACM0 --robot.id=follower \
    --robot.cameras='{wrist: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}, overhead: {type: opencv, index_or_path: /dev/video2, width: 640, height: 480, fps: 30}}' \
    --teleop.type=so101_leader --teleop.port=/dev/ttyACM1 --teleop.id=leader \
    --dataset.repo_id=${HF_USER}/t1-pilot-demos-v1 \
    --dataset.single_task='Pick up the cube and place it in the bowl' \
    --dataset.num_episodes=30 --dataset.fps=30
```

**MOTION. Why:** creates the 30 clean, randomized demonstrations used to train the pilot
policy. It deliberately records the full frozen telemetry state and both camera streams;
the policy-only truncation happens during training. Follow `PILOT_PROTOCOL.md` for episode
acceptance, alignment audits, visual inspection, and backup before training.

Run the complete dataset audit before visualization, backup, or training:

```bash
python research/telemetry/audit_corpus.py \
    /path/to/local/t1-pilot-demos-v1 --expected-episodes 30
```

**Why:** performs no robot I/O. Unlike an `info.json` spot check, it reads every stored
Parquet state and action, rejects wrong widths/non-finite values, checks 30/6-dimensional
normalization statistics, requires both declared camera streams and nonempty MP4 files,
and audits every episode sidecar against its camera timestamps. A failure quarantines the
dataset until its cause is understood; do not edit metadata to make the audit pass.

## 9. Train the positions-only pilot policy (no robot connection)

```bash
python research/telemetry/train_positions_only.py \
    --policy.path=lerobot/smolvla_base \
    --dataset.repo_id=${HF_USER}/t1-pilot-demos-v1 \
    --policy.device=cuda \
    --rename_map='{"observation.images.wrist": "observation.images.camera1", "observation.images.overhead": "observation.images.camera2"}' \
    --output_dir=outputs/train/t1-pilot-arm1-v1 \
    --job_name=t1-pilot-arm1-v1 \
    --batch_size=4 --steps=20000 \
    --policy.scheduler_decay_steps=20000 \
    --save_freq=5000 --wandb.enable=true
```

**Why:** fine-tunes Arm 1 while provably hiding load, current, velocity, and voltage from
the policy. The wrapper supplies six-value feature metadata and statistics and serializes
the truncation step before normalization. The explicit rename map connects the frozen
wrist/overhead dataset keys to SmolVLA's pretrained camera inputs; the wrapper refuses to
train this two-camera corpus if that mapping is absent. It does not modify the
30-dimensional dataset.
Record the 20k-step wall-clock time and verify the checkpoint as specified in
`PILOT_PROTOCOL.md`.

Before connecting the robot for evaluation, run:

```bash
python research/telemetry/audit_positions_checkpoint.py \
    outputs/train/t1-pilot-arm1-v1/checkpoints/last/pretrained_model
```

**Why:** performs no robot I/O and does not load the large model. It requires a SmolVLA
checkpoint with six-value policy state/action features, explicit `truncate_state keep=6`
as the first step, the frozen camera rename map, six-value normalizer features and stored
state statistics, `n_action_steps == chunk_size`, and nonempty model weights. Any failure
blocks autonomous evaluation; do not hand-edit checkpoint JSON to waive it.

## 10. Autonomous T1 pilot evaluations

Run the session health and camera checks first, then:

```bash
python research/telemetry/rollout_with_telemetry.py \
    --strategy.type=episodic \
    --robot.type=so101_follower_telemetry \
    --robot.port=/dev/ttyACM0 --robot.id=follower \
    --robot.cameras='{wrist: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}, overhead: {type: opencv, index_or_path: /dev/video2, width: 640, height: 480, fps: 30}}' \
    --teleop.type=so101_leader --teleop.port=/dev/ttyACM1 --teleop.id=leader \
    --dataset.repo_id=${HF_USER}/t1-pilot-arm1-eval-v1 \
    --dataset.single_task='Pick up the cube and place it in the bowl' \
    --dataset.num_episodes=30 --dataset.fps=30 \
    --policy.path=outputs/train/t1-pilot-arm1-v1/checkpoints/last/pretrained_model
```

**MOTION. Why:** executes the fixed Arm 1 checkpoint and records the roughly 30 rollouts
used only to estimate and lock task difficulty. The checkpoint preprocessor truncates the
live 30-value state to six positions. The episodic strategy runs the policy during each
episode and uses the leader only during its between-episode reset phase. An operator must
supervise recovery and use the already accepted reset path; this command does not yet
supply detector-triggered recovery. End with the post-session diagnostic command in §7
and label from video without consulting detector scores.

Audit the evaluation dataset with the same command and `--expected-episodes 30` before
labeling it. This confirms storage and alignment integrity, not task success or video
content; visual review and blinded labels remain mandatory.

## 11. Recovery-enabled rollout evidence (only after the recovery gates pass)

Do not add detector-triggered recovery to the §10 command yet. Once every prerequisite in
`CLOSED_LOOP_RECOVERY_INTEGRATION.md` passes, create the immutable run manifest before
robot connection using the non-motion command in `RECOVERY_EVIDENCE_SCHEMA.md`. The live
strategy must then write supervisor events, one exclusive CSV per physical attempt, and
one terminal outcome per episode automatically. It must call the runtime verifier in
`RECOVERY_READINESS_GATE.md` before constructing recovery; a token copied beside changed
files is not valid authorization.

After disconnecting the robot, run:

```bash
python research/telemetry/audit_recovery_evidence.py \
  --manifest /path/to/run/meta/recovery/run_manifest.json \
  --events /path/to/run/meta/recovery/recovery_events.jsonl \
  --outcomes /path/to/run/meta/recovery/episode_outcomes.jsonl \
  /path/to/run/meta/recovery/recovery_*.csv
```

**NO MOTION. Why:** proves every detector trigger and physical attempt joins to the frozen
policy/chunk configuration and exactly one final episode outcome. This relational PASS is
required in addition to `audit_recovery.py`; it does not establish geometric clearance.
