# T1 pilot protocol

Status: prepared offline 28 July 2026; no pilot data exists yet. This protocol does not
open the pilot gate. Materials, camera reference/alignment, physically accepted recovery
routes, and the 10-reset exit test must pass first.

## Fixed purpose

The pilot estimates the autonomous failure rate of Arm 1 and locks T1 difficulty before
the main corpus. Record 30 clean demonstrations with the frozen 30-dimensional state,
train a SmolVLA policy that receives only the first six joint positions, then collect
about 30 autonomous evaluations. The target is 40–60% failures.

The demonstration files are never rewritten to six dimensions. Full telemetry remains
available to detectors. `train_positions_only.py` changes only the in-memory policy view:
it narrows state feature metadata and normalization statistics to six entries and inserts
`TruncateStateStep(keep=6)` before normalization. The saved checkpoint preprocessor carries
that step into evaluation through `rollout_with_telemetry.py`. The recording wrapper is
for teleoperated demonstrations only and must not be used for policy deployment.

## Freeze before episode 1

Create `PILOT_DIFFICULTY_SPEC.local.md` and record:

- cube/bowl identifiers, dimensions, mass, surface, and reference photographs;
- camera reference manifest and exact wrist/overhead device paths;
- home pose, cube start-region coordinates, bowl coordinates, and randomization method;
- exact task string (unchanged across demonstration, training, and evaluation);
- demonstration repo ID, policy output path, seed, checkpoint, `n_action_steps`, and code
  commit/worktree description;
- recovery waypoint configuration and evidence that its exit gate passed.

Do not use Arm 3 results to adjust any of these choices.

## Demonstrations and acceptance

Record 30 successful, clean demonstrations. Randomize the cube within the declared bounds
before every episode. Reject an episode only for a contemporaneously logged reason such as
camera corruption, operator collision, object outside the frozen region, or hardware
alarm. Never reject it because telemetry looks inconvenient.

After recording, require:

- exactly 30 accepted episodes and a written list of rejected attempts;
- 30-value `observation.state`, 6-value action, two videos, and one alignment sidecar per
  episode;
- strict alignment audit PASS for every sidecar;
- visual review for blur, missing views, inconsistent task text, and object bounds;
- an off-machine backup verified according to `DATA_BACKUP_PROTOCOL.md`.

Run `audit_corpus.py --expected-episodes 30` before training. This turns the first four
structural checks into a fail-closed executable gate. It intentionally does not judge
blur, object bounds, task success, or semantic video correctness; those remain visual
checks rather than pretending file existence proves useful images.

## Train and verify Arm 1

Use the exact command in `ROBOT_DATA_RUNBOOK.md`. Preserve the complete output directory.
Before autonomous motion, inspect
the checkpoint with `audit_positions_checkpoint.py`. It requires `truncate_state` with an
explicit `keep: 6` as the first step, six-value policy and normalizer state, six-value
stored statistics, the frozen wrist/overhead camera rename map, and full-chunk execution.
A failing checkpoint is ineligible. The camera rename map is load-bearing: the corpus keys
do not match SmolVLA's pretrained camera feature names without it.

Time the first 20k-step run and record GPU, peak VRAM, elapsed time, final loss, and the
checkpoint selected for evaluation. This closes the outstanding wall-clock estimate; it
does not select difficulty by training loss.

## Autonomous evaluations and stopping rule

Run approximately 30 evaluations using the frozen checkpoint and task setup. Label video
under `LABELING_GUIDE.md`; detector output must not decide ground truth. Report the failure
fraction and a Wilson 95% interval.

- If 40–60% fail, lock difficulty immediately.
- Otherwise change one declared physical lever (prefer randomization radius or object
  geometry), document it as a new iteration, and repeat training/evaluation only as needed.
- Stop after at most three iterations. Never tune Arm 1 after observing Arm 3.

Capture trajectory C during this pilot but keep it sealed under
`TRAJECTORY_C_PROTOCOL.md`. Record start/end temperatures and grasp duration per episode.
