# Recovery rollout evidence schema

**Version:** 1 for manifests/outcomes/recovery frames; supervisor events version 2  
**Status:** frozen before live recovery data exists, 28 July 2026

This contract makes every recovery decision traceable from policy identity through physical
motion to the final episode result. It does not authorize recovery motion. Live collection
remains gated by validated routes and the ten-reset protocol.

## Stable identity

Choose one `run_id` before connecting the robot. It must contain only letters, digits,
period, underscore, or hyphen and must not be `unbound`. Never reuse it for a different
checkpoint, detector configuration, waypoint configuration, or inference mode.

- episode: `<run_id>:e<six-digit episode index>`
- physical recovery attempt: `<episode_id>:r<two-digit one-based attempt>`

The supervisor emits these values directly. Do not reconstruct identity from timestamps or
filenames during analysis.

## Immutable run manifest

`run_manifest.json` is created with exclusive-create semantics and cannot be overwritten by
the writer. It freezes:

- policy checkpoint and immutable revision;
- policy type and sync/RTC mode;
- `chunk_size`, `n_action_steps`, and FPS;
- SHA-256 of detector and waypoint configurations.

For this experiment `n_action_steps` must equal `chunk_size`. Example, before robot startup:

```bash
python research/telemetry/recovery_evidence.py manifest \
  --out /path/to/run/meta/recovery/run_manifest.json \
  --run-id t1-arm1-seed0-20260728-a \
  --policy-checkpoint outputs/train/t1-arm1/checkpoints/last/pretrained_model \
  --policy-revision REPLACE_WITH_IMMUTABLE_REVISION \
  --policy-type smolvla --inference-mode rtc \
  --chunk-size 50 --n-action-steps 50 --fps 30 \
  --detector-config-sha256 REPLACE_WITH_64_HEX \
  --waypoint-config-sha256 REPLACE_WITH_64_HEX
```

This command does not move or connect to the robot. It fails if the output already exists.

## Supervisor events and recovery frames

`recovery_events.jsonl` is append-only and fsynced. Event schema 2 includes `run_id`,
canonical `episode_id`, and canonical `attempt_id`. An accepted trigger, recovery start,
completion/abort, and policy reinvocation share the same attempt ID.

Each physical attempt has one exclusive-create CSV. `RecoveryAttemptCSVLogger` adds the
same identifiers and route to every frame, plus monotonic time, phase, raw goal positions,
and the complete telemetry block. The executor binds its actually selected route through
`on_route=logger.bind_route` before passing frames through `on_frame=logger`; this avoids a
second, potentially divergent route decision in the rollout layer. Manual CSV editing is
not valid evidence.

## Episode outcomes

`episode_outcomes.jsonl` contains exactly one terminal row per started episode:

- `success`, `failure`, or `aborted`;
- number of physical recovery attempts;
- autonomous policy ticks, excluding recovery motion;
- optional factual detail.

The live strategy should write this automatically after dataset outcome handling. The CLI
exists for attended recovery validation and repair of a process that terminated after the
physical episode but before its terminal append:

```bash
python research/telemetry/recovery_evidence.py outcome \
  --out /path/to/run/meta/recovery/episode_outcomes.jsonl \
  --run-id t1-arm1-seed0-20260728-a --episode-index 0 \
  --outcome success --recovery-attempts 1 --policy-ticks 900
```

Never use the CLI to change an existing outcome; duplicate terminal rows fail audit.

## Join audit

After the run:

```bash
python research/telemetry/audit_recovery_evidence.py \
  --manifest /path/to/run/meta/recovery/run_manifest.json \
  --events /path/to/run/meta/recovery/recovery_events.jsonl \
  --outcomes /path/to/run/meta/recovery/episode_outcomes.jsonl \
  /path/to/run/meta/recovery/recovery_*.csv
```

The auditor rejects unbound or conflicting identity, missing/orphan episode outcomes,
missing/orphan recovery logs, missing trigger/start/terminal transitions, duplicate
attempts, noncontiguous recovery frames, policy/chunk configuration defects, and mismatch
between completed route/frame metadata and the physical CSV. A PASS proves relational
completeness, not that the route was geometrically safe; `audit_recovery.py` and physical
clearance review remain separate gates.
