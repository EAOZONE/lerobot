# Recovery enablement readiness gate

**Status:** software implemented; no token can be issued from the current repository
because physical route and ten-reset evidence do not exist.

This gate prevents “we ran the tests” from becoming permission to enable autonomous
recovery. It combines software checks with reviewed physical evidence and issues an
immutable token bound to the exact checkpoint and configurations. The future live strategy
must verify that token immediately before enabling recovery.

## Physical validation record

Create one JSON object conforming to `recovery_validation.schema.json` only after the
attended ladder in `RECOVERY_PROTOCOL.md`. Paths are resolved relative to the record.

For every configured route, record:

- operator-observed clearance with no objects;
- clearance with static objects at allowed boundaries;
- representative slip, collision, awkward-release, and payload-pose clearance;
- one or more supervised per-frame logs.

The reset exit section references ten **distinct redesigned-recovery logs**, not the old
`reset_soak.csv`. It also records zero manual interventions, alarm-free pre/post diagnostic
files, start/end shoulder-lift temperature, visual clearance, and raw peak shoulder current.
The auditor reruns `audit_recovery.py` on every referenced log and checks that the declared
peak equals the maximum in those ten logs.

Setting a boolean to true is a signed operator judgment, not an inferred software result.
Name the operator and preserve the source photographs/video. The auditor can establish
completeness and numeric limits; it cannot see Cartesian clearance.

## Issue the token

After creating the immutable run manifest from `RECOVERY_EVIDENCE_SCHEMA.md`, run:

```bash
python research/telemetry/audit_recovery_readiness.py \
  --validation-record /path/to/run/meta/recovery/physical_validation.json \
  --waypoints /path/to/recovery_waypoints.validated.json \
  --checkpoint /path/to/checkpoint/pretrained_model \
  --detector-config /path/to/frozen_detector_config.json \
  --run-manifest /path/to/run/meta/recovery/run_manifest.json \
  --out-token /path/to/run/meta/recovery/enablement_token.json
```

**NO MOTION. Why:** reruns checkpoint and every recovery-log audit, checks all physical
attestations, binds waypoint/detector/validation hashes and checkpoint revision, requires
full-chunk execution, and records the exact inference engine allowed. RTC tokens require
`recovery_rtc_guard.RecoverySafeRTCInferenceEngine`. The output uses exclusive-create
semantics; changing any input requires a new run ID and new token, never overwriting one.

The command fails closed on missing or duplicate routes, `validated:false`, fewer than ten
distinct reset logs, any intervention/alarm/visual failure, absent diagnostics, stale
hashes, a failed physical log, or an ineligible checkpoint. A token is necessary but not
sufficient: the live hardware dry run and joined-evidence PASS remain separate gates.

## Runtime verification

Possessing a token is insufficient because a configuration or checkpoint path may change
after issuance. Before constructing any recovery supervisor or action gate, the future live
strategy must call `verify_recovery_enablement(...)` from `recovery_enablement.py`, passing
the token, physical record, waypoint and detector files, checkpoint, run manifest, and the
actual inference-engine instance.

The verifier recomputes every hash, rechecks run/revision/chunk/mode/checkpoint identity,
and compares the engine's fully qualified class. It raises `RecoveryEnablementError` on any
drift. For RTC, both token and runtime must name
`recovery_rtc_guard.RecoverySafeRTCInferenceEngine`; an upstream RTC instance is rejected.
Do not catch this exception to select a less-safe recovery path. Continue only in the
ordinary no-autonomous-recovery strategy or stop the run.
