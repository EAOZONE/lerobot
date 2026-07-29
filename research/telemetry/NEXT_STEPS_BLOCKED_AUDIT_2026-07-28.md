# NEXT_STEPS completion and blocker audit — 28 July 2026

This is a current-state evidence audit, not a claim that the research project is complete.
It records why further autonomous desk work cannot close the remaining gates without
external action or new physical data.

## Gate-by-gate verdict

| gate | evidence required for completion | authoritative current evidence | verdict |
|---|---|---|---|
| 1. Materials | Completed vendor/part/order/ETA fields and later receipt confirmation for two servos and T1–T3 objects | `PROCUREMENT_CHECKLIST.md` says **not ordered** and every vendor/order/ETA/received cell is blank | **Blocked externally: purchase not placed** |
| 2. Recovery routes | Session waypoint config with every route `validated:true`, reviewed clearance record, passing supervised CSV audits | Only `recovery_waypoints.example.json` exists; it is deliberately `validated:false`. No physical-validation record, qualifying route log, readiness token, or route PASS exists | **Blocked on attended robot validation** |
| 3. Camera rig | Mounted/taped cameras, reference photographs, and a recorded passing alignment-check manifest | Protocol and capture utility exist; filesystem contains no session reference image/manifest evidence | **Blocked on physical rigging** |
| 4. Alignment smoke | One disposable two-camera episode and strict `audit_alignment.py` PASS | Offline auditor tests pass; no hardware episode sidecar exists | **Blocked on two-camera robot recording** |
| 5. Inference latency | Measured sync/chunk populations for 6- and 30-state paths | Recorded in `WEEK2_REPORT.md`; implementation and benchmark complete | **Complete** |
| 6. Reset exit | Ten distinct redesigned-recovery logs, zero interventions, pre/post health, clearance review, temperatures/current, readiness PASS/token | Only `reset_soak.csv` exists and is the superseded five-repeat lift-first experiment. No redesigned ten-log record or token exists | **Blocked on gate 2 and attended ten-reset run** |
| 7. Second annotator | Named person, contact date, acceptance, availability, and conflicts recorded | Brief and outreach text exist; no accepted commitment is recorded | **Blocked on human coordination** |
| 8. T1 pilot | Materials received, cameras/alignment passed, reset gate passed, 30 demonstrations, checkpoint, ~30 evaluations | No pilot dataset or checkpoint exists; `outputs/train` contains no qualifying artifact | **Blocked by gates 1, 3, and 6** |

## Later scheduled work

- D0r cannot proceed until corpus training/calibration data and sealed trajectory C exist.
- The 20k-step wall-clock measurement requires the first pilot dataset and checkpoint.
- Full demonstrations, learned detectors, closed-loop hardware trials, and joined live
  recovery evidence all depend on the pilot and physical gates above.
- The workshop tracker is current as of 28 July; its next action is the scheduled weekly
  page check, not speculative deadline invention.

## Desk-side work proven complete

The current software suite covers the frozen telemetry schema, synchronized timing
sidecars, corpus/checkpoint audits, positions-only training bridge, causal detectors,
arbitrary-pose recovery executor and route audit, supervisor/lifecycle/action boundary,
guarded RTC generation semantics, retry and policy-tick accounting, joined evidence,
readiness-token issuance, and runtime token verification.

At this audit point:

- 84 focused telemetry tests pass;
- Ruff passes the touched Python surface;
- `git diff --check` is clean;
- frozen hashes match their ledgers;
- `src/lerobot/` has no research changes; and
- no robot command, procurement action, annotator outreach, upload, or token issuance was
  performed by the agent.

These facts prove desk readiness, not physical safety or experimental completion.

## Minimum bundle needed to resume

Return any one of these to unblock meaningful work:

1. A completed purchase record or received-material inventory.
2. Pre/post `diagnose.py` output plus supervised recovery logs and the exact session
   waypoint configuration from the ladder in `ROBOT_DATA_RUNBOOK.md`.
3. Camera reference/check artifacts and the disposable two-camera dataset sidecar.
4. A named second annotator's acceptance and availability window.
5. Once gates 1–6 pass, the local pilot dataset path or checkpoint path.

For robot work, follow `ROBOT_DATA_RUNBOOK.md` exactly. It is the requested inventory of
every Python command the operator must run and explains why each command exists. Do not
substitute the old reset soak, manually set a readiness boolean without observing it, or
attach autonomous recovery without a freshly verified enablement token.
