import json

import pytest
from audit_recovery_evidence import audit_recovery_evidence
from feetech_block import BLOCK_FIELDS
from probe_bus import SO101_MOTORS
from recovery import RecoveryResult
from recovery_evidence import (
    EpisodeOutcome,
    EpisodeOutcomeLogger,
    RecoveryAttemptContext,
    RecoveryAttemptCSVLogger,
    write_run_manifest,
)
from recovery_supervisor import DetectorTrigger, RecoveryEventLogger, RecoverySupervisor


def pose(value):
    return dict.fromkeys(SO101_MOTORS, value)


def manifest(run_id="pilot-20260728-a"):
    return {
        "run_id": run_id,
        "policy_checkpoint": "outputs/train/checkpoint",
        "policy_revision": "a" * 64,
        "policy_type": "smolvla",
        "inference_mode": "rtc",
        "chunk_size": 50,
        "n_action_steps": 50,
        "fps": 30,
        "detector_config_sha256": "b" * 64,
        "waypoint_config_sha256": "c" * 64,
    }


def make_valid_bundle(tmp_path):
    run_id = "pilot-20260728-a"
    manifest_path = tmp_path / "run_manifest.json"
    events_path = tmp_path / "recovery_events.jsonl"
    outcomes_path = tmp_path / "episode_outcomes.jsonl"
    recovery_path = tmp_path / "recovery_e000000_r01.csv"
    write_run_manifest(manifest_path, manifest(run_id))

    def execute(history, flush):
        flush()
        return RecoveryResult("center", reverse_frames=1, completed_frames=2)

    supervisor = RecoverySupervisor(
        run_id=run_id,
        flush_policy_queue=lambda: None,
        reinvoke_policy=lambda: None,
        execute=execute,
        history_frames=2,
        on_event=RecoveryEventLogger(events_path),
    )
    supervisor.start_episode(0)
    supervisor.record_sent_command(pose(100))
    supervisor.recover(DetectorTrigger("d0", 1.2, 1.0, 4, "slip"))

    telemetry = {field: pose(0) for field in BLOCK_FIELDS}
    clock = iter((1_000_000_000, 1_010_000_000, 1_020_000_000))
    context = RecoveryAttemptContext(run_id, episode_index=0, attempt=1, route="center")
    with RecoveryAttemptCSVLogger(recovery_path, context, clock_ns=lambda: next(clock)) as logger:
        logger("reverse", 0, pose(100), telemetry)
        logger("home", 1, pose(101), telemetry)

    EpisodeOutcomeLogger(outcomes_path)(
        EpisodeOutcome(run_id, 0, "success", recovery_attempts=1, policy_ticks=900)
    )
    return manifest_path, events_path, outcomes_path, recovery_path


def test_joined_recovery_evidence_passes_with_stable_identifiers(tmp_path):
    bundle = make_valid_bundle(tmp_path)
    summary, errors = audit_recovery_evidence(*bundle[:3], [bundle[3]])
    assert errors == []
    assert summary.episodes == 1
    assert summary.attempts == 1
    assert summary.recovery_frames == 2
    events = [json.loads(line) for line in bundle[1].read_text().splitlines()]
    started = next(row for row in events if row["event"] == "recovery_started")
    trigger = next(row for row in events if row["event"] == "trigger")
    assert started["run_id"] == "pilot-20260728-a"
    assert started["episode_id"] == "pilot-20260728-a:e000000"
    assert started["attempt_id"] == "pilot-20260728-a:e000000:r01"
    assert trigger["attempt_id"] == started["attempt_id"]


def test_auditor_rejects_missing_recovery_log_and_attempt_count_mismatch(tmp_path):
    manifest_path, events_path, outcomes_path, recovery_path = make_valid_bundle(tmp_path)
    row = json.loads(outcomes_path.read_text())
    row["recovery_attempts"] = 0
    outcomes_path.write_text(json.dumps(row) + "\n")
    _, errors = audit_recovery_evidence(manifest_path, events_path, outcomes_path, [])
    assert any("no per-frame recovery log" in error for error in errors)
    assert any("recovery_attempts does not match" in error for error in errors)
    assert recovery_path.exists()


def test_auditor_rejects_conflicting_completed_metadata(tmp_path):
    manifest_path, events_path, outcomes_path, recovery_path = make_valid_bundle(tmp_path)
    rows = [json.loads(line) for line in events_path.read_text().splitlines()]
    for row in rows:
        if row["event"] == "recovery_completed":
            row["completed_frames"] = 99
            row["route"] = "wrong"
    events_path.write_text("".join(json.dumps(row) + "\n" for row in rows))
    _, errors = audit_recovery_evidence(
        manifest_path, events_path, outcomes_path, [recovery_path]
    )
    assert any("completed_frames does not match" in error for error in errors)
    assert any("completed route does not match" in error for error in errors)


def test_manifest_is_immutable_and_unbound_run_ids_are_rejected(tmp_path):
    path = tmp_path / "manifest.json"
    write_run_manifest(path, manifest())
    with pytest.raises(FileExistsError):
        write_run_manifest(path, manifest())
    with pytest.raises(ValueError, match="bound"):
        write_run_manifest(tmp_path / "bad.json", manifest("unbound"))


def test_recovery_csv_route_is_bound_from_executor_before_first_frame(tmp_path):
    telemetry = {field: pose(0) for field in BLOCK_FIELDS}
    clock = iter((1_000_000_000, 1_010_000_000))
    context = RecoveryAttemptContext("pilot-a", episode_index=2, attempt=1)
    path = tmp_path / "attempt.csv"
    with RecoveryAttemptCSVLogger(path, context, clock_ns=lambda: next(clock)) as logger:
        with pytest.raises(RuntimeError, match="route must be bound"):
            logger("reverse", 0, pose(100), telemetry)
        logger.bind_route("left")
        logger("reverse", 0, pose(100), telemetry)
        with pytest.raises(ValueError, match="already bound"):
            logger.bind_route("right")
    assert ",left," in path.read_text()
