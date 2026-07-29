import hashlib
import json
from types import SimpleNamespace

import audit_recovery_readiness as readiness
import pytest

MOTORS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def pose(value):
    return dict.fromkeys(MOTORS, value)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def make_bundle(tmp_path, *, validated=True):
    checkpoint = tmp_path / "checkpoint"
    checkpoint.mkdir()
    detector = tmp_path / "detector.json"
    detector.write_text('{"detector":"d0"}\n')
    waypoint = tmp_path / "waypoints.json"
    waypoint.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "limits": {
                    "joint_min": pose(0),
                    "joint_max": pose(4095),
                    "max_current": pose(100),
                    "max_following_error": pose(100),
                    "max_step_ticks": 10,
                    "phase_timeout_s": 5,
                    "reverse_frames": 2,
                    "fault_guard_frames": 1,
                    "gripper_open": 100,
                },
                "routes": [
                    {
                        "name": "center",
                        "validated": validated,
                        "region_min": pose(0),
                        "region_max": pose(4095),
                        "waypoints": [pose(1000)],
                    }
                ],
            }
        )
    )
    route_log = tmp_path / "route.csv"
    route_log.write_text("reviewed route log\n")
    reset_logs = []
    for index in range(10):
        path = tmp_path / f"reset_{index:02d}.csv"
        path.write_text(f"reset {index}\n")
        reset_logs.append({"path": path.name, "route": "center"})
    pre = tmp_path / "pre_health.txt"
    post = tmp_path / "post_health.txt"
    pre.write_text("temperature 31 C, alarms 0\n")
    post.write_text("temperature 34 C, alarms 0\n")
    record = tmp_path / "validation.json"
    record.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "operator": "operator-a",
                "reviewed_at": "2026-07-28T12:00:00-05:00",
                "waypoint_config_sha256": sha(waypoint),
                "routes": [
                    {
                        "name": "center",
                        "no_object_clearance_pass": True,
                        "static_object_clearance_pass": True,
                        "representative_fault_pose_pass": True,
                        "logs": [route_log.name],
                    }
                ],
                "reset_exit": {
                    "consecutive_resets": 10,
                    "manual_interventions": 0,
                    "visual_clearance_pass": True,
                    "pre_health_no_alarms": True,
                    "post_health_no_alarms": True,
                    "start_shoulder_lift_temperature_c": 31,
                    "end_shoulder_lift_temperature_c": 34,
                    "peak_shoulder_current_raw": 42,
                    "pre_health": pre.name,
                    "post_health": post.name,
                    "logs": reset_logs,
                },
            }
        )
    )
    manifest = tmp_path / "manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "run_id": "pilot-a",
                "policy_checkpoint": str(checkpoint.resolve()),
                "policy_revision": "revision-a",
                "policy_type": "smolvla",
                "inference_mode": "rtc",
                "chunk_size": 50,
                "n_action_steps": 50,
                "fps": 30,
                "detector_config_sha256": sha(detector),
                "waypoint_config_sha256": sha(waypoint),
            }
        )
    )
    return record, waypoint, checkpoint, detector, manifest


def patch_component_audits(monkeypatch):
    monkeypatch.setattr(
        readiness,
        "audit_checkpoint",
        lambda path: (SimpleNamespace(chunk_size=50, policy_type="smolvla"), []),
    )
    monkeypatch.setattr(
        readiness,
        "audit_recovery_log",
        lambda path, config, route: (
            SimpleNamespace(peak_current={"shoulder_lift": 42}),
            [],
        ),
    )


def test_complete_reviewed_bundle_issues_guarded_rtc_token(tmp_path, monkeypatch):
    patch_component_audits(monkeypatch)
    record, waypoint, checkpoint, detector, manifest = make_bundle(tmp_path)
    summary, token, errors = readiness.audit_recovery_readiness(
        validation_record_path=record,
        waypoint_path=waypoint,
        checkpoint_path=checkpoint,
        detector_config_path=detector,
        run_manifest_path=manifest,
    )
    assert errors == []
    assert summary.reset_logs == 10
    assert token["required_inference_engine"].endswith("RecoverySafeRTCInferenceEngine")
    assert token["waypoint_config_sha256"] == sha(waypoint)


def test_unvalidated_route_and_incomplete_reset_cannot_issue_token(tmp_path, monkeypatch):
    patch_component_audits(monkeypatch)
    record, waypoint, checkpoint, detector, manifest = make_bundle(tmp_path, validated=False)
    data = json.loads(record.read_text())
    data["reset_exit"]["manual_interventions"] = 1
    data["reset_exit"]["logs"] = data["reset_exit"]["logs"][:9]
    record.write_text(json.dumps(data))
    _, token, errors = readiness.audit_recovery_readiness(
        validation_record_path=record,
        waypoint_path=waypoint,
        checkpoint_path=checkpoint,
        detector_config_path=detector,
        run_manifest_path=manifest,
    )
    assert token is None
    assert any("validated=false" in error for error in errors)
    assert any("zero manual interventions" in error for error in errors)
    assert any("exactly 10" in error for error in errors)


def test_stale_manifest_hash_and_failed_component_audit_block_token(tmp_path, monkeypatch):
    record, waypoint, checkpoint, detector, manifest = make_bundle(tmp_path)
    monkeypatch.setattr(
        readiness,
        "audit_checkpoint",
        lambda path: (SimpleNamespace(chunk_size=50, policy_type="smolvla"), ["bad weights"]),
    )
    monkeypatch.setattr(
        readiness, "audit_recovery_log", lambda path, config, route: (None, ["overcurrent"])
    )
    data = json.loads(manifest.read_text())
    data["detector_config_sha256"] = "0" * 64
    manifest.write_text(json.dumps(data))
    _, token, errors = readiness.audit_recovery_readiness(
        validation_record_path=record,
        waypoint_path=waypoint,
        checkpoint_path=checkpoint,
        detector_config_path=detector,
        run_manifest_path=manifest,
    )
    assert token is None
    assert any("detector hash" in error for error in errors)
    assert any("checkpoint: bad weights" in error for error in errors)
    assert any("overcurrent" in error for error in errors)


def test_enablement_token_is_exclusive_create(tmp_path):
    path = tmp_path / "enablement.json"
    readiness.write_enablement_token(path, {"run_id": "pilot-a"})
    with pytest.raises(FileExistsError):
        readiness.write_enablement_token(path, {"run_id": "pilot-b"})
