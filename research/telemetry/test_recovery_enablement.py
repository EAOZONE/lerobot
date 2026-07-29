import hashlib
import json

import pytest
from recovery_enablement import RecoveryEnablementError, verify_recovery_enablement
from recovery_rtc_guard import RecoverySafeRTCInferenceEngine


class GuardedEngine:
    pass


class WrongEngine:
    pass


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def bundle(tmp_path):
    validation = tmp_path / "validation.json"
    waypoint = tmp_path / "waypoints.json"
    detector = tmp_path / "detector.json"
    checkpoint = tmp_path / "checkpoint"
    checkpoint.mkdir()
    validation.write_text('{"reviewed":true}\n')
    waypoint.write_text('{"routes":["center"]}\n')
    detector.write_text('{"detector":"d0"}\n')
    engine = GuardedEngine()
    engine_name = f"{type(engine).__module__}.{type(engine).__qualname__}"
    manifest = tmp_path / "manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "run_id": "pilot-a",
                "policy_checkpoint": str(checkpoint.resolve()),
                "policy_revision": "revision-a",
                "chunk_size": 50,
                "n_action_steps": 50,
                "inference_mode": "sync",
                "waypoint_config_sha256": sha(waypoint),
                "detector_config_sha256": sha(detector),
            }
        )
    )
    token = tmp_path / "token.json"
    token.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "run_id": "pilot-a",
                "policy_revision": "revision-a",
                "chunk_size": 50,
                "n_action_steps": 50,
                "inference_mode": "sync",
                "validation_record_sha256": sha(validation),
                "waypoint_config_sha256": sha(waypoint),
                "detector_config_sha256": sha(detector),
                "checkpoint": str(checkpoint.resolve()),
                "required_inference_engine": engine_name,
            }
        )
    )
    return token, validation, waypoint, detector, checkpoint, manifest, engine


def verify(parts, engine=None):
    token, validation, waypoint, detector, checkpoint, manifest, default_engine = parts
    return verify_recovery_enablement(
        token_path=token,
        validation_record_path=validation,
        waypoint_path=waypoint,
        detector_config_path=detector,
        checkpoint_path=checkpoint,
        run_manifest_path=manifest,
        inference_engine=default_engine if engine is None else engine,
    )


def test_runtime_accepts_exact_bound_artifacts_and_engine(tmp_path):
    parts = bundle(tmp_path)
    assert verify(parts)["run_id"] == "pilot-a"


def test_changed_waypoint_invalidates_token_and_manifest(tmp_path):
    parts = bundle(tmp_path)
    parts[2].write_text('{"routes":["changed"]}\n')
    with pytest.raises(RecoveryEnablementError, match="waypoint"):
        verify(parts)


def test_wrong_engine_is_rejected_before_recovery_construction(tmp_path):
    parts = bundle(tmp_path)
    with pytest.raises(RecoveryEnablementError, match="inference engine mismatch"):
        verify(parts, WrongEngine())


def test_manifest_drift_and_non_full_chunk_execution_are_rejected(tmp_path):
    parts = bundle(tmp_path)
    manifest = json.loads(parts[5].read_text())
    manifest["policy_revision"] = "revision-b"
    manifest["n_action_steps"] = 1
    parts[5].write_text(json.dumps(manifest))
    with pytest.raises(RecoveryEnablementError) as exc:
        verify(parts)
    assert "policy_revision" in str(exc.value)
    assert "full-chunk" in str(exc.value)


def test_rtc_token_cannot_name_an_unguarded_engine(tmp_path):
    parts = bundle(tmp_path)
    manifest = json.loads(parts[5].read_text())
    manifest["inference_mode"] = "rtc"
    parts[5].write_text(json.dumps(manifest))
    token = json.loads(parts[0].read_text())
    token["inference_mode"] = "rtc"
    parts[0].write_text(json.dumps(token))
    with pytest.raises(RecoveryEnablementError, match="generation-guarded"):
        verify(parts)


def test_rtc_token_accepts_exact_research_guard_class_without_starting_engine(tmp_path):
    parts = bundle(tmp_path)
    manifest = json.loads(parts[5].read_text())
    manifest["inference_mode"] = "rtc"
    parts[5].write_text(json.dumps(manifest))
    token = json.loads(parts[0].read_text())
    token["inference_mode"] = "rtc"
    token["required_inference_engine"] = (
        "recovery_rtc_guard.RecoverySafeRTCInferenceEngine"
    )
    parts[0].write_text(json.dumps(token))
    engine = object.__new__(RecoverySafeRTCInferenceEngine)
    assert verify(parts, engine)["inference_mode"] == "rtc"
