#!/usr/bin/env python
"""Fail-closed runtime verification of a recovery enablement token."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any


class RecoveryEnablementError(RuntimeError):
    """Recovery must remain disabled because its evidence token is invalid or stale."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as file:
            for block in iter(lambda: file.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise RecoveryEnablementError(f"cannot hash required artifact {path}: {exc}") from exc
    return digest.hexdigest()


def _load(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise RecoveryEnablementError(f"cannot load {label} {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise RecoveryEnablementError(f"{label} must contain one JSON object")
    return value


def _engine_name(engine: Any) -> str:
    cls = type(engine)
    return f"{cls.__module__}.{cls.__qualname__}"


def verify_recovery_enablement(
    *,
    token_path: Path,
    validation_record_path: Path,
    waypoint_path: Path,
    detector_config_path: Path,
    checkpoint_path: Path,
    run_manifest_path: Path,
    inference_engine: Any,
) -> dict[str, Any]:
    """Return the verified token or raise before recovery can be constructed."""
    token = _load(token_path, "enablement token")
    manifest = _load(run_manifest_path, "run manifest")
    errors: list[str] = []

    if token.get("schema_version") != 1:
        errors.append("enablement token schema_version must be 1")
    if manifest.get("schema_version") != 1:
        errors.append("run manifest schema_version must be 1")
    for key in (
        "run_id",
        "policy_revision",
        "chunk_size",
        "n_action_steps",
        "inference_mode",
    ):
        if token.get(key) != manifest.get(key):
            errors.append(f"token {key} does not match run manifest")
    if manifest.get("n_action_steps") != manifest.get("chunk_size"):
        errors.append("run manifest no longer uses full-chunk execution")

    expected_hashes = {
        "validation_record_sha256": _sha256(validation_record_path),
        "waypoint_config_sha256": _sha256(waypoint_path),
        "detector_config_sha256": _sha256(detector_config_path),
    }
    for key, actual in expected_hashes.items():
        if token.get(key) != actual:
            errors.append(f"token {key} does not match current artifact")
    if manifest.get("waypoint_config_sha256") != expected_hashes["waypoint_config_sha256"]:
        errors.append("run manifest waypoint hash does not match current artifact")
    if manifest.get("detector_config_sha256") != expected_hashes["detector_config_sha256"]:
        errors.append("run manifest detector hash does not match current artifact")

    resolved_checkpoint = str(checkpoint_path.resolve())
    if token.get("checkpoint") != resolved_checkpoint:
        errors.append("token checkpoint path does not match runtime checkpoint")
    try:
        manifest_checkpoint = str(Path(str(manifest.get("policy_checkpoint", ""))).resolve())
    except OSError:
        manifest_checkpoint = ""
    if manifest_checkpoint != resolved_checkpoint:
        errors.append("run manifest checkpoint path does not match runtime checkpoint")

    actual_engine = _engine_name(inference_engine)
    if token.get("required_inference_engine") != actual_engine:
        errors.append(
            f"inference engine mismatch: token requires {token.get('required_inference_engine')!r}, "
            f"runtime supplied {actual_engine!r}"
        )
    mode = manifest.get("inference_mode")
    guarded_rtc = "recovery_rtc_guard.RecoverySafeRTCInferenceEngine"
    if mode == "rtc" and token.get("required_inference_engine") != guarded_rtc:
        errors.append("RTC recovery requires the generation-guarded inference engine")
    if mode not in {"sync", "rtc"}:
        errors.append("run manifest inference_mode must be sync or rtc")

    if errors:
        raise RecoveryEnablementError("recovery enablement refused:\n- " + "\n- ".join(errors))
    return token
