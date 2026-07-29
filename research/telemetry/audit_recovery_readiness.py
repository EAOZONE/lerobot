#!/usr/bin/env python
"""Issue an immutable recovery-enablement token only from complete reviewed evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from audit_positions_checkpoint import audit_checkpoint
from audit_recovery import audit_recovery_log
from recovery import load_recovery_config


@dataclass(frozen=True)
class ReadinessSummary:
    routes: int
    route_logs: int
    reset_logs: int
    run_id: str


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for block in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_object(path: Path, label: str, errors: list[str]) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        errors.append(f"cannot load {label} {path}: {exc}")
        return {}
    if not isinstance(value, dict):
        errors.append(f"{label} must contain a JSON object")
        return {}
    return value


def _resolve(record_path: Path, value: Any, label: str, errors: list[str]) -> Path | None:
    if not isinstance(value, str) or not value:
        errors.append(f"{label} must be a nonempty path")
        return None
    path = Path(value)
    if not path.is_absolute():
        path = record_path.parent / path
    path = path.resolve()
    if not path.is_file() or path.stat().st_size == 0:
        errors.append(f"{label} does not identify a nonempty file: {path}")
        return None
    return path


def audit_recovery_readiness(
    *,
    validation_record_path: Path,
    waypoint_path: Path,
    checkpoint_path: Path,
    detector_config_path: Path,
    run_manifest_path: Path,
) -> tuple[ReadinessSummary, dict[str, Any] | None, list[str]]:
    errors: list[str] = []
    record = _load_object(validation_record_path, "validation record", errors)
    manifest = _load_object(run_manifest_path, "run manifest", errors)
    try:
        config = load_recovery_config(waypoint_path)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        errors.append(f"cannot load waypoint configuration {waypoint_path}: {exc}")
        config = None

    waypoint_hash = _sha256(waypoint_path) if waypoint_path.is_file() else ""
    detector_hash = _sha256(detector_config_path) if detector_config_path.is_file() else ""
    record_hash = _sha256(validation_record_path) if validation_record_path.is_file() else ""
    run_id = str(manifest.get("run_id", ""))
    if manifest.get("schema_version") != 1:
        errors.append("run manifest schema_version must be 1")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}", run_id) or run_id == "unbound":
        errors.append("run manifest has no bound canonical run_id")
    if manifest.get("waypoint_config_sha256") != waypoint_hash:
        errors.append("run manifest waypoint hash does not match reviewed configuration")
    if manifest.get("detector_config_sha256") != detector_hash:
        errors.append("run manifest detector hash does not match supplied configuration")
    if manifest.get("n_action_steps") != manifest.get("chunk_size"):
        errors.append("run manifest must use n_action_steps == chunk_size")
    if manifest.get("inference_mode") not in {"sync", "rtc"}:
        errors.append("run manifest inference_mode must be sync or rtc")
    if not isinstance(manifest.get("policy_revision"), str) or not manifest["policy_revision"]:
        errors.append("run manifest policy_revision must be nonempty")

    checkpoint_summary, checkpoint_errors = audit_checkpoint(checkpoint_path)
    errors.extend(f"checkpoint: {error}" for error in checkpoint_errors)
    try:
        declared_checkpoint = Path(str(manifest.get("policy_checkpoint", ""))).resolve()
    except OSError:
        declared_checkpoint = Path()
    if declared_checkpoint != checkpoint_path.resolve():
        errors.append("run manifest policy_checkpoint does not match audited checkpoint")
    if checkpoint_summary.chunk_size and manifest.get("chunk_size") != checkpoint_summary.chunk_size:
        errors.append("run manifest chunk_size does not match audited checkpoint")
    if checkpoint_summary.policy_type and manifest.get("policy_type") != checkpoint_summary.policy_type:
        errors.append("run manifest policy_type does not match audited checkpoint")

    if record.get("schema_version") != 1:
        errors.append("validation record schema_version must be 1")
    for key in ("operator", "reviewed_at"):
        if not isinstance(record.get(key), str) or not record[key]:
            errors.append(f"validation record {key} must be nonempty")
    if record.get("waypoint_config_sha256") != waypoint_hash:
        errors.append("validation record waypoint hash does not match supplied configuration")

    configured_routes = {route.name: route for route in config.routes} if config else {}
    route_rows = record.get("routes")
    if not isinstance(route_rows, list):
        errors.append("validation record routes must be a list")
        route_rows = []
    names = [row.get("name") for row in route_rows if isinstance(row, dict)]
    if len(names) != len(set(names)):
        errors.append("validation record contains duplicate route names")
    if set(names) != set(configured_routes):
        errors.append("validation record route names must exactly match waypoint configuration")

    route_log_count = 0
    for row_number, row in enumerate(route_rows, 1):
        if not isinstance(row, dict):
            errors.append(f"route record {row_number} must be an object")
            continue
        name = row.get("name")
        route = configured_routes.get(name)
        if route is None:
            continue
        if not route.validated:
            errors.append(f"route {name!r} remains validated=false")
        for key in (
            "no_object_clearance_pass",
            "static_object_clearance_pass",
            "representative_fault_pose_pass",
        ):
            if row.get(key) is not True:
                errors.append(f"route {name!r} requires {key}=true")
        logs = row.get("logs")
        if not isinstance(logs, list) or not logs:
            errors.append(f"route {name!r} must reference at least one supervised log")
            continue
        route_log_count += len(logs)
        for log_index, value in enumerate(logs, 1):
            path = _resolve(
                validation_record_path,
                value,
                f"route {name!r} log {log_index}",
                errors,
            )
            if path is not None and config is not None:
                _, audit_errors = audit_recovery_log(path, config, str(name))
                errors.extend(f"route {name!r} log {path}: {error}" for error in audit_errors)

    reset = record.get("reset_exit")
    reset_logs: list[Any] = []
    if not isinstance(reset, dict):
        errors.append("validation record reset_exit must be an object")
    else:
        if reset.get("consecutive_resets") != 10:
            errors.append("reset exit requires exactly 10 declared consecutive resets")
        if reset.get("manual_interventions") != 0:
            errors.append("reset exit requires zero manual interventions")
        if reset.get("visual_clearance_pass") is not True:
            errors.append("reset exit requires visual_clearance_pass=true")
        for key in ("pre_health_no_alarms", "post_health_no_alarms"):
            if reset.get(key) is not True:
                errors.append(f"reset exit requires {key}=true")
        for key in (
            "start_shoulder_lift_temperature_c",
            "end_shoulder_lift_temperature_c",
        ):
            try:
                temperature = float(reset.get(key))
                if not math.isfinite(temperature) or not 0 <= temperature <= 100:
                    raise ValueError
            except (TypeError, ValueError):
                errors.append(f"reset exit {key} must be a finite value in 0..100 C")
        declared_peak = reset.get("peak_shoulder_current_raw")
        if not isinstance(declared_peak, int) or declared_peak < 0:
            errors.append("reset exit peak_shoulder_current_raw must be a non-negative integer")
        for key in ("pre_health", "post_health"):
            _resolve(validation_record_path, reset.get(key), f"reset_exit {key}", errors)
        candidate_logs = reset.get("logs")
        if not isinstance(candidate_logs, list) or len(candidate_logs) != 10:
            errors.append("reset exit must reference exactly 10 redesigned recovery logs")
        else:
            reset_logs = candidate_logs
            resolved_reset_paths: list[Path] = []
            measured_peaks: list[int] = []
            for index, item in enumerate(reset_logs, 1):
                if not isinstance(item, dict):
                    errors.append(f"reset log {index} must contain path and route")
                    continue
                route_name = item.get("route")
                path = _resolve(
                    validation_record_path,
                    item.get("path"),
                    f"reset log {index}",
                    errors,
                )
                if route_name not in configured_routes:
                    errors.append(f"reset log {index} names unknown route {route_name!r}")
                elif path is not None and config is not None:
                    resolved_reset_paths.append(path)
                    audit_summary, audit_errors = audit_recovery_log(path, config, str(route_name))
                    errors.extend(f"reset log {index} {path}: {error}" for error in audit_errors)
                    if audit_summary is not None:
                        measured_peaks.append(audit_summary.peak_current["shoulder_lift"])
            if len(resolved_reset_paths) != len(set(resolved_reset_paths)):
                errors.append("reset exit logs must be 10 distinct files")
            if measured_peaks and declared_peak != max(measured_peaks):
                errors.append(
                    "reset exit peak_shoulder_current_raw does not match audited reset logs"
                )

    summary = ReadinessSummary(len(configured_routes), route_log_count, len(reset_logs), run_id)
    if errors:
        return summary, None, errors
    token = {
        "schema_version": 1,
        "run_id": run_id,
        "issued_wall_time_ns": time.time_ns(),
        "validation_record_sha256": record_hash,
        "waypoint_config_sha256": waypoint_hash,
        "detector_config_sha256": detector_hash,
        "checkpoint": str(checkpoint_path.resolve()),
        "policy_revision": manifest["policy_revision"],
        "chunk_size": manifest["chunk_size"],
        "n_action_steps": manifest["n_action_steps"],
        "inference_mode": manifest["inference_mode"],
        "required_inference_engine": (
            "recovery_rtc_guard.RecoverySafeRTCInferenceEngine"
            if manifest["inference_mode"] == "rtc"
            else "lerobot.rollout.inference.sync.SyncInferenceEngine"
        ),
        "routes": sorted(configured_routes),
        "reset_logs": len(reset_logs),
    }
    return summary, token, []


def write_enablement_token(path: Path, token: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(descriptor, "w") as file:
        json.dump(token, file, indent=2, sort_keys=True)
        file.write("\n")
        file.flush()
        os.fsync(file.fileno())
    path.chmod(0o444)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--validation-record", required=True, type=Path)
    parser.add_argument("--waypoints", required=True, type=Path)
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--detector-config", required=True, type=Path)
    parser.add_argument("--run-manifest", required=True, type=Path)
    parser.add_argument("--out-token", required=True, type=Path)
    args = parser.parse_args()
    summary, token, errors = audit_recovery_readiness(
        validation_record_path=args.validation_record,
        waypoint_path=args.waypoints,
        checkpoint_path=args.checkpoint,
        detector_config_path=args.detector_config,
        run_manifest_path=args.run_manifest,
    )
    if errors:
        print(f"FAIL: {len(errors)} recovery-readiness issue(s)")
        for error in errors:
            print(f"  - {error}")
        raise SystemExit(1)
    assert token is not None
    write_enablement_token(args.out_token, token)
    print(
        f"PASS: issued {args.out_token} for {summary.run_id}; {summary.routes} routes, "
        f"{summary.route_logs} validation logs, {summary.reset_logs} consecutive reset logs"
    )


if __name__ == "__main__":
    main()
