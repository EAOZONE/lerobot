#!/usr/bin/env python
"""Audit joins across one recovery-enabled rollout's immutable evidence files."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from feetech_block import BLOCK_FIELDS
from probe_bus import SO101_MOTORS
from recovery_evidence import OUTCOMES, attempt_id, episode_id


@dataclass(frozen=True)
class EvidenceAuditSummary:
    episodes: int
    attempts: int
    recovery_frames: int


def _json(path: Path, errors: list[str]) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        errors.append(f"cannot read JSON {path}: {exc}")
        return {}
    if not isinstance(value, dict):
        errors.append(f"{path} must contain one JSON object")
        return {}
    return value


def _jsonl(path: Path, label: str, errors: list[str]) -> list[dict[str, Any]]:
    rows = []
    try:
        lines = path.read_text().splitlines()
    except OSError as exc:
        errors.append(f"cannot read {label} {path}: {exc}")
        return rows
    for line_number, line in enumerate(lines, 1):
        try:
            row = json.loads(line)
        except json.JSONDecodeError as exc:
            errors.append(f"{label} line {line_number}: invalid JSON: {exc}")
            continue
        if not isinstance(row, dict):
            errors.append(f"{label} line {line_number}: row must be an object")
            continue
        rows.append(row)
    return rows


def audit_recovery_evidence(
    manifest_path: Path,
    events_path: Path,
    outcomes_path: Path,
    recovery_logs: list[Path],
) -> tuple[EvidenceAuditSummary, list[str]]:
    errors: list[str] = []
    manifest = _json(manifest_path, errors)
    run_id = manifest.get("run_id")
    required_manifest = {
        "schema_version",
        "run_id",
        "policy_checkpoint",
        "policy_revision",
        "policy_type",
        "inference_mode",
        "chunk_size",
        "n_action_steps",
        "fps",
        "detector_config_sha256",
        "waypoint_config_sha256",
    }
    missing = sorted(required_manifest - set(manifest))
    if missing:
        errors.append(f"manifest missing fields: {', '.join(missing)}")
    if run_id in {None, "", "unbound"}:
        errors.append("manifest run_id must be bound and nonempty")
    canonical_run = (
        run_id
        if isinstance(run_id, str)
        and run_id != "unbound"
        and re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}", run_id)
        else "invalid"
    )
    if manifest.get("schema_version") != 1:
        errors.append("manifest schema_version must be 1")
    if manifest.get("inference_mode") not in {"sync", "rtc"}:
        errors.append("manifest inference_mode must be sync or rtc")
    for key in ("chunk_size", "n_action_steps"):
        if not isinstance(manifest.get(key), int) or manifest.get(key, 0) <= 0:
            errors.append(f"manifest {key} must be a positive integer")
    if manifest.get("chunk_size") != manifest.get("n_action_steps"):
        errors.append("manifest n_action_steps must equal chunk_size")
    for key in (
        "policy_checkpoint",
        "policy_revision",
        "policy_type",
        "detector_config_sha256",
        "waypoint_config_sha256",
    ):
        if not isinstance(manifest.get(key), str) or not manifest[key]:
            errors.append(f"manifest {key} must be nonempty")
    for key in ("detector_config_sha256", "waypoint_config_sha256"):
        if not re.fullmatch(r"[0-9a-f]{64}", str(manifest.get(key, ""))):
            errors.append(f"manifest {key} must be lowercase SHA-256")
    try:
        fps = float(manifest.get("fps"))
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError
    except (TypeError, ValueError):
        errors.append("manifest fps must be finite and positive")

    events = _jsonl(events_path, "events", errors)
    outcomes = _jsonl(outcomes_path, "outcomes", errors)
    started_episodes: set[str] = set()
    trigger_attempts: set[str] = set()
    started_attempts: set[str] = set()
    terminal_attempts: dict[str, dict[str, Any]] = {}

    for number, event in enumerate(events, 1):
        if event.get("schema_version") != 2:
            errors.append(f"events row {number}: schema_version must be 2")
        if event.get("run_id") != run_id:
            errors.append(f"events row {number}: run_id does not match manifest")
        try:
            index = int(event["episode_index"])
            expected_episode = episode_id(canonical_run, index)
        except (KeyError, TypeError, ValueError) as exc:
            errors.append(f"events row {number}: invalid episode identity: {exc}")
            continue
        if event.get("episode_id") != expected_episode:
            errors.append(f"events row {number}: episode_id is not canonical")
        attempt = event.get("attempt")
        expected_attempt = None
        if isinstance(attempt, int) and attempt > 0:
            expected_attempt = attempt_id(canonical_run, index, attempt)
        if event.get("attempt_id") != expected_attempt:
            errors.append(f"events row {number}: attempt_id is not canonical")
        kind = event.get("event")
        if kind == "episode_started":
            started_episodes.add(expected_episode)
        if kind == "trigger" and expected_attempt is not None:
            trigger_attempts.add(expected_attempt)
        if kind == "recovery_started" and expected_attempt is not None:
            if expected_attempt in started_attempts:
                errors.append(f"duplicate recovery_started for {expected_attempt}")
            started_attempts.add(expected_attempt)
        if kind in {"recovery_completed", "recovery_aborted"} and expected_attempt is not None:
            if expected_attempt in terminal_attempts:
                errors.append(f"duplicate terminal recovery event for {expected_attempt}")
            terminal_attempts[expected_attempt] = event

    outcome_by_episode: dict[str, dict[str, Any]] = {}
    for number, outcome in enumerate(outcomes, 1):
        if outcome.get("schema_version") != 1:
            errors.append(f"outcomes row {number}: schema_version must be 1")
        if outcome.get("run_id") != run_id:
            errors.append(f"outcomes row {number}: run_id does not match manifest")
        try:
            index = int(outcome["episode_index"])
            expected = episode_id(canonical_run, index)
        except (KeyError, TypeError, ValueError) as exc:
            errors.append(f"outcomes row {number}: invalid episode identity: {exc}")
            continue
        if outcome.get("episode_id") != expected:
            errors.append(f"outcomes row {number}: episode_id is not canonical")
        if expected in outcome_by_episode:
            errors.append(f"duplicate terminal outcome for {expected}")
        outcome_by_episode[expected] = outcome
        if outcome.get("outcome") not in OUTCOMES:
            errors.append(f"outcomes row {number}: invalid outcome")
        if not isinstance(outcome.get("policy_ticks"), int) or outcome["policy_ticks"] < 0:
            errors.append(f"outcomes row {number}: policy_ticks must be non-negative integer")
        if (
            not isinstance(outcome.get("recovery_attempts"), int)
            or outcome["recovery_attempts"] < 0
        ):
            errors.append(
                f"outcomes row {number}: recovery_attempts must be non-negative integer"
            )

    log_by_attempt: dict[str, tuple[int, str]] = {}
    total_frames = 0
    for path in recovery_logs:
        try:
            with path.open(newline="") as file:
                reader = csv.DictReader(file)
                columns = set(reader.fieldnames or ())
                rows = list(reader)
        except OSError as exc:
            errors.append(f"cannot read recovery log {path}: {exc}")
            continue
        required_columns = {
            "schema_version",
            "run_id",
            "episode_id",
            "attempt_id",
            "episode_index",
            "attempt",
            "route",
            "t",
            "monotonic_ns",
            "phase",
            "frame_idx",
        }
        required_columns.update(f"goal_pos.{motor}" for motor in SO101_MOTORS)
        required_columns.update(
            f"{field}.{motor}" for field in BLOCK_FIELDS for motor in SO101_MOTORS
        )
        missing_columns = sorted(required_columns - columns)
        if missing_columns:
            errors.append(f"recovery log {path} missing columns: {', '.join(missing_columns)}")
            continue
        if not rows:
            errors.append(f"recovery log {path} has no frames")
            continue
        ids = {row.get("attempt_id") for row in rows}
        if len(ids) != 1 or None in ids or "" in ids:
            errors.append(f"recovery log {path} must contain exactly one attempt_id")
            continue
        log_attempt = ids.pop()
        assert log_attempt is not None
        if log_attempt in log_by_attempt:
            errors.append(f"multiple recovery logs claim {log_attempt}")
            continue
        phases = []
        route_values = set()
        previous_t = -math.inf
        previous_monotonic_ns = -1
        for row_number, row in enumerate(rows, 2):
            missing_values = sorted(
                column for column in required_columns if row.get(column) in {None, ""}
            )
            if missing_values:
                errors.append(
                    f"{path} line {row_number}: empty required values: {', '.join(missing_values)}"
                )
                continue
            if row.get("schema_version") != "1" or row.get("run_id") != run_id:
                errors.append(f"{path} line {row_number}: schema/run identity mismatch")
            try:
                index = int(row["episode_index"])
                attempt = int(row["attempt"])
                frame = int(row["frame_idx"])
                relative_time = float(row["t"])
                monotonic_ns = int(row["monotonic_ns"])
                expected_attempt = attempt_id(canonical_run, index, attempt)
            except (KeyError, TypeError, ValueError) as exc:
                errors.append(f"{path} line {row_number}: invalid identity/frame: {exc}")
                continue
            if (
                not math.isfinite(relative_time)
                or relative_time < 0
                or relative_time <= previous_t
            ):
                errors.append(f"{path} line {row_number}: t must be finite and strictly increasing")
            if monotonic_ns <= previous_monotonic_ns:
                errors.append(f"{path} line {row_number}: monotonic_ns must increase strictly")
            previous_t = relative_time
            previous_monotonic_ns = monotonic_ns
            for motor in SO101_MOTORS:
                numeric_columns = [f"goal_pos.{motor}"] + [
                    f"{field}.{motor}" for field in BLOCK_FIELDS
                ]
                for column in numeric_columns:
                    try:
                        float(row[column])
                    except ValueError:
                        errors.append(f"{path} line {row_number}: {column} is not numeric")
            if row.get("episode_id") != episode_id(canonical_run, index):
                errors.append(f"{path} line {row_number}: episode_id is not canonical")
            if log_attempt != expected_attempt:
                errors.append(f"{path} line {row_number}: attempt_id is not canonical")
            if frame != row_number - 2:
                errors.append(f"{path} line {row_number}: frame_idx is not contiguous")
            phases.append(row.get("phase", ""))
            route_values.add(row.get("route", ""))
        if len(route_values) != 1 or "" in route_values:
            errors.append(f"recovery log {path} must contain exactly one nonempty route")
            route = ""
        else:
            route = route_values.pop()
        log_by_attempt[log_attempt] = (len(rows), route)
        total_frames += len(rows)

    for identifier in sorted(started_attempts - set(terminal_attempts)):
        errors.append(f"started attempt has no terminal supervisor event: {identifier}")
    for identifier in sorted(started_attempts - trigger_attempts):
        errors.append(f"started attempt has no trigger event: {identifier}")
    for identifier in sorted(set(terminal_attempts) - trigger_attempts):
        errors.append(f"terminal supervisor event has no trigger: {identifier}")
    for identifier in sorted(started_attempts - set(log_by_attempt)):
        errors.append(f"started attempt has no per-frame recovery log: {identifier}")
    for identifier in sorted(set(log_by_attempt) - started_attempts):
        errors.append(f"orphan recovery log has no supervisor start: {identifier}")
    for identifier in sorted(started_attempts & set(log_by_attempt)):
        event = terminal_attempts.get(identifier)
        if event and event.get("event") == "recovery_completed":
            frames, route = log_by_attempt[identifier]
            if event.get("completed_frames") != frames:
                errors.append(f"{identifier}: completed_frames does not match recovery CSV")
            if event.get("route") != route:
                errors.append(f"{identifier}: completed route does not match recovery CSV")

    for identifier in sorted(started_episodes - set(outcome_by_episode)):
        errors.append(f"started episode has no terminal outcome: {identifier}")
    for identifier in sorted(set(outcome_by_episode) - started_episodes):
        errors.append(f"orphan outcome has no episode_started event: {identifier}")

    attempts_per_episode = Counter(identifier.rsplit(":r", 1)[0] for identifier in started_attempts)
    for identifier, outcome in outcome_by_episode.items():
        if outcome.get("recovery_attempts") != attempts_per_episode[identifier]:
            errors.append(f"{identifier}: outcome recovery_attempts does not match events")

    return EvidenceAuditSummary(len(outcome_by_episode), len(started_attempts), total_frames), errors


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--events", required=True, type=Path)
    parser.add_argument("--outcomes", required=True, type=Path)
    parser.add_argument("recovery_logs", nargs="+", type=Path)
    args = parser.parse_args()
    summary, errors = audit_recovery_evidence(
        args.manifest, args.events, args.outcomes, args.recovery_logs
    )
    if errors:
        print(f"FAIL: {len(errors)} joined-evidence issue(s)")
        for error in errors:
            print(f"  - {error}")
        raise SystemExit(1)
    print(
        f"PASS: {summary.episodes} episodes, {summary.attempts} recovery attempts, "
        f"{summary.recovery_frames} recovery frames"
    )


if __name__ == "__main__":
    main()
