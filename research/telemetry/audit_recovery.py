#!/usr/bin/env python
"""Audit supervised recovery CSVs against their frozen waypoint safety limits."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

from probe_bus import SO101_MOTORS
from recovery import RecoveryConfig, load_recovery_config

PHASE_ORDER = ("reverse", "waypoint", "home", "open")


@dataclass(frozen=True)
class RecoveryAuditSummary:
    frames: int
    duration_s: float
    peak_current: dict[str, int]
    peak_following_error: dict[str, int]
    max_command_step: int


def _required_columns() -> set[str]:
    columns = {"t", "phase", "frame_idx"}
    for motor in SO101_MOTORS:
        columns.update((f"goal_pos.{motor}", f"pos.{motor}", f"curr.{motor}"))
    return columns


def audit_recovery_log(
    path: Path, config: RecoveryConfig, route_name: str
) -> tuple[RecoveryAuditSummary, list[str]]:
    """Validate one completed supervised run; geometric clearance remains out of band."""
    errors: list[str] = []
    matching_routes = [route for route in config.routes if route.name == route_name]
    if len(matching_routes) != 1:
        return RecoveryAuditSummary(0, 0, {}, {}, 0), [
            f"route {route_name!r} is not unique in the waypoint configuration"
        ]
    route = matching_routes[0]
    try:
        file = path.open(newline="")
    except OSError as exc:
        return RecoveryAuditSummary(0, 0, {}, {}, 0), [f"cannot open {path}: {exc}"]

    with file:
        reader = csv.DictReader(file)
        columns = set(reader.fieldnames or ())
        missing = sorted(_required_columns() - columns)
        if missing:
            return RecoveryAuditSummary(0, 0, {}, {}, 0), [f"missing required columns: {', '.join(missing)}"]
        rows = list(reader)
    if not rows:
        return RecoveryAuditSummary(0, 0, {}, {}, 0), ["recovery log has no frames"]

    limits = config.limits
    peak_current = dict.fromkeys(SO101_MOTORS, 0)
    peak_error = dict.fromkeys(SO101_MOTORS, 0)
    max_step = 0
    previous_goal: dict[str, int] | None = None
    seen_phases: list[str] = []
    phase_started: dict[str, float] = {}
    phase_ended: dict[str, float] = {}
    previous_time = -math.inf
    valid_times: list[float] = []

    for row_number, row in enumerate(rows, 2):
        try:
            timestamp = float(row["t"])
            frame_index = int(row["frame_idx"])
            goal = {motor: int(float(row[f"goal_pos.{motor}"])) for motor in SO101_MOTORS}
            present = {motor: int(float(row[f"pos.{motor}"])) for motor in SO101_MOTORS}
            current = {motor: int(float(row[f"curr.{motor}"])) for motor in SO101_MOTORS}
        except (TypeError, ValueError) as exc:
            errors.append(f"line {row_number}: invalid numeric field: {exc}")
            continue
        if not math.isfinite(timestamp) or timestamp <= previous_time:
            errors.append(f"line {row_number}: timestamps must be finite and strictly increasing")
        previous_time = timestamp
        valid_times.append(timestamp)
        phase = row["phase"]
        if phase not in PHASE_ORDER:
            errors.append(f"line {row_number}: unknown phase {phase!r}")
        elif not seen_phases or phase != seen_phases[-1]:
            seen_phases.append(phase)
            phase_started.setdefault(phase, timestamp)
        phase_ended[phase] = timestamp
        expected_frame = row_number - 2
        if frame_index != expected_frame:
            errors.append(
                f"line {row_number}: frame_idx must be contiguous; expected {expected_frame}, got {frame_index}"
            )

        for motor in SO101_MOTORS:
            if not limits.joint_min[motor] <= goal[motor] <= limits.joint_max[motor]:
                errors.append(f"line {row_number}: {motor} goal violates joint range")
            magnitude = abs(current[motor])
            peak_current[motor] = max(peak_current[motor], magnitude)
            if magnitude > limits.max_current[motor]:
                errors.append(f"line {row_number}: {motor} current exceeds configured limit")
            following_error = abs(goal[motor] - present[motor])
            peak_error[motor] = max(peak_error[motor], following_error)
            if following_error > limits.max_following_error[motor]:
                errors.append(f"line {row_number}: {motor} following error exceeds configured limit")
            if previous_goal is not None:
                step = abs(goal[motor] - previous_goal[motor])
                max_step = max(max_step, step)
                if step > limits.max_step_ticks:
                    errors.append(f"line {row_number}: {motor} command step exceeds configured limit")
        previous_goal = goal

    expected = list(PHASE_ORDER)
    if seen_phases != expected:
        errors.append(f"phase sequence must be {expected}, got {seen_phases}")
    for phase in seen_phases:
        duration = phase_ended[phase] - phase_started[phase]
        if duration > limits.phase_timeout_s:
            errors.append(f"{phase} phase duration {duration:.3f}s exceeds {limits.phase_timeout_s:.3f}s")
    if seen_phases and seen_phases[-1] == "open" and previous_goal is not None:
        final_gripper = previous_goal["gripper"]
        if final_gripper != limits.gripper_open:
            errors.append(f"final gripper goal {final_gripper} != configured open goal {limits.gripper_open}")
    if not route.validated:
        errors.append(
            f"route {route_name!r} remains validated=false; this log may support review but cannot close the gate"
        )

    duration = valid_times[-1] - valid_times[0] if valid_times else 0.0
    return (
        RecoveryAuditSummary(
            frames=len(rows),
            duration_s=duration,
            peak_current=peak_current,
            peak_following_error=peak_error,
            max_command_step=max_step,
        ),
        errors,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--waypoints", type=Path, required=True)
    parser.add_argument("--route", required=True)
    args = parser.parse_args()
    try:
        config = load_recovery_config(args.waypoints)
    except ValueError as exc:
        parser.error(str(exc))
    failed = False
    for path in args.logs:
        summary, errors = audit_recovery_log(path, config, args.route)
        if errors:
            failed = True
            print(f"FAIL {path} ({len(errors)} issue(s))")
            for error in errors:
                print(f"  - {error}")
        else:
            print(
                f"PASS {path}: {summary.frames} frames, {summary.duration_s:.2f}s, "
                f"max step {summary.max_command_step} ticks"
            )
            print(f"  peak current: {summary.peak_current}")
            print(f"  peak following error: {summary.peak_following_error}")
    raise SystemExit(1 if failed else 0)


if __name__ == "__main__":
    main()
