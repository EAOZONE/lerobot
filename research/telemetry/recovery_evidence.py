#!/usr/bin/env python
"""Stable identifiers and append-only evidence writers for recovery rollouts."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from feetech_block import BLOCK_FIELDS
from probe_bus import SO101_MOTORS
from recovery import RawPose

SAFE_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}")
OUTCOMES = {"success", "failure", "aborted"}


def episode_id(run_id: str, episode_index: int) -> str:
    _validate_run_id(run_id)
    if episode_index < 0:
        raise ValueError("episode_index must be non-negative")
    return f"{run_id}:e{episode_index:06d}"


def attempt_id(run_id: str, episode_index: int, attempt: int) -> str:
    if attempt <= 0:
        raise ValueError("attempt must be positive")
    return f"{episode_id(run_id, episode_index)}:r{attempt:02d}"


def _validate_run_id(run_id: str) -> None:
    if run_id == "unbound" or not SAFE_ID.fullmatch(run_id):
        raise ValueError("run_id must be a bound 1..128 character safe identifier")


@dataclass(frozen=True)
class RecoveryAttemptContext:
    run_id: str
    episode_index: int
    attempt: int
    route: str | None = None

    def __post_init__(self) -> None:
        _validate_run_id(self.run_id)
        if self.episode_index < 0 or self.attempt <= 0:
            raise ValueError("episode_index must be non-negative and attempt must be positive")
        if self.route is not None and not self.route:
            raise ValueError("route must be nonempty when provided")

    @property
    def episode_id(self) -> str:
        return episode_id(self.run_id, self.episode_index)

    @property
    def attempt_id(self) -> str:
        return attempt_id(self.run_id, self.episode_index, self.attempt)


@dataclass(frozen=True)
class EpisodeOutcome:
    run_id: str
    episode_index: int
    outcome: str
    recovery_attempts: int
    policy_ticks: int
    detail: str | None = None

    def __post_init__(self) -> None:
        _validate_run_id(self.run_id)
        if self.episode_index < 0:
            raise ValueError("episode_index must be non-negative")
        if self.outcome not in OUTCOMES:
            raise ValueError(f"outcome must be one of {sorted(OUTCOMES)}")
        if self.recovery_attempts < 0 or self.policy_ticks < 0:
            raise ValueError("recovery_attempts and policy_ticks must be non-negative")

    def as_row(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "episode_id": episode_id(self.run_id, self.episode_index),
            **asdict(self),
        }


class EpisodeOutcomeLogger:
    """Append and fsync one terminal rollout outcome per episode."""

    def __init__(self, path: Path):
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)

    def __call__(self, outcome: EpisodeOutcome) -> None:
        with self.path.open("a") as file:
            file.write(json.dumps(outcome.as_row(), sort_keys=True) + "\n")
            file.flush()
            os.fsync(file.fileno())


def write_run_manifest(path: Path, manifest: dict[str, Any]) -> None:
    """Create, never overwrite, the immutable policy/config identity for one rollout run."""
    required = {
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
    missing = sorted(required - set(manifest))
    if missing:
        raise ValueError(f"run manifest missing fields: {', '.join(missing)}")
    _validate_run_id(str(manifest["run_id"]))
    if manifest["inference_mode"] not in {"sync", "rtc"}:
        raise ValueError("inference_mode must be sync or rtc")
    for key in ("policy_checkpoint", "policy_revision", "policy_type"):
        if not isinstance(manifest[key], str) or not manifest[key]:
            raise ValueError(f"{key} must be nonempty")
    for key in ("chunk_size", "n_action_steps"):
        if not isinstance(manifest[key], int) or manifest[key] <= 0:
            raise ValueError(f"{key} must be a positive integer")
    if manifest["n_action_steps"] != manifest["chunk_size"]:
        raise ValueError("n_action_steps must equal chunk_size")
    for key in ("detector_config_sha256", "waypoint_config_sha256"):
        if not re.fullmatch(r"[0-9a-f]{64}", str(manifest[key])):
            raise ValueError(f"{key} must be lowercase SHA-256")
    fps = float(manifest["fps"])
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError("fps must be finite and positive")
    row = {"schema_version": 1, **manifest}
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(descriptor, "w") as file:
        json.dump(row, file, indent=2, sort_keys=True)
        file.write("\n")
        file.flush()
        os.fsync(file.fileno())


class RecoveryAttemptCSVLogger:
    """Write physical recovery frames with immutable run/episode/attempt identity."""

    def __init__(
        self,
        path: Path,
        context: RecoveryAttemptContext,
        clock_ns: Callable[[], int] = time.perf_counter_ns,
    ) -> None:
        self.path = path
        self.context = context
        self.clock_ns = clock_ns
        self._file = None
        self._writer = None
        self._started_ns = 0
        self._route = context.route

    def bind_route(self, route: str) -> None:
        """Bind the executor-selected route exactly once, before the first frame."""
        if not route:
            raise ValueError("route must not be empty")
        if self._route is not None and self._route != route:
            raise ValueError(f"recovery logger route already bound to {self._route!r}")
        self._route = route

    def __enter__(self) -> RecoveryAttemptCSVLogger:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("x", newline="")
        columns = [
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
        ]
        columns += [f"goal_pos.{motor}" for motor in SO101_MOTORS]
        columns += [f"{field}.{motor}" for field in BLOCK_FIELDS for motor in SO101_MOTORS]
        self._writer = csv.DictWriter(self._file, fieldnames=columns)
        self._writer.writeheader()
        self._started_ns = self.clock_ns()
        return self

    def __call__(
        self,
        phase: str,
        frame_idx: int,
        goal: RawPose,
        telemetry: dict[str, dict[str, int]],
    ) -> None:
        if self._writer is None:
            raise RuntimeError("recovery attempt logger is not open")
        if self._route is None:
            raise RuntimeError("recovery route must be bound before logging frames")
        now_ns = self.clock_ns()
        row = {
            "schema_version": 1,
            "run_id": self.context.run_id,
            "episode_id": self.context.episode_id,
            "attempt_id": self.context.attempt_id,
            "episode_index": self.context.episode_index,
            "attempt": self.context.attempt,
            "route": self._route,
            "t": (now_ns - self._started_ns) / 1e9,
            "monotonic_ns": now_ns,
            "phase": phase,
            "frame_idx": frame_idx,
        }
        for motor, value in goal.items():
            row[f"goal_pos.{motor}"] = value
        for field, values in telemetry.items():
            for motor, value in values.items():
                row[f"{field}.{motor}"] = value
        self._writer.writerow(row)

    def __exit__(self, *_args: object) -> None:
        if self._file is not None:
            self._file.flush()
            os.fsync(self._file.fileno())
            self._file.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    manifest_parser = subparsers.add_parser("manifest", help="create immutable run manifest")
    manifest_parser.add_argument("--out", required=True, type=Path)
    manifest_parser.add_argument("--run-id", required=True)
    manifest_parser.add_argument("--policy-checkpoint", required=True)
    manifest_parser.add_argument("--policy-revision", required=True)
    manifest_parser.add_argument("--policy-type", required=True)
    manifest_parser.add_argument("--inference-mode", choices=("sync", "rtc"), required=True)
    manifest_parser.add_argument("--chunk-size", type=int, required=True)
    manifest_parser.add_argument("--n-action-steps", type=int, required=True)
    manifest_parser.add_argument("--fps", type=float, required=True)
    manifest_parser.add_argument("--detector-config-sha256", required=True)
    manifest_parser.add_argument("--waypoint-config-sha256", required=True)

    outcome_parser = subparsers.add_parser("outcome", help="append terminal episode outcome")
    outcome_parser.add_argument("--out", required=True, type=Path)
    outcome_parser.add_argument("--run-id", required=True)
    outcome_parser.add_argument("--episode-index", type=int, required=True)
    outcome_parser.add_argument("--outcome", choices=sorted(OUTCOMES), required=True)
    outcome_parser.add_argument("--recovery-attempts", type=int, required=True)
    outcome_parser.add_argument("--policy-ticks", type=int, required=True)
    outcome_parser.add_argument("--detail")
    args = parser.parse_args()

    if args.command == "manifest":
        write_run_manifest(
            args.out,
            {
                "run_id": args.run_id,
                "policy_checkpoint": args.policy_checkpoint,
                "policy_revision": args.policy_revision,
                "policy_type": args.policy_type,
                "inference_mode": args.inference_mode,
                "chunk_size": args.chunk_size,
                "n_action_steps": args.n_action_steps,
                "fps": args.fps,
                "detector_config_sha256": args.detector_config_sha256,
                "waypoint_config_sha256": args.waypoint_config_sha256,
            },
        )
        print(f"WROTE immutable run manifest {args.out}")
        return
    EpisodeOutcomeLogger(args.out)(
        EpisodeOutcome(
            run_id=args.run_id,
            episode_index=args.episode_index,
            outcome=args.outcome,
            recovery_attempts=args.recovery_attempts,
            policy_ticks=args.policy_ticks,
            detail=args.detail,
        )
    )
    print(f"APPENDED terminal outcome for {episode_id(args.run_id, args.episode_index)}")


if __name__ == "__main__":
    main()
