#!/usr/bin/env python
"""Fail-closed audit of a recorded telemetry dataset before training or backup."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pyarrow.parquet as pq
from audit_alignment import audit as audit_alignment, load_rows

STATE_KEY = "observation.state"
ACTION_KEY = "action"
CAMERA_KEYS = ("observation.images.wrist", "observation.images.overhead")
MOTORS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
STATE_NAMES = tuple(
    f"{motor}.{field}" for field in ("pos", "load", "current", "vel", "volt") for motor in MOTORS
)
ACTION_NAMES = tuple(f"{motor}.pos" for motor in MOTORS)


@dataclass(frozen=True)
class AuditSummary:
    episodes: int
    frames: int
    data_files: int
    video_files: int
    sidecars: int


def _load_json(path: Path) -> dict[str, Any]:
    with path.open() as file:
        value = json.load(file)
    if not isinstance(value, dict):
        raise ValueError(f"{path}: expected a JSON object")
    return value


def _shape(feature: dict[str, Any]) -> tuple[int, ...]:
    return tuple(int(value) for value in feature.get("shape", []))


def _audit_stats(root: Path) -> list[str]:
    errors: list[str] = []
    path = root / "meta" / "stats.json"
    try:
        stats = _load_json(path)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        return [f"cannot load dataset statistics: {exc}"]
    for key, width in ((STATE_KEY, 30), (ACTION_KEY, 6)):
        feature_stats = stats.get(key)
        if not isinstance(feature_stats, dict):
            errors.append(f"meta/stats.json: missing {key} statistics")
            continue
        for statistic in ("mean", "std", "min", "max"):
            value = feature_stats.get(statistic)
            if not isinstance(value, list) or len(value) != width:
                errors.append(f"meta/stats.json: {key}.{statistic} must contain {width} values")
    return errors


def _audit_vector_column(path: Path, key: str, width: int) -> tuple[int, list[str]]:
    errors: list[str] = []
    frames = 0
    try:
        parquet = pq.ParquetFile(path)
        if key not in parquet.schema_arrow.names:
            return 0, [f"{path}: missing {key} column"]
        for batch in parquet.iter_batches(columns=[key], batch_size=4096):
            for offset, value in enumerate(batch.column(0).to_pylist()):
                row = frames + offset
                if not isinstance(value, list) or len(value) != width:
                    errors.append(f"{path}: row {row} {key} width is not {width}")
                    continue
                if not all(isinstance(item, (int, float)) and math.isfinite(item) for item in value):
                    errors.append(f"{path}: row {row} {key} contains non-finite/non-numeric data")
            frames += batch.num_rows
    except (OSError, ValueError) as exc:
        errors.append(f"{path}: cannot inspect {key}: {exc}")
    return frames, errors


def audit_dataset(root: Path, expected_episodes: int | None = None) -> tuple[AuditSummary, list[str]]:
    """Audit one local LeRobot telemetry dataset without connecting to hardware."""
    root = root.resolve()
    errors: list[str] = []
    info_path = root / "meta" / "info.json"
    try:
        info = _load_json(info_path)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        return AuditSummary(0, 0, 0, 0, 0), [f"cannot load dataset metadata: {exc}"]

    fps = info.get("fps")
    if fps != 30:
        errors.append(f"meta/info.json: fps must be 30, got {fps!r}")
    features = info.get("features")
    if not isinstance(features, dict):
        errors.append("meta/info.json: features must be an object")
        features = {}

    required = {
        STATE_KEY: ("float32", (30,)),
        ACTION_KEY: ("float32", (6,)),
        CAMERA_KEYS[0]: ("video", None),
        CAMERA_KEYS[1]: ("video", None),
    }
    for key, (dtype, shape) in required.items():
        feature = features.get(key)
        if not isinstance(feature, dict):
            errors.append(f"meta/info.json: missing required feature {key}")
            continue
        if feature.get("dtype") != dtype:
            errors.append(f"meta/info.json: {key} dtype must be {dtype}, got {feature.get('dtype')!r}")
        if shape is not None and _shape(feature) != shape:
            errors.append(f"meta/info.json: {key} shape must be {shape}, got {_shape(feature)}")
    for key, names in ((STATE_KEY, STATE_NAMES), (ACTION_KEY, ACTION_NAMES)):
        feature = features.get(key)
        if isinstance(feature, dict) and tuple(feature.get("names") or ()) != names:
            errors.append(f"meta/info.json: {key} names/order do not match frozen schema")
    errors.extend(_audit_stats(root))

    declared_episodes = info.get("total_episodes")
    declared_frames = info.get("total_frames")
    if not isinstance(declared_episodes, int) or declared_episodes < 1:
        errors.append(f"meta/info.json: invalid total_episodes {declared_episodes!r}")
        declared_episodes = 0
    if not isinstance(declared_frames, int) or declared_frames < 1:
        errors.append(f"meta/info.json: invalid total_frames {declared_frames!r}")
        declared_frames = 0
    if expected_episodes is not None and declared_episodes != expected_episodes:
        errors.append(f"expected {expected_episodes} episodes, metadata declares {declared_episodes}")

    data_files = sorted((root / "data").glob("**/*.parquet"))
    if not data_files:
        errors.append("no data/**/*.parquet files found")
    state_frames = action_frames = 0
    for path in data_files:
        count, column_errors = _audit_vector_column(path, STATE_KEY, 30)
        state_frames += count
        errors.extend(column_errors)
        count, column_errors = _audit_vector_column(path, ACTION_KEY, 6)
        action_frames += count
        errors.extend(column_errors)
    if state_frames != declared_frames:
        errors.append(f"stored state rows {state_frames} != metadata total_frames {declared_frames}")
    if action_frames != declared_frames:
        errors.append(f"stored action rows {action_frames} != metadata total_frames {declared_frames}")

    video_files = sorted((root / "videos").glob("**/*.mp4"))
    for camera in CAMERA_KEYS:
        camera_files = list((root / "videos" / camera).glob("**/*.mp4"))
        if not camera_files:
            errors.append(f"no MP4 files found for {camera}")
        for path in camera_files:
            if path.stat().st_size == 0:
                errors.append(f"empty MP4 file {path}")

    alignment_dir = root / "meta" / "alignment"
    sidecars = sorted(alignment_dir.glob("episode_*.jsonl"))
    expected_names = {f"episode_{index:06d}.jsonl" for index in range(declared_episodes)}
    actual_names = {path.name for path in sidecars}
    for name in sorted(expected_names - actual_names):
        errors.append(f"missing alignment sidecar {name}")
    for name in sorted(actual_names - expected_names):
        errors.append(f"unexpected alignment sidecar {name}")

    sidecar_frames = 0
    for path in sidecars:
        try:
            rows = load_rows(path)
            sidecar_frames += len(rows)
            alignment_errors = audit_alignment(rows, float(fps or 30))
            episode_index = int(path.stem.removeprefix("episode_"))
            if any(row.get("episode_index") != episode_index for row in rows):
                alignment_errors.append("episode indices do not match sidecar filename")
            for row in rows:
                cameras = row.get("cameras")
                if isinstance(cameras, dict) and set(cameras) != {"wrist", "overhead"}:
                    alignment_errors.append(f"frame {row.get('frame_index')}: cameras must be wrist+overhead")
            errors.extend(f"{path}: {error}" for error in alignment_errors)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            errors.append(f"{path}: {exc}")
    if sidecar_frames != declared_frames:
        errors.append(f"sidecar rows {sidecar_frames} != metadata total_frames {declared_frames}")

    return (
        AuditSummary(
            episodes=declared_episodes,
            frames=declared_frames,
            data_files=len(data_files),
            video_files=len(video_files),
            sidecars=len(sidecars),
        ),
        errors,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset_root", type=Path)
    parser.add_argument("--expected-episodes", type=int)
    args = parser.parse_args()
    summary, errors = audit_dataset(args.dataset_root, args.expected_episodes)
    if errors:
        print(f"FAIL {args.dataset_root} ({len(errors)} issue(s))")
        for error in errors:
            print(f"  - {error}")
        raise SystemExit(1)
    print(
        f"PASS {args.dataset_root}: {summary.episodes} episodes, {summary.frames} frames, "
        f"{summary.data_files} data files, {summary.video_files} videos, "
        f"{summary.sidecars} aligned sidecars"
    )


if __name__ == "__main__":
    main()
