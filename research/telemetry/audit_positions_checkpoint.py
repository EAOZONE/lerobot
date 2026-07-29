#!/usr/bin/env python
"""Fail-closed eligibility audit for an Arm 1/2 positions-only checkpoint."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from safetensors import SafetensorError, safe_open
from telemetry_policy_bridge import CAMERA_RENAME_MAP

STATE_KEY = "observation.state"
ACTION_KEY = "action"


@dataclass(frozen=True)
class CheckpointAuditSummary:
    policy_type: str
    chunk_size: int
    n_action_steps: int
    processor_steps: int
    state_stat_tensors: int


def _load_json(path: Path) -> dict[str, Any]:
    with path.open() as file:
        value = json.load(file)
    if not isinstance(value, dict):
        raise ValueError(f"{path}: expected a JSON object")
    return value


def _feature_shape(features: Any, key: str) -> tuple[int, ...]:
    if not isinstance(features, dict) or not isinstance(features.get(key), dict):
        return ()
    return tuple(int(value) for value in features[key].get("shape", []))


def audit_checkpoint(root: Path) -> tuple[CheckpointAuditSummary, list[str]]:
    """Inspect checkpoint configuration and processor state without loading model weights."""
    root = root.resolve()
    errors: list[str] = []
    try:
        policy = _load_json(root / "config.json")
        processor = _load_json(root / "policy_preprocessor.json")
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        return CheckpointAuditSummary("", 0, 0, 0, 0), [f"cannot load checkpoint config: {exc}"]

    policy_type = str(policy.get("type", ""))
    if policy_type != "smolvla":
        errors.append(f"policy type must be smolvla, got {policy_type!r}")
    if _feature_shape(policy.get("input_features"), STATE_KEY) != (6,):
        errors.append("policy config observation.state input shape must be (6,)")
    if _feature_shape(policy.get("output_features"), ACTION_KEY) != (6,):
        errors.append("policy config action output shape must be (6,)")
    visual_features = {
        key for key, value in policy.get("input_features", {}).items() if value.get("type") == "VISUAL"
    }
    for camera in CAMERA_RENAME_MAP.values():
        if camera not in visual_features:
            errors.append(f"policy config is missing expected visual feature {camera}")

    try:
        chunk_size = int(policy.get("chunk_size", 0))
        n_action_steps = int(policy.get("n_action_steps", 0))
    except (TypeError, ValueError):
        chunk_size = n_action_steps = 0
    if chunk_size <= 0 or n_action_steps != chunk_size:
        errors.append(f"n_action_steps must equal positive chunk_size, got {n_action_steps} and {chunk_size}")

    steps = processor.get("steps")
    if not isinstance(steps, list) or not steps:
        errors.append("policy_preprocessor.json has no steps")
        steps = []
    if steps:
        first = steps[0]
        if first.get("registry_name") != "truncate_state":
            errors.append("truncate_state must be the first preprocessor step")
        if first.get("config", {}).get("keep") != 6:
            errors.append("truncate_state must explicitly serialize keep=6")

    rename_steps = [step for step in steps if step.get("registry_name") == "rename_observations_processor"]
    if len(rename_steps) != 1:
        errors.append(f"expected exactly one rename processor, found {len(rename_steps)}")
    elif rename_steps[0].get("config", {}).get("rename_map") != CAMERA_RENAME_MAP:
        errors.append("camera rename map does not match frozen wrist/overhead mapping")

    normalizers = [step for step in steps if step.get("registry_name") == "normalizer_processor"]
    state_stat_tensors = 0
    state_stat_names: set[str] = set()
    if len(normalizers) != 1:
        errors.append(f"expected exactly one normalizer processor, found {len(normalizers)}")
    else:
        normalizer = normalizers[0]
        features = normalizer.get("config", {}).get("features")
        if _feature_shape(features, STATE_KEY) != (6,):
            errors.append("normalizer observation.state feature shape must be (6,)")
        if _feature_shape(features, ACTION_KEY) != (6,):
            errors.append("normalizer action feature shape must be (6,)")
        state_file = normalizer.get("state_file")
        if not isinstance(state_file, str):
            errors.append("normalizer does not declare a state_file")
        else:
            state_path = root / state_file
            try:
                with safe_open(state_path, framework="pt", device="cpu") as tensors:
                    for key in list(tensors.keys()):
                        shape = tuple(tensors.get_slice(key).get_shape())
                        if key.startswith(f"{STATE_KEY}."):
                            state_stat_tensors += 1
                            statistic = key.removeprefix(f"{STATE_KEY}.")
                            state_stat_names.add(statistic)
                            expected_shape = (1,) if statistic == "count" else (6,)
                            if shape != expected_shape:
                                errors.append(
                                    f"normalizer tensor {key} must have shape {expected_shape}, got {shape}"
                                )
                        elif key.startswith(f"{ACTION_KEY}."):
                            statistic = key.removeprefix(f"{ACTION_KEY}.")
                            expected_shape = (1,) if statistic == "count" else (6,)
                            if shape != expected_shape:
                                errors.append(
                                    f"normalizer tensor {key} must have shape {expected_shape}, got {shape}"
                                )
            except (OSError, SafetensorError) as exc:
                errors.append(f"cannot read normalizer state {state_path}: {exc}")
            missing_stats = {"mean", "std", "min", "max"} - state_stat_names
            if missing_stats:
                errors.append(
                    f"normalizer state is missing observation.state statistics: {sorted(missing_stats)}"
                )

    model_path = root / "model.safetensors"
    if not model_path.is_file() or model_path.stat().st_size == 0:
        errors.append("missing or empty model.safetensors")

    return (
        CheckpointAuditSummary(
            policy_type=policy_type,
            chunk_size=chunk_size,
            n_action_steps=n_action_steps,
            processor_steps=len(steps),
            state_stat_tensors=state_stat_tensors,
        ),
        errors,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checkpoint", type=Path)
    args = parser.parse_args()
    summary, errors = audit_checkpoint(args.checkpoint)
    if errors:
        print(f"FAIL {args.checkpoint} ({len(errors)} issue(s))")
        for error in errors:
            print(f"  - {error}")
        raise SystemExit(1)
    print(
        f"PASS {args.checkpoint}: {summary.policy_type}, state/action=6, "
        f"chunk={summary.chunk_size}, {summary.processor_steps} processor steps, "
        f"{summary.state_stat_tensors} state-stat tensors"
    )


if __name__ == "__main__":
    main()
