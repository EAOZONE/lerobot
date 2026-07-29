#!/usr/bin/env python
"""Connect the frozen 30-dim telemetry corpus to a positions-only policy.

Arm 1/2 must see only the first six position values, while the stored dataset remains
30 dimensional for detector work. This module narrows the in-memory metadata/statistics
view used to build the policy and prepends truncation before normalization.
"""

from __future__ import annotations

from collections.abc import Callable
from copy import copy, deepcopy
from typing import Any

import numpy as np
import torch
from truncate_state_step import TruncateStateStep

from lerobot.utils.constants import OBS_STATE

CAMERA_RENAME_MAP = {
    "observation.images.wrist": "observation.images.camera1",
    "observation.images.overhead": "observation.images.camera2",
}


def _slice_last_dimension(value: Any, keep: int) -> Any:
    """Slice vector-valued stats while leaving scalar counts untouched."""
    if isinstance(value, (torch.Tensor, np.ndarray)):
        return value[..., :keep] if value.ndim else value
    if isinstance(value, list):
        return value[:keep]
    if isinstance(value, tuple):
        return value[:keep]
    return value


def positions_only_stats(stats: dict | None, keep: int = 6) -> dict | None:
    """Return a copy of dataset statistics with only position-state entries."""
    if stats is None:
        return None
    narrowed = deepcopy(stats)
    if OBS_STATE in narrowed:
        narrowed[OBS_STATE] = {
            name: _slice_last_dimension(value, keep) for name, value in narrowed[OBS_STATE].items()
        }
    return narrowed


def positions_only_metadata(meta: Any, keep: int = 6) -> Any:
    """Make a metadata proxy with copied/narrowed features and statistics."""
    narrowed = copy(meta)
    narrowed.info = copy(meta.info)
    narrowed.info.features = deepcopy(meta.info.features)

    state_feature = narrowed.info.features.get(OBS_STATE)
    if state_feature is None:
        raise KeyError(f"dataset has no required {OBS_STATE!r} feature")
    shape = list(state_feature.get("shape", []))
    if not shape or shape[0] < keep:
        raise ValueError(f"{OBS_STATE} shape must contain at least {keep} values, got {shape}")
    shape[0] = keep
    state_feature["shape"] = shape
    if isinstance(state_feature.get("names"), list):
        state_feature["names"] = state_feature["names"][:keep]

    narrowed.stats = positions_only_stats(meta.stats, keep)
    return narrowed


def install_positions_only_training(train_module: Any, keep: int = 6) -> None:
    """Patch one imported ``lerobot_train`` module for an Arm 1/2 process.

    This patch is process-local and never changes the dataset or ``src/lerobot``.
    Checkpoints serialize the inserted step, retaining the input contract at inference.
    """
    original_make_policy: Callable = train_module.make_policy
    original_make_processors: Callable = train_module.make_pre_post_processors

    def make_policy_positions_only(*, cfg, ds_meta=None, **kwargs):
        if ds_meta is not None:
            if set(CAMERA_RENAME_MAP).issubset(ds_meta.info.features):
                rename_map = kwargs.get("rename_map") or {}
                if any(rename_map.get(source) != target for source, target in CAMERA_RENAME_MAP.items()):
                    raise ValueError(
                        "positions-only SmolVLA training requires --rename_map="
                        f"'{CAMERA_RENAME_MAP}' so wrist/overhead images reach the pretrained camera inputs"
                    )
            ds_meta = positions_only_metadata(ds_meta, keep)
        return original_make_policy(cfg=cfg, ds_meta=ds_meta, **kwargs)

    def make_processors_positions_only(*args, **kwargs):
        if "dataset_stats" in kwargs:
            kwargs["dataset_stats"] = positions_only_stats(kwargs["dataset_stats"], keep)
        overrides = kwargs.get("preprocessor_overrides")
        if overrides is not None:
            overrides = deepcopy(overrides)
            normalizer = overrides.get("normalizer_processor")
            if normalizer is not None and "stats" in normalizer:
                normalizer["stats"] = positions_only_stats(normalizer["stats"], keep)
            kwargs["preprocessor_overrides"] = overrides
        preprocessor, postprocessor = original_make_processors(*args, **kwargs)
        if not any(isinstance(step, TruncateStateStep) for step in preprocessor.steps):
            preprocessor.steps = [TruncateStateStep(keep=keep), *preprocessor.steps]
        return preprocessor, postprocessor

    train_module.make_policy = make_policy_positions_only
    train_module.make_pre_post_processors = make_processors_positions_only
