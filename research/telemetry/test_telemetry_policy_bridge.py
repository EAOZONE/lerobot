from types import SimpleNamespace

import numpy as np
import pytest
import torch
from telemetry_policy_bridge import (
    install_positions_only_training,
    positions_only_metadata,
    positions_only_stats,
)
from truncate_state_step import TruncateStateStep

from lerobot.processor import PolicyProcessorPipeline
from lerobot.utils.constants import OBS_STATE


def _stats():
    return {
        OBS_STATE: {
            "mean": torch.arange(30, dtype=torch.float32),
            "std": np.arange(30, dtype=np.float32),
            "count": 100,
        },
        "action": {"mean": torch.arange(6, dtype=torch.float32)},
    }


def test_positions_only_stats_copies_and_slices_only_state():
    original = _stats()
    narrowed = positions_only_stats(original)
    assert narrowed[OBS_STATE]["mean"].shape == (6,)
    assert narrowed[OBS_STATE]["std"].shape == (6,)
    assert narrowed[OBS_STATE]["count"] == 100
    assert narrowed["action"]["mean"].shape == (6,)
    assert original[OBS_STATE]["mean"].shape == (30,)


def test_positions_only_metadata_does_not_mutate_source():
    meta = SimpleNamespace(
        info=SimpleNamespace(
            features={OBS_STATE: {"dtype": "float32", "shape": [30], "names": list(range(30))}}
        ),
        stats=_stats(),
    )
    narrowed = positions_only_metadata(meta)
    assert narrowed.info.features[OBS_STATE]["shape"] == [6]
    assert narrowed.info.features[OBS_STATE]["names"] == list(range(6))
    assert narrowed.stats[OBS_STATE]["mean"].shape == (6,)
    assert meta.info.features[OBS_STATE]["shape"] == [30]
    assert meta.stats[OBS_STATE]["mean"].shape == (30,)


def test_install_patches_policy_view_and_prepends_step():
    captured = {}

    def fake_make_policy(*, cfg, ds_meta=None, **kwargs):
        captured["meta"] = ds_meta
        return "policy"

    def fake_make_processors(*args, **kwargs):
        captured["stats"] = kwargs["dataset_stats"]
        captured["override_stats"] = kwargs["preprocessor_overrides"]["normalizer_processor"]["stats"]
        return PolicyProcessorPipeline(steps=[]), PolicyProcessorPipeline(steps=[])

    module = SimpleNamespace(
        make_policy=fake_make_policy,
        make_pre_post_processors=fake_make_processors,
    )
    install_positions_only_training(module)
    meta = SimpleNamespace(
        info=SimpleNamespace(features={OBS_STATE: {"dtype": "float32", "shape": [30]}}),
        stats=_stats(),
    )
    assert module.make_policy(cfg=object(), ds_meta=meta) == "policy"
    preprocessor, _ = module.make_pre_post_processors(
        dataset_stats=meta.stats,
        preprocessor_overrides={"normalizer_processor": {"stats": meta.stats}},
    )

    assert captured["meta"].info.features[OBS_STATE]["shape"] == [6]
    assert captured["stats"][OBS_STATE]["mean"].shape == (6,)
    assert captured["override_stats"][OBS_STATE]["mean"].shape == (6,)
    assert isinstance(preprocessor.steps[0], TruncateStateStep)
    assert preprocessor.steps[0].keep == 6


def test_training_refuses_missing_camera_rename_map():
    module = SimpleNamespace(
        make_policy=lambda **kwargs: "policy",
        make_pre_post_processors=lambda *args, **kwargs: (None, None),
    )
    install_positions_only_training(module)
    meta = SimpleNamespace(
        info=SimpleNamespace(
            features={
                OBS_STATE: {"dtype": "float32", "shape": [30]},
                "observation.images.wrist": {"dtype": "video", "shape": [3, 8, 8]},
                "observation.images.overhead": {"dtype": "video", "shape": [3, 8, 8]},
            }
        ),
        stats=_stats(),
    )
    with pytest.raises(ValueError, match="requires --rename_map"):
        module.make_policy(cfg=object(), ds_meta=meta, rename_map={})


def test_truncation_step_survives_checkpoint_round_trip(tmp_path):
    pipeline = PolicyProcessorPipeline(steps=[TruncateStateStep(keep=6)])
    pipeline.save_pretrained(tmp_path, config_filename="policy_preprocessor.json")
    loaded = PolicyProcessorPipeline.from_pretrained(
        tmp_path,
        config_filename="policy_preprocessor.json",
        local_files_only=True,
    )
    output = loaded({OBS_STATE: torch.arange(30, dtype=torch.float32)})
    assert output[OBS_STATE].shape == (6,)
    assert torch.equal(output[OBS_STATE], torch.arange(6, dtype=torch.float32))


def test_nondefault_keep_is_serialized_not_replaced_by_default(tmp_path):
    pipeline = PolicyProcessorPipeline(steps=[TruncateStateStep(keep=5)])
    pipeline.save_pretrained(tmp_path, config_filename="policy_preprocessor.json")
    loaded = PolicyProcessorPipeline.from_pretrained(
        tmp_path,
        config_filename="policy_preprocessor.json",
        local_files_only=True,
    )
    assert isinstance(loaded.steps[0], TruncateStateStep)
    assert loaded.steps[0].keep == 5
