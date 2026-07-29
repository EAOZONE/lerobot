import json
from pathlib import Path

import torch
from audit_positions_checkpoint import CAMERA_RENAME_MAP, audit_checkpoint
from safetensors.torch import save_file


def _write_checkpoint(root: Path, *, keep=6, state_width=6, rename_map=None):
    root.mkdir()
    policy = {
        "type": "smolvla",
        "chunk_size": 50,
        "n_action_steps": 50,
        "input_features": {
            "observation.state": {"type": "STATE", "shape": [6]},
            "observation.images.camera1": {"type": "VISUAL", "shape": [3, 8, 8]},
            "observation.images.camera2": {"type": "VISUAL", "shape": [3, 8, 8]},
        },
        "output_features": {"action": {"type": "ACTION", "shape": [6]}},
    }
    state_file = "policy_preprocessor_step_2_normalizer_processor.safetensors"
    processor = {
        "name": "policy_preprocessor",
        "steps": [
            {"registry_name": "truncate_state", "config": {"keep": keep}},
            {
                "registry_name": "rename_observations_processor",
                "config": {"rename_map": CAMERA_RENAME_MAP if rename_map is None else rename_map},
            },
            {
                "registry_name": "normalizer_processor",
                "config": {
                    "features": {
                        "observation.state": {"type": "STATE", "shape": [6]},
                        "action": {"type": "ACTION", "shape": [6]},
                    }
                },
                "state_file": state_file,
            },
        ],
    }
    (root / "config.json").write_text(json.dumps(policy))
    (root / "policy_preprocessor.json").write_text(json.dumps(processor))
    save_file(
        {
            "observation.state.mean": torch.zeros(state_width),
            "observation.state.std": torch.ones(state_width),
            "observation.state.min": torch.zeros(state_width),
            "observation.state.max": torch.ones(state_width),
            "observation.state.count": torch.ones(1),
            "action.mean": torch.zeros(6),
        },
        root / state_file,
    )
    (root / "model.safetensors").write_bytes(b"model-present")


def test_eligible_checkpoint_passes(tmp_path):
    checkpoint = tmp_path / "checkpoint"
    _write_checkpoint(checkpoint)
    summary, errors = audit_checkpoint(checkpoint)
    assert not errors
    assert summary.chunk_size == 50
    assert summary.state_stat_tensors == 5


def test_wrong_truncation_and_state_stats_fail(tmp_path):
    checkpoint = tmp_path / "checkpoint"
    _write_checkpoint(checkpoint, keep=30, state_width=30)
    _, errors = audit_checkpoint(checkpoint)
    assert any("keep=6" in error for error in errors)
    assert any("must have shape (6,)" in error for error in errors)


def test_missing_camera_mapping_fails(tmp_path):
    checkpoint = tmp_path / "checkpoint"
    _write_checkpoint(checkpoint, rename_map={})
    _, errors = audit_checkpoint(checkpoint)
    assert any("camera rename map" in error for error in errors)
