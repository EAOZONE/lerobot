import json
from pathlib import Path

import pyarrow as pa
import pyarrow.parquet as pq
from audit_corpus import ACTION_KEY, ACTION_NAMES, CAMERA_KEYS, STATE_KEY, STATE_NAMES, audit_dataset


def _write_dataset(root: Path, *, state_width: int = 30, cameras=("wrist", "overhead")) -> None:
    (root / "meta" / "alignment").mkdir(parents=True)
    (root / "data" / "chunk-000").mkdir(parents=True)
    features = {
        STATE_KEY: {"dtype": "float32", "shape": [30], "names": STATE_NAMES},
        ACTION_KEY: {"dtype": "float32", "shape": [6], "names": ACTION_NAMES},
        CAMERA_KEYS[0]: {"dtype": "video", "shape": [3, 8, 8]},
        CAMERA_KEYS[1]: {"dtype": "video", "shape": [3, 8, 8]},
    }
    info = {"fps": 30, "total_episodes": 1, "total_frames": 2, "features": features}
    (root / "meta" / "info.json").write_text(json.dumps(info))
    stats = {
        key: {name: [0.0] * width for name in ("mean", "std", "min", "max")}
        for key, width in ((STATE_KEY, 30), (ACTION_KEY, 6))
    }
    (root / "meta" / "stats.json").write_text(json.dumps(stats))
    table = pa.table(
        {
            STATE_KEY: [[0.0] * state_width, [1.0] * state_width],
            ACTION_KEY: [[0.0] * 6, [1.0] * 6],
        }
    )
    pq.write_table(table, root / "data" / "chunk-000" / "file-000.parquet")
    for key in CAMERA_KEYS:
        directory = root / "videos" / key / "chunk-000"
        directory.mkdir(parents=True)
        (directory / "file-000.mp4").write_bytes(b"placeholder")

    rows = []
    for frame in range(2):
        start = 1_000_000_000 + frame * 33_000_000
        rows.append(
            {
                "episode_index": 0,
                "frame_index": frame,
                "observation_start_ns": start,
                "telemetry_read_end_ns": start + 2_000_000,
                "cameras": {
                    camera: {"capture_ns": start + 1_000_000, "returned_ns": start + 2_000_000}
                    for camera in cameras
                },
            }
        )
    sidecar = root / "meta" / "alignment" / "episode_000000.jsonl"
    sidecar.write_text("".join(json.dumps(row) + "\n" for row in rows))


def test_complete_dataset_passes(tmp_path):
    _write_dataset(tmp_path)
    summary, errors = audit_dataset(tmp_path, expected_episodes=1)
    assert not errors
    assert summary.frames == 2
    assert summary.sidecars == 1


def test_stored_width_is_checked_not_only_metadata(tmp_path):
    _write_dataset(tmp_path, state_width=29)
    _, errors = audit_dataset(tmp_path)
    assert any("row 0 observation.state width is not 30" in error for error in errors)


def test_missing_camera_timing_and_sidecar_count_fail(tmp_path):
    _write_dataset(tmp_path, cameras=("wrist",))
    _, errors = audit_dataset(tmp_path, expected_episodes=2)
    assert any("cameras must be wrist+overhead" in error for error in errors)
    assert any("expected 2 episodes" in error for error in errors)
