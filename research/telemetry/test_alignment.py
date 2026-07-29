"""Offline tests for capture-timing sidecars and their strict audit."""

import json
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).parent))

from alignment_sidecar import AlignmentSidecarRecorder, record_alignment_sidecars  # noqa: E402
from audit_alignment import audit, load_rows  # noqa: E402
from so_follower_telemetry import SOFollowerTelemetry  # noqa: E402

from lerobot.datasets import LeRobotDataset  # noqa: E402


def timing(frame: int) -> dict[str, object]:
    start = 1_000_000_000 + frame * 33_000_000
    return {
        "schema_version": 1,
        "wall_time_ns": start,
        "observation_start_ns": start,
        "position_read_end_ns": start + 1_000_000,
        "telemetry_read_end_ns": start + 2_000_000,
        "observation_end_ns": start + 4_000_000,
        "cameras": {
            "wrist": {"capture_ns": start + 1_500_000, "returned_ns": start + 3_000_000},
            "overhead": {"capture_ns": start + 1_600_000, "returned_ns": start + 4_000_000},
        },
    }


def test_sidecar_round_trip_and_alignment_pass(tmp_path) -> None:
    recorder = AlignmentSidecarRecorder()
    dataset = SimpleNamespace(root=tmp_path, num_episodes=3)
    robot = SimpleNamespace(last_capture_timing=None)
    SOFollowerTelemetry.latest_instance = robot
    for frame in range(3):
        robot.last_capture_timing = timing(frame)
        recorder.add_frame(dataset)
    path = recorder.save_episode(dataset, 3)
    rows = load_rows(path)
    assert [row["frame_index"] for row in rows] == [0, 1, 2]
    assert audit(rows, 30) == []


def test_discard_removes_rerecorded_attempt(tmp_path) -> None:
    recorder = AlignmentSidecarRecorder()
    dataset = SimpleNamespace(root=tmp_path, num_episodes=0)
    SOFollowerTelemetry.latest_instance = SimpleNamespace(last_capture_timing=timing(0))
    recorder.add_frame(dataset)
    recorder.discard_episode()
    assert recorder.rows == []


def test_audit_rejects_duplicate_and_stale_camera_frame() -> None:
    rows = []
    for frame in range(2):
        row = timing(frame)
        row.update(episode_index=0, frame_index=frame)
        rows.append(row)
    rows[1]["cameras"]["wrist"]["capture_ns"] = rows[0]["cameras"]["wrist"]["capture_ns"]
    errors = audit(rows, 30)
    assert any("duplicate" in error for error in errors)
    assert any("older than one step" in error for error in errors)


def test_saved_jsonl_is_plain_machine_readable_json(tmp_path) -> None:
    recorder = AlignmentSidecarRecorder()
    dataset = SimpleNamespace(root=tmp_path, num_episodes=0)
    SOFollowerTelemetry.latest_instance = SimpleNamespace(last_capture_timing=timing(0))
    recorder.add_frame(dataset)
    path = recorder.save_episode(dataset, 0)
    assert json.loads(path.read_text().strip())["schema_version"] == 1


def test_recording_context_commits_and_discards_with_dataset_lifecycle(tmp_path, monkeypatch) -> None:
    dataset = SimpleNamespace(root=tmp_path, num_episodes=0, frames=[])
    SOFollowerTelemetry.latest_instance = SimpleNamespace(last_capture_timing=timing(0))

    monkeypatch.setattr(LeRobotDataset, "add_frame", lambda ds, frame: ds.frames.append(frame))
    monkeypatch.setattr(
        LeRobotDataset, "save_episode", lambda ds, episode_data=None, parallel_encoding=True: None
    )
    monkeypatch.setattr(
        LeRobotDataset, "clear_episode_buffer", lambda ds, delete_images=True: ds.frames.clear()
    )
    with record_alignment_sidecars() as recorder:
        LeRobotDataset.add_frame(dataset, {"observation.state": [0] * 30})
        LeRobotDataset.clear_episode_buffer(dataset)
        assert recorder.rows == []
        LeRobotDataset.add_frame(dataset, {"observation.state": [0] * 30})
        LeRobotDataset.save_episode(dataset)

    sidecar = tmp_path / "meta" / "alignment" / "episode_000000.jsonl"
    assert sidecar.exists()
    assert load_rows(sidecar)[0]["frame_index"] == 0
