"""Per-episode capture-timing sidecars for the telemetry recording wrapper.

This module deliberately patches only the research wrapper's recording context. It
does not modify LeRobot's dataset schema or the frozen 30-dimensional state vector.
"""

import contextlib
import json
import os
from pathlib import Path

from so_follower_telemetry import SOFollowerTelemetry

from lerobot.datasets import LeRobotDataset


class AlignmentSidecarRecorder:
    def __init__(self) -> None:
        self.rows: list[dict[str, object]] = []

    def add_frame(self, dataset: LeRobotDataset) -> None:
        robot = SOFollowerTelemetry.latest_instance
        if robot is None or robot.last_capture_timing is None:
            raise RuntimeError("telemetry frame was added without capture timing")
        timing = dict(robot.last_capture_timing)
        timing["episode_index"] = int(dataset.num_episodes)
        timing["frame_index"] = len(self.rows)
        self.rows.append(timing)

    def discard_episode(self) -> None:
        self.rows.clear()

    def save_episode(self, dataset: LeRobotDataset, episode_index: int) -> Path:
        if not self.rows:
            raise RuntimeError("cannot save an empty alignment sidecar")
        expected = list(range(len(self.rows)))
        actual = [int(row["frame_index"]) for row in self.rows]
        if actual != expected:
            raise RuntimeError(f"non-contiguous alignment frame indices: {actual}")
        root = Path(dataset.root)
        directory = root / "meta" / "alignment"
        directory.mkdir(parents=True, exist_ok=True)
        destination = directory / f"episode_{episode_index:06d}.jsonl"
        temporary = destination.with_suffix(".jsonl.tmp")
        with temporary.open("w") as fh:
            for row in self.rows:
                fh.write(json.dumps(row, sort_keys=True) + "\n")
            fh.flush()
            os.fsync(fh.fileno())
        temporary.replace(destination)
        self.rows.clear()
        return destination


@contextlib.contextmanager
def record_alignment_sidecars():
    """Attach timing capture to LeRobotDataset only while the research wrapper runs."""
    recorder = AlignmentSidecarRecorder()
    original_add = LeRobotDataset.add_frame
    original_save = LeRobotDataset.save_episode
    original_clear = LeRobotDataset.clear_episode_buffer

    def add_frame(dataset, frame):
        original_add(dataset, frame)
        recorder.add_frame(dataset)

    def save_episode(dataset, episode_data=None, parallel_encoding=True):
        episode_index = int(dataset.num_episodes)
        original_save(dataset, episode_data, parallel_encoding)
        recorder.save_episode(dataset, episode_index)

    def clear_episode_buffer(dataset, delete_images=True):
        original_clear(dataset, delete_images)
        recorder.discard_episode()

    LeRobotDataset.add_frame = add_frame
    LeRobotDataset.save_episode = save_episode
    LeRobotDataset.clear_episode_buffer = clear_episode_buffer
    try:
        yield recorder
    finally:
        LeRobotDataset.add_frame = original_add
        LeRobotDataset.save_episode = original_save
        LeRobotDataset.clear_episode_buffer = original_clear
