#!/usr/bin/env python
"""Audit a recording timing sidecar against the one-control-step contract."""

import argparse
import json
import math
from pathlib import Path


def load_rows(path: Path) -> list[dict[str, object]]:
    rows = []
    with path.open() as fh:
        for line_number, line in enumerate(fh, 1):
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_number}: invalid JSON: {exc}") from exc
    if not rows:
        raise ValueError(f"{path}: empty sidecar")
    return rows


def audit(rows: list[dict[str, object]], fps: float) -> list[str]:
    if fps <= 0 or not math.isfinite(fps):
        raise ValueError("fps must be finite and positive")
    step_ns = 1e9 / fps
    errors = []
    expected_frames = list(range(len(rows)))
    actual_frames = [row.get("frame_index") for row in rows]
    if actual_frames != expected_frames:
        errors.append(f"frame indices are not contiguous: {actual_frames}")
    episode_ids = {row.get("episode_index") for row in rows}
    if len(episode_ids) != 1:
        errors.append(f"sidecar contains multiple episode indices: {sorted(episode_ids)}")

    previous_capture: dict[str, int] = {}
    previous_observation = None
    for row in rows:
        frame = row.get("frame_index")
        start = int(row["observation_start_ns"])
        telemetry_end = int(row["telemetry_read_end_ns"])
        if previous_observation is not None and start - previous_observation > 2 * step_ns:
            errors.append(f"frame {frame}: control-loop gap exceeds two steps")
        previous_observation = start
        cameras = row.get("cameras")
        if not isinstance(cameras, dict) or not cameras:
            errors.append(f"frame {frame}: no camera timing")
            continue
        for name, raw in cameras.items():
            capture = raw.get("capture_ns") if isinstance(raw, dict) else None
            returned = raw.get("returned_ns") if isinstance(raw, dict) else None
            if capture is None or returned is None:
                errors.append(f"frame {frame}/{name}: missing capture or return time")
                continue
            capture = int(capture)
            returned = int(returned)
            if abs(capture - telemetry_end) > step_ns:
                errors.append(f"frame {frame}/{name}: telemetry-to-frame delta exceeds one step")
            if returned - capture > step_ns:
                errors.append(f"frame {frame}/{name}: frame was older than one step when read")
            if name in previous_capture:
                gap = capture - previous_capture[name]
                if gap <= 0:
                    errors.append(f"frame {frame}/{name}: duplicate or non-monotonic capture timestamp")
                elif gap > 2 * step_ns:
                    errors.append(f"frame {frame}/{name}: capture gap exceeds two steps")
            previous_capture[name] = capture
    return errors


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sidecars", nargs="+", type=Path)
    parser.add_argument("--fps", type=float, default=30.0)
    args = parser.parse_args()
    failed = False
    for path in args.sidecars:
        try:
            rows = load_rows(path)
            errors = audit(rows, args.fps)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            errors = [str(exc)]
        if errors:
            failed = True
            print(f"FAIL {path} ({len(errors)} issue(s))")
            for error in errors[:20]:
                print(f"  - {error}")
        else:
            print(f"PASS {path}: {len(rows)} frames aligned within {1000 / args.fps:.1f} ms")
    raise SystemExit(1 if failed else 0)


if __name__ == "__main__":
    main()
