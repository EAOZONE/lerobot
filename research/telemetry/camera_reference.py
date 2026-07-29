#!/usr/bin/env python
"""List cameras and capture immutable reference/session views with manifests."""

import argparse
import hashlib
import json
import time
from pathlib import Path

import cv2


def camera_name(device: Path) -> str:
    sysfs = Path("/sys/class/video4linux") / device.name / "name"
    return sysfs.read_text().strip() if sysfs.exists() else "unknown"


def list_devices() -> None:
    for device in sorted(Path("/dev").glob("video*")):
        capture = cv2.VideoCapture(str(device))
        opened = capture.isOpened()
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)) if opened else 0
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)) if opened else 0
        fps = capture.get(cv2.CAP_PROP_FPS) if opened else 0
        capture.release()
        print(f"{device}: {camera_name(device)} capture={opened} {width}x{height}@{fps:g}")


def capture_frame(device: str, width: int, height: int, fps: float):
    camera = cv2.VideoCapture(device)
    if not camera.isOpened():
        raise RuntimeError(f"cannot open {device}")
    try:
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        camera.set(cv2.CAP_PROP_FPS, fps)
        frame = None
        for _ in range(10):
            ok, candidate = camera.read()
            if ok:
                frame = candidate
        if frame is None:
            raise RuntimeError(f"{device} returned no frame")
        return frame
    finally:
        camera.release()


def write_capture(args, *, reference: bool) -> None:
    if len(args.camera) != 2 or any("=" not in value for value in args.camera):
        raise SystemExit("provide exactly two --camera NAME=/dev/videoN arguments")
    destination = args.out
    destination.mkdir(parents=True, exist_ok=True)
    manifest = {
        "schema_version": 1,
        "kind": "reference" if reference else "session_check",
        "session": args.session,
        "wall_time_ns": time.time_ns(),
        "operator_verdict": getattr(args, "verdict", None),
        "cameras": {},
    }
    for assignment in args.camera:
        name, device = assignment.split("=", 1)
        frame = capture_frame(device, args.width, args.height, args.fps)
        image_path = destination / f"{args.session}_{name}.png"
        if not cv2.imwrite(str(image_path), frame):
            raise RuntimeError(f"failed to write {image_path}")
        manifest["cameras"][name] = {
            "device": device,
            "device_name": camera_name(Path(device)),
            "width": int(frame.shape[1]),
            "height": int(frame.shape[0]),
            "fps_requested": args.fps,
            "image": image_path.name,
            "sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
        }
    manifest_path = destination / f"{args.session}_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(f"Captured {manifest['kind']} views and manifest: {manifest_path}")


def add_capture_arguments(parser) -> None:
    parser.add_argument("--camera", action="append", required=True, help="NAME=/dev/videoN; exactly two")
    parser.add_argument("--session", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=30)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("list", help="enumerate camera nodes without saving images")
    reference = sub.add_parser("capture", help="capture the taped-mount reference views")
    add_capture_arguments(reference)
    check = sub.add_parser("check", help="capture session-start views and log the operator verdict")
    add_capture_arguments(check)
    check.add_argument("--verdict", required=True, choices=("pass", "fail"))
    args = parser.parse_args()
    if args.command == "list":
        list_devices()
    else:
        write_capture(args, reference=args.command == "capture")


if __name__ == "__main__":
    main()
