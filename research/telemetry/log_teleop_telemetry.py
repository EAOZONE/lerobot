#!/usr/bin/env python
"""Week 1, Day 3: the signature test that decides the project.

Teleoperates the SO-101 (leader -> follower) while logging follower telemetry to a
CSV on shared timestamps. Deliberately collide; deliberately slip a grasp; mark each
event as it happens. Then plot with plot_signatures.py and eyeball it.

Telemetry is pulled with a single block read of the contiguous SRAM region
(addr 56..70), so all fields in a row come from ONE bus transaction and share a
timestamp -- not four staggered reads. That property is what makes onset labeling
meaningful later.

A live grip readout is printed while recording: slip runs are only meaningful if the
gripper was actually squeezing something first, and discovering otherwise in post-hoc
analysis wastes the whole batch.

Usage:
    # teleoperated
    python research/telemetry/log_teleop_telemetry.py \
        --follower-port /dev/ttyACM0 --leader-port /dev/ttyACM1 \
        --out research/telemetry/runs/slip_a.csv

    # matched-trajectory: replay a recorded episode instead of teleoperating, so a
    # with-obstacle run and a clear run share a commanded trajectory sample-for-sample
    python research/telemetry/log_teleop_telemetry.py \
        --follower-port /dev/ttyACM0 \
        --replay-dataset ${HF_USER}/sweep --replay-episode 0 \
        --out research/telemetry/runs/pair1_obstacle.csv

    # sanity-check the block read against per-register sync_read first:
    python research/telemetry/log_teleop_telemetry.py ... --verify

Mark events with a single keypress (no Enter). Ctrl-C to stop.
"""

import argparse
import contextlib
import csv
import select
import sys
import termios
import time
import tty
from pathlib import Path

from feetech_block import BLOCK_FIELDS, block_read, verify_block_read

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

# Gripper current while genuinely squeezing an object. In the first nine-run batch,
# holding runs sat at 198-262 sustained; slipped/empty runs peaked at 112. 150 is the
# midpoint of that gap -- 200 would flag the weakest genuine grasp as NO GRIP.
GRIP_OK_CURRENT = 150


# Single keystrokes, so a marker costs reaction time only. Typing a word and pressing
# Enter stamps the marker 3-4s after the event -- far wider than the event itself, and
# wide enough that normal teleop motion cannot be excluded from the window.
MARKER_KEYS = {
    "c": "collision",
    "s": "slip",
    "k": "clean",
    "g": "grasp",
    "r": "release",
    "x": "other",
}


@contextlib.contextmanager
def raw_keys():
    """Put the terminal in cbreak mode so a single keypress arrives without Enter."""
    if not sys.stdin.isatty():
        yield
        return
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)


def poll_marker() -> str | None:
    """Non-blocking single-key read, so marking an event never stalls the loop."""
    if select.select([sys.stdin], [], [], 0)[0]:
        key = sys.stdin.read(1).lower()
        return MARKER_KEYS.get(key)
    return None


def load_replay_actions(repo_id: str, root: Path | None, episode: int) -> list[dict[str, float]]:
    """Read one episode's action column, as {motor}.pos dicts.

    Mirrors lerobot_replay.py:109-123. Only the action column is touched, so a dataset
    recorded with extra telemetry columns replays identically to a plain one.
    """
    from lerobot.datasets import LeRobotDataset
    from lerobot.utils.constants import ACTION

    dataset = LeRobotDataset(repo_id, root=root, episodes=[episode])
    names = dataset.features[ACTION]["names"]
    column = dataset.select_columns(ACTION)
    return [
        {name: float(column[i][ACTION][j]) for j, name in enumerate(names)} for i in range(dataset.num_frames)
    ]


def grip_status(telemetry: dict[str, dict[str, int]]) -> str:
    """One-line grip readout.

    The first slip batch was wasted because the object was never firmly gripped, and
    that only became visible in post-hoc analysis. Showing grip force live means the
    operator can confirm a real grasp before initiating the slip.
    """
    curr = telemetry["curr"]["gripper"]
    load = telemetry["load"]["gripper"]
    flag = "GRIP OK  " if curr >= GRIP_OK_CURRENT else "NO GRIP  "
    bar = "#" * min(30, curr * 30 // 600)
    return f"{flag} curr={curr:4d} load={load:5d} |{bar:<30}|"


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--follower-port", required=True)
    parser.add_argument("--leader-port", help="required unless --replay-dataset is given")
    parser.add_argument("--id-follower", default="signature_follower")
    parser.add_argument("--id-leader", default="signature_leader")
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--verify", action="store_true", help="check block read, then exit")
    parser.add_argument(
        "--replay-dataset",
        help="repo_id of a dataset to replay instead of teleoperating. The episode's "
        "action column drives the arm from THIS process -- do not run lerobot-replay "
        "alongside, two processes cannot share the serial port.",
    )
    parser.add_argument("--replay-root", type=Path, default=None, help="local dataset root")
    parser.add_argument("--replay-episode", type=int, default=0)
    args = parser.parse_args()

    replaying = args.replay_dataset is not None
    if not replaying and not args.leader_port:
        parser.error("--leader-port is required unless --replay-dataset is given")

    follower = SO101Follower(SO101FollowerConfig(port=args.follower_port, id=args.id_follower))
    follower.connect()

    if args.verify:
        try:
            if not verify_block_read(follower.bus):
                sys.exit(1)
        finally:
            follower.disconnect()
        return

    leader = None
    actions: list[dict[str, float]] = []
    if replaying:
        actions = load_replay_actions(args.replay_dataset, args.replay_root, args.replay_episode)
        print(f"Replaying {len(actions)} frames from {args.replay_dataset} ep{args.replay_episode}")
    else:
        leader = SO101Leader(SO101LeaderConfig(port=args.leader_port, id=args.id_leader))
        leader.connect()

    motors = list(follower.bus.motors)
    columns = (
        ["t", "marker", "frame_idx"]
        + [f"goal_pos.{m}" for m in motors]
        + [f"{field}.{m}" for field in BLOCK_FIELDS for m in motors]
    )

    args.out.parent.mkdir(parents=True, exist_ok=True)
    period = 1.0 / args.fps
    n = 0

    keys = "  ".join(f"[{k}] {v}" for k, v in MARKER_KEYS.items())
    print(f"\nLogging to {args.out} at {args.fps:.0f} Hz. Ctrl-C to stop.")
    print(f"Mark with a SINGLE keypress (no Enter):  {keys}")
    print("Hit the key AS the event happens, not after -- the timestamp is the label.\n")
    with raw_keys(), args.out.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=columns)
        writer.writeheader()
        t_start = time.perf_counter()
        next_status = 0.0
        try:
            while True:
                loop_start = time.perf_counter()
                t = loop_start - t_start

                if replaying:
                    if n >= len(actions):
                        print(f"\nReplay finished ({n} frames).")
                        break
                    action = actions[n]
                else:
                    action = leader.get_action()
                follower.send_action(action)
                telemetry = block_read(follower.bus)

                marker = poll_marker()
                if marker:
                    # \r\n, not \n: the terminal is in cbreak mode so it does not
                    # translate newlines and output would stair-step otherwise.
                    print(f"\r  [{t:6.2f}s] marker: {marker}\r\n", end="")

                row = {"t": round(t, 4), "marker": marker or "", "frame_idx": n}
                for m in motors:
                    row[f"goal_pos.{m}"] = action[f"{m}.pos"]
                for field in BLOCK_FIELDS:
                    for m in motors:
                        row[f"{field}.{m}"] = telemetry[field][m]
                writer.writerow(row)
                n += 1

                if t >= next_status:
                    print(f"\r{t:6.1f}s  {grip_status(telemetry)}", end="", flush=True)
                    next_status = t + 0.2

                time.sleep(max(0.0, period - (time.perf_counter() - loop_start)))
        except KeyboardInterrupt:
            pass
        finally:
            elapsed = time.perf_counter() - t_start
            print(f"\r\nStopped. {n} rows in {elapsed:.1f}s ({n / max(elapsed, 1e-9):.1f} Hz) -> {args.out}")
            if leader is not None:
                leader.disconnect()
            follower.disconnect()


if __name__ == "__main__":
    main()
