#!/usr/bin/env python
"""Supervised SO-101 recovery validation with a live command ring buffer.

Teleoperate to a representative task pose, then press ``r``. The program halts,
replays recent commands in reverse, traverses a physically validated waypoint
route, replays the recorded home path, opens the gripper, and logs every phase.
This command moves hardware and must only be run with an operator at the power switch.
"""

import argparse
import contextlib
import csv
import select
import sys
import termios
import time
import tty
from collections import deque
from pathlib import Path

from recovery import execute_recovery, load_recovery_config, load_trajectory

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig


@contextlib.contextmanager
def raw_keys():
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


def key_pressed() -> str | None:
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1).lower()
    return None


class ValidationLogger:
    def __init__(self, path: Path, motors: list[str]):
        self.path = path
        self.motors = motors
        self.started = time.perf_counter()
        self.fh = path.open("w", newline="")
        fields = ["t", "phase", "frame_idx"]
        fields += [f"goal_pos.{motor}" for motor in motors]
        for field in ("pos", "vel", "load", "curr", "volt"):
            fields += [f"{field}.{motor}" for motor in motors]
        self.writer = csv.DictWriter(self.fh, fieldnames=fields)
        self.writer.writeheader()

    def __call__(self, phase, frame_idx, goal, telemetry) -> None:
        row = {"t": round(time.perf_counter() - self.started, 4), "phase": phase, "frame_idx": frame_idx}
        row.update({f"goal_pos.{motor}": goal[motor] for motor in self.motors})
        for field in ("pos", "vel", "load", "curr", "volt"):
            row.update({f"{field}.{motor}": telemetry[field][motor] for motor in self.motors})
        self.writer.writerow(row)
        self.fh.flush()

    def close(self) -> None:
        self.fh.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--follower-port", required=True)
    parser.add_argument("--leader-port", required=True)
    parser.add_argument("--home", type=Path, required=True)
    parser.add_argument("--waypoints", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--history-seconds", type=float, default=2.0)
    parser.add_argument("--follower-id", default="follower")
    parser.add_argument("--leader-id", default="leader")
    parser.add_argument(
        "--supervised-trial-route",
        help="name of one unvalidated route approved for this attended trial only",
    )
    args = parser.parse_args()
    if args.fps <= 0 or args.history_seconds <= 0:
        parser.error("--fps and --history-seconds must be positive")

    config = load_recovery_config(args.waypoints)
    home, _ = load_trajectory(args.home)
    follower = SO101Follower(SO101FollowerConfig(port=args.follower_port, id=args.follower_id))
    leader = SO101Leader(SO101LeaderConfig(port=args.leader_port, id=args.leader_id))
    follower.connect()
    leader.connect()
    motors = list(follower.bus.motors)
    history = deque(maxlen=max(1, round(args.history_seconds * args.fps)))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    logger = ValidationLogger(args.out, motors)
    print("Teleoperate only in a clear workspace. Press 'r' to recover; Ctrl-C aborts and halts.")
    period = 1 / args.fps
    try:
        with raw_keys():
            while True:
                started = time.perf_counter()
                follower.send_action(leader.get_action())
                # Read the register back so the ring buffer contains commands actually accepted by the follower.
                history.append(follower.bus.sync_read("Goal_Position", normalize=False))
                if key_pressed() == "r":
                    break
                time.sleep(max(0.0, period - (time.perf_counter() - started)))
        result = execute_recovery(
            follower.bus,
            list(history),
            home,
            config,
            fps=args.fps,
            flush_action_queue=lambda: None,
            on_frame=logger,
            supervised_trial_route=args.supervised_trial_route,
        )
        print(
            f"Recovery complete via {result.route}: {result.reverse_frames} reverse frames, "
            f"{result.completed_frames} monitored frames. Log: {args.out}"
        )
    except KeyboardInterrupt:
        from recovery import halt

        print("\nInterrupted; holding measured pose.")
        halt(follower.bus)
    finally:
        logger.close()
        leader.disconnect()
        follower.bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
