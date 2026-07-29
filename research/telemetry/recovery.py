#!/usr/bin/env python
"""Halt an SO-101 safely or replay a recorded reset trajectory.

The trajectory is a CSV produced by ``log_teleop_telemetry.py``. Recovery uses the
raw ``pos.<motor>`` columns rather than the normalized command columns, so it does
not depend on a particular LeRobot calibration file being selected at runtime.

Legacy examples (experimental lift-first path only):
    python research/telemetry/recovery.py halt --port /dev/ttyACM0

    python research/telemetry/recovery.py legacy-reset --port /dev/ttyACM0 \
        --home research/telemetry/runs/reset_home.csv --retract-frames 30

    python research/telemetry/recovery.py legacy-reset --port /dev/ttyACM0 \
        --home research/telemetry/runs/reset_home.csv --repeat 10 \
        --log research/telemetry/runs/reset_soak.csv \
        --i-understand-lift-first-is-unsafe

Use ``validate_recovery.py`` for the replacement reverse-replay/waypoint path. Neither
command disables torque on exit: halt holds present pose, and recovery holds home.
"""

import argparse
import csv
import json
import math
import time
from collections.abc import Callable, Iterator, Sequence
from dataclasses import dataclass
from pathlib import Path

from feetech_block import BLOCK_FIELDS, block_read
from probe_bus import SO101_MOTORS, assert_no_alarms

from lerobot.motors.feetech import FeetechMotorsBus

DEFAULT_FPS = 30.0
RAW_POSITION_MIN = 0
RAW_POSITION_MAX = 4095

RawPose = dict[str, int]


class RecoveryAbortError(RuntimeError):
    """A monitored recovery stopped and is holding its last measured pose."""


@dataclass(frozen=True)
class RecoveryLimits:
    """Explicit fail-closed limits loaded from the validated waypoint file."""

    joint_min: RawPose
    joint_max: RawPose
    max_current: RawPose
    max_following_error: RawPose
    max_step_ticks: int
    phase_timeout_s: float
    reverse_frames: int
    fault_guard_frames: int
    gripper_open: int


@dataclass(frozen=True)
class RecoveryRoute:
    name: str
    region_min: RawPose
    region_max: RawPose
    waypoints: tuple[RawPose, ...]
    validated: bool


@dataclass(frozen=True)
class RecoveryConfig:
    schema_version: int
    limits: RecoveryLimits
    routes: tuple[RecoveryRoute, ...]


@dataclass(frozen=True)
class RecoveryResult:
    route: str
    reverse_frames: int
    completed_frames: int


def _raw_pose(value: object, label: str) -> RawPose:
    if not isinstance(value, dict) or set(value) != set(SO101_MOTORS):
        raise ValueError(f"{label} must contain exactly these motors: {', '.join(SO101_MOTORS)}")
    pose = {motor: int(value[motor]) for motor in SO101_MOTORS}
    outside = {motor: val for motor, val in pose.items() if not RAW_POSITION_MIN <= val <= RAW_POSITION_MAX}
    if outside:
        raise ValueError(f"{label} contains positions outside 0..4095: {outside}")
    return pose


def load_recovery_config(path: Path) -> RecoveryConfig:
    """Load a versioned, explicit waypoint and safety-limit configuration."""
    try:
        data = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"Cannot load recovery configuration {path}: {exc}") from exc
    if data.get("schema_version") != 1:
        raise ValueError(f"{path}: unsupported schema_version (expected 1)")
    raw_limits = data.get("limits")
    if not isinstance(raw_limits, dict):
        raise ValueError(f"{path}: limits must be an object")
    limits = RecoveryLimits(
        joint_min=_raw_pose(raw_limits.get("joint_min"), "limits.joint_min"),
        joint_max=_raw_pose(raw_limits.get("joint_max"), "limits.joint_max"),
        max_current=_raw_pose(raw_limits.get("max_current"), "limits.max_current"),
        max_following_error=_raw_pose(raw_limits.get("max_following_error"), "limits.max_following_error"),
        max_step_ticks=int(raw_limits.get("max_step_ticks", 0)),
        phase_timeout_s=float(raw_limits.get("phase_timeout_s", 0)),
        reverse_frames=int(raw_limits.get("reverse_frames", 0)),
        fault_guard_frames=int(raw_limits.get("fault_guard_frames", -1)),
        gripper_open=int(raw_limits.get("gripper_open", -1)),
    )
    if limits.max_step_ticks <= 0 or limits.phase_timeout_s <= 0 or limits.reverse_frames <= 0:
        raise ValueError(f"{path}: step, timeout, and reverse limits must be positive")
    if limits.fault_guard_frames < 0 or not RAW_POSITION_MIN <= limits.gripper_open <= RAW_POSITION_MAX:
        raise ValueError(f"{path}: invalid fault_guard_frames or gripper_open")
    for motor in SO101_MOTORS:
        if limits.joint_min[motor] >= limits.joint_max[motor]:
            raise ValueError(f"{path}: invalid joint range for {motor}")

    routes = []
    for idx, raw in enumerate(data.get("routes", [])):
        routes.append(
            RecoveryRoute(
                name=str(raw["name"]),
                region_min=_raw_pose(raw["region_min"], f"routes[{idx}].region_min"),
                region_max=_raw_pose(raw["region_max"], f"routes[{idx}].region_max"),
                waypoints=tuple(
                    _raw_pose(pose, f"routes[{idx}].waypoints[{pose_idx}]")
                    for pose_idx, pose in enumerate(raw.get("waypoints", []))
                ),
                validated=raw.get("validated") is True,
            )
        )
    if not routes:
        raise ValueError(f"{path}: at least one route is required")
    return RecoveryConfig(schema_version=1, limits=limits, routes=tuple(routes))


def select_recovery_route(
    config: RecoveryConfig,
    pose: RawPose,
    *,
    supervised_trial_route: str | None = None,
) -> RecoveryRoute:
    matches = [
        route
        for route in config.routes
        if all(route.region_min[motor] <= pose[motor] <= route.region_max[motor] for motor in SO101_MOTORS)
    ]
    if len(matches) != 1:
        raise RecoveryAbortError(
            f"current pose matches {len(matches)} recovery regions; expected exactly one"
        )
    if not matches[0].validated and matches[0].name != supervised_trial_route:
        raise RecoveryAbortError(f"recovery route '{matches[0].name}' is not physically validated")
    return matches[0]


def bounded_poses(start: RawPose, end: RawPose, max_step_ticks: int) -> Iterator[RawPose]:
    """Interpolate so no motor command changes more than the configured raw-tick limit."""
    largest = max(abs(end[motor] - start[motor]) for motor in SO101_MOTORS)
    frames = max(1, math.ceil(largest / max_step_ticks))
    yield from interpolate_poses(start, end, frames)


def _check_feedback(goal: RawPose, telemetry: dict[str, dict[str, int]], limits: RecoveryLimits) -> None:
    present = telemetry["pos"]
    current = telemetry["curr"]
    for motor in SO101_MOTORS:
        if not limits.joint_min[motor] <= goal[motor] <= limits.joint_max[motor]:
            raise RecoveryAbortError(f"{motor} goal {goal[motor]} violates the configured joint range")
        if abs(current[motor]) > limits.max_current[motor]:
            raise RecoveryAbortError(f"{motor} current {current[motor]} exceeds {limits.max_current[motor]}")
        error = abs(goal[motor] - present[motor])
        if error > limits.max_following_error[motor]:
            raise RecoveryAbortError(
                f"{motor} following error {error} exceeds {limits.max_following_error[motor]}"
            )


def execute_recovery(
    bus: FeetechMotorsBus,
    command_history: Sequence[RawPose],
    home_trajectory: Sequence[RawPose],
    config: RecoveryConfig,
    *,
    fps: float = DEFAULT_FPS,
    flush_action_queue: Callable[[], None],
    on_frame: Callable[[str, int, RawPose, dict[str, dict[str, int]]], None] | None = None,
    on_route: Callable[[str], None] | None = None,
    supervised_trial_route: str | None = None,
) -> RecoveryResult:
    """Execute halt -> reverse replay -> validated waypoints -> home -> open."""
    if not command_history:
        raise ValueError("command history must not be empty")
    if not home_trajectory:
        raise ValueError("home trajectory must not be empty")
    if fps <= 0 or not math.isfinite(fps):
        raise ValueError("fps must be finite and positive")

    current = halt(bus)
    flush_action_queue()
    route = select_recovery_route(config, current, supervised_trial_route=supervised_trial_route)
    if on_route is not None:
        on_route(route.name)
    limits = config.limits
    usable_end = max(0, len(command_history) - limits.fault_guard_frames)
    reverse = list(command_history[:usable_end])[-limits.reverse_frames :]
    targets = [("reverse", pose) for pose in reversed(reverse)]
    targets += [("waypoint", pose) for pose in route.waypoints]
    targets += [("home", pose) for pose in home_trajectory]
    opened = {**home_trajectory[-1], "gripper": limits.gripper_open}
    targets.append(("open", opened))

    period = 1.0 / fps
    completed = 0
    phase_started = time.perf_counter()
    previous_phase = targets[0][0] if targets else ""
    last_goal = current
    try:
        for phase, target in targets:
            if phase != previous_phase:
                phase_started = time.perf_counter()
                previous_phase = phase
            for goal in bounded_poses(last_goal, target, limits.max_step_ticks):
                if time.perf_counter() - phase_started > limits.phase_timeout_s:
                    raise RecoveryAbortError(f"{phase} phase exceeded {limits.phase_timeout_s:.1f}s")
                bus.sync_write("Goal_Position", goal, normalize=False)
                telemetry = block_read(bus)
                _check_feedback(goal, telemetry, limits)
                if on_frame is not None:
                    on_frame(phase, completed, goal, telemetry)
                completed += 1
                last_goal = goal
                time.sleep(period)
    except Exception:
        halt(bus)
        raise
    bus.sync_write("Goal_Position", opened, normalize=False)
    return RecoveryResult(route=route.name, reverse_frames=len(reverse), completed_frames=completed)


def load_trajectory(path: Path) -> tuple[list[RawPose], list[float]]:
    """Load raw measured poses and timestamps from a telemetry logger CSV."""
    try:
        fh = path.open(newline="")
    except OSError as exc:
        raise ValueError(f"Cannot open reset trajectory {path}: {exc}") from exc

    with fh:
        reader = csv.DictReader(fh)
        if reader.fieldnames is None:
            raise ValueError(f"Reset trajectory {path} has no CSV header")

        pose_columns = {motor: f"pos.{motor}" for motor in SO101_MOTORS}
        required = {"t", *pose_columns.values()}
        missing = sorted(required - set(reader.fieldnames))
        if missing:
            raise ValueError(f"Reset trajectory {path} is missing columns: {', '.join(missing)}")

        poses: list[RawPose] = []
        timestamps: list[float] = []
        for line_number, row in enumerate(reader, start=2):
            try:
                timestamp = float(row["t"])
                pose = {motor: int(round(float(row[column]))) for motor, column in pose_columns.items()}
            except (TypeError, ValueError) as exc:
                raise ValueError(f"Invalid number in {path} at CSV line {line_number}") from exc

            if not math.isfinite(timestamp):
                raise ValueError(f"Non-finite timestamp in {path} at CSV line {line_number}")
            outside_range = {
                motor: position
                for motor, position in pose.items()
                if not RAW_POSITION_MIN <= position <= RAW_POSITION_MAX
            }
            if outside_range:
                raise ValueError(
                    f"Raw position outside {RAW_POSITION_MIN}..{RAW_POSITION_MAX} in {path} "
                    f"at CSV line {line_number}: {outside_range}"
                )
            if timestamps and timestamp <= timestamps[-1]:
                raise ValueError(f"Timestamps must increase in {path} (CSV line {line_number})")
            poses.append(pose)
            timestamps.append(timestamp)

    if not poses:
        raise ValueError(f"Reset trajectory {path} contains no data rows")
    return poses, timestamps


def interpolate_poses(start: RawPose, end: RawPose, frames: int) -> Iterator[RawPose]:
    """Yield ``frames`` linear steps after start, including end as the last step."""
    if frames < 0:
        raise ValueError("retract_frames must be non-negative")
    for step in range(1, frames + 1):
        alpha = step / frames
        yield {motor: round(start[motor] + alpha * (end[motor] - start[motor])) for motor in start}


def halt(bus: FeetechMotorsBus) -> RawPose:
    """Replace every stale goal with the present position and ensure torque is on."""
    assert_no_alarms(bus)
    present = bus.sync_read("Present_Position", normalize=False)
    # Set goals before enabling torque: a previously disabled servo may retain an old
    # goal in SRAM and would otherwise jump toward it as soon as it stiffens.
    bus.sync_write("Goal_Position", present, normalize=False)
    bus.enable_torque()
    return present


def wait_until(deadline: float) -> None:
    time.sleep(max(0.0, deadline - time.perf_counter()))


def wait_for_lift_clearance(
    bus: FeetechMotorsBus,
    target_pose: RawPose,
    *,
    tolerance: int,
    timeout: float,
    fps: float,
    on_frame: Callable[[str, int, RawPose], None] | None = None,
) -> None:
    """Hold every other joint until shoulder_lift physically reaches clearance."""
    target = target_pose["shoulder_lift"]
    started = time.perf_counter()
    frame_idx = 0
    while True:
        present = bus.sync_read("Present_Position", normalize=False)
        error = abs(present["shoulder_lift"] - target)
        if error <= tolerance:
            return
        if time.perf_counter() - started >= timeout:
            # Keep the last safe lift-only goal active. The caller disconnects without
            # disabling torque, so pan never starts after this failure.
            raise RuntimeError(
                f"shoulder_lift did not reach clearance: target={target}, "
                f"present={present['shoulder_lift']}, error={error} ticks after {timeout:.1f}s. "
                "Reset aborted before moving shoulder_pan."
            )
        bus.sync_write("Goal_Position", target_pose, normalize=False)
        if on_frame is not None:
            on_frame("lift_wait", frame_idx, target_pose)
        frame_idx += 1
        time.sleep(1.0 / fps)


def run_reset(
    bus: FeetechMotorsBus,
    trajectory: Sequence[RawPose],
    timestamps: Sequence[float],
    *,
    retract_frames: int,
    fps: float,
    lift_tolerance: int = 500,
    lift_timeout: float = 5.0,
    on_frame: Callable[[str, int, RawPose], None] | None = None,
) -> None:
    """Halt, enter the recorded path lift-first, replay it, and hold its final pose."""
    if fps <= 0 or not math.isfinite(fps):
        raise ValueError("fps must be a finite positive number")
    if lift_tolerance < 0:
        raise ValueError("lift_tolerance must be non-negative")
    if lift_timeout <= 0 or not math.isfinite(lift_timeout):
        raise ValueError("lift_timeout must be a finite positive number")
    if len(trajectory) != len(timestamps):
        raise ValueError("trajectory and timestamps must have equal lengths")
    if not trajectory:
        raise ValueError("trajectory must not be empty")

    current = halt(bus)
    period = 1.0 / fps
    deadline = time.perf_counter()

    # Establish vertical clearance before allowing pan or wrist motion. Interpolating
    # every joint together here can sweep a low gripper sideways across the table.
    # The trajectory's first shoulder_lift value is demonstrated clearance, so reach
    # that value while every other joint remains latched at its current position.
    lifted = {**current, "shoulder_lift": trajectory[0]["shoulder_lift"]}
    for frame_idx, pose in enumerate(interpolate_poses(current, lifted, retract_frames)):
        bus.sync_write("Goal_Position", pose, normalize=False)
        if on_frame is not None:
            on_frame("lift", frame_idx, pose)
        deadline += period
        wait_until(deadline)

    wait_for_lift_clearance(
        bus,
        lifted,
        tolerance=lift_tolerance,
        timeout=lift_timeout,
        fps=fps,
        on_frame=on_frame,
    )

    # With the gripper at clearance height, align the remaining joints to the start
    # of the operator-demonstrated reset path. shoulder_lift stays fixed in this stage.
    for frame_idx, pose in enumerate(interpolate_poses(lifted, trajectory[0], retract_frames)):
        bus.sync_write("Goal_Position", pose, normalize=False)
        if on_frame is not None:
            on_frame("align", frame_idx, pose)
        deadline += period
        wait_until(deadline)

    # Preserve the timing captured by the logger. Subtracting timestamps[0] also
    # supports files whose first row does not start exactly at zero.
    replay_start = time.perf_counter()
    t0 = timestamps[0]
    for frame_idx, (pose, timestamp) in enumerate(zip(trajectory, timestamps, strict=True)):
        bus.sync_write("Goal_Position", pose, normalize=False)
        if on_frame is not None:
            on_frame("home", frame_idx, pose)
        wait_until(replay_start + timestamp - t0)

    # Reassert the endpoint explicitly so the arm remains at home even if the last
    # group write was lost on the half-duplex serial bus.
    bus.sync_write("Goal_Position", trajectory[-1], normalize=False)


class SoakLogger:
    """Write commanded poses and observed telemetry on a common timestamp."""

    def __init__(self, path: Path, bus: FeetechMotorsBus):
        self.path = path
        self.bus = bus
        self.fh = None
        self.writer = None
        self.started = 0.0
        self.repeat_idx = 0

    def __enter__(self) -> "SoakLogger":
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.fh = self.path.open("w", newline="")
        motors = list(self.bus.motors)
        columns = (
            ["t", "repeat", "phase", "frame_idx"]
            + [f"goal_pos.{motor}" for motor in motors]
            + [f"{field}.{motor}" for field in BLOCK_FIELDS for motor in motors]
        )
        self.writer = csv.DictWriter(self.fh, fieldnames=columns)
        self.writer.writeheader()
        self.started = time.perf_counter()
        return self

    def __call__(self, phase: str, frame_idx: int, pose: RawPose) -> None:
        telemetry = block_read(self.bus)
        row = {
            "t": round(time.perf_counter() - self.started, 4),
            "repeat": self.repeat_idx,
            "phase": phase,
            "frame_idx": frame_idx,
        }
        for motor, position in pose.items():
            row[f"goal_pos.{motor}"] = position
        for field, values in telemetry.items():
            for motor, value in values.items():
                row[f"{field}.{motor}"] = value
        self.writer.writerow(row)

    def __exit__(self, *_args: object) -> None:
        if self.fh is not None:
            self.fh.close()


def connect_bus(port: str) -> FeetechMotorsBus:
    bus = FeetechMotorsBus(port=port, motors=SO101_MOTORS)
    bus.connect()
    return bus


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    halt_parser = subparsers.add_parser("halt", help="hold the arm at its present position")
    halt_parser.add_argument("--port", required=True, help="follower serial port, e.g. /dev/ttyACM0")

    reset_parser = subparsers.add_parser(
        "legacy-reset", help="EXPERIMENTAL lift-first replay; not an arbitrary-pose recovery"
    )
    reset_parser.add_argument("--port", required=True, help="follower serial port, e.g. /dev/ttyACM0")
    reset_parser.add_argument("--home", required=True, type=Path, help="CSV from log_teleop_telemetry.py")
    reset_parser.add_argument(
        "--retract-frames",
        type=int,
        default=30,
        help="steps per lift-first entry stage (default: 30; two stages total)",
    )
    reset_parser.add_argument("--fps", type=float, default=DEFAULT_FPS, help="bridge rate (default: 30)")
    reset_parser.add_argument(
        "--lift-tolerance",
        type=int,
        default=500,
        help="required shoulder_lift clearance accuracy in raw ticks (default: 500)",
    )
    reset_parser.add_argument(
        "--lift-timeout",
        type=float,
        default=5.0,
        help="abort before pan motion if lift has not reached clearance (default: 5s)",
    )
    reset_parser.add_argument("--repeat", type=int, default=1, help="number of complete resets (default: 1)")
    reset_parser.add_argument("--log", type=Path, help="optional raw telemetry soak-test CSV")
    reset_parser.add_argument(
        "--i-understand-lift-first-is-unsafe",
        action="store_true",
        help="required acknowledgement; never use unattended",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.command == "legacy-reset":
        if not args.i_understand_lift_first_is_unsafe:
            raise SystemExit(
                "legacy-reset is an unsafe lift-first experiment. Use validate_recovery.py with a "
                "physically validated waypoint configuration. To reproduce the old experiment under "
                "direct supervision, pass --i-understand-lift-first-is-unsafe."
            )
        if args.retract_frames < 0:
            raise SystemExit("--retract-frames must be non-negative")
        if args.repeat < 1:
            raise SystemExit("--repeat must be at least 1")
        if args.fps <= 0 or not math.isfinite(args.fps):
            raise SystemExit("--fps must be a finite positive number")
        if args.lift_tolerance < 0:
            raise SystemExit("--lift-tolerance must be non-negative")
        if args.lift_timeout <= 0 or not math.isfinite(args.lift_timeout):
            raise SystemExit("--lift-timeout must be a finite positive number")
        try:
            trajectory, timestamps = load_trajectory(args.home)
        except ValueError as exc:
            raise SystemExit(str(exc)) from exc

    bus = connect_bus(args.port)
    try:
        if args.command == "halt":
            present = halt(bus)
            print(f"Holding present position on {args.port}: {present}")
            return

        print(f"Loaded {len(trajectory)} frames ({timestamps[-1] - timestamps[0]:.2f}s) from {args.home}")
        logger_context = SoakLogger(args.log, bus) if args.log else _NullLogger()
        with logger_context as logger:
            for repeat_idx in range(args.repeat):
                logger.repeat_idx = repeat_idx
                print(f"Reset {repeat_idx + 1}/{args.repeat}...", flush=True)
                run_reset(
                    bus,
                    trajectory,
                    timestamps,
                    retract_frames=args.retract_frames,
                    fps=args.fps,
                    lift_tolerance=args.lift_tolerance,
                    lift_timeout=args.lift_timeout,
                    on_frame=logger if args.log else None,
                )
        suffix = f" Telemetry: {args.log}" if args.log else ""
        print(f"Reset complete; holding recorded home pose.{suffix}")
    except KeyboardInterrupt:
        print("\nInterrupted; halting at present position.")
        halt(bus)
    finally:
        bus.disconnect(disable_torque=False)


class _NullLogger:
    """Context-compatible placeholder that avoids a conditional around the reset loop."""

    repeat_idx = 0

    def __enter__(self) -> "_NullLogger":
        return self

    def __exit__(self, *_args: object) -> None:
        pass


if __name__ == "__main__":
    main()
