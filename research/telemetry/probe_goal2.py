#!/usr/bin/env python
"""Confirm whether Goal_Position_2 (register 71) is populated by this firmware.

Two matched clear replays of trajectory B logged register 71 as a constant 0 on every
motor in all 649 frames (`analyze_goal2.py`, 28 July 2026), which kills the hypothesis
that the servo's internal interpolated setpoint explains repeat-dependent current
variation. That evidence came from replay logs, so it leaves one loophole: perhaps the
register was never read correctly, rather than never written.

This closes it directly. If register 71 were a setpoint it would track `Goal_Position`
and sit near `Present_Position` at rest. Reading 0 while the arm holds a nonzero pose
means the firmware does not populate it at all.

`verify_block_read` cannot settle this on its own: its tolerance for `goal2` is 30 ticks,
so a 0-versus-0 agreement between the block read and `sync_read` passes silently.

This script is READ-ONLY. It does not enable torque, does not write goals, and does not
disable torque on exit, so it cannot move the arm or change the state you inherit.

Usage:
    # snapshot at the current pose -- move the arm somewhere clearly nonzero first
    python research/telemetry/probe_goal2.py --port /dev/ttyACM0

    # watch for 10s while you teleoperate or hand-move, in case 71 only fills in motion
    python research/telemetry/probe_goal2.py --port /dev/ttyACM0 --watch 10
"""

import argparse
import time

from probe_bus import SO101_MOTORS

from lerobot.motors.feetech import FeetechMotorsBus

REGISTERS = ["Present_Position", "Goal_Position", "Goal_Position_2"]
POLL_HZ = 30.0


def _read_all(bus: FeetechMotorsBus) -> dict[str, dict[str, int]]:
    return {reg: bus.sync_read(reg, normalize=False) for reg in REGISTERS}


def _snapshot(bus: FeetechMotorsBus) -> None:
    values = _read_all(bus)
    print(f"\n{'motor':<15}" + "".join(f" {reg:>17}" for reg in REGISTERS))
    print("-" * (15 + 18 * len(REGISTERS)))
    for name in bus.motors:
        print(f"{name:<15}" + "".join(f" {values[reg][name]:>17}" for reg in REGISTERS))

    goal2 = values["Goal_Position_2"]
    position = values["Present_Position"]
    moved_off_zero = [n for n in bus.motors if position[n] != 0]

    print()
    if any(goal2[n] != 0 for n in bus.motors):
        print("  Register 71 is NONZERO on at least one motor.")
        print("  The replay finding needs re-examining -- rerun analyze_goal2.py and check")
        print("  the logger path before treating the internal-setpoint hypothesis as closed.")
    elif not moved_off_zero:
        print("  INCONCLUSIVE: every Present_Position reads 0, so the arm is at (or near) the")
        print("  raw origin and a zero setpoint would be correct. Move the arm to a clearly")
        print("  nonzero pose and run this again.")
    else:
        print("  Register 71 reads 0 on every motor while the arm holds a nonzero pose")
        print(f"  ({len(moved_off_zero)}/{len(bus.motors)} motors off zero). This firmware does not")
        print("  populate Goal_Position_2. The internal-setpoint hypothesis is closed:")
        print("  see WEEK2_REPORT.md section 3 and D0R_CLEAR_DIAGNOSIS.md.")


def _watch(bus: FeetechMotorsBus, seconds: float) -> None:
    print(f"\n  Watching register 71 for {seconds:.0f}s. Move the arm now (teleoperate, or")
    print("  hand-move it if torque is off) so any interpolated setpoint would have to update.")

    period = 1.0 / POLL_HZ
    deadline = time.perf_counter() + seconds
    first = bus.sync_read("Goal_Position_2", normalize=False)
    observed = {name: {value} for name, value in first.items()}
    samples = 1

    while time.perf_counter() < deadline:
        for name, value in bus.sync_read("Goal_Position_2", normalize=False).items():
            observed[name].add(value)
        samples += 1
        time.sleep(period)

    print(f"\n  {samples} samples")
    print(f"\n{'motor':<15} {'distinct values':>16} {'observed':>32}")
    print("-" * 65)
    for name in bus.motors:
        values = sorted(observed[name])
        shown = ", ".join(str(v) for v in values[:5]) + (" ..." if len(values) > 5 else "")
        print(f"{name:<15} {len(values):>16} {shown:>32}")

    if all(observed[name] == {0} for name in bus.motors):
        print("\n  Register 71 never left 0 during motion. Confirms the replay finding.")
    else:
        print("\n  Register 71 CHANGED during motion -- it carries something after all.")
        print("  Re-open the hypothesis and re-run the paired replay diagnostic.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument(
        "--watch",
        type=float,
        default=0.0,
        metavar="SECONDS",
        help="after the snapshot, poll register 71 while the arm moves",
    )
    args = parser.parse_args()

    bus = FeetechMotorsBus(port=args.port, motors=SO101_MOTORS)
    bus.connect(handshake=False)
    try:
        _snapshot(bus)
        if args.watch > 0:
            _watch(bus, args.watch)
    finally:
        # Read-only probe: leave torque exactly as it was found.
        bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
