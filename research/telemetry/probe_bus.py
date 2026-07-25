#!/usr/bin/env python
"""Week 1, Days 1-2: confirm the Feetech bus actually reports load and current.

Bare polling loop, no cameras, no dataset, no calibration required. Answers three
questions before anything else in the project is built:

  1. Does `Present_Current` return non-zero values on *these* servos? The register
     exists in the STS/SMS control table, but whether the firmware populates it
     varies by unit. "Do not assume."
  2. Does `Present_Load` respond to physical resistance?
  3. What read rate is achievable, and how much does each extra register cost?

Usage:
    python research/telemetry/probe_bus.py --port /dev/ttyACM0
    python research/telemetry/probe_bus.py --port /dev/ttyACM0 --seconds 30
    python research/telemetry/probe_bus.py --port /dev/ttyACM0 --hold gripper

While it runs, grab the gripper and squeeze, and push back against a joint by hand.
You want to see Load and Current move.
"""

import argparse
import time

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

SO101_MOTORS = {
    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
    "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
    "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
    "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
    "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
}

# Registers we care about, cheapest-signal-first. All read with normalize=False so
# this works on an uncalibrated arm and reports raw servo units.
REGISTERS = ["Present_Position", "Present_Load", "Present_Current", "Present_Velocity"]


def benchmark_registers(bus: FeetechMotorsBus, n: int = 100) -> None:
    """Time each register individually, then all of them together."""
    print("\n--- read cost (6 motors, sync_read) ---")
    for reg in REGISTERS:
        t0 = time.perf_counter()
        for _ in range(n):
            bus.sync_read(reg, normalize=False)
        dt_ms = (time.perf_counter() - t0) / n * 1e3
        print(f"  {reg:<20} {dt_ms:6.2f} ms/read   ({1e3 / dt_ms:6.1f} Hz alone)")

    t0 = time.perf_counter()
    for _ in range(n):
        for reg in REGISTERS:
            bus.sync_read(reg, normalize=False)
    dt_ms = (time.perf_counter() - t0) / n * 1e3
    print(
        f"  {'ALL ' + str(len(REGISTERS)) + ' registers':<20} {dt_ms:6.2f} ms/cycle ({1e3 / dt_ms:6.1f} Hz)"
    )
    print(
        "\n  Each register is a separate bus round-trip. If this is too slow to sit\n"
        "  inside a 30 Hz control loop, use block_read() from feetech_block.py, which\n"
        "  fetches all of them in one transaction on a shared timestamp."
    )


def enable_torque_safely(bus: FeetechMotorsBus, motors: list[str] | None = None) -> None:
    """Hold current pose, then enable torque.

    Present_Load is the servo's output PWM duty and Present_Current is its drive
    current: with torque disabled the driver is idle and BOTH read ~0 no matter how
    hard you push the arm by hand. Torque must be on for this probe to mean anything.

    Goal_Position is written to the present position *first*. Torque_Enable acts on
    whatever goal is already sitting in SRAM, which may be stale from a previous
    session -- without this the arm snaps to it.

    Pass `motors` to hold only some joints. Enabling all six makes the gravity-loaded
    shoulder fight its own weight for the whole run, which is a good way to trip
    overload protection; if you only need gripper signal, only hold the gripper.
    """
    assert_no_alarms(bus)
    present = bus.sync_read("Present_Position", normalize=False)
    if motors:
        present = {name: pos for name, pos in present.items() if name in motors}
    # sync_write does not wait for a status response, so one grumpy servo cannot abort
    # the whole batch and leave the others holding a stale goal.
    bus.sync_write("Goal_Position", present, normalize=False)
    bus.enable_torque(list(present))


def assert_no_alarms(bus: FeetechMotorsBus) -> None:
    """Refuse to enable torque on a servo that has latched an alarm.

    An alarmed Feetech servo rejects writes until it is power-cycled, and the SDK
    reports the rejection as an empty error string. Catch it here with a message that
    says what to actually do.
    """
    alarmed = []
    for name, motor in bus.motors.items():
        _, comm, error = bus._read(65, 1, motor.id, num_retry=2, raise_on_error=False)
        if bus._is_comm_success(comm) and error:
            alarmed.append(f"{name} (id={motor.id}, alarm=0b{error:08b})")

    if alarmed:
        raise RuntimeError(
            "Servo(s) have latched an alarm and will reject commands:\n  "
            + "\n  ".join(alarmed)
            + "\n\nPower-cycle the servo supply (unplug the power barrel jack, not just USB,"
            "\nwait ~5s, replug), then re-run. Run diagnose.py for the full register dump."
        )


def release_torque_quietly(bus: FeetechMotorsBus) -> None:
    """Disable torque without masking whatever exception sent us here.

    A servo that alarmed mid-run will also reject the Torque_Enable write, and an
    exception raised from a `finally` block replaces the original traceback.
    """
    try:
        bus.disable_torque()
    except Exception as exc:  # noqa: BLE001 - cleanup must not mask the real error
        print(f"  (warning: could not disable torque on exit: {exc})")


def watch(bus: FeetechMotorsBus, seconds: float, hz: float, torque_on: bool) -> None:
    """Print a live table and track whether each register ever moved."""
    names = list(bus.motors)
    seen_nonzero = dict.fromkeys(REGISTERS, False)
    spans = {reg: {m: [float("inf"), float("-inf")] for m in names} for reg in REGISTERS}

    print(f"\n--- live watch ({seconds:.0f}s, torque {'ON' if torque_on else 'OFF'}) ---")
    if torque_on:
        print("Arm is holding position. Squeeze the gripper shut; push a joint off its")
        print("hold point. Both resist you, and Load/Current should climb.\n")
    else:
        print("Torque is OFF -- the driver is idle, so Load and Current will read ~0")
        print("regardless of how hard you push. This mode is for bus debugging only.\n")
    header = f"{'t':>6} | " + " | ".join(f"{m[:9]:>9}" for m in names)

    period = 1.0 / hz
    t_start = time.perf_counter()
    next_print = 0.0
    while (t := time.perf_counter() - t_start) < seconds:
        readings = {reg: bus.sync_read(reg, normalize=False) for reg in REGISTERS}
        for reg, vals in readings.items():
            for m, v in vals.items():
                if v != 0:
                    seen_nonzero[reg] = True
                spans[reg][m][0] = min(spans[reg][m][0], v)
                spans[reg][m][1] = max(spans[reg][m][1], v)

        if t >= next_print:
            print(header)
            for reg in REGISTERS:
                row = " | ".join(f"{readings[reg][m]:>9d}" for m in names)
                print(f"{reg[8:14]:>6} | {row}")
            print(f"  t={t:5.1f}s")
            next_print = t + 0.5

        time.sleep(max(0.0, period - (time.perf_counter() - t_start - t)))

    print("\n--- verdict ---")
    for reg in REGISTERS:
        if not seen_nonzero[reg]:
            print(f"  {reg:<20} ALWAYS ZERO  <-- register not populated by this firmware")
            continue
        widest = max(names, key=lambda m: spans[reg][m][1] - spans[reg][m][0])
        lo, hi = spans[reg][widest]
        print(f"  {reg:<20} ok, range {lo} .. {hi} (widest: {widest})")

    if not seen_nonzero["Present_Current"]:
        if not torque_on:
            print(
                "\n  Present_Current flat zero with torque OFF is EXPECTED, not a finding.\n"
                "  Re-run without --no-torque before concluding anything."
            )
        else:
            print(
                "\n  Present_Current is flat zero WITH TORQUE ON and the arm resisting you.\n"
                "  That is a real result. Before re-scoping anything, run scan_registers.py:\n"
                "  it finds which SRAM addresses actually respond to load, so if this\n"
                "  firmware reports current at a non-standard address it will turn up there.\n"
                "  If only addr 60-61 (Present_Load) responds, re-scope D0 to load-only and\n"
                "  note the firmware version in the lab log."
            )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="e.g. /dev/ttyACM0 (find with lerobot-find-port)")
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument(
        "--no-torque",
        action="store_true",
        help="leave the arm limp (bus debugging only -- Load/Current will read ~0)",
    )
    parser.add_argument(
        "--hold",
        action="append",
        help="only enable torque on these motors (repeatable). Default: all six. "
        "Use --hold gripper to avoid loading the shoulder for a whole run.",
    )
    args = parser.parse_args()

    bus = FeetechMotorsBus(port=args.port, motors=SO101_MOTORS)
    bus.connect()
    print(f"Connected on {args.port}.")

    print("\n--- servo identity ---")
    for name in bus.motors:
        major = bus.read("Firmware_Major_Version", name, normalize=False)
        minor = bus.read("Firmware_Minor_Version", name, normalize=False)
        model = bus.read("Model_Number", name, normalize=False)
        print(f"  {name:<15} id={bus.motors[name].id}  fw={major}.{minor}  model={model}")

    torque_on = not args.no_torque
    try:
        benchmark_registers(bus)
        if torque_on:
            held = args.hold or list(bus.motors)
            print(f"\nAbout to ENABLE TORQUE on: {', '.join(held)}")
            print("Those joints will stiffen and hold their current pose.")
            print("Support the arm if it is in a position that could drop. Ctrl-C to abort.")
            input("Press ENTER when clear...")
            enable_torque_safely(bus, args.hold)
        watch(bus, args.seconds, args.hz, torque_on)
    finally:
        if torque_on:
            release_torque_quietly(bus)
        bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
