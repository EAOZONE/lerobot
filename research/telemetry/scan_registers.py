#!/usr/bin/env python
"""Find which SRAM addresses actually respond to physical load, empirically.

Use when a register that should carry signal reads flat zero. Rather than trusting
the control table, this dumps every SRAM byte at rest and again under load, and
reports which addresses moved. If this firmware reports current somewhere other than
addr 69, it turns up here. If nothing but Present_Load responds, that is a real
finding and D0 becomes load-only.

Torque is enabled (the driver must be driving for load/current to be non-zero).

Usage:
    python research/telemetry/scan_registers.py --port /dev/ttyACM0
    python research/telemetry/scan_registers.py --port /dev/ttyACM0 --motor gripper
"""

import argparse
import time

from probe_bus import SO101_MOTORS, enable_torque_safely, release_torque_quietly

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors.feetech.tables import STS_SMS_SERIES_CONTROL_TABLE

# SRAM only. Below 40 is EPROM config; writing there is how you brick a servo, and
# reading it tells you nothing about load.
SCAN_START = 40
SCAN_END = 90
CHUNK = 16

# addr -> name, for annotating the report. Multi-byte registers claim each of their bytes.
ADDR_NAMES: dict[int, str] = {}
for _name, (_addr, _size) in STS_SMS_SERIES_CONTROL_TABLE.items():
    for _offset in range(_size):
        ADDR_NAMES[_addr + _offset] = _name if _size == 1 else f"{_name}[{_offset}]"


def dump_bytes(bus: FeetechMotorsBus, motor: str) -> dict[int, int]:
    """Every SRAM byte for one motor, read in chunks."""
    motor_id = bus.motors[motor].id
    out: dict[int, int] = {}
    for start in range(SCAN_START, SCAN_END, CHUNK):
        length = min(CHUNK, SCAN_END - start)
        bus._sync_read(start, length, [motor_id], num_retry=2, raise_on_error=True)
        for addr in range(start, start + length):
            out[addr] = bus.sync_reader.getData(motor_id, addr, 1)
    return out


def sample_span(bus: FeetechMotorsBus, motor: str, seconds: float) -> dict[int, tuple[int, int]]:
    """Min/max per address over a sampling window."""
    span: dict[int, list[int]] = {}
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < seconds:
        for addr, val in dump_bytes(bus, motor).items():
            if addr not in span:
                span[addr] = [val, val]
            else:
                span[addr][0] = min(span[addr][0], val)
                span[addr][1] = max(span[addr][1], val)
    return {a: (lo, hi) for a, (lo, hi) in span.items()}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--motor", default="gripper", help="default: gripper")
    parser.add_argument("--seconds", type=float, default=6.0, help="per phase")
    args = parser.parse_args()

    bus = FeetechMotorsBus(port=args.port, motors=SO101_MOTORS)
    bus.connect()
    try:
        # Only the scanned motor needs to be driving, so leave the rest limp rather
        # than making the shoulder hold its own weight for two sampling phases.
        print(f"About to ENABLE TORQUE on '{args.motor}' only. It will stiffen and hold its pose.")
        input("Support the arm if needed, then press ENTER...")
        enable_torque_safely(bus, [args.motor])

        print(f"\nPhase 1/2: REST. Hands off the arm. Sampling {args.seconds:.0f}s...")
        time.sleep(1.0)
        rest = sample_span(bus, args.motor, args.seconds)

        print(f"\nPhase 2/2: LOAD. Squeeze '{args.motor}' shut / push it off its hold point")
        print("            and HOLD the pressure. Starting in 3s...")
        time.sleep(3.0)
        print(f"            Sampling {args.seconds:.0f}s -- keep the pressure on...")
        loaded = sample_span(bus, args.motor, args.seconds)

        print(f"\n--- addresses that responded to load ({args.motor}) ---")
        print(f"{'addr':>5}  {'register':<26} {'rest':>13}  {'loaded':>13}")
        found_any = False
        for addr in sorted(rest):
            r_lo, r_hi = rest[addr]
            l_lo, l_hi = loaded[addr]
            # Responded if the loaded range escapes the rest range.
            if l_lo >= r_lo and l_hi <= r_hi:
                continue
            found_any = True
            name = ADDR_NAMES.get(addr, "-- unmapped --")
            print(f"{addr:>5}  {name:<26} {f'{r_lo}..{r_hi}':>13}  {f'{l_lo}..{l_hi}':>13}")

        if not found_any:
            print("  NOTHING responded. Torque may not have engaged, or you did not load it")
            print("  hard enough. Confirm the arm actually resists you, then re-run.")
        else:
            print("\n  'unmapped' rows are addresses carrying signal that the control table")
            print("  does not name -- worth investigating before concluding current is absent.")
    finally:
        release_torque_quietly(bus)
        bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
