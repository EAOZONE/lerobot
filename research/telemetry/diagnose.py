#!/usr/bin/env python
"""Read each servo's raw alarm byte and health registers.

Run this when a write fails with an empty error string. The scservo SDK's
`getRxPacketError` only names bits 1/2/4/8/32 and returns "" for anything else, so a
latched alarm on an unmapped bit produces a RuntimeError with no explanation. This
prints the raw byte.

Usage:
    python research/telemetry/diagnose.py --port /dev/ttyACM0
"""

import argparse

from probe_bus import SO101_MOTORS

from lerobot.motors.feetech import FeetechMotorsBus

# The SDK's mapping (protocol_packet_handler.py). Bits outside this set exist on the
# hardware but have no SDK message -- which is why the error string comes back empty.
SDK_ERROR_BITS = {
    1: "voltage",
    2: "angle sensor",
    4: "overheat",
    8: "over-current",
    32: "overload",
}

HEALTH_REGISTERS = [
    ("Present_Voltage", "0.1V"),
    ("Present_Temperature", "degC"),
    ("Present_Load", "+/-1000"),
    ("Present_Current", "~6.5mA/LSB"),
    ("Present_Position", "ticks"),
    ("Status", "alarm byte"),
    ("Torque_Enable", "0/1"),
    ("Max_Temperature_Limit", "degC"),
    ("Protection_Current", "raw"),
]


def describe_error(error: int) -> str:
    if error == 0:
        return "clean"
    named = [label for bit, label in SDK_ERROR_BITS.items() if error & bit]
    unmapped = [f"bit{i}" for i in range(8) if (error >> i) & 1 and (1 << i) not in SDK_ERROR_BITS]
    parts = named + [f"{u} (unmapped by SDK)" for u in unmapped]
    return f"0b{error:08b} ({error}) -> " + ", ".join(parts)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    args = parser.parse_args()

    bus = FeetechMotorsBus(port=args.port, motors=SO101_MOTORS)
    bus.connect(handshake=False)
    try:
        print(f"{'motor':<15} {'id':>3}  alarm state")
        print("-" * 70)
        alarmed = []
        for name, motor in bus.motors.items():
            # raise_on_error=False so an alarmed servo reports instead of aborting the sweep.
            _, comm, error = bus._read(65, 1, motor.id, num_retry=2, raise_on_error=False)
            if not bus._is_comm_success(comm):
                print(f"{name:<15} {motor.id:>3}  NO RESPONSE ({bus.packet_handler.getTxRxResult(comm)})")
                continue
            print(f"{name:<15} {motor.id:>3}  {describe_error(error)}")
            if error:
                alarmed.append(name)

        print("\n--- health registers ---")
        header = f"{'register':<24} " + " ".join(f"{n[:8]:>9}" for n in bus.motors)
        print(header)
        for reg, unit in HEALTH_REGISTERS:
            cells = []
            for name in bus.motors:
                try:
                    cells.append(f"{bus.read(reg, name, normalize=False):>9}")
                except Exception:
                    cells.append(f"{'ERR':>9}")
            print(f"{reg + ' (' + unit + ')':<24} " + " ".join(cells))

        if alarmed:
            print(f"\n  ALARMED: {', '.join(alarmed)}")
            print("  Feetech servos latch an alarm and refuse commands until power is cycled.")
            print("  Fix: unplug the servo power supply (not just USB), wait ~5s, plug back in.")
            print("  Then support the arm so the shoulder is not holding a large moment, and")
            print("  check Present_Temperature above -- if it is high, let it cool first.")
        else:
            print("\n  All servos report clean. If a write still fails, it is not a latched alarm.")
    finally:
        bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
