#!/usr/bin/env python
"""Single-transaction telemetry read for Feetech STS/SMS servos.

`sync_read` fetches one register per bus round-trip. Reading position, velocity, load
and current separately therefore costs four round-trips on a half-duplex bus, and the
four values do not share a timestamp -- which makes them useless for onset labeling.

Addresses 56..70 are contiguous SRAM, so one `_sync_read` over the whole span gets
everything at once. `GroupSyncRead.getData` supports sub-address extraction within the
block it fetched, which is what makes this safe rather than clever.

Used by both the standalone logger (log_teleop_telemetry.py) and the recording robot
subclass (so_follower_telemetry.py) so there is exactly one implementation to trust.
"""

from lerobot.motors.encoding_utils import decode_sign_magnitude
from lerobot.motors.feetech import FeetechMotorsBus

# Present_Position(56,2) .. Present_Current(69,2) -> 56..70 inclusive = 15 bytes.
BLOCK_ADDR = 56
BLOCK_LEN = 15

# field -> (address, n_bytes, sign_bit or None).
# Sign bits come from STS_SMS_SERIES_ENCODINGS_TABLE in motors/feetech/tables.py.
# Present_Current is deliberately None: it is absent from that table and comes back
# unsigned, so decoding it as sign-magnitude would corrupt values above 2^15.
BLOCK_FIELDS: dict[str, tuple[int, int, int | None]] = {
    "pos": (56, 2, 15),
    "vel": (58, 2, 15),
    "load": (60, 2, 10),
    "volt": (62, 1, None),
    "temp": (63, 1, None),
    "curr": (69, 2, None),
}

# For cross-checking against LeRobot's own per-register path.
REGISTER_FOR_FIELD = {
    "pos": "Present_Position",
    "vel": "Present_Velocity",
    "load": "Present_Load",
    "volt": "Present_Voltage",
    "temp": "Present_Temperature",
    "curr": "Present_Current",
}


def block_read(bus: FeetechMotorsBus, fields: list[str] | None = None) -> dict[str, dict[str, int]]:
    """One bus transaction -> telemetry for every motor, in raw register units.

    Args:
        bus: a connected FeetechMotorsBus.
        fields: which of BLOCK_FIELDS to return. `None` returns all of them. Limiting
            the fields costs nothing on the wire -- the whole block is fetched either
            way -- it only trims the returned dict.

    Returns:
        {field_name: {motor_name: value}}
    """
    fields = fields or list(BLOCK_FIELDS)
    ids = [m.id for m in bus.motors.values()]
    bus._sync_read(BLOCK_ADDR, BLOCK_LEN, ids, num_retry=2, raise_on_error=True)

    out: dict[str, dict[str, int]] = {}
    for field in fields:
        addr, size, sign_bit = BLOCK_FIELDS[field]
        out[field] = {}
        for name, motor in bus.motors.items():
            raw = bus.sync_reader.getData(motor.id, addr, size)
            out[field][name] = decode_sign_magnitude(raw, sign_bit) if sign_bit is not None else raw
    return out


def verify_block_read(bus: FeetechMotorsBus) -> bool:
    """Cross-check the block read against LeRobot's per-register sync_read.

    block_read reaches into bus.sync_reader directly, bypassing the normal decode path.
    Confirm it agrees before trusting a corpus to it. Returns True on agreement.
    """
    print("Hold the arm still, verifying block read...")
    block = block_read(bus)
    ok = True
    for field, reg in REGISTER_FOR_FIELD.items():
        ref = bus.sync_read(reg, normalize=False)
        for name in bus.motors:
            got, want = block[field][name], ref[name]
            # The two reads happen microseconds apart; anything moving will differ.
            tol = 30 if field in ("pos", "vel", "curr", "load") else 0
            if abs(got - want) > tol:
                print(f"  MISMATCH {field}/{name}: block={got} sync_read={want}")
                ok = False
    print("  block read agrees with sync_read" if ok else "  BLOCK READ DISAGREES -- do not use it")
    return ok
