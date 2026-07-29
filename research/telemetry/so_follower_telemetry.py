#!/usr/bin/env python
"""SO-101 follower that records servo load, current, velocity and voltage with position.

`SOFollower.get_observation()` reads `Present_Position` only, so a recorded
LeRobotDataset carries no force-adjacent signal at all. This subclass widens
`observation.state` from 6 to 30 dims by adding load, current, velocity and supply
voltage per motor.

No change to `record_loop` is needed. `hw_to_dataset_features`
(src/lerobot/utils/feature_utils.py:84-89) funnels every non-image observation feature
into one `observation.state` vector, and `build_dataset_frame` (:128-136) fills it by
looking up each entry of `names` in whatever `get_observation()` returned. Declaring
the keys here is sufficient; validation, writing and replay all follow.

Ordering is grouped by FIELD, not by motor:

    [0:6]   <motor>.pos      normalized, exactly as plain SOFollower reports it
    [6:12]  <motor>.load     raw, signed, +/-1000 = +/-100% of Max_Torque_Limit
    [12:18] <motor>.current  raw, unsigned, ~6.5 mA/LSB
    [18:24] <motor>.vel      raw, signed
    [24:30] <motor>.volt     raw decivolts (120 = 12.0V)

Positions stay first and stay normalized, so `observation.state[:6]` is bit-identical
to what plain `so101_follower` records. That is what lets TruncateStateStep hand
SmolVLA an unchanged 6-dim input while detectors read the full 30.

Usage:
    lerobot-record --robot.type=so101_follower_telemetry --robot.port=/dev/ttyACM0 ...

Note: `robot.name` becomes the dataset's `robot_type`, so `--resume` against a dataset
recorded with plain `so101_follower` will fail its feature diff. Start a fresh dataset.
"""

import logging
import time
from dataclasses import dataclass
from functools import cached_property

from feetech_block import block_read

from lerobot.robots.config import RobotConfig
from lerobot.robots.so_follower import SOFollower, SOFollowerConfig
from lerobot.types import RobotObservation
from lerobot.utils.decorators import check_if_not_connected

logger = logging.getLogger(__name__)

# field name in block_read output -> suffix used in the observation/feature keys.
#
# `volt` is the servo's supply-rail reading in decivolts (120 = 12.0V). It is NOT a
# sensitive collision detector -- measured on the Week 1 matched pairs, the light and
# moderate contacts produced no distinguishable sag, and their largest voltage
# excursions landed 3.5s and 7.2s away from the actual contact, while ordinary clean
# runs swing 0.3-0.4V by themselves. It only tracked the hard stall, where it correlated
# with current at +0.99 and sagged 0.6V.
#
# Kept because it is a severity indicator for stalls (taxonomy E6), costs nothing on the
# wire -- the block read already fetches it -- and cannot be added later without
# re-recording the whole corpus. Treat it as corroboration for current, never as a
# trigger on its own.
TELEMETRY_FIELDS = {"load": "load", "curr": "current", "vel": "vel", "volt": "volt"}


@RobotConfig.register_subclass("so101_follower_telemetry")
@RobotConfig.register_subclass("so100_follower_telemetry")
@dataclass
class SOFollowerTelemetryConfig(RobotConfig, SOFollowerConfig):
    pass


class SOFollowerTelemetry(SOFollower):
    """SO follower whose observations carry load/current/velocity/voltage as well as position."""

    config_class = SOFollowerTelemetryConfig
    name = "so_follower_telemetry"
    latest_instance: "SOFollowerTelemetry | None" = None

    def __init__(self, config: SOFollowerTelemetryConfig):
        super().__init__(config)
        self.last_capture_timing: dict[str, object] | None = None
        type(self).latest_instance = self

    @property
    def _telemetry_ft(self) -> dict[str, type]:
        # Grouped by field so the 30-dim vector slices cleanly: pos, load, current, vel, volt.
        return {
            f"{motor}.{suffix}": float for suffix in TELEMETRY_FIELDS.values() for motor in self.bus.motors
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        # `_motors_ft` (positions) stays first. Deliberately does NOT touch
        # `action_features`, which inherits `_motors_ft` and must remain 6-dim
        # positions -- telemetry is observed, never commanded.
        return {**self._motors_ft, **self._telemetry_ft, **self._cameras_ft}

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        observation_start_ns = time.perf_counter_ns()
        wall_time_ns = time.time_ns()
        start = time.perf_counter()

        # Two bus transactions, not four. Position goes through the normal normalized
        # sync_read so its semantics are identical to the base class; everything else
        # comes from one block read over the contiguous SRAM span (addr 56..72).
        positions = self.bus.sync_read("Present_Position")
        position_read_end_ns = time.perf_counter_ns()
        telemetry = block_read(self.bus, fields=list(TELEMETRY_FIELDS))
        telemetry_read_end_ns = time.perf_counter_ns()

        obs_dict: RobotObservation = {f"{motor}.pos": val for motor, val in positions.items()}
        for field, suffix in TELEMETRY_FIELDS.items():
            for motor, val in telemetry[field].items():
                obs_dict[f"{motor}.{suffix}"] = float(val)

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state+telemetry: {dt_ms:.1f}ms")

        camera_timing: dict[str, dict[str, int | None]] = {}
        for cam_key, cam in self.cameras.items():
            if getattr(cam, "use_rgb", True):
                start = time.perf_counter()
                obs_dict[cam_key] = cam.read_latest()
                returned_ns = time.perf_counter_ns()
                captured = getattr(cam, "latest_timestamp", None)
                camera_timing[cam_key] = {
                    "capture_ns": round(captured * 1e9) if captured is not None else None,
                    "returned_ns": returned_ns,
                }
                logger.debug(f"{self} read {cam_key}: {(time.perf_counter() - start) * 1e3:.1f}ms")

            if getattr(cam, "use_depth", False):
                start = time.perf_counter()
                obs_dict[f"{cam_key}_depth"] = cam.read_latest_depth()
                logger.debug(f"{self} read {cam_key} depth: {(time.perf_counter() - start) * 1e3:.1f}ms")

        self.last_capture_timing = {
            "schema_version": 1,
            "wall_time_ns": wall_time_ns,
            "observation_start_ns": observation_start_ns,
            "position_read_end_ns": position_read_end_ns,
            "telemetry_read_end_ns": telemetry_read_end_ns,
            "observation_end_ns": time.perf_counter_ns(),
            "cameras": camera_timing,
        }

        return obs_dict


SO100FollowerTelemetry = SOFollowerTelemetry
SO101FollowerTelemetry = SOFollowerTelemetry
SO101FollowerTelemetryConfig = SOFollowerTelemetryConfig
