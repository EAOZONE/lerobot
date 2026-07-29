"""Offline safety tests for the arbitrary-pose recovery core."""

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))

import recovery  # noqa: E402,I001


MOTORS = list(recovery.SO101_MOTORS)


def pose(value: int) -> recovery.RawPose:
    return dict.fromkeys(MOTORS, value)


def config(*, validated: bool = True, max_current: int = 100) -> recovery.RecoveryConfig:
    limits = recovery.RecoveryLimits(
        joint_min=pose(0),
        joint_max=pose(4095),
        max_current=pose(max_current),
        max_following_error=pose(100),
        max_step_ticks=100,
        phase_timeout_s=10,
        reverse_frames=2,
        fault_guard_frames=1,
        gripper_open=100,
    )
    route = recovery.RecoveryRoute(
        name="center",
        region_min=pose(0),
        region_max=pose(4095),
        waypoints=(pose(1400),),
        validated=validated,
    )
    return recovery.RecoveryConfig(1, limits, (route,))


class FakeBus:
    def __init__(self, start: recovery.RawPose):
        self.present = start.copy()
        self.goals = []

    def sync_write(self, register, values, normalize=False):
        assert register == "Goal_Position"
        assert normalize is False
        self.present = values.copy()
        self.goals.append(values.copy())


def install_fake_io(monkeypatch, bus: FakeBus, *, current: int = 1) -> None:
    monkeypatch.setattr(recovery, "halt", lambda _bus: bus.present.copy())
    monkeypatch.setattr(recovery.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(
        recovery,
        "block_read",
        lambda _bus: {
            "pos": bus.present.copy(),
            "vel": pose(0),
            "load": pose(0),
            "curr": pose(current),
            "volt": pose(120),
        },
    )


def test_reverse_replay_drops_fault_guard_and_uses_bounded_history(monkeypatch) -> None:
    bus = FakeBus(pose(1600))
    install_fake_io(monkeypatch, bus)
    phases = []
    result = recovery.execute_recovery(
        bus,
        [pose(1100), pose(1200), pose(1300), pose(1600)],
        [pose(1500)],
        config(),
        fps=30,
        flush_action_queue=lambda: None,
        on_frame=lambda phase, *_args: phases.append(phase),
    )
    assert result.reverse_frames == 2
    assert result.route == "center"
    # Guard removes 1600; bounded history keeps 1200/1300 and reverse starts toward 1300.
    assert bus.goals[0]["shoulder_pan"] == 1500
    assert phases[0] == "reverse"
    assert {"reverse", "waypoint", "home", "open"} <= set(phases)
    assert bus.goals[-1]["gripper"] == 100


def test_selected_route_is_published_before_any_recovery_frame(monkeypatch) -> None:
    bus = FakeBus(pose(1600))
    install_fake_io(monkeypatch, bus)
    calls = []
    recovery.execute_recovery(
        bus,
        [pose(1500), pose(1600)],
        [pose(1400)],
        config(),
        fps=30,
        flush_action_queue=lambda: None,
        on_route=lambda route: calls.append(("route", route)),
        on_frame=lambda phase, *_args: calls.append(("frame", phase)),
    )
    assert calls[0] == ("route", "center")
    assert calls[1][0] == "frame"


def test_unvalidated_route_aborts_before_motion(monkeypatch) -> None:
    bus = FakeBus(pose(1600))
    install_fake_io(monkeypatch, bus)
    with pytest.raises(recovery.RecoveryAbortError, match="not physically validated"):
        recovery.execute_recovery(
            bus,
            [pose(1500)],
            [pose(1400)],
            config(validated=False),
            fps=30,
            flush_action_queue=lambda: None,
        )
    assert bus.goals == []


def test_overcurrent_aborts_and_halts(monkeypatch) -> None:
    bus = FakeBus(pose(1600))
    halt_calls = []
    monkeypatch.setattr(recovery, "halt", lambda _bus: halt_calls.append(1) or bus.present.copy())
    monkeypatch.setattr(recovery.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(
        recovery,
        "block_read",
        lambda _bus: {
            "pos": bus.present,
            "curr": pose(101),
            "vel": pose(0),
            "load": pose(0),
            "volt": pose(120),
        },
    )
    with pytest.raises(recovery.RecoveryAbortError, match="current"):
        recovery.execute_recovery(
            bus,
            [pose(1500)],
            [pose(1400)],
            config(max_current=100),
            fps=30,
            flush_action_queue=lambda: None,
        )
    assert len(halt_calls) == 2


def test_example_config_is_intentionally_non_executable(tmp_path) -> None:
    source = Path(__file__).with_name("recovery_waypoints.example.json")
    data = json.loads(source.read_text())
    data["limits"]["max_step_ticks"] = 10
    data["limits"]["phase_timeout_s"] = 1
    data["limits"]["reverse_frames"] = 1
    data["limits"]["gripper_open"] = 1
    path = tmp_path / "config.json"
    path.write_text(json.dumps(data))
    loaded = recovery.load_recovery_config(path)
    with pytest.raises(recovery.RecoveryAbortError, match="not physically validated"):
        recovery.select_recovery_route(loaded, pose(1000))
