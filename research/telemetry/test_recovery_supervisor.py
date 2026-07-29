import json

import pytest
from recovery import RecoveryResult
from recovery_supervisor import (
    DetectorTrigger,
    RecoveryAttemptsExhaustedError,
    RecoveryEventLogger,
    RecoverySupervisor,
    RecoverySupervisorError,
    RolloutRecoveryLifecycle,
    SupervisorState,
)

MOTORS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def pose(value):
    return dict.fromkeys(MOTORS, value)


def trigger(frame=10):
    return DetectorTrigger("d0", score=1.2, threshold=1.0, frame_index=frame, fault_type="slip")


def test_success_flushes_inside_executor_clears_history_and_reinvokes():
    calls = []

    def execute(history, flush):
        calls.append(("history", [item["shoulder_pan"] for item in history]))
        flush()
        calls.append(("execute", None))
        return RecoveryResult("center", 2, 30)

    events = []
    supervisor = RecoverySupervisor(
        flush_policy_queue=lambda: calls.append(("flush", None)),
        reinvoke_policy=lambda: calls.append(("reinvoke", None)),
        execute=execute,
        history_frames=2,
        on_event=events.append,
    )
    for value in (100, 200, 300):
        supervisor.record_sent_command(pose(value))
    result = supervisor.recover(trigger())

    assert result.route == "center"
    assert calls == [
        ("history", [200, 300]),
        ("flush", None),
        ("execute", None),
        ("reinvoke", None),
    ]
    assert not supervisor.command_history
    assert supervisor.state is SupervisorState.READY
    assert [event.event for event in events] == [
        "trigger",
        "recovery_started",
        "recovery_completed",
        "policy_reinvoked",
    ]


def test_executor_that_does_not_flush_aborts_and_never_reinvokes():
    reinvoked = []
    supervisor = RecoverySupervisor(
        flush_policy_queue=lambda: None,
        reinvoke_policy=lambda: reinvoked.append(True),
        execute=lambda history, flush: RecoveryResult("center", 1, 2),
        history_frames=2,
    )
    supervisor.record_sent_command(pose(100))
    with pytest.raises(RecoverySupervisorError, match="without flushing"):
        supervisor.recover(trigger())
    assert supervisor.state is SupervisorState.ABORTED
    assert not reinvoked


def test_failed_recovery_is_fail_closed_and_preserves_history():
    def execute(history, flush):
        flush()
        raise RuntimeError("overcurrent")

    supervisor = RecoverySupervisor(
        flush_policy_queue=lambda: None,
        reinvoke_policy=lambda: pytest.fail("must not reinvoke after abort"),
        execute=execute,
        history_frames=2,
    )
    supervisor.record_sent_command(pose(100))
    with pytest.raises(RuntimeError, match="overcurrent"):
        supervisor.recover(trigger())
    assert supervisor.state is SupervisorState.ABORTED
    assert len(supervisor.command_history) == 1


def test_two_recovery_cap_is_enforced_per_episode():
    def execute(history, flush):
        flush()
        return RecoveryResult("center", 1, 2)

    supervisor = RecoverySupervisor(
        flush_policy_queue=lambda: None,
        reinvoke_policy=lambda: None,
        execute=execute,
        history_frames=2,
        max_recoveries=2,
    )
    for frame in range(2):
        supervisor.record_sent_command(pose(100 + frame))
        supervisor.recover(trigger(frame))
    supervisor.record_sent_command(pose(300))
    with pytest.raises(RecoveryAttemptsExhaustedError):
        supervisor.recover(trigger(3))
    assert supervisor.state is SupervisorState.EXHAUSTED
    assert supervisor.attempts == 2


def test_episode_boundary_flushes_and_resets_accounting():
    flushed = []
    reinvoked = []
    supervisor = RecoverySupervisor(
        flush_policy_queue=lambda: flushed.append(True),
        reinvoke_policy=lambda: reinvoked.append(True),
        execute=lambda history, flush: RecoveryResult("center", 1, 2),
        history_frames=2,
    )
    supervisor.record_sent_command(pose(100))
    supervisor.attempts = 2
    supervisor.state = SupervisorState.EXHAUSTED
    supervisor.start_episode(7)
    assert flushed == [True]
    assert reinvoked == [True]
    assert supervisor.episode_index == 7
    assert supervisor.attempts == 0
    assert supervisor.state is SupervisorState.READY
    assert not supervisor.command_history


def test_jsonl_event_logger_writes_complete_trigger_fields(tmp_path):
    path = tmp_path / "events.jsonl"
    logger = RecoveryEventLogger(path)

    def execute(history, flush):
        flush()
        return RecoveryResult("center", 1, 2)

    supervisor = RecoverySupervisor(
        flush_policy_queue=lambda: None,
        reinvoke_policy=lambda: None,
        execute=execute,
        history_frames=2,
        on_event=logger,
    )
    supervisor.start_episode(4)
    supervisor.record_sent_command(pose(100))
    supervisor.recover(trigger(12))
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    trigger_row = next(row for row in rows if row["event"] == "trigger")
    assert trigger_row["episode_index"] == 4
    assert trigger_row["detector"] == "d0"
    assert trigger_row["score"] == 1.2
    assert trigger_row["threshold"] == 1.0
    assert trigger_row["trigger_frame"] == 12
    assert trigger_row["fault_type"] == "slip"


class FakeEngine:
    def __init__(self, calls):
        self.calls = calls
        self.queue = ["stale_chunk_action"]
        self.producing = True

    def pause(self):
        self.calls.append("pause")
        self.producing = False

    def reset(self):
        self.calls.append("engine_reset")
        assert not self.producing
        self.queue.clear()

    def resume(self):
        self.calls.append("resume")
        assert not self.queue
        self.producing = True


class FakeInterpolator:
    def __init__(self, calls):
        self.calls = calls
        self.pending = ["stale_interpolated_action"]

    def reset(self):
        self.calls.append("interpolator_reset")
        self.pending.clear()


def test_rollout_lifecycle_clears_all_action_layers_before_resume():
    calls = []
    engine = FakeEngine(calls)
    interpolator = FakeInterpolator(calls)
    cached = {"observation": "stale"}

    def invalidate():
        calls.append("invalidate_observation")
        cached.clear()

    def publish():
        calls.append("publish_fresh_observation")
        assert not engine.queue
        assert not interpolator.pending
        assert not cached

    lifecycle = RolloutRecoveryLifecycle(
        inference_engine=engine,
        action_interpolator=interpolator,
        invalidate_cached_observation=invalidate,
        publish_fresh_start_observation=publish,
    )
    lifecycle.flush()
    assert lifecycle.is_flushed
    lifecycle.reinvoke()
    assert calls == [
        "pause",
        "engine_reset",
        "interpolator_reset",
        "invalidate_observation",
        "publish_fresh_observation",
        "resume",
    ]
    assert not lifecycle.is_flushed


def test_rollout_lifecycle_prepares_detector_before_resume():
    calls = []
    lifecycle = RolloutRecoveryLifecycle(
        inference_engine=FakeEngine(calls),
        action_interpolator=FakeInterpolator(calls),
        invalidate_cached_observation=lambda: calls.append("invalidate"),
        publish_fresh_start_observation=lambda: calls.append("publish") or {"gripper.pos": 37},
        prepare_fresh_start_observation=lambda obs: calls.append(("prepare", obs["gripper.pos"])),
    )
    lifecycle.flush()
    lifecycle.reinvoke()
    assert calls == [
        "pause",
        "engine_reset",
        "interpolator_reset",
        "invalidate",
        "publish",
        ("prepare", 37),
        "resume",
    ]


def test_rollout_lifecycle_refuses_resume_when_detector_seed_is_missing():
    calls = []
    lifecycle = RolloutRecoveryLifecycle(
        inference_engine=FakeEngine(calls),
        action_interpolator=FakeInterpolator(calls),
        invalidate_cached_observation=lambda: calls.append("invalidate"),
        publish_fresh_start_observation=lambda: None,
        prepare_fresh_start_observation=lambda obs: pytest.fail("must not prepare None"),
    )
    lifecycle.flush()
    with pytest.raises(RecoverySupervisorError, match="returned no observation"):
        lifecycle.reinvoke()
    assert "resume" not in calls


def test_rollout_lifecycle_refuses_resume_without_flush():
    calls = []
    lifecycle = RolloutRecoveryLifecycle(
        inference_engine=FakeEngine(calls),
        action_interpolator=FakeInterpolator(calls),
        invalidate_cached_observation=lambda: None,
        publish_fresh_start_observation=lambda: None,
    )
    with pytest.raises(RecoverySupervisorError, match="before a complete flush"):
        lifecycle.reinvoke()
    assert calls == []


def test_episode_start_reinvoke_failure_is_terminal():
    supervisor = RecoverySupervisor(
        flush_policy_queue=lambda: None,
        reinvoke_policy=lambda: (_ for _ in ()).throw(RuntimeError("cannot prime policy")),
        execute=lambda history, flush: RecoveryResult("center", 1, 2),
        history_frames=2,
    )
    with pytest.raises(RuntimeError, match="cannot prime policy"):
        supervisor.start_episode(1)
    assert supervisor.state is SupervisorState.ABORTED
