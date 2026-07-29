from threading import Event, Thread

import pytest
import torch
from recovery import RecoveryResult
from recovery_action_gate import (
    OnlineD0ObservationScorer,
    PolicyTickBudget,
    RecoveryAwareActionGate,
    RecoveryControlLoop,
    StepOutcome,
)
from recovery_rtc_guard import RecoverySafeActionQueue
from recovery_supervisor import (
    RecoveryAttemptsExhaustedError,
    RecoverySupervisor,
    RolloutRecoveryLifecycle,
    SupervisorState,
)

from lerobot.policies.rtc.configuration_rtc import RTCConfig

MOTORS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def pose(value):
    return dict.fromkeys(MOTORS, value)


class MockInterpolator:
    def __init__(self, calls):
        self.calls = calls
        self.pending = []

    def reset(self):
        self.calls.append("interpolator_reset")
        self.pending.clear()


class MockSyncEngine:
    def __init__(self, calls):
        self.calls = calls
        self.paused = False
        self.policy_chunk = []
        self.next_value = 20

    def pause(self):
        self.calls.append("pause")
        self.paused = True

    def reset(self):
        assert self.paused
        self.calls.append("engine_reset")
        self.policy_chunk.clear()

    def resume(self):
        self.calls.append("resume")
        self.paused = False

    def compute_or_pop(self, observation):
        self.calls.append(("compute_or_pop", observation["frame"]))
        if not self.policy_chunk:
            self.calls.append(("recompute", observation["frame"]))
            self.policy_chunk.extend(
                [{"gripper.pos": self.next_value}, {"gripper.pos": self.next_value + 1}]
            )
            self.next_value += 10
        return self.policy_chunk.pop(0)


class MockGuardedRTCEngine:
    def __init__(self, calls):
        self.calls = calls
        self.queue = RecoverySafeActionQueue(RTCConfig(enabled=True, execution_horizon=2))

    def pause(self):
        self.calls.append("pause")
        self.queue.suspend_and_clear()

    def reset(self):
        self.calls.append("engine_reset")
        self.queue.clear()

    def resume(self):
        self.calls.append("resume")
        self.queue.resume_empty()

    def pop(self, observation):
        self.calls.append(("rtc_pop", observation["frame"]))
        action = self.queue.get()
        if action is None:
            return None
        return {"gripper.pos": action.item()}

    def publish(self, value):
        cursor = self.queue.get_action_index()
        chunk = torch.tensor([[float(value)], [float(value + 1)]])
        assert self.queue.merge(chunk, chunk, 0, cursor)


def make_runtime(engine, compute_action, calls):
    ticks = iter(range(100))
    scorer = OnlineD0ObservationScorer(
        initial_gripper_goal=20,
        clock=lambda: next(ticks) / 30,
        window=1,
        raw_threshold=100,
    )
    interpolator = MockInterpolator(calls)
    lifecycle = RolloutRecoveryLifecycle(
        inference_engine=engine,
        action_interpolator=interpolator,
        invalidate_cached_observation=lambda: calls.append("invalidate"),
        publish_fresh_start_observation=lambda: calls.append("publish_start")
        or {"gripper.pos": 20},
        prepare_fresh_start_observation=scorer.reset_from_observation,
    )

    def execute(history, flush):
        calls.append(("recover_history", len(history)))
        flush()
        calls.append("recovery_motion")
        return RecoveryResult("center", reverse_frames=len(history), completed_frames=5)

    supervisor = RecoverySupervisor(
        flush_policy_queue=lifecycle.flush,
        reinvoke_policy=lifecycle.reinvoke,
        execute=execute,
        history_frames=4,
    )

    def send(action):
        calls.append(("send", action["gripper.pos"]))
        return action

    gate = RecoveryAwareActionGate(
        supervisor=supervisor,
        score_observation=scorer,
        compute_action=compute_action,
        send_action=send,
        observe_accepted_action=scorer.observe_accepted_action,
        read_accepted_raw_command=lambda: pose(100),
    )
    return scorer, interpolator, supervisor, gate


def run_step(gate, budget, frame, current):
    result = gate.step({"frame": frame, "gripper.current": current}, frame_index=frame)
    budget.consume(result)
    return result


def test_complete_sync_loop_suppresses_trigger_recompute_and_stale_chunk():
    calls = []
    engine = MockSyncEngine(calls)
    scorer, interpolator, supervisor, gate = make_runtime(engine, engine.compute_or_pop, calls)
    budget = PolicyTickBudget(max_ticks=3)
    supervisor.start_episode(0)

    assert run_step(gate, budget, 0, 250).outcome is StepOutcome.ACTION_SENT
    # This action would be served from the old sync policy chunk if recovery failed to
    # reset it. The trigger frame must not even enter compute_or_pop.
    assert engine.policy_chunk == [{"gripper.pos": 21}]
    interpolator.pending.append({"gripper.pos": 999})
    assert run_step(gate, budget, 1, 0).outcome is StepOutcome.RECOVERED
    assert not engine.policy_chunk
    assert not interpolator.pending
    assert scorer.current_gripper_goal == 20

    assert run_step(gate, budget, 2, 250).outcome is StepOutcome.ACTION_SENT
    assert budget.exhausted
    assert ("compute_or_pop", 1) not in calls
    assert ("recompute", 2) in calls
    assert ("send", 21) not in calls


def test_complete_rtc_loop_rejects_active_stale_producer_and_resumes_fresh():
    calls = []
    engine = MockGuardedRTCEngine(calls)
    scorer, interpolator, supervisor, gate = make_runtime(engine, engine.pop, calls)
    budget = PolicyTickBudget(max_ticks=3)
    supervisor.start_episode(0)
    engine.publish(20)

    assert run_step(gate, budget, 0, 250).outcome is StepOutcome.ACTION_SENT

    producer_started = Event()
    finish_inference = Event()
    stale_merge = []

    def pretrigger_producer():
        cursor = engine.queue.get_action_index()
        producer_started.set()
        finish_inference.wait(timeout=1)
        chunk = torch.tensor([[90.0], [91.0]])
        stale_merge.append(engine.queue.merge(chunk, chunk, 0, cursor))

    producer = Thread(target=pretrigger_producer)
    producer.start()
    assert producer_started.wait(timeout=1)
    interpolator.pending.append({"gripper.pos": 999})

    assert run_step(gate, budget, 1, 0).outcome is StepOutcome.RECOVERED
    finish_inference.set()
    producer.join(timeout=1)
    assert stale_merge == [False]
    assert engine.queue.empty()
    assert not interpolator.pending
    assert ("rtc_pop", 1) not in calls

    engine.publish(30)
    assert run_step(gate, budget, 2, 250).outcome is StepOutcome.ACTION_SENT
    assert budget.exhausted
    assert ("send", 90) not in calls
    assert ("send", 30) in calls


def test_two_recoveries_resume_but_third_trigger_terminates_without_motion_or_action():
    calls = []
    engine = MockSyncEngine(calls)
    _scorer, _interpolator, supervisor, gate = make_runtime(
        engine, engine.compute_or_pop, calls
    )
    budget = PolicyTickBudget(max_ticks=6)
    loop = RecoveryControlLoop(gate, budget)
    supervisor.start_episode(0)

    for warmup_frame, trigger_frame in ((0, 1), (2, 3)):
        # Hold the gripper command stable so the real D0 rule treats the current drop as
        # slip, not a deliberate release. Other joints/chunk identity are irrelevant here.
        engine.next_value = 20
        assert loop.step(
            {"frame": warmup_frame, "gripper.current": 250}, warmup_frame
        ).outcome is StepOutcome.ACTION_SENT
        assert loop.step(
            {"frame": trigger_frame, "gripper.current": 0}, trigger_frame
        ).outcome is StepOutcome.RECOVERED
        assert supervisor.state is SupervisorState.READY

    assert supervisor.attempts == 2
    engine.next_value = 20
    assert loop.step({"frame": 4, "gripper.current": 250}, 4).outcome is StepOutcome.ACTION_SENT
    motion_before_third = calls.count("recovery_motion")
    sends_before_third = [call for call in calls if isinstance(call, tuple) and call[0] == "send"]
    provider_calls_before_third = [
        call for call in calls if isinstance(call, tuple) and call[0] == "compute_or_pop"
    ]

    with pytest.raises(RecoveryAttemptsExhaustedError):
        loop.step({"frame": 5, "gripper.current": 0}, frame_index=5)

    assert supervisor.state is SupervisorState.EXHAUSTED
    assert calls.count("recovery_motion") == motion_before_third == 2
    assert [call for call in calls if isinstance(call, tuple) and call[0] == "send"] == (
        sends_before_third
    )
    assert [
        call for call in calls if isinstance(call, tuple) and call[0] == "compute_or_pop"
    ] == provider_calls_before_third
    assert budget.exhausted
    assert calls.count("publish_start") == 3  # episode start + two successful recoveries
