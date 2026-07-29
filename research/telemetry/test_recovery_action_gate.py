import pytest
from recovery import RecoveryResult
from recovery_action_gate import (
    DetectorDecision,
    OnlineD0ObservationScorer,
    PolicyTickBudget,
    RecoveryAwareActionGate,
    StepOutcome,
)
from recovery_supervisor import RecoverySupervisor

MOTORS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def pose(value):
    return dict.fromkeys(MOTORS, value)


class FakeSupervisor:
    def __init__(self, calls):
        self.calls = calls
        self.commands = []

    def recover(self, trigger):
        self.calls.append(("recover", trigger.frame_index))
        return RecoveryResult("center", 2, 20)

    def record_sent_command(self, command):
        self.calls.append(("record", command["shoulder_pan"]))
        self.commands.append(command)


def make_gate(calls, decision):
    return RecoveryAwareActionGate(
        supervisor=FakeSupervisor(calls),
        score_observation=lambda obs: calls.append(("score", obs["frame"])) or decision,
        compute_action=lambda obs: calls.append(("compute", obs["frame"])) or "queued-action",
        send_action=lambda action: calls.append(("send", action)),
        read_accepted_raw_command=lambda: calls.append(("readback", None)) or pose(123),
    )


def test_trigger_recovers_before_action_queue_is_popped_or_sent():
    calls = []
    gate = make_gate(calls, DetectorDecision("d0", 1.0, 1.2, "slip"))
    result = gate.step({"frame": 8}, frame_index=8)
    assert result.outcome is StepOutcome.RECOVERED
    assert result.policy_ticks == 1
    assert calls == [("score", 8), ("recover", 8)]


def test_nontrigger_orders_score_compute_send_readback_record():
    calls = []
    gate = make_gate(calls, DetectorDecision("d0", 1.0, 0.8))
    result = gate.step({"frame": 4}, frame_index=4)
    assert result.outcome is StepOutcome.ACTION_SENT
    assert calls == [
        ("score", 4),
        ("compute", 4),
        ("send", "queued-action"),
        ("readback", None),
        ("record", 123),
    ]


def test_unscorable_warmup_is_not_coerced_to_zero_or_triggered():
    calls = []
    gate = make_gate(calls, DetectorDecision("d0", 1.0, None))
    result = gate.step({"frame": 0}, frame_index=0)
    assert result.outcome is StepOutcome.ACTION_SENT
    assert not result.decision.scorable


def test_unavailable_rtc_action_sends_and_records_nothing():
    calls = []
    gate = RecoveryAwareActionGate(
        supervisor=FakeSupervisor(calls),
        score_observation=lambda obs: DetectorDecision("d0", 1.0, 0.5),
        compute_action=lambda obs: None,
        send_action=lambda action: pytest.fail("must not send None"),
        read_accepted_raw_command=lambda: pytest.fail("must not read back without a send"),
    )
    result = gate.step({}, frame_index=0)
    assert result.outcome is StepOutcome.ACTION_UNAVAILABLE
    assert result.policy_ticks == 1


def test_readback_failure_after_send_is_fail_closed_and_not_recorded():
    calls = []
    supervisor = FakeSupervisor(calls)
    gate = RecoveryAwareActionGate(
        supervisor=supervisor,
        score_observation=lambda obs: DetectorDecision("d0", 1.0, 0.5),
        compute_action=lambda obs: "action",
        send_action=lambda action: calls.append(("send", action)),
        read_accepted_raw_command=lambda: (_ for _ in ()).throw(RuntimeError("bus read failed")),
    )
    with pytest.raises(RuntimeError, match="bus read failed"):
        gate.step({}, frame_index=1)
    assert calls == [("send", "action")]
    assert not supervisor.commands


def test_real_supervisor_flushes_stale_rtc_queue_on_trigger():
    queue = ["stale-1", "stale-2"]
    sent = []

    def execute(history, flush):
        flush()
        return RecoveryResult("center", 1, 5)

    supervisor = RecoverySupervisor(
        flush_policy_queue=queue.clear,
        reinvoke_policy=lambda: None,
        execute=execute,
        history_frames=2,
    )
    supervisor.record_sent_command(pose(100))
    gate = RecoveryAwareActionGate(
        supervisor=supervisor,
        score_observation=lambda obs: DetectorDecision("d0", 1.0, 1.1),
        compute_action=lambda obs: queue.pop(0),
        send_action=sent.append,
        read_accepted_raw_command=lambda: pose(200),
    )
    result = gate.step({}, frame_index=3)
    assert result.outcome is StepOutcome.RECOVERED
    assert queue == []
    assert sent == []


def test_policy_tick_budget_counts_trigger_frame_but_not_recovery_motion():
    calls = []
    action_gate = make_gate(calls, DetectorDecision("d0", 1.0, 0.2))
    recovery_gate = make_gate(calls, DetectorDecision("d0", 1.0, 1.2))
    budget = PolicyTickBudget(max_ticks=2)
    action_result = action_gate.step({"frame": 0}, frame_index=0)
    budget.consume(action_result)
    assert budget.remaining_ticks == 1
    budget.consume(recovery_gate.step({"frame": 1}, frame_index=1))
    assert budget.exhausted
    with pytest.raises(RuntimeError, match="already exhausted"):
        budget.consume(action_result)


def test_d0_scorer_uses_last_accepted_normalized_goal_and_explicit_warmup():
    timestamps = iter([0.0, 1 / 30])
    scorer = OnlineD0ObservationScorer(
        initial_gripper_goal=10,
        clock=lambda: next(timestamps),
        window=3,
        raw_threshold=100,
    )
    first = scorer({"gripper.current": 250})
    assert first.score is None
    assert not first.scorable
    scorer.observe_accepted_action({"gripper.pos": 30})
    second = scorer({"gripper.current": 200})
    assert second.score == 0.0
    assert scorer.detector.goals == [10.0, 30.0]


def test_d0_scorer_reset_clears_history_and_reenters_warmup():
    timestamps = iter([0.0, 1.0])
    scorer = OnlineD0ObservationScorer(initial_gripper_goal=10, clock=lambda: next(timestamps))
    scorer({"gripper.current": 200})
    scorer.reset(initial_gripper_goal=50)
    decision = scorer({"gripper.current": 100})
    assert decision.score is None
    assert scorer.detector.goals == [50.0]


def test_d0_scorer_resets_only_from_normalized_start_observation():
    scorer = OnlineD0ObservationScorer(initial_gripper_goal=10, clock=lambda: 0.0)
    scorer({"gripper.current": 200})
    scorer.reset_from_observation({"gripper.pos": 42})
    assert scorer.current_gripper_goal == 42
    assert scorer.detector.goals == []
    with pytest.raises(KeyError, match="fresh start observation"):
        scorer.reset_from_observation({"goal_pos.gripper": 2048})


def test_gate_updates_detector_goal_from_send_result_before_raw_readback():
    calls = []
    scorer = OnlineD0ObservationScorer(initial_gripper_goal=10, clock=lambda: 0.0)
    gate = RecoveryAwareActionGate(
        supervisor=FakeSupervisor(calls),
        score_observation=scorer,
        compute_action=lambda obs: {"gripper.pos": 20},
        send_action=lambda action: calls.append(("send", action)) or {"gripper.pos": 19},
        observe_accepted_action=lambda action: calls.append(("observe", action))
        or scorer.observe_accepted_action(action),
        read_accepted_raw_command=lambda: calls.append(("readback", None)) or pose(100),
    )
    gate.step({"gripper.current": 200}, frame_index=0)
    assert scorer.current_gripper_goal == 19
    assert calls == [
        ("send", {"gripper.pos": 20}),
        ("observe", {"gripper.pos": 19}),
        ("readback", None),
        ("record", 100),
    ]


def test_gate_refuses_to_guess_accepted_goal_when_send_returns_none():
    scorer = OnlineD0ObservationScorer(initial_gripper_goal=10, clock=lambda: 0.0)
    gate = RecoveryAwareActionGate(
        supervisor=FakeSupervisor([]),
        score_observation=scorer,
        compute_action=lambda obs: {"gripper.pos": 20},
        send_action=lambda action: None,
        observe_accepted_action=scorer.observe_accepted_action,
        read_accepted_raw_command=lambda: pytest.fail("must stop before raw readback"),
    )
    with pytest.raises(RuntimeError, match="returned no accepted action"):
        gate.step({"gripper.current": 200}, frame_index=0)
