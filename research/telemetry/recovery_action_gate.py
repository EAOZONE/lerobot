#!/usr/bin/env python
"""Causal detector boundary between observation and policy action dispatch."""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass
from enum import StrEnum
from typing import Any

from detectors import OnlineD0
from recovery import RawPose, RecoveryResult
from recovery_supervisor import DetectorTrigger, RecoverySupervisor


class StepOutcome(StrEnum):
    ACTION_SENT = "action_sent"
    ACTION_UNAVAILABLE = "action_unavailable"
    RECOVERED = "recovered"


@dataclass(frozen=True)
class DetectorDecision:
    """One causal score for the current frame; unscorable warm-up is explicit."""

    detector: str
    threshold: float
    score: float | None
    fault_type: str | None = None

    def __post_init__(self) -> None:
        if not self.detector:
            raise ValueError("detector name must not be empty")
        if not math.isfinite(self.threshold) or self.threshold <= 0:
            raise ValueError("detector threshold must be finite and positive")
        if self.score is not None and not math.isfinite(self.score):
            raise ValueError("detector score must be finite or None for warm-up")

    @property
    def scorable(self) -> bool:
        return self.score is not None

    @property
    def triggered(self) -> bool:
        return self.score is not None and self.score >= self.threshold


@dataclass(frozen=True)
class GatedStepResult:
    outcome: StepOutcome
    policy_ticks: int
    decision: DetectorDecision
    recovery: RecoveryResult | None = None


@dataclass
class PolicyTickBudget:
    """Count autonomous observation ticks while excluding recovery motion wall time."""

    max_ticks: int
    consumed_ticks: int = 0

    def __post_init__(self) -> None:
        if self.max_ticks <= 0:
            raise ValueError("max_ticks must be positive")
        if not 0 <= self.consumed_ticks <= self.max_ticks:
            raise ValueError("consumed_ticks must be within 0..max_ticks")

    def consume(self, result: GatedStepResult) -> None:
        """Consume the tick count from a successful gate return."""
        self.consume_ticks(result.policy_ticks)

    def consume_ticks(self, ticks: int = 1) -> None:
        """Consume observed autonomous ticks, including a terminal exception frame."""
        if self.exhausted:
            raise RuntimeError("policy tick budget is already exhausted")
        if ticks <= 0:
            raise ValueError("tick count must be positive")
        self.consumed_ticks = min(self.max_ticks, self.consumed_ticks + ticks)

    @property
    def remaining_ticks(self) -> int:
        return self.max_ticks - self.consumed_ticks

    @property
    def exhausted(self) -> bool:
        return self.consumed_ticks >= self.max_ticks


class RecoveryControlLoop:
    """Budgeted step boundary that counts terminal trigger/error observations."""

    def __init__(self, gate: RecoveryAwareActionGate, budget: PolicyTickBudget) -> None:
        self.gate = gate
        self.budget = budget

    def step(self, observation: dict[str, Any], frame_index: int) -> GatedStepResult:
        if self.budget.exhausted:
            raise RuntimeError("policy tick budget is already exhausted")
        try:
            return self.gate.step(observation, frame_index)
        finally:
            self.budget.consume_ticks()


class OnlineD0ObservationScorer:
    """Adapt live telemetry and the last accepted normalized gripper goal to D0."""

    def __init__(
        self,
        *,
        initial_gripper_goal: float,
        current_key: str = "gripper.current",
        action_key: str = "gripper.pos",
        clock: Callable[[], float] = time.perf_counter,
        window: int = 10,
        raw_threshold: float = 100.0,
    ) -> None:
        self.current_key = current_key
        self.action_key = action_key
        self.clock = clock
        self.detector = OnlineD0(window=window, raw_threshold=raw_threshold)
        self.current_gripper_goal = self._finite(initial_gripper_goal, "initial_gripper_goal")

    @staticmethod
    def _finite(value: Any, label: str) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{label} must be numeric") from exc
        if not math.isfinite(result):
            raise ValueError(f"{label} must be finite")
        return result

    def __call__(self, observation: dict[str, Any]) -> DetectorDecision:
        if self.current_key not in observation:
            raise KeyError(f"live observation is missing {self.current_key!r}")
        current = self._finite(observation[self.current_key], self.current_key)
        score = self.detector.update(
            t=self._finite(self.clock(), "monotonic detector timestamp"),
            gripper_current=current,
            gripper_goal=self.current_gripper_goal,
        )
        return DetectorDecision("d0", threshold=1.0, score=score, fault_type="slip")

    def observe_accepted_action(self, accepted_action: Any) -> None:
        """Advance the held goal only from the follower's accepted normalized action."""
        if not isinstance(accepted_action, dict) or self.action_key not in accepted_action:
            raise KeyError(f"accepted action is missing {self.action_key!r}")
        self.current_gripper_goal = self._finite(accepted_action[self.action_key], self.action_key)

    def reset(self, *, initial_gripper_goal: float) -> None:
        """Clear causal history after recovery/episode reset and seed the new held goal."""
        self.detector.reset()
        self.current_gripper_goal = self._finite(initial_gripper_goal, "initial_gripper_goal")

    def reset_from_observation(self, observation: dict[str, Any]) -> None:
        """Reset from a fresh normalized start observation before policy resume."""
        if self.action_key not in observation:
            raise KeyError(f"fresh start observation is missing {self.action_key!r}")
        self.reset(initial_gripper_goal=observation[self.action_key])


class RecoveryAwareActionGate:
    """Guarantee score-before-pop/send ordering for one autonomous control frame.

    Every call consumes exactly one policy-execution tick. Physical recovery runs inside
    the triggered call but contributes no extra policy ticks, so callers can hold episode
    budgets constant in autonomous execution time rather than recovery wall time.
    """

    def __init__(
        self,
        *,
        supervisor: RecoverySupervisor,
        score_observation: Callable[[dict[str, Any]], DetectorDecision],
        compute_action: Callable[[dict[str, Any]], Any | None],
        send_action: Callable[[Any], Any],
        read_accepted_raw_command: Callable[[], RawPose],
        observe_accepted_action: Callable[[Any], None] | None = None,
    ) -> None:
        self.supervisor = supervisor
        self.score_observation = score_observation
        self.compute_action = compute_action
        self.send_action = send_action
        self.read_accepted_raw_command = read_accepted_raw_command
        self.observe_accepted_action = observe_accepted_action

    def step(self, observation: dict[str, Any], frame_index: int) -> GatedStepResult:
        if frame_index < 0:
            raise ValueError("frame_index must be non-negative")
        decision = self.score_observation(observation)
        if decision.triggered:
            recovery = self.supervisor.recover(
                DetectorTrigger(
                    detector=decision.detector,
                    score=decision.score,
                    threshold=decision.threshold,
                    frame_index=frame_index,
                    fault_type=decision.fault_type,
                )
            )
            return GatedStepResult(StepOutcome.RECOVERED, 1, decision, recovery)

        action = self.compute_action(observation)
        if action is None:
            return GatedStepResult(StepOutcome.ACTION_UNAVAILABLE, 1, decision)
        accepted_action = self.send_action(action)
        if self.observe_accepted_action is not None:
            if accepted_action is None:
                raise RuntimeError(
                    "send_action returned no accepted action; cannot advance detector command state"
                )
            self.observe_accepted_action(accepted_action)
        accepted = self.read_accepted_raw_command()
        self.supervisor.record_sent_command(accepted)
        return GatedStepResult(StepOutcome.ACTION_SENT, 1, decision)
