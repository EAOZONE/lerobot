#!/usr/bin/env python
"""Fail-closed state machine around detector-triggered recovery.

This module owns policy queue flushing, a bounded history of commands actually accepted
by the follower, per-episode retry accounting, structured trigger/recovery events, and
explicit policy reinvocation. It performs no robot I/O by itself. A live integration must
provide an executor backed by ``recovery.execute_recovery`` and may only do so after its
waypoint routes pass the physical protocol.
"""

from __future__ import annotations

import json
import math
import os
import re
import time
from collections import deque
from collections.abc import Callable, Sequence
from dataclasses import asdict, dataclass
from enum import StrEnum
from pathlib import Path
from typing import Any, Protocol

from probe_bus import SO101_MOTORS
from recovery import RAW_POSITION_MAX, RAW_POSITION_MIN, RawPose, RecoveryResult


class SupervisorState(StrEnum):
    READY = "ready"
    RECOVERING = "recovering"
    ABORTED = "aborted"
    EXHAUSTED = "exhausted"


class RecoverySupervisorError(RuntimeError):
    """Base error for fail-closed supervisor transitions."""


class RecoveryAttemptsExhaustedError(RecoverySupervisorError):
    """The per-episode recovery budget was already consumed."""


class RecoveryExecutor(Protocol):
    def __call__(
        self, command_history: Sequence[RawPose], flush_action_queue: Callable[[], None]
    ) -> RecoveryResult: ...


class ResettableInferenceEngine(Protocol):
    def pause(self) -> None: ...

    def reset(self) -> None: ...

    def resume(self) -> None: ...


class ResettableInterpolator(Protocol):
    def reset(self) -> None: ...


@dataclass(frozen=True)
class DetectorTrigger:
    detector: str
    score: float
    threshold: float
    frame_index: int
    fault_type: str | None = None

    def __post_init__(self) -> None:
        if not self.detector:
            raise ValueError("detector name must not be empty")
        if not math.isfinite(self.score) or not math.isfinite(self.threshold) or self.threshold <= 0:
            raise ValueError("trigger score and positive threshold must be finite")
        if self.frame_index < 0:
            raise ValueError("trigger frame_index must be non-negative")


@dataclass(frozen=True)
class RecoveryEvent:
    schema_version: int
    run_id: str
    episode_id: str
    attempt_id: str | None
    wall_time_ns: int
    monotonic_ns: int
    episode_index: int
    attempt: int
    event: str
    state: str
    detector: str | None = None
    score: float | None = None
    threshold: float | None = None
    trigger_frame: int | None = None
    fault_type: str | None = None
    route: str | None = None
    reverse_frames: int | None = None
    completed_frames: int | None = None
    detail: str | None = None


class RecoveryEventLogger:
    """Append and fsync one JSON object per supervisor transition."""

    def __init__(self, path: Path):
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)

    def __call__(self, event: RecoveryEvent) -> None:
        with self.path.open("a") as file:
            file.write(json.dumps(asdict(event), sort_keys=True) + "\n")
            file.flush()
            os.fsync(file.fileno())


class RolloutRecoveryLifecycle:
    """Concrete flush/reinvoke callbacks for sync and RTC rollout engines.

    ``flush`` pauses production before clearing every action-bearing layer. ``reinvoke``
    publishes a fresh episode-start observation before allowing production to resume.
    """

    def __init__(
        self,
        *,
        inference_engine: ResettableInferenceEngine,
        action_interpolator: ResettableInterpolator,
        invalidate_cached_observation: Callable[[], None],
        publish_fresh_start_observation: Callable[[], Any],
        prepare_fresh_start_observation: Callable[[Any], None] | None = None,
    ) -> None:
        self.inference_engine = inference_engine
        self.action_interpolator = action_interpolator
        self.invalidate_cached_observation = invalidate_cached_observation
        self.publish_fresh_start_observation = publish_fresh_start_observation
        self.prepare_fresh_start_observation = prepare_fresh_start_observation
        self.is_flushed = False

    def flush(self) -> None:
        """Pause first, then clear inference, interpolation, and observation state."""
        self.is_flushed = False
        self.inference_engine.pause()
        self.inference_engine.reset()
        self.action_interpolator.reset()
        self.invalidate_cached_observation()
        self.is_flushed = True

    def reinvoke(self) -> None:
        """Seed the cleared engine from the start state, then resume action production."""
        if not self.is_flushed:
            raise RecoverySupervisorError("cannot reinvoke rollout before a complete flush")
        fresh_observation = self.publish_fresh_start_observation()
        if self.prepare_fresh_start_observation is not None:
            if fresh_observation is None:
                raise RecoverySupervisorError(
                    "fresh start publisher returned no observation for detector reset"
                )
            self.prepare_fresh_start_observation(fresh_observation)
        self.inference_engine.resume()
        self.is_flushed = False


class RecoverySupervisor:
    """Coordinate one episode's detector-triggered recovery attempts."""

    def __init__(
        self,
        *,
        flush_policy_queue: Callable[[], None],
        reinvoke_policy: Callable[[], None],
        execute: RecoveryExecutor,
        history_frames: int,
        run_id: str = "unbound",
        max_recoveries: int = 2,
        on_event: Callable[[RecoveryEvent], None] | None = None,
    ) -> None:
        if history_frames <= 0:
            raise ValueError("history_frames must be positive")
        if max_recoveries <= 0:
            raise ValueError("max_recoveries must be positive")
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}", run_id):
            raise ValueError("run_id must be 1..128 safe identifier characters")
        self.flush_policy_queue = flush_policy_queue
        self.reinvoke_policy = reinvoke_policy
        self.execute = execute
        self.run_id = run_id
        self.max_recoveries = max_recoveries
        self.on_event = on_event
        self.command_history: deque[RawPose] = deque(maxlen=history_frames)
        self.episode_index = 0
        self.attempts = 0
        self.state = SupervisorState.READY

    def _emit(
        self,
        event: str,
        *,
        trigger: DetectorTrigger | None = None,
        event_attempt: int | None = None,
        **kwargs,
    ) -> None:
        if self.on_event is None:
            return
        selected_attempt = self.attempts if event_attempt is None else event_attempt
        selected_attempt_id = (
            f"{self.episode_id}:r{selected_attempt:02d}" if selected_attempt > 0 else None
        )
        self.on_event(
            RecoveryEvent(
                schema_version=2,
                run_id=self.run_id,
                episode_id=self.episode_id,
                attempt_id=selected_attempt_id,
                wall_time_ns=time.time_ns(),
                monotonic_ns=time.perf_counter_ns(),
                episode_index=self.episode_index,
                attempt=selected_attempt,
                event=event,
                state=self.state,
                detector=trigger.detector if trigger else None,
                score=trigger.score if trigger else None,
                threshold=trigger.threshold if trigger else None,
                trigger_frame=trigger.frame_index if trigger else None,
                fault_type=trigger.fault_type if trigger else None,
                **kwargs,
            )
        )

    @property
    def episode_id(self) -> str:
        return f"{self.run_id}:e{self.episode_index:06d}"

    @property
    def attempt_id(self) -> str | None:
        if self.attempts <= 0:
            return None
        return f"{self.episode_id}:r{self.attempts:02d}"

    def record_sent_command(self, command: RawPose) -> None:
        """Retain a copied raw command only after the follower accepted it."""
        if self.state is not SupervisorState.READY:
            raise RecoverySupervisorError(f"cannot record policy commands while state={self.state}")
        if set(command) != set(SO101_MOTORS):
            raise ValueError(f"command must contain exactly these motors: {', '.join(SO101_MOTORS)}")
        copied = {motor: int(command[motor]) for motor in SO101_MOTORS}
        outside = {
            motor: value
            for motor, value in copied.items()
            if not RAW_POSITION_MIN <= value <= RAW_POSITION_MAX
        }
        if outside:
            raise ValueError(f"command contains raw positions outside 0..4095: {outside}")
        self.command_history.append(copied)

    def recover(self, trigger: DetectorTrigger) -> RecoveryResult:
        """Flush, execute recovery, clear stale history, and explicitly reinvoke policy."""
        if self.state is not SupervisorState.READY:
            raise RecoverySupervisorError(f"cannot trigger recovery while state={self.state}")
        next_attempt = self.attempts + 1
        self._emit("trigger", trigger=trigger, event_attempt=next_attempt)
        if self.attempts >= self.max_recoveries:
            self.state = SupervisorState.EXHAUSTED
            self._emit(
                "recovery_rejected",
                trigger=trigger,
                event_attempt=next_attempt,
                detail="per-episode recovery cap reached",
            )
            raise RecoveryAttemptsExhaustedError(
                f"episode {self.episode_index} already used {self.max_recoveries} recoveries"
            )
        if not self.command_history:
            self.state = SupervisorState.ABORTED
            self._emit(
                "recovery_aborted",
                trigger=trigger,
                event_attempt=next_attempt,
                detail="accepted-command history is empty",
            )
            raise RecoverySupervisorError("cannot recover without accepted-command history")

        self.attempts += 1
        self.state = SupervisorState.RECOVERING
        self._emit("recovery_started", trigger=trigger)
        queue_flushed = False

        def verified_flush() -> None:
            nonlocal queue_flushed
            self.flush_policy_queue()
            queue_flushed = True

        try:
            result = self.execute(tuple(self.command_history), verified_flush)
            if not queue_flushed:
                raise RecoverySupervisorError("recovery executor returned without flushing policy queue")
            self.command_history.clear()
            self._emit(
                "recovery_completed",
                trigger=trigger,
                route=result.route,
                reverse_frames=result.reverse_frames,
                completed_frames=result.completed_frames,
            )
            self.reinvoke_policy()
        except Exception as exc:
            self.state = SupervisorState.ABORTED
            self._emit("recovery_aborted", trigger=trigger, detail=f"{type(exc).__name__}: {exc}")
            raise

        self.state = SupervisorState.READY
        try:
            self._emit("policy_reinvoked", trigger=trigger)
        except Exception:
            self.state = SupervisorState.ABORTED
            raise
        return result

    def start_episode(self, episode_index: int) -> None:
        """Flush and reinvoke at an explicit episode boundary, resetting attempt accounting."""
        if episode_index < 0:
            raise ValueError("episode_index must be non-negative")
        if self.state is SupervisorState.RECOVERING:
            raise RecoverySupervisorError("cannot start a new episode during recovery")
        try:
            self.flush_policy_queue()
            self.command_history.clear()
            self.episode_index = episode_index
            self.attempts = 0
            self.state = SupervisorState.READY
            self.reinvoke_policy()
            self._emit("episode_started")
        except Exception:
            self.state = SupervisorState.ABORTED
            raise
