#!/usr/bin/env python
"""RTC queue generation guard for detector-triggered recovery.

Upstream ``RTCInferenceEngine.pause()`` is intentionally non-blocking. An inference that
already passed its active-event check can therefore finish after a recovery reset and try
to merge a pre-trigger chunk into the newly cleared queue. This module rejects such merges
by binding every inference cursor to a queue generation.

The live recovery adapter must construct ``RecoverySafeRTCInferenceEngine`` rather than
the upstream RTC engine. Nothing in this module enables physical recovery by itself.
"""

from __future__ import annotations

from dataclasses import dataclass
from threading import Thread

from lerobot.policies.rtc import ActionQueue
from lerobot.rollout.inference.rtc import RTCInferenceEngine


@dataclass(frozen=True)
class QueueCursor:
    generation: int
    action_index: int


class RecoverySafeActionQueue(ActionQueue):
    """Reject chunks whose inference began before the latest suspend/resume boundary."""

    def __init__(self, cfg) -> None:
        super().__init__(cfg)
        self._generation = 0
        self._accepting_merges = True

    def get_action_index(self) -> QueueCursor:
        with self.lock:
            return QueueCursor(self._generation, self.last_index)

    def suspend_and_clear(self) -> None:
        """Atomically stop accepting producers and invalidate every outstanding cursor."""
        with self.lock:
            self._accepting_merges = False
            self._generation += 1
            self.queue = None
            self.original_queue = None
            self.last_index = 0

    def resume_empty(self) -> None:
        """Open a new empty generation before the engine resumes production."""
        with self.lock:
            self._generation += 1
            self.queue = None
            self.original_queue = None
            self.last_index = 0
            self._accepting_merges = True

    def clear(self) -> None:
        """Invalidate outstanding inference while preserving suspended/running state."""
        with self.lock:
            self._generation += 1
            self.queue = None
            self.original_queue = None
            self.last_index = 0

    def merge(
        self,
        original_actions,
        processed_actions,
        real_delay: int,
        action_index_before_inference: QueueCursor | None = None,
    ) -> bool:
        """Merge only if the producer cursor belongs to the current running generation."""
        with self.lock:
            if not self._accepting_merges:
                return False
            if not isinstance(action_index_before_inference, QueueCursor):
                # Recovery safety depends on a generation-bearing cursor. Refuse an
                # untracked producer rather than silently degrading to upstream behavior.
                return False
            if action_index_before_inference.generation != self._generation:
                return False
            delay = self._check_and_resolve_delays(
                real_delay, action_index_before_inference.action_index
            )
            if self.cfg.enabled:
                self._replace_actions_queue(original_actions, processed_actions, delay)
            else:
                self._append_actions_queue(original_actions, processed_actions)
            return True


class RecoverySafeRTCInferenceEngine(RTCInferenceEngine):
    """RTC engine whose queue cannot be repopulated by pre-recovery inference."""

    def start(self) -> None:
        self._action_queue = RecoverySafeActionQueue(self._rtc_config)
        self._obs_holder = {"obs": None, "robot_type": self._robot.robot_type}
        self._shutdown_event.clear()
        self._rtc_thread = Thread(target=self._rtc_loop, daemon=True, name="RTCInference")
        self._rtc_thread.start()

    def pause(self) -> None:
        # Clear the producer event before invalidating cursors. A producer that already
        # passed the event check is rejected by the generation change below.
        self._policy_active.clear()
        queue = self._action_queue
        if isinstance(queue, RecoverySafeActionQueue):
            queue.suspend_and_clear()

    def resume(self) -> None:
        queue = self._action_queue
        if not isinstance(queue, RecoverySafeActionQueue):
            raise RuntimeError("recovery-safe RTC engine has no generation-guarded queue")
        # A second generation boundary rejects an inference that raced with pause/reset
        # and obtained a cursor while the engine was suspended.
        queue.resume_empty()
        self._policy_active.set()
