from threading import Event, Thread

import torch
from recovery_rtc_guard import RecoverySafeActionQueue

from lerobot.policies.rtc.configuration_rtc import RTCConfig


def queue():
    return RecoverySafeActionQueue(RTCConfig(enabled=True, execution_horizon=2))


def actions(value):
    return torch.full((2, 1), float(value))


def test_inflight_pretrigger_chunk_cannot_merge_after_suspend_or_resume():
    action_queue = queue()
    stale_cursor = action_queue.get_action_index()

    action_queue.suspend_and_clear()
    assert not action_queue.merge(actions(1), actions(1), 0, stale_cursor)
    assert action_queue.empty()

    action_queue.resume_empty()
    assert not action_queue.merge(actions(1), actions(1), 0, stale_cursor)
    assert action_queue.empty()


def test_producer_that_races_during_pause_is_invalidated_at_resume():
    action_queue = queue()
    action_queue.suspend_and_clear()
    paused_cursor = action_queue.get_action_index()

    action_queue.resume_empty()
    assert not action_queue.merge(actions(2), actions(2), 0, paused_cursor)
    assert action_queue.empty()

    fresh_cursor = action_queue.get_action_index()
    assert action_queue.merge(actions(3), actions(3), 0, fresh_cursor)
    assert action_queue.get().item() == 3


def test_active_producer_finishing_after_recovery_cannot_refill_queue():
    action_queue = queue()
    inference_started = Event()
    allow_finish = Event()
    merge_result = []

    def producer():
        cursor = action_queue.get_action_index()
        inference_started.set()
        allow_finish.wait(timeout=1)
        merge_result.append(action_queue.merge(actions(9), actions(9), 0, cursor))

    thread = Thread(target=producer)
    thread.start()
    assert inference_started.wait(timeout=1)

    action_queue.suspend_and_clear()
    action_queue.resume_empty()
    allow_finish.set()
    thread.join(timeout=1)

    assert not thread.is_alive()
    assert merge_result == [False]
    assert action_queue.empty()
