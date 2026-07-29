# Closed-loop recovery integration contract

**Status:** recovery stack and composed sync/RTC loops tested offline; not attached to live
rollout because no recovery route is physically validated. This document is an integration
boundary, not authorization to move the robot.

## Ownership

`RecoverySupervisor` owns the state that must survive across detector and policy calls:

- a bounded raw-tick ring buffer of commands read back after the follower accepts them;
- the detector name, causal score, operating threshold, trigger frame, and likely fault;
- mandatory policy queue flushing inside the recovery call;
- at most two successful or attempted recoveries per episode;
- fail-closed `ready`, `recovering`, `aborted`, and `exhausted` states;
- clearing stale commands after successful return-home motion;
- one explicit callback that reinvokes the policy from the episode start state; and
- fsynced JSONL events for trigger, start, abort/rejection, completion, and reinvocation.

`execute_recovery()` continues to own physical motion and its per-frame telemetry CSV.
The rollout integration continues to own task time, episode outcome, retry termination,
object/environment reset, and labels. A supervisor event is never ground truth for whether
the rollout truly failed or whether a recovery saved it.

## Required live adapter

After all routes and the ten-reset exit test pass, a rollout strategy may construct the
supervisor along these lines:

```python
verified_token = verify_recovery_enablement(
    token_path=enablement_token_path,
    validation_record_path=validation_record_path,
    waypoint_path=waypoint_path,
    detector_config_path=detector_config_path,
    checkpoint_path=checkpoint_path,
    run_manifest_path=run_manifest_path,
    inference_engine=inference_engine,
)

d0_scorer = OnlineD0ObservationScorer(
    # This is the normalized gripper position held at the declared episode start,
    # never the raw Goal_Position register value.
    initial_gripper_goal=normalized_start_observation["gripper.pos"],
)

lifecycle = RolloutRecoveryLifecycle(
    inference_engine=inference_engine,
    action_interpolator=action_interpolator,
    invalidate_cached_observation=invalidate_cached_observation,
    publish_fresh_start_observation=publish_fresh_start_observation,
    prepare_fresh_start_observation=d0_scorer.reset_from_observation,
)

supervisor = RecoverySupervisor(
    flush_policy_queue=lifecycle.flush,
    reinvoke_policy=lifecycle.reinvoke,
    execute=lambda history, flush: execute_recovery(
        follower.bus,
        history,
        home_trajectory,
        recovery_config,
        flush_action_queue=flush,
        on_route=recovery_csv_logger.bind_route,
        on_frame=recovery_csv_logger,
    ),
    history_frames=recovery_config.limits.reverse_frames
    + recovery_config.limits.fault_guard_frames,
    max_recoveries=2,
    on_event=RecoveryEventLogger(event_path),
)

gate = RecoveryAwareActionGate(
    supervisor=supervisor,
    score_observation=d0_scorer,
    compute_action=pop_or_compute_next_policy_action,
    send_action=send_to_follower,
    read_accepted_raw_command=read_goal_position_raw,
    observe_accepted_action=d0_scorer.observe_accepted_action,
)
budget = PolicyTickBudget(max_ticks=round(episode_seconds * fps))
control_loop = RecoveryControlLoop(gate, budget)
```

Token verification must be the first recovery-specific operation after constructing the
inference engine and before constructing `RecoverySupervisor`, the executor callback, or
the action gate. It recomputes every bound artifact hash and compares the runtime engine's
fully qualified class. A stale or mismatched token raises `RecoveryEnablementError`; the
ordinary supervised episodic strategy may continue only with recovery absent, never by
catching the error and enabling an unverified fallback.

At each episode boundary call `start_episode(index)` after the environment is physically at
its declared start state. That call performs both flush and reinvocation and therefore does
not leave an RTC engine paused. After every policy command is sent,
read back `Goal_Position` in raw ticks and call `record_sent_command()` only if the write
succeeded. `RecoveryAwareActionGate.step()` enforces this order: score the current causal
observation; on a crossing recover without calling the action provider; otherwise obtain
and send one action, pass the follower's returned (possibly safety-clipped) normalized
action to D0, read the raw goal registers back for recovery history, and retain that raw
pose. The two command representations are intentionally separate: D0 conditions on
`gripper.pos` in the policy/state normalization domain, while reverse replay requires raw
servo ticks. Never seed D0 from `Goal_Position` readback. Call `control_loop.step(...)`,
not `gate.step(...)`, from the strategy. Its `finally` boundary consumes exactly one
`PolicyTickBudget` tick even when a terminal detector/recovery exception returns no result.
Every observed autonomous frame counts once, including the trigger frame, while the
synchronous recovery motion adds no ticks. If the gate raises or the
supervisor enters `aborted`/`exhausted`, halt and terminate the episode; never fall through
to another policy action.

The executor receives the queue-flush callback and must invoke it. The existing
`execute_recovery()` does so immediately after halt and before route motion. The supervisor
checks that it happened and refuses to reinvoke otherwise. Flush the rollout inference
engine, not only `policy.reset()`: the engine owns processor state and the RTC queue, while
the strategy's interpolator may still hold a partially served action. Pausing first is
mandatory for RTC, but upstream's non-blocking pause is not sufficient to prevent an
already-running producer from refilling the queue; use the generation guard specified
below.
`RolloutRecoveryLifecycle` implements and tests this exact order and refuses reinvocation
without a completed flush.

The first D0 observation after episode start or recovery is warm-up and returns `None`.
`RolloutRecoveryLifecycle.reinvoke()` passes the freshly published normalized start
observation to `d0_scorer.reset_from_observation()` before resuming action production. This
clears pre-fault current/goal history. The concrete adapter must return that observation
from its publisher, not convert the raw recovery waypoint by assumption. If `send_action()`
does not return an accepted action, or the accepted action lacks `gripper.pos`, halt: the
gate deliberately refuses to guess detector command state.

### RTC generation guard

Do not use the upstream `RTCInferenceEngine` directly for recovery-enabled rollout.
Its `pause()` clears an event but does not wait for an inference already in flight. Such an
inference can finish after `reset()` and merge its pre-trigger chunk back into the cleared
queue. `recovery_rtc_guard.py` supplies `RecoverySafeRTCInferenceEngine` and a
generation-bearing action queue. Suspend and resume each invalidate all outstanding
producer cursors; a late merge is rejected atomically. A deterministic threaded test
reproduces the finish-after-recovery ordering and proves the queue stays empty. The future
strategy factory must select this engine whenever detector-triggered recovery and RTC are
both enabled.

## Integration acceptance status

Do not merge the adapter into a live strategy until all of these are demonstrated:

1. Every configured route is physically `validated: true` and all supporting CSVs pass
   `audit_recovery.py`.
2. Ten consecutive attended resets pass with pre/post health records and no manual
   repositioning.
3. **Passed offline 28 July:** composed mocked sync and RTC strategy loops prove that no
   queued or interpolated pre-trigger action is sent after the threshold crossing,
   including on a chunk-recompute
   frame and while the RTC producer is active. They compose the real D0 scorer, action gate,
   supervisor, lifecycle, interpolator reset, tick budget, and generation-guarded queue.
   The sync loop suppresses trigger-frame compute/pop and discards an old chunk; the RTC
   loop lets a pre-trigger producer finish after recovery, rejects its merge, then accepts
   only a fresh-generation action. See `test_closed_loop_recovery_integration.py`.
4. A hardware dry run proves accepted-command readback and detector scoring fit the 30 Hz
   loop; it must also prove the scorer receives the follower's normalized, clipped send
   result and is reset from the normalized start observation after recovery. Report
   recompute and queue-read populations separately.
5. **Passed offline; live wiring open:** composed loops consume
   `PolicyTickBudget` across ordinary, recovered, and terminal-exception frames. Recovery
   motion adds no ticks, and an exhausted budget prevents another gate call.
6. **Passed offline; hardware demonstration open:** a composed stable-command D0 loop
   proves first and second recoveries resume fresh policy execution; the third trigger
   enters `exhausted` and raises before recovery motion, action-provider access, or send.
7. **Software contract passed offline 28 July; live evidence open:** schema-v2 supervisor
   events, exclusive per-attempt recovery CSVs, an immutable run manifest, append-only
   outcomes, and `audit_recovery_evidence.py` now join policy/chunk configuration through
   final outcome under canonical run/episode/attempt IDs. Synthetic complete, conflicting,
   and missing cases produce their expected verdicts. One real recovery-enabled run must
   still obtain an audit PASS.

Until those tests pass, `rollout_with_telemetry.py` remains supervised episodic evaluation
without detector-triggered recovery.
