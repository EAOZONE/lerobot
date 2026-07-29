# Arbitrary-pose recovery protocol

**Status:** software implemented; physical routes and exit test not yet validated.

## Fixed sequence

1. Read present position, write it back as the goal, and retain torque.
2. Flush queued future policy actions through the runtime hook.
3. Drop the configured fault-adjacent tail from the command ring buffer.
4. Replay a bounded recent command history in reverse with per-frame step limits.
5. Select exactly one validated workspace region from the measured pose.
6. Traverse that region's validated raw-position waypoints.
7. Replay the operator-recorded return-home trajectory and open the gripper.
8. Return a structured completion result. `RecoverySupervisor` then clears stale command
   history, explicitly reinvokes the policy, records the transition, and enforces the
   two-recovery episode cap. Recovery itself never silently restarts a policy.

Every commanded frame reads position, current, load, velocity, and voltage. Joint range,
current, following-error, phase-timeout, and command-step limits are mandatory. A missing,
overlapping, or unvalidated route aborts before recovery motion. Any runtime violation
halts by replacing the goal with measured position and propagates the failure.

## Configuration

Copy `recovery_waypoints.example.json` to a session-specific file. The example contains
zeros and `validated: false` intentionally and cannot move the robot. Populate limits from
health measurements and reduced-speed trials; do not guess them from the example.

Each route declares raw-tick `region_min`, `region_max`, ordered raw-tick `waypoints`, and a
physical-validation flag. Regions must be mutually exclusive over the intended task
workspace. Visual checks must cover the gripper, all links, table, camera hardware, and
every allowed object start region because joint limits alone do not establish Cartesian
clearance.

## Validation ladder

1. Review bounded paths offline and ensure region selection is unique.
2. With no objects, run one supervised recovery per representative region at reduced
   command-step limits, operator at the power switch.
3. Repeat with static objects at every allowed boundary.
4. Test slip, collision, awkward-release, and payload poses separately.
5. Run `audit_recovery.py` over every log using the exact route/configuration under review.
   It checks phase order, time, goal steps, joint range, current, following error, and the
   open endpoint and reports peaks. A numeric PASS cannot substitute for observed
   Cartesian clearance.
6. First trials use the validator's explicit, named `--supervised-trial-route` override;
   this does not alter the configuration. Mark a route `validated: true` only after its
   checklist passes.
7. Then run ten consecutive resets, with pre/post health diagnostics and zero manual
   repositioning. Until this happens, the Week 2 reset exit criterion remains unmet.

The prior lift-first path remains accessible only as `legacy-reset` with an explicit unsafe
acknowledgement. It is not an accepted recovery and must never run unattended.

The supervisor-to-rollout boundary and remaining acceptance tests are frozen in
`CLOSED_LOOP_RECOVERY_INTEGRATION.md`. No live strategy may attach it while any selected
route remains unvalidated.

After the ladder and ten-reset test, record the evidence using
`recovery_validation.schema.json` and run the immutable-token gate in
`RECOVERY_READINESS_GATE.md`. That auditor reruns every referenced numeric log and binds
the human clearance judgments, diagnostics, temperatures, checkpoint, and exact config
hashes. It cannot manufacture missing physical evidence and deliberately cannot accept the
superseded five-repeat soak.
