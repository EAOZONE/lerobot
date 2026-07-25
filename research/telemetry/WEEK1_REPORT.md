# Week 1 — telemetry go/no-go: **PASS**

**Platform:** SO-101 follower + leader, Feetech STS3215, 30 Hz
**Question:** do slip and collision produce a telemetry signature visible without statistics?
**Answer:** yes, for both — but the usable channel is `Present_Current`, not `Present_Load`.

Per the proposal's §9 risk table, "no usable telemetry signature" was the critical
risk and the reason this gate exists. It did not materialise. The project proceeds;
the language-grounding fallback is not needed.

---

## 1. Result

### Slip — gripper current collapse

Three isolated runs, one deliberate slip each, against three clean controls.

| feature | clean | slip |
|---|---|---|
| `grip_curr_held` (mean current while gripping) | 198, 250, 262 | 207, 283, 326 |
| `slip_onset_drop` | 0, 6, 17 | **305, 374, 386** |

Zero overlap, ~18× separation. The signature is a sustained grip current (~200–390)
that collapses to <20 within two samples while the *commanded* gripper position does
not move. That last condition is what distinguishes a slip from a deliberate release —
without it, the clean runs' release scored 471 and looked identical.

**Detection lead time**, measured from the collapse to the operator's keypress:
`slip_a` +0.27s, `slip_b` +0.07s, `slip_c` +0.80s. The keypress itself carries roughly
0.2–0.3s of human reaction lag, so true lead on the physical event is perhaps 0.3–1.1s.
At 30 Hz that is 8–33 samples — enough to halt, likely not enough for elaborate
recovery. This is directly relevant to H4, which already predicts a modest net gain.

**Unprompted true positive:** `slip_a` contains *two* slips. The operator marked only
the second; the detector found the first at t=7.19s (current 337→46, jaws stationary,
no commanded change) before the object was re-grasped and slipped again at t=12.20s.

### Collision — arm current excess against a matched trajectory

Three obstacle runs replayed against a clear baseline of the same recorded episode, so
both runs execute an identical commanded trajectory and align frame-for-frame.

| pair | peak current excess | vs baseline | arm divergence | contact |
|---|---|---|---|---|
| pair1 | +120 | 25× | 12.7° | light resistance |
| pair2 | +154 | 64× | 7.6° | two pushes, 0.5s apart |
| pair3 | **+1707** | **244×** | **38.1°** | hard stall |

`pair3` is the reference exemplar: `shoulder_pan` current ramps 281→1844 in 0.3s, then
position **freezes for 2.3 seconds** while the commanded goal sweeps 36° away and
velocity holds at zero. Bus voltage sags 12.0→11.6V.

---

## 2. Findings that change how we build the detectors

**Current beats load, and not marginally.**

- `Present_Load` saturates. The gripper clips at ±500 (`Max_Torque_Limit`, written by
  `SOFollower.configure()`, `so_follower.py:169`) for 15.6% of one run. The arm joints
  clip at ±1000 — 100% of scale by definition of the units — and `shoulder_pan` sat
  pinned there for the entire `pair3` stall, carrying no information about severity.
  Current meanwhile ran to 1844 and kept discriminating.
- `Present_Load` is also noisy across replays. On `pair1`, smoothed load excess peaked
  at 5.3× baseline spread over **three** separate episodes with its maximum **2.7s away
  from the actual contact**. Smoothed current peaked at 25× in **one** episode exactly
  on the contact.

Treat current as the primary channel and load as corroboration. This holds for both
failure modes, from independent evidence.

**Smoothing is not optional.** Contact is sustained over tenths of a second; replay
variance is single-frame spikes. Unsmoothed, load excess scored 11.6× on a pair that
contained *no contact at all*. A 5-frame (~0.17s) mean separates them.

**Peak magnitude alone never worked.** Whole-file peak load and peak current do not
separate collision from ordinary teleoperation — 450–565 vs 456–528 on load, 172–217 vs
166–195 on current, complete overlap. Normal motion loads joints exactly as hard as a
collision does. What separates is *shape*: sustained excess against a matched control.

**Bus voltage is specific but not sensitive.** The 0.6V sag during the `pair3` stall
correlated with current at **+0.99**. But on the light and moderate contacts there was
no distinguishable sag at all — their largest voltage excursions landed 3.5s and 7.2s
away from the actual contact — and ordinary clean runs swing 0.3–0.4V unprompted. So a
large sag means something serious, but most collisions produce none. Recorded as a
stall-severity indicator (taxonomy `E6`), never as a trigger on its own.

---

## 3. What went wrong first, and why it matters for the protocol

Three failed attempts preceded the result. Each was a protocol defect, not a hardware
one, and each is worth stating in the paper's method section.

**`Present_Current` read flat zero.** Not a firmware limitation — torque was disabled.
`FeetechMotorsBus.connect()` does not enable torque and `SOFollower.disconnect()`
disables it, so an inherited limp arm is the default state between sessions. With the
driver idle, load and current are ~0 by construction no matter how hard the arm is
pushed by hand. Both are *drive* measurements, not force measurements.

**Batch 1 (145s continuous, 9 typed markers) was unusable.** Markers typed as words +
Enter landed 3–4s after the event — far wider than the event itself. Within that
uncertainty, a `clean` motion (peak load 664, current 474) was indistinguishable from
all three collisions (644–731, 469–475). Fixed by single-keypress markers (~0.25s) and
by recording one isolated event per file so the file *is* the window.

**Batch 2's slip runs never gripped anything.** Gripper current sat at 2–3 throughout
every "slip" — there was no grip force to lose, so what got captured was the aftermath
(jaws closing on empty), not the onset. Detection lead time would have been ≤0. Fixed
by a live `GRIP OK` readout so the operator confirms a real grasp *before* initiating
the slip. Grip current went from 57–64 to 206–326 on re-recording.

**Batch 2's collision controls were not matched.** Comparing a collide run against a
differently-executed clean run cannot resolve a signal that normal motion also
produces. Fixed by replaying one recorded episode with and without an obstacle.

A further trap: an obstacle that slides or tips produces no collision at all. The first
obstacle attempt showed arm position diverging by only 5° from the clear run — the arm
had reached the same place both times, so nothing had stopped it.

---

## 4. Hardware incidents

**`shoulder_lift` latched an overload alarm** after holding all six joints under torque
for 20s — it is the gravity-loaded joint. A latched Feetech servo rejects every
subsequent write until the servo power is cycled (unplugging USB is not enough). The
`scservo` SDK names only error bits 1/2/4/8/32, so the rejection surfaced as
`RuntimeError: Failed to write ... after 1 tries.` with an **empty** explanation.

Mitigations now in the tooling: `diagnose.py` prints the raw alarm byte; scripts refuse
to enable torque on an alarmed servo and say what to do; `--hold` energises only the
joints actually needed.

**`shoulder_pan` held ~12A for 2.3s during `pair3`.** That is a genuine stall current
and the mechanism by which these servos burn out. The exemplar is captured and does not
need repeating — corpus collisions should stay at pair1/pair2 intensity. Order the two
spare STS3215 units if that has not happened yet (proposal §9).

---

## 5. Implications for the proposal

- **§2.3 stands.** Servo telemetry carries real force-adjacent information on this
  hardware. The premise of the cost ladder is sound.
- **Amend the load/current framing.** The proposal treats "load and current" as one
  signal. They are not interchangeable: load saturates precisely when the failure is
  most severe. D0 should threshold current; D0+ can use both.
- **Lead time is short.** 0.3–1.1s for slip. Scripted recovery (§5.7) must be fast, and
  H4's "modest" gain should be read as genuinely modest.
- **Add a platform-constraints paragraph.** Gripper load clipping at 50% of range is a
  LeRobot default (`Max_Torque_Limit=500`), not a hardware limit — worth documenting
  since it is invisible unless you look for it. Do not raise it: that guard protects a
  servo we have already alarmed once.
- **n=3.** This is a signature test, not an accuracy estimate. No AUROC or lead-time
  distribution should be claimed from it.

---

## 6. Artifacts

**Data** — `research/telemetry/runs/`: 9 isolated single-event runs
(`clean_*`, `collide_*`, `slip_*`), 3 matched replay pairs (`pair*_obstacle`,
`pair1_clear`), and `collision_01.csv` (the failed continuous batch, kept as the
worked example of marker-lag failure).

**Code** — `research/telemetry/`:

| File | Role |
|---|---|
| `feetech_block.py` | one-transaction read of addr 56–70, shared timestamp |
| `log_teleop_telemetry.py` | teleop or replay logging, live grip readout, keypress markers |
| `gate_analysis.py` | grip-state features, slip onset, matched-pair contact test |
| `plot_signatures.py` | the eyeball artifact |
| `probe_bus.py`, `diagnose.py`, `scan_registers.py` | bus/alarm/register diagnostics |
| `so_follower_telemetry.py`, `record_with_telemetry.py`, `truncate_state_step.py` | Week 2 recording path (built, verified in software, not yet run on hardware) |

Nothing in `src/lerobot/` was modified.

---

## 7. Next

1. Verify the 30 fps record loop holds with cameras attached (`NEXT_STEPS.md` step 5).
   Two bus transactions plus cameras is the tight spot — decide fps/camera count
   *before* the corpus, not during.
2. Run `diagnose.py` on `shoulder_pan` after the `pair3` stall.
3. Schema frozen at 30 dims (`pos · load · current · vel · volt`). `Status` (65) and
   `Present_Temperature` (63) deliberately excluded — see `NEXT_STEPS.md` §1 for the
   reasoning and the lab-log substitutes.
4. Open question worth one run: is there a *precursor* before the slip collapse? In
   `slip_a` cycle 1 current drifted 344→337 over 1.6s before letting go. If that is
   micro-slip rather than thermal drift, it would buy considerably more lead time than
   0.3s. A deliberately slow slide would settle it.
