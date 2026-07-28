# D0r trajectory-B hold-out result

**Evaluated 27 July 2026 against the choices locked in
[`D0R_HOLDOUT_PROTOCOL.md`](./D0R_HOLDOUT_PROTOCOL.md). Primary verdict: FAIL.**

The collision signal transfers strongly, including to joints other than
`shoulder_pan`, but the trajectory-A model does not preserve specificity on the new
clear trajectory. The p99 operating point is therefore not trajectory/session robust.
This result was recorded before fitting or inspecting a trajectory-B model.

## Data integrity

| run | frames | duration | role |
|---|---:|---:|---|
| `freespace_b_01.csv` | 649 | 21.68 s | trajectory-B no-contact model data |
| `pair4_clear.csv` | 649 | 21.68 s | held-out matched negative |
| `pair4_obstacle.csv` | 649 | 21.68 s | held-out multi-contact positive |

The clear and obstacle files contain exactly identical `goal_pos.*` values for all six
motors at all 649 frames. The CSV replay path therefore produced a valid matched-command
pair.

The obstacle run contains five `collision` keypresses at 1.272, 5.491, 10.908, 15.157,
and 20.680 s. Marker timing includes operator reaction delay.

## Locked primary evaluation: train A, test B

Model: `freespace_a.npz`, fitted only on `freespace_01.csv`. Operating point: p99.

| run | peak × floor | peak joint | peak time | alert episodes | verdict |
|---|---:|---|---:|---:|---|
| `pair4_clear` | **4.32** | `wrist_roll` | 12.01 s | 6 | **false positive** |
| `pair4_obstacle` | **21.67** | `shoulder_pan` | 5.89 s | 9 | contact |

The pre-registered pass condition required a sustained obstacle crossing and no clear-run
crossing. The clear run crossed repeatedly, so the primary result fails regardless of the
strong obstacle response. No threshold or percentile was changed after seeing this.

The clear false positives were not isolated single frames. Sustained p99 episodes included
`wrist_flex` at 9.07–9.40 s (2.42×) and `wrist_roll` at 11.88–12.28 s (4.32×),
14.29–14.69 s (3.47×), and 15.49–15.90 s (3.44×).

## Physical contact evidence

Matched obstacle-minus-clear position divergence of at least 150 raw ticks supports four
distinct physically consequential contacts:

| approximate event | affected joint(s) | divergence interval | peak divergence |
|---:|---|---|---:|
| 1 | `shoulder_lift`, `elbow_flex` | 1.64–2.24 s | 287 ticks |
| 2 | `shoulder_pan` | 5.56–6.06 s | 253 ticks |
| 4 | `shoulder_lift` | 12.65–16.09 s | 386 ticks |
| 5 | `elbow_flex` | 20.81–21.08 s | 194 ticks |

The third keypress at 10.908 s has no ≥150-tick matched divergence. It may represent
light contact that did not materially stop the arm, so it must not be counted as a
validated collision solely because a key was pressed.

D0r produced strong residual episodes on the physically affected non-pan joints:

- event 1: `elbow_flex` 18.48× and `shoulder_lift` 7.46×;
- event 4: `shoulder_lift` up to 8.24×;
- event 5: `elbow_flex` 12.95×.

This supports transfer of the mechanical signal across joints. It does not rescue the
detector, because the clear-run false-alarm requirement is equally necessary.

## Multi-event reporting caveat

The common report temporally matched five threshold crossings to five keypress markers.
That raw 5/5 must not be quoted as detector recall:

- the third marker lacks matched position-divergence evidence;
- trajectory-induced crossings also occur in the clear replay;
- contacts 3 and 4 may overlap, making event boundaries ambiguous;
- temporal proximity alone cannot distinguish a causal precursor from a shared
  trajectory false positive.

`detectors.py` was updated to retain and match multiple markers rather than silently using
only the first one. Final corpus evaluation still requires adjudicated event boundaries
and physical-validity labels, not keypresses alone.

## Allowed secondary analysis: train B

After recording the failed A→B primary result, a model was fitted only on
`freespace_b_01.csv`.

| evaluation | clear peak | obstacle peak | result |
|---|---:|---:|---|
| train B → pair4 B | 0.6× | 45.1× | separates B |
| train B → pair1 A | 1.3× | 5.8× | A clear false positive |
| train B → pair2/3 A | — | 6.8× / 50.5× | collisions remain strong |

The B-fitted model fixes B's clear trajectory but becomes false-positive on A's clear
trajectory. This is evidence of trajectory/session-specific normality coverage, not a
reason to replace the locked A model with B.

## Observed session shift

Trajectory-A free-space training began around 40–43 °C. Trajectory B and both pair4 runs
began around 44–49 °C, a roughly 5–8 °C joint-dependent shift. Temperature may contribute,
but trajectory and session changed together, so this experiment cannot identify the
cause. Do not claim thermal causality from this pair.

## Conclusion and next decision

D0r remains a strong contact signature but is not yet a deployable detector at a fixed
p99 operating point. Its current free-space model learns too narrow a normal envelope.

Before another held-out claim:

1. separate trajectory shift from thermal/session shift with repeated clear trajectories;
2. train on multiple no-contact trajectories and sessions using training-only data;
3. define calibration-only threshold selection and keep a new trajectory C untouched;
4. retain per-joint and event-level outputs rather than only a whole-run maximum;
5. include autonomous successful rollouts as hard negatives.

Trajectory B is now observed data. It may be used for development/training but can never
serve as an untouched hold-out again.
