#!/usr/bin/env python
"""D0r: predict the current a healthy, unobstructed arm should be drawing, and
threshold what is left over.

Measured servo current is the sum of two things -- the effort needed to move the arm
through free space (gravity, friction, inertia, payload) and the effort produced by
external contact -- and only the sum is observable. Week 1 separated them by replaying
one trajectory twice, once clear and once with an obstacle, and diffing frame-for-frame.
That works experimentally and cannot ship: at runtime there is no twin to diff against.

This replaces the physical twin with a predicted one:

    residual(t) = measured current(t) - predicted free-space current(t)

FACTR 2 (arXiv 2606.12406) makes the same argument and validates the approach.

Model. Linear in parameters over a physically motivated basis -- the classic rigid-body
regressor, tau ~ M(q)qdd + C(q,qd)qd + g(q) + friction -- solved in closed form by ridge.
Deliberately not a gradient-boosted or neural model: this rung's claim is that it is
cheap, and a dot product per control step is defensible in a way that a GBM forward pass
is less so. It also needs no dependency beyond numpy, and the coefficients can be read.

Two design decisions that matter, both easy to get wrong:

  1. FEATURES ARE COMMANDED, NEVER MEASURED. Everything derives from `goal_pos.*`. Feed
     it measured position and the model gains access to following error, which is a
     direct contact signal -- it would learn to explain contact away, and the residual
     would go flat exactly where it is needed. Conditioning on the commanded trajectory
     also keeps inputs in-distribution during a fault: the controller is asking for
     something ordinary, so the model predicts ordinary current, and the excess is clean.
  2. TRAINING DATA MUST CONTAIN NO CONTACT. This is a normality model. Contact is by
     construction the thing it cannot explain. One collision in the training set and it
     learns collisions are normal.

The gripper is not a target. Its current tracks grasp state rather than contact -- that
is D0's business (see gate_analysis.py). Commanded gripper position IS a feature, because
holding a payload changes gravity load on every joint above it.

Result on the Week 1 pairs (26 Jul), fitted on 113s of no-contact motion. At the default
p99 floor all three collisions are found, on shoulder_pan -- the joint that actually
collided -- and at the right moment:

    run                 x floor   peak t    matched-pair contact t
    pair1_obstacle          3.3    7.95s    7.95s
    pair2_obstacle          3.9    9.06s    8.86s
    pair3_obstacle         28.4    7.42s    7.39s
    pair1_clear             0.7        -    (no contact -- correctly silent)

So the collision signature survives WITHOUT a matched control, which is what Week 1 left
open. One caveat: clean_a scores 1.2x, a false positive, so at this operating point the
false-alarm rate is not zero and n=4 negatives cannot pin it down.

The floor percentile IS the operating point and the choice is not cosmetic. At p95
everything reads as contact. At p99.9 pair1 and pair2 still print CONTACT, but on
wrist_flex at 9.19s and 9.33s -- more than a second from the real event. They are
coincidental crossings, not detections. Always check the peak time and joint against
causal_eval.py's matched-pair output before believing a verdict.

Usage:
    python research/telemetry/freespace_model.py fit \
        research/telemetry/runs/freespace_*.csv \
        --out research/telemetry/models/freespace.npz

    python research/telemetry/freespace_model.py eval \
        --model research/telemetry/models/freespace.npz \
        research/telemetry/runs/pair1_clear.csv \
        research/telemetry/runs/pair*_obstacle.csv

    # sweep the operating point -- do this before trusting any single verdict
    for p in 95 99 99.9; do
        python research/telemetry/freespace_model.py eval --floor-pct $p runs/*.csv
    done
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
JOINTS = [*ARM, "gripper"]

# Trailing frames used to smooth commanded velocity before differencing again for
# acceleration. goal_pos is quantised, so a raw double difference is mostly noise.
VEL_SMOOTH = 5
# Lags (frames) at which past commanded velocity is offered to the model. FACTR 2's
# point: instantaneous state cannot separate free-space effort from contact effort;
# the model needs some history to do it.
LAGS = (2, 5, 10)
# Residual smoothing at eval time. Matches the window that causal_eval.py measured as
# best for D0, so the two rungs are judged on the same timescale.
RESIDUAL_SMOOTH = 10
RIDGE_ALPHAS = (0.1, 1.0, 10.0, 100.0, 1000.0)
# Percentiles of held-out free-space residual stored as candidate detection floors.
FLOOR_PCTS = (50, 95, 99, 99.9)
# Frames discarded at the start of every run, for fitting and for scoring alike. The
# velocity, acceleration and lag features are all zero-initialised, so until the buffer
# fills the model is predicting from history it does not have. Left in, this produced a
# textbook false positive: pair1_clear -- a run with NO contact -- scored 2.2x the floor
# at t=0.13s, and the same artefact outranked the genuine contacts in pair1 and pair2.
# A causal detector simply cannot score before its buffer is full; saying so is honest.
WARMUP = max(LAGS) + VEL_SMOOTH + RESIDUAL_SMOOTH
# Contiguous blocks for cross-validation. Adjacent frames at 30 Hz are near-duplicates,
# so a random split leaks the answer across folds and every alpha looks excellent.
CV_BLOCKS = 6


def _trailing_mean(x: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return x
    return (
        pd.DataFrame(x).rolling(window, min_periods=1).mean().to_numpy()
        if x.ndim > 1
        else pd.Series(x).rolling(window, min_periods=1).mean().to_numpy()
    )


def _trailing_diff(x: np.ndarray, dt: float) -> np.ndarray:
    """Causal derivative. First sample is zero, never a backfilled future value."""
    d = np.zeros_like(x)
    d[1:] = (x[1:] - x[:-1]) / dt
    return d


def feature_names() -> list[str]:
    names = []
    for m in ARM:
        names += [f"sin({m})", f"cos({m})"]
    names += ["sin(lift+elbow)", "cos(lift+elbow)"]
    for m in JOINTS:
        names += [f"v({m})", f"|v|({m})", f"sgn_v({m})", f"a({m})"]
    names += [f"a({m})*reach" for m in JOINTS]
    names += [f"|v|({m})*reach" for m in JOINTS]
    names += ["grip_cmd", "grip_closed"]
    for lag in LAGS:
        names += [f"v({m})@-{lag}" for m in JOINTS]
    return names


def build_features(df: pd.DataFrame) -> np.ndarray:
    """Commanded-trajectory features only. See module docstring for why."""
    t = df["t"].to_numpy(float)
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 1 / 30.0
    goal = np.column_stack([df[f"goal_pos.{m}"].to_numpy(float) for m in JOINTS])

    # Arm goals are degrees, the gripper is a 0-100 normalised aperture.
    q = np.radians(goal[:, : len(ARM)])
    grip = goal[:, len(ARM)]

    v = _trailing_mean(_trailing_diff(goal, dt), VEL_SMOOTH)
    a = _trailing_diff(v, dt)

    cols = []
    # Gravity: pose-dependent hold torque. The lift+elbow sum is the two-link coupling
    # that dominates on this arm -- shoulder_lift carries the whole forearm.
    for i in range(len(ARM)):
        cols += [np.sin(q[:, i]), np.cos(q[:, i])]
    lift_elbow = q[:, 1] + q[:, 2]
    cols += [np.sin(lift_elbow), np.cos(lift_elbow)]

    # Viscous friction (v), load-independent losses (|v|), Coulomb friction (sign v),
    # and inertia (a). Coulomb is why sign matters: friction flips with direction.
    for i in range(len(JOINTS)):
        cols += [v[:, i], np.abs(v[:, i]), np.sign(v[:, i]), a[:, i]]

    # Configuration-dependent inertia. The effective inertia a joint must accelerate
    # depends on how far the arm is extended -- a folded arm spins up far more easily
    # than an outstretched one. This matters most for shoulder_pan, which rotates about
    # the vertical and therefore has no gravity term at all: its current is friction and
    # inertia only, and without these terms it was much the worst-fit joint.
    reach = np.cos(q[:, 1]) + np.cos(lift_elbow)
    cols += [a[:, i] * reach for i in range(len(JOINTS))]
    cols += [np.abs(v[:, i]) * reach for i in range(len(JOINTS))]

    # Payload proxy. Jaws commanded shut usually means something is held, and a held
    # object changes gravity load on every joint above the gripper. Without this the
    # model has never seen a payload and every grasp reads as contact.
    cols += [grip, (grip < np.percentile(grip, 25)).astype(float)]

    for lag in LAGS:
        lagged = np.vstack([np.repeat(v[:1], lag, axis=0), v[:-lag]]) if lag < len(v) else np.zeros_like(v)
        cols += [lagged[:, i] for i in range(len(JOINTS))]

    return np.column_stack(cols)


def _ridge_fit(X: np.ndarray, y: np.ndarray, alpha: float) -> np.ndarray:
    """Closed-form ridge on standardised, centred data. No intercept penalty."""
    n_features = X.shape[1]
    gram = X.T @ X + alpha * np.eye(n_features)
    return np.linalg.solve(gram, X.T @ y)


def _standardise(X: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    mu = X.mean(axis=0)
    sigma = X.std(axis=0)
    sigma[sigma < 1e-9] = 1.0
    return mu, sigma


def _block_folds(n: int, blocks: int) -> list[np.ndarray]:
    """Contiguous blocks, not a random split -- neighbouring frames are near-duplicates."""
    edges = np.linspace(0, n, blocks + 1).astype(int)
    return [np.arange(edges[i], edges[i + 1]) for i in range(blocks)]


def fit(paths: list[Path], out: Path) -> None:
    frames, sources = [], []
    for p in paths:
        df = pd.read_csv(p, keep_default_na=False)
        frames.append(df)
        sources.append(p.name)
        print(f"  {p.name}: {len(df)} frames, {df['t'].iloc[-1]:.1f}s")

    X = np.vstack([build_features(df)[WARMUP:] for df in frames])
    Y = np.column_stack(
        [np.concatenate([df[f"curr.{m}"].to_numpy(float)[WARMUP:] for df in frames]) for m in ARM]
    )
    print(f"\n  {X.shape[0]} frames ({WARMUP} warm-up frames dropped per run), "
          f"{X.shape[1]} features, {len(ARM)} targets")

    mu, sigma = _standardise(X)
    Xs = (X - mu) / sigma

    folds = _block_folds(len(Xs), CV_BLOCKS)
    weights, intercepts, chosen, scores = [], [], [], []
    for j, motor in enumerate(ARM):
        y = Y[:, j]
        best_alpha, best_mae = RIDGE_ALPHAS[0], np.inf
        for alpha in RIDGE_ALPHAS:
            errs = []
            for fold in folds:
                mask = np.ones(len(Xs), bool)
                mask[fold] = False
                y_tr = y[mask]
                w = _ridge_fit(Xs[mask], y_tr - y_tr.mean(), alpha)
                pred = Xs[fold] @ w + y_tr.mean()
                errs.append(np.abs(pred - y[fold]).mean())
            mae = float(np.mean(errs))
            if mae < best_mae:
                best_alpha, best_mae = alpha, mae

        w = _ridge_fit(Xs, y - y.mean(), best_alpha)
        pred = Xs @ w + y.mean()
        ss_res = float(((y - pred) ** 2).sum())
        ss_tot = float(((y - y.mean()) ** 2).sum())
        r2 = 1 - ss_res / max(ss_tot, 1e-9)
        weights.append(w)
        intercepts.append(float(y.mean()))
        chosen.append(best_alpha)
        scores.append((motor, best_alpha, best_mae, r2, float(y.mean()), float(y.std())))

    print(f"\n  {'joint':<15} {'alpha':>7} {'CV MAE':>8} {'in-sample R2':>13} "
          f"{'target mean':>12} {'target sd':>10}")
    for motor, alpha, mae, r2, m, sd in scores:
        print(f"  {motor:<15} {alpha:>7.1f} {mae:>8.1f} {r2:>13.3f} {m:>12.1f} {sd:>10.1f}")

    # The number that decides whether this rung is usable. Held-out free-space residual
    # is what a run with no contact will produce, so any contact has to clear it. Several
    # percentiles are stored rather than one, because the choice IS the operating point:
    # p99.9 is set by a handful of pathological frames (on shoulder_pan, p99 is 63 and
    # p99.9 is 253) and buys specificity at the cost of the lightest contacts.
    print("\n  held-out free-space residual (the noise floor contact must clear):")
    W = np.array(weights).T
    b = np.array(intercepts)
    floor_table = np.zeros((len(ARM), len(FLOOR_PCTS)))
    for j, motor in enumerate(ARM):
        held = []
        for fold in folds:
            mask = np.ones(len(Xs), bool)
            mask[fold] = False
            y_tr = Y[mask, j]
            w = _ridge_fit(Xs[mask], y_tr - y_tr.mean(), chosen[j])
            resid = Y[fold, j] - (Xs[fold] @ w + y_tr.mean())
            held.append(_trailing_mean(np.clip(resid, 0, None), RESIDUAL_SMOOTH))
        h = np.concatenate(held)
        floor_table[j] = [np.percentile(h, p) for p in FLOOR_PCTS]
        cells = "  ".join(f"p{p:g} {v:>6.1f}" for p, v in zip(FLOOR_PCTS, floor_table[j], strict=True))
        print(f"    {motor:<15} {cells}  max {h.max():>7.1f}")

    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        out,
        weights=W,
        intercepts=b,
        mu=mu,
        sigma=sigma,
        alphas=np.array(chosen),
        arm=np.array(ARM),
        floor_table=floor_table,
        floor_pcts=np.array(FLOOR_PCTS),
        sources=np.array(sources),
        feature_names=np.array(feature_names()),
    )
    print(f"\n  wrote {out}")


def predict(model: dict, df: pd.DataFrame) -> np.ndarray:
    X = build_features(df)
    Xs = (X - model["mu"]) / model["sigma"]
    return Xs @ model["weights"] + model["intercepts"]


def evaluate(model: dict, paths: list[Path], floor_pct: float) -> None:
    arm = [str(m) for m in model["arm"]]
    pcts = list(model["floor_pcts"])
    if floor_pct not in pcts:
        raise SystemExit(f"--floor-pct must be one of {pcts}")
    col = pcts.index(floor_pct)
    floors = {m: float(model["floor_table"][j, col]) for j, m in enumerate(arm)}

    print(f"\n=== residual on {len(paths)} run(s), trailing smoothing {RESIDUAL_SMOOTH} frames ===")
    print(f"  free-space p{floor_pct:g} floor per joint: "
          + ", ".join(f"{m}={floors[m]:.0f}" for m in arm))
    print(f"\n  {'run':<24} {'peak':>7} {'joint':<14} {'t':>7} {'x floor':>8} {'sust':>6}  verdict")

    for path in paths:
        df = pd.read_csv(path, keep_default_na=False)
        pred = predict(model, df)
        measured = np.column_stack([df[f"curr.{m}"].to_numpy(float) for m in arm])
        # One-sided: contact ADDS effort. A joint drawing less than predicted is the
        # model being wrong, not a fault, and folding it in would only add noise.
        resid = np.clip(measured - pred, 0, None)
        resid = np.column_stack([_trailing_mean(resid[:, j], RESIDUAL_SMOOTH) for j in range(len(arm))])

        scaled = (resid / np.array([floors[m] for m in arm]))[WARMUP:]
        if len(scaled) == 0:
            print(f"  {path.name:<24} too short to score ({len(df)} frames, need >{WARMUP})")
            continue
        flat = scaled.max(axis=1)
        k = int(flat.argmax())
        joint = arm[int(scaled[k].argmax())]
        t_peak = float(df["t"].iloc[k + WARMUP])
        resid = resid[WARMUP:]
        # Contact is sustained; model error is spiky. Count frames above the floor.
        sustained = int((flat > 1.0).sum())

        if flat.max() < 1.0:
            verdict = "no contact (within model error)"
        elif sustained < 3:
            verdict = "SPIKE ONLY -- likely model error"
        else:
            verdict = "CONTACT"
        print(f"  {path.name:<24} {resid[k].max():>7.0f} {joint:<14} {t_peak:>6.2f}s "
              f"{flat.max():>8.1f} {sustained:>6}  {verdict}")

    print("\n  'x floor' is the peak residual as a multiple of the held-out free-space")
    print("  p99.9 for that joint. Below 1.0 the excursion is indistinguishable from")
    print("  the model's own error. A *_clear run scoring above 1.0 means the floor is")
    print("  optimistic and the model needs more or better free-space coverage.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_fit = sub.add_parser("fit", help="train on no-contact runs")
    p_fit.add_argument("csv", nargs="+", type=Path)
    p_fit.add_argument("--out", type=Path, default=Path("research/telemetry/models/freespace.npz"))

    p_eval = sub.add_parser("eval", help="score runs against a fitted model")
    p_eval.add_argument("csv", nargs="+", type=Path)
    p_eval.add_argument("--model", type=Path, default=Path("research/telemetry/models/freespace.npz"))
    p_eval.add_argument(
        "--floor-pct",
        type=float,
        default=99.0,
        help="percentile of held-out free-space residual used as the detection floor; "
        "this IS the operating point (default: 99)",
    )

    args = parser.parse_args()

    if args.cmd == "fit":
        print(f"fitting free-space model on {len(args.csv)} run(s)")
        for p in args.csv:
            if any(tag in p.stem for tag in ("obstacle", "collide", "slip")):
                raise SystemExit(
                    f"{p.name} looks like it contains a fault. This is a NORMALITY model --"
                    "\ntraining on contact teaches it that contact is normal and the residual"
                    "\ngoes flat exactly where it is needed. Use freespace_* runs only."
                )
        fit(args.csv, args.out)
    else:
        model = dict(np.load(args.model, allow_pickle=True))
        print(f"model: {args.model}  (trained on {', '.join(str(s) for s in model['sources'])})")
        evaluate(model, args.csv, args.floor_pct)


if __name__ == "__main__":
    main()
