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
# Command-space coverage is a diagnostic, not a contact score.  A normality model
# cannot distinguish contact from extrapolation when the requested motion is unlike
# its free-space training data, so evaluation must surface that condition explicitly.
COVERAGE_PCT = 99.0
COVERAGE_CHUNK = 512


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


def _validation_folds(block_lengths: list[int]) -> tuple[list[np.ndarray], str]:
    """Leave out whole runs when possible; use contiguous blocks for one run only."""
    if len(block_lengths) > 1:
        offsets = np.cumsum([0, *block_lengths])
        folds = [np.arange(offsets[i], offsets[i + 1]) for i in range(len(block_lengths))]
        return folds, f"leave-one-run-out ({len(folds)} folds)"
    folds = _block_folds(block_lengths[0], CV_BLOCKS)
    return folds, f"contiguous blocks ({len(folds)} folds; one-run fallback)"


def _nearest_distances(query: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """Euclidean nearest-neighbour distance without allocating the full distance matrix."""
    if len(reference) == 0:
        raise ValueError("coverage reference must contain at least one frame")
    result = np.empty(len(query), dtype=float)
    for start in range(0, len(query), COVERAGE_CHUNK):
        block = query[start : start + COVERAGE_CHUNK]
        squared = ((block[:, None, :] - reference[None, :, :]) ** 2).sum(axis=2)
        result[start : start + len(block)] = np.sqrt(squared.min(axis=1))
    return result


def _coverage_floor(Xs: np.ndarray, folds: list[np.ndarray]) -> float:
    """Held-out command-space distance; folds follow the same run grouping as model CV."""
    held = []
    for fold in folds:
        mask = np.ones(len(Xs), bool)
        mask[fold] = False
        # A one-run fallback has a training complement. With one whole run per fold,
        # multi-run CV likewise measures distance to independently recorded motion.
        held.append(_nearest_distances(Xs[fold], Xs[mask]))
    return float(np.percentile(np.concatenate(held), COVERAGE_PCT))


def fit(paths: list[Path], out: Path) -> None:
    frames, sources = [], []
    for p in paths:
        df = pd.read_csv(p, keep_default_na=False)
        frames.append(df)
        sources.append(p.name)
        print(f"  {p.name}: {len(df)} frames, {df['t'].iloc[-1]:.1f}s")

    feature_blocks = [build_features(df)[WARMUP:] for df in frames]
    X = np.vstack(feature_blocks)
    Y = np.column_stack(
        [np.concatenate([df[f"curr.{m}"].to_numpy(float)[WARMUP:] for df in frames]) for m in ARM]
    )
    print(f"\n  {X.shape[0]} frames ({WARMUP} warm-up frames dropped per run), "
          f"{X.shape[1]} features, {len(ARM)} targets")

    mu, sigma = _standardise(X)
    Xs = (X - mu) / sigma

    # A random or within-run split leaks near-identical trajectory frames into both
    # sides and estimates only interpolation error. Deployment sees whole new runs,
    # so model selection and residual floors must leave out complete runs whenever
    # more than one independent no-contact recording is available.
    folds, cv_description = _validation_folds([len(block) for block in feature_blocks])
    print(f"  validation: {cv_description}")
    coverage_floor = _coverage_floor(Xs, folds)
    print(f"  command coverage: held-out p{COVERAGE_PCT:g} nearest distance = {coverage_floor:.2f}")
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
        coverage_reference=Xs,
        coverage_floor=np.array(coverage_floor),
        coverage_pct=np.array(COVERAGE_PCT),
    )
    print(f"\n  wrote {out}")


def predict(model: dict, df: pd.DataFrame) -> np.ndarray:
    X = build_features(df)
    Xs = (X - model["mu"]) / model["sigma"]
    return Xs @ model["weights"] + model["intercepts"]


def coverage_ratio(model: dict, df: pd.DataFrame) -> np.ndarray | None:
    """Return commanded-feature distance / held-out floor, or None for legacy models."""
    if "coverage_reference" not in model or "coverage_floor" not in model:
        return None
    Xs = (build_features(df) - model["mu"]) / model["sigma"]
    floor = max(float(model["coverage_floor"]), 1e-9)
    return _nearest_distances(Xs, model["coverage_reference"]) / floor


def clear_run_residual_peaks(model: dict, df: pd.DataFrame) -> np.ndarray:
    """Maximum causal residual per joint; one vector is one independent calibration unit."""
    arm = [str(m) for m in model["arm"]]
    pred = predict(model, df)
    measured = np.column_stack([df[f"curr.{m}"].to_numpy(float) for m in arm])
    residual = np.clip(measured - pred, 0, None)
    smoothed = np.column_stack(
        [_trailing_mean(residual[:, j], RESIDUAL_SMOOTH) for j in range(len(arm))]
    )
    if len(smoothed) <= WARMUP:
        raise ValueError(f"clear run has {len(df)} frames; need more than {WARMUP}")
    return smoothed[WARMUP:].max(axis=0)


def _conformal_rank(n_runs: int, alpha: float) -> int:
    """One-based split-conformal order statistic for rollout false-alarm rate alpha."""
    if n_runs < 1:
        raise ValueError("at least one clear calibration run is required")
    if not 0 < alpha < 1:
        raise ValueError("alpha must be between 0 and 1")
    return int(np.ceil((n_runs + 1) * (1 - alpha)))


def calibrate(model: dict, paths: list[Path], alpha: float, out: Path | None) -> None:
    """Select floors from independent clear-rollout maxima, never correlated frames."""
    arm = [str(m) for m in model["arm"]]
    rows = []
    for path in paths:
        if any(tag in path.stem.lower() for tag in ("obstacle", "collide", "slip")):
            raise SystemExit(f"{path.name} looks faulted; calibration accepts clear runs only")
        df = pd.read_csv(path, keep_default_na=False)
        rows.append(clear_run_residual_peaks(model, df))
    peaks = np.vstack(rows)
    rank = _conformal_rank(len(paths), alpha)
    pcts = list(model["floor_pcts"])
    if 99.0 not in pcts:
        raise SystemExit("calibration requires the model's frame-p99 reference floors")
    base_floors = model["floor_table"][:, pcts.index(99.0)].astype(float)
    # The operational false alarm is ANY joint crossing. Calibrate one maximum score
    # across all joints per rollout, not five marginal per-joint rates.
    rollout_scores = (peaks / base_floors).max(axis=1)

    print(f"\n  independent clear-rollout residual maxima ({len(paths)} calibration units):")
    print(f"  {'run':<24}" + "".join(f" {m:>15}" for m in arm))
    for path, row in zip(paths, peaks, strict=True):
        print(f"  {path.name:<24}" + "".join(f" {value:>15.1f}" for value in row))
    print(f"  {'empirical maximum':<24}" + "".join(f" {value:>15.1f}" for value in peaks.max(axis=0)))
    print("  rollout max × frame-p99: " + ", ".join(f"{value:.2f}" for value in rollout_scores))

    if rank > len(paths):
        minimum = int(np.ceil(1 / alpha) - 1)
        print(
            f"\n  NOT CALIBRATED: alpha={alpha:g} requires conformal rank {rank}, but only "
            f"{len(paths)} independent clear runs exist. At least {minimum} are required "
            "for a finite distribution-free threshold."
        )
        print("  The empirical maxima above explain the current false alerts but are development-only.")
        print("  No calibrated model was written; collect more independent clear rollouts.")
        return

    multiplier = float(np.sort(rollout_scores)[rank - 1])
    floors = base_floors * multiplier
    print(f"\n  calibrated rank: {rank}/{len(paths)} for per-rollout alpha={alpha:g}")
    print(f"  shared detector-level multiplier over frame-p99: {multiplier:.2f}")
    print("  floors: " + ", ".join(f"{m}={v:.1f}" for m, v in zip(arm, floors, strict=True)))
    if out is None:
        print("  no --out supplied; calibration was not saved")
        return
    payload = dict(model)
    payload.update(
        calibrated_floors=floors,
        calibration_alpha=np.array(alpha),
        calibration_sources=np.array([p.name for p in paths]),
        calibration_rank=np.array(rank),
        calibration_multiplier=np.array(multiplier),
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out, **payload)
    print(f"  wrote {out}")


def evaluate(model: dict, paths: list[Path], floor_pct: float, use_calibrated: bool = False) -> None:
    arm = [str(m) for m in model["arm"]]
    if use_calibrated:
        if "calibrated_floors" not in model:
            raise SystemExit("--use-calibrated requested, but this model has no calibrated floors")
        floors = {m: float(model["calibrated_floors"][j]) for j, m in enumerate(arm)}
        floor_description = (
            f"clear-rollout conformal alpha={float(model['calibration_alpha']):g} floor"
        )
    else:
        pcts = list(model["floor_pcts"])
        if floor_pct not in pcts:
            raise SystemExit(f"--floor-pct must be one of {pcts}")
        col = pcts.index(floor_pct)
        floors = {m: float(model["floor_table"][j, col]) for j, m in enumerate(arm)}
        floor_description = f"free-space frame p{floor_pct:g} floor"

    print(f"\n=== residual on {len(paths)} run(s), trailing smoothing {RESIDUAL_SMOOTH} frames ===")
    print(f"  {floor_description} per joint: "
          + ", ".join(f"{m}={floors[m]:.0f}" for m in arm))
    print(
        f"\n  {'run':<24} {'peak':>7} {'joint':<14} {'t':>7} {'x floor':>8} "
        f"{'sust':>6} {'OOD x':>8}  verdict"
    )

    for path in paths:
        df = pd.read_csv(path, keep_default_na=False)
        pred = predict(model, df)
        coverage = coverage_ratio(model, df)
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

        coverage_peak = float(np.nanmax(coverage[WARMUP:])) if coverage is not None else np.nan
        coverage_text = f"{coverage_peak:>8.1f}" if coverage is not None else f"{'n/a':>8}"
        outside_coverage = coverage is not None and bool((coverage[WARMUP:] > 1.0).any())

        if outside_coverage:
            # A residual may still be physically real, but this model has no calibrated
            # basis for calling it contact. Preserve the score and abstain from the label.
            verdict = "ABSTAIN -- command outside training coverage"
        elif flat.max() < 1.0:
            verdict = "no contact (within model error)"
        elif sustained < 3:
            verdict = "SPIKE ONLY -- likely model error"
        else:
            verdict = "CONTACT"
        print(f"  {path.name:<24} {resid[k].max():>7.0f} {joint:<14} {t_peak:>6.2f}s "
              f"{flat.max():>8.1f} {sustained:>6} {coverage_text}  {verdict}")

    print("\n  'x floor' is the peak residual as a multiple of the held-out free-space")
    if use_calibrated:
        print("  calibrated clear-rollout floor for that joint. Below 1.0 did not exceed")
        print("  the selected per-rollout operating point on the calibration distribution.")
    else:
        print(f"  p{floor_pct:g} for that joint. Below 1.0 the excursion is indistinguishable from")
        print("  the model's own frame-level error. A *_clear run scoring above 1.0 means the floor is")
        print("  optimistic and the model needs more or better free-space coverage.")
    print("  'OOD x' is commanded-feature nearest-neighbour distance divided by its")
    print("  held-out free-space p99. Above 1.0 means the contact verdict is not calibrated")
    print("  for that motion, so the detector abstains rather than converting extrapolation")
    print("  into a false contact claim. Legacy models print n/a.")


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
    p_eval.add_argument(
        "--use-calibrated",
        action="store_true",
        help="use rollout-level floors written by the calibrate command",
    )

    p_calibrate = sub.add_parser(
        "calibrate", help="select per-rollout floors from independent clear recordings"
    )
    p_calibrate.add_argument("csv", nargs="+", type=Path)
    p_calibrate.add_argument("--model", type=Path, required=True)
    p_calibrate.add_argument("--alpha", type=float, default=0.05)
    p_calibrate.add_argument("--out", type=Path)

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
    elif args.cmd == "eval":
        model = dict(np.load(args.model, allow_pickle=True))
        print(f"model: {args.model}  (trained on {', '.join(str(s) for s in model['sources'])})")
        evaluate(model, args.csv, args.floor_pct, args.use_calibrated)
    else:
        model = dict(np.load(args.model, allow_pickle=True))
        print(f"model: {args.model}  (trained on {', '.join(str(s) for s in model['sources'])})")
        calibrate(model, args.csv, args.alpha, args.out)


if __name__ == "__main__":
    main()
