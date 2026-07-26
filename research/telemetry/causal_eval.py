#!/usr/bin/env python
"""Re-run the Week 1 analysis causally: at time t, using only data up to t.

`gate_analysis.py` scans whole files with full future context. A deployed detector
does not have that, so every separation figure in WEEK1_REPORT.md is an UPPER BOUND.
This script measures how much of it survives, and sweeps the smoothing window, which
adjacent work (arXiv 2509.26308) found dominates detector performance.

Two things are made causal here:

  1. Smoothing. `gate_analysis.report_matched_pairs` uses `rolling(center=True)`, which
     averages over frames that have not happened yet. Trailing means only.
  2. Detection time. The offline slip feature returns the largest drop anywhere in the
     file. Here a detection has a moment: the first frame at which the score crosses
     threshold, which is what lead time has to be measured from.

What this does NOT fix: the matched-pair collision test needs a clear-run twin that does
not exist at runtime. Causal smoothing makes its numbers honest, not deployable. That gap
is D0r's job (NEXT_STEPS.md section 4).

Result on the Week 1 data (26 Jul): the slip rule SURVIVES. Causal separation is 316-365
against clean 0-6 at the best window, and the matched-pair collision ratios are unchanged
under trailing smoothing. Two findings fell out of the sweep, both in NEXT_STEPS.md s3:
the window optimum is 10 frames and the rule stops arming past ~15 (grasp duration is the
upper bound, not noise), and smoothing costs lead time there is little of.

Usage:
    python research/telemetry/causal_eval.py research/telemetry/runs --window 10
    python research/telemetry/causal_eval.py research/telemetry/runs \
        --window BEST --compare-offline
    python research/telemetry/causal_eval.py research/telemetry/runs \
        --sweep 3,5,10,15,30 --report research/telemetry/runs/causal_sweep.csv
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from gate_analysis import MOTORS, SMOOTH_FRAMES, class_of, matched_pairs
from gate_analysis import _slip_onset as offline_slip_onset

# Same rule constants as the offline detector, so the only variable is causality.
HOLD_CURRENT = 150
DROP_WINDOW_S = 0.5
MAX_GOAL_MOVE = 2.0

# Weighted towards the short end deliberately: the measured optimum is 10 frames and the
# rule stops arming past ~15, so a grid that starts at 5 and jumps to 50 steps straight
# over the interesting region. The long entries are kept to show the collapse.
DEFAULT_SWEEP = [3, 5, 10, 15, 30, 50, 150, 500]


def causal_smooth(x: np.ndarray, window: int) -> np.ndarray:
    """Trailing mean. Never `center=True` -- that reads the future."""
    return pd.Series(x).rolling(window, min_periods=1).mean().to_numpy()


def frame_span(t: np.ndarray, seconds: float) -> int:
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 1 / 30
    return max(1, int(round(seconds / max(dt, 1e-6))))


def causal_slip_score(
    df: pd.DataFrame, window: int
) -> tuple[np.ndarray, np.ndarray, bool]:
    """Per-frame slip score using only trailing data. Returns (score, t, truncated).

    The rule is unchanged from `_slip_onset`: a fall in gripper current, from a real
    grasp, that the commanded gripper position does not account for. What changes is
    that at frame j only anchors i < j are visible, so the score at j reflects the drop
    observed SO FAR rather than the drop that will eventually complete.

    `truncated` flags a smoothing window longer than the recording, where the trailing
    mean is really a running mean over a partial window and the sweep point is not a
    fair evaluation of that window length.

    THE TRAP, paid for once: the command condition must cover the same interval the
    smoothed current is built from. A trailing mean at frame k summarises raw frames
    [k-window+1, k], so an anchor's smoothed current still carries pre-release values for
    a whole window after the operator opens the jaws. Check the goal only over
    [i+1, j] and a deliberate release passes the filter -- the command has already
    finished moving by the time the smoothed current gets around to falling. That scored
    the clean controls at 258-305 against slips at 260-349: complete overlap, and it
    looked exactly like the rule failing under causal evaluation. It was not. Widening
    the goal check to [i-window+1, j] restores the separation.
    """
    t = df["t"].to_numpy(float)
    curr = causal_smooth(df["curr.gripper"].to_numpy(float), window)
    goal = df["goal_pos.gripper"].to_numpy(float) if "goal_pos.gripper" in df else None
    span = frame_span(t, DROP_WINDOW_S)

    score = np.zeros(len(t))
    for j in range(1, len(t)):
        best = 0.0
        for i in range(max(0, j - span), j):
            if curr[i] < HOLD_CURRENT:
                continue
            # Jaws commanded to move -> a deliberate release, not a slip. Without this
            # the clean runs' release scores 471 and outranks every real slip. The lower
            # bound is the anchor's smoothing window, not the anchor itself -- see above.
            if goal is not None:
                seen = goal[max(0, i - window + 1) : j + 1]
                if float(seen.max() - seen.min()) > MAX_GOAL_MOVE:
                    continue
            drop = float(curr[i] - curr[i + 1 : j + 1].min())
            if drop > best:
                best = drop
        score[j] = best
    return score, t, window > len(t)


def first_crossing(score: np.ndarray, t: np.ndarray, threshold: float) -> float | None:
    hits = np.flatnonzero(score >= threshold)
    return float(t[hits[0]]) if len(hits) else None


def load_runs(runs_dir: Path) -> list[tuple[str, str, pd.DataFrame]]:
    runs = []
    for path in sorted(runs_dir.glob("*.csv")):
        label = class_of(path)
        if label is not None:
            runs.append((path.name, label, pd.read_csv(path, keep_default_na=False)))
    return runs


def evaluate_window(runs, window: int) -> pd.DataFrame:
    rows = []
    for name, label, df in runs:
        score, t, truncated = causal_slip_score(df, window)
        rows.append(
            {
                "file": name,
                "class": label,
                "window": window,
                "frames": len(df),
                "causal_peak": float(score.max()),
                "causal_peak_t": float(t[int(score.argmax())]),
                "offline_peak": offline_slip_onset(df)[0],
                "truncated": truncated,
            }
        )
    return pd.DataFrame(rows)


def separation(res: pd.DataFrame, column: str = "causal_peak") -> dict:
    """Margin between the slip runs and the clean controls. Negative means overlap."""
    clean = res[res["class"] == "clean"][column]
    slip = res[res["class"] == "slip"][column]
    if clean.empty or slip.empty:
        return {"margin": float("nan"), "separated": False, "threshold": float("nan")}
    margin = float(slip.min() - clean.max())
    return {
        "clean_max": float(clean.max()),
        "slip_min": float(slip.min()),
        "margin": margin,
        "separated": margin > 0,
        # Operating point midway through the gap. With n=3 per class this is an
        # illustration, not a calibrated threshold -- do not report it as one.
        "threshold": float(clean.max() + margin / 2) if margin > 0 else float("nan"),
    }


def report_window(runs, window: int, verbose: bool = True) -> pd.DataFrame:
    res = evaluate_window(runs, window)
    sep = separation(res)

    if verbose:
        print(f"\n=== causal slip detector, trailing window = {window} frames "
              f"({window / 30:.2f}s at 30 Hz) ===")
        cols = ["file", "class", "frames", "causal_peak", "causal_peak_t", "offline_peak", "truncated"]
        print(res[cols].to_string(index=False, float_format=lambda v: f"{v:.1f}"))

        if res["truncated"].any():
            n = int(res["truncated"].sum())
            print(f"\n  WARNING: window exceeds the recording length in {n}/{len(res)} runs.")
            print("  Those rows average over a partial window and are not a fair test of")
            print("  this window length. A 500-frame window is 16.7s at 30 Hz; the shortest")
            print("  isolated run here is 192 frames. Week 1 recorded one event per file --")
            print("  the corpus rollouts will be minutes long and will not have this limit.")

        if np.isnan(sep["margin"]):
            print("\n  no slip/clean pair in this directory")
        elif sep["separated"]:
            print(f"\n  SEPARATED: clean max {sep['clean_max']:.0f} < slip min {sep['slip_min']:.0f}"
                  f"  (margin {sep['margin']:.0f})")
            print(f"  illustrative operating point: {sep['threshold']:.0f}")
            for name, label, df in runs:
                if label != "slip":
                    continue
                score, t, _ = causal_slip_score(df, window)
                fired = first_crossing(score, t, sep["threshold"])
                marks = df[df["marker"] != ""]
                lead = ""
                if fired is not None and not marks.empty:
                    lead = f"   marker at {float(marks['t'].iloc[0]):.2f}s, lead {float(marks['t'].iloc[0]) - fired:+.2f}s"
                when = f"{fired:.2f}s" if fired is not None else "never"
                print(f"    {name:<14} fires at {when}{lead}")
        else:
            print(f"\n  OVERLAP: clean max {sep['clean_max']:.0f} >= slip min {sep['slip_min']:.0f}."
                  "\n  The rule does not survive causal evaluation at this window. Reformulate"
                  "\n  before collecting a corpus against it.")
    return res


def report_sweep(runs, windows: list[int], out: Path | None) -> int:
    frames = [evaluate_window(runs, w) for w in windows]
    all_res = pd.concat(frames, ignore_index=True)

    print("\n=== window sweep ===")
    print(f"{'window':>7} {'seconds':>8} {'clean max':>10} {'slip min':>9} "
          f"{'margin':>8}  {'verdict':<11} {'slips scored':>13}  truncated")
    best_w, best_margin = windows[0], -np.inf
    for w, res in zip(windows, frames, strict=True):
        sep = separation(res)
        n_trunc = int(res["truncated"].sum())
        verdict = "SEPARATED" if sep["separated"] else "overlaps"
        flag = f"{n_trunc}/{len(res)} runs" if n_trunc else "-"
        # A slip scoring exactly 0 did not merely score low -- the window dissolved its
        # grasp below HOLD_CURRENT, so the rule never armed. That is a different failure
        # from a small margin and the sweep is unreadable without it.
        slips = res[res["class"] == "slip"]
        fired = f"{int((slips['causal_peak'] > 0).sum())}/{len(slips)}"
        print(f"{w:>7} {w / 30:>8.2f} {sep['clean_max']:>10.0f} {sep['slip_min']:>9.0f} "
              f"{sep['margin']:>8.0f}  {verdict:<11} {fired:>13}  {flag}")
        # Only untruncated windows are eligible: a window longer than the data is not
        # being measured, whatever margin it happens to produce.
        if n_trunc == 0 and sep["margin"] > best_margin:
            best_w, best_margin = w, sep["margin"]

    print(f"\n  best untruncated window: {best_w} frames ({best_w / 30:.2f}s), "
          f"margin {best_margin:.0f}")
    if best_w != SMOOTH_FRAMES:
        print(f"  gate_analysis.py:34 uses SMOOTH_FRAMES = {SMOOTH_FRAMES}, chosen for "
              "legibility in plots, not by sweep.")
    print("\n  Window length is bounded ABOVE by grasp duration, not by noise. A trailing")
    print("  mean longer than the grasp dilutes held current below HOLD_CURRENT and the")
    print("  rule never arms -- which is why long windows produce 0 rather than a weak")
    print("  score. The 500-sample result in arXiv 2509.26308 is for a learned")
    print("  autoencoder over F/T windows; it informs D0+, not this conditioned rule.")

    if out is not None:
        out.parent.mkdir(parents=True, exist_ok=True)
        all_res.to_csv(out, index=False)
        print(f"  wrote {out}")
    return best_w


def report_matched_pairs_causal(runs_dir: Path, window: int) -> None:
    """The matched-pair collision test with trailing instead of centred smoothing.

    Still not deployable -- it diffs against a clear run of the same trajectory, and no
    such twin exists at runtime. The point is to show what the centred window was worth:
    if the ratios move much, the published collision figures were partly reading the
    future.
    """
    pairs = matched_pairs(runs_dir)
    if not pairs:
        return

    arm = [m for m in MOTORS if m != "gripper"]
    print(f"\n=== matched pairs: centred (offline) vs trailing (causal) smoothing, "
          f"window {window} ===")
    print(f"  {'pair':<10} {'centred':>9} {'trailing':>9} {'change':>8}   peak moves to")
    for name, obstacle_path, clear_path in pairs:
        obstacle = pd.read_csv(obstacle_path, keep_default_na=False).set_index("frame_idx")
        clear = pd.read_csv(clear_path, keep_default_na=False).set_index("frame_idx")
        common = obstacle.index.intersection(clear.index)
        if len(common) == 0:
            continue

        d_curr = (
            obstacle.loc[common, [f"curr.{m}" for m in arm]].max(axis=1)
            - clear.loc[common, [f"curr.{m}" for m in arm]].max(axis=1)
        )
        centred = d_curr.rolling(window, center=True, min_periods=1).mean()
        trailing = d_curr.rolling(window, min_periods=1).mean()

        def ratio(s: pd.Series) -> float:
            return float(s.max() / max(s.abs().median(), 1e-9))

        r_c, r_t = ratio(centred), ratio(trailing)
        t_peak = float(obstacle.loc[trailing.idxmax(), "t"])
        shift = t_peak - float(obstacle.loc[centred.idxmax(), "t"])
        print(f"  {name:<10} {r_c:>8.1f}x {r_t:>8.1f}x {r_t - r_c:>+7.1f}x   "
              f"t={t_peak:.2f}s ({shift:+.2f}s)")

    print("\n  A trailing mean necessarily lags -- a positive shift of about half the")
    print("  window is expected and is not a defect. What matters is whether the ratio")
    print("  survives and whether the peak still lands on the contact.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("runs_dir", type=Path)
    parser.add_argument(
        "--window",
        default=str(SMOOTH_FRAMES),
        help="trailing smoothing window in frames, or BEST to use the sweep winner",
    )
    parser.add_argument("--sweep", help="comma-separated window lengths, e.g. 5,15,50,150,500")
    parser.add_argument("--report", type=Path, help="write per-run sweep results to CSV")
    parser.add_argument(
        "--compare-offline",
        action="store_true",
        help="also re-run the matched-pair test with trailing instead of centred smoothing",
    )
    args = parser.parse_args()

    runs = load_runs(args.runs_dir)
    if not runs:
        raise SystemExit(f"No collide_*/slip_*/clean_* CSVs in {args.runs_dir}")
    print(f"{len(runs)} classified runs in {args.runs_dir}")

    windows = None
    if args.sweep:
        windows = [int(w) for w in args.sweep.split(",")]
    elif args.window.upper() == "BEST":
        windows = DEFAULT_SWEEP

    if windows:
        window = report_sweep(runs, windows, args.report)
    else:
        window = int(args.window)

    report_window(runs, window)

    if args.compare_offline:
        report_matched_pairs_causal(args.runs_dir, window)


if __name__ == "__main__":
    main()
