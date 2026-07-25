#!/usr/bin/env python
"""Week 1 go/no-go: does telemetry separate collision and slip from clean motion?

Takes the isolated single-event recordings (one deliberate event per file, arm still
before and after) and asks whether the event is visible without knowing where the
marker is. Class is taken from the filename stem, so the keypress marker is used only
as a sanity check on WHERE the peak landed -- never as the window that defines it.

That distinction matters: in the first continuous run, keypress lag was 3-4s and the
label window swallowed normal motion. Here the whole file is the window.

Usage:
    python research/telemetry/gate_analysis.py research/telemetry/runs
    python research/telemetry/gate_analysis.py research/telemetry/runs --plot
"""

import argparse
import re
from pathlib import Path

import numpy as np
import pandas as pd

MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
# Gripper Max_Torque_Limit is set to 500 by SOFollower.configure(), so |load| clips there.
GRIPPER_LOAD_CEILING = 500

# Arm position divergence (raw ticks, 4096/rev) between an obstacle run and its clear
# twin that counts as "the arm was actually stopped". Observed run-to-run replay noise
# is ~50 ticks, so 150 (~13 deg) is comfortably outside it.
POS_DIVERGENCE_TICKS = 150

# Frames to average over before judging a matched pair (~0.17s at 30 Hz).
SMOOTH_FRAMES = 5
# Smoothed current excess, as a multiple of its own median, that counts as contact.
# Hand-pushing the arm gave 25x; replay variance alone sits near 1x.
CONTACT_RATIO = 8.0


def class_of(path: Path) -> str | None:
    """collide_a.csv -> collision. Filename is ground truth; the keypress is not."""
    stem = path.stem.lower()
    for prefix, label in (("collide", "collision"), ("slip", "slip"), ("clean", "clean")):
        if re.match(rf"^{prefix}[_-]", stem):
            return label
    return None


def features(df: pd.DataFrame) -> dict[str, float]:
    """Whole-file summary. No event window -- the file IS the event.

    Peak load and peak current over a whole file do NOT separate collision from clean
    teleop: normal motion loads joints just as hard. The features that carried signal
    in the first nine-run batch are about the gripper's grip state, so those are what
    this computes. See README for the reasoning.
    """
    f: dict[str, float] = {}

    arm = [m for m in MOTORS if m != "gripper"]
    load_arm = df[[f"load.{m}" for m in arm]].abs().max(axis=1)
    vel_arm = df[[f"vel.{m}" for m in arm]].abs().max(axis=1)
    f["arm_load_peak"] = load_arm.max()
    f["arm_curr_peak"] = df[[f"curr.{m}" for m in arm]].max(axis=1).max()
    # Collision ought to look like load without motion. Recorded so a future matched-
    # trajectory batch can test it; it did not separate on unmatched controls.
    f["arm_stall_frames"] = int(((load_arm > 300) & (vel_arm < 150)).sum())

    # --- grip state ---
    # Gripper |load| clips at GRIPPER_LOAD_CEILING, so load cannot report grip force
    # once the servo is pushing hard. Current is not clipped and is the usable channel:
    # squeezing an object draws sustained current, closing on air draws almost none.
    g_load, g_curr, g_pos = df["load.gripper"].abs(), df["curr.gripper"], df["pos.gripper"]
    holding = g_load > 300
    f["grip_hold_frames"] = int(holding.sum())
    f["grip_curr_held"] = float(g_curr[holding].mean()) if holding.any() else 0.0
    f["grip_curr_peak"] = float(g_curr.max())
    # Where the jaws end up: closing past the object's width means nothing is in them.
    f["grip_final_pos"] = float(g_pos.iloc[-5:].mean())
    f["grip_pos_travel"] = float(g_pos.max() - g_pos.min())

    # --- slip onset ---
    # The signature that matters for lead time: current collapsing WHILE the jaws are
    # still commanded shut. Batch one never captured this because the object was never
    # firmly gripped -- current was 2-3 throughout, so there was nothing to collapse.
    f["slip_onset_drop"], f["slip_onset_t"] = _slip_onset(df)
    return f


def _slip_onset(
    df: pd.DataFrame,
    window_s: float = 0.5,
    hold_current: int = 150,
    max_goal_move: float = 2.0,
) -> tuple[float, float]:
    """Largest fall in gripper current over `window_s`, from a real grasp, uncommanded.

    Three conditions, all necessary:
      - it started from a genuine grasp (current >= hold_current), otherwise there was
        no grip force to lose;
      - the current collapses within the window;
      - the *commanded* gripper position barely moved. Without this last condition a
        deliberate release scores identically to a slip -- which is exactly what the
        first batch's `clean` runs did, at 471.

    Returns (drop_magnitude, time_of_collapse). Zero means no slip onset was captured.

    The timestamp is the moment the current actually falls -- the steepest single-sample
    decrease inside the window -- NOT the start of the search window. Returning the
    window start overstates lead time by up to `window_s`, and lead time is a headline
    metric, so it has to be the real thing.
    """
    curr = df["curr.gripper"].to_numpy()
    t = df["t"].to_numpy()
    goal = df["goal_pos.gripper"].to_numpy() if "goal_pos.gripper" in df else None
    if len(t) < 2:
        return 0.0, 0.0
    span = max(1, int(round(window_s / max(np.median(np.diff(t)), 1e-6))))

    best_drop, best_t = 0.0, 0.0
    for i in range(len(curr) - span):
        if curr[i] < hold_current:
            continue
        tail = slice(i + 1, i + 1 + span)
        if goal is not None and np.abs(goal[tail] - goal[i]).max() > max_goal_move:
            continue  # jaws were commanded to move -- a release, not a slip
        drop = float(curr[i] - curr[tail].min())
        if drop > best_drop:
            best_drop = drop
            # Steepest fall within [i, i+span] -- where the grip actually let go.
            steps = np.diff(curr[i : i + 1 + span])
            best_t = float(t[i + int(np.argmin(steps)) + 1])
    return best_drop, best_t


def matched_pairs(runs_dir: Path) -> list[tuple[str, Path, Path]]:
    """Pair each `*_obstacle.csv` with a clear baseline.

    Prefers its own `pairN_clear.csv`. Falls back to any other `*_clear.csv` in the
    directory, because every run replays the SAME episode -- one clear baseline is
    legitimately reusable across obstacle runs, and re-recording it each time only adds
    replay variance. Silently skipping unpaired obstacle files (the old behaviour) just
    loses data the user thought had been analysed.
    """
    all_clear = sorted(runs_dir.glob("*_clear.csv"))
    pairs = []
    for obstacle in sorted(runs_dir.glob("*_obstacle.csv")):
        name = obstacle.name.replace("_obstacle.csv", "")
        own = obstacle.with_name(f"{name}_clear.csv")
        clear = own if own.exists() else (all_clear[0] if all_clear else None)
        if clear is None:
            print(f"  {name}: no *_clear.csv baseline anywhere in {runs_dir} -- skipped")
            continue
        pairs.append((name, obstacle, clear))
    return pairs


def report_matched_pairs(runs_dir: Path) -> None:
    """Compare obstacle vs clear runs of the SAME replayed trajectory.

    Both runs are driven by the same action column, so they align on `frame_idx`
    exactly -- no time warping needed, and no confound from the operator moving
    differently. Any load/current difference is the collision.
    """
    pairs = matched_pairs(runs_dir)
    if not pairs:
        return

    print("\n=== matched replay pairs (obstacle - clear, aligned on frame_idx) ===")
    for name, obstacle_path, clear_path in pairs:
        obstacle = pd.read_csv(obstacle_path, keep_default_na=False).set_index("frame_idx")
        clear = pd.read_csv(clear_path, keep_default_na=False).set_index("frame_idx")
        common = obstacle.index.intersection(clear.index)
        if len(common) == 0:
            print(f"  {name}: no overlapping frame_idx -- were both runs replays?")
            continue

        # Gripper excluded throughout: its position and load track grasp state, not
        # contact, and would swamp both tests.
        arm = [m for m in MOTORS if m != "gripper"]
        obstacle_load = obstacle.loc[common, [f"load.{m}" for m in arm]].abs().max(axis=1)
        clear_load = clear.loc[common, [f"load.{m}" for m in arm]].abs().max(axis=1)
        obstacle_curr = obstacle.loc[common, [f"curr.{m}" for m in arm]].max(axis=1)
        clear_curr = clear.loc[common, [f"curr.{m}" for m in arm]].max(axis=1)

        d_load = obstacle_load - clear_load
        d_curr = obstacle_curr - clear_curr

        # The decisive test. Both runs are commanded through the SAME trajectory, so if
        # the arm was genuinely blocked it could not reach where the clear run went and
        # its measured position must diverge. Load excess alone is not enough: ordinary
        # replay-to-replay variation produces load spikes of the same size, which is how
        # a run with no contact at all can otherwise score as a "clear signature".
        d_pos = (
            obstacle.loc[common, [f"pos.{m}" for m in arm]].values
            - clear.loc[common, [f"pos.{m}" for m in arm]].values
        )
        d_pos_max = np.abs(d_pos).max(axis=1)

        # Smooth before judging. Contact is SUSTAINED -- the arm stays in the obstacle
        # for tenths of a second -- while replay-to-replay variance is single-frame
        # spikes. A 5-frame (~0.17s) mean keeps the former and flattens the latter.
        #
        # Current, not load, is the channel that survives this. On the first hand-pushed
        # pair, smoothed load peaked at 5.3x baseline across 3 separate episodes with its
        # maximum 2.7s away from the actual contact, i.e. noise; smoothed current peaked
        # at 25x in a single episode exactly on the contact. Same lesson as the gripper:
        # load is coarse and clips, current is not.
        d_curr_s = d_curr.rolling(SMOOTH_FRAMES, center=True, min_periods=1).mean()
        d_load_s = d_load.rolling(SMOOTH_FRAMES, center=True, min_periods=1).mean()

        curr_ratio = d_curr_s.max() / max(d_curr_s.abs().median(), 1e-9)
        hot = (d_curr_s > 0.6 * d_curr_s.max()).to_numpy()
        episodes = int(np.sum(hot[1:] & ~hot[:-1])) + int(hot[0])
        peak_idx = d_curr_s.idxmax()

        shared = "" if clear_path.name.startswith(name) else f" [baseline: {clear_path.name}]"
        print(
            f"  {name}: {len(common)} frames{shared}  "
            f"peak current excess {d_curr_s.max():+5.0f} "
            f"(frame {peak_idx}, t={obstacle.loc[peak_idx, 't']:.2f}s)  "
            f"in {episodes} episode(s)"
        )
        print(
            f"      smoothed: current {curr_ratio:5.1f}x baseline | "
            f"load {d_load_s.max() / max(d_load_s.abs().median(), 1e-9):.1f}x | "
            f"max arm divergence {d_pos_max.max():.0f} ticks ({d_pos_max.max() * 360 / 4096:.1f} deg)"
        )

        if curr_ratio >= CONTACT_RATIO:
            blocked = d_pos_max.max() > POS_DIVERGENCE_TICKS
            how = "arm stalled" if blocked else "resisted but not stopped"
            # More than one episode means the operator made contact more than once --
            # a labeling matter, not a detection failure. Say so rather than hedging.
            count = "" if episodes == 1 else f", {episodes} separate contacts"
            print(f"      -> CONTACT CONFIRMED ({how}{count})")
        else:
            print("      -> NO CONTACT: current excess is within replay-to-replay variance")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("runs_dir", type=Path)
    parser.add_argument("--plot", action="store_true", help="also render per-file plots")
    args = parser.parse_args()

    rows = []
    for path in sorted(args.runs_dir.glob("*.csv")):
        label = class_of(path)
        if label is None:
            continue
        df = pd.read_csv(path, keep_default_na=False)
        row = {"file": path.name, "class": label, **features(df)}
        marks = df[df["marker"] != ""]
        if not marks.empty:
            load_any = df[[f"load.{m}" for m in MOTORS]].abs().max(axis=1)
            row["peak_t"] = float(df.loc[load_any.idxmax(), "t"])
            row["marker_t"] = float(marks["t"].iloc[0])
            row["key_lag"] = row["marker_t"] - row["peak_t"]
        rows.append(row)

    if not rows:
        # A directory holding only matched replay pairs is a legitimate case -- report
        # those and stop, rather than claiming there is nothing here.
        if matched_pairs(args.runs_dir):
            report_matched_pairs(args.runs_dir)
            return
        raise SystemExit(f"No collide_*/slip_*/clean_* or *_obstacle/_clear CSVs in {args.runs_dir}")

    res = pd.DataFrame(rows).sort_values(["class", "file"])

    print("=== per-file ===")
    cols = [
        "file",
        "class",
        "arm_load_peak",
        "arm_curr_peak",
        "arm_stall_frames",
        "grip_hold_frames",
        "grip_curr_held",
        "grip_curr_peak",
        "grip_final_pos",
        "key_lag",
    ]
    print(res[[c for c in cols if c in res]].to_string(index=False, float_format=lambda v: f"{v:.1f}"))

    print("\n=== class means ===")
    summary = [
        "arm_load_peak",
        "arm_curr_peak",
        "arm_stall_frames",
        "grip_hold_frames",
        "grip_curr_held",
        "grip_curr_peak",
        "grip_final_pos",
        "grip_pos_travel",
        "slip_onset_drop",
    ]
    print(res.groupby("class")[summary].mean().round(1).to_string())

    print("\n=== separation: does each class's range overlap 'clean'? ===")
    clean = res[res["class"] == "clean"]
    for feat in summary:
        c_lo, c_hi = clean[feat].min(), clean[feat].max()
        print(f"\n  {feat}   clean range: {c_lo:.1f} .. {c_hi:.1f}")
        for label in ("collision", "slip"):
            sub = res[res["class"] == label]
            if sub.empty:
                continue
            lo, hi = sub[feat].min(), sub[feat].max()
            sep = "SEPARATED" if lo > c_hi or hi < c_lo else "overlaps"
            print(f"    {label:<10} {lo:8.1f} .. {hi:8.1f}   {sep}")

    slips = res[res["class"] == "slip"]
    if not slips.empty and (slips["slip_onset_drop"] == 0).all():
        print(
            "\n  NOTE: slip_onset_drop is zero for every slip run -- none reached a real"
            "\n  grasp (gripper current >=150) and then lost it uncommanded, so no slip"
            "\n  ONSET was captured, only the aftermath. Detection lead time would be <=0."
            "\n  Re-record watching the live GRIP OK readout in log_teleop_telemetry.py."
        )

    report_matched_pairs(args.runs_dir)

    if args.plot:
        import subprocess
        import sys

        script = Path(__file__).parent / "plot_signatures.py"
        files = [str(args.runs_dir / f) for f in res["file"]]
        subprocess.run([sys.executable, str(script), *files], check=True)


if __name__ == "__main__":
    main()
