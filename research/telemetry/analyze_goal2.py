#!/usr/bin/env python
"""Test whether Feetech's read-only Goal_Position_2 explains clear-replay current drift.

The external ``goal_pos.*`` trace is the command sent by LeRobot. ``goal2.*`` is raw
register 71 sampled in the same bus transaction as current and is suspected to be the
servo's internal/interpolated setpoint. Two obstacle-free replays of the same command
trace let us ask whether differences in that internal trajectory coincide with the
repeat-dependent current bursts that make D0r false-positive.

This is a paired diagnostic, not a detector and not a calibration procedure.

Usage:
    python research/telemetry/analyze_goal2.py \
        research/telemetry/runs/clear_goal2_a.csv \
        research/telemetry/runs/clear_goal2_b.csv
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
from freespace_model import ARM, RESIDUAL_SMOOTH, _trailing_mean

MOTION_LAGS = (1, 2, 5)
MOTION_WARMUP = max(MOTION_LAGS) + 2


def _require(frame: pd.DataFrame, path: Path) -> None:
    required = {
        "t",
        *{f"goal_pos.{motor}" for motor in ARM},
        *{f"goal2.{motor}" for motor in ARM},
        *{f"curr.{motor}" for motor in ARM},
    }
    missing = sorted(required - set(frame.columns))
    if missing:
        raise ValueError(
            f"{path} lacks Goal_Position_2 diagnostic columns: {', '.join(missing)}. "
            "Record it with the updated log_teleop_telemetry.py."
        )


def _correlation(x: np.ndarray, y: np.ndarray) -> float:
    if np.std(x) < 1e-12 or np.std(y) < 1e-12:
        return float("nan")
    return float(np.corrcoef(x, y)[0, 1])


def _motion_features(t: np.ndarray, position: np.ndarray) -> np.ndarray:
    """Small causal basis shared by external and internal setpoint comparisons."""
    dt = float(np.median(np.diff(t)))
    velocity = np.zeros_like(position, dtype=float)
    velocity[1:] = np.diff(position) / dt
    acceleration = np.zeros_like(position, dtype=float)
    acceleration[1:] = np.diff(velocity) / dt
    columns = [position, velocity, np.abs(velocity), acceleration, np.abs(acceleration)]
    for lag in MOTION_LAGS:
        columns.append(np.r_[np.repeat(velocity[0], lag), velocity[:-lag]])
    return np.column_stack(columns)


def _fit_predict(train_x: np.ndarray, train_y: np.ndarray, test_x: np.ndarray) -> np.ndarray:
    mu = train_x.mean(axis=0)
    sigma = train_x.std(axis=0)
    sigma[sigma < 1e-9] = 1.0
    xs = (train_x - mu) / sigma
    test_xs = (test_x - mu) / sigma
    centered = train_y - train_y.mean()
    alpha = 10.0
    weights = np.linalg.solve(xs.T @ xs + alpha * np.eye(xs.shape[1]), xs.T @ centered)
    return test_xs @ weights + train_y.mean()


def _symmetric_prediction_error(
    first: pd.DataFrame, second: pd.DataFrame, motor: str, source: str
) -> tuple[float, float]:
    """Average A→B/B→A MAE and p99 one-sided underprediction for one motion source."""
    x_first = _motion_features(
        first["t"].to_numpy(float), first[f"{source}.{motor}"].to_numpy(float)
    )[MOTION_WARMUP:]
    x_second = _motion_features(
        second["t"].to_numpy(float), second[f"{source}.{motor}"].to_numpy(float)
    )[MOTION_WARMUP:]
    y_first = first[f"curr.{motor}"].to_numpy(float)[MOTION_WARMUP:]
    y_second = second[f"curr.{motor}"].to_numpy(float)[MOTION_WARMUP:]
    pred_second = _fit_predict(x_first, y_first, x_second)
    pred_first = _fit_predict(x_second, y_second, x_first)
    errors = np.concatenate([y_second - pred_second, y_first - pred_first])
    return float(np.mean(np.abs(errors))), float(np.percentile(np.clip(errors, 0, None), 99))


def analyze(first_path: Path, second_path: Path) -> None:
    first = pd.read_csv(first_path, keep_default_na=False)
    second = pd.read_csv(second_path, keep_default_na=False)
    _require(first, first_path)
    _require(second, second_path)
    if len(first) != len(second):
        raise ValueError(f"paired runs differ in length: {len(first)} versus {len(second)} frames")
    if len(first) < 2:
        raise ValueError("paired runs need at least two frames")

    external_equal = all(
        np.array_equal(
            first[f"goal_pos.{motor}"].to_numpy(float),
            second[f"goal_pos.{motor}"].to_numpy(float),
        )
        for motor in ARM
    )
    if not external_equal:
        raise ValueError("external goal_pos traces differ; this is not a matched-command pair")

    max_dt = float(np.max(np.abs(first["t"].to_numpy(float) - second["t"].to_numpy(float))))
    print(f"matched external commands: yes | frames: {len(first)} | max timestamp delta: {max_dt:.4f}s")
    print(
        f"\n{'joint':<15} {'goal2 range A/B':>21} {'exact %':>8} {'p99 |Δg2|':>11} "
        f"{'p99 |ΔI|':>10} {'corr |Δg2|,|ΔI|':>19}"
    )

    for motor in ARM:
        goal2_a = first[f"goal2.{motor}"].to_numpy(float)
        goal2_b = second[f"goal2.{motor}"].to_numpy(float)
        current_a = first[f"curr.{motor}"].to_numpy(float)
        current_b = second[f"curr.{motor}"].to_numpy(float)
        delta_goal2 = np.abs(goal2_b - goal2_a)
        delta_current = np.abs(current_b - current_a)
        range_text = f"{np.ptp(goal2_a):.0f}/{np.ptp(goal2_b):.0f}"
        print(
            f"{motor:<15} {range_text:>21} "
            f"{100 * np.mean(goal2_a == goal2_b):>7.1f}% "
            f"{np.percentile(delta_goal2, 99):>11.1f} "
            f"{np.percentile(delta_current, 99):>10.1f} "
            f"{_correlation(delta_goal2, delta_current):>19.3f}"
        )

    print(
        f"\n{'joint':<15} {'external MAE/p99+':>20} {'goal2 MAE/p99+':>20} "
        f"{'goal2 p99 change':>18}"
    )
    prediction_rows = {}
    for motor in ARM:
        external = _symmetric_prediction_error(first, second, motor, "goal_pos")
        internal = _symmetric_prediction_error(first, second, motor, "goal2")
        prediction_rows[motor] = (external, internal)
        change = 100 * (internal[1] / max(external[1], 1e-9) - 1)
        print(
            f"{motor:<15} {f'{external[0]:.1f}/{external[1]:.1f}':>20} "
            f"{f'{internal[0]:.1f}/{internal[1]:.1f}':>20} {change:>+17.1f}%"
        )

    motor = "wrist_roll"
    delta_current = second[f"curr.{motor}"].to_numpy(float) - first[f"curr.{motor}"].to_numpy(float)
    paired_burst = _trailing_mean(np.clip(delta_current, 0, None), RESIDUAL_SMOOTH)
    peak = int(np.argmax(paired_burst))
    lo = max(0, peak - RESIDUAL_SMOOTH + 1)
    goal2_delta = (
        second[f"goal2.{motor}"].to_numpy(float) - first[f"goal2.{motor}"].to_numpy(float)
    )
    print(
        f"\nLargest positive paired wrist-current window: {first['t'].iloc[lo]:.3f}–"
        f"{first['t'].iloc[peak]:.3f}s, trailing mean Δcurrent={paired_burst[peak]:.1f}, "
        f"max |Δgoal2|={np.max(np.abs(goal2_delta[lo : peak + 1])):.1f} raw ticks."
    )
    external_error, internal_error = prediction_rows[motor]
    if np.ptp(first[f"goal2.{motor}"].to_numpy(float)) == 0:
        print("RESULT: Goal_Position_2 is flat on wrist_roll; it cannot explain the transient.")
    elif internal_error[1] <= 0.8 * external_error[1]:
        print("RESULT: Goal_Position_2 materially reduces cross-replay wrist-current underprediction.")
        print("Next: test it as a commanded/controller-state feature using run-grouped calibration.")
    else:
        print("RESULT: Goal_Position_2 does not materially reduce wrist-current underprediction.")
        print("Friction/controller-state variability remains; rollout calibration is still required.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("first", type=Path)
    parser.add_argument("second", type=Path)
    args = parser.parse_args()
    analyze(args.first, args.second)


if __name__ == "__main__":
    main()
