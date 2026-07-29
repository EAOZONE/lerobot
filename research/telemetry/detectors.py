#!/usr/bin/env python
"""Common causal scoring interface for the VLA failure-detector ladder.

Every detector emits one normalized score per input frame. A score of 1.0 is the
detector's configured operating threshold, which lets later D1/D2/D3 detectors plug
into the same reporting and closed-loop code without detector-specific branches.

Implemented rungs:
  duration  elapsed-time-only baseline
  d0        conditioned gripper-current drop (slip)
  d0r       free-space current residual (collision/contact)

The ``features`` command extracts causal rolling telemetry features for D0+. It does
not fit a classifier yet: fitting before the labeled, leakage-safe corpus splits exist
would bake the Week 1 smoke-test runs into the model-selection procedure.

Usage:
    python research/telemetry/detectors.py score research/telemetry/runs/*.csv \
        --detectors duration,d0,d0r \
        --model research/telemetry/models/freespace.npz \
        --out research/telemetry/runs/scores.parquet

    python research/telemetry/detectors.py report \
        research/telemetry/runs/scores.parquet

    python research/telemetry/detectors.py features research/telemetry/runs/*.csv \
        --window 10 --out research/telemetry/runs/features.parquet
"""

import argparse
import re
from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter

import numpy as np
import pandas as pd
from coverage_index import CoverageIndex
from freespace_model import ARM, RESIDUAL_SMOOTH, WARMUP, _trailing_mean, build_features, predict

MOTORS = [*ARM, "gripper"]
SUPPORTED_DETECTORS = ("duration", "d0", "d0r")

# Week 1's causal D0 scores separated slips at 316--365 from clean controls at
# 0--6. 150 raw current units is deliberately below the observed positive range and
# also has a physical interpretation: a genuine grasp must first draw >=150 units.
D0_THRESHOLD = 150.0
D0_WINDOW = 10
D0_HOLD_CURRENT = 150.0
D0_DROP_WINDOW_S = 0.5
D0_MAX_GOAL_MOVE = 2.0

# Duration is a ranking baseline, not a claim that failures happen at ten seconds.
# This threshold only gives it a closed-loop operating point; AUPRC is threshold-free.
DURATION_THRESHOLD_S = 10.0
D0R_FLOOR_PERCENTILE = 99.0
D0R_SUSTAIN_FRAMES = 3

SCORE_COLUMNS = [
    "run",
    "source",
    "frame_idx",
    "t",
    "marker",
    "run_label",
    "frame_label",
    "detector",
    "score",
    "threshold",
    "triggered",
    "scorable",
    "latency_ms",
]


@dataclass(frozen=True)
class DetectorResult:
    """A detector's causal output, aligned one-for-one with input frames."""

    score: np.ndarray
    scorable: np.ndarray
    triggered: np.ndarray | None = None


class Detector(ABC):
    """Minimal interface all current and future detector rungs implement."""

    name: str
    threshold: float = 1.0

    @abstractmethod
    def score(self, frame: pd.DataFrame) -> DetectorResult:
        """Return causal scores aligned with ``frame``; future samples are forbidden."""


class DurationDetector(Detector):
    name = "duration"

    def __init__(self, threshold_s: float = DURATION_THRESHOLD_S):
        if threshold_s <= 0 or not np.isfinite(threshold_s):
            raise ValueError("duration threshold must be finite and positive")
        self.threshold_s = threshold_s

    def score(self, frame: pd.DataFrame) -> DetectorResult:
        elapsed = frame["t"].to_numpy(float)
        elapsed = elapsed - elapsed[0]
        return DetectorResult(elapsed / self.threshold_s, np.ones(len(frame), dtype=bool))


class D0Detector(Detector):
    name = "d0"

    def __init__(self, window: int = D0_WINDOW, raw_threshold: float = D0_THRESHOLD):
        if window < 1:
            raise ValueError("D0 window must be at least one frame")
        if raw_threshold <= 0 or not np.isfinite(raw_threshold):
            raise ValueError("D0 threshold must be finite and positive")
        self.window = window
        self.raw_threshold = raw_threshold

    def score(self, frame: pd.DataFrame) -> DetectorResult:
        online = OnlineD0(self.window, self.raw_threshold)
        score = np.array(
            [
                online.update(
                    t=float(row.t),
                    gripper_current=float(row.gripper_current),
                    gripper_goal=float(row.gripper_goal),
                )
                for row in frame[["t", "curr.gripper", "goal_pos.gripper"]]
                .rename(
                    columns={
                        "curr.gripper": "gripper_current",
                        "goal_pos.gripper": "gripper_goal",
                    }
                )
                .itertuples(index=False)
            ],
            dtype=float,
        )
        scorable = np.isfinite(score)
        return DetectorResult(score, scorable)


class OnlineD0:
    """Stateful D0 implementation for direct use inside a 30 Hz control loop."""

    def __init__(self, window: int = D0_WINDOW, raw_threshold: float = D0_THRESHOLD):
        if window < 1:
            raise ValueError("D0 window must be at least one frame")
        if raw_threshold <= 0 or not np.isfinite(raw_threshold):
            raise ValueError("D0 threshold must be finite and positive")
        self.window = window
        self.raw_threshold = raw_threshold
        self.reset()

    def reset(self) -> None:
        self.timestamps: list[float] = []
        self.raw_current: list[float] = []
        self.smooth_current: list[float] = []
        self.goals: list[float] = []
        self.frame_deltas: deque[float] = deque(maxlen=60)

    def update(self, *, t: float, gripper_current: float, gripper_goal: float) -> float | None:
        """Return a causal normalized score, or ``None`` until a drop can exist."""
        if self.timestamps and t <= self.timestamps[-1]:
            raise ValueError("OnlineD0 timestamps must increase strictly")
        if self.timestamps:
            self.frame_deltas.append(t - self.timestamps[-1])
        self.timestamps.append(t)
        self.raw_current.append(gripper_current)
        self.goals.append(gripper_goal)
        self.smooth_current.append(float(np.mean(self.raw_current[-self.window :])))

        j = len(self.timestamps) - 1
        if j == 0:
            return None
        dt = float(np.median(self.frame_deltas))
        span = max(1, int(round(D0_DROP_WINDOW_S / max(dt, 1e-6))))
        best = 0.0
        for i in range(max(0, j - span), j):
            if self.smooth_current[i] < D0_HOLD_CURRENT:
                continue
            # The command interval starts one smoothing window before the anchor;
            # otherwise a completed deliberate release looks exactly like a slip.
            seen_goals = self.goals[max(0, i - self.window + 1) : j + 1]
            if max(seen_goals) - min(seen_goals) > D0_MAX_GOAL_MOVE:
                continue
            drop = self.smooth_current[i] - min(self.smooth_current[i + 1 : j + 1])
            best = max(best, drop)
        return best / self.raw_threshold


class D0ResidualDetector(Detector):
    name = "d0r"

    def __init__(
        self,
        model_path: Path,
        floor_percentile: float = D0R_FLOOR_PERCENTILE,
        *,
        use_calibrated: bool = False,
    ):
        self.model_path = model_path
        self.model = dict(np.load(model_path, allow_pickle=True))
        self.arm = [str(m) for m in self.model["arm"]]
        self.coverage_index = (
            CoverageIndex(self.model["coverage_reference"])
            if "coverage_reference" in self.model and "coverage_floor" in self.model
            else None
        )
        if use_calibrated:
            if "calibrated_floors" not in self.model:
                raise ValueError(
                    f"D0r model {model_path} has no rollout-calibrated floors; "
                    "run freespace_model.py calibrate first"
                )
            self.floors = self.model["calibrated_floors"].astype(float)
        else:
            pcts = self.model["floor_pcts"].astype(float)
            matches = np.flatnonzero(np.isclose(pcts, floor_percentile))
            if not len(matches):
                raise ValueError(
                    f"D0r floor percentile {floor_percentile:g} is not stored in {model_path}; "
                    f"choose one of {pcts.tolist()}"
                )
            self.floors = self.model["floor_table"][:, int(matches[0])].astype(float)
        if np.any(self.floors <= 0):
            raise ValueError(f"D0r model {model_path} contains a non-positive residual floor")

    def score(self, frame: pd.DataFrame) -> DetectorResult:
        predicted = predict(self.model, frame)
        measured = np.column_stack([frame[f"curr.{motor}"].to_numpy(float) for motor in self.arm])
        residual = np.clip(measured - predicted, 0, None)
        smoothed = np.column_stack(
            [_trailing_mean(residual[:, j], RESIDUAL_SMOOTH) for j in range(len(self.arm))]
        )
        score = (smoothed / self.floors).max(axis=1)
        scorable = np.arange(len(frame)) >= WARMUP
        coverage = None
        if self.coverage_index is not None:
            features = (build_features(frame) - self.model["mu"]) / self.model["sigma"]
            floor = max(float(self.model["coverage_floor"]), 1e-9)
            coverage = self.coverage_index.nearest_distances(features) / floor
        if coverage is not None:
            # Unsupported commands are an abstention, not evidence of contact. Keeping
            # these frames unscorable also prevents them entering AUPRC/false-alarm metrics.
            scorable &= coverage <= 1.0
        score[~scorable] = np.nan
        above = scorable & (score >= self.threshold)
        sustained = (
            pd.Series(above.astype(int))
            .rolling(D0R_SUSTAIN_FRAMES, min_periods=D0R_SUSTAIN_FRAMES)
            .sum()
            .eq(D0R_SUSTAIN_FRAMES)
            .to_numpy()
        )
        return DetectorResult(score, scorable, sustained)


def require_columns(frame: pd.DataFrame, columns: set[str], path: Path) -> None:
    missing = sorted(columns - set(frame.columns))
    if missing:
        raise ValueError(f"{path} is missing required columns: {', '.join(missing)}")


def validate_run(frame: pd.DataFrame, path: Path) -> pd.DataFrame:
    """Validate shared metadata and provide stable marker/frame columns."""
    require_columns(frame, {"t"}, path)
    if frame.empty:
        raise ValueError(f"{path} contains no frames")
    out = frame.copy()
    out["t"] = pd.to_numeric(out["t"], errors="raise")
    if not np.isfinite(out["t"].to_numpy(float)).all():
        raise ValueError(f"{path} contains non-finite timestamps")
    if len(out) > 1 and np.any(np.diff(out["t"].to_numpy(float)) <= 0):
        raise ValueError(f"{path} timestamps must increase strictly")
    if "frame_idx" not in out:
        out["frame_idx"] = np.arange(len(out))
    if "marker" not in out:
        out["marker"] = ""
    out["marker"] = out["marker"].fillna("").astype(str).str.strip().str.lower()
    return out


def infer_run_label(path: Path, markers: pd.Series) -> str:
    """Infer Week 1 labels without pretending unlabeled files are clean negatives."""
    marker_values = {m for m in markers if m}
    for label in ("collision", "slip"):
        if label in marker_values:
            return label

    stem = path.stem.lower()
    if re.search(r"(^|[_-])(obstacle|collide|collision)([_-]|$)", stem):
        return "collision"
    if re.search(r"(^|[_-])slip([_-]|$)", stem):
        return "slip"
    if re.search(r"(^|[_-])(clean|clear)([_-]|$)", stem):
        return "clean"
    return "unknown"


def frame_labels(markers: pd.Series, run_label: str) -> np.ndarray:
    """Mark frames at/after the first failure onset; unknown onset stays unlabeled.

    Keypress markers are only smoke-test onsets and include reaction delay. The future
    corpus should supply adjudicated onset markers through the same column. A failure run
    with no marker remains NaN rather than labeling every frame positive or inventing an
    onset from the filename.
    """
    values = markers.to_numpy(str)
    onset = np.flatnonzero(np.isin(values, ["collision", "slip", "failure", "onset"]))
    labels = np.full(len(values), np.nan)
    if run_label == "clean":
        labels[:] = 0.0
    elif len(onset):
        labels[:] = 0.0
        labels[onset[0] :] = 1.0
    return labels


def detector_requirements(name: str) -> set[str]:
    if name == "duration":
        return {"t"}
    if name == "d0":
        return {"t", "curr.gripper", "goal_pos.gripper"}
    if name == "d0r":
        return {
            "t",
            *{f"goal_pos.{motor}" for motor in MOTORS},
            *{f"curr.{motor}" for motor in ARM},
        }
    raise ValueError(f"unknown detector: {name}")


def make_detectors(args: argparse.Namespace) -> list[Detector]:
    requested = [name.strip().lower() for name in args.detectors.split(",") if name.strip()]
    unknown = sorted(set(requested) - set(SUPPORTED_DETECTORS))
    if unknown:
        raise ValueError(
            f"unknown detector(s): {', '.join(unknown)}; choose from {', '.join(SUPPORTED_DETECTORS)}"
        )
    if len(requested) != len(set(requested)):
        raise ValueError("--detectors contains duplicates")
    if not requested:
        raise ValueError("--detectors must select at least one detector")

    detectors: list[Detector] = []
    for name in requested:
        if name == "duration":
            detectors.append(DurationDetector(args.duration_threshold))
        elif name == "d0":
            detectors.append(D0Detector(args.d0_window, args.d0_threshold))
        else:
            if args.model is None:
                raise ValueError("--model is required when d0r is selected")
            detectors.append(
                D0ResidualDetector(
                    args.model,
                    args.floor_pct,
                    use_calibrated=args.use_calibrated,
                )
            )
    return detectors


def score_runs(
    paths: list[Path], detectors: list[Detector], out: Path, *, strict: bool = False
) -> pd.DataFrame:
    rows: list[pd.DataFrame] = []
    required = set().union(*(detector_requirements(detector.name) for detector in detectors))
    for path in paths:
        frame = pd.read_csv(path, keep_default_na=False)
        if "phase" in frame.columns:
            message = (
                f"{path} is a recovery log, whose goal_pos columns are raw ticks rather than "
                "normalized policy commands"
            )
            if strict:
                raise ValueError(message)
            print(f"  SKIP {path.name}: recovery-log command units are incompatible")
            continue
        missing = sorted(required - set(frame.columns))
        if missing:
            message = f"{path} is not scoreable; missing: {', '.join(missing)}"
            if strict:
                raise ValueError(message)
            print(f"  SKIP {path.name}: incompatible CSV ({len(missing)} required columns missing)")
            continue
        frame = validate_run(frame, path)
        run_label = infer_run_label(path, frame["marker"])
        labels = frame_labels(frame["marker"], run_label)
        print(f"  {path.name}: {len(frame)} frames, label={run_label}")

        for detector in detectors:
            require_columns(frame, detector_requirements(detector.name), path)
            started = perf_counter()
            result = detector.score(frame)
            elapsed_ms = (perf_counter() - started) * 1e3
            if result.score.shape != (len(frame),) or result.scorable.shape != (len(frame),):
                raise RuntimeError(f"{detector.name} violated the one-score-per-frame contract")
            triggered = (
                result.triggered
                if result.triggered is not None
                else result.scorable & (result.score >= detector.threshold)
            )
            if triggered.shape != (len(frame),):
                raise RuntimeError(f"{detector.name} returned a misaligned trigger mask")

            scored = pd.DataFrame(
                {
                    "run": path.stem,
                    "source": str(path),
                    "frame_idx": frame["frame_idx"].to_numpy(),
                    "t": frame["t"].to_numpy(float),
                    "marker": frame["marker"].to_numpy(str),
                    "run_label": run_label,
                    "frame_label": labels,
                    "detector": detector.name,
                    "score": result.score,
                    "threshold": detector.threshold,
                    "triggered": triggered,
                    "scorable": result.scorable,
                    "latency_ms": elapsed_ms / len(frame),
                }
            )
            rows.append(scored[SCORE_COLUMNS])

    if not rows:
        raise ValueError("none of the input CSV files had the selected detectors' required schema")
    scores = pd.concat(rows, ignore_index=True)
    out.parent.mkdir(parents=True, exist_ok=True)
    scores.to_parquet(out, index=False)
    print(
        f"wrote {len(scores):,} per-frame scores for {scores['run'].nunique()} run(s) "
        f"and {scores['detector'].nunique()} detector(s) -> {out}"
    )
    return scores


def average_precision(y_true: np.ndarray, score: np.ndarray) -> float:
    """Binary average precision without adding a scikit-learn dependency."""
    raw_y = np.asarray(y_true)
    s = np.asarray(score, dtype=float)
    if raw_y.shape != s.shape:
        raise ValueError("average_precision labels and scores must have equal shapes")
    valid = np.isfinite(raw_y.astype(float)) & np.isfinite(s)
    y = raw_y[valid].astype(bool)
    s = s[valid]
    if not y.any():
        return float("nan")
    order = np.argsort(-s, kind="stable")
    y = y[order]
    tp = np.cumsum(y)
    precision = tp / np.arange(1, len(y) + 1)
    return float(precision[y].sum() / y.sum())


def rising_crossings(triggered: np.ndarray) -> int:
    active = np.asarray(triggered, dtype=bool)
    return int((active & ~np.r_[False, active[:-1]]).sum())


def match_event_crossings(
    crossing_times: np.ndarray,
    onset_times: np.ndarray,
    max_early_seconds: float,
    max_late_seconds: float,
) -> np.ndarray:
    """Greedily attribute at most one rising crossing to each ordered event onset."""
    crossings = np.asarray(crossing_times, dtype=float)
    onsets = np.asarray(onset_times, dtype=float)
    matches = np.full(len(onsets), np.nan)
    available = np.ones(len(crossings), dtype=bool)
    for event_idx, onset in enumerate(onsets):
        candidate_idx = np.flatnonzero(
            available & (crossings >= onset - max_early_seconds) & (crossings <= onset + max_late_seconds)
        )
        if not len(candidate_idx):
            continue
        before = candidate_idx[crossings[candidate_idx] <= onset]
        chosen = before[-1] if len(before) else candidate_idx[0]
        matches[event_idx] = crossings[chosen]
        available[chosen] = False
    return matches


def report_scores(
    path: Path,
    recovery_seconds: float,
    max_early_seconds: float,
    max_late_seconds: float,
) -> pd.DataFrame:
    scores = pd.read_parquet(path)
    missing = sorted(set(SCORE_COLUMNS) - set(scores.columns))
    if missing:
        raise ValueError(f"{path} is not a detector score file; missing: {', '.join(missing)}")

    summaries = []
    for detector, detector_rows in scores.groupby("detector", sort=False):
        per_run = []
        for run, run_rows in detector_rows.groupby("run", sort=False):
            run_rows = run_rows.sort_values("frame_idx")
            valid = run_rows[run_rows["scorable"] & run_rows["score"].notna()]
            peak = float(valid["score"].max()) if len(valid) else float("nan")
            run_label = str(run_rows["run_label"].iloc[0])
            trigger_rows = valid[valid["triggered"]]
            first_trigger = float(trigger_rows["t"].iloc[0]) if len(trigger_rows) else float("nan")
            active = valid["triggered"].to_numpy(bool)
            crossing_mask = active & ~np.r_[False, active[:-1]]
            crossing_times = valid.loc[crossing_mask, "t"].to_numpy(float)
            marked = run_rows[run_rows["marker"].isin(["collision", "slip", "failure", "onset"])]
            onset_times = marked["t"].to_numpy(float)
            matched_triggers = match_event_crossings(
                crossing_times,
                onset_times,
                max_early_seconds,
                max_late_seconds,
            )
            leads = onset_times - matched_triggers
            detected = np.isfinite(matched_triggers)
            matched_trigger = matched_triggers[0] if len(matched_triggers) else float("nan")
            onset = onset_times[0] if len(onset_times) else float("nan")
            lead = leads[0] if len(leads) else float("nan")
            false_alarms = len(crossing_times) - int(detected.sum())
            per_run.append(
                {
                    "run": run,
                    "label": run_label,
                    "peak": peak,
                    "triggered": bool(len(trigger_rows)),
                    "event_detected": bool(detected.any()),
                    "event_count": len(onset_times),
                    "events_detected": int(detected.sum()),
                    "false_alarms": false_alarms,
                    "first_trigger_t": first_trigger,
                    "matched_trigger_t": matched_trigger,
                    "onset_t": onset,
                    "lead_s": lead,
                    "lead_values": tuple(float(value) for value in leads[np.isfinite(leads)]),
                    "recovery_ready": int((detected & (leads >= recovery_seconds)).sum()),
                }
            )
        runs = pd.DataFrame(per_run)
        labeled = runs[runs["label"].isin(["clean", "collision", "slip"])].copy()
        labeled["failure"] = labeled["label"] != "clean"
        run_ap = average_precision(labeled["failure"].to_numpy(), labeled["peak"].to_numpy())
        labeled_frames = detector_rows[
            detector_rows["scorable"] & detector_rows["score"].notna() & detector_rows["frame_label"].notna()
        ]
        frame_ap = average_precision(
            labeled_frames["frame_label"].to_numpy(), labeled_frames["score"].to_numpy()
        )
        clean = labeled[~labeled["failure"]]
        failures = labeled[labeled["failure"]]
        slips = labeled[labeled["label"] == "slip"]
        collisions = labeled[labeled["label"] == "collision"]
        onset_known = failures[failures["onset_t"].notna()]
        lead_values = [lead for values in onset_known["lead_values"] for lead in values]

        def event_recall(rows: pd.DataFrame) -> float:
            total = int(rows["event_count"].sum())
            return float(rows["events_detected"].sum() / total) if total else float("nan")

        summaries.append(
            {
                "detector": detector,
                "labeled_runs": len(labeled),
                "failure_runs": len(failures),
                "clean_runs": len(clean),
                "run_auprc": run_ap,
                "frame_auprc": frame_ap,
                "false_alarms_per_clean_run": (
                    float(clean["false_alarms"].mean()) if len(clean) else float("nan")
                ),
                "clean_run_trigger_rate": (float(clean["triggered"].mean()) if len(clean) else float("nan")),
                "failure_recall": (event_recall(onset_known) if len(onset_known) else float("nan")),
                "slip_recall": event_recall(slips[slips["onset_t"].notna()]),
                "collision_recall": (event_recall(collisions[collisions["onset_t"].notna()])),
                "median_lead_s": (float(np.median(lead_values)) if lead_values else float("nan")),
                "recovery_ready_fraction": (
                    float(onset_known["recovery_ready"].sum() / onset_known["event_count"].sum())
                    if len(onset_known) and onset_known["event_count"].sum()
                    else float("nan")
                ),
                "mean_latency_ms_per_frame": float(detector_rows["latency_ms"].mean()),
            }
        )

        print(f"\n=== {detector} ===")
        print(
            runs[
                [
                    "run",
                    "label",
                    "peak",
                    "event_count",
                    "events_detected",
                    "first_trigger_t",
                    "matched_trigger_t",
                    "onset_t",
                    "lead_s",
                    "false_alarms",
                ]
            ].to_string(index=False, float_format=lambda value: f"{value:.3f}")
        )

    summary = pd.DataFrame(summaries)
    print("\n=== detector summary (Week 1 values are smoke tests, not estimates) ===")
    print(summary.to_string(index=False, float_format=lambda value: f"{value:.3f}"))
    print(
        f"\nRecovery-ready means the event-matched crossing is >= {recovery_seconds:.2f}s "
        "before the marker. "
        "Week 1 keypresses include reaction lag; use adjudicated onset labels in the corpus."
    )
    print(
        f"Event attribution window: {max_early_seconds:.2f}s before through "
        f"{max_late_seconds:.2f}s after onset. Other crossings count as false alarms."
    )
    return summary


def rolling_stat(series: pd.Series, window: int, stat: str) -> pd.Series:
    rolling = series.rolling(window, min_periods=1)
    if stat == "mean":
        return rolling.mean()
    if stat == "std":
        return rolling.std(ddof=0)
    if stat == "min":
        return rolling.min()
    if stat == "max":
        return rolling.max()
    raise ValueError(stat)


def extract_features(frame: pd.DataFrame, path: Path, window: int) -> pd.DataFrame:
    """Causal D0+ features; every row uses only the current and trailing frames."""
    required = {
        "t",
        *{f"{field}.{motor}" for field in ("pos", "vel", "load", "curr", "volt") for motor in MOTORS},
        *{f"goal_pos.{motor}" for motor in MOTORS},
    }
    require_columns(frame, required, path)
    run_label = infer_run_label(path, frame["marker"])
    columns: dict[str, object] = {
        "run": path.stem,
        "source": str(path),
        "frame_idx": frame["frame_idx"].to_numpy(),
        "t": frame["t"].to_numpy(float),
        "marker": frame["marker"].to_numpy(str),
        "run_label": run_label,
        "frame_label": frame_labels(frame["marker"], run_label),
        "window": window,
        "scorable": np.arange(len(frame)) >= window - 1,
    }

    dt = frame["t"].diff().replace(0, np.nan).fillna(1 / 30.0)
    for motor in MOTORS:
        goal = frame[f"goal_pos.{motor}"].astype(float)
        goal_velocity = goal.diff().fillna(0.0) / dt
        columns[f"goal.{motor}"] = goal.to_numpy()
        columns[f"goal_velocity.{motor}"] = goal_velocity.to_numpy()
        columns[f"goal_motion_range.{motor}"] = (
            goal.rolling(window, min_periods=1).max() - goal.rolling(window, min_periods=1).min()
        ).to_numpy()

        for field in ("pos", "vel", "load", "curr", "volt"):
            values = frame[f"{field}.{motor}"].astype(float)
            columns[f"{field}.{motor}"] = values.to_numpy()
            columns[f"delta_{field}.{motor}"] = values.diff().fillna(0.0).to_numpy()
            for stat in ("mean", "std", "min", "max"):
                columns[f"{field}_{stat}.{motor}"] = rolling_stat(values, window, stat).to_numpy()
            if field in ("vel", "load"):
                columns[f"abs_{field}_mean.{motor}"] = (
                    values.abs().rolling(window, min_periods=1).mean().to_numpy()
                )

        # Following error cannot be subtracted directly here: goal is normalized
        # degrees/range while pos is raw ticks in these logger CSVs. Persistent low
        # measured velocity and high current is the calibration-independent proxy.
        columns[f"stall_proxy.{motor}"] = columns[f"curr_mean.{motor}"] / (
            columns[f"abs_vel_mean.{motor}"] + 1.0
        )

    return pd.DataFrame(columns)


def feature_runs(paths: list[Path], window: int, out: Path, *, strict: bool = False) -> pd.DataFrame:
    if window < 1:
        raise ValueError("--window must be at least one frame")
    frames = []
    for path in paths:
        frame = pd.read_csv(path, keep_default_na=False)
        if "phase" in frame.columns:
            message = (
                f"{path} is a recovery log, whose goal_pos columns are raw ticks rather than "
                "normalized policy commands"
            )
            if strict:
                raise ValueError(message)
            print(f"  SKIP {path.name}: recovery-log command units are incompatible")
            continue
        required = {
            "t",
            *{f"{field}.{motor}" for field in ("pos", "vel", "load", "curr", "volt") for motor in MOTORS},
            *{f"goal_pos.{motor}" for motor in MOTORS},
        }
        missing = sorted(required - set(frame.columns))
        if missing:
            message = f"{path} cannot produce D0+ features; missing: {', '.join(missing)}"
            if strict:
                raise ValueError(message)
            print(f"  SKIP {path.name}: incompatible CSV ({len(missing)} required columns missing)")
            continue
        frame = validate_run(frame, path)
        features = extract_features(frame, path, window)
        frames.append(features)
        print(f"  {path.name}: {len(features)} rows, {len(features.columns)} columns")
    if not frames:
        raise ValueError("none of the input CSV files had the D0+ telemetry schema")
    result = pd.concat(frames, ignore_index=True)
    out.parent.mkdir(parents=True, exist_ok=True)
    result.to_parquet(out, index=False)
    print(f"wrote {len(result):,} causal D0+ feature rows, {len(result.columns)} columns -> {out}")
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    score_parser = subparsers.add_parser("score", help="emit normalized per-frame detector scores")
    score_parser.add_argument("csv", nargs="+", type=Path)
    score_parser.add_argument("--detectors", default="duration,d0", help="comma-separated detector names")
    score_parser.add_argument("--model", type=Path, help="D0r free-space .npz model")
    score_parser.add_argument("--out", required=True, type=Path)
    score_parser.add_argument("--duration-threshold", type=float, default=DURATION_THRESHOLD_S)
    score_parser.add_argument("--d0-window", type=int, default=D0_WINDOW)
    score_parser.add_argument("--d0-threshold", type=float, default=D0_THRESHOLD)
    score_parser.add_argument("--floor-pct", type=float, default=D0R_FLOOR_PERCENTILE)
    score_parser.add_argument(
        "--use-calibrated",
        action="store_true",
        help="use rollout-level D0r floors written by freespace_model.py calibrate",
    )
    score_parser.add_argument(
        "--strict", action="store_true", help="fail instead of skipping incompatible CSV artifacts"
    )

    report_parser = subparsers.add_parser("report", help="summarize common detector metrics")
    report_parser.add_argument("scores", type=Path)
    report_parser.add_argument(
        "--recovery-seconds",
        type=float,
        default=1.0,
        help="lead-time budget required to count a detection as recovery-ready (default: 1)",
    )
    report_parser.add_argument(
        "--max-early-seconds",
        type=float,
        default=2.0,
        help="earliest pre-onset crossing attributable to an event (default: 2)",
    )
    report_parser.add_argument(
        "--max-late-seconds",
        type=float,
        default=1.0,
        help="latest post-onset crossing counted as detection (default: 1)",
    )

    feature_parser = subparsers.add_parser("features", help="extract causal D0+ telemetry features")
    feature_parser.add_argument("csv", nargs="+", type=Path)
    feature_parser.add_argument("--window", type=int, default=D0_WINDOW)
    feature_parser.add_argument("--out", required=True, type=Path)
    feature_parser.add_argument(
        "--strict", action="store_true", help="fail instead of skipping incompatible CSV artifacts"
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    try:
        if args.command == "score":
            score_runs(args.csv, make_detectors(args), args.out, strict=args.strict)
        elif args.command == "report":
            if args.recovery_seconds < 0 or not np.isfinite(args.recovery_seconds):
                raise ValueError("--recovery-seconds must be finite and non-negative")
            if args.max_early_seconds < 0 or not np.isfinite(args.max_early_seconds):
                raise ValueError("--max-early-seconds must be finite and non-negative")
            if args.max_late_seconds < 0 or not np.isfinite(args.max_late_seconds):
                raise ValueError("--max-late-seconds must be finite and non-negative")
            report_scores(
                args.scores,
                args.recovery_seconds,
                args.max_early_seconds,
                args.max_late_seconds,
            )
        else:
            feature_runs(args.csv, args.window, args.out, strict=args.strict)
    except (FileNotFoundError, ValueError, KeyError) as exc:
        raise SystemExit(str(exc)) from exc


if __name__ == "__main__":
    main()
