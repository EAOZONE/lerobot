"""Focused tests for the detector contract and its causal invariants."""

import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))

from analyze_goal2 import _symmetric_prediction_error  # noqa: E402
from causal_eval import causal_slip_score  # noqa: E402
from coverage_index import CoverageIndex, _bounded_nearest_distances  # noqa: E402
from detectors import (  # noqa: E402
    D0_THRESHOLD,
    D0Detector,
    D0ResidualDetector,
    OnlineD0,
    average_precision,
    extract_features,
    frame_labels,
    infer_run_label,
    match_event_crossings,
    validate_run,
)
from freespace_model import _conformal_rank, _nearest_distances, _validation_folds  # noqa: E402


def test_online_d0_detects_uncommanded_drop() -> None:
    detector = OnlineD0(window=3, raw_threshold=100)
    scores = [
        detector.update(t=i / 30, gripper_current=current, gripper_goal=10)
        for i, current in enumerate([250, 250, 250, 250, 100, 10, 10])
    ]
    assert scores[-1] > 1.0


def test_online_d0_rejects_commanded_release() -> None:
    detector = OnlineD0(window=3, raw_threshold=100)
    scores = [
        detector.update(t=i / 30, gripper_current=current, gripper_goal=goal)
        for i, (current, goal) in enumerate(
            zip([250, 250, 250, 250, 100, 10, 10], [10, 10, 10, 30, 50, 70, 70], strict=True)
        )
    ]
    assert max(score for score in scores if score is not None) < 1.0


def test_online_d0_warmup_is_unscorable_not_zero() -> None:
    detector = OnlineD0(window=3, raw_threshold=100)
    assert detector.update(t=0.0, gripper_current=250, gripper_goal=10) is None


def test_batch_d0_is_the_online_interface_applied_frame_by_frame() -> None:
    frame = pd.DataFrame(
        {
            "t": np.arange(7) / 30,
            "curr.gripper": [250, 250, 250, 250, 100, 10, 10],
            "goal_pos.gripper": [10] * 7,
        }
    )
    batch = D0Detector(window=3, raw_threshold=100).score(frame).score
    online = OnlineD0(window=3, raw_threshold=100)
    streamed = np.array(
        [
            online.update(t=row[0], gripper_current=row[1], gripper_goal=row[2])
            for row in frame.itertuples(index=False, name=None)
        ],
        dtype=float,
    )
    np.testing.assert_allclose(batch, streamed, equal_nan=True)
    result = D0Detector(window=3, raw_threshold=100).score(frame)
    assert np.isnan(result.score[0])
    assert not result.scorable[0]
    assert result.scorable[1:].all()


def test_online_d0_preserves_the_week1_causal_rule() -> None:
    run = Path(__file__).parent / "runs" / "slip_a.csv"
    frame = pd.read_csv(run, keep_default_na=False)
    online_scores = D0Detector().score(frame).score
    established_scores = causal_slip_score(frame, 10)[0] / D0_THRESHOLD
    established_scores[0] = np.nan
    np.testing.assert_allclose(online_scores, established_scores, equal_nan=True)


def test_d0r_preserves_week1_clear_obstacle_separation() -> None:
    root = Path(__file__).parent
    detector = D0ResidualDetector(root / "models" / "freespace.npz")
    clear = detector.score(pd.read_csv(root / "runs" / "pair1_clear.csv", keep_default_na=False))
    obstacle = detector.score(pd.read_csv(root / "runs" / "pair1_obstacle.csv", keep_default_na=False))
    assert np.nanmax(clear.score) < 1.0
    assert np.nanmax(obstacle.score) > 3.0
    assert not clear.triggered.any()
    assert obstacle.triggered.any()


def test_d0r_abstains_on_commands_outside_stored_coverage() -> None:
    root = Path(__file__).parent
    detector = D0ResidualDetector(root / "models" / "freespace_ab_coverage.npz")
    frame = pd.read_csv(root / "runs" / "pair1_clear.csv", keep_default_na=False)
    result = detector.score(frame)
    assert result.scorable[:25].sum() == 0
    assert (~result.scorable[25:]).any()


def test_labels_do_not_invent_onset_for_unmarked_failure() -> None:
    markers = pd.Series(["", "", ""])
    assert infer_run_label(Path("pair1_obstacle.csv"), markers) == "collision"
    assert np.isnan(frame_labels(markers, "collision")).all()
    np.testing.assert_array_equal(frame_labels(markers, "clean"), [0, 0, 0])


def test_average_precision() -> None:
    assert average_precision(np.array([1, 0, 1]), np.array([0.9, 0.8, 0.7])) == (1 + 2 / 3) / 2


def test_multiple_events_receive_distinct_crossings() -> None:
    matches = match_event_crossings(
        np.array([0.9, 1.1, 4.8, 8.0]),
        np.array([1.0, 5.0]),
        max_early_seconds=1.0,
        max_late_seconds=0.5,
    )
    np.testing.assert_allclose(matches, [0.9, 4.8])


def test_multiple_free_space_runs_are_held_out_as_whole_runs() -> None:
    folds, description = _validation_folds([3, 2])
    assert description.startswith("leave-one-run-out")
    np.testing.assert_array_equal(folds[0], [0, 1, 2])
    np.testing.assert_array_equal(folds[1], [3, 4])


def test_command_coverage_uses_nearest_training_feature_vector() -> None:
    query = np.array([[0.0, 0.0], [4.0, 5.0]])
    reference = np.array([[0.0, 1.0], [4.0, 2.0]])
    np.testing.assert_allclose(_nearest_distances(query, reference), [1.0, 3.0])
    np.testing.assert_allclose(_bounded_nearest_distances(query, reference), [1.0, 3.0])
    np.testing.assert_allclose(CoverageIndex(reference).nearest_distances(query), [1.0, 3.0])
    np.testing.assert_allclose(CoverageIndex(reference, use_scipy=False).nearest_distances(query), [1.0, 3.0])


def test_five_percent_rollout_calibration_needs_nineteen_clear_runs() -> None:
    assert _conformal_rank(18, 0.05) == 19
    assert _conformal_rank(19, 0.05) == 19


def test_goal2_comparison_can_identify_internal_motion_explanation() -> None:
    t = np.arange(240) / 30
    external = np.sin(t)
    internal_a = external + 0.2 * np.sin(3 * t)
    internal_b = external - 0.2 * np.sin(3 * t)

    def make_frame(internal: np.ndarray) -> pd.DataFrame:
        velocity = np.r_[0.0, np.diff(internal) * 30]
        return pd.DataFrame(
            {
                "t": t,
                "goal_pos.wrist_roll": external,
                "goal2.wrist_roll": internal,
                "curr.wrist_roll": 10 * np.abs(velocity),
            }
        )

    first, second = make_frame(internal_a), make_frame(internal_b)
    external_error = _symmetric_prediction_error(first, second, "wrist_roll", "goal_pos")
    internal_error = _symmetric_prediction_error(first, second, "wrist_roll", "goal2")
    assert internal_error[1] < external_error[1]


def test_d0plus_features_are_causal() -> None:
    n = 12
    data: dict[str, object] = {
        "t": np.arange(n) / 30,
        "frame_idx": np.arange(n),
        "marker": [""] * n,
    }
    motors = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
    for motor_idx, motor in enumerate(motors):
        data[f"goal_pos.{motor}"] = np.arange(n) + motor_idx
        for field_idx, field in enumerate(("pos", "vel", "load", "curr", "volt")):
            data[f"{field}.{motor}"] = np.arange(n) + motor_idx + field_idx

    original = validate_run(pd.DataFrame(data), Path("clean_test.csv"))
    changed = original.copy()
    changed.loc[8:, "curr.gripper"] = 10000
    original_features = extract_features(original, Path("clean_test.csv"), window=5)
    changed_features = extract_features(changed, Path("clean_test.csv"), window=5)
    pd.testing.assert_frame_equal(original_features.iloc[:8], changed_features.iloc[:8])
