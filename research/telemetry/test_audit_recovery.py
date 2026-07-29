import csv
from pathlib import Path

import recovery
from audit_recovery import audit_recovery_log

MOTORS = list(recovery.SO101_MOTORS)


def _pose(value):
    return dict.fromkeys(MOTORS, value)


def _config(*, validated=True):
    return recovery.RecoveryConfig(
        schema_version=1,
        limits=recovery.RecoveryLimits(
            joint_min=_pose(0),
            joint_max=_pose(4095),
            max_current=_pose(100),
            max_following_error=_pose(50),
            max_step_ticks=20,
            phase_timeout_s=1.0,
            reverse_frames=10,
            fault_guard_frames=1,
            gripper_open=500,
        ),
        routes=(recovery.RecoveryRoute("center", _pose(0), _pose(4095), (_pose(1000),), validated),),
    )


def _write_log(path: Path, *, current=10, step=10):
    fields = ["t", "phase", "frame_idx"]
    fields += [f"goal_pos.{motor}" for motor in MOTORS]
    fields += [f"pos.{motor}" for motor in MOTORS]
    fields += [f"curr.{motor}" for motor in MOTORS]
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        for index, phase in enumerate(("reverse", "waypoint", "home", "open")):
            goal = 1000 + index * step
            gripper = 500 if phase == "open" else 470 + index * 10
            row = {"t": index * 0.1, "phase": phase, "frame_idx": index}
            row.update({f"goal_pos.{motor}": gripper if motor == "gripper" else goal for motor in MOTORS})
            row.update({f"pos.{motor}": gripper if motor == "gripper" else goal for motor in MOTORS})
            row.update({f"curr.{motor}": current for motor in MOTORS})
            writer.writerow(row)


def test_valid_completed_log_passes(tmp_path):
    path = tmp_path / "recovery.csv"
    _write_log(path)
    summary, errors = audit_recovery_log(path, _config(), "center")
    assert not errors
    assert summary.frames == 4
    assert summary.peak_current["shoulder_lift"] == 10


def test_limit_violations_fail(tmp_path):
    path = tmp_path / "recovery.csv"
    _write_log(path, current=101, step=30)
    _, errors = audit_recovery_log(path, _config(), "center")
    assert any("current exceeds" in error for error in errors)
    assert any("command step exceeds" in error for error in errors)


def test_unvalidated_route_cannot_pass(tmp_path):
    path = tmp_path / "recovery.csv"
    _write_log(path)
    _, errors = audit_recovery_log(path, _config(validated=False), "center")
    assert any("validated=false" in error for error in errors)
