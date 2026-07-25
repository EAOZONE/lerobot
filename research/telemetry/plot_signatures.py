#!/usr/bin/env python
"""Week 1, Day 3: plot the signature test. This is the go/no-go artifact.

Renders load, current, d(position)/dt and velocity against time,
with event markers overlaid. The bar is deliberately low: you should SEE a current
spike at the collision marker and a gripper-load drop at the slip marker. If it
takes statistics to find them, the signal is too weak to detect online at useful
lead times, and the proposal's fallback applies.

Usage:
    uv run python research/telemetry/plot_signatures.py research/telemetry/runs/collision_01.csv
    uv run python research/telemetry/plot_signatures.py runs/*.csv --motor gripper
"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def plot_run(csv_path: Path, motors: list[str] | None, out_dir: Path) -> None:
    df = pd.read_csv(csv_path, keep_default_na=False)
    all_motors = sorted({c.split(".", 1)[1] for c in df.columns if c.startswith("load.")})
    motors = motors or all_motors

    panels = [
        ("load", "Present_Load (signed, +/-1000 = +/-100% max torque)"),
        ("curr", "Present_Current (raw units, ~6.5 mA/LSB)"),
        # goal_pos is logged in normalized units (deg / 0-100) and pos in raw ticks, so a
        # true goal-minus-present error needs the calibration file. d(pos)/dt is the
        # calibration-free stand-in: it goes flat when the arm stalls against something.
        ("dpos", "d(Present_Position)/dt (raw ticks per sample)"),
        ("vel", "Present_Velocity (signed)"),
    ]

    fig, axes = plt.subplots(len(panels), 1, figsize=(13, 3 * len(panels)), sharex=True)
    for ax, (field, title) in zip(axes, panels, strict=True):
        for m in motors:
            if field == "dpos":
                ax.plot(df["t"], df[f"pos.{m}"].diff().fillna(0), label=m, linewidth=1.0)
            else:
                ax.plot(df["t"], df[f"{field}.{m}"], label=m, linewidth=1.0)
        ax.set_ylabel(field)
        ax.set_title(title, fontsize=9, loc="left")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, ncol=len(motors), loc="upper right")

    for _, row in df[df["marker"] != ""].iterrows():
        for ax in axes:
            ax.axvline(row["t"], color="crimson", linestyle="--", linewidth=1.2, alpha=0.8)
        axes[0].annotate(
            row["marker"],
            xy=(row["t"], axes[0].get_ylim()[1]),
            xytext=(3, -10),
            textcoords="offset points",
            color="crimson",
            fontsize=8,
            rotation=90,
            va="top",
        )

    axes[-1].set_xlabel("time (s)")
    fig.suptitle(csv_path.name, fontsize=11)
    fig.tight_layout()

    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{csv_path.stem}.png"
    fig.savefig(out_path, dpi=130)
    print(f"wrote {out_path}")

    # Quick numeric summary around each marker: is there anything there at all?
    for _, row in df[df["marker"] != ""].iterrows():
        t = row["t"]
        before = df[(df["t"] > t - 1.0) & (df["t"] < t)]
        after = df[(df["t"] >= t) & (df["t"] < t + 1.0)]
        if before.empty or after.empty:
            continue
        print(f"\n  marker '{row['marker']}' @ {t:.2f}s  (1s before -> 1s after)")
        for m in motors:
            for field in ("load", "curr"):
                b = before[f"{field}.{m}"].abs().mean()
                a = after[f"{field}.{m}"].abs().max()
                ratio = a / b if b else float("inf")
                flag = "  <--" if ratio > 2.0 else ""
                print(f"    {field}.{m:<14} mean_before={b:8.1f}  peak_after={a:8.1f}  x{ratio:5.2f}{flag}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="+", type=Path)
    parser.add_argument("--motor", action="append", dest="motors", help="repeatable; default all")
    parser.add_argument("--out-dir", type=Path, default=Path("research/telemetry/plots"))
    args = parser.parse_args()

    for csv_path in args.csv:
        plot_run(csv_path, args.motors, args.out_dir)


if __name__ == "__main__":
    main()
