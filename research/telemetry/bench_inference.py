#!/usr/bin/env python
"""Measure SmolVLA control-step latency -- the denominator for every cost claim.

`vla-failure-detection.md` section 6 reports detector cost as added latency per control
step. That fraction is meaningless without the policy's own per-step cost, and it has
never been measured on this machine. Section 7.1's Week 6 text also warns that a
misconfigured action chunk "will cripple runtime performance", which is worth discovering
now rather than after the corpus is collected.

This measures the real path: the same `predict_action` that `lerobot-record` calls, with
the actual pre/post-processor pipelines, on synthetic observations shaped like the SO-101
rig. No robot, no corpus, no fine-tuned checkpoint required.

What matters is not one average. A chunked policy computes a fresh action chunk once every
`n_action_steps` control steps and pops from a queue in between, so there are two
populations: cheap queue reads and one expensive recompute. Reporting only the mean hides
a recompute spike that can stall the control loop -- which matters directly for D0r and
recovery, where the whole lead-time budget for a slip is 0.3-1.1 s.

Synthetic images are noise. Vision-transformer cost is shape-dependent, not content-
dependent, so this measures the right thing; it says nothing about action quality.

Usage:
    python research/telemetry/bench_inference.py
    python research/telemetry/bench_inference.py --state-dim 30 --cameras 2
    python research/telemetry/bench_inference.py --n-action-steps 1,10,50 --steps 300
"""

import argparse
import time
from typing import Any

import numpy as np
import torch

from lerobot.common.control_utils import predict_action
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.policies import make_pre_post_processors
from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

CHECKPOINT = "lerobot/smolvla_base"
CONTROL_HZ = 30.0
BUDGET_MS = 1000.0 / CONTROL_HZ


def build_config(state_dim: int, cameras: int, height: int, width: int, device: str) -> SmolVLAConfig:
    """Load the released config, then reshape its features to match this rig.

    State dim is safe to change: SmolVLA pads state to `max_state_dim` (32) internally, so
    6-dim (Arms 1/2) and 30-dim (Arms 3/4) share the same weights. Camera count is safe
    because one vision tower is applied per image rather than one tower per camera.
    """
    config = SmolVLAConfig.from_pretrained(CHECKPOINT)
    config.device = device
    config.input_features = {
        "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(state_dim,)),
        **{
            f"observation.images.camera{i + 1}": PolicyFeature(
                type=FeatureType.VISUAL, shape=(3, height, width)
            )
            for i in range(cameras)
        },
    }
    return config


def build_observation(state_dim: int, cameras: int, height: int, width: int) -> dict[str, Any]:
    """One raw observation in the shape `robot.get_observation()` produces: HWC uint8 images."""
    rng = np.random.default_rng(0)
    observation: dict[str, Any] = {"observation.state": rng.standard_normal(state_dim).astype(np.float32)}
    for i in range(cameras):
        observation[f"observation.images.camera{i + 1}"] = rng.integers(
            0, 256, (height, width, 3), dtype=np.uint8
        )
    return observation


def _sync(device: torch.device) -> None:
    if device.type == "cuda":
        torch.cuda.synchronize()


def measure(
    policy: SmolVLAPolicy,
    preprocessor: Any,
    postprocessor: Any,
    observation: dict[str, Any],
    device: torch.device,
    n_action_steps: int,
    steps: int,
    warmup: int,
    task: str,
) -> dict[str, Any]:
    """Time `steps` control steps, separating chunk recomputes from queue reads."""
    policy.config.n_action_steps = n_action_steps
    policy.reset()

    for _ in range(warmup):
        predict_action(
            observation,
            policy,
            device,
            preprocessor,
            postprocessor,
            use_amp=policy.config.use_amp,
            task=task,
        )
    _sync(device)

    timings, recompute_flags = [], []
    for _ in range(steps):
        # An empty action queue means this step pays for a fresh chunk.
        recompute = len(policy._queues["action"]) == 0
        start = time.perf_counter()
        predict_action(
            observation,
            policy,
            device,
            preprocessor,
            postprocessor,
            use_amp=policy.config.use_amp,
            task=task,
        )
        _sync(device)
        timings.append((time.perf_counter() - start) * 1e3)
        recompute_flags.append(recompute)

    times = np.array(timings)
    is_recompute = np.array(recompute_flags)
    recomputes = times[is_recompute]
    queue_reads = times[~is_recompute]

    return {
        "n_action_steps": n_action_steps,
        "mean_ms": float(times.mean()),
        "recompute_ms": float(recomputes.mean()) if len(recomputes) else float("nan"),
        "recompute_max_ms": float(recomputes.max()) if len(recomputes) else float("nan"),
        "queue_ms": float(queue_reads.mean()) if len(queue_reads) else float("nan"),
        "p95_ms": float(np.percentile(times, 95)),
        "max_ms": float(times.max()),
        "n_recomputes": int(is_recompute.sum()),
        "over_budget": int((times > BUDGET_MS).sum()),
        "steps": steps,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state-dim", type=int, default=6, help="6 for Arms 1/2, 30 for Arms 3/4")
    parser.add_argument("--cameras", type=int, default=2, help="wrist + overhead")
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--n-action-steps", default="1,10,25,50")
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--warmup", type=int, default=20)
    parser.add_argument("--task", default="pick up the cube and place it in the bowl")
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    args = parser.parse_args()

    chunk_settings = [int(v) for v in args.n_action_steps.split(",")]
    device = torch.device(args.device)

    if device.type == "cuda":
        print(f"device: {torch.cuda.get_device_name(0)}")
    print(f"checkpoint: {CHECKPOINT}")
    print(f"observation: state {args.state_dim}-dim, {args.cameras} camera(s) at {args.height}x{args.width}")

    load_start = time.perf_counter()
    config = build_config(args.state_dim, args.cameras, args.height, args.width, args.device)
    policy = SmolVLAPolicy.from_pretrained(CHECKPOINT, config=config)
    policy.to(device)
    policy.eval()
    preprocessor, postprocessor = make_pre_post_processors(
        policy.config,
        pretrained_path=CHECKPOINT,
        preprocessor_overrides={
            "device_processor": {"device": args.device},
            "rename_observations_processor": {"rename_map": {}},
        },
        postprocessor_overrides={"device_processor": {"device": args.device}},
    )
    print(f"chunk_size: {config.chunk_size} | load: {time.perf_counter() - load_start:.1f}s")

    observation = build_observation(args.state_dim, args.cameras, args.height, args.width)

    print(f"\nbudget at {CONTROL_HZ:g} Hz: {BUDGET_MS:.1f} ms per control step")
    print(
        f"\n{'n_action_steps':>14} {'mean':>9} {'recompute':>11} {'worst':>9} "
        f"{'queue read':>11} {'p95':>9} {'over budget':>12}"
    )
    print("-" * 80)

    rows = []
    for n_action_steps in chunk_settings:
        if n_action_steps > config.chunk_size:
            print(f"{n_action_steps:>14}  skipped (exceeds chunk_size {config.chunk_size})")
            continue
        row = measure(
            policy,
            preprocessor,
            postprocessor,
            observation,
            device,
            n_action_steps,
            args.steps,
            args.warmup,
            args.task,
        )
        rows.append(row)
        print(
            f"{row['n_action_steps']:>14} {row['mean_ms']:>8.1f}m {row['recompute_ms']:>10.1f}m "
            f"{row['recompute_max_ms']:>8.1f}m {row['queue_ms']:>10.2f}m {row['p95_ms']:>8.1f}m "
            f"{row['over_budget']:>7}/{row['steps']:<4}"
        )

    if not rows:
        return

    print("\n  'recompute' is the mean cost of steps that computed a fresh action chunk;")
    print("  'queue read' is the cost of steps that popped an already-computed action.")
    print("  'over budget' counts steps exceeding the 30 Hz control period.")

    best = min(rows, key=lambda r: r["mean_ms"])
    single = next((r for r in rows if r["n_action_steps"] == 1), None)
    print(
        f"\n  Cheapest mean step: n_action_steps={best['n_action_steps']} at "
        f"{best['mean_ms']:.1f} ms ({1000 / best['mean_ms']:.1f} Hz sustained)."
    )
    if single is not None and single is not best:
        print(
            f"  Single-step prediction costs {single['mean_ms'] / best['mean_ms']:.1f}x more "
            f"per step ({single['mean_ms']:.1f} ms). Section 7.1's warning is confirmed here."
        )
    if not np.isnan(best["recompute_max_ms"]) and best["recompute_max_ms"] > BUDGET_MS:
        stalled = best["recompute_max_ms"] / BUDGET_MS
        print(
            f"  NOTE: a recompute step costs up to {best['recompute_max_ms']:.1f} ms, "
            f"~{stalled:.1f} control periods. The loop stalls when the chunk refills, so"
        )
        print("  detector lead time is not uniformly available. Budget recovery against the")
        print("  worst step, not the mean.")


if __name__ == "__main__":
    main()
