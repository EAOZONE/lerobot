#!/usr/bin/env python
"""Measure D0r's exact command-coverage cost at projected corpus scale."""

import argparse
import time
from pathlib import Path

import numpy as np
import pandas as pd
from coverage_index import CoverageIndex
from freespace_model import WARMUP, build_features


def measure(index: CoverageIndex, query: np.ndarray, repeats: int) -> tuple[float, float, float]:
    samples = []
    for _ in range(repeats):
        started = time.perf_counter()
        index.nearest_distances(query)
        samples.append((time.perf_counter() - started) * 1000 / len(query))
    return float(np.mean(samples)), float(np.percentile(samples, 95)), float(max(samples))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model", type=Path, default=Path("research/telemetry/models/freespace_ab_coverage.npz")
    )
    parser.add_argument("--query", type=Path, default=Path("research/telemetry/runs/pair4_clear.csv"))
    parser.add_argument("--reference-sizes", type=int, nargs="+", default=[2312, 25000, 100000, 270000])
    parser.add_argument("--batch-frames", type=int, default=100)
    parser.add_argument("--repeats", type=int, default=10)
    parser.add_argument("--force-numpy", action="store_true", help="benchmark the exact fallback")
    args = parser.parse_args()

    model = dict(np.load(args.model, allow_pickle=False))
    frame = pd.read_csv(args.query, keep_default_na=False)
    query = (build_features(frame) - model["mu"]) / model["sigma"]
    query = query[WARMUP : WARMUP + args.batch_frames]
    base = model["coverage_reference"]
    print(f"query={args.query.name}, dimensions={base.shape[1]}, batch={len(query)}")
    print(
        f"{'references':>11} {'backend':>16} {'build ms':>10} {'single ms':>11} {'batch ms/f':>12} {'RAM MiB':>9}"
    )
    for size in args.reference_sizes:
        reference = np.tile(base, (int(np.ceil(size / len(base))), 1))[:size]
        started = time.perf_counter()
        index = CoverageIndex(reference, use_scipy=not args.force_numpy)
        build_ms = (time.perf_counter() - started) * 1000
        single = measure(index, query[:1], args.repeats)[0]
        batch = measure(index, query, args.repeats)[0]
        print(
            f"{size:>11} {index.backend:>16} {build_ms:>10.1f} {single:>11.3f} "
            f"{batch:>12.3f} {reference.nbytes / 2**20:>9.1f}"
        )
    print("\nRepeated reference rows model computational scaling only, not corpus coverage quality.")


if __name__ == "__main__":
    main()
