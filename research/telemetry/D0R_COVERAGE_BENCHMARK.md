# D0r command-coverage cost at projected corpus scale

**Measured:** 28 July 2026 · RTX 4090 workstation CPU · 68-dimensional exact Euclidean NN

The stored training reference grows linearly with free-space corpus frames. The original
NumPy implementation allocated a `(query chunk × full reference × 68)` temporary, which
would require tens of gigabytes at the projected 270k-frame scale. This was a memory bug,
not a reason to change the frozen coverage statistic.

`coverage_index.py` computes the identical nearest-neighbour distance using a reusable
`scipy.spatial.cKDTree` when SciPy is installed and a bounded-memory exact NumPy fallback
otherwise. No approximation, reference subsampling, feature change, or threshold change is
introduced. Scores, scorable masks, and triggers were identical on pair1/pair4 clear and
obstacle development runs (absolute tolerance `1e-12`).

## Result

Command:

```bash
python research/telemetry/bench_coverage.py
```

| reference frames | tree build | single frame | 100-frame batch, per frame | reference RAM |
|---:|---:|---:|---:|---:|
| 2,312 | 1.0 ms | 0.017 ms | 0.002 ms | 1.2 MiB |
| 25,000 | 13.6 ms | 0.017 ms | 0.003 ms | 13.0 MiB |
| 100,000 | 52.8 ms | 0.019 ms | 0.008 ms | 51.9 MiB |
| 270,000 | 177.3 ms | 0.029 ms | 0.031 ms | 140.1 MiB |

The projected 270k rows approximate 300 training rollouts × 30 seconds × 30 Hz. Rows are
repeated from the development reference, so this is a computational scaling test only; it
says nothing about future command-space coverage quality.

Without SciPy, the exact bounded fallback is safe but not free:

```bash
python research/telemetry/bench_coverage.py \
    --reference-sizes 270000 --repeats 5 --force-numpy
```

It measured 32.4 ms for an isolated frame and 1.17 ms/frame in a 100-frame batch. Corpus
deployment should therefore install the existing `scipy-dep` extra. Tree construction is a
one-time model-load cost; do not rebuild it per frame.

## Decision

With SciPy, coverage remains negligible relative to the 3.8 ms mean SmolVLA control step
and uses about 140 MiB at the projected scale. Re-measure with the actual corpus model
before the final cost table because real reference size and geometry may differ.

