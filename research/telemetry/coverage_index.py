"""Memory-safe exact nearest-neighbour index for D0r command coverage."""

import numpy as np

try:
    import scipy.spatial as scipy_spatial
except ImportError:  # Base research stack can still use the exact NumPy fallback.
    scipy_spatial = None

QUERY_CHUNK = 32
REFERENCE_CHUNK = 8192


def _bounded_nearest_distances(query: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """Exact Euclidean nearest distances with bounded temporary memory."""
    if len(reference) == 0:
        raise ValueError("coverage reference must contain at least one frame")
    result = np.full(len(query), np.inf, dtype=float)
    for query_start in range(0, len(query), QUERY_CHUNK):
        block = query[query_start : query_start + QUERY_CHUNK]
        best_squared = np.full(len(block), np.inf, dtype=float)
        query_norm = np.square(block).sum(axis=1)[:, None]
        for reference_start in range(0, len(reference), REFERENCE_CHUNK):
            candidate = reference[reference_start : reference_start + REFERENCE_CHUNK]
            squared = query_norm + np.square(candidate).sum(axis=1)[None, :] - 2 * block @ candidate.T
            best_squared = np.minimum(best_squared, np.maximum(squared, 0).min(axis=1))
        result[query_start : query_start + len(block)] = np.sqrt(best_squared)
    return result


class CoverageIndex:
    """Reusable exact index; SciPy accelerates it, NumPy preserves a safe fallback."""

    def __init__(self, reference: np.ndarray, *, use_scipy: bool = True):
        self.reference = np.asarray(reference, dtype=float)
        if self.reference.ndim != 2 or len(self.reference) == 0:
            raise ValueError("coverage reference must be a non-empty 2D array")
        self.tree = scipy_spatial.cKDTree(self.reference) if use_scipy and scipy_spatial is not None else None

    @property
    def backend(self) -> str:
        return "scipy.cKDTree" if self.tree is not None else "bounded_numpy"

    def nearest_distances(self, query: np.ndarray) -> np.ndarray:
        query = np.asarray(query, dtype=float)
        if query.ndim != 2 or query.shape[1] != self.reference.shape[1]:
            raise ValueError(
                f"coverage query must have shape (n, {self.reference.shape[1]}), got {query.shape}"
            )
        if self.tree is not None:
            distances, _ = self.tree.query(query, k=1, workers=1)
            return np.asarray(distances, dtype=float)
        return _bounded_nearest_distances(query, self.reference)
