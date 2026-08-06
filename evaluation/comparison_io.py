"""Archive identity for method-versus-baseline comparison runs."""

import hashlib

import numpy as np

from evaluation.multifidelity_ablation_io import load_archive, save_archive


MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND = "multifidelity_comparison_raw"
EGERSTEDT_COMPARISON_RAW_ARCHIVE_KIND = "egerstedt_comparison_raw"
BASELINE_COMPARISON_EVALUATED_ARCHIVE_KIND = "baseline_comparison_evaluated"
SCENARIO_ARRAYS = (
    "seeds",
    "state_times",
    "query_points",
    "integration_weights",
    "truth_field_grid",
    "truth_density",
    "free_mask",
    "map_grid",
    "initial_aerial_states",
    "initial_ground_states",
    "aerial_sensing_range",
    "ground_sensing_range",
)


def scenario_fingerprint(arrays: dict[str, np.ndarray]) -> str:
    """Hash every input that a paired baseline must reuse exactly."""
    digest = hashlib.sha256()
    for name in SCENARIO_ARRAYS:
        if name not in arrays:
            raise ValueError(f"comparison archive is missing scenario array {name!r}")
        value = np.ascontiguousarray(arrays[name])
        digest.update(name.encode("utf-8"))
        digest.update(value.dtype.str.encode("ascii"))
        digest.update(np.asarray(value.shape, dtype=np.int64).tobytes())
        digest.update(value.tobytes())
    return digest.hexdigest()


__all__ = (
    "BASELINE_COMPARISON_EVALUATED_ARCHIVE_KIND",
    "EGERSTEDT_COMPARISON_RAW_ARCHIVE_KIND",
    "MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND",
    "load_archive",
    "save_archive",
    "scenario_fingerprint",
)
