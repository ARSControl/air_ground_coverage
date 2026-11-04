"""Mantovani et al. (2024), Algorithm 1 / Eqs. 18–19, for central MFGP data.

The native_epistemic criterion tests latent standard deviation, excluding
observation noise as in the paper. The high_information extension tests the
expected HIGH variance reduction, using cross-fidelity covariance and noise.
Existing samples are not evicted; the shared hard cap is also an extension.
"""

from dataclasses import dataclass
import math

import numpy as np

from .observations import Fidelity, Observation


@dataclass(frozen=True)
class EpistemicFilterSettings:
    enabled: bool = False
    relative_error: float = 0.05
    confidence_z: float = 1.96
    criterion: str = "native_epistemic"
    on_full: str = "reject"
    replacement_min_improvement: float = 1e-8

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise TypeError("enabled must be boolean")
        if self.criterion not in {"native_epistemic", "high_information"}:
            raise ValueError("invalid sample-filter criterion")
        if self.on_full not in {"reject", "information"}:
            raise ValueError("invalid full-budget policy")
        tolerance = self.replacement_min_improvement
        if isinstance(tolerance, bool) or not math.isfinite(tolerance) or tolerance < 0:
            raise ValueError(
                "replacement_min_improvement must be finite and nonnegative"
            )
        for name in ("relative_error", "confidence_z"):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")


@dataclass(frozen=True)
class FilterSelection:
    accepted: tuple[bool, ...]
    low_threshold: float
    high_threshold: float
    redundant: int
    duplicates: int
    capacity_rejected: int
    capacity_candidates: tuple[int, ...] = ()
    replaced: int = 0
    evicted_low: int = 0
    evicted_high: int = 0
    replacement_evaluations: int = 0
    replacement_objective_before: float | None = None
    replacement_objective_after: float | None = None


def select_samples(
    observations,
    covariance,
    *,
    low_mean,
    high_mean,
    settings,
    retained,
    max_low,
    max_high,
    jitter=1e-8,
    max_total=None,
    high_cross_covariance=None,
):
    """Condition candidate uncertainty sequentially in stable timestamp order.

    Thresholds are fixed from the last committed native-field means over the
    configured query grid for this scheduled update. Within-batch covariance
    is updated by rank-one conditioning; the GP itself is refitted only once.
    """
    if not isinstance(settings, EpistemicFilterSettings):
        raise TypeError("invalid filter settings")
    observations = tuple(observations)
    if not all(isinstance(o, Observation) for o in observations):
        raise TypeError("invalid observations")
    covariance = np.array(covariance, dtype=float, copy=True)
    n = len(observations)
    if (
        covariance.shape != (n, n)
        or not np.all(np.isfinite(covariance))
        or not np.allclose(covariance, covariance.T, atol=1e-10)
        or np.any(np.diag(covariance) < -1e-10)
    ):
        raise ValueError("invalid latent covariance")
    if not math.isfinite(jitter) or jitter <= 0:
        raise ValueError("jitter must be positive and finite")
    thresholds = {}
    for fidelity, mean in ((Fidelity.LOW, low_mean), (Fidelity.HIGH, high_mean)):
        mean = np.asarray(mean, dtype=float)
        if mean.ndim != 1 or not len(mean) or not np.all(np.isfinite(mean)):
            raise ValueError("invalid native-field mean")
        thresholds[fidelity] = (
            settings.relative_error * float(np.ptp(mean)) / settings.confidence_z
        )
    cross = None
    if settings.criterion == "high_information":
        cross = np.array(high_cross_covariance, dtype=float, copy=True)
        if cross.shape != (n, n) or not np.all(np.isfinite(cross)):
            raise ValueError("invalid HIGH-to-candidate posterior covariance")
        thresholds[Fidelity.LOW] = thresholds[Fidelity.HIGH]
    if max_total is not None and (
        isinstance(max_total, bool) or not isinstance(max_total, int) or max_total < 1
    ):
        raise ValueError("invalid shared capacity")
    known = set(retained)
    counts = {f: sum(o.fidelity is f for o in retained) for f in Fidelity}
    limits = {Fidelity.LOW: max_low, Fidelity.HIGH: max_high}
    if any(
        isinstance(v, bool) or not isinstance(v, int) or v < 1 for v in limits.values()
    ):
        raise ValueError("invalid capacities")
    accepted = [False] * n
    redundant = duplicates = capacity_rejected = 0
    capacity_candidates = []
    for i in sorted(range(n), key=lambda i: (observations[i].timestamp, i)):
        observation = observations[i]
        if observation in known:
            duplicates += 1
            continue
        variance = max(float(covariance[i, i]), 0.0)
        denominator = variance + observation.noise_variance + jitter
        score = (
            math.sqrt(variance)
            if cross is None
            else abs(float(cross[i, i])) / math.sqrt(denominator)
        )
        if (cross is not None and score == 0.0) or score < thresholds[
            observation.fidelity
        ]:
            redundant += 1
            continue
        if (max_total is not None and sum(counts.values()) >= max_total) or counts[
            observation.fidelity
        ] >= limits[observation.fidelity]:
            capacity_rejected += 1
            capacity_candidates.append(i)
            continue
        accepted[i] = True
        counts[observation.fidelity] += 1
        known.add(observation)
        column = covariance[:, i].copy()
        if cross is not None:
            cross -= np.outer(cross[:, i].copy(), column) / denominator
        covariance -= np.outer(column, column) / denominator
        if np.any(np.diag(covariance) < -1e-9):
            raise FloatingPointError("invalid conditional covariance during filtering")
    return FilterSelection(
        tuple(accepted),
        thresholds[Fidelity.LOW],
        thresholds[Fidelity.HIGH],
        redundant,
        duplicates,
        capacity_rejected,
        capacity_candidates=tuple(capacity_candidates),
    )
