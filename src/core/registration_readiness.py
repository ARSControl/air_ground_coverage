"""Validate stationary, time-aligned input for local-frame registration.

The caller must independently validate the shared reference's accuracy, axis
alignment, clock domain, localization validity, and each paired sample's own
local/shared synchronization. This module cannot infer those properties.
"""

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from .common_frame import AlignedFrameRegistration, register_aligned_frames


@dataclass(frozen=True)
class PositionSample:
    """A pre-synchronized local/shared position pair in one clock domain."""

    timestamp: float
    local: tuple[float, float]
    shared: tuple[float, float]
    reset_counter: int


@dataclass(frozen=True)
class RegistrationResult:
    registration: AlignedFrameRegistration
    reset_counters: Mapping[str, int]

    def resets_match(self, current: Mapping[str, int]) -> bool:
        """Require a complete snapshot with unchanged horizontal reset counters."""
        return dict(current) == dict(self.reset_counters)


def prepare_registration(
    samples: Mapping[str, Sequence[PositionSample]],
    reference_id: str,
    now: float,
    max_age: float,
    max_skew: float,
    max_displacement: float,
) -> RegistrationResult:
    """Check a stationary synchronized window and register its mean positions.

    Samples are paired across vehicles by index. Every local/shared pair must
    already be synchronized and expressed in aligned north/east axes. Maximum
    displacement bounds each sample's distance from its vehicle's window mean,
    in both local and shared frames. No confidence or absolute accuracy is
    inferred from a small displacement.
    """
    limits = np.asarray([now, max_age, max_skew, max_displacement], dtype=float)
    if not np.all(np.isfinite(limits)) or np.any(limits[1:] < 0):
        raise ValueError("time and thresholds must be finite and nonnegative")
    if not samples or reference_id not in samples:
        raise ValueError("samples must include reference")
    counts = {len(window) for window in samples.values()}
    if len(counts) != 1 or next(iter(counts)) < 2:
        raise ValueError(
            "each vehicle requires the same number of samples (at least two)"
        )
    times = []
    local_means = {}
    shared_means = {}
    counters = {}
    for vehicle, window in samples.items():
        timestamps = np.asarray([s.timestamp for s in window], dtype=float)
        if not np.all(np.isfinite(timestamps)) or np.any(np.diff(timestamps) <= 0):
            raise ValueError("timestamps must be finite and strictly increasing")
        if np.any(timestamps > now) or np.any(now - timestamps > max_age):
            raise ValueError("sample timestamp is stale or future")
        resets = [s.reset_counter for s in window]
        if any(isinstance(c, bool) or not isinstance(c, int) or c < 0 for c in resets):
            raise ValueError("invalid reset counter")
        if len(set(resets)) != 1:
            raise ValueError("local frame reset during window")
        counters[vehicle] = resets[0]
        times.append(timestamps)
        means = []
        for name in ("local", "shared"):
            points = np.asarray([getattr(s, name) for s in window], dtype=float)
            if points.shape != (len(window), 2) or not np.all(np.isfinite(points)):
                raise ValueError("positions must contain two finite coordinates")
            mean = np.mean(points, axis=0)
            if np.max(np.linalg.norm(points - mean, axis=1)) > max_displacement:
                raise ValueError("positions are not stationary")
            means.append(mean)
        local_means[vehicle], shared_means[vehicle] = means
    timestamps = np.stack(times)
    if np.any(np.ptp(timestamps, axis=0) > max_skew):
        raise ValueError("inter-vehicle timestamp skew exceeds limit")
    registration = register_aligned_frames(local_means, shared_means, reference_id)
    return RegistrationResult(registration, MappingProxyType(counters))
