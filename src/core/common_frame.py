"""Register aligned 2D local NED frames using a shared position reference.

Callers must establish alignment, synchronization, reference quality and frame
reset handling before using this purely geometric calculation.
"""

from collections.abc import Mapping
from types import MappingProxyType

import numpy as np
from numpy.typing import ArrayLike, NDArray


def _point(value: ArrayLike) -> NDArray[np.float64]:
    point = np.asarray(value, dtype=float)
    if point.shape != (2,) or not np.all(np.isfinite(point)):
        raise ValueError("position must contain two finite coordinates")
    return point


class AlignedFrameRegistration:
    """Immutable origin offsets mapping local coordinates to a common frame."""

    def __init__(self, offsets: Mapping[str, tuple[float, float]]) -> None:
        self._offsets = MappingProxyType(dict(offsets))

    def to_common(self, vehicle_id: str, position: ArrayLike) -> NDArray[np.float64]:
        """Translate a local NED horizontal position into the reference frame."""
        return _point(position) + self._offsets[vehicle_id]

    def to_local(self, vehicle_id: str, position: ArrayLike) -> NDArray[np.float64]:
        """Translate a common-frame horizontal position into a local frame."""
        return _point(position) - self._offsets[vehicle_id]


def register_aligned_frames(
    local_positions: Mapping[str, ArrayLike],
    shared_positions: Mapping[str, ArrayLike],
    reference_id: str,
) -> AlignedFrameRegistration:
    """Compute offsets from simultaneous positions in aligned NED frames.

    Shared positions must use the same north/east axes as local positions;
    they can have an arbitrary common origin. This function does not check
    timestamps, localization uncertainty, axis alignment or estimator resets.
    """
    ids = set(local_positions)
    if not ids or ids != set(shared_positions) or reference_id not in ids:
        raise ValueError("snapshots must contain the same IDs and the reference")
    if any(not isinstance(key, str) or not key.strip() for key in ids):
        raise ValueError("vehicle IDs must be nonempty strings")
    local = {key: _point(value) for key, value in local_positions.items()}
    shared = {key: _point(value) for key, value in shared_positions.items()}
    offsets = {}
    for key in ids:
        delta = (shared[key] - shared[reference_id]) - (
            local[key] - local[reference_id]
        )
        if not np.all(np.isfinite(delta)):
            raise ValueError("offset must be finite")
        offsets[key] = (float(delta[0]), float(delta[1]))
    return AlignedFrameRegistration(offsets)
