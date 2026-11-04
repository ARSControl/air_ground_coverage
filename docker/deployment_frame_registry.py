"""Estimate aligned NED origin offsets from synchronized global positions.

Prototype for SITL. Global source accuracy, heading alignment, datum, clock
synchronization, and reset signaling must be validated independently.
"""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from src.core.global_reference import geodetic_to_local_ne


@dataclass(frozen=True)
class FrameSample:
    timestamp: float
    local: tuple[float, float]
    latitude_longitude: tuple[float, float]
    sequence: int


class FrameRegistry:
    """Stationary pair-based startup offsets with consistency invalidation."""

    def __init__(
        self,
        vehicle_ids,
        reference_id,
        *,
        min_pairs=5,
        max_age=2.0,
        max_skew=0.25,
        max_spread=0.3,
        max_residual=1.0,
    ):
        self.ids = tuple(vehicle_ids)
        if (
            not self.ids
            or len(set(self.ids)) != len(self.ids)
            or reference_id not in self.ids
            or min_pairs < 2
            or any(
                not math.isfinite(v) or v <= 0
                for v in (max_age, max_skew, max_spread, max_residual)
            )
        ):
            raise ValueError("invalid registry configuration")
        self.reference_id = reference_id
        self.min_pairs, self.max_age, self.max_skew = min_pairs, max_age, max_skew
        self.max_spread, self.max_residual = max_spread, max_residual
        self._latest = {}
        self._pairs = {
            v: deque(maxlen=min_pairs) for v in self.ids if v != reference_id
        }
        self._last_pair = {}
        self._offsets = {}
        self._anchor = None
        self.generation = 0
        self.reference_fault = False

    @staticmethod
    def _ne(sample, reference):
        # Project both lat/lon to the WGS84 ellipsoid surface. Heartbeat alt is
        # MSL; this explicit 2D approximation avoids treating it as ellipsoid height.
        return geodetic_to_local_ne(
            (*sample.latitude_longitude, 0.0), (*reference.latitude_longitude, 0.0)
        )

    def _clear(self, vehicle):
        if (
            vehicle in self._offsets
            or self._pairs[vehicle]
            or vehicle in self._last_pair
        ):
            self.generation += 1
        self._offsets.pop(vehicle, None)
        self._pairs[vehicle].clear()
        self._last_pair.pop(vehicle, None)

    def _clear_all(self):
        self.generation += 1
        for vehicle in self._pairs:
            self._clear(vehicle)
        self._anchor = None

    def invalidate(self, vehicle):
        """Discard registration when a vehicle's localization becomes invalid."""
        if vehicle not in self.ids:
            raise ValueError("unknown vehicle")
        self._latest.pop(vehicle, None)
        if vehicle == self.reference_id:
            if self._anchor is not None:
                self.reference_fault = True
            self._clear_all()
        else:
            self._clear(vehicle)

    def observe(self, vehicle, sample, now):
        if vehicle not in self.ids or not isinstance(sample, FrameSample):
            raise ValueError("unknown vehicle or invalid sample")
        local = np.asarray(sample.local, dtype=float)
        geo = np.asarray(sample.latitude_longitude, dtype=float)
        if (
            local.shape != (2,)
            or geo.shape != (2,)
            or not np.all(np.isfinite(local))
            or not np.all(np.isfinite(geo))
            or abs(geo[0]) > 90
            or abs(geo[1]) > 180
            or not math.isfinite(sample.timestamp)
            or not math.isfinite(now)
            or not 0 <= now - sample.timestamp <= self.max_age
            or isinstance(sample.sequence, bool)
            or not isinstance(sample.sequence, int)
            or sample.sequence < 0
        ):
            raise ValueError("invalid or stale frame sample")
        previous = self._latest.get(vehicle)
        if previous is not None and sample.sequence <= previous.sequence:
            if sample.sequence == previous.sequence:
                return
            if vehicle == self.reference_id:
                self.reference_fault = True
                self._clear_all()
            else:
                self._clear(vehicle)
        self._latest[vehicle] = sample
        if self.reference_fault:
            return
        reference = self._latest.get(self.reference_id)
        if reference is None:
            return
        if vehicle == self.reference_id:
            if self._anchor is None:
                self._anchor = reference
            elif (
                np.linalg.norm(
                    self._ne(reference, self._anchor)
                    - (np.asarray(reference.local) - np.asarray(self._anchor.local))
                )
                > self.max_residual
            ):
                self.reference_fault = True
                self._clear_all()
                return
        for other in self._pairs:
            peer = self._latest.get(other)
            if (
                peer is None
                or abs(peer.timestamp - reference.timestamp) > self.max_skew
            ):
                continue
            estimate = self._ne(peer, reference) - (
                np.asarray(peer.local) - np.asarray(reference.local)
            )
            if other in self._offsets:
                if np.linalg.norm(estimate - self._offsets[other]) > self.max_residual:
                    self._clear(other)
                continue
            pair_id = (reference.sequence, peer.sequence)
            previous_pair = self._last_pair.get(other)
            if previous_pair is not None and (
                pair_id[0] == previous_pair[0] or pair_id[1] == previous_pair[1]
            ):
                continue
            self._last_pair[other] = pair_id
            self._pairs[other].append((reference, peer, estimate))
            if len(self._pairs[other]) < self.min_pairs:
                continue
            window = list(self._pairs[other])
            estimates = np.array([entry[2] for entry in window])
            mean = np.mean(estimates, axis=0)
            refs = np.array([entry[0].local for entry in window])
            peers = np.array([entry[1].local for entry in window])
            if all(
                np.max(np.linalg.norm(array - np.mean(array, axis=0), axis=1))
                <= self.max_spread
                for array in (estimates, refs, peers)
            ):
                self._offsets[other] = mean

    def offsets(self, now):
        """Return only offsets whose current paired samples remain fresh."""
        reference = self._latest.get(self.reference_id)
        if (
            self.reference_fault
            or reference is None
            or not math.isfinite(now)
            or not 0 <= now - reference.timestamp <= self.max_age
        ):
            return {}
        result = {self.reference_id: (0.0, 0.0)}
        for vehicle, offset in self._offsets.items():
            peer = self._latest.get(vehicle)
            if (
                peer is not None
                and 0 <= now - peer.timestamp <= self.max_age
                and abs(peer.timestamp - reference.timestamp) <= self.max_skew
            ):
                result[vehicle] = tuple(float(v) for v in offset)
        return result
