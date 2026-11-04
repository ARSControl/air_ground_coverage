"""Timestamped multi-fidelity observations and deterministic bounded storage."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Iterable, Sequence


class Fidelity(Enum):
    """Observation fidelity handled by the central estimator."""

    LOW = "low"
    HIGH = "high"


def _finite_float(value: float, name: str) -> float:
    if isinstance(value, bool):
        raise TypeError(f"{name} must be a real scalar, not a boolean")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must be a real scalar") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True)
class Observation:
    """Immutable scalar measurement with collection and source metadata."""

    timestamp: float
    robot_id: str
    position: tuple[float, float]
    value: float
    fidelity: Fidelity
    noise_variance: float

    def __post_init__(self) -> None:
        timestamp = _finite_float(self.timestamp, "timestamp")
        value = _finite_float(self.value, "value")
        noise_variance = _finite_float(self.noise_variance, "noise_variance")
        if noise_variance < 0.0:
            raise ValueError("noise_variance must be nonnegative")
        if not isinstance(self.robot_id, str) or not self.robot_id.strip():
            raise ValueError("robot_id must be a nonempty string")
        if not isinstance(self.fidelity, Fidelity):
            raise TypeError("fidelity must be a Fidelity value")
        try:
            if len(self.position) != 2:
                raise ValueError
            position = (
                _finite_float(self.position[0], "position[0]"),
                _finite_float(self.position[1], "position[1]"),
            )
        except TypeError as exc:
            raise ValueError("position must be a two-value coordinate") from exc
        except (IndexError, ValueError) as exc:
            raise ValueError("position must be a finite two-value coordinate") from exc

        object.__setattr__(self, "timestamp", timestamp)
        object.__setattr__(self, "position", position)
        object.__setattr__(self, "value", value)
        object.__setattr__(self, "noise_variance", noise_variance)


@dataclass(frozen=True)
class RetentionConfig:
    """Maximum count and minimum same-fidelity spatial separation."""

    max_samples: int
    min_separation: float

    def __post_init__(self) -> None:
        if isinstance(self.max_samples, bool) or not isinstance(self.max_samples, int):
            raise TypeError("max_samples must be an integer")
        if self.max_samples < 1:
            raise ValueError("max_samples must be positive")
        separation = _finite_float(self.min_separation, "min_separation")
        if separation < 0.0:
            raise ValueError("min_separation must be nonnegative")
        object.__setattr__(self, "min_separation", separation)


@dataclass(frozen=True)
class _IndexedObservation:
    insertion_index: int
    observation: Observation


class SpatialAgeRetentionPolicy:
    """Prefer newer observations while enforcing deterministic spacing/count."""

    def select(
        self, entries: Sequence[Observation], config: RetentionConfig
    ) -> tuple[Observation, ...]:
        if not isinstance(config, RetentionConfig):
            raise TypeError("config must be a RetentionConfig")
        indexed: list[_IndexedObservation] = []
        for insertion_index, observation in enumerate(entries):
            if not isinstance(observation, Observation):
                raise TypeError("entries must contain only Observation values")
            indexed.append(_IndexedObservation(insertion_index, observation))
        return tuple(
            entry.observation for entry in self._select_indexed(indexed, config)
        )

    def _select_indexed(
        self,
        entries: Sequence[_IndexedObservation],
        config: RetentionConfig,
    ) -> tuple[_IndexedObservation, ...]:
        priority_order = sorted(
            entries,
            key=lambda entry: (
                entry.observation.timestamp,
                entry.insertion_index,
            ),
            reverse=True,
        )
        selected: list[_IndexedObservation] = []
        minimum_squared = config.min_separation * config.min_separation

        for entry in priority_order:
            x_position, y_position = entry.observation.position
            separated = all(
                (
                    (x_position - retained.observation.position[0]) ** 2
                    + (y_position - retained.observation.position[1]) ** 2
                )
                > 0.0
                and (
                    (x_position - retained.observation.position[0]) ** 2
                    + (y_position - retained.observation.position[1]) ** 2
                )
                >= minimum_squared
                for retained in selected
            )
            if separated:
                selected.append(entry)
                if len(selected) == config.max_samples:
                    break

        return tuple(
            sorted(
                selected,
                key=lambda entry: (
                    entry.observation.timestamp,
                    entry.insertion_index,
                ),
            )
        )


@dataclass(frozen=True)
class _CandidateRecord:
    observations: tuple[Observation, ...]
    entries: tuple[_IndexedObservation, ...]
    pending_watermark: int


class BoundedObservationBuffer:
    """Transactional pending/retained store for exactly one fidelity."""

    def __init__(
        self,
        fidelity: Fidelity,
        config: RetentionConfig,
        policy: SpatialAgeRetentionPolicy | None = None,
    ) -> None:
        if not isinstance(fidelity, Fidelity):
            raise TypeError("fidelity must be a Fidelity value")
        if not isinstance(config, RetentionConfig):
            raise TypeError("config must be a RetentionConfig")
        if policy is not None and not isinstance(policy, SpatialAgeRetentionPolicy):
            raise TypeError("policy must be a SpatialAgeRetentionPolicy")
        self._fidelity = fidelity
        self._config = config
        self._policy = policy or SpatialAgeRetentionPolicy()
        self._retained: tuple[_IndexedObservation, ...] = ()
        self._pending: list[_IndexedObservation] = []
        self._next_insertion_index = 0
        self._last_candidate: _CandidateRecord | None = None

    @property
    def fidelity(self) -> Fidelity:
        return self._fidelity

    @property
    def retained(self) -> tuple[Observation, ...]:
        return tuple(entry.observation for entry in self._retained)

    @property
    def pending_count(self) -> int:
        return len(self._pending)

    def submit(self, observation: Observation) -> None:
        if not isinstance(observation, Observation):
            raise TypeError("observation must be an Observation")
        if observation.fidelity is not self._fidelity:
            raise ValueError(
                f"cannot submit {observation.fidelity.value} observation to "
                f"{self._fidelity.value} buffer"
            )
        self._pending.append(
            _IndexedObservation(self._next_insertion_index, observation)
        )
        self._next_insertion_index += 1

    def submit_many(self, observations: Iterable[Observation]) -> int:
        candidate = tuple(observations)
        for observation in candidate:
            if not isinstance(observation, Observation):
                raise TypeError("observations must contain only Observation values")
            if observation.fidelity is not self._fidelity:
                raise ValueError(
                    f"cannot submit {observation.fidelity.value} observation to "
                    f"{self._fidelity.value} buffer"
                )
        for observation in candidate:
            self.submit(observation)
        return len(candidate)

    @property
    def pending(self) -> tuple[Observation, ...]:
        """Immutable view for scheduled, transactional sample admission."""
        return tuple(entry.observation for entry in self._pending)

    def candidate(
        self, *, pending_admission=None, retained_admission=None
    ) -> tuple[Observation, ...]:
        combined = sorted(
            (*self._retained, *self._pending),
            key=lambda entry: entry.insertion_index,
        )
        if pending_admission is None:
            if retained_admission is not None:
                raise ValueError(
                    "retained admission requires explicit pending admission"
                )
            selected = self._policy._select_indexed(combined, self._config)
        else:
            mask = tuple(pending_admission)
            if len(mask) != len(self._pending) or any(
                type(v) is not bool for v in mask
            ):
                raise ValueError(
                    "admission must contain one boolean per pending sample"
                )
            keep = (
                (True,) * len(self._retained)
                if retained_admission is None
                else tuple(retained_admission)
            )
            if len(keep) != len(self._retained) or any(
                type(v) is not bool for v in keep
            ):
                raise ValueError(
                    "retained admission must contain one boolean per retained sample"
                )
            selected = tuple(
                sorted(
                    (
                        *(e for e, accept in zip(self._retained, keep) if accept),
                        *(e for e, accept in zip(self._pending, mask) if accept),
                    ),
                    key=lambda e: (e.observation.timestamp, e.insertion_index),
                )
            )
            if len(selected) > self._config.max_samples:
                raise ValueError("admission exceeds retained-sample capacity")
        observations = tuple(entry.observation for entry in selected)
        watermark = max((entry.insertion_index for entry in self._pending), default=-1)
        self._last_candidate = _CandidateRecord(observations, selected, watermark)
        return observations

    def commit(self, candidate: Sequence[Observation]) -> None:
        candidate_tuple = tuple(candidate)
        if self._last_candidate is None:
            raise RuntimeError("candidate() must be called before commit()")
        if candidate_tuple != self._last_candidate.observations:
            raise ValueError("candidate does not match the latest buffer candidate")

        self._retained = self._last_candidate.entries
        watermark = self._last_candidate.pending_watermark
        self._pending = [
            entry for entry in self._pending if entry.insertion_index > watermark
        ]
        self._last_candidate = None
