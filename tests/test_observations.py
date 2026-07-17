"""Tests for timestamped observations and transactional bounded buffers."""

from __future__ import annotations

from dataclasses import FrozenInstanceError

import numpy as np
import pytest

from src.core.observations import (
    BoundedObservationBuffer,
    Fidelity,
    Observation,
    RetentionConfig,
    SpatialAgeRetentionPolicy,
)


def observation(
    timestamp: float,
    x_position: float,
    *,
    fidelity: Fidelity = Fidelity.LOW,
    value: float | None = None,
    robot_id: str = "robot-0",
) -> Observation:
    return Observation(
        timestamp=timestamp,
        robot_id=robot_id,
        position=(x_position, 0.0),
        value=x_position if value is None else value,
        fidelity=fidelity,
        noise_variance=0.01,
    )


def test_observation_is_validated_normalized_and_immutable() -> None:
    item = Observation(1, "aerial-2", [2, 3], 4, Fidelity.LOW, 0)
    assert item.timestamp == 1.0
    assert item.position == (2.0, 3.0)
    assert item.value == 4.0
    with pytest.raises(FrozenInstanceError):
        item.value = 2.0


@pytest.mark.parametrize(
    ("field", "bad_value", "error"),
    [
        ("timestamp", np.nan, ValueError),
        ("timestamp", np.inf, ValueError),
        ("robot_id", "", ValueError),
        ("robot_id", "   ", ValueError),
        ("position", (1.0,), ValueError),
        ("position", (1.0, np.inf), ValueError),
        ("value", np.nan, ValueError),
        ("fidelity", "low", TypeError),
        ("noise_variance", -0.1, ValueError),
        ("noise_variance", np.inf, ValueError),
    ],
)
def test_observation_rejects_invalid_fields(
    field: str, bad_value: object, error: type[Exception]
) -> None:
    arguments: dict[str, object] = {
        "timestamp": 0.0,
        "robot_id": "robot",
        "position": (0.0, 1.0),
        "value": 0.5,
        "fidelity": Fidelity.LOW,
        "noise_variance": 0.01,
    }
    arguments[field] = bad_value
    with pytest.raises(error):
        Observation(**arguments)


@pytest.mark.parametrize(
    "arguments",
    [
        {"max_samples": 0, "min_separation": 0.0},
        {"max_samples": 2, "min_separation": -1.0},
        {"max_samples": 2, "min_separation": np.nan},
    ],
)
def test_retention_config_rejects_invalid_values(arguments: dict[str, object]) -> None:
    with pytest.raises((TypeError, ValueError)):
        RetentionConfig(**arguments)


def test_fidelity_mismatch_is_rejected_without_partial_submit() -> None:
    buffer = BoundedObservationBuffer(
        Fidelity.HIGH, RetentionConfig(max_samples=5, min_separation=0.0)
    )
    with pytest.raises(ValueError, match="low observation"):
        buffer.submit(observation(0.0, 0.0))
    with pytest.raises(ValueError):
        buffer.submit_many(
            [
                observation(0.0, 0.0, fidelity=Fidelity.HIGH),
                observation(1.0, 1.0, fidelity=Fidelity.LOW),
            ]
        )
    assert buffer.pending_count == 0


def test_low_and_high_buffers_store_same_position_separately() -> None:
    config = RetentionConfig(max_samples=2, min_separation=0.5)
    low = BoundedObservationBuffer(Fidelity.LOW, config)
    high = BoundedObservationBuffer(Fidelity.HIGH, config)
    low.submit(observation(0.0, 1.0))
    high.submit(observation(0.0, 1.0, fidelity=Fidelity.HIGH))
    low_candidate = low.candidate()
    high_candidate = high.candidate()
    low.commit(low_candidate)
    high.commit(high_candidate)
    assert len(low.retained) == len(high.retained) == 1
    assert low.retained[0].fidelity is Fidelity.LOW
    assert high.retained[0].fidelity is Fidelity.HIGH


def test_out_of_order_arrivals_are_exposed_chronologically() -> None:
    buffer = BoundedObservationBuffer(
        Fidelity.LOW, RetentionConfig(max_samples=5, min_separation=0.0)
    )
    buffer.submit_many(
        [observation(3.0, 3.0), observation(1.0, 1.0), observation(2.0, 2.0)]
    )
    assert [item.timestamp for item in buffer.candidate()] == [1.0, 2.0, 3.0]


def test_equal_timestamp_uses_later_insertion_as_newer_duplicate() -> None:
    buffer = BoundedObservationBuffer(
        Fidelity.LOW, RetentionConfig(max_samples=3, min_separation=0.0)
    )
    older_arrival = observation(1.0, 0.0, value=1.0)
    newer_arrival = observation(1.0, 0.0, value=2.0)
    buffer.submit_many([older_arrival, newer_arrival])
    assert buffer.candidate() == (newer_arrival,)


def test_newer_spatial_duplicate_replaces_older_retained_data() -> None:
    buffer = BoundedObservationBuffer(
        Fidelity.LOW, RetentionConfig(max_samples=3, min_separation=0.25)
    )
    old = observation(1.0, 0.0, value=1.0)
    buffer.submit(old)
    first = buffer.candidate()
    buffer.commit(first)
    new = observation(4.0, 0.1, value=4.0)
    buffer.submit(new)
    second = buffer.candidate()
    assert second == (new,)


def test_minimum_separation_and_maximum_count_are_enforced() -> None:
    policy = SpatialAgeRetentionPolicy()
    entries = (
        observation(1.0, 0.0),
        observation(2.0, 0.2),
        observation(3.0, 1.0),
        observation(4.0, 2.0),
    )
    selected = policy.select(
        entries, RetentionConfig(max_samples=2, min_separation=0.5)
    )
    assert [(item.timestamp, item.position[0]) for item in selected] == [
        (3.0, 1.0),
        (4.0, 2.0),
    ]


def test_points_exactly_at_positive_minimum_separation_are_retained() -> None:
    selected = SpatialAgeRetentionPolicy().select(
        (observation(1.0, 0.0), observation(2.0, 0.5)),
        RetentionConfig(max_samples=2, min_separation=0.5),
    )
    assert len(selected) == 2


def test_candidate_does_not_mutate_retained_or_pending_state() -> None:
    buffer = BoundedObservationBuffer(
        Fidelity.LOW, RetentionConfig(max_samples=2, min_separation=0.0)
    )
    buffer.submit(observation(1.0, 0.0))
    candidate = buffer.candidate()
    assert candidate
    assert buffer.retained == ()
    assert buffer.pending_count == 1


def test_commit_clears_only_pending_entries_in_the_candidate_snapshot() -> None:
    buffer = BoundedObservationBuffer(
        Fidelity.LOW, RetentionConfig(max_samples=3, min_separation=0.0)
    )
    first = observation(1.0, 0.0)
    later_arrival = observation(2.0, 1.0)
    buffer.submit(first)
    candidate = buffer.candidate()
    buffer.submit(later_arrival)
    buffer.commit(candidate)
    assert buffer.retained == (first,)
    assert buffer.pending_count == 1
    assert buffer.candidate() == (first, later_arrival)


def test_repeated_submission_sequences_are_deterministic() -> None:
    def run() -> tuple[Observation, ...]:
        buffer = BoundedObservationBuffer(
            Fidelity.LOW, RetentionConfig(max_samples=3, min_separation=0.4)
        )
        buffer.submit_many(
            [
                observation(3.0, 1.0),
                observation(1.0, 0.0),
                observation(4.0, 1.1),
                observation(2.0, 2.0),
            ]
        )
        candidate = buffer.candidate()
        buffer.commit(candidate)
        return buffer.retained

    assert run() == run()
