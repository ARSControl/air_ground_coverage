"""Tests for simulator-only fidelity fields and dedicated-RNG sensors."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pytest

from src.core.observations import Fidelity
from src.models.sensors import SimulatedScalarFieldSensor, build_fidelity_fields


@dataclass
class FakeAgent:
    id: int
    position: np.ndarray
    heading: float

    @property
    def theta(self) -> float:
        return self.heading


@dataclass
class FakeTeam:
    agents: list[FakeAgent]


def test_low_truth_is_smoother_and_autoregressive_identity_is_exact() -> None:
    high = np.zeros((31, 31), dtype=float)
    high[15, 15] = 1.0
    high[8, 22] = 0.6
    fields = build_fidelity_fields(high, rho=0.75, smoothing_sigma_cells=2.0)
    assert fields.low.max() < fields.high.max()
    assert np.var(fields.low) < np.var(fields.high)
    np.testing.assert_allclose(
        fields.high, 0.75 * fields.low + fields.discrepancy, atol=1.0e-12
    )
    with pytest.raises(ValueError):
        fields.low[0, 0] = 2.0


def test_fidelity_field_mask_excludes_obstacles() -> None:
    high = np.ones((9, 9))
    mask = np.ones((9, 9), dtype=bool)
    mask[3:6, 3:6] = False
    fields = build_fidelity_fields(high, 0.8, 1.0, mask)
    assert np.all(fields.low[~mask] == 0.0)
    assert np.all(fields.high[~mask] == 0.0)
    assert np.all(fields.discrepancy[~mask] == 0.0)


@pytest.mark.parametrize(
    ("fidelity", "expected"),
    [(Fidelity.LOW, Fidelity.LOW), (Fidelity.HIGH, Fidelity.HIGH)],
)
def test_sensor_records_metadata_positions_and_nearest_field_values(
    fidelity: Fidelity, expected: Fidelity
) -> None:
    field = np.add.outer(10.0 * np.arange(20), np.arange(20))
    team = FakeTeam([FakeAgent(7, np.array([10.0, 10.0]), 0.0)])
    sensor = SimulatedScalarFieldSensor(
        fidelity=fidelity,
        sample_count=3,
        sample_range=4.0,
        fov_degrees=90.0,
        noise_variance=0.0,
        rng=np.random.default_rng(123),
    )
    observations = sensor.collect(team, field, timestamp=2.5)
    assert len(observations) == 3
    for observation in observations:
        assert observation.fidelity is expected
        assert observation.timestamp == 2.5
        assert observation.robot_id == "7"
        assert observation.noise_variance == 0.0
        x_position, y_position = observation.position
        assert 0.0 <= x_position <= 19.0
        assert 0.0 <= y_position <= 19.0
        expected_value = field[int(round(y_position)), int(round(x_position))]
        assert observation.value == expected_value


def test_samples_respect_configured_range_and_angular_fov() -> None:
    center = np.array([20.0, 20.0])
    team = FakeTeam([FakeAgent(0, center, 0.0)])
    sensor = SimulatedScalarFieldSensor(
        Fidelity.HIGH,
        sample_count=7,
        sample_range=6.0,
        fov_degrees=80.0,
        noise_variance=0.0,
        rng=np.random.default_rng(91),
    )
    observations = sensor.collect(team, np.ones((41, 41)), 0.0)
    offsets = np.asarray([item.position for item in observations]) - center
    distances = np.linalg.norm(offsets, axis=1)
    angles = np.arctan2(offsets[:, 1], offsets[:, 0])
    assert np.all(distances <= 6.0)
    assert np.all(np.abs(angles) <= np.deg2rad(40.0) + 1.0e-12)


def test_uniform_sector_sampling_is_area_uniform_and_not_fixed_to_rays() -> None:
    center = np.array([50.0, 50.0])
    sample_range = 20.0
    sample_count = 4096
    team = FakeTeam([FakeAgent(0, center, 0.25)])
    sensor = SimulatedScalarFieldSensor(
        Fidelity.HIGH,
        sample_count=sample_count,
        sample_range=sample_range,
        fov_degrees=120.0,
        noise_variance=0.0,
        rng=np.random.default_rng(2026),
    )
    observations = sensor.collect(team, np.ones((101, 101)), 0.0)
    offsets = np.asarray([item.position for item in observations]) - center
    distances = np.linalg.norm(offsets, axis=1)
    relative_angles = np.arctan2(offsets[:, 1], offsets[:, 0]) - 0.25
    assert np.all(distances <= sample_range)
    assert np.all(np.abs(relative_angles) <= np.deg2rad(60.0))
    normalized_mean_squared_radius = float(
        np.mean((distances / sample_range) ** 2)
    )
    assert normalized_mean_squared_radius == pytest.approx(0.5, abs=0.02)
    assert np.unique(np.round(relative_angles, decimals=6)).size > 4000


def test_uniform_sector_sampling_is_deterministic() -> None:
    team = FakeTeam([FakeAgent(0, np.array([10.0, 10.0]), -0.2)])

    def run() -> tuple:
        sensor = SimulatedScalarFieldSensor(
            Fidelity.HIGH,
            16,
            4.0,
            270.0,
            0.01,
            np.random.default_rng(81),
        )
        return sensor.collect(team, np.ones((21, 21)), 2.0)

    assert run() == run()


def test_fixed_sensor_generators_are_deterministic() -> None:
    team = FakeTeam([FakeAgent(0, np.array([10.0, 10.0]), 0.3)])

    def run() -> tuple:
        sensor = SimulatedScalarFieldSensor(
            Fidelity.LOW,
            5,
            3.0,
            180.0,
            0.04,
            np.random.default_rng(450),
        )
        return sensor.collect(team, np.ones((21, 21)), 1.0)

    assert run() == run()


def test_sensor_does_not_advance_numpy_global_rng() -> None:
    team = FakeTeam([FakeAgent(0, np.array([10.0, 10.0]), 0.0)])
    sensor = SimulatedScalarFieldSensor(
        Fidelity.LOW,
        4,
        2.0,
        360.0,
        0.01,
        np.random.default_rng(17),
    )
    np.random.seed(2026)
    expected = np.random.random(5)
    np.random.seed(2026)
    sensor.collect(team, np.ones((21, 21)), 0.0)
    actual = np.random.random(5)
    np.testing.assert_array_equal(actual, expected)


def test_invalid_sensor_configuration_and_field_are_rejected() -> None:
    with pytest.raises(ValueError):
        SimulatedScalarFieldSensor(
            Fidelity.LOW, 0, 1.0, 90.0, 0.0, np.random.default_rng(1)
        )
    with pytest.raises(ValueError):
        build_fidelity_fields(np.ones((3, 3)), 0.8, -1.0)
    with pytest.raises(TypeError, match="sampling_pattern"):
        SimulatedScalarFieldSensor(
            Fidelity.HIGH,
            1,
            1.0,
            90.0,
            0.0,
            np.random.default_rng(1),
            sampling_pattern="rings",
        )
