"""Behavioral tests for the explicitly updated central estimator."""

from __future__ import annotations

import numpy as np
import pytest

from src.core.multifidelity_estimator import (
    CentralAsynchronousEstimator,
    EstimatorSettings,
    HyperparameterOptimizationSettings,
    UpdateStatus,
)
from src.core.multifidelity_gp import (
    KernelHyperparameterBounds,
    MultiFidelityGaussianProcess,
    RBFKernel,
)
from src.core.observations import Fidelity, Observation, RetentionConfig


def make_settings(
    *,
    max_low: int = 20,
    max_high: int = 20,
    low_separation: float = 0.0,
    high_separation: float = 0.0,
    optimization: HyperparameterOptimizationSettings | None = None,
) -> EstimatorSettings:
    x_values = np.linspace(-3.0, 3.0, 61)
    query_points = np.column_stack((x_values, np.zeros_like(x_values)))
    step = x_values[1] - x_values[0]
    weights = np.full(x_values.size, step)
    weights[[0, -1]] *= 0.5
    return EstimatorSettings(
        gp=MultiFidelityGaussianProcess(
            rho=0.8,
            low_kernel=RBFKernel(0.9, 1.4),
            discrepancy_kernel=RBFKernel(0.25, 0.6),
            jitter=1.0e-9,
        ),
        low_retention=RetentionConfig(max_low, low_separation),
        high_retention=RetentionConfig(max_high, high_separation),
        query_points=query_points,
        query_shape=(x_values.size,),
        integration_weights=weights,
        normalization_tolerance=1.0e-9,
        hyperparameter_optimization=(
            HyperparameterOptimizationSettings()
            if optimization is None
            else optimization
        ),
    )


def item(
    timestamp: float,
    x_position: float,
    fidelity: Fidelity,
    value: float,
    robot_id: str = "sensor-0",
) -> Observation:
    return Observation(
        timestamp=timestamp,
        robot_id=robot_id,
        position=(x_position, 0.0),
        value=value,
        fidelity=fidelity,
        noise_variance=1.0e-4,
    )


def broad_low(x_position: float) -> float:
    return float(np.exp(-0.5 * (x_position / 1.2) ** 2))


def high_value(x_position: float) -> float:
    correction = 0.7 * np.exp(-0.5 * ((x_position - 0.5) / 0.2) ** 2)
    return float(0.8 * broad_low(x_position) + correction)


def low_observations() -> list[Observation]:
    return [
        item(float(index), float(x_position), Fidelity.LOW, broad_low(x_position))
        for index, x_position in enumerate(np.linspace(-2.5, 2.5, 11))
    ]


def test_submission_does_not_fit_until_explicit_update() -> None:
    estimator = CentralAsynchronousEstimator(make_settings())
    estimator.submit_many(low_observations())
    assert estimator.version == 0
    assert estimator.low_sample_count == 0
    assert estimator.pending_count == 11
    assert estimator.latest_posterior is None


def test_low_only_update_publishes_normalized_finite_posterior() -> None:
    estimator = CentralAsynchronousEstimator(make_settings())
    estimator.submit_many(low_observations())
    report = estimator.update(10.0)
    snapshot = estimator.latest_posterior
    assert report.status is UpdateStatus.UPDATED
    assert report.version == estimator.version == 1
    assert snapshot is not None
    assert snapshot.low_sample_count == estimator.low_sample_count == 11
    assert snapshot.high_sample_count == estimator.high_sample_count == 0
    assert np.all(np.isfinite(snapshot.high_mean))
    assert np.all(np.isfinite(snapshot.high_variance))
    assert np.all(snapshot.high_variance >= 0.0)
    assert np.all(snapshot.density >= 0.0)
    assert np.sum(snapshot.density * snapshot.integration_weights) == pytest.approx(
        1.0
    )
    expected_density = np.maximum(snapshot.high_mean, 0.0)
    expected_density /= np.sum(
        expected_density * snapshot.integration_weights
    )
    np.testing.assert_array_equal(snapshot.density, expected_density)
    assert snapshot.fit_duration >= 0.0
    assert snapshot.prediction_duration >= 0.0
    assert estimator.pending_count == 0


def test_high_data_arriving_later_changes_posterior_and_version() -> None:
    estimator = CentralAsynchronousEstimator(make_settings())
    estimator.submit_many(low_observations())
    estimator.update(10.0)
    first = estimator.latest_posterior
    estimator.submit_many(
        [
            item(11.0, 0.35, Fidelity.HIGH, high_value(0.35), "ground-0"),
            item(11.0, 0.50, Fidelity.HIGH, high_value(0.50), "ground-1"),
            item(11.0, 0.65, Fidelity.HIGH, high_value(0.65), "ground-2"),
        ]
    )
    report = estimator.update(12.0)
    second = estimator.latest_posterior
    assert first is not None and second is not None
    assert report.status is UpdateStatus.UPDATED
    assert second.version == first.version + 1 == 2
    assert second.high_sample_count == 3
    center = np.argmin(np.abs(second.query_points[:, 0] - 0.5))
    assert second.high_mean[center] > first.high_mean[center] + 0.4


def test_different_sensor_frequencies_accumulate_until_scheduler_update() -> None:
    estimator = CentralAsynchronousEstimator(make_settings())
    events = [
        item(0.0, -2.0, Fidelity.LOW, broad_low(-2.0)),
        item(0.2, 0.4, Fidelity.HIGH, high_value(0.4)),
        item(0.5, -1.0, Fidelity.LOW, broad_low(-1.0)),
        item(1.0, 0.0, Fidelity.LOW, broad_low(0.0)),
        item(1.2, 0.6, Fidelity.HIGH, high_value(0.6)),
        item(1.5, 1.0, Fidelity.LOW, broad_low(1.0)),
    ]
    for event in events:
        estimator.submit(event)
    assert estimator.version == 0
    assert estimator.pending_count == 6
    report = estimator.update(2.0)
    assert report.status is UpdateStatus.UPDATED
    assert estimator.low_sample_count == 4
    assert estimator.high_sample_count == 2


def test_delayed_out_of_order_arrivals_are_order_independent() -> None:
    observations = [
        item(3.0, 1.5, Fidelity.LOW, broad_low(1.5)),
        item(1.0, -1.5, Fidelity.LOW, broad_low(-1.5)),
        item(2.0, 0.0, Fidelity.LOW, broad_low(0.0)),
    ]

    def run(order: list[Observation]) -> tuple[np.ndarray, np.ndarray]:
        estimator = CentralAsynchronousEstimator(make_settings())
        estimator.submit_many(order)
        assert estimator.update(5.0).status is UpdateStatus.UPDATED
        snapshot = estimator.latest_posterior
        assert snapshot is not None
        return snapshot.high_mean, snapshot.high_variance

    forward = run(observations)
    delayed = run([observations[0], observations[2], observations[1]])
    np.testing.assert_array_equal(forward[0], delayed[0])
    np.testing.assert_array_equal(forward[1], delayed[1])


def test_update_with_no_new_data_preserves_snapshot_identity() -> None:
    estimator = CentralAsynchronousEstimator(make_settings())
    initial_report = estimator.update(0.0)
    assert initial_report.status is UpdateStatus.NO_NEW_DATA
    assert estimator.latest_posterior is None
    estimator.submit_many(low_observations())
    estimator.update(10.0)
    previous = estimator.latest_posterior
    report = estimator.update(20.0)
    assert report.status is UpdateStatus.NO_NEW_DATA
    assert report.version == 1
    assert estimator.latest_posterior is previous


def test_separate_dataset_limits_and_minimum_separation() -> None:
    estimator = CentralAsynchronousEstimator(
        make_settings(
            max_low=3,
            max_high=2,
            low_separation=0.4,
            high_separation=0.1,
        )
    )
    estimator.submit_many(
        [
            item(float(i), x, Fidelity.LOW, broad_low(x))
            for i, x in enumerate([-2.0, -1.0, 0.0, 0.1, 1.0, 2.0])
        ]
    )
    estimator.submit_many(
        [
            item(float(i), x, Fidelity.HIGH, high_value(x))
            for i, x in enumerate([0.4, 0.5, 0.6])
        ]
    )
    estimator.update(10.0)
    assert estimator.low_sample_count == 3
    assert estimator.high_sample_count == 2


def test_same_position_fidelities_coexist_and_duplicates_are_stable() -> None:
    estimator = CentralAsynchronousEstimator(make_settings())
    estimator.submit_many(
        [
            item(0.0, 0.0, Fidelity.LOW, 0.5),
            item(1.0, 0.0, Fidelity.LOW, 0.6),
            item(0.5, 0.0, Fidelity.HIGH, 0.9),
            item(1.5, 0.0, Fidelity.HIGH, 1.0),
        ]
    )
    assert estimator.update(2.0).status is UpdateStatus.UPDATED
    assert estimator.low_sample_count == 1
    assert estimator.high_sample_count == 1
    snapshot = estimator.latest_posterior
    assert snapshot is not None
    assert np.all(np.isfinite(snapshot.high_mean))


def test_snapshot_metadata_and_arrays_are_immutable() -> None:
    settings = make_settings()
    estimator = CentralAsynchronousEstimator(settings)
    estimator.submit_many(low_observations())
    estimator.update(10.0)
    snapshot = estimator.latest_posterior
    assert snapshot is not None
    assert snapshot.timestamp == 10.0
    assert snapshot.version == 1
    assert snapshot.query_shape == (61,)
    assert snapshot.is_valid
    assert snapshot.status is UpdateStatus.UPDATED
    assert snapshot.effective_jitter > 0.0
    with pytest.raises(ValueError):
        snapshot.high_mean[0] = 10.0
    with pytest.raises(ValueError):
        snapshot.query_points[0, 0] = 10.0


def test_failed_normalization_preserves_snapshot_buffers_version_and_pending(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    estimator = CentralAsynchronousEstimator(make_settings())
    estimator.submit_many(low_observations())
    estimator.update(10.0)
    previous = estimator.latest_posterior
    previous_counts = (estimator.low_sample_count, estimator.high_sample_count)
    estimator.submit(item(11.0, 0.5, Fidelity.HIGH, high_value(0.5)))

    def fail_normalization(*args: object, **kwargs: object) -> np.ndarray:
        raise FloatingPointError("injected normalization failure")

    monkeypatch.setattr(
        "src.core.multifidelity_estimator.normalize_nonnegative_density",
        fail_normalization,
    )
    report = estimator.update(12.0)
    assert report.status is UpdateStatus.FAILED
    assert report.exception_type == "FloatingPointError"
    assert "injected" in report.message
    assert estimator.latest_posterior is previous
    assert estimator.version == 1
    assert (estimator.low_sample_count, estimator.high_sample_count) == previous_counts
    assert estimator.pending_count == 1

    monkeypatch.undo()
    retry = estimator.update(13.0)
    assert retry.status is UpdateStatus.UPDATED
    assert estimator.version == 2
    assert estimator.high_sample_count == 1
    assert estimator.pending_count == 0


def test_fixed_seed_inputs_produce_deterministic_snapshot_values() -> None:
    def run() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        rng = np.random.default_rng(4404)
        estimator = CentralAsynchronousEstimator(make_settings())
        observations = []
        for index, position in enumerate(rng.uniform(-2.0, 2.0, size=8)):
            observations.append(
                item(float(index), float(position), Fidelity.LOW, broad_low(position))
            )
        for index, position in enumerate(rng.uniform(0.2, 0.8, size=4)):
            observations.append(
                item(
                    float(index) + 0.25,
                    float(position),
                    Fidelity.HIGH,
                    high_value(position),
                )
            )
        estimator.submit_many(observations)
        estimator.update(10.0)
        snapshot = estimator.latest_posterior
        assert snapshot is not None
        return snapshot.high_mean, snapshot.high_variance, snapshot.density

    first = run()
    second = run()
    for first_array, second_array in zip(first, second, strict=True):
        np.testing.assert_array_equal(first_array, second_array)


def _optimization_settings(
    *, interval: int = 1
) -> HyperparameterOptimizationSettings:
    return HyperparameterOptimizationSettings(
        enabled=True,
        fit_interval_updates=interval,
        min_samples=5,
        num_restarts=0,
        max_iterations=60,
        bounds=KernelHyperparameterBounds(
            low_length_scale=(0.2, 3.0),
            low_variance=(0.05, 3.0),
            discrepancy_length_scale=(0.1, 1.5),
            discrepancy_variance=(0.01, 2.0),
        ),
    )


def test_scheduled_hyperparameter_fit_is_recorded_in_snapshot() -> None:
    estimator = CentralAsynchronousEstimator(
        make_settings(optimization=_optimization_settings(interval=2))
    )
    estimator.submit_many(low_observations())
    estimator.update(10.0)
    first = estimator.latest_posterior
    assert first is not None
    assert first.hyperparameter_fit_performed
    assert first.hyperparameter_fit_duration >= 0.0
    assert first.low_kernel_length_scale > 0.0
    assert first.discrepancy_kernel_length_scale > 0.0

    estimator.submit(item(11.0, 0.5, Fidelity.HIGH, high_value(0.5)))
    estimator.update(11.0)
    second = estimator.latest_posterior
    assert second is not None
    assert second.version == 2
    assert not second.hyperparameter_fit_performed
    assert second.low_kernel_length_scale == first.low_kernel_length_scale
    assert (
        second.discrepancy_kernel_length_scale
        == first.discrepancy_kernel_length_scale
    )


def test_failed_hyperparameter_fit_preserves_previous_transaction(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    estimator = CentralAsynchronousEstimator(
        make_settings(optimization=_optimization_settings())
    )
    estimator.submit_many(low_observations())
    assert estimator.update(10.0).status is UpdateStatus.UPDATED
    previous = estimator.latest_posterior
    assert previous is not None
    estimator.submit(item(11.0, 0.5, Fidelity.HIGH, high_value(0.5)))

    def fail_fit(*args, **kwargs):
        raise RuntimeError("injected hyperparameter failure")

    monkeypatch.setattr(
        MultiFidelityGaussianProcess, "fit_hyperparameters", fail_fit
    )
    report = estimator.update(11.0)
    assert report.status is UpdateStatus.FAILED
    assert report.exception_type == "RuntimeError"
    assert estimator.latest_posterior is previous
    assert estimator.version == 1
    assert estimator.pending_count == 1
