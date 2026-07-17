"""Numerical verification of the autoregressive multi-fidelity GP."""

from __future__ import annotations

import numpy as np
import pytest

from conftest import SyntheticMultiFidelityField
from src.core.multifidelity_gp import MultiFidelityGaussianProcess, RBFKernel


EMPTY_POSITIONS = np.empty((0, 2), dtype=float)
EMPTY_VALUES = np.empty(0, dtype=float)


def points_on_x_axis(x_values: np.ndarray) -> np.ndarray:
    return np.column_stack((x_values, np.zeros_like(x_values)))


def direct_rbf(
    left: np.ndarray, right: np.ndarray, length_scale: float, variance: float
) -> np.ndarray:
    differences = left[:, None, :] - right[None, :, :]
    squared_distances = np.sum(differences * differences, axis=2)
    return variance * np.exp(-0.5 * squared_distances / length_scale**2)


def direct_joint_reference(
    *,
    rho: float,
    low_length_scale: float,
    low_variance: float,
    discrepancy_length_scale: float,
    discrepancy_variance: float,
    jitter: float,
    low_positions: np.ndarray,
    low_values: np.ndarray,
    low_noise: np.ndarray,
    high_positions: np.ndarray,
    high_values: np.ndarray,
    high_noise: np.ndarray,
    query: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Independent dense reference assembled directly from the design."""
    low_low = direct_rbf(
        low_positions, low_positions, low_length_scale, low_variance
    )
    low_high = rho * direct_rbf(
        low_positions, high_positions, low_length_scale, low_variance
    )
    high_high = (
        rho**2
        * direct_rbf(high_positions, high_positions, low_length_scale, low_variance)
        + direct_rbf(
            high_positions,
            high_positions,
            discrepancy_length_scale,
            discrepancy_variance,
        )
    )
    low_low[np.diag_indices(low_positions.shape[0])] += low_noise
    high_high[np.diag_indices(high_positions.shape[0])] += high_noise
    covariance = np.block([[low_low, low_high], [low_high.T, high_high]])
    covariance[np.diag_indices_from(covariance)] += jitter

    low_to_query = rho * direct_rbf(
        low_positions, query, low_length_scale, low_variance
    )
    high_to_query = (
        rho**2
        * direct_rbf(high_positions, query, low_length_scale, low_variance)
        + direct_rbf(
            high_positions,
            query,
            discrepancy_length_scale,
            discrepancy_variance,
        )
    )
    cross = np.vstack((low_to_query, high_to_query))
    observations = np.concatenate((low_values, high_values))
    mean = cross.T @ np.linalg.solve(covariance, observations)
    prior = np.full(query.shape[0], rho**2 * low_variance + discrepancy_variance)
    variance = prior - np.sum(cross * np.linalg.solve(covariance, cross), axis=0)
    return mean, variance


def make_synthetic_gp(rho: float) -> MultiFidelityGaussianProcess:
    return MultiFidelityGaussianProcess(
        rho=rho,
        low_kernel=RBFKernel(length_scale=0.85, variance=1.5),
        discrepancy_kernel=RBFKernel(length_scale=0.2, variance=0.65),
        jitter=1.0e-9,
    )


def test_low_observations_recover_broad_high_structure(
    synthetic_multifidelity_field: SyntheticMultiFidelityField,
) -> None:
    field = synthetic_multifidelity_field
    low_positions = points_on_x_axis(np.linspace(-3.5, 3.5, 29))
    query = points_on_x_axis(np.linspace(-3.25, 3.25, 101))
    gp = make_synthetic_gp(field.rho).fit(
        low_positions,
        field.low(low_positions),
        1.0e-5,
        EMPTY_POSITIONS,
        EMPTY_VALUES,
        0.0,
    )

    prediction = gp.predict_high(query)
    broad_target = field.rho * field.low(query)
    rmse = np.sqrt(np.mean((prediction.mean - broad_target) ** 2))
    correlation = np.corrcoef(prediction.mean, broad_target)[0, 1]

    assert rmse < 0.025
    assert correlation > 0.995


def test_high_observations_add_a_narrow_local_correction(
    synthetic_multifidelity_field: SyntheticMultiFidelityField,
) -> None:
    field = synthetic_multifidelity_field
    low_positions = points_on_x_axis(np.linspace(-3.5, 3.5, 29))
    low_values = field.low(low_positions)
    high_positions = points_on_x_axis(np.array([0.38, 0.55, 0.72]))
    query = points_on_x_axis(np.array([-2.0, 0.55]))

    low_only = make_synthetic_gp(field.rho).fit(
        low_positions, low_values, 1.0e-5, EMPTY_POSITIONS, EMPTY_VALUES, 0.0
    )
    corrected = make_synthetic_gp(field.rho).fit(
        low_positions,
        low_values,
        1.0e-5,
        high_positions,
        field.high(high_positions),
        1.0e-5,
    )
    before = low_only.predict_high(query).mean
    after = corrected.predict_high(query).mean
    truth = field.high(query)

    assert abs(after[1] - truth[1]) < 0.25 * abs(before[1] - truth[1])
    local_change = abs(after[1] - before[1])
    far_change = abs(after[0] - before[0])
    assert local_change > 20.0 * max(far_change, 1.0e-12)


def test_posterior_uncertainty_decreases_near_observations() -> None:
    low_position = np.array([[-1.0, 0.0]])
    high_position = np.array([[1.0, 0.0]])
    query = np.array([[-1.0, 0.0], [0.0, 3.0], [1.0, 0.0]])
    empty_gp = make_synthetic_gp(0.8)
    low_gp = make_synthetic_gp(0.8).fit(
        low_position,
        np.array([0.4]),
        1.0e-6,
        EMPTY_POSITIONS,
        EMPTY_VALUES,
        0.0,
    )
    both_gp = make_synthetic_gp(0.8).fit(
        low_position,
        np.array([0.4]),
        1.0e-6,
        high_position,
        np.array([0.7]),
        1.0e-6,
    )

    prior = empty_gp.predict_high(query).variance
    after_low = low_gp.predict_high(query).variance
    after_both = both_gp.predict_high(query).variance

    assert after_low[0] < prior[0]
    assert after_low[0] < after_low[1]
    assert after_both[2] < after_low[2]
    assert after_both[2] < after_both[1]


def test_prediction_matches_direct_joint_covariance_reference() -> None:
    rho = 0.65
    low_length_scale = 1.1
    low_variance = 1.3
    discrepancy_length_scale = 0.35
    discrepancy_variance = 0.45
    jitter = 1.0e-8
    low_positions = np.array([[-1.0, 0.2], [0.4, -0.3], [1.3, 0.5]])
    low_values = np.array([0.2, 0.8, -0.1])
    low_noise = np.array([0.02, 0.01, 0.03])
    high_positions = np.array([[-0.3, 0.1], [0.9, -0.2]])
    high_values = np.array([0.4, 1.1])
    high_noise = np.array([0.005, 0.008])
    query = np.array([[-0.7, 0.0], [0.2, 0.4], [1.1, -0.1]])

    gp = MultiFidelityGaussianProcess(
        rho=rho,
        low_kernel=RBFKernel(low_length_scale, low_variance),
        discrepancy_kernel=RBFKernel(
            discrepancy_length_scale, discrepancy_variance
        ),
        jitter=jitter,
    ).fit(
        low_positions,
        low_values,
        low_noise,
        high_positions,
        high_values,
        high_noise,
    )
    actual = gp.predict_high(query)
    expected_mean, expected_variance = direct_joint_reference(
        rho=rho,
        low_length_scale=low_length_scale,
        low_variance=low_variance,
        discrepancy_length_scale=discrepancy_length_scale,
        discrepancy_variance=discrepancy_variance,
        jitter=jitter,
        low_positions=low_positions,
        low_values=low_values,
        low_noise=low_noise,
        high_positions=high_positions,
        high_values=high_values,
        high_noise=high_noise,
        query=query,
    )

    np.testing.assert_allclose(actual.mean, expected_mean, rtol=1.0e-11, atol=1e-12)
    np.testing.assert_allclose(
        actual.variance, expected_variance, rtol=1.0e-11, atol=1e-12
    )


def test_grid_variances_are_finite_and_nonnegative() -> None:
    rng = np.random.default_rng(7321)
    low_positions = rng.uniform(-2.0, 2.0, size=(15, 2))
    high_positions = rng.uniform(-1.0, 1.0, size=(7, 2))
    query = rng.uniform(-3.0, 3.0, size=(250, 2))
    gp = make_synthetic_gp(0.8).fit(
        low_positions,
        np.sin(low_positions[:, 0]),
        0.01,
        high_positions,
        np.cos(high_positions[:, 1]),
        0.002,
    )

    prediction = gp.predict_high(query)
    assert np.all(np.isfinite(prediction.mean))
    assert np.all(np.isfinite(prediction.variance))
    assert np.all(prediction.variance >= 0.0)


def test_duplicate_and_nearly_coincident_samples_are_stable() -> None:
    low_positions = np.array(
        [[0.0, 0.0], [0.0, 0.0], [1.0e-13, -1.0e-13], [1.0, 0.0]]
    )
    high_positions = np.array([[0.5, 0.0], [0.5, 0.0], [0.5 + 1.0e-14, 0.0]])
    gp = make_synthetic_gp(0.8).fit(
        low_positions,
        np.array([0.5, 0.5, 0.5, 0.2]),
        0.0,
        high_positions,
        np.array([0.9, 0.9, 0.9]),
        0.0,
    )

    prediction = gp.predict_high(np.array([[0.0, 0.0], [0.5, 0.0]]))
    assert gp.effective_jitter >= gp.jitter
    assert np.all(np.isfinite(prediction.mean))
    assert np.all(np.isfinite(prediction.variance))
    assert np.all(prediction.variance >= 0.0)


@pytest.mark.parametrize("include_low", [True, False])
def test_single_fidelity_prediction_matches_direct_reference(
    include_low: bool,
) -> None:
    low_positions = np.array([[-0.8, 0.0], [0.7, 0.2]])
    low_values = np.array([0.3, 0.8])
    low_noise = np.array([0.01, 0.02])
    high_positions = np.array([[-0.2, 0.1], [1.0, -0.1]])
    high_values = np.array([0.6, 0.2])
    high_noise = np.array([0.004, 0.006])
    if include_low:
        high_positions, high_values, high_noise = (
            EMPTY_POSITIONS,
            EMPTY_VALUES,
            EMPTY_VALUES,
        )
    else:
        low_positions, low_values, low_noise = (
            EMPTY_POSITIONS,
            EMPTY_VALUES,
            EMPTY_VALUES,
        )
    query = np.array([[-0.5, 0.0], [0.5, 0.0]])
    gp = make_synthetic_gp(0.8).fit(
        low_positions,
        low_values,
        low_noise,
        high_positions,
        high_values,
        high_noise,
    )
    actual = gp.predict_high(query)
    expected_mean, expected_variance = direct_joint_reference(
        rho=0.8,
        low_length_scale=0.85,
        low_variance=1.5,
        discrepancy_length_scale=0.2,
        discrepancy_variance=0.65,
        jitter=1.0e-9,
        low_positions=low_positions,
        low_values=low_values,
        low_noise=low_noise,
        high_positions=high_positions,
        high_values=high_values,
        high_noise=high_noise,
        query=query,
    )
    np.testing.assert_allclose(actual.mean, expected_mean, rtol=1.0e-10, atol=1e-12)
    np.testing.assert_allclose(
        actual.variance, expected_variance, rtol=1.0e-10, atol=1e-12
    )


def test_fixed_seed_produces_deterministic_output() -> None:
    def run_once() -> tuple[np.ndarray, np.ndarray]:
        rng = np.random.default_rng(20260716)
        low_positions = rng.normal(size=(12, 2))
        high_positions = rng.normal(size=(6, 2))
        query = rng.normal(size=(20, 2))
        gp = make_synthetic_gp(0.8).fit(
            low_positions,
            rng.normal(size=12),
            0.01,
            high_positions,
            rng.normal(size=6),
            0.001,
        )
        prediction = gp.predict_high(query)
        return prediction.mean, prediction.variance

    first_mean, first_variance = run_once()
    second_mean, second_variance = run_once()
    np.testing.assert_array_equal(first_mean, second_mean)
    np.testing.assert_array_equal(first_variance, second_variance)


def test_rho_controls_low_to_high_influence() -> None:
    low_positions = points_on_x_axis(np.array([-1.0, 0.0, 1.0]))
    low_values = np.array([0.2, 1.0, 0.4])
    query = np.array([[0.0, 0.0]])
    uncorrelated = make_synthetic_gp(0.0).fit(
        low_positions, low_values, 1.0e-6, EMPTY_POSITIONS, EMPTY_VALUES, 0.0
    )
    correlated = make_synthetic_gp(0.8).fit(
        low_positions, low_values, 1.0e-6, EMPTY_POSITIONS, EMPTY_VALUES, 0.0
    )

    assert uncorrelated.predict_high(query).mean[0] == 0.0
    assert correlated.predict_high(query).mean[0] > 0.5


def test_discrepancy_length_scale_controls_correction_locality() -> None:
    high_position = np.array([[0.0, 0.0]])
    query = np.array([[0.0, 0.0], [1.0, 0.0]])

    def correction(length_scale: float) -> np.ndarray:
        gp = MultiFidelityGaussianProcess(
            rho=0.0,
            low_kernel=RBFKernel(1.0, 1.0),
            discrepancy_kernel=RBFKernel(length_scale, 1.0),
            jitter=1.0e-10,
        ).fit(
            EMPTY_POSITIONS,
            EMPTY_VALUES,
            0.0,
            high_position,
            np.array([1.0]),
            1.0e-6,
        )
        return gp.predict_high(query).mean

    narrow = correction(0.15)
    broad = correction(1.5)
    assert narrow[0] > 0.99
    assert narrow[1] < 1.0e-8
    assert broad[1] > 0.7
