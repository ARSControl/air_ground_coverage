"""Public-contract and validation tests for the pure multi-fidelity GP."""

from __future__ import annotations

import inspect

import numpy as np
import pytest

from src.core.multifidelity_gp import (
    HighFidelityPrediction,
    MultiFidelityGaussianProcess,
    RBFKernel,
)


EMPTY_POSITIONS = np.empty((0, 2), dtype=float)
EMPTY_VALUES = np.empty(0, dtype=float)


def make_gp(**overrides: object) -> MultiFidelityGaussianProcess:
    arguments: dict[str, object] = {
        "rho": 0.75,
        "low_kernel": RBFKernel(length_scale=1.2, variance=1.4),
        "discrepancy_kernel": RBFKernel(length_scale=0.3, variance=0.4),
        "jitter": 1.0e-9,
        "max_jitter_attempts": 4,
        "jitter_multiplier": 10.0,
    }
    arguments.update(overrides)
    return MultiFidelityGaussianProcess(**arguments)


@pytest.mark.parametrize(
    ("arguments", "error"),
    [
        ({"length_scale": 0.0, "variance": 1.0}, ValueError),
        ({"length_scale": -1.0, "variance": 1.0}, ValueError),
        ({"length_scale": np.inf, "variance": 1.0}, ValueError),
        ({"length_scale": 1.0, "variance": 0.0}, ValueError),
        ({"length_scale": 1.0, "variance": np.nan}, ValueError),
        ({"length_scale": True, "variance": 1.0}, TypeError),
    ],
)
def test_rbf_kernel_rejects_invalid_configuration(
    arguments: dict[str, float], error: type[Exception]
) -> None:
    with pytest.raises(error):
        RBFKernel(**arguments)


@pytest.mark.parametrize(
    ("overrides", "error"),
    [
        ({"rho": np.nan}, ValueError),
        ({"low_kernel": object()}, TypeError),
        ({"discrepancy_kernel": object()}, TypeError),
        ({"jitter": 0.0}, ValueError),
        ({"max_jitter_attempts": 0}, ValueError),
        ({"max_jitter_attempts": 1.5}, TypeError),
        ({"jitter_multiplier": 1.0}, ValueError),
    ],
)
def test_gp_rejects_invalid_configuration(
    overrides: dict[str, object], error: type[Exception]
) -> None:
    with pytest.raises(error):
        make_gp(**overrides)


def test_empty_data_is_explicit_high_fidelity_prior() -> None:
    gp = make_gp()
    query = np.array([[-1.0, 0.2], [0.5, -0.1], [2.0, 1.0]])

    before_fit = gp.predict_high(query)
    returned = gp.fit(
        EMPTY_POSITIONS,
        EMPTY_VALUES,
        0.01,
        EMPTY_POSITIONS,
        EMPTY_VALUES,
        np.empty(0),
    )
    after_fit = gp.predict_high(query)

    expected_variance = (
        gp.rho**2 * gp.low_kernel.variance + gp.discrepancy_kernel.variance
    )
    assert returned is gp
    assert gp.n_low == 0
    assert gp.n_high == 0
    assert gp.effective_jitter == 0.0
    np.testing.assert_array_equal(before_fit.mean, np.zeros(3))
    np.testing.assert_array_equal(after_fit.mean, np.zeros(3))
    np.testing.assert_allclose(before_fit.variance, expected_variance)
    np.testing.assert_array_equal(before_fit.mean, after_fit.mean)
    np.testing.assert_array_equal(before_fit.variance, after_fit.variance)


@pytest.mark.parametrize("keep_low", [True, False])
def test_single_fidelity_fit_is_supported(keep_low: bool) -> None:
    gp = make_gp()
    positions = np.array([[-0.5, 0.0], [0.5, 0.0]])
    values = np.array([0.2, 0.9])
    if keep_low:
        gp.fit(positions, values, 0.02, EMPTY_POSITIONS, EMPTY_VALUES, 0.0)
    else:
        gp.fit(EMPTY_POSITIONS, EMPTY_VALUES, 0.0, positions, values, 0.02)

    prediction = gp.predict_high(np.array([[0.0, 0.0], [1.0, 0.0]]))

    assert prediction.mean.shape == (2,)
    assert prediction.variance.shape == (2,)
    assert np.all(np.isfinite(prediction.mean))
    assert np.all(np.isfinite(prediction.variance))
    assert np.all(prediction.variance >= 0.0)


@pytest.mark.parametrize(
    ("replacement", "match"),
    [
        (np.array([0.0, 1.0]), "low_positions"),
        (np.zeros((2, 3)), "low_positions"),
        (np.array([[0.0, np.nan]]), "low_positions"),
        (np.array([[0.0, 0.0]]), "low_values"),
        (np.array([np.inf]), "low_values"),
        (np.array([[-0.1]]), "low_noise_variances"),
    ],
)
def test_fit_validates_shapes_finiteness_and_noise(
    replacement: np.ndarray, match: str
) -> None:
    arguments: dict[str, object] = {
        "low_positions": np.array([[0.0, 0.0]]),
        "low_values": np.array([0.5]),
        "low_noise_variances": np.array([0.01]),
        "high_positions": EMPTY_POSITIONS,
        "high_values": EMPTY_VALUES,
        "high_noise_variances": 0.01,
    }
    arguments[match] = replacement
    with pytest.raises(ValueError):
        make_gp().fit(**arguments)


@pytest.mark.parametrize(
    "query",
    [np.array([0.0, 1.0]), np.zeros((1, 3)), np.array([[np.nan, 0.0]])],
)
def test_predict_validates_query_positions(query: np.ndarray) -> None:
    with pytest.raises(ValueError, match="query_positions"):
        make_gp().predict_high(query)


def test_failed_fit_preserves_previous_valid_state() -> None:
    gp = make_gp().fit(
        np.array([[-1.0, 0.0], [1.0, 0.0]]),
        np.array([0.2, 0.8]),
        0.01,
        np.array([[0.0, 0.0]]),
        np.array([0.7]),
        0.005,
    )
    query = np.array([[-0.5, 0.0], [0.5, 0.0]])
    before = gp.predict_high(query)
    previous_counts = (gp.n_low, gp.n_high, gp.effective_jitter)

    with pytest.raises(ValueError):
        gp.fit(
            np.array([[np.nan, 0.0]]),
            np.array([1.0]),
            0.01,
            EMPTY_POSITIONS,
            EMPTY_VALUES,
            0.01,
        )

    after = gp.predict_high(query)
    assert (gp.n_low, gp.n_high, gp.effective_jitter) == previous_counts
    np.testing.assert_array_equal(after.mean, before.mean)
    np.testing.assert_array_equal(after.variance, before.variance)


def test_prediction_arrays_are_read_only_and_independent_of_inputs() -> None:
    mean = np.array([1.0, 2.0])
    variance = np.array([0.3, 0.4])
    prediction = HighFidelityPrediction(mean=mean, variance=variance)
    mean[0] = 99.0
    variance[0] = 99.0

    assert prediction.mean[0] == 1.0
    assert prediction.variance[0] == 0.3
    with pytest.raises(ValueError):
        prediction.mean[0] = 2.0


def test_production_module_contains_no_explicit_matrix_inverse() -> None:
    source = inspect.getsource(inspect.getmodule(MultiFidelityGaussianProcess))
    assert "np.linalg.inv" not in source
    assert "numpy.linalg.inv" not in source
