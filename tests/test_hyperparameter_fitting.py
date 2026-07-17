"""Deterministic tests for central multi-fidelity kernel fitting."""

from __future__ import annotations

import numpy as np
import pytest

from src.core.multifidelity_gp import (
    KernelHyperparameterBounds,
    MultiFidelityGaussianProcess,
    RBFKernel,
)


def _points(x_values: np.ndarray) -> np.ndarray:
    return np.column_stack((x_values, np.zeros_like(x_values)))


def _training_case():
    rho = 0.8
    low_x = np.linspace(-4.0, 4.0, 17)
    high_x = np.linspace(-3.5, 3.5, 11)
    query_x = np.linspace(-4.0, 4.0, 161)
    low_values = np.exp(-0.5 * (low_x / 1.8) ** 2)
    high_values = rho * np.exp(-0.5 * (high_x / 1.8) ** 2) + 0.35 * np.exp(
        -0.5 * ((high_x - 1.0) / 0.7) ** 2
    )
    truth = rho * np.exp(-0.5 * (query_x / 1.8) ** 2) + 0.35 * np.exp(
        -0.5 * ((query_x - 1.0) / 0.7) ** 2
    )
    return (
        _points(low_x),
        low_values,
        _points(high_x),
        high_values,
        _points(query_x),
        truth,
    )


def _poor_initial_gp() -> MultiFidelityGaussianProcess:
    return MultiFidelityGaussianProcess(
        rho=0.8,
        low_kernel=RBFKernel(0.35, 0.5),
        discrepancy_kernel=RBFKernel(0.25, 0.5),
        jitter=1.0e-8,
    )


def _bounds() -> KernelHyperparameterBounds:
    return KernelHyperparameterBounds(
        low_length_scale=(0.2, 5.0),
        low_variance=(0.05, 3.0),
        discrepancy_length_scale=(0.2, 3.0),
        discrepancy_variance=(0.01, 2.0),
    )


def test_bounded_fitting_improves_likelihood_and_reconstruction() -> None:
    low_x, low_y, high_x, high_y, query, truth = _training_case()
    gp = _poor_initial_gp()
    gp.fit(low_x, low_y, 1.0e-4, high_x, high_y, 1.0e-4)
    fixed_prediction = gp.predict_high(query).mean

    result = gp.fit_hyperparameters(
        low_x,
        low_y,
        1.0e-4,
        high_x,
        high_y,
        1.0e-4,
        bounds=_bounds(),
        num_restarts=1,
        max_iterations=100,
    )
    gp.fit(low_x, low_y, 1.0e-4, high_x, high_y, 1.0e-4)
    fitted_prediction = gp.predict_high(query).mean

    fixed_rmse = float(np.sqrt(np.mean((fixed_prediction - truth) ** 2)))
    fitted_rmse = float(np.sqrt(np.mean((fitted_prediction - truth) ** 2)))
    assert result.converged
    assert result.improved
    assert result.final_negative_log_likelihood < -40.0
    assert fitted_rmse < fixed_rmse / 5.0
    for value, limits in zip(
        gp.kernel_hyperparameters, _bounds().as_tuple(), strict=True
    ):
        assert limits[0] <= value <= limits[1]


def test_hyperparameter_fitting_is_deterministic() -> None:
    low_x, low_y, high_x, high_y, _, _ = _training_case()

    def run():
        gp = _poor_initial_gp()
        result = gp.fit_hyperparameters(
            low_x,
            low_y,
            1.0e-4,
            high_x,
            high_y,
            1.0e-4,
            bounds=_bounds(),
            num_restarts=1,
            max_iterations=100,
        )
        return gp.kernel_hyperparameters, result

    first_parameters, first_result = run()
    second_parameters, second_result = run()
    np.testing.assert_array_equal(first_parameters, second_parameters)
    assert first_result == second_result


@pytest.mark.parametrize(
    "limits",
    [(0.0, 1.0), (1.0, 1.0), (2.0, 1.0), (1.0, np.inf)],
)
def test_invalid_positive_bounds_are_rejected(limits) -> None:
    with pytest.raises(ValueError):
        KernelHyperparameterBounds(low_length_scale=limits)


def test_hyperparameter_fitting_requires_observations() -> None:
    gp = _poor_initial_gp()
    with pytest.raises(ValueError, match="requires observations"):
        gp.fit_hyperparameters(
            np.empty((0, 2)),
            np.empty(0),
            np.empty(0),
            np.empty((0, 2)),
            np.empty(0),
            np.empty(0),
            bounds=_bounds(),
        )
