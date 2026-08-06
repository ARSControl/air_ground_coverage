"""Pure evaluation metrics for saved multi-fidelity simulation outputs."""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import ArrayLike, NDArray


FloatArray = NDArray[np.float64]


def clipped_reconstruction(posterior_mean: ArrayLike) -> FloatArray:
    """Return the nonnegative field reconstruction ``max(mean, 0)``."""
    return np.maximum(_vector(posterior_mean, "posterior_mean"), 0.0)


def normalized_density(values: ArrayLike, integration_weights: ArrayLike) -> FloatArray:
    field = _vector(values, "values")
    weights = _weights(integration_weights, field.size)
    if np.any(field < 0.0):
        raise ValueError("density values must be nonnegative")
    mass = float(np.sum(field * weights, dtype=float))
    if not math.isfinite(mass) or mass <= 0.0:
        raise ValueError("density must have positive finite weighted mass")
    return field / mass


def kl_divergence(
    truth_density: ArrayLike,
    estimated_density: ArrayLike,
    integration_weights: ArrayLike,
    *,
    epsilon: float = 1.0e-12,
) -> float:
    """Return weighted discrete D_KL(truth || estimate)."""
    truth = normalized_density(truth_density, integration_weights)
    estimate = _vector(estimated_density, "estimated_density")
    weights = _weights(integration_weights, truth.size)
    if np.any(estimate < 0.0):
        raise ValueError("estimated_density must be nonnegative")
    epsilon = _positive_float(epsilon, "epsilon")
    estimate = normalized_density(np.maximum(estimate, epsilon), weights)
    included = truth > 0.0
    value = np.sum(
        weights[included]
        * truth[included]
        * np.log(truth[included] / estimate[included]),
        dtype=float,
    )
    return float(max(value, 0.0))


def nrmse(truth_field: ArrayLike, posterior_mean: ArrayLike) -> float:
    truth = _vector(truth_field, "truth_field")
    mean = clipped_reconstruction(posterior_mean)
    if mean.shape != truth.shape:
        raise ValueError("posterior_mean must match truth_field")
    span = float(np.max(truth) - np.min(truth))
    if span <= 0.0:
        raise ValueError("NRMSE requires a nonconstant truth field")
    rmse = float(np.sqrt(np.mean((mean - truth) ** 2, dtype=float)))
    return rmse / span


def negative_log_predictive_density(
    truth_field: ArrayLike,
    posterior_mean: ArrayLike,
    posterior_variance: ArrayLike,
    integration_weights: ArrayLike,
    *,
    variance_floor: float = 1.0e-12,
) -> float:
    """Return weighted marginal Gaussian NLPD for the latent HIGH field."""
    truth = _vector(truth_field, "truth_field")
    mean = _vector(posterior_mean, "posterior_mean")
    variance = _vector(posterior_variance, "posterior_variance")
    if mean.shape != truth.shape or variance.shape != truth.shape:
        raise ValueError("posterior vectors must match truth_field")
    if np.any(variance < 0.0):
        raise ValueError("posterior_variance must be nonnegative")
    weights = _weights(integration_weights, truth.size)
    floor = _positive_float(variance_floor, "variance_floor")
    safe_variance = np.maximum(variance, floor)
    pointwise = 0.5 * np.log(2.0 * np.pi * safe_variance) + 0.5 * (
        (truth - mean) ** 2 / safe_variance
    )
    value = float(
        np.sum(weights * pointwise, dtype=float)
        / np.sum(weights, dtype=float)
    )
    if not math.isfinite(value):
        raise FloatingPointError("NLPD produced a nonfinite value")
    return value


def calibration_95(
    truth_field: ArrayLike,
    posterior_mean: ArrayLike,
    posterior_variance: ArrayLike,
) -> float:
    truth = _vector(truth_field, "truth_field")
    mean = _vector(posterior_mean, "posterior_mean")
    variance = _vector(posterior_variance, "posterior_variance")
    if mean.shape != truth.shape or variance.shape != truth.shape:
        raise ValueError("posterior vectors must match truth_field")
    if np.any(variance < 0.0):
        raise ValueError("posterior_variance must be nonnegative")
    radius = 1.96 * np.sqrt(variance)
    return float(np.mean((truth >= mean - radius) & (truth <= mean + radius)))


def covered_probability_mass(
    query_points: ArrayLike,
    truth_density: ArrayLike,
    integration_weights: ArrayLike,
    ground_states: ArrayLike,
    *,
    fov_degrees: float,
    sensing_range: float,
) -> float:
    """Integrate truth over the union of heading-centered ground FOV sectors."""
    points = np.asarray(query_points, dtype=float)
    if points.ndim != 2 or points.shape[1:] != (2,):
        raise ValueError("query_points must have shape (N, 2)")
    if not np.all(np.isfinite(points)):
        raise ValueError("query_points must be finite")
    density = normalized_density(truth_density, integration_weights)
    weights = _weights(integration_weights, points.shape[0])
    states = np.asarray(ground_states, dtype=float)
    if states.ndim != 2 or states.shape[1] < 3:
        raise ValueError("ground_states must have shape (robots, state_dim>=3)")
    if not np.all(np.isfinite(states)):
        raise ValueError("ground_states must be finite")
    fov = _positive_float(fov_degrees, "fov_degrees")
    if fov > 360.0:
        raise ValueError("fov_degrees must not exceed 360")
    radius = _nonnegative_float(sensing_range, "sensing_range")
    covered = np.zeros(points.shape[0], dtype=bool)
    half_angle = 0.5 * np.deg2rad(fov)
    for state in states:
        displacement = points - state[:2]
        distances = np.linalg.norm(displacement, axis=1)
        if fov >= 360.0:
            angular = np.ones(points.shape[0], dtype=bool)
        else:
            bearings = np.arctan2(displacement[:, 1], displacement[:, 0])
            difference = np.arctan2(
                np.sin(bearings - state[2]), np.cos(bearings - state[2])
            )
            angular = np.abs(difference) <= half_angle
        covered |= (distances <= radius) & angular
    return float(np.sum(density[covered] * weights[covered], dtype=float))


def sector_footprint_area(*, fov_degrees: float, sensing_range: float) -> float:
    """Return the continuous area of one circular-sector sensing footprint."""
    fov = _positive_float(fov_degrees, "fov_degrees")
    if fov > 360.0:
        raise ValueError("fov_degrees must not exceed 360")
    radius = _nonnegative_float(sensing_range, "sensing_range")
    return 0.5 * np.deg2rad(fov) * radius**2


def maximum_density_mass_for_area(
    truth_density: ArrayLike,
    integration_weights: ArrayLike,
    *,
    area_budget: float,
    free_mask: ArrayLike | None = None,
) -> float:
    """Return mass in the densest free-space subset of a fixed area.

    Grid cells are sorted by probability-density value, and the final cell is
    included fractionally when its quadrature weight crosses the area budget.
    This is the discrete upper bound obtained when footprint geometry is
    relaxed but its total nominal area is preserved.
    """
    field = _vector(truth_density, "truth_density")
    weights = _weights(integration_weights, field.size)
    if np.any(field < 0.0):
        raise ValueError("truth_density must be nonnegative")
    if free_mask is None:
        free = np.ones(field.size, dtype=bool)
    else:
        free = np.asarray(free_mask, dtype=bool)
        if free.shape != field.shape:
            raise ValueError("free_mask must match truth_density")
    if not np.any(free & (weights > 0.0)):
        raise ValueError("free_mask must contain positive integration area")

    density = normalized_density(np.where(free, field, 0.0), weights)
    budget = min(
        _nonnegative_float(area_budget, "area_budget"),
        float(np.sum(weights[free], dtype=float)),
    )
    if budget == 0.0:
        return 0.0

    indices = np.flatnonzero(free & (weights > 0.0))
    order = indices[np.argsort(-density[indices], kind="stable")]
    remaining = budget
    mass = 0.0
    for index in order:
        used_area = min(remaining, float(weights[index]))
        mass += float(density[index]) * used_area
        remaining -= used_area
        if remaining <= 0.0:
            break
    return float(min(max(mass, 0.0), 1.0))


def footprint_normalized_coverage(
    covered_mass: float,
    maximum_mass: float,
    *,
    tolerance: float = 1.0e-9,
) -> float:
    """Normalize actual visible truth mass by the fixed-area density oracle."""
    covered = _nonnegative_float(covered_mass, "covered_mass")
    maximum = _positive_float(maximum_mass, "maximum_mass")
    tolerance_value = _nonnegative_float(tolerance, "tolerance")
    if covered > maximum + tolerance_value:
        raise ValueError("covered_mass must not exceed maximum_mass")
    return float(np.clip(covered / maximum, 0.0, 1.0))


def _vector(values: ArrayLike, name: str) -> FloatArray:
    result = np.asarray(values, dtype=float)
    if result.ndim != 1 or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite one-dimensional array")
    return np.array(result, dtype=float, copy=True)


def _weights(values: ArrayLike, size: int) -> FloatArray:
    result = _vector(values, "integration_weights")
    if result.shape != (size,) or np.any(result < 0.0) or not np.any(result > 0.0):
        raise ValueError("integration_weights must match and contain positive mass")
    return result


def _positive_float(value: float, name: str) -> float:
    result = _nonnegative_float(value, name)
    if result <= 0.0:
        raise ValueError(f"{name} must be positive")
    return result


def _nonnegative_float(value: float, name: str) -> float:
    if isinstance(value, bool):
        raise TypeError(f"{name} must be a real scalar")
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise ValueError(f"{name} must be finite and nonnegative")
    return result
