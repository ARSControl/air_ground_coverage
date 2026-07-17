"""Controller-independent density conversion and structured-grid utilities."""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.interpolate import RegularGridInterpolator


FloatArray = NDArray[np.float64]


def _finite_array(values: ArrayLike, name: str) -> FloatArray:
    try:
        result = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a numeric array") from exc
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return np.array(result, dtype=float, copy=True)


def _positive_tolerance(tolerance: float) -> float:
    if isinstance(tolerance, bool):
        raise TypeError("tolerance must be a real scalar")
    try:
        result = float(tolerance)
    except (TypeError, ValueError) as exc:
        raise TypeError("tolerance must be a real scalar") from exc
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError("tolerance must be finite and positive")
    return result


def _weights(integration_weights: ArrayLike, shape: tuple[int, ...]) -> FloatArray:
    result = _finite_array(integration_weights, "integration_weights")
    if result.shape != shape:
        raise ValueError(
            "integration_weights must match the density shape; "
            f"got {result.shape} and {shape}"
        )
    if np.any(result < 0.0):
        raise ValueError("integration_weights must be nonnegative")
    if not np.any(result > 0.0):
        raise ValueError("integration_weights must contain positive mass")
    return result


def _inclusion_mask(mask: ArrayLike | None, shape: tuple[int, ...]) -> NDArray[np.bool_]:
    if mask is None:
        return np.ones(shape, dtype=bool)
    result = np.asarray(mask)
    if result.shape != shape:
        raise ValueError(f"mask must have shape {shape}; got {result.shape}")
    if result.dtype != np.bool_:
        raise TypeError("mask must be a boolean array")
    return np.array(result, dtype=bool, copy=True)


def positive_part(values: ArrayLike) -> FloatArray:
    """Clip a finite scalar field at zero without changing positive values."""
    array = _finite_array(values, "values")
    return np.maximum(array, 0.0)


def normalize_nonnegative_density(
    values: ArrayLike,
    integration_weights: ArrayLike,
    mask: ArrayLike | None = None,
    tolerance: float = 1.0e-8,
) -> FloatArray:
    """Mask and normalize nonnegative values to weighted integral one.

    Boolean ``mask`` values of ``True`` identify included/free cells.
    """
    tolerance = _positive_tolerance(tolerance)
    density = _finite_array(values, "values")
    if np.any(density < 0.0):
        raise ValueError("values must be nonnegative")
    weights = _weights(integration_weights, density.shape)
    included = _inclusion_mask(mask, density.shape)
    density = np.where(included, density, 0.0)
    mass = float(np.sum(density * weights, dtype=float))
    if not math.isfinite(mass) or mass <= 0.0:
        raise ValueError("density has zero or nonfinite weighted mass")
    density /= mass
    validate_density(density, weights, tolerance)
    return density


def validate_density(
    density: ArrayLike,
    integration_weights: ArrayLike,
    tolerance: float = 1.0e-8,
) -> None:
    """Raise when a density is not finite, nonnegative, and normalized."""
    tolerance = _positive_tolerance(tolerance)
    array = _finite_array(density, "density")
    if np.any(array < 0.0):
        raise ValueError("density must be nonnegative")
    weights = _weights(integration_weights, array.shape)
    mass = float(np.sum(array * weights, dtype=float))
    if not math.isfinite(mass) or not math.isclose(
        mass, 1.0, rel_tol=tolerance, abs_tol=tolerance
    ):
        raise ValueError(f"density weighted integral must equal one; got {mass}")


def build_aerial_target(
    density_high: ArrayLike,
    variance_high: ArrayLike,
    lambda_interest: float,
    lambda_uncertainty: float,
    integration_weights: ArrayLike,
    mask: ArrayLike | None = None,
    tolerance: float = 1.0e-8,
) -> FloatArray:
    """Build a normalized interest/uncertainty target from a cached posterior."""
    density = _finite_array(density_high, "density_high")
    variance = _finite_array(variance_high, "variance_high")
    if density.shape != variance.shape:
        raise ValueError("density_high and variance_high must have matching shapes")
    if np.any(density < 0.0):
        raise ValueError("density_high must be nonnegative")
    if np.any(variance < 0.0):
        raise ValueError("variance_high must be nonnegative")
    interest_weight = _nonnegative_weight(lambda_interest, "lambda_interest")
    uncertainty_weight = _nonnegative_weight(
        lambda_uncertainty, "lambda_uncertainty"
    )
    if interest_weight == 0.0 and uncertainty_weight == 0.0:
        raise ValueError("at least one aerial-target weight must be positive")
    standard_deviation = np.sqrt(variance)
    maximum_standard_deviation = float(np.max(standard_deviation))
    if maximum_standard_deviation > 0.0:
        normalized_standard_deviation = (
            standard_deviation / maximum_standard_deviation
        )
    else:
        normalized_standard_deviation = np.zeros_like(standard_deviation)
    combined = (
        interest_weight * density
        + uncertainty_weight * normalized_standard_deviation
    )
    return normalize_nonnegative_density(
        combined, integration_weights, mask=mask, tolerance=tolerance
    )


def _nonnegative_weight(value: float, name: str) -> float:
    if isinstance(value, bool):
        raise TypeError(f"{name} must be a real scalar")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must be a real scalar") from exc
    if not math.isfinite(result) or result < 0.0:
        raise ValueError(f"{name} must be finite and nonnegative")
    return result


def resample_structured_grid(
    values: ArrayLike,
    source_x: ArrayLike,
    source_y: ArrayLike,
    query_points: ArrayLike,
) -> FloatArray:
    """Bilinearly resample a ``(y, x)`` grid at in-bounds ``(x, y)`` points."""
    x_coordinates = _finite_array(source_x, "source_x")
    y_coordinates = _finite_array(source_y, "source_y")
    grid_values = _finite_array(values, "values")
    queries = _finite_array(query_points, "query_points")
    if x_coordinates.ndim != 1 or x_coordinates.size < 2:
        raise ValueError("source_x must be one-dimensional with at least two points")
    if y_coordinates.ndim != 1 or y_coordinates.size < 2:
        raise ValueError("source_y must be one-dimensional with at least two points")
    if np.any(np.diff(x_coordinates) <= 0.0):
        raise ValueError("source_x must be strictly increasing")
    if np.any(np.diff(y_coordinates) <= 0.0):
        raise ValueError("source_y must be strictly increasing")
    expected_shape = (y_coordinates.size, x_coordinates.size)
    if grid_values.shape != expected_shape:
        raise ValueError(f"values must have shape {expected_shape}")
    if queries.ndim != 2 or queries.shape[1:] != (2,):
        raise ValueError("query_points must have shape (N, 2)")

    interpolator = RegularGridInterpolator(
        (y_coordinates, x_coordinates),
        grid_values,
        method="linear",
        bounds_error=True,
    )
    return np.asarray(interpolator(queries[:, [1, 0]]), dtype=float)
