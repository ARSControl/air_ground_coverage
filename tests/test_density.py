"""Tests for positive transformation, weighted normalization, and resampling."""

from __future__ import annotations

import numpy as np
import pytest

from src.core.density import (
    build_aerial_target,
    normalize_nonnegative_density,
    positive_part,
    resample_structured_grid,
    validate_density,
)


def test_positive_part_clips_negative_values_and_preserves_positive_values() -> None:
    result = positive_part(np.array([-1000.0, -0.1, 0.0, 2.5, 1000.0]))
    np.testing.assert_array_equal(result, np.array([0.0, 0.0, 0.0, 2.5, 1000.0]))


def test_positive_part_rejects_nonfinite_values() -> None:
    with pytest.raises(ValueError, match="finite"):
        positive_part(np.array([0.0, np.nan]))


def test_weighted_density_is_nonnegative_and_integrates_to_one() -> None:
    values = np.array([1.0, 3.0, 2.0])
    weights = np.array([0.25, 0.5, 0.25])
    density = normalize_nonnegative_density(values, weights)
    assert np.all(density >= 0.0)
    assert np.sum(density * weights) == pytest.approx(1.0)
    validate_density(density, weights)


def test_masked_cells_are_zero_before_normalization() -> None:
    density = normalize_nonnegative_density(
        np.array([1.0, 100.0, 3.0]),
        np.ones(3),
        mask=np.array([True, False, True]),
    )
    np.testing.assert_array_equal(density, np.array([0.25, 0.0, 0.75]))


@pytest.mark.parametrize(
    ("values", "weights", "mask"),
    [
        (np.zeros(2), np.ones(2), None),
        (np.array([1.0, -1.0]), np.ones(2), None),
        (np.ones(2), np.array([1.0]), None),
        (np.ones(2), np.array([1.0, -1.0]), None),
        (np.ones(2), np.array([1.0, np.inf]), None),
        (np.ones(2), np.ones(2), np.array([1, 0])),
        (np.ones(2), np.ones(2), np.array([False, False])),
    ],
)
def test_normalization_rejects_invalid_inputs(
    values: np.ndarray, weights: np.ndarray, mask: np.ndarray | None
) -> None:
    with pytest.raises((TypeError, ValueError)):
        normalize_nonnegative_density(values, weights, mask=mask)


@pytest.mark.parametrize(
    "density",
    [np.array([0.2, 0.2]), np.array([-0.5, 1.5]), np.array([np.nan, 1.0])],
)
def test_validate_density_rejects_invalid_density(density: np.ndarray) -> None:
    with pytest.raises(ValueError):
        validate_density(density, np.ones(2))


def test_aerial_target_interest_uncertainty_and_mixed_cases() -> None:
    density = np.array([0.25, 0.75])
    variance = np.array([1.0, 4.0])
    weights = np.ones(2)
    interest = build_aerial_target(density, variance, 1.0, 0.0, weights)
    uncertainty = build_aerial_target(density, variance, 0.0, 1.0, weights)
    mixed = build_aerial_target(density, variance, 2.0, 0.5, weights)
    np.testing.assert_allclose(interest, density)
    np.testing.assert_allclose(uncertainty, np.array([1.0 / 3.0, 2.0 / 3.0]))
    expected_mixed = 2.0 * density + 0.5 * np.array([0.5, 1.0])
    expected_mixed /= expected_mixed.sum()
    np.testing.assert_allclose(mixed, expected_mixed)
    assert np.sum(mixed) == pytest.approx(1.0)
    assert np.all(mixed >= 0.0)


def test_aerial_target_zero_uncertainty_is_stable_with_interest() -> None:
    density = np.array([0.4, 0.6])
    result = build_aerial_target(density, np.zeros(2), 1.0, 2.0, np.ones(2))
    np.testing.assert_allclose(result, density)
    with pytest.raises(ValueError, match="zero"):
        build_aerial_target(density, np.zeros(2), 0.0, 1.0, np.ones(2))


def test_structured_grid_resampling_is_bilinear_and_deterministic() -> None:
    source_x = np.array([0.0, 1.0, 2.0])
    source_y = np.array([0.0, 2.0])
    grid_x, grid_y = np.meshgrid(source_x, source_y)
    values = grid_x + 2.0 * grid_y
    query = np.array([[0.5, 1.0], [2.0, 0.0], [1.25, 0.5]])
    expected = query[:, 0] + 2.0 * query[:, 1]
    first = resample_structured_grid(values, source_x, source_y, query)
    second = resample_structured_grid(values, source_x, source_y, query)
    np.testing.assert_array_equal(first, second)
    np.testing.assert_allclose(first, expected)


def test_structured_grid_resampling_has_explicit_bounds_errors() -> None:
    with pytest.raises(ValueError, match="bounds"):
        resample_structured_grid(
            np.zeros((2, 2)),
            np.array([0.0, 1.0]),
            np.array([0.0, 1.0]),
            np.array([[1.1, 0.0]]),
        )
