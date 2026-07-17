"""Deterministic fixtures for the multi-fidelity GP tests."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
import pytest
from numpy.typing import NDArray


FloatArray = NDArray[np.float64]
Field = Callable[[FloatArray], FloatArray]


@dataclass(frozen=True)
class SyntheticMultiFidelityField:
    """Broad LOW structure plus a narrow HIGH-only correction."""

    rho: float
    low: Field
    discrepancy: Field
    high: Field


@pytest.fixture
def synthetic_multifidelity_field() -> SyntheticMultiFidelityField:
    rho = 0.8

    def low(points: FloatArray) -> FloatArray:
        points = np.asarray(points, dtype=float)
        first = np.exp(
            -(
                (points[:, 0] + 1.1) ** 2
                + 0.55 * (points[:, 1] - 0.1) ** 2
            )
            / (2.0 * 1.45**2)
        )
        second = 0.5 * np.exp(
            -(
                (points[:, 0] - 1.6) ** 2
                + 0.8 * (points[:, 1] + 0.2) ** 2
            )
            / (2.0 * 1.0**2)
        )
        return first + second

    def discrepancy(points: FloatArray) -> FloatArray:
        points = np.asarray(points, dtype=float)
        return 0.7 * np.exp(
            -(
                (points[:, 0] - 0.55) ** 2
                + (points[:, 1] + 0.05) ** 2
            )
            / (2.0 * 0.2**2)
        )

    def high(points: FloatArray) -> FloatArray:
        return rho * low(points) + discrepancy(points)

    return SyntheticMultiFidelityField(
        rho=rho,
        low=low,
        discrepancy=discrepancy,
        high=high,
    )
