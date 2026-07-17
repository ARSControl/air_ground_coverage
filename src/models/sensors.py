"""Deterministic simulated LOW/HIGH scalar-field sensors."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.ndimage import gaussian_filter

from ..core.observations import Fidelity, Observation


FloatArray = NDArray[np.float64]
BoolArray = NDArray[np.bool_]


def _finite_array(values: ArrayLike, name: str, ndim: int) -> FloatArray:
    try:
        result = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if result.ndim != ndim:
        raise ValueError(f"{name} must be {ndim}-dimensional; got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return np.array(result, dtype=float, copy=True)


def _finite_float(value: float, name: str) -> float:
    if isinstance(value, (bool, np.bool_)):
        raise TypeError(f"{name} must be a real scalar")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must be a real scalar") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _readonly(values: FloatArray) -> FloatArray:
    result = np.array(values, dtype=float, copy=True, order="C")
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class FidelityFields:
    """Simulator-only LOW, HIGH, and discrepancy truth fields."""

    low: FloatArray
    high: FloatArray
    discrepancy: FloatArray

    def __post_init__(self) -> None:
        low = _finite_array(self.low, "low", ndim=2)
        high = _finite_array(self.high, "high", ndim=2)
        discrepancy = _finite_array(self.discrepancy, "discrepancy", ndim=2)
        if low.shape != high.shape or high.shape != discrepancy.shape:
            raise ValueError("all fidelity fields must have the same shape")
        object.__setattr__(self, "low", _readonly(low))
        object.__setattr__(self, "high", _readonly(high))
        object.__setattr__(self, "discrepancy", _readonly(discrepancy))


def build_fidelity_fields(
    high_field: ArrayLike,
    rho: float,
    smoothing_sigma_cells: float,
    mask: ArrayLike | None = None,
) -> FidelityFields:
    """Create a broad LOW truth and exact autoregressive discrepancy truth."""
    high = _finite_array(high_field, "high_field", ndim=2)
    rho = _finite_float(rho, "rho")
    sigma = _finite_float(smoothing_sigma_cells, "smoothing_sigma_cells")
    if sigma < 0.0:
        raise ValueError("smoothing_sigma_cells must be nonnegative")
    if mask is None:
        included = np.ones(high.shape, dtype=bool)
    else:
        raw_mask = np.asarray(mask)
        if raw_mask.dtype != np.bool_ or raw_mask.shape != high.shape:
            raise ValueError("mask must be boolean and match high_field shape")
        included = np.array(raw_mask, dtype=bool, copy=True)
        if not np.any(included):
            raise ValueError("mask must include at least one field cell")

    masked_high = np.where(included, high, 0.0)
    if sigma == 0.0:
        low = masked_high.copy()
    else:
        numerator = gaussian_filter(masked_high, sigma=sigma, mode="nearest")
        denominator = gaussian_filter(
            included.astype(float), sigma=sigma, mode="nearest"
        )
        low = np.divide(
            numerator,
            denominator,
            out=np.zeros_like(numerator),
            where=denominator > np.finfo(float).eps,
        )
        low[~included] = 0.0
    high = masked_high
    discrepancy = high - rho * low
    reconstructed = rho * low + discrepancy
    if not np.allclose(reconstructed, high, rtol=1.0e-12, atol=1.0e-12):
        raise FloatingPointError("fidelity-field autoregressive identity failed")
    return FidelityFields(low=low, high=high, discrepancy=discrepancy)


class SimulatedScalarFieldSensor:
    """Area-uniformly sample a raster field within each robot's FOV sector."""

    def __init__(
        self,
        fidelity: Fidelity,
        sample_count: int,
        sample_range: float,
        fov_degrees: float,
        noise_variance: float,
        rng: np.random.Generator,
        max_resample_attempts: int = 50,
    ) -> None:
        if not isinstance(fidelity, Fidelity):
            raise TypeError("fidelity must be a Fidelity value")
        if isinstance(sample_count, bool) or not isinstance(sample_count, int):
            raise TypeError("sample_count must be an integer")
        if sample_count < 1:
            raise ValueError("sample_count must be positive")
        sample_range = _finite_float(sample_range, "sample_range")
        fov_degrees = _finite_float(fov_degrees, "fov_degrees")
        noise_variance = _finite_float(noise_variance, "noise_variance")
        if sample_range < 0.0:
            raise ValueError("sample_range must be nonnegative")
        if not 0.0 < fov_degrees <= 360.0:
            raise ValueError("fov_degrees must be in (0, 360]")
        if noise_variance < 0.0:
            raise ValueError("noise_variance must be nonnegative")
        if not isinstance(rng, np.random.Generator):
            raise TypeError("rng must be a numpy.random.Generator")
        if (
            isinstance(max_resample_attempts, bool)
            or not isinstance(max_resample_attempts, int)
            or max_resample_attempts < 0
        ):
            raise ValueError("max_resample_attempts must be a nonnegative integer")
        self.fidelity = fidelity
        self.sample_count = sample_count
        self.sample_range = sample_range
        self.fov_degrees = fov_degrees
        self.noise_variance = noise_variance
        self.rng = rng
        self.max_resample_attempts = max_resample_attempts

    def collect(
        self, team: Any, field: ArrayLike, timestamp: float
    ) -> tuple[Observation, ...]:
        field_array = _finite_array(field, "field", ndim=2)
        timestamp = _finite_float(timestamp, "timestamp")
        agents = getattr(team, "agents", None)
        if agents is None:
            raise TypeError("team must expose an agents sequence")
        height, width = field_array.shape
        observations: list[Observation] = []
        noise_standard_deviation = math.sqrt(self.noise_variance)

        for index, agent in enumerate(agents):
            position = _finite_array(getattr(agent, "position", None), "position", 1)
            if position.shape != (2,):
                raise ValueError("agent position must have shape (2,)")
            heading = _agent_heading(agent)
            sample_positions = (
                self._sample_position(position, heading, width, height)
                for _ in range(self.sample_count)
            )
            for x_position, y_position in sample_positions:
                x_index = int(np.clip(np.rint(x_position), 0, width - 1))
                y_index = int(np.clip(np.rint(y_position), 0, height - 1))
                value = float(field_array[y_index, x_index])
                if noise_standard_deviation > 0.0:
                    value += float(self.rng.normal(0.0, noise_standard_deviation))
                observations.append(
                    Observation(
                        timestamp=timestamp,
                        robot_id=str(getattr(agent, "id", index)),
                        position=(x_position, y_position),
                        value=value,
                        fidelity=self.fidelity,
                        noise_variance=self.noise_variance,
                    )
                )
        return tuple(observations)

    def _sample_position(
        self,
        center: FloatArray,
        heading: float,
        width: int,
        height: int,
    ) -> tuple[float, float]:
        """Draw an area-uniform point inside the heading-centered FOV sector."""
        half_width = 0.5 * math.radians(self.fov_degrees)
        for _ in range(self.max_resample_attempts + 1):
            angle = heading + float(self.rng.uniform(-half_width, half_width))
            distance = self.sample_range * math.sqrt(float(self.rng.uniform()))
            x_position = float(center[0] + distance * math.cos(angle))
            y_position = float(center[1] + distance * math.sin(angle))
            if 0.0 <= x_position <= width - 1 and 0.0 <= y_position <= height - 1:
                return x_position, y_position
        return (
            float(np.clip(center[0], 0.0, width - 1)),
            float(np.clip(center[1], 0.0, height - 1)),
        )


def _agent_heading(agent: Any) -> float:
    if hasattr(agent, "theta"):
        heading = getattr(agent, "theta")
    elif hasattr(agent, "heading"):
        heading = getattr(agent, "heading")
    else:
        heading = 0.0
    return _finite_float(heading, "agent heading")
