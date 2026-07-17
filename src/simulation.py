"""Simulation adapters for deterministic multi-fidelity scheduling and feedback."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Any

import numpy as np
from numpy.typing import ArrayLike

from .core.density import (
    build_aerial_target,
    normalize_nonnegative_density,
    resample_structured_grid,
)
from .core.multifidelity_estimator import (
    CentralAsynchronousEstimator,
    EstimatorSettings,
    HyperparameterOptimizationSettings,
    PosteriorSnapshot,
    UpdateReport,
)
from .core.multifidelity_gp import (
    KernelHyperparameterBounds,
    MultiFidelityGaussianProcess,
    RBFKernel,
)
from .core.observations import Fidelity, RetentionConfig
from .models.sensors import (
    FidelityFields,
    SimulatedScalarFieldSensor,
    build_fidelity_fields,
)


class EstimatorMode(Enum):
    """Estimator branch selected by the top-level configuration."""

    LEGACY = "legacy"
    MULTIFIDELITY = "multifidelity"


def _get(params: Any, key: str, default: Any = None) -> Any:
    if hasattr(params, "get") and not isinstance(params, dict):
        return params.get(key, default)
    value: Any = params
    for component in key.split("."):
        if not isinstance(value, dict):
            return default
        value = value.get(component, default)
    return value


def parse_estimator_mode(params: Any) -> EstimatorMode:
    """Parse the configured estimator mode; a missing key means legacy."""
    raw_mode = _get(params, "estimator_mode", EstimatorMode.LEGACY.value)
    if isinstance(raw_mode, EstimatorMode):
        return raw_mode
    if not isinstance(raw_mode, str):
        raise TypeError("estimator_mode must be a string")
    try:
        return EstimatorMode(raw_mode.strip().lower())
    except ValueError as exc:
        choices = ", ".join(mode.value for mode in EstimatorMode)
        raise ValueError(f"estimator_mode must be one of: {choices}") from exc


class PeriodicEvent:
    """Drift-free deterministic event schedule in simulated seconds."""

    def __init__(
        self,
        period: float,
        start_time: float = 0.0,
        tolerance: float = 1.0e-10,
    ) -> None:
        self.period = _positive_float(period, "period")
        self.start_time = _finite_float(start_time, "start_time")
        self.tolerance = _positive_float(tolerance, "tolerance")
        self._fire_count = 0
        self._next_fire_time = self.start_time

    @property
    def next_fire_time(self) -> float:
        return self._next_fire_time

    @property
    def fire_count(self) -> int:
        return self._fire_count

    def is_due(self, simulation_time: float) -> bool:
        timestamp = _finite_float(simulation_time, "simulation_time")
        return timestamp + self.tolerance >= self._next_fire_time

    def mark_fired(self) -> None:
        self._fire_count += 1
        self._next_fire_time = self.start_time + self._fire_count * self.period


class CoordinatorEventType(Enum):
    LOW_COLLECTION = "low_collection"
    HIGH_COLLECTION = "high_collection"
    ESTIMATOR_UPDATE = "estimator_update"


@dataclass(frozen=True)
class CoordinatorEventReport:
    """Result of checking one coordinator event at one simulation time."""

    event_type: CoordinatorEventType
    simulation_time: float
    fired: bool
    submitted_count: int
    update_report: UpdateReport | None
    posterior_version: int

    def __post_init__(self) -> None:
        if not isinstance(self.event_type, CoordinatorEventType):
            raise TypeError("event_type must be a CoordinatorEventType")
        simulation_time = _finite_float(self.simulation_time, "simulation_time")
        if self.fired is not True and self.fired is not False:
            raise TypeError("fired must be boolean")
        for value, name in (
            (self.submitted_count, "submitted_count"),
            (self.posterior_version, "posterior_version"),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{name} must be a nonnegative integer")
        if self.update_report is not None and not isinstance(
            self.update_report, UpdateReport
        ):
            raise TypeError("update_report must be an UpdateReport or None")
        object.__setattr__(self, "simulation_time", simulation_time)


class MultifidelitySimulationCoordinator:
    """Own one estimator, two sensors, and three independent event schedules."""

    def __init__(
        self,
        estimator: CentralAsynchronousEstimator,
        low_sensor: SimulatedScalarFieldSensor,
        high_sensor: SimulatedScalarFieldSensor,
        fields: FidelityFields,
        low_event: PeriodicEvent,
        high_event: PeriodicEvent,
        update_event: PeriodicEvent,
        lambda_interest: float,
        lambda_uncertainty: float,
        normalization_tolerance: float,
    ) -> None:
        if not isinstance(estimator, CentralAsynchronousEstimator):
            raise TypeError("estimator must be a CentralAsynchronousEstimator")
        if low_sensor.fidelity is not Fidelity.LOW:
            raise ValueError("low_sensor must produce LOW observations")
        if high_sensor.fidelity is not Fidelity.HIGH:
            raise ValueError("high_sensor must produce HIGH observations")
        self.estimator = estimator
        self.low_sensor = low_sensor
        self.high_sensor = high_sensor
        self.fields = fields
        self.low_event = low_event
        self.high_event = high_event
        self.update_event = update_event
        self.lambda_interest = _nonnegative_float(
            lambda_interest, "lambda_interest"
        )
        self.lambda_uncertainty = _nonnegative_float(
            lambda_uncertainty, "lambda_uncertainty"
        )
        if self.lambda_interest == 0.0 and self.lambda_uncertainty == 0.0:
            raise ValueError("at least one aerial-target weight must be positive")
        self.normalization_tolerance = _positive_float(
            normalization_tolerance, "normalization_tolerance"
        )
        self._cached_aerial_target: np.ndarray | None = None
        self._cached_aerial_target_version = 0
        self._cached_target_x: np.ndarray | None = None
        self._cached_target_y: np.ndarray | None = None
        self._cached_target_weights: np.ndarray | None = None
        self._cached_target_mask: np.ndarray | None = None
        self._cached_ground_density: np.ndarray | None = None
        self._cached_ground_density_version = 0
        self._cached_ground_points: np.ndarray | None = None
        self._cached_ground_weights: np.ndarray | None = None

    @property
    def latest_posterior(self) -> PosteriorSnapshot | None:
        return self.estimator.latest_posterior

    @property
    def aerial_target_version(self) -> int:
        """Posterior version used by the currently cached aerial target."""
        return self._cached_aerial_target_version

    @property
    def ground_density_version(self) -> int:
        """Posterior version used by the currently cached ground density."""
        return self._cached_ground_density_version

    def aerial_target(
        self,
        map_x: ArrayLike,
        map_y: ArrayLike,
        integration_weights: ArrayLike,
        mask: ArrayLike | None,
    ) -> np.ndarray | None:
        """Return the cached posterior target resampled onto a HEDAC grid."""
        snapshot = self.latest_posterior
        if snapshot is None:
            return None
        x_coordinates = _strict_axis(map_x, "map_x")
        y_coordinates = _strict_axis(map_y, "map_y")
        controller_shape = (y_coordinates.size, x_coordinates.size)
        weights = _controller_array(
            integration_weights, "integration_weights", controller_shape, float
        )
        if np.any(weights < 0.0) or not np.any(weights > 0.0):
            raise ValueError(
                "integration_weights must be nonnegative with positive mass"
            )
        controller_mask = None
        if mask is not None:
            controller_mask = _controller_array(
                mask, "mask", controller_shape, bool
            )
            if not np.any(controller_mask & (weights > 0.0)):
                raise ValueError("mask must include positive integration weight")

        if self._target_cache_matches(
            snapshot.version,
            x_coordinates,
            y_coordinates,
            weights,
            controller_mask,
        ):
            return self._cached_aerial_target

        source_target = build_aerial_target(
            snapshot.density,
            snapshot.high_variance,
            self.lambda_interest,
            self.lambda_uncertainty,
            snapshot.integration_weights,
            tolerance=self.normalization_tolerance,
        )
        source_x, source_y = _structured_snapshot_axes(snapshot)
        grid_x, grid_y = np.meshgrid(x_coordinates, y_coordinates)
        controller_points = np.column_stack((grid_x.ravel(), grid_y.ravel()))
        projected = resample_structured_grid(
            source_target.reshape(snapshot.query_shape),
            source_x,
            source_y,
            controller_points,
        )
        normalized = normalize_nonnegative_density(
            projected,
            weights.ravel(),
            mask=None if controller_mask is None else controller_mask.ravel(),
            tolerance=self.normalization_tolerance,
        ).reshape(controller_shape)
        normalized.setflags(write=False)

        # Replace the cache only after all validation, interpolation, and
        # normalization have succeeded.
        self._cached_aerial_target = normalized
        self._cached_aerial_target_version = snapshot.version
        self._cached_target_x = x_coordinates
        self._cached_target_y = y_coordinates
        self._cached_target_weights = weights
        self._cached_target_mask = controller_mask
        return self._cached_aerial_target

    def _target_cache_matches(
        self,
        version: int,
        map_x: np.ndarray,
        map_y: np.ndarray,
        weights: np.ndarray,
        mask: np.ndarray | None,
    ) -> bool:
        if self._cached_aerial_target is None:
            return False
        if version != self._cached_aerial_target_version:
            return False
        if not (
            np.array_equal(map_x, self._cached_target_x)
            and np.array_equal(map_y, self._cached_target_y)
            and np.array_equal(weights, self._cached_target_weights)
        ):
            return False
        if mask is None:
            return self._cached_target_mask is None
        return np.array_equal(mask, self._cached_target_mask)

    def ground_density(
        self,
        query_points: ArrayLike,
        integration_weights: ArrayLike | None = None,
    ) -> np.ndarray | None:
        """Return the cached normalized high density on controller points.

        Posterior uncertainty is intentionally not an active ground-objective
        term in this milestone.
        """
        snapshot = self.latest_posterior
        if snapshot is None:
            return None
        points = _query_points(query_points)
        if integration_weights is None:
            weights = (
                snapshot.integration_weights
                if np.array_equal(points, snapshot.query_points)
                else np.ones(points.shape[0], dtype=float)
            )
        else:
            weights = _ground_integration_weights(
                integration_weights, points.shape[0]
            )
        if (
            self._cached_ground_density is not None
            and snapshot.version == self._cached_ground_density_version
            and np.array_equal(points, self._cached_ground_points)
            and np.array_equal(weights, self._cached_ground_weights)
        ):
            return self._cached_ground_density

        exact_snapshot_grid = np.array_equal(points, snapshot.query_points)
        exact_snapshot_weights = np.array_equal(
            weights, snapshot.integration_weights
        )
        if exact_snapshot_grid:
            projected = snapshot.density
        else:
            source_x, source_y = _structured_snapshot_axes(snapshot)
            projected = resample_structured_grid(
                snapshot.density.reshape(snapshot.query_shape),
                source_x,
                source_y,
                points,
            )
        if exact_snapshot_grid and exact_snapshot_weights:
            density = snapshot.density
        else:
            density = normalize_nonnegative_density(
                projected,
                weights,
                tolerance=self.normalization_tolerance,
            )
            density.setflags(write=False)

        # Replace only after query, interpolation, and normalization validation.
        self._cached_ground_density = density
        self._cached_ground_density_version = snapshot.version
        self._cached_ground_points = points
        self._cached_ground_weights = np.array(weights, dtype=float, copy=True)
        return self._cached_ground_density

    def collect_low_if_due(
        self, simulation_time: float, aerial_team: Any
    ) -> CoordinatorEventReport:
        return self._collect_if_due(
            simulation_time,
            aerial_team,
            self.low_event,
            self.low_sensor,
            self.fields.low,
            CoordinatorEventType.LOW_COLLECTION,
        )

    def collect_high_if_due(
        self, simulation_time: float, ground_team: Any
    ) -> CoordinatorEventReport:
        return self._collect_if_due(
            simulation_time,
            ground_team,
            self.high_event,
            self.high_sensor,
            self.fields.high,
            CoordinatorEventType.HIGH_COLLECTION,
        )

    def update_if_due(self, simulation_time: float) -> CoordinatorEventReport:
        timestamp = _finite_float(simulation_time, "simulation_time")
        if not self.update_event.is_due(timestamp):
            return CoordinatorEventReport(
                CoordinatorEventType.ESTIMATOR_UPDATE,
                timestamp,
                False,
                0,
                None,
                self.estimator.version,
            )
        report = self.estimator.update(timestamp)
        self.update_event.mark_fired()
        return CoordinatorEventReport(
            CoordinatorEventType.ESTIMATOR_UPDATE,
            timestamp,
            True,
            0,
            report,
            self.estimator.version,
        )

    def _collect_if_due(
        self,
        simulation_time: float,
        team: Any,
        event: PeriodicEvent,
        sensor: SimulatedScalarFieldSensor,
        field: np.ndarray,
        event_type: CoordinatorEventType,
    ) -> CoordinatorEventReport:
        timestamp = _finite_float(simulation_time, "simulation_time")
        if not event.is_due(timestamp):
            return CoordinatorEventReport(
                event_type, timestamp, False, 0, None, self.estimator.version
            )
        observations = sensor.collect(team, field, timestamp)
        submitted_count = self.estimator.submit_many(observations)
        event.mark_fired()
        return CoordinatorEventReport(
            event_type,
            timestamp,
            True,
            submitted_count,
            None,
            self.estimator.version,
        )


def build_multifidelity_coordinator(
    params: Any,
    ground_params: Any,
    high_field: ArrayLike,
    query_points: ArrayLike,
    query_shape: tuple[int, ...],
    integration_weights: ArrayLike,
    mask: ArrayLike | None = None,
    *,
    seed: int | None = None,
) -> MultifidelitySimulationCoordinator:
    """Map reviewed configuration into one multi-fidelity coordinator."""
    rho = float(_get(params, "multifidelity.rho", 0.8))
    raw_high = np.asarray(high_field, dtype=float)
    field_mask = _compatible_boolean_mask(mask, raw_high.shape)
    fields = build_fidelity_fields(
        raw_high,
        rho,
        float(
            _get(
                params,
                "multifidelity.sensor.low_fidelity_smoothing_sigma_cells",
                2.0,
            )
        ),
        field_mask,
    )
    query_array = np.asarray(query_points, dtype=float)
    query_mask = _compatible_boolean_mask(mask, (query_array.shape[0],))
    gp = MultiFidelityGaussianProcess(
        rho=rho,
        low_kernel=RBFKernel(
            length_scale=float(
                _get(params, "multifidelity.low_kernel.length_scale", 5.0)
            ),
            variance=float(_get(params, "multifidelity.low_kernel.variance", 1.0)),
        ),
        discrepancy_kernel=RBFKernel(
            length_scale=float(
                _get(params, "multifidelity.discrepancy_kernel.length_scale", 1.5)
            ),
            variance=float(
                _get(params, "multifidelity.discrepancy_kernel.variance", 0.25)
            ),
        ),
        jitter=float(_get(params, "multifidelity.jitter", 1.0e-8)),
        max_jitter_attempts=int(
            _get(params, "multifidelity.max_jitter_attempts", 5)
        ),
        jitter_multiplier=float(
            _get(params, "multifidelity.jitter_multiplier", 10.0)
        ),
    )
    settings = EstimatorSettings(
        gp=gp,
        low_retention=RetentionConfig(
            max_samples=int(
                _get(params, "multifidelity.retention.max_low_samples", 250)
            ),
            min_separation=float(
                _get(params, "multifidelity.retention.min_low_separation", 0.25)
            ),
        ),
        high_retention=RetentionConfig(
            max_samples=int(
                _get(params, "multifidelity.retention.max_high_samples", 250)
            ),
            min_separation=float(
                _get(params, "multifidelity.retention.min_high_separation", 0.1)
            ),
        ),
        query_points=query_array,
        query_shape=query_shape,
        integration_weights=integration_weights,
        mask=query_mask,
        normalization_tolerance=float(
            _get(params, "multifidelity.density.normalization_tolerance", 1.0e-8)
        ),
        hyperparameter_optimization=HyperparameterOptimizationSettings(
            enabled=_get(
                params,
                "multifidelity.hyperparameter_optimization.enabled",
                False,
            ),
            fit_interval_updates=int(
                _get(
                    params,
                    "multifidelity.hyperparameter_optimization.fit_interval_updates",
                    5,
                )
            ),
            min_samples=int(
                _get(
                    params,
                    "multifidelity.hyperparameter_optimization.min_samples",
                    20,
                )
            ),
            num_restarts=int(
                _get(
                    params,
                    "multifidelity.hyperparameter_optimization.num_restarts",
                    0,
                )
            ),
            max_iterations=int(
                _get(
                    params,
                    "multifidelity.hyperparameter_optimization.max_iterations",
                    100,
                )
            ),
            bounds=KernelHyperparameterBounds(
                low_length_scale=_get(
                    params,
                    "multifidelity.hyperparameter_optimization.bounds.low_length_scale",
                    (0.1, 100.0),
                ),
                low_variance=_get(
                    params,
                    "multifidelity.hyperparameter_optimization.bounds.low_variance",
                    (1.0e-4, 100.0),
                ),
                discrepancy_length_scale=_get(
                    params,
                    "multifidelity.hyperparameter_optimization.bounds.discrepancy_length_scale",
                    (0.1, 100.0),
                ),
                discrepancy_variance=_get(
                    params,
                    "multifidelity.hyperparameter_optimization.bounds.discrepancy_variance",
                    (1.0e-4, 100.0),
                ),
            ),
        ),
    )
    if seed is None:
        resolved_seed = int(_get(params, "simulation.random_seed", 42))
    elif isinstance(seed, bool) or not isinstance(seed, int):
        raise TypeError("seed must be an integer or None")
    else:
        resolved_seed = seed
    offset = int(_get(params, "multifidelity.sensor.random_seed_offset", 10000))
    low_sequence, high_sequence = np.random.SeedSequence(
        [resolved_seed, offset]
    ).spawn(2)
    low_sensor = SimulatedScalarFieldSensor(
        Fidelity.LOW,
        sample_count=int(_get(params, "gpr.obs_per_step", 10)),
        sample_range=float(_get(params, "sensor.fov_depth", 5.0)),
        fov_degrees=float(_get(params, "sensor.fov_degrees", 360.0)),
        noise_variance=float(
            _get(params, "multifidelity.low_noise_variance", 0.04)
        ),
        rng=np.random.default_rng(low_sequence),
    )
    high_sensor = SimulatedScalarFieldSensor(
        Fidelity.HIGH,
        sample_count=int(_get(ground_params, "gpr.obs_per_step", 10)),
        sample_range=float(_get(ground_params, "sensor.fov_depth", 5.0)),
        fov_degrees=float(_get(ground_params, "sensor.fov_degrees", 90.0)),
        noise_variance=float(
            _get(params, "multifidelity.high_noise_variance", 1.0e-4)
        ),
        rng=np.random.default_rng(high_sequence),
    )
    return MultifidelitySimulationCoordinator(
        estimator=CentralAsynchronousEstimator(settings),
        low_sensor=low_sensor,
        high_sensor=high_sensor,
        fields=fields,
        low_event=PeriodicEvent(
            float(_get(params, "multifidelity.aerial_sensor_period", 0.5))
        ),
        high_event=PeriodicEvent(
            float(_get(params, "multifidelity.ground_sensor_period", 0.1))
        ),
        update_event=PeriodicEvent(
            float(_get(params, "multifidelity.gp_update_period", 1.0))
        ),
        lambda_interest=float(
            _get(params, "multifidelity.aerial_target.lambda_interest", 1.0)
        ),
        lambda_uncertainty=float(
            _get(params, "multifidelity.aerial_target.lambda_uncertainty", 0.25)
        ),
        normalization_tolerance=float(
            _get(params, "multifidelity.density.normalization_tolerance", 1.0e-8)
        ),
    )


def _structured_snapshot_axes(
    snapshot: PosteriorSnapshot,
) -> tuple[np.ndarray, np.ndarray]:
    if len(snapshot.query_shape) != 2:
        raise ValueError("aerial target requires a two-dimensional posterior grid")
    points = snapshot.query_points.reshape((*snapshot.query_shape, 2))
    source_x = np.array(points[0, :, 0], dtype=float, copy=True)
    source_y = np.array(points[:, 0, 1], dtype=float, copy=True)
    expected_x, expected_y = np.meshgrid(source_x, source_y)
    if not (
        np.allclose(points[..., 0], expected_x, rtol=0.0, atol=1.0e-12)
        and np.allclose(points[..., 1], expected_y, rtol=0.0, atol=1.0e-12)
    ):
        raise ValueError("posterior query points must form a structured (x, y) grid")
    return _strict_axis(source_x, "posterior source_x"), _strict_axis(
        source_y, "posterior source_y"
    )


def build_ground_weight_vectors(
    density: ArrayLike, voronoi_masks: ArrayLike
) -> tuple[np.ndarray, ...]:
    """Validate and multiply one ground density by per-robot Voronoi masks."""
    try:
        density_array = np.asarray(density, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError("density must be a numeric array") from exc
    if density_array.ndim != 1 or density_array.size == 0:
        raise ValueError("density must be a nonempty one-dimensional array")
    if not np.all(np.isfinite(density_array)):
        raise ValueError("density must contain only finite values")
    if np.any(density_array < 0.0):
        raise ValueError("density must be nonnegative")
    try:
        masks = np.asarray(voronoi_masks, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError("voronoi_masks must be a numeric array") from exc
    if masks.ndim != 2 or masks.shape[1:] != (density_array.size,):
        raise ValueError(
            "voronoi_masks must have shape (num_robots, density_size)"
        )
    if not np.all(np.isfinite(masks)):
        raise ValueError("voronoi_masks must contain only finite values")
    if np.any(masks < 0.0):
        raise ValueError("voronoi_masks must be nonnegative")
    return tuple(
        np.array(density_array * robot_mask, dtype=float, copy=True)
        for robot_mask in masks
    )


def _query_points(values: ArrayLike) -> np.ndarray:
    try:
        points = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError("query_points must be a numeric array") from exc
    if points.ndim != 2 or points.shape[1:] != (2,) or points.shape[0] == 0:
        raise ValueError("query_points must have shape (N, 2) with N > 0")
    if not np.all(np.isfinite(points)):
        raise ValueError("query_points must contain only finite values")
    return np.array(points, dtype=float, copy=True)


def _ground_integration_weights(values: ArrayLike, count: int) -> np.ndarray:
    try:
        weights = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError("integration_weights must be a numeric array") from exc
    if weights.shape != (count,):
        raise ValueError("integration_weights must match the query-point count")
    if not np.all(np.isfinite(weights)):
        raise ValueError("integration_weights must contain only finite values")
    if np.any(weights < 0.0) or not np.any(weights > 0.0):
        raise ValueError(
            "integration_weights must be nonnegative with positive mass"
        )
    return np.array(weights, dtype=float, copy=True)


def _strict_axis(values: ArrayLike, name: str) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if result.ndim != 1 or result.size < 2:
        raise ValueError(f"{name} must be one-dimensional with at least two points")
    if not np.all(np.isfinite(result)) or np.any(np.diff(result) <= 0.0):
        raise ValueError(f"{name} must be finite and strictly increasing")
    return np.array(result, dtype=float, copy=True)


def _controller_array(
    values: ArrayLike,
    name: str,
    shape: tuple[int, int],
    dtype: type,
) -> np.ndarray:
    raw = np.asarray(values)
    if dtype is bool and raw.dtype != np.bool_:
        raise TypeError("mask must be a boolean array")
    try:
        result = np.asarray(values, dtype=dtype)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a numeric array") from exc
    if result.shape != shape and result.shape != (math.prod(shape),):
        raise ValueError(f"{name} must have shape {shape} or {(math.prod(shape),)}")
    result = np.array(result, dtype=dtype, copy=True).reshape(shape)
    if dtype is not bool and not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return result


def _compatible_boolean_mask(
    mask: ArrayLike | None, expected_shape: tuple[int, ...]
) -> np.ndarray | None:
    if mask is None:
        return None
    candidate = np.asarray(mask)
    if candidate.dtype == np.bool_ and candidate.shape == expected_shape:
        return np.array(candidate, dtype=bool, copy=True)
    if candidate.dtype == np.bool_ and candidate.size == math.prod(expected_shape):
        return np.array(candidate, dtype=bool, copy=True).reshape(expected_shape)
    return None


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


def _positive_float(value: float, name: str) -> float:
    result = _finite_float(value, name)
    if result <= 0.0:
        raise ValueError(f"{name} must be positive")
    return result


def _nonnegative_float(value: float, name: str) -> float:
    result = _finite_float(value, name)
    if result < 0.0:
        raise ValueError(f"{name} must be nonnegative")
    return result
