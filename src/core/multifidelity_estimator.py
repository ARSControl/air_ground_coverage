"""Central, simulation-independent asynchronous multi-fidelity estimator."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from time import perf_counter
from typing import Iterable

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .density import normalize_nonnegative_density, positive_part, validate_density
from .multifidelity_gp import (
    KernelHyperparameterBounds,
    MultiFidelityGaussianProcess,
)
from .observations import (
    BoundedObservationBuffer,
    Fidelity,
    Observation,
    RetentionConfig,
)


FloatArray = NDArray[np.float64]
BoolArray = NDArray[np.bool_]


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


def _readonly_float_array(
    values: ArrayLike, name: str, ndim: int | None = None
) -> FloatArray:
    try:
        result = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if ndim is not None and result.ndim != ndim:
        raise ValueError(f"{name} must be {ndim}-dimensional; got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    result = np.array(result, dtype=float, copy=True, order="C")
    result.setflags(write=False)
    return result


def _query_shape(value: tuple[int, ...], count: int) -> tuple[int, ...]:
    if not isinstance(value, tuple) or not value:
        raise TypeError("query_shape must be a nonempty tuple")
    if any(isinstance(item, bool) or not isinstance(item, int) for item in value):
        raise TypeError("query_shape entries must be integers")
    if any(item <= 0 for item in value):
        raise ValueError("query_shape entries must be positive")
    if math.prod(value) != count:
        raise ValueError("query_shape product must equal the query-point count")
    return value


@dataclass(frozen=True)
class HyperparameterOptimizationSettings:
    """Schedule and bounds for optional empirical-Bayes kernel fitting."""

    enabled: bool = False
    fit_interval_updates: int = 5
    min_samples: int = 20
    num_restarts: int = 0
    max_iterations: int = 100
    bounds: KernelHyperparameterBounds = field(
        default_factory=KernelHyperparameterBounds
    )

    def __post_init__(self) -> None:
        if not isinstance(self.enabled, bool):
            raise TypeError("enabled must be a boolean")
        for value, name, minimum in (
            (self.fit_interval_updates, "fit_interval_updates", 1),
            (self.min_samples, "min_samples", 1),
            (self.num_restarts, "num_restarts", 0),
            (self.max_iterations, "max_iterations", 1),
        ):
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"{name} must be an integer")
            if value < minimum:
                raise ValueError(f"{name} must be at least {minimum}")
        if not isinstance(self.bounds, KernelHyperparameterBounds):
            raise TypeError("bounds must be KernelHyperparameterBounds")


@dataclass(frozen=True)
class EstimatorSettings:
    """Validated GP, retention, normalization, and query-grid settings."""

    gp: MultiFidelityGaussianProcess
    low_retention: RetentionConfig
    high_retention: RetentionConfig
    query_points: FloatArray
    query_shape: tuple[int, ...]
    integration_weights: FloatArray
    mask: BoolArray | None = None
    normalization_tolerance: float = 1.0e-8
    hyperparameter_optimization: HyperparameterOptimizationSettings = field(
        default_factory=HyperparameterOptimizationSettings
    )
    publish_low_only_projection: bool = False

    def __post_init__(self) -> None:
        if not isinstance(self.gp, MultiFidelityGaussianProcess):
            raise TypeError("gp must be a MultiFidelityGaussianProcess")
        if not isinstance(self.low_retention, RetentionConfig):
            raise TypeError("low_retention must be a RetentionConfig")
        if not isinstance(self.high_retention, RetentionConfig):
            raise TypeError("high_retention must be a RetentionConfig")
        if not isinstance(
            self.hyperparameter_optimization, HyperparameterOptimizationSettings
        ):
            raise TypeError(
                "hyperparameter_optimization must be HyperparameterOptimizationSettings"
            )
        if not isinstance(self.publish_low_only_projection, bool):
            raise TypeError("publish_low_only_projection must be a boolean")
        query_points = _readonly_float_array(self.query_points, "query_points", ndim=2)
        if query_points.shape[1:] != (2,):
            raise ValueError("query_points must have shape (N, 2)")
        if query_points.shape[0] == 0:
            raise ValueError("query_points must not be empty")
        query_shape = _query_shape(self.query_shape, query_points.shape[0])
        integration_weights = _readonly_float_array(
            self.integration_weights, "integration_weights", ndim=1
        )
        if integration_weights.shape != (query_points.shape[0],):
            raise ValueError("integration_weights must match query-point count")
        if np.any(integration_weights < 0.0) or not np.any(integration_weights > 0.0):
            raise ValueError(
                "integration_weights must be nonnegative with positive mass"
            )
        if self.mask is None:
            mask = None
        else:
            raw_mask = np.asarray(self.mask)
            if raw_mask.dtype != np.bool_ or raw_mask.shape != (query_points.shape[0],):
                raise ValueError("mask must be boolean and match query-point count")
            mask = np.array(raw_mask, dtype=bool, copy=True)
            mask.setflags(write=False)
            if not np.any(mask & (integration_weights > 0.0)):
                raise ValueError("mask must include at least one positive-weight point")
        tolerance = _finite_float(
            self.normalization_tolerance, "normalization_tolerance"
        )
        if tolerance <= 0.0:
            raise ValueError("normalization_tolerance must be positive")

        object.__setattr__(self, "query_points", query_points)
        object.__setattr__(self, "query_shape", query_shape)
        object.__setattr__(self, "integration_weights", integration_weights)
        object.__setattr__(self, "mask", mask)
        object.__setattr__(self, "normalization_tolerance", tolerance)


class UpdateStatus(Enum):
    """Outcome of an explicit estimator update request."""

    UPDATED = "updated"
    NO_NEW_DATA = "no_new_data"
    FAILED = "failed"


@dataclass(frozen=True)
class PosteriorSnapshot:
    """Immutable, validated high-fidelity posterior published to consumers."""

    timestamp: float
    version: int
    query_points: FloatArray
    query_shape: tuple[int, ...]
    high_mean: FloatArray
    high_variance: FloatArray
    density: FloatArray
    low_sample_count: int
    high_sample_count: int
    is_valid: bool
    status: UpdateStatus
    effective_jitter: float
    fit_duration: float
    prediction_duration: float
    integration_weights: FloatArray
    hyperparameter_fit_performed: bool
    hyperparameter_fit_duration: float
    low_kernel_length_scale: float
    low_kernel_variance: float
    discrepancy_kernel_length_scale: float
    discrepancy_kernel_variance: float
    discrepancy_enabled: bool = True
    low_only_high_mean: FloatArray | None = None
    low_only_high_variance: FloatArray | None = None
    low_only_projection_duration: float = 0.0

    def __post_init__(self) -> None:
        timestamp = _finite_float(self.timestamp, "timestamp")
        if isinstance(self.version, bool) or not isinstance(self.version, int):
            raise TypeError("version must be an integer")
        if self.version < 1:
            raise ValueError("version must be positive")
        query_points = _readonly_float_array(self.query_points, "query_points", ndim=2)
        if query_points.shape[1:] != (2,):
            raise ValueError("query_points must have shape (N, 2)")
        query_shape = _query_shape(self.query_shape, query_points.shape[0])
        mean = _readonly_float_array(self.high_mean, "high_mean", ndim=1)
        variance = _readonly_float_array(self.high_variance, "high_variance", ndim=1)
        density = _readonly_float_array(self.density, "density", ndim=1)
        weights = _readonly_float_array(
            self.integration_weights, "integration_weights", ndim=1
        )
        expected_shape = (query_points.shape[0],)
        if not all(
            array.shape == expected_shape
            for array in (mean, variance, density, weights)
        ):
            raise ValueError("posterior vectors must match query-point count")
        if np.any(variance < 0.0):
            raise ValueError("high_variance must be nonnegative")
        validate_density(density, weights)
        for count, name in (
            (self.low_sample_count, "low_sample_count"),
            (self.high_sample_count, "high_sample_count"),
        ):
            if isinstance(count, bool) or not isinstance(count, int):
                raise TypeError(f"{name} must be an integer")
            if count < 0:
                raise ValueError(f"{name} must be nonnegative")
        if self.is_valid is not True:
            raise ValueError("published posterior snapshots must be valid")
        if self.status is not UpdateStatus.UPDATED:
            raise ValueError("published posterior status must be UPDATED")
        effective_jitter = _finite_float(self.effective_jitter, "effective_jitter")
        fit_duration = _finite_float(self.fit_duration, "fit_duration")
        prediction_duration = _finite_float(
            self.prediction_duration, "prediction_duration"
        )
        optimization_duration = _finite_float(
            self.hyperparameter_fit_duration,
            "hyperparameter_fit_duration",
        )
        if not isinstance(self.hyperparameter_fit_performed, bool):
            raise TypeError("hyperparameter_fit_performed must be a boolean")
        if (
            min(
                effective_jitter,
                fit_duration,
                prediction_duration,
                optimization_duration,
            )
            < 0.0
        ):
            raise ValueError("jitter and durations must be nonnegative")
        low_only_duration = _finite_float(
            self.low_only_projection_duration, "low_only_projection_duration"
        )
        if low_only_duration < 0.0:
            raise ValueError("low_only_projection_duration must be nonnegative")
        if not isinstance(self.discrepancy_enabled, bool):
            raise TypeError("discrepancy_enabled must be a boolean")
        hyperparameters = tuple(
            _finite_float(getattr(self, name), name)
            for name in (
                "low_kernel_length_scale",
                "low_kernel_variance",
                "discrepancy_kernel_length_scale",
                "discrepancy_kernel_variance",
            )
        )
        if min(hyperparameters) <= 0.0:
            raise ValueError("kernel hyperparameters must be positive")

        object.__setattr__(self, "timestamp", timestamp)
        object.__setattr__(self, "query_points", query_points)
        object.__setattr__(self, "query_shape", query_shape)
        object.__setattr__(self, "high_mean", mean)
        object.__setattr__(self, "high_variance", variance)
        object.__setattr__(self, "density", density)
        object.__setattr__(self, "integration_weights", weights)
        object.__setattr__(self, "effective_jitter", effective_jitter)
        object.__setattr__(self, "fit_duration", fit_duration)
        object.__setattr__(self, "prediction_duration", prediction_duration)
        object.__setattr__(self, "hyperparameter_fit_duration", optimization_duration)
        object.__setattr__(self, "low_only_projection_duration", low_only_duration)
        low_only_mean = self.low_only_high_mean
        low_only_variance = self.low_only_high_variance
        if (low_only_mean is None) != (low_only_variance is None):
            raise ValueError(
                "low_only_high_mean and low_only_high_variance must both be set or None"
            )
        if low_only_mean is not None:
            validated_low_mean = _readonly_float_array(
                low_only_mean, "low_only_high_mean", ndim=1
            )
            validated_low_variance = _readonly_float_array(
                low_only_variance, "low_only_high_variance", ndim=1
            )
            if (
                validated_low_mean.shape != expected_shape
                or validated_low_variance.shape != expected_shape
            ):
                raise ValueError("low-only posterior vectors must match query points")
            if np.any(validated_low_variance < 0.0):
                raise ValueError("low_only_high_variance must be nonnegative")
            object.__setattr__(self, "low_only_high_mean", validated_low_mean)
            object.__setattr__(self, "low_only_high_variance", validated_low_variance)


@dataclass(frozen=True)
class UpdateReport:
    """Small immutable report for every explicit update request."""

    status: UpdateStatus
    timestamp: float
    version: int
    message: str
    exception_type: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.status, UpdateStatus):
            raise TypeError("status must be an UpdateStatus")
        timestamp = _finite_float(self.timestamp, "timestamp")
        if isinstance(self.version, bool) or not isinstance(self.version, int):
            raise TypeError("version must be an integer")
        if self.version < 0:
            raise ValueError("version must be nonnegative")
        if not isinstance(self.message, str) or not self.message:
            raise ValueError("message must be a nonempty string")
        if self.exception_type is not None and (
            not isinstance(self.exception_type, str) or not self.exception_type
        ):
            raise ValueError("exception_type must be None or a nonempty string")
        object.__setattr__(self, "timestamp", timestamp)


class CentralAsynchronousEstimator:
    """Explicitly updated owner of LOW/HIGH buffers and one published GP state."""

    def __init__(self, settings: EstimatorSettings) -> None:
        if not isinstance(settings, EstimatorSettings):
            raise TypeError("settings must be EstimatorSettings")
        self.settings = settings
        self._low_buffer = BoundedObservationBuffer(
            Fidelity.LOW, settings.low_retention
        )
        self._high_buffer = BoundedObservationBuffer(
            Fidelity.HIGH, settings.high_retention
        )
        self._gp = settings.gp
        self._latest_posterior: PosteriorSnapshot | None = None
        self._version = 0

    @property
    def latest_posterior(self) -> PosteriorSnapshot | None:
        return self._latest_posterior

    @property
    def low_sample_count(self) -> int:
        return len(self._low_buffer.retained)

    @property
    def high_sample_count(self) -> int:
        return len(self._high_buffer.retained)

    @property
    def pending_count(self) -> int:
        return self._low_buffer.pending_count + self._high_buffer.pending_count

    @property
    def version(self) -> int:
        return self._version

    def submit(self, observation: Observation) -> None:
        if not isinstance(observation, Observation):
            raise TypeError("observation must be an Observation")
        if observation.fidelity is Fidelity.LOW:
            self._low_buffer.submit(observation)
        else:
            self._high_buffer.submit(observation)

    def submit_many(self, observations: Iterable[Observation]) -> int:
        candidate = tuple(observations)
        if not all(isinstance(item, Observation) for item in candidate):
            raise TypeError("observations must contain only Observation values")
        for observation in candidate:
            self.submit(observation)
        return len(candidate)

    def update(self, simulation_time: float) -> UpdateReport:
        timestamp = _finite_float(simulation_time, "simulation_time")
        if self.pending_count == 0:
            return UpdateReport(
                status=UpdateStatus.NO_NEW_DATA,
                timestamp=timestamp,
                version=self._version,
                message="no pending observations",
            )

        low_candidate = self._low_buffer.candidate()
        high_candidate = self._high_buffer.candidate()
        try:
            candidate_gp = self._new_candidate_gp()
            low_positions, low_values, low_noise = _observation_arrays(low_candidate)
            high_positions, high_values, high_noise = _observation_arrays(
                high_candidate
            )

            optimization_settings = self.settings.hyperparameter_optimization
            fit_hyperparameters = (
                optimization_settings.enabled
                and len(low_candidate) + len(high_candidate)
                >= optimization_settings.min_samples
                and self._version % optimization_settings.fit_interval_updates == 0
            )
            optimization_duration = 0.0
            if fit_hyperparameters:
                optimization_start = perf_counter()
                candidate_gp.fit_hyperparameters(
                    low_positions,
                    low_values,
                    low_noise,
                    high_positions,
                    high_values,
                    high_noise,
                    bounds=optimization_settings.bounds,
                    num_restarts=optimization_settings.num_restarts,
                    max_iterations=optimization_settings.max_iterations,
                )
                optimization_duration = perf_counter() - optimization_start

            fit_start = perf_counter()
            candidate_gp.fit(
                low_positions,
                low_values,
                low_noise,
                high_positions,
                high_values,
                high_noise,
            )
            fit_duration = perf_counter() - fit_start

            prediction_start = perf_counter()
            prediction = candidate_gp.predict_high(self.settings.query_points)
            density = normalize_nonnegative_density(
                positive_part(prediction.mean),
                self.settings.integration_weights,
                mask=self.settings.mask,
                tolerance=self.settings.normalization_tolerance,
            )
            prediction_duration = perf_counter() - prediction_start
            validate_density(
                density,
                self.settings.integration_weights,
                self.settings.normalization_tolerance,
            )

            low_only_mean = None
            low_only_variance = None
            low_only_projection_duration = 0.0
            if self.settings.publish_low_only_projection:
                low_only_start = perf_counter()
                low_only_gp = type(candidate_gp)(
                    rho=candidate_gp.rho,
                    low_kernel=candidate_gp.low_kernel,
                    discrepancy_kernel=candidate_gp.discrepancy_kernel,
                    discrepancy_enabled=candidate_gp.discrepancy_enabled,
                    jitter=candidate_gp.jitter,
                    max_jitter_attempts=candidate_gp.max_jitter_attempts,
                    jitter_multiplier=candidate_gp.jitter_multiplier,
                )
                low_only_gp.fit(
                    low_positions,
                    low_values,
                    low_noise,
                    np.empty((0, 2), dtype=float),
                    np.empty(0, dtype=float),
                    np.empty(0, dtype=float),
                )
                low_only_prediction = low_only_gp.predict_high(
                    self.settings.query_points
                )
                low_only_mean = low_only_prediction.mean
                low_only_variance = low_only_prediction.variance
                low_only_projection_duration = perf_counter() - low_only_start

            next_version = self._version + 1
            snapshot = PosteriorSnapshot(
                timestamp=timestamp,
                version=next_version,
                query_points=self.settings.query_points,
                query_shape=self.settings.query_shape,
                high_mean=prediction.mean,
                high_variance=prediction.variance,
                density=density,
                low_sample_count=len(low_candidate),
                high_sample_count=len(high_candidate),
                is_valid=True,
                status=UpdateStatus.UPDATED,
                effective_jitter=candidate_gp.effective_jitter,
                fit_duration=fit_duration,
                prediction_duration=prediction_duration,
                integration_weights=self.settings.integration_weights,
                hyperparameter_fit_performed=fit_hyperparameters,
                hyperparameter_fit_duration=optimization_duration,
                low_kernel_length_scale=candidate_gp.low_kernel.length_scale,
                low_kernel_variance=candidate_gp.low_kernel.variance,
                discrepancy_kernel_length_scale=(
                    candidate_gp.discrepancy_kernel.length_scale
                ),
                discrepancy_kernel_variance=(candidate_gp.discrepancy_kernel.variance),
                discrepancy_enabled=candidate_gp.discrepancy_enabled,
                low_only_high_mean=low_only_mean,
                low_only_high_variance=low_only_variance,
                low_only_projection_duration=low_only_projection_duration,
            )

            self._low_buffer.commit(low_candidate)
            self._high_buffer.commit(high_candidate)
            self._gp = candidate_gp
            self._version = next_version
            self._latest_posterior = snapshot
            return UpdateReport(
                status=UpdateStatus.UPDATED,
                timestamp=timestamp,
                version=next_version,
                message=(
                    f"published posterior with {len(low_candidate)} LOW and "
                    f"{len(high_candidate)} HIGH samples; "
                    f"hyperparameters={'fitted' if fit_hyperparameters else 'reused'}"
                ),
            )
        except Exception as exc:
            message = str(exc).strip() or "estimator update failed"
            return UpdateReport(
                status=UpdateStatus.FAILED,
                timestamp=timestamp,
                version=self._version,
                message=message[:240],
                exception_type=type(exc).__name__,
            )

    def _new_candidate_gp(self) -> MultiFidelityGaussianProcess:
        template = self._gp
        return type(template)(
            rho=template.rho,
            low_kernel=template.low_kernel,
            discrepancy_kernel=template.discrepancy_kernel,
            discrepancy_enabled=template.discrepancy_enabled,
            jitter=template.jitter,
            max_jitter_attempts=template.max_jitter_attempts,
            jitter_multiplier=template.jitter_multiplier,
        )


def _observation_arrays(
    observations: tuple[Observation, ...],
) -> tuple[FloatArray, FloatArray, FloatArray]:
    if not observations:
        return (
            np.empty((0, 2), dtype=float),
            np.empty(0, dtype=float),
            np.empty(0, dtype=float),
        )
    return (
        np.asarray([observation.position for observation in observations], dtype=float),
        np.asarray([observation.value for observation in observations], dtype=float),
        np.asarray(
            [observation.noise_variance for observation in observations],
            dtype=float,
        ),
    )
