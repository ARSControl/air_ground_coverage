"""Pure autoregressive multi-fidelity Gaussian-process mathematics.

The model implemented here is

    f_H(q) = rho * f_L(q) + delta(q),

where ``f_L`` and ``delta`` are independent zero-mean Gaussian processes.  This
module deliberately contains no simulation, robot, controller, or plotting
dependencies.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.linalg import solve_triangular
from scipy.optimize import minimize


FloatArray = NDArray[np.float64]


def _finite_scalar(value: float, name: str) -> float:
    """Return ``value`` as a finite float or raise a descriptive error."""
    if isinstance(value, (bool, np.bool_)):
        raise TypeError(f"{name} must be a real scalar, not a boolean")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must be a real scalar") from exc
    if not np.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _positions(values: ArrayLike, name: str) -> FloatArray:
    """Validate and copy an ``(N, 2)`` array of Cartesian positions."""
    try:
        result = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be convertible to a numeric array") from exc
    if result.ndim != 2 or result.shape[1:] != (2,):
        raise ValueError(f"{name} must have shape (N, 2); got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return np.array(result, dtype=float, copy=True, order="C")


def _values(values: ArrayLike, size: int, name: str) -> FloatArray:
    """Validate and copy a one-dimensional observation vector."""
    try:
        result = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be convertible to a numeric array") from exc
    if result.ndim != 1 or result.shape != (size,):
        raise ValueError(f"{name} must have shape ({size},); got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return np.array(result, dtype=float, copy=True, order="C")


def _noise_variances(values: ArrayLike, size: int, name: str) -> FloatArray:
    """Validate scalar or per-observation nonnegative noise variances."""
    try:
        result = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc

    if result.ndim == 0:
        scalar = _finite_scalar(result.item(), name)
        result = np.full(size, scalar, dtype=float)
    elif result.ndim == 1 and result.shape == (size,):
        result = np.array(result, dtype=float, copy=True, order="C")
    else:
        raise ValueError(
            f"{name} must be a scalar or have shape ({size},); got {result.shape}"
        )

    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    if np.any(result < 0.0):
        raise ValueError(f"{name} must contain nonnegative variances")
    return result


def _readonly_vector(values: ArrayLike, name: str) -> FloatArray:
    """Create a finite, immutable one-dimensional floating-point array."""
    result = np.asarray(values, dtype=float)
    if result.ndim != 1:
        raise ValueError(f"{name} must be one-dimensional; got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    result = np.array(result, dtype=float, copy=True, order="C")
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class RBFKernel:
    """Isotropic squared-exponential kernel configuration.

    ``variance`` is the signal variance (the covariance at zero distance), not
    a signal standard deviation.
    """

    length_scale: float
    variance: float

    def __post_init__(self) -> None:
        length_scale = _finite_scalar(self.length_scale, "length_scale")
        variance = _finite_scalar(self.variance, "variance")
        if length_scale <= 0.0:
            raise ValueError("length_scale must be positive")
        if variance <= 0.0:
            raise ValueError("variance must be positive")
        object.__setattr__(self, "length_scale", length_scale)
        object.__setattr__(self, "variance", variance)

    def covariance(self, x_left: ArrayLike, x_right: ArrayLike) -> FloatArray:
        """Return the covariance matrix between two ``(N, 2)`` point sets."""
        left = _positions(x_left, "x_left")
        right = _positions(x_right, "x_right")
        with np.errstate(over="ignore", invalid="ignore"):
            differences = left[:, None, :] - right[None, :, :]
            squared_distances = np.einsum("ijk,ijk->ij", differences, differences)
            scaled = squared_distances / (self.length_scale * self.length_scale)
            covariance = self.variance * np.exp(-0.5 * scaled)
        if not np.all(np.isfinite(covariance)):
            raise FloatingPointError("RBF covariance produced nonfinite values")
        return covariance

    def diagonal(self, x: ArrayLike) -> FloatArray:
        """Return the kernel's prior marginal variance at each position."""
        positions = _positions(x, "x")
        return np.full(positions.shape[0], self.variance, dtype=float)


def _positive_bounds(values: tuple[float, float], name: str) -> tuple[float, float]:
    if not isinstance(values, (tuple, list)) or len(values) != 2:
        raise TypeError(f"{name} must contain exactly two values")
    lower = _finite_scalar(values[0], f"{name}[0]")
    upper = _finite_scalar(values[1], f"{name}[1]")
    if lower <= 0.0 or upper <= lower:
        raise ValueError(f"{name} must be positive and strictly increasing")
    return lower, upper


@dataclass(frozen=True)
class KernelHyperparameterBounds:
    """Positive bounds for the four fitted RBF kernel parameters."""

    low_length_scale: tuple[float, float] = (0.1, 100.0)
    low_variance: tuple[float, float] = (1.0e-4, 100.0)
    discrepancy_length_scale: tuple[float, float] = (0.1, 100.0)
    discrepancy_variance: tuple[float, float] = (1.0e-4, 100.0)

    def __post_init__(self) -> None:
        for name in (
            "low_length_scale",
            "low_variance",
            "discrepancy_length_scale",
            "discrepancy_variance",
        ):
            object.__setattr__(self, name, _positive_bounds(getattr(self, name), name))

    def as_tuple(self) -> tuple[tuple[float, float], ...]:
        return (
            self.low_length_scale,
            self.low_variance,
            self.discrepancy_length_scale,
            self.discrepancy_variance,
        )


@dataclass(frozen=True)
class HyperparameterFitResult:
    """Diagnostics from one bounded marginal-likelihood optimization."""

    initial_negative_log_likelihood: float
    final_negative_log_likelihood: float
    iterations: int
    evaluations: int
    converged: bool

    @property
    def improved(self) -> bool:
        return self.final_negative_log_likelihood < self.initial_negative_log_likelihood


@dataclass(frozen=True)
class HighFidelityPrediction:
    """Immutable latent high-fidelity posterior marginals."""

    mean: FloatArray
    variance: FloatArray

    def __post_init__(self) -> None:
        mean = _readonly_vector(self.mean, "mean")
        variance = _readonly_vector(self.variance, "variance")
        if mean.shape != variance.shape:
            raise ValueError("mean and variance must have the same shape")
        if np.any(variance < 0.0):
            raise ValueError("variance must be nonnegative")
        object.__setattr__(self, "mean", mean)
        object.__setattr__(self, "variance", variance)


class LowFidelityPrediction(HighFidelityPrediction):
    """Immutable latent LOW posterior marginals with the same array contract."""


@dataclass(frozen=True)
class JointLatentPrediction:
    """Immutable mixed LOW/HIGH latent posterior mean and full covariance."""

    mean: FloatArray
    covariance: FloatArray

    def __post_init__(self):
        mean = _readonly_vector(self.mean, "mean")
        covariance = np.array(self.covariance, dtype=float, copy=True)
        if (
            covariance.shape != (len(mean), len(mean))
            or not np.all(np.isfinite(covariance))
            or not np.allclose(covariance, covariance.T, atol=1e-10)
            or np.any(np.diag(covariance) < 0)
        ):
            raise ValueError("invalid joint latent covariance")
        covariance.setflags(write=False)
        object.__setattr__(self, "mean", mean)
        object.__setattr__(self, "covariance", covariance)


@dataclass(frozen=True)
class ReplacementScores:
    """Exact weighted HIGH-variance improvement for each old/new swap."""

    improvement: FloatArray
    objective_before: float

    def __post_init__(self):
        values = np.array(self.improvement, dtype=float, copy=True)
        if values.ndim != 2 or not np.all(np.isfinite(values)):
            raise ValueError("invalid replacement improvements")
        objective = _finite_scalar(self.objective_before, "objective_before")
        if objective < 0:
            raise ValueError("negative variance objective")
        values.setflags(write=False)
        object.__setattr__(self, "improvement", values)
        object.__setattr__(self, "objective_before", objective)


class MultiFidelityGaussianProcess:
    """Zero-mean autoregressive GP with fixed or empirically fitted parameters.

    A newly constructed object represents the explicit empty-data prior.
    ``fit`` builds all candidate state locally and commits it only after a
    successful Cholesky factorization and solve, preserving the last good state
    if validation or numerical factorization fails.
    """

    def __init__(
        self,
        rho: float,
        low_kernel: RBFKernel,
        discrepancy_kernel: RBFKernel,
        discrepancy_enabled: bool = True,
        jitter: float = 1.0e-8,
        max_jitter_attempts: int = 5,
        jitter_multiplier: float = 10.0,
    ) -> None:
        self.rho = _finite_scalar(rho, "rho")
        if not isinstance(low_kernel, RBFKernel):
            raise TypeError("low_kernel must be an RBFKernel")
        if not isinstance(discrepancy_kernel, RBFKernel):
            raise TypeError("discrepancy_kernel must be an RBFKernel")
        if not isinstance(discrepancy_enabled, bool):
            raise TypeError("discrepancy_enabled must be a boolean")
        self.low_kernel = low_kernel
        self.discrepancy_kernel = discrepancy_kernel
        self.discrepancy_enabled = discrepancy_enabled

        self.jitter = _finite_scalar(jitter, "jitter")
        if self.jitter <= 0.0:
            raise ValueError("jitter must be positive")
        if isinstance(max_jitter_attempts, (bool, np.bool_)) or not isinstance(
            max_jitter_attempts, (int, np.integer)
        ):
            raise TypeError("max_jitter_attempts must be an integer")
        if max_jitter_attempts < 1:
            raise ValueError("max_jitter_attempts must be at least 1")
        self.max_jitter_attempts = int(max_jitter_attempts)
        self.jitter_multiplier = _finite_scalar(jitter_multiplier, "jitter_multiplier")
        if self.jitter_multiplier <= 1.0:
            raise ValueError("jitter_multiplier must be greater than 1")

        self._low_positions = np.empty((0, 2), dtype=float)
        self._high_positions = np.empty((0, 2), dtype=float)
        self._cholesky: FloatArray | None = None
        self._alpha = np.empty(0, dtype=float)
        self._effective_jitter = 0.0
        self._replacement_cache = None

    @property
    def kernel_hyperparameters(self) -> tuple[float, float, float, float]:
        """Return LOW length/variance then discrepancy length/variance."""
        return (
            self.low_kernel.length_scale,
            self.low_kernel.variance,
            self.discrepancy_kernel.length_scale,
            self.discrepancy_kernel.variance,
        )

    @property
    def n_low(self) -> int:
        """Number of low-fidelity observations in the current fitted state."""
        return self._low_positions.shape[0]

    @property
    def n_high(self) -> int:
        """Number of high-fidelity observations in the current fitted state."""
        return self._high_positions.shape[0]

    @property
    def effective_jitter(self) -> float:
        """Diagonal jitter used by the last successful nonempty fit."""
        return self._effective_jitter

    def fit(
        self,
        low_positions: ArrayLike,
        low_values: ArrayLike,
        low_noise_variances: ArrayLike,
        high_positions: ArrayLike,
        high_values: ArrayLike,
        high_noise_variances: ArrayLike,
    ) -> MultiFidelityGaussianProcess:
        """Fit the joint LOW/HIGH covariance and return ``self``."""
        candidate_low_positions = _positions(low_positions, "low_positions")
        candidate_high_positions = _positions(high_positions, "high_positions")
        n_low = candidate_low_positions.shape[0]
        n_high = candidate_high_positions.shape[0]
        candidate_low_values = _values(low_values, n_low, "low_values")
        candidate_high_values = _values(high_values, n_high, "high_values")
        candidate_low_noise = _noise_variances(
            low_noise_variances, n_low, "low_noise_variances"
        )
        candidate_high_noise = _noise_variances(
            high_noise_variances, n_high, "high_noise_variances"
        )

        observation_count = n_low + n_high
        if observation_count == 0:
            candidate_cholesky = None
            candidate_alpha = np.empty(0, dtype=float)
            candidate_effective_jitter = 0.0
        else:
            covariance = self._training_covariance(
                candidate_low_positions,
                candidate_low_noise,
                candidate_high_positions,
                candidate_high_noise,
            )
            candidate_cholesky, candidate_effective_jitter = self._factorize(covariance)
            observations = np.concatenate((candidate_low_values, candidate_high_values))
            intermediate = solve_triangular(
                candidate_cholesky, observations, lower=True, check_finite=False
            )
            candidate_alpha = solve_triangular(
                candidate_cholesky.T,
                intermediate,
                lower=False,
                check_finite=False,
            )
            if not np.all(np.isfinite(candidate_alpha)):
                raise FloatingPointError("GP solve produced nonfinite coefficients")

        self._low_positions = candidate_low_positions
        self._high_positions = candidate_high_positions
        self._cholesky = candidate_cholesky
        self._alpha = candidate_alpha
        self._effective_jitter = candidate_effective_jitter
        self._replacement_cache = None
        return self

    def fit_hyperparameters(
        self,
        low_positions: ArrayLike,
        low_values: ArrayLike,
        low_noise_variances: ArrayLike,
        high_positions: ArrayLike,
        high_values: ArrayLike,
        high_noise_variances: ArrayLike,
        *,
        bounds: KernelHyperparameterBounds,
        num_restarts: int = 0,
        max_iterations: int = 100,
    ) -> HyperparameterFitResult:
        """Fit kernel parameters by bounded log-space marginal likelihood."""
        if not self.discrepancy_enabled:
            raise ValueError(
                "hyperparameter fitting is unavailable when discrepancy is disabled"
            )
        if not isinstance(bounds, KernelHyperparameterBounds):
            raise TypeError("bounds must be KernelHyperparameterBounds")
        for value, name in (
            (num_restarts, "num_restarts"),
            (max_iterations, "max_iterations"),
        ):
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"{name} must be an integer")
        if num_restarts < 0:
            raise ValueError("num_restarts must be nonnegative")
        if max_iterations < 1:
            raise ValueError("max_iterations must be positive")

        low_x = _positions(low_positions, "low_positions")
        high_x = _positions(high_positions, "high_positions")
        low_y = _values(low_values, low_x.shape[0], "low_values")
        high_y = _values(high_values, high_x.shape[0], "high_values")
        low_noise = _noise_variances(
            low_noise_variances, low_x.shape[0], "low_noise_variances"
        )
        high_noise = _noise_variances(
            high_noise_variances, high_x.shape[0], "high_noise_variances"
        )
        if low_x.shape[0] + high_x.shape[0] == 0:
            raise ValueError("hyperparameter fitting requires observations")
        observations = np.concatenate((low_y, high_y))
        log_bounds = tuple(
            (np.log(lower), np.log(upper)) for lower, upper in bounds.as_tuple()
        )
        initial = np.log(
            np.clip(
                np.asarray(self.kernel_hyperparameters),
                [item[0] for item in bounds.as_tuple()],
                [item[1] for item in bounds.as_tuple()],
            )
        )

        def objective(log_parameters: np.ndarray) -> float:
            parameters = np.exp(log_parameters)
            trial = type(self)(
                rho=self.rho,
                low_kernel=RBFKernel(parameters[0], parameters[1]),
                discrepancy_kernel=RBFKernel(parameters[2], parameters[3]),
                discrepancy_enabled=self.discrepancy_enabled,
                jitter=self.jitter,
                max_jitter_attempts=self.max_jitter_attempts,
                jitter_multiplier=self.jitter_multiplier,
            )
            try:
                covariance = trial._training_covariance(
                    low_x, low_noise, high_x, high_noise
                )
                factor, _ = trial._factorize(covariance)
                intermediate = solve_triangular(
                    factor, observations, lower=True, check_finite=False
                )
                alpha = solve_triangular(
                    factor.T, intermediate, lower=False, check_finite=False
                )
                value = (
                    0.5 * float(observations @ alpha)
                    + float(np.sum(np.log(np.diag(factor))))
                    + 0.5 * observations.size * np.log(2.0 * np.pi)
                )
            except (FloatingPointError, np.linalg.LinAlgError, ValueError):
                return float(np.finfo(float).max)
            return value if np.isfinite(value) else float(np.finfo(float).max)

        initial_objective = objective(initial)
        if initial_objective >= np.finfo(float).max:
            raise FloatingPointError("initial marginal likelihood is nonfinite")
        starts = [initial]
        restart_fractions = np.array([0.2113, 0.4177, 0.6180, 0.8319])
        lower_logs = np.asarray([item[0] for item in log_bounds])
        upper_logs = np.asarray([item[1] for item in log_bounds])
        for restart in range(num_restarts):
            fractions = np.mod((restart + 1) * restart_fractions, 1.0)
            starts.append(lower_logs + fractions * (upper_logs - lower_logs))

        best_parameters = initial
        best_objective = initial_objective
        best_iterations = 0
        total_evaluations = 1
        converged = False
        for start in starts:
            result = minimize(
                objective,
                start,
                method="L-BFGS-B",
                bounds=log_bounds,
                options={"maxiter": max_iterations, "ftol": 1.0e-9},
            )
            total_evaluations += int(result.nfev)
            if np.isfinite(result.fun) and float(result.fun) < best_objective:
                best_parameters = np.asarray(result.x, dtype=float)
                best_objective = float(result.fun)
                best_iterations = int(result.nit)
                converged = bool(result.success)

        fitted = np.exp(best_parameters)
        self.low_kernel = RBFKernel(fitted[0], fitted[1])
        self.discrepancy_kernel = RBFKernel(fitted[2], fitted[3])
        return HyperparameterFitResult(
            initial_negative_log_likelihood=float(initial_objective),
            final_negative_log_likelihood=float(best_objective),
            iterations=best_iterations,
            evaluations=total_evaluations,
            converged=converged,
        )

    def predict_high(self, query_positions: ArrayLike) -> HighFidelityPrediction:
        """Predict latent ``f_H`` posterior mean and marginal variance."""
        query = _positions(query_positions, "query_positions")
        prior_variance = self.rho * self.rho * self.low_kernel.diagonal(
            query
        ) + self._discrepancy_diagonal(query)

        cross_covariance = self._training_to_high_covariance(query)
        return self._predict_marginals(query, prior_variance, cross_covariance)

    def predict_low(self, query_positions: ArrayLike) -> LowFidelityPrediction:
        """Predict latent f_L conditioned on both LOW and HIGH observations."""
        query = _positions(query_positions, "query_positions")
        cross = np.vstack(
            (
                self.low_kernel.covariance(self._low_positions, query),
                self.rho * self.low_kernel.covariance(self._high_positions, query),
            )
        )
        result = self._predict_marginals(query, self.low_kernel.diagonal(query), cross)
        return LowFidelityPrediction(result.mean, result.variance)

    def _predict_marginals(self, query, prior_variance, cross_covariance):
        if self._cholesky is None:
            return HighFidelityPrediction(
                mean=np.zeros(len(query)), variance=prior_variance
            )
        mean = cross_covariance.T @ self._alpha
        projected = solve_triangular(
            self._cholesky,
            cross_covariance,
            lower=True,
            check_finite=False,
        )
        variance = prior_variance - np.einsum("ij,ij->j", projected, projected)

        if not np.all(np.isfinite(mean)) or not np.all(np.isfinite(variance)):
            raise FloatingPointError("GP prediction produced nonfinite values")
        tolerance = 1.0e-10 * np.maximum(1.0, np.abs(prior_variance))
        if np.any(variance < -tolerance):
            minimum = float(np.min(variance))
            raise FloatingPointError(
                f"GP prediction produced materially negative variance: {minimum}"
            )
        variance = np.maximum(variance, 0.0)
        return HighFidelityPrediction(mean=mean, variance=variance)

    def predict_joint_latent(
        self, query_positions, high_fidelity
    ) -> JointLatentPrediction:
        """Mixed latent posterior; high_fidelity is a boolean flag per position."""
        query = _positions(query_positions, "query_positions")
        high = np.asarray(high_fidelity)
        if high.dtype != np.bool_ or high.shape != (len(query),):
            raise ValueError(
                "high_fidelity must be a boolean vector matching positions"
            )
        scale = np.where(high, self.rho, 1.0)
        covariance = self.low_kernel.covariance(query, query) * np.outer(scale, scale)
        covariance += self._discrepancy_covariance(query, query) * np.outer(high, high)
        mean = np.zeros(len(query))
        if self._cholesky is not None:
            cross = np.vstack(
                (
                    self.low_kernel.covariance(self._low_positions, query) * scale,
                    self.rho
                    * self.low_kernel.covariance(self._high_positions, query)
                    * scale
                    + self._discrepancy_covariance(self._high_positions, query) * high,
                )
            )
            mean = cross.T @ self._alpha
            projected = solve_triangular(
                self._cholesky, cross, lower=True, check_finite=False
            )
            covariance -= projected.T @ projected
        covariance = 0.5 * (covariance + covariance.T)
        diagonal = np.diag(covariance)
        if np.any(
            diagonal
            < -1e-10
            * max(
                1.0,
                self.low_kernel.variance,
                self.rho**2 * self.low_kernel.variance
                + self.discrepancy_kernel.variance,
            )
        ):
            raise FloatingPointError("negative joint latent variance")
        covariance[np.diag_indices(len(query))] = np.maximum(diagonal, 0.0)
        return JointLatentPrediction(mean, covariance)

    def score_replacements(
        self,
        query_positions,
        integration_weights,
        candidate_positions,
        candidate_high,
        candidate_noise,
    ):
        """Score every one-for-one swap using the current Cholesky factor.

        Rows follow retained [LOW,HIGH] training order; columns follow new
        candidates. No candidate GP is fitted and no inverse matrix is formed.
        Precision diagonals are obtained by blocked triangular basis solves.
        """
        if self._cholesky is None:
            raise ValueError("replacement scoring requires a fitted GP")
        query = _positions(query_positions, "query_positions")
        weights = _values(integration_weights, len(query), "integration_weights")
        if np.any(weights < 0) or not np.isfinite(weights.sum()) or weights.sum() <= 0:
            raise ValueError("replacement weights require positive total mass")
        weights = weights / weights.sum()
        candidates = _positions(candidate_positions, "candidate_positions")
        high = np.asarray(candidate_high)
        if high.dtype != np.bool_ or high.shape != (len(candidates),):
            raise ValueError("candidate_high must be a matching boolean vector")
        noise = _noise_variances(candidate_noise, len(candidates), "candidate_noise")
        factor = self._cholesky
        projected_query, influence, objective, precision_diagonal, norm_old = (
            self._replacement_geometry(query, weights)
        )
        scale = np.where(high, self.rho, 1.0)
        cross_new = np.vstack(
            (
                self.low_kernel.covariance(self._low_positions, candidates) * scale,
                self.rho
                * self.low_kernel.covariance(self._high_positions, candidates)
                * scale
                + self._discrepancy_covariance(self._high_positions, candidates) * high,
            )
        )
        projected_new = solve_triangular(
            factor, cross_new, lower=True, check_finite=False
        )
        coefficients = solve_triangular(
            factor.T, projected_new, lower=False, check_finite=False
        )
        prior_new = scale**2 * self.low_kernel.diagonal(
            candidates
        ) + high * self._discrepancy_diagonal(candidates)
        latent_variance = prior_new - np.einsum(
            "ij,ij->j", projected_new, projected_new
        )
        if np.any(latent_variance < -1e-10 * np.maximum(1.0, prior_new)):
            raise FloatingPointError("negative candidate variance during replacement")
        denominator = np.maximum(latent_variance, 0.0) + noise + self._effective_jitter
        prior_cross = self.rho * self.low_kernel.covariance(query, candidates) * scale
        prior_cross += self._discrepancy_covariance(query, candidates) * high
        posterior_cross = prior_cross - projected_query.T @ projected_new
        norm_new = np.einsum("ij,i,ij->j", posterior_cross, weights, posterior_cross)
        gain = norm_new / denominator
        cross_terms = influence @ (weights[:, None] * posterior_cross)
        ratio = coefficients / denominator
        numerator = norm_old[:, None] - 2 * ratio * cross_terms + ratio**2 * norm_new
        removal_loss = np.maximum(numerator, 0.0) / (
            precision_diagonal[:, None] + coefficients**2 / denominator
        )
        return ReplacementScores(gain[None, :] - removal_loss, objective)

    def _replacement_geometry(self, query, weights):
        """Cache one grid/factor geometry; invalidated by successful refitting."""
        signature = (
            id(self._cholesky),
            self.rho,
            self.kernel_hyperparameters,
            self.discrepancy_enabled,
            self._effective_jitter,
        )
        cache = self._replacement_cache
        if (
            cache is not None
            and cache[0] == signature
            and np.array_equal(query, cache[1])
            and np.array_equal(weights, cache[2])
        ):
            return cache[3]
        factor = self._cholesky
        count = len(factor)
        cross_query = self._training_to_high_covariance(query)
        projected_query = solve_triangular(
            factor, cross_query, lower=True, check_finite=False
        )
        influence = solve_triangular(
            factor.T, projected_query, lower=False, check_finite=False
        )
        prior = self.rho**2 * self.low_kernel.diagonal(
            query
        ) + self._discrepancy_diagonal(query)
        variance = np.maximum(
            prior - np.einsum("ij,ij->j", projected_query, projected_query), 0.0
        )
        objective = float(weights @ variance)
        precision_diagonal = np.empty(count)
        for start in range(0, count, 64):
            width = min(64, count - start)
            basis = np.zeros((count, width))
            basis[start + np.arange(width), np.arange(width)] = 1.0
            solved = solve_triangular(factor, basis, lower=True, check_finite=False)
            precision_diagonal[start : start + width] = np.einsum(
                "ij,ij->j", solved, solved
            )
        norm_old = np.einsum("ij,j,ij->i", influence, weights, influence)
        values = (projected_query, influence, objective, precision_diagonal, norm_old)
        self._replacement_cache = (signature, query.copy(), weights.copy(), values)
        return values

    def _training_covariance(
        self,
        low_positions: FloatArray,
        low_noise: FloatArray,
        high_positions: FloatArray,
        high_noise: FloatArray,
    ) -> FloatArray:
        """Assemble the designed joint covariance in ``[LOW, HIGH]`` order."""
        n_low = low_positions.shape[0]
        n_high = high_positions.shape[0]
        covariance = np.empty((n_low + n_high, n_low + n_high), dtype=float)

        low_low = self.low_kernel.covariance(low_positions, low_positions)
        low_high = self.rho * self.low_kernel.covariance(low_positions, high_positions)
        high_high = self.rho * self.rho * self.low_kernel.covariance(
            high_positions, high_positions
        ) + self._discrepancy_covariance(high_positions, high_positions)
        if n_low:
            low_low[np.diag_indices(n_low)] += low_noise
        if n_high:
            high_high[np.diag_indices(n_high)] += high_noise

        covariance[:n_low, :n_low] = low_low
        covariance[:n_low, n_low:] = low_high
        covariance[n_low:, :n_low] = low_high.T
        covariance[n_low:, n_low:] = high_high
        return 0.5 * (covariance + covariance.T)

    def _training_to_high_covariance(self, query: FloatArray) -> FloatArray:
        """Return ``Cov([y_L, y_H], f_H(query))``."""
        low_to_high = self.rho * self.low_kernel.covariance(self._low_positions, query)
        high_to_high = self.rho * self.rho * self.low_kernel.covariance(
            self._high_positions, query
        ) + self._discrepancy_covariance(self._high_positions, query)
        return np.vstack((low_to_high, high_to_high))

    def _discrepancy_covariance(
        self, x_left: ArrayLike, x_right: ArrayLike
    ) -> FloatArray:
        """Return configured discrepancy covariance or the exact zero kernel."""
        left = _positions(x_left, "x_left")
        right = _positions(x_right, "x_right")
        if not self.discrepancy_enabled:
            return np.zeros((left.shape[0], right.shape[0]), dtype=float)
        return self.discrepancy_kernel.covariance(left, right)

    def _discrepancy_diagonal(self, x: ArrayLike) -> FloatArray:
        """Return configured discrepancy prior variance or exact zeros."""
        positions = _positions(x, "x")
        if not self.discrepancy_enabled:
            return np.zeros(positions.shape[0], dtype=float)
        return self.discrepancy_kernel.diagonal(positions)

    def _factorize(self, covariance: FloatArray) -> tuple[FloatArray, float]:
        """Cholesky-factor covariance using deterministic jitter escalation."""
        diagonal = np.diag_indices_from(covariance)
        effective_jitter = self.jitter
        last_error: np.linalg.LinAlgError | None = None

        for _ in range(self.max_jitter_attempts):
            regularized = np.array(covariance, copy=True)
            regularized[diagonal] += effective_jitter
            try:
                factor = np.linalg.cholesky(regularized)
            except np.linalg.LinAlgError as exc:
                last_error = exc
                effective_jitter *= self.jitter_multiplier
                continue
            if not np.all(np.isfinite(factor)):
                raise FloatingPointError("Cholesky factor contains nonfinite values")
            return factor, effective_jitter

        raise np.linalg.LinAlgError(
            "joint covariance was not positive definite after "
            f"{self.max_jitter_attempts} jitter attempts"
        ) from last_error
