"""
HEDAC: Heat Equation Driven Area Coverage

A cleaned implementation of the HEDAC algorithm for multi-agent ergodic control.
"""

from .base import HEDACParams, MapLoader, compute_ergodic_metric
from .hedac import HEDACAlgorithm
from .GaussianProcess import GaussianProcess
from .multifidelity_gp import (
    HighFidelityPrediction,
    HyperparameterFitResult,
    KernelHyperparameterBounds,
    MultiFidelityGaussianProcess,
    RBFKernel,
)
from .observations import (
    BoundedObservationBuffer,
    Fidelity,
    Observation,
    RetentionConfig,
    SpatialAgeRetentionPolicy,
)
from .density import (
    build_aerial_target,
    normalize_nonnegative_density,
    positive_part,
    resample_structured_grid,
    validate_density,
)
from .multifidelity_estimator import (
    CentralAsynchronousEstimator,
    EstimatorSettings,
    HyperparameterOptimizationSettings,
    PosteriorSnapshot,
    UpdateReport,
    UpdateStatus,
)

__all__ = [
    "HEDACParams",
    "MapLoader",
    "compute_ergodic_metric",
    "HEDACAlgorithm",
    "GaussianProcess",
    "HighFidelityPrediction",
    "HyperparameterFitResult",
    "KernelHyperparameterBounds",
    "MultiFidelityGaussianProcess",
    "RBFKernel",
    "BoundedObservationBuffer",
    "Fidelity",
    "Observation",
    "RetentionConfig",
    "SpatialAgeRetentionPolicy",
    "build_aerial_target",
    "normalize_nonnegative_density",
    "positive_part",
    "resample_structured_grid",
    "validate_density",
    "CentralAsynchronousEstimator",
    "EstimatorSettings",
    "HyperparameterOptimizationSettings",
    "PosteriorSnapshot",
    "UpdateReport",
    "UpdateStatus",
]
