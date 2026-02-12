"""
HEDAC: Heat Equation Driven Area Coverage

A cleaned implementation of the HEDAC algorithm for multi-agent ergodic control.
"""

from .base import HEDACParams, MapLoader, compute_ergodic_metric
from .hedac import HEDACAlgorithm

__all__ = [
    "HEDACParams",
    "MapLoader",
    "compute_ergodic_metric",
    "HEDACAlgorithm",
]
