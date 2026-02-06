"""
HEDAC: Heat Equation Driven Area Coverage

This is a cleaned implementation of the HEDAC algorithm for multi-agent ergodic control.
Based on the paper: "Heat Equation Driven Area Coverage" by Sarah Dean et al.
"""

import numpy as np
from typing import Tuple, Optional, Union, Dict, Any
import yaml
import os


class HEDACParams:
    """
    Parameters for HEDAC algorithm loaded from YAML file.

    Usage:
        # Load from YAML file
        params = HEDACParams.from_yaml('configs/default_params.yaml')

        # Access parameters
        print(params.num_agents)
        print(params.heat_equation['alpha'])
    """

    def __init__(self, config: Dict[str, Any]):
        """
        Initialize from configuration dictionary.

        Args:
            config: Configuration dictionary loaded from YAML
        """
        self._config = config

        # Extract commonly used parameters for convenience
        sim = config.get("simulation", {})
        self.num_steps = sim.get("num_steps", 1000)
        self.num_agents = sim.get("num_agents", 1)
        self.dt = sim.get("dt", 0.1)
        self.random_seed = sim.get("random_seed", 42)

        # Heat equation parameters
        self.heat_equation = config.get("heat_equation", {})
        self.alpha = self.heat_equation.get("alpha", 0.1)
        self.source_strength = self.heat_equation.get("source_strength", 1.0)
        self.beta = self.heat_equation.get("beta", 0.01)
        self.local_cooling = self.heat_equation.get("local_cooling", 0.1)

        # Agent parameters
        agents = config.get("agents", {})
        self.max_dx = agents.get("max_velocity", 1.0)
        self.max_ddx = agents.get("max_acceleration", 0.5)
        self.max_dtheta = agents.get("max_angular_velocity", np.pi / 4)
        self.max_ddtheta = agents.get("max_angular_acceleration", np.pi / 8)
        self.dt_agent = agents.get("dt_agent", 0.1)
        self.agent_radius = agents.get("agent_radius", 0.5)
        self.min_kernel_val = agents.get("min_kernel_val", 0.01)

        # Sensor parameters
        sensor = config.get("sensor", {})
        self.fov_deg = sensor.get("fov_degrees", 90.0)
        self.fov_depth = sensor.get("fov_depth", 5.0)

        # Multi-agent parameters
        multi = config.get("multi_agent", {})
        self.sens_range = multi.get("sensing_range", 10.0)
        self.min_safe_range = multi.get("min_safe_distance", 1.0)

        # Map parameters
        self.map_config = config.get("map", {})
        self.resolution = self.map_config.get("resolution", 1.0)

        # Grid dimensions (set after loading map)
        self.width = 50
        self.height = 50
        self.nb_var = 2
        self.dx = self.resolution

    @classmethod
    def from_yaml(cls, filepath: str) -> "HEDACParams":
        """
        Load parameters from YAML file.

        Args:
            filepath: Path to YAML configuration file

        Returns:
            HEDACParams instance
        """
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Config file not found: {filepath}")

        with open(filepath, "r") as f:
            config = yaml.safe_load(f)

        return cls(config)

    @classmethod
    def from_dict(cls, config: Dict[str, Any]) -> "HEDACParams":
        """
        Create from configuration dictionary.

        Args:
            config: Configuration dictionary

        Returns:
            HEDACParams instance
        """
        return cls(config)

    def update_from_map(self, map_shape: Tuple[int, int]):
        """
        Update grid parameters after loading map.

        Args:
            map_shape: Map shape (height, width)
        """
        self.height, self.width = map_shape
        self.dx = self.resolution

    def get(self, key: str, default: Any = None) -> Any:
        """
        Get parameter by key (supports nested keys with '.').

        Args:
            key: Parameter key (e.g., 'simulation.num_steps' or 'alpha')
            default: Default value if key not found

        Returns:
            Parameter value
        """
        if "." in key:
            keys = key.split(".")
            value = self._config
            for k in keys:
                if isinstance(value, dict):
                    value = value.get(k, default)
                else:
                    return default
            return value
        else:
            # Try to get from config or from instance attributes
            if key in self._config:
                return self._config[key]
            return getattr(self, key, default)

    def __getitem__(self, key: str) -> Any:
        """Allow dictionary-style access."""
        return self.get(key)

    def to_dict(self) -> Dict[str, Any]:
        """Return configuration as dictionary."""
        return self._config.copy()

    def __repr__(self) -> str:
        """String representation."""
        return f"HEDACParams(agents={self.num_agents}, steps={self.num_steps}, map={self.width}x{self.height})"


class MapLoader:
    """
    Flexible map loader that handles both obstacle-free and provided maps.

    Maps should be numpy arrays where:
    - 0 represents free space
    - 1 represents occupied/obstacle
    """

    def __init__(
        self,
        map_path: Optional[str] = None,
        size: Tuple[int, int] = (50, 50),
        resolution: float = 1.0,
    ):
        """
        Initialize map loader.

        Args:
            map_path: Path to .npy file containing map (optional)
            size: Size of map if creating obstacle-free (height, width)
            resolution: Spatial resolution of each cell
        """
        self.map_path = map_path
        self.size = size
        self.resolution = resolution
        self.map = None
        self.free_cells = None

    def load(self) -> np.ndarray:
        """
        Load or create map.

        Returns:
            numpy array with shape (height, width) containing 0s (free) and 1s (occupied)
        """
        if self.map_path is not None:
            # Load existing map
            self.map = np.load(self.map_path)
            # Ensure binary map (0 and 1 only)
            self.map = (self.map > 0).astype(np.int32)
        else:
            # Create obstacle-free map
            self.map = np.zeros(self.size, dtype=np.int32)

        # Cache free cells
        self.free_cells = np.array(np.where(self.map == 0)).T

        return self.map

    def get_free_cells(self) -> np.ndarray:
        """Get coordinates of free cells."""
        if self.free_cells is None:
            self.load()
        return self.free_cells

    @property
    def shape(self) -> Tuple[int, int]:
        """Get map shape (height, width)."""
        if self.map is None:
            self.load()
        return self.map.shape

    @property
    def area(self) -> float:
        """Get total free area."""
        if self.map is None:
            self.load()
        free_count = np.sum(self.map == 0)
        return free_count * self.resolution * self.resolution


def compute_ergodic_metric(
    coverage_density: np.ndarray, target_distribution: np.ndarray, map_array: np.ndarray
) -> float:
    """
    Compute the ergodic metric as L2 distance between coverage and target distributions.

    This follows the standard definition from the ergodic control literature:
    ergodic_metric = sqrt(sum((coverage - target)^2))

    Args:
        coverage_density: Coverage density map (unnormalized)
        target_distribution: Target goal distribution (should be normalized)
        map_array: Binary map where 1=obstacle, 0=free

    Returns:
        Ergodic metric value (scalar)
    """
    # Normalize coverage density to sum to 1
    coverage_normalized = coverage_density / (np.sum(coverage_density) + 1e-10)

    # Only consider free cells for the metric
    free_mask = map_array == 0

    # Compute L2 distance
    diff = (coverage_normalized - target_distribution) * free_mask
    ergodic_metric = np.sqrt(np.sum(diff**2))

    return ergodic_metric


def compute_ergodic_metric_vectorized(
    coverage_histories: np.ndarray,
    target_distribution: np.ndarray,
    map_array: np.ndarray,
) -> np.ndarray:
    """
    Compute ergodic metric over time for multiple agents.

    Args:
        coverage_histories: Array of shape (T, H, W) or (T, N, H, W) where
                           T=timesteps, N=agents, H=height, W=width
        target_distribution: Target distribution (H, W)
        map_array: Binary map (H, W)

    Returns:
        Ergodic metric over time (T,) or (T, N)
    """
    free_mask = map_array == 0

    if coverage_histories.ndim == 3:
        # Single agent: (T, H, W)
        # Normalize each timestep
        sums = np.sum(coverage_histories, axis=(1, 2), keepdims=True)
        coverage_normalized = coverage_histories / (sums + 1e-10)

        # Compute L2 distance at each timestep
        diff = (coverage_normalized - target_distribution[None, :, :]) * free_mask[
            None, :, :
        ]
        metrics = np.sqrt(np.sum(diff**2, axis=(1, 2)))

    elif coverage_histories.ndim == 4:
        # Multiple agents: (T, N, H, W)
        sums = np.sum(coverage_histories, axis=(2, 3), keepdims=True)
        coverage_normalized = coverage_histories / (sums + 1e-10)

        diff = (
            coverage_normalized - target_distribution[None, None, :, :]
        ) * free_mask[None, None, :, :]
        metrics = np.sqrt(np.sum(diff**2, axis=(2, 3)))
    else:
        raise ValueError(
            f"Unexpected coverage_histories shape: {coverage_histories.shape}"
        )

    return metrics


# For backward compatibility - can be removed once migration is complete
def load_params_from_yaml(filepath: str) -> HEDACParams:
    """
    Convenience function to load parameters from YAML.

    Args:
        filepath: Path to YAML file

    Returns:
        HEDACParams instance
    """
    return HEDACParams.from_yaml(filepath)
