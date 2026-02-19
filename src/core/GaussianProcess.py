"""
Gaussian Process implementation for HEDAC algorithm.

This module provides a GaussianProcess class that wraps scikit-learn's
GaussianProcessRegressor with HEDAC-specific functionality.
"""

import numpy as np
from typing import Optional, Tuple, Any, Union, cast

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel, ConstantKernel as C

from ..core.base import HEDACParams
from ..models.agents import AgentTeam
from ..utils.math_utils import min_max_normalize, bilinear_interpolate


class GaussianProcess:
    """
    Gaussian Process regressor for goal density estimation.

    This class wraps scikit-learn's GaussianProcessRegressor and provides
    HEDAC-specific functionality for:
    - Filtering observations based on uncertainty
    - Combining mean and uncertainty for exploration
    - Managing the GP dataset with size limits
    """

    def __init__(self, params: HEDACParams, map_shape: Tuple[int, int]):
        """
        Initialize Gaussian Process.

        Args:
            params: HEDAC parameters containing GPR configuration
            map_shape: Shape of the map (height, width)
        """
        self.params = params
        self.map_shape = map_shape
        self.height, self.width = map_shape

        # GP hyperparameters from params
        self.length_scale = float(params.get("gpr.length_scale", 1.0) or 1.0)
        self.sigma_f = float(params.get("gpr.sigma_f", 1.0) or 1.0)
        self.noise_level = float(params.get("gpr.noise_level", 0.1) or 0.1)
        self.gpr_impl = str(params.get("gpr.implementation", "sklearn"))

        # Observation settings
        self.obs_noise_std = float(params.get("gpr.obs_noise_std", 0.1) or 0.1)
        self.obs_per_step = int(params.get("gpr.obs_per_step", 10) or 10)
        self.use_filter = bool(params.get("gpr.use_filter", True))
        self.take_threshold = float(params.get("gpr.take_threshold", 0.25) or 0.25)
        self.remove_threshold = float(params.get("gpr.remove_threshold", 0.15) or 0.15)
        self.combo_gamma = float(params.get("gpr.gamma", 0.5) or 0.5)

        min_samples_default = self.obs_per_step
        self.min_samples = int(
            params.get("gpr.min_samples", min_samples_default) or min_samples_default
        )

        max_dataset_size = params.get("gpr.max_dataset_size", None)
        self.max_dataset_size = (
            int(max_dataset_size) if max_dataset_size is not None else None
        )

        # Hyperparameter optimization
        self.fit_interval = int(params.get("gpr.fit_interval", 10) or 10)
        self.last_fit_step = -1
        self.new_data_since_last_fit = False

        # Initialize kernel
        self.kernel = C(1.0, constant_value_bounds=(1e-3, 1e3)) * RBF(
            length_scale=1.0, length_scale_bounds=(1e-3, 1e3)
        ) + WhiteKernel(noise_level=1e-3, noise_level_bounds=(1e-5, 1e1))

        # Initialize GP model
        self.model = GaussianProcessRegressor(
            kernel=self.kernel,
            alpha=1e-5,
            normalize_y=False,
            n_restarts_optimizer=1,
        )

        # GP dataset
        self.dataset = np.empty((0, 3), dtype=float)
        self.all_observations = np.empty((0, 3), dtype=float)

        # Predictions
        self.gp_mean: Optional[np.ndarray] = None
        self.gp_std: Optional[np.ndarray] = None
        self.gp_std_normalized: Optional[np.ndarray] = None

        # Precompute grid points for prediction
        self._precompute_grid()

    def _precompute_grid(self):
        """Precompute grid points for GP prediction."""
        grid_x, grid_y = np.meshgrid(np.arange(self.width), np.arange(self.height))
        self.grid_points = np.column_stack((grid_x.ravel(), grid_y.ravel()))

    def _sample_map_at_points(
        self, map_values: np.ndarray, points: np.ndarray
    ) -> np.ndarray:
        """Sample a 2D map at (x, y) points using bilinear interpolation."""
        values = np.empty(points.shape[0], dtype=float)
        for idx, point in enumerate(points):
            values[idx] = bilinear_interpolate(map_values, point)
        return values

    def _filter_observations_by_uncertainty(
        self,
        new_observations: np.ndarray,
        std_map_normalized: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Filter observations based on normalized uncertainty.

        Returns:
            Updated dataset and filtered new observations.
        """
        if new_observations.size == 0:
            return self.dataset, new_observations

        filtered_new = new_observations

        if self.dataset.size == 0:
            self.dataset = new_observations.copy()
        else:
            new_std = self._sample_map_at_points(
                std_map_normalized, new_observations[:, :2]
            )
            keep_new = new_std > float(self.take_threshold)
            filtered_new = new_observations[keep_new]

            remove_threshold = (
                None if self.remove_threshold <= 0 else float(self.remove_threshold)
            )
            if remove_threshold is not None and self.dataset.size > 0:
                existing_std = self._sample_map_at_points(
                    std_map_normalized, self.dataset[:, :2]
                )
                keep_existing = existing_std > remove_threshold
                self.dataset = self.dataset[keep_existing]

            if filtered_new.size > 0:
                self.dataset = np.vstack((self.dataset, filtered_new))

        if (
            self.max_dataset_size is not None
            and self.dataset.shape[0] > self.max_dataset_size
        ):
            self.dataset = self.dataset[-self.max_dataset_size :]

        return self.dataset, filtered_new

    def _combine_mean_std_density(
        self,
        mean_map: np.ndarray,
        std_map: np.ndarray,
        gamma: float = 0.5,
    ) -> np.ndarray:
        """Combine mean and std into a goal density map."""
        mean_clipped = np.maximum(mean_map, 0)
        std_clipped = np.maximum(std_map, 0)

        mean_norm = min_max_normalize(mean_clipped)
        std_norm = min_max_normalize(std_clipped)

        combined = np.exp(mean_norm) + np.exp(std_norm) - 2

        return min_max_normalize(np.maximum(combined, 0))

    def _remove_duplicates(self):
        """Remove duplicate observations from dataset."""
        if self.dataset.shape[0] > 0:
            rounded_positions = np.round(self.dataset[:, :2]).astype(int)
            _, unique_indices = np.unique(rounded_positions, axis=0, return_index=True)
            self.dataset = self.dataset[unique_indices]

    def update_gp(self, new_observations: np.ndarray, step_num: int = 0) -> bool:
        """
        Update GP dataset and posterior estimates.

        Args:
            new_observations: New observations to add (N, 3) array with [x, y, value]
            step_num: Current step number for fit interval tracking

        Returns:
            True if GP predictions were updated, False otherwise
        """
        added_new_data = False

        # Add to all observations history
        if new_observations.size > 0:
            if self.all_observations.size == 0:
                self.all_observations = new_observations.copy()
            else:
                self.all_observations = np.vstack(
                    (self.all_observations, new_observations)
                )

        # Filter and add to dataset
        if new_observations.size > 0:
            if self.use_filter and self.gp_std_normalized is not None:
                self.dataset, filtered_new = self._filter_observations_by_uncertainty(
                    new_observations, self.gp_std_normalized
                )
                added_new_data = filtered_new.size > 0
            else:
                if self.dataset.size == 0:
                    self.dataset = new_observations.copy()
                else:
                    self.dataset = np.vstack((self.dataset, new_observations))
                added_new_data = True

                if (
                    self.max_dataset_size is not None
                    and self.dataset.shape[0] > self.max_dataset_size
                ):
                    self.dataset = self.dataset[-self.max_dataset_size :]

        # Remove duplicates
        self._remove_duplicates()

        if added_new_data:
            self.new_data_since_last_fit = True

        # Fit and predict if we have enough samples
        if self.dataset.shape[0] >= self.min_samples:
            x_train = self.dataset[:, :2]
            y_train = self.dataset[:, 2]

            should_optimize = (
                self.new_data_since_last_fit
                and (step_num - self.last_fit_step) >= self.fit_interval
            )

            if should_optimize:
                self.model.fit(x_train, y_train)
                self.last_fit_step = step_num
                self.new_data_since_last_fit = False

            pred = self.model.predict(self.grid_points, return_std=True)
            y_pred, y_std = cast(Tuple[np.ndarray, np.ndarray], pred)

            self.gp_mean = y_pred.reshape(self.map_shape)
            self.gp_std = y_std.reshape(self.map_shape)
            self.gp_std_normalized = min_max_normalize(self.gp_std)

            return True
        else:
            self.gp_mean = None
            self.gp_std = None
            self.gp_std_normalized = None
            return False

    def get_goal_density(self) -> Optional[np.ndarray]:
        """
        Get the current goal density combining mean and uncertainty.

        Returns:
            Goal density map or None if not enough samples
        """
        if self.gp_mean is not None and self.gp_std is not None:
            return self._combine_mean_std_density(
                self.gp_mean, self.gp_std, gamma=self.combo_gamma
            )
        return None

    def predict(
        self, points: np.ndarray, return_std: bool = False
    ) -> Union[np.ndarray, Tuple[np.ndarray, np.ndarray]]:
        """
        Predict mean and optionally std at given points.

        Args:
            points: Array of (x, y) coordinates with shape (N, 2)
            return_std: If True, return (mean, std) tuple. If False, return only mean.

        Returns:
            Mean array or tuple of (mean, std) arrays depending on return_std
        """
        if self.dataset.shape[0] < self.min_samples:
            raise ValueError(
                f"Not enough samples. Have {self.dataset.shape[0]}, need {self.min_samples}"
            )

        if return_std:
            pred = self.model.predict(points, return_std=True)
            return cast(Tuple[np.ndarray, np.ndarray], pred)
        else:
            mean = self.model.predict(points, return_std=False)
            return mean

    def get_uncertainty_at_points(self, points: np.ndarray) -> np.ndarray:
        """Get normalized uncertainty at given points."""
        if self.gp_std_normalized is None:
            return np.zeros(points.shape[0])
        return self._sample_map_at_points(self.gp_std_normalized, points)

    @property
    def n_samples(self) -> int:
        """Get number of samples in dataset."""
        return self.dataset.shape[0]

    @property
    def has_prediction(self) -> bool:
        """Check if GP has valid predictions."""
        return self.gp_mean is not None and self.gp_std is not None

    def reset(self):
        """Reset GP state."""
        self.dataset = np.empty((0, 3), dtype=float)
        self.all_observations = np.empty((0, 3), dtype=float)
        self.gp_mean = None
        self.gp_std = None
        self.gp_std_normalized = None
        self.last_fit_step = -1
        self.new_data_since_last_fit = False

    def collect_observations(
        self, agent_team: AgentTeam, goal_density: np.ndarray
    ) -> np.ndarray:
        """
        Collect noisy observations from all agents.

        Args:
            agent_team: Team of agents to collect observations from
            goal_density: True goal density map for sampling

        Returns:
            Array of observations with shape (N, 3) containing [x, y, value]
        """
        obs_list = []
        for agent in agent_team.agents:
            obs = agent.sense_environment_gp(
                goal_density,
                noise_std=self.obs_noise_std,
                n_samples=self.obs_per_step,
            )
            if obs.size > 0:
                obs_list.append(obs)
        if not obs_list:
            return np.empty((0, 3), dtype=float)
        return np.vstack(obs_list)
