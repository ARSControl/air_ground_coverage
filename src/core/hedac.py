"""
Main HEDAC algorithm implementation.
"""

import os
import numpy as np
from typing import Optional, List, Dict, Any, Tuple, cast

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel, ConstantKernel as C

from ..core.base import HEDACParams, MapLoader, compute_ergodic_metric
from ..utils.math_utils import (
    calculate_gradient,
    update_heat,
    create_agent_block,
    clamp_kernel_1d,
    normalize_to_pdf,
    min_max_normalize,
    bilinear_interpolate,
)
from ..models.agents import AgentLike, AgentTeam
from ..utils.visualize_gp import visualize_gp_debug


class HEDACAlgorithm:
    """
    Heat Equation Driven Area Coverage (HEDAC) algorithm.

    This implements the standard HEDAC algorithm where agents follow the gradient
    of a heat field to achieve ergodic coverage of a target distribution.
    """

    def __init__(
        self, params: HEDACParams, map_loader: MapLoader, goal_density: np.ndarray
    ):
        """
        Initialize HEDAC algorithm.

        Args:
            params: HEDAC parameters
            map_loader: Map loader instance
            goal_density: Target distribution to cover (should be normalized)
        """
        self.params = params
        self.map_loader = map_loader
        self.map = map_loader.load()
        self.width = self.map.shape[0]
        self.height = self.map.shape[1]
        self.goal_density = goal_density
        self.current_goal_density = self.goal_density.copy()

        # Update params with actual map dimensions
        params.update_from_map(self.map.shape)

        # Initialize heat field
        self.heat_field = np.zeros_like(self.map, dtype=float)

        # Coverage tracking
        self.coverage_density = np.zeros_like(self.map, dtype=float)
        self.local_cooling = np.zeros_like(self.map, dtype=float)

        # Create agent coverage block
        self.coverage_block = create_agent_block(
            params.nb_var, params.min_kernel_val, params.agent_radius
        )
        self.kernel_size = self.coverage_block.shape[0]
        self.half_kernel = self.kernel_size // 2

        # GP hyperparameters (fixed, no optimization)
        self.length_scale = float(params.get("gpr.length_scale", 1.0) or 1.0)
        self.sigma_f = float(params.get("gpr.sigma_f", 1.0) or 1.0)
        self.noise_level = float(params.get("gpr.noise_level", 0.1) or 0.1)
        self.gpr_impl = str(params.get("gpr.implementation", "handcoded"))

        self.kernel = (
            C(1.0, constant_value_bounds=(1e-3, 1e3))
            * RBF(length_scale=1.0, length_scale_bounds=(1e-3, 1e3))  # space
            + WhiteKernel(noise_level=1e-3, noise_level_bounds=(1e-5, 1e1))
        )

        self.gpr_model = GaussianProcessRegressor(
            kernel=self.kernel,
            alpha=1e-5,
            normalize_y=False,
            n_restarts_optimizer=1,
        )

        # GP dataset and predictions
        self.dataset = np.empty((0, 3), dtype=float)
        self.all_observations = np.empty((0, 3), dtype=float)
        self.gp_mean: Optional[np.ndarray] = None
        self.gp_std: Optional[np.ndarray] = None
        self.gp_std_normalized: Optional[np.ndarray] = None

        # Observation settings
        self.obs_noise_std = float(params.get("gpr.obs_noise_std", 0.1) or 0.1)
        self.obs_per_step = int(params.get("gpr.obs_per_step", 10) or 10)

        min_samples_default = self.obs_per_step
        self.min_samples = int(
            params.get("gpr.min_samples", min_samples_default) or min_samples_default
        )
        self.use_filter = bool(params.get("gpr.use_filter", True))
        self.take_threshold = float(params.get("gpr.take_threshold", 0.25) or 0.25)
        self.remove_threshold = float(params.get("gpr.remove_threshold", 0.15) or 0.15)
        max_dataset_size = params.get("gpr.max_dataset_size", None)
        self.max_dataset_size = (
            int(max_dataset_size) if max_dataset_size is not None else None
        )
        self.combo_gamma = float(params.get("gpr.gamma", 0.5) or 0.5)

        # Hyperparameter optimization controls
        self.fit_interval = int(params.get("gpr.fit_interval", 10) or 10)

        self.last_fit_step = -1
        self.new_data_since_last_fit = False

        # GP debug visualization
        self.gp_debug_enabled = bool(params.get("visualization.gp_debug", False))
        self.gp_debug_interval = int(
            params.get("visualization.gp_debug_interval", 50) or 50
        )
        self.gp_save_frames = bool(params.get("visualization.save_gp_frames", False))
        self.gp_output_dir = str(
            params.get("visualization.gp_output_dir", "output/gp_debug")
        )

        # Precompute grid points for GP prediction
        height, width = self.map.shape
        grid_x, grid_y = np.meshgrid(np.arange(width), np.arange(height))
        self.grid_points = np.column_stack((grid_x.ravel(), grid_y.ravel()))

        # Normalize beta and local_cooling by the map area (as per original HEDAC paper)
        # This ensures proper scaling of the heat equation terms
        area = self.map_loader.area
        self._beta_normalized = params.beta / area if area > 0 else params.beta
        self._local_cooling_normalized = (
            params.local_cooling / area if area > 0 else params.local_cooling
        )

        # Metrics storage
        self.ergodic_metrics: List[float] = []

        # Initialize heat field with normalized coverage
        self._init_heat_field()

    # def _normalize_uncertainty(
    #     self, std_map: np.ndarray, map_array: Optional[np.ndarray] = None
    # ) -> np.ndarray:
    #     """Normalize a std map to [0, 1], optionally masking obstacles."""
    #     if map_array is None:
    #         return min_max_normalize(std_map)

    #     normalized = np.zeros_like(std_map, dtype=float)
    #     free_mask = map_array == 0
    #     if np.any(free_mask):
    #         free_vals = std_map[free_mask]
    #         min_val = float(np.min(free_vals))
    #         max_val = float(np.max(free_vals))
    #         denom = max(max_val - min_val, 1e-10)
    #         normalized[free_mask] = (free_vals - min_val) / denom
    #     return normalized

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
        Filter observations based on normalized uncertainty using instance settings.

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
        map_array: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Combine mean and std into a goal density map without final normalization."""
        mean_clipped = np.maximum(mean_map, 0)
        std_clipped = np.maximum(std_map, 0)
        # min-Max normalize
        mean_norm = min_max_normalize(mean_clipped)
        std_norm = min_max_normalize(std_clipped)

        combined = np.exp(mean_norm) + np.exp(std_norm) - 2

        return min_max_normalize(np.maximum(combined, 0))
        if map_array is not None:
            combined = combined * (map_array == 0)

    # @property
    # def goal_density(self) -> np.ndarray:
    #     """Get goal density."""
    #     return self._goal_density

    # @goal_density.setter
    # def goal_density(self, density: np.ndarray):
    #     """Set goal density and normalize it."""
    #     normalized = normalize_to_pdf(density, self.map)
    #     self._goal_density = normalized
    #     self.true_goal_density = density.copy()  # Store original for observations

    def _init_heat_field(self):
        """Initialize heat field from initial coverage."""
        # Start with uniform heat
        self.heat_field = np.ones_like(self.map, dtype=float)
        # Zero out obstacles
        self.heat_field[self.map == 1] = 0.0
        # Normalize
        self.heat_field = normalize_to_pdf(self.heat_field, self.map)

    def update_coverage(self, agent: AgentLike):
        """
        Update coverage density from agent position.

        Args:
            agent: Agent to update coverage from
        """
        x, y = agent.position.astype(int)

        # Clamp kernel to grid boundaries
        x_slice, x_start, x_num = clamp_kernel_1d(x, 0, self.width, self.kernel_size)
        y_slice, y_start, y_num = clamp_kernel_1d(y, 0, self.height, self.kernel_size)

        # Add coverage
        # numpy arrays are indexed as [row, col] = [y, x]
        self.coverage_density[y_slice, x_slice] += self.coverage_block[
            y_start : y_start + y_num, x_start : x_start + x_num
        ]

        # # Add local cooling
        # self.local_cooling[y_slice, x_slice] += self.coverage_block[
        #     y_start : y_start + y_num, x_start : x_start + x_num
        # ]

    def compute_source_term(self) -> np.ndarray:
        """
        Compute source term for heat equation.

        Returns:
            Source term field
        """
        # Normalize coverage and goal density
        coverage_norm = normalize_to_pdf(self.coverage_density, self.map)
        goal_norm = (
            self.current_goal_density
            if self.current_goal_density is not None
            else self.goal_density
        )

        # Difference between goal and coverages
        diff = goal_norm - coverage_norm

        # Source is squared positive difference
        source = np.maximum(diff, 0) ** 2

        # Zero out obstacles
        source[self.map == 1] = 0.0

        # Normalize and scale by area
        area = self.map_loader.area
        source = normalize_to_pdf(source, self.map) * area

        return source

    def update_heat_field(self):
        """Update heat field using heat equation."""
        source = self.compute_source_term()

        # Normalize local cooling
        local_cooling_norm = normalize_to_pdf(self.local_cooling, self.map)
        local_cooling_norm *= self.map_loader.area

        # Enforce CFL stability for the explicit heat update
        cfl_safety = float(self.params.get("heat_equation.cfl_safety", 0.9) or 0.9)
        alpha = float(self.params.alpha)
        dx = float(self.params.dx)
        # Guard alpha to avoid division by zero; fall back to dt if alpha is zero
        dt_cfl = self.params.dt if alpha == 0 else cfl_safety * dx * dx / (4.0 * alpha)
        dt_heat = min(self.params.dt, dt_cfl)

        # Update heat equation with area-normalized beta and local_cooling
        # self.heat_field = update_heat_optimized(
        #     self.heat_field,
        #     source,
        #     self.map,
        #     local_cooling_norm,
        #     dt_heat,
        #     alpha,
        #     self.params.source_strength,
        #     self._beta_normalized,
        #     self._local_cooling_normalized,
        #     dx,
        # ).astype(np.float32)
        self.heat_field = update_heat(
            self.heat_field,
            source,
            self.map,
            local_cooling_norm,
            dt_heat,
            alpha,
            self.params.source_strength,
            self._beta_normalized,
            self._local_cooling_normalized,
            dx,
        ).astype(np.float32)

    def get_agent_gradient(self, agent: AgentLike) -> np.ndarray:
        """
        Get movement gradient for an agent.

        Args:
            agent: Agent to compute gradient for

        Returns:
            Gradient direction vector (normalized)
        """
        # Compute gradients from heat field
        # heat_field has shape (height, width) = (y, x)
        # np.gradient returns [d/d(row), d/d(col)] = [d/dy, d/dx]
        gradient_y, gradient_x = np.gradient(self.heat_field)

        # Normalize gradient fields globally (critical for stable agent movement)
        # This prevents the raw gradient magnitudes from dominating the movement
        grad_mag = np.sqrt(np.mean(gradient_x**2) + np.mean(gradient_y**2))
        if grad_mag > 1e-10:
            gradient_x = gradient_x / grad_mag
            gradient_y = gradient_y / grad_mag

        # Get the interpolated gradient at agent position with boundary handling
        gradient = calculate_gradient(
            gradient_x,
            gradient_y,
            agent.position,
            self.map,
            heading=agent.theta,
            wall_avoidance_weight=float(
                self.params.get("agents.wall_avoidance_weight", 0.5) or 0.0
            ),
            kernel_radius=self.half_kernel,
        )

        # Normalize the final gradient direction
        norm = np.linalg.norm(gradient)
        if norm > 0:
            gradient /= norm

        return gradient

    def collect_observations(self, agent_team: AgentTeam) -> np.ndarray:
        """Collect noisy observations from all agents."""
        obs_list = []
        for agent in agent_team.agents:
            obs = agent.sense_environment_gp(
                self.goal_density,
                noise_std=self.obs_noise_std,
                n_samples=self.obs_per_step,
            )
            if obs.size > 0:
                obs_list.append(obs)
        if not obs_list:
            return np.empty((0, 3), dtype=float)
        return np.vstack(obs_list)

    def update_gp(self, new_observations: np.ndarray, step_num: int = 0):
        """Update GP dataset and posterior estimates."""
        added_new_data = False
        if new_observations.size > 0:
            if self.all_observations.size == 0:
                self.all_observations = new_observations.copy()
            else:
                self.all_observations = np.vstack(
                    (self.all_observations, new_observations)
                )

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

        # Polish dataset from duplicates
        if self.dataset.shape[0] > 0:
            # Round positions to nearest integer grid cell for duplicate removal
            rounded_positions = np.round(self.dataset[:, :2]).astype(int)
            _, unique_indices = np.unique(rounded_positions, axis=0, return_index=True)
            self.dataset = self.dataset[unique_indices]

        if added_new_data:
            self.new_data_since_last_fit = True

        if self.dataset.shape[0] >= self.min_samples:
            x_train = self.dataset[:, :2]
            y_train = self.dataset[:, 2]

            should_optimize = (
                self.new_data_since_last_fit
                and (step_num - self.last_fit_step) >= self.fit_interval
            )

            if should_optimize:
                print(
                    f"Fitting GP at step {step_num} with {self.dataset.shape[0]} samples..."
                )
                self.gpr_model.fit(x_train, y_train)
                self.last_fit_step = step_num
                self.new_data_since_last_fit = False

            pred = self.gpr_model.predict(self.grid_points, return_std=True)
            y_pred, y_std = cast(Tuple[np.ndarray, np.ndarray], pred)

            gp_mean = y_pred.reshape(self.map.shape)
            gp_std = y_std.reshape(self.map.shape)
            self.gp_mean = gp_mean
            self.gp_std = gp_std
            self.gp_std_normalized = min_max_normalize(gp_std)
            self.current_goal_density = self._combine_mean_std_density(
                gp_mean, gp_std, gamma=self.combo_gamma, map_array=self.map
            )
        else:
            self.gp_mean = None
            self.gp_std = None
            self.gp_std_normalized = None
            self.current_goal_density = self.goal_density

    def step(self, agent_team: AgentTeam, step_num: int = 0) -> float:
        """
        Execute one step of HEDAC algorithm.

        Args:
            agent_team: Team of agents

        Returns:
            Current ergodic metric
        """
        # Collect observations and update GP
        new_observations = self.collect_observations(agent_team)
        self.update_gp(new_observations, step_num=step_num)

        # Reset local cooling
        self.local_cooling = np.zeros_like(self.map, dtype=float)

        # Update coverage from all agents
        for agent in agent_team.agents:
            self.update_coverage(agent)

        # Update heat field
        self.update_heat_field()

        # Compute ergodic metric
        # coverage_norm = normalize_to_pdf(self.coverage_density, self.map)
        goal_density_norm = normalize_to_pdf(self.goal_density, self.map)
        erg_metric = compute_ergodic_metric(
            self.coverage_density, goal_density_norm, self.map
        )
        self.ergodic_metrics.append(erg_metric)

        # Move agents following gradient
        for agent in agent_team.agents:
            gradient = self.get_agent_gradient(agent)

            # Target velocity and heading
            k_target = 1.0
            v_target = k_target * gradient * self.params.max_dx

            theta_target = np.arctan2(gradient[1], gradient[0])

            # Update agent
            agent.track_velocity_and_heading(
                v_target, theta_target, penalize_lateral=True
            )

            # Clip position to stay within map boundaries
            agent.clip_position(0, self.map.shape[1] - 1, 0, self.map.shape[0] - 1)

        if self.gp_debug_enabled and step_num % self.gp_debug_interval == 0:
            save_path = None
            if self.gp_save_frames:
                save_path = os.path.join(
                    self.gp_output_dir, f"gp_debug_{step_num:05d}.png"
                )
            agent = agent_team.agents[0] if len(agent_team.agents) > 0 else None
            if agent is not None:
                visualize_gp_debug(
                    step_num=step_num,
                    map_array=self.map,
                    true_goal_density=self.goal_density,
                    combined_goal_density=self.current_goal_density,
                    coverage_density=self.coverage_density,
                    ergodic_metrics=np.array(self.ergodic_metrics),
                    trajectory=agent.x_hist,
                    current_position=agent.position,
                    observations_all=self.all_observations,
                    observations_filtered=self.dataset,
                    save_path=save_path,
                    show=not self.gp_save_frames,
                )

        return erg_metric

    def run(
        self,
        agent_team: AgentTeam,
        num_steps: Optional[int] = None,
        verbose: bool = True,
    ) -> Dict[str, Any]:
        """
        Run HEDAC algorithm for specified number of steps.

        Args:
            agent_team: Team of agents
            num_steps: Number of steps (default from params)
            verbose: Whether to print progress

        Returns:
            Dictionary with results
        """
        if num_steps is None:
            num_steps = self.params.num_steps

        num_steps = int(num_steps or 0)  # Ensure it's an integer

        print_freq_val = self.params.get("output.print_frequency", 100)
        print_freq = int(print_freq_val) if print_freq_val is not None else 100

        for step in range(num_steps):
            if verbose and step % print_freq == 0:
                print(f"Step {step}/{num_steps}")

            erg_metric = self.step(agent_team, step_num=step)

            if verbose and step % print_freq == 0:
                print(f"  Ergodic metric: {erg_metric:.6f}")
                print(f"  GP dataset size: {self.dataset.shape[0]}")

        return {
            "ergodic_metrics": np.array(self.ergodic_metrics),
            "coverage_density": self.coverage_density.copy(),
            "heat_field": self.heat_field.copy(),
            "final_positions": agent_team.get_positions(),
            "trajectories": agent_team.get_histories(),
        }
