"""
Main HEDAC algorithm implementation.
"""

import numpy as np
from typing import Tuple, Optional, List, Dict, Any

from ..core.base import HEDACParams, MapLoader, compute_ergodic_metric
from ..utils.math_utils import (
    calculate_gradient,
    update_heat,
    update_heat_optimized,
    create_agent_block,
    clamp_kernel_1d,
    normalize_to_pdf,
    # compute_gradient_direction_v2,
    compute_gradient_direction,
)
from ..models.agents import DoubleIntegratorAgent, AgentTeam


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

    def _init_heat_field(self):
        """Initialize heat field from initial coverage."""
        # Start with uniform heat
        self.heat_field = np.ones_like(self.map, dtype=float)
        # Zero out obstacles
        self.heat_field[self.map == 1] = 0.0
        # Normalize
        self.heat_field = normalize_to_pdf(self.heat_field, self.map)

    def update_coverage(self, agent: DoubleIntegratorAgent):
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
        goal_norm = self.goal_density  # Assume already normalized

        # Difference between goal and coverage
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

    def get_agent_gradient(self, agent: DoubleIntegratorAgent) -> np.ndarray:
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

    def step(self, agent_team: AgentTeam) -> float:
        """
        Execute one step of HEDAC algorithm.

        Args:
            agent_team: Team of agents

        Returns:
            Current ergodic metric
        """
        # Reset local cooling
        self.local_cooling = np.zeros_like(self.map, dtype=float)

        # Update coverage from all agents
        for agent in agent_team.agents:
            self.update_coverage(agent)

        # Update heat field
        self.update_heat_field()

        # Compute ergodic metric
        coverage_norm = normalize_to_pdf(self.coverage_density, self.map)
        erg_metric = compute_ergodic_metric(coverage_norm, self.goal_density, self.map)
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
                v_target, theta_target, penalize_lateral=False
            )

            # Clip position to stay within map boundaries
            agent.clip_position(0, self.map.shape[1] - 1, 0, self.map.shape[0] - 1)

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

            erg_metric = self.step(agent_team)

            if verbose and step % print_freq == 0:
                print(f"  Ergodic metric: {erg_metric:.6f}")

        return {
            "ergodic_metrics": np.array(self.ergodic_metrics),
            "coverage_density": self.coverage_density.copy(),
            "heat_field": self.heat_field.copy(),
            "final_positions": agent_team.get_positions(),
            "trajectories": agent_team.get_histories(),
        }
