"""
Decentralized HEDAC Algorithm Implementation

In this version, each agent maintains its own:
- Heat field
- Coverage density
- Local cooling field

Agents share information when within sensing range and use local cooling
to avoid collisions, similar to the approach in main.py.
"""

import numpy as np
from typing import Tuple, Optional, List, Dict, Any

from ..core.base import HEDACParams, MapLoader, compute_ergodic_metric
from ..utils.math_utils import (
    update_heat_optimized,
    create_agent_block,
    clamp_kernel_1d,
    normalize_to_pdf,
    compute_gradient_direction,
)
from ..models.agents import DoubleIntegratorAgent, AgentTeam


class DecentralizedHEDACAlgorithm:
    """
        Decentralized Heat Equation Driven Area Coverage (HEDAC) algorithm.

        Each agent maintains its own heat field, coverage density, and local cooling.
        When agents come within sensing range, they share coverage information and
    n    coordinate to avoid collisions through local cooling.

        This approach is inspired by the multi-agent coordination in main.py where
        agents share samples and use local cooling for collision avoidance.
    """

    def __init__(
        self, params: HEDACParams, map_loader: MapLoader, goal_density: np.ndarray
    ):
        """
        Initialize Decentralized HEDAC algorithm.

        Args:
            params: HEDAC parameters
            map_loader: Map loader instance
            goal_density: Target distribution to cover (should be normalized)
        """
        self.params = params
        self.map_loader = map_loader
        self.map = map_loader.load()
        self.goal_density = goal_density

        # Update params with actual map dimensions
        params.update_from_map(self.map.shape)

        # Create agent coverage block
        self.coverage_block = create_agent_block(
            params.nb_var, params.min_kernel_val, params.agent_radius
        )
        self.kernel_size = self.coverage_block.shape[0]
        self.half_kernel = self.kernel_size // 2

        # Per-agent state (will be initialized when agents are added)
        self.agent_heat_fields: Dict[int, np.ndarray] = {}
        self.agent_coverage_densities: Dict[int, np.ndarray] = {}
        self.agent_local_coolings: Dict[int, np.ndarray] = {}
        self.agent_ergodic_metrics: Dict[int, List[float]] = {}

        # Shared global coverage for visualization/analysis
        self.global_coverage_density = np.zeros_like(self.map, dtype=float)
        self.global_ergodic_metrics: List[float] = []

        # Neighbor tracking
        self.neighbor_pairs: List[Tuple[int, int]] = []

    def initialize_agents(self, agent_team: AgentTeam):
        """
        Initialize per-agent state for all agents.

        Args:
            agent_team: Team of agents
        """
        for agent in agent_team.agents:
            agent_id = agent.id

            # Initialize each agent's heat field
            heat_field = np.ones_like(self.map, dtype=float)
            heat_field[self.map == 1] = 0.0
            self.agent_heat_fields[agent_id] = normalize_to_pdf(heat_field, self.map)

            # Initialize coverage and local cooling
            self.agent_coverage_densities[agent_id] = np.zeros_like(
                self.map, dtype=float
            )
            self.agent_local_coolings[agent_id] = np.zeros_like(self.map, dtype=float)
            self.agent_ergodic_metrics[agent_id] = []

    def detect_neighbors(self, agent_team: AgentTeam) -> List[Tuple[int, int]]:
        """
        Detect neighbor pairs within sensing range.

        Args:
            agent_team: Team of agents

        Returns:
            List of neighbor pairs (agent_id1, agent_id2)
        """
        neighbor_pairs = []
        agents = agent_team.agents

        for i, agent1 in enumerate(agents):
            for j, agent2 in enumerate(agents):
                if i < j:  # Avoid duplicates
                    distance = np.linalg.norm(agent1.position - agent2.position)
                    if distance <= self.params.sens_range:
                        neighbor_pairs.append((agent1.id, agent2.id))

        self.neighbor_pairs = neighbor_pairs
        return neighbor_pairs

    def update_agent_coverage(self, agent: DoubleIntegratorAgent):
        """
        Update coverage density for a single agent.

        Args:
            agent: Agent to update
        """
        agent_id = agent.id
        x, y = agent.position.astype(int)

        # Clamp kernel to grid boundaries
        x_slice, x_start, x_num = clamp_kernel_1d(
            x, 0, self.map.shape[1], self.kernel_size
        )
        y_slice, y_start, y_num = clamp_kernel_1d(
            y, 0, self.map.shape[0], self.kernel_size
        )

        # Add coverage to agent's own coverage density
        self.agent_coverage_densities[agent_id][y_slice, x_slice] += (
            self.coverage_block[y_start : y_start + y_num, x_start : x_start + x_num]
        )

    def share_coverage_with_neighbors(self, agent_team: AgentTeam):
        """
        Share coverage density among neighboring agents.

        When agents are within sensing range, they share their coverage
        information to coordinate exploration.

        Args:
            agent_team: Team of agents
        """
        # For each neighbor pair, share coverage in the overlapping region
        for id1, id2 in self.neighbor_pairs:
            agent1 = agent_team.agents[id1]
            agent2 = agent_team.agents[id2]

            # Get positions
            pos1 = agent1.position.astype(int)
            pos2 = agent2.position.astype(int)

            # Define local sharing region around each agent
            share_radius = int(self.params.sens_range / 2)

            # Share coverage from agent1 to agent2's coverage density
            self._share_local_coverage(id1, id2, pos1, pos2, share_radius)

            # Share coverage from agent2 to agent1's coverage density
            self._share_local_coverage(id2, id1, pos2, pos1, share_radius)

    def _share_local_coverage(
        self,
        from_id: int,
        to_id: int,
        from_pos: np.ndarray,
        to_pos: np.ndarray,
        radius: int,
    ):
        """
        Share coverage from one agent to another in a local region.

        Args:
            from_id: Source agent ID
            to_id: Target agent ID
            from_pos: Source agent position
            to_pos: Target agent position
            radius: Sharing radius
        """
        # Get the region around the target agent
        x_min = max(0, to_pos[0] - radius)
        x_max = min(self.map.shape[1], to_pos[0] + radius + 1)
        y_min = max(0, to_pos[1] - radius)
        y_max = min(self.map.shape[0], to_pos[1] + radius + 1)

        # Share coverage from source agent's coverage density
        shared_coverage = self.agent_coverage_densities[from_id][
            y_min:y_max, x_min:x_max
        ]

        # Add to target agent's coverage (with some weighting)
        share_weight = 0.5  # Weight for shared coverage
        self.agent_coverage_densities[to_id][y_min:y_max, x_min:x_max] += (
            shared_coverage * share_weight
        )

    def compute_agent_source_term(self, agent_id: int) -> np.ndarray:
        """
        Compute source term for a specific agent's heat equation.

        Args:
            agent_id: Agent ID

        Returns:
            Source term field
        """
        # Normalize agent's coverage
        coverage_norm = normalize_to_pdf(
            self.agent_coverage_densities[agent_id], self.map
        )
        goal_norm = self.goal_density

        # Difference between goal and coverage
        diff = goal_norm - coverage_norm

        # Source is squared positive difference
        source = np.maximum(diff, 0) ** 2
        source[self.map == 1] = 0.0

        # Normalize and scale by area
        area = self.map_loader.area
        source = normalize_to_pdf(source, self.map) * area

        return source

    def update_agent_heat_field(self, agent_id: int):
        """
        Update heat field for a specific agent.

        Args:
            agent_id: Agent ID
        """
        source = self.compute_agent_source_term(agent_id)

        # Normalize local cooling
        local_cooling_norm = normalize_to_pdf(
            self.agent_local_coolings[agent_id], self.map
        )
        local_cooling_norm *= self.map_loader.area

        # Update heat equation
        self.agent_heat_fields[agent_id] = update_heat_optimized(
            self.agent_heat_fields[agent_id],
            source,
            self.map,
            local_cooling_norm,
            self.params.dt,
            self.params.alpha,
            self.params.source_strength,
            self.params.beta,
            self.params.local_cooling,
            self.params.dx,
        )

    def get_agent_gradient(self, agent: DoubleIntegratorAgent) -> np.ndarray:
        """
        Get movement gradient for an agent from its own heat field.

        Args:
            agent: Agent

        Returns:
            Gradient direction vector
        """
        # Compute gradients from heat field (matching old implementation)
        gradient_y, gradient_x = np.gradient(self.agent_heat_fields[agent.id].T, 1, 1)

        # Normalize gradients by magnitude (critical for proper movement)
        grad_mag = np.sqrt(np.mean(gradient_x**2) + np.mean(gradient_y**2))
        if grad_mag > 0:
            gradient_x /= grad_mag
            gradient_y /= grad_mag

        # Use improved gradient computation with proper wall avoidance
        from ..utils.math_utils import compute_gradient_direction_v2

        return compute_gradient_direction_v2(
            gradient_x,
            gradient_y,
            agent.position,
            self.map,
            wall_avoidance_weight=0.1,
            kernel_radius=self.half_kernel,
        )

    def apply_collision_avoidance(
        self, agent: DoubleIntegratorAgent, gradient: np.ndarray, agent_team: AgentTeam
    ) -> np.ndarray:
        """
        Apply collision avoidance through local cooling and gradient modification.

        This is inspired by the approach in main.py where agents use local
        cooling and gradient adjustments to avoid collisions.

        Args:
            agent: Current agent
            gradient: Current gradient direction
            agent_team: Team of agents

        Returns:
            Modified gradient with collision avoidance
        """
        modified_gradient = gradient.copy()

        # Check for neighbors within safe range
        for neighbor_id in [
            id2 for id1, id2 in self.neighbor_pairs if id1 == agent.id
        ] + [id1 for id1, id2 in self.neighbor_pairs if id2 == agent.id]:
            neighbor = agent_team.agents[neighbor_id]
            distance = np.linalg.norm(agent.position - neighbor.position)

            if distance < self.params.min_safe_range * 2:
                # Add local cooling around neighbor's position to repel current agent
                neighbor_pos = neighbor.position.astype(int)
                cooling_radius = int(self.params.min_safe_range * 2)

                x_min = max(0, neighbor_pos[0] - cooling_radius)
                x_max = min(self.map.shape[1], neighbor_pos[0] + cooling_radius + 1)
                y_min = max(0, neighbor_pos[1] - cooling_radius)
                y_max = min(self.map.shape[0], neighbor_pos[1] + cooling_radius + 1)

                # Add local cooling (higher near the neighbor)
                y_grid, x_grid = np.mgrid[y_min:y_max, x_min:x_max]
                dist_from_neighbor = np.sqrt(
                    (x_grid - neighbor_pos[0]) ** 2 + (y_grid - neighbor_pos[1]) ** 2
                )

                cooling_strength = np.exp(
                    -dist_from_neighbor / (self.params.min_safe_range)
                )
                self.agent_local_coolings[agent.id][y_min:y_max, x_min:x_max] += (
                    cooling_strength
                )

                # Modify gradient to move away from neighbor
                direction_away = agent.position - neighbor.position
                if np.linalg.norm(direction_away) > 0:
                    direction_away = direction_away / np.linalg.norm(direction_away)

                    # Add repulsive force to gradient
                    repulsion_strength = 2.0 * (
                        1.0 - distance / (self.params.min_safe_range * 2)
                    )
                    repulsion_strength = max(0, repulsion_strength)
                    modified_gradient += direction_away * repulsion_strength

        # Normalize modified gradient
        norm = np.linalg.norm(modified_gradient)
        if norm > 0:
            modified_gradient = modified_gradient / norm

        return modified_gradient

    def compute_global_metrics(self):
        """
        Compute global coverage and ergodic metric by combining all agents.

        Returns:
            Global ergodic metric
        """
        # Sum all agents' coverage
        self.global_coverage_density = np.zeros_like(self.map, dtype=float)
        for agent_id in self.agent_coverage_densities:
            self.global_coverage_density += self.agent_coverage_densities[agent_id]

        # Compute global ergodic metric
        global_erg_metric = compute_ergodic_metric(
            self.global_coverage_density, self.goal_density, self.map
        )
        self.global_ergodic_metrics.append(global_erg_metric)

        return global_erg_metric

    def step(self, agent_team: AgentTeam) -> float:
        """
        Execute one step of decentralized HEDAC algorithm.

        Args:
            agent_team: Team of agents

        Returns:
            Global ergodic metric
        """
        # Detect neighbors
        self.detect_neighbors(agent_team)

        # Update coverage for each agent
        for agent in agent_team.agents:
            self.update_agent_coverage(agent)

        # Share coverage among neighbors
        self.share_coverage_with_neighbors(agent_team)

        # Update heat fields and move agents
        for agent in agent_team.agents:
            agent_id = agent.id

            # Reset local cooling for this agent
            self.agent_local_coolings[agent_id] = np.zeros_like(self.map, dtype=float)

            # Update heat field
            self.update_agent_heat_field(agent_id)

            # Get gradient from agent's own heat field
            gradient = self.get_agent_gradient(agent)

            # Apply collision avoidance
            gradient = self.apply_collision_avoidance(agent, gradient, agent_team)

            # Compute target velocity and heading
            k_target = 2.0
            v_target = k_target * gradient * self.params.max_dx
            theta_target = np.arctan2(gradient[1], gradient[0])

            # Move agent
            agent.track_velocity_and_heading(
                v_target, theta_target, penalize_lateral=True
            )

            # Clip position to stay within map boundaries
            agent.clip_position(0, self.map.shape[1] - 1, 0, self.map.shape[0] - 1)

            # Compute individual agent's ergodic metric
            agent_coverage_norm = normalize_to_pdf(
                self.agent_coverage_densities[agent_id], self.map
            )
            agent_erg = compute_ergodic_metric(
                agent_coverage_norm, self.goal_density, self.map
            )
            self.agent_ergodic_metrics[agent_id].append(agent_erg)

        # Compute global metric
        global_erg_metric = self.compute_global_metrics()

        return global_erg_metric

    def run(
        self,
        agent_team: AgentTeam,
        num_steps: Optional[int] = None,
        verbose: bool = True,
    ) -> Dict[str, Any]:
        """
        Run decentralized HEDAC algorithm.

        Args:
            agent_team: Team of agents
            num_steps: Number of steps (default from params)
            verbose: Whether to print progress

        Returns:
            Dictionary with results
        """
        if num_steps is None:
            num_steps = self.params.num_steps

        num_steps = int(num_steps)

        # Initialize agents
        self.initialize_agents(agent_team)

        print_freq = self.params.get("output.print_frequency", 100)

        for step in range(num_steps):
            if verbose and step % print_freq == 0:
                print(f"Step {step}/{num_steps}")

            global_erg_metric = self.step(agent_team)

            if verbose and step % print_freq == 0:
                print(f"  Global ergodic metric: {global_erg_metric:.6f}")
                if self.neighbor_pairs:
                    print(f"  Active neighbor pairs: {len(self.neighbor_pairs)}")

        return {
            "global_ergodic_metrics": np.array(self.global_ergodic_metrics),
            "agent_ergodic_metrics": {
                aid: np.array(metrics)
                for aid, metrics in self.agent_ergodic_metrics.items()
            },
            "global_coverage_density": self.global_coverage_density.copy(),
            "agent_coverage_densities": {
                aid: cov.copy() for aid, cov in self.agent_coverage_densities.items()
            },
            "agent_heat_fields": {
                aid: heat.copy() for aid, heat in self.agent_heat_fields.items()
            },
            "final_positions": agent_team.get_positions(),
            "trajectories": agent_team.get_histories(),
        }
