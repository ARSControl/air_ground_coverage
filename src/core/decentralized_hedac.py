"""Range-limited decentralized HEDAC coverage control.

The controller keeps one coverage memory and heat field per aerial robot.  It
uses a single inherited GP only for legacy compatibility; multifidelity mode
continues to supply the target from the simulation-owned central estimator.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .base import compute_ergodic_metric
from .hedac import HEDACAlgorithm
from ..models.agents import AgentLike, AgentTeam
from ..utils.math_utils import normalize_to_pdf


@dataclass
class _LocalHEDACState:
    coverage_density: np.ndarray
    aggregate_coverage_density: np.ndarray
    heat_field: np.ndarray
    coverage_updates: int = 0


class DecentralizedHEDACAlgorithm(HEDACAlgorithm):
    """Paper-inspired local-history, local-heat aerial coverage controller."""

    def __init__(self, params, map_loader, goal_density: np.ndarray) -> None:
        super().__init__(params, map_loader, goal_density)
        self.control_mode = "decentralized"
        weighting = params.get("ergodic_control.neighbor_weighting", "hard")
        if not isinstance(weighting, str) or weighting.strip().lower() != "hard":
            raise ValueError("ergodic_control.neighbor_weighting must be 'hard'")
        self.neighbor_weighting = "hard"
        self.communication_range = float(params.sens_range)
        if not np.isfinite(self.communication_range) or self.communication_range <= 0:
            raise ValueError("multi_agent.sensing_range must be positive")
        self._states: dict[int, _LocalHEDACState] = {}
        self._agent_ids: tuple[int, ...] | None = None
        self._neighbor_sets: tuple[tuple[int, ...], ...] = ()
        self.local_ergodic_metrics: list[np.ndarray] = []

        height, width = self.map.shape
        grid_x, grid_y = np.meshgrid(
            np.arange(width, dtype=float), np.arange(height, dtype=float)
        )
        self._grid_x = grid_x
        self._grid_y = grid_y

    @property
    def neighbor_sets(self) -> tuple[tuple[int, ...], ...]:
        """Current neighbor identifiers in the most recent team order."""
        return self._neighbor_sets

    @property
    def local_coverage_densities(self) -> tuple[np.ndarray, ...]:
        """Read-only snapshots of each robot's own running coverage density."""
        return self._state_snapshots("coverage_density")

    @property
    def local_aggregate_coverage_densities(self) -> tuple[np.ndarray, ...]:
        """Read-only snapshots after range-limited neighbor aggregation."""
        return self._state_snapshots("aggregate_coverage_density")

    @property
    def local_heat_fields(self) -> tuple[np.ndarray, ...]:
        """Read-only snapshots of the independent robot heat fields."""
        return self._state_snapshots("heat_field")

    def _state_snapshots(self, name: str) -> tuple[np.ndarray, ...]:
        if self._agent_ids is None:
            return ()
        snapshots = []
        for agent_id in self._agent_ids:
            value = np.array(getattr(self._states[agent_id], name), copy=True)
            value.setflags(write=False)
            snapshots.append(value)
        return tuple(snapshots)

    def _ensure_local_states(self, agents: list[AgentLike]) -> tuple[int, ...]:
        identifiers = tuple(int(getattr(agent, "id")) for agent in agents)
        if len(set(identifiers)) != len(identifiers):
            raise ValueError("decentralized HEDAC requires unique agent identifiers")
        if self._agent_ids is None:
            self._agent_ids = identifiers
            for agent_id in identifiers:
                self._states[agent_id] = _LocalHEDACState(
                    coverage_density=np.zeros_like(self.map, dtype=float),
                    aggregate_coverage_density=np.zeros_like(self.map, dtype=float),
                    heat_field=np.array(self.heat_field, dtype=float, copy=True),
                )
        elif set(identifiers) != set(self._agent_ids):
            raise ValueError("the decentralized aerial team cannot change after start")
        else:
            self._agent_ids = identifiers
        return identifiers

    def _update_own_coverage(self, agent: AgentLike, state: _LocalHEDACState) -> None:
        footprint = np.zeros_like(self.map, dtype=float)
        super().update_coverage(agent, coverage_density=footprint)
        count = state.coverage_updates
        state.coverage_density = (count * state.coverage_density + footprint) / float(
            count + 1
        )
        state.coverage_updates += 1

    def _current_neighbors(
        self, agents: list[AgentLike], identifiers: tuple[int, ...]
    ) -> tuple[tuple[int, ...], ...]:
        positions = np.asarray([agent.position for agent in agents], dtype=float)
        result: list[tuple[int, ...]] = []
        for index, agent in enumerate(agents):
            distances = np.linalg.norm(positions - positions[index], axis=1)
            neighbors = tuple(
                identifiers[other]
                for other in range(len(agents))
                if other != index and distances[other] <= self.communication_range
            )
            agent.neighbors = list(neighbors)
            result.append(neighbors)
        return tuple(result)

    def _aggregate_neighbor_coverage(
        self,
        agents: list[AgentLike],
        identifiers: tuple[int, ...],
        neighbor_sets: tuple[tuple[int, ...], ...],
    ) -> None:
        own_snapshots = {
            agent_id: np.array(state.coverage_density, copy=True)
            for agent_id, state in self._states.items()
        }
        radius_squared = self.communication_range**2
        for agent, agent_id, neighbors in zip(
            agents, identifiers, neighbor_sets, strict=True
        ):
            support = (self._grid_x - float(agent.position[0])) ** 2 + (
                self._grid_y - float(agent.position[1])
            ) ** 2 <= radius_squared
            aggregate = np.array(own_snapshots[agent_id], copy=True)
            for neighbor_id in neighbors:
                aggregate += support * own_snapshots[neighbor_id]
            self._states[agent_id].aggregate_coverage_density = aggregate

    def step(
        self,
        agent_team: AgentTeam,
        step_num: int = 0,
        *,
        external_goal_density: np.ndarray | None = None,
        update_legacy_gp: bool = True,
    ) -> float:
        """Advance every local controller synchronously by one simulation step."""
        if not isinstance(update_legacy_gp, (bool, np.bool_)):
            raise TypeError("update_legacy_gp must be boolean")
        external_target = None
        if external_goal_density is not None:
            external_target = self._validate_external_goal_density(
                external_goal_density
            )
        if update_legacy_gp:
            observations = self.collect_observations(agent_team)
            self.update_gp(observations, step_num=step_num)
        elif external_target is None:
            self.current_goal_density = self.goal_density.copy()
        if external_target is not None:
            self.current_goal_density = external_target

        agents = list(agent_team.agents)
        identifiers = self._ensure_local_states(agents)
        self.local_cooling = np.zeros_like(self.map, dtype=float)
        for agent, agent_id in zip(agents, identifiers, strict=True):
            self._update_own_coverage(agent, self._states[agent_id])

        neighbor_sets = self._current_neighbors(agents, identifiers)
        self._neighbor_sets = neighbor_sets
        self._aggregate_neighbor_coverage(agents, identifiers, neighbor_sets)

        commands: list[tuple[AgentLike, np.ndarray, float]] = []
        local_metrics = np.empty(len(agents), dtype=float)
        for index, (agent, agent_id) in enumerate(
            zip(agents, identifiers, strict=True)
        ):
            state = self._states[agent_id]
            source = self.compute_source_term(
                coverage_density=state.aggregate_coverage_density,
                goal_density=self.current_goal_density,
            )
            state.heat_field = self._advance_heat_field(state.heat_field, source)
            gradient = self.get_agent_gradient(agent, heat_field=state.heat_field)
            velocity = gradient * self.params.max_dx
            commands.append(
                (agent, velocity, float(np.arctan2(gradient[1], gradient[0])))
            )
            local_metrics[index] = compute_ergodic_metric(
                state.aggregate_coverage_density,
                normalize_to_pdf(self.current_goal_density, self.map),
                self.map,
            )

        if identifiers:
            self.coverage_density = np.sum(
                [self._states[agent_id].coverage_density for agent_id in identifiers],
                axis=0,
            )
            self.heat_field = np.mean(
                [self._states[agent_id].heat_field for agent_id in identifiers],
                axis=0,
            )
        else:
            self.coverage_density = np.zeros_like(self.map, dtype=float)
        team_metric = compute_ergodic_metric(
            self.coverage_density,
            normalize_to_pdf(self.goal_density, self.map),
            self.map,
        )
        self.local_ergodic_metrics.append(local_metrics)
        self.ergodic_metrics.append(team_metric)

        for agent, velocity, heading in commands:
            agent.track_velocity_and_heading(velocity, heading, penalize_lateral=True)
            agent.clip_position(0, self.map.shape[1] - 1, 0, self.map.shape[0] - 1)
        return team_metric


__all__ = ("DecentralizedHEDACAlgorithm",)
