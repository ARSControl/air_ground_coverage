"""Distributed air-ground coverage baseline from Zhang et al. (T-ASE 2026).

The implementation is deliberately independent of the production estimator and
robot classes.  Communication is represented by immutable neighbor snapshots in
one deterministic Python process; no networking or real concurrency is used.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from time import perf_counter
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class ZhangParameters:
    """Numerical parameters for Algorithms 1 and 2."""

    dt: float = 0.05
    deployment_gain: float = 0.4
    deployment_tolerance: float = 1.0e-3
    deployment_max_steps: int = 1000
    consensus_gain: float = 0.1
    consensus_gain_fraction: float | None = None
    consensus_iterations: int = 100
    weight_gain: float = 0.09
    aerial_gain: float = 0.1
    transfer_gain: float = 0.2
    ground_gain: float = 0.4
    ground_sensing_range: float = 0.25
    aerial_sensing_range: float | None = None

    def __post_init__(self) -> None:
        positive_reals = (
            "dt",
            "deployment_gain",
            "deployment_tolerance",
            "consensus_gain",
            "weight_gain",
            "aerial_gain",
            "transfer_gain",
            "ground_gain",
            "ground_sensing_range",
        )
        for name in positive_reals:
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(float(value)) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("deployment_max_steps", "consensus_iterations"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        if self.consensus_gain_fraction is not None:
            value = self.consensus_gain_fraction
            if (
                isinstance(value, bool)
                or not math.isfinite(float(value))
                or not 0.0 < value < 1.0
            ):
                raise ValueError("consensus_gain_fraction must lie strictly in (0, 1)")
        if self.aerial_sensing_range is not None:
            value = self.aerial_sensing_range
            if (
                isinstance(value, bool)
                or not math.isfinite(float(value))
                or value <= 0.0
            ):
                raise ValueError("aerial_sensing_range must be finite and positive")


@dataclass(frozen=True)
class TargetAllocation:
    """Result of the paper's distributed load and mass calculation."""

    target_loads: np.ndarray
    ratios: np.ndarray
    target_masses: np.ndarray
    load_history: np.ndarray
    ratio_history: np.ndarray


@dataclass(frozen=True)
class ZhangStepResult:
    """State diagnostics published after one Algorithm 2 integration step."""

    aerial_commands: np.ndarray
    ground_commands: np.ndarray
    cell_masses: np.ndarray
    cell_loads: np.ndarray
    ground_parents: np.ndarray
    transferred_ground: np.ndarray
    aerial_support_fractions: np.ndarray
    phase: str


def uniform_grid(
    bounds: Sequence[float], grid_shape: Sequence[int]
) -> tuple[np.ndarray, np.ndarray, tuple[int, int]]:
    """Return a row-major Cartesian grid and trapezoidal integration weights."""
    if len(bounds) != 4:
        raise ValueError("bounds must be (x_min, x_max, y_min, y_max)")
    if len(grid_shape) != 2:
        raise ValueError("grid_shape must be (rows, columns)")
    x_min, x_max, y_min, y_max = (float(value) for value in bounds)
    rows, columns = (int(value) for value in grid_shape)
    if not (x_max > x_min and y_max > y_min and rows >= 2 and columns >= 2):
        raise ValueError("bounds must be increasing and grid dimensions at least two")
    xs = np.linspace(x_min, x_max, columns)
    ys = np.linspace(y_min, y_max, rows)
    x_grid, y_grid = np.meshgrid(xs, ys)
    points = np.column_stack((x_grid.ravel(), y_grid.ravel()))
    x_weights = np.ones(columns)
    y_weights = np.ones(rows)
    x_weights[[0, -1]] = 0.5
    y_weights[[0, -1]] = 0.5
    weights = (
        np.outer(y_weights, x_weights)
        * (x_max - x_min)
        / (columns - 1)
        * (y_max - y_min)
        / (rows - 1)
    )
    return points, weights.ravel(), (rows, columns)


def paper_scalar_fields(
    points: np.ndarray,
    *,
    low_resolution: float = 1.0e-2,
    high_resolution: float = 1.0e-5,
) -> tuple[np.ndarray, np.ndarray]:
    """Evaluate and quantize the three-source field used by Zhang et al."""
    query = _points(points)
    x = query[:, 0]
    y = query[:, 1]
    field = (
        1.2 * np.exp(-((x + 1.0) ** 2) - ((y + 1.0) ** 2))
        + 1.2 * np.exp(-((x + 1.0) ** 2) - ((y - 1.0) ** 2))
        + np.exp(-((x - 1.0) ** 2) - y**2)
    )
    low = _quantize(field, low_resolution)
    high = _quantize(field, high_resolution)
    return low, high


def power_diagram_labels(
    points: np.ndarray, positions: np.ndarray, cell_weights: np.ndarray
) -> np.ndarray:
    """Assign grid points using ``||q-p_i||^2-w_i`` with stable ties."""
    query = _points(points)
    sites = _positions(positions, "positions")
    weights = np.asarray(cell_weights, dtype=float)
    if weights.shape != (sites.shape[0],) or not np.all(np.isfinite(weights)):
        raise ValueError("cell_weights must be finite and match positions")
    power_distance = (
        np.sum((query[:, None, :] - sites[None, :, :]) ** 2, axis=2) - weights[None, :]
    )
    return np.argmin(power_distance, axis=1).astype(np.int64)


def cell_adjacency(
    labels: np.ndarray, grid_shape: Sequence[int], cell_count: int
) -> np.ndarray:
    """Find power-cell neighbors sharing a raster edge (4-connectivity)."""
    rows, columns = (int(value) for value in grid_shape)
    values = np.asarray(labels, dtype=np.int64)
    if values.shape != (rows * columns,):
        raise ValueError("labels do not match grid_shape")
    if cell_count < 1 or np.any((values < 0) | (values >= cell_count)):
        raise ValueError("labels must identify configured cells")
    image = values.reshape(rows, columns)
    adjacency = np.zeros((cell_count, cell_count), dtype=bool)
    for first, second in (
        (image[:, :-1].ravel(), image[:, 1:].ravel()),
        (image[:-1, :].ravel(), image[1:, :].ravel()),
    ):
        boundary = first != second
        adjacency[first[boundary], second[boundary]] = True
        adjacency[second[boundary], first[boundary]] = True
    np.fill_diagonal(adjacency, False)
    return adjacency


def cell_statistics(
    points: np.ndarray,
    integration_weights: np.ndarray,
    density: np.ndarray,
    labels: np.ndarray,
    cell_count: int,
    *,
    fallback_positions: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute raster quadrature masses and weighted centroids per cell."""
    query = _points(points)
    quadrature = _vector(integration_weights, query.shape[0], "integration_weights")
    values = _vector(density, query.shape[0], "density")
    assignment = np.asarray(labels, dtype=np.int64)
    fallback = _positions(fallback_positions, "fallback_positions")
    if assignment.shape != (query.shape[0],) or fallback.shape[0] != cell_count:
        raise ValueError("labels or fallback_positions do not match cell_count")
    masses = np.zeros(cell_count, dtype=float)
    centroids = fallback.copy()
    weighted_density = quadrature * values
    for index in range(cell_count):
        selected = assignment == index
        mass = float(np.sum(weighted_density[selected], dtype=float))
        masses[index] = mass
        if mass > 0.0:
            centroids[index] = (
                np.sum(query[selected] * weighted_density[selected, None], axis=0)
                / mass
            )
    return masses, centroids


def distributed_target_allocation(
    initial_loads: np.ndarray,
    initial_masses: np.ndarray,
    adjacency: np.ndarray,
    *,
    gain: float,
    iterations: int,
    rng: np.random.Generator,
) -> TargetAllocation:
    """Execute Algorithm 1 with simultaneous neighbor snapshots."""
    loads = np.asarray(initial_loads, dtype=np.int64).copy()
    masses = np.asarray(initial_masses, dtype=float)
    graph = np.asarray(adjacency, dtype=bool)
    count = loads.size
    if loads.shape != masses.shape or graph.shape != (count, count):
        raise ValueError("loads, masses, and adjacency have incompatible shapes")
    if np.any(loads < 0) or int(np.sum(loads)) < 1:
        raise ValueError("initial loads must be nonnegative with positive total")
    if np.any(~np.isfinite(masses)) or np.any(masses <= 0.0):
        raise ValueError("initial cell masses must be finite and positive")
    if not np.array_equal(graph, graph.T) or np.any(np.diag(graph)):
        raise ValueError("adjacency must be symmetric with a false diagonal")
    if not _connected(graph):
        raise ValueError("aerial power-cell neighbor graph must be connected")
    if (
        isinstance(iterations, bool)
        or not isinstance(iterations, int)
        or iterations < 1
    ):
        raise ValueError("iterations must be a positive integer")
    degree = np.sum(graph, axis=1)
    stability_limit = float(np.min(masses / degree))
    if not (math.isfinite(float(gain)) and 0.0 < gain < stability_limit):
        raise ValueError(
            f"gain must satisfy 0 < gain < min(s_i/degree_i)={stability_limit:.6g}"
        )

    ratios = loads.astype(float) / masses
    load_history = [loads.copy()]
    ratio_history = [ratios.copy()]
    for _ in range(iterations):
        snapshot_loads = loads.copy()
        offers = np.full(count, -1, dtype=np.int64)
        for source in range(count):
            neighbors = np.flatnonzero(graph[source])
            minimum = np.min(snapshot_loads[neighbors])
            candidates = neighbors[snapshot_loads[neighbors] == minimum]
            target = int(rng.choice(candidates))
            if snapshot_loads[source] > snapshot_loads[target]:
                offers[source] = target

        delta = np.zeros(count, dtype=np.int64)
        for receiver in range(count):
            offerers = np.flatnonzero(offers == receiver)
            if offerers.size == 0:
                continue
            maximum = np.max(snapshot_loads[offerers])
            candidates = offerers[snapshot_loads[offerers] == maximum]
            sender = int(rng.choice(candidates))
            amount = int(
                math.ceil((snapshot_loads[sender] - snapshot_loads[receiver]) / 2.0)
            )
            if amount > 0:
                delta[sender] -= amount
                delta[receiver] += amount
        loads += delta
        ratios = ratios + gain / masses * (
            graph.astype(float) @ ratios - degree * ratios
        )
        load_history.append(loads.copy())
        ratio_history.append(ratios.copy())

    if np.any(ratios <= 0.0):
        raise RuntimeError("consensus did not produce positive load-to-mass ratios")
    return TargetAllocation(
        target_loads=loads.copy(),
        ratios=ratios.copy(),
        target_masses=loads / ratios,
        load_history=np.asarray(load_history, dtype=np.int64),
        ratio_history=np.asarray(ratio_history, dtype=float),
    )


def limited_ground_centroids(
    points: np.ndarray,
    integration_weights: np.ndarray,
    high_field: np.ndarray,
    aerial_labels: np.ndarray,
    ground_positions: np.ndarray,
    ground_parents: np.ndarray,
    *,
    sensing_range: float,
) -> np.ndarray:
    """Compute Eq. (14) centroids in parent cell, local Voronoi cell, and disk."""
    query = _points(points)
    quadrature = _vector(integration_weights, query.shape[0], "integration_weights")
    field = _vector(high_field, query.shape[0], "high_field")
    ground = _positions(ground_positions, "ground_positions")
    parents = np.asarray(ground_parents, dtype=np.int64)
    parent_cells = np.asarray(aerial_labels, dtype=np.int64)
    if parents.shape != (ground.shape[0],) or parent_cells.shape != (query.shape[0],):
        raise ValueError("ground parents or aerial labels have invalid shape")
    if not math.isfinite(float(sensing_range)) or sensing_range <= 0.0:
        raise ValueError("sensing_range must be finite and positive")
    centroids = ground.copy()
    range_squared = sensing_range**2
    for parent in np.unique(parents):
        robots = np.flatnonzero(parents == parent)
        cells = np.flatnonzero(parent_cells == parent)
        if robots.size == 0 or cells.size == 0:
            continue
        distances = np.sum(
            (query[cells, None, :] - ground[None, robots, :]) ** 2, axis=2
        )
        closest = np.argmin(distances, axis=1)
        for local_index, robot in enumerate(robots):
            selected_local = (closest == local_index) & (
                distances[:, local_index] <= range_squared
            )
            selected = cells[selected_local]
            weighted = quadrature[selected] * field[selected]
            mass = float(np.sum(weighted, dtype=float))
            if mass > 0.0:
                centroids[robot] = (
                    np.sum(query[selected] * weighted[:, None], axis=0) / mass
                )
    return centroids


class ZhangSimulation:
    """Sequential, logically distributed implementation of Algorithms 1 and 2."""

    def __init__(
        self,
        *,
        points: np.ndarray,
        integration_weights: np.ndarray,
        grid_shape: Sequence[int],
        low_field: np.ndarray,
        high_field: np.ndarray,
        aerial_positions: np.ndarray,
        ground_positions: np.ndarray,
        parameters: ZhangParameters,
        rng: np.random.Generator,
        deployment_within_mission: bool = False,
    ) -> None:
        self.points = _points(points).copy()
        self.integration_weights = _vector(
            integration_weights, self.points.shape[0], "integration_weights"
        ).copy()
        self.grid_shape = (int(grid_shape[0]), int(grid_shape[1]))
        if np.prod(self.grid_shape) != self.points.shape[0]:
            raise ValueError("grid_shape does not match points")
        self.low_field = _vector(low_field, self.points.shape[0], "low_field").copy()
        self.high_field = _vector(high_field, self.points.shape[0], "high_field").copy()
        self.deployment_initial_positions = _positions(
            aerial_positions, "aerial_positions"
        ).copy()
        self.aerial_positions = self.deployment_initial_positions.copy()
        self.ground_positions = _positions(ground_positions, "ground_positions").copy()
        if self.aerial_positions.shape[0] <= 1:
            raise ValueError("Zhang baseline requires more than one aerial robot")
        if self.ground_positions.shape[0] <= self.aerial_positions.shape[0]:
            raise ValueError("Zhang baseline requires more ground than aerial robots")
        self.parameters = parameters
        self.rng = rng
        if not isinstance(deployment_within_mission, bool):
            raise TypeError("deployment_within_mission must be a boolean")
        self.deployment_within_mission = deployment_within_mission
        self.bounds = (
            float(np.min(self.points[:, 0])),
            float(np.max(self.points[:, 0])),
            float(np.min(self.points[:, 1])),
            float(np.max(self.points[:, 1])),
        )
        self.cell_weights = np.zeros(self.aerial_positions.shape[0], dtype=float)
        self.deployment_steps = 0
        self.time = 0.0
        self.phase_transition_time: float | None = None
        self.phase = "deployment" if deployment_within_mission else "coverage"
        self.allocation: TargetAllocation | None = None
        self.consensus_gain_used: float | None = None
        self.last_consensus_duration = 0.0
        self.total_consensus_duration = 0.0
        if not deployment_within_mission:
            self.deployment_steps = self._deploy_aerial_team()
        labels = power_diagram_labels(
            self.points, self.aerial_positions, self.cell_weights
        )
        masses, _ = self._low_cell_statistics(labels)
        parents = power_diagram_labels(
            self.ground_positions, self.aerial_positions, self.cell_weights
        )
        loads = np.bincount(parents, minlength=self.aerial_positions.shape[0])
        self.cell_masses = masses
        self.aerial_support_fractions = self._aerial_support(labels)
        self.ground_parents = parents
        self.cell_loads = loads
        if not deployment_within_mission:
            self._initialize_allocation(labels, masses, loads)
            self.phase_transition_time = 0.0

    @property
    def target_loads(self) -> np.ndarray:
        if self.allocation is None:
            raise RuntimeError("target loads are unavailable during aerial deployment")
        return self.allocation.target_loads

    @property
    def target_masses(self) -> np.ndarray:
        if self.allocation is None:
            raise RuntimeError("target masses are unavailable during aerial deployment")
        return self.allocation.target_masses

    def step(self) -> ZhangStepResult:
        """Advance the active mission phase by one physical timestep."""
        if self.phase == "deployment":
            return self._step_mission_deployment()
        return self._step_coverage()

    def _step_coverage(self) -> ZhangStepResult:
        """Advance Algorithm 2 once using one immutable state snapshot."""
        aerial_count = self.aerial_positions.shape[0]
        labels = power_diagram_labels(
            self.points, self.aerial_positions, self.cell_weights
        )
        masses, _ = self._low_cell_statistics(labels)
        adjacency = cell_adjacency(labels, self.grid_shape, aerial_count)
        mass_errors = self.target_masses - masses
        aerial_commands = self.parameters.aerial_gain * _cell_adjustment_direction(
            self.points,
            self.grid_shape,
            self.low_field,
            labels,
            self.aerial_positions,
            mass_errors,
            sensing_range=self.parameters.aerial_sensing_range,
        )
        weight_commands = self.parameters.weight_gain * mass_errors

        parents = power_diagram_labels(
            self.ground_positions, self.aerial_positions, self.cell_weights
        )
        loads = np.bincount(parents, minlength=aerial_count)
        centroids = limited_ground_centroids(
            self.points,
            self.integration_weights,
            self.high_field,
            labels,
            self.ground_positions,
            parents,
            sensing_range=self.parameters.ground_sensing_range,
        )
        ground_commands = self.parameters.ground_gain * (
            centroids - self.ground_positions
        )
        transferred = self._apply_guidance(adjacency, loads, parents, ground_commands)

        dt = self.parameters.dt
        self.aerial_positions = _clip_positions(
            self.aerial_positions + dt * aerial_commands, self.bounds
        )
        self.ground_positions = _clip_positions(
            self.ground_positions + dt * ground_commands, self.bounds
        )
        self.cell_weights = self.cell_weights + dt * weight_commands
        self.time += dt

        new_labels = power_diagram_labels(
            self.points, self.aerial_positions, self.cell_weights
        )
        self.cell_masses, _ = self._low_cell_statistics(new_labels)
        self.aerial_support_fractions = self._aerial_support(new_labels)
        self.ground_parents = power_diagram_labels(
            self.ground_positions, self.aerial_positions, self.cell_weights
        )
        self.cell_loads = np.bincount(self.ground_parents, minlength=aerial_count)
        return ZhangStepResult(
            aerial_commands=aerial_commands,
            ground_commands=ground_commands,
            cell_masses=self.cell_masses.copy(),
            cell_loads=self.cell_loads.copy(),
            ground_parents=self.ground_parents.copy(),
            transferred_ground=transferred,
            aerial_support_fractions=self.aerial_support_fractions.copy(),
            phase="coverage",
        )

    def locational_cost(self) -> float:
        """Evaluate Eq. (5) on the raster using the nearest ground robot."""
        distances = np.sum(
            (self.points[:, None, :] - self.ground_positions[None, :, :]) ** 2,
            axis=2,
        )
        return float(
            np.sum(
                np.min(distances, axis=1) * self.high_field * self.integration_weights,
                dtype=float,
            )
        )

    def _deploy_aerial_team(self) -> int:
        count = self.aerial_positions.shape[0]
        for step in range(1, self.parameters.deployment_max_steps + 1):
            labels = power_diagram_labels(
                self.points, self.aerial_positions, np.zeros(count)
            )
            _, centroids = cell_statistics(
                self.points,
                self.integration_weights,
                np.ones(self.points.shape[0]),
                labels,
                count,
                fallback_positions=self.aerial_positions,
            )
            error = centroids - self.aerial_positions
            if float(np.max(np.linalg.norm(error, axis=1))) <= (
                self.parameters.deployment_tolerance
            ):
                return step - 1
            self.aerial_positions = _clip_positions(
                self.aerial_positions
                + self.parameters.dt * self.parameters.deployment_gain * error,
                self.bounds,
            )
        return self.parameters.deployment_max_steps

    def _step_mission_deployment(self) -> ZhangStepResult:
        """Advance Algorithm 1 deployment while holding every ground robot fixed."""
        count = self.aerial_positions.shape[0]
        labels = power_diagram_labels(
            self.points, self.aerial_positions, np.zeros(count)
        )
        _, centroids = cell_statistics(
            self.points,
            self.integration_weights,
            np.ones(self.points.shape[0]),
            labels,
            count,
            fallback_positions=self.aerial_positions,
        )
        error = centroids - self.aerial_positions
        if float(np.max(np.linalg.norm(error, axis=1))) <= (
            self.parameters.deployment_tolerance
        ):
            self._refresh_diagnostics()
            self._initialize_allocation(
                power_diagram_labels(
                    self.points, self.aerial_positions, self.cell_weights
                ),
                self.cell_masses,
                self.cell_loads,
            )
            self.phase = "coverage"
            self.phase_transition_time = self.time
            return self._step_coverage()

        aerial_commands = self.parameters.deployment_gain * error
        self.aerial_positions = _clip_positions(
            self.aerial_positions + self.parameters.dt * aerial_commands,
            self.bounds,
        )
        self.time += self.parameters.dt
        self.deployment_steps += 1
        self._refresh_diagnostics()

        # The transition is timestamped at the first saved state satisfying the
        # deployment criterion.  No ground command is applied over this step.
        updated_labels = power_diagram_labels(
            self.points, self.aerial_positions, np.zeros(count)
        )
        _, updated_centroids = cell_statistics(
            self.points,
            self.integration_weights,
            np.ones(self.points.shape[0]),
            updated_labels,
            count,
            fallback_positions=self.aerial_positions,
        )
        if (
            float(
                np.max(
                    np.linalg.norm(updated_centroids - self.aerial_positions, axis=1)
                )
            )
            <= self.parameters.deployment_tolerance
        ):
            self._initialize_allocation(
                power_diagram_labels(
                    self.points, self.aerial_positions, self.cell_weights
                ),
                self.cell_masses,
                self.cell_loads,
            )
            self.phase = "coverage"
            self.phase_transition_time = self.time

        return ZhangStepResult(
            aerial_commands=aerial_commands,
            ground_commands=np.zeros_like(self.ground_positions),
            cell_masses=self.cell_masses.copy(),
            cell_loads=self.cell_loads.copy(),
            ground_parents=self.ground_parents.copy(),
            transferred_ground=np.empty(0, dtype=np.int64),
            aerial_support_fractions=self.aerial_support_fractions.copy(),
            phase=self.phase,
        )

    def _refresh_diagnostics(self) -> None:
        count = self.aerial_positions.shape[0]
        labels = power_diagram_labels(
            self.points, self.aerial_positions, self.cell_weights
        )
        self.cell_masses, _ = self._low_cell_statistics(labels)
        self.aerial_support_fractions = self._aerial_support(labels)
        self.ground_parents = power_diagram_labels(
            self.ground_positions, self.aerial_positions, self.cell_weights
        )
        self.cell_loads = np.bincount(self.ground_parents, minlength=count)

    def _low_cell_statistics(self, labels: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        density = self.low_field
        sensing_range = self.parameters.aerial_sensing_range
        if sensing_range is not None:
            distances = np.linalg.norm(
                self.points[:, None, :] - self.aerial_positions[None, :, :], axis=2
            )
            visible = distances <= sensing_range
            density = np.where(
                visible[np.arange(self.points.shape[0]), labels], density, 0.0
            )
        return cell_statistics(
            self.points,
            self.integration_weights,
            density,
            labels,
            self.aerial_positions.shape[0],
            fallback_positions=self.aerial_positions,
        )

    def _aerial_support(self, labels: np.ndarray) -> np.ndarray:
        sensing_range = self.parameters.aerial_sensing_range
        if sensing_range is None:
            return np.ones(self.aerial_positions.shape[0], dtype=float)
        distances = np.linalg.norm(
            self.points[:, None, :] - self.aerial_positions[None, :, :], axis=2
        )
        support = np.empty(self.aerial_positions.shape[0], dtype=float)
        for index in range(self.aerial_positions.shape[0]):
            selected = labels == index
            area = float(np.sum(self.integration_weights[selected], dtype=float))
            sensed = selected & (distances[:, index] <= sensing_range)
            sensed_area = float(np.sum(self.integration_weights[sensed], dtype=float))
            support[index] = sensed_area / area if area > 0.0 else 0.0
        return support

    def _initialize_allocation(
        self,
        labels: np.ndarray,
        masses: np.ndarray,
        loads: np.ndarray,
    ) -> None:
        adjacency = cell_adjacency(
            labels, self.grid_shape, self.aerial_positions.shape[0]
        )
        gain = self.parameters.consensus_gain
        if self.parameters.consensus_gain_fraction is not None:
            degree = np.sum(adjacency, axis=1)
            if np.any(degree == 0):
                raise ValueError(
                    "aerial power-cell neighbor graph must have no isolated cells"
                )
            gain = self.parameters.consensus_gain_fraction * float(
                np.min(masses / degree)
            )
        self.consensus_gain_used = gain
        started = perf_counter()
        self.allocation = distributed_target_allocation(
            loads,
            masses,
            adjacency,
            gain=gain,
            iterations=self.parameters.consensus_iterations,
            rng=self.rng,
        )
        self.last_consensus_duration = perf_counter() - started
        self.total_consensus_duration += self.last_consensus_duration

    def _apply_guidance(
        self,
        adjacency: np.ndarray,
        loads: np.ndarray,
        parents: np.ndarray,
        commands: np.ndarray,
    ) -> np.ndarray:
        imbalance = loads - self.target_loads
        aerial_count = loads.size
        offers = np.full(aerial_count, -1, dtype=np.int64)
        for source in range(aerial_count):
            neighbors = np.flatnonzero(adjacency[source])
            if neighbors.size == 0:
                continue
            minimum = np.min(imbalance[neighbors])
            candidates = neighbors[imbalance[neighbors] == minimum]
            target = int(self.rng.choice(candidates))
            if imbalance[source] > imbalance[target]:
                offers[source] = target
        transferred: list[int] = []
        for receiver in range(aerial_count):
            offerers = np.flatnonzero(offers == receiver)
            if offerers.size == 0:
                continue
            maximum = np.max(imbalance[offerers])
            candidates = offerers[imbalance[offerers] == maximum]
            sender = int(self.rng.choice(candidates))
            robots = np.flatnonzero(parents == sender)
            if robots.size == 0:
                continue
            distances = np.linalg.norm(
                self.ground_positions[robots] - self.aerial_positions[receiver],
                axis=1,
            )
            robot = int(robots[np.argmin(distances)])
            commands[robot] = self.parameters.transfer_gain * (
                self.aerial_positions[receiver] - self.ground_positions[robot]
            )
            transferred.append(robot)
        return np.asarray(transferred, dtype=np.int64)


def _cell_adjustment_direction(
    points: np.ndarray,
    grid_shape: tuple[int, int],
    low_field: np.ndarray,
    labels: np.ndarray,
    positions: np.ndarray,
    mass_errors: np.ndarray,
    *,
    sensing_range: float | None = None,
) -> np.ndarray:
    """Raster face-quadrature approximation of the boundary integral in Eq. (8)."""
    rows, columns = grid_shape
    point_image = points.reshape(rows, columns, 2)
    field_image = low_field.reshape(rows, columns)
    label_image = labels.reshape(rows, columns)
    dx = float(point_image[0, 1, 0] - point_image[0, 0, 0])
    dy = float(point_image[1, 0, 1] - point_image[0, 0, 1])
    direction = np.zeros_like(positions)

    def accumulate(
        first_labels: np.ndarray,
        second_labels: np.ndarray,
        first_points: np.ndarray,
        second_points: np.ndarray,
        first_field: np.ndarray,
        second_field: np.ndarray,
        face_length: float,
    ) -> None:
        boundary = first_labels != second_labels
        for first, second, midpoint, density in zip(
            first_labels[boundary],
            second_labels[boundary],
            0.5 * (first_points[boundary] + second_points[boundary]),
            0.5 * (first_field[boundary] + second_field[boundary]),
            strict=True,
        ):
            separation = float(np.linalg.norm(positions[first] - positions[second]))
            if separation <= 0.0:
                continue
            scale = float(density) * face_length / separation
            if (
                sensing_range is None
                or np.linalg.norm(midpoint - positions[first]) <= sensing_range
            ):
                direction[first] += (
                    (mass_errors[first] - mass_errors[second])
                    * scale
                    * (midpoint - positions[first])
                )
            if (
                sensing_range is None
                or np.linalg.norm(midpoint - positions[second]) <= sensing_range
            ):
                direction[second] += (
                    (mass_errors[second] - mass_errors[first])
                    * scale
                    * (midpoint - positions[second])
                )

    accumulate(
        label_image[:, :-1].ravel(),
        label_image[:, 1:].ravel(),
        point_image[:, :-1].reshape(-1, 2),
        point_image[:, 1:].reshape(-1, 2),
        field_image[:, :-1].ravel(),
        field_image[:, 1:].ravel(),
        dy,
    )
    accumulate(
        label_image[:-1, :].ravel(),
        label_image[1:, :].ravel(),
        point_image[:-1, :].reshape(-1, 2),
        point_image[1:, :].reshape(-1, 2),
        field_image[:-1, :].ravel(),
        field_image[1:, :].ravel(),
        dx,
    )
    return direction


def _connected(adjacency: np.ndarray) -> bool:
    visited = {0}
    frontier = [0]
    while frontier:
        node = frontier.pop()
        for neighbor in np.flatnonzero(adjacency[node]):
            if int(neighbor) not in visited:
                visited.add(int(neighbor))
                frontier.append(int(neighbor))
    return len(visited) == adjacency.shape[0]


def _quantize(values: np.ndarray, resolution: float) -> np.ndarray:
    if not math.isfinite(float(resolution)) or resolution <= 0.0:
        raise ValueError("sensor resolution must be finite and positive")
    return np.round(values / resolution) * resolution


def _points(values: np.ndarray) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if result.ndim != 2 or result.shape[1] != 2 or not np.all(np.isfinite(result)):
        raise ValueError("points must have shape (N, 2) and be finite")
    return result


def _positions(values: np.ndarray, name: str) -> np.ndarray:
    result = _points(values)
    if result.shape[0] < 1:
        raise ValueError(f"{name} must contain at least one position")
    return result


def _vector(values: np.ndarray, length: int, name: str) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite vector of length {length}")
    if name in {"integration_weights", "low_field", "high_field", "density"} and np.any(
        result < 0.0
    ):
        raise ValueError(f"{name} must be nonnegative")
    if name == "integration_weights" and np.any(result <= 0.0):
        raise ValueError("integration_weights must be positive")
    return result


def _clip_positions(positions: np.ndarray, bounds: Sequence[float]) -> np.ndarray:
    result = np.asarray(positions, dtype=float).copy()
    result[:, 0] = np.clip(result[:, 0], bounds[0], bounds[1])
    result[:, 1] = np.clip(result[:, 1], bounds[2], bounds[3])
    return result


__all__ = (
    "TargetAllocation",
    "ZhangParameters",
    "ZhangSimulation",
    "ZhangStepResult",
    "cell_adjacency",
    "cell_statistics",
    "distributed_target_allocation",
    "limited_ground_centroids",
    "paper_scalar_fields",
    "power_diagram_labels",
    "uniform_grid",
)
