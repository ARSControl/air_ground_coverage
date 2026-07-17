"""Reusable coupled loop with optional multi-fidelity estimation and feedback."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Protocol

import casadi as ca
import numpy as np
from numpy.typing import NDArray

from .core import costFunctions
from .core.GaussianProcess import GaussianProcess
from .core.base import HEDACParams, MapLoader
from .core.gmm import GMM
from .core.hedac import HEDACAlgorithm
from .models import models
from .models.agents import (
    AgentTeam,
    DoubleIntegratorAgent,
    DubinsAgent,
    UnicycleAgent,
)
from .simulation import (
    CoordinatorEventReport,
    EstimatorMode,
    MultifidelitySimulationCoordinator,
    build_ground_weight_vectors,
    build_multifidelity_coordinator,
    parse_estimator_mode,
)
from .utils.math_utils import min_max_normalize
from .utils.voronoi import compute_voronoi_partitioning


FloatArray = NDArray[np.float64]


class GroundController(Protocol):
    """Narrow adapter around the unchanged legacy ground-control block."""

    @property
    def density_source(self) -> str: ...

    @property
    def query_points(self) -> np.ndarray: ...

    @property
    def integration_weights(self) -> np.ndarray: ...

    def step(
        self,
        step_num: int,
        hedac: HEDACAlgorithm,
        *,
        external_density: np.ndarray | None = None,
    ) -> None: ...


@dataclass(frozen=True)
class CoupledStepResult:
    step_num: int
    simulation_time: float
    ergodic_metric: float
    coordinator_reports: tuple[CoordinatorEventReport, ...]
    posterior_version: int
    aerial_density_source: str
    ground_density_source: str


@dataclass(frozen=True)
class CoupledSimulationResult:
    ergodic_metrics: FloatArray
    aerial_trajectories: tuple[FloatArray, ...]
    ground_trajectories: tuple[FloatArray, ...]
    estimator_reports: tuple[CoordinatorEventReport, ...]
    step_results: tuple[CoupledStepResult, ...]
    aerial_density_source: str
    ground_density_source: str
    final_posterior_version: int


class CoupledSimulation:
    """Own both teams/controllers and an optional multi-fidelity coordinator."""

    def __init__(
        self,
        *,
        mode: EstimatorMode,
        dt: float,
        aerial_team: AgentTeam,
        hedac: HEDACAlgorithm,
        ground_team: AgentTeam,
        ground_controller: GroundController,
        coordinator: MultifidelitySimulationCoordinator | None = None,
    ) -> None:
        if not isinstance(mode, EstimatorMode):
            raise TypeError("mode must be an EstimatorMode")
        self.dt = _positive_float(dt, "dt")
        if mode is EstimatorMode.LEGACY and coordinator is not None:
            raise ValueError("legacy mode must not construct a coordinator")
        if mode is EstimatorMode.MULTIFIDELITY and coordinator is None:
            raise ValueError("multifidelity mode requires one coordinator")
        self.mode = mode
        self.aerial_team = aerial_team
        self.hedac = hedac
        self.ground_team = ground_team
        self.ground_controller = ground_controller
        self.coordinator = coordinator
        self._step_results: list[CoupledStepResult] = []
        self._estimator_reports: list[CoordinatorEventReport] = []

    @property
    def latest_posterior(self):
        return None if self.coordinator is None else self.coordinator.latest_posterior

    def step(self, step_num: int) -> CoupledStepResult:
        if isinstance(step_num, bool) or not isinstance(step_num, int) or step_num < 0:
            raise ValueError("step_num must be a nonnegative integer")
        simulation_time = step_num * self.dt
        reports: list[CoordinatorEventReport] = []

        if self.coordinator is not None:
            reports.append(
                self.coordinator.collect_low_if_due(
                    simulation_time, self.aerial_team
                )
            )

        if self.coordinator is None:
            aerial_target = None
            aerial_density_source = "legacy_aerial_gp"
            ergodic_metric = float(
                self.hedac.step(self.aerial_team, step_num=step_num)
            )
        else:
            map_height, map_width = self.hedac.map.shape
            aerial_target = self.coordinator.aerial_target(
                np.arange(map_width, dtype=float),
                np.arange(map_height, dtype=float),
                np.ones(self.hedac.map.shape, dtype=float),
                self.hedac.map == 0,
            )
            aerial_density_source = (
                "multifidelity_static_initial"
                if aerial_target is None
                else "multifidelity_high_posterior"
            )
            ergodic_metric = float(
                self.hedac.step(
                    self.aerial_team,
                    step_num=step_num,
                    external_goal_density=aerial_target,
                    update_legacy_gp=False,
                )
            )

        if self.coordinator is not None:
            reports.append(
                self.coordinator.collect_high_if_due(
                    simulation_time, self.ground_team
                )
            )
            reports.append(self.coordinator.update_if_due(simulation_time))

        if self.coordinator is None:
            self.ground_controller.step(step_num, self.hedac)
        else:
            ground_density = self.coordinator.ground_density(
                self.ground_controller.query_points,
                self.ground_controller.integration_weights,
            )
            self.ground_controller.step(
                step_num,
                self.hedac,
                external_density=ground_density,
            )
        self._estimator_reports.extend(reports)
        posterior_version = (
            0 if self.coordinator is None else self.coordinator.estimator.version
        )
        result = CoupledStepResult(
            step_num=step_num,
            simulation_time=simulation_time,
            ergodic_metric=ergodic_metric,
            coordinator_reports=tuple(reports),
            posterior_version=posterior_version,
            aerial_density_source=aerial_density_source,
            ground_density_source=self.ground_controller.density_source,
        )
        self._step_results.append(result)
        return result

    def run(self, num_steps: int) -> CoupledSimulationResult:
        if isinstance(num_steps, bool) or not isinstance(num_steps, int):
            raise TypeError("num_steps must be an integer")
        if num_steps < 0:
            raise ValueError("num_steps must be nonnegative")
        start_step = len(self._step_results)
        for step_num in range(start_step, start_step + num_steps):
            self.step(step_num)
        return CoupledSimulationResult(
            ergodic_metrics=_readonly(
                np.asarray(
                    [result.ergodic_metric for result in self._step_results],
                    dtype=float,
                )
            ),
            aerial_trajectories=_trajectories(self.aerial_team),
            ground_trajectories=_trajectories(self.ground_team),
            estimator_reports=tuple(self._estimator_reports),
            step_results=tuple(self._step_results),
            aerial_density_source=(
                "legacy_aerial_gp"
                if not self._step_results
                else self._step_results[-1].aerial_density_source
            ),
            ground_density_source=self.ground_controller.density_source,
            final_posterior_version=(
                0 if self.coordinator is None else self.coordinator.estimator.version
            ),
        )


def build_coupled_simulation(
    aerial_params: HEDACParams,
    ground_params: HEDACParams,
    *,
    seed: int,
) -> CoupledSimulation:
    """Build the new reusable coupled path from existing parameter objects."""
    if not isinstance(aerial_params, HEDACParams) or not isinstance(
        ground_params, HEDACParams
    ):
        raise TypeError("aerial_params and ground_params must be HEDACParams")
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise TypeError("seed must be an integer")
    mode = parse_estimator_mode(aerial_params)

    # Legacy setup uses the global RNG; seed it exactly once before constructing
    # maps, truth, agents, and MPC state. Shadow sensors use separate generators.
    np.random.seed(seed)
    map_loader = _build_map_loader(aerial_params)
    map_array = map_loader.load()
    goal_density = _build_goal_density(aerial_params, map_loader)
    aerial_team = _build_team(aerial_params, map_array, map_loader.get_free_cells())
    hedac = HEDACAlgorithm(aerial_params, map_loader, goal_density)

    ground_params.dt = aerial_params.dt
    ground_team = _build_team(ground_params, map_array, None)
    ground_controller: GroundController
    controller_type = _ground_controller_type(ground_params)
    if len(ground_team) == 0:
        query_points, query_shape, integration_weights = _structured_query_grid(
            map_array.shape,
            max(2, int(ground_params.local_grid_points)),
        )
        ground_controller = _NoOpGroundController(
            query_points, integration_weights
        )
    elif controller_type == "mpc":
        legacy_ground = _LegacyGroundMPC(
            ground_params, map_array, ground_team, goal_density
        )
        ground_controller = legacy_ground
        query_points = legacy_ground.query_points
        query_shape = legacy_ground.query_shape
        integration_weights = legacy_ground.integration_weights
    else:
        lloyd_ground = _GroundLloydController(
            ground_params, map_array, ground_team, goal_density
        )
        ground_controller = lloyd_ground
        query_points = lloyd_ground.query_points
        query_shape = lloyd_ground.query_shape
        integration_weights = lloyd_ground.integration_weights

    coordinator = None
    if mode is EstimatorMode.MULTIFIDELITY:
        coordinator = build_multifidelity_coordinator(
            aerial_params,
            ground_params,
            goal_density,
            query_points,
            query_shape,
            integration_weights,
            mask=None,
            seed=seed,
        )
    return CoupledSimulation(
        mode=mode,
        dt=float(aerial_params.dt),
        aerial_team=aerial_team,
        hedac=hedac,
        ground_team=ground_team,
        ground_controller=ground_controller,
        coordinator=coordinator,
    )


class _NoOpGroundController:
    def __init__(
        self, query_points: np.ndarray, integration_weights: np.ndarray
    ) -> None:
        self.query_points = _readonly(query_points)
        self.integration_weights = _readonly(integration_weights)
        self.last_density = np.zeros(self.query_points.shape[0], dtype=float)
        self._density_source = "legacy_posthoc_fusion"

    @property
    def density_source(self) -> str:
        return self._density_source

    def step(
        self,
        step_num: int,
        hedac: HEDACAlgorithm,
        *,
        external_density: np.ndarray | None = None,
    ) -> None:
        del step_num, hedac
        if external_density is not None:
            validated = build_ground_weight_vectors(
                external_density,
                np.ones((1, self.query_points.shape[0]), dtype=float),
            )[0]
            self.last_density = validated
            self._density_source = "multifidelity_high_posterior"


class _LegacyGroundMPC:
    """Existing ground MPC law with legacy or external density selection."""

    def __init__(
        self,
        params: HEDACParams,
        map_array: np.ndarray,
        team: AgentTeam,
        high_field: np.ndarray,
    ) -> None:
        self.params = params
        self.map_array = map_array
        self.team = team
        self.high_field = high_field
        self.gp = GaussianProcess(params, map_array.shape)
        self.query_points, self.query_shape, self.integration_weights = (
            _structured_query_grid(map_array.shape, int(params.local_grid_points))
        )
        self._u_previous, self._solver, self._bounds = self._build_solver()
        self.last_density = np.zeros(self.query_points.shape[0], dtype=float)
        self._density_source = "legacy_posthoc_fusion"

    @property
    def density_source(self) -> str:
        return self._density_source

    def _build_solver(self):
        params = self.params
        model_type = params.get("agents.model_type", "double_integrator")
        model_config = models.get_model_config(model_type)
        state_size = model_config["nx"]
        control_size = model_config["nu"]
        dynamics = models.get_dynamics(model_type, backend="casadi")
        horizon = int(params.mpc_horizon)
        controls = ca.SX.sym("U", control_size, horizon)
        initial_state = ca.SX.sym("x0", state_size)
        weights = ca.SX.sym("W", self.query_points.shape[0])
        grid = ca.DM(self.query_points)
        state = initial_state
        objective = 0
        half_fov = np.deg2rad(params.fov_deg) / 2.0
        for stage in range(horizon):
            objective += 10 * costFunctions.limfov_coverage_cost(
                state,
                grid,
                weights,
                r_max=params.fov_depth,
                half_fov=half_fov,
            )
            state = dynamics(state, controls[:, stage], params.dt)
        height, width = self.map_array.shape
        constraints = ca.vertcat(
            state[0] - width, -state[0], state[1] - height, -state[1]
        )
        solver = ca.nlpsol(
            "multifidelity_ground_solver",
            "ipopt",
            {
                "x": ca.vec(controls),
                "f": objective,
                "g": constraints,
                "p": ca.vertcat(initial_state, weights),
            },
            {
                "ipopt": {
                    "print_level": 0,
                    "sb": "yes",
                    "max_iter": params.mpc_max_iters,
                    "tol": params.mpc_tolerance,
                },
                "print_time": params.mpc_print_time,
            },
        )
        acceleration_limit = params.max_ddx
        lower_control = np.tile(
            np.array([-acceleration_limit, -acceleration_limit]), horizon
        )
        upper_control = np.tile(
            np.array([acceleration_limit, acceleration_limit]), horizon
        )
        bounds = (
            lower_control,
            upper_control,
            -np.inf * np.ones(constraints.shape),
            np.zeros(constraints.shape),
        )
        previous = np.zeros((len(self.team), control_size * horizon))
        return previous, solver, bounds

    def step(
        self,
        step_num: int,
        hedac: HEDACAlgorithm,
        *,
        external_density: np.ndarray | None = None,
    ) -> None:
        if external_density is None:
            candidate_density = _legacy_ground_density(
                self.query_points,
                self.team,
                self.high_field,
                self.gp,
                hedac,
                step_num,
            )
        else:
            candidate_density = external_density
        states = self.team.get_states()
        masks = compute_voronoi_partitioning(
            self.query_points, states[:, :2], 50.0
        )
        if external_density is None:
            weight_vectors = tuple(
                candidate_density * masks[index]
                for index in range(len(self.team.agents))
            )
            density_source = "legacy_posthoc_fusion"
        else:
            weight_vectors = build_ground_weight_vectors(candidate_density, masks)
            density_source = "multifidelity_high_posterior"

        # Commit only after the external density and masks have been validated.
        self.last_density = np.array(candidate_density, dtype=float, copy=True)
        self._density_source = density_source
        lower_control, upper_control, lower_constraint, upper_constraint = self._bounds
        for index, (agent, weights) in enumerate(
            zip(self.team.agents, weight_vectors)
        ):
            state = np.hstack([agent.position, agent.theta])
            parameters = np.concatenate([state, weights])
            solution = self._solver(
                x0=self._u_previous[index],
                p=parameters,
                lbx=lower_control,
                ubx=upper_control,
                lbg=lower_constraint,
                ubg=upper_constraint,
            )
            controls = solution["x"].full().reshape(-1, 2)
            self._u_previous[index] = solution["x"].full().ravel()
            velocity, angular_velocity = controls[0]
            agent.step(velocity, angular_velocity)


class _GroundLloydController:
    """Weighted centroidal-Voronoi coverage with unicycle tracking."""

    def __init__(
        self,
        params: HEDACParams,
        map_array: np.ndarray,
        team: AgentTeam,
        high_field: np.ndarray,
    ) -> None:
        if params.get("agents.model_type", "double_integrator") != "unicycle":
            raise ValueError("ground Lloyd controller requires unicycle agents")
        self.params = params
        self.map_array = map_array
        self.team = team
        self.high_field = high_field
        self.gp = GaussianProcess(params, map_array.shape)
        self.query_points, self.query_shape, self.integration_weights = (
            _structured_query_grid(map_array.shape, int(params.local_grid_points))
        )
        self.position_gain = _positive_float(
            params.get("lloyd.position_gain", 1.0), "ground.lloyd.position_gain"
        )
        self.heading_gain = _positive_float(
            params.get("lloyd.heading_gain", 2.0), "ground.lloyd.heading_gain"
        )
        self.centroid_tolerance = _nonnegative_float(
            params.get("lloyd.centroid_tolerance", 0.05),
            "ground.lloyd.centroid_tolerance",
        )
        self.max_linear_velocity = _positive_float(
            params.get("lloyd.max_linear_velocity", params.max_dx),
            "ground.lloyd.max_linear_velocity",
        )
        self.max_angular_velocity = _positive_float(
            params.get("lloyd.max_angular_velocity", params.max_dtheta),
            "ground.lloyd.max_angular_velocity",
        )
        self.last_density = np.zeros(self.query_points.shape[0], dtype=float)
        self.last_centroids = np.empty((len(team), 2), dtype=float)
        self.last_masses = np.zeros(len(team), dtype=float)
        self.last_controls = np.zeros((len(team), 2), dtype=float)
        self._density_source = "legacy_posthoc_fusion"

    @property
    def density_source(self) -> str:
        return self._density_source

    def step(
        self,
        step_num: int,
        hedac: HEDACAlgorithm,
        *,
        external_density: np.ndarray | None = None,
    ) -> None:
        if external_density is None:
            candidate_density = _legacy_ground_density(
                self.query_points,
                self.team,
                self.high_field,
                self.gp,
                hedac,
                step_num,
            )
            density_source = "legacy_posthoc_fusion"
        else:
            candidate_density = external_density
            density_source = "multifidelity_high_posterior"

        states = self.team.get_states()
        positions = states[:, :2]
        map_height, map_width = self.map_array.shape
        masks = compute_voronoi_partitioning(
            self.query_points,
            positions,
            math.hypot(map_width, map_height) + 1.0,
        )
        centroids, masses = compute_weighted_voronoi_centroids(
            self.query_points,
            candidate_density,
            masks,
            self.integration_weights,
            positions,
        )

        controls = np.zeros((len(self.team), 2), dtype=float)
        for index, (agent, centroid) in enumerate(
            zip(self.team.agents, centroids)
        ):
            displacement = centroid - agent.position
            distance = float(np.linalg.norm(displacement))
            if distance <= self.centroid_tolerance:
                velocity = 0.0
                angular_velocity = 0.0
            else:
                target_heading = math.atan2(displacement[1], displacement[0])
                heading_error = _wrap_angle(target_heading - agent.theta)
                velocity = float(
                    np.clip(
                        self.position_gain * distance * math.cos(heading_error),
                        -self.max_linear_velocity,
                        self.max_linear_velocity,
                    )
                )
                angular_velocity = float(
                    np.clip(
                        self.heading_gain * heading_error,
                        -self.max_angular_velocity,
                        self.max_angular_velocity,
                    )
                )
            controls[index] = (velocity, angular_velocity)

        # Commit only after all density, geometry, and control computations pass.
        self.last_density = np.array(candidate_density, dtype=float, copy=True)
        self.last_centroids = centroids
        self.last_masses = masses
        self.last_controls = controls
        self._density_source = density_source
        for agent, (velocity, angular_velocity) in zip(
            self.team.agents, controls
        ):
            agent.step(velocity, angular_velocity)


def compute_weighted_voronoi_centroids(
    query_points: np.ndarray,
    density: np.ndarray,
    voronoi_masks: np.ndarray,
    integration_weights: np.ndarray,
    fallback_positions: np.ndarray,
) -> tuple[FloatArray, FloatArray]:
    """Compute discrete density-weighted centroids for each Voronoi cell."""
    points = np.asarray(query_points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("query_points must have shape (N, 2)")
    if not np.all(np.isfinite(points)):
        raise ValueError("query_points must contain only finite values")
    weights = np.asarray(integration_weights, dtype=float)
    if weights.shape != (points.shape[0],):
        raise ValueError("integration_weights must have shape (N,)")
    if not np.all(np.isfinite(weights)) or np.any(weights < 0.0):
        raise ValueError("integration_weights must be finite and nonnegative")
    fallback = np.asarray(fallback_positions, dtype=float)
    masks = np.asarray(voronoi_masks)
    if masks.ndim != 2:
        raise ValueError("voronoi_masks must have shape (R, N)")
    if fallback.shape != (masks.shape[0], 2):
        raise ValueError("fallback_positions must have shape (R, 2)")
    if not np.all(np.isfinite(fallback)):
        raise ValueError("fallback_positions must contain only finite values")

    cell_densities = build_ground_weight_vectors(density, masks)
    centroids = np.empty_like(fallback)
    masses = np.zeros(masks.shape[0], dtype=float)
    for index, cell_density in enumerate(cell_densities):
        mass_weights = cell_density * weights
        mass = float(np.sum(mass_weights))
        masses[index] = mass
        if mass <= np.finfo(float).eps:
            centroids[index] = fallback[index]
        else:
            centroids[index] = np.sum(
                points * mass_weights[:, np.newaxis], axis=0
            ) / mass
    return _readonly(centroids), _readonly(masses)


def _legacy_ground_density(
    query_points: np.ndarray,
    team: AgentTeam,
    high_field: np.ndarray,
    gp: GaussianProcess,
    hedac: HEDACAlgorithm,
    step_num: int,
) -> np.ndarray:
    aerial_mean, aerial_std = hedac.gpr_model.predict(
        query_points, return_std=True
    )
    aerial_std = _unit_max(aerial_std)
    observations = gp.collect_observations(team, high_field)
    gp.update_gp(observations, step_num)
    ground_mean, ground_std = gp.predict(query_points, return_std=True)
    ground_std = _unit_max(ground_std)
    denominator = 1.0 / (ground_std + 1.0e-10) + 1.0 / (
        aerial_std + 1.0e-10
    )
    aerial_weight = (1.0 / (aerial_std + 1.0e-10)) / denominator
    ground_weight = (1.0 / (ground_std + 1.0e-10)) / denominator
    return aerial_weight * aerial_mean + ground_weight * ground_mean


def _build_map_loader(params: HEDACParams) -> MapLoader:
    map_path = params.map_config.get("path")
    if map_path is not None:
        return MapLoader(map_path=str(Path(map_path)), resolution=params.resolution)
    size = tuple(params.map_config.get("size", [50, 50]))
    return MapLoader(size=size, resolution=params.resolution)


def _build_goal_density(
    params: HEDACParams, map_loader: MapLoader
) -> np.ndarray:
    map_array = map_loader.load()
    height, width = map_array.shape
    number_of_peaks = int(params.get("goal_density.num_peaks", 3))
    means = 5.0 + np.random.rand(number_of_peaks, 2) * np.array(
        [max(width - 10, 1), max(height - 10, 1)]
    )
    covariances = []
    for _ in range(number_of_peaks):
        angle = np.random.rand() * 2.0 * np.pi
        cosine, sine = np.cos(angle), np.sin(angle)
        rotation = np.array([[cosine, -sine], [sine, cosine]])
        scale_x = (width / 4.0 + np.random.rand() * width / 4.0) / (
            2.0 + 2.0 * np.random.rand()
        )
        scale_y = (height / 4.0 + np.random.rand() * height / 4.0) / (
            2.0 + 2.0 * np.random.rand()
        )
        covariance = rotation @ np.diag([scale_x**2, scale_y**2]) @ rotation.T
        covariances.append(covariance)
    mixture = GMM(
        means=means,
        covariances=np.asarray(covariances),
        weights=np.random.dirichlet(np.ones(number_of_peaks)),
    )
    grid_x, grid_y = np.meshgrid(np.arange(width), np.arange(height))
    query = np.column_stack((grid_x.ravel(), grid_y.ravel()))
    density = mixture.sample_pdf(query).reshape(map_array.shape)
    density = min_max_normalize(density) * (map_array == 0)
    return density


def _build_team(
    params: HEDACParams,
    map_array: np.ndarray,
    free_cells: np.ndarray | None,
) -> AgentTeam:
    count = int(params.num_agents)
    height, width = map_array.shape
    if count == 0:
        return AgentTeam([])
    if free_cells is not None and len(free_cells) >= count:
        indices = np.random.choice(len(free_cells), count, replace=False)
        positions = free_cells[indices][:, [1, 0]].astype(float)
    else:
        positions = np.random.rand(count, 2) * np.array([width, height])
    agents = []
    model_type = params.get("agents.model_type", "double_integrator")
    observation_count = int(params.get("gpr.obs_per_step", 10))
    for index, position in enumerate(positions):
        heading = float(np.random.uniform(0.0, 2.0 * np.pi))
        common = {
            "x0": position,
            "theta0": heading,
            "dt": params.dt_agent,
            "agent_id": index,
            "observations_range": params.fov_depth,
            "observations_count": observation_count,
            "sens_range": params.sens_range,
        }
        if model_type == "dubins":
            dubins = params.get("agents.dubins", {})
            agent = DubinsAgent(
                **common,
                forward_speed=float(dubins.get("forward_speed", 5.0)),
                max_bank_angle=float(dubins.get("max_bank_angle", 30.0)),
            )
        elif model_type == "unicycle":
            agent = UnicycleAgent(
                **common,
                max_v=params.max_dx,
                max_omega=params.max_dtheta,
                fov_degrees=params.fov_deg,
            )
        else:
            agent = DoubleIntegratorAgent(
                **common,
                max_dx=params.max_dx,
                max_ddx=params.max_ddx,
                max_dtheta=params.max_dtheta,
                max_ddtheta=params.max_ddtheta,
            )
        agents.append(agent)
    return AgentTeam(agents)


def _structured_query_grid(
    map_shape: tuple[int, int], points_per_axis: int
) -> tuple[np.ndarray, tuple[int, int], np.ndarray]:
    if points_per_axis < 2:
        raise ValueError("local_grid_points must be at least 2")
    height, width = map_shape
    x_values = np.linspace(0.0, float(width), points_per_axis)
    y_values = np.linspace(0.0, float(height), points_per_axis)
    grid_x, grid_y = np.meshgrid(x_values, y_values)
    query = np.column_stack((grid_x.ravel(), grid_y.ravel()))
    x_weights = np.full(points_per_axis, width / (points_per_axis - 1))
    y_weights = np.full(points_per_axis, height / (points_per_axis - 1))
    x_weights[[0, -1]] *= 0.5
    y_weights[[0, -1]] *= 0.5
    weights = np.outer(y_weights, x_weights).ravel()
    return query, grid_x.shape, weights


def _trajectories(team: AgentTeam) -> tuple[FloatArray, ...]:
    return tuple(_readonly(np.asarray(history, dtype=float)) for history in team.get_histories())


def _readonly(values: np.ndarray) -> FloatArray:
    result = np.array(values, dtype=float, copy=True)
    result.setflags(write=False)
    return result


def _unit_max(values: np.ndarray) -> np.ndarray:
    maximum = float(np.max(values))
    return values / (maximum + 1.0e-10)


def _positive_float(value: float, name: str) -> float:
    if isinstance(value, bool):
        raise TypeError(f"{name} must be a real scalar")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _nonnegative_float(value: float, name: str) -> float:
    if isinstance(value, bool):
        raise TypeError(f"{name} must be a real scalar")
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise ValueError(f"{name} must be finite and nonnegative")
    return result


def _ground_controller_type(params: HEDACParams) -> str:
    value = params.get("controller.type", "mpc")
    if not isinstance(value, str):
        raise TypeError("ground.controller.type must be a string")
    normalized = value.strip().lower()
    if normalized not in {"mpc", "lloyd"}:
        raise ValueError("ground.controller.type must be 'mpc' or 'lloyd'")
    return normalized


def _wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))
