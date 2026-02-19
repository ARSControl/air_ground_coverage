import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import argparse
import casadi as ca
from types import SimpleNamespace
from typing import Tuple, cast

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.core.base import HEDACParams, MapLoader
from src.core.hedac import HEDACAlgorithm
from src.core.gmm import GMM
from src.core import costFunctions
from src.models.agents import (
    DoubleIntegratorAgent,
    DubinsAgent,
    UnicycleAgent,
    AgentTeam,
)
from src.models import models
from src.utils.math_utils import min_max_normalize
from src.utils.voronoi import compute_anisotropic_voronoi_partitioning, agent_fov

def points_in_fov(position, theta, points, fov_depth, fov_deg):
    """
    Check which points are within the field of view of the agent.

    Args:
        position: Agent position [x, y]
        theta: Agent heading in radians
        points: Array of points to check (N, 2)
        fov_depth: Maximum distance for FOV
        fov_deg: FOV angle in degrees

    Returns:
        Boolean array indicating which points are within FOV
    """
    if len(points) == 0:
        return np.array([], dtype=bool)

    # Compute vectors from agent to points
    dx = points[:, 0] - position[0]
    dy = points[:, 1] - position[1]

    # Compute distances
    distances = np.sqrt(dx**2 + dy**2)

    # Check distance constraint
    in_range = distances <= fov_depth

    # Compute angles from agent to points
    angles = np.arctan2(dy, dx)

    # Compute angle difference relative to agent heading
    half_fov = np.deg2rad(fov_deg) / 2
    angle_diff = np.abs(np.arctan2(np.sin(angles - theta), np.cos(angles - theta)))

    # Check angle constraint
    in_angle = angle_diff <= half_fov

    return in_range & in_angle

def dH_dtheta(p, theta, q, w, gamma):
    """Compute gradient of coverage cost w.r.t. orientation theta for anisotropic partitioning."""
    c = np.cos(theta)
    s = np.sin(theta)

    dq11 = 2 * c * s * (gamma - 1)
    dq12 = (1 - gamma) * (c * c - s * s)
    dq22 = 2 * c * s * (1 - gamma)

    dx = q[:, 0] - p[0]
    dy = q[:, 1] - p[1]

    val = dx * (dq11 * dx + dq12 * dy) + dy * (dq12 * dx + dq22 * dy)

    return -np.sum(w * val)


def create_gaussian_goal_density(
    map_loader: MapLoader, num_peaks: int = 3
) -> np.ndarray:
    """
    Create a simple Gaussian mixture goal density.

    Args:
        map_loader: Map loader
        num_peaks: Number of Gaussian peaks

    Returns:
        Goal density map
    """
    map_array = map_loader.load()
    free_cells = map_loader.get_free_cells()

    # Create grid points
    height, width = map_array.shape
    x = np.arange(width)
    y = np.arange(height)
    grid_x, grid_y = np.meshgrid(x, y)
    grid_points = np.column_stack([grid_x.ravel(), grid_y.ravel()])

    # Random Gaussian centers in free space

    if len(free_cells) > num_peaks:
        indices = np.random.choice(len(free_cells), num_peaks, replace=False)
        # free_cells is in [y, x] format (row, col), convert to [x, y]
        means = free_cells[indices].astype(float)
        means = means[:, [1, 0]]  # Swap to [x, y]
    else:
        # Fallback to random positions
        means = np.random.rand(num_peaks, 2) * np.array([width, height])

    # Fix the mean for testing
    # means = np.array([[width / 4, height / 4]])  # Single peak at center

    # Covariances - use wider Gaussians for better gradient coverage
    # sigma = width/3 instead of width/5 for wider spread
    covariances = np.array(
        [[[width / 3, 0], [0, height / 3]] for _ in range(num_peaks)]
    )

    # Create density
    density = np.zeros(height * width)
    for mean, cov in zip(means, covariances):
        # Compute Gaussian
        diff = grid_points - mean
        inv_cov = np.linalg.inv(cov)
        det_cov = np.linalg.det(cov)

        exponent = -0.5 * np.sum(diff @ inv_cov * diff, axis=1)
        coefficient = 1 / np.sqrt((2 * np.pi) ** 2 * det_cov)
        density += coefficient * np.exp(exponent)

    # Normalize and reshape
    density = min_max_normalize(density)
    density_map = density.reshape(height, width)

    # Apply map mask and renormalize
    density_map = density_map * (map_array == 0)
    density_map = density_map / (np.sum(density_map) + 1e-10)

    return density_map


def init_goal_density(map_loader: MapLoader, num_peaks: int = 3) -> GMM:
    """
    Create a simple Gaussian mixture goal density.

    Args:
        map_loader: Map loader
        num_peaks: Number of Gaussian peaks

    Returns:
        GMM
    """
    map_array = map_loader.load()
    free_cells = map_loader.get_free_cells()

    # Create grid points
    height, width = map_array.shape
    # x = np.arange(width)
    # y = np.arange(height)
    # grid_x, grid_y = np.meshgrid(x, y)
    # grid_points = np.column_stack([grid_x.ravel(), grid_y.ravel()])

    # Random Gaussian centers in free space
    # if len(free_cells) > num_peaks:
    #     indices = np.random.choice(len(free_cells), num_peaks, replace=False)
    #     # free_cells is in [y, x] format (row, col), convert to [x, y]
    #     means = free_cells[indices].astype(float)
    #     means = means[:, [1, 0]]  # Swap to [x, y]
    # else:
    #     # Fallback to random positions
    means = np.random.rand(num_peaks, 2) * np.array([width, height])

    # Fix the mean for testing
    # means = np.array([[width / 4, height / 4]])  # Single peak at center

    # Covariances - use wider Gaussians for better gradient coverage
    # sigma = width/3 instead of width/5 for wider spread
    covariances = np.array(
        [[[width / 3, 0], [0, height / 3]] for _ in range(num_peaks)]
    )

    weights = np.random.dirichlet(np.ones(num_peaks))  # Random weights that sum to 1
    gmm = GMM(means=means, covariances=covariances, weights=weights)
    return gmm


def init_hedac_from_params(params: HEDACParams, config_path: str):
    """
    Init HEDAC params from params.

    Args:
        config_path: Path to YAML configuration file

    Returns:
        agent_team, hedac_algorithm, map_array, goal_density
    """

    # Setup map
    print("Loading map...")
    map_config = params.map_config

    if "path" in map_config and map_config["path"] is not None:
        # Load from file
        map_path = map_config["path"]
        # Handle relative paths
        if not os.path.isabs(map_path):
            config_dir = os.path.dirname(config_path)
            map_path = os.path.join(config_dir, map_path)
        map_loader = MapLoader(map_path=map_path, resolution=params.resolution)
    else:
        # Create obstacle-free
        size = map_config.get("size", [50, 50])
        map_loader = MapLoader(size=tuple(size), resolution=params.resolution)

    map_array = map_loader.load()
    height, width = map_array.shape
    free_cells = map_loader.get_free_cells()
    print(f"Map shape: {map_array.shape}, Free cells: {len(free_cells)}")

    # Create goal density
    print("Creating goal density...")
    goal_config = params.get("goal_density", {})
    goal_type = goal_config.get("type", "gaussian_mixture")

    if goal_type == "gaussian_mixture":
        num_peaks = goal_config.get("num_peaks", 3)
        gmm = init_goal_density(map_loader, num_peaks=num_peaks)
        x_grid = np.arange(0, width, params.resolution)
        y_grid = np.arange(0, height, params.resolution)
        xy_grid = np.column_stack(
            [np.repeat(x_grid, len(y_grid)), np.tile(y_grid, len(x_grid))]
        )
        goal_density = gmm.sample_pdf(xy_grid).reshape(height, width)
        goal_density = min_max_normalize(goal_density)
        goal_density = goal_density * (map_array == 0)
        # goal_density = goal_density / (np.sum(goal_density) + 1e-10)
    # elif goal_type == "uniform":
    #     goal_density = np.ones_like(map_array, dtype=float)
    #     goal_density[map_array == 1] = 0.0
    #     goal_density = goal_density / np.sum(goal_density)
    # elif goal_type == "file":
    #     file_path = goal_config.get("file_path")
    #     if file_path is None:
    #         raise ValueError("file_path required for goal_density type 'file'")
    #     goal_density = np.load(file_path)
    else:
        raise ValueError(f"Unknown goal_density type: {goal_type}")

    # Create agents
    print(f"Creating {params.num_agents} aerial agents...")
    agents = []
    if len(free_cells) >= params.num_agents:
        # Random initial positions in free space
        indices = np.random.choice(len(free_cells), params.num_agents, replace=False)
        # free_cells is in [y, x] format (row, col), convert to [x, y]
        initial_positions_yx = free_cells[indices]
        initial_positions = initial_positions_yx[:, [1, 0]]  # Swap to [x, y]
    else:
        # Fallback to random positions
        initial_positions = np.random.rand(params.num_agents, 2) * np.array(
            [map_array.shape[1], map_array.shape[0]]  # [width, height]
        )

    model_type = params.get("agents.model_type", "double_integrator")

    for i in range(params.num_agents):
        if model_type == "dubins":
            dubins_config = params.get("agents.dubins", {})
            agent = DubinsAgent(
                x0=initial_positions[i],
                theta0=np.random.uniform(0, 2 * np.pi),
                forward_speed=float(dubins_config.get("forward_speed", 5.0) or 5.0),
                max_bank_angle=float(dubins_config.get("max_bank_angle", 30.0) or 30.0),
                dt=params.dt_agent,
                agent_id=i,
            )
        elif model_type == "unicycle":
            agent = UnicycleAgent(
                x0=initial_positions[i],
                theta0=np.random.uniform(0, 2 * np.pi),
                max_speed=params.max_dx,
                max_angular_speed=params.max_dtheta,
                dt=params.dt_agent,
                agent_id=i,
            )
        else:
            agent = DoubleIntegratorAgent(
                x0=initial_positions[i],
                theta0=np.random.uniform(0, 2 * np.pi),
                max_dx=params.max_dx,
                max_ddx=params.max_ddx,
                max_dtheta=params.max_dtheta,
                max_ddtheta=params.max_ddtheta,
                dt=params.dt_agent,
                agent_id=i,
            )
        agent.sens_range = params.sens_range
        agents.append(agent)

    agent_team = AgentTeam(agents)

    # Initialize HEDAC
    print("Initializing HEDAC...")
    hedac = HEDACAlgorithm(params, map_loader, goal_density)

    return agent_team, hedac, map_array, goal_density, gmm


def init_mpc_from_params(params: HEDACParams, map_array: np.ndarray):
    """
    Init MPC params from params.

    Args:
        config_path: Path to YAML configuration file

    Returns:
        mpc_controller
    """
    mpc_params = SimpleNamespace()
    mpc_params.model_type = params.get("agents.model_type", "double_integrator")
    mpc_params.model_config = models.get_model_config(mpc_params.model_type)
    mpc_params.nx = mpc_params.model_config["nx"]
    mpc_params.nu = mpc_params.model_config["nu"]

    # Get dynamics functions
    mpc_params.dynamics_numba = models.get_dynamics(
        mpc_params.model_type, backend="numba"
    )
    mpc_params.dynamics_casadi = models.get_dynamics(
        mpc_params.model_type, backend="casadi"
    )
    mpc_params.simulate_trajectory_numba = models.simulate_trajectory_numba

    # Map
    height, width = map_array.shape
    xg = np.arange(0, width, params.resolution)
    yg = np.arange(0, height, params.resolution)
    mpc_params.X, mpc_params.Y = np.meshgrid(xg, yg)
    mpc_params.xy_grid = np.column_stack([mpc_params.X.ravel(), mpc_params.Y.ravel()])
    nxcells = int(width / params.resolution)
    nycells = int(height / params.resolution)
    mpc_params.map = np.zeros((nxcells, nycells))  # placeholder for now
    GRID_SIZE = len(xg)

    # Obstacles
    num_obstacles = getattr(params, "num_obstacles", 5)
    mpc_params.obstacles_radius = getattr(params, "obstaclesRadius", 1.0)
    mpc_params.x_obs = np.random.rand(num_obstacles, 2) * np.array([width, height])

    # Create agents
    print(f"Creating {params.num_agents} ground agents...")
    agents = []
    initial_positions = np.random.rand(params.num_agents, 2) * np.array(
        [width, height]  # [width, height]
    )

    model_type = params.get("agents.model_type", "double_integrator")

    for i in range(params.num_agents):
        if model_type == "dubins":
            dubins_config = params.get("agents.dubins", {})
            agent = DubinsAgent(
                x0=initial_positions[i],
                theta0=np.random.uniform(0, 2 * np.pi),
                forward_speed=float(dubins_config.get("forward_speed", 5.0) or 5.0),
                max_bank_angle=float(dubins_config.get("max_bank_angle", 30.0) or 30.0),
                dt=params.dt_agent,
                agent_id=i,
            )
        elif model_type == "unicycle":
            agent = UnicycleAgent(
                x0=initial_positions[i],
                theta0=np.random.uniform(0, 2 * np.pi),
                max_v=params.max_dx,
                max_omega=params.max_dtheta,
                dt=params.dt_agent,
                agent_id=i,
            )
        else:
            agent = DoubleIntegratorAgent(
                x0=initial_positions[i],
                theta0=np.random.uniform(0, 2 * np.pi),
                max_dx=params.max_dx,
                max_ddx=params.max_ddx,
                max_dtheta=params.max_dtheta,
                max_ddtheta=params.max_ddtheta,
                dt=params.dt_agent,
                agent_id=i,
            )
        agent.sens_range = params.sens_range
        agents.append(agent)

    agent_team = AgentTeam(agents)

    opts = {
        #   'jit': True,
        #   'jit_options': {'flags': '-O2'},
        #   'compiler': 'shell',
        "ipopt": {
            "print_level": 0,
            "sb": "yes",
            "max_iter": params.mpc_max_iters,
            "tol": params.mpc_tolerance,
            "warm_start_init_point": "yes",
            "warm_start_bound_push": 1e-6,
            "warm_start_mult_bound_push": 1e-6,
        },
        "print_time": params.mpc_print_time,
    }
    R_u = np.zeros((mpc_params.nu,))
    # GRID_DM = ca.DM(mpc_params.xy_grid)

    # MPC horizon - use a default if not specified
    T = params.mpc_horizon if hasattr(params, "mpc_horizon") else 10
    LOCAL_GRID_CELLS = params.local_grid_points

    # Sensor parameters for coverage cost
    r = params.fov_depth  # Range for coverage
    half_fov = np.deg2rad(params.fov_deg) / 2  # Half FOV in radians
    Ds = params.min_safe_range  # Safety distance for collision avoidance

    # for idx in range(ROBOTS_NUM):
    U = ca.SX.sym("U", mpc_params.nu, T)
    x0 = ca.SX.sym("x0", mpc_params.nx)
    W = ca.SX.sym("W", LOCAL_GRID_CELLS**2)  # weights

    
    xg = np.linspace(0, width, LOCAL_GRID_CELLS)
    yg = np.linspace(0, height, LOCAL_GRID_CELLS)
    mpc_params.X_mpc, mpc_params.Y_mpc = np.meshgrid(xg, yg)
    mpc_params.xy_mpc_grid = np.column_stack([mpc_params.X_mpc.ravel(), mpc_params.Y_mpc.ravel()])
    GRID_DM = ca.DM(mpc_params.xy_mpc_grid)

    x = x0
    obj = 0

    for k in range(T):
        obj += costFunctions.limfov_coverage_cost(
            x, GRID_DM, W, r_max=ground_params.fov_depth, half_fov=half_fov
        )
        # obj += ca.sumsqr(U[:, k]) * 0.01
        # obj += control_effort_cost(U[:, k].reshape(-1,1), R_u)
        for obs in mpc_params.x_obs:
            obj += costFunctions.collision_cost(x[:2], obs, Ds)
        x = mpc_params.dynamics_casadi(x, U[:, k], params.dt)

    g = ca.vertcat(x[0] - width, -x[0], x[1] - height, -x[1])
    p = ca.vertcat(x0, W)

    print("===== Desired p shape: ", p.shape)
    solver = ca.nlpsol(
        "solver",
        "ipopt",
        {"x": ca.vec(U), "f": obj, "g": g, "p": p},
        opts,
    )

    # solvers.append(solver)

    # bounds and initial guess
    amax = params.max_ddx
    u_min = np.array([-amax, -amax])
    u_max = np.array([amax, amax])
    mpc_params.lbx = np.tile(u_min, T)
    mpc_params.ubx = np.tile(u_max, T)

    # bounds on g: g <= 0
    mpc_params.lbg = -np.inf * np.ones(g.shape)
    mpc_params.ubg = np.zeros(g.shape)  # Ds**2 - ||x - x_obs||**2 < 0

    return agent_team, mpc_params, solver


def run_hedac_from_config(config_path: str):
    """
    Run HEDAC simulation from YAML configuration file.

    Args:
        config_path: Path to YAML configuration file

    Returns:
        results, map_array, goal_density, agent_team
    """
    # Load parameters from YAML
    print(f"Loading configuration from: {config_path}")
    params = HEDACParams.from_yaml(config_path)
    print(f"Configuration loaded: {params}")

    # Set random seed
    np.random.seed(params.random_seed)

    # Setup map
    print("Loading map...")
    map_config = params.map_config

    if "path" in map_config and map_config["path"] is not None:
        # Load from file
        map_path = map_config["path"]
        # Handle relative paths
        if not os.path.isabs(map_path):
            config_dir = os.path.dirname(config_path)
            map_path = os.path.join(config_dir, map_path)
        map_loader = MapLoader(map_path=map_path, resolution=params.resolution)
    else:
        # Create obstacle-free
        size = map_config.get("size", [50, 50])
        map_loader = MapLoader(size=tuple(size), resolution=params.resolution)

    map_array = map_loader.load()
    free_cells = map_loader.get_free_cells()
    print(f"Map shape: {map_array.shape}, Free cells: {len(free_cells)}")

    # Create goal density
    print("Creating goal density...")
    goal_config = params.get("goal_density", {})
    goal_type = goal_config.get("type", "gaussian_mixture")

    if goal_type == "gaussian_mixture":
        num_peaks = goal_config.get("num_peaks", 3)
        goal_density = create_gaussian_goal_density(map_loader, num_peaks=num_peaks)
    elif goal_type == "uniform":
        goal_density = np.ones_like(map_array, dtype=float)
        goal_density[map_array == 1] = 0.0
        goal_density = goal_density / np.sum(goal_density)
    elif goal_type == "file":
        file_path = goal_config.get("file_path")
        if file_path is None:
            raise ValueError("file_path required for goal_density type 'file'")
        goal_density = np.load(file_path)
    else:
        raise ValueError(f"Unknown goal_density type: {goal_type}")

    # Create agents
    print(f"Creating {params.num_agents} agents...")
    agents = []
    if len(free_cells) >= params.num_agents:
        # Random initial positions in free space
        indices = np.random.choice(len(free_cells), params.num_agents, replace=False)
        # free_cells is in [y, x] format (row, col), convert to [x, y]
        initial_positions_yx = free_cells[indices]
        initial_positions = initial_positions_yx[:, [1, 0]]  # Swap to [x, y]
    else:
        # Fallback to random positions
        initial_positions = np.random.rand(params.num_agents, 2) * np.array(
            [map_array.shape[1], map_array.shape[0]]  # [width, height]
        )

    model_type = params.get("agents.model_type", "double_integrator")

    for i in range(params.num_agents):
        if model_type == "dubins":
            dubins_config = params.get("agents.dubins", {})
            agent = DubinsAgent(
                x0=initial_positions[i],
                theta0=np.random.uniform(0, 2 * np.pi),
                forward_speed=float(dubins_config.get("forward_speed", 5.0) or 5.0),
                max_bank_angle=float(dubins_config.get("max_bank_angle", 30.0) or 30.0),
                dt=params.dt_agent,
                agent_id=i,
            )
        else:
            agent = DoubleIntegratorAgent(
                x0=initial_positions[i],
                theta0=np.random.uniform(0, 2 * np.pi),
                max_dx=params.max_dx,
                max_ddx=params.max_ddx,
                max_dtheta=params.max_dtheta,
                max_ddtheta=params.max_ddtheta,
                dt=params.dt_agent,
                agent_id=i,
            )
        agent.sens_range = params.sens_range
        agents.append(agent)

    agent_team = AgentTeam(agents)

    # Initialize HEDAC
    print("Initializing HEDAC...")
    hedac = HEDACAlgorithm(params, map_loader, goal_density)

    # Run simulation
    print(f"Running simulation for {params.num_steps} steps...")
    results = hedac.run(agent_team, num_steps=params.num_steps, verbose=True)

    print("Simulation complete!")
    print(f"Final ergodic metric: {results['ergodic_metrics'][-1]:.6f}")

    # Save results if requested
    output_config = params.get("output", {})
    if output_config.get("save_results", False):
        results_path = output_config.get("results_path", "output/results.npz")
        os.makedirs(os.path.dirname(results_path), exist_ok=True)
        np.savez(
            results_path,
            ergodic_metrics=results["ergodic_metrics"],
            coverage_density=results["coverage_density"],
            heat_field=results["heat_field"],
            final_positions=results["final_positions"],
            goal_density=goal_density,
            map_array=map_array,
        )
        print(f"Results saved to: {results_path}")

    return results, map_array, goal_density, agent_team


def plot_results(results, map_array, goal_density, agent_team, save_path=None):
    """Plot simulation results."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))

    # Plot 1: Goal density
    ax = axes[0, 0]
    ax.set_title("Goal Density")
    ax.contourf(goal_density, cmap="RdPu", levels=10)
    ax.pcolormesh(np.where(map_array == 0, np.nan, map_array), cmap="gray", alpha=0.3)
    ax.set_aspect("equal")

    # Plot 2: Coverage density
    ax = axes[0, 1]
    ax.set_title("Final Coverage Density")
    coverage_norm = results["coverage_density"] / (
        np.sum(results["coverage_density"]) + 1e-10
    )
    ax.contourf(coverage_norm, cmap="RdPu", levels=10)
    ax.pcolormesh(np.where(map_array == 0, np.nan, map_array), cmap="gray", alpha=0.3)
    ax.set_aspect("equal")

    # Plot 3: Agent trajectories
    ax = axes[1, 0]
    ax.set_title("Agent Trajectories")
    ax.contourf(goal_density, cmap="RdPu", levels=10, alpha=0.3)
    ax.pcolormesh(np.where(map_array == 0, np.nan, map_array), cmap="gray", alpha=0.3)

    for i, traj in enumerate(results["trajectories"]):
        ax.plot(traj[:, 0], traj[:, 1], label=f"Agent {i}", alpha=0.7)
        ax.scatter(traj[0, 0], traj[0, 1], c=f"C{i}", marker="o", s=100)
        ax.scatter(traj[-1, 0], traj[-1, 1], c=f"C{i}", marker="x", s=100)

    ax.legend()
    ax.set_aspect("equal")

    # Plot 4: Ergodic metric over time
    ax = axes[1, 1]
    ax.set_title("Ergodic Metric Over Time")
    ax.plot(results["ergodic_metrics"])
    ax.set_xlabel("Time Step")
    ax.set_ylabel("Ergodic Metric")
    ax.grid(True)

    plt.tight_layout()

    if save_path:
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        plt.savefig(save_path, dpi=150)
        print(f"Plot saved to: {save_path}")

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Run HEDAC simulation from YAML config"
    )
    parser.add_argument(
        "--aerial_config",
        "-ac",
        type=str,
        default="configs/iros26_aerial.yaml",
        help="Path to YAML aerial robots configuration file",
    )
    parser.add_argument(
        "--ground_config",
        "-gc",
        type=str,
        default="configs/iros26_ground.yaml",
        help="Path to YAML ground robots configuration file",
    )
    parser.add_argument("--no-plot", action="store_true", help="Disable plotting")

    args = parser.parse_args()

    # Check if config file exists
    if not os.path.exists(args.aerial_config):
        # Try with hedac prefix
        alt_path = os.path.join("hedac", args.aerial_config)
        if os.path.exists(alt_path):
            args.aerial_config = alt_path
        else:
            print(f"Error: Config file not found: {args.aerial_config}")
            print(f"Also tried: {alt_path}")
            sys.exit(1)

    # Init simulation
    # Load parameters from YAML
    print(f"Loading configuration from: {args.aerial_config} and {args.ground_config}")
    params = HEDACParams.from_yaml(args.aerial_config)
    ground_params = HEDACParams.from_yaml(args.ground_config)
    ground_params.dt = getattr(params, "dt", 0.1)
    print(f"Configurations loaded: {params}")

    colors = [
        "tab:blue",
        "tab:orange",
        "tab:green",
        "tab:red",
        "tab:purple",
        "tab:brown",
        "tab:pink",
        "tab:gray",
        "tab:olive",
        "tab:cyan",
    ]
    cmaps = ["Blues", "Oranges", "Greens", "Reds", "Purples", "Browns"]

    # Set random seed
    np.random.seed(params.random_seed)

    # Aerial and common params
    aerial_team, hedac, map_array, aerial_goal_density, gmm = init_hedac_from_params(
        params, config_path=args.aerial_config
    )
    print_freq_val = params.get("output.print_frequency", 100)
    print_freq = int(print_freq_val) if print_freq_val is not None else 100
    verbose = params.get("output.verbose", True)

    # Ground params
    ground_team, mpc_params, solver = init_mpc_from_params(ground_params, map_array)
    plt.ion()
    fig, axs = plt.subplots(1, 3, figsize=(18, 6))
    img = axs[1].imshow(
        np.zeros_like(mpc_params.X), extent=[0, 50, 0, 50], origin="lower", cmap="Blues"
    )
    x_lr = np.arange(0, map_array.shape[1], params.resolution)
    y_lr = np.arange(0, map_array.shape[0], params.resolution)
    X_lr, Y_lr = np.meshgrid(x_lr, y_lr)
    xy_lr_grid = np.column_stack([X_lr.flatten(), Y_lr.flatten()])
    x_hr = np.arange(0, map_array.shape[1], ground_params.resolution)
    y_hr = np.arange(0, map_array.shape[0], ground_params.resolution)
    X_hr, Y_hr = np.meshgrid(x_hr, y_hr)
    xy_hr_grid = np.column_stack([X_hr.flatten(), Y_hr.flatten()])
    u_prev = np.zeros(
        (ground_params.num_agents, mpc_params.nu * ground_params.mpc_horizon)
    )
    x_mpc = np.linspace(0, map_array.shape[1], ground_params.local_grid_points)
    y_mpc = np.linspace(0, map_array.shape[0], ground_params.local_grid_points)
    X_mpc, Y_mpc = np.meshgrid(x_mpc, y_mpc)
    xy_mpc_grid = np.column_stack([X_mpc.flatten(), Y_mpc.flatten()])
    print("xy_mpc_grid shape: ", xy_mpc_grid.shape)
    for step in range(params.num_steps):
        if step % print_freq == 0 and verbose:
            print(f"Step {step + 1}/{params.num_steps}")
        
        # Aerial robots step
        erg_metric = hedac.step(aerial_team, step_num=step)
        lr_pred = hedac.gpr_model.predict(xy_lr_grid, return_std=True)
        lr_mean, lr_std = cast(Tuple[np.ndarray, np.ndarray], lr_pred)
        lr_mean = min_max_normalize(lr_mean)
        # gp_mean = hedac.gp_mean
        # gp_mean -= np.min(gp_mean)  # Shift to zero for better visualization
        # gp_mean /= np.max(gp_mean) + 1e-10  # Normalize for better visualization

        # Ground robots step
        states = ground_team.get_states()
        voronoi_masks = compute_anisotropic_voronoi_partitioning(
            xy_mpc_grid, states[:, :3], ground_params.fov_depth
        )
        ground_target_pdf = gmm.sample_pdf(xy_mpc_grid)
        ground_target_pdf = min_max_normalize(ground_target_pdf)
        grid_pts = []
        weights_list = []
        for idx, agent in enumerate(ground_team.agents):
            weights = ground_target_pdf * voronoi_masks[idx]
            weights /= (
                np.sum(weights) + 1e-10
            )  # Normalize weights for this agent's local grid
            weights_list.append(weights)

        for idx, agent in enumerate(ground_team.agents):
            voronoi_points = xy_mpc_grid[voronoi_masks[idx], :]
            grid_pts.append(voronoi_points)

        for idx, agent in enumerate(ground_team.agents):
            # Solve MPC for this agent
            local_grid = grid_pts[idx]
            weights = weights_list[idx]
            x0 = np.hstack([agent.position, agent.theta])
            p = np.concatenate([x0, weights_list[idx]])
            u0_guess = u_prev[idx, :].copy()
            sol = solver(
                x0=u0_guess,
                p=p,
                lbx=mpc_params.lbx,
                ubx=mpc_params.ubx,
                lbg=mpc_params.lbg,
                ubg=mpc_params.ubg,
            )
            u_opt = sol["x"].full().reshape(-1, mpc_params.nu)
            u_prev[idx, :] = sol["x"].full().flatten()  # Store for warm start
            # print(f"Agent {idx} MPC solution: ", u_opt)
            v, omega = u_opt[0]
            agent.step(v, omega)

        # plot
        axs[0].clear()
        axs[0].set_title("Ground Robots - Goal Density")
        axs[0].contourf(
            X_mpc,
            Y_mpc,
            ground_target_pdf.reshape(X_mpc.shape),
            cmap="RdPu",
            levels=10,
        )
        axs[0].set_aspect("equal")

        axs[1].clear()
        axs[1].set_title("Ground Robots - Local Voronoi Partitions")
        axs[1].set_aspect("equal")

        axs[2].clear()

        for ax in axs:
            for obs in mpc_params.x_obs:
                circle = plt.Circle(
                    obs, mpc_params.obstacles_radius, color="k", alpha=0.5
                )
                ax.add_patch(circle)

        for idx, agent in enumerate(ground_team.agents):
            local_grid = grid_pts[idx]
            weights = weights_list[idx]
            for ax in axs:
                # voronoi_points = mpc_params.xy_grid[voronoi_masks[idx], :]
                # ax.scatter(
                #     xy_mpc_grid[:, 0],
                #     xy_mpc_grid[:, 1],
                #     c=weights_list[idx],
                #     cmap=cmaps[idx],
                #     s=5,
                #     alpha=0.5,
                # )
                # ax.scatter(grid_pts[idx][:, 0], grid_pts[idx][:, 1], c=weights, cmap=cmaps[idx], s=5, alpha=0.5)
                ax.plot(
                    agent.position[0], agent.position[1], marker="o", color=colors[idx]
                )
                state = np.hstack([agent.position, agent.theta])
                fov_lines = agent_fov(
                    state, ground_params.fov_depth, np.deg2rad(ground_params.fov_deg)
                )
                ax.plot(fov_lines[:, 0], fov_lines[:, 1], color=colors[idx], alpha=0.5)
                # ax.scatter(voronoi_points[:, 0], voronoi_points[:, 1], c=colors[idx], s=10)

        for idx, agent in enumerate(aerial_team.agents):
            for ax in axs:
                ax.scatter(
                    agent.position[0], agent.position[1], marker="x", color=colors[idx]
                )
                state = np.hstack([agent.position, agent.theta])
                fov_lines = agent_fov(
                    state, params.fov_depth, np.deg2rad(params.fov_deg), num_points=30
                )
                ax.plot(fov_lines[:, 0], fov_lines[:, 1], color=colors[idx], alpha=0.5)
                ax.set_xlim(0, map_array.shape[1])
                ax.set_ylim(0, map_array.shape[0])

        plt.pause(0.01)

    plt.ioff()
    plt.show()

    # Plot results
    # if not args.no_plot:
    #     params = HEDACParams.from_yaml(args.aerial_config)
    #     plot_path = params.get("output.results_path", "output/results.npz")
    #     plot_path = plot_path.replace(".npz", ".png")
    #     plot_results(results, map_array, goal_density, agent_team, save_path=plot_path)
