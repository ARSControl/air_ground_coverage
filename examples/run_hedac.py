import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import argparse

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.core.base import HEDACParams, MapLoader
from src.core.hedac import HEDACAlgorithm
from src.models.agents import DoubleIntegratorAgent, DubinsAgent, AgentTeam
from src.utils.math_utils import min_max_normalize


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
        "--config",
        "-c",
        type=str,
        default="configs/default_params.yaml",
        help="Path to YAML configuration file",
    )
    parser.add_argument("--no-plot", action="store_true", help="Disable plotting")

    args = parser.parse_args()

    # Check if config file exists
    if not os.path.exists(args.config):
        # Try with hedac prefix
        alt_path = os.path.join("hedac", args.config)
        if os.path.exists(alt_path):
            args.config = alt_path
        else:
            print(f"Error: Config file not found: {args.config}")
            print(f"Also tried: {alt_path}")
            sys.exit(1)

    # Run simulation
    results, map_array, goal_density, agent_team = run_hedac_from_config(args.config)

    # Plot results
    if not args.no_plot:
        params = HEDACParams.from_yaml(args.config)
        plot_path = params.get("output.results_path", "output/results.npz")
        plot_path = plot_path.replace(".npz", ".png")
        plot_results(results, map_array, goal_density, agent_team, save_path=plot_path)
