import os
import sys
import argparse
import numpy as np

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.core.base import HEDACParams, MapLoader
from src.core.hedac import HEDACAlgorithm
from src.models.agents import DoubleIntegratorAgent, DubinsAgent, AgentTeam
from src.utils.math_utils import create_gaussian_goal_density


def create_single_gaussian_goal(map_loader: MapLoader) -> np.ndarray:
    map_array = map_loader.load()
    height, width = map_array.shape
    x = np.arange(width)
    y = np.arange(height)
    grid_x, grid_y = np.meshgrid(x, y)
    grid_points = np.column_stack([grid_x.ravel(), grid_y.ravel()])

    mean = np.array([[width / 2.0, height / 2.0]])
    cov = np.array([[[width / 3.0, 0.0], [0.0, height / 3.0]]])

    return create_gaussian_goal_density(
        grid_points, mean, cov, map_array.shape, map_array
    )


def run_single_robot_gp(config_path: str):
    params = HEDACParams.from_yaml(config_path)
    np.random.seed(params.random_seed)

    map_config = params.map_config
    if "path" in map_config and map_config["path"] is not None:
        map_path = map_config["path"]
        if not os.path.isabs(map_path):
            config_dir = os.path.dirname(config_path)
            map_path = os.path.join(config_dir, map_path)
        map_loader = MapLoader(map_path=map_path, resolution=params.resolution)
    else:
        size = map_config.get("size", [50, 50])
        map_loader = MapLoader(size=tuple(size), resolution=params.resolution)

    goal_density = create_single_gaussian_goal(map_loader)

    map_array = map_loader.load()
    free_cells = map_loader.get_free_cells()
    if len(free_cells) > 0:
        start_idx = free_cells[np.random.choice(len(free_cells))]
        initial_pos = np.array([start_idx[1], start_idx[0]], dtype=float)
    else:
        initial_pos = np.array([5.0, 5.0], dtype=float)

    model_type = params.get("agents.model_type", "double_integrator")
    if model_type == "dubins":
        dubins_config = params.get("agents.dubins", {})
        agent = DubinsAgent(
            x0=initial_pos,
            theta0=np.random.uniform(0, 2 * np.pi),
            forward_speed=float(dubins_config.get("forward_speed", 5.0) or 5.0),
            max_bank_angle=float(dubins_config.get("max_bank_angle", 30.0) or 30.0),
            dt=params.dt_agent,
            agent_id=0,
            observations_range=params.get("sensor.fov_depth", 5.0),
            observations_count=int(params.get("gpr.obs_per_step", 10) or 10),
        )
    else:
        agent = DoubleIntegratorAgent(
            x0=initial_pos,
            theta0=np.random.uniform(0, 2 * np.pi),
            max_dx=params.max_dx,
            max_ddx=params.max_ddx,
            max_dtheta=params.max_dtheta,
            max_ddtheta=params.max_ddtheta,
            dt=params.dt_agent,
            agent_id=0,
            observations_range=params.get("sensor.fov_depth", 5.0),
            observations_count=int(params.get("gpr.obs_per_step", 10) or 10),
        )

    agent_team = AgentTeam([agent])

    hedac = HEDACAlgorithm(params, map_loader, goal_density)
    results = hedac.run(agent_team, num_steps=params.num_steps, verbose=True)

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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run single-robot GP-HEDAC simulation")
    parser.add_argument(
        "--config",
        "-c",
        type=str,
        default="configs/default_params.yaml",
        help="Path to YAML configuration file",
    )

    args = parser.parse_args()

    if not os.path.exists(args.config):
        alt_path = os.path.join("hedac", args.config)
        if os.path.exists(alt_path):
            args.config = alt_path
        else:
            print(f"Error: Config file not found: {args.config}")
            print(f"Also tried: {alt_path}")
            sys.exit(1)

    run_single_robot_gp(args.config)
