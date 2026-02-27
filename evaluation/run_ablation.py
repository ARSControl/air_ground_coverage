import numpy as np
import os
import sys
import argparse
import itertools
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from evaluation.run_evaluation import (
    init_hedac_from_params,
    init_mpc_from_params,
)
from src.core.base import HEDACParams
from src.core.GaussianProcess import GaussianProcess
from src.utils.math_utils import min_max_normalize
from src.utils.eval_utils import (
    eval_effectiveness,
    eval_kl_divergence,
    eval_norm_effectiveness,
    eval_wasserstein
)
from src.utils.voronoi import compute_voronoi_partitioning


def run_ablation(
    aerial_config_path: str,
    ground_config_path: str,
    num_ground_robots: int,
    num_aerial_robots: int,
    output_dir: str,
) -> dict:
    """
    Run a single ablation experiment with specified number of robots.
    """
    print(f"\n{'=' * 60}")
    print(f"Running ablation: {num_aerial_robots} aerial, {num_ground_robots} ground")
    print(f"{'=' * 60}")

    np.random.seed(42)

    params = HEDACParams.from_yaml(aerial_config_path)
    ground_params = HEDACParams.from_yaml(ground_config_path)

    original_aerial_num = params.num_agents
    original_ground_num = ground_params.num_agents

    params.num_agents = num_aerial_robots
    ground_params.num_agents = num_ground_robots

    ground_params.dt = getattr(params, "dt", 0.1)

    EVAL = True
    num_episodes = params.num_episodes
    num_steps = params.num_steps

    cov_metrics = np.zeros((num_episodes, num_steps))
    effectiveness_metrics = np.zeros((num_episodes, num_steps))
    kl_divergence = np.zeros((num_episodes, num_steps))
    w_distances = np.zeros(num_episodes)

    for ep in range(num_episodes):
        if ep > 0:
            np.random.seed(42 + ep)

        aerial_team, hedac, map_array, target_density, gmm = init_hedac_from_params(
            params, config_path=aerial_config_path
        )

        ground_team, mpc_params, solver = init_mpc_from_params(ground_params, map_array)
        ground_gp = GaussianProcess(ground_params, map_array.shape)

        x_lr = np.arange(0, map_array.shape[1], params.resolution)
        y_lr = np.arange(0, map_array.shape[0], params.resolution)
        X_lr, Y_lr = np.meshgrid(x_lr, y_lr)
        xy_lr_grid = np.column_stack([X_lr.flatten(), Y_lr.flatten()])

        x_hr = np.arange(0, map_array.shape[1], ground_params.resolution)
        y_hr = np.arange(0, map_array.shape[0], ground_params.resolution)
        X_hr, Y_hr = np.meshgrid(x_hr, y_hr)
        xy_hr_grid = np.column_stack([X_hr.flatten(), Y_hr.flatten()])

        high_res_target_pdf = gmm.sample_pdf(xy_hr_grid).reshape(X_hr.shape)

        x_mpc = np.linspace(0, map_array.shape[1], ground_params.local_grid_points)
        y_mpc = np.linspace(0, map_array.shape[0], ground_params.local_grid_points)
        X_mpc, Y_mpc = np.meshgrid(x_mpc, y_mpc)
        xy_mpc_grid = np.column_stack([X_mpc.flatten(), Y_mpc.flatten()])

        ground_target_pdf = gmm.sample_pdf(xy_mpc_grid)
        ground_target_pdf /= np.sum(ground_target_pdf) + 1e-10
        # ground_target_pdf = min_max_normalize(ground_target_pdf)

        u_prev = np.zeros(
            (ground_params.num_agents, mpc_params.nu * ground_params.mpc_horizon)
        )

        for step in range(num_steps):
            if params.num_agents > 0:
                hedac.step(aerial_team, step_num=step)

                aerial_gp_mean, aerial_gp_std = hedac.gpr_model.predict(
                    xy_mpc_grid, return_std=True
                )
                aerial_gp_std /= np.max(aerial_gp_std) + 1e-10

            if ground_params.num_agents > 0:
                new_observations = ground_gp.collect_observations(
                    ground_team, target_density
                )
                ground_gp.update_gp(new_observations, step)
                ground_gp_mean, ground_gp_std = ground_gp.predict(
                    xy_mpc_grid, return_std=True
                )
                ground_gp_std /= np.max(ground_gp_std) + 1e-10

                states = ground_team.get_states()
                voronoi_masks = compute_voronoi_partitioning(
                    xy_mpc_grid, states[:, :2], 50.0
                )

                if params.num_agents > 0:
                    den = 1 / (ground_gp_std + 1e-10) + 1 / (aerial_gp_std + 1e-10)
                    w_a = 1 / (aerial_gp_std + 1e-10) / den
                    w_g = 1 / (ground_gp_std + 1e-10) / den
                    combo_density = w_a * aerial_gp_mean + w_g * ground_gp_mean
                else:
                    combo_density = ground_gp_mean.copy()

                weights_list = []
                for idx, agent in enumerate(ground_team.agents):
                    weights = combo_density * voronoi_masks[idx]
                    weights_list.append(weights)

                grid_pts = []
                for idx, agent in enumerate(ground_team.agents):
                    voronoi_points = xy_mpc_grid[voronoi_masks[idx], :]
                    grid_pts.append(voronoi_points)

                for idx, agent in enumerate(ground_team.agents):
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
                    u_prev[idx, :] = sol["x"].full().flatten()
                    v, omega = u_opt[0]
                    agent.step(v, omega)

            if EVAL:
                if ground_params.num_agents > 0:
                    ground_states = ground_team.get_states()
                    effectiveness = eval_norm_effectiveness(
                        ground_states,
                        xy_hr_grid,
                        high_res_target_pdf,
                        fov_degrees=ground_params.fov_deg,
                        robot_range=ground_params.sens_range,
                        env_area=map_array.shape[0] * map_array.shape[1],
                    )
                    effectiveness_metrics[ep, step] = effectiveness
                if params.num_agents == 0:
                    eval_density = ground_gp_mean
                elif ground_params.num_agents == 0:
                    eval_density = aerial_gp_mean
                else:
                    eval_density = combo_density
                eval_density -= np.min(eval_density)
                eval_density /= np.max(eval_density) + 1e-10
                kl_divergence[ep, step] = eval_kl_divergence(
                    ground_target_pdf, eval_density
                )

        w_distances[ep] = eval_wasserstein(ground_target_pdf, eval_density)
        print(
            f"Episode {ep + 1} | KL: {kl_divergence[ep, -1]:.4f} | Effectiveness: {effectiveness_metrics[ep, -1]:.4f} | Wasserstein: {w_distances[ep]:.4f}"
        )

    params.num_agents = original_aerial_num
    ground_params.num_agents = original_ground_num

    os.makedirs(output_dir, exist_ok=True)

    exp_name = f"aerial{num_aerial_robots}_ground{num_ground_robots}"
    np.save(
        os.path.join(output_dir, f"effectiveness_{exp_name}.npy"), effectiveness_metrics
    )
    np.save(os.path.join(output_dir, f"kl_divergence_{exp_name}.npy"), kl_divergence)
    np.save(os.path.join(output_dir, f"wasserstein_{exp_name}.npy"), w_distances)

    results = {
        "num_aerial": num_aerial_robots,
        "num_ground": num_ground_robots,
        "mean_effectiveness": np.mean(effectiveness_metrics[:, -1]),
        "std_effectiveness": np.std(effectiveness_metrics[:, -1]),
        "mean_kl": np.mean(kl_divergence[:, -1]),
        "std_kl": np.std(kl_divergence[:, -1]),
        "mean_wasserstein": np.mean(w_distances),
        "std_wasserstein": np.std(w_distances),
    }

    print(
        f"Results: KL={results['mean_kl']:.4f}±{results['std_kl']:.4f}, Effectiveness={results['mean_effectiveness']:.4f}±{results['std_effectiveness']:.4f} | Wasserstein={results['mean_wasserstein']:.4f}±{results['std_wasserstein']:.4f}"
    )

    return results


def main():
    parser = argparse.ArgumentParser(
        description="Run ablation study on robot team size"
    )
    parser.add_argument(
        "--aerial_config",
        "-ac",
        type=str,
        default="configs/iros26_aerial.yaml",
        help="Path to aerial robots config",
    )
    parser.add_argument(
        "--ground_config",
        "-gc",
        type=str,
        default="configs/iros26_ground.yaml",
        help="Path to ground robots config",
    )
    parser.add_argument(
        "--total_robots",
        "-t",
        type=int,
        default=10,
        help="Total number of robots (aerial + ground)",
    )
    parser.add_argument(
        "--output_dir",
        "-o",
        type=str,
        default="output/ablation_fixedtotal",
        help="Output directory for results",
    )

    args = parser.parse_args()

    aerial_config = args.aerial_config
    ground_config = args.ground_config

    if not os.path.exists(aerial_config):
        alt_path = os.path.join("hedac", aerial_config)
        if os.path.exists(alt_path):
            aerial_config = alt_path

    if not os.path.exists(ground_config):
        alt_path = os.path.join("hedac", ground_config)
        if os.path.exists(alt_path):
            ground_config = alt_path

    aerial_robots_list = [0, 1, 2, 3, 4]
    ground_robots_list = [args.total_robots - a for a in aerial_robots_list]
    # aerial_robots_list = [5]
    # ground_robots_list = [0]

    combinations = list(zip(aerial_robots_list, ground_robots_list))

    print(
        f"Running ablation study with {len(combinations)} combinations (total={args.total_robots}):"
    )
    for n_a, n_g in combinations:
        print(f"  - {n_a} aerial, {n_g} ground")

    all_results = []

    for num_aerial, num_ground in combinations:
        result = run_ablation(
            aerial_config_path=aerial_config,
            ground_config_path=ground_config,
            num_ground_robots=num_ground,
            num_aerial_robots=num_aerial,
            output_dir=args.output_dir,
        )
        all_results.append(result)

    results_summary_path = os.path.join(args.output_dir, "ablation_results.npy")
    np.save(results_summary_path, all_results)

    print(f"\n{'=' * 60}")
    print("Ablation Study Complete")
    print(f"{'=' * 60}")
    print(f"Results saved to: {args.output_dir}")
    print("\nSummary:")
    print(f"{'Aerial':<8} {'Ground':<8} {'KL Div':<12} {'Effectiveness':<15}")
    print("-" * 45)
    for r in all_results:
        print(
            f"{r['num_aerial']:<8} {r['num_ground']:<8} {r['mean_kl']:.4f}±{r['std_kl']:.4f}  {r['mean_effectiveness']:.4f}±{r['std_effectiveness']:.4f}"
        )


if __name__ == "__main__":
    main()
