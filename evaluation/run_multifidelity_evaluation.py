"""Run deterministic repeated episodes through the reusable coupled loop."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from src.coupled_config import load_coupled_configuration
from src.coupled_simulation import build_coupled_simulation


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        "-c",
        default="configs/multifidelity.yaml",
        help="Self-contained coupled aerial/ground configuration",
    )
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument("--num-steps", type=int, default=None)
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


def main() -> None:
    arguments = parse_args()
    configuration = load_coupled_configuration(arguments.config)
    aerial_params = configuration.aerial
    ground_params = configuration.ground
    episode_count = (
        aerial_params.num_episodes
        if arguments.episodes is None
        else arguments.episodes
    )
    number_of_steps = (
        aerial_params.num_steps
        if arguments.num_steps is None
        else arguments.num_steps
    )
    if episode_count < 1 or number_of_steps < 0:
        raise ValueError("episodes must be positive and num-steps nonnegative")

    final_metrics = np.empty(episode_count, dtype=float)
    posterior_versions = np.empty(episode_count, dtype=int)
    for episode in range(episode_count):
        seed = int(aerial_params.random_seed) + episode
        simulation = build_coupled_simulation(
            aerial_params, ground_params, seed=seed
        )
        result = simulation.run(number_of_steps)
        final_metrics[episode] = (
            np.nan if not result.ergodic_metrics.size else result.ergodic_metrics[-1]
        )
        posterior_versions[episode] = result.final_posterior_version

    print(f"episodes={episode_count}")
    print(f"mean_final_ergodic_metric={np.nanmean(final_metrics):.12g}")
    print(f"posterior_versions={posterior_versions.tolist()}")
    if arguments.output is not None:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        np.savez(
            arguments.output,
            final_ergodic_metrics=final_metrics,
            posterior_versions=posterior_versions,
        )


if __name__ == "__main__":
    main()
