"""Run the reusable coupled simulation in legacy or multifidelity mode."""

from __future__ import annotations

import argparse
from collections.abc import Callable
from pathlib import Path
from typing import Any
import sys
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.core.base import HEDACParams
from src.coupled_config import load_coupled_configuration
from src.coupled_simulation import build_coupled_simulation
from src.utils.multifidelity_video import MultifidelityVideoRecorder

from examples.plot_final_multifidelity_state import (
    render_final_multifidelity_state,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        "-c",
        default="configs/multifidelity.yaml",
        help="Self-contained coupled aerial/ground configuration",
    )
    parser.add_argument(
        "--num-steps",
        type=int,
        default=None,
        help="Override simulation.num_steps for a smoke or diagnostic run",
    )
    parser.add_argument(
        "--seed", type=int, default=None, help="Override simulation.random_seed"
    )
    parser.add_argument(
        "--log-every",
        type=int,
        default=10,
        help="Print live progress every N completed steps; use 0 to disable",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Do not save the configured final summary plot",
    )
    return parser.parse_args()


def run_with_progress(
    simulation: Any,
    number_of_steps: int,
    log_every: int,
    emit: Callable[[str], None] = print,
    on_completed_step: Callable[[Any], None] | None = None,
):
    """Run one simulation while emitting deterministic step summaries."""
    for value, name in (
        (number_of_steps, "number_of_steps"),
        (log_every, "log_every"),
    ):
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{name} must be an integer")
        if value < 0:
            raise ValueError(f"{name} must be nonnegative")
    for step_num in range(number_of_steps):
        step_result = simulation.step(step_num)
        if on_completed_step is not None:
            on_completed_step(step_result)
        completed = step_num + 1
        if log_every > 0 and (
            completed % log_every == 0 or completed == number_of_steps
        ):
            emit(
                f"progress={completed}/{number_of_steps} "
                f"time={step_result.simulation_time:.3f} "
                f"ergodic_metric={step_result.ergodic_metric:.12g}"
            )
    return simulation.run(0)


def video_recorder_if_requested(params: HEDACParams) -> MultifidelityVideoRecorder | None:
    """Build an independent recorder from active visualization configuration."""
    enabled = params.get("visualization.save_video", False)
    if not isinstance(enabled, bool):
        raise TypeError("visualization.save_video must be a boolean")
    if not enabled:
        return None
    fps = params.get("visualization.video_fps", 30)
    interval = params.get("visualization.video_frame_interval", 10)
    output_path = params.get(
        "visualization.video_path", "output/multifidelity_simulation.gif"
    )
    if not isinstance(output_path, (str, Path)) or not str(output_path):
        raise ValueError("visualization.video_path must be a nonempty path")
    return MultifidelityVideoRecorder(
        output_path,
        fps=fps,
        frame_interval=interval,
    )


def save_final_plot_if_requested(
    params: HEDACParams,
    simulation: Any,
    result: Any,
    *,
    plots_disabled: bool,
    ground_params: HEDACParams | None = None,
    emit: Callable[[str], None] = print,
) -> Path | None:
    """Render the final state when enabled by the visualization configuration."""
    enabled = params.get("visualization.save_final_plot", False)
    if not isinstance(enabled, bool):
        raise TypeError("visualization.save_final_plot must be a boolean")
    if plots_disabled or not enabled:
        return None
    snapshot = simulation.latest_posterior
    if snapshot is None:
        emit("note=final plot skipped because no valid posterior is available")
        return None
    configured_path = params.get(
        "visualization.final_plot_path",
        "output/multifidelity_final_state.png",
    )
    if not isinstance(configured_path, (str, Path)) or not str(configured_path):
        raise ValueError("visualization.final_plot_path must be a nonempty path")
    output_path = render_final_multifidelity_state(
        result,
        snapshot,
        simulation.hedac.goal_density,
        Path(configured_path),
        ground_fov_degrees=(
            None if ground_params is None else ground_params.fov_deg
        ),
        ground_sensing_range=(
            None if ground_params is None else ground_params.fov_depth
        ),
        ground_final_states=(
            None
            if ground_params is None
            else simulation.ground_team.get_states()
        ),
    )
    emit(f"final_plot={output_path}")
    return output_path


def main() -> None:
    arguments = parse_args()
    configuration = load_coupled_configuration(arguments.config)
    aerial_params = configuration.aerial
    ground_params = configuration.ground
    seed = aerial_params.random_seed if arguments.seed is None else arguments.seed
    number_of_steps = (
        aerial_params.num_steps
        if arguments.num_steps is None
        else arguments.num_steps
    )
    print(
        f"building_simulation config={configuration.path} seed={seed}",
        flush=True,
    )
    simulation = build_coupled_simulation(aerial_params, ground_params, seed=seed)
    print(
        f"running mode={simulation.mode.value} steps={number_of_steps} "
        f"log_every={arguments.log_every}",
        flush=True,
    )
    recorder = video_recorder_if_requested(aerial_params)
    result = None
    video_path = None
    try:
        result = run_with_progress(
            simulation,
            number_of_steps,
            arguments.log_every,
            lambda message: print(message, flush=True),
            on_completed_step=(
                None
                if recorder is None
                else lambda step: recorder.capture(
                    step.step_num,
                    step.simulation_time,
                    simulation.latest_posterior,
                    simulation.aerial_team.get_positions(),
                    simulation.ground_team.get_positions(),
                )
            ),
        )
        if recorder is not None and result.step_results:
            final_step = result.step_results[-1]
            recorder.capture(
                final_step.step_num,
                final_step.simulation_time,
                simulation.latest_posterior,
                simulation.aerial_team.get_positions(),
                simulation.ground_team.get_positions(),
                force=True,
            )
    finally:
        if recorder is not None:
            video_path = recorder.close()
    if result is None:
        raise RuntimeError("simulation did not return a result")
    print(f"mode={simulation.mode.value}")
    print(f"steps={len(result.step_results)}")
    print(f"posterior_version={result.final_posterior_version}")
    print(f"aerial_density_source={result.aerial_density_source}")
    print(f"ground_density_source={result.ground_density_source}")
    if result.ergodic_metrics.size:
        print(f"final_ergodic_metric={result.ergodic_metrics[-1]:.12g}")
    snapshot = simulation.latest_posterior
    if snapshot is not None and hasattr(snapshot, "low_kernel_length_scale"):
        print(
            "kernel_hyperparameters="
            f"({snapshot.low_kernel_length_scale:.12g}, "
            f"{snapshot.low_kernel_variance:.12g}, "
            f"{snapshot.discrepancy_kernel_length_scale:.12g}, "
            f"{snapshot.discrepancy_kernel_variance:.12g})"
        )
    if recorder is not None:
        if video_path is None:
            print("note=video skipped because no valid posterior was published")
        else:
            print(f"video={video_path}")
        print(
            "hyperparameter_fit_performed_last_update="
            f"{snapshot.hyperparameter_fit_performed}"
        )
    save_final_plot_if_requested(
        aerial_params,
        simulation,
        result,
        plots_disabled=arguments.no_plot,
        ground_params=ground_params,
        emit=lambda message: print(message, flush=True),
    )


if __name__ == "__main__":
    main()
