"""Tests for live progress reporting in the multifidelity CLI."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from examples.run_multifidelity import run_with_progress, video_recorder_if_requested
from src.core.base import HEDACParams


class FakeSimulation:
    def __init__(self) -> None:
        self.steps: list[int] = []
        self.final_result = object()

    def step(self, step_num: int):
        self.steps.append(step_num)
        return SimpleNamespace(
            step_num=step_num,
            simulation_time=step_num * 0.1,
            posterior_version=step_num // 5,
            aerial_density_source="multifidelity_high_posterior",
            ground_density_source="multifidelity_high_posterior",
            ergodic_metric=1.0 / (step_num + 1.0),
        )

    def run(self, number_of_steps: int):
        assert number_of_steps == 0
        return self.final_result


def test_progress_is_emitted_at_interval_and_final_step() -> None:
    simulation = FakeSimulation()
    messages: list[str] = []
    result = run_with_progress(simulation, 21, 10, messages.append)

    assert result is simulation.final_result
    assert simulation.steps == list(range(21))
    assert [message.split()[0] for message in messages] == [
        "progress=10/21",
        "progress=20/21",
        "progress=21/21",
    ]
    assert all("posterior_version=" in message for message in messages)
    assert all("ergodic_metric=" in message for message in messages)


def test_zero_interval_disables_progress_without_disabling_simulation() -> None:
    simulation = FakeSimulation()
    messages: list[str] = []
    run_with_progress(simulation, 3, 0, messages.append)
    assert simulation.steps == [0, 1, 2]
    assert messages == []


def test_completed_step_callback_observes_every_completed_step() -> None:
    simulation = FakeSimulation()
    observed_steps: list[int] = []
    run_with_progress(
        simulation,
        3,
        0,
        on_completed_step=lambda result: observed_steps.append(result.step_num),
    )
    assert observed_steps == [0, 1, 2]


def test_video_recorder_uses_active_visualization_values(tmp_path) -> None:
    output_path = tmp_path / "run.gif"
    recorder = video_recorder_if_requested(
        HEDACParams.from_dict(
            {
                "visualization": {
                    "save_video": True,
                    "video_fps": 11,
                    "video_frame_interval": 3,
                    "video_path": str(output_path),
                }
            }
        )
    )
    assert recorder is not None
    assert recorder.output_path == output_path
    assert recorder.fps == 11
    assert recorder.frame_interval == 3
    assert recorder.close() is None


@pytest.mark.parametrize(
    ("number_of_steps", "log_every", "exception"),
    [(-1, 10, ValueError), (1, -1, ValueError), (True, 10, TypeError)],
)
def test_progress_arguments_are_validated(
    number_of_steps: int, log_every: int, exception: type[Exception]
) -> None:
    with pytest.raises(exception):
        run_with_progress(FakeSimulation(), number_of_steps, log_every)
