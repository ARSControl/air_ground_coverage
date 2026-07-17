"""Deterministic contracts for multi-fidelity video rendering."""

from __future__ import annotations

from contextlib import contextmanager
from types import SimpleNamespace

import numpy as np

from src.utils.multifidelity_video import MultifidelityVideoRecorder


def _snapshot() -> SimpleNamespace:
    grid_x, grid_y = np.meshgrid(np.linspace(0.0, 2.0, 3), np.linspace(0.0, 2.0, 3))
    query_points = np.column_stack((grid_x.ravel(), grid_y.ravel()))
    return SimpleNamespace(
        query_points=query_points,
        query_shape=grid_x.shape,
        high_mean=np.linspace(0.1, 0.9, query_points.shape[0]),
        high_variance=np.linspace(0.02, 0.1, query_points.shape[0]),
        density=np.full(query_points.shape[0], 1.0 / query_points.shape[0]),
        version=4,
    )


class _FakeWriter:
    def __init__(self) -> None:
        self.frames = 0

    @contextmanager
    def saving(self, figure, output_path, dpi):
        output_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            yield self
        finally:
            output_path.write_bytes(b"synthetic-video")

    def grab_frame(self) -> None:
        self.frames += 1


def test_recorder_samples_configured_steps_and_forces_final_frame(tmp_path) -> None:
    writer = _FakeWriter()
    recorder = MultifidelityVideoRecorder(
        tmp_path / "recording.mp4",
        fps=12,
        frame_interval=2,
        writer_factory=lambda path, fps: writer,
    )
    snapshot = _snapshot()
    positions = np.array([[0.5, 0.5], [1.5, 1.5]])

    assert recorder.capture(0, 0.0, snapshot, positions[:1], positions[1:])
    initial_uncertainty_vmax = recorder.uncertainty_vmax
    assert initial_uncertainty_vmax == np.sqrt(np.max(snapshot.high_variance))
    assert not recorder.capture(1, 0.1, snapshot, positions[:1], positions[1:])
    assert recorder.capture(2, 0.2, snapshot, positions[:1], positions[1:])
    assert recorder.uncertainty_vmax == initial_uncertainty_vmax
    assert recorder.capture(2, 0.2, snapshot, positions[:1], positions[1:], force=True) is False
    assert recorder.capture(3, 0.3, snapshot, positions[:1], positions[1:], force=True)

    assert recorder.close() == tmp_path / "recording.mp4"
    assert writer.frames == 3
    assert (tmp_path / "recording.mp4").read_bytes() == b"synthetic-video"


def test_recorder_skips_frames_without_a_posterior(tmp_path) -> None:
    recorder = MultifidelityVideoRecorder(
        tmp_path / "empty.mp4",
        fps=12,
        frame_interval=1,
        writer_factory=lambda path, fps: _FakeWriter(),
    )
    assert not recorder.capture(0, 0.0, None, np.empty((0, 2)), np.empty((0, 2)))
    assert recorder.close() is None
    assert not (tmp_path / "empty.mp4").exists()
