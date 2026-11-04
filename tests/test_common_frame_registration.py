"""Contracts for shared-reference registration of aligned NED frames."""

import numpy as np
import pytest

from src.core.common_frame import register_aligned_frames


def test_registration_recovers_offsets_and_round_trips() -> None:
    local = {"uav_1": [2.0, -1.0], "uav_2": [-3.0, 4.0]}
    shared = {"uav_1": [102.0, 199.0], "uav_2": [107.0, 214.0]}
    frame = register_aligned_frames(local, shared, "uav_1")
    np.testing.assert_allclose(frame.to_common("uav_1", local["uav_1"]), [2, -1])
    np.testing.assert_allclose(frame.to_common("uav_2", local["uav_2"]), [7, 14])
    np.testing.assert_allclose(frame.to_local("uav_2", [7, 14]), local["uav_2"])
    np.testing.assert_allclose(frame.to_common("uav_2", [0, 0]), [10, 10])


@pytest.mark.parametrize(
    ("local", "shared", "reference"),
    [
        ({}, {}, "uav_1"),
        ({"uav_1": [0, 0]}, {}, "uav_1"),
        ({"uav_1": [0, 0]}, {"uav_1": [0, 0]}, "missing"),
        ({"uav_1": [float("nan"), 0]}, {"uav_1": [0, 0]}, "uav_1"),
        ({"uav_1": [0, 0, 0]}, {"uav_1": [0, 0]}, "uav_1"),
    ],
)
def test_rejects_invalid_snapshots(local, shared, reference) -> None:
    with pytest.raises((ValueError, TypeError)):
        register_aligned_frames(local, shared, reference)


def test_registration_is_not_mutated_by_input_or_output() -> None:
    local = {"a": [0.0, 0.0], "b": [0.0, 0.0]}
    shared = {"a": [0.0, 0.0], "b": [2.0, 3.0]}
    frame = register_aligned_frames(local, shared, "a")
    shared["b"][0] = 99
    result = frame.to_common("b", [0, 0])
    result[0] = 88
    np.testing.assert_allclose(frame.to_common("b", [0, 0]), [2, 3])
    with pytest.raises(KeyError):
        frame.to_common("unknown", [0, 0])
