"""Registration readiness checks for timestamped stationary samples."""

import pytest

from src.core.registration_readiness import PositionSample, prepare_registration


def sample(t, local, shared, reset=0):
    return PositionSample(t, local, shared, reset)


def snapshots():
    return {
        "a": [sample(10, (1, 2), (101, 202)), sample(11, (1, 2), (101, 202))],
        "b": [sample(10.05, (0, 0), (105, 208)), sample(11.05, (0, 0), (105, 208))],
    }


def test_stationary_snapshots_register_and_detect_resets():
    result = prepare_registration(snapshots(), "a", 11.1, 2, 0.1, 0.2)
    assert result.registration.to_common("b", (0, 0)).tolist() == [5, 8]
    assert result.resets_match({"a": 0, "b": 0})
    assert not result.resets_match({"a": 1, "b": 0})
    assert not result.resets_match({"a": 0})


def test_rejects_stale_future_skew_motion_and_reset():
    s = snapshots()
    with pytest.raises(ValueError, match="stale|future"):
        prepare_registration(s, "a", 14, 2, 0.1, 0.2)
    with pytest.raises(ValueError, match="stale|future"):
        prepare_registration(s, "a", 10, 2, 0.1, 0.2)
    with pytest.raises(ValueError, match="skew"):
        prepare_registration(s, "a", 11.1, 2, 0.01, 0.2)
    s["b"][1] = sample(11.05, (1, 0), (105, 208))
    with pytest.raises(ValueError, match="stationary"):
        prepare_registration(s, "a", 11.1, 2, 0.1, 0.2)
    s = snapshots()
    s["b"][1] = sample(11.05, (0, 0), (105, 208), 1)
    with pytest.raises(ValueError, match="reset"):
        prepare_registration(s, "a", 11.1, 2, 0.1, 0.2)


def test_rejects_missing_samples_nonfinite_and_invalid_thresholds():
    s = snapshots()
    s["b"] = []
    with pytest.raises(ValueError):
        prepare_registration(s, "a", 11.1, 2, 0.1, 0.2)
    with pytest.raises(ValueError):
        prepare_registration(snapshots(), "a", 11.1, 2, 0.1, float("nan"))
    s = snapshots()
    s["b"][0] = sample(float("nan"), (0, 0), (0, 0))
    with pytest.raises(ValueError):
        prepare_registration(s, "a", 11.1, 2, 0.1, 0.2)
