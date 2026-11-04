"""Paper Eqs. 18–19 and explicit bounded multi-fidelity adaptations."""

import numpy as np
import pytest

from src.core.epistemic_sample_filter import EpistemicFilterSettings, select_samples
from src.core.observations import Fidelity, Observation


def obs(t, x=0.0, fidelity=Fidelity.HIGH, noise=0.04):
    return Observation(t, "robot", (x, 0.0), 1.0, fidelity, noise)


def select(
    observations,
    covariance,
    *,
    low_mean=(0.0, 1.0),
    high_mean=(0.0, 1.0),
    retained=(),
    cap=250,
    error=0.05,
):
    return select_samples(
        tuple(observations),
        np.array(covariance, dtype=float),
        low_mean=np.array(low_mean),
        high_mean=np.array(high_mean),
        settings=EpistemicFilterSettings(enabled=True, relative_error=error),
        retained=tuple(retained),
        max_low=cap,
        max_high=cap,
    )


def test_standard_deviation_threshold_equality_and_noise_exclusion():
    threshold = 0.05 / 1.96
    result = select(
        [obs(0), obs(1, 1), obs(2, 2, noise=100)],
        np.diag([threshold**2, (threshold / 2) ** 2, (threshold / 2) ** 2]),
    )
    assert result.accepted == (True, False, False)
    assert result.high_threshold == pytest.approx(threshold)


def test_same_update_conditions_on_each_accepted_noisy_sample():
    # A near-exact sample removes almost all epistemic uncertainty for its twin.
    result = select([obs(0, noise=1e-8), obs(1, noise=1e-8)], [[1.0, 1.0], [1.0, 1.0]])
    assert result.accepted == (True, False)
    assert result.redundant == 1
    # Noisy repeated samples remain useful and can both be admitted.
    result = select([obs(0, noise=1), obs(1, noise=1)], [[1.0, 1.0], [1.0, 1.0]])
    assert result.accepted == (True, True)


def test_native_fidelity_thresholds_and_no_cross_fidelity_duplicate_removal():
    result = select(
        [obs(0, fidelity=Fidelity.LOW), obs(0)],
        np.diag([0.01, 0.01]),
        low_mean=(0.0, 10.0),
        high_mean=(0.0, 1.0),
    )
    assert result.accepted == (False, True)


def test_cap_keeps_existing_information_instead_of_age_eviction():
    old = obs(0)
    result = select([obs(1, 10)], [[1]], retained=(old,), cap=1)
    assert result.accepted == (False,)
    assert result.capacity_rejected == 1


def test_paper_duplicate_identity_and_flat_mean_bootstrap():
    old = obs(0)
    result = select([old, obs(1)], [[1, 0], [0, 1]], retained=(old,), high_mean=(0, 0))
    assert result.accepted == (False, True)
    assert result.duplicates == 1
    assert result.high_threshold == 0


@pytest.mark.parametrize(
    "kwargs",
    [
        {"relative_error": 0},
        {"relative_error": float("nan")},
        {"confidence_z": 0},
        {"enabled": 1},
    ],
)
def test_invalid_filter_settings(kwargs):
    with pytest.raises((ValueError, TypeError)):
        EpistemicFilterSettings(**kwargs)


def test_shared_budget_can_be_spent_entirely_on_one_fidelity():
    result = select_samples(
        tuple(obs(i, i, Fidelity.LOW) for i in range(3)),
        np.eye(3),
        high_cross_covariance=0.8 * np.eye(3),
        low_mean=np.array([0.0, 1.0]),
        high_mean=np.array([0.0, 1.0]),
        settings=EpistemicFilterSettings(enabled=True, criterion="high_information"),
        retained=(),
        max_low=3,
        max_high=3,
        max_total=3,
    )
    assert result.accepted == (True, True, True)


def test_common_high_objective_rejects_low_with_no_high_information():
    # HIGH uncertainty may be large, but an independent LOW measurement cannot reduce it.
    result = select_samples(
        (obs(0, 0, Fidelity.LOW), obs(1, 0)),
        np.eye(2),
        high_cross_covariance=np.array([[0.0, 0.0], [0.0, 1.0]]),
        low_mean=np.array([0.0, 100.0]),
        high_mean=np.array([0.0, 1.0]),
        settings=EpistemicFilterSettings(enabled=True, criterion="high_information"),
        retained=(),
        max_low=2,
        max_high=2,
        max_total=2,
    )
    assert result.accepted == (False, True)


def test_high_information_score_accounts_for_noise_and_cross_conditioning():
    threshold = 0.05 / 1.96
    # First sample explains the HIGH variable at both sites; second adds nothing useful.
    result = select_samples(
        (obs(0, noise=1e-8), obs(1, noise=1e-8)),
        np.ones((2, 2)),
        high_cross_covariance=np.ones((2, 2)),
        low_mean=np.array([0.0, 1.0]),
        high_mean=np.array([0.0, 1.0]),
        settings=EpistemicFilterSettings(enabled=True, criterion="high_information"),
        retained=(),
        max_low=2,
        max_high=2,
        max_total=2,
    )
    assert result.accepted == (True, False)
    result = select_samples(
        (obs(0, noise=100.0),),
        np.array([[1.0]]),
        high_cross_covariance=np.array([[threshold / 2]]),
        low_mean=np.array([0.0, 1.0]),
        high_mean=np.array([0.0, 1.0]),
        settings=EpistemicFilterSettings(enabled=True, criterion="high_information"),
        retained=(),
        max_low=2,
        max_high=2,
        max_total=2,
    )
    assert result.accepted == (False,)
