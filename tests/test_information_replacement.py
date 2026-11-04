"""Scheduled replacement improves the HIGH objective and remains transactional."""

import numpy as np
import pytest

from src.core.epistemic_sample_filter import EpistemicFilterSettings
from src.core.multifidelity_estimator import (
    CentralAsynchronousEstimator,
    EstimatorSettings,
    UpdateStatus,
)
from src.core.multifidelity_gp import (
    MultiFidelityGaussianProcess,
    RBFKernel,
    ReplacementScores,
)
from src.core.observations import Fidelity, Observation, RetentionConfig


def obs(t, x, fidelity=Fidelity.HIGH, noise=0.0001):
    return Observation(t, "robot", (x, 0.0), 1.0 if x == 0 else 0.2, fidelity, noise)


def estimator(cap=3, query=None):
    query = np.array([[0.0, 0.0], [10.0, 0.0]]) if query is None else query
    return CentralAsynchronousEstimator(
        EstimatorSettings(
            gp=MultiFidelityGaussianProcess(
                0.8, RBFKernel(2.0, 1.0), RBFKernel(0.6, 0.25)
            ),
            low_retention=RetentionConfig(cap, 0.0),
            high_retention=RetentionConfig(cap, 0.0),
            query_points=query,
            query_shape=(len(query),),
            integration_weights=np.ones(len(query)),
            total_retention=cap,
            sample_filter=EpistemicFilterSettings(
                enabled=True, criterion="high_information", on_full="information"
            ),
        )
    )


def filled():
    result = estimator()
    # The oldest HIGH sample is essential; later LOW samples are redundant.
    result.submit_many(
        (obs(0, 10), obs(1, 0, Fidelity.LOW, 0.04), obs(2, 0.05, Fidelity.LOW, 0.04))
    )
    assert result.update(2).status is UpdateStatus.UPDATED
    return result


def test_new_high_replaces_redundant_low_and_preserves_important_oldest():
    model = filled()
    before = model.latest_posterior
    model.submit(obs(3, 0))
    assert model.update(3).status is UpdateStatus.UPDATED
    after = model.latest_posterior
    assert (after.low_sample_count, after.high_sample_count) == (1, 2)
    assert np.mean(after.high_variance) < np.mean(before.high_variance)
    assert after.high_variance[0] < before.high_variance[0]
    assert model.filter_diagnostics["replaced"] == 1
    assert model.filter_diagnostics["evicted_low"] == 1
    assert model.filter_diagnostics["evicted_high"] == 0
    assert model.filter_diagnostics["retained_total"] == 3
    assert not np.allclose(after.high_mean, before.high_mean)


def test_best_one_swap_per_update_and_other_candidates_are_accounted_for():
    model = filled()
    model.submit_many((obs(3, 0, noise=0.001), obs(3, 0, noise=0.0001)))
    assert model.update(3).status is UpdateStatus.UPDATED
    assert model.filter_diagnostics["replaced"] == 1
    assert model.filter_diagnostics["capacity_rejected"] == 1
    assert any(
        o.timestamp == 3 and o.noise_variance == 0.0001
        for o in model._high_buffer.retained
    )


def test_new_sample_rejected_when_every_swap_worsens_high_objective():
    model = estimator(1, np.array([[0.0, 0.0]]))
    first = obs(0, 0)
    model.submit(first)
    model.update(0)
    before = model.latest_posterior
    model.submit(obs(1, 10))
    assert model.update(1).status is UpdateStatus.NO_NEW_DATA
    assert model.latest_posterior is before
    assert model.filter_diagnostics["replaced"] == 0
    assert model.filter_diagnostics["capacity_rejected"] == 1
    assert model.filter_diagnostics["replacement_evaluations"] == 1


def test_swap_fit_failure_preserves_old_data_and_retries(monkeypatch):
    model = filled()
    before = model.latest_posterior
    counters = model.filter_diagnostics
    model.submit(obs(3, 0))

    def fail(*args, **kwargs):
        raise FloatingPointError("injected swap failure")

    monkeypatch.setattr(MultiFidelityGaussianProcess, "fit", fail)
    assert model.update(3).status is UpdateStatus.FAILED
    assert model.latest_posterior is before and model.pending_count == 1
    assert model.filter_diagnostics == counters
    assert (model.low_sample_count, model.high_sample_count) == (2, 1)
    monkeypatch.undo()
    assert model.update(4).status is UpdateStatus.UPDATED
    assert model.filter_diagnostics["replaced"] == 1


def test_final_actual_posterior_guard_rejects_bad_predicted_swap(monkeypatch):
    model = estimator(1, np.array([[0.0, 0.0]]))
    model.submit(obs(0, 0))
    model.update(0)
    before = model.latest_posterior

    def misleading(*args, **kwargs):
        return ReplacementScores(np.ones((1, 1)), float(before.high_variance.mean()))

    monkeypatch.setattr(MultiFidelityGaussianProcess, "score_replacements", misleading)
    model.submit(obs(1, 10))
    assert model.update(1).status is UpdateStatus.NO_NEW_DATA
    assert model.latest_posterior is before and model.pending_count == 0
    assert model.filter_diagnostics["replaced"] == 0


@pytest.mark.parametrize(
    "kwargs",
    [
        {"on_full": "oldest"},
        {"replacement_min_improvement": -1},
        {"replacement_min_improvement": float("nan")},
    ],
)
def test_invalid_replacement_settings(kwargs):
    with pytest.raises(ValueError):
        EpistemicFilterSettings(**kwargs)


def test_replacement_fits_the_joint_gp_only_once(monkeypatch):
    model = filled()
    original = MultiFidelityGaussianProcess.fit
    calls = []

    def counted(self, *args, **kwargs):
        calls.append(1)
        return original(self, *args, **kwargs)

    monkeypatch.setattr(MultiFidelityGaussianProcess, "fit", counted)
    model.submit_many((obs(3, 0, noise=0.001), obs(3, 0, noise=0.0001)))
    assert model.update(3).status is UpdateStatus.UPDATED
    assert len(calls) == 1
