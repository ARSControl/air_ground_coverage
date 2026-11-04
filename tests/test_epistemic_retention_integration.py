"""Paper admission is scheduled, bounded and transactional in the central GP."""

from dataclasses import replace

import numpy as np

from src.core.epistemic_sample_filter import EpistemicFilterSettings
from src.core.multifidelity_estimator import (
    CentralAsynchronousEstimator,
    EstimatorSettings,
    UpdateStatus,
)
from src.core.multifidelity_gp import MultiFidelityGaussianProcess, RBFKernel
from src.core.observations import (
    BoundedObservationBuffer,
    Fidelity,
    Observation,
    RetentionConfig,
)


def observation(t, x, fidelity=Fidelity.HIGH, noise=1e-8):
    return Observation(t, "robot", (x, 0.0), 1.0 if x == 0 else 0.2, fidelity, noise)


def settings(cap=10):
    return EstimatorSettings(
        gp=MultiFidelityGaussianProcess(0.8, RBFKernel(2.0, 1.0), RBFKernel(0.5, 0.25)),
        low_retention=RetentionConfig(cap, 0.0),
        high_retention=RetentionConfig(cap, 0.0),
        query_points=np.array([[0.0, 0.0], [5.0, 0.0], [10.0, 0.0]]),
        query_shape=(3,),
        integration_weights=np.ones(3),
        sample_filter=EpistemicFilterSettings(enabled=True),
    )


def test_buffer_admission_preserves_retained_and_clears_only_snapshot_pending():
    buffer = BoundedObservationBuffer(Fidelity.HIGH, RetentionConfig(1, 0.0))
    first = observation(0, 0)
    new = observation(1, 5)
    buffer.submit(first)
    c = buffer.candidate(pending_admission=(True,))
    buffer.commit(c)
    buffer.submit(new)
    c = buffer.candidate(pending_admission=(False,))
    assert c == (first,)
    buffer.submit(observation(2, 10))
    buffer.commit(c)
    assert buffer.retained == (first,)
    assert buffer.pending_count == 1


def test_redundant_rejected_informative_accepted_and_cap_does_not_forget():
    estimator = CentralAsynchronousEstimator(settings(cap=2))
    first = observation(0, 0)
    estimator.submit(first)
    assert estimator.update(0).status is UpdateStatus.UPDATED
    before = estimator.latest_posterior
    estimator.submit(observation(1, 0))
    assert estimator.update(1).status is UpdateStatus.NO_NEW_DATA
    assert estimator.latest_posterior is before
    assert estimator.pending_count == 0
    estimator.submit(observation(2, 10))
    assert estimator.update(2).status is UpdateStatus.UPDATED
    before = estimator.latest_posterior
    estimator.submit(observation(3, 5))
    assert estimator.update(3).status is UpdateStatus.NO_NEW_DATA
    assert estimator.latest_posterior is before
    assert estimator.filter_diagnostics["capacity_rejected"] == 1
    assert estimator.high_sample_count == 2
    assert (
        estimator.latest_posterior.high_variance[0] <= before.high_variance[0] + 1e-12
    )


def test_filter_failure_preserves_last_posterior_pending_and_diagnostics(monkeypatch):
    estimator = CentralAsynchronousEstimator(settings())
    estimator.submit(observation(0, 0))
    estimator.update(0)
    before = estimator.latest_posterior
    counters = estimator.filter_diagnostics
    estimator.submit(observation(1, 5))

    def fail(*args, **kwargs):
        raise FloatingPointError("filter failure")

    monkeypatch.setattr(MultiFidelityGaussianProcess, "predict_joint_latent", fail)
    assert estimator.update(1).status is UpdateStatus.FAILED
    assert estimator.latest_posterior is before
    assert estimator.pending_count == 1
    assert estimator.filter_diagnostics == counters
    monkeypatch.undo()
    assert estimator.update(2).status is UpdateStatus.UPDATED


def test_new_low_does_not_suppress_high_discrepancy_observation():
    estimator = CentralAsynchronousEstimator(settings())
    estimator.submit(observation(0, 0, Fidelity.LOW))
    estimator.update(0)
    previous = estimator.latest_posterior
    estimator.submit(observation(1, 0, Fidelity.HIGH))
    assert estimator.update(1).status is UpdateStatus.UPDATED
    assert estimator.high_sample_count == 1
    assert not np.allclose(estimator.latest_posterior.high_mean, previous.high_mean)


def test_legacy_disabled_filter_keeps_existing_newest_first_behavior():
    estimator = CentralAsynchronousEstimator(
        replace(settings(1), sample_filter=EpistemicFilterSettings())
    )
    estimator.submit(observation(0, 0))
    estimator.update(0)
    estimator.submit(observation(1, 10))
    estimator.update(1)
    assert estimator.latest_posterior.high_variance[0] > 0.8


def test_shared_budget_allows_unequal_split_and_preserves_old_samples():
    configured = replace(
        settings(cap=3),
        total_retention=3,
        sample_filter=EpistemicFilterSettings(
            enabled=True, criterion="high_information"
        ),
    )
    estimator = CentralAsynchronousEstimator(configured)
    estimator.submit_many([observation(0, x, Fidelity.LOW) for x in (0, 5, 10)])
    assert estimator.update(0).status is UpdateStatus.UPDATED
    assert estimator.low_sample_count == 3 and estimator.high_sample_count == 0
    previous = estimator.latest_posterior
    estimator.submit(observation(1, 20, Fidelity.HIGH))
    assert estimator.update(1).status is UpdateStatus.NO_NEW_DATA
    assert estimator.latest_posterior is previous
    assert estimator.filter_diagnostics["retained_total"] == 3
    assert estimator.filter_diagnostics["capacity_rejected"] == 1


def test_actual_uncorrelated_low_is_rejected_by_shared_high_objective():
    configured = replace(
        settings(cap=5),
        total_retention=5,
        gp=MultiFidelityGaussianProcess(0.0, RBFKernel(2.0, 1.0), RBFKernel(0.5, 0.25)),
        sample_filter=EpistemicFilterSettings(
            enabled=True, criterion="high_information"
        ),
    )
    estimator = CentralAsynchronousEstimator(configured)
    estimator.submit(observation(0, 0, Fidelity.LOW))
    assert estimator.update(0).status is UpdateStatus.NO_NEW_DATA
    assert estimator.low_sample_count == 0 and estimator.latest_posterior is None
    estimator.submit(observation(1, 0, Fidelity.HIGH))
    assert estimator.update(1).status is UpdateStatus.UPDATED
    assert estimator.high_sample_count == 1
