"""Exact swap scores must match independent brute-force GP refits."""

import numpy as np
import pytest

from src.core.multifidelity_gp import MultiFidelityGaussianProcess, RBFKernel


def model():
    return MultiFidelityGaussianProcess(0.8, RBFKernel(2.0, 1.0), RBFKernel(0.6, 0.25))


def fit(observations):
    gp = model()
    arrays = []
    for high in (False, True):
        rows = [o for o in observations if o[0] is high]
        arrays.extend(
            (
                np.array([o[1] for o in rows]).reshape((-1, 2)),
                np.array([o[2] for o in rows]),
                np.array([o[3] for o in rows]),
            )
        )
    gp.fit(*arrays)
    return gp


def test_all_swap_scores_match_brute_force_with_noisy_mixed_fidelities():
    retained = [
        (False, [0.0, 0.0], 0.3, 0.04),
        (False, [0.05, 0.0], 0.31, 0.04),
        (True, [3.0, 0.0], 1.0, 0.001),
    ]
    candidates = [(True, [0.0, 0.0], 0.9, 0.0001), (False, [7.0, 0.0], 0.1, 0.04)]
    gp = fit(retained)
    query = np.array([[0.0, 0.0], [1.0, 0.0], [3.0, 0.0], [7.0, 0.0]])
    weights = np.array([1.0, 0.5, 2.0, 0.0])
    scores = gp.score_replacements(
        query,
        weights,
        np.array([o[1] for o in candidates]),
        np.array([o[0] for o in candidates]),
        np.array([o[3] for o in candidates]),
    )
    before = np.average(gp.predict_high(query).variance, weights=weights)
    assert scores.objective_before == pytest.approx(before)
    for old in range(len(retained)):
        for new, candidate in enumerate(candidates):
            other = fit([o for i, o in enumerate(retained) if i != old] + [candidate])
            actual = np.average(other.predict_high(query).variance, weights=weights)
            assert scores.improvement[old, new] == pytest.approx(
                before - actual, abs=1e-10
            )
    assert not scores.improvement.flags.writeable


def test_score_prefers_replacing_redundant_low_with_nearby_high():
    retained = [
        (False, [0.0, 0.0], 0.3, 0.04),
        (False, [0.05, 0.0], 0.3, 0.04),
        (True, [10.0, 0.0], 1.0, 0.0001),
    ]
    gp = fit(retained)
    scores = gp.score_replacements(
        np.array([[0.0, 0.0], [10.0, 0.0]]),
        np.ones(2),
        np.array([[0.0, 0.0]]),
        np.array([True]),
        np.array([0.0001]),
    )
    old, new = np.unravel_index(np.argmax(scores.improvement), scores.improvement.shape)
    assert old in (0, 1) and new == 0
    assert scores.improvement[old, new] > 0


def test_replacement_geometry_cache_reuses_solves_and_invalidates_after_fit(
    monkeypatch,
):
    import src.core.multifidelity_gp as module

    retained = [(False, [0.0, 0.0], 0.3, 0.04), (True, [3.0, 0.0], 1.0, 0.001)]
    gp = fit(retained)
    query = np.array([[0.0, 0.0], [3.0, 0.0]])
    weights = np.ones(2)
    original = module.solve_triangular
    calls = []

    def counted(*args, **kwargs):
        calls.append(1)
        return original(*args, **kwargs)

    monkeypatch.setattr(module, "solve_triangular", counted)
    arguments = (
        query,
        weights,
        np.array([[1.0, 0.0]]),
        np.array([True]),
        np.array([0.001]),
    )
    first = gp.score_replacements(*arguments)
    initial = len(calls)
    second = gp.score_replacements(*arguments)
    assert (
        len(calls) - initial == 2
    )  # Only candidate projections, not retained geometry.
    np.testing.assert_array_equal(first.improvement, second.improvement)
    gp.fit(
        np.array([[2.0, 0.0]]), [0.5], [0.04], np.array([[3.0, 0.0]]), [1.0], [0.001]
    )
    initial = len(calls)
    third = gp.score_replacements(*arguments)
    assert len(calls) - initial > 2
    assert not np.allclose(first.improvement, third.improvement)


def test_geometry_cache_does_not_reuse_a_different_query_weighting():
    retained = [(False, [0.0, 0.0], 0.3, 0.04), (True, [3.0, 0.0], 1.0, 0.001)]
    gp = fit(retained)
    query = np.array([[0.0, 0.0], [3.0, 0.0]])
    args = (np.array([[1.0, 0.0]]), np.array([True]), np.array([0.001]))
    left = gp.score_replacements(query, np.array([1.0, 0.0]), *args)
    right = gp.score_replacements(query, np.array([0.0, 1.0]), *args)
    fresh = fit(retained).score_replacements(query, np.array([0.0, 1.0]), *args)
    assert not np.allclose(left.improvement, right.improvement)
    np.testing.assert_array_equal(right.improvement, fresh.improvement)
