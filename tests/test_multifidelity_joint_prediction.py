"""Mixed latent covariance needed for sequential, native-fidelity filtering."""

import numpy as np

from src.core.multifidelity_gp import MultiFidelityGaussianProcess, RBFKernel


def gp():
    return MultiFidelityGaussianProcess(
        0.8, RBFKernel(2.0, 1.2), RBFKernel(0.5, 0.3), jitter=1e-8
    )


def test_joint_prior_matches_autoregressive_blocks_and_is_immutable():
    model = gp()
    q = np.array([[0.0, 0.0], [0.0, 0.0]])
    result = model.predict_joint_latent(q, np.array([False, True]))
    np.testing.assert_allclose(result.mean, [0, 0])
    np.testing.assert_allclose(
        result.covariance, [[1.2, 0.8 * 1.2], [0.8 * 1.2, 0.8**2 * 1.2 + 0.3]]
    )
    assert not result.covariance.flags.writeable


def test_joint_posterior_matches_independent_conditioning_and_existing_high_api():
    model = gp()
    low = np.array([[0.0, 0.0]])
    high = np.array([[1.0, 0.0]])
    model.fit(low, [0.4], [0.04], high, [1.0], [0.001])
    query = np.array([[0.0, 0.0], [0.5, 0.0], [1.0, 0.0]])
    flags = np.array([False, True, False])
    prior = gp().predict_joint_latent(
        np.vstack((low, high, query)), np.array([False, True, *flags])
    )
    train = prior.covariance[:2, :2] + np.diag([0.04 + 1e-8, 0.001 + 1e-8])
    cross = prior.covariance[:2, 2:]
    actual = model.predict_joint_latent(query, flags)
    np.testing.assert_allclose(
        actual.mean, cross.T @ np.linalg.solve(train, [0.4, 1.0])
    )
    np.testing.assert_allclose(
        actual.covariance,
        prior.covariance[2:, 2:] - cross.T @ np.linalg.solve(train, cross),
        atol=1e-12,
    )
    all_high = model.predict_joint_latent(query, np.ones(3, dtype=bool))
    expected = model.predict_high(query)
    np.testing.assert_allclose(all_high.mean, expected.mean)
    np.testing.assert_allclose(
        np.diag(all_high.covariance), expected.variance, atol=1e-12
    )
    low_prediction = model.predict_low(query)
    all_low = model.predict_joint_latent(query, np.zeros(3, dtype=bool))
    np.testing.assert_allclose(low_prediction.mean, all_low.mean)
    np.testing.assert_allclose(
        low_prediction.variance, np.diag(all_low.covariance), atol=1e-12
    )
