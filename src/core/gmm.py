import numpy as np
import numba
from numba import njit, prange


@njit(cache=True, parallel=True)
def _e_step_numba(
    points, means, cov_inv, coefficients, weights, n_components, n_points
):
    """JIT-compiled E-step: compute Gaussian probabilities and responsibilities."""
    responsibilities = np.empty((n_points, n_components))

    for k in prange(n_components):
        mean = means[k]
        inv = cov_inv[k]
        coef = coefficients[k]

        for i in range(n_points):
            diff_x = points[i, 0] - mean[0]
            diff_y = points[i, 1] - mean[1]

            temp_x = diff_x * inv[0, 0] + diff_y * inv[1, 0]
            temp_y = diff_x * inv[0, 1] + diff_y * inv[1, 1]
            exponent = -0.5 * (temp_x * diff_x + temp_y * diff_y)

            responsibilities[i, k] = weights[k] * coef * np.exp(exponent)

    return responsibilities


@njit(cache=True, parallel=True)
def _m_step_cov_numba(points, weighted_resp, means, n_components, n_points, N_k):
    """JIT-compiled M-step: update covariances."""
    covariances = np.empty((n_components, 2, 2))

    for k in prange(n_components):
        cov = np.zeros((2, 2))
        mean_k = means[k]

        for i in range(n_points):
            w = weighted_resp[i, k]
            diff_x = points[i, 0] - mean_k[0]
            diff_y = points[i, 1] - mean_k[1]

            cov[0, 0] += w * diff_x * diff_x
            cov[0, 1] += w * diff_x * diff_y
            cov[1, 0] += w * diff_y * diff_x
            cov[1, 1] += w * diff_y * diff_y

        cov[0, 0] = cov[0, 0] / N_k[k] + 1e-6
        cov[0, 1] = cov[0, 1] / N_k[k]
        cov[1, 0] = cov[1, 0] / N_k[k]
        cov[1, 1] = cov[1, 1] / N_k[k] + 1e-6

        covariances[k] = cov

    return covariances


@njit(cache=True, parallel=True)
def _compute_log_likelihood(
    points, means, cov_inv, coefficients, weights, n_components, n_points, weights_data
):
    """JIT-compiled log-likelihood computation."""
    log_likelihood = 0.0

    for i in prange(n_points):
        gmm_val = 0.0
        px = points[i, 0]
        py = points[i, 1]
        w_data = weights_data[i]

        for k in range(n_components):
            diff_x = px - means[k, 0]
            diff_y = py - means[k, 1]
            inv = cov_inv[k]

            temp_x = diff_x * inv[0, 0] + diff_y * inv[1, 0]
            temp_y = diff_x * inv[0, 1] + diff_y * inv[1, 1]
            exponent = -0.5 * (temp_x * diff_x + temp_y * diff_y)

            gmm_val += weights[k] * coefficients[k] * np.exp(exponent)

        log_likelihood += w_data * np.log(gmm_val + 1e-300)

    return log_likelihood


@njit(cache=True, parallel=True)
def _gauss_pdf_numba(points, mean, cov_inv, coefficient):
    """JIT-compiled multivariate Gaussian PDF for a single component."""
    n_points = points.shape[0]
    prob = np.empty(n_points)

    for i in prange(n_points):
        diff_x = points[i, 0] - mean[0]
        diff_y = points[i, 1] - mean[1]

        temp_x = diff_x * cov_inv[0, 0] + diff_y * cov_inv[1, 0]
        temp_y = diff_x * cov_inv[0, 1] + diff_y * cov_inv[1, 1]
        exponent = -0.5 * (temp_x * diff_x + temp_y * diff_y)

        prob[i] = coefficient * np.exp(exponent)

    return prob


class GMM:
    def __init__(
        self,
        means: np.ndarray,  # (K, 2)
        covariances: np.ndarray,  # (K, 2, 2)
        weights: np.ndarray,  # (K,)
    ):
        self.means = means
        self.covariances = covariances
        self.weights = weights
        self.n_components = means.shape[0]
        # Pre-compute inverses and determinants for efficiency
        self.cov_inv = np.linalg.inv(covariances)  # (K, 2, 2)
        self.cov_det = np.linalg.det(covariances)
        self.coefficients = 1.0 / np.sqrt((2.0 * np.pi) ** 2 * self.cov_det)  # (K,)

    def component_pdf(self, x, y, mean, cov_inv, coefficient):
        points = np.column_stack([x.flatten(), y.flatten()])
        return _gauss_pdf_numba(points, mean, cov_inv, coefficient).reshape(x.shape)

    def sample_pdf(
        self,
        x: np.ndarray,  # (N, 2) points where to evaluate the GMM PDF
    ) -> np.ndarray:  # (N,) GMM PDF values at the input points
        total_prob = np.zeros(x.shape[0])
        for k in range(self.n_components):
            total_prob += self.weights[k] * self.component_pdf(
                x[:, 0], x[:, 1], self.means[k], self.cov_inv[k], self.coefficients[k]
            )
        return total_prob

    def fit(self, density_map, grid_x, grid_y, max_iters=100, tol=1e-4):
        """
        Fit GMM to a 2D density map using EM algorithm.

        Args:
            density_map: (H, W) 2D array of density values
            grid_x: (H, W) x-coordinates corresponding to density_map
            grid_y: (H, W) y-coordinates corresponding to density_map
            max_iters: maximum number of EM iterations
            tol: convergence tolerance for log-likelihood
        """
        H, W = density_map.shape
        n_points = H * W

        # Flatten coordinates and density values
        points = np.column_stack([grid_x.flatten(), grid_y.flatten()])
        weights_data = density_map.flatten()
        weights_data = weights_data / np.sum(weights_data)

        prev_log_likelihood = -np.inf

        for iteration in range(max_iters):
            # E-step: JIT-compiled parallel computation
            responsibilities = _e_step_numba(
                points,
                self.means,
                self.cov_inv,
                self.coefficients,
                self.weights,
                self.n_components,
                n_points,
            )

            # Normalize responsibilities
            resp_sum = responsibilities.sum(axis=1)
            resp_sum[resp_sum < 1e-300] = 1.0
            responsibilities = responsibilities / resp_sum[:, np.newaxis]

            # Compute weighted responsibilities
            weighted_resp = responsibilities * weights_data[:, np.newaxis]

            # M-step: Update parameters
            N_k = weighted_resp.sum(axis=0)
            N_k[N_k < 1e-10] = 1e-10

            # Update means using vectorized operations
            self.means = (weighted_resp.T @ points) / N_k[:, np.newaxis]

            # Update covariances using JIT-compiled function
            self.covariances = _m_step_cov_numba(
                points, weighted_resp, self.means, self.n_components, n_points, N_k
            )

            # Update pre-computed values
            self.cov_inv = np.linalg.inv(self.covariances)
            self.cov_det = np.linalg.det(self.covariances)
            self.coefficients = 1.0 / np.sqrt((2.0 * np.pi) ** 2 * self.cov_det)

            # Update weights
            self.weights = N_k / N_k.sum()

            # Compute log-likelihood using JIT-compiled function
            log_likelihood = _compute_log_likelihood(
                points,
                self.means,
                self.cov_inv,
                self.coefficients,
                self.weights,
                self.n_components,
                n_points,
                weights_data,
            )

            # Check convergence
            if iteration > 0 and abs(log_likelihood - prev_log_likelihood) < tol:
                break

            prev_log_likelihood = log_likelihood
