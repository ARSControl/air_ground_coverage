import numpy as np
from scipy.stats import multivariate_normal
import matplotlib.pyplot as plt

# ----------------------------- Domain & density -----------------------------

class Domain:
    def __init__(self, xmin, xmax, ymin, ymax, grid_res=80):
        self.bbox = (xmin, xmax, ymin, ymax)
        xs = np.linspace(xmin, xmax, grid_res)
        ys = np.linspace(ymin, ymax, grid_res)
        X, Y = np.meshgrid(xs, ys)
        self.grid = np.stack([X.ravel(), Y.ravel()], axis=1)
        dx = (xmax - xmin) / (grid_res - 1)
        dy = (ymax - ymin) / (grid_res - 1)
        self.cell_area = dx * dy

def bivariate_gaussian_density(points, means, cov):
    """phi^G(q): sum of Gaussians, later normalized to a probability density."""
    val = np.zeros(points.shape[0])
    for m in means:
        val += multivariate_normal(mean=m, cov=cov).pdf(points)
    return val

def normalize_density(phi, cell_area):
    total = phi.sum() * cell_area
    return phi / total if total > 0 else phi

# ----------------------------- Voronoi utilities -----------------------------

def assign_voronoi(grid_points, agent_positions, sensing_range=None):
    """
    Assign each grid point to the closest agent (Voronoi partition).

    If sensing_range is given, points whose closest agent is farther than
    sensing_range are masked out to emulate range-limited sensing.
    """
    d = np.linalg.norm(grid_points[:, None, :] - agent_positions[None, :, :], axis=2)
    labels = np.argmin(d, axis=1)
    if sensing_range is not None:
        min_d = d[np.arange(len(labels)), labels]
        out_of_range = min_d > sensing_range
    else:
        out_of_range = np.zeros(len(labels), dtype=bool)
    return labels, out_of_range, d

def voronoi_centroids(grid_points, labels, phi, agent_positions, cell_area,
                       out_of_range=None):
    N = agent_positions.shape[0]
    centroids = agent_positions.copy()
    mass = np.zeros(N)
    for i in range(N):
        mask = labels == i
        if out_of_range is not None:
            mask = mask & (~out_of_range)
        w = phi[mask]
        if w.sum() > 1e-9:
            centroids[i] = (grid_points[mask] * w[:, None]).sum(axis=0) / w.sum()
            mass[i] = w.sum() * cell_area
        # else: stay in place, no local mass detected
    return centroids, mass

def coverage_cost(grid_points, labels, phi, agent_positions, cell_area):
    d2 = np.sum((grid_points - agent_positions[labels]) ** 2, axis=1)
    return np.sum(d2 * phi) * cell_area

# ----------------------------- Controllers -----------------------------

def lloyd_step(positions, centroids, kappa=1.0, dt=0.05):
    return positions + dt * kappa * (centroids - positions)

def heterogeneous_step(ground_pos, ground_centroids, aerial_pos, aerial_centroids,
                       aerial_labels, aerial_grid, phi_ground_on_aerial_grid, N_ground,
                     kappa=1.0, gamma=1.0, dt=0.05, aerial_oor=None):
   
    K = aerial_pos.shape[0]
    n_j = np.zeros(K)
    Phi_j = np.zeros(K)
    total_phi = phi_ground_on_aerial_grid.sum()

    # count ground robots per aerial cell (nearest aerial center)
    d_ground_to_aerial = np.linalg.norm(
        ground_pos[:, None, :] - aerial_pos[None, :, :], axis=2)
    ground_cell = np.argmin(d_ground_to_aerial, axis=1)
    for j in range(K):
        n_j[j] = np.sum(ground_cell == j)
        mask = (aerial_labels == j)
        if aerial_oor is not None:
            mask = mask & (~aerial_oor)
        Phi_j[j] = phi_ground_on_aerial_grid[mask].sum() / total_phi if total_phi > 0 else 0

    sigma_j = n_j / N_ground - Phi_j
    # discrete-friendly threshold (Eq. 14)
    sigma_hat = np.where(sigma_j > 1.0 / N_ground, sigma_j, 0.0)

    j_min = np.argmin(sigma_j)
    #C_min = aerial_pos[j_min]
    C_min = aerial_centroids[j_min] 

    new_pos = ground_pos.copy()
    for i in range(ground_pos.shape[0]):
        j = ground_cell[i]
        u_local = kappa * (ground_centroids[i] - ground_pos[i])
        u_global = gamma * (C_min - ground_pos[i])
        w = sigma_hat[j]
        u = (1 - w) * u_local + w * u_global
        new_pos[i] = ground_pos[i] + dt * u
    return new_pos, sigma_j

# ----------------------------- Simulation -----------------------------

def run_simulation(scenario="rudolph", N=5, K=4, iterations=400,
                    sensing_range=0.8, domain_size=10, seed=0, num_peaks=2):
    rng = np.random.default_rng(seed)
    ground_domain = Domain(0, domain_size, 0, domain_size, grid_res=90)
    means = [rng.random(2) * domain_size for _ in range(num_peaks)]
    cov = [[1.5, 0.0], [0.0, 1.5]]
    phi_ground = bivariate_gaussian_density(ground_domain.grid, means, cov)
    phi_ground = normalize_density(phi_ground, ground_domain.cell_area)

    ground_pos = rng.random((N, 2)) * domain_size
    aerial_pos = rng.random((K, 2)) * domain_size

    cost_history = []
    traj = [ground_pos.copy()]
    aerial_traj = [aerial_pos.copy()]

    for t in range(iterations):
        # aerial team: standard Lloyd's with uniform density over full domain
        aerial_labels, aerial_oor, _ = assign_voronoi(ground_domain.grid, aerial_pos, 
                                                sensing_range=2*sensing_range)
        uniform = np.ones(ground_domain.grid.shape[0])
        aerial_centroids, _ = voronoi_centroids(ground_domain.grid, aerial_labels,
                                                    uniform, aerial_pos,
                                                    ground_domain.cell_area)
        aerial_pos = lloyd_step(aerial_pos, aerial_centroids, kappa=0.5)

        # ground team: local range-limited Lloyd + global distribution term
        ground_labels, ground_oor, _ = assign_voronoi(ground_domain.grid, ground_pos,
                                                sensing_range=sensing_range)
        ground_centroids, _ = voronoi_centroids(ground_domain.grid, ground_labels,
                                                    phi_ground, ground_pos,
                                                    ground_domain.cell_area,
                                                    out_of_range=ground_oor)
        ground_pos, sigma_j = heterogeneous_step(
            ground_pos, ground_centroids, aerial_pos, aerial_centroids, aerial_labels,
            ground_domain.grid, phi_ground, N, aerial_oor=aerial_oor)


        # clip to domain
        ground_pos = np.clip(ground_pos, 0, domain_size)

        labels_full, _, _ = assign_voronoi(ground_domain.grid, ground_pos)
        cost = coverage_cost(ground_domain.grid, labels_full, phi_ground,
                              ground_pos, ground_domain.cell_area)
        cost_history.append(cost)
        traj.append(ground_pos.copy())
        aerial_traj.append(aerial_pos.copy())

    return np.array(cost_history), np.array(traj), np.array(aerial_traj), ground_domain, phi_ground

if __name__ == "__main__":
    iterations = 400
    results = {}
    ground_sensing = 0.8
    aerial_sensing = 2 * ground_sensing
    for scen in ["rudolph"]:
        cost, traj, aerial_traj, domain, phi = run_simulation(
            scenario=scen, iterations=iterations, sensing_range=ground_sensing,
            num_peaks=1
        )
        results[scen] = cost

    N = traj.shape[1]
    K = aerial_traj.shape[1]
    grid_res = int(np.sqrt(domain.grid.shape[0]))
    xs = np.linspace(domain.bbox[0], domain.bbox[1], grid_res)
    ys = np.linspace(domain.bbox[2], domain.bbox[3], grid_res)
    X, Y = np.meshgrid(xs, ys)

    # final Voronoi cells
    ground_labels, ground_oor, _ = assign_voronoi(
        domain.grid, traj[-1], sensing_range=ground_sensing)
    aerial_labels, aerial_oor, _ = assign_voronoi(domain.grid, aerial_traj[-1], sensing_range=aerial_sensing)

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 5))

    # ---- left: ground Voronoi cells ----
    ground_cells = np.ma.masked_where(ground_oor, ground_labels)
    ax1.pcolormesh(X, Y, ground_cells.reshape(grid_res, grid_res),
                   cmap="tab10", alpha=0.25, vmin=0, vmax=9)
    ax1.tricontourf(domain.grid[:, 0], domain.grid[:, 1], phi,
                    cmap="RdPu", alpha=0.15)
    for i in range(N):
        ax1.plot(traj[:, i, 0], traj[:, i, 1], label=f"G{i}", alpha=0.7)
        ax1.scatter(traj[0, i, 0], traj[0, i, 1], marker="o", s=40)
        ax1.scatter(traj[-1, i, 0], traj[-1, i, 1], marker="x", s=50)
    for i in range(K):
        ax1.plot(aerial_traj[:, i, 0], aerial_traj[:, i, 1], '--',
                 label=f"A{i}", alpha=0.5)
    ax1.set_title("Ground Voronoi Cells")
    ax1.set_xlabel("x"); ax1.set_ylabel("y")
    ax1.legend(fontsize=6)

    # ---- right: aerial Voronoi cells ----
    # ax2.pcolormesh(X, Y, aerial_labels.reshape(grid_res, grid_res),
    #                cmap="Set2", alpha=0.25)
    aerial_cells = np.ma.masked_where(aerial_oor, aerial_labels)
    ax2.pcolormesh(X, Y, aerial_cells.reshape(grid_res, grid_res),
                   cmap="tab10", alpha=0.25, vmin=0, vmax=9)
    ax2.tricontourf(domain.grid[:, 0], domain.grid[:, 1], phi,
                    cmap="RdPu", alpha=0.15)
    for i in range(N):
        ax2.plot(traj[:, i, 0], traj[:, i, 1], label=f"G{i}", alpha=0.5)
    for i in range(K):
        ax2.plot(aerial_traj[:, i, 0], aerial_traj[:, i, 1], '--',
                 label=f"A{i}", alpha=0.7)
        ax2.scatter(aerial_traj[0, i, 0], aerial_traj[0, i, 1], marker="s", s=40)
        ax2.scatter(aerial_traj[-1, i, 0], aerial_traj[-1, i, 1], marker="*", s=60)
    ax2.set_title("Aerial Voronoi Cells")
    ax2.set_xlabel("x"); ax2.set_ylabel("y")
    ax2.legend(fontsize=6)


    # trajectories plot
    ax3.tricontourf(domain.grid[:, 0], domain.grid[:, 1], phi,
                    cmap="RdPu", alpha=0.25)
    for i in range(N):
        ax3.plot(traj[:, i, 0], traj[:, i, 1], label=f"G{i}", alpha=0.7)
        ax3.scatter(traj[0, i, 0], traj[0, i, 1], marker="o", s=40)
        ax3.scatter(traj[-1, i, 0], traj[-1, i, 1], marker="x", s=50)
    for i in range(K):
        ax3.plot(aerial_traj[:, i, 0], aerial_traj[:, i, 1], '--',
                 label=f"A{i}", alpha=0.5)
        ax3.scatter(aerial_traj[0, i, 0], aerial_traj[0, i, 1], marker="s", s=40)
        ax3.scatter(aerial_traj[-1, i, 0], aerial_traj[-1, i, 1], marker="*", s=60)
    ax3.set_title("Trajectories over Density")
    ax3.set_xlabel("x"); ax3.set_ylabel("y")
    ax3.legend(fontsize=6)

    plt.tight_layout()
    plt.savefig("output/trajectories.png", dpi=150)
    plt.show()

    print("Done. Final costs:", {k: v[-1] for k, v in results.items()})
