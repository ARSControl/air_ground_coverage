import numpy as np
import matplotlib.pyplot as plt
import sys
import os
import pickle
import ot
import matplotlib as mpl

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.utils.voronoi import agent_fov
from src.utils.math_utils import min_max_normalize
from src.utils.eval_utils import eval_kl_divergence

# increase size for labels, lines,  for better readability
plt.rcParams.update(
    {
        "font.size": 24,
        "axes.titlesize": 28,
        "axes.labelsize": 32,
        "lines.linewidth": 5,
        "lines.markersize": 8,
        "legend.fontsize": 18,
    }
)
# fix font errors
plt.rcParams["pdf.fonttype"] = 42

res_path = "output/eval/"
colors = [
    "tab:blue",
    "tab:orange",
    "tab:green",
    "tab:red",
    "tab:purple",
    "tab:brown",
    "tab:pink",
    "tab:gray",
    "tab:olive",
    "tab:cyan",
]

"""
eff = np.load(res_path + "effectiveness_metrics.npy")
print("Effectiveness shape: ", eff.shape)
eff_mean = np.mean(eff, axis=0)
eff_std = np.std(eff, axis=0)
plt.figure(figsize=(10, 6))
plt.plot(eff_mean, label="Mean Effectiveness", color="blue")
plt.fill_between(
    np.arange(len(eff_mean)),
    eff_mean - eff_std,
    eff_mean + eff_std,
    color="blue",
    alpha=0.2,
    label="Std Dev",
)
plt.title("Effectiveness Over Time")
plt.xlabel("Time Step")
plt.ylabel("Effectiveness")
plt.grid()
plt.legend()
# plt.savefig("output/eval/effectiveness_over_time.png")
plt.show()

kl = np.load(res_path + "kl_divergence.npy")
print("Final KL Divergence: ", kl[:, -1])
kl_mean = np.mean(kl, axis=0)
print("Final avg KL: ", kl_mean[-1])
kl_std = np.std(kl, axis=0)
plt.figure(figsize=(10, 6))
plt.plot(kl_mean, label="Mean KL Divergence", color="orange")
plt.fill_between(
    np.arange(len(kl_mean)),
    kl_mean - kl_std,
    kl_mean + kl_std,
    color="orange",
    alpha=0.2,
    label="Std Dev",
)
plt.title("KL Divergence Over Time")
plt.xlabel("Time Step")
plt.ylabel("KL Divergence")
plt.grid()
plt.legend()
plt.show()
"""

trajs = np.load(res_path + "ground_trajectories.npy")
ground_trajs = np.load(res_path + "baseline_ground_trajectories.npy")
aerial_trajs = np.load(res_path + "aerial_trajectories.npy")
combo_density = np.load(res_path + "final_combined_density.npy")
ground_density = np.load(res_path + "final_ground_density.npy")
aerial_density = np.load(res_path + "final_aerial_density2.npy")
obstacles = np.load(res_path + "obstacles.npy")

with open(os.path.join(res_path, "gmm_model.pkl"), "rb") as f:
    gmm = pickle.load(f)

nbPts = combo_density.shape[0]
xg = np.linspace(0, 50, nbPts)
yg = np.linspace(0, 50, nbPts)
X, Y = np.meshgrid(xg, yg)
grid_points = np.column_stack((X.ravel(), Y.ravel()))
target_density = gmm.sample_pdf(grid_points).reshape(X.shape)
target_density = min_max_normalize(target_density)

figsz = (8, 8)
fig, ax = plt.subplots(figsize=figsz)
ax.imshow(
    target_density,
    extent=[0, 50, 0, 50],
    origin="lower",
    cmap="RdPu",
    vmin=0,
    vmax=1,
)
# for obs in obstacles:
#     circle = plt.Circle(
#         obs, 1.0, color="k", alpha=1
#     )
#     ax.add_patch(circle)
ax.set_title("Ground Truth Density")
# axs[0].set_title("Target Density")
# for traj in trajs:
#     ax.plot(traj[:, 0], traj[:, 1], color="tab:blue", alpha=0.5)
#     state = traj[-1, :3]
#     fov_lines = agent_fov(
#         state,
#         5.0,
#         np.deg2rad(90.0),
#     )
#     ax.plot(fov_lines[:, 0], fov_lines[:, 1], color="tab:blue")
#     ax.scatter(state[0], state[1], color="tab:blue", s=75)

ax.set_xlim(0, 50)
ax.set_ylim(0, 50)
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")

plt.tight_layout()
plt.savefig(os.path.join(res_path, "pics", "final_target.pdf"))
# plt.show()

fig, ax = plt.subplots(figsize=figsz)
ax.imshow(
    min_max_normalize(combo_density),
    extent=[0, 50, 0, 50],
    origin="lower",
    cmap="RdPu",
    vmin=0,
    vmax=1,
)
for obs in obstacles:
    circle = plt.Circle(obs, 1.0, color="k", alpha=1)
    ax.add_patch(circle)
ax.set_title("Estimated Density - Combined")
for i, traj in enumerate(trajs[1:]):
    ax.plot(traj[:, 0], traj[:, 1], color=colors[0], alpha=0.5)
    state = traj[-1, :3]
    fov_lines = agent_fov(
        state,
        5.0,
        np.deg2rad(90.0),
    )
    ax.plot(fov_lines[:, 0], fov_lines[:, 1], color=colors[0])
    ax.scatter(state[0], state[1], color=colors[0], s=75)

    ax.set_xlim(0, 50)
    ax.set_ylim(0, 50)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
plt.tight_layout()
plt.savefig(os.path.join(res_path, "pics", "final_combined.pdf"))
# plt.show()

fig, ax = plt.subplots(figsize=figsz)
ax.imshow(
    min_max_normalize(ground_density),
    extent=[0, 50, 0, 50],
    origin="lower",
    cmap="RdPu",
    vmin=0,
    vmax=1,
)
for obs in obstacles:
    circle = plt.Circle(obs, 1.0, color="k", alpha=1)
    ax.add_patch(circle)
ax.set_title("Estimated Density - Ground")
for traj in ground_trajs[1:]:
    ax.plot(traj[:, 0], traj[:, 1], color="tab:blue", alpha=0.5)
    state = traj[-1, :3]
    fov_lines = agent_fov(
        state,
        5.0,
        np.deg2rad(90.0),
    )
    ax.plot(fov_lines[:, 0], fov_lines[:, 1], color="tab:blue")
    ax.scatter(state[0], state[1], color="tab:blue", s=75)

    ax.set_xlim(0, 50)
    ax.set_ylim(0, 50)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
plt.tight_layout()
plt.savefig(os.path.join(res_path, "pics", "final_ground.pdf"))

fig, ax = plt.subplots(figsize=figsz)
ax.imshow(
    min_max_normalize(aerial_density),
    extent=[0, 50, 0, 50],
    origin="lower",
    cmap="RdPu",
    vmin=0,
    vmax=1,
)
# for obs in obstacles:
#     circle = plt.Circle(
#         obs, 1.0, color="k", alpha=1
#     )
#     ax.add_patch(circle)
ax.set_title("Estimated Density - Aerial")
# axs[1].set_title("Final Combined Density")
for i, traj in enumerate(aerial_trajs):
    ax.plot(traj[:, 0], traj[:, 1], color=colors[i], alpha=0.5)
    state = traj[-1, :3]
    # fov_lines = agent_fov(
    #     state,
    #     5.0,
    #     np.deg2rad(90.0),
    # )
    # ax.plot(fov_lines[:, 0], fov_lines[:, 1], color="tab:blue")
    ax.scatter(state[0], state[1], color=colors[i], s=75)

    ax.set_xlim(0, 50)
    ax.set_ylim(0, 50)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
plt.tight_layout()
plt.savefig(os.path.join(res_path, "pics", "final_aerial.pdf"))

combo_diff = np.abs(combo_density - target_density)
aerial_diff = np.abs(aerial_density - target_density)
ground_diff = np.abs(ground_density - target_density)

# find max diff
max_diff = max(np.max(combo_diff), np.max(aerial_diff), np.max(ground_diff))


fig, ax = plt.subplots(figsize=figsz)
im = ax.imshow(
    combo_diff,
    origin="lower",
    extent=[0, 50, 0, 50],
    cmap="Reds",
    vmin=0,
    vmax=max_diff,
)
ax.set_title("Combined Density Error")
ax.set_xlim(0, 50)
ax.set_ylim(0, 50)
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
plt.tight_layout()
# plt.colorbar(im, label="Error")
plt.savefig(os.path.join(res_path, "pics", "diff_combined.pdf"))

fig, ax = plt.subplots(figsize=figsz)
im = ax.imshow(
    ground_diff,
    origin="lower",
    extent=[0, 50, 0, 50],
    cmap="Reds",
    vmin=0,
    vmax=max_diff,
)
ax.set_title("Ground Density Error")
ax.set_xlim(0, 50)
ax.set_ylim(0, 50)
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
plt.tight_layout()
# plt.colorbar(im, label="Error")
plt.savefig(os.path.join(res_path, "pics", "diff_ground.pdf"))

fig, ax = plt.subplots(figsize=figsz)
im = im = ax.imshow(
    aerial_diff,
    origin="lower",
    extent=[0, 50, 0, 50],
    cmap="Reds",
    vmin=0,
    vmax=max_diff,
)
ax.set_title("Aerial Density Error")
ax.set_xlim(0, 50)
ax.set_ylim(0, 50)
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
plt.tight_layout()
# plt.colorbar(im, label="Error")
plt.savefig(os.path.join(res_path, "pics", "diff_aerial.pdf"))

# fig_cb, ax_cb = plt.subplots(figsize=(2,6))  # tall & narrow for vertical colorbar
# fig_cb.colorbar(im, cax=ax_cb, label="Error")
# plt.tight_layout()
# plt.savefig(os.path.join(res_path, "pics", "error_colorbar.pdf"))

fig, ax = plt.subplots(figsize=(2, 8))  # width, height

# Create a ScalarMappable for the colorbar
norm = mpl.colors.Normalize(vmin=0, vmax=max_diff)
sm = mpl.cm.ScalarMappable(cmap="Reds", norm=norm)
sm.set_array([])  # no data

# Add vertical colorbar
cbar = fig.colorbar(sm, cax=ax, orientation='vertical')
cbar.set_label("Error", rotation=270, labelpad=25)
plt.tight_layout()
plt.savefig(os.path.join(res_path, "pics", "error_colorbar.pdf"))

# Print final KL divergence
# combo_kl = eval_kl_divergence(target_density.ravel(), min_max_normalize(combo_density).ravel())
# ground_kl = eval_kl_divergence(target_density.ravel(), min_max_normalize(ground_density).ravel())
# aerial_kl = eval_kl_divergence(target_density.ravel(), min_max_normalize(aerial_density).ravel())
# print(f"Final KL Divergence - Combined: {combo_kl:.4f}")
# print(f"Final KL Divergence - Ground: {ground_kl:.4f}")
# print(f"Final KL Divergence - Aerial: {aerial_kl:.4f}")

kl_combo = np.load(res_path + "kl_divergence.npy")
kl_avg = np.mean(kl_combo, axis=0)
kl_std = np.std(kl_combo, axis=0)
print(f"Avg KL Divergence at final time step: {kl_avg[-1]:.4f}")
kl_aerial = np.load(res_path + "aerial_kl_divergence.npy")
kl_aerial_avg = np.mean(kl_aerial, axis=0)
kl_aerial_std = np.std(kl_aerial, axis=0)
print(f"Avg KL Divergence for aerial at final time step: {kl_aerial_avg[-1]:.4f}")
kl_ground = np.load(res_path + "ground_kl_divergence.npy")
kl_ground_avg = np.mean(kl_ground, axis=0)
kl_ground_std = np.std(kl_ground, axis=0)
print(f"Avg KL Divergence for ground at final time step: {kl_ground_avg[-1]:.4f}")
fig, ax = plt.subplots(figsize=(10, 6))
ax.plot(kl_avg, label="Mean KL Divergence", color="orange")
ax.fill_between(
    np.arange(len(kl_avg)),
    kl_avg - kl_std,
    kl_avg + kl_std,
    color="orange",
    alpha=0.2,
    label="Std Dev",
)
ax.plot(kl_aerial_avg, label="Mean KL Divergence (Aerial)", color="blue")
ax.fill_between(
    np.arange(len(kl_aerial_avg)),
    kl_aerial_avg - kl_aerial_std,
    kl_aerial_avg + kl_aerial_std,
    color="blue",
    alpha=0.2,
    label="Std Dev (Aerial)",
)
ax.plot(kl_ground_avg, label="Mean KL Divergence (Ground)", color="green")
ax.fill_between(
    np.arange(len(kl_ground_avg)),
    kl_ground_avg - kl_ground_std,
    kl_ground_avg + kl_ground_std,
    color="green",
    alpha=0.2,
    label="Std Dev (Ground)",
)
ax.set_title("KL Divergence Over Time")
ax.set_xlabel("Time Step")
ax.set_ylabel("KL Divergence")
ax.grid()
ax.legend()
plt.tight_layout()
# plt.savefig(os.path.join(res_path, "pics", "kl_over_time.pdf"))


plt.show()
