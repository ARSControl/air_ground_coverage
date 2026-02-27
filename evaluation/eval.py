import numpy as np
import matplotlib.pyplot as plt
import sys
import os
import itertools

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

plt.rcParams.update(
    {
        "font.size": 14,
        "axes.titlesize": 16,
        "axes.labelsize": 14,
        "lines.linewidth": 2,
        "lines.markersize": 6,
        "legend.fontsize": 12,
    }
)
plt.rcParams["pdf.fonttype"] = 42

res_path = "output/ablation_fixedtotal/"
aerial_robots_list = [0, 1, 2, 3, 4]
ground_robots_list = [10 - a for a in aerial_robots_list]

print("=" * 70)
print("Ablation Study Results")
print("=" * 70)

combinations = list(itertools.product(aerial_robots_list, ground_robots_list))

results = []
for i in range(len(aerial_robots_list)):
    num_aerial = aerial_robots_list[i]
    num_ground = 10 - num_aerial
    exp_name = f"aerial{num_aerial}_ground{num_ground}"
    eff_file = os.path.join(res_path, f"effectiveness_{exp_name}.npy")
    kl_file = os.path.join(res_path, f"kl_divergence_{exp_name}.npy")
    wasserstein_file = os.path.join(res_path, f"wasserstein_{exp_name}.npy")

    if os.path.exists(eff_file) and os.path.exists(kl_file):
        effectiveness = np.load(eff_file)
        kl_div = np.load(kl_file)

        mean_eff = np.mean(effectiveness[:, -1])
        std_eff = np.std(effectiveness[:, -1])
        mean_kl = np.mean(kl_div[:, -1])
        std_kl = np.std(kl_div[:, -1])

        mean_wass = None
        std_wass = None
        if os.path.exists(wasserstein_file):
            wasserstein = np.load(wasserstein_file)
            mean_wass = np.mean(wasserstein)
            std_wass = np.std(wasserstein)

        results.append(
            {
                "num_aerial": num_aerial,
                "num_ground": num_ground,
                "mean_effectiveness": mean_eff,
                "std_effectiveness": std_eff,
                "mean_kl": mean_kl,
                "std_kl": std_kl,
                "mean_wasserstein": mean_wass,
                "std_wasserstein": std_wass,
            }
        )

        print(f"Aerial: {num_aerial}, Ground: {num_ground}")
        print(f"  Effectiveness: {mean_eff:.4f} ± {std_eff:.4f}")
        print(f"  KL Divergence: {mean_kl:.4f} ± {std_kl:.4f}")
        if mean_wass is not None:
            print(f"  Wasserstein Distance: {mean_wass:.4f} ± {std_wass:.4f}")
        print()
    else:
        print(f"Missing data for Aerial: {num_aerial}, Ground: {num_ground}")
        print()

eff_matrix = np.zeros((len(aerial_robots_list), len(ground_robots_list)))
kl_matrix = np.zeros((len(aerial_robots_list), len(ground_robots_list)))
wass_matrix = np.zeros((len(aerial_robots_list), len(ground_robots_list)))

for r in results:
    i = aerial_robots_list.index(r["num_aerial"])
    j = ground_robots_list.index(r["num_ground"])
    eff_matrix[i, j] = r["mean_effectiveness"]
    kl_matrix[i, j] = r["mean_kl"]
    if r["mean_wasserstein"] is not None:
        wass_matrix[i, j] = r["mean_wasserstein"]

fig, axes = plt.subplots(1, 3, figsize=(18, 5))

cmap_eff = plt.cm.Greens
cmap_kl = plt.cm.Reds
cmap_wass = plt.cm.Blues

im1 = axes[0].imshow(eff_matrix, cmap=cmap_eff, aspect="auto")
axes[0].set_xticks(range(len(ground_robots_list)))
axes[0].set_xticklabels(ground_robots_list)
axes[0].set_yticks(range(len(aerial_robots_list)))
axes[0].set_yticklabels(aerial_robots_list)
axes[0].set_xlabel("Number of Ground Robots")
axes[0].set_ylabel("Number of Aerial Robots")
axes[0].set_title("Effectiveness (higher is better)")

for i in range(len(aerial_robots_list)):
    for j in range(len(ground_robots_list)):
        val = eff_matrix[i, j]
        if val > 0:
            text = axes[0].text(
                j,
                i,
                f"{val:.3f}",
                ha="center",
                va="center",
                color="black",
                fontsize=12,
                fontweight="bold",
            )

plt.colorbar(im1, ax=axes[0], shrink=0.8)

im2 = axes[1].imshow(kl_matrix, cmap=cmap_kl, aspect="auto")
axes[1].set_xticks(range(len(ground_robots_list)))
axes[1].set_xticklabels(ground_robots_list)
axes[1].set_yticks(range(len(aerial_robots_list)))
axes[1].set_yticklabels(aerial_robots_list)
axes[1].set_xlabel("Number of Ground Robots")
axes[1].set_ylabel("Number of Aerial Robots")
axes[1].set_title("KL Divergence (lower is better)")

for i in range(len(aerial_robots_list)):
    for j in range(len(ground_robots_list)):
        val = kl_matrix[i, j]
        if val > 0:
            text = axes[1].text(
                j,
                i,
                f"{kl_matrix[i, j]:.3f}",
                ha="center",
                va="center",
                color="black",
                fontsize=12,
                fontweight="bold",
            )

plt.colorbar(im2, ax=axes[1], shrink=0.8)

fig, ax = plt.subplots(figsize=(10, 6))
ax.axis("off")

table_data = []
col_labels = ["Team", "KL Divergence", "Wasserstein", "Effectiveness"]
cell_colors = []

for r in results:
    team = f"A{r['num_aerial']}/G{r['num_ground']}"
    kl = f"{r['mean_kl']:.3f} ± {r['std_kl']:.3f}"
    wass = (
        f"{r['mean_wasserstein']:.1f} ± {r['std_wasserstein']:.1f}"
        if r["mean_wasserstein"] is not None
        else "N/A"
    )
    eff = f"{r['mean_effectiveness']:.3f} ± {r['std_effectiveness']:.3f}"
    table_data.append([team, kl, wass, eff])

table = ax.table(
    cellText=table_data,
    colLabels=col_labels,
    cellLoc="center",
    loc="center",
    colColours=["#f0f0f0"] * 4,
)
table.auto_set_font_size(False)
table.set_fontsize(12)
table.scale(1.2, 2.0)

for i in range(len(table_data) + 1):
    for j in range(4):
        cell = table[(i, j)]
        if i == 0:
            cell.set_text_props(fontweight="bold")
        if i > 0:
            if j == 1:
                cell.set_facecolor(
                    plt.cm.Reds(
                        np.clip(1 - float(table_data[i - 1][j].split()[0]) / 0.5, 0, 1)
                        * 0.5
                        + 0.2
                    )
                )
            elif j == 2:
                val = (
                    float(table_data[i - 1][j].split()[0])
                    if table_data[i - 1][j] != "N/A"
                    else 0
                )
                cell.set_facecolor(
                    plt.cm.Blues(np.clip(1 - val / 800, 0, 1) * 0.5 + 0.2)
                )
            elif j == 3:
                cell.set_facecolor(
                    plt.cm.Greens(
                        np.clip(float(table_data[i - 1][j].split()[0]) / 0.4, 0, 1)
                        * 0.5
                        + 0.2
                    )
                )

ax.set_title("Ablation Study Results", fontsize=16, fontweight="bold", pad=20)

plt.tight_layout()
plt.savefig(os.path.join(res_path, "ablation_table.png"), dpi=150, bbox_inches="tight")
plt.savefig(os.path.join(res_path, "ablation_table.pdf"), bbox_inches="tight")
plt.show()

print("\n" + "=" * 70)
print("Wasserstein Distance Results")
print("=" * 70)
for r in results:
    if r["mean_wasserstein"] is not None:
        print(
            f"Aerial: {r['num_aerial']}, Ground: {r['num_ground']}: {r['mean_wasserstein']:.4f} ± {r['std_wasserstein']:.4f}"
        )

summary_file = os.path.join(res_path, "results_summary.npy")
np.save(summary_file, results)
print(f"\nPlots saved to: {os.path.join(res_path, 'ablation_table.png')}")
