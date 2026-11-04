"""Plot real planner/gate results with a controlled 240-second virtual clock."""

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from docker.deployment_timing_validation import simulate_timing


def main():
    result = simulate_timing(duration=240.0, compute_time=1.45)
    fig, axes = plt.subplots(1, 2, figsize=(12, 4), layout="constrained")
    for key, label, color in [
        ("old_age", "Completion-only publication", "#a3a3a3"),
        ("fixed_age", "Before/after publication", "#2563eb"),
    ]:
        axes[0].plot(
            result["times"], result[key], label=label, color=color, linewidth=0.8
        )
    axes[0].axhline(
        2.0, linestyle="--", color="#dc2626", label="Unchanged target age limit"
    )
    axes[0].set(
        title="1.45 s compute delay, 240 s virtual recorder clock",
        xlabel="Virtual time [s]",
        ylabel="Age of latest target [s]",
        xlim=(110, 135),
    )
    axes[0].legend(fontsize=8)
    metrics = result["metrics"]
    bars = axes[1].bar(
        ["Old publication", "Fixed publication"],
        [metrics["old_command_rejections"], metrics["fixed_command_rejections"]],
        color=["#a3a3a3", "#2563eb"],
    )
    axes[1].bar_label(bars, padding=3)
    axes[1].set(
        title="Three real planners and command gates; fixed poses",
        ylabel="Rejected command checks",
        ylim=(0, max(metrics["old_command_rejections"] * 1.15, 1)),
    )
    root = Path("docs/assets/sitl")
    fig.savefig(root / "central_timing.png", dpi=140)
    plt.close(fig)
    (root / "central_timing_metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
