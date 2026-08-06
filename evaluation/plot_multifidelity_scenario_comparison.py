"""Compare reconstruction performance across two saved composition scenarios."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.multifidelity_composition_io import (  # noqa: E402
    EVALUATED_ARCHIVE_KIND,
    load_archive,
)
from evaluation.plot_multifidelity_composition import _plot_history  # noqa: E402
from evaluation.plot_multifidelity_composition import (  # noqa: E402
    _hyperparameter_policy,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--easy", type=Path, required=True)
    parser.add_argument("--hard", type=Path, required=True)
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        default=Path("output/multifidelity_scenario_comparison.png"),
    )
    parser.add_argument("--bootstrap-samples", type=int, default=1000)
    return parser.parse_args()


def plot_scenario_comparison(
    easy_path: str | Path,
    hard_path: str | Path,
    output_path: str | Path,
    *,
    bootstrap_samples: int = 1000,
) -> Path:
    """Render NRMSE, KL, and NLPD histories for two named scenarios."""
    if isinstance(bootstrap_samples, bool) or bootstrap_samples < 1:
        raise ValueError("bootstrap_samples must be a positive integer")
    easy_metadata, easy = load_archive(easy_path, expected_kind=EVALUATED_ARCHIVE_KIND)
    hard_metadata, hard = load_archive(hard_path, expected_kind=EVALUATED_ARCHIVE_KIND)
    _validate_pair(easy_metadata, easy, hard_metadata, hard)

    names = [str(value) for value in easy["composition_names"].tolist()]
    colors = plt.get_cmap("viridis")(np.linspace(0.08, 0.92, max(len(names), 2)))
    figure, axes = plt.subplots(
        2,
        3,
        figsize=(18.0, 8.5),
        sharey="col",
        constrained_layout=True,
    )
    rng = np.random.default_rng(20260804)
    rows = ((easy_metadata, easy), (hard_metadata, hard))
    metrics = (
        ("nrmse", "zero-clipped HIGH-field NRMSE history", "NRMSE"),
        ("kl", "density KL divergence history", "KL divergence"),
        ("nlpd", "latent HIGH marginal NLPD history", "Marginal NLPD"),
    )
    labels = (("A", "B", "C"), ("D", "E", "F"))
    for row, (metadata, values) in enumerate(rows):
        scenario = str(metadata.get("scenario", "default"))
        title = scenario.replace("_", " ").title()
        for column, (metric, description, ylabel) in enumerate(metrics):
            _plot_history(
                axes[row, column],
                values,
                names,
                colors,
                metric,
                rng,
                bootstrap_samples,
            )
            axes[row, column].set_title(
                f"{labels[row][column]}. {title}: {description}"
            )
            axes[row, column].set_xlabel("Mission time [s]")
            axes[row, column].set_ylabel(ylabel)
            axes[row, column].grid(alpha=0.25)

    handles = [
        plt.Line2D([0], [0], color=colors[index], linewidth=2.2, label=name)
        for index, name in enumerate(names)
    ]
    figure.legend(
        handles=handles,
        loc="outside lower center",
        ncol=len(handles),
        frameon=False,
    )
    figure.suptitle(
        "Reconstruction histories in easy/long and hard/short missions\n"
        f"N={easy_metadata['total_robots']}; "
        f"kernels={_hyperparameter_policy(easy_metadata)}; "
        "episode means and bootstrap 95% intervals",
        fontsize=14,
    )
    destination = Path(output_path)
    if not destination.name:
        raise ValueError("output_path must name a file")
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=180, facecolor="white", bbox_inches="tight")
    plt.close(figure)
    return destination


def _validate_pair(
    easy_metadata: dict,
    easy: dict[str, np.ndarray],
    hard_metadata: dict,
    hard: dict[str, np.ndarray],
) -> None:
    easy_scenario = str(easy_metadata.get("scenario", "default"))
    hard_scenario = str(hard_metadata.get("scenario", "default"))
    if easy_scenario == hard_scenario:
        raise ValueError("scenario comparison requires two distinct scenarios")
    for key in ("composition_names", "aerial_counts", "ground_counts", "seeds"):
        if not np.array_equal(easy[key], hard[key]):
            raise ValueError(f"scenario archives must have identical {key}")
    if int(easy_metadata["total_robots"]) != int(hard_metadata["total_robots"]):
        raise ValueError("scenario archives must have the same total robot count")
    easy_optimization = easy_metadata.get(
        "hyperparameter_optimization", {"enabled": False}
    )
    hard_optimization = hard_metadata.get(
        "hyperparameter_optimization", {"enabled": False}
    )
    if easy_optimization != hard_optimization:
        raise ValueError(
            "scenario archives must use the same hyperparameter optimization policy"
        )


def main() -> None:
    arguments = parse_args()
    destination = plot_scenario_comparison(
        arguments.easy,
        arguments.hard,
        arguments.output,
        bootstrap_samples=arguments.bootstrap_samples,
    )
    print(f"scenario_comparison_plot={destination}")


if __name__ == "__main__":
    main()
