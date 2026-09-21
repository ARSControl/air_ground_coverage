# Multi-Fidelity Air–Ground Coverage

This repository simulates coordinated aerial and ground coverage of a spatial
field using a centralized, asynchronous autoregressive multi-fidelity Gaussian
Process (GP). Aerial robots collect LOW-fidelity observations, ground robots
collect HIGH-fidelity observations, and one estimator produces the HIGH-field
posterior used by both coverage controllers.

The implemented model is

$$
f_H(q) = \rho f_L(q) + \delta(q),
$$

where $f_L$ is the LOW-fidelity latent field, $\delta$ is an independent
discrepancy GP, and $\rho$ is the configurable fidelity-correlation parameter.
The estimator uses Cholesky-based linear solves with adaptive jitter and
publishes the most recent valid posterior if an update fails.

## Features

- One central estimator, owned by the coupled simulation, rather than a GP per
  robot.
- Asynchronous LOW/HIGH observation scheduling and batched GP updates that are
  separate from controller and numerical-integration rates.
- HIGH-field posterior mean and variance for aerial target construction.
- Posterior-derived density for ground coverage.
- HEDAC aerial coverage with Dubins aircraft and Lloyd or MPC ground coverage
  with unicycle robots.
- Deterministic configurations, tests, evaluation scripts, and saved visual
  summaries.

## Installation

The project requires Python 3.10 or newer. [uv](https://docs.astral.sh/uv/) is
the recommended dependency manager.

```bash
git clone <repository-url>
cd air_ground_coverage
uv sync
```

Alternatively, install the package and its dependencies with pip:

```bash
pip install -e .
```

## Quick start

Run a short deterministic smoke simulation:

```bash
uv run python examples/run_multifidelity.py \
  --config configs/multifidelity_smoke.yaml \
  --no-plot
```

Run the full coupled scenario and save its final-state figure:

```bash
uv run python examples/run_multifidelity.py \
  --config configs/multifidelity.yaml
```

The full configuration controls the fixed random seed, event periods, GP
hyperparameters, robot models, and output path. Its default final-state figure
is written to `output/multifidelity_final_state.png`.

Run the default optimization-enabled decentralized `easy_long` composition
experiment, including evaluation and plots, with:

```bash
uv run python evaluation/run_multifidelity_scenario_pipeline.py \
  --config configs/multifidelity_composition.yaml \
  --composition A2/G8 \
  --episode 0
```

The pipeline writes the easy raw and evaluated archives, aggregate metric plot,
and selected-episode trajectory/reconstruction/error figure below
`output/multifidelity_scenario_pipeline/`. The publication configuration
declares only the 30-second `easy_long` scenario. Generic configurations may
declare additional named scenarios and select one with `--scenario NAME`.
If compatible raw archives already exist, add `--reuse-raw` to recompute NLPD
and regenerate every plot without rerunning any simulation or changing saved
trajectories.
The composition protocol deploys both robot classes from the lower-left 20% of
the environment, using deterministic class-specific candidate sequences so
conditions remain paired as team counts change.

After running the four-method comparison, render one already-saved paired
episode as ten independent MP4 videos and four companion PNG images without
rerunning any method with:

```bash
uv run python evaluation/render_four_method_episode_videos.py \
  --input-dir output/four_method_comparison \
  --episode 7 \
  --output-dir output/four_method_comparison/videos/episode_007_revised \
  --fps 20 \
  --frame-interval 1
```

Episode indices are zero-based. The command writes robot-marker videos over
the proposed method's saved MFGP density and a synchronized four-panel density
comparison. In that comparison, the proposed panel uses the current saved MFGP
estimate while the three baseline panels use their shared ground truth. It also writes
progressively revealed videos for MFGP NRMSE, KL, and NLPD, plus four-method
comparisons of locational cost $\mathcal{L}$ and footprint-normalized
effectiveness $\mathcal{E}$.
Egerstedt and Zhang do not publish GP posteriors, so reconstruction metrics are
correctly labeled MFGP-only. Robot paths are deliberately not drawn.
Posterior-based fields and metrics change only when the proposed raw archive
contains a new asynchronous posterior; locational cost $\mathcal{L}$ and
effectiveness update at every saved paired state. All outputs use H.264 video
with a `.mp4` suffix. The
locked `imageio-ffmpeg` dependency supplies the encoder when no system ffmpeg
is installed.

The two additional spatial videos are `mfgp_variance.mp4` and
`mfgp_density_difference.mp4`. They show the moving proposed robots over the
latest causally available latent HIGH posterior variance and signed normalized
density error $\widehat\phi-\phi$. Their color limits remain fixed for the
whole selected episode: high variance is dark in `Greys`; underestimation is
blue and overestimation red in symmetric `RdBu_r`.

`mfgp_spatial_reconstruction_comparison.mp4` combines the spatial views in one
two-by-two video: ground truth at top left, current MFGP estimate at top right,
current MFGP variance at bottom left, and current signed density error at bottom
right. Every panel uses the same proposed robot state at each frame. All
four-method video and PNG overlays reuse the paper figure palette: aerial robots
and paths are orange, while ground robots and paths are green.

Video titles are static: they do not include episode numbers, steps, current
values, or metric-direction subtitles. The $\mathcal{L}$ and $\mathcal{E}$
videos intentionally have no y-axis label because the metric is identified in
the title. The same output directory also receives `ground_truth_density.png`,
`mfgp_trajectories_estimated_density.png`,
`mfgp_trajectories_variance.png`, and
`mfgp_trajectories_density_difference.png`. The first is an uncluttered truth
reference; the others show every proposed aerial and ground trajectory over the
final causally available MFGP density, latent variance, and signed estimate
minus truth fields. Every density video and PNG uses `Reds`, with important
regions rendered red. Variance uses `Greys`, with high-variance unexplored
regions darker. The signed difference uses symmetric `RdBu_r`: blue is
underestimation, white is agreement, and red is overestimation. Truth and
estimated density share one color scale.

To compare one heterogeneous composition against its equal-size aerial-only
and ground-only baselines from the saved composition sweep, run:

```bash
uv run python evaluation/render_homogeneous_baseline_episode_videos.py \
  --input output/multifidelity_composition_raw.npz \
  --composition A2/G8 \
  --episode 7 \
  --output-dir output/homogeneous_baseline_videos/A2_G8/episode_007 \
  --fps 20 \
  --frame-interval 1
```

This writes five H.264 MP4s without rerunning the simulation: synchronized
three-panel estimated- and ground-truth-density videos, followed by live NRMSE,
KL, and NLPD comparisons. Each reconstruction curve updates only when that
composition publishes a saved asynchronous posterior. The renderer verifies
that all three runs share the seed, truth, map, query grid, and mission clock.
It deliberately omits $\mathcal{L}$ and $\mathcal{E}$ because those
ground-coverage metrics are undefined for the aerial-only endpoint. Titles are
static and robot trajectory trails are not drawn.

Regenerate the two-column paper reconstruction figure as vector graphics with:

```bash
uv run python evaluation/plot_multifidelity_composition.py \
  --input output/multifidelity_scenario_pipeline/evaluated/easy_long.npz \
  --output paper/pics/error_metrics.pdf
```

The defaults target the IEEE two-column text width used by
`paper/updated.tex`. The directly editable style flags are `--figure-width`,
`--figure-height`, `--title-font-size`, `--axis-label-font-size`,
`--tick-label-font-size`, `--legend-font-size`, `--line-width`,
`--band-alpha`, and `--dpi`. PDF output is vector-based; `--dpi` controls raster
formats such as PNG. Publication PDF generators embed fonts as CID TrueType
rather than Matplotlib Type 3 glyphs. Audit a generated asset with
`pdffonts paper/pics/error_metrics.pdf`; its `type` column must not contain
`Type 3`.

Run the separate obstacle-free Zhang et al. baseline with:

```bash
uv run python evaluation/run_zhang_baseline.py \
  --config configs/zhang_baseline.yaml
uv run python evaluation/plot_zhang_baseline.py
```

Its default experiment fixes the paper's `A5/G14` team and sweeps ground
sensing range. Additional valid compositions can be declared independently in
the Zhang configuration; each must satisfy `aerial > 1` and `ground > aerial`.
This runner is not invoked by the multifidelity scenario pipeline and does not
use the multifidelity estimator.

Run the complete paired obstacle-free `A3/G7` comparison with one command:

```bash
uv run python evaluation/run_baseline_comparison_pipeline.py \
  --config configs/baseline_comparison_a3_g7.yaml \
  --zhang-config configs/zhang_comparison.yaml
```

The pipeline writes separate raw archives for the proposed, Egerstedt, and
Zhang methods below `output/three_method_comparison/`, then creates a common
evaluated archive, metric plot, and three-panel trajectory plot. Zhang aerial
deployment consumes the shared 30-second mission clock; its ground robots stay
stationary until Algorithm 1 finishes. Add `--reuse-raw` to recompute only the
offline evaluation and plots.

To run the explicitly labelled finite-aerial-range Zhang-LR stress-test variant
on the same paired comparison, replace the Zhang configuration:

```bash
uv run python evaluation/run_baseline_comparison_pipeline.py \
  --episodes 1 \
  --config configs/baseline_comparison_a3_g7.yaml \
  --zhang-config configs/zhang_lr_comparison.yaml \
  --output-dir output/three_method_comparison_zhang_lr
```

Zhang-LR clips LOW-field power-cell information to the saved aerial sensing
disk. It is an opt-in finite-range extension, not a control law claimed by the
original Zhang paper. Ground sensing remains unchanged.

Run a direct, exactly paired standard-Zhang versus Zhang-LR comparison with:

```bash
uv run python evaluation/run_zhang_variant_comparison.py \
  --episodes 1 \
  --config configs/baseline_comparison_a3_g7.yaml \
  --standard-config configs/zhang_comparison.yaml \
  --lr-config configs/zhang_lr_comparison.yaml \
  --output-dir output/zhang_variant_comparison
```

This command generates one immutable obstacle-free A3/G7 scenario, runs both
variants on its identical initial states and 30-second mission clock, evaluates
their paired differences, and writes `comparison.png`. The six panels show both
trajectory sets, per-robot position divergence, Zhang-LR power-cell support,
consensus target masses, and ground coverage. Pass `--scenario PATH` to reuse
an existing proposed-method raw scenario archive instead of generating one.

Run the final four-condition paired comparison with:

```bash
uv run python evaluation/run_four_method_comparison_pipeline.py \
  --episodes 30 \
  --episode 0 \
  --config configs/baseline_comparison_a3_g7.yaml \
  --zhang-config configs/zhang_comparison.yaml \
  --egerstedt-full-range 75 \
  --output-dir output/four_method_comparison
```

The immutable scenario keeps the proposed method's limited aerial radius 10.
It compares the proposed method and Egerstedt-LR at radius 10 against
full-domain Egerstedt at radius 75 and standard full-cell Zhang. The
controller-specific Egerstedt override is archived separately and never changes
the paired scenario fingerprint. The only final figure is
`plots/coverage_locational_cost.png`, containing footprint-normalized ground
coverage and nearest-ground locational cost with paired-bootstrap 95% confidence
bands. The optional `--episode 0` additionally writes the illustrative
`plots/trajectories_episode_000.png`; omit the option to create only the final
quantitative figure. Episode indices are zero-based and must refer to a saved
episode. Egerstedt consumes oracle truth density in both range conditions. Add
`--reuse-raw` to regenerate evaluation and either plot without rerunning robots.

The standalone four-method plotters default to IEEE single-column width. The
two metric histories remain side by side and use one shared legend below the
panels so the data remain readable at `width=\linewidth`:

```bash
uv run python evaluation/plot_four_method_comparison.py \
  --evaluated output/four_method_comparison/evaluated.npz \
  --output paper/pics/coverage_locational_cost.pdf
uv run python evaluation/plot_four_method_trajectories.py \
  --input-dir output/four_method_comparison \
  --episode 0 \
  --output paper/pics/four_method_trajectories.pdf
```

Both scripts expose `--figure-width`, `--figure-height`, `--title-font-size`,
`--axis-label-font-size`, `--tick-label-font-size`, and `--dpi`. The metric
plotter additionally exposes legend, line-width, and confidence-band-opacity
controls. The trajectory figure intentionally has no color scale, legend, or
global title.
Its colors are centralized near the top of
`evaluation/plot_four_method_trajectories.py`; edit `AERIAL_COLOR`,
`GROUND_COLOR`, and the adjacent footprint/outline constants to restyle all
four panels consistently.

Generate the simulation-independent aerial/ground observation-resolution
illustration with:

```bash
uv run python examples/plot_observation_resolution.py
```

The default outputs are `paper/pics/aerial_observation.pdf`,
`paper/pics/ground_observation.pdf`, `paper/pics/density_estimate.pdf`, and
`paper/pics/mfgp_uncertainty.pdf`. The first two sample one Gaussian peak over
the same 10-by-10-meter footprint at the publication resolutions
$\Delta_a=1.0$ m and $\Delta_g=0.1$ m, making the aerial observation visibly
coarser without introducing simulation state or sensor noise. The latter two
place that observation patch in a 30-by-20-meter illustrative full domain. The
estimate follows the observed peak locally and approaches intermediate
prior-like values in unexplored areas through a broad radial Gaussian-distance
blend. The explored footprint is a 5-meter-radius disk, and normalized
uncertainty follows the same smooth circular envelope, remaining low inside the
disk and increasing outside it. Thus, `Greys` renders the footprint bright and
the unexplored domain dark without a hard boundary. Each file contains only one
bare heatmap, with no titles, axes, ticks, annotations, or colorbars. Use the
four `--*-output` options and the extent, resolution, full-domain, and
observation-position options shown by `--help` to customize the figures.

## Configuration

`configs/multifidelity.yaml` is the canonical self-contained configuration.
The estimator is selected explicitly:

```yaml
estimator_mode: multifidelity

multifidelity:
  rho: 0.8
  gp_update_period: 1.0
  aerial_sensor_period: 0.5
  ground_sensor_period: 0.1
```

The complete, leaf-by-leaf reference—including units, valid ranges, and
runtime effects—is in
[`docs/multifidelity_config_reference.md`](docs/multifidelity_config_reference.md).

## How the coupled loop works

1. Aerial and ground sensors collect area-uniform samples in their configured
   fields of view. 
2. The simulation timestamps and submits observations to the central estimator.
3. On scheduled estimator updates, the GP fits the joint LOW/HIGH observation
   model and publishes a versioned HIGH-field posterior.
4. The aerial controller receives an interest-plus-uncertainty target density
   derived from that posterior.
5. The ground controller receives a posterior-derived importance density.
6. HEDAC and the ground controller advance the robot states at their configured
   rates.

This ordering permits a HIGH-fidelity ground observation to change the
posterior, aerial target density, and subsequent aerial command or trajectory.

## Documentation

 equations for the fields, sensors, multi-fidelity GP, densities, controllers,
 dynamics, scheduling, and metrics, with an equation-to-code map.
- [Mathematical formulation (PDF)](https://arscontrol.github.io/air_ground_coverage/mathematical_formulation.pdf): typeset implemented equations for the fields, sensors, multi-fidelity GP, densities, controllers, dynamics, scheduling, and metrics. The [source and build details](docs/mathematical_formulation.md) remain in the repository.
- [Multi-fidelity configuration reference](docs/multifidelity_config_reference.md):
  detailed documentation for every parameter in the canonical configuration.

## Repository layout

```text
configs/       Canonical and smoke multi-fidelity configurations
docs/          Mathematical reference, configuration guide, status, and figures
evaluation/    Reproducible evaluation entry points
examples/      Coupled simulation CLI and standalone plot scripts
src/           Simulation, estimator, density, sensors, and controllers
tests/         Deterministic unit, integration, and behavioral tests
```

## Validation and reproducible figures

Run the deterministic test suite with:

```bash
uv run pytest
```

Milestone plots are generated by standalone scripts in `examples/` and saved
under `docs/assets/`. The exact commands and interpretation of each artifact
are recorded in
[`docs/multifidelity_gp_status.md`](docs/multifidelity_gp_status.md).
