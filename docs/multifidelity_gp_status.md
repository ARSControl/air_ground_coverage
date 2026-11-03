# Multi-Fidelity GP Implementation Status

## Current Milestone

The full-visibility Proposed--Rudolph--Zhang comparison pipeline is
**implemented and smoke-verified on 2026-10-21**. It resolves a shared
obstacle-free, diagonal-covering configuration and runs all three methods on
the same scenarios and mission clock.

## Full-visibility three-method comparison milestone

- Added `evaluation/run_full_visibility_comparison_pipeline.py` and
  `tests/test_full_visibility_comparison_pipeline.py`. Tests were added first;
  their intentional red run failed because the new module did not yet exist.
- The runner derives the shared full-visibility radius from the map diagonal,
  applies it to both aerial and ground sensing with 360-degree fields of view,
  archives the resolved YAML, and rejects obstacles or an undersized explicit
  radius. Internally `egerstedt` denotes the Rudolph [17] implementation; the
  three-method figures now use the publication label `Rudolph [17]`.
- Proposed MFGP, Rudolph, and standard Zhang consume the same saved truth,
  seeds, initial states, and duration. Zhang's initial deployment remains part
  of the mission clock. Full visibility changes the proposed method's sampling
  support, not its configured sample count.
- A deterministic one-episode/four-step real smoke run completed all three
  methods and produced evaluated metrics and trajectories. The original-size
  trajectory figure was inspected, and reproducible copies are under
  `docs/assets/full_visibility_comparison_smoke/`.
- Validation passed 2/2 new tests and 22/22 in the relevant three-method,
  Rudolph-adapter, Zhang, and evaluator selection. Ruff, Python compilation,
  and changed-file whitespace checks passed. The mathematical reference built
  successfully as a 26-page PDF with its existing layout warnings. The full
  suite reported 325 passed and the same four known unrelated failures:
  canonical step count, ground trajectory color, canonical/ablation FOV
  mismatch, and progress posterior-version text.
- Production 30-episode/long-duration experiments were not run. With full
  ground visibility, probability-mass coverage saturates at one and cannot
  distinguish controllers. Nearest-ground locational cost, travel, and timing
  remain informative; this limitation is documented rather than hidden.

## Combined parameter-sweep metric summary milestone

- Added `evaluation/plot_multifidelity_parameter_sweep_summary.py` and reusable
  overview/effect plotting functions. No simulation is rerun: both figures read
  only the completed evaluated archives and manifest.
- The absolute view places all six factors in aligned panels with a shared
  metric axis, episode points, mean markers, and deterministic bootstrap 95%
  intervals. The effect view pairs conditions and references by episode seed
  and plots the mean condition-minus-reference difference with a paired
  bootstrap interval.
- The completed 21-condition study generated
  `docs/assets/all_parameters__final_kl.png` and
  `docs/assets/all_parameters__final_kl__paired_effects.png`. Both were visually
  inspected at original resolution. The full-range effect is visibly distinct;
  the plots retain outliers rather than clipping them.
- Deterministic tests were added first and intentionally failed during import
  before the functions existed. The final parameter-sweep selection passed
  6/6, Ruff and whitespace checks passed, and the existing mathematical PDF
  built successfully. No estimator, controller, archive value, or simulation
  behavior changed.

## Proposed-method parameter-sensitivity sweep milestone

- Added `evaluation/multifidelity_parameter_sweep.py` and
  `evaluation/run_multifidelity_parameter_sweep.py`, production and smoke YAML
  configurations, deterministic tests, a standalone milestone plotter, and the
  real-output visual `docs/assets/multifidelity_parameter_sweep_milestone.png`.
- Tests were written first; the intentional red run failed at collection with
  `ModuleNotFoundError`. During implementation, the resolved-condition contract
  exposed that a placeholder base composition would make conditions differ in
  more than one factor. The resolver now constructs every condition from the
  six explicitly declared reference levels, preserving a true one-factor-at-a-
  time design.
- Production levels are 30/50/70-m square environments; A1/G9, A2/G8, and
  A5/G5 teams; 10/30/60-s missions; 0/5/15 radius-1 obstacles; short,
  reference, and dynamically diagonal-covering ranges; and matched low,
  reference, and high LOW/HIGH sensor and GP noise variances. All values remain
  declarative and may be revised before publication runs.
- The full-environment range is resolved as $\sqrt{W^2+H^2}$ for both robot
  types and archives the diagonal-coverage check. This removes only the range
  restriction: obstacle line of sight remains active.
- Each condition receives a resolved YAML, proposed-method raw archive,
  separate evaluated archive, and manifest entry. Evaluation reports final and
  held-last time-averaged posterior and coverage metrics. Factor plots show
  episode values with bootstrap 95% mean intervals. `--reuse-raw` regenerates
  evaluation and figures without rerunning simulation.
- The smoke run completed all 12 conditions (two levels for each factor) and
  generated six factor plots. The inspected sensing-range milestone PNG is
  2360-by-1316 pixels and is generated from those real archives. These smoke
  outputs verify plumbing only and are not scientific conclusions.
- Validation: the five new direct tests passed; the relevant 32-test selection
  passed; Ruff and Python compilation passed; and the full suite reported 322
  passed with the same four known unrelated failures (canonical step count,
  ground trajectory color, canonical/ablation FOV mismatch, and progress text).
  The mathematical reference built successfully as a 25-page PDF, retaining
  only its existing layout warnings. No numerical or behavioral regression was
  found outside the deliberately varied conditions.
- The production repeated-episode study was not run because of its cost. The
  Zhang timing/convergence pilot and all baseline comparisons remain deferred,
  as requested. Documentation changed in `README.md`,
  `docs/multifidelity_config_reference.md`, both mathematical references,
  `docs/reviewer_revision_progress.md`, and this file.

## Obstacle-aware line-of-sight sampling milestone

- Files changed: `src/core/obstacles.py`, `src/models/sensors.py`,
  `src/simulation.py`, `tests/test_circular_ground_obstacles.py`,
  `tests/test_multifidelity_sensors.py`, new
  `tests/test_line_of_sight_sampling_plot.py`, new
  `examples/plot_line_of_sight_sampling_milestone.py`, new real-output
  `docs/assets/line_of_sight_sampling_milestone.png`, `README.md`,
  `docs/multifidelity_config_reference.md`,
  `docs/mathematical_formulation.md`,
  `docs/latex/mathematical_formulation.tex`,
  `docs/reviewer_revision_progress.md`, and this status file.
- Deterministic tests were added before production code. Their intentional red
  run failed during collection because `line_of_sight_is_clear` did not exist.
  The first implementation run then exposed one over-strong assertion that
  assumed every single-attempt random draw would cross a wall; it was narrowed
  to the actual contract that returned samples are visible and exhausted
  retries never create a blocked fallback.
- An occupied raster cell is treated as a closed unit square centred at its
  raster coordinate. LOS uses a deterministic segment--closed-square slab
  intersection, so direct intersections, occupied endpoints, and tangent or
  boundary contact are blocked. Invalid out-of-map endpoints fail explicitly.
- Every requested sample retains the existing area-uniform sector proposal.
  Out-of-bounds and occluded proposals are rejection-sampled up to the existing
  attempt limit. A failed opportunity returns no observation, so a collection
  event can submit fewer than the configured count or an empty batch. Noise is
  generated only after visibility succeeds.
- The coupled builder supplies the shared ground occupancy raster to both the
  LOW aerial and HIGH ground sensors. This exact 2-D model treats ground
  obstacles as vertically occluding aerial measurements even though aerial
  motion may overfly them. Obstacle-free maps bypass LOS work and a regression
  contract proves that they preserve the original deterministic sensor stream.
- The real-output visual contains the production sensor's accepted samples and
  fixed probes classified by the production LOS predicate. The inspected PNG
  is 1296-by-1007 and clearly separates visible and occluded rays around an
  L-shaped raster obstacle.
- Commands executed included the intentional red focused tests; repeated
  focused geometry/sensor/scheduler/feedback/legacy selections; Ruff format
  and lint; Python compilation; the four-step obstacle-aware multi-fidelity
  smoke run; deterministic figure generation and original-resolution visual
  inspection; the full test suite; an out-of-tree two-pass LaTeX build; and
  changed-file whitespace checks.
- The final relevant selection passed 75/75. The full suite run immediately
  before the final focused empty-batch contract collected 320 tests: 316
  passed, with exactly the same four unrelated failures as the previous
  milestone (canonical config step expectation, ground trajectory color,
  ablation/canonical configuration mismatch, and progress posterior-version
  text). The final empty-batch contract passed in the focused selection. The
  mathematical reference built successfully as a 25-page PDF with its
  pre-existing underfull/overfull equation-to-code table warnings.
- Intended behavioral change: obstacle-containing scenarios may submit fewer
  samples and consequently produce different posteriors, controller targets,
  and trajectories. Numerical or behavioral regressions on obstacle-free
  sensor streams: none. No GP equation, controller law, robot motion model,
  evaluation metric, or legacy estimator behavior changed.

## Illustrative observation-resolution figure milestone

- Files changed: new `examples/plot_observation_resolution.py`, new
  `tests/test_observation_resolution_plot.py`, `README.md`, the generated vector
  assets `paper/pics/aerial_observation.pdf` and
  `paper/pics/ground_observation.pdf`, plus the wider vector assets
  `paper/pics/density_estimate.pdf` and `paper/pics/mfgp_uncertainty.pdf`; the
  generated previews
  `docs/assets/aerial_observation_milestone.png` and
  `docs/assets/ground_observation_milestone.png`, plus
  `docs/assets/density_estimate_milestone.png` and
  `docs/assets/mfgp_uncertainty_milestone.png`; and this status file. The
  superseded combined PDF and preview were removed.
- The figure compares one 10-by-10-meter footprint sampled at
  $\Delta_a=1.0$ m and $\Delta_g=0.1$ m. The resulting 10-by-10 aerial grid and
  100-by-100 ground grid are saved as separate square files and use the same
  domain, `YlOrRd` colormap, and color normalization. Each contains only its
  heatmap: titles, labels, axes, ticks, cell outlines, annotations, and colorbar
  are absent. The underlying field is a single isotropic Gaussian
  peak, ready for later composition beside a wider-domain MFGP estimate.
- The additional 30-by-20-meter illustrative full-domain estimate uses the
  same peak inside a 5-meter-radius circular observation footprint and smoothly
  blends to intermediate prior-like values elsewhere. Its matching normalized
  uncertainty is low over the observed disk and high over the unexplored
  domain. With the `Greys` colormap, this deliberately renders observed space
  bright and unexplored space dark. Both wider maps are also bare heatmaps.
- The observed-to-unexplored transition now uses radial Gaussian decay outside
  the circular footprint, with a fade length equal to 30% of the original
  10-meter observation extent. Observation weight remains one inside the disk
  and decays smoothly outside it; both density estimate and uncertainty use
  this same envelope.
- Deterministic tests were added first. Their initial collection failed because
  the plotting module did not exist. The separate-file/single-peak revision was
  also observed failing against the combined multi-feature implementation.
  After implementation, all ten focused contracts passed. They verify the
  single symmetric peak, grid dimensions, the 100-to-1 sample-count ratio,
  common physical extent, input validation, absence of non-heatmap decoration,
  exact colormaps, shared color limits, full-domain array dimensions, observed
  versus unexplored mean/uncertainty behavior, circular radial symmetry,
  bounded per-cell transition steps, broad transition support, and creation of
  all four files.
- Commands executed: `.venv/bin/python -m pytest -q
  tests/test_observation_resolution_plot.py`; the final relevant selection with
  `tests/test_multifidelity_sensors.py`; `.venv/bin/python
  examples/plot_observation_resolution.py`; the same plot command with separate
  `--aerial-output`, `--ground-output`, `--estimate-output`, and
  `--uncertainty-output` preview paths; `file`/`pdfinfo` artifact inspection;
  and original-resolution visual inspection of both 1065-by-1065 observation
  previews and both 1597-by-1065 wider previews. The final relevant selection
  passed 20/20; Ruff lint and formatting, Python compilation, and changed-file
  whitespace checks also passed.
- No simulation data are represented, and no estimator, sensor implementation,
  field transform, controller, coverage law, motion model, discretization,
  scheduler, or metric changed. The illustration uses configured resolution
  values only, so `docs/mathematical_formulation.md` remains current. Numerical
  and behavioral regressions: none.

## Experimental-figure caption milestone

- Files changed: `paper/updated.tex` and this status file.
- Figure 2 now identifies all three team compositions, the three reconstruction
  metrics and their direction, the 30 paired episodes, and the bootstrap 95%
  confidence bands.
- Figure 3 now identifies all four paired A3/G7 conditions, distinguishes the
  finite-range and full-visibility assumptions, defines the direction of
  coverage effectiveness and locational cost, and states the mean and paired
  bootstrap confidence-band semantics.
- Figure 4 now explains the shared grayscale target density, aerial and ground
  trajectory styles, initial and final markers, and final sensing-footprint
  outlines.
- Validation commands: `latexmk -pdf -interaction=nonstopmode -halt-on-error
  updated.tex` from `paper/`; `pdftoppm -png -r 150 -f 7 -l 8 updated.pdf
  /tmp/ral-caption-paper`; original-resolution inspection of both rendered
  evaluation pages; caption-placeholder, PDF-text, and whitespace checks.
- Validation result: the two-pass paper build succeeded and produced a
  nine-page PDF. Figures 2--4, their captions, and their cross-references are
  readable and unclipped. No code tests were run because this milestone changes
  only prose in the paper and status document. The build retains the existing
  missing-author, package, underfull-box, and undefined `da fare` reference
  warnings.
- This is a paper-only documentation change. No estimator, field transform,
  sensor distribution, controller, coverage law, motion model, discretization,
  scheduler, evaluation metric, numerical result, or figure asset changed, so
  `docs/mathematical_formulation.md` remains current.

## Optional four-condition trajectory milestone

- Files changed: new `evaluation/plot_four_method_trajectories.py`;
  `evaluation/run_four_method_comparison_pipeline.py`,
  `tests/test_four_method_comparison_pipeline.py`, `README.md`,
  `docs/experimental_evaluation_plan.md`,
  `docs/architecture/implementation_three_method_comparison.md`,
  `docs/mathematical_formulation.md`,
  `docs/latex/mathematical_formulation.tex`, `.gitignore`; new real-output
  `docs/assets/four_method_trajectory_milestone.png`; and this status file.
- The deterministic contract was changed before production code. Its red run
  failed because `FourMethodComparisonArtifacts` had no trajectory field. After
  implementation, the focused four-condition selection passed `3/3`; the
  broader Egerstedt/four-condition/three-method/Zhang compatibility selection
  passed `22/22`.
- `--episode` is optional and zero-based. Omitting it returns
  `trajectory_plot=None` and creates no trajectory file. Supplying index $N$
  writes `plots/trajectories_episode_NNN.png`. Invalid negative, Boolean, or
  out-of-archive indices fail explicitly. `--reuse-raw --episode N` evaluates
  and plots saved archives without rerunning any controller.
- The standalone plotter reloads all four raw archives and revalidates every
  scenario-contract array and recorded/recomputed fingerprint. It also requires
  same-range Egerstedt-LR, domain-covering Egerstedt full-range, and standard
  full-cell Zhang metadata before rendering.
- The two-by-two figure uses one hidden-truth density and color scale for all
  panels. It plots class-consistent aerial/ground paths, initial and final
  states, limited aerial disks, final ground footprints, dynamically archived
  range labels, and Zhang's Algorithm 2 transition. Episode index, seed, and
  fingerprint prefix are recorded in the title. Full-range Egerstedt's radius
  75 disk is omitted because it exceeds the complete plotted domain; its value
  remains explicit in the panel title.
- The real visual used episode 0, seed 42, from a valid shared five-episode,
  300-step archive with radius 10 and fingerprint `2bb850380184...`.
  `docs/assets/four_method_trajectory_milestone.png` was inspected at its
  original 2062-by-1856 resolution. Plotting did not mutate the raw archives.
- This milestone changes no estimator, field transform, sensor distribution,
  controller, coverage law, motion model, numerical discretization, scheduler,
  or evaluation metric. The mathematical equations are unchanged; only the
  equation-to-code map now includes the optional plotter.
- Commands executed included the red and green contract tests, Ruff format and
  lint, the real `--reuse-raw --episode 0` pipeline, standalone milestone plot
  generation, original-resolution image inspection, archive provenance checks,
  broader compatibility tests, Python compilation, `git diff --check`, the
  full suite, and a forced LaTeX build. The mathematical reference remains 25
  pages with the existing stream, glue, and table-width warnings.
- The full suite collected 275 tests: `271` passed, with exactly the same four
  unrelated failures as before (canonical config step expectation, ground
  trajectory color, ablation ground FOV, and progress posterior-version text).
  Numerical and behavioral regressions: none; this is a read-only plotting path
  over already saved trajectories.

## Final four-condition comparison milestone

- Files changed: `configs/baseline_comparison_a3_g7.yaml`,
  `evaluation/comparison_io.py`, `evaluation/run_egerstedt_comparison.py`; new
  `evaluation/evaluate_four_method_comparison.py`,
  `evaluation/plot_four_method_comparison.py`,
  `evaluation/run_four_method_comparison_pipeline.py`, and
  `tests/test_four_method_comparison_pipeline.py`; expanded
  `tests/test_egerstedt_comparison_runner.py`; new real-output visual
  `docs/assets/four_method_comparison_milestone.png`; plus `README.md`,
  `docs/experimental_evaluation_plan.md`,
  `docs/architecture/implementation_three_method_comparison.md`,
  `docs/mathematical_formulation.md`,
  `docs/latex/mathematical_formulation.tex`, `.gitignore`, and this status
  file.
- Deterministic tests were added first. The intentional red collection failed
  because the four-method archive identity did not exist. After implementation,
  the focused Egerstedt/four-condition selection passed `6/6`; the broader
  new/three-method/Zhang compatibility selection passed `22/22`.
- The proposed-method scenario and Egerstedt-LR use aerial radius 10.
  Full-range Egerstedt uses an effective controller override of 75, which
  exceeds the query-domain diagonal `70.710678`. Standard Zhang retains complete
  LOW-field power cells. Both Egerstedt conditions retain oracle truth-density
  access and are labelled accordingly.
- The Egerstedt runner now preserves the scenario-contract
  `aerial_sensing_range` unchanged and archives the actual controller value as
  `effective_aerial_sensing_range`. Metadata records scenario/effective ranges,
  whether an override occurred, the domain diagonal, and whether that radius
  covers the complete domain. The omission default remains backward compatible.
- The four-condition evaluator requires exact equality of every scenario array
  and fingerprint, same-range Egerstedt-LR, domain-covering full Egerstedt, and
  standard `full_cell` Zhang. It computes footprint-normalized ground coverage
  and truth-weighted nearest-ground locational cost, their final/time-average
  values, and shared-index paired-bootstrap 95% intervals. It fabricates no
  posterior metric for a baseline.
- The pipeline emits four independent raw archives, one evaluated archive, and
  only `plots/coverage_locational_cost.png`. The plot is a one-by-two history
  figure with dynamically archived radius labels; no trajectory, runtime,
  travel, reconstruction, or phase panel is produced.
- The first production-duration validation exposed that the base A3/G7 file had
  been set to radius 75: both Egerstedt conditions were identical. Those outputs
  were rejected. A new publication-contract test failed intentionally on that
  setting; the base radius was restored to 10, the test passed, and the complete
  300-step run was regenerated before any result was accepted.
- The valid deterministic A3/G7 run used seed 42, 300 steps at 0.1 seconds, and
  fingerprint `a8eff4a2571a...`. Effective aerial ranges were `[10, 10, 75,
  full-cell]`. Maximum limited/full Egerstedt aerial and ground path differences
  were `25.192459` and `30.231697`, confirming a behavioral rather than metadata
  change.
- Valid single-episode final footprint-normalized coverage was `0.556577`,
  `0.000363`, `0.854806`, and `0.154819` for proposed-LR, Egerstedt-LR,
  Egerstedt-full, and Zhang-full. Corresponding final locational costs were
  `35.359655`, `576.422556`, `26.515176`, and `156.238443`. Time-average
  coverage was `0.217912`, `0.000168`, `0.516103`, and `0.025067`; time-average
  cost was `350.073170`, `769.783224`, `162.027111`, and `1040.926796`. These
  validate the implementation and are not multi-episode statistical claims.
- The inspected real-output figure
  `docs/assets/four_method_comparison_milestone.png` is 2396-by-920 and contains
  exactly the requested coverage and locational-cost panels. With one validation
  episode the confidence bounds coincide with the means; the documented
  publication command runs 30 paired episodes.
- Commands executed included intentional red and green tests, the invalid and
  corrected complete 300-step runs, archive/fingerprint/range/path-difference
  inspection, real-figure regeneration and original-resolution inspection,
  focused and compatibility tests, Ruff format and lint, Python compilation,
  `git diff --check`, the full suite, and a forced two-pass LaTeX build. The
  mathematical reference builds as 25 pages; existing stream, glue, and
  table-width warnings remain.
- The full suite collected 275 tests: `271` passed, with exactly the same four
  unrelated failures as before (canonical config step expectation, ground
  trajectory color, ablation ground FOV, and progress posterior-version text).
  Numerical and behavioral regressions: none in the existing three-method,
  Zhang-LR, direct Zhang, standalone Zhang, proposed estimator/controller, or
  default Egerstedt paths.

## Direct standard-Zhang versus Zhang-LR comparison milestone

- Files changed: `evaluation/comparison_io.py`; new
  `evaluation/evaluate_zhang_variant_comparison.py`,
  `evaluation/plot_zhang_variant_comparison.py`,
  `evaluation/run_zhang_variant_comparison.py`,
  `tests/test_zhang_variant_comparison.py`, and
  `docs/assets/zhang_variant_comparison_milestone.png`; plus `README.md`,
  `docs/experimental_evaluation_plan.md`,
  `docs/architecture/implementation_three_method_comparison.md`,
  `docs/mathematical_formulation.md`,
  `docs/latex/mathematical_formulation.tex`, `.gitignore`, and this status
  file.
- Tests were written before the new production modules. The intentional red
  collection failed because the evaluated-archive kind did not exist. The
  first implementation run then exposed an over-strong smoke assertion: eight
  steps make the aerial policies differ, but are too short for their raster
  assignments and ground paths necessarily to differ. The corrected contract
  requires the aerial difference and treats delayed ground divergence as a
  reported metric. The focused new tests passed `2/2`; the broader
  Zhang/Egerstedt/evaluator/plot selection passed `25/25`.
- `evaluation/run_zhang_variant_comparison.py` generates one scenario archive
  or accepts `--scenario PATH`, passes it unchanged to the full-cell and
  finite-range configurations, evaluates, and plots. The evaluator rejects
  method-label, fingerprint, scenario-array, and result-shape mismatches.
- The evaluated archive contains per-robot aerial and ground Euclidean
  differences, first-divergence times at threshold $10^{-10}$, standard and LR
  cell support, consensus target masses and loads, final and time-average
  coverage and locational cost, cumulative class travel, and transition times.
  No fabricated posterior metric is assigned to either Zhang variant.
- The deterministic real A3/G7 run used seed 42, 300 steps at 0.1 seconds,
  scenario fingerprint `a8eff4a2571a...`, and the physical aerial radius 10.
  Both variants entered Algorithm 2 at 18.4 seconds. Aerial paths first differed
  at 18.5 seconds and ground paths at 18.6 seconds; their final maximum
  per-robot differences were `2.800114` and `3.174696`, respectively.
- Zhang-LR's minimum and episode-mean cell support were `0.053148` and
  `0.397249`. Standard target masses were `[0.347852, 0.341337, 0.532747]`,
  compared with `[0.101132, 0.071969, 0.263460]` for Zhang-LR. Final normalized
  ground coverage was `0.154819` versus `0.135384`; time-average coverage was
  `0.025067` versus `0.020479`. Final locational cost was `156.238443` versus
  `170.021073`. These are deterministic single-episode validation values, not
  statistical claims.
- The inspected real-output visual
  `docs/assets/zhang_variant_comparison_milestone.png` is 2864-by-1676. It shows
  both trajectory sets, LR sensing disks, aerial/ground path divergence,
  minimum/mean support, target-mass changes, and direct coverage histories.
- Commands executed included the red and focused tests, the complete one-command
  300-step comparison, standalone plot regeneration, archive metric extraction,
  original-resolution image inspection, the broader regression selection,
  Ruff format and lint, Python compilation, `git diff --check`, the full suite,
  and a forced two-pass LaTeX build. The mathematical reference remains 24
  pages; existing stream, glue, and table-width warnings remain.
- The full suite collected 271 tests: `267` passed, with exactly the same four
  unrelated failures as the prior milestone (canonical config step
  expectation, ground trajectory color, ablation ground FOV, and progress
  posterior-version text). Numerical and behavioral regressions: none in the
  standard Zhang, Zhang-LR, proposed multi-fidelity, or Egerstedt controllers;
  this milestone adds paired orchestration, offline metrics, and plotting only.

## Zhang-LR finite-aerial-range milestone

- Files changed: `evaluation/zhang.py`,
  `evaluation/run_zhang_comparison.py`,
  `evaluation/evaluate_three_method_comparison.py`,
  `evaluation/plot_three_method_comparison.py`, new
  `configs/zhang_lr_comparison.yaml`, new
  `configs/zhang_lr_comparison_smoke.yaml`,
  `tests/test_zhang_baseline.py`,
  `tests/test_three_method_comparison_pipeline.py`, new
  `examples/plot_zhang_lr_milestone.py`, `README.md`,
  `docs/experimental_evaluation_plan.md`,
  `docs/architecture/implementation_three_method_comparison.md`,
  `docs/multifidelity_config_reference.md`,
  `docs/mathematical_formulation.md`,
  `docs/latex/mathematical_formulation.tex`, new real-output figures under
  `docs/assets/zhang_lr_milestone/`, `.gitignore`, and this status file.
- Tests were added before production changes. The intentional red selection
  failed twice: `ZhangParameters` did not accept an aerial range and the
  Zhang-LR paired configuration did not exist. After implementation, the
  core/paired/pipeline Zhang selection passed `13/13`; the broader relevant
  Zhang/Egerstedt/evaluator/plotter selection passed `23/23`.
- `algorithm.aerial_sensing_mode: full_cell` is the omission default and keeps
  the original Zhang equations. `limited` selects Zhang-LR and reads the exact
  physical aerial sensing range saved by the proposed-method scenario archive.
  The range is transformed with the existing isotropic 3-by-3 coordinate map.
- Zhang-LR leaves Algorithm 1 geometric deployment, load consensus, mission
  timing, ground sensing, and ground control unchanged. Its LOW-field cell
  masses use the rasterized intersection between each power cell and its
  aerial sensing disk. Shared boundary-face information contributes separately
  to each incident aerial command only when that face midpoint is inside that
  robot's disk.
- Each state archives the quadrature-area support fraction of every complete
  power cell inside its sensing disk. Metadata identifies the method as
  `zhang_lr`, records both physical and normalized radii, and explicitly marks
  the extension as not paper-faithful. The evaluator propagates the diagnostic;
  aggregate and trajectory figures label Zhang-LR, plot support histories, and
  draw final aerial sensing disks.
- The deterministic real run used the production A3/G7 scenario, 300 steps,
  and physical aerial radius 10 (normalized radius 0.6). Algorithm 2 still
  began at 18.4 seconds. The minimum cell support over the episode was
  `0.053148`, final mean support was `0.378521`, and final normalized ground
  coverage was `0.135384`. The corresponding original full-cell Zhang result
  was `0.154819`; these single-episode values validate behavior and are not
  statistical claims.
- Real implementation outputs were inspected at original resolution:
  `docs/assets/zhang_lr_milestone/metrics.png` is 2756-by-1568 and includes the
  minimum/mean support histories; `trajectories_episode_000.png` is 2928-by-992
  and shows the radius-10 aerial disks, common truth, and all three paired
  trajectories.
- Commands executed included the intentional red tests, focused green tests,
  the complete 300-step milestone pipeline, image inspection, Ruff format and
  lint checks, Python compilation, `git diff --check`, the full test suite, and
  a forced two-pass LaTeX build. The mathematical reference now builds as 24
  pages; existing stream, glue, and table-width warnings remain.
- The full suite collected 269 tests: `265` passed, with only the same four
  unrelated failures (canonical config step expectation, ground trajectory
  color, ablation ground FOV, and progress posterior-version text).
- Numerical and behavioral regressions: none in omitted/`full_cell` Zhang,
  standalone Zhang, proposed multi-fidelity, or Egerstedt paths. Zhang-LR
  intentionally changes sensed masses, target masses, boundary commands,
  aerial and ground trajectories, coverage, and runtime. Its outputs must not
  be reported as the original Zhang et al. method.

## Thirty-second paired-mission horizon update

- The user extended `configs/baseline_comparison_a3_g7.yaml` from 200 to 300
  integration steps. With the unchanged 0.1-second step, every proposed,
  Egerstedt, and Zhang episode now has the common physical horizon
  $T=K\Delta t=30$ seconds.
- Files updated for the new contract: `README.md`,
  `docs/experimental_evaluation_plan.md`,
  `docs/architecture/implementation_three_method_comparison.md`,
  `docs/mathematical_formulation.md`,
  `docs/latex/mathematical_formulation.tex`,
  `tests/test_three_method_comparison_pipeline.py`,
  `examples/plot_three_method_comparison_milestone.py`, the two real-output
  figures under `docs/assets/three_method_comparison_milestone/`, and this
  status file. No controller, estimator, baseline, scheduler, or metric code
  changed.
- The production-contract test now requires exactly 300 steps and 30 seconds.
  The focused comparison selection passed `22/22`; Ruff format and lint checks,
  Python compilation, and `git diff --check` passed.
- The deterministic one-episode production run was repeated for all 300 steps.
  Zhang's deployment transition remained 18.4 seconds, so Algorithm 2 now gets
  11.6 seconds instead of 1.6 seconds. The regenerated trajectory figure shows
  the materially longer ground paths, and the metrics figure spans the complete
  30-second clock. Both figures were inspected at 2928-by-992 and 2756-by-1568,
  respectively.
- In this validation episode, final normalized coverage was `0.556577`,
  `0.000363`, and `0.154819` for proposed, Egerstedt, and Zhang; corresponding
  time averages were `0.217912`, `0.000168`, and `0.025067`. These values
  replace the obsolete 20-second milestone observations and are not population
  claims.
- A forced LaTeX build produced the 23-page mathematical reference. The full
  suite collected 266 tests: `262` passed, and only the same four unrelated
  failures remain (canonical config step expectation, ground trajectory color,
  ablation ground FOV, and progress posterior-version text).
- Numerical and behavioral regressions: the extended horizon intentionally
  changes final/time-averaged metrics, travel, trajectories, runtime, posterior
  history length, and Zhang's usable Algorithm 2 duration. Pairing, initial
  conditions, controller equations, sensing policy, and estimators are
  unchanged. Raw 20-second archives must not be reused as 30-second results.

## Paired A3/G7 three-method baseline-comparison milestone

- Files changed: `evaluation/zhang.py`, `evaluation/comparison_io.py`, new
  `evaluation/run_zhang_comparison.py`, new
  `evaluation/evaluate_three_method_comparison.py`, new
  `evaluation/plot_three_method_comparison.py`, new
  `evaluation/run_baseline_comparison_pipeline.py`, new
  `configs/baseline_comparison_a3_g7.yaml`, new
  `configs/zhang_comparison.yaml`, new
  `configs/zhang_comparison_smoke.yaml`, new
  `tests/test_three_method_comparison_pipeline.py`, new
  `examples/plot_three_method_comparison_milestone.py`, new
  `docs/architecture/implementation_three_method_comparison.md`, `README.md`,
  `docs/experimental_evaluation_plan.md`, `docs/mathematical_formulation.md`,
  `docs/latex/mathematical_formulation.tex`, new real-output figures under
  `docs/assets/three_method_comparison_milestone/`, `.gitignore`, and this
  status file.
- The production protocol fixes ten robots as A3/G7, 300 steps at 0.1 seconds,
  no obstacles, common corner initial states, a 360-degree radius-5 ground
  footprint, and decentralized aerial HEDAC for the proposed method. The
  proposed archive is the immutable scenario contract: both baselines verify
  its fingerprint and reuse its exact seeds, state times, initial positions,
  truth fields, domain, and sensing radius.
- The paired Zhang adapter maps the physical square affinely to the paper's
  3-by-3 coordinates. Algorithm 1 aerial deployment starts at mission time
  zero and holds all ground states fixed. On convergence, distributed load and
  mass consensus executes without advancing physical time, while its measured
  duration remains part of wall runtime. Algorithm 2 receives only the time
  left before the common 30-second horizon. The standalone Zhang runner keeps
  its predeployment behavior for backward compatibility.
- The normalized raster spacing is about 0.061, so the paired configuration
  declares a 0.02 deployment residual tolerance rather than relying on
  sub-grid centroid changes. Its theorem-compliant consensus gain is
  `0.5 * min_i(s_i / degree_i)` and is recorded in the raw archive.
- Tests were written before production integration. The intentional red run
  failed for the missing three-method archive constants and runner/pipeline
  modules. After implementation, the focused Zhang, comparison-runner,
  evaluator, and plotter selection passed `22/22`.
- The one-command real validation used the production A3/G7 configuration for
  one deterministic 300-step episode. Zhang transitioned at 18.4 seconds, so
  its ground team was stationary for 184 steps and Algorithm 2 received 11.6
  seconds, or 116 integration intervals, before the common deadline.
- On that validation episode, final footprint-normalized coverage was
  `0.556577` for proposed multi-fidelity, `0.000363` for Egerstedt, and
  `0.154819` for Zhang. Time-averaged values were `0.217912`, `0.000168`, and
  `0.025067`; final nearest-ground locational costs were `35.3597`, `576.423`,
  and `156.238`. Final accumulated ground travel was `323.156`, `790.370`, and
  `119.056`, respectively. These are validation observations, not statistical
  claims;
  the publication default remains 30 episodes with paired bootstrap intervals.
- The common evaluator reports coverage histories, final and time-averaged
  coverage, locational cost, aerial and ground travel, complete controller-step
  wall runtime, bootstrap 95% intervals, and paired coverage differences.
  Reconstruction and calibration are explicitly `N/A` for both literature
  baselines because they do not natively publish a probabilistic HIGH-field
  posterior.
- Deterministic real-output figures were regenerated and visually inspected:
  `docs/assets/three_method_comparison_milestone/metrics.png` is 2756-by-1568,
  and `trajectories_episode_000.png` is 2928-by-992. They show common truth and
  axes, class-consistent trajectories and footprints, the Zhang transition,
  coverage/cost/travel histories, runtime scope, and confidence summaries.
- Commands executed included Ruff format/lint, the real one-episode pipeline
  through the milestone script, the focused 22-test selection, the complete
  test suite, image inspection, Python compilation and diff checks, and a
  forced `latexmk` PDF build. The full suite collected 266 tests: `262` passed,
  with only the four previously documented unrelated failures (canonical 200
  versus expected 300 steps, cyan versus expected orange ground trajectory
  color, ablation 90-degree versus expected 360-degree ground FOV, and missing
  `posterior_version=` progress text).
- Mathematical documentation now distinguishes the continuous paper model,
  raster quadrature, the paired affine coordinate transform, the full mission
  clock and phase transition, the adaptive consensus gain, and every common
  metric. The LaTeX reference built a 23-page PDF; existing stream, glue, and
  table-width warnings remain.
- Numerical and behavioral regressions: none in the proposed, Egerstedt, or
  standalone Zhang runners. The paired Zhang adapter is new and intentionally
  differs from standalone predeployment by charging deployment to the mission
  horizon. A one-episode comparison is insufficient for method ranking; use
  the default 30 paired episodes for reported results.

## Decentralized easy-only scenario-pipeline milestone

- Files changed: `evaluation/run_multifidelity_scenario_pipeline.py`,
  `evaluation/run_multifidelity_composition.py`,
  `evaluation/evaluate_multifidelity_composition.py`,
  `evaluation/plot_multifidelity_scenario_comparison.py`,
  `tests/test_multifidelity_scenario_pipeline.py`, `README.md`,
  `docs/experimental_evaluation_plan.md`,
  `docs/architecture/scenario_comparison_design.md`,
  `docs/architecture/implementation_composition_sweep.md`,
  `docs/multifidelity_config_reference.md`, the real pipeline artifacts under
  `output/multifidelity_decentralized_easy_pipeline_smoke`, and this status
  file.
- Omitting a selector resolves `composition_sweep.default_scenario`, which is
  `easy_long` in both composition configurations. `--scenario NAME` explicitly
  selects one declaration and `--all-scenarios` restores the prior two-scenario
  behavior. A cross-scenario plot is emitted only when exactly two scenarios
  are selected, and the result's comparison path is otherwise `None`.
- Raw archives now record the normalized aerial controller mode, neighbor
  weighting, and communication range. The evaluator propagates this protocol,
  and the cross-scenario plotter rejects mismatched controller policies.
- Tests were changed before production code. The intentional red pipeline run
  produced `3` failures for missing CLI options, the old both-scenario default,
  and the missing API flag. After implementation, the scenario, composition,
  and trajectory selection passed `29/29`.
- The default deterministic smoke command ran without `--scenario` or
  `--all-scenarios`, produced only `easy_long` artifacts, and recorded
  decentralized/hard/range-3 controller metadata in both archives. Its metric
  and A1/G1 trajectory/reconstruction figures were inspected from real output.
- Commands executed include the red and green pipeline test selections and the
  default `evaluation/run_multifidelity_scenario_pipeline.py` smoke command
  with one episode, ten bootstrap samples, and no progress display.
- Final validation: Ruff lint and formatting checks passed, Python compilation
  passed, and `git diff --check` passed. The full suite collected 256 tests:
  `252` passed and the four previously documented unrelated expectations remain
  (canonical 200 versus expected 300 steps, cyan versus expected orange ground
  trajectory color, ablation 90-degree versus expected 360-degree ground FOV,
  and missing `posterior_version=` progress text). The approved
  `./scripts/validate.sh` command could not be run because that file is absent
  from this checkout.
- Mathematical documentation: estimator, controller, sensor, dynamics,
  scheduler, and metric equations are unchanged in this orchestration
  milestone. The decentralized controller equations were documented in the two
  preceding milestones.
- Numerical and behavioral regressions: default scenario-pipeline runtime and
  output count decrease because `hard_short` is no longer implicit. Explicit
  `--all-scenarios` reproduces the selection topology of the old pipeline. Raw
  archives made with centralized coverage must not be reused as decentralized
  results.

## Decentralized aerial-controller integration milestone

- Files changed: `src/coupled_simulation.py`,
  `configs/multifidelity_composition.yaml`,
  `configs/multifidelity_composition_smoke.yaml`,
  `tests/test_decentralized_hedac.py`,
  `tests/test_aerial_multifidelity_feedback.py`, new
  `docs/architecture/implementation_decentralized_aerial_control.md`,
  `docs/multifidelity_config_reference.md`, new
  `docs/assets/decentralized_multifidelity_easy_milestone.png`, and this status
  file.
- `aerial.ergodic_control.mode` is an independent controller selector with
  `centralized` as the omission default and `decentralized` as the composition
  protocol choice. Unknown modes fail before the simulation is constructed.
- The coupled schedule and estimator are unchanged: LOW collection precedes
  aerial motion, all local aerial controllers consume one cached central HIGH
  target, and HIGH collection/update follows aerial motion. The inherited
  legacy GP is not updated in multifidelity mode, and no local state owns a GP.
- Tests were added before builder changes. The intentional red integration run
  had `2` failures because the builder ignored both the decentralized and
  invalid mode values, while `12` mathematical and causal tests passed. After
  integration, the expanded controller/scheduler/ground-feedback selection
  passed `34/34`.
- The deterministic ground-observation causal test verifies a changed HIGH
  posterior mean, changed cached aerial target, changed local heat, and changed
  real Dubins trajectory. It therefore preserves the required ground-to-aerial
  closed-loop effect under decentralized control.
- Real integrated output used the smoke composition configuration, one
  `easy_long` episode, 12 steps, and the A1/G1 trajectory selection. It
  published six central posterior versions and produced the inspected
  trajectory/truth/reconstruction/error figure
  `docs/assets/decentralized_multifidelity_easy_milestone.png`.
- Commands executed: the focused `34`-test selection, the single-scenario smoke
  composition runner, and the existing real-archive trajectory plotter with
  `MPLCONFIGDIR=/tmp/ral_marta_mpl`.
- Numerical and behavioral regressions: the composition configuration's aerial
  trajectories and therefore later LOW observations, posterior fits, runtime,
  and metrics intentionally change. Existing raw composition archives must not
  be reused for this protocol. Configurations omitting the new selector retain
  centralized trajectories.

## Decentralized aerial-HEDAC core milestone

- Files changed: new `src/core/decentralized_hedac.py`, reusable optional-field
  seams in `src/core/hedac.py`, new `tests/test_decentralized_hedac.py`, new
  `examples/plot_decentralized_hedac_core_milestone.py`,
  `docs/mathematical_formulation.md`, new
  `docs/assets/decentralized_hedac_core_milestone.png`, and this status file.
- Each aerial robot maintains only its own time-averaged raster footprint and
  heat field. A fresh Euclidean range graph is built from pre-motion snapshots;
  only current neighbors contribute through a hard spatial support of the same
  configured radius. Same-step snapshots and command batching make the
  implementation deterministic and independent of in-loop robot motion.
- The class inherits one legacy-compatibility GP from `HEDACAlgorithm`; it does
  not construct a GP per robot. In multifidelity use the simulation-owned
  central estimator remains the sole estimator and supplies one cached HIGH
  target to the local controllers.
- Tests were added before production code. The intentional red run failed at
  collection because `src.core.decentralized_hedac` did not exist. The first
  shell invocation also exposed an environment-only `pytest` entry-point path
  issue; rerunning via `.venv/bin/python -m pytest` reached the intended red
  failure. After implementation, the new contracts plus existing aerial
  feedback tests passed `10/10`.
- Deterministic real-output validation runs three Dubins agents for 80 control
  steps and renders their target, trajectories, final range links, and three
  distinct local heat fields in
  `docs/assets/decentralized_hedac_core_milestone.png`.
- Commands executed: `.venv/bin/python -m pytest -q
  tests/test_decentralized_hedac.py`, `.venv/bin/python -m pytest -q
  tests/test_decentralized_hedac.py tests/test_aerial_multifidelity_feedback.py`,
  and `MPLCONFIGDIR=/tmp/ral_marta_mpl .venv/bin/python
  examples/plot_decentralized_hedac_core_milestone.py`.
- Numerical and behavioral regressions: none in the existing centralized path.
  The reusable HEDAC methods now accept optional caller-owned coverage and heat
  arrays, while their default shared-state behavior is unchanged. The new
  controller is not yet selected by the coupled builder in this milestone.

## Common-corner composition-initialization milestone

- Files changed: `configs/multifidelity_composition.yaml`,
  `configs/multifidelity_composition_smoke.yaml`, `src/coupled_simulation.py`,
  `src/core/multifidelity_estimator.py`,
  `evaluation/run_multifidelity_composition.py`,
  `evaluation/evaluate_multifidelity_composition.py`, new
  `tests/test_corner_initialization.py`,
  `tests/test_multifidelity_estimator.py`,
  `tests/test_multifidelity_composition_pipeline.py`, `README.md`,
  `docs/mathematical_formulation.md`,
  `docs/latex/mathematical_formulation.tex`,
  `docs/multifidelity_config_reference.md`,
  `docs/experimental_evaluation_plan.md`,
  `docs/architecture/implementation_composition_sweep.md`,
  `docs/architecture/scenario_comparison_design.md`, new
  `docs/assets/corner_initialization_milestone.png`, and this status file.
- The opt-in policy uses the free cells in the lower-left 20% by 20% rectangle.
  Complete aerial and ground candidate permutations use episode seeds plus
  `40000` and `50000`, respectively. Truncating those sequences to each team
  count makes same-class initial states nested across compositions. Positions
  are sampled without replacement, headings use the same class-specific
  streams, and too few free corner cells cause an explicit failure.
- Configurations that omit `initialization.position_policy`, including the
  canonical non-composition configuration, retain the historical uniform
  initializer and its global-RNG draw order. The raw and evaluated composition
  archives now record the resolved initialization policy.
- Corner deployment exposed a valid startup case in which every raw GP mean was
  nonpositive. The estimator now publishes a uniform free-space controller
  density only in that zero-positive-mass case. It preserves the actual raw
  posterior mean and variance, so NRMSE, NLPD, calibration, and reconstruction
  plots are not replaced or made artificially uniform. Other candidate-update
  failures retain the most recent valid posterior as before.
- Tests were added before production changes and initially failed all seven
  corner contracts. After the first integration, broader tests exposed the
  zero-mass startup failure; a dedicated negative-observation estimator test
  reproduced it before the density fallback was implemented. The expanded
  focused selection then passed `56/56`.
- Deterministic real-output validation ran the complete two-scenario smoke
  pipeline with one episode and wrote
  `output/multifidelity_corner_pipeline_smoke`. In both scenarios, A2/G0 starts
  aerial robots at `(1,0),(0,1)`, A1/G1 reuses aerial `(1,0)` and ground
  `(1,1)`, and A0/G2 reuses ground `(1,1),(1,0)`. Successful posterior versions
  were `[1,2,3]` for easy/long and `[1,2]` for hard/short. The real A1/G1
  trajectory/truth/reconstruction/error figure was inspected at its original
  2362-by-1964 resolution and saved as
  `docs/assets/corner_initialization_milestone.png`; starts, paths, obstacles,
  fields, error map, legend, and axes are readable.
- Commands executed included the dedicated red/green corner tests, the
  56-test estimator/obstacle/composition/trajectory selection, the complete
  scenario smoke pipeline, the standalone milestone plotter, Ruff lint and
  format checks, `git diff --check`, the full test suite, and a two-pass
  `latexmk` build of the mathematical reference.
- Validation: the full suite collected 247 tests; `243` passed. The four
  failures are the previously documented unrelated expectations: canonical 300
  versus configured 200 steps, orange versus current cyan ground trajectory
  color, ablation 360-degree versus configured 90-degree ground FOV, and
  required `posterior_version=` progress text. Ruff and formatting checks plus
  `git diff --check` passed. LaTeX built a 21-page PDF successfully; existing
  stream and table-width warnings remain.
- Numerical and behavioral regressions: composition trajectories, observations,
  fitted hyperparameters, and all reconstruction metrics intentionally change
  because initial positions change. Previously saved uniformly initialized raw
  archives cannot represent this protocol and must not be reused; the full
  simulation pipeline must be rerun. Outside the opt-in composition configs,
  initialization is unchanged. The only general behavioral change is the
  uniform controller-density fallback for an all-nonpositive posterior; raw GP
  outputs remain unchanged.

## Optimization-enabled scenario-pipeline milestone

- Files changed: `configs/multifidelity_composition_smoke.yaml`,
  `evaluation/run_multifidelity_composition.py`,
  `evaluation/evaluate_multifidelity_composition.py`,
  `evaluation/plot_multifidelity_composition.py`,
  `evaluation/plot_multifidelity_composition_trajectories.py`,
  `evaluation/plot_multifidelity_scenario_comparison.py`, new
  `evaluation/run_multifidelity_scenario_pipeline.py`,
  `tests/test_multifidelity_composition_pipeline.py`, new
  `tests/test_multifidelity_scenario_pipeline.py`,
  `README.md`,
  `docs/mathematical_formulation.md`,
  `docs/latex/mathematical_formulation.tex`,
  `docs/experimental_evaluation_plan.md`,
  `docs/multifidelity_config_reference.md`,
  `docs/architecture/implementation_composition_sweep.md`,
  `docs/architecture/scenario_comparison_design.md`, and this status file.
  The user-supplied publication configuration already had online optimization
  enabled and a discrepancy-length lower bound of `3.0`; this milestone keeps
  those choices and removes the runner guard that rejected them.
- Every composition and episode starts from the same configured four kernel
  values and uses the same bounds, minimum sample count, fit interval,
  deterministic restart count, and iteration limit. The fitted values are
  allowed to differ because each condition uses its own retained observations.
  `rho`, observation-noise variances, and the Cholesky jitter policy remain
  fixed. No new estimator or per-robot GP was introduced.
- Every raw posterior record now stores whether optimization ran, its duration,
  and the realized LOW length/variance and discrepancy length/variance. The
  offline evaluator propagates these arrays and the complete optimization
  policy. The two-scenario plotter rejects archives with different policies;
  metric titles identify online optimization, and the episode plot reports the
  final realized LOW and discrepancy length scales.
- The new Python-only CLI runs exactly the two scenarios declared in YAML,
  evaluates each archive, writes one aggregate metric plot and one explicitly
  selected episode trajectory/truth/reconstruction/error plot per scenario,
  and writes the combined scenario history plot. It executes the existing
  stages sequentially and adds no multiprocessing, middleware, or networking.
- Primary command:
  `MPLCONFIGDIR=/tmp/ral_marta_mpl .venv/bin/python
  evaluation/run_multifidelity_scenario_pipeline.py --config
  configs/multifidelity_composition.yaml --composition A2/G8 --episode 0
  --output-dir output/multifidelity_scenario_pipeline`.
- Tests were added before production changes. The optimization test first
  failed at the former `composition sweep requires fixed GP hyperparameters`
  guard. The orchestration test then failed during collection because the new
  module did not exist. After implementation, the focused
  composition/trajectory/GP selection passed `87/87`.
- Deterministic real-output validation used the smoke configuration with one
  episode, `A1/G1`, 100 bootstrap samples, and output directory
  `output/multifidelity_optimized_pipeline_smoke`. It produced two raw
  archives, two evaluated archives, two 2220-by-1491-class metric PNGs, two
  2362-by-1964 trajectory/reconstruction PNGs, and one 2180-by-1545 comparison
  PNG. The comparison and easy trajectory figures were visually inspected at
  original resolution; labels, online-optimization annotation, fitted kernel
  values, trajectories, obstacles, fields, errors, legends, and axes are
  readable and unclipped.
- The smoke run produced nine fitted posterior records in `easy_long` and six
  in `hard_short`. For easy A1/G1 the final LOW/discrepancy length scales were
  `5.0/1.8655020077`, final NRMSE was `0.1444061003`, and final KL was
  `2.8002040847`. For hard A1/G1 they were `2.4573193300/0.7134529719`,
  `0.2211305987`, and `0.2917497744`. Several homogeneous endpoint parameters
  hit or retained bounds, as expected from their single-fidelity
  identifiability limitations; smoke values are diagnostics, not publication
  evidence.
- Validation: all 236 collected tests were executed in smaller groups after a
  monolithic run terminated while retaining large archive fixtures. `232`
  passed. Four unrelated existing expectations still fail: canonical 300 versus
  configured 200 steps, orange versus current cyan final-plot ground paths,
  ablation 360-degree versus configured 90-degree ground FOV, and required
  `posterior_version=` progress text. Ruff check, Ruff format check, Python
  compilation, and `git diff --check` passed. The LaTeX mathematical reference
  built successfully in two passes; existing stream and table-width warnings
  remain.
- Mathematical documentation now states the exact composition-specific fitting
  schedule, including the `(r-1) mod K_fit` condition, and distinguishes common
  optimization policy from data-dependent realized parameters. Evaluation
  equations, controller laws, field transforms, sensor laws, robot dynamics,
  and legacy mode are unchanged.
- Numerical and behavioral regressions: composition trajectories and metrics
  intentionally change relative to fixed-kernel archives because fitting is now
  active in the closed loop and therefore changes posterior feedback. Runtime
  also includes scheduled optimization. No behavior outside the composition
  experiment was changed.

## Scenario time-series figure milestone

- Files changed: `evaluation/plot_multifidelity_scenario_comparison.py`,
  `tests/test_multifidelity_composition_pipeline.py`,
  `docs/experimental_evaluation_plan.md`,
  `docs/architecture/implementation_composition_sweep.md`,
  `docs/architecture/scenario_comparison_design.md`,
  `docs/assets/scenario_time_series_metrics_milestone.png`, and this status
  file.
- The comparison figure is now a 2-by-2 time-series layout: easy/long NRMSE,
  easy/long KL divergence, hard/short NRMSE, and hard/short KL divergence. All
  panels use physical mission time, episode means, and deterministic bootstrap
  95% bands with a shared composition color mapping. Y limits are shared by
  metric column (NRMSE across rows and KL across rows), while x limits remain
  scenario-specific because the mission horizons differ.
- Final and time-averaged NRMSE/KL arrays remain unchanged in evaluated
  archives for tables and offline analysis; the figure simply no longer calls
  the summary-panel helper or displays those aggregates.
- The regression test was added first and failed because the plotter still
  invoked `_plot_summary`. After implementation it records the metric requested
  by every history call and requires the exact sequence `nrmse`, `kl`, `nrmse`,
  `kl`; it also asserts column-wise y sharing and independent x axes.
- Visual regeneration command: `MPLCONFIGDIR=/tmp/ral_marta_mpl
  .venv/bin/python evaluation/plot_multifidelity_scenario_comparison.py --easy
  /tmp/default_both_easy_long_evaluated.npz --hard
  /tmp/default_both_hard_short_evaluated.npz --output
  docs/assets/scenario_time_series_metrics_milestone.png --bootstrap-samples
  100`. The 2180-by-1545 deterministic smoke PNG was inspected; all four
  histories, scenario-specific time axes, metric labels, titles, grid lines,
  and composition legend are readable and unclipped. Its numerical values are
  pipeline diagnostics, not publication results.
- Mathematical documentation: evaluation metrics and time averaging are
  unchanged. This is a visualization-only selection change, so no estimator,
  field, sensor, controller, motion, scheduler, discretization, or metric
  equation changed.
- Numerical and behavioral regressions: none. Evaluation archives and all
  simulation outputs are byte-independent of this read-only plotting change.
- Validation: the focused composition/trajectory selection passed `25/25`;
  Ruff lint/formatting, Python compilation, and `git diff --check` passed. The
  full suite passed 228 tests with five failures. Four are the previously
  documented canonical-step, final-plot color, ablation-FOV, and CLI-progress
  mismatches. The fifth reflects a concurrent user edit of the publication
  aerial counts to `[10,4,2,0]` while the existing frozen constant/test still
  declares `[10,2,0]`; this plotting milestone preserves that edit and does
  not silently redefine the experiment protocol.

## Default-all-scenarios CLI milestone

This preceding milestone remains current for zero-argument scenario execution
and output naming.

- Files changed: `evaluation/run_multifidelity_composition.py`,
  `tests/test_multifidelity_composition_pipeline.py`,
  `docs/experimental_evaluation_plan.md`,
  `docs/multifidelity_config_reference.md`,
  `docs/architecture/implementation_composition_sweep.md`,
  `docs/architecture/scenario_comparison_design.md`,
  `docs/assets/default_all_scenarios_cli_milestone.png`, and this status file.
- CLI omission of `--scenario` reads every key in
  `composition_sweep.scenarios` in configuration order. For a requested path
  `composition_raw.npz`, the current protocol writes
  `composition_easy_long_raw.npz` and `composition_hard_short_raw.npz`; the
  unsuffixed path is not written, so one scenario cannot overwrite another.
- Supplying `--scenario NAME` preserves exact single-scenario behavior and uses
  the exact `--output` path. Configurations with no named scenarios also retain
  the historical one-archive behavior. The programmatic
  `run_composition_sweep()` API remains a single-scenario function and still
  uses `default_scenario` when its scenario argument is omitted.
- The regression test was added first and failed because only the unsuffixed
  easy/default archive existed. It then passed after CLI orchestration and
  output naming were implemented. The complete focused composition and
  trajectory selection passed `25/25`.
- End-to-end regeneration omitted `--scenario` with the deterministic smoke
  configuration and printed two paths:
  `/tmp/default_both_easy_long_raw.npz` and
  `/tmp/default_both_hard_short_raw.npz`. Both were evaluated and combined with
  the existing scenario-comparison plotter. Visual regeneration used
  `docs/assets/default_all_scenarios_cli_milestone.png`; the 2180-by-1545 image
  was inspected and all panels, scenario-specific time axes, legends, labels,
  and titles are readable and unclipped.
- Full validation passed 229 tests. The same four unrelated workspace failures
  remain: canonical steps 200 versus a test expecting 300, final-plot ground
  color cyan versus a test expecting orange, ablation ground FOV 90 degrees
  versus canonical 360 degrees, and CLI progress missing `posterior_version`.
  Ruff lint/formatting, Python compilation, and `git diff --check` passed.
- Mathematical documentation: no estimator, metric, field transform, sensing,
  controller, motion, discretization, or scheduler equation changed; only CLI
  orchestration changed, so the mathematical reference remains current.
- Numerical and behavioral regressions: none. Each individual scenario invokes
  the same runner with the same resolved inputs as an explicit invocation. The
  change affects only which scenarios the CLI schedules and how multiple raw
  output paths are named.

## Easy/long versus hard/short composition milestone

This preceding milestone remains current for the scenario definitions,
archive metadata, evaluator, and comparison-plot semantics.

- Files changed: `configs/multifidelity_composition.yaml`,
  `configs/multifidelity_composition_smoke.yaml`,
  `evaluation/run_multifidelity_composition.py`,
  `evaluation/evaluate_multifidelity_composition.py`,
  `evaluation/plot_multifidelity_composition.py`,
  `evaluation/plot_multifidelity_composition_trajectories.py`, new
  `evaluation/plot_multifidelity_scenario_comparison.py`,
  `tests/test_multifidelity_composition_pipeline.py`,
  `docs/experimental_evaluation_plan.md`,
  `docs/multifidelity_config_reference.md`,
  `docs/mathematical_formulation.md`,
  `docs/latex/mathematical_formulation.tex`,
  `docs/architecture/implementation_composition_sweep.md`, new
  `docs/architecture/scenario_comparison_design.md`,
  `docs/assets/easy_long_hard_short_composition_milestone.png`, and this
  status file.
- The publication composition list is now `(10, 2, 0)`: homogeneous A10/G0
  and A0/G10 bracket the operationally motivated A2/G8 scout-plus-ground team.
  The prior broader sweep is pilot evidence and must not be described as the
  confirmatory protocol.
- `composition_sweep.scenarios` declares `easy_long` as 200 steps, five
  radius-one ground circles and `hard_short` as 100 steps, fifteen radius-two
  circles. With `dt=0.1`, their horizons are 20 and 10 simulated seconds. Both
  duration and geometry change, so this implementation compares bundled
  operating regimes and cannot identify an obstacle-only or duration-only
  causal effect.
- The single-scenario runner API accepts a scenario name; omission selects
  `default_scenario`. It resolves step count and ground geometry before
  building each existing production simulation and records the scenario name, exact resolved
  parameters, and scenario-resolved aerial/ground configurations. An explicit
  `--num-steps` remains the highest-precedence development override and is
  recorded honestly.
- One scenario is stored per raw archive. This preserves the existing
  rectangular composition/episode/time array contract even though the two
  scenarios have different horizons. Configurations without named scenarios
  and old raw/evaluated archives remain supported under the synthetic name
  `default`; no archive schema version changed.
- The evaluator propagates scenario metadata but uses the existing metric
  equations. The single-scenario plotter and saved-episode trajectory title now
  identify the scenario. The new comparison plotter reads two evaluated
  archives only, requires matching composition/count/seed arrays, and plots
  scenario-specific zero-clipped NRMSE histories plus final/time-averaged
  summaries.
- The active retention budget remains 400. A10/G0, A2/G8, and A0/G10 receive
  LOW/HIGH caps `(400,1)`, `(80,320)`, and `(1,400)`. Every composition submits
  100 observations per sensing event; the easy/long and hard/short horizons
  therefore submit 4000 and 2000 observations per episode respectively.
- Tests were added before implementation. The first red run failed collection
  because the new comparison plotter was absent. A second red test found that
  resolved hard-scenario metadata still reported base values, and a third
  found that legacy single-scenario obstacle defaults were incorrectly read
  from the aerial rather than ground parameter view. All three defects were
  fixed before broader validation.
- Deterministic end-to-end validation commands ran the smoke configuration
  once with `--scenario easy_long` and once with `--scenario hard_short`, wrote
  separate `/tmp/composition_*_smoke_raw.npz` archives, evaluated both, and
  combined them with `evaluation/plot_multifidelity_scenario_comparison.py`.
  Easy/long saved shape `(3,1,7,2,6)`, one three-cell obstacle, six submitted
  samples per composition, and final NRMSE `[0.23826975, 0.10887563,
  0.27799765]`. Hard/short saved shape `(3,1,5,2,6)`, two six-cell obstacles,
  four submitted samples per composition, and final NRMSE `[0.28388622,
  0.19192342, 0.27836863]`. These are pipeline diagnostics, not publication
  evidence.
- A separate construction smoke check used the publication configuration with
  `--scenario hard_short --episodes 1 --num-steps 12`. It produced A10/G0,
  A2/G8, and A0/G10 with state shape `(3,1,13,10,6)`, the exact resolved
  fifteen radius-two obstacles, 190 identical occupied cells per composition,
  and 300 submitted observations per composition over three sensing events.
  The explicit 12-step override was recorded and this short archive is not a
  substitute for the declared 100-step hard mission.
- Visual regeneration command: `MPLCONFIGDIR=/tmp/ral_marta_mpl
  .venv/bin/python evaluation/plot_multifidelity_scenario_comparison.py --easy
  /tmp/composition_easy_long_smoke_evaluated.npz --hard
  /tmp/composition_hard_short_smoke_evaluated.npz --output
  docs/assets/easy_long_hard_short_composition_milestone.png
  --bootstrap-samples 100`. The 2180-by-1545 PNG was visually inspected: all
  four panels, separate time axes, scenario labels, composition legend,
  summary legends, and axis labels are readable and unclipped.
- A geometry-only audit of all 30 publication hard-scenario seeds completed
  without generation failure. Fifteen radius-two circles occupy 7.28% to 7.84%
  of the 50-by-50 raster (mean 7.584%); the largest free component contains at
  least 99.9568% of free cells. This verifies feasible, essentially connected
  clutter without using reconstruction outcomes to tune geometry. It does not
  by itself prove a particular ground-reachability reduction.
- Validation: focused scenario/trajectory/obstacle/documentation tests passed
  `34/34`; the full suite passed 228 tests. Four unrelated workspace failures
  remain: canonical steps 200 versus a test expecting 300, final-plot ground
  color cyan versus a test expecting orange, ablation ground FOV 90 degrees
  versus canonical 360 degrees, and CLI progress missing `posterior_version`.
  Ruff lint and formatting, Python compilation, and `git diff --check` passed.
  The LaTeX mathematical reference built successfully; existing underfull and
  table-width warnings remain.
- Numerical and behavioral regressions: estimator mathematics, target
  densities, controller laws, sensor equations, robot dynamics, logical
  scheduling, legacy mode, and existing archive arrays are unchanged. Expected
  protocol changes are the reduced composition set and scenario-specific
  horizon, obstacle geometry, submitted counts, metadata, and output files.

## Ten-robot composition-protocol alignment milestone

This preceding milestone records the broader six-composition pilot protocol.
It is preserved historically and is superseded for new confirmatory runs by
the three-composition, two-scenario protocol above.

- The user-updated `configs/multifidelity_composition.yaml` fixes `N=10` and
  aerial counts `[10, 8, 6, 4, 2, 0]`. Files aligned in this milestone are
  `evaluation/run_multifidelity_composition.py`,
  `tests/test_multifidelity_composition_pipeline.py`,
  `docs/experimental_evaluation_plan.md`,
  `docs/mathematical_formulation.md`,
  `docs/architecture/implementation_composition_sweep.md`,
  `docs/assets/team_composition_ten_robot_protocol_milestone.png`, and this
  status file.
- The exported publication count contract is now
  `DEFAULT_AERIAL_COUNTS = (10, 8, 6, 4, 2, 0)`. The runner itself continues
  to read `total_robots` and `aerial_counts` from configuration, pads state
  arrays to the configured total, and saves resolved counts and labels in the
  raw archive. No estimator, controller, robot, sensor, or scheduler production
  path changed.
- The evaluator and both composition plotters required no code changes. They
  derive composition count, labels, active A/G counts, aerial fractions, and
  legend entries from the archive. An existing eight-robot raw archive remains
  evaluable and plottable as an eight-robot result, but it cannot be converted
  into ten-robot evidence: the 30-episode composition simulation must be rerun
  before reporting the new protocol.
- The fixed active retention budget remains 400. In A10/G0 through A0/G10
  order, the LOW/HIGH caps are `(400,1)`, `(320,80)`, `(240,160)`, `(160,240)`,
  `(80,320)`, and `(1,400)`, where endpoint caps of one are inactive
  placeholders. Ten robots with ten samples per robot submit 100 observations
  per sensing event and 4000 over the configured 20-second episode, independent
  of composition.
- The regression test was changed before the runner constant. Its red run
  produced one expected protocol failure and 12 passes because the constant
  still omitted A10/G0. After implementation, the composition and trajectory
  selection passed all 20 tests.
- Deterministic real-output validation used:
  `.venv/bin/python evaluation/run_multifidelity_composition.py --config
  configs/multifidelity_composition.yaml --output
  /tmp/multifidelity_composition_n10_raw.npz --episodes 1 --num-steps 12
  --no-progress`, followed by the offline evaluator and metric plotter. The raw
  archive contains labels `A10/G0`, `A8/G2`, `A6/G4`, `A4/G6`, `A2/G8`, and
  `A0/G10`; both state arrays have shape `(6, 1, 13, 10, 6)`, and 12 posterior
  records were evaluated. Each condition submitted exactly 300 observations
  during the three sensing events in this shortened run.
- Visual regeneration command:
  `MPLCONFIGDIR=/tmp/ral_marta_mpl .venv/bin/python
  evaluation/plot_multifidelity_composition.py --input
  /tmp/multifidelity_composition_n10_evaluated.npz --output
  docs/assets/team_composition_ten_robot_protocol_milestone.png
  --bootstrap-samples 100`. The 2144-by-1491 PNG was inspected at original and
  resized resolution; its two history panels, two aerial-fraction summaries,
  N=10 title, all six labels, and axes are readable and unclipped. This
  one-episode, 1.2-second run is a pipeline-validation artifact, not a
  scientific composition result.
- Full validation passed 224 tests. Four unrelated workspace failures remain:
  canonical configuration steps 200 versus a test expecting 300, final-plot
  ground color cyan versus a test expecting orange, ablation ground FOV 90
  degrees versus canonical 360 degrees, and CLI progress text missing
  `posterior_version`. Ruff, Python compilation, and `git diff --check` passed.
- Numerical and behavioral regressions: none outside the intentional protocol
  expansion. The new full runs will have ten-slot state padding, six
  compositions, and 100 rather than 80 submitted observations per sensing
  event. The retained GP training budget is still capped at 400, so increasing
  robot count changes spatial sampling opportunity but not maximum active
  training-set size.

## Zero-clipped reconstruction evaluation milestone

This preceding milestone remains current for metric semantics: field
reconstruction plots and NRMSE evaluation use `max(high_mean, 0)`, density
normalization remains separate, and posterior calibration retains the raw
Gaussian mean and variance.

- Files changed: `evaluation/multifidelity_metrics.py`,
  `evaluation/evaluate_multifidelity_composition.py`,
  `evaluation/evaluate_multifidelity_ablation.py`,
  `evaluation/plot_multifidelity_composition.py`,
  `evaluation/plot_multifidelity_ablation.py`,
  `evaluation/plot_multifidelity_composition_trajectories.py`,
  `tests/test_multifidelity_ablation_pipeline.py`,
  `tests/test_multifidelity_composition_pipeline.py`,
  `docs/assets/composition_trajectory_plotter_milestone.png`,
  `docs/assets/team_composition_reconstruction_zero_clipped_milestone.png`,
  `docs/experimental_evaluation_plan.md`,
  `docs/mathematical_formulation.md`,
  `docs/latex/mathematical_formulation.tex`,
  `docs/architecture/implementation_composition_sweep.md`, and this status
  file.
- The pure evaluation helper `clipped_reconstruction` implements
  $\mu_H^+=\max(\mu_H,0)$ without density normalization. The shared `nrmse`
  function now applies this transform, so both composition and ablation
  evaluators use the same zero-clipped reconstruction automatically.
- KL evaluation is unchanged because it already consumes the saved
  nonnegative normalized posterior density. Empirical 95% calibration remains
  centred on the raw GP mean with raw posterior variance; clipping its centre
  would no longer evaluate the stated Gaussian posterior interval.
- The episode CLI plots $\mu_H^+$ and computes its absolute error against
  hidden HIGH truth on free queries. Truth and reconstruction share a common
  scale whose minimum is now zero for nonnegative truth. Panel titles and the
  composition/ablation metric plot labels explicitly identify zero clipping.
- Tests were added before implementation. The red run failed during collection
  because `clipped_reconstruction` did not exist. The final focused selection
  passed `9/9`, including an exact negative/zero/positive clipping contract,
  an NRMSE case whose negative mean must contribute as zero, evaluated-archive
  metadata, endpoint pipeline execution, latest-posterior selection, and PNG
  generation.
- Full-archive evaluation command used without overwriting the user's existing
  evaluated archive: `.venv/bin/python
  evaluation/evaluate_multifidelity_composition.py --input
  output/multifidelity_composition_raw.npz --output
  /tmp/multifidelity_composition_clipped_evaluated.npz --no-progress`.
  In A8/G0 through A0/G8 order, mean final clipped NRMSE is `0.12419998`,
  `0.11290556`, `0.10427253`, `0.10301847`, and `0.08232371`; the corresponding
  previous raw-mean values in the existing evaluated archive are `0.14941907`,
  `0.13746364`, `0.13199190`, `0.12997328`, and `0.11638623`. These are a metric
  definition change, not new trajectories or posterior fits.
- Visual regeneration uses the existing trajectory CLI command recorded below.
  For A2/G6 episode zero, the selected version-20 reconstruction at time 19 s
  has zero-clipped free-space RMSE `0.114485994786`, maximum absolute error
  `0.590275029077`, and minimum reconstructed value exactly zero. The
  regenerated 2382-by-1964 PNG was inspected at original resolution; all four
  panels, shared truth/reconstruction scale, obstacle masks, labels, and
  colorbars are readable and unclipped.
- Full repeated-run metric visual regeneration command:
  `MPLCONFIGDIR=/tmp/ral_marta_mpl .venv/bin/python
  evaluation/plot_multifidelity_composition.py --input
  /tmp/multifidelity_composition_clipped_evaluated.npz --output
  docs/assets/team_composition_reconstruction_zero_clipped_milestone.png
  --bootstrap-samples 1000`. The 2144-by-1491 PNG was inspected at original
  resolution; zero-clipped NRMSE histories, bootstrap bands, final/time-average
  summaries, unchanged KL panels, labels, and legends are readable and
  unclipped. The earlier `team_composition_reconstruction_milestone.png` is a
  preserved historical artifact using the superseded raw-mean NRMSE definition
  and must not be used for the current metric.
- Mathematical documentation now defines NRMSE using $\mu_H^+$ and separately
  states why calibration retains raw $\mu_H$. No GP posterior, controller
  density, sensor, scheduler, motion, or simulation equation changed.
- Numerical and behavioral regressions: raw archives, posterior means,
  variances, densities, trajectories, and KL values are unchanged. NRMSE values
  intentionally decrease wherever negative raw means previously added error.
  Existing evaluated archives retain the old definition and must be regenerated
  before reporting clipped-NRMSE results.
- Validation results: the broader ablation/composition/trajectory selection
  passed `30` tests and retained only the documented ablation-FOV and
  10-versus-8-robot protocol mismatches. The full suite passed `222` tests with
  the same five unrelated failures recorded in the preceding milestone. Ruff
  lint, changed/new-file formatting, Python compilation, and `git diff --check`
  passed. The LaTeX mathematical reference built successfully; only existing
  table-width and environment stream warnings remain.

## Saved composition-trajectory CLI milestone

The original version below displayed the raw latent posterior mean; the
zero-clipped reconstruction milestone above supersedes that display and its
reported raw-mean error diagnostics while retaining the same CLI selection and
archive-reading behavior.

- Files changed: `evaluation/plot_multifidelity_composition_trajectories.py`,
  `tests/test_plot_multifidelity_composition_trajectories.py`,
  `docs/assets/composition_trajectory_plotter_milestone.png`,
  `docs/experimental_evaluation_plan.md`,
  `docs/architecture/implementation_composition_sweep.md`, and this status
  file.
- The CLI requires `--composition` to be one exact label stored in the raw
  archive and accepts an explicit zero-based `--episode` index, defaulting to
  zero. Omitting `--output` produces an unambiguous path of the form
  `output/multifidelity_composition_trajectories/A2_G6_episode_000.png`.
- The plotter reads the raw archive only. It selects active state slots using
  the saved aerial/ground counts, validates that active trajectories are
  finite, and supports aerial-only, mixed, and ground-only endpoints despite
  their `NaN`-padded inactive slots. It does not import a controller or mutate
  the archive.
- Each figure has four panels: saved trajectories over the hidden HIGH field,
  standalone hidden HIGH truth, the last saved latent HIGH posterior mean, and
  absolute latent-field error. Truth and reconstruction use one common color
  scale; obstacle queries are masked consistently with reconstruction metrics.
  The trajectory panel overlays the saved binary ground map, uses blue aerial
  and orange ground paths, and marks initial positions with circles and final
  positions with crosses. The title records composition, episode index, seed,
  and mission time.
- Tests were added before implementation. The red test failed during
  collection with `ModuleNotFoundError` for the new plotting module. After
  implementation, all seven dedicated tests passed, covering CLI parsing/direct
  execution, both homogeneous endpoints, a mixed team, deterministic selection
  of the latest step/time/version posterior, invalid selection, PNG output, and
  raw-archive immutability.
- The combined new/existing composition selection passed 18 tests and retained
  one workspace protocol failure: the current publication YAML declares 10
  robots and aerial counts `[10, 8, 6, 4, 2, 0]`, while the pre-existing test
  still asserts the former eight-robot protocol. Neither file was changed by
  this plotting milestone.
- Visual regeneration command:
  `MPLCONFIGDIR=/tmp/ral_marta_mpl .venv/bin/python
  evaluation/plot_multifidelity_composition_trajectories.py --input
  output/multifidelity_composition_raw.npz --composition A2/G6 --episode 0
  --output docs/assets/composition_trajectory_plotter_milestone.png`.
  This reads the real 30-episode, 200-step archive and renders episode index
  zero (seed 42), with two aerial and six ground paths over five radius-one
  obstacles occupying 15 cells. The final saved reconstruction is posterior
  version 20 at step 190 and simulated time 19 seconds; on free queries its
  RMSE is `0.136137448624` and maximum absolute error is
  `0.659669780314`. The A2/G6 selection is an explicit visual validation input,
  not an ablation choice or optimality claim.
- The 2384-by-1964 PNG was visually inspected at original resolution. The
  complete 20-second trajectories, truth, reconstruction, error, obstacles,
  markers, legend, titles, axes, and colorbars are readable and unclipped.
- Architecture comparison: one new read-only branch leaves the raw archive and
  ends at a PNG. The estimator, sensor streams, scheduler, controllers, maps,
  robot dynamics, trajectory arrays, metric evaluator, approved target, and
  legacy mode are unchanged.
- Mathematical documentation: no estimator, field transform, sensor law,
  controller, motion model, discretization, scheduler, or metric changed, so
  `docs/mathematical_formulation.md` and its LaTeX source remain current.
- Numerical and behavioral regressions: none. The figure reads and displays
  saved arrays only; it cannot change the previously recorded episode or any
  reconstruction result.
- The full suite completed with `222` passes and five unrelated failures: the
  four previously recorded canonical-step, trajectory-color, ablation-FOV, and
  CLI-progress mismatches, plus the current 10-robot publication YAML versus
  eight-robot test expectation described above. Ruff lint/formatting, Python
  compilation, and `git diff --check` passed for the new Python files.

## Deterministic circular ground-obstacle milestone

- Files changed: `src/core/obstacles.py`, `src/coupled_simulation.py`,
  `src/simulation.py`, `configs/multifidelity_composition.yaml`,
  `configs/multifidelity_composition_smoke.yaml`,
  `evaluation/run_multifidelity_composition.py`,
  `evaluation/run_multifidelity_ablation.py`,
  `evaluation/run_multifidelity_comparison.py`,
  `tests/test_circular_ground_obstacles.py`,
  `tests/test_multifidelity_composition_pipeline.py`,
  `examples/plot_ground_obstacles_milestone.py`,
  `docs/assets/circular_ground_obstacles_milestone.png`,
  `docs/experimental_evaluation_plan.md`,
  `docs/multifidelity_config_reference.md`,
  `docs/mathematical_formulation.md`,
  `docs/latex/mathematical_formulation.tex`,
  `docs/architecture/implementation_composition_sweep.md`,
  `docs/architecture/implementation_ground_obstacles.md`, and this status
  file.
- `generate_circular_obstacle_map` uses a dedicated NumPy generator seeded by
  `episode_seed + 30000`. It samples the exact requested number of continuous
  circle centres inside the raster boundary, rejects centre distances at or
  below twice the configured radius, rasterizes cell centres inside each
  circle, and rejects infeasible inputs rather than silently changing the
  protocol.
- `CoupledSimulation` exposes immutable `ground_map`,
  `ground_obstacle_centers`, and `ground_obstacle_radius` state. Generated
  circles are unioned into the ground map only; the HEDAC/aerial motion map is
  unchanged so aerial robots may overfly ground obstacles. Ground robots are
  initialized from ground-free cells.
- The hidden HIGH field and smoothed LOW field use the two-dimensional ground
  free mask. The estimator receives a separate nearest-cell query mask, so
  published target density is exactly zero at obstacle queries. Composition,
  ablation, and comparison raw archives now record the ground map and evaluate
  reconstruction on its free cells.
- Lloyd motion combines the existing weighted centroid displacement with a
  bounded nearby-occupied-cell repulsion using
  `ground.agents.wall_avoidance_weight`. A heading-first unicycle segment guard
  samples proposed motion at no more than 0.25 map-unit spacing, suppresses
  translation across obstacles or map boundaries, and retains turning. The
  same hard guard protects the unicycle MPC compatibility path. It is bypassed
  exactly on all-free maps.
- The publication composition configuration now freezes five ground circles
  of radius `1.0`; the smoke protocol uses one radius-one circle. Because
  obstacle placement depends only on the paired episode seed, all five team
  compositions receive identical maps and hidden fields within an episode.
- Tests were written before production changes. The red run failed at
  collection with `ModuleNotFoundError: src.core.obstacles`. After
  implementation, the dedicated obstacle plus composition selection passed
  `19/19`; the final obstacle, composition, scheduler, and Lloyd selection
  passed `36/36`.
- End-to-end smoke commands completed successfully: the composition runner
  wrote `/tmp/obstacle_composition_raw.npz`, and the offline evaluator wrote
  `/tmp/obstacle_composition_evaluated.npz`. The raw runner uses the ground map
  for paired free masks and truth-density normalization.
- Visual regeneration command:
  `MPLCONFIGDIR=/tmp/ral_marta_mpl .venv/bin/python
  examples/plot_ground_obstacles_milestone.py`. The four panels show the
  obstacle-free aerial motion map with circle outlines, the occupied ground
  map and trajectories, the masked hidden HIGH field, and the masked posterior
  target density from a deterministic A2/G6 validation run. This illustrative
  team is not a selected ablation composition or an optimality claim.
- Fixed-seed visual diagnostics: five radius-one circles occupied 15 raster
  cells; all 246 saved ground states were free; the final posterior version was
  four; and maximum target density across obstacle queries was exactly zero.
  The regenerated 2085-by-1694 pixel image was inspected at original
  resolution; circle alignment, trajectories, titles, axes, legends, and
  colorbars are readable and unclipped.
- The full suite completed with `216` passes and the same four known unrelated
  failures: a stale expectation of 300 rather than 200 canonical steps, the
  old orange rather than current cyan ground-trajectory color, the existing
  canonical/ablation 360/90-degree ground-FOV mismatch, and missing
  `posterior_version=` text in CLI progress messages. No obstacle test or
  affected controller/scheduler test failed.
- Ruff lint passed for all affected Python files. Ruff formatting passed for
  the new and directly modified formatted files; the two existing evaluation
  runners retain their pre-existing formatting differences to avoid rewriting
  unrelated dirty-worktree edits. Python compilation and `git diff --check`
  passed.
- The mathematical reference records exact placement, rasterization, split-map
  semantics, field/query masking, repulsion, and discrete segment-guard
  equations. `latexmk -pdf -interaction=nonstopmode -halt-on-error
  mathematical_formulation.tex` completed in two passes; only existing table
  width warnings and environment stream warnings remain.
- Architecture comparison: the implementation matches the preserved target
  diagram. It adds no estimator, per-robot GP, concurrency, middleware, or
  aerial no-fly behavior. The legacy estimator branch remains selectable and
  obstacle-free configurations retain their prior control updates.
- Numerical and behavioral regressions: none in obstacle-free focused tests.
  Enabling obstacles intentionally changes truth support, ground initial
  states, trajectories, observations, and reconstruction metrics. Therefore
  the earlier 30-episode obstacle-free composition archive is not directly
  comparable to the new publication protocol and must be regenerated before
  drawing updated composition conclusions.

## Closed-loop team-composition reconstruction milestone

- Files changed: `configs/multifidelity_composition.yaml`,
  `configs/multifidelity_composition_smoke.yaml`,
  `evaluation/multifidelity_composition_io.py`,
  `evaluation/run_multifidelity_composition.py`,
  `evaluation/evaluate_multifidelity_composition.py`,
  `evaluation/plot_multifidelity_composition.py`,
  `tests/test_multifidelity_composition_pipeline.py`,
  `docs/experimental_evaluation_plan.md`,
  `docs/mathematical_formulation.md`,
  `docs/latex/mathematical_formulation.tex`,
  `docs/architecture/implementation_composition_sweep.md`,
  `docs/assets/team_composition_reconstruction_milestone.png`, and this status
  file.
- The publication configuration fixes eight total robots and sweeps A8/G0,
  A6/G2, A4/G4, A2/G6, and A0/G8 over 30 paired episodes. The smoke
  configuration uses the smaller A2/G0, A1/G1, and A0/G2 sweep. Both endpoint
  teams use the same autoregressive MFGP: no single-fidelity replacement model
  is constructed.
- The runner uses equal LOW/HIGH sensing periods and equal observations per
  robot per event. It divides one fixed active retention budget in proportion
  to each composition, uses a positive unused placeholder cap for the absent
  fidelity at homogeneous endpoints, and rejects uncertainty-based observation
  admission or online hyperparameter fitting in this controlled study.
- The raw archive saves paired truth, integration geometry, NaN-padded team
  state histories, posterior mean/variance/density histories, submitted
  LOW/HIGH counts, retained counts, and phase timings. It validates that the
  hidden HIGH field is exactly identical across compositions for each seed.
- The offline evaluator computes final and trapezoidal time-average HIGH-field
  NRMSE and normalized-density KL divergence, plus empirical 95% calibration
  and sample/timing diagnostics. It imports no controllers and never reruns the
  simulation. The standalone plotter reads only the evaluated archive.
- Default commands:
  `.venv/bin/python evaluation/run_multifidelity_composition.py`, then
  `.venv/bin/python evaluation/evaluate_multifidelity_composition.py`, then
  `MPLCONFIGDIR=/tmp/ral_marta_mpl .venv/bin/python
  evaluation/plot_multifidelity_composition.py`.
- Visual regeneration command used for this milestone:
  `.venv/bin/python evaluation/run_multifidelity_composition.py --config
  configs/multifidelity_composition.yaml --output
  /tmp/multifidelity_composition_milestone_raw.npz --episodes 1 --num-steps 20
  --no-progress`, followed by the evaluator and
  `MPLCONFIGDIR=/tmp/mplconfig .venv/bin/python
  evaluation/plot_multifidelity_composition.py --input
  /tmp/multifidelity_composition_milestone_evaluated.npz --output
  docs/assets/team_composition_reconstruction_milestone.png
  --bootstrap-samples 200`.
- The visual is a deterministic one-episode validation artifact, not a
  publication result. Panels A/B show NRMSE and KL histories; panels C/D show
  final and time-average values against aerial fraction. It was inspected at
  original resolution: titles, axes, legends, metric directions, composition
  labels, and markers are readable and unclipped.
- Deterministic visual-run diagnostics, in A8/G0 through A0/G8 order: final
  NRMSE was `0.16043514`, `0.13319560`, `0.13192861`, `0.17107781`, and
  `0.20359170`; final KL was `0.81166223`, `1.13178293`, `1.91858124`,
  `2.23710402`, and `5.15016917`. Each composition submitted exactly 320 total
  observations, shifting from all LOW to all HIGH. Final retained totals were
  235, 236, 237, 235, and 238; these small differences arise from the existing
  fidelity-specific spatial-separation rules rather than different caps.
- Tests were added before the implementation. The initial focused run failed
  during import because the new modules did not exist. After implementation,
  the dedicated pipeline passed `12/12`; the composition plus estimator,
  scheduler, and GP selection passed `44/44`. Ruff, Ruff formatting, and Python
  compilation passed for every new Python file.
- The full suite completed with `209` passes and four known unrelated failures:
  the canonical config test still expects 300 rather than the current 200
  steps; the final-state plot test expects the former orange rather than the
  current cyan ground trajectory; the ablation/canonical protocol comparison
  sees the existing 90/360-degree ground-FOV mismatch; and the CLI progress
  test expects a `posterior_version=` field that the current formatter omits.
  The first broader selection similarly passed 66 tests and exposed the first
  three of those failures. None of the failing files or behaviors was changed
  by this milestone.
- The mathematical formulation now documents the exact discrete NRMSE, KL,
  calibration, held-posterior time average, and proportional retention-cap
  equations. `latexmk -pdf -interaction=nonstopmode -halt-on-error
  mathematical_formulation.tex` completed successfully in two passes; only
  pre-existing table underfull/overfull warnings remain.
- Architecture comparison: the new runner is downstream experiment
  infrastructure. It constructs the existing `CoupledSimulation`, which still
  owns one central estimator. No GP, controller, sensor, scheduler, motion,
  networking, concurrency, or target-architecture behavior changed, and there
  is no approved-target deviation.
- Numerical and behavioral regressions: none attributable to this milestone.
  The smoke numbers above validate data flow only and must not be interpreted
  as evidence that a particular composition is optimal. The next discrepancy
  ablation remains useful, but its mixed composition will be selected and
  documented later rather than assumed to be A4/G4.

## Baseline-comparison metric output-plot milestone

- Files changed: `evaluation/plot_baseline_comparison_metrics.py`,
  `evaluation/evaluate_baseline_comparison.py`,
  `tests/test_plot_baseline_comparison_metrics.py`,
  `examples/plot_baseline_comparison_output_milestone.py`,
  `docs/assets/baseline_comparison_output_plot_milestone.png`,
  `docs/architecture/implementation_baseline_comparison_runner.md`,
  `.gitignore`, and this status file.
- Running `PYTHONPATH=. .venv/bin/python
  evaluation/evaluate_baseline_comparison.py` now writes both
  `output/baseline_comparison/evaluated.npz` and
  `output/baseline_comparison/coverage_metrics.png`. The image is at the common
  comparison root beside the `multifidelity/` and `egerstedt/` directories that
  contain the separate trajectory plots.
- `--plot-output PATH` selects another PNG destination and `--no-plot` retains
  archive-only evaluation. An existing archive can be plotted without
  recomputation using `PYTHONPATH=. .venv/bin/python
  evaluation/plot_baseline_comparison_metrics.py`.
- Panel A shows mean footprint-normalized coverage histories and labels the
  higher-is-better direction. Panel B shows raw visible hidden-truth mass with
  each method's equal-area oracle. Panel C places every episode by its
  time-average and final coverage, with a diamond for the mean. Panel D shows
  complete method-step runtime distributions and labels lower as better. For
  repeated episodes, the history plots add the episode 2.5--97.5 percentile
  envelope.
- Visual regeneration command: `MPLCONFIGDIR=/tmp/ral_marta_mpl PYTHONPATH=.
  .venv/bin/python
  examples/plot_baseline_comparison_output_milestone.py`. It uses one real,
  fixed-seed, eight-step paired smoke run. The resulting image was visually
  inspected at original resolution: all four panels, legends, method labels,
  direction annotations, axes, and oracle lines are readable and unclipped.
  Coverage histories are deterministic for the fixed inputs; the honest
  wall-clock runtime panel can vary with machine load.
- Deterministic coverage results for scenario fingerprint
  `6849ca260e9c11200963c37267a9faced3222f280c7713d3175a82f569f1729c`:
  proposed final/time-average coverage was
  `0.150183157774`/`0.243146327962`, and Egerstedt final/time-average coverage
  was `0.470002753942`/`0.451425010575`.
- Architecture comparison: only the existing offline evaluation branch gained
  an archive-to-figure node. The approved central-estimator architecture is
  unchanged and there is no target deviation.
- Commands executed: the focused plot/evaluator tests; the combined comparison
  runner, evaluator, trajectory-plot, metric-plot, and ablation regression
  selection; changed-file Ruff; Python compilation; `git diff --check`; the
  full Pytest suite; and the fixed-seed visual regeneration command.
- Validation results: the focused plotting/evaluator selection passed `7/7`.
  The broader comparison/ablation selection passed `24` tests with one known
  unrelated configuration-consistency failure. The full suite passed `197`
  tests and retained four unrelated failures: stale expectations of `300`
  rather than `200` canonical steps, the former orange ground-trajectory
  color, missing `posterior_version=` progress text, and the dirty-tree
  protocol mismatch between canonical ground FOV `360` and ablation FOV `90`.
  Ruff, compilation, and diff checks passed.
- Mathematical documentation: no equations or metric definitions changed, so
  `docs/mathematical_formulation.md` and its LaTeX source remain current.
- Numerical and behavioral regressions: none. Plotting reads saved arrays only;
  it does not alter the evaluated values, estimator, controllers, trajectories,
  or baseline dynamics.

## Paired baseline-comparison metric evaluator milestone

- Files changed: `evaluation/comparison_io.py`,
  `evaluation/evaluate_baseline_comparison.py`,
  `tests/test_evaluate_baseline_comparison.py`,
  `examples/plot_baseline_comparison_metrics_milestone.py`,
  `docs/assets/baseline_comparison_metrics_milestone.png`,
  `docs/architecture/implementation_baseline_comparison_runner.md`,
  `docs/mathematical_formulation.md`,
  `docs/latex/mathematical_formulation.tex`, and this status file.
- The evaluator verifies both recorded and recomputed scenario fingerprints and
  exact paired seeds, times, query points, weights, truth densities, and free
  masks before computing a metric. It never reruns or imports either
  controller.
- For each saved state it computes raw hidden-truth probability mass in the
  union of the actual ground footprints and the existing footprint-normalized
  coverage effectiveness. The proposed method uses saved headings and its
  configured sector; the Egerstedt point-robot baseline uses its declared
  omnidirectional disk. Each method receives a separate equal-area oracle so a
  larger footprint is not silently assigned the same denominator.
- The evaluated archive stores complete coverage series, equal-area oracle
  mass, area budgets, final and trapezoidal time-average coverage, raw total
  step runtimes, and per-episode mean/median/p95/maximum runtime. Its metadata
  records `coverage: true` and `runtime: true` under `comparable_metrics`, while
  KL, NRMSE, and calibration remain false because the baseline publishes no
  posterior.
- Default evaluation command after generating both raw archives:
  `PYTHONPATH=. .venv/bin/python
  evaluation/evaluate_baseline_comparison.py`. The default output is
  `output/baseline_comparison/evaluated.npz`. The later output-plot milestone
  also makes this command write `output/baseline_comparison/coverage_metrics.png`.
- Visual regeneration command: `MPLCONFIGDIR=/tmp/ral_marta_mpl PYTHONPATH=.
  .venv/bin/python
  examples/plot_baseline_comparison_metrics_milestone.py`. The left panel shows
  the common footprint-normalized coverage series. The right panel shows raw
  visible hidden-truth mass against each method's fixed equal-area oracle.
  Both panels use one real, fixed-seed, eight-step paired run.
- Deterministic visual results for scenario fingerprint
  `6849ca260e9c11200963c37267a9faced3222f280c7713d3175a82f569f1729c`:
  proposed final/time-average coverage was
  `0.150183157774`/`0.243146327962` with oracle mass `0.217767014364`;
  Egerstedt final/time-average coverage was
  `0.470002753942`/`0.451425010575` with oracle mass `0.605807901228`.
  The figure was visually inspected at original resolution; titles, legends,
  axes, endpoints, oracle lines, and scales are readable without clipping.
- Commands executed: changed-file Ruff, Python compilation, `git diff --check`,
  focused evaluator tests, comparison/ablation regression tests, the full
  Pytest suite, the visual regeneration command, and a two-pass `latexmk -pdf
  -interaction=nonstopmode -halt-on-error mathematical_formulation.tex` build.
- Validation results: the new evaluator tests passed `3/3`; the combined
  evaluator/comparison/ablation selection passed `21` tests with one unrelated
  configuration-consistency failure; the final full suite passed `195` tests with
  four unrelated failures. Those failures are stale expectations of `300`
  rather than `200` canonical steps, the former orange ground-trajectory
  color, missing `posterior_version=` progress text, and a dirty-tree protocol
  mismatch between canonical ground FOV `360` and ablation FOV `90`. The LaTeX
  build passed with only existing table line-width warnings.
- Numerical and behavioral regressions: none in production or baseline code.
  The evaluated numbers describe the current saved trajectories, including the
  previously identified range-clipped aerial-cell behavior in the Egerstedt
  implementation; this milestone evaluates that behavior but does not claim
  to correct or validate its paper fidelity.

## Completed Work

- Implemented the zero-mean autoregressive model
  `f_H(q) = rho * f_L(q) + delta(q)` with independent isotropic RBF kernels.
- Implemented the complete LOW/HIGH joint covariance, separate per-observation
  noise variances, deterministic adaptive jitter, Cholesky factorization, and
  triangular solves without explicit matrix inversion.
- Added latent high-fidelity posterior mean and marginal-variance prediction.
- Added input validation, empty/single-fidelity behavior, immutable prediction
  results, and transactional preservation of the previous fitted state.
- Added deterministic contract and numerical tests using a broad LOW field and
  a narrow discrepancy field.
- Added a standalone deterministic visualization of the same synthetic field,
  showing truth components, posterior means, reconstruction error, samples, and
  posterior uncertainty.
- Recorded target-architecture approval before production changes and added a
  separate implementation diagram for the affected mathematical component.
- Added immutable timestamped LOW/HIGH observations with deterministic
  collection-time/insertion-order handling.
- Added separate transactional bounded buffers with newer duplicate
  replacement, spatial separation, maximum counts, and chronological output.
- Initially added stable softplus with masking and weighted normalization; the
  softplus transform was later superseded and removed by the user-approved
  positive-part refinement described below.
- Added one central estimator that buffers submissions, updates only on explicit
  request, publishes immutable versioned snapshots, and preserves the last
  valid state and pending data after failure.
- Added a deterministic asynchronous-event visualization driven by the real
  estimator implementation.
- Added simulator-only smoothed LOW/HIGH/discrepancy truth fields and
  configurable scalar-field sensors using dedicated RNGs.
- Added drift-free simulated-time events and one coordinator that owns both
  sensors and the central estimator.
- Added a reusable phased coupled loop preserving LOW-before-aerial and
  HIGH/update-before-ground ordering while leaving both controller laws and
  density inputs unchanged.
- Added legacy/multifidelity configuration modes, new CLI/evaluation entry
  points, production multifidelity configuration, and small smoke configs.
- Added deterministic sensor, scheduler, concrete-builder, legacy HEDAC, and
  ground GP/fusion/IPOPT smoke validation.
- Added a backward-compatible HEDAC external-target interface. Multifidelity
  calls skip the internal legacy aerial GP; existing callers retain its original
  collection/update behavior by default.
- Added configurable normalized-standard-deviation target construction,
  structured resampling onto the HEDAC map, immutable version/grid-aware target
  caching, and strict target validation.
- Connected the cached multifidelity target to the existing HEDAC heat/gradient
  law without changing controller frequency or robot dynamics.
- Demonstrated the required aerial causal chain from one ground HIGH
  observation through posterior and target changes to a changed real
  HEDAC-plus-Dubins trajectory.
- Added exact cached high-posterior density lookup on the existing MPC query
  grid, with validated interpolation/renormalization for alternate grids.
- Added pure validated construction of per-robot `density × Voronoi mask`
  vectors, without changing partitioning or applying the mask twice.
- Connected the normalized posterior density to the existing CasADi ground MPC
  while preserving its objective, dynamics, constraints, horizon, solver
  settings, warm start, and update frequency.
- Preserved the original two-GP post-hoc fusion as the no-posterior and legacy
  fallback, and explicitly kept posterior variance out of the ground objective.
- Demonstrated that a local HIGH correction changes ground weights, lowers the
  coverage cost at the corrected location, and changes a real MPC trajectory.
- Added an opt-in, headless final-state renderer driven only by the immutable
  coupled result and final posterior snapshot.
- Added configurable final-plot enable/path settings, a `--no-plot` override,
  missing-posterior reporting, and final saved-path logging to the coupled CLI.
- Generated a deterministic real coupled-run visual containing the posterior
  mean, posterior standard deviation, and both robot classes' trajectories.
- Refined the final-state visual so both robot classes appear together in both
  lower panels, with one color per class and explicit final-state markers over
  the final GP estimate and simulator ground truth.
- Historically replaced the duplicated raw-mean top-left panel with the exact
  then-active softplus density. That artifact is preserved; the current final
  renderer now labels and plots the clipped positive-mean density.
- Added one coupled-configuration loader that recursively resolves shared
  settings plus explicit aerial/ground overrides without aliasing mappings.
- Consolidated production, smoke, and visual multifidelity settings into one
  self-contained YAML file per scenario and migrated both multifidelity CLIs to
  one `--config` argument.
- Removed experiment-specific names and split aerial/ground config arguments
  from active multifidelity entry points while leaving legacy scripts intact.
- Added deterministic bounded log-space marginal-likelihood fitting for LOW
  length/variance and discrepancy length/variance, using only Cholesky and
  triangular solves.
- Added scheduled empirical-Bayes settings to the central estimator, including
  minimum sample count, update interval, deterministic restarts, iteration
  budget, and per-parameter bounds.
- Made hyperparameter fitting part of the existing estimator transaction: a
  failed optimization/fit preserves the last valid GP, snapshot, buffers, and
  pending observations.
- Recorded fitted kernel values, whether fitting occurred, and optimization
  duration in each published posterior snapshot and exposed the final values in
  the CLI and final-state plot.
- Added deterministic fixed-versus-fitted numerical tests and a real-output
  visual diagnostic. In the synthetic validation case RMSE fell from
  `0.030626098141` to `0.002923881087`.
- Added a table reference for all 130 leaf parameters in
  `configs/multifidelity.yaml`, including units, current runtime scope,
  lower/higher behavior, and explicit warnings for inactive compatibility keys.
- Historically added selectable fixed-ray and uniform-sector sensor sampling;
  this selector was later removed by the user-approved uniform-sector-only
  refinement below.
- Made area-uniform sector sampling unconditional for both LOW and HIGH central
  sensors and removed the sampling parameter from the sensor constructor,
  coordinator mapping, and all multifidelity YAML files.
- Preserved independent aerial/ground FOV, range, sample-count, noise, and RNG
  configuration while eliminating the ray-producing branch.
- Added a persistent `AGENTS.md` rule requiring every future mathematical or
  numerical behavior change to update `docs/mathematical_formulation.md` in the
  same milestone.
- Added an opt-in post-step multi-fidelity video recorder with configurable
  frame interval, playback rate, and GIF/ffmpeg encoding. Rendering observes
  published posterior products and robot positions only; it has no feedback
  path into sensing, estimation, control, or dynamics.
- Updated the living mathematical reference and affected implementation diagram
  to describe the single active sampling law.
- Added and visually inspected a deterministic single-law diagnostic for sector
  support, radial area uniformity, and angular uniformity.
- Added backward-compatible ground-controller selection. An omitted value
  defaults to `mpc`; the canonical production configuration now explicitly
  selects `lloyd`.
- Implemented quadrature-weighted Voronoi centroids, zero-mass hold behavior,
  wrapped-heading unicycle tracking, configurable gains/tolerance/velocity
  caps, and transactional external-density validation.
- Verified that the real coupled builder routes the exact central posterior
  density into Lloyd without updating the legacy ground GP.
- Recorded the exact common-density, ground-only Voronoi, weighted Lloyd
  centroid, bounded unicycle, discrete execution, and existing MPC equations in
  the Lloyd implementation architecture note. The documentation explicitly
  identifies the MPC angular weighting and quadrature behavior as implemented.
- Compared softplus-driven Lloyd motion against a normalized positive-part GP
  mean using one real deterministic posterior and identical robot initial
  states, then replaced softplus in production after the clipped version moved
  robots into higher-interest regions.
- Removed the public `stable_softplus` utility and replaced it with exact
  `positive_part(values) = max(values, 0)`. Published estimator snapshots are
  asserted array-for-array against clipped-and-weighted-normalized `high_mean`.
- Preserved the estimator transaction for the zero-mass edge case: an
  all-nonpositive candidate fails normalization and cannot replace the last
  valid snapshot.
- Added one GitHub-renderable mathematical reference covering the complete
  coupled system, including a known-versus-learned GP table and an
  equation-to-code map.
- Recorded exact implementation qualifications for the active heat stencil,
  local cooling, obstacle handling, MPC FOV cost, MPC/agent discretization
  difference, and spatial coverage metric instead of silently replacing them
  with idealized equations.
- Linked the mathematical reference from the repository README.

## Files Changed

- `AGENTS.md` — persistent living-mathematics documentation rule.
- `src/models/sensors.py` — unconditional uniform-sector sampling; removed the
  selector, fixed-ray branch, and related public state.
- `src/simulation.py` — removed aerial/ground sampling-pattern config plumbing.
- `configs/multifidelity.yaml`, `configs/multifidelity_smoke.yaml`, and
  `configs/multifidelity_plot_smoke.yaml` — removed the obsolete parameter.
- `tests/test_multifidelity_sensors.py`, `tests/test_simulation_scheduling.py`,
  and `tests/test_coupled_config.py` — single-law behavior and removed-surface
  regression coverage.
- `examples/plot_uniform_sector_only_milestone.py` and
  `docs/assets/uniform_sector_only_milestone.png` — deterministic validation of
  the only production sampling distribution.
- `examples/plot_uniform_sector_sampling_milestone.py` — keeps the historical
  comparison reproducible by locally reconstructing the removed baseline.
- `docs/mathematical_formulation.md`,
  `docs/multifidelity_config_reference.md`, and
  `docs/architecture/implementation_uniform_sector_sampling.md` — updated math,
  parameter inventory, and implemented sampling flow.
- `docs/mathematical_formulation.md` — consolidated GitHub-renderable equations
  for the simulated fields, estimator, density transforms, controllers,
  dynamics, scheduling, metrics, and compatibility path.
- `README.md` — links the consolidated mathematical reference.
- `src/core/multifidelity_gp.py` — new pure mathematical GP module.
- `src/core/__init__.py` — exports the three new public mathematical types.
- `tests/conftest.py` — deterministic synthetic multi-fidelity fixture.
- `tests/test_multifidelity_gp_contract.py` — API, validation, empty-data, and
  transactional-state tests.
- `tests/test_multifidelity_gp.py` — independent numerical/reference tests.
- `examples/plot_multifidelity_gp_core.py` — standalone diagnostic plot script.
- `docs/assets/multifidelity_gp_core_demo.png` — generated GP-core comparison
  figure.
- `docs/architecture/target_multifidelity_architecture.md` — approval metadata
  only; target diagrams were not changed.
- `docs/architecture/implementation_gp_core.md` — milestone implementation view
  and target comparison.
- `docs/multifidelity_gp_status.md` — this milestone record.
- `src/core/observations.py` — observation schema, retention policy, and
  transactional buffers.
- `src/core/density.py` and `src/core/__init__.py` — removed softplus and expose
  exact positive-part clipping with existing density normalization, aerial
  target, and grid-resampling utilities.
- `src/core/multifidelity_estimator.py` — central explicit-update estimator,
  snapshots, settings, and update reports.
- `tests/test_observations.py` — observation validation and retention tests.
- `tests/test_density.py` — density and interpolation tests.
- `tests/test_multifidelity_estimator.py` — asynchronous estimator behavior and
  failure-transaction tests.
- `examples/plot_async_estimator_milestone.py` — reproducible event/posterior
  visualization script.
- `docs/assets/async_estimator_milestone.png` — generated milestone summary.
- `docs/architecture/implementation_async_estimator.md` — affected
  implementation diagrams and approved-target comparison.
- `src/models/sensors.py` — fidelity-field construction and dedicated-RNG
  LOW/HIGH sensors.
- `src/models/__init__.py` — exports the new simulator sensor types.
- `src/simulation.py` — mode parser, periodic events, coordinator, and reviewed
  configuration mapping.
- `src/coupled_simulation.py` — reusable phased loop and existing-controller
  setup/adapters.
- `src/coupled_config.py` — validated single-file YAML loading and deep
  resolution into the existing aerial and ground parameter views.
- `examples/run_multifidelity.py` — coupled-simulation CLI with immediate
  startup output, configurable live step summaries, and an explicit warning
  when the unsupported video setting is enabled.
- `tests/test_run_multifidelity_cli.py` — progress cadence, final-step, disabled
  logging, and argument-validation tests.
- `evaluation/run_multifidelity_evaluation.py` — thin repeated-episode CLI.
- `configs/iros26_aerial.yaml` — explicit legacy mode only; multifidelity
  settings now live exclusively in self-contained multifidelity configs.
- `configs/multifidelity.yaml` — self-contained production configuration.
- `configs/multifidelity_smoke.yaml` — self-contained fast CLI validation
  configuration; production defaults are not reduced.
- `tests/test_multifidelity_sensors.py` — fidelity truth, sensor metadata,
  geometry, determinism, and global-RNG isolation.
- `tests/test_simulation_scheduling.py` — event schedules, cached posterior,
  controller progress, single ownership, and deterministic coordinator tests.
- `tests/test_legacy_smoke.py` — mode fallback, real HEDAC baseline, legacy
  branch, and concrete multifidelity-builder smoke tests.
- `examples/plot_shadow_simulation_milestone.py` — reproducible shadow-loop
  visualization.
- `docs/assets/shadow_simulation_milestone.png` — generated milestone summary.
- `docs/architecture/implementation_shadow_simulation.md` — affected
  implementation diagrams and target comparison.
- `src/core/hedac.py` — optional validated external goal density and compatible
  `update_legacy_gp` control.
- `src/core/density.py` — corrected the mixed target to use spatially normalized
  posterior standard deviation.
- `src/simulation.py` — configurable, validated, version/grid-aware aerial
  target projection and cache.
- `src/coupled_simulation.py` — multifidelity target lookup/injection and
  density-source reporting; its Milestone 6 ground path was legacy and is
  superseded by the Milestone 7 entry below.
- `configs/multifidelity_smoke.yaml` — explicit aerial-target weights.
- `tests/test_aerial_multifidelity_feedback.py` — external-target validation,
  legacy signature, HIGH-to-target causality, cache, and real HEDAC/Dubins
  response tests.
- `tests/test_density.py`, `tests/test_simulation_scheduling.py`, and
  `tests/test_legacy_smoke.py` — normalized-std formula and updated feedback
  integration assertions.
- `examples/plot_aerial_feedback_milestone.py` — deterministic real-estimator,
  real-HEDAC, real-Dubins milestone visualization.
- `docs/assets/aerial_feedback_milestone.png` — generated feedback summary.
- `examples/plot_shadow_simulation_milestone.py` — isolated the frozen
  Milestone 5 sequence locally so its historical artifact remains reproducible
  after production feedback was enabled.
- `docs/architecture/implementation_aerial_feedback.md` — affected component,
  sequence, and approved-target comparison diagrams.
- `src/simulation.py` — exact/cached ground posterior-density lookup, consumer
  version metadata, and pure Voronoi weight-vector construction.
- `src/coupled_simulation.py` — optional external ground density at the existing
  adapter seam; legacy fusion remains the default/fallback branch.
- `tests/test_ground_multifidelity_feedback.py` — density identity/cache,
  validation transaction, Voronoi products, local correction, CasADi cost, and
  real MPC integration tests.
- `tests/test_simulation_scheduling.py` and `tests/test_legacy_smoke.py` — both
  controller sources and between-update ground-cache assertions.
- `examples/plot_ground_feedback_milestone.py` — deterministic posterior,
  weight, cost, and real-MPC response visualization.
- `docs/assets/ground_feedback_milestone.png` — generated Milestone 7 summary.
- `docs/architecture/implementation_ground_feedback.md` — affected flow,
  sequence, preserved-controller boundary, and approved-target comparison.
- `docs/architecture/implementation_aerial_feedback.md` — marked its legacy
  ground statement as superseded while preserving the Milestone 6 view.
- `examples/plot_final_multifidelity_state.py` — reusable headless final-state
  renderer and deterministic standalone artifact command.
- `examples/run_multifidelity.py` — configuration-controlled final rendering
  after simulation completion; `--no-plot` is the explicit override.
- `configs/multifidelity.yaml` — enables the final plot and configures
  `output/multifidelity_final_state.png` for the canonical coupled run.
- `configs/default_params.yaml` and `configs/multifidelity_smoke.yaml` —
  document the final-plot setting with a disabled backward-compatible default.
- `configs/multifidelity_plot_smoke.yaml` — self-contained one-aerial,
  one-ground deterministic visual validation configuration.
- `tests/test_final_multifidelity_plot.py` — PNG output, configuration enable,
  CLI disable, missing-posterior, combined-overlay color, and final-state label
  tests.
- `docs/assets/final_multifidelity_state_milestone.png` — deterministic real-run
  final-output artifact.
- `docs/architecture/implementation_final_state_output.md` — affected output
  flow, sequence, isolation boundary, and approved-target comparison.
- `tests/test_coupled_config.py` — canonical resolved values, recursive merge,
  and invalid-schema validation.
- `examples/plot_unified_config_milestone.py` and
  `docs/assets/unified_config_milestone.png` — deterministic loader-derived
  schema and resolved-value summary.
- `docs/architecture/implementation_unified_configuration.md` — unified config
  ownership, resolution sequence, and approved-target comparison.
- `tests/test_hyperparameter_fitting.py` — likelihood improvement, prediction
  improvement, determinism, bounds, and validation tests.
- `examples/plot_hyperparameter_fitting_milestone.py` and
  `docs/assets/hyperparameter_fitting_milestone.png` — reproducible
  fixed-versus-fitted GP diagnostic.
- `docs/architecture/implementation_hyperparameter_fitting.md` — scheduled fit
  flow, transaction sequence, and explicit target comparison.
- `docs/multifidelity_config_reference.md` — exhaustive canonical coupled-config
  parameter and tuning reference.
- `README.md` — link to the canonical multi-fidelity configuration reference.
- `src/models/sensors.py` — historically introduced deterministic
  uniform-sector sampling alongside a fixed-ray mode; the selector and
  fixed-ray production branch were removed in the later single-law refinement.
- `src/simulation.py` — historically mapped independent aerial/ground sampling
  patterns; that mapping was removed with the selector.
- `tests/test_multifidelity_sensors.py`, `tests/test_simulation_scheduling.py`,
  and `tests/test_coupled_config.py` — geometry, area distribution,
  determinism, validation, and configuration-routing coverage.
- `examples/plot_uniform_sector_sampling_milestone.py` and
  `docs/assets/uniform_sector_sampling_milestone.png` — deterministic
  fixed-ray versus uniform-sector diagnostic.
- `docs/architecture/implementation_uniform_sector_sampling.md` — sampling
  flow and approved-target comparison.
- `src/coupled_simulation.py` — selectable Lloyd construction, shared legacy
  density calculation, weighted-centroid helper, and bounded unicycle law.
- `tests/test_ground_lloyd_controller.py` — centroid quadrature, empty mass,
  bounded motion, invalid-density transaction, configuration, and real coupled
  density-routing tests.
- `configs/multifidelity.yaml`, `configs/multifidelity_smoke.yaml`, and
  `configs/multifidelity_plot_smoke.yaml` — explicit backward-compatible
  controller selection; the canonical file also documents Lloyd tuning values.
- `examples/plot_lloyd_controller_milestone.py` and
  `docs/assets/lloyd_controller_milestone.png` — deterministic real-controller
  MPC/Lloyd comparison.
- `docs/architecture/implementation_lloyd_ground_controller.md` — affected
  flow, sequence, and intentional target-extension record.
- `examples/plot_ground_mean_density_comparison.py` and
  `docs/assets/ground_mean_density_comparison.png` — reproducible real-posterior
  previous-softplus versus current-clipped-mean Lloyd diagnostic.
- `examples/plot_final_multifidelity_state.py` and
  `docs/assets/final_multifidelity_state_positive_mean.png` — current clipped
  density label and preserved new real-run final-state artifact; the earlier
  softplus final-state image remains unchanged for historical comparison.
- `tests/test_density.py`, `tests/test_multifidelity_estimator.py`, and
  `tests/test_coupled_config.py` — exact clipping/export, snapshot-density, and
  canonical Lloyd selection assertions.

## Commands Executed

- `git status --short`
- `uv lock --check` — not executed because `uv` is unavailable.
- `python -m pytest --collect-only -q` — not executed because the system has no
  `python` alias.
- `python3 -m pytest --collect-only -q` — failed because system Python lacks
  NumPy and pytest.
- `.venv/bin/python -c 'import numpy, scipy ...'` — verified NumPy 2.4.6 and
  SciPy 1.18.0 in the repository virtual environment.
- `.venv/bin/pip install pytest ruff` — sandboxed attempt failed due restricted
  network; the approved network-enabled retry installed pytest 9.1.1 and Ruff
  0.15.21 into `.venv` without changing dependency files.
- `.venv/bin/python -m py_compile src/core/multifidelity_gp.py`
- `.venv/bin/python -m pytest -q tests/test_multifidelity_gp_contract.py`
- `.venv/bin/python -m pytest -q tests/test_multifidelity_gp.py`
- `.venv/bin/python -m pytest -q`
- `.venv/bin/ruff check src/core/multifidelity_gp.py tests/conftest.py
  tests/test_multifidelity_gp_contract.py tests/test_multifidelity_gp.py`
- `.venv/bin/python -m examples.plot_multifidelity_gp_core`
- `git diff --check`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest
  tests/test_ground_lloyd_controller.py tests/test_ground_multifidelity_feedback.py
  tests/test_coupled_config.py -q`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.plot_lloyd_controller_milestone`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.plot_ground_mean_density_comparison`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest
  tests/test_ground_lloyd_controller.py -q`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m ruff check
  examples/plot_ground_mean_density_comparison.py`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q` on the density,
  estimator, scheduling, aerial/ground feedback, Lloyd, and final-renderer
  suites.
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.plot_final_multifidelity_state --output
  docs/assets/final_multifidelity_state_positive_mean.png`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q` — first full
  rerun: 167 passed, with one stale canonical-controller assertion and the
  previously known CLI progress-field mismatch failing. The canonical
  assertion was updated to the existing `lloyd` value before the final rerun.
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q` — final rerun:
  **168 passed, 1 failed in 5.10 s**; the sole failure is the previously known
  unrelated CLI progress-field mismatch.
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.run_multifidelity --config configs/multifidelity.yaml --num-steps 2
  --log-every 1 --no-plot`
- Final source/reference audit: no softplus reference remains in `src/` or
  `tests/`; all **130/130** canonical YAML leaf paths are documented; public
  `positive_part([-1, 0, 2])` returned `[0, 0, 2]`; `git diff --check` passed.
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q
  tests/test_hyperparameter_fitting.py tests/test_multifidelity_estimator.py
  tests/test_multifidelity_gp.py tests/test_multifidelity_gp_contract.py
  tests/test_simulation_scheduling.py tests/test_coupled_config.py
  tests/test_run_multifidelity_cli.py tests/test_final_multifidelity_plot.py`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m ruff check` on the GP,
  estimator, builder, CLI, plotting, and hyperparameter-integration test files.
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.plot_hyperparameter_fitting_milestone`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.plot_final_multifidelity_state --config
  configs/multifidelity_plot_smoke.yaml`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q
  tests/test_final_multifidelity_plot.py`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q` — completed
  with one unrelated CLI progress assertion failure because the current runner
  no longer emits `posterior_version`, while its existing test still requires
  that field; the new plot tests all passed.
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.run_multifidelity --config configs/multifidelity.yaml --num-steps 1
  --log-every 1 --no-plot`
- A YAML/Markdown coverage check flattened `configs/multifidelity.yaml` and
  compared it with the reference table: **124 leaves documented, zero missing,
  zero extra, zero duplicates**.
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q
  tests/test_coupled_config.py`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q
  tests/test_multifidelity_sensors.py tests/test_simulation_scheduling.py
  tests/test_coupled_config.py`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m ruff check` on the sensor,
  coordinator mapping, milestone plot, and affected tests.
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.plot_uniform_sector_sampling_milestone`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q
  tests/test_final_multifidelity_plot.py tests/test_run_multifidelity_cli.py`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m ruff check
  examples/plot_final_multifidelity_state.py examples/run_multifidelity.py
  tests/test_final_multifidelity_plot.py tests/test_run_multifidelity_cli.py`
- `.venv/bin/python -m py_compile examples/plot_final_multifidelity_state.py
  examples/run_multifidelity.py tests/test_final_multifidelity_plot.py`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.plot_final_multifidelity_state`
- `.venv/bin/python examples/run_multifidelity.py ... --num-steps 0 --no-plot`
  — failed at the repository's pre-existing direct-script import boundary
  (`src` is not on `sys.path`); canonical module execution below passed.
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.run_multifidelity --num-steps 0 --log-every 10`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.run_multifidelity --num-steps 1 --log-every 1`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m ruff check` on the final
  output production/example and test files, followed by `git diff --check`.
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q
  tests/test_coupled_config.py tests/test_ground_multifidelity_feedback.py
  tests/test_run_multifidelity_cli.py tests/test_final_multifidelity_plot.py`
- The first unified-config Ruff run found one stale `HEDACParams` import in the
  migrated ground-feedback test. The import was removed and the rerun passed.
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.run_multifidelity --config configs/multifidelity_smoke.yaml
  --num-steps 2 --log-every 1 --no-plot`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  evaluation.run_multifidelity_evaluation --config
  configs/multifidelity_smoke.yaml --episodes 1 --num-steps 2`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.plot_unified_config_milestone`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.plot_final_multifidelity_state --config
  configs/multifidelity_plot_smoke.yaml`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.run_multifidelity --config configs/multifidelity.yaml --num-steps 1
  --log-every 1`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q`
- Final focused Ruff and `git diff --check` after the combined-overlay change.
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.run_multifidelity --config configs/multifidelity.yaml --num-steps 1
  --log-every 1 --no-plot`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q`
- Final focused Ruff, `git diff --check`, CLI `--help`, and active
  multifidelity old-name/split-flag scan.
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q
  tests/test_final_multifidelity_plot.py tests/test_run_multifidelity_cli.py`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.plot_final_multifidelity_state --config
  configs/multifidelity_plot_smoke.yaml`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q
  tests/test_run_multifidelity_cli.py`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.run_multifidelity --config configs/multifidelity_smoke.yaml
  --num-steps 2 --log-every 1 --no-plot`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m ruff check
  examples/run_multifidelity.py tests/test_run_multifidelity_cli.py`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q`
- `git diff --check`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q
  tests/test_ground_multifidelity_feedback.py
  tests/test_aerial_multifidelity_feedback.py tests/test_legacy_smoke.py`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.plot_ground_feedback_milestone`
- The first plot diagnostic used a 9×9 controller grid, then a 17×17 grid, and
  showed that a global squared-distance diagnostic depends on discretization
  and evaluation position. The final test/figure evaluates cost at the local
  HIGH-correction position, where increased local importance has the explicit
  expected direction; no controller equation was changed during this visual
  iteration.
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.run_multifidelity --config configs/multifidelity_smoke.yaml
  --no-plot`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  evaluation.run_multifidelity_evaluation --config
  configs/multifidelity_smoke.yaml
  --episodes 1 --num-steps 2`
- `.venv/bin/python -m py_compile src/simulation.py
  src/coupled_simulation.py examples/plot_ground_feedback_milestone.py
  tests/test_ground_multifidelity_feedback.py`
- `.venv/bin/ruff check` on all Milestone 7 production, test, and plot files.
- `git diff --check`
- A deterministic diagnostic command measured broad recovery and correction
  locality. Its first direct-import attempt failed because pytest's `tests/`
  path was absent; the retry used `PYTHONPATH=tests` and succeeded.
- `.venv/bin/python -m py_compile src/core/observations.py src/core/density.py
  src/core/multifidelity_estimator.py`
- `.venv/bin/python -m pytest -q tests/test_observations.py
  tests/test_density.py tests/test_multifidelity_estimator.py`
- `.venv/bin/ruff check src/core/observations.py src/core/density.py
  src/core/multifidelity_estimator.py ...`
- `.venv/bin/python -m examples.plot_async_estimator_milestone`
- A deterministic diagnostic command extracted update versions/counts, local
  posterior changes, uncertainty changes, and density integrals from the visual
  event history.
- `.venv/bin/python -m pytest -q tests/test_multifidelity_sensors.py
  tests/test_simulation_scheduling.py tests/test_legacy_smoke.py`
- `.venv/bin/python -m examples.run_multifidelity --config
  configs/multifidelity_smoke.yaml --no-plot`
- `.venv/bin/python -m evaluation.run_multifidelity_evaluation --config
  configs/multifidelity_smoke.yaml
  --episodes 1 --num-steps 1`
- A one-robot diagnostic exercised the extracted legacy ground
  GP/fusion/IPOPT adapter. Its first attempt correctly stopped at the smoke
  configuration's 10-sample GP minimum; rerunning with the diagnostic minimum
  set to one completed successfully.
- `.venv/bin/python -m examples.plot_shadow_simulation_milestone`
- A deterministic diagnostic command extracted event counts, controller-step
  count, final version, posterior RMSE, density integral, and density-source
  metadata from the milestone visualization.
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q
  tests/test_aerial_multifidelity_feedback.py tests/test_legacy_smoke.py
  tests/test_simulation_scheduling.py tests/test_density.py`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.plot_aerial_feedback_milestone`
- `.venv/bin/python -m py_compile src/core/density.py src/core/hedac.py
  src/simulation.py src/coupled_simulation.py
  examples/plot_aerial_feedback_milestone.py`
- `.venv/bin/ruff check` on every Milestone 6 production, test, and plotting
  file, including the preserved historical shadow plot.
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.plot_shadow_simulation_milestone --output
  /tmp/shadow_simulation_milestone_check.png`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.run_multifidelity --config configs/multifidelity_smoke.yaml
  --no-plot`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  evaluation.run_multifidelity_evaluation --config
  configs/multifidelity_smoke.yaml
  --episodes 1 --num-steps 2`
- A one-ground-robot diagnostic initially stopped because the legacy ground
  uncertainty filter removed its only smoke sample. The rerun used the explicit
  diagnostic-only preconditions `min_samples=1` and `use_filter=False` and
  exercised two real CasADi MPC steps successfully.
- `git diff --check`

All pytest commands used `MPLCONFIGDIR=/tmp/matplotlib` to avoid writes to the
read-only user configuration directory.

Documentation-reference validation:

- `rg` audits across the GP, estimator, density, HEDAC, sensor, dynamics,
  Voronoi, Lloyd, MPC, scheduler, and metric implementations.
- A delimiter/link validation script for GitHub math syntax and local Markdown
  targets.
- `git diff --check`.

Uniform-sector-only milestone validation:

- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q
  tests/test_multifidelity_sensors.py tests/test_simulation_scheduling.py`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q
  tests/test_multifidelity_sensors.py tests/test_simulation_scheduling.py
  tests/test_coupled_config.py`
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m pytest -q`
- `.venv/bin/python -m ruff check` on the affected production, test, and plot
  files.
- `.venv/bin/python -m py_compile` on the affected production and plot files.
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.plot_uniform_sector_only_milestone`
- Regenerated the historical comparison with `MPLCONFIGDIR=/tmp/matplotlib
  .venv/bin/python -m examples.plot_uniform_sector_sampling_milestone`.
- `MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
  examples.run_multifidelity --config configs/multifidelity_smoke.yaml
  --no-plot`
- Flattened canonical-YAML versus Markdown parameter-coverage validation.
- GitHub math delimiter/link validation and `git diff --check`.

## Test Results

- Uniform-sector sensor and scheduler suite: **18 passed in 1.09 s**.
- Focused suite including canonical config: **22 passed, 1 unrelated failure**;
  the current YAML selects `mpc` while its pre-existing assertion expects
  `lloyd`.
- Accumulated repository suite: **167 passed, 2 unrelated failures in 4.92 s**.
  The failures are the same stale controller assertion and the already-known
  CLI progress assertion requiring `posterior_version=`.
- Ruff, compilation, canonical config-reference coverage, smoke simulation, and
  `git diff --check`: **passed**.
- Canonical config reference: **129/129 leaf paths documented**, with zero
  missing or extra paths after removing the sampling parameter.
- Uniform-sector-only visual: **passed and visually inspected**. For 4096
  samples, mean normalized squared radius was `0.502940966904` versus the ideal
  `0.5`; radial/angular CDF distances were `0.009694316025` and
  `0.012356479278`.
- Historical fixed-ray comparison regeneration: **passed** with its original
  numerical outputs preserved.
- Mathematical-reference validation: **passed**; display-math delimiters are
  balanced, no unsupported bracket delimiters are used, and every relative
  Markdown link resolves.
- Runtime tests were not rerun because this milestone changes documentation
  only.
- Contract suite: **28 passed in 1.37 s**.
- Numerical suite: **11 passed in 1.02 s**.
- Full repository suite after adding the visualization: **39 passed in 1.21 s**.
- Ruff: **all checks passed**.
- Python compilation and `git diff --check`: **passed**.
- Plot generation: **passed**; the output is a 2321 x 1595 RGBA PNG and was
  visually inspected for labels, clipping, and consistency with the test case.
- Existing tests before this milestone: none were collected; all 39 tests are
  new milestone tests.
- Observation/density/estimator milestone suite: **52 passed in 1.07 s**.
- Accumulated repository suite: **91 passed in 1.32 s**.
- New production modules, tests, exports, and visualization script: **Ruff
  passed**.
- Async-estimator plot generation: **passed**; the 2321 x 1595 RGBA PNG was
  visually inspected for correct event labels, axes, and posterior consistency.
- Sensor/scheduling/legacy milestone suite: **19 passed in 1.24 s**.
- Accumulated repository suite: **110 passed in 1.72 s**.
- New production modules, tests, entry points, and visualization: **Ruff
  passed**.
- Coupled and evaluation CLI smoke commands: **passed**.
- One-ground-robot legacy GP/fusion/IPOPT diagnostic: **passed** after applying
  the explicitly reported one-sample diagnostic precondition.
- Shadow-simulation plot: **passed**; the 2319 x 1596 RGBA PNG was visually
  inspected for event alignment, labels, trajectories, and controller-source
  interpretation.
- Aerial-feedback focused suite: **35 passed in 1.62 s**.
- Accumulated repository suite: **117 passed in 1.49 s** on the final rerun
  (an earlier validation run completed in 1.72 s).
- Milestone production/test/plot compilation and Ruff checks: **passed**.
- Coupled and evaluation CLI feedback smoke: **passed**; final sources were
  `multifidelity_high_posterior` and `legacy_posthoc_fusion`.
- One-ground-robot real CasADi MPC diagnostic: **passed** with the documented
  diagnostic-only legacy-GP preconditions; two retained ground samples and one
  trajectory were produced.
- Hyperparameter/integration focused suite: **82 passed in 3.39 s**.
- Final accumulated repository suite: **155 passed in 4.44 s**.
- Hyperparameter plot: **passed and visually inspected**; fixed RMSE
  `0.030626098141`, fitted RMSE `0.002923881087`, and likelihood objective
  `18.779067613934 -> -51.943408691538`.
- Production-config one-step smoke: **passed in 7.8 s wall time** and performed
  fitting on its first posterior. Final kernel tuple was
  `(8.17735393128, 0.100245372298, 2.55342139784, 0.01)`; the discrepancy
  variance reached its configured lower bound, which is reported rather than
  hidden.
- Configuration-reference validation: **5 coupled-config tests passed in 0.68
  s**; exhaustive path coverage check passed for all 124 YAML leaves.
- Uniform-sector focused suite: **23 passed in 1.13 s**. The deterministic
  4,096-sample test measured mean squared normalized radius within `0.02` of
  the area-uniform expectation `0.5` and found more than 4,000 distinct
  bearings.
- Uniform-sector full repository check: **158 passed, 1 failed in 4.71 s**. The
  sole failure is the previously reported unrelated CLI progress-field mismatch;
  all new and affected tests passed. Ruff and `git diff --check` passed.
- Canonical configuration reference audit at that milestone: **130/130 paths
  and values matched**, including the then-present ground sampling parameter.
- Aerial-feedback plot: **passed**; the white-background four-panel PNG was
  visually inspected at original resolution for orientation, labels, clipping,
  HIGH-observation location, target change, and separated Dubins trajectories.
- Historical shadow-plot regeneration to `/tmp`: **passed** without overwriting
  the preserved Milestone 5 artifact.
- New ground-feedback suite: **12 passed in 1.96 s** on the final focused
  rerun, including exact legacy-fusion and invalid-input transaction checks.
- Ground/aerial/legacy focused suite: **24 tests passed** across the final
  constituent suites.
- Accumulated repository suite: **129 passed in 2.34 s**.
- Milestone compilation, Ruff, and `git diff --check`: **passed**.
- Coupled and evaluation CLI smoke: **passed**; both controller sources were
  `multifidelity_high_posterior` at final version 1.
- Real one-ground-robot integration: **passed** without legacy-GP diagnostic
  preconditions because the central posterior was available before ground MPC.
- Ground-feedback plot: **passed** and visually inspected at original resolution
  for common density scales, orientation, HIGH location, difference sign,
  trajectory visibility, labels, and clipping.
- CLI progress tests: **5 passed in 1.06 s**.
- Live coupled CLI smoke: **passed**; startup output appeared before stepping and
  summaries appeared after both requested steps.
- Accumulated repository suite after the CLI usability change: **134 passed in
  2.22 s**.
- CLI Ruff and `git diff --check`: **passed**.
- Final-renderer and CLI-output focused suite: **10 passed in 2.75 s**.
- Final-state standalone plot: **passed** with posterior version 6, 12 LOW
  samples, 11 HIGH samples, one aerial trajectory, and one ground trajectory;
  the PNG was visually inspected at original resolution for panel labels,
  coordinate orientation, start/end markers, colorbars, clipping, and readable
  trajectory overlays.
- Revised ground-driver panel suite: **7 passed in 2.89 s**, including an exact
  array-level assertion that the top-left panel source equals
  `PosteriorSnapshot.density` rather than `high_mean`. The regenerated PNG was
  visually inspected and shows the limited contrast introduced by softplus.
- Full repository check after the plot change: **155 passed, 1 failed**. The
  failure is `test_progress_is_emitted_at_interval_and_final_step`; current
  `run_with_progress()` output omits the pre-existing test's required
  `posterior_version` field. This plot-only change did not modify that runner or
  its test, so the mismatch is reported rather than silently expanded in scope.
- Canonical production-config CLI integration: **passed** for one complete
  coupled step with 3 aerial and 7 ground robots; it published posterior version
  1 and saved `output/multifidelity_final_state.png`. The resulting production
  PNG was visually inspected for all four panels and all robot markers.
- Accumulated repository suite after final-state output integration: **139
  passed in 4.57 s**.
- Final focused Ruff and `git diff --check`: **passed**.
- Unified-config focused suite: **27 passed in 3.45 s** before the final full
  run; the only initial Ruff issue was the removed stale test import.
- Unified smoke CLI and repeated-evaluation CLI: **passed** from the same YAML;
  the two-step smoke retained the exact pre-consolidation metric
  `0.538776926355` and posterior version 1.
- Canonical `configs/multifidelity.yaml` production smoke: **passed** with 3
  aerial and 7 ground robots; its one-step metric remained
  `0.109648281234`, posterior version 1, and the expected controller sources.
- Unified configuration plot and regenerated final-state plot: **passed** and
  visually inspected at original resolution for hierarchy arrows, resolved
  values, labels, clipping, and trajectory/predictive-field consistency.
- Accumulated repository suite after consolidation: **144 passed in 3.47 s**
  on the final rerun (the earlier validation run completed in 4.56 s).
- Final active multifidelity scan: **no experiment-specific names or split
  aerial/ground CLI flags found**. Focused Ruff and `git diff --check` passed.
- Revised final-state plot suite: **11 passed in 2.82 s**; the new assertion
  verifies a single shared color for every aerial trajectory, a different
  shared color for every ground trajectory, and both final-state legend labels.
- Revised deterministic artifact: **passed** and visually inspected at original
  resolution. Both lower panels show both teams and their final states; the
  left background is the final GP mean and the right background is the exact
  simulator high-fidelity truth on a common color scale.
- Production-config final rendering: **passed** with all 3 aerial and 7 ground
  final states visible in both lower panels and the saved-path message emitted.
- Accumulated repository suite after the plot refinement: **145 passed in 3.76
  s**. Focused Ruff and `git diff --check` passed.
- Lloyd/controller/config focused suite: **26 passed in 1.56 s**.
- Lloyd comparison plot: **passed and visually inspected**; labels, paths,
  start/final/centroid markers, color scales, and residual curves are readable
  and unclipped.
- Positive-mean diagnostic: **passed and visually inspected**; the four-panel
  PNG uses real posterior version 6 and identical three-robot Lloyd inputs.
  Lloyd regression suite: **9 passed in 1.43 s**. Ruff and `git diff --check`
  passed.
- Clipped-density focused integration suite: **74 passed in 4.11 s**. The
  density/estimator subset passed **31 tests in 1.35 s**, including exact
  snapshot equality to normalized `max(high_mean, 0)`.
- Current clipped-density comparison and final-state artifacts: **passed and
  visually inspected** for density zeros/contrast, trajectory visibility,
  labels, colorbars, clipping, and consistency with printed metrics.
- Final repository suite: **168 passed, 1 failed in 5.10 s**. The only failure
  remains `test_progress_is_emitted_at_interval_and_final_step`, whose expected
  `posterior_version` logging field is absent from the current CLI runner. No
  density, estimator, controller, configuration, or visualization test failed.
- Canonical two-step Lloyd/multifidelity smoke: **passed**, publishing posterior
  version 1 with both aerial and ground density sources reported as
  `multifidelity_high_posterior`; final ergodic metric was
  `0.0916217077134`.

No milestone-related failure was bypassed. The unrelated CLI progress mismatch
is explicitly retained in the full-suite result above rather than silently
modified outside this density-transform scope.

## Numerical Validation

![Deterministic selectable-ground-controller comparison](assets/lloyd_controller_milestone.png)

Panels A and B run the real MPC and Lloyd implementations from identical
initial states for 35 steps over the same fixed two-peak density. Circles mark
initial positions, stars final positions, crosses current density-weighted cell
centroids, and dotted lines the final centroid residuals. Panel C evaluates
both laws with the same mean distance-to-current-centroid diagnostic. MPC
reduced it from `6.172901892548` to `2.786604117199`; Lloyd reduced it to
`1.142271534588`. Both runs retained zero legacy GP samples, proving the plotted
response came only from the supplied common density. Regenerate with:
`MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
examples.plot_lloyd_controller_milestone`.

![Real-posterior softplus versus positive-mean diagnostic](assets/ground_mean_density_comparison.png)

Panel A shows current real posterior version 6, including nine negative cells.
Panel B reconstructs the removed softplus transform locally for historical
comparison; its maximum is only `1.322985846907` times its minimum. Panel C is
the exact production snapshot density, `normalize(max(mean, 0))`. Panel D
evaluates mean GP interest at the three robot positions. After 60 identical
Lloyd steps, that value is `0.112853621199` for the previous transform and
`0.191920027698` for current clipping. Mean distance to the posterior peak
falls from `2.562081680096` to `2.138516512185`. Regenerate with:
`MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
examples.plot_ground_mean_density_comparison`.

![Deterministic multi-fidelity GP test visualization](assets/multifidelity_gp_core_demo.png)

The panels show the truth decomposition, LOW-only and joint posterior means,
absolute HIGH-field reconstruction error, and posterior standard deviation.
The three HIGH samples recover the narrow discrepancy that the broad LOW data
cannot identify by itself.

- Dense independent joint-covariance reference agrees with the Cholesky result
  at relative tolerance `1e-11` for mixed-fidelity data.
- LOW-only broad-structure diagnostic: RMSE
  `7.474877897375379e-07`, correlation `0.9999999999961381`.
- At the narrow discrepancy center, absolute error decreased from
  `0.6784635380325933` to `1.043816504808781e-05` after HIGH observations.
- The measured local-to-far posterior-mean change ratio was
  `13006793.918672804` for the deterministic locality diagnostic.
- Grid, duplicate-point, and nearly-coincident-point tests returned finite,
  nonnegative marginal variances.
- Repeated fixed-seed fits produced bitwise-equal mean and variance arrays.

![Deterministic asynchronous-estimator milestone visualization](assets/async_estimator_milestone.png)

The panels distinguish observation timestamp from arrival time, show that only
explicit update requests publish versions, and compare the LOW-only snapshot
with the later snapshot containing delayed/out-of-order HIGH data. The final
no-data update preserves version 2 and its cached posterior.

![Historical softplus final multi-fidelity state](assets/final_multifidelity_state_milestone.png)

This preserved historical artifact records the formerly active normalized
softplus density. It is intentionally not overwritten after the transform
change.

![Current positive-mean final multi-fidelity state](assets/final_multifidelity_state_positive_mean.png)

The current top-left panel records normalized `max(high_mean, 0)` before
robot-specific Voronoi masks; the top-right records final
high-fidelity posterior standard deviation. Both bottom panels draw the
complete aerial and ground position
histories with blue reserved for aerial robots and orange for ground robots;
crosses mark final states. The lower-left background is the final GP mean and
the lower-right background is the simulator high-fidelity ground truth, using a
common color scale. The artifact uses the real coupled loop with one robot of
each class and a fixed seed.

![Unified coupled configuration](assets/unified_config_milestone.png)

The left panel shows the one-file resolution boundary: shared settings and the
two robot-specific override sections become the existing aerial and ground
parameter views. The right panel is populated by the real loader and confirms
the preserved production values for team sizes, dynamics, map, sensing,
observation noise, query grid, and shared simulation/GP settings.

![Deterministic hyperparameter-fitting diagnostic](assets/hyperparameter_fitting_milestone.png)

Panel A shows the deterministic LOW/HIGH training case and complete HIGH truth.
Panel B uses deliberately short fixed initial length scales, while panel C uses
the same observations after bounded marginal-likelihood fitting. Panel D reports
the prediction RMSE, likelihood objective, and all four initial/fitted kernel
values. The high-fidelity RMSE improves from `0.030626098141` to
`0.002923881087`; negative log marginal likelihood improves from
`18.779067613934` to `-51.943408691538`. Regenerate with:
`MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
examples.plot_hyperparameter_fitting_milestone`.

![Deterministic uniform-sector sampling diagnostic](assets/uniform_sector_sampling_milestone.png)

Panel A reproduces the old fixed-bearing spokes over 30 sensing events from a
stationary ground robot. Panel B uses the new random bearing and
`R*sqrt(U)` radius, filling the sensing disk without rays. Panel C compares the
empirical radial CDF with the ideal uniform-area CDF. The fixed-ray/uniform-radius
run has mean squared normalized radius `0.336376411466`; uniform-sector sampling
has `0.501389735339`, close to the theoretical `0.5`. Regenerate with:
`MPLCONFIGDIR=/tmp/matplotlib .venv/bin/python -m
examples.plot_uniform_sector_sampling_milestone`.

![Deterministic uniform-sector-only diagnostic](assets/uniform_sector_only_milestone.png)

Panel A shows 4096 real sensor samples filling the configured 120-degree FOV
sector without fixed bearings. Panel B compares the implemented radial CDF with
the area-uniform law $F(r)=r^2$; the mean normalized squared radius is
`0.502940966904` and the CDF distance is `0.009694316025`. Panel C compares the
relative-bearing CDF with the uniform angular law and has CDF distance
`0.012356479278`. Regenerate with `MPLCONFIGDIR=/tmp/matplotlib
.venv/bin/python -m examples.plot_uniform_sector_only_milestone`.

- Update history: `UPDATED` version 1 at time 1.0, `UPDATED` version 2 at time
  2.0, then `NO_NEW_DATA` with version 2 at time 2.5.
- Retained counts progressed from 6 LOW/0 HIGH to 7 LOW/3 HIGH and stayed fixed
  on the no-data update.
- At `x = 0.5`, delayed HIGH data changed posterior mean from
  `0.7334528417608333` to `1.4331973280163017` and posterior standard deviation
  from `0.7746379721107686` to `0.009986551400912595`.
- Both cached densities had weighted integrals equal to one within floating
  point precision (`0.9999999999999998` and `0.9999999999999999`).

![Deterministic shadow-simulation milestone visualization](assets/shadow_simulation_milestone.png)

The panels show independent LOW/HIGH/update schedules, uninterrupted controller
steps, deterministic team trajectories over simulator HIGH truth, and the
latest diagnostic-only posterior. The title and final panel explicitly record
that controllers still consume legacy densities.

- Over 21 integration steps, the visual run fired 6 LOW events, 11 HIGH events,
  and 5 GP updates while both controllers stepped 21 times.
- The final cached posterior version was 5 and its weighted density integral was
  exactly `1.0` for the configured weights.
- Final shadow-posterior mean RMSE against the complete simulator HIGH truth was
  `0.20301998589517364`; this is diagnostic, not a tuned performance claim.
- Density-source metadata remained `legacy_aerial_gp` and
  `legacy_posthoc_fusion` for the entire run.

![Deterministic aerial-feedback milestone visualization](assets/aerial_feedback_milestone.png)

The top row compares the joint high latent posterior mean before and after one
ground HIGH observation; these are estimator outputs, not direct HEDAC inputs.
The bottom-left panel is the diagnostic target difference `v2 - v1`, which is
never passed to HEDAC. The bottom-right background is the absolute cached target
v2 and overlays both real HEDAC-plus-Dubins trajectories: the dashed trajectory
was generated with absolute target v1, and the orange trajectory with absolute
target v2.

- The ground HIGH observation increased the retained HIGH count from zero to
  one and advanced the posterior from version 1 to version 2.
- Posterior-mean L2 change: `11.052928258033`.
- Posterior-variance L2 change: `4.027072985800`.
- Resampled normalized aerial-target L2 change: `0.006870448294`.
- Thirty-step paired Dubins-trajectory L2 change: `1.377960614875`.
- The target on each controller grid sums to one, remains nonnegative, and is
  returned by identity from the cache for the same posterior version/grid.

![Deterministic ground-feedback milestone visualization](assets/ground_feedback_milestone.png)

The top row shows the exact normalized high-fidelity densities supplied to the
ground controller before and after one local HIGH correction. The lower-left
panel shows the resulting density/weight change before the existing one-robot
Voronoi mask; the lower-right panel overlays two runs of the unchanged CasADi
MPC using densities v1 and v2.

- Ground-density L2 change: `0.094090124693`.
- Fifteen-step real ground-trajectory L2 change: `1.670324202866`; the paired
  robot starts at `(5, 4)`, close enough to the new peak for the response
  difference to be visually clear.
- Coverage cost evaluated at the HIGH-correction location changed from
  `70.099092594586` to `60.570347024588`, the expected direction when local
  importance increases at the robot position.
- Both densities have weighted integral exactly `1.0` within displayed
  precision.
- Both paired MPC runs retained zero samples in their legacy ground GP,
  confirming that only the density source—not a hidden legacy estimate—drove
  the comparison.

## Runtime Measurements

Only small deterministic unit-test and per-snapshot fit/prediction timing is
recorded. Exact wall-clock values are deliberately not asserted. No
production-sized benchmark is meaningful before simulation scheduling chooses
final sample limits and query-grid resolution.

The Milestone 7 full test/CLI/evaluation validation completed in approximately
8.1 seconds in this environment. The deterministic real-MPC plot completed in
approximately 4.0 seconds. The hyperparameter-extension full suite completed in
4.44 seconds. A production-config one-step initialization/update/fitting smoke
completed in 7.8 seconds; this is not a full-run benchmark.

## Design Decisions

- `rho`, observation-noise variances, and the jitter policy remain known fixed
  inputs. The four RBF kernel values are configured initial values and become
  learned empirical-Bayes state when scheduled fitting is enabled; the GP also
  learns posterior information about `f_L`, `delta`, and therefore `f_H`.
- Hyperparameter optimization is bounded and deterministic. It is scheduled by
  estimator update count, not numerical integration step, and is skipped until
  the configured minimum retained-sample count is available.
- Signal `variance` and observation-noise values are variances, not standard
  deviations. Scalar noise inputs are expanded to per-observation vectors.
- A newly constructed model explicitly represents the empty-data prior. Empty
  LOW and HIGH datasets, and either single-fidelity dataset, are valid.
- Training order is `[y_LOW, y_HIGH]`; prediction is for the latent high field,
  so observation noise is not added to returned marginal variance.
- The shared controller density is the quadrature-normalized positive part of
  the latent high mean: `max(high_mean, 0)`. Negative estimates contribute zero
  mass; positive values retain their amplitude. No softplus/softmax or
  selectable transform remains in production.
- Tiny negative posterior variances caused by roundoff are clipped using a
  scale-aware `1e-10` tolerance; materially negative or nonfinite results raise.
- All candidate fitted state is local until factorization and solves succeed.
- Retention selection is newest-first by collection timestamp and insertion
  index, while published GP inputs are chronological. LOW and HIGH stores remain
  independent even at identical positions.
- Buffer `candidate()` is non-mutating. `commit()` clears only pending entries
  incorporated into that candidate, preserving later arrivals.
- Every estimator update uses a fresh candidate GP configured from the last
  valid model; it replaces the owned GP only after prediction and density
  validation succeed.
- The scheduler supplies simulated update time. `perf_counter()` is used only
  for fit/prediction metrics, never scheduling.
- A boolean mask uses `True` for included/free query points. Structured-grid
  resampling is bilinear and raises explicitly for out-of-bounds queries.
- Periodic schedules are anchored to `start_time + fire_count * period`, avoiding
  cumulative floating-point drift; calls occur on the next integration step
  when a period is not an integer multiple of `dt`.
- LOW and HIGH sensors use separately spawned generators derived from the
  configured simulation seed and sensor offset, never NumPy's global RNG.
- The explicit coupled-builder seed overrides the YAML seed for both legacy
  initialization and dedicated shadow sensors, so evaluation episodes vary as
  intended while remaining reproducible.
- `CoupledSimulation` reads the aerial target before each HEDAC step. A HIGH
  observation collected and published later in that step therefore changes
  aerial control on the following step, matching the approved sequence.
- The aerial target uses `std_H / max(std_H)` before applying its configurable
  uncertainty weight. It is normalized again after resampling and masking on
  the controller grid.
- Multifidelity HEDAC calls set `update_legacy_gp=False`; legacy calls retain the
  default `True`. External-target validation occurs before controller state is
  mutated, and a valid external target is applied after the optional GP update.
- The cache key includes posterior version, both controller axes, integration
  weights, and mask. Cache replacement is transactional after successful
  validation/resampling/normalization.
- Ground lookup returns the exact immutable snapshot density when controller
  points and integration weights match the estimator grid. Alternate points use
  bilinear interpolation followed by weighted normalization.
- `build_ground_weight_vectors()` performs only validated elementwise
  multiplication. Voronoi geometry remains owned by the existing utility and
  the result still enters the solver as `concatenate([state, weights])`.
- Multifidelity external density skips both legacy ground estimation and
  post-hoc fusion. `None` retains those exact equations as fallback. Controller
  density/source state is committed only after external validation.
- Posterior variance has no active path into the ground objective. The method
  documentation records this disabled future extension point without adding a
  configuration option or objective term.
- The existing ground GP/fusion/MPC equations were extracted into a private
  adapter in the new reusable loop; existing scripts and public controller/agent
  signatures were left untouched.

## Deviations From the Approved Plan

No architectural or controller deviation. The implementation follows the
approved ground-density interface, preserves every audited MPC equation and
setting, and keeps target diagrams frozen. At the user's request, a lightweight
direct final-state plot was added before the plan's later structured logging
milestone. It does not replace that milestone's planned NPZ recorder or
saved-record evaluation plots. The user subsequently approved a single-file
configuration refinement; the implementation plan now records this boundary,
and no runtime control or estimation behavior changed.

The later user-approved hyperparameter-fitting extension intentionally advances
beyond the target's “fixed hyperparameters initially” statement. Only the four
RBF kernel parameters are fitted; `rho`, sensor noise, GP ownership, scheduling,
transaction semantics, controllers, and dynamics retain the approved design.

The user-approved positive-part refinement intentionally replaces the target
diagram's softplus density block. The target diagram remains frozen and is now
historical at that transform boundary; current implementation diagrams show
clipping. Both aerial interest and ground coverage use the replacement because
they share `PosteriorSnapshot.density`.

The user-approved uniform-sector-only refinement intentionally removes the
sampling selector and fixed-ray central-sensor branch that were initially kept
for compatibility. It affects simulated LOW and HIGH sample geometry only; the
target architecture, estimator, controllers, and robot dynamics are unchanged.

## Known Issues

- The `uv` executable is absent, so `uv lock --check` could not be run. The lock
  file was not modified.
- Importing through `src.core` also initializes legacy core dependencies. The
  new module itself imports only NumPy and SciPy and remains independent of
  simulation/controller code.
- An uncertainty-only aerial target with identically zero variance has zero mass
  and raises instead of inventing a distribution. A mixed or interest-only
  target remains valid in that edge case.
- An all-nonpositive high posterior mean has zero clipped mass. Such an update
  fails transactionally and preserves the latest valid snapshot; before any
  valid snapshot exists, controller startup fallback behavior remains active.
- The production 50x50/100x100, seven-ground-robot configuration has not yet
  received a runtime benchmark; fast CLI validation uses explicit smoke files.
- Video encoding depends on the selected suffix: the canonical `.gif` path uses
  Pillow and has been validated. MP4/MOV/M4V/AVI require an `ffmpeg` executable
  visible to Matplotlib; this workspace has no `ffmpeg`, so those encoders are
  intentionally not claimed as locally validated.
- Obstacle-aware fidelity smoothing is implemented, but the current production
  configurations are obstacle-free and no query-grid obstacle-mask resampling
  is performed by the builder yet.
- Both controllers now consume products of the central posterior in
  multifidelity mode. End-to-end paired closed-loop experiment validation and
  logging remain future milestones, so final project completion is not yet
  claimed.
- Marginal-likelihood fitting produces point estimates rather than a posterior
  over hyperparameters. Sparse or spatially clustered retained samples can push
  values to configured bounds; fitted values and bounds should therefore be
  interpreted together. The synthetic validation proves the mechanism, not
  universal reconstruction improvement for every trajectory.
- `tests/test_coupled_config.py` currently expects canonical ground controller
  `lloyd`, while the user-edited `configs/multifidelity.yaml` selects `mpc`.
  This sensing milestone preserves the user's controller choice and reports the
  stale assertion rather than changing unrelated controller behavior.
- `tests/test_run_multifidelity_cli.py` still expects `posterior_version=` in
  progress messages, while the current runner omits it. This is the previously
  reported unrelated CLI progress mismatch.

## Backward-Compatibility Status

Legacy GP, robot, MPC, hierarchical, and existing evaluation equations remain
unchanged. Both new controller inputs are keyword-only/optional, and the legacy
coupled branch never constructs a coordinator or passes an external density.
It still reports `legacy_aerial_gp` / `legacy_posthoc_fusion`. The
multi-fidelity scalar-field sensor constructor intentionally no longer accepts
a sampling selector, and stale multifidelity YAML keys are no longer consumed.
The new multifidelity CLI interface intentionally accepts only `--config`; its
former split aerial/ground arguments were removed as requested. Legacy scripts
retain their historical interfaces. The two current accumulated-suite failures
are listed under Known Issues.

## Next Milestone

Stop here pending the next approved prompt. Milestone 8 will harden and name the
legacy fusion/mode boundary, add the paired ablation entry point, and prove the
frozen legacy references remain operational after both controller integrations.

## LaTeX publication milestone

The mathematical reference now has a pure LaTeX source at
`docs/latex/mathematical_formulation.tex`. The Markdown file is a concise
landing page that links to the source and the GitHub Pages PDF. The PDF is not
committed: pull requests upload it as a 30-day workflow artifact, and a push to
`main` deploys it to GitHub Pages through an artifact-only deployment.

- Files changed: `docs/latex/mathematical_formulation.tex`,
  `docs/mathematical_formulation.md`,
  `.github/workflows/mathematical-formulation-pdf.yml`, `README.md`, and
  this status document.
- Regeneration command: GitHub Actions runs the
  `Build mathematical formulation PDF` workflow; its `build` job compiles
  `docs/latex/mathematical_formulation.tex`.
- Visual artifact: the compiled PDF is the deterministic rendered artifact.
  It is available as a pull-request workflow artifact and, after a successful
  `main` build, at the GitHub Pages URL linked above.
- The deterministic publication-flow diagram is
  `docs/assets/latex_publication_pipeline.svg`, generated by
  `python3 examples/plot_latex_publication_pipeline.py`. Its left-to-right
  flow distinguishes temporary pull-request artifacts from the `main`-only
  GitHub Pages deployment and makes explicit that no generated PDF is written
  to Git history.
- Local validation: this workspace has neither `latexmk` nor a LaTeX engine,
  so local PDF compilation could not be run. The workflow's
  `-halt-on-error` compile step is the required build validation.
- Deterministic validation: `python3 -m unittest
  tests/test_documentation_publication.py` checks the LaTeX document structure,
  the least-privilege build/deploy workflow contract, and the SVG artifact.
- Commands executed: `python3 examples/plot_latex_publication_pipeline.py`,
  `python3 -m unittest tests/test_documentation_publication.py`, and
  `git diff --check`. All completed successfully after replacing the initial
  dependency-based renderer with the standard-library SVG renderer. Earlier
  local attempts using `python` and `uv` could not run because those executables
  are unavailable in this workspace; no project behavior was affected.
- Inspection note: the generated SVG is well-formed and its source geometry was
  checked locally. The workspace image viewer cannot render SVG, so visual
  rendering verification remains pending the GitHub Pages PDF build.
- Numerical and behavioral regressions: none expected; this milestone changes
  documentation and publishing automation only.

## Video recording milestone

- Files changed: `src/utils/multifidelity_video.py`,
  `examples/run_multifidelity.py`, `configs/multifidelity.yaml`,
  `configs/multifidelity_video_smoke.yaml`,
  `tests/test_multifidelity_video.py`, `tests/test_run_multifidelity_cli.py`,
  `docs/multifidelity_config_reference.md`,
  `docs/latex/mathematical_formulation.tex`,
  `docs/architecture/implementation_video_recording.md`, this status document,
  and the video-recording milestone visual assets.
- Regeneration commands: `python3 examples/plot_video_recording_milestone.py`
  for the observer-flow SVG, and `PYTHONPATH=. MPLCONFIGDIR=/tmp/matplotlib-codex
  .venv/bin/python -m examples.run_multifidelity --config
  configs/multifidelity_video_smoke.yaml --no-plot` for the real GIF.
  The SVG shows the completed-step observer flow, interval gate, renderer, and
  encoder, including the intentional absence of a feedback path to the loop.
- Validation: the new deterministic recorder tests use a fake writer and assert
  interval sampling, final-frame forcing, duplicate prevention, and no-output
  behavior before a valid posterior. `ruff` passed and 11 focused tests passed.
  The real 12-step smoke run completed with posterior version `6`, final
  ergodic metric `0.538776926363`, and a nonempty 1600x720 GIF. Its frame was
  visually inspected for labels, clipping, and both robot classes.
- Numerical and behavioral regressions: none expected. The recorder runs after
  completed steps and cannot affect the mathematical model or controller state.
- Video-scale refinement: the HIGH-posterior uncertainty color range is now
  fixed from the first captured frame through the end of that video. This is a
  visualization-only change; the published variance array and all estimator
  behavior remain unchanged.

## Footprint-normalized coverage evaluation milestone

- Files changed: `evaluation/multifidelity_metrics.py`,
  `evaluation/evaluate_multifidelity_ablation.py`,
  `evaluation/plot_multifidelity_ablation.py`,
  `tests/test_multifidelity_ablation_pipeline.py`,
  `configs/multifidelity_ablation.yaml`,
  `examples/plot_coverage_metric_milestone.py`,
  `docs/assets/coverage_metric_milestone.png`,
  `docs/mathematical_formulation.md`,
  `docs/latex/mathematical_formulation.tex`,
  `docs/experimental_evaluation_plan.md`,
  `docs/architecture/implementation_coverage_metric.md`, and this status file.
  `.gitignore` also gained narrow exceptions for the required milestone diagram
  and PNG artifact.
- The evaluator retains `covered_probability_mass` as the raw truth mass in
  the actual union of ground sensing sectors. Its primary `coverage` array is
  now that mass divided by a fixed episode oracle. The oracle's area budget is
  `min(free_area, robot_count * 0.5 * fov_radians * range**2)`; it sorts free
  cells by truth-density value and fractionally accepts the final cell. The
  archive also records the oracle mass, area budget, free area, area fraction,
  and raw final and time-averaged values.
- The ablation plot's fourth panel now displays footprint-normalized coverage
  effectiveness. No estimator or closed-loop behavior changed. The ablation
  YAML was resynchronized with the current canonical 200-step/two-aerial-robot
  values while retaining its 20 episodes and ablation isolation settings.
- Regeneration command: `MPLCONFIGDIR=/tmp/ral_marta_mpl PYTHONPATH=.
  .venv/bin/python examples/plot_coverage_metric_milestone.py`. The fixed
  synthetic field produced visible mass `0.330499593435`, densest-area oracle
  mass `0.342483594409`, and effectiveness `0.965008540060`. The PNG was
  visually inspected: sector overlays, axes, density colorbar, bar labels, and
  score annotation are readable and unclipped.
- Smoke validation commands: `evaluation/run_multifidelity_ablation.py` for one
  four-step full-method episode, followed by
  `evaluation/evaluate_multifidelity_ablation.py` and
  `evaluation/plot_multifidelity_ablation.py`, using `/tmp` archives. The
  episode had area budget `3.14159265`, free area `81`, oracle mass
  `0.21776701`, raw visible mass from `0.02886549` to `0.06157043`, and
  normalized effectiveness from `0.13255216` to `0.28273532`.
- Validation results: Ruff passed for all affected Python files. The focused
  metric/pipeline suite passed `13/13`. The accumulated suite passed 186 tests
  and retained three unrelated failures: stale expectations of 300 canonical
  steps, the former orange ground-trajectory color, and
  `posterior_version=` in progress text. No test related to the new metric
  failed.
- Numerical and behavioral regressions: the meaning of evaluated `coverage`
  intentionally changed from a low-magnitude raw probability mass to a bounded
  footprint-normalized effectiveness. Consumers requiring the former quantity
  must read `covered_probability_mass`. Simulation trajectories, estimator
  posteriors, controller targets, and runtimes are unchanged because all new
  calculations occur in offline evaluation.

## Proposed-method baseline-comparison runner milestone

- Files changed: `evaluation/comparison_io.py`,
  `evaluation/run_multifidelity_comparison.py`,
  `tests/test_multifidelity_comparison_runner.py`,
  `examples/plot_multifidelity_comparison_runner_milestone.py`,
  `docs/assets/multifidelity_comparison_runner_milestone.png`,
  `docs/architecture/implementation_baseline_comparison_runner.md`,
  `.gitignore`, and this status file.
- The runner instantiates the existing production `CoupledSimulation` for one
  predeclared seed per episode. It saves the simulator HIGH, LOW, and
  discrepancy truth rasters; normalized truth on the controller grid; map and
  free mask; initial and full aerial/ground states; physical timestamps;
  sensor geometry; posterior records and sample counts; and per-phase wall
  timings. It deliberately computes no KL, reconstruction, calibration, or
  coverage metric during simulation.
- The archive metadata makes the pairing rule explicit: a baseline must reuse
  the saved truth, map, and initial states rather than regenerating them from a
  seed. This is necessary because method implementations consume random draws
  in different orders. The current `evaluation/egerstedt.py` demonstration has
  not yet been adapted to this archive contract.
- Direct smoke command: `PYTHONPATH=. .venv/bin/python
  evaluation/run_multifidelity_comparison.py --config
  configs/multifidelity_ablation_smoke.yaml --output
  /tmp/multifidelity_comparison_smoke.npz --episodes 1 --num-steps 4
  --no-progress`. It produced one five-state aerial trajectory, one five-state
  ground trajectory, two posterior records, and truth-density weighted mass
  `0.9999999999999999`.
- Visual regeneration command: `MPLCONFIGDIR=/tmp/ral_marta_mpl PYTHONPATH=.
  .venv/bin/python
  examples/plot_multifidelity_comparison_runner_milestone.py`. The resulting
  fixed-seed figure was visually inspected after adding plot margins: both
  robot classes, the shared truth field, labels, colorbar, and boundary states
  are readable without clipping.
- Validation results: Ruff passed. The new runner tests and existing ablation
  pipeline tests passed `15/15`. The accumulated suite passed 188 tests and
  retained three unrelated failures: stale expectations of 300 canonical
  steps, the former orange ground-trajectory color, and
  `posterior_version=` in progress text.
- Mathematical documentation: no equations changed. The new file only records
  existing simulation inputs and outputs, so `docs/mathematical_formulation.md`
  and its LaTeX source remain current without modification.
- Numerical and behavioral regressions: none. The smoke archive's normalized
  truth integrated to one, and the runner does not feed archived values back
  into the simulation. Comparison metrics remain a separate future stage.

## Separate Egerstedt baseline runner milestone

- Files changed: `evaluation/comparison_io.py`,
  `evaluation/run_multifidelity_comparison.py`,
  `evaluation/run_egerstedt_comparison.py`,
  `tests/test_egerstedt_comparison_runner.py`,
  `examples/plot_egerstedt_comparison_runner_milestone.py`,
  `docs/assets/egerstedt_comparison_runner_milestone.png`,
  `docs/architecture/implementation_baseline_comparison_runner.md`,
  `docs/mathematical_formulation.md`,
  `docs/latex/mathematical_formulation.tex`, `.gitignore`, and this status
  file. The baseline algorithm source `evaluation/egerstedt.py` remains
  separate and unchanged.
- Default outputs are now
  `output/baseline_comparison/multifidelity/raw.npz` for the proposed method
  and `output/baseline_comparison/egerstedt/raw.npz` for the baseline. The
  Egerstedt default input is the proposed-method archive in the first folder.
- The paired scenario fingerprint hashes seeds, physical timestamps, query
  grid and weights, truth raster and density, free mask, map, full initial
  states, and aerial/ground sensing ranges. Both archives retain enough source
  arrays to recompute the same fingerprint. Method-specific FOV is excluded:
  the proposed ground sector is configured independently, whereas this
  baseline faithfully uses an omnidirectional range-limited disk.
- The runner rejects obstacle maps because the baseline implementation assumes
  a convex obstacle-free domain. It reuses the paired timestep, positions,
  horizon, ranges, and robot counts; executes uniform range-limited aerial
  Lloyd motion and truth-weighted hierarchical ground Lloyd motion; and saves
  positions, allocation errors, locational cost, and phase timings. It saves no
  fabricated posterior, reconstruction metric, or heading state.
- Commands: first run `python evaluation/run_multifidelity_comparison.py`, then
  `python evaluation/run_egerstedt_comparison.py`. Both support direct
  execution, progress display, and explicit input/output overrides.
- Visual regeneration command: `MPLCONFIGDIR=/tmp/ral_marta_mpl PYTHONPATH=.
  .venv/bin/python examples/plot_egerstedt_comparison_runner_milestone.py`.
  The fixed-seed paired smoke run produced fingerprint
  `6849ca260e9c11200963c37267a9faced3222f280c7713d3175a82f569f1729c`
  and final baseline locational cost `10.2814150565`. The two-panel PNG was
  visually inspected; shared density scale, initial crosses, final markers,
  both robot-class trajectories, boundary margins, labels, and colorbar are
  readable without clipping.
- Validation results: Ruff, compilation, and diff checks passed. Eight focused
  runner/publication tests passed. The accumulated suite passed 191 tests and
  retained the same three unrelated failures: stale expectations of 300
  canonical steps, the former orange ground-trajectory color, and
  `posterior_version=` in progress text.
- Numerical and behavioral regressions: none in production code. The proposed
  runner's default output path intentionally changed to its method subfolder.
  The baseline uses the paired mission timestep instead of the demonstration
  function's hard-coded default and records that exact implemented choice in
  the mathematical reference. Common comparison metric computation remains a
  separate offline stage.

## Separate comparison-trajectory plotting milestone

- Files changed: `evaluation/plot_baseline_comparison.py`,
  `tests/test_plot_baseline_comparison.py`,
  `examples/plot_separate_comparison_trajectories_milestone.py`, the two PNGs
  below `docs/assets/comparison_trajectory_milestone/`,
  `docs/architecture/implementation_baseline_comparison_runner.md`,
  `.gitignore`, and this status file.
- Running `python evaluation/plot_baseline_comparison.py` reads the default
  paired archives and writes
  `output/baseline_comparison/multifidelity/trajectories_episode_000.png` and
  `output/baseline_comparison/egerstedt/trajectories_episode_000.png`. The
  `--episode N` and `--output-root PATH` arguments select another episode or
  destination without combining the methods into one image.
- Before plotting, the script recomputes both scenario fingerprints and
  requires identical saved truth. Each independent image uses the same truth
  normalization, domain extent, physical-unit axes, figure size, and marker
  vocabulary. Aerial paths are dashed, ground paths are solid, initial
  positions are crosses, and final positions use class-specific markers.
- Final ground footprints are method-specific: the proposed plot uses saved
  headings, FOV, and range to draw sectors; the Egerstedt plot draws its saved
  omnidirectional disks. Footprint portions outside the environment are clipped
  by the axes and therefore visually expose boundary loss.
- Visual regeneration command: `MPLCONFIGDIR=/tmp/ral_marta_mpl PYTHONPATH=.
  .venv/bin/python
  examples/plot_separate_comparison_trajectories_milestone.py`. Both generated
  PNGs were inspected separately. Titles, legends, axes, initial/final markers,
  trajectories, footprints, density colorbars, boundary clipping, and margins
  are readable; the files have identical pixel dimensions.
- Validation results: Ruff passed. Seven focused plot/runner tests passed. The
  accumulated suite passed 193 tests and retained the same three unrelated
  failures: stale expectations of 300 canonical steps, the former orange
  ground-trajectory color, and `posterior_version=` in progress text.
- Mathematical documentation and regressions: no equations, metrics,
  controllers, fields, sensing, or dynamics changed. This is offline plotting
  only, so the existing mathematical references remain current.

## Standalone Zhang distributed-coverage baseline milestone

- Files changed: new `evaluation/zhang.py`,
  `evaluation/run_zhang_baseline.py`, `evaluation/plot_zhang_baseline.py`, and
  `evaluation/zhang_io.py`; new `configs/zhang_baseline.yaml` and
  `configs/zhang_baseline_smoke.yaml`; new `tests/test_zhang_baseline.py`,
  `examples/plot_zhang_baseline_milestone.py`,
  `docs/assets/zhang_baseline_milestone.png`, and
  `docs/architecture/implementation_zhang_baseline.md`; plus `.gitignore`,
  `README.md`, `docs/experimental_evaluation_plan.md`,
  `docs/mathematical_formulation.md`, its LaTeX source, and this status file.
- The new controller is isolated from both `CoupledSimulation` and the
  Egerstedt runner. It implements Zhang Algorithms 1 and 2 with rasterized
  weighted power cells, power-cell edge adjacency, geometric aerial
  deployment, distributed load and load-to-mass consensus, cell-weight and
  aerial-position adjustment, neighbor-cell ground transfers, and
  high-resolution limited-range Lloyd centroids. All communication is logical
  same-step neighbor exchange in one Python process; both robot classes retain
  the paper's first-order point dynamics.
- The standalone experiment removes obstacles rather than silently extending
  the paper. Its fixed scalar field is quantized independently to configured
  LOW and HIGH resolutions. The default fixes A5/G14 and sweeps
  `R_G = [0.15, 0.25, 0.35, 0.50]`; arbitrary composition entries are also
  accepted only when `aerial > 1` and `ground > aerial`. Multiple composition
  and range declarations form an explicit Cartesian product.
- Initial position streams are nested across conditions. Random tie streams are
  paired across sensing ranges within an episode/composition. The 61-by-61
  default grid uses deployment tolerance `0.01`; all ten configured seeds
  settled in 415--714 deployment steps. The earlier `0.001` candidate caused
  some sub-grid raster-centroid limit cycles to reach the 1000-step cap and was
  rejected before finalizing the configuration.
- Tests were written before production modules and initially failed collection
  because the Zhang modules did not exist. The five deterministic contracts now
  cover weighted-cell expansion and adjacency, load conservation and ratio
  consensus, sensing-limited hierarchical centroids, repeatable closed-loop
  cost reduction, paired composition/range archive shapes, and real plotting.
- The 80-step real-output smoke run used four conditions:
  A2/G5 at radii 0.35 and 0.55 and A3/G6 at the same radii. Their initial costs
  were `4.71220682449`, `4.71220682449`, `3.85230378855`, and
  `3.85230378855`; final costs were `3.98624940490`, `3.85875685639`,
  `3.73335508545`, and `3.25289557049`. Final loads exactly matched targets
  `[3,2]`, `[3,2]`, `[2,2,2]`, and `[2,2,2]`. Accumulated ground travel was
  `0.688596293341`, `1.11381538145`, `1.06895192802`, and
  `1.43638264829`.
- The deterministic 2350-by-1801 milestone figure was regenerated from the
  saved real run and visually inspected. The HIGH field, final power-cell
  boundaries, aerial and ground trajectories, full cost histories, sensing
  sweep summaries, titles, axes, and legends are readable and unclipped.
- Commands executed included the initial red test, repeated focused Zhang
  tests, the existing Egerstedt/scenario-pipeline regression selection, an
  80-step smoke run, all-ten-seed default initialization validation, milestone
  regeneration, Ruff lint and format checks, Python compilation,
  `git diff --check`, the full pytest suite, and a forced LaTeX build.
- Validation: the focused cross-component selection passed `11/11`; the new
  Zhang suite passed `5/5`; Ruff, formatting, compilation, and diff checks
  passed. The full suite passed 257 tests and retained four previously
  documented unrelated failures: canonical 300 versus configured 200 steps,
  orange versus current cyan ground trajectory color, ablation 360-degree
  versus configured 90-degree ground FOV, and required
  `posterior_version=` progress text. The mathematical reference built a
  22-page PDF; existing stream, box-splitting, and table-width warnings remain.
- Numerical and behavioral regressions: none in the multifidelity or Egerstedt
  implementations because no production controller, estimator, pipeline, or
  existing baseline file imports the Zhang code. Zhang outputs are new. The
  baseline deliberately has no posterior or reconstruction metrics, uses no
  obstacles, and is not automatically invoked by the multifidelity pipeline.

## Single-scenario 30-second homogeneous-composition protocol milestone

- Files changed for this milestone: `configs/multifidelity_composition.yaml`,
  `README.md`, `.gitignore`, `docs/experimental_evaluation_plan.md`,
  `docs/multifidelity_config_reference.md`,
  `docs/mathematical_formulation.md`, its LaTeX source,
  `tests/test_multifidelity_composition_pipeline.py`, this status file, and the
  new real-output figure
  `docs/assets/homogeneous_composition_30s_milestone.png`.
- The publication composition configuration now declares only `easy_long`.
  Both its base simulation horizon and scenario override are 300 integration
  steps at $\Delta t=0.1$ s, for a 30-second mission. The former
  `hard_short` declaration was removed from the publication configuration.
  Reusable multi-scenario parsing, selection, and smoke-test coverage remain
  available for other configurations.
- The experiment documentation now specifies one conditional five-obstacle
  comparison rather than a two-scenario study. At the shared 0.5-second
  sensing period there are 60 sensing events and 6000 submitted observations
  per episode for every ten-robot composition. LOW/HIGH totals are 6000/0,
  1200/4800, and 0/6000 for A10/G0, A2/G8, and A0/G10, respectively.
- Deterministic real-output validation used seed 42, one episode, all three
  publication compositions, and the full 300-step horizon. The archive stored
  301 states through exactly 30.0 s; its last posterior publication was at
  29.0 s and the evaluator retained the existing final-posterior hold through
  the mission endpoint for time averages. The pipeline produced raw and
  evaluated archives plus metric and A2/G8 trajectory figures under
  `/tmp/homogeneous_30s_milestone`. The 3080-by-1491 metric figure copied to
  `docs/assets/homogeneous_composition_30s_milestone.png` was visually
  inspected; all six panels, legends, physical-time axes, labels, and margins
  are readable.
- Commands executed included the focused composition/scenario/trajectory test
  selection, the one-episode publication-config pipeline, archive metadata and
  observation-count checks, the full pytest suite, Ruff lint and format
  checks, Python compilation, YAML assertions, `git diff --check`, image
  inspection, and a forced `latexmk` build of the mathematical reference.
- Validation results: 29 focused tests passed. The full suite passed 271 tests
  and retained the same four previously documented unrelated failures:
  canonical `configs/multifidelity.yaml` 200 versus its test's 300-step
  expectation, ground-trajectory cyan versus orange, ablation ground FOV 90
  versus 360 degrees, and missing `posterior_version=` progress text. Ruff,
  formatting, compilation, YAML validation, and diff checks passed. The
  mathematical reference built successfully as a 25-page PDF with its existing
  stream, box-splitting, and table-width warnings.
- Numerical and behavioral regressions: no estimator, controller, scheduler,
  robot, or metric equation changed. The intentional protocol change increases
  the publication composition mission and observation opportunities by 50%
  relative to the former 20-second `easy_long` setting and removes the hard
  condition from that publication sweep.

## Paper reconstruction-history figure milestone

- Files changed for this milestone:
  `evaluation/plot_multifidelity_composition.py`,
  `tests/test_multifidelity_composition_pipeline.py`, the regenerated
  `docs/assets/homogeneous_composition_30s_milestone.png`, and this status file.
- The composition plot is now a single row containing only the NRMSE, KL, and
  NLPD histories. Its subplot titles are exactly `NRMSE`, `KL Divergence`, and
  `NLPD`; the global title and the former final/time-average summary row were
  removed. The evaluated archive still retains every final and time-average
  value for tabulation.
- Legend labels are derived from the saved aerial and ground counts: a zero
  ground count maps to `aerial-only`, two active classes map to
  `heterogeneous`, and a zero aerial count maps to `ground-only`. This avoids
  hard-coding the publication's numeric composition strings while producing
  the requested labels for A10/G0, A2/G8, and A0/G10.
- A deterministic contract now captures the real Matplotlib figure before it
  closes and asserts three axes, the exact titles, no suptitle, and the exact
  legend order. The test was first observed failing against the old six-panel
  plot. Removing the old hyperparameter-policy helper initially exposed its
  import by the reusable two-scenario plotter; the helper was restored for
  backward compatibility and remains absent from the paper figure.
- The figure was regenerated from the real one-episode, 300-step evaluated
  archive at `/tmp/homogeneous_30s_milestone/evaluated/easy_long.npz`. The
  resulting 2180-by-645 PNG was inspected at original resolution; titles,
  curves, axes, grid, legend, and margins are readable and unclipped.
- Validation commands covered the focused composition and scenario pipelines,
  Ruff lint and formatting, Python compilation, and `git diff --check`.
  Twenty-two focused tests passed and all static checks passed. No estimator,
  controller, metric, archive, or numerical result changed; this milestone is
  an offline presentation change only, so the mathematical reference remains
  unchanged.

## Paper reconstruction typography and two-column layout milestone

- Files changed for this milestone:
  `evaluation/plot_multifidelity_composition.py`,
  `tests/test_multifidelity_composition_pipeline.py`, `README.md`,
  `paper/updated.tex`, the new vector
  `paper/pics/error_metrics.pdf`, the regenerated real-output preview
  `docs/assets/homogeneous_composition_30s_milestone.png`, and this status file.
- The plotter now exposes paper-style defaults as named constants and accepts
  matching Python keywords and CLI flags for figure width, figure height,
  subplot-title size, axis-label size, tick-label size, legend size, line
  width, confidence-band opacity, and raster DPI. Inputs are validated before
  reading the archive. PDF output remains vector graphics, while the DPI
  setting controls raster output.
- Defaults target the IEEE conference template's approximate two-column text
  width: 7.16-by-2.55 inches, 10-point titles, 9-point axis and legend labels,
  8-point ticks, 2.2-point curves, 0.16 confidence-band opacity, and 300 DPI.
  `paper/updated.tex` now embeds `pics/error_metrics.pdf` in a top-positioned
  `figure*` at `\textwidth`; the former single-column `figure` was the main
  source of unreadable downscaling.
- The deterministic styling contract first failed because the old plotting API
  did not accept the requested controls. It now asserts the requested physical
  figure size, all font sizes, curve widths, band opacity, three-axis layout,
  titles, absence of a suptitle, and legend labels.
- Both the vector paper asset and the 2182-by-793 PNG preview were regenerated
  from the saved 30-episode, 300-step `easy_long` evaluated archive. The preview
  was inspected at original resolution. A forced two-pass LaTeX build produced
  an eight-page `paper/updated.pdf`; rendered page 7 was inspected and confirms
  that titles, ticks, axes, curves, bands, and legend are readable at full-page
  scale without clipping.
- Validation commands covered 22 focused composition/scenario tests, two
  post-format regression tests, Ruff lint and formatting, Python compilation,
  CLI-help inspection, `git diff --check`, vector/raster regeneration, the
  forced paper build, and rendered-page inspection. All focused and static
  checks passed. The paper retains its existing missing-author, package,
  underfull-box, and undefined `da fare` reference warnings.
- Numerical and behavioral regressions: none. Estimation, metrics, bootstrap
  samples, saved archives, and plotted data are unchanged; only configurable
  presentation and LaTeX placement changed. The mathematical reference remains
  current because no implemented equation changed.

## Single-column four-method paper figures milestone

- Files changed for this milestone:
  `evaluation/plot_four_method_comparison.py`,
  `evaluation/plot_four_method_trajectories.py`,
  `evaluation/run_four_method_comparison_pipeline.py`,
  `tests/test_four_method_comparison_pipeline.py`, `README.md`, the regenerated
  real-output assets `docs/assets/four_method_comparison_milestone.png` and
  `docs/assets/four_method_trajectory_milestone.png`, and this status file.
- Both four-method figures now target an IEEE single column rather than full
  text width. The metric figure uses a 3.5-by-5.2-inch canvas and stacks the
  unchanged coverage-effectiveness and locational-cost histories vertically.
  The trajectory figure keeps its two-by-two map arrangement on a
  3.5-by-4.0-inch canvas. Saving no longer uses a tight bounding box, so the
  declared physical width is retained exactly instead of expanding around long
  text.
- Typography reuses the reconstruction figure's paper defaults: 10-point
  titles, 9-point axis labels, and 8-point ticks. Both standalone CLIs expose
  the physical canvas and typography controls; the metric CLI additionally
  exposes legend size, curve width, and confidence-band opacity. Raster output
  defaults to 300 DPI, including the one-command four-method pipeline.
- The trajectory figure now contains only the four map axes. Its side density
  color scale, figure legend, global episode/seed/scenario title, and Zhang
  phase-transition title suffix were removed. Titles are exactly `Proposed`,
  `Rudolph [17] - LR`, `Rudolph [17]`, and `Zhang [18]` in archive order.
- A deterministic Matplotlib contract first failed against the old plotters and
  now asserts the two physical canvases, vertical metric-panel ordering, four
  trajectory axes, exact titles, empty figure-legend list, and absent suptitle.
- Real previews were regenerated from the saved four-method comparison
  archives. The metric PNG is exactly 1050-by-1560 pixels and the trajectory
  PNG is exactly 1050-by-1200 pixels at 300 DPI. Both were inspected at
  original resolution; titles, axes, curves, confidence bands, trajectories,
  footprints, and margins are readable and unclipped. These saved archives are
  milestone data rather than a replacement for the final publication episode
  set.
- Validation commands covered the initial failing contract, a 12-test
  four-/three-method and Zhang regression selection, the three focused
  four-method tests after formatting, Ruff lint and formatting, Python
  compilation, `git diff --check`, both real-output regenerations, and original
  resolution image inspection. All tests and static checks passed.
- Numerical and behavioral regressions: none. No controller, baseline,
  estimator, metric, confidence interval, or saved result changed; only figure
  sizing, presentation, and the pipeline's raster-resolution default changed.
  The mathematical reference therefore remains current.

## Side-by-side single-column metric revision

- Files changed for this revision:
  `evaluation/plot_four_method_comparison.py`,
  `tests/test_four_method_comparison_pipeline.py`, `README.md`, the regenerated
  `docs/assets/four_method_comparison_milestone.png`, and this status file.
- In response to the preferred paper layout, the coverage-effectiveness and
  locational-cost histories now occupy one horizontal row on the same exact
  3.5-inch single-column canvas. The canvas height is 3.0 inches. The long
  coverage title wraps onto two lines, while the global title is shortened to
  `Paired A3/G7 comparison`.
- The redundant per-axis legends were replaced by one shared two-column legend
  below the panels. All four method labels, curves, confidence bands, axis
  meanings, colors, and line styles are unchanged. Saving retains the exact
  physical canvas rather than expanding a tight bounding box.
- The deterministic figure contract was observed failing against the vertical
  arrangement and now asserts left-to-right axis placement plus one shared
  figure legend. The real 30-second milestone preview was regenerated at
  1050-by-900 pixels and inspected at original resolution; both panels, the
  wrapped title, axes, curves, bands, and legend are readable and unclipped.
- Numerical and behavioral regressions: none. This is an offline layout-only
  revision; no metric, confidence interval, archive, controller, estimator, or
  trajectory output changed.

## Title-free four-method metric revision

- Files changed for this revision:
  `evaluation/plot_four_method_comparison.py`,
  `tests/test_four_method_comparison_pipeline.py`, `README.md`, the regenerated
  `docs/assets/four_method_comparison_milestone.png`, and this status file.
- The side-by-side coverage/cost figure now omits its global title, both subplot
  titles, and both y-axis labels. The x-axis mission-time labels, numeric ticks,
  curves, confidence bands, and shared method legend remain unchanged. The
  trajectory figure and its four requested method titles are unchanged.
- The deterministic contract was first observed failing against the titled
  figure and now asserts the absence of a suptitle, subplot titles, and y-axis
  labels. The 1050-by-900 real-output preview was regenerated and inspected at
  original resolution; both data panels and the shared legend are readable and
  unclipped.
- Numerical and behavioral regressions: none. This is an offline presentation
  change only; all archived and plotted numerical values are unchanged.

## Mathematical-title four-method metric revision

- Files changed for this revision:
  `evaluation/plot_four_method_comparison.py`,
  `tests/test_four_method_comparison_pipeline.py`, `README.md`, the regenerated
  `docs/assets/four_method_comparison_milestone.png`, and this status file.
- The two side-by-side metric panels again have subplot titles, now exactly
  `$\mathcal{E}$` for coverage effectiveness and `$\mathcal{H}$` for
  locational cost. The global title and both y-axis labels remain absent. The
  shared legend, mission-time labels, curves, and confidence bands are
  unchanged.
- The configurable `--title-font-size` option was restored to the metric CLI
  and Python API. The deterministic figure contract asserts the exact
  math-text title strings and the continued absence of a suptitle and y-axis
  labels.
- The 1050-by-900 real-output preview was regenerated and visually inspected;
  both calligraphic titles render cleanly and the figure remains unclipped.
  Numerical and behavioral regressions: none; this is an offline labeling
  change only.

## Grayscale four-method trajectory revision

- Files changed for this revision:
  `evaluation/plot_four_method_trajectories.py`,
  `tests/test_four_method_comparison_pipeline.py`, the regenerated
  `docs/assets/four_method_trajectory_milestone.png`, and this status file.
- All four trajectory panels now render the shared hidden-truth density using
  the sequential Matplotlib `Greys` colormap instead of `magma`. A deterministic
  contract was first observed failing against the former colormap and now
  asserts `Greys` on every map axis.
- The cyan aerial and green ground trajectories remain unchanged. Because the
  former white ground-footprint outlines disappeared over the grayscale map's
  white low-density regions, those outlines now use mid-gray `#707070` at 0.8
  opacity; they remain visible over both light and dark field regions.
- The 1050-by-1200 real-output preview was regenerated and inspected at
  original resolution. Density peaks, trajectories, endpoint markers, sensing
  footprints, titles, and axes are distinct and unclipped. Three focused tests,
  Ruff lint and formatting, Python compilation, and `git diff --check` passed.
- Numerical and behavioral regressions: none. The saved truth normalization,
  trajectories, states, sensing radii, and all quantitative results are
  unchanged; this is an offline color-presentation change only.

## Editable tab-color trajectory revision

- Files changed for this revision:
  `evaluation/plot_four_method_trajectories.py`,
  `tests/test_four_method_comparison_pipeline.py`, `README.md`, the regenerated
  `docs/assets/four_method_trajectory_milestone.png`, and this status file.
- Aerial paths, start markers, end markers, and limited-range footprint circles
  now use `tab:blue`; ground paths and their start/end markers use `tab:green`.
  These replace the former high-saturation cyan and lime colors while retaining
  the sequential `Greys` truth background.
- All presentation colors are declared together near the beginning of the
  trajectory plotter as `AERIAL_COLOR`, `GROUND_COLOR`,
  `AERIAL_FOOTPRINT_COLOR`, `GROUND_FOOTPRINT_COLOR`,
  `ENDPOINT_EDGE_COLOR`, and `FIGURE_BACKGROUND_COLOR`. Editing those constants
  consistently restyles all four panels.
- The deterministic contract was first observed failing before the new color
  constants existed and now verifies that every map contains both configured
  trajectory colors. The 1050-by-1200 real-output preview was regenerated and
  inspected at original resolution; blue and green remain distinguishable from
  each other and from the grayscale field.
- Numerical and behavioral regressions: none. This is an offline color change;
  saved trajectories, states, footprints, truth, and metrics are unchanged.

## Offline selected-episode MP4 replay milestone

- Files changed for this milestone:
  `evaluation/render_multifidelity_episode_videos.py`,
  `tests/test_render_multifidelity_episode_videos.py`, `pyproject.toml`,
  `uv.lock`, `README.md`, `docs/mathematical_formulation.md`,
  `docs/assets/offline_episode_video_milestone.png`, and this status file.
- The new CLI reads an immutable `multifidelity_composition_raw` archive,
  selects an exact composition and zero-based episode, and never reruns the
  simulator, estimator, or controllers. It writes exactly seven independent
  H.264/YUV420P `.mp4` files: MFGP estimated controller density, hidden
  ground-truth density, NRMSE, KL, NLPD, locational cost $H$, and
  footprint-normalized effectiveness. Density videos contain current aerial
  and ground markers only; trajectory trails are intentionally absent.
- Saved state index `k + 1` is aligned with completed outer step `k`. The
  estimated-density video holds the latest posterior whose saved
  `posterior_step <= k`. NRMSE, KL, and NLPD reveal only actual asynchronous
  posterior records, while $H$ and effectiveness are evaluated for every
  post-step saved ground state using the repository's existing equations.
  The final completed step is always encoded even when `--frame-interval`
  does not divide the episode length.
- Aerial-only compositions are rejected because nearest-ground $H$ and
  ground-footprint effectiveness are undefined without a ground robot. The
  renderer does not invent surrogate values. `imageio-ffmpeg>=0.6.0` is locked
  so MP4 encoding works without a separately installed system ffmpeg;
  Matplotlib still performs the synchronous frame rendering and H.264 encode.
- Test-first validation began with the expected import failure because the new
  module did not exist. After implementation, the six dedicated contracts
  passed and verify CLI selection, direct execution, active-robot slicing,
  asynchronous posterior ordering, exact $H$ reuse, finite metric histories,
  seven MP4 names, frame subsampling/final-frame inclusion, raw-archive hash
  preservation, and the zero-ground rejection. The broader focused selection
  passed 35 tests in 12.91 s; the post-format dedicated rerun passed six tests
  in 2.28 s.
- Real encoder validation first generated seven four-frame MP4s from a
  deterministic four-step `A1/G1` smoke archive. It then replayed the existing
  30-episode archive at composition `A2/G8`, episode index 7 (seed 49), using
  `--frame-interval 50 --fps 5 --dpi 72`. All seven files were encoded with
  H.264 High profile and YUV420P: density videos are 518-by-432 pixels and
  metric videos are 576-by-324 pixels. They are available below
  `output/multifidelity_episode_video_A2_G8_episode_007/`. A frame extracted
  from the real estimated-density MP4 at saved step 150 was visually inspected
  for the MFGP field, both robot classes, obstacle overlay, fixed spatial
  limits, title, posterior version, and absence of trails; that frame is the
  milestone PNG.
- Commands executed included the initial failing and passing dedicated pytest
  runs; the 35-test focused composition/metric/trajectory selection; locked
  dependency sync; smoke and real episode-7 CLI renders; ffmpeg stream and
  frame inspection; scoped Ruff lint and format checks; Python compilation;
  `git diff --check`; repository-wide Ruff; and the complete pytest inventory
  split into bounded groups.
- Repository validation totals 286 passes and five unrelated failures already
  present in the dirty worktree: canonical 200 steps versus a stale 300-step
  assertion, cyan versus former orange final-plot ground color, the
  four-method plot's $\mathcal L$ title versus a test expecting $\mathcal H$,
  canonical/ablation ground-FOV mismatch, and missing `posterior_version=` in
  CLI progress text. Repository-wide Ruff reports 61 pre-existing violations
  in legacy files. The new module and test are Ruff-clean, Ruff-formatted,
  compile successfully, and pass `git diff --check`.
- Numerical and behavioral regressions: none introduced. No estimator,
  scheduler, sensor, controller, field, dynamics, or saved raw array changed.
  All new calculations and encoding are offline and read-only. NRMSE, KL,
  NLPD, $H$, and effectiveness reuse the exact existing implementations rather
  than defining alternative metrics.

## Four-method selected-episode MP4 replay milestone

- Files added or revised for this follow-up:
  `evaluation/render_four_method_episode_videos.py`,
  `tests/test_render_four_method_episode_videos.py`, `README.md`,
  `docs/mathematical_formulation.md`, this status file, and
  `docs/assets/four_method_offline_video_milestone.png`. The preceding
  composition-archive renderer remains available and unchanged.
- The preferred offline CLI now consumes the complete
  `output/four_method_comparison/` artifact directory and validates the four
  raw archives, evaluated archive, scenario fingerprints, paired scenario
  arrays, seeds, state times, method order, and requested zero-based episode.
  It reruns no simulation or controller.
- Seven synchronized H.264 MP4s are written. The proposed estimated-density
  video shows current MFGP robots over the latest causally available saved
  controller density. The ground-truth video shows a two-by-two panel for
  proposed MFGP-LR, Egerstedt-LR, full-range Egerstedt, and standard Zhang over
  their common hidden truth. All map panels show current markers only and no
  trajectory trails. NRMSE, KL, and NLPD are explicitly MFGP-only because the
  baseline archives publish no GP posterior. $H$ and footprint-normalized
  effectiveness use all four per-episode histories already stored in
  `evaluated.npz`.
- Six deterministic tests were added before implementation; their initial run
  failed at import because the module did not exist. They now pass and cover
  CLI defaults/direct execution, five-archive loading, paired method order,
  active state shapes, MFGP posterior selection, reconstruction histories,
  exact reuse of evaluated $H$/effectiveness arrays, seven `.mp4` destinations,
  common frame counts, final-frame forcing, and episode bounds.
- Real validation used the existing publication comparison episode index 7,
  seed 49, with `--frame-interval 75 --fps 5 --dpi 72`. The first run exposed
  an odd 345-pixel comparison-metric height after the first five encodes;
  libx264 correctly rejected that YUV420P frame. The metric figure was changed
  to an even 576-by-324 encoded size and the entire command was rerun. All
  seven outputs then passed stream inspection as H.264 High profile/YUV420P.
  The four-panel truth video is 662-by-576, the MFGP density video is
  518-by-432, and all five metric videos are 576-by-324.
- Real outputs are under
  `output/four_method_comparison/videos/episode_007/`. The milestone image was
  extracted from the synchronized truth video at completed step 225 and
  visually inspected: all four labels, three aerial and seven ground markers
  per panel, common density scale, spatial limits, time/episode title, and
  absence of trails are correct. The final $H$ frame was also inspected for
  all four labeled histories and the complete 30-second mission axis.
- Validation after formatting passed all 18 focused offline-renderer,
  reconstruction-metric, and baseline-evaluation tests in 6.71 s. The existing
  four-method pipeline suite retained its sole unrelated failure: production
  labels the cost panel `$\mathcal L$`, while the stale test expects
  `$\mathcal H$`; its other two tests passed. Scoped Ruff lint/format, Python
  compilation, and `git diff --check` passed.
- Episode-7 numerical diagnostics from the real saved data were: MFGP NRMSE
  `0.089154460737 -> 0.032626787285`, KL
  `0.726531099677 -> 1.905287588250`, and NLPD
  `-0.503446981200 -> -1.436482409090`. Final $H$ in declared method order was
  `66.7834775527, 98.3449360230, 83.6839785615, 92.2629668557`; final
  effectiveness was `0.822825628470, 0.865427065410, 0.792315146244,
  0.831554640526`. These are replay diagnostics, not altered evaluation
  results.
- Numerical and behavioral regressions: none. This is a read-only offline
  replay addition. Proposed reconstruction values and four-method comparison
  metrics use the existing saved arrays/equations without changing evaluation,
  estimation, scheduling, control, or trajectories.

### Four-method video presentation revision

- The seven-video interface now uses the ordered keys `estimated_density`,
  `density_comparison`, `nrmse`, `kl`, `nlpd`, `L`, and `effectiveness`.
  Locational-cost output is renamed `four_method_L.mp4`; the mixed-field panel
  output is `four_method_density_comparison.mp4`.
- Every dynamic subtitle containing episode, step, current value, or
  higher/lower-is-better text was removed. Density panels retain only static
  descriptive titles. Reconstruction plots retain their static MFGP metric
  titles. Locational cost is titled `Locational cost $\mathcal{L}$` and
  effectiveness is titled
  `Footprint-normalized effectiveness $\mathcal{E}$`; both intentionally have
  an empty y-axis label.
- In the synchronized two-by-two density comparison, the proposed panel now
  uses the latest causally available saved MFGP `posterior_density`. The three
  baseline panels continue to use the shared hidden `truth_density`, because
  their raw archives contain no GP estimate. All four field panels use a
  common zero-based color range whose maximum covers both truth and every
  selected-episode MFGP posterior, preventing clipping or per-panel rescaling.
- A deterministic contract first failed because the new field-selection
  helper did not exist. Seven contracts now verify the new `L` key/filename,
  density-comparison filename, proposed-posterior versus baseline-truth field
  selection, absence of forbidden subtitle fragments, exact $\mathcal L$ and
  $\mathcal E$ titles, and empty y labels in addition to all previous replay
  guarantees.
- The corrected real episode-7 outputs were rendered into
  `output/four_method_comparison/videos/episode_007_revised/` with
  `--frame-interval 75 --fps 5 --dpi 72`. All seven streams were inspected as
  H.264 High profile/YUV420P. The revised two-by-two comparison frame and the
  final $\mathcal L$ and $\mathcal E$ frames were visually inspected: the
  proposed MFGP estimate differs from the three truth backgrounds, titles are
  static, and neither comparison metric has a y-axis label. The deterministic
  summary at `docs/assets/four_method_offline_video_milestone.png` was updated
  from the revised density-comparison video.
- Focused replay, metric, and evaluation validation passed 19 tests in 6.36 s.
  The established four-method pipeline initially exposed its stale
  `$\mathcal H$` test expectation; after updating that presentation contract
  to the requested `$\mathcal L$`, the pipeline and video contracts passed 10
  tests in 5.98 s. Scoped Ruff lint/format, Python compilation, and
  `git diff --check` passed.
- Numerical and behavioral regressions: none. The revision changes only
  offline field selection and presentation; it does not mutate raw archives,
  estimator state, metric arrays, controllers, commands, or trajectories.

### Four-method static density and trajectory companions

- Files revised for this follow-up:
  `evaluation/render_four_method_episode_videos.py`,
  `tests/test_render_four_method_episode_videos.py`, `README.md`,
  `docs/mathematical_formulation.md`, and this status file.
- Every normal four-method video render now also writes four PNG companions:
  `ground_truth_density.png`, `mfgp_trajectories_estimated_density.png`, and
  `mfgp_trajectories_variance.png`, plus
  `mfgp_trajectories_density_difference.png`. A separate Python entry point can
  render only these images without re-encoding the MP4s.
- The truth image is deliberately uncluttered. All three trajectory images use all
  saved proposed aerial and ground positions, distinct class colors and line
  styles, start crosses, and final robot markers without inferred path data.
  Their backgrounds are the final causally available saved MFGP controller
  density and raw latent HIGH posterior variance. The truth and estimate share
  the evolving comparison video's density normalization; variance has its own
  zero-based scale.
- Two deterministic contracts were written first and failed at import because
  the image API did not exist. The expanded nine-test renderer contract now
  checks exact truth/final-density/final-variance array selection, four PNG
  names and nonempty raster files, and automatic PNG creation alongside the
  unchanged seven MP4 outputs.
- Real episode-7 companions were generated at 150 DPI under
  `output/four_method_comparison/videos/episode_007_revised/`. All four are
  1080-by-900 RGBA PNGs. Original-resolution inspection confirmed the common
  density scale, uncluttered truth reference, complete proposed paths and
  endpoint markers, readable obstacle overlay, and visible final latent
  variance structure. A targeted pixel crop additionally confirmed that the
  estimated-density figure's axes and colorbar are present in the file despite
  the initial multi-image preview viewport clipping them.
- The final combined selection passed all 28 four-method pipeline, four-method
  renderer, homogeneous renderer, original replay renderer, and metric tests in
  10.28 s. Scoped Ruff lint/format, Python compilation, and `git diff --check`
  passed.
- Numerical and behavioral regressions: none. All four images use saved arrays
  only and do not change or rerun the estimator, controllers, robot motion,
  metrics, video frames, or raw/evaluated archives.

#### Density, variance, and signed-error colormap revision

- All four-method density video panels and static density images now use
  Matplotlib `YlOrRd`, so increasing probability density progresses toward red.
  The final MFGP variance companion uses `Greys`, so large posterior variance
  and therefore less-explored regions are darker.
- The fourth trajectory companion displays the exact signed difference
  `posterior_density[-1] - truth_density`. Its zero-centered symmetric
  `RdBu_r` scale renders underestimation blue, agreement white, and
  overestimation red without favoring either sign.
- The deterministic contract first failed because the palette constants and
  signed-difference output did not exist. It now verifies the exact field
  subtraction, requested density/variance colormaps, chosen difference
  colormap, and fourth PNG filename. All nine focused renderer tests pass.
- The complete episode-7 output directory was regenerated with
  `--frame-interval 75 --fps 5 --dpi 72`, followed by 150-DPI regeneration of
  the four static companions. The four-method density-comparison video frame
  and all companion images were inspected at original resolution. Density
  peaks are red, high-variance unsampled regions are dark, and the signed error
  has a visually neutral zero with balanced blue/red limits. All PNGs are
  1080-by-900 RGBA files.
- The combined replay/pipeline selection passed 28 tests in 11.31 s. The
  post-format focused renderer rerun, scoped Ruff lint/format, Python
  compilation, and `git diff --check` also passed.
- Numerical and behavioral regressions: none. The change affects colormap
  presentation and adds a deterministic subtraction of two saved normalized
  density arrays; it does not change estimation, normalization, variance,
  controllers, trajectories, metrics, or archives.

#### Live variance and spatial reconstruction-error videos

- The normal four-method selected-episode replay now produces nine MP4s. New
  files `mfgp_variance.mp4` and `mfgp_density_difference.mp4` place current
  proposed-method robot markers over the latest causally available saved MFGP
  latent HIGH variance and signed normalized-density error, respectively.
- Both videos use fixed episode-wide color limits. Variance uses zero through
  the maximum of all selected posterior-variance records. Signed error uses
  symmetric limits based on the maximum absolute `posterior_density -
  truth_density` value over every selected posterior. This prevents apparent
  color changes caused by per-frame rescaling.
- One new deterministic contract was written first and failed because the
  causal spatial-field helper did not exist. The ten focused renderer tests now
  verify latest-posterior density, variance, and signed-error selection; nine
  synchronized MP4 destinations and frame counts; and the two new filenames.
- The complete episode-7 replay was regenerated with `--frame-interval 75
  --fps 5 --dpi 72`. Both new files are 518-by-432 H.264 High profile/YUV420P
  streams at 5 fps. Early and later frames from each video were extracted and
  inspected: robot markers advance, variance changes from the initial posterior
  to explored low-variance regions, and the signed spatial error changes while
  retaining one balanced episode-wide color scale.
- The final combined four-method pipeline, four-method renderer, homogeneous
  renderer, original replay renderer, and metric selection passed 29 tests in
  12.70 s. Scoped Ruff lint/format, Python compilation, and `git diff --check`
  passed.
- Numerical and behavioral regressions: none. The videos read saved posterior
  arrays and robot states only; they do not alter the estimator, controller,
  scheduler, metrics, trajectories, or archives.

#### Combined four-panel spatial reconstruction video

- The four-method replay now emits a tenth synchronized MP4 named
  `mfgp_spatial_reconstruction_comparison.mp4`. Its fixed layout is ground truth
  at top left, latest causal MFGP density at top right, latest causal latent
  HIGH variance at bottom left, and latest causal signed density error at bottom
  right.
- Every panel overlays the identical current proposed aerial and ground robot
  markers without trails. The top row shares one episode-wide `Reds` density
  scale; the lower panels reuse the standalone `Greys` variance and symmetric
  `RdBu_r` error scales. Three colorbars state the corresponding units.
- The deterministic contract first failed because the combined-field helper did
  not exist. It now verifies exact panel ordering and array selection, the ten
  synchronized MP4 writers, the output filename, and all four static panel
  titles.
- Real episode-7 rendering produced the combined video at 662-by-576 pixels,
  5 fps, H.264 High profile/YUV420P. An extracted middle frame was inspected:
  panel order and titles are correct, current robot positions match across all
  four axes, and the shared density plus separate variance/error colorbars are
  readable without clipping.
- The final relevant four-method pipeline, four-method renderer, homogeneous
  renderer, original replay renderer, and metric selection passed 29 tests in
  16.27 s. Scoped Ruff lint/format, Python compilation, and `git diff --check`
  passed.
- Numerical and behavioral regressions: none. This is a synchronized offline
  composition of already validated fields and states; no simulation or saved
  value changes.

#### Paper-consistent robot colors in replay outputs

- Every robot overlay emitted by
  `evaluation/render_four_method_episode_videos.py` now imports the paper
  trajectory plotter's palette constants directly: aerial robots and paths use
  `tab:orange`, and ground robots and paths use `tab:green`.
- This applies consistently to the estimated-density, variance, signed-error,
  combined spatial, and four-method density-comparison videos, plus all static
  trajectory companion PNGs. Metric-series colors are unchanged.
- A deterministic contract first failed against the former blue/orange replay
  palette and now verifies the exact paper constants. The focused ten-test
  renderer suite passes. The working tree's current `Reds` density colormap was
  preserved; this revision changes robot presentation only.
- All ten episode-7 MP4s and four companion PNGs were regenerated. An extracted
  combined-video frame and the variance trajectory PNG were inspected at
  original resolution; aerial markers/paths are orange and ground
  markers/paths are green in both moving and accumulated-trajectory views.
- The final relevant five-suite selection passed 29 tests in 15.08 s. Scoped
  Ruff lint/format, Python compilation, and `git diff --check` passed.
- Numerical and behavioral regressions: none. Only plotting constants changed;
  saved fields, posteriors, robot states, metrics, and encoded frame timing are
  unchanged.

## Type-3-free publication PDFs for Figures 3--5

- The affected assets in `paper/updated.tex` were identified as Figure 3
  `paper/pics/error_metrics.pdf`, Figure 4
  `paper/pics/coverage_locational_cost.pdf`, and Figure 5
  `paper/pics/four_method_trajectories.pdf`. Before this milestone, `pdffonts`
  reported DejaVu Sans, DejaVu Sans Oblique, and STIX math glyphs as embedded
  Type 3 fonts in these files.
- Added `evaluation/publication_fonts.py`, which configures Matplotlib PDF and
  PostScript output with font type 42. The three source plotters invoke this
  shared policy before importing `matplotlib.pyplot`, including when executed
  directly as scripts. This embeds TrueType outlines while retaining vector
  curves, text, math text, confidence bands, and map content.
- Added `tests/test_publication_pdf_fonts.py`. Its deterministic contract checks
  the PDF/PS settings and generates a real PDF containing ordinary and
  calligraphic math text, then requires a `/FontFile2` TrueType stream and
  rejects any `/Subtype /Type3` font dictionary.
- Figures 3--5 were regenerated from the existing real evaluated/raw archives
  using the documented commands. `pdffonts` now reports only embedded
  `CID TrueType` for all fonts in the three assets; no Type 3 entry remains.
  Rasterized previews of all three regenerated PDFs were inspected: subplot
  layouts, mathematical titles, legends, axes, confidence bands, density map,
  and trajectories are readable and unclipped.
- `paper/updated.tex` compiled successfully with `latexmk -pdf
  -interaction=nonstopmode -halt-on-error updated.tex`. The final
  `paper/updated.pdf` font audit reports only embedded Type 1 and CID TrueType
  fonts and prints `NO_TYPE3_FONTS`. The compile retains existing non-blocking
  warnings about the absent author, underfull boxes, and the paper's figure
  references during the first pass; the final pass completed successfully.
- The relevant font, composition, and four-method plot suites passed 24 tests
  in 12.35 s. All three direct `--help` entry points passed. Scoped Ruff
  lint/format, Python compilation, and `git diff --check` passed after the one
  import-order annotation correction.
- Numerical and behavioral regressions: none. Only PDF/PS font embedding and
  regenerated paper assets changed. Plot data, equations, layout parameters,
  metrics, confidence intervals, trajectories, and simulation behavior are
  unchanged. `docs/mathematical_formulation.md` remains current because no
  mathematical implementation changed.

## Homogeneous-baseline selected-episode MP4 replay milestone

- Files added or revised for this milestone:
  `evaluation/render_homogeneous_baseline_episode_videos.py`, the reusable
  reconstruction-only path in `evaluation/render_multifidelity_episode_videos.py`,
  `tests/test_render_homogeneous_baseline_episode_videos.py`, `README.md`,
  `docs/mathematical_formulation.md`, and this status file.
- The new offline CLI selects one saved heterogeneous composition and
  automatically locates the unique equal-total-size aerial-only and ground-only
  endpoints. It validates their episode, seed, mission clock, query grid,
  integration weights, HIGH truth, normalized truth density, free-space mask,
  and obstacle map before comparing them.
- Five H.264 MP4s are produced: three-panel estimated-density and shared-truth
  comparisons plus progressively revealed NRMSE, KL, and NLPD histories. Every
  estimated panel and curve uses that composition's own saved asynchronous GP
  posterior. Titles are static, current robot markers are shown without trails,
  and metric plots retain a synchronized mission-time cursor.
- $\mathcal L$ and $\mathcal E$ are intentionally excluded because the
  aerial-only endpoint has no ground positions or sensing footprints. The
  existing seven-video single-composition API still requires a ground robot;
  only its reconstruction calculation was factored into a reusable function.
- Seven deterministic contracts were written first and initially failed at
  import because the renderer did not exist. They cover CLI execution, exact
  endpoint selection and pairing, zero-sized inactive robot arrays, per-method
  asynchronous metrics, latest-posterior field selection, shared truth,
  five static-titled MP4 destinations, common frame counts, raw-archive
  immutability, and rejection of a homogeneous proposed selection. Together
  with the original renderer contracts, 13 tests passed before formatting.
- Real validation replayed publication composition `A2/G8`, episode index 7,
  against `A10/G0` and `A0/G10` with `--frame-interval 75 --fps 5 --dpi 72`.
  Outputs are under
  `output/homogeneous_baseline_videos/A2_G8/episode_007/`. All five streams are
  H.264 High profile/YUV420P at 5 fps; density comparisons are 864-by-302 and
  metrics are 576-by-324 pixels. Original-resolution inspection first exposed
  a clipped right-panel suffix; per-panel titles were shortened to the complete
  method/composition label and the five videos were regenerated. The final
  density and NRMSE frames are readable, synchronized, statically titled, and
  contain no trajectory trails.
- Episode-7 saved-posterior diagnostics in declared order (heterogeneous,
  aerial-only, ground-only) were: NRMSE `0.096261755436 -> 0.085666802166`,
  `0.180268949286 -> 0.124952014616`, and
  `0.196079170359 -> 0.070156490830`; KL
  `0.975823183108 -> 1.712097479120`,
  `7.102790928240 -> 2.592007604160`, and
  `9.506159673040 -> 2.006572900520`; NLPD
  `0.483891688801 -> 0.090652553122`,
  `0.485502057962 -> 0.461615737436`, and
  `0.525236005464 -> 0.155836848381`. These are replay diagnostics, not new or
  altered evaluation data.
- The final focused selection passed all 35 composition-pipeline, metric,
  original-renderer, and new-renderer tests in 14.90 s. Scoped Ruff lint and
  formatting, Python compilation, and `git diff --check` passed.
- Numerical and behavioral regressions: none. The renderer reads immutable raw
  arrays and does not rerun or mutate simulation, estimation, scheduling,
  controllers, fields, robot states, or published posteriors.
