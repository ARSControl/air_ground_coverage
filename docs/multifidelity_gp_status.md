# Multi-Fidelity GP Implementation Status

## Current Milestone

The separate comparison-trajectory plotting milestone is **complete and
verified on 2026-08-02**. One command now reads the paired proposed-method and
Egerstedt archives and writes one independent plot into each method's result
folder. Both files use the same episode, hidden-truth background, density color
scale, axes, canvas dimensions, and endpoint conventions. No production or
baseline control behavior changed.

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
