# Central Multi-Fidelity GP Implementation Plan

Architecture diagrams:

- current verified baseline: `docs/architecture/current_architecture.md`;
- proposed target pending approval:
  `docs/architecture/target_multifidelity_architecture.md`.

## 1. Purpose and approval boundary

This document plans the implementation of the central asynchronous
autoregressive multi-fidelity Gaussian Process described in
`docs/multifidelity_gp_design.md` and grounded in the repository audit in
`docs/multifidelity_gp_repo_audit.md`.

This is a plan only. No production code is to be changed until this plan has
been reviewed and approved.

The implementation remains Python-only. “Asynchronous” means independently
scheduled events in deterministic simulated time; it does not mean threads,
`asyncio`, multiprocessing, networking, ROS, or external middleware.

## 2. Planned architecture

### 2.1 New modules

| File | Planned responsibility |
| --- | --- |
| `src/core/multifidelity_gp.py` | Pure autoregressive GP mathematics, kernels, validation, Cholesky factorization, and high-fidelity prediction. No robot, controller, plotting, or experiment imports. |
| `src/core/observations.py` | Fidelity enum, immutable timestamped observation records, deterministic retention policy, and transactional bounded buffers. |
| `src/core/density.py` | Positive-part clipping, weighted density normalization, aerial target construction, density validation, and structured-grid resampling. Softplus was later removed by user-approved refinement. |
| `src/core/multifidelity_estimator.py` | Central buffered estimator, cached posterior snapshots, update reports, failure preservation, and timing/sample metadata. |
| `src/models/sensors.py` | Multifidelity-only simulated scalar-field sensors and construction of low/high synthetic truth fields. Legacy agent sensor methods remain unchanged. |
| `src/simulation.py` | Deterministic periodic scheduling and a simulation adapter that owns exactly one central estimator and the fidelity sensors. It does not implement either control law. |
| `src/coupled_config.py` | Load one self-contained coupled YAML file and resolve shared settings plus aerial/ground overrides into the two existing controller parameter views. |
| `src/coupled_simulation.py` | Reusable coupled-simulation setup and phased aerial/ground loop used only by the new entry points. It calls existing HEDAC and MPC laws without redefining them. |
| `src/utils/multifidelity_logging.py` | Evaluation-only history collection and NPZ serialization for posterior, density, timing, sample-count, and trajectory outputs. |

### 2.2 Existing integration seams

- New `examples/run_multifidelity.py` becomes the canonical coupled simulation
  entry point for the new architecture.
- New `evaluation/run_multifidelity_evaluation.py` becomes the canonical
  repeated multifidelity evaluation entry point.
- Existing `examples/hierarchical.py`, `evaluation/run_evaluation.py`, baseline
  scripts, and `evaluation/run_ablation.py` remain unchanged as executable
  legacy references.
- `HEDACAlgorithm.current_goal_density` remains the aerial controller's density
  seam. A backward-compatible optional external-target path will be added to
  `HEDACAlgorithm.step()`.
- The CasADi solver parameter `W` remains the ground controller's density seam.
  Only the vector used to populate `W` changes in multifidelity mode.
- `src/core/GaussianProcess.py` remains untouched as the legacy estimator.
- The existing post-hoc fusion expression remains available in legacy mode.

### 2.3 Planned ownership

```text
coupled simulation / evaluation loop
  -> MultifidelitySimulationCoordinator
       -> one CentralAsynchronousEstimator
            -> one MultiFidelityGaussianProcess
            -> one LOW buffer
            -> one HIGH buffer
       -> one aerial LOW sensor schedule
       -> one ground HIGH sensor schedule
       -> one GP update schedule
  -> HEDACAlgorithm (density consumer only in multifidelity mode)
  -> existing ground MPC solver (density consumer only)
```

No robot owns an estimator, and no independent multifidelity GP is created per
robot.

## 3. Decisions proposed for review

These choices resolve the ambiguities identified by the audit. They should be
approved before implementation begins.

1. **Canonical mode key:** use top-level `estimator_mode` with values `legacy`
   and `multifidelity`. A missing key resolves to `legacy`.
2. **Canonical coupled host:** implement the new architecture in
   `src/coupled_simulation.py`, expose it through new
   `examples/run_multifidelity.py` and
   `evaluation/run_multifidelity_evaluation.py`, and later add a new
   `evaluation/run_multifidelity_ablation.py`. Keep the existing hierarchical,
   evaluation, baseline, and ablation scripts untouched as frozen legacy
   references.
3. **Latent truth:** treat the current GMM field as simulated `f_H`. Construct
   a deterministic broad `f_L` by Gaussian smoothing of `f_H`, and define the
   reference discrepancy as `delta = f_H - rho * f_L`. Aerial sensors sample
   `f_L`; ground sensors sample `f_H`.
4. **Fixed hyperparameters initially:** `rho`, low-kernel parameters, and
   discrepancy-kernel parameters are configured and not optimized. Legacy
   scikit-learn optimization remains unchanged.
   **Post-Milestone-7 update (2026-07-16):** following explicit user approval,
   the central estimator now optionally fits the four RBF parameters on a
   slower configured schedule. `rho` and observation-noise variances remain
   fixed. See `docs/architecture/implementation_hyperparameter_fitting.md`.
5. **Kernel:** use an isotropic squared-exponential/RBF kernel with explicit
   signal variance and length scale for both processes. The API leaves room for
   other kernels later without adding them now.
6. **Means:** use zero GP prior means in the first implementation. Density
   positivity is handled separately by positive-part clipping (superseding the
   originally planned stable softplus).
7. **Timing units:** configure sensor and estimator periods in simulated
   seconds. Scheduling uses a monotonic next-fire time with a small numerical
   tolerance, so periods need not be integer multiples of `dt`.
8. **Team ordering:** preserve the current aerial-then-ground controller order
   and each team's sense-before-own-motion behavior. At a step, collect due LOW
   data before aerial motion; after aerial motion, collect due HIGH data and run
   a due estimator update before ground control. Thus a ground observation can
   affect ground control at that update and aerial control on the following
   aerial step. No integration rate changes.
9. **Retention:** maintain separate LOW and HIGH stores. Within each fidelity,
   sort candidates newest-first, retain a point only when it is at least the
   configured separation from already retained newer points, cap the count,
   then expose retained data in deterministic chronological order. Thus a newer
   same-fidelity duplicate replaces an older one; LOW and HIGH observations at
   the same position coexist in different stores.
10. **Failed update transaction:** candidate observations are not committed and
    the latest posterior is not replaced when fit, prediction, validation, or
    normalization fails. The update report records failure. Pending observations
    remain available for explicit retry or administrative rejection.
11. **Estimator query grid:** use the existing ground MPC grid in the coupled
    experiment because it is the highest-resolution controller grid already
    present. Resample the cached aerial target to the HEDAC map. Grid points,
    shape, and numerical integration weights are posterior metadata, never
    hard-coded in the GP core.
12. **Numerical integral:** normalize densities with precomputed nonnegative
    integration weights for the configured structured grid. Tests check the
    weighted integral, not only array sum.
13. **Pre-posterior behavior:** both controllers keep their current initial or
    legacy density until the first valid multifidelity posterior exists.
14. **Obstacle handling:** multifidelity densities are zeroed at invalid cells
    before normalization when a compatible mask is supplied. Legacy behavior is
    unchanged.
15. **Randomness:** new sensors receive a dedicated seeded
    `numpy.random.Generator`; they do not consume the global RNG used by legacy
    code. This permits identical legacy comparisons.

## 4. Configuration contract

The planned coupled configuration is:

```yaml
estimator_mode: legacy  # legacy | multifidelity

multifidelity:
  rho: 0.8
  gp_update_period: 1.0
  aerial_sensor_period: 0.5
  ground_sensor_period: 0.1

  low_kernel:
    length_scale: 5.0
    variance: 1.0

  discrepancy_kernel:
    length_scale: 1.5
    variance: 0.25

  low_noise_variance: 0.04
  high_noise_variance: 0.0001
  jitter: 1.0e-8
  max_jitter_attempts: 5
  jitter_multiplier: 10.0

  hyperparameter_optimization:
    enabled: true
    fit_interval_updates: 5
    min_samples: 40
    num_restarts: 1
    max_iterations: 75
    bounds:
      low_length_scale: [1.0, 20.0]
      low_variance: [0.05, 5.0]
      discrepancy_length_scale: [0.5, 10.0]
      discrepancy_variance: [0.01, 2.0]

  retention:
    max_low_samples: 250
    max_high_samples: 250
    min_low_separation: 0.25
    min_high_separation: 0.10

  aerial_target:
    lambda_interest: 1.0
    lambda_uncertainty: 0.25

  density:
    normalization_tolerance: 1.0e-8

  sensor:
    low_fidelity_smoothing_sigma_cells: 2.0
    random_seed_offset: 10000
```

The numerical values are initial deterministic defaults for review, not claimed
to be tuned. `low_noise_variance` and `high_noise_variance` are variances; the
legacy `gpr.obs_noise_std` settings remain standard deviations and are not
reinterpreted.

Validation rules will require finite values, `rho` finite, positive kernel
length scales/variances, nonnegative observation variances, positive periods,
positive sample limits, nonnegative minimum separations and target weights,
at least one positive aerial target weight, positive jitter, and positive
normalization tolerance.

## 5. Milestone execution protocol

Every implementation milestone is gated as follows:

1. Make only the files listed for that milestone, plus unavoidable formatting
   changes.
2. Run the milestone-specific tests.
3. Run every accumulated test.
4. Run static checks on touched files.
5. Stop immediately on failure, report it, and do not continue silently.
6. Update `docs/multifidelity_gp_status.md` with files changed, commands,
   results, numerical checks, regressions, runtime observations, known issues,
   and next milestone.
7. Review the diff before starting the next milestone.

Because the repository currently has no tests, “existing tests” in Milestone 1
means none. From Milestone 2 onward, every previously added test is an existing
test that must be rerun.

Before the first implementation milestone, run these read-only preflight checks:

```bash
git status --short
uv lock --check
python -m pytest --collect-only -q
```

The audit predicts that the lock check may fail because `pyproject.toml` and
`uv.lock` differ. If so, record it and obtain approval for a dependency-only
lock synchronization before relying on `uv run --frozen`; do not fold unrelated
dependency churn into a GP milestone.

## 6. Milestone 1 — Pure autoregressive GP mathematical core

### Objective

Introduce only the controller-independent mathematical implementation of

```text
f_H(q) = rho * f_L(q) + delta(q)
```

with full LOW/HIGH joint covariance and high-fidelity posterior prediction.

### Exact files

- Add `src/core/multifidelity_gp.py`.
- Modify `src/core/__init__.py` to export the new public mathematical types.
- Add `tests/test_multifidelity_gp_contract.py` for API validation and empty-data
  behavior.
- Create `docs/multifidelity_gp_status.md` from the existing template and record
  the milestone.

Do not modify `src/core/GaussianProcess.py`, robots, HEDAC, MPC, examples,
evaluation, or configuration files.

### Classes and functions

- `RBFKernel(length_scale: float, variance: float)`
  - immutable configuration dataclass;
  - `covariance(x_left, x_right) -> np.ndarray`;
  - `diagonal(x) -> np.ndarray`.
- `HighFidelityPrediction`
  - immutable result with one-dimensional `mean` and `variance` arrays.
- `MultiFidelityGaussianProcess`
  - constructor accepts `rho`, low kernel, discrepancy kernel, `jitter`,
    `max_jitter_attempts`, and `jitter_multiplier`;
  - `fit(low_positions, low_values, low_noise_variances, high_positions,
    high_values, high_noise_variances) -> MultiFidelityGaussianProcess`;
  - `predict_high(query_positions) -> HighFidelityPrediction`;
  - read-only `n_low`, `n_high`, and `effective_jitter` properties.

Private helpers validate arrays, construct the four training covariance blocks,
construct high-to-training cross-covariance, perform adaptive-jitter Cholesky,
and commit fitted state only after all computations succeed.

### Public interface and numerical behavior

- Positions use shape `(N, 2)` and `(x, y)` order.
- Values and noise variances accept shape `(N,)` only.
- Scalar noise variances may be expanded by a private validation helper, but the
  stored representation is per-observation.
- Training vector order is `[y_LOW, y_HIGH]`.
- The implementation uses `numpy.linalg.cholesky` and
  `scipy.linalg.solve_triangular`; it performs no explicit inverse.
- Empty LOW or empty HIGH data are valid. Both empty returns the high-fidelity
  prior on prediction. Prediction before `fit()` is allowed only as the explicit
  empty-data prior, not as an ambiguous unfitted state.
- Marginal variance is symmetrized numerically, tiny negative roundoff is
  clipped to zero, and materially negative or nonfinite variance raises.
- Failed `fit()` leaves the preceding fitted state unchanged.

### Configuration parameters

No YAML is read in this milestone. The constructor-level parameters correspond
to the future `rho`, kernel, jitter, and jitter-escalation configuration.

### Tests to add

`tests/test_multifidelity_gp_contract.py` will verify:

- accepted shapes and deterministic output shapes;
- rejection of wrong dimensions, NaN/Inf, negative noise, invalid kernel values,
  and invalid query points;
- LOW-only, HIGH-only, and both-empty calls do not crash;
- both-empty prediction equals the analytical high-fidelity prior variance;
- no fitted state is replaced after an induced failed fit;
- source inspection or a focused review check finds no `np.linalg.inv` or
  `numpy.linalg.inv` in the new production module.

### Existing tests to rerun

None exist before this milestone.

### Validation commands

```bash
python -m pytest -q tests/test_multifidelity_gp_contract.py
python -m pytest -q
python -m ruff check src/core/multifidelity_gp.py tests/test_multifidelity_gp_contract.py
git diff --check
```

### Expected behavior

The module can fit any valid combination of LOW and HIGH observations and return
finite high-fidelity posterior mean/variance without importing simulation or
controller code. Nothing in the running repository uses it yet.

### Likely risks

- LOW/HIGH block orientation or cross-covariance transposition errors.
- Incorrect cross-covariance for HIGH predictions.
- Accidental mutation of the last good factorization on failure.
- Treating signal variance as standard deviation.
- Over-aggressive clipping hiding a real negative-variance defect.

### Rollback strategy

Remove the new module/export/contract test and revert only the status entry. No
existing runtime path references the core, so rollback has no behavioral effect.

## 7. Milestone 2 — Deterministic numerical unit tests

### Objective

Establish mathematical correctness before the core is used by buffers,
estimators, sensors, or controllers.

### Exact files

- Add `tests/conftest.py` with deterministic broad LOW, narrow discrepancy, and
  autoregressive HIGH synthetic-field fixtures.
- Add `tests/test_multifidelity_gp.py`.
- Modify `docs/multifidelity_gp_status.md`.

No production file changes are planned. A core defect discovered by these tests
must be fixed in `src/core/multifidelity_gp.py` within this milestone and called
out explicitly in the status report.

### Test helpers and interfaces

- No public production interface changes in this milestone.
- `synthetic_multifidelity_field` fixture returns callables or fixed arrays for
  `f_L`, `delta`, and `f_H = rho * f_L + delta`.
- `direct_joint_reference(...)` assembles the designed covariance independently
  and uses a general linear solve, not production helpers.
- Fixed sampling/query positions are checked into the tests; randomness, when
  used, comes from `np.random.default_rng(fixed_seed)`.

### Configuration parameters

Tests use explicit local constants for `rho`, kernel parameters, noise
variances, and jitter. No YAML is introduced.

### Tests to add

`tests/test_multifidelity_gp.py` will verify:

- LOW observations recover the broad structure with a defined RMSE/correlation
  threshold;
- a later HIGH observation creates a local correction toward `f_H` that LOW
  data alone cannot create;
- posterior variance decreases near LOW and HIGH observations in the expected
  high-fidelity covariance channels;
- mean and variance match `direct_joint_reference` to strict tolerances on a
  small dataset;
- variances remain finite and nonnegative on a grid;
- duplicate and nearly coincident points remain stable with jitter;
- LOW-only and HIGH-only predictions match their analytical covariance cases;
- repeated fixed-seed runs are bitwise equal where practical, otherwise equal
  to a documented numerical tolerance;
- changing `rho` changes LOW-to-HIGH influence as expected;
- discrepancy length scale controls locality of a HIGH correction.

### Existing tests to rerun

- `tests/test_multifidelity_gp_contract.py`.

### Validation commands

```bash
python -m pytest -q tests/test_multifidelity_gp_contract.py tests/test_multifidelity_gp.py
python -m pytest -q
python -m ruff check tests/conftest.py tests/test_multifidelity_gp.py
git diff --check
```

### Expected behavior

The mathematical core has independent reference agreement and demonstrates the
broad-LOW/local-HIGH behavior needed to justify later integration.

### Likely risks

- Tests that only duplicate production formulas instead of independently
  checking them.
- Brittle thresholds that depend on BLAS details.
- A synthetic discrepancy too weak or too broad to prove local correction.
- Conflating posterior latent variance with observation-noise variance.

### Rollback strategy

Revert the test fixtures/tests and any narrowly documented core correction. Do
not proceed to Milestone 3 until reference agreement is restored.

## 8. Milestone 3 — Timestamped observations and bounded buffers

### Objective

Introduce immutable observation metadata and deterministic, separate LOW/HIGH
retention without constructing an estimator or modifying simulation code.

### Exact files

- Add `src/core/observations.py`.
- Modify `src/core/__init__.py` to export observation types.
- Add `tests/test_observations.py`.
- Modify `docs/multifidelity_gp_status.md`.

### Classes and functions

- `Fidelity(Enum)` with `LOW` and `HIGH`.
- `Observation`
  - frozen dataclass fields: `timestamp`, `robot_id`, `position` as a two-float
    tuple, `value`, `fidelity`, and `noise_variance`;
  - construction validates finite timestamp/position/value, nonempty ID, enum
    fidelity, and finite nonnegative noise variance.
- `RetentionConfig(max_samples, min_separation)` frozen dataclass.
- `SpatialAgeRetentionPolicy.select(entries, config) -> tuple[Observation, ...]`.
- `BoundedObservationBuffer`
  - constructor fixes one fidelity and retention policy;
  - `submit(observation) -> None` and `submit_many(observations) -> int`;
  - `candidate() -> tuple[Observation, ...]` returns retained plus pending after
    deterministic selection without committing;
  - `commit(candidate) -> None` replaces retained data and clears incorporated
    pending data;
  - `retained`, `pending_count`, and `fidelity` read-only properties.

An internal monotonic insertion index supplies deterministic tie-breaking for
equal timestamps without becoming part of the public observation schema.

### Public interface and behavior

- LOW observations cannot be submitted to a HIGH buffer or vice versa.
- Arrival order and collection timestamp are distinct: submission order records
  arrival while retention priority uses collection timestamp then insertion
  order.
- Newer spatial duplicates replace older same-fidelity data.
- Out-of-order timestamps are accepted and sorted deterministically.
- Candidate/commit separation supports estimator-level transactional updates.
- No wall-clock calls or concurrency primitives are introduced.

### Configuration parameters

Constructor values correspond to future:

- `multifidelity.retention.max_low_samples`;
- `multifidelity.retention.max_high_samples`;
- `multifidelity.retention.min_low_separation`;
- `multifidelity.retention.min_high_separation`.

### Tests to add

- valid observation construction and immutable fields;
- rejection of NaN/Inf, malformed position, invalid variance, and invalid ID;
- fidelity mismatch rejection;
- separate LOW/HIGH storage;
- arrival out of timestamp order;
- deterministic equal-timestamp tie-breaking;
- newest duplicate replacement;
- minimum-separation behavior;
- maximum-count behavior;
- candidate does not mutate retained state;
- commit clears only incorporated pending entries;
- repeated sequences produce identical retained records.

### Existing tests to rerun

- All Milestone 1 and 2 GP tests.

### Validation commands

```bash
python -m pytest -q tests/test_observations.py
python -m pytest -q
python -m ruff check src/core/observations.py tests/test_observations.py
git diff --check
```

### Expected behavior

The repository can represent independently arriving LOW/HIGH data with all
required metadata and bounded deterministic retention, but no GP update occurs.

### Likely risks

- Frozen dataclasses containing mutable arrays; use a coordinate tuple to avoid
  this.
- Ambiguous timestamp versus arrival ordering.
- Greedy spatial retention depending on input order.
- Failed estimator updates later losing pending data if commit semantics are not
  truly transactional.

### Rollback strategy

Remove the module/export/tests and revert the status entry. No runtime imports
the buffer yet.

## 9. Milestone 4 — Central asynchronous estimator

### Objective

Compose the mathematical GP, buffers, density conversion, and cached posterior
into one normal Python estimator object with explicit submission and update.

### Exact files

- Add `src/core/density.py`.
- Add `src/core/multifidelity_estimator.py`.
- Modify `src/core/__init__.py` to export estimator/posterior/update types.
- Add `tests/test_density.py`.
- Add `tests/test_multifidelity_estimator.py`.
- Modify `docs/multifidelity_gp_status.md`.

No robot, controller, config, example, or evaluation file changes are planned.

### Classes and functions

In `density.py`:

- `positive_part(values) -> np.ndarray`;
- `normalize_nonnegative_density(values, integration_weights, mask,
  tolerance) -> np.ndarray`;
- `validate_density(density, integration_weights, tolerance) -> None`;
- `build_aerial_target(density_high, variance_high, lambda_interest,
  lambda_uncertainty, integration_weights, mask, tolerance) -> np.ndarray`;
- `resample_structured_grid(values, source_x, source_y, query_points) ->
  np.ndarray`, using SciPy regular-grid interpolation or existing bilinear
  behavior with explicit bounds handling.

In `multifidelity_estimator.py`:

- `EstimatorSettings` frozen dataclass containing GP, retention, normalization,
  and grid settings but no robot/controller objects;
- `UpdateStatus(Enum)`: `UPDATED`, `NO_NEW_DATA`, `FAILED`;
- `PosteriorSnapshot` frozen dataclass with timestamp, version, query points and
  shape, high mean, high variance, normalized density, LOW/HIGH counts,
  validity/status, effective jitter, fit duration, and prediction duration;
- `UpdateReport` frozen dataclass with status, timestamp, version, message, and
  optional exception type (not a live exception object);
- `CentralAsynchronousEstimator`
  - `submit(observation) -> None`;
  - `submit_many(observations) -> int`;
  - `update(simulation_time) -> UpdateReport`;
  - `latest_posterior -> Optional[PosteriorSnapshot]`;
  - `low_sample_count`, `high_sample_count`, `pending_count`, and `version`.

### Public interface and behavior

- Submission never triggers fit/prediction.
- Only explicit `update()` performs expensive work.
- `update()` with no pending data returns `NO_NEW_DATA` and preserves identity
  or value of the cached posterior.
- Candidate retained arrays are converted into GP inputs in deterministic order.
- Fit and prediction complete into local candidate objects. Only after finite
  mean/variance and normalized density pass validation are buffers committed,
  version incremented, and the posterior swapped atomically.
- Any exception produces `FAILED`, preserves the previous snapshot/version and
  retained data, and records a concise failure message.
- Timing uses `time.perf_counter()` only for metrics; scheduling remains based on
  supplied simulated time.
- Posterior arrays are copied and marked read-only before publication.

### Configuration parameters

All mathematical, retention, density, and grid parameters from the planned
`multifidelity` YAML section map into `EstimatorSettings`. YAML parsing is not
added until Milestone 5.

### Tests to add

`tests/test_density.py`:

- exact positive-part clipping at negative, zero, and positive magnitudes;
- nonnegative density and weighted integral equal to one;
- obstacle masking;
- rejection of zero mass, bad shapes, negative/nonfinite weights, and invalid
  density;
- aerial target interest-only, uncertainty-only, mixed weights, and zero-std
  edge case;
- deterministic structured-grid resampling and bounds policy.

`tests/test_multifidelity_estimator.py`:

- LOW-only update;
- HIGH data arriving later changes posterior and increments version;
- out-of-order and delayed submissions;
- intervals with no new observations;
- separate dataset limits and minimum separation;
- same-position LOW/HIGH observations coexist;
- duplicate same-fidelity measurements are stable;
- posterior mean/variance finite and variance nonnegative;
- density nonnegative with weighted integral one;
- snapshot metadata/counts/grid/version correct;
- explicit injected GP or normalization failure preserves the latest posterior,
  buffer state, and version;
- fixed input order and seed produce deterministic snapshots.

### Existing tests to rerun

- All GP and observation tests from Milestones 1–3.

### Validation commands

```bash
python -m pytest -q tests/test_density.py tests/test_multifidelity_estimator.py
python -m pytest -q
python -m ruff check src/core/density.py src/core/multifidelity_estimator.py tests/test_density.py tests/test_multifidelity_estimator.py
git diff --check
```

### Expected behavior

One controller-independent estimator accepts independently arriving
observations, updates only on request, and exposes the latest valid
high-fidelity posterior/density with transactional failure behavior.

### Likely risks

- Buffer commit occurring before posterior validation.
- Published arrays remaining mutable through shared references.
- Density normalization using array sum instead of configured integration
  weights.
- Catching programming errors too broadly without an actionable report.
- Timing assertions becoming brittle; tests should assert nonnegative timing,
  not exact durations.

### Rollback strategy

Remove estimator/density modules, exports, and tests. The lower-level GP and
observation milestones remain independently usable and tested.

## 10. Milestone 5 — Simulation-loop integration in shadow mode

### Objective

Integrate independent sensing and estimator scheduling into the coupled
simulation while leaving both controller density sources unchanged. This proves
timing, ownership, and deterministic operation before closing either feedback
loop.

### Exact files

- Add `src/models/sensors.py`.
- Add `src/simulation.py`.
- Add `src/coupled_simulation.py` with the reusable new coupled loop and setup
  functions; copy only the necessary setup behavior from the audited scripts.
- Add `src/coupled_config.py` as the single-file configuration boundary.
- Modify `src/models/__init__.py` to export new sensor types.
- Add `examples/run_multifidelity.py` as the new simulation CLI.
- Add `evaluation/run_multifidelity_evaluation.py` as the new repeated
  evaluation CLI.
- Keep existing legacy configuration files unchanged as legacy references.
- Add `configs/multifidelity.yaml` as the self-contained production
  configuration, with shared settings and explicit `aerial` / `ground`
  sections.
- Add `tests/test_multifidelity_sensors.py`.
- Add `tests/test_simulation_scheduling.py`.
- Add `tests/test_legacy_smoke.py` before changing the loop, capturing a small
  deterministic legacy behavior baseline.
- Modify `docs/multifidelity_gp_status.md`.

### Classes and functions

In `sensors.py`:

- `FidelityFields(low, high, discrepancy)` frozen dataclass;
- `build_fidelity_fields(high_field, rho, smoothing_sigma_cells, mask) ->
  FidelityFields` using `scipy.ndimage.gaussian_filter`, with
  `discrepancy = high - rho * low` checked numerically;
- `SimulatedScalarFieldSensor` configured with fidelity, period-independent
  sample geometry, count, range, FOV, noise variance, and an injected RNG;
- `collect(team, field, timestamp) -> tuple[Observation, ...]`.

In `simulation.py`:

- `EstimatorMode(Enum)`: `LEGACY`, `MULTIFIDELITY`;
- `parse_estimator_mode(params) -> EstimatorMode`, missing means legacy;
- `PeriodicEvent(period, start_time=0.0, tolerance=...)` with
  `is_due(simulation_time)` and `mark_fired()`;
- `CoordinatorEventReport` with event type, fired flag, submitted count, update
  report, and posterior version;
- `MultifidelitySimulationCoordinator`
  - owns one central estimator, both sensors, and three periodic events;
  - `collect_low_if_due(simulation_time, aerial_team) ->
    CoordinatorEventReport`;
  - `collect_high_if_due(simulation_time, ground_team) ->
    CoordinatorEventReport`;
  - `update_if_due(simulation_time) -> CoordinatorEventReport`;
  - `latest_posterior` proxy;
  - does not call HEDAC or ground MPC.
- `build_multifidelity_coordinator(params, ground_params, high_field,
  query_points, query_shape, integration_weights, mask) -> coordinator` maps
  YAML through existing `HEDACParams.get()` and creates a dedicated RNG.

These coordinator/sensor methods are the new public simulation-adapter
interfaces. Existing controller and agent public signatures do not change in
this milestone.

In `coupled_simulation.py`:

- `CoupledSimulationResult` stores metrics, trajectories, estimator reports,
  and controller-density metadata without plotting;
- `CoupledSimulation` owns the two teams, existing HEDAC instance, existing
  ground MPC solver/state, and—only in multifidelity mode—one coordinator;
- `step(step_num) -> CoupledStepResult` implements the reviewed phased ordering;
- `run(num_steps) -> CoupledSimulationResult`;
- `build_coupled_simulation(aerial_params, ground_params, *, seed) ->
  CoupledSimulation` reuses existing classes and configuration conventions.

The new CLI scripts accept one `--config` path and are thin wrappers around the
loader and reusable loop. Plotting and repeated episodes stay outside the loop.

### Simulation behavior

- Legacy mode follows the current loop without constructing or calling the new
  coordinator.
- Multifidelity mode initially runs the coordinator beside the current aerial
  GP, ground GP, post-hoc fusion, and controller inputs.
- The coupled loop calls LOW collection immediately before the aerial HEDAC
  step, then calls HIGH collection and the due estimator update after aerial
  motion and before the existing ground GP/MPC block.
- Aerial LOW and ground HIGH events fire at independently configured times.
- GP update events fire independently and may occur with no new data.
- Controllers continue to step on every current integration step and still
  consume legacy densities in this milestone.
- A cached posterior persists unchanged between scheduled updates.
- New sensor RNG activity cannot change the global RNG sequence.

### Configuration parameters

- top-level `estimator_mode`;
- all planned `multifidelity` keys in Section 4;
- root shared settings plus `aerial` and `ground` override mappings in the one
  coupled YAML file remain authoritative for map, control, team size, and
  legacy GP behavior.

### Tests to add

`test_multifidelity_sensors.py`:

- low field is broader/smoother than high field;
- `high == rho * low + discrepancy` within tolerance;
- aerial records are LOW and ground records HIGH;
- timestamps, IDs, positions, values, and configured variances are correct;
- fixed dedicated RNG is deterministic;
- samples respect configured bounds/FOV policy;
- sensor calls do not advance NumPy's global RNG.

`test_simulation_scheduling.py`:

- exact expected event times for distinct sensor/update periods;
- noninteger period-to-`dt` ratios do not drift or double-fire;
- controllers represented by step counters continue between GP updates;
- no-new-data update leaves the cached posterior unchanged;
- first valid posterior is cached at the expected time;
- delayed/out-of-order injected observations remain supported;
- one and only one estimator exists for multiple robots;
- fixed-seed coordinator runs are deterministic.

`test_legacy_smoke.py`:

- absent `estimator_mode` resolves to legacy;
- explicit legacy resolves identically;
- a short standalone HEDAC path retains deterministic result shapes and selected
  baseline values;
- the legacy coupled branch does not construct the new coordinator.

### Existing tests to rerun

- The complete accumulated unit suite from Milestones 1–4.

### Validation commands

```bash
python -m pytest -q tests/test_multifidelity_sensors.py tests/test_simulation_scheduling.py tests/test_legacy_smoke.py
python -m pytest -q
python -m ruff check src/models/sensors.py src/simulation.py src/coupled_config.py src/coupled_simulation.py examples/run_multifidelity.py evaluation/run_multifidelity_evaluation.py tests/test_multifidelity_sensors.py tests/test_simulation_scheduling.py tests/test_legacy_smoke.py
python -m examples.run_multifidelity --config configs/multifidelity.yaml --no-plot
git diff --check
```

The full legacy evaluation is expensive; during implementation, add or use a
small temporary/test configuration for the smoke command rather than silently
reducing production defaults.

### Expected behavior

The coupled simulation owns and schedules one functioning central estimator in
multifidelity mode. Its posterior is observable in diagnostics but has no effect
on aerial or ground commands yet. Legacy mode remains unchanged.

### Likely risks

- Duplicate sensing changing global randomness or runtime substantially.
- New sensors accidentally sampling the normalized controller target rather
  than the defined latent LOW/HIGH fields.
- Recreating only the necessary setup in the new coupled module may accidentally
  diverge from legacy initialization; paired setup tests must make differences
  intentional.
- Coupled script duplication causing hierarchical and evaluation paths to drift.
- Initial coordinator updates being scheduled before enough data exist.

### Rollback strategy

Remove the coordinator calls and multifidelity config/example while retaining
the already tested core/estimator modules. Legacy control code remains intact,
so rollback is a narrow branch deletion.

## 11. Milestone 6 — Aerial target-density feedback

### Objective

Make the latest joint high-fidelity posterior drive the aerial HEDAC target in
multifidelity mode, while ground MPC continues to use legacy fusion.

### Exact files

- Modify `src/core/hedac.py` to accept a validated optional external density and
  allow legacy GP collection/update to be skipped.
- Modify `src/core/density.py` only if the already planned aerial target helper
  requires a defect correction.
- Modify `src/simulation.py` to expose the cached target on the HEDAC grid.
- Modify `src/coupled_simulation.py` to pass the cached external target in
  multifidelity mode.
- Modify the new `examples/run_multifidelity.py` and
  `evaluation/run_multifidelity_evaluation.py` only if their result display or
  reporting must expose the target; their loop remains in the shared module.
- Add `tests/test_aerial_multifidelity_feedback.py`.
- Modify `docs/multifidelity_gp_status.md`.

### Functions and public interfaces

- Extend `HEDACAlgorithm.step()` compatibly:

```python
step(
    agent_team,
    step_num=0,
    *,
    external_goal_density=None,
    update_legacy_gp=True,
) -> float
```

Existing callers use the same defaults and behavior.

- Add private `_validate_external_goal_density()` or a public narrow
  `set_current_goal_density()` only if tests show persistence is clearer than a
  per-step argument.
- Add coordinator method
  `aerial_target(map_x, map_y, integration_weights, mask) -> Optional[np.ndarray]`.
  It returns `None` before the first valid posterior and otherwise resamples:

```text
normalize(lambda_interest * density_H
          + lambda_uncertainty * normalized_std_H)
```

### Configuration parameters

- `multifidelity.aerial_target.lambda_interest`;
- `multifidelity.aerial_target.lambda_uncertainty`;
- `multifidelity.density.normalization_tolerance`.

### Expected behavior

- Legacy mode continues internal aerial observation collection/GP update.
- Multifidelity mode calls HEDAC with `update_legacy_gp=False` and the latest
  cached target.
- Before a posterior exists, HEDAC uses its original target.
- Between estimator updates, HEDAC repeatedly uses the same cached target while
  continuing its normal heat/controller integration.
- A HIGH ground observation can change mean and/or variance, which changes this
  aerial target at the next valid update.
- HEDAC heat equation, coverage, gradient law, agent dynamics, and controller
  frequency otherwise remain unchanged.

### Tests to add

- external target shape/finite/nonnegative validation;
- default `HEDACAlgorithm.step()` remains legacy-compatible;
- no-posterior fallback uses the original target;
- the same posterior version yields the same aerial target between updates;
- interest-only and uncertainty-only weights behave as designed;
- adding a HIGH observation changes posterior mean/variance and aerial target
  by a nonzero documented norm;
- two otherwise identical HEDAC instances receiving before/after targets produce
  a different gradient, command proxy, or deterministic short trajectory;
- HIGH influence is absent when using the legacy branch.

### Existing tests to rerun

- Full accumulated suite, especially `test_legacy_smoke.py`, estimator failure
  tests, and scheduling tests.

### Validation commands

```bash
python -m pytest -q tests/test_aerial_multifidelity_feedback.py tests/test_legacy_smoke.py
python -m pytest -q
python -m ruff check src/core/hedac.py src/simulation.py src/coupled_simulation.py examples/run_multifidelity.py evaluation/run_multifidelity_evaluation.py tests/test_aerial_multifidelity_feedback.py
git diff --check
```

### Likely risks

- HEDAC `update_gp()` resetting `current_goal_density` after an external target
  is supplied.
- Grid orientation (`x/y` versus `[y, x]`) producing a transposed target.
- Normalizing the uncertainty term inconsistently after resampling.
- A target change not immediately changing trajectory because heat has state;
  tests should allow a short deterministic horizon and also inspect gradient.
- Accidentally changing the reported legacy ergodic metric definition.

### Rollback strategy

Disable/remove only the external-target arguments and multifidelity branch.
Default legacy HEDAC behavior remains the rollback target; the shadow estimator
from Milestone 5 may remain for diagnostics.

## 12. Milestone 7 — Ground controller density integration

### Objective

Replace only the ground MPC density source with the cached normalized
high-fidelity density in multifidelity mode.

### Exact files

- Modify `src/simulation.py` to expose validated posterior density on arbitrary
  controller points or the existing MPC grid.
- Modify `src/coupled_simulation.py` at its ground density-selection and
  `weights_list` block.
- Do not modify the corresponding inline blocks in existing legacy scripts.
- Add `tests/test_ground_multifidelity_feedback.py`.
- Modify `docs/multifidelity_gp_status.md`.

Do not change `src/core/costFunctions.py`, `src/models/models.py`, CasADi solver
construction, dynamics, constraints, objective terms, warm-start logic,
Voronoi partitioning, or control frequency unless a failing test reveals a
strictly necessary interface defect.

### Functions and public interfaces

- Coordinator method
  `ground_density(query_points, integration_weights=None) -> Optional[np.ndarray]`.
- Small pure helper in `src/simulation.py`:
  `build_ground_weight_vectors(density, voronoi_masks) -> tuple[np.ndarray, ...]`.
  It validates shape/finiteness/nonnegativity and performs only multiplication.
- Existing solver parameter remains `p = concatenate([state, weights])`.

### Configuration parameters

No new active control parameter. The future information-gain term remains out of
scope and disabled. Existing posterior/density configuration applies.

### Expected behavior

- Multifidelity mode uses exactly the latest snapshot's normalized
  high-fidelity density, evaluated on `mpc_params.xy_mpc_grid`.
- Ground controllers keep using the same density between estimator updates.
- Before a valid snapshot, the approved current/legacy density is used.
- Legacy mode keeps the current independent aerial/ground predictions and
  post-hoc fusion.
- The density given to ground MPC corresponds to the same posterior version
  used to build the aerial target.

### Tests to add

- weight-vector shape and exact equality to density times Voronoi mask;
- ground density stays unchanged between estimator updates;
- fallback before first posterior;
- invalid/nonfinite/mismatched density is rejected without replacing the last
  controller density;
- a local HIGH correction changes the relevant ground weight vector in the
  expected region;
- using `costFunctions.coverage_cost` through a small CasADi function, the local
  correction changes the ground coverage objective in the expected direction;
- aerial and ground consumers report/use the same posterior version;
- legacy fusion values remain unchanged.

### Existing tests to rerun

- Full suite, particularly aerial feedback, legacy smoke, scheduling, density,
  and estimator failure tests.

### Validation commands

```bash
python -m pytest -q tests/test_ground_multifidelity_feedback.py tests/test_aerial_multifidelity_feedback.py tests/test_legacy_smoke.py
python -m pytest -q
python -m ruff check src/simulation.py src/coupled_simulation.py tests/test_ground_multifidelity_feedback.py
git diff --check
```

### Likely risks

- Supplying a 2-D density where the solver expects a flattened vector.
- Grid endpoint/orientation mismatch.
- Accidentally applying Voronoi masking twice.
- Changing density scale in a way that changes solver conditioning; unit-integral
  normalization is required, but runtime/solver behavior must be reported.
- CasADi availability due to the audited lock-file mismatch.

### Rollback strategy

Switch the multifidelity ground branch back to the existing `combo_density`
construction without changing the solver. Aerial multifidelity feedback and the
central estimator remain separately testable.

## 13. Milestone 8 — Legacy compatibility hardening

### Objective

Make the mode boundary explicit in the new entry points and prove that the
frozen existing scripts remain operational legacy references after both
controller integrations.

### Exact files

- Add `src/core/legacy_fusion.py` containing the exact current post-hoc fusion
  expression as a named helper.
- Modify `src/core/__init__.py` to export the helper only if needed by existing
  imports.
- Modify `src/coupled_simulation.py`, `examples/run_multifidelity.py`, and
  `evaluation/run_multifidelity_evaluation.py` to use the common legacy helper
  and explicit mode dispatch.
- Add `evaluation/run_multifidelity_ablation.py`, using the shared new coupled
  module for both reviewed modes.
- Do not modify `examples/hierarchical.py`, `evaluation/run_evaluation.py`,
  `evaluation/run_ablation.py`, `evaluation/aerial_baseline.py`, or
  `evaluation/ground_baseline.py`; they remain frozen regression references.
- Modify `configs/default_params.yaml`, `configs/iros26_aerial.yaml`, and
  `configs/iros26_ground.yaml` to document explicit `estimator_mode: legacy`
  where applicable; missing-key fallback remains tested.
- Add `tests/test_legacy_mode.py`.
- Extend `tests/test_legacy_smoke.py`.
- Modify `docs/multifidelity_gp_status.md`.

### Classes and functions

- `legacy_fuse(aerial_mean, aerial_std, ground_mean, ground_std, epsilon=1e-10)
  -> np.ndarray`, preserving separate max normalization and inverse-std weights
  exactly as current code.
- `parse_estimator_mode()` remains the single validator; unknown modes raise a
  clear error before simulation starts.
- Entry points branch once during initialization and keep mode-specific state
  explicit rather than mixing both estimators in one code block.

The only new public production function in this milestone is `legacy_fuse()`;
all mode-dispatch changes preserve existing CLI and controller interfaces.

### Configuration parameters

- `estimator_mode` only. Legacy `gpr.*` values remain untouched and retain their
  current meanings and quirks.

### Tests to add

- missing mode and explicit legacy yield identical dispatch;
- invalid mode fails clearly;
- `legacy_fuse()` matches the pre-refactor inline expression on fixed arrays;
- legacy mode constructs the current aerial GP and separate ground GP and does
  not construct the central estimator;
- multifidelity mode constructs one central estimator and does not use post-hoc
  fusion for controller inputs;
- existing `GaussianProcess` public fields and `HEDACAlgorithm` default method
  calls remain available;
- fixed-seed short legacy trajectory/metric/dataset outputs match the
  pre-integration baseline within documented tolerances;
- default standalone example still runs in legacy mode;
- aerial-only, ground-only, and mixed ablation dispatch remains defined.

### Existing tests to rerun

- Entire accumulated suite.

### Validation commands

```bash
python -m pytest -q tests/test_legacy_mode.py tests/test_legacy_smoke.py
python -m pytest -q
python -m ruff check src/core/legacy_fusion.py src/coupled_simulation.py examples/run_multifidelity.py evaluation/run_multifidelity_evaluation.py evaluation/run_multifidelity_ablation.py tests/test_legacy_mode.py tests/test_legacy_smoke.py
python examples/run_hedac.py --config configs/default_params.yaml --no-plot
git diff --check
```

Use a reviewed short config for routine smoke execution; do not run 2,000 steps
as an implicit unit test on every iteration.

### Expected behavior

Every supported path has an unambiguous mode. Existing configs remain legacy by
default, and legacy numerical behavior is unchanged except for explicitly
reported pre-existing nondeterminism or dependency failures.

### Likely risks

- Consolidating duplicated inline fusion could introduce a subtle operation-order
  difference.
- Baseline scripts contain unused opposite-team scaffolding and may not support
  zero-agent cases consistently.
- Full scikit-learn hyperparameter fitting may be sensitive to library/BLAS
  versions, so tolerances must distinguish true regression from optimizer noise.
- Explicit mode keys in ground config could conflict with the canonical aerial/
  common owner unless precedence is documented.

### Rollback strategy

Revert helper adoption and restore the original inline legacy blocks from the
pre-milestone diff. Keep `estimator_mode` defaulting to legacy. Do not remove the
validated multifidelity path merely because a baseline wrapper needs repair.

## 14. Milestone 9 — Closed-loop validation

### Objective

Demonstrate the complete mandatory causal chain in a deterministic automated
test and a reproducible short experiment:

```text
ground HIGH observation
  -> discrepancy-informed joint update
  -> changed high-fidelity mean and/or variance
  -> changed aerial target
  -> changed aerial guidance/trajectory
  -> same high-fidelity density used by ground MPC
```

### Exact files

- Modify `examples/run_multifidelity.py` to expose the deterministic demo result
  and optional output path; do not create a second overlapping simulation CLI.
- Add one self-contained `configs/multifidelity_demo.yaml`.
- Add `tests/test_closed_loop_multifidelity.py`.
- Add `tests/test_multifidelity_robustness.py`.
- Modify `src/coupled_simulation.py` only if additional deterministic diagnostics
  are needed; keep them reusable by simulation and evaluation scripts.
- Modify `docs/multifidelity_gp_status.md`.

### Functions and public interfaces

- `run_multifidelity_demo(config, *, seed, num_steps, output_path=None) ->
  DemoResult` in `examples/run_multifidelity.py`, backed by
  `src/coupled_simulation.py`.
- `DemoResult` contains observation histories, posterior versions and maps,
  controller density/target histories, aerial guidance/trajectory, ground
  weights/trajectory, and causal-difference metrics.
- Add a narrow HEDAC diagnostic for the commanded gradient/target velocity only
  if trajectory comparison cannot identify the controller response reliably;
  keep the default controller API compatible.

### Scenario design

- Fixed map, initial states, RNG seed, update schedule, and GP hyperparameters.
- LOW truth contains broad modes after smoothing.
- HIGH truth contains a narrow local feature represented in the reference
  discrepancy and initially unobserved by aerial data.
- Ground path deliberately samples the local feature after an initial LOW-only
  phase.
- Two paired runs use identical initial conditions: one receives the informative
  HIGH observation and one omits it. No other randomness differs.

### Configuration parameters

Use the full reviewed multifidelity contract with small sample caps and a short
runtime. No validation threshold is silently read from production control
weights; test tolerances are named constants in tests.

### Tests to add

`test_closed_loop_multifidelity.py` asserts:

- LOW-only posterior recovers broad structure;
- a retained HIGH sample appears at the expected simulated time and increments
  the posterior version/HIGH count;
- posterior mean and/or variance changes near the local feature by more than a
  documented tolerance;
- the aerial target changes by a nonzero L1/L2 norm;
- HEDAC gradient/command or short aerial trajectory diverges from the paired
  no-HIGH run;
- ground controller weights are derived from the exact same posterior version
  and normalized density;
- the ground objective/trajectory responds in the expected local direction;
- repeated runs are deterministic;
- legacy paired run has no ground-to-aerial causal path.

`test_multifidelity_robustness.py` asserts continued operation under:

- different aerial and ground sensor periods;
- delayed and out-of-order observations;
- temporary intervals with no observations;
- dataset replacement at LOW/HIGH limits;
- duplicate/nearly coincident observations;
- one induced failed update with subsequent recovery;
- cached controller densities remaining valid through no-update/failure periods.

### Existing tests to rerun

- Entire accumulated unit, integration, feedback, and compatibility suite.

### Validation commands

```bash
python -m pytest -q tests/test_closed_loop_multifidelity.py tests/test_multifidelity_robustness.py
python -m pytest -q
python -m examples.run_multifidelity --config configs/multifidelity_demo.yaml --no-plot
python -m ruff check src/coupled_simulation.py examples/run_multifidelity.py tests/test_closed_loop_multifidelity.py tests/test_multifidelity_robustness.py
git diff --check
```

### Expected behavior

The causal chain is demonstrated numerically, not inferred from a plot. The
ground and aerial controllers consume products of the same cached posterior,
and a HIGH observation changes aerial behavior.

### Likely risks

- The heat field's memory may delay observable aerial trajectory divergence.
- A local correction may be too small relative to uncertainty weight or HEDAC
  normalization.
- Paired runs may accidentally consume different RNG sequences.
- A solver tolerance may obscure a small ground control change.
- End-to-end tests may become slow; keep the map, horizon, team, samples, and
  steps minimal while retaining the causal effect.

### Rollback strategy

The feature cannot be declared complete if this milestone fails. Revert only
the demo/test scaffolding if it is flawed; keep earlier tested components and
return to the first failed link in the chain. Do not weaken assertions merely to
obtain a pass.

## 15. Milestone 10 — Experiment logging and runtime evaluation

### Objective

Record all designed outputs, measure update/prediction cost, compare legacy and
multifidelity modes under identical seeds, and document user-facing operation.

### Exact files

- Add `src/utils/multifidelity_logging.py`.
- Modify `src/utils/__init__.py` only if a public logger export is useful.
- Modify `evaluation/run_multifidelity_evaluation.py` to emit structured
  multifidelity output.
- Add `evaluation/plot_multifidelity.py` rather than extending the brittle
  top-level `evaluation/plots.py`.
- Modify `evaluation/run_multifidelity_ablation.py` to record mode and estimator
  statistics.
- Modify `pyproject.toml` and `uv.lock` only as needed to synchronize the already
  declared SciPy/CasADi dependencies and declare POT for the existing `ot`
  imports; keep this dependency-only diff explicit.
- Modify `README.md`.
- Modify `YAML_CONFIG_GUIDE.md`.
- Add `docs/multifidelity_gp_experiments.md`.
- Add `tests/test_multifidelity_logging.py`.
- Add `tests/test_multifidelity_runtime.py`.
- Modify `docs/multifidelity_gp_status.md` with final measured results and known
  limitations.

### Classes and functions

- `MultifidelityRunLogger`
  - `record_observations(...)`;
  - `record_posterior(snapshot)`;
  - `record_controller_inputs(time, posterior_version, aerial_target,
    ground_density)`;
  - `record_trajectories(...)`;
  - `finalize(metrics) -> MultifidelityRunRecord`;
  - `save_npz(path) -> None`.
- `MultifidelityRunRecord` dataclass containing arrays plus schema version,
  mode, seed, config snapshot, query-grid metadata, and validity masks for
  variable update counts.
- `plot_multifidelity_run(record_path, output_dir, show=False)` reads only saved
  output and has no estimator/controller imports.

### Logged outputs

- LOW and HIGH observations with timestamps/robot IDs/noise variance;
- high-fidelity posterior mean and variance at every successful version;
- normalized ground density and aerial target density actually consumed;
- LOW/HIGH retained and pending counts;
- effective jitter and update status;
- GP fit/update and map-prediction duration;
- integrated posterior uncertainty;
- high-field RMSE/MAE and density KL/Wasserstein where dependencies are present;
- posterior version used by each controller step;
- aerial commands or gradient proxies and trajectories;
- ground weights/commands and trajectories;
- legacy/multifidelity mode, seed, and configuration metadata.

### Configuration parameters

Reuse existing `output.*` paths and verbosity where possible. Add only:

```yaml
output:
  save_multifidelity_history: true
  multifidelity_results_path: output/multifidelity/results.npz
  log_posterior_every_update: true
```

Do not put plotting/logging options in the GP core.

### Tests to add

`test_multifidelity_logging.py`:

- round-trip NPZ save/load preserves shapes, versions, times, and densities;
- logger does not mutate snapshot/controller arrays;
- variable observation/update histories serialize deterministically;
- schema/mode/seed/config metadata is present;
- plotting loader can run headless on a minimal record.

`test_multifidelity_runtime.py`:

- GP updates occur less frequently than controller steps under reviewed config;
- retained counts never exceed limits;
- measured durations are finite and nonnegative;
- a reviewed small scenario completes within a generous CI budget;
- legacy and multifidelity paired runs share seed/initial conditions;
- field reconstruction and integrated uncertainty metrics are finite;
- full required causal metrics are present in saved output.

### Existing tests to rerun

- Entire suite, including complete closed-loop and legacy compatibility tests.

### Validation commands

```bash
python -m pytest -q tests/test_multifidelity_logging.py tests/test_multifidelity_runtime.py
python -m pytest -q
python -m examples.run_multifidelity --config configs/multifidelity_demo.yaml --no-plot
python -m evaluation.run_multifidelity_evaluation --config configs/multifidelity.yaml
python evaluation/plot_multifidelity.py --input output/multifidelity/results.npz --output-dir output/multifidelity/plots
python -m ruff check src tests examples evaluation
uv lock --check
git diff --check
```

Long evaluation commands must use an explicitly reviewed episode/step count or
be marked as manual/slow; unit CI must not silently execute the full default
experiment.

### Expected behavior

The repository produces reproducible, mode-qualified evidence for mathematical
accuracy, runtime cost, controller inputs, and the ground-to-aerial causal
effect. User documentation explains configuration, execution, and the remaining
sensor/model limitations.

### Likely risks

- Posterior history can be large; log only successful update versions and use
  configured experiment size rather than every controller step.
- Object arrays/pickle-dependent NPZ data would harm portability; use numeric
  arrays and JSON-compatible metadata.
- Runtime thresholds can be machine-sensitive; use generous regression bounds
  and report measured values.
- Adding POT/lock synchronization may produce a large dependency diff and must
  remain separate from numerical behavior changes.
- Evaluation scripts currently assume output directories and files; new logger
  must create its own parent directory safely.

### Rollback strategy

Disable the new logging flags and revert logger/plot/dependency/documentation
changes without removing the validated controller integration. If dependency
sync is problematic, retain core RMSE/uncertainty metrics and postpone optional
Wasserstein plotting rather than altering GP behavior.

## 16. Planned test matrix

| Requirement | Primary test file |
| --- | --- |
| Joint covariance/reference agreement | `tests/test_multifidelity_gp.py` |
| Cholesky/jitter/duplicates | `tests/test_multifidelity_gp_contract.py`, `tests/test_multifidelity_gp.py` |
| LOW-only/HIGH-later behavior | `tests/test_multifidelity_estimator.py` |
| Timestamp, delay, out-of-order | `tests/test_observations.py`, `tests/test_multifidelity_robustness.py` |
| Dataset limits/retention | `tests/test_observations.py`, `tests/test_multifidelity_estimator.py` |
| Nonnegative unit-integral density | `tests/test_density.py`, `tests/test_multifidelity_estimator.py` |
| Failed update preserves posterior | `tests/test_multifidelity_estimator.py` |
| Independent simulated periods | `tests/test_simulation_scheduling.py` |
| Genuine smoothed LOW sensor | `tests/test_multifidelity_sensors.py` |
| Ground observation changes aerial target | `tests/test_aerial_multifidelity_feedback.py` |
| Aerial command/trajectory changes | `tests/test_aerial_multifidelity_feedback.py`, `tests/test_closed_loop_multifidelity.py` |
| Ground uses same posterior density | `tests/test_ground_multifidelity_feedback.py`, `tests/test_closed_loop_multifidelity.py` |
| Legacy remains operational | `tests/test_legacy_smoke.py`, `tests/test_legacy_mode.py` |
| Logging/runtime outputs | `tests/test_multifidelity_logging.py`, `tests/test_multifidelity_runtime.py` |

## 17. Completion criteria

The project is not complete merely because the GP core or central estimator
passes unit tests. Completion requires all of the following:

- `legacy` and `multifidelity` modes both run with reviewed configurations;
- no real concurrency or per-robot multifidelity estimator exists;
- GP work is scheduled separately from controller integration;
- production GP math contains no explicit inverse;
- posterior mean/variance are finite, variance is nonnegative, and density has a
  verified unit numerical integral;
- failed updates preserve the last valid posterior and controller inputs;
- aerial and ground sensors operate at independent simulated periods;
- LOW sensing represents a systematic coarse field, not only higher noise;
- ground MPC consumes the normalized high-fidelity posterior density;
- aerial HEDAC consumes the high-fidelity interest/uncertainty target;
- an automated deterministic test demonstrates:

```text
ground HIGH observation
  -> changed joint high-fidelity posterior
  -> changed aerial target
  -> changed aerial command or trajectory
```

- the same posterior version/density is shown to feed ground control;
- the full accumulated test suite passes;
- status, configuration, experiment, runtime, and known-limitation documentation
  is current.

Only after those conditions are met should removal or deprecation of any legacy
implementation be considered, and removal is outside this plan.
