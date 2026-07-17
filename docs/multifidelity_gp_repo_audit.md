# Repository Audit for the Central Multi-Fidelity GP

Architecture baseline diagrams: `docs/architecture/current_architecture.md`.

## 1. Scope and notation

This audit covers the complete authored repository, the generated `build/` copy,
and the existing generated outputs. It was prepared without modifying production
code.

Statements under **Facts** describe behavior present in the repository at the
time of the audit. Statements under **Recommendations** are proposed integration
choices; they are not descriptions of current behavior.

The repository uses `(x, y)` for robot and GP point coordinates, while NumPy maps
are indexed as `[y, x]`. The principal map is currently square, which hides some
width/height inconsistencies noted below.

## 2. Executive summary

### Facts

- `main.py` is a placeholder and is not the simulation entry point.
- Standalone aerial GP-HEDAC runs through `examples/run_hedac.py` or
  `examples/run_gp_single_robot.py`, which call `HEDACAlgorithm.run()`.
- The coupled aerial/ground simulation is implemented procedurally in
  `examples/hierarchical.py`. The evaluation version and the code most relevant
  to the requested feature are in `evaluation/run_evaluation.py`.
- Aerial and ground estimation use two separate instances of the same
  `src.core.GaussianProcess.GaussianProcess` class. There is no distinct ground
  GP implementation class.
- In the coupled loop, the two posterior means are fused after prediction using
  weights derived from separately normalized posterior standard deviations.
  The fused map is used only by ground MPC. It never feeds back into aerial
  HEDAC.
- Both robot teams sample the same static GMM-derived `target_density` using
  point samples plus additive Gaussian noise. The configured aerial and ground
  noise levels differ, and the robots have different motion/FOV implementations,
  but the measured latent quantity is otherwise identical.
- There are no timestamps, fidelity labels, robot identifiers, or per-sample
  noise variances in an observation. An observation is an `N x 3` array row
  `[x, y, value]`.
- Sensing, GP update/prediction, heat update, and controller updates all happen
  on every outer simulation step. `gpr.fit_interval` gates calls to
  `GaussianProcessRegressor.fit`; it is not a sensor or posterior-update period.
- No test files or test directories are present.
- There is no `estimator_mode` configuration and no implementation of the
  autoregressive model `f_H = rho f_L + delta`.

### Recommendations

- Treat `evaluation/run_evaluation.py` (and the simpler
  `examples/hierarchical.py`) as the initial integration host. It is the only
  current owner of both teams, both estimators, fusion, and both control paths.
- Add one simulation-owned central estimator, not an estimator per robot and
  not one inside each controller.
- Preserve the existing `GaussianProcess` plus post-hoc fusion path under a
  default `legacy` mode. Add a separate, controller-independent mathematical GP
  core and asynchronous estimator for `multifidelity` mode.
- Inject the cached multifidelity aerial target at the existing
  `HEDACAlgorithm.current_goal_density` seam and inject the cached high-fidelity
  density at the existing ground MPC `W` parameter seam. Do not rewrite either
  control law.
- Decide and implement a genuine low-fidelity aerial sensor transformation
  (for example, footprint averaging or altitude-dependent smoothing). Different
  additive noise alone does not provide the systematic coarse/fine relationship
  assumed by the design.

## 3. File-by-file architecture map

### Root, configuration, and documentation

| File | Facts |
| --- | --- |
| `main.py` | Defines only `main()` printing `"Hello from hedac!"`; it does not construct or run a simulation. |
| `README.md` | Documents standalone GP-HEDAC, centralized observations within one team, YAML usage, goal-density combination, and debug plotting. Some listed files/configs are no longer present. |
| `YAML_CONFIG_GUIDE.md` | Documents `HEDACParams`, configuration sections, and CLI examples. It describes a broader schema than every executable path actually supports. |
| `pyproject.toml` | Declares Python `>=3.10`; runtime dependencies include NumPy, SciPy, scikit-learn, Numba, CasADi, Matplotlib, PyYAML/PyAML, and PyQt6; dev dependencies are pytest and Ruff. |
| `uv.lock` | Locks the environment, but its root package metadata does not include the currently declared SciPy or CasADi dependencies. It also contains no POT package for the imported `ot` module. |
| `.python-version` | Selects Python 3.10. |
| `.gitignore` | Ignores build products, virtual environments, output contents, PNG, and NPZ files. |
| `AGENTS.md` | Defines the required autoregressive multifidelity model, legacy compatibility, no-real-concurrency constraint, validation rules, and required closed-loop causal chain. |
| `docs/multifidelity_gp_design.md` | Defines joint covariance mathematics, asynchronous semantics, observation metadata, bounded datasets, posterior representation, density conversion, controller feedback, configuration, and tests. |
| `docs/multifidelity_gp_status_template.md` | Empty status-report template for later implementation milestones. |
| `prompts/01_repository_audit.md` | Requests this audit and forbids production changes. |
| `prompts/02_implementation_plan.md` | Requests a milestone-by-milestone implementation plan after this audit. |
| `prompts/03_gp_core.md` | Scopes the pure mathematical multifidelity GP and deterministic mathematical tests. |
| `prompts/04_async_estimator.md` | Scopes timestamped observations, buffers, retention, cached posterior, and estimator tests. |
| `prompts/05_simulation_integration.md` | Scopes scheduling and legacy/multifidelity integration without changing controller density sources. |
| `prompts/06_aerial_feedback.md` | Scopes high-fidelity posterior feedback into the aerial target and its causal tests. |
| `prompts/07_ground_controller.md` | Scopes replacement of the ground MPC density source while preserving its control law. |
| `prompts/08_validation.md` | Scopes complete closed-loop validation, logging, robustness checks, and legacy comparison. |
| `configs/default_params.yaml` | Standalone GP-HEDAC defaults: one Dubins agent, 50x50 map, 0.1 s steps, and GP/filter/debug settings. |
| `configs/iros26_aerial.yaml` | Coupled/evaluation aerial configuration: three Dubins aircraft, ten episodes of 300 steps, aerial observation noise std 0.2, 50x50 map at resolution 1.0. |
| `configs/iros26_ground.yaml` | Coupled/evaluation ground configuration: seven unicycle robots, horizon-2 MPC, observation noise std 0.01, resolution 0.1, and a 100x100 MPC query grid. |

### Core production modules

| File | Relevant classes/functions and facts |
| --- | --- |
| `src/core/base.py` | `HEDACParams` loads arbitrary YAML dictionaries, exposes convenience attributes and dot-key `get()`, and has no schema validation. `MapLoader` loads/creates binary obstacle maps and computes free area. `compute_ergodic_metric()` compares normalized coverage with a target. |
| `src/core/GaussianProcess.py` | `GaussianProcess` wraps scikit-learn `GaussianProcessRegressor`; owns one `N x 3` dataset and an unbounded all-observation history; collects robot samples; filters/caps/deduplicates data; fits; predicts; and creates a mean/uncertainty target. It is used for both aerial and ground data. |
| `src/core/hedac.py` | `HEDACAlgorithm` owns the aerial GP, heat field, coverage, original and current goal densities, metrics, and the standalone loop. `step()` senses, updates the GP, updates coverage/heat, computes a metric, commands agents, then optionally plots. |
| `src/core/costFunctions.py` | Defines CasADi ground objectives: `collision_cost`, distance-based `coverage_cost`, limited-FOV `limfov_coverage_cost`, and `orientation_cost`. The coupled MPC uses the first three, with density passed as symbolic weights `W`. |
| `src/core/gmm.py` | `GMM` samples and optionally fits a Gaussian mixture. Evaluation entry points generate the static ground-truth field with this class. |
| `src/core/utilities.py` | Older/duplicate Gaussian PDF, Voronoi, and FOV helpers. Coupled scripts import the parallel implementations from `src/utils/voronoi.py`, not this module. |
| `src/core/__init__.py` | Exports `HEDACParams`, `MapLoader`, metric, `HEDACAlgorithm`, and `GaussianProcess`. |

### Robot and numerical modules

| File | Relevant classes/functions and facts |
| --- | --- |
| `src/models/agents.py` | `AgentState`; `DoubleIntegratorAgent`; `DubinsAgent`; `UnicycleAgent`; `AgentLike`; and `AgentTeam`. All three agents duplicate a `sense_environment_gp()` implementation. Double-integrator and Dubins sensing is radial over 360 degrees; unicycle sensing uses a heading-centered angular FOV. |
| `src/models/models.py` | Numba and CasADi single-integrator, double-integrator, and unicycle dynamics; `get_dynamics()` and `get_model_config()` select them; `simulate_trajectory_numba()` rolls out controls. Ground MPC uses the CasADi dynamics selected here. |
| `src/models/__init__.py` | Exports double-integrator and Dubins types but omits `UnicycleAgent`, which coupled scripts therefore import directly from `agents.py`. |
| `src/utils/math_utils.py` | Density normalization, heat-equation updates, coverage kernels, FOV transforms, bilinear interpolation, Gaussian density creation, and gradient guidance. `normalize_to_pdf()` and `bilinear_interpolate()` are directly reusable integration utilities. |
| `src/utils/voronoi.py` | Isotropic and anisotropic Voronoi partitioning plus an FOV polygon helper. The coupled ground loop uses `compute_voronoi_partitioning()`. |
| `src/utils/visualize_gp.py` | Four-panel standalone GP-HEDAC debugger showing true target/observations, GP-derived target, coverage, and ergodic metric. |
| `src/utils/eval_utils.py` | Ground sensing-effectiveness, KL, and Wasserstein metrics. Imports `ot`, although POT is not declared or locked. |
| `src/utils/__init__.py` | Re-exports selected math utilities. |

### Executable examples and evaluations

| File | Facts |
| --- | --- |
| `examples/run_hedac.py` | Primary standalone CLI/API entry point. Builds a map/density/team, runs `HEDACAlgorithm.run()`, saves NPZ results, and plots summary figures. Supports Dubins or double-integrator agents. |
| `examples/run_gp_single_robot.py` | Deterministic single-agent GP-HEDAC debug entry point using a centered synthetic Gaussian. |
| `examples/hierarchical.py` | Interactive coupled prototype. Builds aerial HEDAC and procedural ground MPC, maintains a separate `ground_gp`, performs inline post-hoc fusion, and plots six panels. |
| `evaluation/run_evaluation.py` | Main coupled evaluation loop. Repeats episodes, owns aerial HEDAC plus ground GP/MPC, performs legacy fusion, computes effectiveness/KL/Wasserstein metrics, and saves selected arrays. |
| `evaluation/aerial_baseline.py` | Aerial-only estimation baseline. Runs HEDAC/aerial GP and saves aerial KL/Wasserstein/final-density outputs. It still constructs unused ground/MPC objects. |
| `evaluation/ground_baseline.py` | Ground-only control/estimation baseline. Does not step aerial robots; ground MPC weights use `ground_gp_mean`. It still queries an unfitted aerial GP for plotting/legacy scaffolding. |
| `evaluation/run_ablation.py` | Varies aerial/ground team counts, reuses initialization functions from `run_evaluation.py`, repeats the same two-GP/fusion/control logic, and saves aggregate metrics. |
| `evaluation/eval.py` | Top-level plotting/summary script for ablation output files; it executes on import rather than behind a function/main guard. |
| `evaluation/plots.py` | Top-level publication plotting script for saved trajectories/densities/metrics; assumes a fixed 50x50 domain and several output files that current save blocks leave commented out. |
| `evaluation/egerstedt.py` | Independent Lloyd/Voronoi heterogeneous-coverage experiment. It does not use the GP-HEDAC/MPC architecture and is not part of the current coupled GP data flow. |

### Generated and non-authoritative content

| Path | Facts |
| --- | --- |
| `build/lib/**` | Generated copy of all `src/**` modules. A recursive comparison found no authored-code differences. It is ignored and must not be edited as an integration target. |
| `src/hedac.egg-info/**` | Generated editable/install metadata (`PKG-INFO`, source/dependency lists, and top-level package metadata). It is ignored and not an implementation target. |
| `src/**/__pycache__/**` | Generated Python/Numba cache content. It is ignored and non-authoritative. |
| `output/eval/obstacles.npy` | Generated obstacle locations from an evaluation run. |
| `output/trajectories.png` | Generated plot from the independent Egerstedt experiment. |

## 4. Current simulation loops and time stepping

### 4.1 Standalone aerial GP-HEDAC

#### Facts

`examples/run_hedac.py` constructs the simulation and calls
`HEDACAlgorithm.run()`. `run()` loops over integer `step` and calls
`HEDACAlgorithm.step(agent_team, step_num=step)`.

One `HEDACAlgorithm.step()` performs this order:

1. Collect `obs_per_step` observations from every aerial agent at its current
   pose, from the original static `goal_density`.
2. Append/filter the observations and invoke the aerial GP update/prediction.
3. Set `current_goal_density` to the GP-derived target, or fall back to the
   original target if the update reports insufficient samples.
4. Add coverage at every current agent position.
5. Advance the heat equation once using `params.dt`, capped by a CFL-derived
   `dt_heat`.
6. Compute the reported ergodic metric against the original goal density, not
   `current_goal_density`.
7. Compute a heat gradient and command each agent once. Robot dynamics use each
   agent's `dt`, normally `params.dt_agent`.
8. Clip agent positions to the map and optionally create a debug plot.

There is no explicit simulation clock. The only time-like input to GP logic is
the integer `step_num`.

### 4.2 Coupled aerial/ground workflow

#### Facts

`examples/hierarchical.py` and `evaluation/run_evaluation.py` own the coupled
outer loop. The evaluation version performs this order on every step:

1. Call `hedac.step()`, including aerial sensing, aerial GP work, heat update,
   and aerial movement.
2. Query the aerial scikit-learn GP on the ground MPC grid.
3. Collect observations for all ground robots at their current poses.
4. Update the separate ground GP and query it on the same MPC grid.
5. Compute ground Voronoi masks.
6. Fuse aerial and ground posterior means inline.
7. Multiply the fused map by each robot's Voronoi mask and pass the resulting
   full-grid weight vector to the CasADi solver.
8. Apply the first MPC control to each ground robot once.
9. Compute evaluation metrics and optionally plot.

`ground_params.dt` is overwritten with the aerial `params.dt` for MPC dynamics,
while the already configured ground agent integration step remains
`ground_params.dt_agent`. Both are currently 0.1 seconds. No independent sensor
or estimator periods exist.

#### Recommendations

- Define simulation time once as `step * simulation.dt` (or an accumulated
  clock) and make sensor/estimator scheduling explicit in the coupled owner.
- Preserve the numerical integration and robot update frequencies. Ground
  observations collected after the aerial command can update the cached
  posterior for ground control immediately and aerial control on the next step;
  this preserves current team ordering while still providing a deterministic
  causal path.
- Do not call multifidelity GP optimization/prediction from every controller
  step. Controllers should read a cached posterior between configured estimator
  updates.

## 5. Robot models and sensing

### 5.1 Aerial robots

#### Facts

- Coupled aerial configuration selects `DubinsAgent`: state `(x, y, heading)`,
  constant forward speed, and a clipped coordinated-turn rate derived from bank
  angle. HEDAC supplies a desired gradient direction.
- Standalone configuration can alternatively select `DoubleIntegratorAgent`,
  which tracks target velocity/heading through acceleration commands.
- In the coupled aerial constructor, `DubinsAgent` is not given configured
  `sensor.fov_depth` or `gpr.obs_per_step`, so its sensing radius uses the class
  default 10. `GaussianProcess.collect_observations()` does override the number
  of samples with its GP setting.
- Dubins and double-integrator GP sensing distributes sample bearings over the
  full circle. The aerial `sensor.fov_degrees` value is used for visualization,
  not by these GP sensing methods.

### 5.2 Ground robots

#### Facts

- Coupled ground configuration selects `UnicycleAgent`: state
  `(x, y, heading)` and controls `(linear velocity, angular velocity)`.
- Ground initialization passes `sensor.fov_depth` as `observations_range` and
  `sensor.fov_degrees` into the agent.
- `UnicycleAgent.sense_environment_gp()` distributes fixed bearings across the
  current heading-centered FOV and draws random radial distances.
- The ground controller is not represented by a controller class. A CasADi
  `nlpsol` object and a `SimpleNamespace` of bounds/grid/dynamics are built by
  `init_mpc_from_params()`, then driven by procedural code in each loop.

### 5.3 Shared sensor value model

#### Facts

All `sense_environment_gp()` variants:

- generate point locations within a radius/FOV;
- repeatedly resample only the radial distance when a point is outside bounds;
- clip any remaining out-of-bounds point to `[0, width-1] x [0, height-1]`;
- use nearest-neighbor lookup into the same 2-D `true_density_map`;
- add zero-mean Gaussian noise with a team-level configured standard deviation;
- return `[x, y, noisy_value]` and append it to per-agent history.

Obstacles do not mask or occlude sensor samples. Samples may fall inside obstacle
cells when a nonempty map is used.

#### Mismatch with genuine multifidelity semantics

The present difference is primarily noise (`0.2` aerial versus `0.01` ground),
sampling geometry, and robot trajectory. Aerial measurements are not spatially
averaged, blurred, downsampled, altitude-dependent, biased, or otherwise made a
systematically coarser observation of a low-fidelity latent field. Both teams
look up the same point value from `target_density`. Consequently, the current
sensor model does not by itself justify interpreting aerial observations as
measurements of `f_L` and ground observations as measurements of
`f_H = rho f_L + delta`.

#### Recommendations

- Put fidelity-specific measurement generation outside robot dynamics or behind
  a sensor interface. Retain the robot methods for legacy mode.
- Define the aerial low-fidelity truth as an explicit transformation of the
  high-fidelity truth, preferably deterministic spatial footprint averaging or
  smoothing for the first implementation. Keep additive noise separate from
  this transformation.
- Store the noise **variance** on each new observation, converting from the
  existing standard-deviation configuration once at the sensor boundary.

## 6. Existing GP implementations and fusion

### 6.1 Shared legacy `GaussianProcess`

#### Facts

There is one GP class, instantiated as `HEDACAlgorithm.gp` for aerial data and as
the standalone `ground_gp` in coupled scripts. Its public/reused surface is:

- constructor `GaussianProcess(params, map_shape)`;
- `collect_observations(agent_team, goal_density) -> N x 3`;
- `update_gp(new_observations, step_num) -> bool`;
- `predict(points, return_std)`;
- `get_goal_density()`;
- `dataset`, `all_observations`, `gp_mean`, `gp_std`,
  `gp_std_normalized`, `grid_points`, `model`, `n_samples`, and
  `has_prediction`.

The class precomputes an integer-cell query grid over the supplied map shape.
Its dataset behavior is:

- append all samples to an unbounded diagnostic history;
- after a posterior exists, accept new points only where normalized uncertainty
  exceeds `take_threshold` when filtering is enabled;
- optionally remove existing points below `remove_threshold` (disabled in all
  current configs by `-1`);
- cap the active dataset by retaining the most recent rows;
- round coordinates to integer cells and retain the first row at each rounded
  position.

The configured `gpr.length_scale`, `sigma_f`, `noise_level`, `implementation`,
and `fit_hyperparams` values do not determine the constructed model. The code
constructs a hard-coded `Constant * RBF + WhiteKernel`, hard-codes
`alpha=1e-5`, enables optimizer restarts, and does not branch on
`implementation` or `fit_hyperparams`.

`fit_interval` controls whether `.fit()` is called when new data exist. The code
still calls prediction over the full internal grid on every update with enough
samples. Before the first scheduled fit, scikit-learn returns its unfitted prior;
the update is nevertheless reported as successful.

The target conversion clips negative mean/std values, min-max normalizes both,
and computes `exp(mean_norm) + exp(std_norm) - 2`. Although `gamma` is read and
passed into this function, it is not used in the formula. The result is min-max
normalized, not normalized to a unit numerical integral.

No exception handling preserves a previous valid posterior. If an update drops
below `min_samples`, cached mean/std are explicitly cleared. Fit or prediction
exceptions propagate.

### 6.2 Current post-hoc fusion

#### Facts

Fusion is duplicated inline in `examples/hierarchical.py`,
`evaluation/run_evaluation.py`, and `evaluation/run_ablation.py`:

```text
den = 1 / ground_std + 1 / aerial_std
w_a = (1 / aerial_std) / den
w_g = (1 / ground_std) / den
combo = w_a * aerial_mean + w_g * ground_mean
```

Small epsilons are added in code. Each standard-deviation vector is first divided
by its own maximum, destroying absolute calibration between the two independent
models. Weighting uses inverse standard deviation rather than inverse variance.
The resulting `combo_density` is not guaranteed nonnegative and is not
normalized before entering MPC.

This is not a joint GP posterior: it has no cross-covariance, no `rho`, no
discrepancy process, and no joint high-fidelity posterior variance.

## 7. Controller density inputs

### 7.1 Aerial HEDAC

#### Facts

`HEDACAlgorithm.update_gp()` obtains `gp.get_goal_density()` and assigns it to
`current_goal_density`. `compute_source_term()` subtracts normalized accumulated
coverage from `current_goal_density`, keeps the positive squared difference,
and uses the result as the heat source. Agents follow the heat-field gradient.

Therefore the current aerial causal path is:

```text
aerial robot point samples
  -> HEDACAlgorithm.gp
  -> GP mean/std target
  -> current_goal_density
  -> heat source and heat field
  -> gradient
  -> target velocity/heading
  -> aerial motion
```

Ground observations do not enter any object in this path. The legacy fused map
exists only after `hedac.step()` has already moved aerial robots.

### 7.2 Ground MPC

#### Facts

`init_mpc_from_params()` creates a fixed `local_grid_points x local_grid_points`
query grid and a CasADi symbolic parameter `W` with one entry per grid point. At
runtime, the coupled loop:

1. predicts both GPs on this grid;
2. builds `combo_density`;
3. constructs a Voronoi mask for each ground robot;
4. computes `weights = combo_density * mask`;
5. concatenates the robot state and the full weights vector as solver parameter
   `p`;
6. solves the limited-FOV/distance/collision objective and applies the first
   control.

The collected `local_grid`/`voronoi_points` variables are not used by the solver;
the solver always receives weights over its fixed full grid.

The ground-only baseline substitutes `ground_gp_mean` for `combo_density`. The
ablation uses aerial-only, ground-only, or fused means depending on team counts.

### Recommendations: exact density seams

- **Aerial seam:** in multifidelity mode, prevent `HEDACAlgorithm.step()` from
  collecting/updating its legacy internal GP and set its
  `current_goal_density` from the central cached aerial target before heat-field
  update. A narrow method such as `set_current_goal_density()` or an optional
  externally supplied target is preferable to coupling HEDAC to the new GP
  class. Legacy mode should retain the current `collect_observations()` and
  `update_gp()` path unchanged.
- **Ground seam:** replace only the construction of `combo_density` in the
  coupled loop. Evaluate/interpolate the cached normalized high-fidelity density
  on `mpc_params.xy_mpc_grid`, then retain the current Voronoi masking, solver
  parameter `p`, objective, constraints, warm start, and robot update.
- **Ownership seam:** create one estimator after both teams and grids are
  initialized in `run_evaluation.py`/`hierarchical.py`. Submit aerial LOW and
  ground HIGH observations to that same object. Do not attach it to any robot.

## 8. Grids, maps, and density representations

### Facts

- `MapLoader` represents occupancy as a 2-D array, with `0` free and `1`
  occupied. `resolution` contributes to free area but array indices are used as
  physical coordinates in most HEDAC and sensing code.
- The aerial map and internal legacy GP grid are 50x50 integer cells with points
  `(0..49, 0..49)`.
- The ground GP is also constructed with `map_array.shape`, so its internal grid
  is still 50x50 even though ground config resolution is 0.1.
- Evaluation creates a separate 500x500 high-resolution metric grid using
  `np.arange(..., 0.1)`.
- Ground MPC uses 100 points per axis from `np.linspace(0, 50, 100)`, including
  coordinate 50. Sensor points are clipped to at most 49, so predictions at the
  upper MPC boundary extrapolate beyond the sensor/map coordinate range.
- HEDAC goal/coverage/heat fields are shaped like the occupancy map. Ground MPC
  densities and predictions are flattened vectors of length 10,000.
- GMM target creation in coupled evaluation min-max normalizes the 50x50 aerial
  map rather than normalizing its numerical integral. Other paths sometimes
  normalize by sum, and the controller's legacy fused map is not normalized.
- `HEDACAlgorithm.__init__` assigns `width = map.shape[0]` and
  `height = map.shape[1]`, the reverse of the usual NumPy interpretation used
  elsewhere. Current 50x50 maps conceal this issue.

### Reusable components

- `MapLoader` and its occupancy mask/free-area calculation.
- `(x, y)` point convention and `[y, x]` map convention, once documented and
  validated consistently.
- `math_utils.bilinear_interpolate()` for controller-grid resampling.
- `math_utils.normalize_to_pdf()` as a starting primitive, extended to account
  explicitly for cell area and normalization tolerance.
- Existing aerial map grid and `mpc_params.xy_mpc_grid` as query-grid inputs;
  neither should be hard-coded inside the GP mathematics.
- Existing GMM machinery for deterministic synthetic high-fidelity truth and
  reconstruction evaluation.

### Recommendations

- Define one physical domain convention (bounds, endpoint policy, and cell
  area), and have estimator posterior metadata describe its query grid.
- Keep posterior mean/variance distinct from controller densities. Convert with
  the designed positive transform and unit-integral normalization only at the
  estimator/posterior layer.
- Mask obstacles before density normalization if obstacle maps are in scope.
- Validate rectangular maps even if initial closed-loop tests remain square.

## 9. Configuration and entry points

### Facts

`HEDACParams` permits new nested keys without parser changes because it retains
the raw dictionary and supports dot-key lookup. Common values are also copied to
attributes at construction.

Current executable entry points are:

- `python examples/run_hedac.py --config ...` — standalone aerial GP-HEDAC;
- `python examples/run_gp_single_robot.py --config ...` — single-agent debug;
- `python examples/hierarchical.py --aerial_config ... --ground_config ...` —
  interactive coupled prototype;
- `python evaluation/run_evaluation.py ...` — repeated coupled evaluation;
- `python evaluation/aerial_baseline.py ...` and `ground_baseline.py ...` —
  baselines;
- `python evaluation/run_ablation.py ...` — team-count ablation;
- `python evaluation/egerstedt.py` — independent Voronoi experiment.

No installed console script is declared. `main.py` is not useful as an entry
point.

### Recommendations

- Add multifidelity settings under one clearly named nested section and read
  them through `HEDACParams.get()` initially; add validation at estimator
  construction.
- Use `estimator_mode: legacy` as the default so existing configurations retain
  behavior when the key is absent.
- Expose sensor and GP update periods in simulated-time units, with a documented
  conversion/tolerance policy for step scheduling.
- Keep aerial and ground observation noise variances distinct from kernel noise
  and jitter.

## 10. Plotting, logging, and evaluation

### Facts

- Standalone runs save ergodic metrics, final coverage/heat/positions, original
  goal density, and map to NPZ. Trajectories are returned in memory but not saved
  by the NPZ block.
- `visualize_gp_debug()` can save or show standalone GP frames and uses
  `HEDACAlgorithm`'s legacy observation arrays.
- Coupled evaluation can show target, fused estimate, both independent GP means
  and standard deviations, obstacles, and trajectories.
- `run_evaluation.py` currently saves effectiveness, KL divergence, and
  Wasserstein arrays. Saves for trajectories, target, and final fused density
  are commented out.
- Baseline and plotting scripts use inconsistent output filenames (including
  `final_aerial_density2.npy`) and assume directories/files already exist.
- No current output records timestamps, fidelity-specific sample counts, GP
  update/prediction time, high-fidelity posterior history, integrated
  uncertainty, controller density history, or aerial command changes.

### Recommendations

- Extend evaluation logging outside the GP core and record all fields required
  by the design, including estimator version/timestamp and the density actually
  consumed by each controller.
- Add a minimal deterministic closed-loop artifact that records before/after
  ground observation, posterior, aerial target, and aerial command/trajectory so
  the required causal chain is inspectable numerically.
- Retain current legacy metric/output names where consumers rely on them, and add
  mode-qualified outputs rather than silently changing meanings.

## 11. Tests and numerical dependencies

### Existing tests: facts

- No files matching test naming conventions and no `tests/` directory are
  present.
- Pytest is declared as a development dependency, but there is no current suite
  to rerun.
- The repository contains executable evaluations and examples, not automated
  assertions for GP math, timing, failure handling, or closed-loop behavior.

### Dependencies: facts

- scikit-learn provides the current GP and internally uses stable factorization
  machinery, but the repository does not expose/configure the Cholesky and
  jitter behavior required for the new mathematical core.
- NumPy/SciPy are sufficient to implement the required dense joint covariance,
  Cholesky factorization, triangular solves, interpolation, and deterministic
  test references without a new GP framework.
- CasADi/IPOPT supports the ground MPC; Numba accelerates dynamics, GMM, and
  Voronoi/numerical utilities; Matplotlib handles plotting.
- `src/utils/eval_utils.py` and `evaluation/plots.py` import the Python Optimal
  Transport module `ot`, but POT is absent from `pyproject.toml` and `uv.lock`.
- `pyproject.toml` currently declares SciPy and CasADi, but the checked-in lock's
  root dependency list does not, so lock synchronization must be resolved before
  relying on reproducible validation commands.

### Recommendations

- Add deterministic pytest coverage before integration, beginning with a pure
  NumPy/SciPy mathematical core and a direct small joint-covariance reference.
- Use `numpy.linalg.cholesky` plus `scipy.linalg.solve_triangular` (or equivalent
  Cholesky solves), never an explicit inverse, in the new GP core.
- Keep current scikit-learn behavior untouched in legacy mode; do not force the
  new implementation into the legacy wrapper.
- Repair dependency/lock consistency as a separately reviewed support change
  when tests or evaluation require CasADi and POT.

## 12. Observation timing and missing asynchronous semantics

### Facts

Current timing is represented only by loop order and `step_num`:

- every active agent senses on every outer step;
- every observation is submitted immediately as part of one bulk array;
- aerial and ground sensing have no separate periods;
- there is no pending buffer;
- observations cannot express delay or out-of-order arrival;
- no data carry collection or arrival time;
- GP update calls occur every step, while `fit_interval` only controls fitting;
- controllers consume predictions made during that same procedural step;
- no cached posterior object carries version, validity, timestamp, sample counts,
  grid metadata, or update status.

### Recommendations

- Introduce an immutable observation record with collection timestamp, robot ID,
  position, scalar value, LOW/HIGH fidelity, and noise variance.
- Keep arrival order separate from collection timestamp so delayed and
  out-of-order tests are meaningful.
- Have the coupled simulation independently schedule aerial sensing, ground
  sensing, and estimator updates. The estimator should expose explicit
  `submit()` and `update()` operations and an immutable latest-posterior view.
- On failure, keep the last valid posterior and return/record a failed update
  status rather than clearing it or propagating failure into controller code.

## 13. Exact multifidelity integration points

### Recommendations

The following points minimize changes and preserve current control behavior:

1. **Pure GP core:** add a module under `src/core/` with no imports from
   `models`, `hedac`, plotting, evaluation, or CasADi. It should accept separate
   LOW/HIGH arrays and implement the designed block covariance and high-fidelity
   posterior through Cholesky solves.
2. **Observation and estimator layer:** add controller-independent observation,
   retention, posterior, and central estimator types under `src/core/`. Give the
   estimator the configured query grid rather than a robot or map-loader object.
3. **Configuration read:** branch on `estimator_mode` in coupled initialization.
   Missing mode selects `legacy`; `multifidelity` constructs exactly one central
   estimator.
4. **Aerial collection:** at the point where `HEDACAlgorithm.step()` currently
   calls `collect_observations()`, preserve that code only for legacy mode. In
   multifidelity mode, the coupled scheduler calls the same agent sensing seam
   (or the new low-fidelity sensor interface) on the aerial sensor schedule and
   submits LOW observations centrally.
5. **Ground collection:** replace the direct
   `ground_gp.collect_observations()`/`update_gp()` block in the coupled loop
   only in multifidelity mode. On the ground sensor schedule, collect HIGH
   observations and submit them to the same estimator.
6. **Estimator schedule:** immediately after the relevant submissions, call the
   central estimator only when its update period expires. Cache its last valid
   posterior in the simulation-owned estimator object.
7. **Aerial density:** before the next HEDAC heat/source update, derive the
   interest-plus-uncertainty aerial target from the cached posterior, resample it
   to the HEDAC map if necessary, validate it, and assign through the narrow
   current-density seam. Before the first valid posterior, retain the original
   target.
8. **Ground density:** where `combo_density` is currently computed, select the
   cached normalized high-fidelity density in multifidelity mode, resample it to
   `xy_mpc_grid`, and leave all subsequent Voronoi/MPC code unchanged. Before
   the first valid posterior, use the approved initial legacy density.
9. **Legacy preservation:** retain the current `HEDACAlgorithm.gp`, separate
   `ground_gp`, direct `gpr_model.predict()`, post-hoc weights, public aliases,
   controller order, and legacy evaluation path behind `legacy`.
10. **Validation hook:** record the posterior version used to form each aerial
    target and ground `W`, plus aerial guidance/command before and after a HIGH
    observation. This directly tests the required closed-loop chain.

## 14. Interfaces that should remain backward compatible

### Recommendations

- `HEDACParams.from_yaml()`, `.from_dict()`, `.get()`, and existing convenience
  attributes.
- Existing YAML files with no estimator-mode or multifidelity section.
- `MapLoader` and map conventions.
- `AgentTeam`, agent `position`/`theta`/history interfaces, and legacy
  `sense_environment_gp()` return arrays.
- `GaussianProcess` constructor, methods, attributes, and scikit-learn `model`
  alias for all legacy examples/evaluations.
- `HEDACAlgorithm.step()`, `run()`, `current_goal_density`, and result keys.
- `init_hedac_from_params()` and `init_mpc_from_params()` return shapes unless a
  later plan introduces an additive wrapper rather than a breaking change.
- Ground solver parameter layout `[robot_state, full_grid_weights]`, objective,
  constraints, warm start, and update rate.
- Current CLI flags and default config paths.
- Existing legacy output names needed by baseline/plot scripts.

## 15. Technical risks

### Facts and implications

1. **No current aerial feedback from ground data.** The mandatory causal chain
   cannot be demonstrated without changing the aerial density source.
2. **Sensor semantics are not genuinely multifidelity.** A joint autoregressive
   model could appear to work numerically while modeling only different noise.
3. **Dataset cost.** Current caps allow 500 samples in each independent GP; a
   dense joint model can require factorization of up to the combined retained
   count. Update cadence must be independent of controller cadence.
4. **Duplicate policy loses measurements.** Integer rounding plus “keep first”
   removes later values from the same cell and cannot preserve independent noise
   or HIGH corrections at coincident LOW locations.
5. **Failure semantics conflict with the design.** Current code clears or throws
   instead of retaining the latest valid posterior.
6. **Uncalibrated legacy fusion/density.** Independently normalized std values,
   inverse-std weights, possible negative means, and missing unit-integral
   normalization make controller comparisons sensitive to scale.
7. **Configuration drift.** Several advertised GP parameters are unused and the
   dependency lock is stale relative to project metadata.
8. **Grid inconsistency.** Array-index coordinates, physical resolution,
   inclusive/exclusive endpoints, and width/height naming are inconsistent.
9. **Obstacle mismatch.** Sensors and estimator grids do not reject obstacle
   locations, while HEDAC density operations mask obstacles in only some paths.
10. **Initialization behavior.** `fit_interval` can report a successful prior
    prediction before the first fit; a new estimator must define exactly when a
    posterior becomes valid.
11. **Procedural duplication.** Coupled, baseline, and ablation loops copy
    substantial logic, so updating only one path could invalidate comparisons.
12. **Ground MPC assumptions.** The “local” grid is actually a fixed full grid,
    and current unicycle control bounds are derived from `max_acceleration` rather
    than its velocity/turn limits. These are pre-existing behaviors and should
    not be changed incidentally during density integration.
13. **Square-map dependence.** Swapped width/height assignments and hard-coded
    50-unit plot bounds can fail or misalign on rectangular/non-default maps.
14. **Missing automated regression baseline.** There are no tests proving the
    legacy path currently runs or remains numerically identical.

## 16. Unclear behavior requiring design decisions

### Facts needing a decision before implementation

1. **Definition of the two latent truths:** whether the GMM is `f_H`, and how
   the simulated `f_L` is derived from it.
2. **Aerial footprint:** kernel/shape, footprint scale, boundary handling, and
   whether it depends on altitude (altitude is not currently a robot state).
3. **`rho`:** fixed configured value versus later optimization, and whether a
   discrepancy mean/bias is required.
4. **Observation clock:** whether periods are specified in seconds or integer
   steps, and how noninteger period/dt ratios are scheduled.
5. **Arrival semantics:** whether delayed observations become eligible at
   arrival time regardless of their older collection timestamp, and whether
   future-dated observations are rejected or held.
6. **Same-step ordering:** whether a ground observation affects ground control
   immediately and aerial control on the next step (preserves current team
   order), or whether all sensing precedes both controllers.
7. **Retention:** exact minimum spatial separation, age policy, and deterministic
   tie-breaking for coincident LOW/HIGH observations.
8. **Noise units:** migration from existing `obs_noise_std` values to the
   variance required by the model and observation record.
9. **Grid and normalization measure:** cell-center versus endpoint grid,
   obstacle mask, and whether unit integral includes physical cell area.
10. **Pre-posterior controller inputs:** the original true target, legacy
    GP-derived target, or another configured prior for each controller.
11. **Hyperparameters:** fixed kernels for deterministic first implementation
    versus retaining scikit-learn optimization only in legacy mode.
12. **High-only operation:** prior assumptions for unobserved `f_L` when only
    HIGH data exist.
13. **Legacy scope:** which of `hierarchical.py`, `run_evaluation.py`, baselines,
    and ablation must expose the mode initially, versus one canonical coupled
    entry point followed by migration.
14. **Evaluation ground truth:** whether metrics compare posterior mean,
    normalized controller density, or both against `f_H` and its normalized
    density separately.

## 17. Current required-chain gap

### Fact

The repository currently implements only:

```text
ground observation
  -> independent ground GP
  -> post-hoc fused map
  -> ground MPC weights
  -> ground command
```

The aerial path remains driven exclusively by aerial observations. There is no
discrepancy process and no route from a ground observation to the aerial target,
command, or trajectory. Therefore the completion condition in `AGENTS.md` is not
currently met.

### Recommendation

Make the first closed-loop acceptance test explicitly compare otherwise
identical deterministic runs immediately before and after adding one informative
HIGH observation, and assert all four transitions:

```text
HIGH observation
  -> changed high-fidelity posterior mean and/or variance
  -> changed aerial target density
  -> changed HEDAC gradient/command or short aerial trajectory
  -> same posterior density supplied to ground MPC
```

This should be a numerical automated test, with plotting only as supplementary
evidence.
