# `configs/multifidelity.yaml` parameter reference

This document describes every leaf parameter in the canonical coupled
configuration, its current runtime scope, and the expected effect of changing
it. It reflects the implementation as of 2026-07-16, including parameters that
are retained for legacy compatibility but are not currently wired into the
multifidelity runner.

Status legend:

- **MF active** — used by the current central multi-fidelity run.
- **Both** — used in both `multifidelity` and `legacy` modes.
- **Legacy only** — relevant only when the old per-team GP/fusion path runs.
- **Inactive** — present in the YAML but currently has no behavioral effect in
  `examples.run_multifidelity`; this is stated explicitly rather than inferred.

Unless stated otherwise, time periods are simulated seconds, positions and
distances are grid-coordinate units, angles in the YAML are degrees, GP
`variance`/`noise_variance` values are variances rather than standard
deviations, and counts must be integers. Shared top-level settings are copied
into both robot parameter views; `aerial` and `ground` recursively override
their respective view.

## Mode and simulation

| Parameter | Current value | Status | Meaning | Expected effect when changed |
|---|---:|---|---|---|
| `estimator_mode` | `multifidelity` | Both | Selects `multifidelity` central GP or `legacy` separate GPs plus post-hoc fusion. | Categorical, not ordered. Changing to `legacy` bypasses the central estimator and its feedback. |
| `simulation.num_steps` | `300` | Both | Default number of outer coupled-loop steps. | Higher runs longer and collects more data, but costs more. It does not change `dt`. |
| `simulation.dt` | `0.1` | Both | Simulated time between outer steps; also used by HEDAC heat integration and ground MPC prediction. | Lower gives finer scheduling/integration and more steps per simulated second. Higher is cheaper but coarser; CFL limiting may reduce only the heat-update substep. |
| `simulation.random_seed` | `42` | Both | Base seed for truth, initial states, and deterministic sensor streams. | No monotonic effect; a different value produces a different but reproducible run. CLI `--seed` overrides it. |
| `simulation.num_episodes` | `1` | Both | Default episode count for `evaluation.run_multifidelity_evaluation`. | Higher improves aggregate robustness estimates and increases runtime roughly proportionally. |

## Map and simulated truth

| Parameter | Current value | Status | Meaning | Expected effect when changed |
|---|---:|---|---|---|
| `map.size` | `[50, 50]` | Both | Shared map raster `[height, width]` used to construct the environment. | Larger maps increase travel distances, HEDAC grid work, and potentially reconstruction difficulty. |
| `map.resolution` | `1.0` | Both | Aerial/HEDAC physical cell size; affects map area, heat normalization, and diffusion discretization. | Higher means larger physical area per cell and permits a larger CFL heat substep. Keep coordinate/unit assumptions consistent. |
| `goal_density.type` | `gaussian_mixture` | Inactive | Intended truth-field family selector. The coupled builder currently always creates a random Gaussian mixture. | No current effect; other strings are not interpreted by this runner. |
| `goal_density.num_peaks` | `2` | Both | Number of components in the generated Gaussian-mixture HIGH truth. | Higher generally makes the target more multimodal and harder to reconstruct/cover. |

## HEDAC heat equation

The implemented update is approximately
`dT/dt = alpha*Laplacian(T) + source_strength*source - beta*T - local_cooling*cooling`.

| Parameter | Current value | Status | Meaning | Expected effect when changed |
|---|---:|---|---|---|
| `heat_equation.alpha` | `1.0` | Both | Heat diffusion coefficient. | Higher spreads attraction more broadly/faster but tightens the explicit CFL limit; lower keeps heat more local. Zero disables diffusion. |
| `heat_equation.source_strength` | `1.0` | Both | Gain on uncovered-target heat injection. | Higher strengthens attraction to under-covered regions; lower weakens it relative to diffusion/decay. |
| `heat_equation.beta` | `0.1` | Both | Global heat-decay coefficient, normalized by map area. | Higher removes old heat faster and makes response more local/reactive; lower retains heat longer. |
| `heat_equation.local_cooling` | `0.1` | Inactive | Intended gain for cooling around recently visited locations. | No current effect: the local-cooling raster is reset but its accumulation is commented out. |
| `heat_equation.cfl_safety` | `1.0` | Both | Multiplier in the stability cap `dt_heat <= safety*dx^2/(4*alpha)`. | Lower is more conservative/stable but diffuses less per outer step. Higher allows larger heat steps; values above the stable range are risky. |

## Visualization and output

| Parameter | Current value | Status | Meaning | Expected effect when changed |
|---|---:|---|---|---|
| `visualization.plot_frequency` | `10` | Inactive | Intended interval for interactive/intermediate plots. | No current effect in the coupled CLI. |
| `visualization.save_video` | `true` | Inactive | Requests video recording. | No video renderer is implemented; `true` only causes an explicit CLI warning. |
| `visualization.video_fps` | `30` | Inactive | Intended output video frame rate. | No current effect until video recording exists. |
| `visualization.video_path` | `output/multifidelity_simulation.mp4` | Inactive | Intended video destination. | No current effect until video recording exists. |
| `visualization.save_final_plot` | `true` | MF active | Enables the four-panel final posterior/trajectory PNG. | `false` skips rendering and reduces end-of-run work. CLI `--no-plot` also overrides it. |
| `visualization.final_plot_path` | `output/multifidelity_final_state.png` | MF active | Destination for the final-state PNG. | Path only; changing it does not affect simulation behavior. |
| `visualization.gp_debug` | `false` | Both | Enables HEDAC GP-debug frames during HEDAC steps. | `true` adds plotting overhead. In MF mode the legacy aerial GP is not updated, so these are not the central-GP milestone plots. |
| `visualization.gp_debug_interval` | `100` | Both | Step interval between HEDAC GP-debug frames. | Higher saves fewer frames and is cheaper; lower saves/shows more often. |
| `visualization.save_gp_frames` | `true` | Both | Saves enabled HEDAC debug views instead of showing them interactively. | Boolean mode choice; meaningful only when `gp_debug` is true. |
| `visualization.gp_output_dir` | `output/gp_debug` | Both | Directory for enabled HEDAC debug frames. | Path only; no control/estimation effect. |
| `output.save_results` | `true` | Inactive | Intended NPZ result-recording switch. | No current effect in the coupled CLI; structured recording is a future milestone. |
| `output.results_path` | `output/results.npz` | Inactive | Intended default NPZ destination. | No current effect; the evaluation CLI uses its explicit `--output` argument. |
| `output.verbose` | `true` | Inactive | Intended generic verbosity switch. | No current effect in the coupled CLI. Use `--log-every` for live summaries. |
| `output.print_frequency` | `100` | Inactive | Legacy `HEDACAlgorithm.run()` print interval. | No current effect because the coupled runner calls `step()` directly. |

## Central multi-fidelity estimator

| Parameter | Current value | Status | Meaning | Expected effect when changed |
|---|---:|---|---|---|
| `multifidelity.rho` | `0.8` | MF active | Autoregressive coupling in `f_H = rho*f_L + delta`; also used to construct simulator discrepancy truth. | Larger absolute values transfer LOW information more strongly into HIGH. Near zero decouples fidelities and makes HIGH rely on discrepancy data. Values around `0..1` are easiest to interpret. |
| `multifidelity.gp_update_period` | `1.0` | MF active | Period between central estimator update attempts. | Higher reduces GP cost but increases posterior/control latency; lower updates more responsively and costs more. |
| `multifidelity.aerial_sensor_period` | `0.5` | MF active | Period between LOW collection events. | Higher yields fewer LOW samples; lower increases sample rate and buffer/fit load. |
| `multifidelity.ground_sensor_period` | `0.1` | MF active | Period between HIGH collection events. | Higher yields fewer ground corrections; lower gives faster local correction but increases cost and can fill retention quickly. |
| `multifidelity.low_kernel.length_scale` | `5.0` | MF active | Initial LOW RBF correlation distance. With fitting enabled, this is the optimizer's starting value. | Higher assumes a smoother, longer-range LOW field; lower allows more local variation and can look scattered. |
| `multifidelity.low_kernel.variance` | `1.0` | MF active | Initial LOW GP signal variance. | Higher permits larger LOW amplitudes and prior uncertainty; lower shrinks the LOW component. |
| `multifidelity.discrepancy_kernel.length_scale` | `2.0` | MF active | Initial spatial scale of `delta`, the HIGH-minus-scaled-LOW correction. | Higher makes corrections broad/smooth; lower permits sharp local corrections and may produce patchier estimates. |
| `multifidelity.discrepancy_kernel.variance` | `0.25` | MF active | Initial signal variance allocated to the discrepancy process. | Higher allows HIGH to depart more from `rho*f_L`; lower forces stronger agreement with scaled LOW. |
| `multifidelity.low_noise_variance` | `0.04` | MF active | Known variance attached to each simulated aerial/LOW observation (`std = 0.2`). | Higher trusts LOW less and retains more uncertainty; lower fits LOW data more closely and risks following noise/model mismatch. |
| `multifidelity.high_noise_variance` | `0.001` | MF active | Known variance attached to each ground/HIGH observation (`std ≈ 0.0316`). | Higher softens local ground corrections; lower makes HIGH samples dominate nearby and can create sharp local features. |
| `multifidelity.jitter` | `1e-8` | MF active | Initial diagonal numerical regularizer for Cholesky factorization. | Higher improves conditioning but increasingly regularizes the covariance; lower perturbs less but may fail on duplicate/near-duplicate data. |
| `multifidelity.max_jitter_attempts` | `5` | MF active | Maximum Cholesky attempts while escalating jitter. | Higher tolerates harder numerical cases with a small failure-path cost; it does not affect successful first attempts. |
| `multifidelity.jitter_multiplier` | `10.0` | MF active | Factor applied to jitter after each failed Cholesky attempt; must exceed one. | Higher reaches a stable value faster but in coarser jumps; lower explores regularization more gradually and may need more attempts. |

### Retention, controller density, and simulator fidelity

The shared multifidelity density is not a selectable transformation: it is
always the quadrature-normalized positive part `max(high_mean, 0)`. This density
drives both ground controllers and the aerial target's interest component;
posterior uncertainty enters only through the separate aerial uncertainty term.

| Parameter | Current value | Status | Meaning | Expected effect when changed |
|---|---:|---|---|---|
| `multifidelity.retention.max_low_samples` | `250` | MF active | Maximum retained LOW observations. | Higher can preserve more information but GP work/memory rises strongly (dense fitting is roughly cubic in total samples). Lower is faster but forgets more. |
| `multifidelity.retention.max_high_samples` | `250` | MF active | Maximum retained HIGH observations. | Same tradeoff as LOW; too low can discard important local corrections. |
| `multifidelity.retention.min_low_separation` | `0.25` | MF active | Minimum spatial distance between retained LOW points (newest candidates selected first). | Higher favors spatial diversity and fewer clustered points; lower retains denser local samples and can worsen conditioning. |
| `multifidelity.retention.min_high_separation` | `0.10` | MF active | Minimum spatial distance between retained HIGH points. | Higher spreads retained corrections spatially; lower preserves fine local ground detail but admits more near-duplicates. |
| `multifidelity.aerial_target.lambda_interest` | `1.0` | MF active | Weight on normalized estimated HIGH density in the HEDAC target. | Higher relative to uncertainty favors exploiting estimated important regions. Multiplying both lambdas by the same factor has no effect after normalization. |
| `multifidelity.aerial_target.lambda_uncertainty` | `0.25` | MF active | Weight on spatially normalized HIGH posterior standard deviation. | Higher relative to interest promotes exploration of uncertain regions; zero disables uncertainty-driven visitation. |
| `multifidelity.density.normalization_tolerance` | `1e-8` | MF active | Relative/absolute tolerance when checking weighted density integral equals one. | Higher accepts larger floating-point normalization error; lower is stricter and may reject numerically harmless deviations. It does not smooth the field. |
| `multifidelity.sensor.low_fidelity_smoothing_sigma_cells` | `2.0` | MF active | Gaussian blur sigma used only to create simulator LOW truth from HIGH truth. | Higher makes LOW broader/smoother and moves more structure into discrepancy; zero makes LOW equal HIGH before `rho` scaling. |
| `multifidelity.sensor.random_seed_offset` | `10000` | MF active | Separates LOW/HIGH sensor RNG streams from the base episode seed. | No monotonic effect; changing it gives different deterministic sensing/noise sequences without changing initial states. |

### Kernel hyperparameter optimization

| Parameter | Current value | Status | Meaning | Expected effect when changed |
|---|---:|---|---|---|
| `multifidelity.hyperparameter_optimization.enabled` | `true` | MF active | Enables bounded empirical-Bayes fitting of the four RBF parameters. | `false` keeps the configured kernel values fixed; `true` adds optimization cost and adapts smoothness/amplitudes to retained data. |
| `multifidelity.hyperparameter_optimization.fit_interval_updates` | `5` | MF active | Number of successfully published estimator versions between fitting opportunities. Version zero is also an opportunity if enough data exist. | Higher reuses parameters longer and is cheaper; lower adapts more often and is more expensive. |
| `multifidelity.hyperparameter_optimization.min_samples` | `40` | MF active | Minimum combined retained LOW+HIGH count before fitting is allowed. | Higher delays fitting until evidence is stronger; lower adapts earlier but is more vulnerable to clustered/sparse-data estimates. |
| `multifidelity.hyperparameter_optimization.num_restarts` | `1` | MF active | Number of deterministic additional L-BFGS-B start points beyond the current parameter vector. | Higher can escape poorer local optima but scales optimization cost roughly with starts. Zero uses only the current vector. |
| `multifidelity.hyperparameter_optimization.max_iterations` | `75` | MF active | Per-start L-BFGS-B iteration limit. | Higher may converge more fully but can cost more; lower caps latency and may stop before the best likelihood. |
| `multifidelity.hyperparameter_optimization.bounds.low_length_scale` | `[1.0, 20.0]` | MF active | Allowed interval for fitted LOW length scale. | Raising the lower bound enforces smoothness; lowering it permits local/scattered structure. A wider range is more flexible but harder to regularize. |
| `multifidelity.hyperparameter_optimization.bounds.low_variance` | `[0.05, 5.0]` | MF active | Allowed interval for fitted LOW signal variance. | Higher limits permit larger LOW amplitude; a higher lower bound prevents the LOW process from being nearly suppressed. |
| `multifidelity.hyperparameter_optimization.bounds.discrepancy_length_scale` | `[1.5, 10.0]` | MF active | Allowed interval for fitted discrepancy length scale. | Raising the lower bound forces smoother corrections; lowering it permits sharper corrections. |
| `multifidelity.hyperparameter_optimization.bounds.discrepancy_variance` | `[0.01, 2.0]` | MF active | Allowed interval for fitted discrepancy signal variance. | A lower lower-bound lets the optimizer nearly remove discrepancy; a higher upper-bound permits larger HIGH/LOW mismatch. Boundary hits are useful diagnostics. |

## Aerial robots and HEDAC

| Parameter | Current value | Status | Meaning | Expected effect when changed |
|---|---:|---|---|---|
| `aerial.simulation.num_agents` | `3` | Both | Number of aerial robots. | Higher collects more LOW samples per event and covers more area, with more controller/sensor cost. |
| `aerial.agents.model_type` | `dubins` | Both | Aerial dynamics class (`dubins`, `unicycle`, or fallback double integrator). | Categorical. Changing it changes which motion parameters below are active and changes trajectories substantially. |
| `aerial.agents.max_velocity` | `3.0` | Limited | HEDAC target-velocity scale. For the current Dubins tracker only its direction is used. | Little/no trajectory effect with `dubins`; it becomes a true speed cap for double-integrator agents. |
| `aerial.agents.max_acceleration` | `0.5` | Inactive for Dubins | Double-integrator acceleration cap. | No current Dubins effect; higher makes double-integrator motion more responsive. |
| `aerial.agents.max_angular_velocity` | `0.785` | Inactive for Dubins | Double-integrator/unicycle angular-rate cap. Dubins rate comes from speed and bank angle. | No current Dubins effect. |
| `aerial.agents.max_angular_acceleration` | `0.393` | Inactive for Dubins | Double-integrator angular-acceleration cap. | No current Dubins effect. |
| `aerial.agents.dt_agent` | `0.1` | Both | Aerial dynamics integration step. | Higher moves farther per outer call and integrates headings more coarsely. Normally keep equal to `simulation.dt`. |
| `aerial.agents.agent_radius` | `10.0` | Both | Spread parameter of the HEDAC coverage footprint. | Higher creates a broader footprint, so one visit covers a larger neighborhood. |
| `aerial.agents.min_kernel_val` | `1e-6` | Both | Tail cutoff used to choose HEDAC coverage-footprint array size. | Lower retains farther Gaussian tails and makes a larger/more expensive footprint; higher truncates it sooner. |
| `aerial.agents.wall_avoidance_weight` | `1.0` | Inactive in current gradient | Intended obstacle-repulsion weight passed to `calculate_gradient`. That current function implements boundary repulsion but ignores this argument. | No current obstacle-avoidance effect. |
| `aerial.agents.dubins.forward_speed` | `10.0` | Both | Constant Dubins translational speed. | Higher moves farther per step and, at fixed bank angle, lowers maximum turn rate/increases turn radius. |
| `aerial.agents.dubins.max_bank_angle` | `60.0` | Both | Bank-angle limit used to derive `max_turn_rate = g*tan(bank)/speed`. | Higher permits tighter/faster turns; keep below the tangent singularity near 90 degrees. |
| `aerial.sensor.fov_degrees` | `360.0` | MF active | Angular span over which each LOW sensor event distributes samples. | Higher gives broader directional coverage; lower focuses samples around heading. Valid range is `(0, 360]`. |
| `aerial.sensor.fov_depth` | `10.0` | MF active | Maximum LOW sampling distance from each aerial robot. | Higher observes farther and improves spatial spread; lower concentrates samples locally. |
| `aerial.multi_agent.sensing_range` | `10.0` | Inactive | Stored neighbor-detection range. The coupled loop does not call neighbor updates. | No current effect. |
| `aerial.multi_agent.min_safe_distance` | `1.0` | Inactive | Intended minimum inter-robot separation. | No current effect; no collision constraint reads it. |

### Aerial legacy-GP compatibility keys

`aerial.gpr.obs_per_step` is also reused by the central LOW sensor. The other
keys in this table belong to the old aerial GP or are known inactive wiring.

| Parameter | Current value | Status | Meaning | Expected effect when changed |
|---|---:|---|---|---|
| `aerial.gpr.sigma_f` | `1.0` | Inactive | Intended legacy GP signal scale; read into an attribute, but the sklearn kernel is currently constructed with hard-coded initial values. | No current effect. |
| `aerial.gpr.length_scale` | `1.0` | Inactive | Intended legacy GP RBF length scale; not applied to the constructed sklearn kernel. | No current effect. |
| `aerial.gpr.noise_level` | `0.2` | Inactive | Intended legacy kernel white-noise level; not applied to the constructed kernel. | No current effect. |
| `aerial.gpr.gamma` | `0` | Inactive | Intended mean/uncertainty mixing control in the legacy target. The current combination function ignores its `gamma` argument. | No current effect. |
| `aerial.gpr.obs_noise_std` | `0.2` | Legacy only | Standard deviation of synthetic samples collected by the legacy aerial GP. | Higher makes legacy observations noisier. Central LOW noise instead uses `multifidelity.low_noise_variance`. |
| `aerial.gpr.obs_per_step` | `10` | MF active | Number of LOW samples per aerial robot at each LOW event; also legacy samples per step. | Higher improves sampling throughput but fills buffers and increases GP cost faster. |
| `aerial.gpr.min_samples` | `10` | Legacy only | Minimum legacy dataset size before prediction. | Higher delays legacy GP availability; lower predicts sooner with less evidence. |
| `aerial.gpr.use_filter` | `true` | Legacy only | Enables legacy uncertainty-based sample filtering. | `false` accepts all legacy samples; `true` uses the thresholds below. |
| `aerial.gpr.take_threshold` | `0.15` | Legacy only | Legacy normalized-uncertainty threshold for accepting new data. | Higher accepts fewer, more uncertain points; lower accepts more points. |
| `aerial.gpr.remove_threshold` | `-1` | Legacy only | Legacy threshold for removing retained points whose uncertainty has fallen low; nonpositive disables removal. | Positive/higher values remove more existing points. |
| `aerial.gpr.max_dataset_size` | `500` | Legacy only | Maximum legacy aerial GP dataset size. | Higher retains more history but makes legacy fits slower; lower forgets sooner. |
| `aerial.gpr.fit_hyperparams` | `true` | Inactive | Intended legacy optimizer switch, but current `GaussianProcess` does not read it. | No current effect. |
| `aerial.gpr.fit_interval` | `10` | Legacy only | Minimum legacy simulation-step gap between sklearn fits when new data exist. | Higher fits less often; lower adapts more often and costs more. |

## Ground robots and selectable coverage controller

| Parameter | Current value | Status | Meaning | Expected effect when changed |
|---|---:|---|---|---|
| `ground.controller.type` | `lloyd` | Both | Selects the existing limited-FOV receding-horizon `mpc` law or density-weighted centroidal-Voronoi `lloyd` law. Both use the same density, grid, dynamics, and update rate. If omitted, the builder defaults to `mpc` for backward compatibility. | Categorical. `mpc` preserves prior behavior and optimization cost; `lloyd` removes the nonlinear solve and tracks each weighted cell centroid. |
| `ground.simulation.num_agents` | `7` | Both | Number of ground robots. | Higher collects more HIGH samples and solves more MPC problems per step; runtime rises substantially. |
| `ground.simulation.num_obstacles` | `0` | Inactive | Intended random-obstacle count. The coupled builder currently creates an obstacle-free map and does not instantiate these obstacles. | No current effect. |
| `ground.simulation.obstacles_radius` | `1.0` | Inactive | Intended radius for generated ground obstacles. | No current effect. |
| `ground.agents.model_type` | `unicycle` | Both | Ground dynamics model used by the agent and MPC. | Categorical; changing it requires compatible MPC state/control dimensions. |
| `ground.agents.max_velocity` | `4.0` | Both | Unicycle linear-speed clip. | Higher permits faster motion if the MPC command reaches it; current MPC controls are already bounded to ±`max_acceleration` (`2.5`). |
| `ground.agents.max_acceleration` | `2.5` | Both | Current ground MPC control bound applied to both linear velocity and angular velocity, despite the name. | Higher allows more aggressive commands and may reduce solver robustness/safety; lower slows both translation and turning. |
| `ground.agents.max_angular_velocity` | `5.0` | Both | Unicycle angular-rate clip. | Higher permits faster turning if not already limited by the MPC's `max_acceleration` bound. |
| `ground.agents.max_angular_acceleration` | `3.0` | Inactive for Unicycle | Double-integrator angular-acceleration limit. | No current unicycle/MPC effect. |
| `ground.agents.dt_agent` | `0.1` | Both | Ground agent state-integration step. | Higher executes larger/coarser state changes. Keep equal to `simulation.dt`, which is used inside the MPC model. |
| `ground.agents.agent_radius` | `7.0` | Inactive for ground MPC | HEDAC coverage-footprint setting, but ground robots do not use HEDAC motion. | No current ground effect. |
| `ground.agents.min_kernel_val` | `1e-6` | Inactive for ground MPC | HEDAC footprint cutoff inherited into ground parameters. | No current ground effect. |
| `ground.agents.wall_avoidance_weight` | `1.0` | Inactive for ground MPC | HEDAC gradient parameter, not used by ground MPC. | No current ground effect. |
| `ground.sensor.fov_degrees` | `360.0` | MF active | Angular span of HIGH samples and, for MPC only, half-angle of the coverage-cost FOV. Lloyd uses the full Voronoi cell independently of heading/FOV. | Higher broadens sensing and MPC visible coverage; with Lloyd it changes sensing only. |
| `ground.sensor.fov_depth` | `7.0` | MF active | Maximum HIGH sample distance and, for MPC only, coverage-cost radius. Lloyd uses it for sensing but not centroid geometry. | Higher observes farther and increases MPC reach; with Lloyd it changes sensing only. |
| `ground.multi_agent.sensing_range` | `10.0` | Inactive | Stored neighbor-detection range. | No current effect because neighbor updates are not called. |
| `ground.multi_agent.min_safe_distance` | `1.0` | Inactive | Intended ground inter-robot safety distance. | No current effect; MPC has no collision constraint using it. |

### Ground legacy-GP compatibility keys

`ground.gpr.obs_per_step` is also reused by the central HIGH sensor. The central
GP uses `multifidelity.high_noise_variance`, not the legacy noise keys.

| Parameter | Current value | Status | Meaning | Expected effect when changed |
|---|---:|---|---|---|
| `ground.gpr.sigma_f` | `1.0` | Inactive | Intended legacy ground GP signal scale; not applied to its constructed sklearn kernel. | No current effect. |
| `ground.gpr.length_scale` | `1.0` | Inactive | Intended legacy ground GP RBF length; not applied to its constructed kernel. | No current effect. |
| `ground.gpr.noise_level` | `0.01` | Inactive | Intended legacy ground kernel noise; not applied to its constructed kernel. | No current effect. |
| `ground.gpr.gamma` | `0` | Inactive | Intended legacy mean/uncertainty mixing value; ignored by the current combination formula. | No current effect. |
| `ground.gpr.obs_noise_std` | `0.01` | Legacy only | Standard deviation of legacy ground observations. | Higher makes legacy HIGH samples noisier; central HIGH noise uses the multifidelity variance instead. |
| `ground.gpr.obs_per_step` | `10` | MF active | Number of HIGH samples per ground robot at each HIGH event; also legacy samples per step. | Higher corrects more locations but fills the HIGH buffer and raises GP cost faster. |
| `ground.gpr.min_samples` | `10` | Legacy only | Minimum legacy ground dataset size before prediction. | Higher delays legacy availability; lower predicts with fewer samples. |
| `ground.gpr.use_filter` | `true` | Legacy only | Enables legacy uncertainty-based filtering. | `false` retains all samples; `true` uses the take/remove thresholds. |
| `ground.gpr.take_threshold` | `0.15` | Legacy only | Legacy normalized-uncertainty threshold for new samples. | Higher keeps fewer new points; lower keeps more. |
| `ground.gpr.remove_threshold` | `-1` | Legacy only | Legacy retained-point removal threshold; nonpositive disables it. | Positive/higher values remove more low-uncertainty retained points. |
| `ground.gpr.max_dataset_size` | `500` | Legacy only | Maximum legacy ground GP dataset size. | Higher retains more data and costs more; lower bounds legacy fitting more tightly. |
| `ground.gpr.fit_hyperparams` | `true` | Inactive | Intended legacy optimizer switch; not read by the current GP wrapper. | No current effect. |
| `ground.gpr.fit_interval` | `10` | Legacy only | Minimum step gap between legacy ground sklearn fits. | Higher is cheaper/slower to adapt; lower fits more often. |

### Ground query grid and controller settings

| Parameter | Current value | Status | Meaning | Expected effect when changed |
|---|---:|---|---|---|
| `ground.map.resolution` | `0.1` | Inactive in coupled grid | Ground parameter-view resolution. The coupled query grid spans map dimensions using `local_grid_points` and does not read this override. | No current coupled-run effect. |
| `ground.map.local_grid_points` | `100` | Both | Number of query points per axis for density and either controller (`100 × 100 = 10,000` points). | Higher improves Voronoi quadrature and field resolution but increases GP prediction, centroid, and especially MPC work; lower is faster and blockier. |
| `ground.mpc.horizon` | `2` | Both, MPC only | Number of forward model stages in each MPC problem. Ignored by Lloyd. | Higher adds foresight and optimization variables, usually increasing solve time/nonlinearity; lower is more myopic and faster. |
| `ground.mpc.max_iterations` | `300` | Both, MPC only | IPOPT iteration cap per MPC solve. Ignored by Lloyd. | Higher gives difficult solves more opportunity but can increase worst-case latency; lower fails/stops sooner. |
| `ground.mpc.print_time` | `false` | Both, MPC only | Enables CasADi solver timing output. Ignored by Lloyd. | Logging only; `true` produces more console output. |
| `ground.mpc.tolerance` | `1e-4` | Both, MPC only | IPOPT convergence tolerance. Ignored by Lloyd. | Lower is stricter and may require more iterations; higher is faster/looser and can reduce solution accuracy. |
| `ground.lloyd.position_gain` | `1.0` | Both, Lloyd only | Gain from distance-to-centroid to commanded linear speed. | Higher approaches the speed cap sooner and converges more aggressively; lower produces slower, smoother translation. |
| `ground.lloyd.heading_gain` | `2.0` | Both, Lloyd only | Gain from wrapped centroid-bearing error to angular velocity. | Higher aligns faster but can create sharper turns at coarse `dt`; lower turns more gradually. |
| `ground.lloyd.centroid_tolerance` | `0.05` | Both, Lloyd only | Distance within which both Lloyd commands are set to zero. | Higher reduces small motions but stops farther from the discrete centroid; lower tracks more precisely. |
| `ground.lloyd.max_linear_velocity` | `4.0` | Both, Lloyd only | Controller-level absolute linear-speed cap; the unicycle agent also applies `agents.max_velocity`. | Higher is effective only up to the agent cap; lower directly slows centroid tracking. |
| `ground.lloyd.max_angular_velocity` | `5.0` | Both, Lloyd only | Controller-level absolute angular-rate cap; the unicycle agent also applies `agents.max_angular_velocity`. | Higher is effective only up to the agent cap; lower produces wider/slower turns. |

## Practical tuning order

For reconstruction smoothness, start with the fitted kernel values and whether
they hit their bounds, then adjust hyperparameter bounds, sensor-noise
variances, and retention. For computational cost, tune sample counts/periods,
retention maxima, hyperparameter fit interval/restarts, and
`ground.map.local_grid_points`. For robot behavior, first choose
`ground.controller.type`, then tune only that controller's section: FOV,
horizon, and control bounds for MPC, or centroid gains, tolerance, and caps for
Lloyd. Avoid tuning keys marked **Inactive**; they currently cannot change the
coupled result.
