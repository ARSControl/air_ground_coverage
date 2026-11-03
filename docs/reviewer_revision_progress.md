# Reviewer Revision Implementation Tracker

## Purpose

This document tracks the implementation, validation, and paper integration of
the requested revisions. It is a living checklist: an item may be marked
complete only when its code, deterministic tests, real-output artifact, and
paper-facing interpretation have all been verified.

Status labels used below:

- **Not started**: no revision-specific implementation exists.
- **Partial**: reusable infrastructure exists, but the reviewer request is not
  yet satisfied end to end.
- **Implemented**: production code and focused tests exist.
- **Validated**: repeated experiments and artifacts have been checked.
- **Integrated**: the result and its limitations appear in the paper.

## Revision summary

| Revision | Current status | Completion criterion |
| --- | --- | --- |
| Parameter sensitivity/ablation study | **Implemented** | The proposed-method-only runner covers all six declared factors and produces resolved configurations, immutable raw/evaluated archives, and figures. Repeated publication runs and paper integration remain. |
| Range sweep with a full-environment condition | **Implemented pipeline** | The proposed method has a diagonal-covering sweep condition, and a separate paired Proposed--Rudolph--Zhang full-visibility runner is smoke-verified. Repeated publication validation remains. |
| Long-duration comparison mitigating Zhang's initial delay | **Partial** | A predeclared horizon gives Zhang adequate post-deployment operating time, with phase-transition and convergence evidence reported. |
| Line-of-sight sensor occlusion | **Implemented** | LOW- and HIGH-fidelity samples are admitted only when they satisfy range, field-of-view, free-space, and line-of-sight constraints. Paper integration and repeated obstacle-scenario evaluation remain. |
| Real-world two-drone/one-quadruped demonstration | **Future work** | Experimental architecture, calibration, synchronization, safety, and evaluation protocol are specified; implementation is not required for this revision. |

The repository's existing `multifidelity` ablation varies information pathways
(`full`, `no_discrepancy`, `no_high_to_aerial`, and `no_uncertainty`). That is a
mechanism ablation and must remain distinct from the new parameter sensitivity
study requested by the reviewers.

## 1. Parameter sensitivity and ablation study

### 1.1 Experimental principle

Use a one-factor-at-a-time sensitivity study around one frozen reference
scenario. This avoids an impractically large full factorial and keeps each
effect interpretable. If interactions are later required, add a small,
predeclared secondary experiment rather than silently expanding the primary
sweep.

Every level within a sweep must use paired episode seeds and, wherever the
changed factor permits it, identical hidden fields, initial states, obstacle
realizations, and sensor-noise random streams. The experimental unit is an
episode, not a grid cell or time step. Publication results should use at least
20 paired episodes, preferably 30; smoke configurations may use one or two.

The frozen reference condition should specify:

- environment dimensions and grid resolution;
- aerial/ground team composition;
- mission duration and integration step;
- obstacle-generation rule and density;
- aerial and ground sensing ranges;
- LOW- and HIGH-fidelity measurement noise;
- estimator kernels, retention budget, update periods, and `rho`;
- controller parameters and initialization policy.

The retained-sample budget and scheduling policy must not change accidentally
with environment area, team size, or mission duration. Any intentional scaling
rule must be declared and archived.

### 1.2 Factors to sweep

The exact numerical levels must be frozen in configuration before the
publication runs. The entries below define the intended semantics and leave
values marked `TBD` where a short feasibility pilot is still needed.

| Factor | Proposed levels | Quantity held fixed | Current support | Status |
| --- | --- | --- | --- | --- |
| Environment size | 30x30 / reference 50x50 / 70x70 | Spatial resolution and density-generation rule | Implemented by the proposed-method parameter sweep. | **Implemented**, publication validation pending |
| Team composition | A1/G9 / reference A2/G8 / A5/G5 | Total team size of ten, with both fidelities present | Implemented by the proposed-method parameter sweep. Endpoint compositions remain in the dedicated composition study. | **Implemented**, publication validation pending |
| Duration | 10 s / reference 30 s / candidate long 60 s | Dynamics, sensing, controllers, and scenario realization | Implemented for proposed-method sensitivity. The separate Zhang convergence criterion is not yet validated. | **Implemented for sensitivity**, baseline validation pending |
| Obstacle density | 0 / reference 5 / 15 circular obstacles of radius 1 | Obstacle shape and radius | Implemented; resolved metadata records the nominal occupied-area fraction. | **Implemented**, publication validation pending |
| Sensing range | Short A5/G2.5 / reference A10/G5 / full-environment | Environment and all non-range parameters | Full range is resolved dynamically to the map diagonal for both fidelities; LOS remains active. | **Implemented for the proposed method** |
| Measurement noise | Matched low / reference / high for both fidelities | Sensor and GP noise variances change together | Implemented as the primary matched-noise experiment. | **Implemented**, publication validation pending |

For measurement noise, distinguish two scientifically different experiments:

1. **Matched noise:** change the simulated observation noise and the GP's
   corresponding noise variance together. This measures degradation with
   correctly specified noise.
2. **Noise misspecification:** change only the simulated noise while retaining
   the reference GP assumption. This measures robustness to calibration error.

The primary reviewer response should use matched noise. Misspecification is an
optional secondary study and must be labelled separately.

### 1.3 Full-environment range condition

Define a range as full-environment only when every point in the domain is
reachable from every admissible robot position. For a rectangular domain with
width $W$ and height $H$, use

$$
R_{\mathrm{full}} \geq \sqrt{W^2 + H^2}.
$$

The current 50-by-50 domain therefore requires at least
$\sqrt{50^2+50^2}\approx70.71$; the existing value 75 is a suitable explicit
full-range level. Archive both the configured radius and a Boolean check that
it covers the domain diagonal.

Required comparison rules:

- Run the proposed method and each range-compatible baseline on the same
  obstacle-free scenario archive.
- Use the same physical mission clock, truth, initial states, and ground sensing
  geometry.
- Keep standard Zhang labelled `full-cell`, because its paper-assumption
  information model is not a finite-radius sensor model.
- Do not treat the existing full-range Egerstedt-only override as completing
  this sweep; the proposed method also needs an explicit full-range condition.
- If occlusions are enabled, `full range` removes only the distance limitation;
  obstacles may still block line of sight. Report this distinction clearly.

### 1.4 Long-duration Zhang comparison

The long duration must be selected to mitigate Zhang's deployment delay, not
to hide it. Zhang's Algorithm 1 starts at mission time zero while ground robots
remain stationary; Algorithm 2 receives the remaining mission time. Continue
to include that initial delay in all full-mission metrics.

Before freezing the long horizon:

1. Run a small, seed-predeclared timing pilot that records Zhang's deployment
   transition time for every episode.
2. Select a horizon that leaves a substantial post-transition interval for all
   or a predeclared high fraction of episodes. A candidate 60-second horizon
   may be tested, but it is not accepted until the criterion below is met.
3. Freeze the horizon before comparing outcome metrics across methods.

Acceptance criterion for the selected horizon:

- at least 95% of Zhang episodes enter Algorithm 2;
- report the median, 95th percentile, and maximum transition times;
- report both complete-mission metrics and post-transition metrics using the
  same transition convention;
- show that conclusions are not determined solely by the deployment transient,
  using time histories or a predeclared terminal window;
- never remove the initial delay from the primary complete-mission comparison.

“Convergence” must be tied to an archived numerical condition. Deployment
convergence should use Zhang's existing deployment tolerance. If steady
coverage convergence is claimed, define a separate tolerance and window before
running the main study.

### 1.5 Metrics and outputs

At minimum, record for every sweep condition:

- HIGH-field NRMSE and normalized-density KL divergence for the proposed MFGP;
- empirical 95% posterior calibration for the proposed MFGP;
- raw covered probability mass and footprint-normalized coverage effectiveness;
- nearest-ground locational cost;
- cumulative aerial and ground travel;
- estimator-update timing, retained LOW/HIGH sample counts, and update failures;
- Zhang phase-transition time where applicable.

Baselines must receive `N/A`, not fabricated values, for posterior metrics they
do not natively produce. Summaries must use episode-level means or medians with
paired 95% confidence intervals. Plot the dependent metric against the actual
factor value; do not use a bar chart when episode-level distributions or
histories are available.

### 1.6 Implementation tasks

- [x] Freeze the reference scenario and numerical levels for all six factors.
- [x] Add a declarative sweep section to a new YAML configuration, including a
      deterministic smoke version.
- [x] Implement one Python sweep orchestrator using the existing coupled
      simulation, scenario archive, evaluators, and progress reporting.
- [x] Ensure the orchestrator runs sequentially; do not add multiprocessing or
      other concurrency.
- [x] Generalize scenario fingerprints and archive metadata to include every
      varied factor and the resolved full-range check.
- [x] Preserve paired seeds and compatible scenario arrays across levels and
      methods.
- [x] Add the proposed full-environment range condition.
- [ ] Add the long-duration Zhang timing pilot and freeze its acceptance rule.
- [x] Add offline evaluation and plotting from immutable raw archives.
- [x] Support `--reuse-raw` so figures and statistics can be regenerated
      without rerunning simulations.
- [x] Add deterministic unit, archive-contract, smoke-pipeline, and plotting
      tests before the production implementation.
- [ ] Run the complete repeated-episode study and inspect failed/outlier runs.
- [ ] Integrate the resulting protocol, figures, tables, and limitations into
      the paper and reviewer response.

## 2. Occlusion-aware sampling

### 2.1 Required behavior

For a sensor at position $p$ and candidate sample location $q$, accept the
sample only if all applicable conditions hold:

$$
\lVert q-p\rVert_2 \leq R,
\qquad
q \in \operatorname{FOV}(p),
\qquad
q \in Q_{\mathrm{free}},
\qquad
\operatorname{LOS}(p,q)=1.
$$

`LOS(p, q) = 1` means that the closed segment from $p$ to $q$ does not intersect
the closed square represented by any occupied raster cell. A full-environment
radius bypasses only the first
restriction; it must not bypass field-of-view or line-of-sight checks.

### 2.2 Geometry and policy decisions

Use one sensor-independent visibility utility so aerial and ground observation
paths cannot drift into inconsistent definitions. The utility should operate
on the simulation's actual obstacle representation and return a deterministic
Boolean mask for a batch of candidate points.

The implementation freezes these policies:

- the shared 2-D ground-obstacle raster occludes both aerial and ground
  sensing; obstacle heights and altitude-dependent overflight visibility are
  not modelled;
- touching an obstacle boundary counts as blocked with a documented numerical
  tolerance;
- a target endpoint may not lie in an occupied cell;
- generated circular and loaded obstacles use their common rasterized
  occupancy representation for LOS;
- when no candidate sample is visible, the sensor submits an empty observation
  batch and preserves normal estimator scheduling.

The visibility decision should be made before measurement noise is sampled so
blocked candidates do not consume the random stream. Archive attempted,
range/FOV-admissible, LOS-visible, and accepted sample counts for diagnostics.

### 2.3 Tests and validation

- [x] Add deterministic geometry tests: unobstructed ray, direct blockage,
      tangent/boundary contact, endpoint occupancy, multiple obstacles, and
      zero-length ray.
- [x] Add sensor tests proving that samples outside range are rejected.
- [x] Add sensor tests proving that in-range but occluded samples are rejected.
- [x] Add tests for partial visibility of one sensing footprint.
- [x] Add separate LOW/aerial and HIGH/ground sensor tests and verify that the
      coupled coordinator supplies the same obstacle raster to both.
- [x] Verify that an empty visible set does not crash or invalidate the most
      recent posterior.
- [x] Add a deterministic visual showing sensor position, nominal footprint,
      obstacles, candidate samples, accepted samples, and rejected occluded
      samples.
- [x] Run legacy-mode regression tests to ensure the compatibility path remains
      functional.
- [x] Update `docs/mathematical_formulation.md`, its equation-to-code map, the
      LaTeX mathematical reference, and the configuration reference in the
      same implementation milestone.
- [ ] Evaluate how occlusion changes sample counts, reconstruction metrics, and
      coverage relative to the same obstacle maps without occlusion.
- [ ] Integrate the sensor model and its limitations into the paper and reviewer
      response.

## 3. Future real-world demonstration

Status: **Future work; no hardware implementation required for the present
revision.**

The proposed demonstration uses two aerial robots and one quadruped:

- two drones provide spatially broad, lower-fidelity observations;
- one quadruped provides localized, higher-fidelity observations;
- one central computer owns the asynchronous autoregressive MFGP and publishes
  the HIGH-field posterior used by the coverage planners.

The future implementation should preserve the simulation architecture's
logical interfaces while replacing simulated state and sensors with recorded
or live measurements. The paper should describe this as a planned validation,
not as completed experimental evidence.

Items to specify before hardware work begins:

- test area, safety boundaries, geofencing, emergency stop, and operator roles;
- drone and quadruped platforms, payload limits, endurance, and sensing rates;
- definition and calibration of the physical LOW- and HIGH-fidelity signals;
- common world frame, localization system, map construction, and obstacle
  representation;
- timestamping, clock synchronization, observation latency, and dropped-data
  handling;
- central-computer hardware and the transport layer needed for real robots;
- controller command interfaces and platform-specific safety filters;
- takeoff, landing, loss-of-localization, and communication-loss procedures;
- a scripted two-drone/one-quadruped mission and deterministic log/replay mode;
- comparison against simulation using reconstruction, coverage, timing, and
  trajectory-tracking metrics.

The current repository constraint forbidding networking, middleware, and real
concurrency applies to this Python simulation implementation. Any future
hardware bridge should live outside the estimator mathematics and simulation
core, and its software/safety design should be reviewed as a separate project
milestone.

## 4. Recommended implementation order

1. Freeze sweep definitions, pairing rules, full-range semantics, and the
   long-duration acceptance criterion.
2. Implement and validate the general parameter-sweep archive pipeline.
3. Implement LOS geometry and unit tests.
4. Integrate occlusion into LOW/HIGH sampling and rerun regression tests.
5. Run smoke sweeps, then the frozen repeated-episode experiments.
6. Generate paper-ready figures and tables strictly from saved archives.
7. Update the paper, mathematical documentation, status log, and reviewer
   response with commands, test results, artifacts, and limitations.
8. Retain the two-drone/one-quadruped demonstration as clearly scoped future
   work unless a separate hardware milestone is approved.

## 5. Revision evidence log

Add one row after each completed milestone. Links should point to the exact
configuration, raw/evaluated archive schema, test selection, deterministic
visual, and paper change.

| Date | Milestone | Status | Files/artifacts | Tests and commands | Regressions or limitations |
| --- | --- | --- | --- | --- | --- |
| 2026-10-20 | Reviewer-revision tracker created | **Complete** | `docs/reviewer_revision_progress.md` | Documentation inspection and Markdown structure check | Planning only; no estimator, sensor, controller, metric, or experiment behavior changed. |
| 2026-10-20 | Obstacle-aware LOS sampling | **Implemented** | LOS geometry, both multi-fidelity sensors, tests, mathematical/configuration documentation, and `docs/assets/line_of_sight_sampling_milestone.png` | Final focused selection: 75 passed; full suite before the final focused empty-batch contract: 316 passed with four unchanged known failures | Obstacle scenarios intentionally collect different samples. The 2-D model treats ground obstacles as occluding both sensors; height-dependent aerial visibility remains unsupported. |
| 2026-10-20 | Proposed-method parameter sensitivity sweep | **Implemented** | Production/smoke YAML, resolver/orchestrator/evaluator/plotter, five deterministic tests, six smoke plots, and `docs/assets/multifidelity_parameter_sweep_milestone.png` | All 12 smoke conditions completed; 5 direct and 32 relevant tests passed; full suite 322 passed with four unchanged known failures | No baselines were run. Production repetitions and paper integration remain; the 60-s duration is only a proposed-method sensitivity level until Zhang's timing pilot validates it. |
| 2026-10-21 | Combined KL sensitivity summaries | **Implemented** | Six-panel absolute-KL overview, paired-reference effect plot, standalone archive-only plotter, mathematical documentation, and deterministic plotting test | Completed 21-condition archives reused without simulation; parameter-sweep tests 6/6 passed | Intervals describe episode variability. They do not establish causal interactions between factors in the one-factor-at-a-time design. |
| 2026-10-21 | Full-visibility Proposed--Rudolph--Zhang pipeline | **Implemented** | Dedicated resolver/runner, resolved-config archive, deterministic tests, smoke raw/evaluated archives, and `docs/assets/full_visibility_comparison_smoke/` | One-episode/four-step real smoke completed; focused validation recorded in the status log | Production repetitions remain. Full-domain ground coverage saturates, so locational cost, travel, and timing must carry the common comparison. |

