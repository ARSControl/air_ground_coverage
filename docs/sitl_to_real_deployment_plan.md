# SITL-to-real deployment plan

Status: **SITL integration implemented; supervised motion validation pending**. The shared field/MFGP, frame registry, three agents, planners, optional MULLET command bridge and readiness recorder have been built. Three-drone live readiness and isolated ROS command tests pass; no moving-drone or real-world validation has been claimed. Update the checkboxes and evidence log as milestones are completed; record failures and limitations as well as successes.

## Objective and experiment

Run the existing central autoregressive multi-fidelity GP and coverage approach with MULLET, first in Gazebo/PX4 SITL and then in a supervised real-world experiment. The initial team has two low-altitude drones in the existing **ground role** and one high-altitude drone in the **aerial role**:

- Ground-role drones collect HIGH-fidelity abstract samples in heading-dependent 2D cones. Range and virtual-obstacle line of sight restrict sampling. They also avoid the virtual obstacles.
- The aerial-role drone collects LOW-fidelity abstract samples in a downward footprint without obstacle occlusion. It flies above the modeled virtual obstacles and does not avoid them.
- A common hidden field and virtual obstacle map remain simulated in both SITL and real flights. Obstacles need not exist as physical objects in Gazebo. They affect ground-role sensing and navigation.
- Takeoff and landing are outside this repository and handled through MULLET's own operational workflow. The initial integration assumes each drone is already airborne, in HOLD, and at its assigned altitude before the coverage mission starts. An optional altitude-transition phase after a common-altitude takeoff is future work and requires a separate design and validation; do not assume planar commands change altitude.
- One central estimator produces the HIGH-field posterior. Demonstrate that a ground-role observation changes that posterior, the aerial target density, and the aerial command or trajectory.

Virtual obstacles are experimental constraints, not a substitute for detecting or avoiding real physical hazards. Inter-drone separation and physical flight safety require separate consideration.

## Architecture and repository boundary

Place the deployment integration in a top-level `docker/` directory in this repository. The user's requested ROS, container, and communications integration is scoped to this deployment layer; keep GP mathematics and the existing Python simulation core independent of ROS and retain the Python-only, sequential simulation and legacy estimator mode. Do not add middleware or concurrency to the existing simulation core.

- Build one configurable drone-agent image. Each instance has a vehicle ID, MULLET namespace, role and scenario configuration. In SITL, run three instances; on hardware, run one on each onboard computer.
- Run a central MFGP and planning service on the laptop. There must be only one MFGP, with expensive updates separated from command streaming. Preserve the last valid posterior after failed updates.
- Run a scenario service on the laptop in both SITL and real experiments. It owns the common hidden field and virtual obstacle map and returns simulated observations. The estimator receives observations rather than hidden truth. Make virtual obstacle geometry available to ground-role navigation.
- Use a companion Compose configuration for integration with the existing `~/mullet_sitl_sim` deployment, and a single-drone Compose configuration for onboard use. Pin compatible image and interface versions. Verify actual onboard CPU architecture and network requirements before building hardware images.
- Use MULLET's per-vehicle mission interface and `MissionCommand`, not direct PX4 control. Decide whether to implement the complete MULLET mission-framework discovery, loading and status interfaces after inspecting the operator workflow; document the choice.

## Shared scenario contract

Define one versioned scenario configuration and referenced assets, with schema validation and a resolved copy saved for every run. Include:

- Common map frame based on `uav_1` local NED, units, bounds, grid resolution, origin, and explicit transforms to Gazebo and each other vehicle's local PX4 NED frame. Verify origins, axis alignment, heading conventions and reset behavior rather than assuming they coincide.
- Assume aligned NED axes initially and estimate other vehicles' origin offsets at mission initialization from synchronized local poses and a validated shared position reference. Local coordinates alone cannot identify the offsets. Validate reference accuracy, initialization residuals, heading alignment and local-origin reset handling in SITL and on hardware.
- Hidden LOW and HIGH fields, their relation, generation parameters or immutable assets, seeds, and measurement noise. Define whether fields are static or time-varying.
- Virtual obstacle geometry, rasterization and clearance margins; use consistent geometry for sampling and navigation. Define outside-map and boundary behavior.
- Vehicle IDs and namespaces, role, flight altitude, cone angle and range or footprint, sample rate, and noise. No camera transforms or image processing are required.
- Estimator kernels, `rho`, jitter, retention and update schedule; planning and command rates; command freshness, time source, and timeout behavior.

Record scenario version, resolved settings, asset hashes, software versions and seeds. Validate all participants use the same scenario before enabling a mission. Choose and document reproducible per-vehicle noise streams and sampling behavior.

## Milestones

### 0. Contract and baseline

Initial source inspection and proposed contracts are recorded in `docs/architecture/deployment_contract_baseline.md`. Frame registration, concrete wire/schema details and measured thresholds remain open; milestone 0 is therefore **in progress**.
The draft logical schema, frame conventions and initial SITL gates are in `docs/architecture/deployment_interface_spec.md`; they require validation against the live simulator before being frozen.
An illustrative, deliberately incomplete manifest is in `docker/scenarios/example_sitl.yaml`; the standalone sensing builder consumes it, but it must not be used for flight.

- [ ] Inspect MULLET mission, heartbeat, pose and command APIs and QoS in the installed versions; establish how local positions and headings map into the shared frame.
- [ ] Freeze scenario schema, message contracts, responsibility for planning, obstacle semantics, time and startup/stop behavior.
- [x] Record current simulation tests, known failures and existing closed-loop evidence. Preserve reviewer-revision work and existing functionality.
- [ ] Document the exact implementation design and acceptance criteria before integrating ROS.

### 1. Ground-role obstacle policy in Python (deployment integration deferred at user request)

- [ ] Add deterministic tests first for a candidate CBF safety filter using the actual ground-role dynamics and occupancy geometry. Define obstacle clearance, map boundaries, infeasible optimization and failure behavior.
- [ ] Compare the candidate with the current bounded repulsion and hard segment guard. Test representative obstacles, narrow passages, starting near boundaries, local minima and multiple robots. Measure minimum clearance, stalled time, travel and coverage.
- [ ] Choose and document the policy using evidence. The existing unicycle step guard is not a guarantee for a tracked drone; define how a command-level filter will account for dynamics, tracking error, latency and margins in SITL. No physical-flight safety guarantee is implied.
- [ ] Update mathematical formulation and equation-to-code map for any changed equations; run relevant tests and produce a deterministic visual from real outputs.

### 2. Container and MULLET integration

- [x] Add drone-agent and laptop service images, pinned dependencies, configuration validation and Compose profiles under `docker/`. Do not change MULLET's existing simulator deployment unnecessarily.
- [ ] Connect three namespaced agent instances to MULLET SITL. Verify heartbeat, pose validity, coordinate transforms, mission lifecycle, fixed-altitude command mode and command freshness. MULLET commands use local NED; planar commands hold altitude and require NaN z fields. Takeoff and landing remain external to this repository.
- [ ] Stream commands independently of GP updates; verify startup, stale-command and lost-connection behavior against actual MULLET configuration. Establish a deterministic log and replay format.
- [ ] Add contract tests and a supervised SITL smoke test with logged evidence and a visual summary.

### 3. Virtual scenario and sensing

- [x] Implement and test the versioned scenario service, shared hidden field and virtual obstacle map. Validate scenario identity at every participant.
- [x] Test HIGH cone sampling, heading, range, obstacle line of sight, boundaries and empty observations; test LOW footprint sampling with no occlusion. Ensure both fidelities observe the specified related fields.
- [ ] Give the ground-role navigation policy the same obstacle geometry. Show virtual-obstacle avoidance in SITL without placing physical obstacles in Gazebo, including clearance and stalled cases.
- [ ] Archive scenario, observations, poses, commands and visualization from deterministic runs.

### 4. Closed-loop estimator and coverage

- [x] Adapt the existing central MFGP and controllers to the service interfaces. Keep one estimator, asynchronous observation handling, scheduled updates, configurable jitter and last-valid-posterior behavior.
- [ ] Adapt the ground controller to low-altitude drone commands and evaluate its tracking and avoidance behavior; retain the aerial controller's high-altitude role.
- [ ] Add deterministic integration tests and a paired SITL experiment isolating the effect of a HIGH observation: posterior change → aerial target-density change → aerial command or trajectory change.
- [ ] Log and assess estimator timings, sample counts, dropped/stale observations, update failures, trajectory tracking, obstacle clearance and reconstruction/coverage metrics. Report regressions and limitations.

### 5. Real-world preparation and staged flights

- [ ] Validate onboard architecture, MULLET versions, network discovery and transport, clock synchronization, frame registration and latency under the actual deployment conditions.
- [ ] Survey the flight area and register the virtual map and field to the shared localization frame. Define geofence, operator roles, takeoff/landing, separation, emergency stop and loss-of-link/localization/estimator procedures using the platform's safety mechanisms.
- [ ] Bench-test and replay recorded SITL data on the target computers; verify image configuration and logs. Test loss and recovery of services before flight.
- [ ] Conduct supervised staged trials: localization and passive logging; simulated sensing during controlled flights; command tracking and virtual-obstacle behavior; then closed-loop multi-drone coverage when preceding evidence supports it.
- [ ] Compare flight logs with SITL using declared metrics and document discrepancies. Treat this as a simulated-sensing flight demonstration, not physical sensor validation.

## Validation and tracking

At each implementation milestone, write deterministic tests before integrating new components, run relevant existing and new tests, and generate a deterministic visual from real implementation outputs. Update `docs/multifidelity_gp_status.md` with changed files, commands, test results, numerical or behavioral regressions, and limitations. Update `docs/mathematical_formulation.md` and its equation-to-code map in the same milestone whenever implemented mathematics changes. Do not silently continue after a failed test. Maintain reviewer-facing status separately in `docs/reviewer_revision_progress.md` when claims or evidence change.

Store SITL/deployment visual outputs in `docs/assets/sitl/` and link them from their milestone records.

| Date | Milestone | Status | Evidence and limitations |
| --- | --- | --- | --- |
| 2026-11-03 | Deployment plan | Planning | Design only; no implementation or validation performed. |
| 2026-11-03 | Contract and baseline inspection | In progress | Source-level MULLET contracts and proposed architecture documented; focused feedback tests 8 passed. No live SITL validation. |
| 2026-11-03 | Draft interface specification | In progress | Coordinate, observation, scenario and initial SITL contracts drafted; 60 existing sensor/observation/feedback tests passed. No wire schema or live simulator implemented. |
| 2026-11-03 | Example scenario | In progress | Draft YAML parsed and illustrative field generated with the existing field builder; transforms, clock and safety settings remain unset. Live MULLET inspection awaits a running simulator. |
| 2026-11-03 | Example field contrast | In progress | Increased illustrative LOW smoothing to four cells and used a common color scale; not a calibrated sensor model. |
| 2026-11-03 | Live MULLET inspection | In progress | Two-vehicle read-only discovery, heartbeat and configured timeouts observed; map registration and command behavior remain unverified. |
| 2026-11-03 | Live clock and world-pose inspection | In progress | Gazebo clock and world pose are available; ROS clock absent in discovery and both cores use wall time. Frame registration still requires synchronized data. |
| 2026-11-03 | Common frame decision | In progress | Use `uav_1` local NED as map frame; other origins and axis alignment need validation and reset handling. SITL images consolidated under `docs/assets/sitl/`. |
| 2026-11-03 | Aligned frame registration helper | Implemented, integration pending | Pure Python snapshot translation and deterministic tests; no ROS, timing, reference-quality or reset handling. |
| 2026-11-03 | Registration readiness helper | Implemented, integration pending | Stationary windows, inter-vehicle timestamp skew, age and reset counters checked; source synchronization, accuracy and live adapters remain open. |
| 2026-11-03 | Unicycle CBF prototype | In progress | Standalone circular-obstacle speed projection and deterministic tests; not integrated and no discrete-time or flight safety guarantee. Comparison with current policy remains. |
| 2026-11-03 | Discrete CBF guard prototype | In progress | Demonstrated discrete-step CBF crossing and added a separate simulated segment guard. Controller comparison, map bounds and drone tracking remain open. |
| 2026-11-03 | Symmetric obstacle-policy pilot | In progress | Both policies stall in one symmetric target-behind-obstacle scenario; more scenarios and actual coverage evaluation are needed before selecting a policy. |
| 2026-11-03 | Deployment field-service core | In progress | Transport-independent HIGH/LOW sampling and idempotent in-memory requests implemented and tested; scenario loader, ROS transport and estimator integration remain open. Obstacle-policy comparison deferred at user request. |
| 2026-11-03 | Deployment sensing scenario builder | In progress | Static Gaussian field and circle occupancy built from example manifest; ROS, resolved asset hashes and flight-readiness validation remain open. |
| 2026-11-03 | Deployment estimator adapter | In progress | One central estimator receives idempotent role-checked batches and updates on an explicit period; ROS, obstacle masking, planner and command integration remain open. |
| 2026-11-03 | Deployment aerial target | In progress | Existing interest/uncertainty target now derives from the deployment estimator; command and trajectory feedback remain unimplemented. |
| 2026-11-03 | Compose smoke image | In progress | One-shot offline sensing/estimator smoke builds and runs on the MULLET base. No ROS transport, agent, or drone control is included. |
| 2026-11-03 | ROS central sensing prototype | In progress | Separate central container receives synthetic ROS request and publishes a target; no drone agent, frame registration, planner or command output. |
| 2026-11-03 | ROS sensing agent prototype | In progress | `uav_1` heartbeat produces sensing requests and target updates; unregistered `uav_2` remains idle. Automatic frame registration and control remain open. |
| 2026-11-03 | Shared geodetic projection helper | In progress | WGS84 tangent north/east projection implemented and tested; altitude datum, timestamps, accuracy and integration remain open. |
| 2026-11-03 | Offline aerial HEDAC adapter | In progress | Existing heat and gradient routines generate a velocity changed by a HIGH observation; no ROS command integration or flight validation. |
| 2026-11-03 | ROS aerial velocity proposals | In progress | Synthetic ROS target and pose yielded a bounded velocity proposal; no MULLET command output or live aerial agent registration. |
| 2026-11-03 | ROS ground-role velocity proposals | In progress | Weighted Voronoi proposals for both ground roles tested with synthetic ROS poses; no obstacle avoidance or MULLET command output. |
| 2026-11-03 | Automatic SITL frame offsets | In progress | Stationary paired heartbeat registration produced both live map poses; localization quality, true timestamp alignment and reset detection are not flight-validated. |
| 2026-11-03 | Optional MULLET command bridge | In progress | Acceptance gate and isolated ROS command delivery tested; live tracking, scenario alignment and mission operation remain open. |
| 2026-11-03 | Three-drone SITL launch and readiness | Ready for supervised SITL motion test | Shared 5/5/10 m scenario, startup-safe map bounds, full default Compose, single-agent Compose, launch/check scripts, session and namespace guards, diagnostics and recorder implemented. Live three-drone HOLD readiness passed with both fidelities and all proposals; isolated bridge passed at 20.01 Hz. No live motion or real-world validation claimed. |
| 2026-11-03 | Fixed-duration mission runner | Implemented; isolated ROS validation | Timed start/record/HOLD orchestration and expiring command leases; real moving SITL validation remains pending. |

Current launch and test instructions are maintained in `docker/README.md`.
The default Compose stack starts the full pipeline with commands disabled;
`bash docker/run_sitl.sh up --motion` enables the bridge while MULLET still
requires an external MISSION transition. The single-agent Compose file is
`docker/compose.drone.yaml`; physical multi-host deployment remains unvalidated.


### Onboard planner layout (2026-11-04)

Supersedes the earlier shared planner-container layout: each drone agent now
owns its planner. The laptop retains central field/MFGP, frame registration
and mission tools. Ground-role agents exchange all ground-role positions;
the single aerial agent uses its own position. Role-specific pose topics have
no communication radius. Central sensing and posterior communication remain
unchanged. Multiple aerial vehicles are explicitly unsupported by this deployment
adapter until coverage-history coordination is implemented. Rebuild and repeat
live HOLD readiness before a motion trial; previous live readiness results
predate this layout.


### Shared-budget paper-based sample admission (2026-11-04)

The temporary 1,000-per-fidelity buffers are replaced with one shared 500-sample
budget. LOW and HIGH compete using expected reduction of HIGH variance at the
sample location, with the confidence threshold from Mantovani et al. (2024),
Algorithm 1/Eqs. (18)–(19). This is an explicit multi-fidelity extension of the
paper, with no fixed fidelity quotas. Previously admitted samples are protected;
capacity rejection is reported when the budget is full. Native-paper admission
and historical newest-first configurations remain available.

Deterministic/recorded-input checks and isolated ROS filter/feedback/refresh
checks pass; no new live mission is claimed. Restart the entire deployment in
HOLD with the rebuilt image and repeat readiness. The preceding faster-aerial,
1,000-per-fidelity manifest is archived as
`docker/scenarios/sitl_three_drone_1000_samples_role_speeds.yaml` for old logs.
See the mathematical reference and implementation status for exact equations,
capacity limitations and evidence.


### Information replacement at capacity (2026-11-04)

The active 1,000-sample shared configuration now uses `on_full: information`.
At most one best swap is permitted per scheduled update, provided the actual
new GP lowers integration-weighted HIGH variance. Old snapshots preserve
`on_full: reject` and old hashes via
`docker/scenarios/sitl_three_drone_1000_samples_high_information_no_replacement.yaml`.
Analytic scores, bounded compute, failure rollback and HIGH-to-aerial behavior
are covered by deterministic tests and the isolated ROS probe. No live moving
replacement experiment is claimed; restart in HOLD and repeat readiness.


### Central timing fix following a 129.95 s abort (2026-11-04)

Generic BLAS in the image and completion-only/start-stamped target publication
caused the observed early stop at shared-budget saturation. The rebuilt image
uses single-thread OpenBLAS wheels; the central object publishes valid cached
state before compute, stamps completed results at completion, and waits a full
update period after completion. Replacement geometry is cached without changing
selection mathematics. Target, pose, command and lease age limits are unchanged.

A controlled 240 s clock test passes all 14,403 real planner/gate checks, and
isolated ROS and full-budget image benchmarks pass. The runner now saves exact
health failures before HOLD. A new moving 240 s SITL mission is still required;
no flight/deployment restart was performed during these checks.

### 2026-11-04: Per-host deployment tooling

The clone/build/start workflow is now implemented in `docker/deploy.py`; see
[`docker/DEPLOYMENT.md`](../docker/DEPLOYMENT.md). Each host saves its role,
vehicle identity, shared scenario and ROS domain locally. A ROS-only hardware
image builds the exact local MULLET interface package. AMD64 image build,
both role import/manifest validations, offline estimator smoke, and 44 focused
regression tests passed. These checks start no flight nodes or missions.
ARM64 builds and multi-host hardware communication/flight remain pending;
this does not close the real-world validation gates above.
