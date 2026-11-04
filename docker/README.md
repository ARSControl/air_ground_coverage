For cloned onboard/laptop installations, use the Python deployment commands in
[DEPLOYMENT.md](DEPLOYMENT.md). They build a ROS-only hardware image and save
per-host role/scenario settings without changing the existing SITL defaults.

# Three-drone MFGP SITL deployment

The default Compose stack runs the central simulated field/MFGP, common-frame
registry and three drone agents, each with its own onboard planner.
Takeoff and landing are performed through MULLET. Command output is disabled
by default; the agents stream motion only when explicitly enabled and MULLET
is in MISSION.

## Prepare MULLET

Start your existing MULLET simulator with `/uav_1`, `/uav_2` and `/uav_3` on
ROS domain 42, then take off through MULLET and leave all three in HOLD:

| Vehicle | Role | Target local height | Sensing |
| --- | --- | --- | --- |
| `uav_1` | Ground / low altitude | 5 m | HIGH cone with virtual LOS |
| `uav_2` | Ground / low altitude | 5 m | HIGH cone with virtual LOS |
| `uav_3` | Aerial / high altitude | 10 m | LOW unoccluded footprint |

All services mount the same [scenario](scenarios/sitl_three_drone.yaml).
`uav_1`'s local NED frame defines map north/east. The map extends from -10 to
50 m on both axes, giving startup drift room. The altitude check allows 0.75 m
error. Set `ROS_DOMAIN_ID` to match another MULLET domain, or set
`MFGP_SCENARIO_FILE` to an absolute path to a different shared scenario.

Builds read the local MULLET interface source from
`../../mullet_sitl_sim/ws/src/mullet-interfaces` relative to this directory.
Set `MULLET_INTERFACES_PATH` to an alternative checkout if needed. A recent
Compose version with named build contexts is required; the tested version is
Compose 5.1.4. The built image is `mfgp-deployment:local`.

## Run and check while holding

From the repository root:

```bash
bash docker/run_sitl.sh up
```

The equivalent direct command is `docker compose -f docker/compose.yaml up --build`.
Keep that terminal open. In another terminal:

```bash
bash docker/run_sitl.sh check --timeout 60 --output /artifacts/preflight.json
```

The read-only check requires three consecutive ready snapshots. It verifies
all registered frames, fresh map poses, expected heights, both observation
fidelities, an updated posterior, and bounded fresh proposals for every drone.
It writes `output/sitl/preflight.json` and `preflight.events.jsonl`. Missing
vehicles or a wrong altitude appear in `reasons`; the process exits 1 when it
cannot become ready. `ready: true` means the SITL data pipeline is ready for
a supervised test, not that real-world flight is validated.

## Enable a supervised SITL mission

Stop the holding-mode deployment with Ctrl+C, then run:

```bash
bash docker/run_sitl.sh up --motion
```

Recheck readiness in another terminal. The drones remain in HOLD. Start a
fixed-duration recorded mission with:

```bash
bash docker/run_sitl.sh mission --duration 60 --output /artifacts/mission.json
```

This command replaces the manual `start_mission` calls for the default scenario.
It waits for all three agents to be ready, command-enabled and in HOLD; calls
MULLET's `start_mission` services; records the run; and requests HOLD for all
three when 60 seconds have elapsed. The duration is monotonic wall time measured
from the first start request, including startup-service latency. Physical
stopping follows MULLET and the vehicle controller; it is not instantaneous.
Takeoff and landing are never performed by this runner.

The runner checks the reported HOLD states before exiting. A rejected/timed-out
start, cancellation, or unhealthy running pipeline also triggers HOLD requests
for the whole team and returns a non-success result. The default scenario
requires a short-lived mission lease: while active the runner renews it every
0.1 s with a 0.5 s lifetime, and agents stop streaming if it expires. A runner
interruption attempts HOLD; abrupt process loss also stops lease renewal.
MULLET's own command-loss behavior then applies. For an initial trial, choose a
short duration such as `--duration 10`.

The agents publish planar MULLET VELOCITY commands at 20 Hz, capped at
1.0 m/s for aerial robots and 0.5 m/s for ground-role robots, holding the altitude latched by MULLET on entering MISSION. They reject
wrong identities, stale proposals, missing leases, old registration sessions,
stale telemetry, bad altitude, failsafe state, and motions projected outside
the configured map margin. Obstacle avoidance remains deferred.

The mission runner automatically saves `output/sitl/mission.json` and
`mission.events.jsonl`. Recording starts before the mission and includes stop
confirmation. The summary includes the requested and observed command-phase
durations, service results and HOLD confirmation. The JSONL retains common-frame
trajectories, accepted observations, HIGH posterior mean/variance, aerial targets,
proposals, leases and observed commands. Generate a figure afterward with:

```bash
.venv/bin/python -m docker.plot_readiness output/sitl/mission.json --output docs/assets/sitl/mission.png
```

The summary includes the scenario's ground-truth HIGH field in place of the
planner-proposal panel. For a custom scenario, pass `--scenario path/to/original.yaml`
to the plotting command. Its hash must match the recording; retain the original
YAML when changing scenarios. Truth is reconstructed offline and is never sent
to the estimator by this plotting path.

Replay a completed mission as an MP4 with progressively revealed measured
trajectories and the recorded HIGH posterior mean/variance updating in time:

```bash
.venv/bin/python -m docker.replay_sitl output/sitl/mission.json \
  --output output/sitl_replay/mission.mp4 --speed 4 --fps 10
```

Use a unique recording name for each mission and wait for the mission command
to finish before replaying. The tool rejects an older summary paired with a
new overwritten event log. The replay output uses a separate host-owned
directory because the container-created `output/sitl/` may not be writable
by the host user.

The three panels show the static ground-truth HIGH field, recorded posterior
mean, and variance, with trajectories overlaid. All field backgrounds use pale grayscale;
colored paths have a white outline and markers have white edges for contrast.
Aerial markers are orange
triangles and ground markers are green circles. `--speed 1` gives real-time
playback; the default is 4×. This is offline rendering using FFmpeg, not ROS
replay or a new simulation. No deployment restart is needed. The output also
includes a final-frame PNG and `.replay.json` with timing/frame counts.
Use `--scenario path/to/original.yaml` for a custom scenario; its hash must match.
Replay covers the entire recorder interval, including initial HOLD and stopping.
It selects only packets already received at each frame time, holds each
posterior until another arrives and leaves the estimate blank before the first
recorded posterior. It does not interpolate paths or rerun the GP. The displayed
clock is recorder elapsed time; irregular delivery and missing poses remain
visible as jumps/held positions, rather than reconstructed vehicle motion.

The separate read-only recorder remains available for manually controlled runs:

```bash
bash docker/run_sitl.sh check --in-mission --timeout 120 --record-seconds 60 --output /artifacts/manual_run.json
```

That `check` command only records and never ends a mission; use `mission
--duration` for automatic HOLD. For deliberate manual start/hold operation,
set `control.require_mission_lease: false` in the shared scenario and restart
the deployment. The timed runner requires it to be true.

## Stop

Use MULLET to HOLD each drone:

```bash
ros2 service call /uav_1/hold std_srvs/srv/Trigger '{}'
ros2 service call /uav_2/hold std_srvs/srv/Trigger '{}'
ros2 service call /uav_3/hold std_srvs/srv/Trigger '{}'
```

Then stop the deployment using Ctrl+C or `bash docker/run_sitl.sh stop`.
MULLET owns the command-loss response if the stream disappears: the inspected
configuration holds on a 0.25 s stale command and transitions to HOLD after
2 s of command loss. Verify these parameters if your MULLET configuration differs.
Stopping this Compose project does not stop the separate MULLET simulator.

## Topics and logs

| Topic | Publisher | Contents |
| --- | --- | --- |
| `/mfgp/frame_offsets` | Frame registry | Expiring common-frame offsets and session |
| `/mfgp/pose` | Each drone agent | One map pose per vehicle message |
| `/mfgp/agent_status` | Each drone agent | Registration, height, state and command diagnostics |
| `/mfgp/sample_request` | Each drone agent | Timestamped simulated-sensing request |
| `/mfgp/sample_response` | Central service | Accepted observations or rejection |
| `/mfgp/status` | Central service | Posterior version, retained counts and errors |
| `/mfgp/aerial_target` | Central service | HIGH posterior, density, variance and aerial target |
| `/mfgp/proposed_velocity` | Each onboard planner | Expiring per-vehicle proposals |
| `/<vehicle>/command` | Enabled drone agent | MULLET planar command stream |

The central runtime waits at least 1 s after each model attempt completes before
starting another attempt, independently of each agent's command timer. It
publishes cached state before expensive work and completion-stamped state after.
This avoids back-to-back overdue fits and keeps publication age separate from
GP computation time. Target and central-status freshness limits remain 2 s;
pose, command and lease checks are unchanged.
The ROS topics above use experimental JSON payloads in String messages except
MULLET heartbeat/command. The single-threaded central runtime retains the
previous valid posterior after model-update failure. Request caches are
bounded and process-local; recordings are diagnostic artifacts, not a complete
restart/persistence or deterministic ROS replay implementation.

## Role speed limits

The current scenario sets:

```yaml
control:
  max_speed_mps: 0.5  # fallback for roles without an override
  max_speed_by_role_mps:
    aerial: 1.0
    ground: 0.5
```

Each onboard planner, its agent command gate and the readiness checker use the
same role-specific limit. These are planar speed limits, not guarantees of
measured speed: ground centroid tracking may slow down, and MULLET/PX4 tracking
and acceleration limits still apply. The aerial limit is twice the ground
limit. Scenario files without the mapping retain the shared fallback behavior.
Rebuild the image and restart the complete deployment while drones are in HOLD,
then rerun readiness. No mission is started by this change.

The previous 1,000-sample equal-speed manifest is archived as
`docker/scenarios/sitl_three_drone_1000_samples_equal_speed.yaml`; use the
matching original manifest when plotting/replaying earlier recordings. The
250-sample archive remains available for older missions.

## Observation retention and the paper filter

The current deployment uses **one shared 1,000-sample training budget** for the
central MFGP, replacing 1,000 LOW plus 1,000 HIGH. Either fidelity can use any
part of the budget; LOW/HIGH counts remain visible, but there are no separate
active quotas. Speed limits remain aerial 1.0 m/s and ground 0.5 m/s.

```yaml
estimator:
  retention:
    max_total_samples: 1000
    policy: paper_epistemic
    criterion: high_information
    on_full: information
    replacement_min_improvement: 1.0e-8
    relative_error: 0.05
    confidence_z: 1.96
```

The implementation builds on Mantovani, Pratissoli and Sabattini (2024),
*Distributed Coverage Control for Spatial Processes Estimation With Noisy
Observations*, Algorithm 1 and Eqs. (18)–(19). The paper admits samples when
latent standard deviation exceeds a confidence threshold based on the estimated
field range. Our `high_information` extension compares each LOW or HIGH sample's
expected reduction of HIGH variance at its location against the same threshold,
using the MFGP cross-covariance and that sample's noise. LOW samples are therefore
not retained merely because independent HIGH discrepancy remains uncertain.
The explicit `native_epistemic` criterion is also available for the paper's
latent-standard-deviation rule applied to each sample's native fidelity.

Filtering runs only at the scheduled GP update. Within a batch, accepted samples
condition the uncertainty used for later candidates; the main joint GP is fitted
once at most. When the budget is already full, `on_full: information` considers
all informative incoming samples and every retained sample, then selects at most
one best swap for that update. Its objective is mean HIGH latent variance over
the existing query grid, weighted by integration weights (and any query mask).
A HIGH sample can replace a LOW sample, or vice versa; age and fidelity labels
do not override information value. The final fitted posterior must improve the
objective by more than `replacement_min_improvement` (and a numerical floor)
before any removal is committed.

Swap scoring uses Cholesky-based addition/removal identities, not a separate GP
refit for every possibility. The fit, removals, admissions and diagnostic counts
commit together. A failed update preserves the old posterior and observations.
If every swap is worse, the new samples are rejected. For compatibility,
`on_full: reject` keeps the previous behavior of preserving all retained samples.
The update that first fills the budget uses ordinary admission; replacement
starts on subsequent updates that begin full. This bounds replacement work.

Lower average variance does not imply lower variance at every location: a swap
can sacrifice uncertainty locally to improve it overall. Readiness still requires
both fidelities to have contributed, but in replacement mode it does not demand
permanent retained LOW/HIGH quotas. HOLD sensing uses the same budget and policy.
Independent noisy readings at the same location may be useful; identical records
and idempotent request retries are deduplicated.

`/mfgp/status` and target packets expose `sample_filter` counters for accepted
LOW/HIGH, redundant, duplicate and capacity-rejected samples, plus the retained
total and thresholds. Replacement diagnostics include `replaced`, `evicted_low`,
`evicted_high`, `replacement_evaluations`, and the last verified before/after
weighted-variance objectives and improvement. All-rejected updates skip GP fitting. The central service
continues publishing the valid static posterior at the configured update period:
`timestamp` is publication time, `posterior_timestamp` records the last fit,
and `model_updated` distinguishes a new fit from a refresh. Refreshes keep the
same version and arrays. The central publisher sends cached valid state before
compute and completion-stamped state afterward. `update_in_progress`,
`update_started_at`, `update_completed_at` and `update_duration_s` expose that
lifecycle. After a known update failure, further cached refreshes are suppressed
until a successful result; a heartbeat sent before the failure was known may
already have been received.
Pose, lease and motion-command freshness limits are unchanged.

Rebuild the image and restart the whole deployment while drones are in HOLD,
then rerun readiness. Running nodes do not hot-reconfigure. For old plots and
replays, select the archived original scenario with `--scenario`:

- `sitl_three_drone_1000_samples_high_information_no_replacement.yaml`: shared-1,000 filtered runs before replacement.
- `sitl_three_drone_500_samples_high_information.yaml`: previous shared-500 filtered runs.
- `sitl_three_drone_250_samples.yaml`: original equal-speed, 250-per-fidelity runs.
- `sitl_three_drone_1000_samples_equal_speed.yaml`: 1,000 per fidelity, equal speeds.
- `sitl_three_drone_1000_samples_role_speeds.yaml`: 1,000 per fidelity, faster aerial.

These files live in `docker/scenarios/`. The illustrative `example_sitl.yaml`
and historical scenarios retain the old newest-first policy unless they opt
in. The original Python simulation paths and `legacy` mode are unchanged.

## Central computation and early-stop diagnostics

The image installs pinned NumPy 1.26.4/SciPy 1.13.1 binary wheels with OpenBLAS,
using one numerical-library thread. This replaces the base image's generic BLAS
without changing NumPy's major ABI. No background GP worker or controller
threads are added. Replacement scoring caches the retained-data/grid geometry
while the fitted posterior is unchanged; a successful fit invalidates it.

`/mfgp/status.update_timing` reports the last start/completion, duration,
in-progress state and failure flag. A blocking operation longer than the
unchanged freshness limit can still stop a mission; cached publication is not
an unlimited validity override.

If a mission stops with `pipeline_unhealthy`, the summary now includes
`mission.health_failure`, and the JSONL includes `mission_health_failure` before
HOLD. These capture the failing checks and ages of the target, central status,
poses, proposals, agent commands and observed command messages. Final HOLD
readiness can recover afterward; it does not change the recorded mission outcome.

The isolated timing and service regression commands are:

```bash
docker run --rm --network none -e ROS_DOMAIN_ID=101 mfgp-deployment:local docker.central_timing_probe
docker run --rm --network none -e ROS_DOMAIN_ID=102 mfgp-deployment:local docker.timed_mission_probe
```

The timing probe benchmarks a full 1,000-sample replacement and injects a 1.45 s
blocking update in an isolated ROS graph. It does not command simulator vehicles.
Rebuild/restart in HOLD and rerun readiness before a moving test.

## Onboard planning and communication

Each agent runs its own planner every 0.5 s, even with commands disabled.
The aerial agent owns its HEDAC state. Each ground agent computes its own
Voronoi velocity using all ground-role positions, in scenario vehicle order.
All ground teammates must have fresh poses from the same registration session;
missing or invalid inputs clear the local proposal. No range filter is applied.
The present deployment supports exactly one aerial vehicle; multi-aerial
coverage-history coordination is not implemented and is rejected at startup.

Agents publish positions on `/mfgp/team_pose/ground` or
`/mfgp/team_pose/aerial` and subscribe only to their own role topic. They still
publish `/mfgp/pose` for central services and recording. This is application
routing, not network access control. Central observation requests, the shared
MFGP and target publication are unchanged. Agents publish their own velocity
proposals for readiness/recording, but command execution uses the locally
computed proposal and does not subscribe to external velocity proposals.

On hardware, run `compose.drone.yaml` on each onboard computer. The laptop runs
only `central` and `frame-registry` plus the mission/recording tools. No shared
planner containers are needed. Multi-host ROS communication remains unvalidated.
After upgrading, stop/remove the old `aerial-planner` and `ground-planner`
containers from the previous layout; the new Compose manifest does not own
those services. Rebuild the deployment image before testing.

## Other deployment layouts and checks

For a single drone agent using an already built image and the same scenario:

```bash
VEHICLE_ID=uav_1 docker compose -f docker/compose.drone.yaml up
```

On the laptop, start only the shared services with:

```bash
docker compose -f docker/compose.yaml up --build central frame-registry
```

Multi-host DDS discovery, clock synchronization, localization accuracy, CPU
architecture and physical tracking still require hardware validation. The
current image was built and tested on amd64. Ground circular-obstacle avoidance is available through the optional command-rate
CBF described below. Inter-drone collision avoidance remains unimplemented. The frame registry assumes aligned NED axes and a 2D WGS84 surface
projection, and detects only selected frame inconsistencies. A detected
reference-frame fault latches registration off; restart the full deployment
in HOLD after resolving it. This is a SITL prototype, not a validated onboard
flight system. The central status retains `flight_ready: false` for this reason.

The independent offline estimator smoke check is:

```bash
docker compose -f docker/compose.yaml --profile smoke up --build deployment-smoke
```

The command bridge can be tested without network access:

```bash
docker run --rm --network none -e ROS_DOMAIN_ID=101 mfgp-deployment:local docker.command_bridge_probe
```

The fixed-duration runner has an isolated ROS service test:

```bash
docker run --rm --network none -e ROS_DOMAIN_ID=101 mfgp-deployment:local docker.timed_mission_probe
```


### Two circular obstacles with a ground-role CBF

Use the same Gaussian-mixture field, sensing settings, role speeds and 1000-sample
high-information retention configuration as
`sitl_three_drone_1000_samples_high_information_no_replacement.yaml`:

```bash
export MFGP_SCENARIO_FILE="$PWD/docker/scenarios/sitl_three_drone_two_obstacles.yaml"
bash docker/run_sitl.sh up --motion
```

Use the existing registration, readiness and timed mission workflow after startup;
the new scenario has a different hash and needs fresh registration. The obstacles
are virtual disks at north/east `(25,25)` and `(32,30)` metres, both radius 3 m.
The field values are unchanged. HIGH sensing uses obstacle line of sight; LOW
sensing remains unoccluded. Aerial-role flight remains above these obstacles.

`control.obstacle_cbf` enables the ground command filter, with `alpha_per_s: 1`
and `clearance_m: 1`. Clearance inflates each radius to 4 m and represents the
combined vehicle footprint/tracking allowance. At each command tick, fresh map
position is used to project the nominal planar velocity onto the CBF constraints,
speed limit and existing geofence horizon constraints. Existing mission, lease,
registration and freshness gates still apply. Unsafe/interior or infeasible
filter states command zero planar velocity and log a throttled warning; agent
status exposes `obstacle_cbf_status`. Configurations omitting the block preserve
the old command behavior.

This is a velocity-tracking CBF, not an acceleration-aware PX4 safety guarantee.
CBFs can stall, including when an attractive target lies inside an obstacle;
there is no global path planner or inter-vehicle avoidance. Validate tracking
and clearance in SITL before hardware use. A deterministic ideal-tracking
rollout and its numerical metrics are in `docs/assets/sitl_obstacle_cbf/`.


`bash docker/run_sitl.sh check` and `mission` now reuse the running central
container's mounted scenario when `MFGP_SCENARIO_FILE` is unset. An explicit
variable still overrides that selection. This avoids checking the default
scenario accidentally from a second terminal. A startup planner warning about
missing team poses means it is waiting for the first registered poses, rather
than reporting a scenario hash disagreement.

SITL summary plots and MP4 replays draw the original scenario's physical
obstacles as slate disks with black edges, and enabled ground CBF clearance as
red dashed circles. The overlays appear on every map panel (including the
aerial target) to show shared environment geometry; they do not imply aerial
collision constraints. Paths and vehicle markers remain above the overlays.
Pass the original `--scenario` YAML when rendering the two-obstacle recording.
Previously recorded runs can be rendered again without restarting simulation.
