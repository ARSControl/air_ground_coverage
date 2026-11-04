# Deployment interface specification, draft v1

This is a design contract for the first SITL integration, not an implemented ROS API. Values below are initial test settings, not flight limits. Version the actual schema and wire definitions before implementation and revise this document if tests expose a mismatch.

## Coordinate and time conventions

The scenario map uses `uav_1`'s PX4 local NED frame as its reference: horizontal x is north and y is east, in metres, and heading is clockwise from north. For `uav_1`, the local-to-map transform is identity. For every other vehicle configure a separately verified rigid transform from local NED xy to map xy: $p_m=R_i p_i+t_i$. If local axes are aligned, $R_i=I$ and $t_i$ is the offset between local origins expressed in `uav_1`'s frame. Transform direction vectors with $R_i$, not translation; transform heading using its unit vector. The inverse maps velocity commands to local NED. Starting positions yield $t_i$ only if each estimator's local origin and axis alignment are known; a spawn pose alone does not establish them. Verify initialization, axis alignment and reset behavior against synchronized SITL truth and independently register the map on hardware. A PX4 local-origin reset invalidates the transform and must stop mission readiness until re-registration. Vertical altitude and takeoff are handled by MULLET; planar command z values are NaN.

For the initial implementation assume all local horizontal NED axes are aligned, so $R_i=I$, and estimate offsets at mission initialization rather than hard-coding spawn positions. Let $g_i$ be each drone's synchronized horizontal position in a shared, north/east-aligned reference and $p_i$ its simultaneous local NED xy. With `uav_1` as reference, compute $t_i=(g_i-g_1)-(p_i-p_1)$ and $t_1=0$. Local positions alone cannot reveal relative origins: $g_i$ must come from a common reference, such as synchronized Gazebo world positions converted to north/east in SITL, or a validated shared localization system on hardware. Heartbeat global WGS84 positions are a candidate only after their source, origin handling and accuracy are verified; do not assume GPS is sufficiently accurate for obstacle clearance. Collect a stationary time window, reject stale or invalid samples, measure spread and residuals, and fail readiness if predeclared accuracy thresholds are not met. Test alignment and heading conventions despite the initial assumption, and detect local-frame resets or loss of reference during a mission. Exact thresholds and the reference provider remain to be selected and validated.

Grid metadata explicitly defines the map coordinate of cell (0,0), metres per cell, row direction, shape and bounds. Grid columns follow map x (north), rows follow map y (east). Convert to grid coordinates before using existing raster utilities, which currently interpret x/y as cell indices; no implicit one-metre-cell assumption is allowed. Fix rounding, edge inclusion and occupancy boundary rules in tests before accepting a schema.

SITL uses a single verified simulation clock; physical flights use a synchronized time source whose offset and drift are measured. Every measurement carries acquisition time and clock-domain ID; logs also carry receipt time. Fail readiness on unverified registration or incompatible clocks. Durations and local watchdogs use a monotonic clock. Actual `/clock` propagation and PX4/MULLET timing must be tested, not assumed.

Takeoff and landing are handled outside this repository by MULLET's operational workflow. Initial mission readiness requires each drone already airborne, in HOLD, and verified at its assigned altitude within a tolerance to be established and tested. Planar mission commands hold the altitude latched when entering MISSION; they do not establish the intended altitude. An optional 3D altitude-transition phase is deferred pending separate design and validation.

## Scenario document

The scenario file is a versioned, validated immutable manifest. Resolve defaults and asset paths, then hash canonical resolved content and asset bytes. Record the resolver and schema versions alongside the hash. Containers verify the same hash before enabling a mission. Required logical sections:

| Section | Required contents |
| --- | --- |
| `map` | frame ID, axes, origin, resolution, dimensions, free-space and boundary rules, obstacle asset and digest |
| `field` | static HIGH grid or generator and seed, LOW generation parameters, mask and digests; initial implementation reuses `build_fidelity_fields` and records `rho` and smoothing sigma in cells |
| `vehicles` | unique ID and MULLET namespace, role, per-vehicle transform, target altitude, sensor geometry, sample count/period, noise variance and random seed |
| `estimator` | GP kernels, rho, jitter, retention budget, update period, density transform and query grid |
| `control` | controller and filter settings, desired command and streaming periods, plan expiry, clearance, startup and failure behavior |
| `time` | clock-domain ID, maximum observation/pose age and allowed future skew |

All numeric limits must be finite and valid; reject duplicate IDs, role/fidelity mismatches, malformed transforms, invalid footprints, missing assets, hashes that differ and incompatible field/map shapes. Ground role means HIGH with an obstacle-occluded cone; aerial role means LOW with an unoccluded circular footprint. Starting values should be selected in a frozen SITL scenario, not inferred from this specification.

Existing `build_fidelity_fields` constructs LOW by mask-normalized Gaussian smoothing of HIGH and defines discrepancy as HIGH minus rho times LOW. This gives a precise simulation relationship, not evidence that a physical sensor has those properties. Existing sensors draw area-uniform points in a sector and use nearest rounded grid cells; preserve these initial semantics after coordinate conversion. Bound retries and return an empty batch when no candidate is accepted. Noise is Gaussian with configured variance and reproducible independent per-vehicle streams; define seed derivation and request ordering in the implementation contract tests. No noise draw is consumed for a rejected candidate.

## Logical messages and ownership

The following describes data semantics, not final ROS types or topic names. The estimator and scenario service are laptop-owned; each vehicle owns one agent and one MULLET command stream.

| Message | Required fields and behavior |
| --- | --- |
| Scenario identity | schema/resolver version, scenario ID, content hash and map hash; every participant checks equality |
| Pose snapshot | vehicle ID, frame ID, acquisition time/domain, position and heading, validity and sequence; reject stale, invalid or unregistered pose |
| Sample request | scenario hash, vehicle ID, unique request ID, role, pose snapshot and sampling event time; scenario service validates identity and role |
| Observation batch | request ID, scenario hash, acquisition time/domain, robot ID, fidelity, ordered positions and scalar values, and noise variance; empty batch is valid. Map each scalar to existing `Observation(timestamp, robot_id, position, value, fidelity, noise_variance)` |
| Plan | vehicle ID, scenario hash, posterior version, sequence, creation time, expiry and desired map-frame planar velocity or target; define target-to-velocity conversion before implementation |
| Diagnostics | scenario identity, pose/plan ages, update version, failures, sample counts, filter feasibility and minimum virtual clearance; archive without exposing truth to the estimator |

Requests must be idempotent by `(vehicle_id, request_id)`: duplicate requests return the same result or an explicit duplicate response without drawing additional randomness. Reject conflicting duplicate payloads. Define bounded caches, restart/session identifiers and out-of-order behavior; never silently merge observations from different scenario hashes. QoS, transport, bandwidth and message size are implementation decisions that require SITL tests.

## Initial SITL gates

Freeze the exact test scenario and numerical tolerances before running these gates. Proposed initial targets for a controlled slow-motion SITL pilot, subject to feasibility measurement rather than retroactive tuning:

1. Test coordinate transforms analytically at origin and independent axis/heading directions and round-trip to $10^{-9}$ m numerical tolerance. Compare registered SITL poses with independent Gazebo truth at stationary known points: maximum horizontal error 0.25 m and heading error 5 degrees. Log actual errors and state the truth source.
2. Stream at 20 Hz initially and record worst inter-command gap; verify the configured MULLET timeout and demonstrate hold on intentionally stale plans. No GP update blocks the command stream. Verify behavior using observed supervisor state and telemetry, not only published messages.
3. In deterministic sampling tests, every HIGH point is in bounds, in range, inside the cone and visible; every LOW point is in bounds and in its footprint regardless of virtual obstacles. Repeated same-seed scenarios produce identical batches under the declared ordering policy. No sample is emitted when no valid candidate is found.
4. Define clearance before execution as obstacle boundary plus vehicle footprint and tracking margin; record minimum measured clearance throughout each maneuver. The filter must flag infeasibility and trigger a tested stop/hold policy. Include blocked paths and local minima; report stalls rather than counting them as successful navigation.
5. For the paired causal test, fix scenario, initial state and random streams; change only the HIGH observation intervention. Record nonzero changes above predeclared numerical tolerance in HIGH posterior and aerial target, and demonstrate an actual command or trajectory change. The current Python feedback tests are baseline evidence, not a SITL result.

These are development gates only; the real flight area, localization quality, timing, separation and failsafes require independent validation. No claim of hardware readiness follows from satisfying them.


## Implemented posterior refresh metadata and sample admission (2026-11-04)

The central MFGP target adds backward-compatible `posterior_timestamp`,
`model_updated`, and `sample_filter` diagnostics. In filtered static-field mode,
`timestamp` is the scheduled publication/update-start time; the posterior
version and `posterior_timestamp` change only after a fit. A heartbeat may
publish the same valid arrays/version when all new observations were rejected
or no new samples arrived. No refresh is published after an update failure.
Motion-command, pose and lease freshness semantics are unchanged.

`sample_filter` reports enabled criterion, shared `max_total_samples`, retained
LOW+HIGH total, cumulative accepted LOW/HIGH, redundant/duplicate/capacity
rejections, and the last confidence thresholds. Active LOW/HIGH thresholds are
identical because admission targets HIGH uncertainty with one shared budget.
Request-response `submitted` continues to mean queued for a scheduled update;
it does not promise later admission to the retained training set.


## Information replacement metadata (2026-11-04)

Active `on_full: information` permits one verified improving swap per scheduled
GP update. Status/target `sample_filter` adds cumulative `replaced`,
`evicted_low`, `evicted_high`, `replacement_evaluations` and last replacement
objective before/after/improvement. Accepted counters are historical and may
exceed current retained counts. In this shared mode, readiness permits a
fidelity to have zero currently retained samples if committed admissions prove
that fidelity contributed previously; the nonempty-model and freshness checks
remain required. See the mathematical reference for exact swap equations.


## Completion-stamped cached publication and failure snapshots (2026-11-04)

The central node now publishes its valid cache immediately before scheduled
compute and publishes the completed state afterward. `timestamp` is actual
emission time, `posterior_timestamp` remains last fit time, and added
`update_in_progress`, `update_started_at`, `update_completed_at` and
`update_duration_s` distinguish those cases. `status.update_timing` reports
attempt state. The next attempt waits a full configured period after completion.
No age gate is extended. After a failed attempt, further cache refreshes are
suppressed until successful recovery, so repeated failures still expire targets.

Mission summaries gain `mission.health_failure`, with a matching JSONL event
before HOLD. It records reasons and packet/command ages at the actual trigger,
independent of whether the final HOLD readiness report recovers afterward.
The new fields are additive; topic names and motion-command schemas are unchanged.
