# Deploy beside onboard MULLET

Run one MFGP agent on each drone, beside its existing MULLET container. Run
one central service and one frame registry on the laptop. The Python host
script manages only this repository's containers. It does not install Docker,
change MULLET, take off, land, or start missions during `up`.

## Prerequisites on every machine

- Linux with Docker Engine, Docker Compose v2+ and Buildx. Confirm
  `docker info`, `docker compose version`, and `docker buildx version` work
  for your account. Root access to flight devices is not required by our agent.
- Python 3.10+ on the host. The script uses only the standard library; no host
  ROS installation, pip install, or virtual environment is needed.
- A checkout of this repository at the same revision on all machines.
- Internet access for the initial image build. Docker fetches
  `https://github.com/skymule-ai/mullet-interfaces.git` at the full commit in
  `docker/mullet_interfaces.lock.json`, confirmed to match the deployed MULLET
  version. No separate interface checkout is required. A local `--interfaces`
  override remains available for development or offline source use.
- Compatible ROS Jazzy MULLET stack. The hardware image uses
  `ghcr.io/skymule-ai/mullet-base:v0.5.1`, as the hardware base. Docker builds natively on
  each machine; AMD64 and ARM64 use their respective base image variants.
  The ARM64 build and hardware execution still need testing on your computers.
- Matching ROS domain across the team, reachable laptop/drone network, working
  DDS discovery and synchronized clocks. This setup uses Linux host networking
  and Fast DDS UDPv4. Verify the existing MULLET container's networking and DDS
  configuration; host networking alone does not fix multicast/firewall issues.
  Keep another SITL experiment in a different domain or stopped.

## On each drone

From your cloned repository root:

```bash
python3 docker/deploy.py init --role drone --vehicle uav_1 --domain 42
python3 docker/deploy.py build
python3 docker/deploy.py smoke
python3 docker/deploy.py up
python3 docker/deploy.py status
python3 docker/deploy.py logs
```

Use `uav_2` and `uav_3` on the other drones. IDs and namespaces must agree with
MULLET and the scenario. The default scenario is
`docker/scenarios/sitl_three_drone_two_obstacles.yaml`. Use `init --scenario
/path/to/scenario.yaml` for another scenario. Deploy the identical original YAML
on every host. This example retains SITL altitudes/map settings: survey and
configure the real experiment before authorizing physical motion.

Settings are saved in git-ignored `docker/deployment.local.json`. The script
uses absolute paths, so subsequent commands work from any directory when called
by full script path. `--config /path/to/host.json` before the action selects
another settings file. Reinitializing requires `init --force` with the full new
settings. The saved raw YAML SHA-256 detects later edits before build/start or
mission tools; it is distinct from the canonical scenario hash printed by
`validate`, which should match across all hosts.

`build` uses Docker Buildx `--load`, fetching and compiling the pinned interface
Git context and installing pinned numerical Python dependencies. `init` saves
that full Git SHA in each host config so later lock-file changes do not silently
upgrade existing hosts. New configurations use the current lock. Older configs
with no Git pin fall back to the checked-in lock; local overrides retain priority.
To upgrade the interfaces, first validate the onboard MULLET version, then
update the checked-in lock and intentionally reinitialize/rebuild all hosts. It builds the same image
for agent and laptop hosts. `smoke` runs simulated LOW/HIGH estimation without
network access. `validate` checks imports, IDs, manifest and control settings
without starting ROS nodes; `up` runs this validation before creating services.
Failures stop execution and return the original failing command's exit status.

## On the laptop

```bash
python3 docker/deploy.py init --role laptop --domain 42
python3 docker/deploy.py build
python3 docker/deploy.py smoke
python3 docker/deploy.py up
python3 docker/deploy.py logs
```

Laptop `up` starts only `central` and `frame-registry`, never agent replicas.
The central process also owns the shared simulated field and sole GP. Run the
following after MULLET's external takeoff/altitude/HOLD workflow and all
onboard agents are running:

```bash
python3 docker/deploy.py check --timeout 60 \
  --output /artifacts/preflight_hardware_01.json
```

Reports and JSONL telemetry appear in host `output/sitl/` (the shared historical
artifact location; the directory name does not imply the source was SITL).
Use unique report names and retain the original scenario YAML for offline plots.

## Enabling commands and recording a mission

Default drone `up` explicitly sets `ENABLE_COMMANDS=0`, regardless of inherited
shell variables. Once the staged hardware checks support commanding, enable it
explicitly on **each drone**, with the vehicles in HOLD:

```bash
python3 docker/deploy.py up --motion
```

This recreates the agent as needed; registration and fresh readiness must be
rechecked. It does not itself start a mission. `--motion` is rejected on the
laptop. Command enablement is not persisted: a later ordinary `up` disables it.
Logs, status, check and mission do not alter already-running agents' enablement.

After readiness and operator authorization, use the laptop's existing recorded
mission workflow:

```bash
python3 docker/deploy.py check --timeout 60 \
  --output /artifacts/preflight_hardware_02.json
python3 docker/deploy.py mission --duration 60 \
  --output /artifacts/hardware_mission_01.json
```

`mission` calls the existing MULLET mission-start/HOLD workflow and records the
run. `check` is read-only. This guide describes deployment, not hardware safety
certification: localization, tracking, vehicle separation and failsafe behavior
must be validated separately. Obstacles/field remain simulated.

## Inspect, stop, upgrade

```bash
python3 docker/deploy.py validate
python3 docker/deploy.py --dry-run up
python3 docker/deploy.py stop
```

`logs` follows the last 100 log lines; Ctrl-C exits log viewing. `stop` stops
only this role's services; it does not stop MULLET or request a flight HOLD.
Use MULLET's established HOLD/landing workflow before stopping services.
`stop`, `status` and `logs` remain usable after editing a scenario file.

Compose project names are `mfgp_onboard` on a drone and `mfgp_laptop` on the
laptop. They differ from the existing SITL project. Do not run duplicate agents
or centrals on the same ROS domain across these projects. Upgrades should occur
with vehicles in HOLD: update the checkout, rebuild, run smoke, and run `up`
again with your intended command mode, then recheck readiness.

For an already distributed image, use `init --image registry/name:version` and
load/pull that image yourself, then run `validate`/`up`. For a local source
override, use `init --interfaces /path/to/mullet-interfaces`; that directory must
contain `package.xml`. This avoids fetching interface sources but other base
images/dependencies may still need downloading. `init --base-image ...` permits a compatible Jazzy image
or digest. Other ROS distributions are not supported by changing this flag.
Neither the default mutable base tag nor apt repositories are fully locked;
freeze a tested image digest for a particular flight campaign.

References: [Docker Buildx named contexts and local image loading](https://docs.docker.com/reference/cli/docker/buildx/build/),
[official ROS images](https://hub.docker.com/_/ros), and
[Linux host networking](https://docs.docker.com/engine/network/drivers/host/).

The first entry in the shared scenario `vehicles` list defines the map reference
vehicle. IDs may be renamed when their namespaces and host configurations agree;
keep vehicle order identical on all hosts.

### Existing MULLET instance 0 at the root namespace

Use the identity and namespace advertised by `/mullet/heartbeat` verbatim.
For an unchanged instance-0 MULLET reporting `drone_id: uav_0` and
`ros_namespace: /`, the first hardware scenario entry is:

```yaml
- id: uav_0
  namespace: /
  role: ground
```

Keep its remaining altitude and sensor settings. Our adapter accepts `/` and
resolves relative endpoints to `/command`, `/start_mission`, and `/hold`.
Heartbeat ID/namespace/schema checks remain strict; no aliases or remapping of
PX4 instance/system settings are performed. Multiple vehicles with the same ROS
namespace are rejected, including multiple root namespaces on the shared domain.
Confirm every other drone's heartbeat before assigning its scenario entry.

After distributing changed source and the identical hardware scenario to all
hosts, reinitialize the drone with `--vehicle uav_0 --domain 99` and the laptop
with `--role laptop --domain 99 --image mfgp-deployment:local`, both using
`--force --scenario docker/scenarios/hardware_three_drone.yaml`. Reinitializing
records the new scenario hash. Rebuild each image (SITL laptop:
`docker compose -f docker/compose.yaml build central`; hardware Jetson:
`python3 docker/deploy.py build`), run `smoke` and `validate`, then ordinary `up`
with commands disabled. A disarmed heartbeat should show `state: IDLE` rather
than `UNKNOWN`; registration and flight pose validity still require the current
armed/airborne gates. This configuration is not hardware flight validation.
