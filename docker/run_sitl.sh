#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Tools launched from a second terminal must inspect the same mounted manifest.
reuse_running_scenario() {
  if [[ -n "${MFGP_SCENARIO_FILE:-}" ]]; then return; fi
  local central_id running_scenario
  central_id="$(docker compose -f "$script_dir/compose.yaml" ps -q central)"
  if [[ -z "$central_id" ]]; then return; fi
  running_scenario="$(docker inspect --format '{{range .Mounts}}{{if eq .Destination "/config/scenario.yaml"}}{{.Source}}{{end}}{{end}}' "$central_id")"
  if [[ -n "$running_scenario" ]]; then
    export MFGP_SCENARIO_FILE="$running_scenario"
    echo "Using running scenario: $MFGP_SCENARIO_FILE" >&2
  fi
}
case "${1:-up}" in
  up)
    if [[ $# -gt 0 ]]; then shift; fi
    export ENABLE_COMMANDS=0
    if [[ "${1:-}" == "--motion" ]]; then
      export ENABLE_COMMANDS=1
      shift
    fi
    exec docker compose -f "$script_dir/compose.yaml" up --build "$@"
    ;;
  check)
    shift
    reuse_running_scenario
    exec docker compose -f "$script_dir/compose.yaml" run --rm -T --no-deps readiness "$@"
    ;;
  mission)
    shift
    reuse_running_scenario
    exec docker compose -f "$script_dir/compose.yaml" run --rm -T --no-deps mission "$@"
    ;;
  stop)
    shift
    exec docker compose -f "$script_dir/compose.yaml" stop "$@"
    ;;
  *)
    echo "Usage: bash docker/run_sitl.sh {up [--motion]|check [options]|mission --duration SECONDS [options]|stop}" >&2
    exit 2
    ;;
esac
