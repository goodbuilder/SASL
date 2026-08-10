#!/usr/bin/env bash
# Shared config loader for Simulation shell entry points.
#
# Source this file after setting ROOT_DIR and optional CALLER_CWD, SIM_CONFIG,
# and SIM_SCENARIO_CONFIG. Then call:
#
#   load_sim_config_preserving_env VAR_NAME ...
#
# The named variables keep values that were explicitly present in the caller
# environment before config files are sourced. Unset variables are populated
# from config/simulation.env and then optional scenario config.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  printf '%s\n' "error: scripts/load_sim_config.sh must be sourced, not executed." >&2
  exit 2
fi

load_sim_config_preserving_env() {
  if [[ -z "${ROOT_DIR:-}" ]]; then
    printf '%s\n' "error: ROOT_DIR must be set before sourcing simulation config." >&2
    return 2
  fi

  SIM_CONFIG="${SIM_CONFIG:-$ROOT_DIR/config/simulation.env}"
  SIM_SCENARIO_CONFIG="${SIM_SCENARIO_CONFIG:-}"

  local name scenario_path
  local -A explicit_set=()
  local -A explicit_value=()
  for name in "$@"; do
    if [[ -v "$name" ]]; then
      explicit_set["$name"]=1
      explicit_value["$name"]="${!name}"
    fi
  done

  _restore_explicit_sim_env() {
    local restore_name
    for restore_name in "$@"; do
      if [[ -n "${explicit_set[$restore_name]:-}" ]]; then
        printf -v "$restore_name" '%s' "${explicit_value[$restore_name]}"
        export "$restore_name"
      fi
    done
  }

  if [[ ! -f "$SIM_CONFIG" ]]; then
    printf '%s\n' "error: SIM_CONFIG not found: $SIM_CONFIG" >&2
    return 2
  fi

  set -a
  # shellcheck disable=SC1090
  source "$SIM_CONFIG"
  set +a
  _restore_explicit_sim_env "$@"

  if [[ -n "$SIM_SCENARIO_CONFIG" ]]; then
    scenario_path="$SIM_SCENARIO_CONFIG"
    if [[ "$scenario_path" != /* && -n "${CALLER_CWD:-}" && -f "$CALLER_CWD/$scenario_path" ]]; then
      scenario_path="$CALLER_CWD/$scenario_path"
    elif [[ "$scenario_path" != /* && -f "$ROOT_DIR/config/scenarios/$scenario_path" ]]; then
      scenario_path="$ROOT_DIR/config/scenarios/$scenario_path"
    elif [[ "$scenario_path" != /* && -f "$ROOT_DIR/$scenario_path" ]]; then
      scenario_path="$ROOT_DIR/$scenario_path"
    fi
    if [[ ! -f "$scenario_path" ]]; then
      printf '%s\n' "error: SIM_SCENARIO_CONFIG not found: $SIM_SCENARIO_CONFIG" >&2
      return 2
    fi
    SIM_SCENARIO_CONFIG="$scenario_path"
    export SIM_SCENARIO_CONFIG
    set -a
    # shellcheck disable=SC1090
    source "$SIM_SCENARIO_CONFIG"
    set +a
    _restore_explicit_sim_env "$@"
  fi

  unset -f _restore_explicit_sim_env
}
