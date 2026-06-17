#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_DIR="${SIM_DIR:-$(cd "$ROOT_DIR/../.." && pwd)/Simulation}"
PYTHON="${PYTHON:-python3}"
OPERATOR_RECORDER_PYTHON="${OPERATOR_RECORDER_PYTHON:-/home/moontaehyung/Capstone/.venv/bin/python}"
OUTPUT_DIR=""
PROFILE_JSON=""
REPEATS="${ORIGIN_EVAL_REPEATS:-5}"
PROFILE_KIND="${ORIGIN_PROFILE_KIND:-auto}"
DRY_RUN=false
START_SIM=false
KEEP_SIM=false
VALIDATION_RUN=false
RESET_SIM_PER_REPEAT="${ORIGIN_EVAL_RESET_SIM_PER_REPEAT:-auto}"
SIM_STARTUP_S="${ORIGIN_EVAL_SIM_STARTUP_S:-45}"
SPAWN_FRONT_M="${ORIGIN_EVAL_SPAWN_FRONT_M:-}"
BBOX_ADJUSTMENT_PATH="${ORIGIN_EVAL_BBOX_ADJUSTMENT_PATH:-}"
MISSION_GAP_S="${ORIGIN_EVAL_MISSION_GAP_S:-6}"
RAW_VIEW=false
RANDOM_ORIGIN_PATH=false
RANDOM_SEED_BASE="${ORIGIN_EVAL_RANDOM_SEED_BASE:-$(date +%s)}"
RANDOM_WAYPOINT_COUNT="${ORIGIN_EVAL_RANDOM_WAYPOINT_COUNT:-6}"
RANDOM_RANGE_MIN_M="${ORIGIN_EVAL_RANDOM_RANGE_MIN_M:-}"
RANDOM_RANGE_MAX_M="${ORIGIN_EVAL_RANDOM_RANGE_MAX_M:-}"
RANDOM_LATERAL_FRACTION_LIMIT="${ORIGIN_EVAL_RANDOM_LATERAL_FRACTION_LIMIT:-0.85}"
RANDOM_ALTITUDE_JITTER_M="${ORIGIN_EVAL_RANDOM_ALTITUDE_JITTER_M:-2.5}"
SKIP_AUTO_MISSION=false
FAIL_FAST_AXIS_REGRESSION=false
AXIS_REGRESSION_TOLERANCE_M="${ORIGIN_EVAL_AXIS_REGRESSION_TOLERANCE_M:-0.000001}"
PX4_DIR="${PX4_DIR:-/home/moontaehyung/Capstone/source/PX4-Autopilot}"
PX4_PARAM_BIN="${PX4_PARAM_BIN:-$PX4_DIR/build/px4_sitl_default/bin/px4-param}"
PX4_ROOTFS_DIR="${PX4_ROOTFS_DIR:-$PX4_DIR/build/px4_sitl_default/rootfs}"
PX4_PARAM_OVERRIDES=()
if [[ -n "${ORIGIN_EVAL_PX4_PARAMS:-}" ]]; then
  IFS=',' read -r -a PX4_PARAM_OVERRIDES <<< "$ORIGIN_EVAL_PX4_PARAMS"
fi

usage() {
  cat <<'USAGE'
Usage:
  scripts/simulation_origin_evaluate.sh [options]

Applies a Simulation Origin Profile and evaluates both run_auto_mission.sh and
the Origin Profile path for 5+ repeated runs. Metrics compare YOLO Range against
Origin Range and report Forward/Right/Down position errors.

Requires Simulation/run_operator_view.sh to already be running.

Options:
  --output-dir DIR
  --profile JSON         Existing profile. If omitted, one calibration run is made.
  --profile-kind KIND    auto, alignment, or origin-bbox. Default: auto
  --repeats N            Default: 5
  --validation-run       Allow repeats below 5 for profile-method validation.
  --start-sim            Start Simulation/run_operator_view.sh before evaluation.
  --sim-startup-s N      Wait time after --start-sim. Default: 45
  --spawn-front-m N      Start/evaluate with target drone spawned Front N meters.
  --bbox-adjustment FILE Start Simulation with bbox-center correction CSV enabled.
  --mission-gap-s N      Wait between PX4 missions. Default: 6
  --px4-param NAME=VALUE Apply PX4 SITL param override after --start-sim startup wait.
                         Repeatable. Use only for validation-lane health gates.
  --reset-sim-per-repeat Restart Simulation before each repeat when --start-sim is set.
                         Default: auto for --start-sim with repeats > 1.
  --reuse-sim-for-repeats
                         Keep one Simulation instance for all repeats.
  --random-origin-path   Use a different random overlap-safe waypoint path per repeat.
  --random-seed-base N   Seed base for --random-origin-path. Seed = base + repeat_index - 1.
  --random-waypoint-count N
  --random-range-min-m N
  --random-range-max-m N
  --random-lateral-fraction-limit N
  --random-altitude-jitter-m N
  --skip-auto-mission    Evaluate only the origin/random waypoint path.
  --fail-fast-axis-regression
                         Stop immediately when corrected Right/Down/Forward RMSE increases over raw.
  --axis-regression-tolerance-m N
                         Tolerance for --fail-fast-axis-regression. Default: 0.000001
  --raw-view             Show raw camera topics during evaluation.
  --keep-sim             Do not stop the started Simulation process on exit.
  --dry-run              Print commands without running missions.
USAGE
}

while (($#)); do
  case "$1" in
    --output-dir) OUTPUT_DIR="${2:?missing value for --output-dir}"; shift 2 ;;
    --profile) PROFILE_JSON="${2:?missing value for --profile}"; shift 2 ;;
    --profile-kind) PROFILE_KIND="${2:?missing value for --profile-kind}"; shift 2 ;;
    --repeats) REPEATS="${2:?missing value for --repeats}"; shift 2 ;;
    --validation-run) VALIDATION_RUN=true; shift ;;
    --start-sim) START_SIM=true; shift ;;
    --sim-startup-s) SIM_STARTUP_S="${2:?missing value for --sim-startup-s}"; shift 2 ;;
    --spawn-front-m) SPAWN_FRONT_M="${2:?missing value for --spawn-front-m}"; shift 2 ;;
    --bbox-adjustment) BBOX_ADJUSTMENT_PATH="${2:?missing value for --bbox-adjustment}"; shift 2 ;;
    --mission-gap-s) MISSION_GAP_S="${2:?missing value for --mission-gap-s}"; shift 2 ;;
    --px4-param) PX4_PARAM_OVERRIDES+=("${2:?missing value for --px4-param}"); shift 2 ;;
    --reset-sim-per-repeat) RESET_SIM_PER_REPEAT=true; shift ;;
    --reuse-sim-for-repeats) RESET_SIM_PER_REPEAT=false; shift ;;
    --random-origin-path) RANDOM_ORIGIN_PATH=true; shift ;;
    --random-seed-base) RANDOM_SEED_BASE="${2:?missing value for --random-seed-base}"; shift 2 ;;
    --random-waypoint-count) RANDOM_WAYPOINT_COUNT="${2:?missing value for --random-waypoint-count}"; shift 2 ;;
    --random-range-min-m) RANDOM_RANGE_MIN_M="${2:?missing value for --random-range-min-m}"; shift 2 ;;
    --random-range-max-m) RANDOM_RANGE_MAX_M="${2:?missing value for --random-range-max-m}"; shift 2 ;;
    --random-lateral-fraction-limit) RANDOM_LATERAL_FRACTION_LIMIT="${2:?missing value for --random-lateral-fraction-limit}"; shift 2 ;;
    --random-altitude-jitter-m) RANDOM_ALTITUDE_JITTER_M="${2:?missing value for --random-altitude-jitter-m}"; shift 2 ;;
    --skip-auto-mission) SKIP_AUTO_MISSION=true; shift ;;
    --fail-fast-axis-regression) FAIL_FAST_AXIS_REGRESSION=true; shift ;;
    --axis-regression-tolerance-m) AXIS_REGRESSION_TOLERANCE_M="${2:?missing value for --axis-regression-tolerance-m}"; shift 2 ;;
    --raw-view) RAW_VIEW=true; shift ;;
    --keep-sim) KEEP_SIM=true; shift ;;
    --dry-run) DRY_RUN=true; shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$OUTPUT_DIR" ]]; then
  OUTPUT_DIR="$SIM_DIR/bags/origin_eval_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$OUTPUT_DIR"

if (( REPEATS < 5 )) && [[ "$VALIDATION_RUN" != true ]]; then
  echo "repeats must be at least 5 for reliability" >&2
  exit 2
fi
if [[ "$RESET_SIM_PER_REPEAT" == "auto" ]]; then
  if [[ "$START_SIM" == true ]] && (( REPEATS > 1 )); then
    RESET_SIM_PER_REPEAT=true
  else
    RESET_SIM_PER_REPEAT=false
  fi
elif [[ "$RESET_SIM_PER_REPEAT" != true && "$RESET_SIM_PER_REPEAT" != false ]]; then
  echo "ORIGIN_EVAL_RESET_SIM_PER_REPEAT must be auto, true, or false" >&2
  exit 2
fi

sim_pid=""
cleanup_sim() {
  if [[ "$KEEP_SIM" == true || -z "$sim_pid" ]]; then
    return 0
  fi
  local pid="$sim_pid"
  sim_pid=""
  kill -TERM "-$pid" >/dev/null 2>&1 || kill "$pid" >/dev/null 2>&1 || true
  wait "$pid" >/dev/null 2>&1 || true
}
trap cleanup_sim EXIT INT TERM

wait_for_px4_param_server() {
  if [[ "$DRY_RUN" == true || ${#PX4_PARAM_OVERRIDES[@]} -eq 0 ]]; then
    return 0
  fi
  if [[ ! -x "$PX4_PARAM_BIN" ]]; then
    echo "px4-param not found or not executable: $PX4_PARAM_BIN" >&2
    return 1
  fi
  for _ in $(seq 1 30); do
    if (cd "$PX4_ROOTFS_DIR" && "$PX4_PARAM_BIN" show SYS_AUTOSTART >/dev/null 2>&1); then
      return 0
    fi
    sleep 1
  done
  echo "PX4 parameter server did not become ready" >&2
  return 1
}

apply_px4_param_overrides() {
  if [[ ${#PX4_PARAM_OVERRIDES[@]} -eq 0 ]]; then
    return 0
  fi
  if [[ "$DRY_RUN" == true ]]; then
    local override
    for override in "${PX4_PARAM_OVERRIDES[@]}"; do
      printf 'DRY RUN px4-param set %q\n' "$override"
    done
    return 0
  fi
  wait_for_px4_param_server
  local override name value
  for override in "${PX4_PARAM_OVERRIDES[@]}"; do
    if [[ "$override" != *=* ]]; then
      echo "--px4-param expects NAME=VALUE, got: $override" >&2
      return 2
    fi
    name="${override%%=*}"
    value="${override#*=}"
    printf 'Applying PX4 param override: %s=%s\n' "$name" "$value"
    (cd "$PX4_ROOTFS_DIR" && "$PX4_PARAM_BIN" set "$name" "$value")
  done
}

start_simulation() {
  local log_path="${1:-$OUTPUT_DIR/operator_view.log}"
  sim_env=()
  sim_args=()
  if [[ -n "$SPAWN_FRONT_M" ]]; then
    sim_env+=(DRONE_SPAWN_Y_M="$SPAWN_FRONT_M")
  fi
  if [[ -n "$BBOX_ADJUSTMENT_PATH" ]]; then
    sim_env+=(BBOX_CENTER_ADJUSTMENT_ENABLE=true)
    sim_env+=(BBOX_CENTER_ADJUSTMENT_PATH="$BBOX_ADJUSTMENT_PATH")
  fi
  if [[ "$RAW_VIEW" == true ]]; then
    sim_args+=(--raw-view)
  fi
  sim_args+=(gz_gui:=false launch_rviz:=false)
  if [[ "$DRY_RUN" == true ]]; then
    printf 'DRY RUN start Simulation:'
    printf ' %q' "${sim_env[@]}" "$SIM_DIR/run_operator_view.sh" "${sim_args[@]}"
    printf '\n'
  else
    cleanup_sim
    mkdir -p "$(dirname "$log_path")"
    env "${sim_env[@]}" setsid "$SIM_DIR/run_operator_view.sh" "${sim_args[@]}" \
      >"$log_path" 2>&1 &
    sim_pid="$!"
    printf 'Started Simulation PID %s. Log: %s\n' "$sim_pid" "$log_path"
    sleep "$SIM_STARTUP_S"
    apply_px4_param_overrides
  fi
}

if [[ "$START_SIM" == true && "$RESET_SIM_PER_REPEAT" != true ]]; then
  start_simulation "$OUTPUT_DIR/operator_view.log"
fi

if [[ -z "$PROFILE_JSON" ]]; then
  profile_dir="$OUTPUT_DIR/profile_source"
  if [[ "$START_SIM" == true && "$RESET_SIM_PER_REPEAT" == true ]]; then
    start_simulation "$profile_dir/operator_view.log"
  fi
  profile_args=()
  build_profile_kind="$PROFILE_KIND"
  if [[ "$build_profile_kind" == "auto" ]]; then
    build_profile_kind="origin-bbox"
  fi
  profile_args+=(--profile-kind "$build_profile_kind")
  if [[ -n "$SPAWN_FRONT_M" ]]; then
    profile_args+=(--spawn-front-m "$SPAWN_FRONT_M")
  fi
  if [[ "$DRY_RUN" == true ]]; then
    "$ROOT_DIR/scripts/simulation_origin_profile_generate.sh" --output-dir "$profile_dir" "${profile_args[@]}" --dry-run
    PROFILE_JSON="$profile_dir/origin_profile.json"
  else
    "$ROOT_DIR/scripts/simulation_origin_profile_generate.sh" --output-dir "$profile_dir" "${profile_args[@]}"
    PROFILE_JSON="$profile_dir/origin_profile.json"
  fi
  if [[ "$START_SIM" == true && "$RESET_SIM_PER_REPEAT" == true ]]; then
    cleanup_sim
  fi
fi

source_ros() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  if [[ -f "$SIM_DIR/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$SIM_DIR/install/setup.bash"
  fi
  set -u
}

record_auto_mission() {
  local run_dir="$1"
  mkdir -p "$run_dir"
  if [[ "$DRY_RUN" == true ]]; then
    printf 'DRY RUN auto recorder: %q -m stereo_ground_sim.operator_recorder --mode lite --output-dir %q\n' "$OPERATOR_RECORDER_PYTHON" "$run_dir/telemetry"
    printf 'DRY RUN auto mission: %q --target-distance-m 300\n' "$SIM_DIR/run_auto_mission.sh"
    return 0
  fi
  source_ros
  setsid "$OPERATOR_RECORDER_PYTHON" -m stereo_ground_sim.operator_recorder \
    --mode lite \
    --output-dir "$run_dir/telemetry" &
  local recorder_pid="$!"
  cleanup_recorder() {
    kill -TERM "-$recorder_pid" >/dev/null 2>&1 || kill "$recorder_pid" >/dev/null 2>&1 || true
    wait "$recorder_pid" >/dev/null 2>&1 || true
  }
  trap cleanup_recorder RETURN
  sleep 1
  "$SIM_DIR/run_auto_mission.sh" --target-distance-m 300
}

evaluate_telemetry() {
  local label="$1"
  local telemetry_csv="$2"
  local out_dir="$3"
  "$PYTHON" "$ROOT_DIR/scripts/simulation_origin_tools.py" evaluate \
    --telemetry "$telemetry_csv" \
    --profile "$PROFILE_JSON" \
    --profile-kind "$PROFILE_KIND" \
    --comparison-csv "$out_dir/${label}_comparison.csv" \
    --summary-json "$out_dir/${label}_summary.json" \
    --label "$label"
}

check_axis_regression() {
  local summary_json="$1"
  if [[ "$FAIL_FAST_AXIS_REGRESSION" != true || "$DRY_RUN" == true ]]; then
    return 0
  fi
  "$PYTHON" - "$summary_json" "$AXIS_REGRESSION_TOLERANCE_M" <<'PY'
import json
import sys

summary_path = sys.argv[1]
tolerance_m = float(sys.argv[2])
summary = json.load(open(summary_path, encoding="utf-8"))
failures = []
for axis in ("right", "down", "forward"):
    raw = summary.get(f"raw_{axis}_error_rmse_m")
    corrected = summary.get(f"origin_bbox_{axis}_error_rmse_m")
    if raw is None or corrected is None:
        failures.append(f"{axis}: missing RMSE")
        continue
    raw = float(raw)
    corrected = float(corrected)
    if corrected > raw + tolerance_m:
        failures.append(f"{axis}: {raw:.6f}->{corrected:.6f}")
if failures:
    print(f"Axis regression failed for {summary.get('label', summary_path)}: " + "; ".join(failures), file=sys.stderr)
    sys.exit(1)
PY
}

mission_gap() {
  if [[ "$DRY_RUN" == true ]]; then
    return 0
  fi
  sleep "$MISSION_GAP_S"
}

summary_files=()
for index in $(seq 1 "$REPEATS"); do
  run_dir="$OUTPUT_DIR/run_${index}"
  auto_dir="$run_dir/auto"
  path_dir="$run_dir/origin_path"
  mkdir -p "$run_dir"
  if [[ "$START_SIM" == true && "$RESET_SIM_PER_REPEAT" == true ]]; then
    start_simulation "$run_dir/operator_view.log"
  fi

  if [[ "$SKIP_AUTO_MISSION" != true ]]; then
    printf '=== Repeat %s/%s: auto mission ===\n' "$index" "$REPEATS"
    record_auto_mission "$auto_dir"
    if [[ "$DRY_RUN" == false ]]; then
      evaluate_telemetry "auto_${index}" "$auto_dir/telemetry/telemetry.csv" "$auto_dir"
      auto_summary="$auto_dir/auto_${index}_summary.json"
      check_axis_regression "$auto_summary"
      summary_files+=("$auto_summary")
      mission_gap
    fi
  fi

  printf '=== Repeat %s/%s: origin profile path ===\n' "$index" "$REPEATS"
  path_args=()
  if [[ -n "$SPAWN_FRONT_M" ]]; then
    path_args+=(--spawn-front-m "$SPAWN_FRONT_M")
  fi
  if [[ "$DRY_RUN" == true ]]; then
    path_args+=(--dry-run)
  fi
  if [[ "$RANDOM_ORIGIN_PATH" == true ]]; then
    path_args+=(
      --randomize
      --random-seed "$((RANDOM_SEED_BASE + index - 1))"
      --random-waypoint-count "$RANDOM_WAYPOINT_COUNT"
      --random-lateral-fraction-limit "$RANDOM_LATERAL_FRACTION_LIMIT"
      --random-altitude-jitter-m "$RANDOM_ALTITUDE_JITTER_M"
    )
    if [[ -n "$RANDOM_RANGE_MIN_M" ]]; then
      path_args+=(--random-range-min-m "$RANDOM_RANGE_MIN_M")
    fi
    if [[ -n "$RANDOM_RANGE_MAX_M" ]]; then
      path_args+=(--random-range-max-m "$RANDOM_RANGE_MAX_M")
    fi
  fi
  "$ROOT_DIR/scripts/simulation_origin_profile_path.sh" --output-dir "$path_dir" "${path_args[@]}"
  if [[ "$DRY_RUN" == false ]]; then
    evaluate_telemetry "origin_path_${index}" "$path_dir/telemetry/telemetry.csv" "$path_dir"
    path_summary="$path_dir/origin_path_${index}_summary.json"
    check_axis_regression "$path_summary"
    summary_files+=("$path_summary")
    mission_gap
  fi
  if [[ "$START_SIM" == true && "$RESET_SIM_PER_REPEAT" == true ]]; then
    cleanup_sim
  fi
done

if [[ "$DRY_RUN" == false ]]; then
  "$PYTHON" "$ROOT_DIR/scripts/simulation_origin_tools.py" aggregate \
    --summary-json "$OUTPUT_DIR/origin_eval_summary.json" \
    "${summary_files[@]}"
  printf 'Aggregate summary: %s\n' "$OUTPUT_DIR/origin_eval_summary.json"
else
  printf 'DRY RUN output root: %s\n' "$OUTPUT_DIR"
fi
