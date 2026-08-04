#!/usr/bin/env bash
set -euo pipefail

# Explicit calibration runner.  Each case restarts Gazebo/PX4 because the
# physical optical pose is generated in the world SDF at launch.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if REPO_ROOT="$(git -C "$ROOT_DIR" rev-parse --show-toplevel 2>/dev/null)"; then
  :
else
  REPO_ROOT="$(cd "$ROOT_DIR/.." && pwd)"
fi

search_text() {
  if command -v rg >/dev/null 2>&1; then
    rg "$@"
  else
    grep -E "$@"
  fi
}

search_quiet() {
  local pattern="$1" file="$2"
  if command -v rg >/dev/null 2>&1; then
    rg -q "$pattern" "$file"
  else
    grep -Eq "$pattern" "$file"
  fi
}

SIM_CONFIG="${SIM_CONFIG:-$ROOT_DIR/config/simulation.env}"
SIM_SCENARIO_CONFIG="${SIM_SCENARIO_CONFIG:-}"
SIM_EXPLICIT_NAMES=(
  SWEEP_AXES SWEEP_VALUES_DEG SWEEP_REPETITIONS SWEEP_OUTPUT_DIR
  SWEEP_WAIT_S SWEEP_SEED SWEEP_RANDOMIZE SWEEP_SKIP_COLCON_BUILD
  SWEEP_SKIP_PX4_BUILD
  SWEEP_KEEP_GOING SWEEP_FORCE_EXISTING_SIM SWEEP_CAMERA_UPDATE_RATE_HZ
  SWEEP_YOLO_TARGET_HZ
)
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/load_sim_config.sh"
load_sim_config_preserving_env "${SIM_EXPLICIT_NAMES[@]}"

usage() {
  cat <<'USAGE'
Usage:
  ./run_auto_attitude_sweep.sh [options] -- [run_auto.sh options]

Runs the PX4/FOV-guarded Auto mission repeatedly. Each case launches a fresh
simulation with one right-camera optical RDF attitude perturbation and enables
the explicit S2 research pair recorder for offline calibration/validation.

Options:
  --axes LIST              Comma-separated axes: roll,pitch,yaw. Default: all.
  --values-deg LIST        Comma-separated degrees. Default: -1,0,1.
  --repetitions N          Repeat every non-zero attitude case. Default: 1.
  --output-dir DIR         Evidence directory. Default: Simulation/recordings/auto-attitude-sweep-....
  --px4-dir DIR            PX4 checkout passed to run_operator_view.sh.
  --skip-colcon-build      Reuse the existing ROS install for every case.
  --skip-px4-build         Reuse the existing PX4 SITL build for every case.
  --wait-s N               PX4 readiness timeout per case. Default: 120.
  --camera-hz N            Camera/measurement sampling Hz per sweep case. Default: 2.
  --yolo-hz N              YOLO target Hz per sweep case. Default: camera Hz.
  --seed N                 Randomization seed. Default: current epoch seconds.
  --ordered                Keep axis/value order instead of randomizing it.
  --keep-going             Continue after a failed case.
  --force-existing-sim     Allow cleanup of an existing operator session.
  --help, -h               Show this help.

The non-zero attitude cases are randomized by default. A 0 deg baseline runs
before each condition and once after the final condition, so the default
sweep has thirteen cases: six attitude cases plus seven baselines. Use
--quick after `--` only for smoke tests.

This explicit research mode also starts a full telemetry/MP4 recorder per case.
It stays armed but writes no CSV rows or MP4 frames until the first valid
left/right bbox pair is detected. Normal run_auto.sh behavior is unchanged:
interactive Auto still records only after rqt Save CSV or Save MP4 is pressed.
Defaults are loaded from SIM_CONFIG (normally config/simulation.env); CLI
options override SWEEP_* environment values.

Coordinate contract:
  injected attitude: right-camera camera_optical_rdf, degrees;
  Gazebo world: ENU; PX4 flight control: local NED/body FRD;
  recorded public vectors: baseline_center_frd [Forward, Right, Down], metres;
  Gazebo Target GT: offline calibration/validation truth only.
  case PASS: PX4 readiness and Auto exit successfully, and the S2 recorder
  produces RUN_FINALIZED_VALID, run_valid=true, and pair_records.csv.
USAGE
}

normalize_bool() {
  local name="$1" value="$2"
  case "$value" in
    1|true|TRUE|yes|YES|on|ON) printf 'true' ;;
    0|false|FALSE|no|NO|off|OFF) printf 'false' ;;
    *) printf 'error: %s must be true or false, got: %s\n' "$name" "$value" >&2; exit 2 ;;
  esac
}

axes_csv="${SWEEP_AXES:-roll,pitch,yaw}"
values_csv="${SWEEP_VALUES_DEG:--1,0,1}"
repetitions="${SWEEP_REPETITIONS:-1}"
output_dir="${SWEEP_OUTPUT_DIR:-}"
px4_dir="${PX4_DIR:-}"
skip_colcon_build="$(normalize_bool SWEEP_SKIP_COLCON_BUILD "${SWEEP_SKIP_COLCON_BUILD:-false}")"
skip_px4_build="$(normalize_bool SWEEP_SKIP_PX4_BUILD "${SWEEP_SKIP_PX4_BUILD:-false}")"
wait_s="${SWEEP_WAIT_S:-120}"
camera_update_rate_hz="${SWEEP_CAMERA_UPDATE_RATE_HZ:-2.0}"
yolo_target_hz="${SWEEP_YOLO_TARGET_HZ:-$camera_update_rate_hz}"
seed="${SWEEP_SEED:-$(date +%s)}"
randomize="$(normalize_bool SWEEP_RANDOMIZE "${SWEEP_RANDOMIZE:-true}")"
keep_going="$(normalize_bool SWEEP_KEEP_GOING "${SWEEP_KEEP_GOING:-false}")"
force_existing_sim="$(normalize_bool SWEEP_FORCE_EXISTING_SIM "${SWEEP_FORCE_EXISTING_SIM:-false}")"
auto_args=()

while (($#)); do
  case "$1" in
    --)
      shift
      auto_args+=("$@")
      break
      ;;
    --axes) axes_csv="${2:?missing value for --axes}"; shift 2 ;;
    --values-deg) values_csv="${2:?missing value for --values-deg}"; shift 2 ;;
    --repetitions) repetitions="${2:?missing value for --repetitions}"; shift 2 ;;
    --output-dir) output_dir="${2:?missing value for --output-dir}"; shift 2 ;;
    --px4-dir) px4_dir="${2:?missing value for --px4-dir}"; shift 2 ;;
    --skip-colcon-build) skip_colcon_build=true; shift ;;
    --skip-px4-build) skip_px4_build=true; shift ;;
    --wait-s) wait_s="${2:?missing value for --wait-s}"; shift 2 ;;
    --camera-hz) camera_update_rate_hz="${2:?missing value for --camera-hz}"; shift 2 ;;
    --yolo-hz) yolo_target_hz="${2:?missing value for --yolo-hz}"; shift 2 ;;
    --seed) seed="${2:?missing value for --seed}"; shift 2 ;;
    --ordered) randomize=false; shift ;;
    --keep-going) keep_going=true; shift ;;
    --force-existing-sim) force_existing_sim=true; shift ;;
    --help|-h) usage; exit 0 ;;
    *) printf 'error: unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

observation_mode="bbox_yolo"
for auto_arg in "${auto_args[@]}"; do
  if [[ "$auto_arg" == observation_mode:=* ]]; then
    observation_mode="${auto_arg#observation_mode:=}"
  fi
done
auto_args_serialized=""
if ((${#auto_args[@]} > 0)); then
  printf -v auto_args_serialized '%s\037' "${auto_args[@]}"
fi

if [[ ! "$repetitions" =~ ^[1-9][0-9]*$ ]]; then
  printf '%s\n' 'error: --repetitions must be a positive integer' >&2
  exit 2
fi
if ! awk -v value="$wait_s" 'BEGIN { exit !(value == value && value > 0) }'; then
  printf '%s\n' 'error: --wait-s must be a positive number' >&2
  exit 2
fi
for rate_name in camera_update_rate_hz yolo_target_hz; do
  rate_value="${!rate_name}"
  if ! awk -v value="$rate_value" 'BEGIN { exit !(value == value && value > 0) }'; then
    printf 'error: %s must be a positive number\n' "$rate_name" >&2
    exit 2
  fi
done
wait_seconds="$(awk -v value="$wait_s" 'BEGIN { printf "%d", value + 0.999 }')"
if ! awk -v value="$seed" 'BEGIN { exit !(value != "" && value == value && value + 0 == value) }'; then
  printf '%s\n' 'error: --seed must be numeric' >&2
  exit 2
fi

IFS=',' read -r -a axes <<< "$axes_csv"
if ((${#axes[@]} == 0)); then
  printf '%s\n' 'error: --axes must not be empty' >&2
  exit 2
fi
for axis in "${axes[@]}"; do
  axis="${axis//[[:space:]]/}"
  case "$axis" in
    roll|pitch|yaw) ;;
    *) printf 'error: unsupported axis %s; use roll,pitch,yaw\n' "$axis" >&2; exit 2 ;;
  esac
done
IFS=',' read -r -a values <<< "$values_csv"
if ((${#values[@]} == 0)); then
  printf '%s\n' 'error: --values-deg must not be empty' >&2
  exit 2
fi
for value in "${values[@]}"; do
  value="${value//[[:space:]]/}"
  if ! awk -v value="$value" 'BEGIN { exit !(value != "" && value == value && value + 0 == value) }'; then
    printf 'error: invalid degree value: %s\n' "$value" >&2
    exit 2
  fi
done

if [[ -z "$output_dir" ]]; then
  output_dir="$ROOT_DIR/recordings/auto-attitude-sweep-$(date +%Y%m%d_%H%M%S)"
fi
output_dir="$(realpath -m "$output_dir")"
recordings_root="$(realpath -m "$ROOT_DIR/recordings")"
case "$output_dir" in
  "$recordings_root"|"$recordings_root"/*)
    ;;
  "$REPO_ROOT"|"$REPO_ROOT"/*)
    printf '%s\n' "error: repository output is allowed only under $recordings_root" >&2
    exit 2
    ;;
esac
if [[ -e "$output_dir" ]]; then
  if [[ ! -d "$output_dir" ]] || [[ -n "$(find "$output_dir" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
    printf 'error: output directory must be new or empty: %s\n' "$output_dir" >&2
    exit 2
  fi
else
  mkdir -p "$output_dir"
fi

if [[ "$force_existing_sim" != true ]]; then
  existing_session="$(ps -eo args= | search_text -e '(^|/)run_operator_view\.sh([[:space:]]|$)|ros2 launch stereo_ground_sim stereo_ground_sim\.launch.py|px4_auto_mission\.py' || true)"
  if [[ -n "$existing_session" ]]; then
    printf '%s\n' \
      'error: an operator/PX4/Auto session is already running.' \
      'Stop it first, or pass --force-existing-sim if cleaning it is intentional.' >&2
    exit 2
  fi
fi

branch="$(git -C "$REPO_ROOT" branch --show-current 2>/dev/null || true)"
commit_sha="$(git -C "$REPO_ROOT" rev-parse HEAD 2>/dev/null || true)"
git_status=dirty
if [[ -z "$(git -C "$REPO_ROOT" status --short 2>/dev/null)" ]]; then git_status=clean; fi
session_id="auto-attitude-sweep-$(date -u +%Y%m%dT%H%M%SZ)-$$"
run_id="auto-attitude-sweep"
manifest="$output_dir/sweep_manifest.tsv"
printf 'execution_order\tcase_id\taxis\tangle_deg\trepetition\tstatus\trecording_dir\teffective_config_sha256\n' > "$manifest"
printf '%s\n' \
  "output_dir=$output_dir" \
  "session_id=$session_id" \
  "run_id=$run_id" \
  "repository=$REPO_ROOT" \
  "commit_sha=$commit_sha" \
  "git_status=$git_status" \
  "randomize=$randomize" \
  "seed=$seed" \
  "attitude_frame=camera_optical_rdf" \
  "output_frame=baseline_center_frd[forward,right,down]" \
  "truth_source=Gazebo Target GT (offline calibration/validation only)" \
  "camera_update_rate_hz=$camera_update_rate_hz" \
  "yolo_target_hz=$yolo_target_hz" \
  > "$output_dir/sweep_contract.txt"

write_effective_config() {
  local config_path="$1" case_id="$2" axis="$3" angle="$4" repetition="$5"
  local roll="$6" pitch="$7" yaw="$8" recording_dir="$9" operator_recording_dir="${10}"
  python3 - "$config_path" "$session_id" "$run_id" "$case_id" "$axis" "$angle" "$repetition" "$observation_mode" \
    "$roll" "$pitch" "$yaw" "$seed" "$camera_update_rate_hz" "$yolo_target_hz" \
    "${CAMERA_BASELINE_M:-25.0}" "${CAMERA_HEIGHT_M:-30.0}" "${CAMERA_HORIZONTAL_FOV_RAD:-0.154997710698}" \
    "${CAMERA_IMAGE_WIDTH_PX:-1280}" "${CAMERA_IMAGE_HEIGHT_PX:-720}" \
    "$SIM_CONFIG" "$REPO_ROOT" "$branch" "$commit_sha" "$git_status" \
    "$recording_dir" "$operator_recording_dir" "$auto_args_serialized" <<'PY'
import hashlib
import json
import math
import sys
from pathlib import Path

(path, session_id, run_id, case_id, axis, angle, repetition, observation_mode, roll, pitch, yaw,
 seed, camera_hz, yolo_hz, baseline_m, camera_height_m, fov_rad, image_width, image_height,
 sim_config, repo_root, branch, commit_sha, git_status,
 recording_dir, operator_recording_dir, auto_args_serialized) = sys.argv[1:]
payload = {
    "schema_version": "simulation_effective_config_v1",
    "session_id": session_id,
    "run_id": run_id,
    "attempt_id": case_id,
    "case_id": case_id,
    "experiment": {
        "sweep_axis": axis,
        "sweep_value_deg": float(angle),
        "repetition_index": int(repetition),
        "is_baseline_case": axis == "baseline",
        "random_seed": float(seed),
        "application_mode": "right_optical_ofat",
        "observation_mode": observation_mode,
        "run_auto_args": [item for item in auto_args_serialized.split("\x1f") if item],
    },
    "attitude_injection": {
        "target": "right_camera_optical",
        "frame": "camera_optical_rdf",
        "left_applied_deg": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        "right_applied_deg": {"roll": float(roll), "pitch": float(pitch), "yaw": float(yaw)},
        "composition": "R_Down(yaw) @ R_Right(pitch) @ R_Forward(roll)",
        "euler_order": "roll_pitch_yaw",
    },
    "camera": {
        "image_width_px": int(image_width),
        "image_height_px": int(image_height),
        "baseline_m": float(baseline_m),
        "camera_height_m": float(camera_height_m),
        "horizontal_fov_rad": float(fov_rad),
        "camera_update_rate_hz": float(camera_hz),
        "yolo_target_hz": float(yolo_hz),
        "intrinsic_profile_id": "simulation_pinhole_from_horizontal_fov",
        "intrinsic_profile_sha256": "not_applicable_generated_from_effective_config",
        "fx_px": (float(image_width) * 0.5) / math.tan(float(fov_rad) * 0.5),
        "fy_px": (float(image_width) * 0.5) / math.tan(float(fov_rad) * 0.5),
        "cx_px": float(image_width) * 0.5,
        "cy_px": float(image_height) * 0.5,
        "skew_px": 0.0,
        "distortion_model": "none",
        "distortion_coefficients": [],
        "pixel_origin": "top_left",
        "u_axis_direction": "right",
        "v_axis_direction": "down",
        "distorted_or_undistorted": "undistorted_pinhole",
        "crop_policy": "none",
        "resize_policy": "none",
        "letterbox_policy": "none",
    },
    "coordinate_contract": {
        "gazebo_world": "ENU",
        "px4_local": "NED",
        "px4_body": "FRD",
        "camera_optical": "RDF",
        "output_frame": "baseline_center_frd",
        "output_axes": "forward_right_down",
        "length_unit": "m",
        "angle_unit": "deg",
        "px4_quaternion_order": "wxyz",
        "gazebo_quaternion_order": "xyzw",
    },
    "ground_truth": {
        "source": "gazebo",
        "model": "x500_15m",
        "link": "vehicle_reference_link",
        "runtime_feature_allowed": False,
    },
    "provenance": {
        "simulation_env_path": sim_config,
        "repository": repo_root,
        "git_branch": branch,
        "git_head_sha": commit_sha,
        "git_status": git_status,
    },
    "outputs": {
        "recording_dir": recording_dir,
        "operator_recording_dir": operator_recording_dir,
        "effective_config_path": str(Path(path).resolve()),
        "sample_jsonl": str(Path(recording_dir) / "pair_records.jsonl"),
        "sample_csv": str(Path(recording_dir) / "pair_records.csv"),
        "metadata": str(Path(recording_dir) / "run_metadata.json"),
        "summary": str(Path(recording_dir) / "run_summary.json"),
    },
}
canonical = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
payload["effective_config_sha256"] = hashlib.sha256(canonical.encode("utf-8")).hexdigest()
Path(path).write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
print(payload["effective_config_sha256"])
PY
}

operator_pid=""
stop_operator() {
  local pid="$operator_pid"
  operator_pid=""
  [[ -n "$pid" ]] || return 0
  if kill -0 "$pid" >/dev/null 2>&1; then
    # The operator is launched as a background shell, where SIGINT may be
    # ignored. SIGTERM reliably runs run_operator_view.sh's cleanup trap,
    # which first finalizes the S2 research recorder before stopping ROS/PX4.
    kill -TERM "$pid" >/dev/null 2>&1 || true
    for _ in {1..240}; do
      kill -0 "$pid" >/dev/null 2>&1 || break
      sleep 1
    done
    kill -KILL "$pid" >/dev/null 2>&1 || true
  fi
  wait "$pid" >/dev/null 2>&1 || true
}
trap stop_operator EXIT INT TERM

wait_for_px4() {
  local deadline=$((SECONDS + wait_seconds))
  local check_log="$1"
  while ((SECONDS < deadline)); do
    kill -0 "$operator_pid" >/dev/null 2>&1 || return 1
    if SIM_CONFIG="$SIM_CONFIG" "$ROOT_DIR/run_auto.sh" --check-connection >> "$check_log" 2>&1; then
      return 0
    fi
    sleep 2
  done
  return 1
}

format_value() {
  local value="$1"
  value="${value//+/p}"; value="${value//-/m}"; value="${value//./p}"
  printf '%s' "$value"
}

condition_specs=()
for axis in "${axes[@]}"; do
  axis="${axis//[[:space:]]/}"
  for value in "${values[@]}"; do
    value="${value//[[:space:]]/}"
    if awk -v value="$value" 'BEGIN { exit !(value + 0 == 0) }'; then continue; fi
    for ((repetition = 1; repetition <= repetitions; repetition++)); do
      condition_specs+=("$axis|$value|$repetition")
    done
  done
done

condition_order=()
if [[ "$randomize" == true && ${#condition_specs[@]} -gt 1 ]]; then
  mapfile -t condition_order < <(
    printf '%s\n' "${condition_specs[@]}" \
      | awk -v seed="$seed" 'BEGIN { srand(seed) } { printf "%.17f\t%d\n", rand(), NR - 1 }' \
      | sort -n -k1,1 \
      | cut -f2
  )
else
  for ((condition_index = 0; condition_index < ${#condition_specs[@]}; condition_index++)); do
    condition_order+=("$condition_index")
  done
fi
printf 'condition_count=%s\n' "${#condition_specs[@]}" >> "$output_dir/sweep_contract.txt"
condition_sequence=0
for condition_index in "${condition_order[@]}"; do
  condition_sequence=$((condition_sequence + 1))
  printf 'execution_condition_%03d=%s\n' "$condition_sequence" "${condition_specs[$condition_index]}" >> "$output_dir/sweep_contract.txt"
done

run_case() {
  local case_id="$1" axis="$2" angle="$3" repetition="$4"
  local case_dir="$output_dir/$case_id"
  local effective_config_path="$case_dir/effective_config.json"
  local recording_dir="$case_dir/recording"
  local operator_recording_dir="$case_dir/operator-recording"
  local operator_log="$case_dir/operator.log" readiness_log="$case_dir/readiness.log" auto_log="$case_dir/auto.log"
  local roll=0 pitch=0 yaw=0 operator_status=0 auto_status=0
  execution_order=$((execution_order + 1))
  case "$axis" in roll) roll="$angle" ;; pitch) pitch="$angle" ;; yaw) yaw="$angle" ;; baseline) ;; esac
  mkdir -p "$case_dir"
  local effective_config_sha256
  effective_config_sha256="$(write_effective_config "$effective_config_path" "$case_id" "$axis" "$angle" "$repetition" "$roll" "$pitch" "$yaw" "$recording_dir" "$operator_recording_dir")"
  printf '%s\n' \
    "case_id=$case_id" "axis=$axis" "angle_deg=$angle" "repetition=$repetition" \
    "session_id=$session_id" "run_id=$run_id" "attempt_id=$case_id" \
    "effective_config_sha256=$effective_config_sha256" \
    "attitude_frame=camera_optical_rdf" "cam_r_optical_roll_deg=$roll" \
    "cam_r_optical_pitch_deg=$pitch" "cam_r_optical_yaw_deg=$yaw" \
    "output_frame=baseline_center_frd[forward,right,down]" \
    "truth_source=Gazebo Target GT (offline calibration/validation only)" \
    "operator_recording_dir=$operator_recording_dir" \
    > "$case_dir/case_contract.txt"

  printf '%s\n' "Starting $case_id: $axis=$angle deg, repetition=$repetition"
  operator_cmd=(
    "$ROOT_DIR/run_operator_view.sh"
    --record-full "$operator_recording_dir"
    --record-wait-for-bbox
  )
  if [[ "$skip_colcon_build" == true || -n "${completed_case:-}" ]]; then operator_cmd+=(--skip-colcon-build); fi
  if [[ "$skip_px4_build" == true ]]; then operator_cmd+=(--skip-px4-build); fi
  if [[ -n "$px4_dir" ]]; then operator_cmd+=(--px4-dir "$px4_dir"); fi
  operator_cmd+=(
    "enable_research_recording:=true" "research_output_dir:=$recording_dir"
    "research_run_id:=$run_id" "research_session_id:=$session_id"
    "research_attempt_id:=$case_id" "research_case_id:=$case_id"
    "research_effective_config_path:=$effective_config_path"
    "research_effective_config_sha256:=$effective_config_sha256"
    "research_repository:=$REPO_ROOT" "research_branch:=$branch"
    "research_commit_sha:=$commit_sha" "research_git_status:=$git_status"
    "research_gazebo_version:=${GAZEBO_MAJOR_VERSION:-8}"
    "research_px4_version:=${PX4_REQUIRED_VERSION:-v1.17.0}"
    "cam_l_optical_roll_deg:=0.0" "cam_l_optical_pitch_deg:=0.0" "cam_l_optical_yaw_deg:=0.0"
    "cam_r_optical_roll_deg:=$roll" "cam_r_optical_pitch_deg:=$pitch" "cam_r_optical_yaw_deg:=$yaw"
    "camera_update_rate_hz:=$camera_update_rate_hz" "yolo_target_hz:=$yolo_target_hz"
    # At the sweep's bounded 2 Hz default, retain overlay publication so the
    # full operator recorder produces its required left/right MP4 artifacts.
    "enable_detection_overlay:=true"
  )
  SIM_CONFIG="$SIM_CONFIG" "${operator_cmd[@]}" > "$operator_log" 2>&1 &
  operator_pid="$!"
  printf '%s\n' "$operator_pid" > "$case_dir/operator.pid"
  if ! wait_for_px4 "$readiness_log"; then
    operator_status=1
    printf '%s\n' "PX4 readiness failed for $case_id; see $readiness_log and $operator_log" >&2
  else
    set +e
    SIM_CONFIG="$SIM_CONFIG" "$ROOT_DIR/run_auto.sh" "${auto_args[@]}" > "$auto_log" 2>&1
    auto_status=$?
    set -e
    if ((auto_status != 0)); then
      operator_status=1
      printf '%s\n' "Auto failed for $case_id (status $auto_status); see $auto_log" >&2
    fi
  fi
  stop_operator
  if ((operator_status == 0)); then
    if [[ ! -f "$recording_dir/RUN_FINALIZED_VALID" || ! -f "$recording_dir/pair_records.csv" ]] \
      || ! search_quiet '"run_valid"[[:space:]]*:[[:space:]]*true' "$recording_dir/run_summary.json" 2>/dev/null; then
      operator_status=1
      printf '%s\n' "Research recording is not valid for $case_id; see $recording_dir" >&2
    fi
  fi
  if ((operator_status == 0)); then
    printf '%s\n' "PASS $case_id"
    printf '%s\t%s\t%s\t%s\t%s\tPASS\t%s\t%s\n' "$execution_order" "$case_id" "$axis" "$angle" "$repetition" "$recording_dir" "$effective_config_sha256" >> "$manifest"
    return 0
  fi
  printf '%s\n' "FAIL $case_id"
  printf '%s\t%s\t%s\t%s\t%s\tFAIL\t%s\t%s\n' "$execution_order" "$case_id" "$axis" "$angle" "$repetition" "$recording_dir" "$effective_config_sha256" >> "$manifest"
  return 1
}

case_index=0 failures=0 execution_order=0 completed_case=""
for condition_index in "${condition_order[@]}"; do
  case_spec="${condition_specs[$condition_index]}"
  IFS='|' read -r axis value repetition <<< "$case_spec"

  case_index=$((case_index + 1))
  if ! run_case "case-$(printf '%03d' "$case_index")-baseline-before" baseline 0 0; then
    failures=$((failures + 1)); [[ "$keep_going" == true ]] || exit 1
  fi
  completed_case=baseline

  case_index=$((case_index + 1))
  case_id="case-$(printf '%03d' "$case_index")-${axis}-$(format_value "$value")-r$(printf '%02d' "$repetition")"
  if ! run_case "$case_id" "$axis" "$value" "$repetition"; then
    failures=$((failures + 1)); [[ "$keep_going" == true ]] || exit 1
  fi
done

if ((${#condition_order[@]} > 0)); then
  case_index=$((case_index + 1))
  if ! run_case "case-$(printf '%03d' "$case_index")-baseline-after" baseline 0 0; then
    failures=$((failures + 1)); [[ "$keep_going" == true ]] || exit 1
  fi
else
  case_index=$((case_index + 1))
  if ! run_case "case-$(printf '%03d' "$case_index")-baseline" baseline 0 0; then
    failures=$((failures + 1)); [[ "$keep_going" == true ]] || exit 1
  fi
fi

printf '%s\n' "Sweep complete: cases=$case_index failures=$failures manifest=$manifest"
if ((failures > 0)); then exit 1; fi
