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
  SWEEP_YOLO_TARGET_HZ SWEEP_ALTITUDES_M SWEEP_LATERAL_BIASES_M
  SWEEP_LATERAL_SWEEP_AMPLITUDE_M SWEEP_LATERAL_SWEEP_PERIOD_S
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
  --axes LIST              Comma-separated axes: roll,pitch,yaw,combined. Default: all.
                           combined applies the same angle to roll, pitch, and
                           yaw simultaneously in one case.
  --values-deg LIST        Comma-separated degrees. Default: -1.0..1.0 in 0.1 steps.
  --repetitions N          Repeat every non-zero attitude case. Default: 1.
  --altitudes-m LIST       Comma-separated takeoff altitudes (m), cycled by case
                           index for elevation-angle diversity. Default: 20,30,40.
                           Skipped for a case whose -- run_auto.sh args already
                           include --altitude-m.
  --lateral-biases-m LIST  Comma-separated constant lateral offsets (m), cycled by
                           case index for off-boresight diversity. Default: -5,0,5.
                           Skipped for a case whose -- run_auto.sh args already
                           include --lateral-bias-m.
  --lateral-sweep-amplitude-m N  Triangle-wave lateral sweep amplitude (m) added on
                           top of the per-case lateral bias during every case's
                           approach, so one flight actively samples a range of
                           off-boresight offsets within the safe-FOV margin
                           instead of holding one fixed value. Default: 10.
                           Skipped for a case whose -- run_auto.sh args already
                           include --lateral-sweep-amplitude-m.
  --lateral-sweep-period-s N     Sweep period in seconds. Default: 24.
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
before each condition and once after the final condition. With the default
axes/values grid every case also cycles through --altitudes-m and
--lateral-biases-m, so case count grows with axes x values x2 + 1. Use
--quick after `--` only for smoke tests; the sweep preserves the configured
500 m FOV-safe target distance unless an explicit --target-distance-m follows.
RViz and rqt_gui stay visible for every case; each case launches its own
window set, so a long default sweep keeps a display busy throughout.

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

axes_csv="${SWEEP_AXES:-roll,pitch,yaw,combined}"
values_csv="${SWEEP_VALUES_DEG:--1.0,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0}"
altitudes_csv="${SWEEP_ALTITUDES_M:-20,30,40}"
lateral_biases_csv="${SWEEP_LATERAL_BIASES_M:--5,0,5}"
lateral_sweep_amplitude_m="${SWEEP_LATERAL_SWEEP_AMPLITUDE_M:-10}"
lateral_sweep_period_s="${SWEEP_LATERAL_SWEEP_PERIOD_S:-24}"
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
    --altitudes-m) altitudes_csv="${2:?missing value for --altitudes-m}"; shift 2 ;;
    --lateral-biases-m) lateral_biases_csv="${2:?missing value for --lateral-biases-m}"; shift 2 ;;
    --lateral-sweep-amplitude-m) lateral_sweep_amplitude_m="${2:?missing value for --lateral-sweep-amplitude-m}"; shift 2 ;;
    --lateral-sweep-period-s) lateral_sweep_period_s="${2:?missing value for --lateral-sweep-period-s}"; shift 2 ;;
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

# run_mission.sh's generic --quick profile stops at 200 m, which is outside
# this stereo rig's validated common FOV and can leave the mission waiting for
# a bbox that cannot reappear. Keep its shorter hover/speed settings, but use
# the Simulation campaign target unless the caller explicitly supplied one.
quick_requested=false
target_distance_requested=false
altitude_requested=false
lateral_bias_requested=false
lateral_sweep_requested=false
for auto_arg in "${auto_args[@]}"; do
  [[ "$auto_arg" == "--quick" ]] && quick_requested=true
  if [[ "$auto_arg" == "--target-distance-m" || "$auto_arg" == --target-distance-m=* ]]; then
    target_distance_requested=true
  fi
  if [[ "$auto_arg" == "--altitude-m" || "$auto_arg" == --altitude-m=* ]]; then
    altitude_requested=true
  fi
  if [[ "$auto_arg" == "--lateral-bias-m" || "$auto_arg" == --lateral-bias-m=* ]]; then
    lateral_bias_requested=true
  fi
  if [[ "$auto_arg" == "--lateral-sweep-amplitude-m" || "$auto_arg" == --lateral-sweep-amplitude-m=* ]]; then
    lateral_sweep_requested=true
  fi
done
if [[ "$quick_requested" == true && "$target_distance_requested" == false ]]; then
  auto_args+=(--target-distance-m "${DRONE_TARGET_DISTANCE_M:-500.0}")
fi

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
    roll|pitch|yaw|combined) ;;
    *) printf 'error: unsupported axis %s; use roll,pitch,yaw,combined\n' "$axis" >&2; exit 2 ;;
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
IFS=',' read -r -a altitudes_m <<< "$altitudes_csv"
if ((${#altitudes_m[@]} == 0)); then
  printf '%s\n' 'error: --altitudes-m must not be empty' >&2
  exit 2
fi
for altitude_m in "${altitudes_m[@]}"; do
  altitude_m="${altitude_m//[[:space:]]/}"
  if ! awk -v value="$altitude_m" 'BEGIN { exit !(value == value && value > 0) }'; then
    printf 'error: invalid --altitudes-m value: %s\n' "$altitude_m" >&2
    exit 2
  fi
done
IFS=',' read -r -a lateral_biases_m <<< "$lateral_biases_csv"
if ((${#lateral_biases_m[@]} == 0)); then
  printf '%s\n' 'error: --lateral-biases-m must not be empty' >&2
  exit 2
fi
for lateral_bias_m in "${lateral_biases_m[@]}"; do
  lateral_bias_m="${lateral_bias_m//[[:space:]]/}"
  if ! awk -v value="$lateral_bias_m" 'BEGIN { exit !(value != "" && value == value && value + 0 == value) }'; then
    printf 'error: invalid --lateral-biases-m value: %s\n' "$lateral_bias_m" >&2
    exit 2
  fi
done
if ! awk -v value="$lateral_sweep_amplitude_m" 'BEGIN { exit !(value == value && value >= 0) }'; then
  printf '%s\n' 'error: --lateral-sweep-amplitude-m must be zero or a positive number' >&2
  exit 2
fi
if ! awk -v value="$lateral_sweep_period_s" 'BEGIN { exit !(value == value && value > 0) }'; then
  printf '%s\n' 'error: --lateral-sweep-period-s must be a positive number' >&2
  exit 2
fi

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
git_status_short="$(git -C "$REPO_ROOT" status --short 2>/dev/null || true)"
git_status=dirty
if [[ -z "$git_status_short" ]]; then git_status=clean; fi
git_remote_url="$(git -C "$REPO_ROOT" config --get remote.origin.url 2>/dev/null || true)"
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
  "git_remote_url=$git_remote_url" \
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
  local geometry_altitude_m="${11}" geometry_lateral_bias_m="${12}"
  local geometry_lateral_sweep_amplitude_m="${13}" geometry_lateral_sweep_period_s="${14}"
  python3 - "$config_path" "$session_id" "$run_id" "$case_id" "$axis" "$angle" "$repetition" "$observation_mode" \
    "$roll" "$pitch" "$yaw" "$seed" "$camera_update_rate_hz" "$yolo_target_hz" \
    "${CAMERA_BASELINE_M:-25.0}" "${CAMERA_HEIGHT_M:-30.0}" "${CAMERA_HORIZONTAL_FOV_RAD:-0.154997710698}" \
    "${CAMERA_IMAGE_WIDTH_PX:-1280}" "${CAMERA_IMAGE_HEIGHT_PX:-720}" \
    "$SIM_CONFIG" "$REPO_ROOT" "$branch" "$commit_sha" "$git_status" \
    "$git_status_short" "$git_remote_url" \
    "$recording_dir" "$operator_recording_dir" "$auto_args_serialized" \
    "$geometry_altitude_m" "$geometry_lateral_bias_m" \
    "$geometry_lateral_sweep_amplitude_m" "$geometry_lateral_sweep_period_s" \
    "${PX4_GZ_MODEL_NAME:-x500_15m}" <<'PY'
import hashlib
import json
import math
import sys
from pathlib import Path

(path, session_id, run_id, case_id, axis, angle, repetition, observation_mode, roll, pitch, yaw,
 seed, camera_hz, yolo_hz, baseline_m, camera_height_m, fov_rad, image_width, image_height,
 sim_config, repo_root, branch, commit_sha, git_status, git_status_short, git_remote_url,
 recording_dir, operator_recording_dir, auto_args_serialized,
 geometry_altitude_m, geometry_lateral_bias_m,
 geometry_lateral_sweep_amplitude_m, geometry_lateral_sweep_period_s,
 px4_gz_model_name) = sys.argv[1:]
intrinsic_contract = {
    "image_width_px": int(image_width),
    "image_height_px": int(image_height),
    "horizontal_fov_rad": float(fov_rad),
    "fx_px": (float(image_width) * 0.5) / math.tan(float(fov_rad) * 0.5),
    "fy_px": (float(image_width) * 0.5) / math.tan(float(fov_rad) * 0.5),
    "cx_px": float(image_width) * 0.5,
    "cy_px": float(image_height) * 0.5,
    "skew_px": 0.0,
    "distortion_model": "none",
    "distortion_coefficients": [],
}
intrinsic_canonical = json.dumps(
    intrinsic_contract, ensure_ascii=False, sort_keys=True, separators=(",", ":")
)
intrinsic_sha256 = hashlib.sha256(intrinsic_canonical.encode("utf-8")).hexdigest()
baseline = float(baseline_m)
rig_contract = {
    "left_nominal_origin_baseline_center_frd_m": [0.0, -baseline * 0.5, 0.0],
    "right_nominal_origin_baseline_center_frd_m": [0.0, baseline * 0.5, 0.0],
    "baseline_vector_baseline_center_frd_m": [0.0, baseline, 0.0],
    "baseline_length_m": baseline,
}
rig_canonical = json.dumps(
    rig_contract, ensure_ascii=False, sort_keys=True, separators=(",", ":")
)
rig_sha256 = hashlib.sha256(rig_canonical.encode("utf-8")).hexdigest()
env_path = Path(sim_config)
env_sha256 = (
    hashlib.sha256(env_path.read_bytes()).hexdigest() if env_path.is_file() else None
)
payload = {
    "schema_version": "simulation_effective_config_v1",
    "hash_algorithm": "sha256",
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
        "application_mode": "right_optical_combined" if axis == "combined" else "right_optical_ofat",
        "observation_mode": observation_mode,
        "trajectory_id": "px4_auto_takeoff_forward_approach_land",
        "run_auto_args": [item for item in auto_args_serialized.split("\x1f") if item],
        "geometry_altitude_m": (
            float(geometry_altitude_m) if geometry_altitude_m != "" else None
        ),
        "geometry_lateral_bias_m": (
            float(geometry_lateral_bias_m) if geometry_lateral_bias_m != "" else None
        ),
        "geometry_lateral_sweep_amplitude_m": (
            float(geometry_lateral_sweep_amplitude_m)
            if geometry_lateral_sweep_amplitude_m != "" else None
        ),
        "geometry_lateral_sweep_period_s": (
            float(geometry_lateral_sweep_period_s)
            if geometry_lateral_sweep_amplitude_m != "" else None
        ),
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
        **intrinsic_contract,
        "baseline_m": float(baseline_m),
        "camera_height_m": float(camera_height_m),
        "camera_update_rate_hz": float(camera_hz),
        "yolo_target_hz": float(yolo_hz),
        "intrinsic_profile_id": "simulation_pinhole_from_horizontal_fov",
        "intrinsic_profile_sha256": intrinsic_sha256,
        "pixel_origin": "top_left",
        "u_axis_direction": "right",
        "v_axis_direction": "down",
        "distorted_or_undistorted": "undistorted_pinhole",
        "crop_policy": "none",
        "resize_policy": "none",
        "letterbox_policy": "none",
    },
    "rig": {
        "profile_id": "simulation_symmetric_baseline",
        "profile_sha256": rig_sha256,
        **rig_contract,
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
        "model": px4_gz_model_name,
        "link": "vehicle_reference_link",
        "runtime_feature_allowed": False,
    },
    "provenance": {
        "simulation_env_path": sim_config,
        "repository": repo_root,
        "git_branch": branch,
        "git_head_sha": commit_sha,
        "git_status": git_status,
        "git_dirty": git_status != "clean",
        "git_status_short": git_status_short.splitlines(),
        "git_remote_url": git_remote_url,
        "simulation_env_sha256": env_sha256,
    },
    "outputs": {
        "recording_dir": recording_dir,
        "operator_recording_dir": operator_recording_dir,
        "effective_config_path": str(Path(path).resolve()),
        "sample_jsonl": str(Path(recording_dir) / "pair_records.jsonl"),
        "sample_csv": str(Path(recording_dir) / "pair_records.csv"),
        "metadata": str(Path(recording_dir) / "run_metadata.json"),
        "summary": str(Path(recording_dir) / "run_summary.json"),
        "case_result": str(Path(path).parent / "case_result.json"),
        "data_validation": str(Path(path).parent / "data_validation.json"),
    },
}
canonical = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
payload["effective_config_sha256"] = hashlib.sha256(canonical.encode("utf-8")).hexdigest()
Path(path).write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
print(payload["effective_config_sha256"])
PY
}

operator_pid=""
auto_pid=""
stop_auto() {
  local pid="$auto_pid"
  auto_pid=""
  [[ -n "$pid" ]] || return 0
  if kill -0 "$pid" >/dev/null 2>&1; then
    kill -TERM -- "-$pid" >/dev/null 2>&1 || kill -TERM "$pid" >/dev/null 2>&1 || true
    for _ in {1..30}; do
      kill -0 "$pid" >/dev/null 2>&1 || break
      sleep 1
    done
    kill -KILL -- "-$pid" >/dev/null 2>&1 || kill -KILL "$pid" >/dev/null 2>&1 || true
  fi
  wait "$pid" >/dev/null 2>&1 || true
}
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
cleanup_case_processes() {
  stop_auto
  stop_operator
}
trap cleanup_case_processes EXIT INT TERM

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
  local case_id="$1" axis="$2" angle="$3" repetition="$4" geometry_index="$5"
  local case_dir="$output_dir/$case_id"
  local effective_config_path="$case_dir/effective_config.json"
  local recording_dir="$case_dir/recording"
  local operator_recording_dir="$case_dir/operator-recording"
  local operator_log="$case_dir/operator.log" readiness_log="$case_dir/readiness.log" auto_log="$case_dir/auto.log"
  local validation_log="$case_dir/data_validation.log" validation_report="$case_dir/data_validation.json"
  local roll=0 pitch=0 yaw=0 operator_status=0 auto_status=0
  local failure_stage="" failure_reason=""
  local start_time_utc start_monotonic_ns end_time_utc end_monotonic_ns
  local geometry_altitude_m="" geometry_lateral_bias_m="" geometry_lateral_sweep_amplitude_m=""
  start_time_utc="$(date -u +%Y-%m-%dT%H:%M:%S.%NZ)"
  start_monotonic_ns="$(python3 -c 'import time; print(time.monotonic_ns())')"
  execution_order=$((execution_order + 1))
  case "$axis" in
    roll) roll="$angle" ;;
    pitch) pitch="$angle" ;;
    yaw) yaw="$angle" ;;
    combined) roll="$angle"; pitch="$angle"; yaw="$angle" ;;
    baseline) ;;
  esac
  if [[ "$altitude_requested" == false ]]; then
    geometry_altitude_m="${altitudes_m[$(( geometry_index % ${#altitudes_m[@]} ))]}"
  fi
  if [[ "$lateral_bias_requested" == false ]]; then
    geometry_lateral_bias_m="${lateral_biases_m[$(( geometry_index % ${#lateral_biases_m[@]} ))]}"
  fi
  if [[ "$lateral_sweep_requested" == false ]] \
    && awk -v value="$lateral_sweep_amplitude_m" 'BEGIN { exit !(value > 0) }'; then
    geometry_lateral_sweep_amplitude_m="$lateral_sweep_amplitude_m"
  fi
  local case_auto_args=("${auto_args[@]}")
  [[ -n "$geometry_altitude_m" ]] && case_auto_args+=(--altitude-m "$geometry_altitude_m")
  [[ -n "$geometry_lateral_bias_m" ]] && case_auto_args+=(--lateral-bias-m "$geometry_lateral_bias_m")
  if [[ -n "$geometry_lateral_sweep_amplitude_m" ]]; then
    case_auto_args+=(
      --lateral-sweep-amplitude-m "$geometry_lateral_sweep_amplitude_m"
      --lateral-sweep-period-s "$lateral_sweep_period_s"
    )
  fi
  mkdir -p "$case_dir"
  local effective_config_sha256
  effective_config_sha256="$(write_effective_config "$effective_config_path" "$case_id" "$axis" "$angle" "$repetition" "$roll" "$pitch" "$yaw" "$recording_dir" "$operator_recording_dir" "$geometry_altitude_m" "$geometry_lateral_bias_m" "$geometry_lateral_sweep_amplitude_m" "$lateral_sweep_period_s")"
  printf '%s\n' \
    "case_id=$case_id" "axis=$axis" "angle_deg=$angle" "repetition=$repetition" \
    "session_id=$session_id" "run_id=$run_id" "attempt_id=$case_id" \
    "effective_config_sha256=$effective_config_sha256" \
    "attitude_frame=camera_optical_rdf" "cam_r_optical_roll_deg=$roll" \
    "cam_r_optical_pitch_deg=$pitch" "cam_r_optical_yaw_deg=$yaw" \
    "output_frame=baseline_center_frd[forward,right,down]" \
    "truth_source=Gazebo Target GT (offline calibration/validation only)" \
    "operator_recording_dir=$operator_recording_dir" \
    "geometry_altitude_m=$geometry_altitude_m" \
    "geometry_lateral_bias_m=$geometry_lateral_bias_m" \
    "geometry_lateral_sweep_amplitude_m=$geometry_lateral_sweep_amplitude_m" \
    "geometry_lateral_sweep_period_s=$lateral_sweep_period_s" \
    > "$case_dir/case_contract.txt"

  printf '%s\n' "Starting $case_id: $axis=$angle deg, repetition=$repetition, altitude_m=${geometry_altitude_m:-inherited}, lateral_bias_m=${geometry_lateral_bias_m:-inherited}, lateral_sweep_amplitude_m=${geometry_lateral_sweep_amplitude_m:-0}"
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
    "mission_profile:=takeoff_forward_land"
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
    failure_stage="px4_readiness"
    failure_reason="PX4 heartbeat was not ready before the configured timeout"
    printf '%s\n' "PX4 readiness failed for $case_id; see $readiness_log and $operator_log" >&2
  else
    set +e
    setsid env SIM_CONFIG="$SIM_CONFIG" "$ROOT_DIR/run_auto.sh" "${case_auto_args[@]}" > "$auto_log" 2>&1 &
    auto_pid="$!"
    while kill -0 "$auto_pid" >/dev/null 2>&1; do
      if ! kill -0 "$operator_pid" >/dev/null 2>&1; then
        operator_status=1
        auto_status=125
        failure_stage="operator_runtime"
        failure_reason="operator stack exited while Auto mission was running"
        printf '%s\n' "Operator stack exited during Auto for $case_id; see $operator_log" >&2
        stop_auto
        break
      fi
      sleep 1
    done
    if [[ -n "$auto_pid" ]]; then
      wait "$auto_pid"
      auto_status=$?
      auto_pid=""
    fi
    set -e
    if ((auto_status != 0)); then
      operator_status=1
      if [[ -z "$failure_stage" ]]; then
        failure_stage="auto_mission"
        failure_reason="run_auto.sh exited with status $auto_status"
      fi
      printf '%s\n' "Auto failed for $case_id (status $auto_status); see $auto_log" >&2
    fi
  fi
  stop_operator
  if ((operator_status == 0)); then
    if [[ ! -f "$recording_dir/RUN_FINALIZED_VALID" || ! -f "$recording_dir/pair_records.csv" ]] \
      || ! search_quiet '"run_valid"[[:space:]]*:[[:space:]]*true' "$recording_dir/run_summary.json" 2>/dev/null; then
      operator_status=1
      failure_stage="research_recording"
      failure_reason="research recorder did not produce a valid finalized CSV"
      printf '%s\n' "Research recording is not valid for $case_id; see $recording_dir" >&2
    fi
  fi
  if ((operator_status == 0)); then
    if ! python3 "$ROOT_DIR/scripts/validate_attitude_sweep_output.py" \
      --case-dir "$case_dir" --report "$validation_report" > "$validation_log" 2>&1; then
      operator_status=1
      failure_stage="data_validation"
      failure_reason="calibration-transfer data validation failed"
      printf '%s\n' "Data validation failed for $case_id; see $validation_log" >&2
    fi
  fi
  end_time_utc="$(date -u +%Y-%m-%dT%H:%M:%S.%NZ)"
  end_monotonic_ns="$(python3 -c 'import time; print(time.monotonic_ns())')"
  python3 - "$case_dir/case_result.json" "$case_id" "$session_id" "$run_id" \
    "$effective_config_sha256" "$start_time_utc" "$end_time_utc" \
    "$start_monotonic_ns" "$end_monotonic_ns" "$auto_status" "$operator_status" \
    "$failure_stage" "$failure_reason" "$validation_report" <<'PY'
import json
import sys
from pathlib import Path

(path, case_id, session_id, run_id, config_hash, start_utc, end_utc,
 start_ns, end_ns, auto_status, status, failure_stage, failure_reason,
 validation_report) = sys.argv[1:]
start_ns = int(start_ns)
end_ns = int(end_ns)
payload = {
    "schema_version": "simulation_sweep_case_result_v1",
    "session_id": session_id,
    "run_id": run_id,
    "attempt_id": case_id,
    "case_id": case_id,
    "effective_config_sha256": config_hash,
    "status": "PASS" if int(status) == 0 else "FAIL",
    "failure_stage": failure_stage,
    "failure_reason": failure_reason,
    "auto_exit_status": int(auto_status),
    "start_time_utc": start_utc,
    "end_time_utc": end_utc,
    "start_monotonic_ns": start_ns,
    "end_monotonic_ns": end_ns,
    "actual_duration_s": (end_ns - start_ns) / 1_000_000_000.0,
    "data_validation_path": str(Path(validation_report).resolve()),
}
Path(path).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY
  if ((operator_status == 0)); then
    printf '%s\n' "PASS $case_id"
    printf '%s\t%s\t%s\t%s\t%s\tPASS\t%s\t%s\n' "$execution_order" "$case_id" "$axis" "$angle" "$repetition" "$recording_dir" "$effective_config_sha256" >> "$manifest"
    return 0
  fi
  printf '%s\n' "FAIL $case_id"
  printf '%s\t%s\t%s\t%s\t%s\tFAIL\t%s\t%s\n' "$execution_order" "$case_id" "$axis" "$angle" "$repetition" "$recording_dir" "$effective_config_sha256" >> "$manifest"
  return 1
}

# geometry_index is shared by a baseline-before and its paired attitude case
# (one increment per condition, not per case), so the two share the same
# altitude_m/lateral_bias_m and the baseline stays a same-geometry zero
# reference for isolating the attitude-only effect. It advances once more
# between condition pairs and for each closing baseline.
case_index=0 pair_index=0 failures=0 execution_order=0 completed_case=""
for condition_index in "${condition_order[@]}"; do
  case_spec="${condition_specs[$condition_index]}"
  IFS='|' read -r axis value repetition <<< "$case_spec"

  case_index=$((case_index + 1))
  if ! run_case "case-$(printf '%03d' "$case_index")-baseline-before" baseline 0 0 "$pair_index"; then
    failures=$((failures + 1)); [[ "$keep_going" == true ]] || exit 1
  fi
  completed_case=baseline

  case_index=$((case_index + 1))
  case_id="case-$(printf '%03d' "$case_index")-${axis}-$(format_value "$value")-r$(printf '%02d' "$repetition")"
  if ! run_case "$case_id" "$axis" "$value" "$repetition" "$pair_index"; then
    failures=$((failures + 1)); [[ "$keep_going" == true ]] || exit 1
  fi
  pair_index=$((pair_index + 1))
done

if ((${#condition_order[@]} > 0)); then
  case_index=$((case_index + 1))
  if ! run_case "case-$(printf '%03d' "$case_index")-baseline-after" baseline 0 0 "$pair_index"; then
    failures=$((failures + 1)); [[ "$keep_going" == true ]] || exit 1
  fi
else
  case_index=$((case_index + 1))
  if ! run_case "case-$(printf '%03d' "$case_index")-baseline" baseline 0 0 "$pair_index"; then
    failures=$((failures + 1)); [[ "$keep_going" == true ]] || exit 1
  fi
fi

if ((failures == 0)); then
  if python3 "$ROOT_DIR/scripts/validate_attitude_sweep_output.py" \
    --sweep-dir "$output_dir" --report "$output_dir/sweep_validation.json" \
    > "$output_dir/sweep_validation.log" 2>&1; then
    touch "$output_dir/SWEEP_FINALIZED_VALID"
  else
    failures=$((failures + 1))
    touch "$output_dir/SWEEP_FINALIZED_INVALID"
    printf '%s\n' "Sweep data validation failed; see $output_dir/sweep_validation.log" >&2
  fi
else
  touch "$output_dir/SWEEP_FINALIZED_INVALID"
fi
printf '%s\n' "Sweep complete: cases=$case_index failures=$failures manifest=$manifest"
if ((failures > 0)); then exit 1; fi
