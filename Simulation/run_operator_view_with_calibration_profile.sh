#!/usr/bin/env bash
set -euo pipefail

CALLER_CWD="$(pwd)"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

usage() {
  cat <<'USAGE'
Usage:
  ./run_operator_view_with_calibration_profile.sh [options] [-- run_operator_view args...]

Starts operator view with an exported estimated-to-truth StereoDistance runtime profile.

Options:
  --profile-json FILE    Runtime profile JSON. Default: newest active_runtime_profile.json
                         or sim3_runtime_profile.json under results/.
  --package-dir DIR      Deployment package directory containing active_runtime_profile.json.
	  --selector-json FILE   Runtime profile label selector manifest.
	  --deployment-label L   Explicit deployment label for --selector-json.
	  --scenario-label L     Scenario label for --selector-json.
	  --scenario-name NAME   Scenario name for --selector-json.
	  --observation-fusion-manifest FILE
	                         Observation-fusion runtime manifest JSON.
	  --observation-fusion-package DIR
	                         Package directory containing observation_fusion_runtime_manifest.json.
	  --observation-fusion-tier TIER
	                         Tier inside the observation-fusion manifest.
	  --disparity-residual-profile-json FILE
	                         Runtime residual disparity profile JSON.
	  --disparity-residual-decision-json FILE
	                         Gated operator telemetry selector decision JSON.
	  --disparity-residual-selector-json FILE
	                         Residual disparity profile label selector manifest.
	  --disparity-residual-feature-selector-json FILE
	                         Residual disparity feature selector model JSON.
	  --disparity-residual-feature-csv FILE
	                         Feature CSV row for --disparity-residual-feature-selector-json.
	  --disparity-residual-feature-json FILE
	                         Feature JSON for --disparity-residual-feature-selector-json.
	  --disparity-residual-feature-row-index N
	                         Feature CSV row index. Default: 0.
	  --disparity-residual-feature-catalog-json FILE
	                         Residual profile catalog JSON for feature selector.
	  --node-disparity-residual-feature-selector-json FILE
	                         Enable StereoDistanceNode runtime residual feature selection.
	  --node-disparity-residual-feature-catalog-json FILE
	                         Residual profile catalog JSON for node-side feature selection.
	  --node-disparity-residual-feature-window-size N
	                         Rolling runtime feature window size for node-side feature selection.
	  --node-disparity-residual-feature-min-samples N
	                         Minimum samples before node-side feature selection is allowed.
	  --results-root DIR     Search root for latest profile. Default: results
  --print-only           Print the resolved command without running it.
  --help, -h             Show this help.

Examples:
  ./run_operator_view_with_calibration_profile.sh
  ./run_operator_view_with_calibration_profile.sh --profile-json results/estimated_to_truth_calibration_exp/sim3_runtime_profile.json
  ./run_operator_view_with_calibration_profile.sh --package-dir results/rtk_stereo_acceptance_matrix_summary/deployment_package
  ./run_operator_view_with_calibration_profile.sh --selector-json results/rtk_stereo_runtime_observation_stress_EXP_20260617/runtime_profile_label_selector.json --deployment-label runtime_observation_stress
  ./run_operator_view_with_calibration_profile.sh -- --raw-view --no-px4
USAGE
}

profile_json=""
package_dir=""
selector_json=""
deployment_label=""
scenario_label=""
scenario_name=""
observation_fusion_manifest=""
observation_fusion_package=""
observation_fusion_tier=""
disparity_residual_profile_json=""
disparity_residual_decision_json=""
disparity_residual_selector_json=""
disparity_residual_feature_selector_json=""
disparity_residual_feature_csv=""
disparity_residual_feature_json=""
disparity_residual_feature_row_index="0"
disparity_residual_feature_catalog_json=""
node_disparity_residual_feature_selector_json=""
node_disparity_residual_feature_catalog_json=""
node_disparity_residual_feature_window_size=""
node_disparity_residual_feature_min_samples=""
results_root="$ROOT_DIR/results"
print_only=false
operator_args=()

while (($#)); do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --profile-json)
      profile_json="${2:?missing value for --profile-json}"
      shift 2
      ;;
    --package-dir)
      package_dir="${2:?missing value for --package-dir}"
      shift 2
      ;;
    --selector-json)
      selector_json="${2:?missing value for --selector-json}"
      shift 2
      ;;
    --deployment-label)
      deployment_label="${2:?missing value for --deployment-label}"
      shift 2
      ;;
    --scenario-label)
      scenario_label="${2:?missing value for --scenario-label}"
      shift 2
      ;;
	    --scenario-name)
	      scenario_name="${2:?missing value for --scenario-name}"
	      shift 2
	      ;;
	    --observation-fusion-manifest)
	      observation_fusion_manifest="${2:?missing value for --observation-fusion-manifest}"
	      shift 2
	      ;;
	    --observation-fusion-package)
	      observation_fusion_package="${2:?missing value for --observation-fusion-package}"
	      shift 2
	      ;;
	    --observation-fusion-tier)
	      observation_fusion_tier="${2:?missing value for --observation-fusion-tier}"
	      shift 2
	      ;;
	    --disparity-residual-profile-json)
	      disparity_residual_profile_json="${2:?missing value for --disparity-residual-profile-json}"
	      shift 2
	      ;;
	    --disparity-residual-decision-json)
	      disparity_residual_decision_json="${2:?missing value for --disparity-residual-decision-json}"
	      shift 2
	      ;;
	    --disparity-residual-selector-json)
	      disparity_residual_selector_json="${2:?missing value for --disparity-residual-selector-json}"
	      shift 2
	      ;;
	    --disparity-residual-feature-selector-json)
	      disparity_residual_feature_selector_json="${2:?missing value for --disparity-residual-feature-selector-json}"
	      shift 2
	      ;;
	    --disparity-residual-feature-csv)
	      disparity_residual_feature_csv="${2:?missing value for --disparity-residual-feature-csv}"
	      shift 2
	      ;;
	    --disparity-residual-feature-json)
	      disparity_residual_feature_json="${2:?missing value for --disparity-residual-feature-json}"
	      shift 2
	      ;;
	    --disparity-residual-feature-row-index)
	      disparity_residual_feature_row_index="${2:?missing value for --disparity-residual-feature-row-index}"
	      shift 2
	      ;;
	    --disparity-residual-feature-catalog-json)
	      disparity_residual_feature_catalog_json="${2:?missing value for --disparity-residual-feature-catalog-json}"
	      shift 2
	      ;;
	    --node-disparity-residual-feature-selector-json)
	      node_disparity_residual_feature_selector_json="${2:?missing value for --node-disparity-residual-feature-selector-json}"
	      shift 2
	      ;;
	    --node-disparity-residual-feature-catalog-json)
	      node_disparity_residual_feature_catalog_json="${2:?missing value for --node-disparity-residual-feature-catalog-json}"
	      shift 2
	      ;;
	    --node-disparity-residual-feature-window-size)
	      node_disparity_residual_feature_window_size="${2:?missing value for --node-disparity-residual-feature-window-size}"
	      shift 2
	      ;;
	    --node-disparity-residual-feature-min-samples)
	      node_disparity_residual_feature_min_samples="${2:?missing value for --node-disparity-residual-feature-min-samples}"
	      shift 2
	      ;;
	    --results-root)
	      results_root="${2:?missing value for --results-root}"
	      shift 2
      ;;
    --print-only)
      print_only=true
      shift
      ;;
    --)
      shift
      operator_args+=("$@")
      break
      ;;
    *)
      operator_args+=("$1")
      shift
      ;;
  esac
done

source_count=0
[[ -n "$profile_json" ]] && source_count=$((source_count + 1))
[[ -n "$package_dir" ]] && source_count=$((source_count + 1))
[[ -n "$selector_json" ]] && source_count=$((source_count + 1))
if ((source_count > 1)); then
  echo "error: pass only one of --profile-json, --package-dir, or --selector-json" >&2
  exit 2
fi

if [[ "$results_root" != /* ]]; then
  if [[ -d "$results_root" ]]; then
    results_root="$ROOT_DIR/$results_root"
  else
    results_root="$CALLER_CWD/$results_root"
  fi
fi

if [[ -n "$package_dir" && "$package_dir" != /* ]]; then
  if [[ -d "$package_dir" ]]; then
    package_dir="$ROOT_DIR/$package_dir"
  elif [[ -d "$CALLER_CWD/$package_dir" ]]; then
    package_dir="$CALLER_CWD/$package_dir"
  fi
fi

if [[ -n "$profile_json" && "$profile_json" != /* ]]; then
  if [[ -f "$profile_json" ]]; then
    profile_json="$ROOT_DIR/$profile_json"
  elif [[ -f "$CALLER_CWD/$profile_json" ]]; then
    profile_json="$CALLER_CWD/$profile_json"
  fi
fi

if [[ -n "$selector_json" && "$selector_json" != /* ]]; then
  if [[ -f "$selector_json" ]]; then
    selector_json="$ROOT_DIR/$selector_json"
  elif [[ -f "$CALLER_CWD/$selector_json" ]]; then
    selector_json="$CALLER_CWD/$selector_json"
  fi
fi

if [[ -n "$observation_fusion_package" && "$observation_fusion_package" != /* ]]; then
  if [[ -d "$observation_fusion_package" ]]; then
    observation_fusion_package="$ROOT_DIR/$observation_fusion_package"
  elif [[ -d "$CALLER_CWD/$observation_fusion_package" ]]; then
    observation_fusion_package="$CALLER_CWD/$observation_fusion_package"
  fi
fi

if [[ -n "$observation_fusion_manifest" && "$observation_fusion_manifest" != /* ]]; then
  if [[ -f "$observation_fusion_manifest" ]]; then
    observation_fusion_manifest="$ROOT_DIR/$observation_fusion_manifest"
  elif [[ -f "$CALLER_CWD/$observation_fusion_manifest" ]]; then
    observation_fusion_manifest="$CALLER_CWD/$observation_fusion_manifest"
  fi
fi

if [[ -n "$disparity_residual_profile_json" && "$disparity_residual_profile_json" != /* ]]; then
  if [[ -f "$disparity_residual_profile_json" ]]; then
    disparity_residual_profile_json="$ROOT_DIR/$disparity_residual_profile_json"
  elif [[ -f "$CALLER_CWD/$disparity_residual_profile_json" ]]; then
    disparity_residual_profile_json="$CALLER_CWD/$disparity_residual_profile_json"
  fi
fi

if [[ -n "$disparity_residual_decision_json" && "$disparity_residual_decision_json" != /* ]]; then
  if [[ -f "$disparity_residual_decision_json" ]]; then
    disparity_residual_decision_json="$ROOT_DIR/$disparity_residual_decision_json"
  elif [[ -f "$CALLER_CWD/$disparity_residual_decision_json" ]]; then
    disparity_residual_decision_json="$CALLER_CWD/$disparity_residual_decision_json"
  fi
fi

if [[ -n "$disparity_residual_selector_json" && "$disparity_residual_selector_json" != /* ]]; then
  if [[ -f "$disparity_residual_selector_json" ]]; then
    disparity_residual_selector_json="$ROOT_DIR/$disparity_residual_selector_json"
  elif [[ -f "$CALLER_CWD/$disparity_residual_selector_json" ]]; then
    disparity_residual_selector_json="$CALLER_CWD/$disparity_residual_selector_json"
  fi
fi

if [[ -n "$disparity_residual_feature_selector_json" && "$disparity_residual_feature_selector_json" != /* ]]; then
  if [[ -f "$disparity_residual_feature_selector_json" ]]; then
    disparity_residual_feature_selector_json="$ROOT_DIR/$disparity_residual_feature_selector_json"
  elif [[ -f "$CALLER_CWD/$disparity_residual_feature_selector_json" ]]; then
    disparity_residual_feature_selector_json="$CALLER_CWD/$disparity_residual_feature_selector_json"
  fi
fi

if [[ -n "$disparity_residual_feature_csv" && "$disparity_residual_feature_csv" != /* ]]; then
  if [[ -f "$disparity_residual_feature_csv" ]]; then
    disparity_residual_feature_csv="$ROOT_DIR/$disparity_residual_feature_csv"
  elif [[ -f "$CALLER_CWD/$disparity_residual_feature_csv" ]]; then
    disparity_residual_feature_csv="$CALLER_CWD/$disparity_residual_feature_csv"
  fi
fi

if [[ -n "$disparity_residual_feature_json" && "$disparity_residual_feature_json" != /* ]]; then
  if [[ -f "$disparity_residual_feature_json" ]]; then
    disparity_residual_feature_json="$ROOT_DIR/$disparity_residual_feature_json"
  elif [[ -f "$CALLER_CWD/$disparity_residual_feature_json" ]]; then
    disparity_residual_feature_json="$CALLER_CWD/$disparity_residual_feature_json"
  fi
fi

if [[ -n "$disparity_residual_feature_catalog_json" && "$disparity_residual_feature_catalog_json" != /* ]]; then
  if [[ -f "$disparity_residual_feature_catalog_json" ]]; then
    disparity_residual_feature_catalog_json="$ROOT_DIR/$disparity_residual_feature_catalog_json"
  elif [[ -f "$CALLER_CWD/$disparity_residual_feature_catalog_json" ]]; then
    disparity_residual_feature_catalog_json="$CALLER_CWD/$disparity_residual_feature_catalog_json"
  fi
fi

if [[ -n "$node_disparity_residual_feature_selector_json" && "$node_disparity_residual_feature_selector_json" != /* ]]; then
  if [[ -f "$node_disparity_residual_feature_selector_json" ]]; then
    node_disparity_residual_feature_selector_json="$ROOT_DIR/$node_disparity_residual_feature_selector_json"
  elif [[ -f "$CALLER_CWD/$node_disparity_residual_feature_selector_json" ]]; then
    node_disparity_residual_feature_selector_json="$CALLER_CWD/$node_disparity_residual_feature_selector_json"
  fi
fi

if [[ -n "$node_disparity_residual_feature_catalog_json" && "$node_disparity_residual_feature_catalog_json" != /* ]]; then
  if [[ -f "$node_disparity_residual_feature_catalog_json" ]]; then
    node_disparity_residual_feature_catalog_json="$ROOT_DIR/$node_disparity_residual_feature_catalog_json"
  elif [[ -f "$CALLER_CWD/$node_disparity_residual_feature_catalog_json" ]]; then
    node_disparity_residual_feature_catalog_json="$CALLER_CWD/$node_disparity_residual_feature_catalog_json"
  fi
fi

selected_label=""
if [[ -n "$selector_json" ]]; then
  if [[ ! -f "$selector_json" ]]; then
    echo "error: runtime profile selector not found: $selector_json" >&2
    exit 2
  fi

  selector_python="$ROOT_DIR/.venv/bin/python"
  if [[ ! -x "$selector_python" ]]; then
    selector_python="python3"
  fi
  selector_output="$(mktemp "${TMPDIR:-/tmp}/stereodistance_selector.XXXXXX.json")"
  selector_cmd=(
    "$selector_python"
    "$ROOT_DIR/scripts/select_stereodistance_runtime_profile_by_label.py"
    --selector-json
    "$selector_json"
    --output-json
    "$selector_output"
  )
  [[ -n "$deployment_label" ]] && selector_cmd+=(--deployment-label "$deployment_label")
  [[ -n "$scenario_label" ]] && selector_cmd+=(--scenario-label "$scenario_label")
  [[ -n "$scenario_name" ]] && selector_cmd+=(--scenario-name "$scenario_name")
  selector_stdout="$("${selector_cmd[@]}")"
  package_dir="$("$selector_python" -c "import json,sys; p=json.load(open(sys.argv[1], encoding='utf-8')); print(p.get('selected_package_dir',''))" "$selector_output")"
  profile_json="$("$selector_python" -c "import json,sys; p=json.load(open(sys.argv[1], encoding='utf-8')); print(p.get('selected_profile',''))" "$selector_output")"
  selected_label="$("$selector_python" -c "import json,sys; p=json.load(open(sys.argv[1], encoding='utf-8')); print(p.get('selected_label',''))" "$selector_output")"
  rm -f "$selector_output"
  if [[ -z "$profile_json" ]]; then
    echo "error: selector did not resolve a runtime profile: $selector_json" >&2
    echo "$selector_stdout" >&2
    exit 2
  fi
fi

if [[ -n "$disparity_residual_selector_json" ]]; then
  if [[ -n "$disparity_residual_profile_json" || -n "$disparity_residual_decision_json" || -n "$disparity_residual_feature_selector_json" ]]; then
    echo "error: pass only one residual source: profile, decision, label selector, or feature selector" >&2
    exit 2
  fi
  if [[ ! -f "$disparity_residual_selector_json" ]]; then
    echo "error: disparity residual selector not found: $disparity_residual_selector_json" >&2
    exit 2
  fi
  selector_python="$ROOT_DIR/.venv/bin/python"
  if [[ ! -x "$selector_python" ]]; then
    selector_python="python3"
  fi
  residual_selector_output="$(mktemp "${TMPDIR:-/tmp}/stereodistance_residual_selector.XXXXXX.json")"
  residual_selector_cmd=(
    "$selector_python"
    "$ROOT_DIR/scripts/select_disparity_residual_profile_by_label.py"
    --selector-json
    "$disparity_residual_selector_json"
    --output-json
    "$residual_selector_output"
  )
  [[ -n "$deployment_label" ]] && residual_selector_cmd+=(--deployment-label "$deployment_label")
  [[ -n "$scenario_label" ]] && residual_selector_cmd+=(--scenario-label "$scenario_label")
  [[ -n "$scenario_name" ]] && residual_selector_cmd+=(--scenario-name "$scenario_name")
  residual_selector_stdout="$("${residual_selector_cmd[@]}")"
  disparity_residual_profile_json="$("$selector_python" -c "import json,sys; p=json.load(open(sys.argv[1], encoding='utf-8')); print(p.get('selected_profile',''))" "$residual_selector_output")"
  residual_selected_label="$("$selector_python" -c "import json,sys; p=json.load(open(sys.argv[1], encoding='utf-8')); print(p.get('selected_label',''))" "$residual_selector_output")"
  rm -f "$residual_selector_output"
  if [[ -z "$disparity_residual_profile_json" ]]; then
    echo "error: residual selector did not resolve a profile: $disparity_residual_selector_json" >&2
    echo "$residual_selector_stdout" >&2
    exit 2
  fi
fi

if [[ -n "$disparity_residual_feature_selector_json" ]]; then
  if [[ -n "$disparity_residual_profile_json" || -n "$disparity_residual_decision_json" || -n "$disparity_residual_selector_json" ]]; then
    echo "error: pass only one residual source: profile, decision, label selector, or feature selector" >&2
    exit 2
  fi
  if [[ ! -f "$disparity_residual_feature_selector_json" ]]; then
    echo "error: disparity residual feature selector not found: $disparity_residual_feature_selector_json" >&2
    exit 2
  fi
  if [[ -z "$disparity_residual_feature_catalog_json" || ! -f "$disparity_residual_feature_catalog_json" ]]; then
    echo "error: --disparity-residual-feature-catalog-json is required for feature selector" >&2
    exit 2
  fi
  if [[ -n "$disparity_residual_feature_csv" && -n "$disparity_residual_feature_json" ]]; then
    echo "error: pass only one of --disparity-residual-feature-csv or --disparity-residual-feature-json" >&2
    exit 2
  fi
  if [[ -z "$disparity_residual_feature_csv" && -z "$disparity_residual_feature_json" ]]; then
    echo "error: feature selector requires --disparity-residual-feature-csv or --disparity-residual-feature-json" >&2
    exit 2
  fi
  selector_python="$ROOT_DIR/.venv/bin/python"
  if [[ ! -x "$selector_python" ]]; then
    selector_python="python3"
  fi
  residual_feature_selector_output="$(mktemp "${TMPDIR:-/tmp}/stereodistance_residual_feature_selector.XXXXXX.json")"
  residual_feature_selector_cmd=(
    "$selector_python"
    "$ROOT_DIR/scripts/select_disparity_residual_profile_by_features.py"
    --selector-model-json
    "$disparity_residual_feature_selector_json"
    --catalog-json
    "$disparity_residual_feature_catalog_json"
    --output-json
    "$residual_feature_selector_output"
  )
  if [[ -n "$disparity_residual_feature_csv" ]]; then
    residual_feature_selector_cmd+=(--feature-csv "$disparity_residual_feature_csv" --csv-row-index "$disparity_residual_feature_row_index")
  else
    residual_feature_selector_cmd+=(--feature-json "$disparity_residual_feature_json")
  fi
  residual_feature_selector_stdout="$("${residual_feature_selector_cmd[@]}")"
  disparity_residual_profile_json="$("$selector_python" -c "import json,sys; p=json.load(open(sys.argv[1], encoding='utf-8')); print(p.get('selected_profile',''))" "$residual_feature_selector_output")"
  residual_feature_selected_label="$("$selector_python" -c "import json,sys; p=json.load(open(sys.argv[1], encoding='utf-8')); print(p.get('selected_label',''))" "$residual_feature_selector_output")"
  rm -f "$residual_feature_selector_output"
  if [[ -z "$disparity_residual_profile_json" ]]; then
    echo "error: residual feature selector did not resolve a profile: $disparity_residual_feature_selector_json" >&2
    echo "$residual_feature_selector_stdout" >&2
    exit 2
  fi
fi

if [[ -n "$disparity_residual_decision_json" ]]; then
  if [[ -n "$disparity_residual_profile_json" || -n "$disparity_residual_selector_json" || -n "$disparity_residual_feature_selector_json" ]]; then
    echo "error: pass only one residual source: profile, decision, label selector, or feature selector" >&2
    exit 2
  fi
  if [[ ! -f "$disparity_residual_decision_json" ]]; then
    echo "error: disparity residual decision not found: $disparity_residual_decision_json" >&2
    exit 2
  fi
  selector_python="$ROOT_DIR/.venv/bin/python"
  if [[ ! -x "$selector_python" ]]; then
    selector_python="python3"
  fi
  decision_allowed="$("$selector_python" -c "import json,sys; p=json.load(open(sys.argv[1], encoding='utf-8')); print('true' if p.get('apply_residual_allowed') is True else 'false')" "$disparity_residual_decision_json")"
  if [[ "$decision_allowed" != "true" ]]; then
    echo "error: residual decision does not allow applying a residual profile: $disparity_residual_decision_json" >&2
    exit 2
  fi
  disparity_residual_profile_json="$("$selector_python" -c "import json,sys; p=json.load(open(sys.argv[1], encoding='utf-8')); print(p.get('recommended_residual_profile',''))" "$disparity_residual_decision_json")"
  residual_decision_selected_label="$("$selector_python" -c "import json,sys; p=json.load(open(sys.argv[1], encoding='utf-8')); print(p.get('recommended_residual_label',''))" "$disparity_residual_decision_json")"
  if [[ -z "$disparity_residual_profile_json" ]]; then
    echo "error: residual decision did not provide recommended_residual_profile: $disparity_residual_decision_json" >&2
    exit 2
  fi
fi

if [[ -n "$node_disparity_residual_feature_selector_json" ]]; then
  if [[ -n "$disparity_residual_profile_json" || -n "$disparity_residual_decision_json" || -n "$disparity_residual_selector_json" || -n "$disparity_residual_feature_selector_json" ]]; then
    echo "error: node-side residual feature selector cannot be combined with a pre-resolved residual source" >&2
    exit 2
  fi
  if [[ ! -f "$node_disparity_residual_feature_selector_json" ]]; then
    echo "error: node disparity residual feature selector not found: $node_disparity_residual_feature_selector_json" >&2
    exit 2
  fi
  if [[ -z "$node_disparity_residual_feature_catalog_json" || ! -f "$node_disparity_residual_feature_catalog_json" ]]; then
    echo "error: --node-disparity-residual-feature-catalog-json is required for node-side feature selection" >&2
    exit 2
  fi
fi

if [[ -n "$package_dir" ]]; then
  if [[ ! -d "$package_dir" ]]; then
    echo "error: deployment package directory not found: $package_dir" >&2
    exit 2
  fi
  profile_json="$package_dir/active_runtime_profile.json"
  if [[ -z "$observation_fusion_manifest" && -f "$package_dir/observation_fusion_runtime_manifest.json" ]]; then
    observation_fusion_manifest="$package_dir/observation_fusion_runtime_manifest.json"
  fi
fi

if [[ -n "$observation_fusion_package" ]]; then
  if [[ ! -d "$observation_fusion_package" ]]; then
    echo "error: observation fusion package directory not found: $observation_fusion_package" >&2
    exit 2
  fi
  observation_fusion_manifest="$observation_fusion_package/observation_fusion_runtime_manifest.json"
fi

if [[ -n "$observation_fusion_manifest" && ! -f "$observation_fusion_manifest" ]]; then
  echo "error: observation fusion manifest not found: $observation_fusion_manifest" >&2
  exit 2
fi

if [[ -n "$disparity_residual_profile_json" && ! -f "$disparity_residual_profile_json" ]]; then
  echo "error: disparity residual profile not found: $disparity_residual_profile_json" >&2
  exit 2
fi

if [[ -z "$profile_json" ]]; then
  profile_json="$(
    find "$results_root" -type f \( -name 'active_runtime_profile.json' -o -name 'sim3_runtime_profile.json' \) -printf '%T@ %p\n' 2>/dev/null \
      | sort -nr \
      | awk 'NR == 1 { $1=""; sub(/^ /, ""); print; exit }'
  )"
fi

if [[ -z "$profile_json" ]]; then
  echo "error: no active_runtime_profile.json or sim3_runtime_profile.json found under $results_root" >&2
  echo "Run the RTK stereo acceptance/deployment wrapper first, or pass --profile-json/--package-dir." >&2
  exit 2
fi

if [[ ! -f "$profile_json" ]]; then
  echo "error: runtime profile not found: $profile_json" >&2
  exit 2
fi

cmd=(
  "$ROOT_DIR/run_operator_view.sh"
  --bbox-adjustment
  "$profile_json"
)
if [[ -n "$observation_fusion_manifest" ]]; then
  cmd+=(--observation-fusion-manifest "$observation_fusion_manifest")
fi
if [[ -n "$observation_fusion_tier" ]]; then
  cmd+=(--observation-fusion-tier "$observation_fusion_tier")
fi
if [[ -n "$disparity_residual_profile_json" ]]; then
  cmd+=(--disparity-residual-profile "$disparity_residual_profile_json")
fi
if [[ -n "$node_disparity_residual_feature_selector_json" ]]; then
  cmd+=(
    --disparity-residual-feature-selector
    "$node_disparity_residual_feature_selector_json"
    --disparity-residual-feature-catalog
    "$node_disparity_residual_feature_catalog_json"
  )
  if [[ -n "$node_disparity_residual_feature_window_size" ]]; then
    cmd+=(--disparity-residual-feature-window-size "$node_disparity_residual_feature_window_size")
  fi
  if [[ -n "$node_disparity_residual_feature_min_samples" ]]; then
    cmd+=(--disparity-residual-feature-min-samples "$node_disparity_residual_feature_min_samples")
  fi
fi
cmd+=("${operator_args[@]}")

if [[ -n "$selected_label" ]]; then
  printf '%s\n' "Selector: $selector_json"
  printf '%s\n' "Selected label: $selected_label"
fi
if [[ -n "${residual_selected_label:-}" ]]; then
  printf '%s\n' "Residual selector: $disparity_residual_selector_json"
  printf '%s\n' "Residual selected label: $residual_selected_label"
fi
if [[ -n "${residual_feature_selected_label:-}" ]]; then
  printf '%s\n' "Residual feature selector: $disparity_residual_feature_selector_json"
  printf '%s\n' "Residual feature selected label: $residual_feature_selected_label"
fi
if [[ -n "${residual_decision_selected_label:-}" ]]; then
  printf '%s\n' "Residual decision: $disparity_residual_decision_json"
  printf '%s\n' "Residual decision selected label: $residual_decision_selected_label"
fi
printf '%s\n' "Runtime profile: $profile_json"
if [[ -n "$disparity_residual_profile_json" ]]; then
  printf '%s\n' "Disparity residual profile: $disparity_residual_profile_json"
fi
if [[ -n "$node_disparity_residual_feature_selector_json" ]]; then
  printf '%s\n' "Node disparity residual feature selector: $node_disparity_residual_feature_selector_json"
  printf '%s\n' "Node disparity residual feature catalog: $node_disparity_residual_feature_catalog_json"
  printf '%s\n' "Node disparity residual feature window size: ${node_disparity_residual_feature_window_size:-1}"
  printf '%s\n' "Node disparity residual feature min samples: ${node_disparity_residual_feature_min_samples:-1}"
fi
if [[ -n "$observation_fusion_manifest" ]]; then
  printf '%s\n' "Observation fusion manifest: $observation_fusion_manifest"
fi
if [[ -n "$package_dir" ]]; then
  printf '%s\n' "Deployment package: $package_dir"
fi
printf '%s\n' "Command:"
printf '  %q' "${cmd[@]}"
printf '\n'

if [[ "$print_only" == true ]]; then
  exit 0
fi

exec "${cmd[@]}"
