#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from pathlib import Path
from typing import Mapping, Sequence


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from compare_stereodistance_profiles_on_common_csv import (  # noqa: E402
    RUNTIME_INPUT_CONTRACT,
    compare_profiles_on_common_csv,
    json_safe,
)
from select_stereodistance_profile_by_calibration_replay import write_csv, write_json  # noqa: E402
from stereodistance_profile_replay import PUBLIC_TOPIC_CONTRACT, RDF_AXIS_CONTRACT  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Select top method-search frontier runtime profiles and replay them on one calibration CSV. "
            "PX4/RTK truth is used only by the common-CSV offline evaluator; runtime profiles consume "
            "stereo-center RDF plus measured disparity."
        )
    )
    parser.add_argument("--frontier-csv", type=Path, required=True)
    parser.add_argument("--runtime-profiles-dir", type=Path, required=True)
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--active-profile-json", type=Path)
    parser.add_argument("--active-label", default="current_active")
    parser.add_argument("--top-n", type=int, default=12)
    parser.add_argument("--split", default="validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    return parser.parse_args()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def safe_label(value: str) -> str:
    label = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    label = label.strip("._-")
    return label or "candidate"


def frontier_profile_candidates(
    *,
    frontier_csv: Path,
    runtime_profiles_dir: Path,
    active_profile_json: Path | None,
    active_label: str,
    top_n: int,
) -> tuple[list[str], list[dict[str, object]]]:
    if top_n <= 0:
        raise ValueError("--top-n must be positive")
    profile_specs: list[str] = []
    manifest_rows: list[dict[str, object]] = []
    seen_paths: set[Path] = set()

    if active_profile_json is not None:
        active_path = active_profile_json
        active_found = active_path.exists()
        if active_found:
            profile_specs.append(f"{safe_label(active_label)}:{active_path}")
            seen_paths.add(active_path.resolve())
        manifest_rows.append(
            {
                "source": "active_profile",
                "rank": 0,
                "label": safe_label(active_label),
                "method": "",
                "profile_json": str(active_path),
                "profile_found": active_found,
                "selected_for_replay": active_found,
                "axis_contract": RDF_AXIS_CONTRACT,
                "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
                "runtime_truth_used": False,
            }
        )

    for row in read_csv(frontier_csv)[:top_n]:
        method = str(row.get("method", "")).strip()
        if not method:
            continue
        profile_json = runtime_profiles_dir / f"{method}.json"
        label = safe_label(f"frontier_r{row.get('rank', '')}_{method}")
        resolved = profile_json.resolve()
        found = profile_json.exists()
        selected = found and resolved not in seen_paths
        manifest_rows.append(
            {
                "source": "method_search_frontier",
                "rank": row.get("rank", ""),
                "label": label,
                "method": method,
                "profile_json": str(profile_json),
                "profile_found": found,
                "selected_for_replay": selected,
                "frontier_strict_all_scenarios": row.get("strict_all_scenarios", ""),
                "frontier_validation_range_rmse_mean_m": row.get("validation_range_rmse_mean_m", ""),
                "frontier_target_abs_mean_error_max_m": row.get("target_abs_mean_error_max_m", ""),
                "frontier_rdf_axis_nonregression_pass": row.get("rdf_axis_nonregression_pass", ""),
                "axis_contract": RDF_AXIS_CONTRACT,
                "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
                "runtime_truth_used": False,
            }
        )
        if selected:
            profile_specs.append(f"{label}:{profile_json}")
            seen_paths.add(resolved)

    if not profile_specs:
        raise ValueError("no runtime profile candidates were found for replay")
    return profile_specs, manifest_rows


def write_frontier_summary_txt(
    path: Path,
    *,
    replay_summary: Mapping[str, object],
    manifest_rows: Sequence[Mapping[str, object]],
) -> None:
    selected = replay_summary.get("selected_candidate", "")
    lines = [
        "Common-CSV method-frontier runtime profile comparison",
        "",
        f"status: {replay_summary.get('status', '')}",
        f"selected_candidate: {selected}",
        f"selected_method: {replay_summary.get('selected_method', '')}",
        f"candidate_count: {replay_summary.get('candidate_count', '')}",
        f"passing_candidate_count: {replay_summary.get('passing_candidate_count', '')}",
        f"axis_contract: {RDF_AXIS_CONTRACT}",
        f"public_topic_contract: {PUBLIC_TOPIC_CONTRACT}",
        f"runtime_input_contract: {RUNTIME_INPUT_CONTRACT}",
        "runtime_truth_used: False",
        "",
        "Frontier candidates:",
    ]
    for row in manifest_rows:
        lines.append(
            (
                "- rank={rank} label={label} method={method} found={found} "
                "selected_for_replay={selected}"
            ).format(
                rank=row.get("rank", ""),
                label=row.get("label", ""),
                method=row.get("method", ""),
                found=row.get("profile_found", ""),
                selected=row.get("selected_for_replay", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_frontier_profile_comparison(
    *,
    frontier_csv: Path,
    runtime_profiles_dir: Path,
    input_csv: Path,
    output_dir: Path,
    active_profile_json: Path | None = None,
    active_label: str = "current_active",
    top_n: int = 12,
    split: str = "validation",
    target_range_m: float = 1852.0,
    target_window_m: float = 30.0,
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    profile_specs, manifest_rows = frontier_profile_candidates(
        frontier_csv=frontier_csv,
        runtime_profiles_dir=runtime_profiles_dir,
        active_profile_json=active_profile_json,
        active_label=active_label,
        top_n=top_n,
    )
    replay_summary = compare_profiles_on_common_csv(
        input_csv=input_csv,
        profile_specs=profile_specs,
        output_dir=output_dir,
        split=split,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    manifest = {
        "frontier_csv": str(frontier_csv),
        "runtime_profiles_dir": str(runtime_profiles_dir),
        "input_csv": str(input_csv),
        "output_dir": str(output_dir),
        "top_n": top_n,
        "split": split,
        "target_range_m": target_range_m,
        "target_window_m": target_window_m,
        "selected_candidate": replay_summary.get("selected_candidate", ""),
        "selected_method": replay_summary.get("selected_method", ""),
        "candidate_count": replay_summary.get("candidate_count", 0),
        "passing_candidate_count": replay_summary.get("passing_candidate_count", 0),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "runtime_truth_used": False,
        "rows": [{key: json_safe(value) for key, value in row.items()} for row in manifest_rows],
    }
    write_csv(output_dir / "frontier_profile_candidate_manifest.csv", manifest_rows)
    write_json(output_dir / "frontier_profile_candidate_manifest.json", manifest)
    write_frontier_summary_txt(
        output_dir / "frontier_profile_candidate_manifest.txt",
        replay_summary=replay_summary,
        manifest_rows=manifest_rows,
    )
    return manifest


def main() -> int:
    args = parse_args()
    manifest = run_frontier_profile_comparison(
        frontier_csv=args.frontier_csv,
        runtime_profiles_dir=args.runtime_profiles_dir,
        input_csv=args.input_csv,
        output_dir=args.output_dir,
        active_profile_json=args.active_profile_json,
        active_label=args.active_label,
        top_n=args.top_n,
        split=args.split,
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
    )
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
