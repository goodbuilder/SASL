#!/usr/bin/env python3
"""Validate the frozen v1.1 manifest and produce a non-executing campaign plan.

This entry point never starts Gazebo/PX4 and cannot mark a run as completed.
Its output root is deliberately separate from the legacy attitude sweep.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path


CAMPAIGN_ROOT = Path(__file__).resolve().parent
SIM_ROOT = CAMPAIGN_ROOT.parents[1]
PROTOCOL_ROOT = CAMPAIGN_ROOT / "protocol"
RECORDINGS_ROOT = SIM_ROOT / "recordings" / "stereo_v1_1"
PROTOCOL_FILES = (
    "03_experiment_spec.json",
    "04_run_manifest.json",
    "05_generate_reference.py",
)
PROFILE_ROOT = SIM_ROOT / "config" / "campaign" / "v1_1"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _profile_values(group: str) -> dict[str, str]:
    path = PROFILE_ROOT / f"{group}.env"
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped and not stripped.startswith("#"):
            key, value = stripped.split("=", 1)
            if key in values:
                raise ValueError(f"duplicate profile value: {group}/{key}")
            values[key] = value
    return values


def build_plan() -> dict:
    provenance = json.loads((PROTOCOL_ROOT / "provenance.json").read_text(encoding="utf-8"))
    if provenance["files_sha256"] != {
        name: _sha256(PROTOCOL_ROOT / name) for name in PROTOCOL_FILES
    }:
        raise ValueError("frozen protocol source hash mismatch")
    spec = json.loads((PROTOCOL_ROOT / PROTOCOL_FILES[0]).read_text(encoding="utf-8-sig"))
    manifest = json.loads((PROTOCOL_ROOT / PROTOCOL_FILES[1]).read_text(encoding="utf-8-sig"))
    model_manifest_path = CAMPAIGN_ROOT / "model_manifest.json"
    model_manifest = json.loads(model_manifest_path.read_text(encoding="utf-8"))
    if len(model_manifest["sha256"]) != 64 or not all(
        character in "0123456789abcdef" for character in model_manifest["sha256"]
    ):
        raise ValueError("campaign YOLO model hash is invalid")
    runs = manifest["runs"]
    if manifest["total"] != 39 or len(runs) != 39:
        raise ValueError("protocol requires exactly 39 manifest slots")
    if len({row["run_id"] for row in runs}) != 39:
        raise ValueError("manifest run_id is not unique")
    if [row["sequence"] for row in runs] != list(range(1, 40)):
        raise ValueError("manifest sequence must be 1..39")
    if len({row["seed_requested"] for row in runs}) != 39:
        raise ValueError("manifest seed_requested is not unique")

    profiles = {}
    for group in ("G0", "G1"):
        values = _profile_values(group)
        expected_heading = spec["camera"]["physical_pose_groups"][group]
        for side, marker in (("left", "L"), ("right", "R")):
            if float(values[f"CAM_{marker}_OPTICAL_ROLL_DEG"]) != 0.0:
                raise ValueError(f"{group} {side} optical roll must be zero")
            if float(values[f"CAM_{marker}_OPTICAL_PITCH_DEG"]) != 20.0:
                raise ValueError(f"{group} {side} optical pitch must be 20 deg")
            if float(values[f"CAM_{marker}_OPTICAL_YAW_DEG"]) != expected_heading[
                f"heading_{side}_deg"
            ]:
                raise ValueError(f"{group} {side} optical heading differs from protocol")
        if float(values["CAMERA_BASELINE_M"]) != spec["camera"]["baseline_m"]:
            raise ValueError(f"{group} camera baseline differs from protocol")
        if float(values["CAMERA_HEIGHT_M"]) != spec["camera"]["left_center_world_m"][2]:
            raise ValueError(f"{group} camera height differs from protocol")
        profiles[group] = {
            "path_from_sim_root": str((PROFILE_ROOT / f"{group}.env").relative_to(SIM_ROOT)),
            "sha256": _sha256(PROFILE_ROOT / f"{group}.env"),
            "world_heading_deg": expected_heading,
        }

    planned = []
    group_counts = {"G0": 0, "G1": 0}
    for row in runs:
        group = row["rig_group"]
        path_id = row["path_id"]
        if group not in group_counts or path_id not in spec["trajectories"]:
            raise ValueError(f"unknown rig/path in {row['run_id']}")
        if group == "G1" and path_id not in ("P0", "P4", "V0"):
            raise ValueError(f"G1 path is outside protocol: {path_id}")
        expected_manifest_heading = {
            side: profiles[group]["world_heading_deg"][f"heading_{side}_deg"]
            for side in ("left", "right")
        }
        if row["true_camera_heading_deg"] != expected_manifest_heading:
            raise ValueError(f"manifest heading mismatch in {row['run_id']}")
        if row["role"] != spec["trajectories"][path_id]["role"]:
            raise ValueError(f"manifest role mismatch in {row['run_id']}")
        if row["status"] != "PLANNED" or row["attempt_id"] != "a01":
            raise ValueError(f"source manifest is not pristine in {row['run_id']}")
        group_counts[group] += 1
        planned.append(
            {
                "sequence": row["sequence"],
                "run_id": row["run_id"],
                "rig_group": group,
                "path_id": path_id,
                "repetition": row["repetition"],
                "seed_requested": row["seed_requested"],
                "role": row["role"],
                "status": "PLANNED",
                "attempt_count": 0,
                "recording_path": None,
            }
        )
    if group_counts != {"G0": 30, "G1": 9}:
        raise ValueError(f"unexpected group counts: {group_counts}")
    return {
        "schema_version": "stereo_v1_1_campaign_plan_v1",
        "document_version": spec["document_version"],
        "execution_enabled": False,
        "flight_results_present": False,
        "planned_run_count": 39,
        "group_counts": group_counts,
        "protocol_sha256": {
            name: _sha256(PROTOCOL_ROOT / name) for name in PROTOCOL_FILES
        },
        "model_manifest_sha256": _sha256(model_manifest_path),
        "yolo_model": {
            "sha256": model_manifest["sha256"],
            "size_bytes": model_manifest["size_bytes"],
            "owned_path_from_repo_root": model_manifest["owned_path_from_repo_root"],
        },
        "profiles": profiles,
        "planned_runs": planned,
    }


def write_plan(plan: dict, output_dir: Path) -> Path:
    requested = output_dir.expanduser().resolve()
    allowed = RECORDINGS_ROOT.resolve()
    if requested == allowed or not requested.is_relative_to(allowed):
        raise ValueError(f"plan output must be a new child of {allowed}")
    if requested.exists():
        raise FileExistsError(f"campaign output already exists: {requested}")
    requested.mkdir(parents=True, exist_ok=False)
    destination = requested / "campaign_plan.json"
    encoded = json.dumps(plan, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    with destination.open("x", encoding="utf-8") as handle:
        handle.write(encoded)
        handle.flush()
        os.fsync(handle.fileno())
    return destination


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir", type=Path,
        help="Write a plan to a new directory under apps/Simulation/recordings/stereo_v1_1/",
    )
    args = parser.parse_args()
    plan = build_plan()
    output = None
    if args.output_dir:
        output = write_plan(plan, args.output_dir)
    print(json.dumps({
        "status": "PLAN_ONLY",
        "execution_enabled": False,
        "planned_run_count": plan["planned_run_count"],
        "group_counts": plan["group_counts"],
        "plan_written_to": None if output is None else str(output),
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
