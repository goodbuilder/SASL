#!/usr/bin/env python3
"""Flag byte-identical image/GT content sequences among repeated raw runs.

Image pixels are compared per left/right original source frame. GT compares
ordered world-ENU [east,north,up] positions and world-from-reference XYZW
quaternions. Exposure/receipt timestamps and file paths are omitted so they
cannot make otherwise identical content appear different. Nonidentity does
not prove statistically independent stochastic repetitions.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
from collections import defaultdict
from pathlib import Path


def _read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _sequences(attempt: Path) -> dict:
    digests = {name: hashlib.sha256() for name in ("left", "right", "gt")}
    counts = {name: 0 for name in digests}
    with (attempt / "image_index.jsonl").open(encoding="utf-8") as handle:
        for line in handle:
            row = json.loads(line)
            side = row["camera_id"]
            if side not in ("left", "right"):
                raise ValueError("unknown camera side in source index")
            digests[side].update((row["image_pixels_sha256"] + "\n").encode("ascii"))
            counts[side] += 1
    with (attempt / "ground_truth.jsonl").open(encoding="utf-8") as handle:
        for line in handle:
            row = json.loads(line)
            if row["position_frame"] != "gazebo_world_enu" or row["quaternion_order"] != "xyzw":
                raise ValueError("GT coordinate contract changed")
            payload = [row["position_world_enu_m"], row["orientation_world_from_reference_xyzw"]]
            digests["gt"].update((json.dumps(payload, separators=(",", ":"), allow_nan=False) + "\n").encode())
            counts["gt"] += 1
    return {"counts": counts, "sha256": {name: digest.hexdigest() for name, digest in digests.items()}}


def audit(root: Path) -> dict:
    root = root.resolve()
    status = _read(root / "run_status.json")
    if status["valid_run_count"] != 39:
        raise ValueError("audit repetitions only after all 39 raw slots pass QC")
    groups = defaultdict(list)
    for slot in status["run_slots"]:
        attempt = root / slot["attempts"][-1]["recording_path"]
        groups[(slot["rig_group"], slot["path_id"])].append({
            "run_id": slot["run_id"], **_sequences(attempt)
        })
    identical = []
    group_rows = []
    for (group, path), rows in sorted(groups.items()):
        for left, right in itertools.combinations(rows, 2):
            if left["counts"] == right["counts"] and left["sha256"] == right["sha256"]:
                identical.append([left["run_id"], right["run_id"]])
        group_rows.append({"rig_group": group, "path_id": path, "runs": rows})
    return {
        "schema_version": "stereo_v1_1_repetition_sequence_audit_v1",
        "method_name": "EXACT_SOURCE_SEQUENCE_DUPLICATE_CHECK",
        "purpose": "flag completely identical camera-pixel and GT pose sequences within one physical group/path",
        "image_contract": "ordered original pixel SHA256 per left/right source frame; full attempt, timestamps excluded",
        "gt_contract": "ordered world ENU east,north,up metres and world-from-reference XYZW quaternion; full attempt, timestamps excluded",
        "pass_criteria": "identical repetition pairs listed, never silently interpreted as independent replicates",
        "independent_repetition_verified": False,
        "nonidentity_note": "Different content hashes do not establish independent random seed effects.",
        "identical_run_pairs": identical,
        "groups": group_rows,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    args = parser.parse_args()
    root = args.output_root.resolve()
    result = audit(root)
    path = root / "repetition_sequence_audit.json"
    with path.open("x", encoding="utf-8") as handle:
        json.dump(result, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")
    print(json.dumps({"groups": len(result["groups"]),
                      "identical_run_pairs": len(result["identical_run_pairs"])}))


if __name__ == "__main__":
    main()
