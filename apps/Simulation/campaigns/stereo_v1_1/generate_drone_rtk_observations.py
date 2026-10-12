#!/usr/bin/env python3
"""Generate explicit synthetic 10 Hz drone RTK inputs from recorded GT.

This producer alone may read ``ground_truth.jsonl``. Its result is an ENU
measurement stream shared byte-for-byte by methods A and B; method C must never
receive it. The observation reference is the recorded vehicle_reference_link,
not an inferred GNSS antenna point.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path


CAMPAIGN_ROOT = Path(__file__).resolve().parent
SPEC_PATH = CAMPAIGN_ROOT / "protocol" / "03_experiment_spec.json"
MANIFEST_PATH = CAMPAIGN_ROOT / "protocol" / "04_run_manifest.json"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _normal(key: str) -> float:
    digest = hashlib.sha256(key.encode("utf-8")).digest()
    first = (int.from_bytes(digest[:8], "big") + 0.5) / 2**64
    second = (int.from_bytes(digest[8:16], "big") + 0.5) / 2**64
    return math.sqrt(-2.0 * math.log(first)) * math.cos(2.0 * math.pi * second)


def _read_rows(path: Path) -> list[dict]:
    with path.open(encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def _sample_linear(rows_by_epoch: dict[int, list[dict]], epoch: int, stamp_ns: int) -> list[float] | None:
    rows = rows_by_epoch.get(epoch, [])
    if not rows or stamp_ns < rows[0]["source_timestamp_ns"] or stamp_ns > rows[-1]["source_timestamp_ns"]:
        return None
    low, high = 0, len(rows) - 1
    while low <= high:
        middle = (low + high) // 2
        if rows[middle]["source_timestamp_ns"] < stamp_ns:
            low = middle + 1
        elif rows[middle]["source_timestamp_ns"] > stamp_ns:
            high = middle - 1
        else:
            return [float(value) for value in rows[middle]["position_world_enu_m"]]
    before, after = rows[high], rows[low]
    gap = int(after["source_timestamp_ns"]) - int(before["source_timestamp_ns"])
    if gap <= 0 or gap > 40_000_000:
        return None
    alpha = (stamp_ns - int(before["source_timestamp_ns"])) / gap
    return [
        (1.0 - alpha) * float(before["position_world_enu_m"][axis])
        + alpha * float(after["position_world_enu_m"][axis])
        for axis in range(3)
    ]


def generate_attempt(output_root: Path, run_id: str, attempt_id: str,
                     trial_index: int, *, shared_bias: bool = False,
                     sigma_xyz_m: list[float] | None = None,
                     shared_bias_sigma_xyz_m: list[float] | None = None,
                     timestamp_offset_ns: int = 0,
                     output_dir: Path | None = None,
                     scenario_id: str = "main") -> tuple[Path, Path]:
    root = output_root.resolve()
    status = json.loads((root / "run_status.json").read_text(encoding="utf-8"))
    slot = next((row for row in status["run_slots"] if row["run_id"] == run_id), None)
    if slot is None or slot["status"] not in ("VALID", "VALID_WITH_WARNINGS"):
        raise ValueError(f"run is not a source-QC-valid 39-slot attempt: {run_id}")
    attempt = slot["attempts"][-1]
    if attempt["attempt_id"] != attempt_id:
        raise ValueError("requested attempt is not the current valid source attempt")
    if trial_index < 0:
        raise ValueError("trial index must be nonnegative")

    spec = json.loads(SPEC_PATH.read_text(encoding="utf-8-sig"))
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8-sig"))
    source = next(row for row in manifest["runs"] if row["run_id"] == run_id)
    sigma = ([float(value) for value in sigma_xyz_m] if sigma_xyz_m is not None else
             [float(value) for value in spec["reprocessing"]["rtk_sigma_xyz_m"][1]])
    if len(sigma) != 3 or any(not math.isfinite(value) or value < 0 for value in sigma):
        raise ValueError("RTK sigma must contain three finite nonnegative ENU values")
    bias_sigma = ([float(value) for value in shared_bias_sigma_xyz_m]
                  if shared_bias_sigma_xyz_m is not None else sigma)
    if len(bias_sigma) != 3 or any(not math.isfinite(value) or value < 0 for value in bias_sigma):
        raise ValueError("shared RTK bias sigma must contain three finite nonnegative ENU values")
    if not isinstance(timestamp_offset_ns, int):
        raise ValueError("RTK timestamp offset must be an integer nanosecond count")
    if not scenario_id or any(char not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-"
                              for char in scenario_id):
        raise ValueError("scenario_id must be a path-free identifier")
    period_ns = int(round(1e9 / float(spec["reprocessing"]["rtk_sample_rate_hz"])))
    attempt_dir = root / attempt["recording_path"]
    truth_path = attempt_dir / "ground_truth.jsonl"
    truth_rows = _read_rows(truth_path)
    groups: dict[int, list[dict]] = {}
    for row in truth_rows:
        groups.setdefault(int(row["simulation_epoch"]), []).append(row)
    by_epoch = {epoch: sorted(rows, key=lambda row: int(row["source_timestamp_ns"]))
                for epoch, rows in groups.items()}

    shared = [0.0, 0.0, 0.0]
    if shared_bias:
        shared = [bias_sigma[axis] * _normal(f"stereo_v1_1:drone-rtk-bias:{run_id}:MC{trial_index:03d}:{axis}")
                  for axis in range(3)]
    records = []
    for epoch, rows in sorted(by_epoch.items()):
        first = int(rows[0]["source_timestamp_ns"])
        last = int(rows[-1]["source_timestamp_ns"])
        first_tick = ((first + period_ns - 1) // period_ns) * period_ns
        for sample_index, stamp_ns in enumerate(range(first_tick, last + 1, period_ns)):
            truth = _sample_linear(by_epoch, epoch, stamp_ns)
            if truth is None:
                continue
            key = f"stereo_v1_1:drone-rtk:{run_id}:seed{source['seed_requested']}:MC{trial_index:03d}:E{epoch}:S{sample_index}"
            measured = [truth[axis] + shared[axis]
                        + sigma[axis] * _normal(f"{key}:{axis}") for axis in range(3)]
            record = {
                "schema_version": "stereo_v1_1_synthetic_drone_rtk_observation_v1",
                "run_id": run_id, "attempt_id": attempt_id, "trial_index": trial_index,
                "sample_index": sample_index, "simulation_epoch": epoch,
                "timestamp_ns": stamp_ns + timestamp_offset_ns,
                "position_frame": "gazebo_world_enu",
                "position_axis_order": "east,north,up",
                "position_world_enu_m": measured,
                "covariance_world_enu_m2": [
                    [sigma[0] ** 2, 0.0, 0.0],
                    [0.0, sigma[1] ** 2, 0.0],
                    [0.0, 0.0, sigma[2] ** 2],
                ],
                "reference_point": "vehicle_reference_link",
                "synthetic": True,
                "shared_bias_m": shared if shared_bias else None,
            }
            if timestamp_offset_ns:
                record["unshifted_timestamp_ns"] = stamp_ns
                record["timestamp_offset_ns"] = timestamp_offset_ns
            records.append(record)
    if not records:
        raise ValueError(f"no GT-bracketed 10 Hz RTK observations generated for {run_id}")

    target_dir = (Path(output_dir).resolve() if output_dir is not None else
                  root / "파생_재처리" / run_id / attempt_id / "sensors" / f"trial_{trial_index:03d}")
    target_dir.mkdir(parents=True, exist_ok=True)
    output = target_dir / "drone_rtk_observations.jsonl"
    encoded = "".join(json.dumps(row, ensure_ascii=False, sort_keys=True, allow_nan=False) + "\n"
                      for row in records)
    provenance = target_dir / "drone_rtk_generation_provenance.json"
    provenance_payload = {
        "schema_version": "stereo_v1_1_drone_rtk_generation_provenance_v1",
        "source_ground_truth_sha256": _sha256(truth_path),
        "source_seed_requested": source["seed_requested"], "trial_index": trial_index,
        "sample_rate_hz": spec["reprocessing"]["rtk_sample_rate_hz"],
        "sigma_xyz_m": sigma, "shared_bias_enabled": shared_bias,
        "shared_bias_sigma_xyz_m": bias_sigma if shared_bias else None,
        "timestamp_offset_ns": timestamp_offset_ns,
        "scenario_id": scenario_id,
        "observation_count": len(records),
        "method_allowlist": ["A_rtk_independent", "B_rtk_joint"],
        "estimator_input": False,
    }
    provenance_text = json.dumps(provenance_payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    if output.exists() or provenance.exists():
        if not output.is_file() or not provenance.is_file():
            raise FileExistsError(f"partial drone RTK trial exists and is preserved: {target_dir}")
        if output.read_text(encoding="utf-8") != encoded or provenance.read_text(encoding="utf-8") != provenance_text:
            raise FileExistsError(f"drone RTK trial differs from requested inputs; refusing overwrite: {target_dir}")
    else:
        with output.open("x", encoding="utf-8") as handle:
            handle.write(encoded)
            handle.flush()
        with provenance.open("x", encoding="utf-8") as handle:
            handle.write(provenance_text)
            handle.flush()
    return output, provenance


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--attempt-id", required=True)
    parser.add_argument("--trial-index", type=int, default=0)
    parser.add_argument("--shared-bias", action="store_true")
    parser.add_argument("--timestamp-offset-ns", type=int, default=0)
    args = parser.parse_args()
    output, provenance = generate_attempt(args.output_root, args.run_id, args.attempt_id,
                                          args.trial_index, shared_bias=args.shared_bias,
                                          timestamp_offset_ns=args.timestamp_offset_ns)
    print(json.dumps({"output": str(output), "provenance": str(provenance)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
