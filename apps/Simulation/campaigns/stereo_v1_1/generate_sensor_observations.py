#!/usr/bin/env python3
"""Offline-only synthetic camera RTK/IMU observations for v1.1 raw runs.

The only truth access is inside this generator. Each rig/repetition/trial gets
one fixed noisy optical-center and camera-mounted-IMU observation shared by
all calibration paths, methods, attitude variants and the matching V0 run.
No estimator result or performance claim is produced here.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from collections import defaultdict
from pathlib import Path


SPEC = Path(__file__).resolve().parent / "protocol" / "03_experiment_spec.json"


def _read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _normal(key: str) -> float:
    """Stable independent N(0,1) draw for a named coordinate using SHA256."""
    digest = hashlib.sha256(key.encode("utf-8")).digest()
    first = (int.from_bytes(digest[:8], "big") + 0.5) / 2**64
    second = (int.from_bytes(digest[8:16], "big") + 0.5) / 2**64
    return math.sqrt(-2.0 * math.log(first)) * math.cos(2.0 * math.pi * second)


def _q_equivalent(a: list[float], b: list[float], tolerance: float = 1e-8) -> bool:
    return min(math.dist(a, b), math.dist(a, [-value for value in b])) < tolerance


def _source_camera_truth(root: Path, slots: list[dict]) -> tuple[dict, list[dict]]:
    truth_sources = []
    payloads = None
    for slot in slots:
        attempt = slot["attempts"][-1]
        path = root / attempt["recording_path"] / "camera_truth.json"
        current = _read(path)["camera_payloads"]
        if set(current) != {"left", "right"}:
            raise ValueError(f"missing camera truth side: {slot['run_id']}")
        for side in ("left", "right"):
            candidate = current[side]
            if candidate["desired_to_observed_optical_delta_norm_m"] > 1e-6 or candidate["desired_to_observed_optical_orientation_delta_deg"] > 1e-5:
                raise ValueError(f"physical optical pose deviates from protocol: {slot['run_id']} {side}")
            if payloads is not None:
                original = payloads[side]
                if math.dist(candidate["observed_optical_position_world_enu_m"],
                             original["observed_optical_position_world_enu_m"]) > 1e-6:
                    raise ValueError(f"rig optical center changed across paths: {slot['run_id']} {side}")
                if not _q_equivalent(candidate["observed_optical_orientation_world_from_entity_xyzw"],
                                     original["observed_optical_orientation_world_from_entity_xyzw"]):
                    raise ValueError(f"rig optical orientation changed across paths: {slot['run_id']} {side}")
        payloads = payloads or current
        truth_sources.append({"run_id": slot["run_id"],
                              "camera_truth_sha256": _sha256(path)})
    assert payloads is not None
    return payloads, truth_sources


def _observation(group: str, repetition: int, trial: int, truth: dict,
                 spec: dict, *, position_sigma_xyz_m=None,
                 imu_sigma_deg: float | None = None,
                 scenario_id: str = "main") -> dict:
    sigma_xyz = ([float(value) for value in position_sigma_xyz_m]
                 if position_sigma_xyz_m is not None else
                 [float(value) for value in spec["calibration_contract"]["camera_centers"]["sigma_xyz_m"]])
    if len(sigma_xyz) != 3 or any(not math.isfinite(value) or value < 0 for value in sigma_xyz):
        raise ValueError("camera RTK sigma must contain three finite nonnegative ENU values")
    angle_sigma = (float(imu_sigma_deg) if imu_sigma_deg is not None else
                   float(spec["calibration_contract"]["imu_prior"]["main_sigma_deg"]))
    if not math.isfinite(angle_sigma) or angle_sigma <= 0:
        raise ValueError("camera IMU sigma must be finite and positive; zero belongs to YAW_IDEAL")
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", scenario_id):
        raise ValueError("scenario_id must be a path-free identifier")
    sigma_angle_rad = math.radians(angle_sigma)
    draw_key = f"stereo_v1_1:{group}:R{repetition:02d}:MC{trial:03d}"
    shared_key = draw_key if scenario_id == "main" else f"sensitivity:{scenario_id}:{draw_key}"
    camera = {}
    for side in ("left", "right"):
        center = truth[side]["observed_optical_position_world_enu_m"]
        # Protocol G0/G1 renders phi=theta=0. The pose reader above verifies
        # the physical optical rotation against that declared true pose.
        # Keep the standardized random draw paired across one-factor levels;
        # only the declared sigma changes.
        position = [center[axis] + sigma_xyz[axis] * _normal(f"{draw_key}:{side}:rtk:{axis}")
                    for axis in range(3)]
        roll = sigma_angle_rad * _normal(f"{draw_key}:{side}:imu:roll")
        pitch = sigma_angle_rad * _normal(f"{draw_key}:{side}:imu:pitch")
        camera[side] = {
            "camera_id": side,
            "optical_center_rtk_position_world_enu_m": position,
            "optical_center_rtk_covariance_world_enu_m2": [
                [sigma_xyz[0] ** 2, 0.0, 0.0],
                [0.0, sigma_xyz[1] ** 2, 0.0],
                [0.0, 0.0, sigma_xyz[2] ** 2],
            ],
            "camera_imu_roll_pitch_mean_optical_rad": [roll, pitch],
            "camera_imu_roll_pitch_covariance_optical_rad2": [
                [sigma_angle_rad ** 2, 0.0], [0.0, sigma_angle_rad ** 2],
            ],
            "camera_imu_to_optical_rotation_matrix": [
                [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0],
            ],
            "rtk_observation_reference": "idealized_optical_center_direct",
            "physical_antenna_to_optical_center_lever_arm_m": None,
            "physical_antenna_lever_arm_status": "unverified; not used by this synthetic optical-center observation",
        }
    return {
        "schema_version": "stereo_v1_1_synthetic_camera_sensor_observations_v1",
        "method_name": ("CAMERA_RTK_IMU_SYNTHETIC_MAIN" if scenario_id == "main"
                         else "CAMERA_RTK_IMU_SYNTHETIC_SENSITIVITY"),
        "purpose": "fixed camera position and finite roll/pitch prior inputs for offline calibration",
        "synthetic": True,
        "runtime_sensor_claim": False,
        "shared_sensor_key": shared_key,
        "rig_group": group,
        "flight_repetition": repetition,
        "monte_carlo_trial_index": trial,
        "input_assumptions": {
            "camera_position_frame": "gazebo_world_enu:east,north,up_m",
            "camera_imu_angle_parameterization": "phi optical-forward roll, theta nominal-pitch correction; rad",
            "position_noise_sigma_world_enu_m": sigma_xyz,
            "roll_pitch_noise_sigma_rad": sigma_angle_rad,
            "noise_draw": "independent SHA256-keyed Box-Muller normal per camera and component; held across paths",
            "antenna_to_optical_center": "not applied; observation synthesized directly at optical center",
            "sensitivity_scenario_id": scenario_id,
        },
        "output_contract": "estimator may read cameras only; do not pass generator provenance or GT files",
        "estimator_input_allowlist": ["cameras.left", "cameras.right", "shared_sensor_key"],
        "cameras": camera,
        "pass_criteria": "same scenario key and byte-identical camera observations across paths in one rig/repetition/trial",
    }


def make_sensitivity_observation(root: Path, run_id: str, trial: int,
                                 scenario_id: str, *,
                                 position_sigma_xyz_m=None,
                                 imu_sigma_deg: float | None = None) -> tuple[dict, list[dict]]:
    """Create one allowlisted camera-sensor realization for a stress case.

    This sensor producer is allowed to read camera truth. The estimator only
    receives the returned camera RTK/IMU values and shared key.
    """
    root = root.resolve()
    status = _read(root / "run_status.json")
    matches = [row for row in status.get("run_slots", []) if row.get("run_id") == run_id]
    if len(matches) != 1 or matches[0].get("status") not in ("VALID", "VALID_WITH_WARNINGS"):
        raise ValueError(f"sensitivity source is not a source-QC-valid run: {run_id}")
    slot = matches[0]
    if trial < 0:
        raise ValueError("trial index must be nonnegative")
    group, repetition = str(slot["rig_group"]), int(slot["repetition"])
    shared_slots = [row for row in status["run_slots"]
                    if row["rig_group"] == group and int(row["repetition"]) == repetition]
    truth, sources = _source_camera_truth(root, shared_slots)
    spec = json.loads(SPEC.read_text(encoding="utf-8-sig"))
    return (_observation(group, repetition, trial, truth, spec,
                         position_sigma_xyz_m=position_sigma_xyz_m,
                         imu_sigma_deg=imu_sigma_deg,
                         scenario_id=scenario_id), sources)


def generate(root: Path, trial: int = 0, *, output_root: Path | None = None) -> list[Path]:
    root = root.resolve()
    status = _read(root / "run_status.json")
    if status["valid_run_count"] != 39:
        raise ValueError("generate main shared sensor observations after all 39 source QC slots")
    if trial < 0:
        raise ValueError("trial index must be nonnegative")
    spec = json.loads(SPEC.read_text(encoding="utf-8-sig"))
    groups = defaultdict(list)
    for slot in status["run_slots"]:
        if slot["status"] not in ("VALID", "VALID_WITH_WARNINGS"):
            raise ValueError(f"nonvalid raw slot: {slot['run_id']}")
        groups[(slot["rig_group"], slot["repetition"])].append(slot)
    written = []
    for (group, repetition), slots in sorted(groups.items()):
        truth, sources = _source_camera_truth(root, slots)
        observation = _observation(group, repetition, trial, truth, spec)
        for slot in slots:
            attempt = slot["attempts"][-1]["attempt_id"]
            target_parent = Path(output_root).resolve() if output_root is not None else root / "파생_재처리"
            target_dir = target_parent / slot["run_id"] / attempt / "sensors" / f"trial_{trial:03d}"
            target_dir.mkdir(parents=True, exist_ok=True)
            target = target_dir / "sensor_observations.json"
            provenance = target_dir / "sensor_generation_provenance.json"
            expected_observation = json.dumps(observation, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
            expected_provenance = {
                "schema_version": "stereo_v1_1_sensor_generation_provenance_v1",
                "offline_truth_sources": sources,
                "observation_sha256": hashlib.sha256(expected_observation.encode("utf-8")).hexdigest(),
                "estimator_input": False,
            }
            expected_provenance_text = json.dumps(
                expected_provenance, ensure_ascii=False, indent=2, allow_nan=False
            ) + "\n"
            if target.exists() or provenance.exists():
                if not target.is_file() or not provenance.is_file():
                    raise FileExistsError(f"partial sensor generation exists and is preserved: {target_dir}")
                if (target.read_text(encoding="utf-8") != expected_observation
                        or provenance.read_text(encoding="utf-8") != expected_provenance_text):
                    raise FileExistsError(f"sensor generation differs from requested trial; refusing overwrite: {target_dir}")
            else:
                with target.open("x", encoding="utf-8") as handle:
                    handle.write(expected_observation)
                    handle.flush()
                with provenance.open("x", encoding="utf-8") as handle:
                    handle.write(expected_provenance_text)
                    handle.flush()
            written.append(target)
    return written


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--trial-index", type=int, default=0)
    args = parser.parse_args()
    paths = generate(args.output_root, args.trial_index)
    print(json.dumps({"sensor_observation_files": len(paths), "trial_index": args.trial_index}))


if __name__ == "__main__":
    main()
