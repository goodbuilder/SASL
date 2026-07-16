from __future__ import annotations

import csv
import json
import os
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path

from wiris_stereo_distance import coordinate_contract as contract

SESSION_SCHEMA_VERSION = 2


def session_id_now() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


def session_dir(root: Path, session_id: str | None = None) -> Path:
    return root / "sessions" / (session_id or session_id_now())


def raw_video_paths(root: Path, session_id: str | None = None) -> dict[str, Path]:
    output_dir = session_dir(root, session_id)
    return {
        "left": output_dir / "video" / "raw_left.mp4",
        "right": output_dir / "video" / "raw_right.mp4",
    }


def format_video_paths(paths: dict[str, Path]) -> str:
    if not paths:
        return "저장 경로 없음"
    return ", ".join(f"{camera}={path}" for camera, path in paths.items())


@dataclass(frozen=True)
class SavedSessionFiles:
    metadata: Path
    distance_history: Path | None
    external_parameters: Path | None
    pixhawk_mag_status: Path | None
    uam_analysis: Path | None = None
    gps_source_comparison: Path | None = None
    video_outputs: dict[str, Path] = field(default_factory=dict)

    @property
    def outputs(self) -> list[Path]:
        return [
            path
            for path in (
                self.metadata,
                self.distance_history,
                self.external_parameters,
                self.pixhawk_mag_status,
                self.uam_analysis,
                self.gps_source_comparison,
                *self.video_outputs.values(),
            )
            if path is not None
        ]


def existing_video_outputs(video_paths: dict[str, Path] | None) -> dict[str, Path]:
    """Return non-empty video files that are already present in a session."""

    outputs: dict[str, Path] = {}
    for camera, path in (video_paths or {}).items():
        try:
            if path.is_file() and path.stat().st_size > 0:
                outputs[camera] = path
        except OSError:
            continue
    return outputs


def write_dynamic_csv(output: Path, rows: list[dict[str, object]]) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        fieldnames = list(dict.fromkeys(field for row in rows for field in row.keys()))
        writer = csv.DictWriter(
            handle,
            fieldnames=fieldnames,
            extrasaction="ignore",
        )
        writer.writeheader()
        writer.writerows(rows)


def evidence_contract_metadata() -> dict[str, object]:
    return {
        "system_claim": contract.SYSTEM_CLAIM,
        "claim_boundary": contract.CLAIM_BOUNDARY,
        "coordinate_contract": {
            "world_frame": contract.WORLD_FRAME,
            "body_frame": contract.BODY_FRAME,
            "camera_frame": contract.CAMERA_FRAME,
            "camera_mount_profile": contract.CAMERA_MOUNT_PROFILE,
            "measurement_frame": contract.MEASUREMENT_FRAME,
            "measurement_axis_contract": contract.MEASUREMENT_AXIS_CONTRACT,
            "frd_comparison_axis_contract": contract.FRD_COMPARISON_AXIS_CONTRACT,
            "truth_frame": contract.TRUTH_FRAME,
            "truth_axis_contract": contract.TRUTH_AXIS_CONTRACT,
            "mixed_frame_policy": "diagnostic-only",
        },
        "mount_assumption": {
            "model": contract.MOUNT_ASSUMPTION_MODEL,
            "status": contract.MOUNT_ASSUMPTION_STATUS,
            "detail": contract.MOUNT_ASSUMPTION_DETAIL,
            "unmodeled_error_policy": "remaining rig, lever-arm, and optical-axis errors are included in final error",
        },
        "evidence_policy": {
            "hardware_pass_requires_smoke": True,
            "simulation_pass_is_hardware_pass": False,
            "runtime_truth_required_for_measurement": False,
            "all_axis_improvement_claimed": False,
        },
        "connection_policy": {
            "policy": contract.CONNECTION_POLICY,
            "preferred_state": "mavlink_and_ros2_connected",
            "readiness_authority": "mavlink",
            "mavlink_role": contract.MAVLINK_PRIORITY_ROLE,
            "ros2_role": contract.ROS2_PRIORITY_ROLE,
            "ros2_replaces_mavlink_control": False,
            "mavlink_telemetry_fallback_when_ros2_unavailable": True,
        },
    }


def save_session_files(
    output_dir: Path,
    *,
    metadata: dict[str, object],
    distance_rows: list[dict[str, object]] | None = None,
    external_parameter_rows: list[dict[str, object]] | None = None,
    pixhawk_mag_rows: list[dict[str, object]] | None = None,
    uam_analysis_rows: list[dict[str, object]] | None = None,
    gps_source_comparison_rows: list[dict[str, object]] | None = None,
    video_paths: dict[str, Path] | None = None,
) -> SavedSessionFiles:
    output_dir.mkdir(parents=True, exist_ok=True)
    metadata_output = output_dir / "metadata" / "session_metadata.json"
    metadata_output.parent.mkdir(parents=True, exist_ok=True)
    metadata_payload = dict(metadata)
    metadata_payload.setdefault("session_schema_version", SESSION_SCHEMA_VERSION)
    for key, value in evidence_contract_metadata().items():
        metadata_payload.setdefault(key, value)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=metadata_output.parent,
            prefix=f".{metadata_output.name}.",
            suffix=".tmp",
            delete=False,
        ) as handle:
            temporary_path = Path(handle.name)
            json.dump(metadata_payload, handle, indent=2, ensure_ascii=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_path, metadata_output)
        temporary_path = None
    finally:
        if temporary_path is not None:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass
    distance_output = None
    if distance_rows:
        distance_output = output_dir / "measurements" / "distance_history.csv"
        write_dynamic_csv(distance_output, distance_rows)
    external_output = None
    if external_parameter_rows:
        external_output = output_dir / "alignment" / "external_parameters.csv"
        write_dynamic_csv(external_output, external_parameter_rows)
    mag_output = None
    if pixhawk_mag_rows:
        mag_output = output_dir / "alignment" / "pixhawk_mag_status.csv"
        write_dynamic_csv(mag_output, pixhawk_mag_rows)
    uam_analysis_output = None
    if uam_analysis_rows:
        uam_analysis_output = output_dir / "measurements" / "uam_analysis.csv"
        write_dynamic_csv(uam_analysis_output, uam_analysis_rows)
    gps_source_comparison_output = None
    if gps_source_comparison_rows:
        gps_source_comparison_output = output_dir / "alignment" / "gps_source_comparison.csv"
        write_dynamic_csv(gps_source_comparison_output, gps_source_comparison_rows)
    return SavedSessionFiles(
        metadata=metadata_output,
        distance_history=distance_output,
        external_parameters=external_output,
        pixhawk_mag_status=mag_output,
        uam_analysis=uam_analysis_output,
        gps_source_comparison=gps_source_comparison_output,
        video_outputs=existing_video_outputs(video_paths),
    )
