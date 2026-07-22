"""Sealed Calibration/Validation qualification for Simulation correction profiles.

This module is ROS-free.  It consumes finalized R2 atomic records through the
R3 artifact contract, fits bounded ray-rotation candidates using Calibration
only, scores them using Validation only, and freezes a plan for a later sealed
evaluation.  Public positions are always ``baseline_center_frd``
(Forward, Right, Down); camera rays are always optical RDF (Right, Down,
Forward).
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from datetime import datetime, timezone
from decimal import Decimal
import json
import math
import os
from pathlib import Path, PurePosixPath
import stat
import subprocess
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET

import numpy as np

from sasl_common.stereo_geometry import (
    BASELINE_CENTER_FRD_AXIS_CONTRACT,
    BASELINE_CENTER_FRD_FRAME,
    BASELINE_CENTER_FRD_VECTOR_CONTRACT,
    GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    IDENTITY_ROTATION,
    PinholeIntrinsics,
    normalize_vector,
    ray_from_pixel,
    rotate_vector,
    triangulate_rays,
)

from .coordinate_frames import (
    BASELINE_CENTER_FRD_FRAME as COORDINATE_BASELINE_CENTER_FRD_FRAME,
    REFERENCE_OFFSET_PARENT_M,
    REFERENCE_OFFSET_PARENT_RPY_RAD,
    SOURCE_GAZEBO_ENTITY,
    VEHICLE_REFERENCE_DEFINITION,
    VEHICLE_REFERENCE_LINK,
    WORLD_FRAME,
    build_baseline_center_frd,
    nominal_camera_poses,
)
from .correction_sweep import (
    ATTITUDE_AXES,
    CameraContract,
    S2_SWEEP_SCHEMA_VERSION,
    analysis_window,
    atomic_write_json,
    build_sweep_plan,
    canonical_json,
    compute_spatial_validation,
    parse_attitude_axis_range,
    percentile,
    sha256_bytes,
    sha256_file,
    validate_r2_artifacts,
    write_plan,
)
from .gazebo_ground_truth_adapter import OUTPUT_TOPIC, POSE_TOPIC
from .research_recording import PAIR_RECORD_COLUMNS


S2_QUALIFICATION_SCHEMA_VERSION = "s2_sealed_qualification_v1"
DATASET_ROLES = ("calibration", "validation", "sealed_evaluation")
QUALIFICATION_STATES = (
    "PLANS_PREPARED",
    "CALIBRATION_VALIDATION_READY",
    "PROFILE_FROZEN",
    "SEALED_OPENED",
    "SEALED_EVALUATED",
    "INVALID",
)
CANDIDATE_FAMILIES = (
    "raw_nominal",
    "shared_wahba_so3",
    "dual_camera_wahba_so3",
)
COMPLEXITY_RANK = {
    "raw_nominal": 0,
    "shared_wahba_so3": 1,
    "dual_camera_wahba_so3": 2,
}
SELECTION_RULE = (
    "passing families only; lexicographic nonzero_profile_macro_median, "
    "nonzero_profile_worst_p99, zero_profile_p99; complexity rank only "
    "when compared metrics differ by at most 1e-12"
)
PROFILE_CATALOG_ROLE = "simulation_method_qualification_only"
MATRIX_DIRECTION = "R_Cnominal_Cactual maps actual optical RDF rays to nominal optical RDF"
ANGLE_TIE_TOLERANCE = 1.0e-12
IDENTITY_MATRIX = tuple(tuple(float(value) for value in row) for row in IDENTITY_ROTATION)
LEDGER_STAGES = (
    "prepare",
    "fit",
    "validation_score",
    "freeze",
    "unseal",
    "sealed_evaluation",
)
FREEZE_ANCHOR_SCHEMA_VERSION = "s2_r4a_freeze_anchor_v1"
FREEZE_ANCHOR_REPOSITORY_PATH = "Simulation/config/s2_r4a_freeze_anchor.json"
EVALUATION_REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
MAX_VERIFIED_FILE_BYTES = 64 * 1024 * 1024
_FREEZE_ANCHOR_KEYS = {
    "schema_version",
    "qualification_id",
    "qualification_source",
    "selected_profile",
    "dataset_contract",
    "artifact_sha256",
    "qualification_input_sha256",
    "ground_truth_provenance",
    "coordinate_contract_sources",
}
_QUALIFICATION_SOURCE_KEYS = {"repository_name", "branch", "commit_sha"}
_SELECTED_PROFILE_KEYS = {
    "method_family",
    "profile_count",
    "output_frame",
    "output_axes",
    "position_unit",
    "matrix_direction_convention",
    "quaternion_order",
}
_DATASET_CONTRACT_KEYS = {
    "calibration_case_count",
    "validation_case_count",
    "sealed_case_count",
    "calibration_run_metadata_count",
    "validation_run_metadata_count",
    "warmup_eligible_samples",
    "analysis_eligible_samples",
    "split_unit",
    "dataset_role_overlap_count",
    "sealed_runtime_access_count_at_freeze",
    "sealed_metrics_present_at_freeze",
}
_ARTIFACT_PATHS = {
    "qualification_state.json",
    "frozen_selection_manifest.json",
    "frozen_correction_catalog.json",
    "sealed_evaluation_commitment.json",
    "sealed_plan/sealed_sweep_plan.json",
    "sealed_plan/sealed_sweep_cases.jsonl",
    "data_access_ledger.jsonl",
    "qualification_plan.json",
    "candidate_family_results.json",
    "candidate_search_plan.json",
    "validation_scorecard.csv",
    "calibration_plan/sweep_plan.json",
    "calibration_plan/sweep_cases.jsonl",
    "validation_plan/sweep_plan.json",
    "validation_plan/sweep_cases.jsonl",
}
_QUALIFICATION_INPUT_KEYS = {
    "calibration_sweep_summary",
    "validation_sweep_summary",
    "candidate_search_code",
}
_GROUND_TRUTH_PROVENANCE_KEYS = {
    "semantic_reference_name",
    "semantic_reference_definition",
    "semantic_reference_kind",
    "target_reference_link_present",
    "gazebo_source_entity",
    "parent_model",
    "parent_link",
    "reference_translation_offset_m",
    "reference_rotation_offset_rpy_rad",
    "raw_pose_topic",
    "output_gt_topic",
    "world_frame",
    "fallback_policy",
    "fallback_used_for_qualification",
    "px4_fallback_used",
    "output_frame",
    "output_origin",
    "output_axes",
    "position_unit",
    "quaternion_order",
}
_COORDINATE_CONTRACT_SOURCE_PATHS = {
    "Simulation/stereo_ground_sim/stereo_ground_sim/coordinate_frames.py",
    "Simulation/stereo_ground_sim/stereo_ground_sim/gazebo_ground_truth_adapter.py",
    "Simulation/stereo_ground_sim/models/x500_15m/model.sdf",
    "Simulation/stereo_ground_sim/launch/stereo_ground_sim.launch.py",
    "Simulation/stereo_ground_sim/stereo_ground_sim/research_recording.py",
    "Simulation/stereo_ground_sim/stereo_ground_sim/stereo_distance_node.py",
}
_LEDGER_KEYS = {
    "artifact_hash",
    "canonical_path",
    "operation",
    "process_pid",
    "role",
    "sequence",
    "stage",
    "utc",
}
_FROZEN_LEDGER_OPERATIONS = {
    "read_r2_artifact",
    "read_sweep_artifact",
    "read_precommitted_role_plan",
    "read_sealed_plan",
    "read_commitment",
    "read_qualification_plan",
    "check_sweep_valid_marker",
}
_R2_ARTIFACT_NAMES = {
    "run_metadata.json",
    "pair_records.jsonl",
    "pair_records.csv",
    "run_summary.json",
    "RUN_FINALIZED_VALID",
}
_R2_CASE_LEVEL_ARTIFACT_NAMES = _R2_ARTIFACT_NAMES - {"run_metadata.json"}
_FROZEN_MARKERS = {
    "QUALIFICATION_PROFILE_FROZEN",
    "SEALED_EVALUATION_NOT_EXECUTED",
}
_FORBIDDEN_SEALED_RUNTIME_NAMES = {
    "pair_records.jsonl",
    "pair_records.csv",
    "run_summary.json",
    "RUN_FINALIZED_VALID",
    "RUN_FINALIZED_INVALID",
    "RUN_INCOMPLETE",
    "SEALED_RUNNING",
    "SEALED_INTERRUPTED",
    "SEALED_FAILED",
    "SEALED_COMPLETED",
}


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _require_sha(value: str) -> str:
    result = _require_exact_str(value, "commit_sha")
    if len(result) != 40 or any(char not in "0123456789abcdef" for char in result):
        raise ValueError("commit SHA must be 40 lowercase hexadecimal characters")
    return result


def _finite(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _vector3(values: Sequence[float], name: str) -> tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError(f"{name} must contain three values")
    return tuple(_finite(value, name) for value in values)  # type: ignore[return-value]


def _matrix3(values: Sequence[Sequence[float]], name: str) -> tuple[tuple[float, ...], ...]:
    if len(values) != 3 or any(len(row) != 3 for row in values):
        raise ValueError(f"{name} must be a 3x3 matrix")
    matrix = tuple(tuple(_finite(value, name) for value in row) for row in values)
    array = np.asarray(matrix, dtype=float)
    determinant = float(np.linalg.det(array))
    orthonormality = float(np.max(np.abs(array.T @ array - np.eye(3))))
    if abs(determinant - 1.0) > 1.0e-10 or orthonormality > 1.0e-10:
        raise ValueError(f"{name} must be a proper orthonormal rotation")
    return matrix


def _canonical_file_bytes(records: Sequence[Mapping[str, Any]]) -> bytes:
    return "".join(canonical_json(dict(record)) + "\n" for record in records).encode("utf-8")


@dataclass(frozen=True)
class QualificationSpec:
    qualification_id: str
    repository_name: str
    branch: str
    commit_sha: str
    camera_contract: CameraContract = field(default_factory=CameraContract)
    warmup_eligible_samples: int = 10
    analysis_eligible_samples: int = 30
    seed: int = 20260722
    repetitions: int = 1
    spatial_range_tolerance_m: float = 0.0001
    spatial_angle_tolerance_deg: float = 0.00001

    def validate(self) -> None:
        for name, value in (
            ("qualification_id", self.qualification_id),
            ("repository_name", self.repository_name),
            ("branch", self.branch),
        ):
            if not str(value).strip():
                raise ValueError(f"{name} must be non-empty")
        _require_sha(self.commit_sha)
        self.camera_contract.validate()
        if self.warmup_eligible_samples < 0 or self.analysis_eligible_samples <= 0:
            raise ValueError("sample window is invalid")
        if self.repetitions != 1:
            raise ValueError("sealed qualification requires exactly one repetition")


@dataclass(frozen=True)
class CalibrationRepresentative:
    profile_key: str
    case_id: str
    role: str
    left_pixel_px: tuple[float, float]
    right_pixel_px: tuple[float, float]
    gt_frd_m: tuple[float, float, float]
    camera_contract: CameraContract
    source_attitude_deg: tuple[float, ...] = ()
    analysis_sample_count: int = 30

    def __post_init__(self) -> None:
        if self.role != "calibration":
            raise ValueError("CalibrationRepresentative role must be calibration")
        if not self.profile_key or not self.case_id:
            raise ValueError("representative identity must be non-empty")
        for name, values in (
            ("left_pixel_px", self.left_pixel_px),
            ("right_pixel_px", self.right_pixel_px),
        ):
            if len(values) != 2:
                raise ValueError(f"{name} must contain two values")
            tuple(_finite(value, name) for value in values)
        _vector3(self.gt_frd_m, "gt_frd_m")
        self.camera_contract.validate()
        if self.analysis_sample_count != 30:
            raise ValueError("Calibration representative requires exactly 30 samples")


@dataclass(frozen=True)
class ValidationCase:
    profile_key: str
    case_id: str
    role: str
    records: tuple[Mapping[str, Any], ...]
    camera_contract: CameraContract
    source_attitude_deg: tuple[float, ...] = ()

    def __post_init__(self) -> None:
        if self.role != "validation":
            raise ValueError("ValidationCase role must be validation")
        if len(self.records) != 30:
            raise ValueError("Validation case requires exactly 30 analysis records")
        self.camera_contract.validate()


@dataclass(frozen=True)
class WahbaObservation:
    source_ray_actual_rdf: tuple[float, float, float]
    target_ray_nominal_rdf: tuple[float, float, float]
    weight: float
    case_id: str
    camera: str

    def __post_init__(self) -> None:
        normalize_vector(self.source_ray_actual_rdf)
        normalize_vector(self.target_ray_nominal_rdf)
        if not math.isfinite(float(self.weight)) or self.weight <= 0.0:
            raise ValueError("Wahba weight must be positive and finite")
        if self.camera not in {"left", "right"}:
            raise ValueError("Wahba camera must be left or right")


@dataclass(frozen=True)
class WahbaFitResult:
    rotation_cnominal_from_cactual: tuple[tuple[float, ...], ...]
    support_count: int
    case_count: int
    singular_values: tuple[float, float, float]
    determinant: float
    orthonormality_error: float
    residual_rms_rad: float
    status: str = "fitted"


@dataclass(frozen=True)
class CandidateProfile:
    method_family: str
    profile_key: str
    left_rotation_cnominal_from_cactual: tuple[tuple[float, ...], ...]
    right_rotation_cnominal_from_cactual: tuple[tuple[float, ...], ...]
    left_fit: WahbaFitResult | None = None
    right_fit: WahbaFitResult | None = None
    shared_fit: WahbaFitResult | None = None

    def __post_init__(self) -> None:
        if self.method_family not in CANDIDATE_FAMILIES:
            raise ValueError("unknown candidate family")
        _matrix3(self.left_rotation_cnominal_from_cactual, "left rotation")
        _matrix3(self.right_rotation_cnominal_from_cactual, "right rotation")


@dataclass(frozen=True)
class CandidatePrediction:
    computation_status: str
    forward_m: float | None
    right_m: float | None
    down_m: float | None
    range_m: float | None
    ray_gap_m: float | None
    intersection_angle_deg: float | None
    left_depth_m: float | None
    right_depth_m: float | None
    output_frame: str = BASELINE_CENTER_FRD_FRAME
    position_unit: str = "m"


class DataAccessLedger:
    """Append-only record of every qualification artifact read."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.sequence = len(self.path.read_text(encoding="utf-8").splitlines()) if self.path.exists() else 0

    def record(self, *, stage: str, role: str, path: Path, operation: str) -> dict[str, Any]:
        if stage not in LEDGER_STAGES or role not in DATASET_ROLES:
            raise ValueError("invalid access-ledger stage or role")
        target = Path(path).resolve()
        digest = sha256_file(target) if target.is_file() else None
        self.sequence += 1
        entry = {
            "sequence": self.sequence,
            "stage": stage,
            "role": role,
            "canonical_path": str(target),
            "artifact_hash": digest,
            "operation": str(operation),
            "utc": _utc_now(),
            "process_pid": os.getpid(),
        }
        with self.path.open("a", encoding="utf-8", newline="") as handle:
            handle.write(canonical_json(entry) + "\n")
            handle.flush()
            os.fsync(handle.fileno())
        return entry

    def read_json(self, path: Path, *, stage: str, role: str) -> Any:
        self.record(stage=stage, role=role, path=path, operation="read_json")
        return json.loads(Path(path).read_text(encoding="utf-8"), parse_constant=_reject_json_constant)


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"invalid JSON constant: {value}")


def _strict_json_object(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"duplicate_json_key: {key}")
        value[key] = item
    return value


@dataclass(frozen=True)
class VerifiedFileBytes:
    canonical_path: Path
    raw_bytes: bytes
    sha256: str
    stat_identity: tuple[int, int, int, int]


def _stat_identity(value: os.stat_result) -> tuple[int, int, int, int]:
    return (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns)


def _reject_symlink_components(path: Path, *, name: str) -> None:
    candidate = Path(path)
    if not candidate.is_absolute():
        raise ValueError(f"{name}_absolute_path_required")
    current = Path(candidate.anchor)
    for part in candidate.parts[1:]:
        current /= part
        try:
            mode = current.stat(follow_symlinks=False).st_mode
        except FileNotFoundError as exc:
            raise ValueError(f"{name}_missing: {candidate}") from exc
        if stat.S_ISLNK(mode):
            raise ValueError(f"{name}_symlink_rejected: {candidate}")


def _read_verified_absolute_file(path: Path, *, name: str) -> VerifiedFileBytes:
    candidate = Path(path)
    _reject_symlink_components(candidate, name=name)
    flags = os.O_RDONLY
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(candidate, flags)
    except (FileNotFoundError, OSError) as exc:
        raise ValueError(f"{name}_open_failed: {candidate}") from exc
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise ValueError(f"{name}_nonregular_file: {candidate}")
        if before.st_size > MAX_VERIFIED_FILE_BYTES:
            raise ValueError(f"{name}_file_too_large: {candidate}")
        with os.fdopen(descriptor, "rb", closefd=False) as handle:
            raw_bytes = handle.read(MAX_VERIFIED_FILE_BYTES + 1)
        if len(raw_bytes) > MAX_VERIFIED_FILE_BYTES:
            raise ValueError(f"{name}_file_too_large: {candidate}")
        after = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    try:
        path_after = candidate.stat(follow_symlinks=False)
    except FileNotFoundError as exc:
        raise ValueError(f"{name}_changed_during_read: {candidate}") from exc
    identity = _stat_identity(before)
    if (
        identity != _stat_identity(after)
        or identity != _stat_identity(path_after)
        or len(raw_bytes) != before.st_size
    ):
        raise ValueError(f"{name}_changed_during_read: {candidate}")
    return VerifiedFileBytes(
        canonical_path=candidate.resolve(strict=True),
        raw_bytes=raw_bytes,
        sha256=sha256_bytes(raw_bytes),
        stat_identity=identity,
    )


def _parse_verified_json(snapshot: VerifiedFileBytes, *, name: str) -> dict[str, Any]:
    try:
        text = snapshot.raw_bytes.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError(f"{name}_utf8_invalid") from exc
    value = json.loads(
        text,
        parse_constant=_reject_json_constant,
        object_pairs_hook=_strict_json_object,
    )
    if type(value) is not dict:
        raise ValueError(f"{name}_json_object_required")
    return value


def _parse_verified_jsonl(
    snapshot: VerifiedFileBytes, *, name: str
) -> list[dict[str, Any]]:
    try:
        text = snapshot.raw_bytes.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError(f"{name}_utf8_invalid") from exc
    records = []
    for line_number, line in enumerate(text.splitlines(), start=1):
        if not line:
            raise ValueError(f"{name}_blank_jsonl_line: {line_number}")
        value = json.loads(
            line,
            parse_constant=_reject_json_constant,
            object_pairs_hook=_strict_json_object,
        )
        if type(value) is not dict:
            raise ValueError(f"{name}_jsonl_object_required: {line_number}")
        records.append(value)
    return records


def _load_strict_json(path: Path) -> dict[str, Any]:
    return _parse_verified_json(
        _read_verified_absolute_file(Path(path).resolve(strict=True), name="json"),
        name="json",
    )


def _require_exact_keys(value: Any, expected: set[str], name: str) -> dict[str, Any]:
    if type(value) is not dict or set(value) != expected:
        raise ValueError(f"{name}_schema_mismatch")
    return value


def _require_exact_str(value: Any, name: str) -> str:
    if type(value) is not str:
        raise ValueError(f"{name}_exact_str_required")
    return value


def _require_exact_int(value: Any, name: str) -> int:
    if type(value) is not int:
        raise ValueError(f"{name}_exact_int_required")
    return value


def _require_exact_bool(value: Any, name: str) -> bool:
    if type(value) is not bool:
        raise ValueError(f"{name}_exact_bool_required")
    return value


def _require_exact_number_list(value: Any, name: str) -> list[float]:
    if type(value) is not list or len(value) != 3 or any(type(item) is not float for item in value):
        raise ValueError(f"{name}_exact_number_list_required")
    if any(not math.isfinite(item) for item in value):
        raise ValueError(f"{name}_finite_number_list_required")
    return value


def _require_sha256(value: Any, name: str) -> str:
    if type(value) is not str:
        raise ValueError(f"{name}_sha256_invalid")
    result = value
    if len(result) != 64 or any(char not in "0123456789abcdef" for char in result):
        raise ValueError(f"{name}_sha256_invalid")
    return result


def _canonical_relative_path(value: str, *, name: str) -> str:
    text = _require_exact_str(value, name)
    path = PurePosixPath(text)
    if not text or "\\" in text or path.is_absolute() or any(part in {"", ".", ".."} for part in path.parts):
        raise ValueError(f"{name}_path_invalid")
    canonical = path.as_posix()
    if canonical != text:
        raise ValueError(f"{name}_path_not_canonical")
    return canonical


def load_freeze_anchor(path: Path) -> dict[str, Any]:
    anchor_path = Path(path)
    if not anchor_path.is_absolute():
        anchor_path = anchor_path.absolute()
    try:
        snapshot = _read_verified_absolute_file(anchor_path, name="freeze_anchor")
    except ValueError as exc:
        if not anchor_path.exists():
            raise ValueError("freeze_anchor_missing") from exc
        raise
    return validate_freeze_anchor(_parse_verified_json(snapshot, name="freeze_anchor"))


def validate_freeze_anchor(anchor: Mapping[str, Any]) -> dict[str, Any]:
    value = _require_exact_keys(dict(anchor), _FREEZE_ANCHOR_KEYS, "freeze_anchor")
    if _require_exact_str(value["schema_version"], "schema_version") != FREEZE_ANCHOR_SCHEMA_VERSION:
        raise ValueError("freeze_anchor_schema_version_unknown")
    if _require_exact_str(value["qualification_id"], "qualification_id") != "s2-r4a-ofat-12axis-v1":
        raise ValueError("freeze_anchor_qualification_id_mismatch")

    source = _require_exact_keys(
        value["qualification_source"], _QUALIFICATION_SOURCE_KEYS, "qualification_source"
    )
    for field in _QUALIFICATION_SOURCE_KEYS:
        _require_exact_str(source[field], f"qualification_source:{field}")
    if source != {
        "repository_name": "goodbuilder/SASL",
        "branch": "sim/s2-r4a-sealed-profile-freeze",
        "commit_sha": "88a360f6023f00517106b9f9915116b10fd5c311",
    }:
        raise ValueError("qualification_source_mismatch")
    _require_sha(source["commit_sha"])

    selected = _require_exact_keys(
        value["selected_profile"], _SELECTED_PROFILE_KEYS, "selected_profile"
    )
    _require_exact_int(selected["profile_count"], "selected_profile:profile_count")
    for field in _SELECTED_PROFILE_KEYS - {"profile_count"}:
        _require_exact_str(selected[field], f"selected_profile:{field}")
    expected_selected = {
        "method_family": "dual_camera_wahba_so3",
        "profile_count": 25,
        "output_frame": BASELINE_CENTER_FRD_FRAME,
        "output_axes": "forward_right_down",
        "position_unit": "m",
        "matrix_direction_convention": MATRIX_DIRECTION,
        "quaternion_order": "xyzw",
    }
    if selected != expected_selected:
        raise ValueError("selected_profile_contract_mismatch")

    dataset = _require_exact_keys(
        value["dataset_contract"], _DATASET_CONTRACT_KEYS, "dataset_contract"
    )
    for field in _DATASET_CONTRACT_KEYS - {"split_unit", "sealed_metrics_present_at_freeze"}:
        _require_exact_int(dataset[field], f"dataset_contract:{field}")
    _require_exact_str(dataset["split_unit"], "dataset_contract:split_unit")
    _require_exact_bool(
        dataset["sealed_metrics_present_at_freeze"],
        "dataset_contract:sealed_metrics_present_at_freeze",
    )
    expected_dataset = {
        "calibration_case_count": 450,
        "validation_case_count": 225,
        "sealed_case_count": 225,
        "calibration_run_metadata_count": 450,
        "validation_run_metadata_count": 225,
        "warmup_eligible_samples": 10,
        "analysis_eligible_samples": 30,
        "split_unit": "spatial_cell_group",
        "dataset_role_overlap_count": 0,
        "sealed_runtime_access_count_at_freeze": 0,
        "sealed_metrics_present_at_freeze": False,
    }
    if dataset != expected_dataset:
        raise ValueError("dataset_contract_mismatch")

    artifact_hashes = value["artifact_sha256"]
    if type(artifact_hashes) is not dict or set(artifact_hashes) != _ARTIFACT_PATHS:
        raise ValueError("artifact_sha256_schema_mismatch")
    canonical_paths = []
    for path, digest in artifact_hashes.items():
        canonical_paths.append(_canonical_relative_path(path, name="artifact"))
        _require_sha256(digest, f"artifact:{path}")
    if len(canonical_paths) != len(set(canonical_paths)):
        raise ValueError("artifact_path_duplicate")

    inputs = _require_exact_keys(
        value["qualification_input_sha256"],
        _QUALIFICATION_INPUT_KEYS,
        "qualification_input_sha256",
    )
    for name, digest in inputs.items():
        _require_sha256(digest, f"qualification_input:{name}")

    ground_truth = _require_exact_keys(
        value["ground_truth_provenance"],
        _GROUND_TRUTH_PROVENANCE_KEYS,
        "ground_truth_provenance",
    )
    for field in _GROUND_TRUTH_PROVENANCE_KEYS - {
        "target_reference_link_present",
        "fallback_used_for_qualification",
        "px4_fallback_used",
        "reference_translation_offset_m",
        "reference_rotation_offset_rpy_rad",
    }:
        _require_exact_str(ground_truth[field], f"ground_truth_provenance:{field}")
    for field in (
        "target_reference_link_present",
        "fallback_used_for_qualification",
        "px4_fallback_used",
    ):
        _require_exact_bool(ground_truth[field], f"ground_truth_provenance:{field}")
    _require_exact_number_list(
        ground_truth["reference_translation_offset_m"],
        "ground_truth_provenance:reference_translation_offset_m",
    )
    _require_exact_number_list(
        ground_truth["reference_rotation_offset_rpy_rad"],
        "ground_truth_provenance:reference_rotation_offset_rpy_rad",
    )
    expected_ground_truth = {
        "semantic_reference_name": "vehicle_reference_link",
        "semantic_reference_definition": "project_defined_vehicle_reference_at_base_link_origin",
        "semantic_reference_kind": "project_defined_alternative_reference",
        "target_reference_link_present": False,
        "gazebo_source_entity": "base_link",
        "parent_model": "x500_15m",
        "parent_link": "base_link",
        "reference_translation_offset_m": [0.0, 0.0, 0.0],
        "reference_rotation_offset_rpy_rad": [0.0, 0.0, 0.0],
        "raw_pose_topic": "/world/stereo_ground_drone/pose/info",
        "output_gt_topic": "/simulation/ground_truth/vehicle_reference_link",
        "world_frame": "gazebo_world_enu",
        "fallback_policy": "fail_closed",
        "fallback_used_for_qualification": False,
        "px4_fallback_used": False,
        "output_frame": BASELINE_CENTER_FRD_FRAME,
        "output_origin": "nominal_left_right_optical_center_midpoint",
        "output_axes": "forward_right_down",
        "position_unit": "m",
        "quaternion_order": "xyzw",
    }
    if ground_truth != expected_ground_truth:
        raise ValueError("ground_truth_provenance_mismatch")
    sources = _require_exact_keys(
        value["coordinate_contract_sources"],
        {"hash_algorithm", "files"},
        "coordinate_contract_sources",
    )
    if _require_exact_str(sources["hash_algorithm"], "coordinate_contract_hash_algorithm") != "sha256":
        raise ValueError("coordinate_contract_hash_algorithm_mismatch")
    files = sources["files"]
    if type(files) is not dict or set(files) != _COORDINATE_CONTRACT_SOURCE_PATHS:
        raise ValueError("coordinate_contract_sources_schema_mismatch")
    source_paths = []
    for path, digest in files.items():
        source_paths.append(_canonical_relative_path(path, name="coordinate_source"))
        _require_sha256(digest, f"coordinate_source:{path}")
    if len(source_paths) != len(set(source_paths)):
        raise ValueError("coordinate_source_path_duplicate")
    return value


def _attitude_values(value: Mapping[str, Any]) -> tuple[float, ...]:
    source = value.get("attitude_source_deg", value)
    parsed = []
    for axis in ATTITUDE_AXES:
        item = source[axis]
        if isinstance(item, Mapping):
            item = item.get("decimal", item.get("value"))
        parsed.append(_finite(item, axis))
    return tuple(parsed)


def profile_key(attitude: Mapping[str, Any] | Sequence[float]) -> str:
    values = _attitude_values(attitude) if isinstance(attitude, Mapping) else tuple(_finite(v, "attitude") for v in attitude)
    if len(values) != len(ATTITUDE_AXES):
        raise ValueError("attitude profile must contain 12 values")
    payload = {
        axis: "0" if value == 0.0 else format(Decimal(str(value)).normalize(), "f")
        for axis, value in zip(ATTITUDE_AXES, values)
    }
    return "profile-" + sha256_bytes(canonical_json(payload).encode("utf-8"))[:16]


def spatial_group_key(value: Mapping[str, Any] | Sequence[float]) -> str:
    if isinstance(value, Mapping):
        spatial = value.get("spatial", value)
        parsed = []
        for name in ("range_m", "bearing_deg", "elevation_deg"):
            item = spatial[name]
            if isinstance(item, Mapping):
                item = item.get("decimal", item.get("value"))
            parsed.append(_finite(item, name))
    else:
        parsed = list(_vector3(value, "spatial cell"))
    payload = {
        name: "0" if number == 0.0 else format(Decimal(str(number)).normalize(), "f")
        for name, number in zip(("range_m", "bearing_deg", "elevation_deg"), parsed)
    }
    return "spatial-" + sha256_bytes(canonical_json(payload).encode("utf-8"))[:16]


def _axis_ranges() -> tuple[tuple[str, tuple[Decimal, ...]], ...]:
    return tuple(parse_attitude_axis_range(f"{axis}:-0.1:0.1:0.2") for axis in ATTITUDE_AXES)


def _role_plan(spec: QualificationSpec, role: str) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    ranges = {
        "calibration": (Decimal("300"), Decimal("1000")),
        "validation": (Decimal("500"),),
        "sealed_evaluation": (Decimal("1852"),),
    }[role]
    return build_sweep_plan(
        sweep_id=f"{spec.qualification_id}-{role}",
        commit_sha=spec.commit_sha,
        branch=spec.branch,
        repository_name=spec.repository_name,
        attitude_mode="ofat",
        axis_ranges=_axis_ranges(),
        range_values=ranges,
        bearing_values=(Decimal("-1"), Decimal("0"), Decimal("1")),
        elevation_values=(Decimal("-0.5"), Decimal("0"), Decimal("0.5")),
        repetitions=spec.repetitions,
        seed=spec.seed,
        case_order="canonical",
        invalid_spatial_policy="fail",
        max_cases=500,
        contract=spec.camera_contract,
        spatial_range_tolerance_m=spec.spatial_range_tolerance_m,
        spatial_angle_tolerance_deg=spec.spatial_angle_tolerance_deg,
    )


def build_qualification_bundle(spec: QualificationSpec) -> dict[str, Any]:
    spec.validate()
    roles: dict[str, Any] = {}
    profile_sets: dict[str, set[str]] = {}
    spatial_sets: dict[str, set[str]] = {}
    case_sets: dict[str, set[str]] = {}
    for role in DATASET_ROLES:
        plan, cases = _role_plan(spec, role)
        profiles = {profile_key(case["payload"]) for case in cases}
        spatials = {spatial_group_key(case["payload"]) for case in cases}
        profile_sets[role] = profiles
        spatial_sets[role] = spatials
        case_sets[role] = {case["case_id"] for case in cases}
        roles[role] = {"plan": plan, "cases": cases}
    if any(spatial_sets[a] & spatial_sets[b] for index, a in enumerate(DATASET_ROLES) for b in DATASET_ROLES[index + 1 :]):
        raise ValueError("dataset roles overlap by spatial group")
    if any(case_sets[a] & case_sets[b] for index, a in enumerate(DATASET_ROLES) for b in DATASET_ROLES[index + 1 :]):
        raise ValueError("dataset roles overlap by case ID")
    if any(len(profile_sets[role]) != 25 for role in DATASET_ROLES):
        raise ValueError("each role must contain 25 attitude profiles")
    sealed_cases = roles["sealed_evaluation"]["cases"]
    sealed_bytes = _canonical_file_bytes(sealed_cases)
    commitment_core = {
        "schema_version": S2_QUALIFICATION_SCHEMA_VERSION,
        "sweep_schema_version": S2_SWEEP_SCHEMA_VERSION,
        "qualification_id": spec.qualification_id,
        "branch": spec.branch,
        "commit_sha": spec.commit_sha,
        "sealed_plan_hash": roles["sealed_evaluation"]["plan"]["plan_hash"],
        "sealed_cases_file_sha256": sha256_bytes(sealed_bytes),
        "case_count": len(sealed_cases),
        "case_id_ordered_hash": sha256_bytes(canonical_json([case["case_id"] for case in sealed_cases]).encode("utf-8")),
        "profile_key_ordered_hash": sha256_bytes(canonical_json([profile_key(case["payload"]) for case in sealed_cases]).encode("utf-8")),
        "spatial_group_ordered_hash": sha256_bytes(canonical_json([spatial_group_key(case["payload"]) for case in sealed_cases]).encode("utf-8")),
        "camera_contract": spec.camera_contract.payload(),
        "target_physics_contract": {"trajectory_mode": "static_gazebo_reference_point", "target_physics_mode": "static_model", "target_static_for_research": True},
        "expected_warmup_samples": spec.warmup_eligible_samples,
        "expected_analysis_samples": spec.analysis_eligible_samples,
        "expected_output_frame": BASELINE_CENTER_FRD_FRAME,
        "created_before_profile_freeze": True,
    }
    commitment = {**commitment_core, "commitment_hash": sha256_bytes(canonical_json(commitment_core).encode("utf-8"))}
    plan_core = {
        "schema_version": S2_QUALIFICATION_SCHEMA_VERSION,
        "qualification_id": spec.qualification_id,
        "repository_name": spec.repository_name,
        "branch": spec.branch,
        "commit_sha": spec.commit_sha,
        "dataset_roles": list(DATASET_ROLES),
        "role_plan_hashes": {role: roles[role]["plan"]["plan_hash"] for role in DATASET_ROLES},
        "role_case_counts": {role: len(roles[role]["cases"]) for role in DATASET_ROLES},
        "profile_count": 25,
        "warmup_eligible_samples": spec.warmup_eligible_samples,
        "analysis_eligible_samples": spec.analysis_eligible_samples,
        "split_unit": "spatial_cell_group",
        "selection_rule": SELECTION_RULE,
        "candidate_families": list(CANDIDATE_FAMILIES),
        "camera_contract": spec.camera_contract.payload(),
    }
    qualification_plan = {**plan_core, "plan_hash": sha256_bytes(canonical_json(plan_core).encode("utf-8"))}
    return {"qualification_plan": qualification_plan, "roles": roles, "sealed_commitment": commitment}


def prepare_plans(output_dir: Path, spec: QualificationSpec) -> dict[str, Any]:
    root = Path(output_dir)
    if root.exists() and any(root.iterdir()):
        raise ValueError("qualification output must be new or empty")
    root.mkdir(parents=True, exist_ok=True)
    bundle = build_qualification_bundle(spec)
    atomic_write_json(root / "qualification_plan.json", bundle["qualification_plan"])
    for role in DATASET_ROLES:
        directory = root / ("sealed_plan" if role == "sealed_evaluation" else f"{role}_plan")
        directory.mkdir()
        plan = bundle["roles"][role]["plan"]
        cases = bundle["roles"][role]["cases"]
        if role == "sealed_evaluation":
            atomic_write_json(directory / "sealed_sweep_plan.json", plan)
            (directory / "sealed_sweep_cases.jsonl").write_bytes(_canonical_file_bytes(cases))
        else:
            write_plan(directory, plan, cases)
    atomic_write_json(root / "sealed_evaluation_commitment.json", bundle["sealed_commitment"])
    atomic_write_json(root / "candidate_search_plan.json", {
        "schema_version": S2_QUALIFICATION_SCHEMA_VERSION,
        "candidate_families": list(CANDIDATE_FAMILIES),
        "selection_rule": SELECTION_RULE,
        "validation_gates": validation_gate_contract(),
        "solver_input_allowlist": ["left_u_px", "left_v_px", "right_u_px", "right_v_px", "gt_forward_m", "gt_right_m", "gt_down_m", "baseline_m", "camera_height_m", "horizontal_fov_rad", "image_width_px", "image_height_px", "case_id", "profile_key", "role"],
        "runtime_forbidden_inputs": ["attitude_corrected_*", "effective_injected_rpy", "source_injected_rpy_as_rotation", "sealed_runtime_data"],
    })
    atomic_write_json(root / "qualification_state.json", {"schema_version": S2_QUALIFICATION_SCHEMA_VERSION, "qualification_id": spec.qualification_id, "state": "PLANS_PREPARED", "branch": spec.branch, "commit_sha": spec.commit_sha})
    (root / "QUALIFICATION_PLANS_PREPARED").write_text("\n", encoding="utf-8")
    (root / "SEALED_EVALUATION_NOT_EXECUTED").write_text("\n", encoding="utf-8")
    (root / "data_access_ledger.jsonl").touch()
    return {"qualification_root": str(root.resolve()), "qualification_plan": bundle["qualification_plan"], "sealed_commitment": bundle["sealed_commitment"], "role_case_counts": bundle["qualification_plan"]["role_case_counts"]}


def _case_payload(case: Mapping[str, Any]) -> Mapping[str, Any]:
    return case.get("payload", case)


def _case_contract(case: Mapping[str, Any], fallback: CameraContract | None = None) -> CameraContract:
    payload = _case_payload(case)
    values = payload.get("camera_contract")
    if values is None:
        if fallback is None:
            raise ValueError("case camera contract is missing")
        return fallback
    return CameraContract(**{name: values[name] for name in CameraContract.__dataclass_fields__})


def _median(records: Sequence[Mapping[str, Any]], field: str) -> float:
    values = [_finite(record.get(field), field) for record in records]
    return percentile(values, 50.0)


def _validate_analysis_record(record: Mapping[str, Any], expected_case_id: str) -> None:
    if record.get("sample_eligible") is not True:
        raise ValueError("analysis record must be eligible")
    if record.get("case_id") != expected_case_id:
        raise ValueError("analysis record case ID mismatch")
    if record.get("output_frame") != BASELINE_CENTER_FRD_FRAME:
        raise ValueError("analysis record output frame mismatch")
    if record.get("output_axes") != "forward_right_down" or record.get("position_unit") != "m":
        raise ValueError("analysis record coordinate contract mismatch")
    for name in ("left_u_px", "left_v_px", "right_u_px", "right_v_px", "gt_forward_m", "gt_right_m", "gt_down_m"):
        _finite(record.get(name), name)


def build_calibration_representative(
    records: Sequence[Mapping[str, Any]],
    case: Mapping[str, Any],
    contract: CameraContract | None = None,
    *,
    warmup_count: int = 10,
    analysis_count: int = 30,
) -> CalibrationRepresentative:
    case_id = str(case.get("case_id", ""))
    payload = _case_payload(case)
    window, _ = analysis_window(records, warmup_count=warmup_count, analysis_count=analysis_count)
    if len(window) != 30:
        raise ValueError("Calibration representative requires exactly 30 analysis samples")
    for record in window:
        _validate_analysis_record(record, case_id)
    source = _attitude_values(payload)
    return CalibrationRepresentative(
        profile_key=profile_key(payload),
        case_id=case_id,
        role="calibration",
        left_pixel_px=(_median(window, "left_u_px"), _median(window, "left_v_px")),
        right_pixel_px=(_median(window, "right_u_px"), _median(window, "right_v_px")),
        gt_frd_m=(_median(window, "gt_forward_m"), _median(window, "gt_right_m"), _median(window, "gt_down_m")),
        camera_contract=_case_contract(case, contract),
        source_attitude_deg=source,
        analysis_sample_count=len(window),
    )


def build_validation_case(
    records: Sequence[Mapping[str, Any]],
    case: Mapping[str, Any],
    contract: CameraContract | None = None,
    *,
    warmup_count: int = 10,
    analysis_count: int = 30,
) -> ValidationCase:
    case_id = str(case.get("case_id", ""))
    payload = _case_payload(case)
    window, _ = analysis_window(records, warmup_count=warmup_count, analysis_count=analysis_count)
    if len(window) != 30:
        raise ValueError("Validation case requires exactly 30 analysis samples")
    for record in window:
        _validate_analysis_record(record, case_id)
    return ValidationCase(
        profile_key=profile_key(payload),
        case_id=case_id,
        role="validation",
        records=tuple(window),
        camera_contract=_case_contract(case, contract),
        source_attitude_deg=_attitude_values(payload),
    )


def _intrinsics(contract: CameraContract) -> PinholeIntrinsics:
    fx = (contract.image_width_px * 0.5) / math.tan(contract.horizontal_fov_rad * 0.5)
    return PinholeIntrinsics(fx, fx, contract.image_width_px * 0.5, contract.image_height_px * 0.5)


def build_wahba_observations(representative: CalibrationRepresentative) -> tuple[WahbaObservation, WahbaObservation]:
    nominal_left, nominal_right = nominal_camera_poses(representative.camera_contract.baseline_m, representative.camera_contract.camera_height_m)
    baseline = build_baseline_center_frd(
        nominal_left.center_world_enu_m,
        nominal_right.center_world_enu_m,
        nominal_left.forward_world_enu,
        nominal_right.forward_world_enu,
    )
    point_world = baseline.point_to_world(representative.gt_frd_m)
    intrinsics = _intrinsics(representative.camera_contract)
    observations = []
    for camera, pose, pixel in (
        ("left", nominal_left, representative.left_pixel_px),
        ("right", nominal_right, representative.right_pixel_px),
    ):
        source = ray_from_pixel(pixel[0], pixel[1], intrinsics)
        target = normalize_vector(pose.world_to_rdf(point_world))
        observations.append(WahbaObservation(source, target, 0.5, representative.case_id, camera))
    return tuple(observations)  # type: ignore[return-value]


def _geodesic_rotation_error(left: np.ndarray, right: np.ndarray) -> float:
    relative = left @ right.T
    sine = 0.5 * math.sqrt(
        (relative[2, 1] - relative[1, 2]) ** 2
        + (relative[0, 2] - relative[2, 0]) ** 2
        + (relative[1, 0] - relative[0, 1]) ** 2
    )
    cosine = 0.5 * (float(np.trace(relative)) - 1.0)
    return abs(math.atan2(sine, max(-1.0, min(1.0, cosine))))


def fit_wahba(observations: Sequence[WahbaObservation]) -> WahbaFitResult:
    if len(observations) < 3:
        raise ValueError("insufficient_geometry: at least three observations are required")
    covariance = np.zeros((3, 3), dtype=float)
    cases: set[str] = set()
    normalized: list[tuple[np.ndarray, np.ndarray, float]] = []
    for observation in observations:
        source = np.asarray(normalize_vector(observation.source_ray_actual_rdf), dtype=float)
        target = np.asarray(normalize_vector(observation.target_ray_nominal_rdf), dtype=float)
        weight = _finite(observation.weight, "weight")
        if weight <= 0.0:
            raise ValueError("Wahba weight must be positive")
        covariance += weight * np.outer(target, source)
        normalized.append((source, target, weight))
        cases.add(observation.case_id)
    if not np.all(np.isfinite(covariance)):
        raise ValueError("Wahba covariance must be finite")
    left, singular_values, right_t = np.linalg.svd(covariance)
    tolerance = max(float(singular_values[0]) * 1.0e-12, np.finfo(float).eps * float(singular_values[0]) * len(observations) * 10.0)
    if int(np.sum(singular_values > tolerance)) < 3:
        raise ValueError("degenerate: Wahba observations do not span three dimensions")
    correction = np.eye(3)
    correction[2, 2] = 1.0 if np.linalg.det(left @ right_t) >= 0.0 else -1.0
    rotation = left @ correction @ right_t
    determinant = float(np.linalg.det(rotation))
    orthonormality = float(np.max(np.abs(rotation.T @ rotation - np.eye(3))))
    if abs(determinant - 1.0) > 1.0e-10 or orthonormality > 1.0e-10:
        raise ValueError("Wahba result is not a proper rotation")
    weighted_square = 0.0
    weight_sum = 0.0
    for source, target, weight in normalized:
        cosine = max(-1.0, min(1.0, float(np.dot(rotation @ source, target))))
        weighted_square += weight * math.acos(cosine) ** 2
        weight_sum += weight
    residual = math.sqrt(weighted_square / weight_sum)
    matrix = tuple(tuple(float(value) for value in row) for row in rotation)
    return WahbaFitResult(matrix, len(observations), len(cases), tuple(float(value) for value in singular_values), determinant, orthonormality, residual)


def fit_candidate_families(representatives: Sequence[CalibrationRepresentative]) -> dict[str, dict[str, CandidateProfile]]:
    grouped: dict[str, list[CalibrationRepresentative]] = {}
    for representative in representatives:
        if representative.role != "calibration":
            raise ValueError("candidate fitting accepts Calibration representatives only")
        grouped.setdefault(representative.profile_key, []).append(representative)
    if len(grouped) != 25:
        raise ValueError("candidate fitting requires exactly 25 attitude profiles")
    result = {family: {} for family in CANDIDATE_FAMILIES}
    for key, items in grouped.items():
        if len(items) != 18 or len({item.case_id for item in items}) != 18:
            raise ValueError("each profile requires 18 unique Calibration cases")
        observations = [item for representative in items for item in build_wahba_observations(representative)]
        left_obs = [item for item in observations if item.camera == "left"]
        right_obs = [item for item in observations if item.camera == "right"]
        shared = fit_wahba(observations)
        left_fit = fit_wahba(left_obs)
        right_fit = fit_wahba(right_obs)
        result["raw_nominal"][key] = CandidateProfile("raw_nominal", key, IDENTITY_MATRIX, IDENTITY_MATRIX, None, None)
        result["shared_wahba_so3"][key] = CandidateProfile("shared_wahba_so3", key, shared.rotation_cnominal_from_cactual, shared.rotation_cnominal_from_cactual, None, None, shared)
        result["dual_camera_wahba_so3"][key] = CandidateProfile("dual_camera_wahba_so3", key, left_fit.rotation_cnominal_from_cactual, right_fit.rotation_cnominal_from_cactual, left_fit, right_fit)
    return result


def predict_candidate(
    profile: CandidateProfile,
    left_pixel_px: Sequence[float],
    right_pixel_px: Sequence[float],
    camera_contract: CameraContract,
) -> CandidatePrediction:
    camera_contract.validate()
    if len(left_pixel_px) != 2 or len(right_pixel_px) != 2:
        raise ValueError("candidate pixels must contain u,v")
    nominal_left, nominal_right = nominal_camera_poses(camera_contract.baseline_m, camera_contract.camera_height_m)
    baseline = build_baseline_center_frd(nominal_left.center_world_enu_m, nominal_right.center_world_enu_m, nominal_left.forward_world_enu, nominal_right.forward_world_enu)
    intrinsics = _intrinsics(camera_contract)
    left_source = ray_from_pixel(_finite(left_pixel_px[0], "left_u_px"), _finite(left_pixel_px[1], "left_v_px"), intrinsics)
    right_source = ray_from_pixel(_finite(right_pixel_px[0], "right_u_px"), _finite(right_pixel_px[1], "right_v_px"), intrinsics)
    left_nominal = rotate_vector(profile.left_rotation_cnominal_from_cactual, left_source)
    right_nominal = rotate_vector(profile.right_rotation_cnominal_from_cactual, right_source)
    left_world = nominal_left.ray_to_world(left_nominal)
    right_world = nominal_right.ray_to_world(right_nominal)
    attempt = triangulate_rays(
        left_origin_left_rdf_m=baseline.point_from_world(nominal_left.center_world_enu_m),
        right_origin_left_rdf_m=baseline.point_from_world(nominal_right.center_world_enu_m),
        left_ray_left_rdf=baseline.ray_from_world(left_world),
        right_ray_left_rdf=baseline.ray_from_world(right_world),
        common_frame=BASELINE_CENTER_FRD_FRAME,
        axis_contract=BASELINE_CENTER_FRD_AXIS_CONTRACT,
        vector_contract=BASELINE_CENTER_FRD_VECTOR_CONTRACT,
        geometry_policy=GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    )
    if attempt.result is None:
        return CandidatePrediction(attempt.reason, None, None, None, None, None, None, None, None)
    result = attempt.result
    point = result.midpoint_rdf_m or result.point_left_rdf_m
    return CandidatePrediction(result.computation_status, float(point[0]), float(point[1]), float(point[2]), float(result.range_m), float(result.closest_gap_m), float(result.ray_angle_deg), float(result.left_depth_m), float(result.right_depth_m))


def score_prediction(prediction: CandidatePrediction, gt_frd_m: Sequence[float]) -> dict[str, Any]:
    gt = _vector3(gt_frd_m, "gt_frd_m")
    if prediction.computation_status != "computed" or None in (prediction.forward_m, prediction.right_m, prediction.down_m, prediction.range_m):
        return {"valid": False, "computation_status": prediction.computation_status}
    errors = (prediction.forward_m - gt[0], prediction.right_m - gt[1], prediction.down_m - gt[2])
    gt_range = math.sqrt(sum(value * value for value in gt))
    return {
        "valid": True,
        "computation_status": prediction.computation_status,
        "position_error_m": math.sqrt(sum(value * value for value in errors)),
        "forward_abs_error_m": abs(errors[0]),
        "right_abs_error_m": abs(errors[1]),
        "down_abs_error_m": abs(errors[2]),
        "range_abs_error_m": abs(float(prediction.range_m) - gt_range),
        "ray_gap_m": prediction.ray_gap_m,
        "intersection_angle_deg": prediction.intersection_angle_deg,
    }


def _case_metric(scores: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    valid = [score for score in scores if score.get("valid") is True]
    if len(valid) != len(scores):
        return {"valid": False, "invalid_count": len(scores) - len(valid), "sample_count": len(scores)}
    def values(name: str) -> list[float]:
        return [_finite(score[name], name) for score in valid]
    position = values("position_error_m")
    return {
        "valid": True,
        "sample_count": len(valid),
        "invalid_count": 0,
        "position_error_median": percentile(position, 50.0),
        "position_error_p95": percentile(position, 95.0),
        "position_error_p99": percentile(position, 99.0),
        "position_error_max": max(position),
        "forward_abs_error_median": percentile(values("forward_abs_error_m"), 50.0),
        "right_abs_error_median": percentile(values("right_abs_error_m"), 50.0),
        "down_abs_error_median": percentile(values("down_abs_error_m"), 50.0),
        "range_abs_error_median": percentile(values("range_abs_error_m"), 50.0),
        "ray_gap_median": percentile(values("ray_gap_m"), 50.0),
        "ray_gap_p99": percentile(values("ray_gap_m"), 99.0),
    }


def evaluate_candidate(profile: CandidateProfile, validation_cases: Sequence[ValidationCase]) -> dict[str, Any]:
    cases = [case for case in validation_cases if case.profile_key == profile.profile_key]
    if len(cases) != 9 or len({case.case_id for case in cases}) != 9:
        raise ValueError("candidate evaluation requires nine unique Validation cases")
    case_results = []
    for case in sorted(cases, key=lambda item: item.case_id):
        scores = []
        for record in case.records:
            prediction = predict_candidate(
                profile,
                (_finite(record["left_u_px"], "left_u_px"), _finite(record["left_v_px"], "left_v_px")),
                (_finite(record["right_u_px"], "right_u_px"), _finite(record["right_v_px"], "right_v_px")),
                case.camera_contract,
            )
            scores.append(score_prediction(prediction, (record["gt_forward_m"], record["gt_right_m"], record["gt_down_m"])))
        metric = _case_metric(scores)
        case_results.append({"case_id": case.case_id, **metric})
    valid_cases = [case for case in case_results if case["valid"]]
    profile_valid = len(valid_cases) == 9
    source = cases[0].source_attitude_deg
    if any(case.source_attitude_deg != source for case in cases):
        raise ValueError("Validation profile source attitude is inconsistent")
    return {
        "profile_key": profile.profile_key,
        "method_family": profile.method_family,
        "source_attitude_deg": list(source),
        "zero_profile": all(abs(value) <= 0.0 for value in source),
        "valid": profile_valid,
        "valid_case_count": len(valid_cases),
        "validation_support_count": sum(int(case.get("sample_count", 0)) for case in case_results),
        "case_results": case_results,
        "position_error_median": percentile([case["position_error_median"] for case in valid_cases], 50.0) if valid_cases else None,
        "position_error_p95": percentile([case["position_error_median"] for case in valid_cases], 95.0) if valid_cases else None,
        "worst_case_p99": max((case["position_error_p99"] for case in valid_cases), default=None),
        "worst_case_max": max((case["position_error_max"] for case in valid_cases), default=None),
        "invalid_prediction_count": sum(int(case.get("invalid_count", 0)) for case in case_results),
    }


def validation_gate_contract() -> dict[str, Any]:
    return {
        "required_profile_count": 25,
        "fit_case_support_per_profile": 18,
        "validation_case_support_per_profile": 9,
        "invalid_prediction_count_max": 0,
        "zero_profile_worst_p99_max_m": 1.0e-6,
        "nonzero_profile_median_max_m": 1.0e-4,
        "nonzero_profile_worst_p99_max_m": 1.0e-3,
        "nonzero_candidate_raw_macro_median_ratio_max": 0.01,
        "rotation_orthonormality_error_max": 1.0e-10,
        "rotation_determinant_abs_error_max": 1.0e-10,
    }


def _fit_support_ok(profile: CandidateProfile) -> bool:
    if profile.method_family == "shared_wahba_so3":
        return profile.shared_fit is not None and profile.shared_fit.case_count == 18
    if profile.method_family == "dual_camera_wahba_so3":
        return (
            profile.left_fit is not None
            and profile.right_fit is not None
            and profile.left_fit.case_count == 18
            and profile.right_fit.case_count == 18
        )
    return True


def evaluate_candidate_families(
    candidates: Mapping[str, Mapping[str, CandidateProfile]],
    validation_cases: Sequence[ValidationCase],
) -> dict[str, Any]:
    grouped_keys = {case.profile_key for case in validation_cases}
    if len(grouped_keys) != 25:
        raise ValueError("Validation requires exactly 25 profiles")
    family_results: dict[str, Any] = {}
    for family in CANDIDATE_FAMILIES:
        profiles = candidates.get(family)
        if profiles is None or set(profiles) != grouped_keys:
            raise ValueError(f"candidate family {family} does not cover all profiles")
        evaluated = {key: evaluate_candidate(profiles[key], validation_cases) for key in sorted(grouped_keys)}
        zero = [value for value in evaluated.values() if value["zero_profile"]]
        nonzero = [value for value in evaluated.values() if not value["zero_profile"]]
        if len(zero) != 1 or len(nonzero) != 24:
            raise ValueError("Validation profile partition must contain one zero and 24 nonzero profiles")
        family_results[family] = {
            "method_family": family,
            "profiles": evaluated,
            "zero_profile_p99": zero[0]["worst_case_p99"],
            "nonzero_profile_macro_median": percentile([value["position_error_median"] for value in nonzero], 50.0),
            "nonzero_profile_macro_p95": percentile([value["position_error_median"] for value in nonzero], 95.0),
            "nonzero_profile_worst_p99": max(value["worst_case_p99"] for value in nonzero),
            "profile_pass_count": 0,
            "invalid_prediction_count": sum(value["invalid_prediction_count"] for value in evaluated.values()),
            "invalid_rate": sum(value["invalid_prediction_count"] for value in evaluated.values()) / (25 * 9 * 30),
        }
    raw_macro = family_results["raw_nominal"]["nonzero_profile_macro_median"]
    gate = validation_gate_contract()
    for family, result in family_results.items():
        result["corrected_raw_macro_median_ratio"] = (
            result["nonzero_profile_macro_median"] / raw_macro if raw_macro > 0.0 else math.inf
        )
        passes = []
        for key, profile_result in result["profiles"].items():
            profile = candidates[family][key]
            is_zero = profile_result["zero_profile"]
            passes.append(
                profile_result["valid"]
                and profile_result["valid_case_count"] == 9
                and profile_result["validation_support_count"] == 270
                and _fit_support_ok(profile)
                and (
                    profile_result["worst_case_p99"] <= gate["zero_profile_worst_p99_max_m"]
                    if is_zero
                    else profile_result["position_error_median"] <= gate["nonzero_profile_median_max_m"]
                    and profile_result["worst_case_p99"] <= gate["nonzero_profile_worst_p99_max_m"]
                )
            )
        result["profile_pass_count"] = sum(passes)
        result["gate_pass"] = (
            family != "raw_nominal"
            and all(passes)
            and result["invalid_prediction_count"] == 0
            and result["corrected_raw_macro_median_ratio"] <= gate["nonzero_candidate_raw_macro_median_ratio_max"]
        )
    return {"schema_version": S2_QUALIFICATION_SCHEMA_VERSION, "families": family_results, "validation_gates": gate}


def _metric_compare(left: Mapping[str, Any], right: Mapping[str, Any], tolerance: float) -> int:
    for name in ("nonzero_profile_macro_median", "nonzero_profile_worst_p99", "zero_profile_p99"):
        delta = _finite(left[name], name) - _finite(right[name], name)
        if abs(delta) > tolerance:
            return -1 if delta < 0.0 else 1
    left_rank = COMPLEXITY_RANK[str(left["method_family"])]
    right_rank = COMPLEXITY_RANK[str(right["method_family"])]
    return -1 if left_rank < right_rank else (1 if left_rank > right_rank else 0)


def select_candidate_family(results: Mapping[str, Any], *, tolerance: float = ANGLE_TIE_TOLERANCE) -> str:
    tolerance = _finite(tolerance, "selection tolerance")
    if tolerance != ANGLE_TIE_TOLERANCE:
        raise ValueError("selection tolerance is frozen at 1e-12")
    families = results.get("families", results)
    passing = [value for value in families.values() if value.get("gate_pass") is True]
    if not passing:
        raise ValueError("no fitted candidate family passed Validation")
    selected = passing[0]
    for candidate in passing[1:]:
        if _metric_compare(candidate, selected, tolerance) < 0:
            selected = candidate
    return str(selected["method_family"])


def validate_access_ledger(
    ledger: Path | Sequence[Mapping[str, Any]], *, require_r4a_sealed_closed: bool = True
) -> list[dict[str, Any]]:
    if isinstance(ledger, (str, Path)):
        snapshot = _read_verified_absolute_file(
            Path(ledger).resolve(strict=True), name="access_ledger"
        )
        entries = _parse_verified_jsonl(snapshot, name="access_ledger")
    else:
        entries = [dict(entry) for entry in ledger]
    for index, entry in enumerate(entries, start=1):
        _require_exact_keys(entry, _LEDGER_KEYS, "access_ledger_row")
        if _require_exact_int(entry["sequence"], "access_ledger:sequence") != index:
            raise ValueError("access ledger sequence is not contiguous")
        stage = _require_exact_str(entry["stage"], "access_ledger:stage")
        role = _require_exact_str(entry["role"], "access_ledger:role")
        operation = _require_exact_str(entry["operation"], "access_ledger:operation")
        canonical_path = _require_exact_str(
            entry["canonical_path"], "access_ledger:canonical_path"
        )
        _require_sha256(entry["artifact_hash"], "access_ledger:artifact_hash")
        _require_exact_str(entry["utc"], "access_ledger:utc")
        if _require_exact_int(entry["process_pid"], "access_ledger:process_pid") <= 0:
            raise ValueError("access_ledger_process_pid_invalid")
        if not Path(canonical_path).is_absolute():
            raise ValueError("access_ledger_canonical_path_not_absolute")
        if stage not in LEDGER_STAGES or role not in DATASET_ROLES:
            raise ValueError("access ledger contains an invalid stage or role")
        if role == "sealed_evaluation" and stage in {"fit", "validation_score"}:
            raise ValueError("sealed Evaluation data was accessed during candidate search")
        if role == "sealed_evaluation" and stage == "freeze" and operation not in {"read_commitment", "read_sealed_plan"}:
            raise ValueError("freeze accessed sealed runtime data")
        if require_r4a_sealed_closed and stage in {"unseal", "sealed_evaluation"}:
            raise ValueError("R4-A access ledger shows sealed Evaluation was opened")
    return entries


_SEALED_COMMITMENT_KEYS = {
    "schema_version", "sweep_schema_version", "qualification_id", "branch", "commit_sha",
    "sealed_plan_hash", "sealed_cases_file_sha256", "case_count", "case_id_ordered_hash",
    "profile_key_ordered_hash", "spatial_group_ordered_hash", "camera_contract",
    "target_physics_contract", "expected_warmup_samples", "expected_analysis_samples",
    "expected_output_frame", "created_before_profile_freeze", "commitment_hash",
}


def validate_sealed_commitment(
    commitment: Mapping[str, Any],
    *,
    expected_branch: str | None = None,
    expected_commit_sha: str | None = None,
    expected_qualification_id: str | None = None,
) -> dict[str, Any]:
    value = dict(commitment)
    if set(value) != _SEALED_COMMITMENT_KEYS:
        raise ValueError("sealed commitment schema mismatch")
    if value["schema_version"] != S2_QUALIFICATION_SCHEMA_VERSION or value["sweep_schema_version"] != S2_SWEEP_SCHEMA_VERSION:
        raise ValueError("sealed commitment version mismatch")
    if expected_branch is not None and value["branch"] != expected_branch:
        raise ValueError("sealed commitment branch mismatch")
    if expected_commit_sha is not None and value["commit_sha"] != _require_sha(expected_commit_sha):
        raise ValueError("sealed commitment SHA mismatch")
    if expected_qualification_id is not None and value["qualification_id"] != expected_qualification_id:
        raise ValueError("sealed commitment qualification ID mismatch")
    if value["case_count"] != 225 or value["created_before_profile_freeze"] is not True:
        raise ValueError("sealed commitment count/order contract mismatch")
    if value["expected_output_frame"] != BASELINE_CENTER_FRD_FRAME:
        raise ValueError("sealed commitment output frame mismatch")
    physics = value["target_physics_contract"]
    if physics != {"trajectory_mode": "static_gazebo_reference_point", "target_physics_mode": "static_model", "target_static_for_research": True}:
        raise ValueError("sealed commitment target physics mismatch")
    core = {key: item for key, item in value.items() if key != "commitment_hash"}
    if value["commitment_hash"] != sha256_bytes(canonical_json(core).encode("utf-8")):
        raise ValueError("sealed commitment hash mismatch")
    return value


_FROZEN_CATALOG_KEYS = {
    "schema_version", "qualification_id", "branch", "commit_sha",
    "selected_method_family", "selection_rule", "calibration_plan_hash",
    "validation_plan_hash", "sealed_plan_commitment_hash",
    "candidate_search_code_sha256", "camera_contract", "observation_mode",
    "output_frame", "output_axes", "position_unit", "matrix_direction_convention",
    "quaternion_order", "profile_catalog_role", "runtime_selection_source",
    "deployment_ready", "profile_count", "profiles",
}
_FROZEN_PROFILE_KEYS = {
    "profile_key", "source_attitude_deg", "fitted_left_matrix_row_major",
    "fitted_right_matrix_row_major", "left_determinant", "right_determinant",
    "left_orthonormality_error", "right_orthonormality_error",
    "left_support_case_count", "right_support_case_count", "left_singular_values",
    "right_singular_values", "calibration_residual_diagnostics",
    "validation_metric_summary",
}


def validate_frozen_catalog(
    catalog: Mapping[str, Any], *, expected_branch: str | None = None, expected_commit_sha: str | None = None
) -> dict[str, Any]:
    value = dict(catalog)
    if set(value) != _FROZEN_CATALOG_KEYS:
        raise ValueError("frozen catalog schema mismatch")
    if value["schema_version"] != S2_QUALIFICATION_SCHEMA_VERSION or value["selected_method_family"] not in CANDIDATE_FAMILIES:
        raise ValueError("frozen catalog version or family mismatch")
    if expected_branch is not None and value["branch"] != expected_branch:
        raise ValueError("frozen catalog branch mismatch")
    if expected_commit_sha is not None and value["commit_sha"] != _require_sha(expected_commit_sha):
        raise ValueError("frozen catalog SHA mismatch")
    if value["selection_rule"] != SELECTION_RULE or value["output_frame"] != BASELINE_CENTER_FRD_FRAME:
        raise ValueError("frozen catalog contract mismatch")
    if value["matrix_direction_convention"] != MATRIX_DIRECTION or value["quaternion_order"] != "xyzw":
        raise ValueError("frozen catalog rotation convention mismatch")
    if value["profile_catalog_role"] != PROFILE_CATALOG_ROLE or value["runtime_selection_source"] != "not_implemented" or value["deployment_ready"] is not False:
        raise ValueError("frozen catalog deployment meaning mismatch")
    profiles = value["profiles"]
    if not isinstance(profiles, list) or value["profile_count"] != 25 or len(profiles) != 25:
        raise ValueError("frozen catalog must contain 25 profiles")
    keys = set()
    for profile in profiles:
        if not isinstance(profile, dict) or set(profile) != _FROZEN_PROFILE_KEYS:
            raise ValueError("frozen profile schema mismatch")
        if profile["profile_key"] in keys:
            raise ValueError("duplicate frozen profile key")
        keys.add(profile["profile_key"])
        if set(profile["source_attitude_deg"]) != set(ATTITUDE_AXES):
            raise ValueError("frozen source attitude schema mismatch")
        for name in ("fitted_left_matrix_row_major", "fitted_right_matrix_row_major"):
            _matrix3(profile[name], name)
        for name in ("left_determinant", "right_determinant"):
            if abs(_finite(profile[name], name) - 1.0) > 1.0e-10:
                raise ValueError("frozen rotation determinant mismatch")
        for name in ("left_orthonormality_error", "right_orthonormality_error"):
            if _finite(profile[name], name) > 1.0e-10:
                raise ValueError("frozen rotation orthonormality mismatch")
    return value


def _regular_file_beneath(root: Path, relative_path: str, *, name: str) -> Path:
    canonical = _canonical_relative_path(relative_path, name=name)
    resolved_root = Path(root).resolve(strict=True)
    candidate = resolved_root
    for part in PurePosixPath(canonical).parts:
        candidate = candidate / part
        if candidate.is_symlink():
            raise ValueError(f"{name}_symlink_rejected: {canonical}")
    try:
        mode = candidate.stat(follow_symlinks=False).st_mode
    except FileNotFoundError as exc:
        raise ValueError(f"{name}_missing: {canonical}") from exc
    if not stat.S_ISREG(mode):
        raise ValueError(f"{name}_nonregular_file: {canonical}")
    resolved = candidate.resolve(strict=True)
    if resolved_root not in resolved.parents:
        raise ValueError(f"{name}_outside_root: {canonical}")
    return resolved


def _read_verified_file_beneath(
    root: Path, relative_path: str, *, name: str
) -> VerifiedFileBytes:
    path = _regular_file_beneath(root, relative_path, name=name)
    snapshot = _read_verified_absolute_file(path, name=name)
    resolved_root = Path(root).resolve(strict=True)
    if resolved_root not in snapshot.canonical_path.parents:
        raise ValueError(f"{name}_outside_root: {relative_path}")
    return snapshot


def _load_strict_jsonl(path: Path) -> list[dict[str, Any]]:
    snapshot = _read_verified_absolute_file(Path(path).resolve(strict=True), name="jsonl")
    return _parse_verified_jsonl(snapshot, name="jsonl")


def _git_output(repository_root: Path, *arguments: str) -> str:
    try:
        return subprocess.check_output(
            ("git", *arguments), cwd=repository_root, text=True, stderr=subprocess.STDOUT
        ).strip()
    except subprocess.CalledProcessError as exc:
        raise ValueError(f"git_anchor_validation_failed: {' '.join(arguments)}") from exc


def resolve_freeze_anchor_commit(
    repository_root: Path,
    freeze_anchor_path: Path,
    *,
    qualification_source_sha: str,
    evaluation_code_sha: str | None = None,
) -> dict[str, str]:
    root = Path(repository_root).resolve(strict=True)
    if _git_output(root, "rev-parse", "--show-toplevel") != str(root):
        raise ValueError("freeze_anchor_repository_toplevel_mismatch")
    if _git_output(root, "status", "--short"):
        raise ValueError("freeze_anchor_repository_not_clean")
    anchor_file = Path(freeze_anchor_path)
    if not anchor_file.is_absolute():
        anchor_file = root / anchor_file
    anchor_file = anchor_file.resolve(strict=True)
    try:
        relative = anchor_file.relative_to(root).as_posix()
    except ValueError as exc:
        raise ValueError("freeze_anchor_outside_repository") from exc
    if relative != FREEZE_ANCHOR_REPOSITORY_PATH:
        raise ValueError("freeze_anchor_repository_path_mismatch")
    source_sha = _require_sha(qualification_source_sha)
    head = _require_sha(evaluation_code_sha or _git_output(root, "rev-parse", "HEAD"))
    if _git_output(root, "rev-parse", "HEAD") != head:
        raise ValueError("evaluation_code_sha_mismatch")
    _git_output(root, "cat-file", "-e", f"{source_sha}^{{commit}}")
    additions = [
        line
        for line in _git_output(
            root, "log", "--diff-filter=A", "--format=%H", head, "--", relative
        ).splitlines()
        if line
    ]
    if len(additions) != 1:
        raise ValueError("freeze_anchor_add_commit_count_mismatch")
    anchor_sha = _require_sha(additions[0])
    for older, newer, reason in (
        (source_sha, anchor_sha, "freeze_anchor_not_descendant_of_source"),
        (anchor_sha, head, "evaluation_head_not_descendant_of_anchor"),
    ):
        result = subprocess.run(
            ("git", "merge-base", "--is-ancestor", older, newer),
            cwd=root,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if result.returncode != 0:
            raise ValueError(reason)
    modifications = _git_output(root, "log", "--format=%H", f"{anchor_sha}..{head}", "--", relative)
    if modifications:
        raise ValueError("freeze_anchor_modified_after_addition")
    anchor_blob = _git_output(root, "rev-parse", f"{anchor_sha}:{relative}")
    head_blob = _git_output(root, "rev-parse", f"{head}:{relative}")
    working_blob = _git_output(root, "hash-object", str(anchor_file))
    if anchor_blob != head_blob or anchor_blob != working_blob:
        raise ValueError("freeze_anchor_blob_mismatch")
    return {
        "qualification_source_sha": source_sha,
        "freeze_anchor_sha": anchor_sha,
        "evaluation_code_sha": head,
        "freeze_anchor_blob_sha": anchor_blob,
    }


def _validate_coordinate_contract_sources(anchor: Mapping[str, Any], repository_root: Path) -> None:
    root = Path(repository_root).resolve(strict=True)
    files = anchor["coordinate_contract_sources"]["files"]
    snapshots: dict[str, VerifiedFileBytes] = {}
    for relative, expected in files.items():
        snapshot = _read_verified_file_beneath(root, relative, name="coordinate_source")
        snapshots[relative] = snapshot
        if snapshot.sha256 != expected:
            raise ValueError(f"coordinate_source_sha256_mismatch: {relative}")

    ground_truth = anchor["ground_truth_provenance"]
    actual = {
        "semantic_reference_name": VEHICLE_REFERENCE_LINK,
        "semantic_reference_definition": VEHICLE_REFERENCE_DEFINITION,
        "gazebo_source_entity": SOURCE_GAZEBO_ENTITY,
        "reference_translation_offset_m": list(REFERENCE_OFFSET_PARENT_M),
        "reference_rotation_offset_rpy_rad": list(REFERENCE_OFFSET_PARENT_RPY_RAD),
        "raw_pose_topic": POSE_TOPIC,
        "output_gt_topic": OUTPUT_TOPIC,
        "world_frame": WORLD_FRAME,
        "output_frame": COORDINATE_BASELINE_CENTER_FRD_FRAME,
    }
    if any(ground_truth[name] != value for name, value in actual.items()):
        raise ValueError("ground_truth_source_constant_mismatch")

    model_snapshot = snapshots[
        "Simulation/stereo_ground_sim/models/x500_15m/model.sdf"
    ]
    model = ET.fromstring(model_snapshot.raw_bytes).find("model")
    if model is None or model.attrib.get("name") != ground_truth["parent_model"]:
        raise ValueError("ground_truth_parent_model_mismatch")
    frames = model.findall(f"frame[@name='{ground_truth['semantic_reference_name']}']")
    if len(frames) != 1:
        raise ValueError("ground_truth_semantic_frame_count_mismatch")
    frame = frames[0]
    if frame.attrib.get("attached_to") != ground_truth["parent_link"]:
        raise ValueError("ground_truth_parent_link_mismatch")
    pose_values = tuple(float(value) for value in (frame.findtext("pose") or "").split())
    if pose_values != (0.0, 0.0, 0.0, 0.0, 0.0, 0.0):
        raise ValueError("ground_truth_reference_offset_mismatch")

    try:
        launch_source = snapshots[
            "Simulation/stereo_ground_sim/launch/stereo_ground_sim.launch.py"
        ].raw_bytes.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError("ground_truth_launch_utf8_invalid") from exc
    required_launch_contracts = (
        '"parent_model_name": LaunchConfiguration("target_model_name")',
        '"source_link_name": "base_link"',
        'DeclareLaunchArgument("target_model_name", default_value="x500_15m")',
    )
    if any(contract not in launch_source for contract in required_launch_contracts):
        raise ValueError("ground_truth_launch_binding_mismatch")
    required_pair_fields = {
        "gt_source",
        "gt_source_entity",
        "gt_reference_name",
        "gt_reference_definition",
        "gt_fallback_used",
        "output_frame",
    }
    if not required_pair_fields.issubset(PAIR_RECORD_COLUMNS):
        raise ValueError("ground_truth_pair_schema_mismatch")


def _canonical_absolute_ledger_path(value: Any, *, name: str) -> Path:
    text = _require_exact_str(value, f"{name}:canonical_path")
    path = PurePosixPath(text)
    if (
        not text
        or "\\" in text
        or not path.is_absolute()
        or any(part in {"", ".", ".."} for part in path.parts[1:])
        or path.as_posix() != text
    ):
        raise ValueError(f"{name}_path_invalid")
    return Path(text)


def _external_ledger_file(
    entry: Mapping[str, Any], *, name: str
) -> VerifiedFileBytes:
    path = _canonical_absolute_ledger_path(entry.get("canonical_path"), name=name)
    expected = _require_sha256(entry.get("artifact_hash"), f"{name}:ledger")
    snapshot = _read_verified_absolute_file(path, name="ledger_referenced_artifact")
    if snapshot.sha256 != expected:
        raise ValueError(
            f"ledger_referenced_artifact_sha256_mismatch: {path.as_posix()}"
        )
    return snapshot


def _verify_ledger_referenced_artifacts(
    ledger: Sequence[Mapping[str, Any]],
    dataset: Mapping[str, Any],
) -> tuple[dict[str, int], dict[str, VerifiedFileBytes]]:
    entries = validate_access_ledger(ledger)
    operation_counts = {operation: 0 for operation in _FROZEN_LEDGER_OPERATIONS}
    total_cases = (
        dataset["calibration_case_count"] + dataset["validation_case_count"]
    )
    expected_operation_counts = {
        "read_r2_artifact": total_cases * len(_R2_ARTIFACT_NAMES),
        "read_sweep_artifact": 6,
        "read_precommitted_role_plan": 4,
        "read_sealed_plan": 2,
        "read_commitment": 1,
        "read_qualification_plan": 1,
        "check_sweep_valid_marker": 2,
    }
    expected_hash_by_path: dict[str, str] = {}
    conflict_count = 0
    for entry in entries:
        operation = entry["operation"]
        if operation not in _FROZEN_LEDGER_OPERATIONS:
            raise ValueError(f"access_ledger_operation_not_allowed: {operation}")
        operation_counts[operation] += 1
        canonical_path = _canonical_absolute_ledger_path(
            entry["canonical_path"], name="ledger_referenced_artifact"
        ).as_posix()
        expected_hash = entry["artifact_hash"]
        prior = expected_hash_by_path.setdefault(canonical_path, expected_hash)
        if prior != expected_hash:
            conflict_count += 1
    if conflict_count:
        raise ValueError("ledger_referenced_path_hash_conflict")
    if operation_counts != expected_operation_counts:
        raise ValueError("access_ledger_operation_count_mismatch")

    expected_role_cases = {
        "calibration": dataset["calibration_case_count"],
        "validation": dataset["validation_case_count"],
    }
    for role, case_count in expected_role_cases.items():
        role_entries = [
            entry
            for entry in entries
            if entry["role"] == role and entry["operation"] == "read_r2_artifact"
        ]
        if len(role_entries) != case_count * len(_R2_ARTIFACT_NAMES):
            raise ValueError(f"qualification_r2_artifact_count_mismatch: {role}")
        basename_counts = {
            name: sum(
                PurePosixPath(entry["canonical_path"]).name == name
                for entry in role_entries
            )
            for name in _R2_ARTIFACT_NAMES
        }
        if any(count != case_count for count in basename_counts.values()):
            raise ValueError(f"qualification_r2_artifact_set_mismatch: {role}")

    snapshots: dict[str, VerifiedFileBytes] = {}
    for entry in entries:
        canonical_path = entry["canonical_path"]
        if canonical_path in snapshots:
            continue
        snapshots[canonical_path] = _external_ledger_file(
            entry, name="ledger_referenced_artifact"
        )
    r2_reference_count = operation_counts["read_r2_artifact"]
    case_level_reference_count = sum(
        entry["operation"] == "read_r2_artifact"
        and PurePosixPath(entry["canonical_path"]).name
        in _R2_CASE_LEVEL_ARTIFACT_NAMES
        for entry in entries
    )
    return (
        {
            "ledger_row_count": len(entries),
            "ledger_r2_artifact_reference_count": r2_reference_count,
            "ledger_case_level_artifact_reference_count": case_level_reference_count,
            "ledger_unique_file_count": len(snapshots),
            "ledger_rehashed_file_count": len(snapshots),
            "ledger_skipped_file_count": 0,
            "ledger_hash_mismatch_count": 0,
            "ledger_path_hash_conflict_count": 0,
        },
        snapshots,
    )


def _validate_qualification_inputs_and_gt_metadata(
    anchor: Mapping[str, Any],
    ledger: Sequence[Mapping[str, Any]],
    ledger_snapshots: Mapping[str, VerifiedFileBytes],
    repository_root: Path,
) -> None:
    source = anchor["qualification_source"]
    inputs = anchor["qualification_input_sha256"]
    repository = Path(repository_root).resolve(strict=True)
    candidate_bytes = subprocess.check_output(
        (
            "git",
            "show",
            f"{source['commit_sha']}:Simulation/stereo_ground_sim/"
            "stereo_ground_sim/correction_qualification.py",
        ),
        cwd=repository,
    )
    if sha256_bytes(candidate_bytes) != inputs["candidate_search_code"]:
        raise ValueError("candidate_search_source_blob_sha256_mismatch")

    summary_contracts = {
        "calibration": inputs["calibration_sweep_summary"],
        "validation": inputs["validation_sweep_summary"],
    }
    for role, expected_hash in summary_contracts.items():
        matches = [
            entry
            for entry in ledger
            if entry.get("role") == role
            and entry.get("operation") == "read_sweep_artifact"
            and Path(str(entry.get("canonical_path", ""))).name == "sweep_summary.json"
        ]
        if len(matches) != 1 or matches[0].get("artifact_hash") != expected_hash:
            raise ValueError(f"{role}_sweep_summary_ledger_identity_mismatch")
        summary_snapshot = ledger_snapshots[matches[0]["canonical_path"]]
        summary = _parse_verified_json(
            summary_snapshot, name=f"{role}_sweep_summary"
        )
        if (
            summary.get("branch") != source["branch"]
            or summary.get("commit_sha") != source["commit_sha"]
            or summary.get("run_valid") is not True
            or summary.get("finalization_status") != "VALID"
        ):
            raise ValueError(f"{role}_sweep_summary_contract_mismatch")

    ground_truth = anchor["ground_truth_provenance"]
    expected_metadata = {
        "branch": source["branch"],
        "commit_sha": source["commit_sha"],
        "evaluation_gt_source": "gazebo_reference_link_world_pose",
        "gazebo_world_frame": ground_truth["world_frame"],
        "reference_source": ground_truth["semantic_reference_name"],
        "target_model_name": ground_truth["parent_model"],
        "output_frame": ground_truth["output_frame"],
        "output_axes": ground_truth["output_axes"],
        "position_unit": ground_truth["position_unit"],
        "quaternion_order": ground_truth["quaternion_order"],
        "observation_source": "gazebo_reference_point_projection",
    }
    metadata_entries = [
        entry
        for entry in ledger
        if entry.get("role") in {"calibration", "validation"}
        and entry.get("operation") == "read_r2_artifact"
        and Path(str(entry.get("canonical_path", ""))).name == "run_metadata.json"
    ]
    if len({entry.get("canonical_path") for entry in metadata_entries}) != len(metadata_entries):
        raise ValueError("qualification_run_metadata_path_duplicate")
    for entry in metadata_entries:
        snapshot = ledger_snapshots[entry["canonical_path"]]
        metadata = _parse_verified_json(
            snapshot, name="qualification_run_metadata"
        )
        if any(metadata.get(field) != value for field, value in expected_metadata.items()):
            raise ValueError("qualification_run_metadata_gt_provenance_mismatch")


def verify_freeze_anchor_artifacts(
    qualification_root: Path,
    freeze_anchor: Mapping[str, Any],
    *,
    repository_root: Path,
) -> dict[str, Any]:
    anchor = validate_freeze_anchor(freeze_anchor)
    root_path = Path(qualification_root)
    if not root_path.is_absolute():
        root_path = root_path.absolute()
    _reject_symlink_components(root_path, name="qualification_root")
    root = root_path.resolve(strict=True)
    if not root.is_dir():
        raise ValueError("qualification_root_not_directory")
    _assert_no_sealed_runtime(root, require_frozen_layout=True)

    artifact_snapshots: dict[str, VerifiedFileBytes] = {}
    for relative, expected in anchor["artifact_sha256"].items():
        snapshot = _read_verified_file_beneath(root, relative, name="artifact")
        artifact_snapshots[relative] = snapshot
        if snapshot.sha256 != expected:
            raise ValueError(f"artifact_sha256_mismatch: {relative}")

    state = _parse_verified_json(
        artifact_snapshots["qualification_state.json"], name="qualification_state"
    )
    manifest = _parse_verified_json(
        artifact_snapshots["frozen_selection_manifest.json"],
        name="frozen_selection_manifest",
    )
    catalog = _parse_verified_json(
        artifact_snapshots["frozen_correction_catalog.json"],
        name="frozen_correction_catalog",
    )
    commitment = _parse_verified_json(
        artifact_snapshots["sealed_evaluation_commitment.json"],
        name="sealed_evaluation_commitment",
    )
    qualification_plan = _parse_verified_json(
        artifact_snapshots["qualification_plan.json"], name="qualification_plan"
    )
    calibration_plan = _parse_verified_json(
        artifact_snapshots["calibration_plan/sweep_plan.json"],
        name="calibration_plan",
    )
    validation_plan = _parse_verified_json(
        artifact_snapshots["validation_plan/sweep_plan.json"],
        name="validation_plan",
    )
    sealed_plan = _parse_verified_json(
        artifact_snapshots["sealed_plan/sealed_sweep_plan.json"],
        name="sealed_plan",
    )
    source = anchor["qualification_source"]
    selected = anchor["selected_profile"]
    dataset = anchor["dataset_contract"]
    qid = anchor["qualification_id"]

    validate_frozen_catalog(
        catalog,
        expected_branch=source["branch"],
        expected_commit_sha=source["commit_sha"],
    )
    validate_sealed_commitment(
        commitment,
        expected_branch=source["branch"],
        expected_commit_sha=source["commit_sha"],
        expected_qualification_id=qid,
    )
    if catalog["qualification_id"] != qid or catalog["selected_method_family"] != selected["method_family"]:
        raise ValueError("catalog_anchor_identity_mismatch")
    for field in (
        "profile_count",
        "output_frame",
        "output_axes",
        "position_unit",
        "matrix_direction_convention",
        "quaternion_order",
    ):
        if catalog[field] != selected[field]:
            raise ValueError(f"catalog_anchor_contract_mismatch: {field}")

    expected_source_fields = {
        "qualification_id": qid,
        "repository_name": source["repository_name"],
        "branch": source["branch"],
        "commit_sha": source["commit_sha"],
    }
    if any(qualification_plan.get(name) != value for name, value in expected_source_fields.items()):
        raise ValueError("qualification_plan_source_mismatch")
    plan_core = {name: value for name, value in qualification_plan.items() if name != "plan_hash"}
    if qualification_plan.get("plan_hash") != sha256_bytes(canonical_json(plan_core).encode("utf-8")):
        raise ValueError("qualification_plan_hash_mismatch")
    if qualification_plan.get("role_case_counts") != {
        "calibration": dataset["calibration_case_count"],
        "validation": dataset["validation_case_count"],
        "sealed_evaluation": dataset["sealed_case_count"],
    }:
        raise ValueError("qualification_case_count_mismatch")
    for field in ("warmup_eligible_samples", "analysis_eligible_samples", "split_unit"):
        if qualification_plan.get(field) != dataset[field]:
            raise ValueError(f"qualification_dataset_contract_mismatch: {field}")
    role_plans = {
        "calibration": calibration_plan,
        "validation": validation_plan,
        "sealed_evaluation": sealed_plan,
    }
    for role, role_plan in role_plans.items():
        plan_core = {name: value for name, value in role_plan.items() if name != "plan_hash"}
        if role_plan.get("plan_hash") != sha256_bytes(canonical_json(plan_core).encode("utf-8")):
            raise ValueError(f"role_plan_hash_mismatch: {role}")
        if qualification_plan["role_plan_hashes"].get(role) != role_plan["plan_hash"]:
            raise ValueError(f"qualification_role_plan_identity_mismatch: {role}")
    if (
        _require_exact_int(
            commitment["expected_warmup_samples"],
            "sealed_commitment:expected_warmup_samples",
        )
        != dataset["warmup_eligible_samples"]
        or _require_exact_int(
            commitment["expected_analysis_samples"],
            "sealed_commitment:expected_analysis_samples",
        )
        != dataset["analysis_eligible_samples"]
    ):
        raise ValueError("sealed_commitment_sample_count_mismatch")
    if (
        commitment["qualification_id"] != qid
        or commitment["commit_sha"] != source["commit_sha"]
        or _require_exact_int(
            commitment["case_count"], "sealed_commitment:case_count"
        )
        != dataset["sealed_case_count"]
        or commitment["expected_output_frame"] != selected["output_frame"]
        or commitment["camera_contract"] != qualification_plan.get("camera_contract")
        or commitment["camera_contract"] != sealed_plan.get("camera_contract")
    ):
        raise ValueError("sealed_commitment_anchor_contract_mismatch")

    manifest_expected = {
        "qualification_id": qid,
        "branch": source["branch"],
        "commit_sha": source["commit_sha"],
        "selected_method_family": selected["method_family"],
        "qualification_plan_sha256": anchor["artifact_sha256"]["qualification_plan.json"],
        "candidate_family_results_sha256": anchor["artifact_sha256"]["candidate_family_results.json"],
        "validation_scorecard_sha256": anchor["artifact_sha256"]["validation_scorecard.csv"],
        "frozen_correction_catalog_sha256": anchor["artifact_sha256"]["frozen_correction_catalog.json"],
        "sealed_commitment_sha256": anchor["artifact_sha256"]["sealed_evaluation_commitment.json"],
        "data_access_ledger_sha256": anchor["artifact_sha256"]["data_access_ledger.jsonl"],
        "calibration_sweep_summary_sha256": anchor["qualification_input_sha256"]["calibration_sweep_summary"],
        "validation_sweep_summary_sha256": anchor["qualification_input_sha256"]["validation_sweep_summary"],
        "sealed_runtime_access_count": dataset["sealed_runtime_access_count_at_freeze"],
        "sealed_metrics_present": dataset["sealed_metrics_present_at_freeze"],
    }
    if any(manifest.get(name) != value for name, value in manifest_expected.items()):
        raise ValueError("frozen_manifest_anchor_mismatch")
    state_expected = {
        "qualification_id": qid,
        "state": "PROFILE_FROZEN",
        "branch": source["branch"],
        "commit_sha": source["commit_sha"],
        "selected_method_family": selected["method_family"],
        "frozen_catalog_sha256": anchor["artifact_sha256"]["frozen_correction_catalog.json"],
        "sealed_commitment_sha256": anchor["artifact_sha256"]["sealed_evaluation_commitment.json"],
        "frozen_selection_manifest_sha256": anchor["artifact_sha256"]["frozen_selection_manifest.json"],
    }
    if any(state.get(name) != value for name, value in state_expected.items()):
        raise ValueError("qualification_state_anchor_mismatch")
    if catalog["candidate_search_code_sha256"] != anchor["qualification_input_sha256"]["candidate_search_code"]:
        raise ValueError("candidate_search_code_anchor_mismatch")

    role_records = {
        "calibration": _parse_verified_jsonl(
            artifact_snapshots["calibration_plan/sweep_cases.jsonl"],
            name="calibration_cases",
        ),
        "validation": _parse_verified_jsonl(
            artifact_snapshots["validation_plan/sweep_cases.jsonl"],
            name="validation_cases",
        ),
        "sealed_evaluation": _parse_verified_jsonl(
            artifact_snapshots["sealed_plan/sealed_sweep_cases.jsonl"],
            name="sealed_cases",
        ),
    }
    expected_counts = {
        "calibration": dataset["calibration_case_count"],
        "validation": dataset["validation_case_count"],
        "sealed_evaluation": dataset["sealed_case_count"],
    }
    case_sets: dict[str, set[str]] = {}
    spatial_sets: dict[str, set[str]] = {}
    for role, records in role_records.items():
        case_ids = [record.get("case_id") for record in records]
        if len(records) != expected_counts[role] or len(set(case_ids)) != expected_counts[role]:
            raise ValueError(f"qualification_role_case_identity_mismatch: {role}")
        if len({profile_key(record["payload"]) for record in records}) != selected["profile_count"]:
            raise ValueError(f"qualification_role_profile_count_mismatch: {role}")
        case_sets[role] = set(case_ids)
        spatial_sets[role] = {
            spatial_group_key(record["payload"]) for record in records
        }
    case_overlap_count = sum(
        len(case_sets[left] & case_sets[right])
        for index, left in enumerate(DATASET_ROLES)
        for right in DATASET_ROLES[index + 1 :]
    )
    if case_overlap_count != 0:
        raise ValueError("qualification_case_id_overlap_mismatch")
    spatial_overlap_count = sum(
        len(spatial_sets[left] & spatial_sets[right])
        for index, left in enumerate(DATASET_ROLES)
        for right in DATASET_ROLES[index + 1 :]
    )
    if spatial_overlap_count != dataset["dataset_role_overlap_count"]:
        raise ValueError("qualification_spatial_group_overlap_mismatch")

    sealed_records = role_records["sealed_evaluation"]
    sealed_case_ids = [record["case_id"] for record in sealed_records]
    if sealed_plan.get("plan_hash") != commitment["sealed_plan_hash"]:
        raise ValueError("sealed_plan_identity_mismatch")
    if sealed_plan.get("case_ids_in_execution_order") != sealed_case_ids:
        raise ValueError("sealed_plan_case_order_mismatch")
    if artifact_snapshots["sealed_plan/sealed_sweep_cases.jsonl"].sha256 != commitment["sealed_cases_file_sha256"]:
        raise ValueError("sealed_cases_commitment_mismatch")
    ordered_hashes = {
        "case_id_ordered_hash": sha256_bytes(canonical_json(sealed_case_ids).encode("utf-8")),
        "profile_key_ordered_hash": sha256_bytes(
            canonical_json([profile_key(record["payload"]) for record in sealed_records]).encode("utf-8")
        ),
        "spatial_group_ordered_hash": sha256_bytes(
            canonical_json([spatial_group_key(record["payload"]) for record in sealed_records]).encode("utf-8")
        ),
    }
    if any(commitment.get(name) != value for name, value in ordered_hashes.items()):
        raise ValueError("sealed_commitment_ordered_identity_mismatch")

    ledger = _parse_verified_jsonl(
        artifact_snapshots["data_access_ledger.jsonl"], name="access_ledger"
    )
    validate_access_ledger(ledger)
    run_metadata_counts = {
        role: sum(
            1
            for entry in ledger
            if entry.get("role") == role
            and entry.get("operation") == "read_r2_artifact"
            and PurePosixPath(str(entry.get("canonical_path", ""))).name == "run_metadata.json"
        )
        for role in ("calibration", "validation")
    }
    if run_metadata_counts != {
        "calibration": dataset["calibration_run_metadata_count"],
        "validation": dataset["validation_run_metadata_count"],
    }:
        raise ValueError("qualification_run_metadata_count_mismatch")

    ledger_verification, ledger_snapshots = _verify_ledger_referenced_artifacts(
        ledger, dataset
    )
    _validate_qualification_inputs_and_gt_metadata(
        anchor,
        ledger,
        ledger_snapshots,
        repository_root,
    )

    for marker in _FROZEN_MARKERS:
        _read_verified_file_beneath(root, marker, name="qualification_marker")
    if state["state"] != "PROFILE_FROZEN":
        raise ValueError("qualification_frozen_unopened_state_mismatch")

    _validate_coordinate_contract_sources(anchor, repository_root)
    return {
        "qualification_id": qid,
        "selected_family": selected["method_family"],
        "profile_count": selected["profile_count"],
        "qualification_source_sha": source["commit_sha"],
        "artifact_verification_count": len(artifact_snapshots),
        "ground_truth_provenance_verdict": "validated",
        "qualification_state": "PROFILE_FROZEN",
        "sealed_state": "SEALED_UNOPENED",
        "case_id_overlap_count": case_overlap_count,
        "spatial_group_overlap_count": spatial_overlap_count,
        **ledger_verification,
    }


def verify_freeze_anchor(
    qualification_root: Path,
    freeze_anchor_path: Path,
    *,
    repository_root: Path,
) -> dict[str, Any]:
    repository = Path(repository_root).resolve(strict=True)
    if repository != EVALUATION_REPOSITORY_ROOT.resolve(strict=True):
        raise ValueError("evaluation_repository_identity_mismatch")
    anchor = load_freeze_anchor(freeze_anchor_path)
    result = verify_freeze_anchor_artifacts(
        qualification_root,
        anchor,
        repository_root=repository,
    )
    history = resolve_freeze_anchor_commit(
        repository,
        freeze_anchor_path,
        qualification_source_sha=anchor["qualification_source"]["commit_sha"],
    )
    return {**result, **history}


def _assert_no_sealed_runtime(
    root: Path, *, require_frozen_layout: bool = False
) -> None:
    directory = Path(root).resolve(strict=True)
    sealed = directory / "sealed_plan"
    if not sealed.is_dir() or sealed.is_symlink():
        raise ValueError("sealed_plan_directory_invalid")
    allowed_files = _ARTIFACT_PATHS | _FROZEN_MARKERS
    allowed_directories = {
        parent.as_posix()
        for relative in _ARTIFACT_PATHS
        for parent in PurePosixPath(relative).parents
        if parent.as_posix() != "."
    }
    for path in directory.rglob("*"):
        relative = path.relative_to(directory).as_posix()
        if require_frozen_layout and (
            relative in allowed_files or relative in allowed_directories
        ):
            continue
        if not require_frozen_layout and (
            relative in {
                "sealed_plan/sealed_sweep_plan.json",
                "sealed_plan/sealed_sweep_cases.jsonl",
            }
            or relative == "sealed_plan"
        ):
            continue
        if (
            path.name in _FORBIDDEN_SEALED_RUNTIME_NAMES
            or require_frozen_layout
            or relative.startswith("sealed_plan/")
        ):
            raise ValueError(f"unexpected_sealed_runtime_artifact: {relative}")


def qualification_status(root: Path) -> dict[str, Any]:
    directory = Path(root)
    markers = {
        name: (directory / name).exists()
        for name in (
            "QUALIFICATION_PLANS_PREPARED", "QUALIFICATION_PROFILE_FROZEN",
            "SEALED_EVALUATION_NOT_EXECUTED", "SEALED_EVALUATION_OPENED",
            "SEALED_EVALUATION_COMPLETED",
        )
    }
    if markers["SEALED_EVALUATION_NOT_EXECUTED"] and markers["SEALED_EVALUATION_OPENED"]:
        raise ValueError("sealed markers conflict")
    state = json.loads((directory / "qualification_state.json").read_text(encoding="utf-8"), parse_constant=_reject_json_constant)
    if state.get("state") not in QUALIFICATION_STATES:
        raise ValueError("qualification state is invalid")
    return {"state": state["state"], "markers": markers, "state_payload": state}


def authorize_unseal(
    root: Path,
    *,
    freeze_anchor_path: Path,
    repository_root: Path,
    frozen_catalog_sha256: str,
    sealed_commitment_sha256: str,
) -> dict[str, Any]:
    directory = Path(root)
    verification = verify_freeze_anchor(
        directory,
        freeze_anchor_path,
        repository_root=repository_root,
    )
    anchor = load_freeze_anchor(freeze_anchor_path)
    expected_catalog_hash = anchor["artifact_sha256"]["frozen_correction_catalog.json"]
    expected_commitment_hash = anchor["artifact_sha256"]["sealed_evaluation_commitment.json"]
    if (
        frozen_catalog_sha256 != expected_catalog_hash
        or sealed_commitment_sha256 != expected_commitment_hash
    ):
        raise ValueError("caller artifact hash mismatch with Git freeze anchor")

    status = qualification_status(directory)
    if status["state"] != "PROFILE_FROZEN" or not status["markers"]["QUALIFICATION_PROFILE_FROZEN"]:
        raise ValueError("qualification profile is not frozen")
    source = directory / "SEALED_EVALUATION_NOT_EXECUTED"
    if not source.is_file() or status["markers"]["SEALED_EVALUATION_OPENED"]:
        raise ValueError("sealed Evaluation is not in the unopened state")
    catalog = directory / "frozen_correction_catalog.json"
    commitment_path = directory / "sealed_evaluation_commitment.json"
    manifest_path = directory / "frozen_selection_manifest.json"
    state = status["state_payload"]
    manifest_hash = sha256_file(manifest_path)
    if state.get("frozen_selection_manifest_sha256") != manifest_hash:
        raise ValueError("frozen selection manifest hash mismatch")
    manifest = _load_strict_json(manifest_path)
    if manifest.get("schema_version") != S2_QUALIFICATION_SCHEMA_VERSION:
        raise ValueError("frozen selection manifest schema mismatch")
    qualification_source = anchor["qualification_source"]
    if (
        manifest.get("branch") != qualification_source["branch"]
        or manifest.get("commit_sha") != qualification_source["commit_sha"]
    ):
        raise ValueError("frozen selection manifest provenance mismatch")
    if verification["qualification_state"] != "PROFILE_FROZEN":
        raise ValueError("freeze anchor verification did not confirm PROFILE_FROZEN")
    if (
        state.get("frozen_catalog_sha256") != expected_catalog_hash
        or manifest.get("frozen_correction_catalog_sha256") != expected_catalog_hash
        or sha256_file(catalog) != expected_catalog_hash
        or state.get("sealed_commitment_sha256") != expected_commitment_hash
        or manifest.get("sealed_commitment_sha256") != expected_commitment_hash
        or sha256_file(commitment_path) != expected_commitment_hash
    ):
        raise ValueError("frozen catalog or sealed commitment hash mismatch")
    catalog_value = _load_strict_json(catalog)
    ledger = DataAccessLedger(directory / "data_access_ledger.jsonl")
    ledger.record(stage="unseal", role="sealed_evaluation", path=commitment_path, operation="authorize_unseal")
    os.replace(source, directory / "SEALED_EVALUATION_OPENED")
    state = dict(state)
    state["state"] = "SEALED_OPENED"
    atomic_write_json(directory / "qualification_state.json", state)
    return {"state": "SEALED_OPENED", "selected_family": catalog_value["selected_method_family"]}


def _load_cases_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line, parse_constant=_reject_json_constant) for line in path.read_text(encoding="utf-8").splitlines() if line]


def _latest_attempt(case_root: Path) -> Path:
    attempts = []
    for path in Path(case_root).glob("attempt-*"):
        try:
            attempts.append((int(path.name.removeprefix("attempt-")), path))
        except ValueError:
            continue
    if not attempts:
        raise ValueError(f"case has no attempt: {case_root.name}")
    return max(attempts)[1]


def _record_r2_reads(ledger: DataAccessLedger, recording: Path, *, stage: str, role: str) -> None:
    for name in ("run_metadata.json", "pair_records.jsonl", "pair_records.csv", "run_summary.json", "RUN_FINALIZED_VALID"):
        ledger.record(stage=stage, role=role, path=recording / name, operation="read_r2_artifact")


def _load_role_dataset(
    *,
    root: Path,
    planned_dir: Path,
    role: str,
    expected_branch: str,
    expected_sha: str,
    ledger: DataAccessLedger,
    warmup_count: int,
    analysis_count: int,
) -> list[CalibrationRepresentative] | list[ValidationCase]:
    stage = "fit" if role == "calibration" else "validation_score"
    for path in (root / "sweep_plan.json", root / "sweep_cases.jsonl", root / "sweep_summary.json"):
        ledger.record(stage=stage, role=role, path=path, operation="read_sweep_artifact")
    for path in (planned_dir / "sweep_plan.json", planned_dir / "sweep_cases.jsonl"):
        ledger.record(stage=stage, role=role, path=path, operation="read_precommitted_role_plan")
    actual_plan = json.loads((root / "sweep_plan.json").read_text(encoding="utf-8"), parse_constant=_reject_json_constant)
    planned_plan = json.loads((planned_dir / "sweep_plan.json").read_text(encoding="utf-8"), parse_constant=_reject_json_constant)
    if actual_plan != planned_plan:
        raise ValueError(f"{role} sweep plan differs from precommitted plan")
    summary = json.loads((root / "sweep_summary.json").read_text(encoding="utf-8"), parse_constant=_reject_json_constant)
    valid_marker = root / "SWEEP_FINALIZED_VALID"
    ledger.record(stage=stage, role=role, path=valid_marker, operation="check_sweep_valid_marker")
    if not valid_marker.is_file():
        raise ValueError(f"{role} sweep VALID marker is missing")
    if (root / "SWEEP_FINALIZED_INVALID").exists() or (root / "SWEEP_INCOMPLETE").exists():
        raise ValueError(f"{role} sweep markers are contradictory")
    if summary.get("run_valid") is not True or summary.get("finalization_status") != "VALID":
        raise ValueError(f"{role} sweep is not valid")
    cases = _load_cases_jsonl(root / "sweep_cases.jsonl")
    planned_cases = _load_cases_jsonl(planned_dir / "sweep_cases.jsonl")
    if cases != planned_cases:
        raise ValueError(f"{role} sweep cases differ from precommitted cases")
    output: list[Any] = []
    for case in cases:
        case_id = case["case_id"]
        attempt = _latest_attempt(root / "cases" / case_id)
        recording = attempt / "recording"
        _record_r2_reads(ledger, recording, stage=stage, role=role)
        records, _ = validate_r2_artifacts(
            recording,
            expected_sha=expected_sha,
            expected_branch=expected_branch,
            expected_run_id=actual_plan["sweep_id"],
            expected_case_id=case_id,
            minimum_eligible=warmup_count + analysis_count,
        )
        spatial = case["payload"]["spatial"]
        check = compute_spatial_validation(
            records,
            warmup_count=warmup_count,
            analysis_count=analysis_count,
            requested_range_m=spatial["range_m"]["value"],
            requested_bearing_deg=spatial["bearing_deg"]["value"],
            requested_elevation_deg=spatial["elevation_deg"]["value"],
            range_tolerance_m=case["payload"]["spatial_range_tolerance_m"],
            angle_tolerance_deg=case["payload"]["spatial_angle_tolerance_deg"],
        )
        if check["spatial_validation_pass"] is not True:
            raise ValueError(f"{role} case failed spatial validation: {case_id}")
        if role == "calibration":
            output.append(build_calibration_representative(records, case, warmup_count=warmup_count, analysis_count=analysis_count))
        else:
            output.append(build_validation_case(records, case, warmup_count=warmup_count, analysis_count=analysis_count))
    return output


def _fit_payload(result: WahbaFitResult | None) -> Mapping[str, Any] | None:
    if result is None:
        return None
    return {
        "matrix_row_major": [list(row) for row in result.rotation_cnominal_from_cactual],
        "support_count": result.support_count,
        "case_count": result.case_count,
        "singular_values": list(result.singular_values),
        "determinant": result.determinant,
        "orthonormality_error": result.orthonormality_error,
        "residual_rms_rad": result.residual_rms_rad,
        "status": result.status,
    }


def _candidate_results_payload(candidates: Mapping[str, Mapping[str, CandidateProfile]], evaluation: Mapping[str, Any]) -> dict[str, Any]:
    fits = {}
    for family, profiles in candidates.items():
        fits[family] = {
            key: {
                "left_matrix_row_major": [list(row) for row in profile.left_rotation_cnominal_from_cactual],
                "right_matrix_row_major": [list(row) for row in profile.right_rotation_cnominal_from_cactual],
                "left_fit": _fit_payload(profile.left_fit),
                "right_fit": _fit_payload(profile.right_fit),
                "shared_fit": _fit_payload(profile.shared_fit),
            }
            for key, profile in profiles.items()
        }
    return {"schema_version": S2_QUALIFICATION_SCHEMA_VERSION, "candidate_fits": fits, "validation": evaluation}


def _write_validation_scorecard(path: Path, evaluation: Mapping[str, Any]) -> None:
    fields = (
        "method_family", "gate_pass", "profile_pass_count", "zero_profile_p99",
        "nonzero_profile_macro_median", "nonzero_profile_macro_p95",
        "nonzero_profile_worst_p99", "corrected_raw_macro_median_ratio",
        "invalid_prediction_count", "invalid_rate",
    )
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for family in CANDIDATE_FAMILIES:
            writer.writerow({name: evaluation["families"][family][name] for name in fields})
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def _rotation_summary(profile: CandidateProfile, side: str) -> tuple[float, float, int, list[float]]:
    matrix = np.asarray(
        profile.left_rotation_cnominal_from_cactual if side == "left" else profile.right_rotation_cnominal_from_cactual,
        dtype=float,
    )
    fit = profile.shared_fit or (profile.left_fit if side == "left" else profile.right_fit)
    return (
        float(np.linalg.det(matrix)),
        float(np.max(np.abs(matrix.T @ matrix - np.eye(3)))),
        0 if fit is None else fit.case_count,
        [] if fit is None else list(fit.singular_values),
    )


def _frozen_catalog(
    *,
    qualification_plan: Mapping[str, Any],
    commitment: Mapping[str, Any],
    selected_family: str,
    candidates: Mapping[str, Mapping[str, CandidateProfile]],
    evaluation: Mapping[str, Any],
    validation_cases: Sequence[ValidationCase],
    code_hash: str,
) -> dict[str, Any]:
    source_by_key = {case.profile_key: case.source_attitude_deg for case in validation_cases}
    entries = []
    for key in sorted(candidates[selected_family]):
        profile = candidates[selected_family][key]
        left_det, left_ortho, left_support, left_singular = _rotation_summary(profile, "left")
        right_det, right_ortho, right_support, right_singular = _rotation_summary(profile, "right")
        entries.append(
            {
                "profile_key": key,
                "source_attitude_deg": {axis: value for axis, value in zip(ATTITUDE_AXES, source_by_key[key])},
                "fitted_left_matrix_row_major": [list(row) for row in profile.left_rotation_cnominal_from_cactual],
                "fitted_right_matrix_row_major": [list(row) for row in profile.right_rotation_cnominal_from_cactual],
                "left_determinant": left_det,
                "right_determinant": right_det,
                "left_orthonormality_error": left_ortho,
                "right_orthonormality_error": right_ortho,
                "left_support_case_count": left_support,
                "right_support_case_count": right_support,
                "left_singular_values": left_singular,
                "right_singular_values": right_singular,
                "calibration_residual_diagnostics": {
                    "left_residual_rms_rad": None if (profile.shared_fit or profile.left_fit) is None else (profile.shared_fit or profile.left_fit).residual_rms_rad,
                    "right_residual_rms_rad": None if (profile.shared_fit or profile.right_fit) is None else (profile.shared_fit or profile.right_fit).residual_rms_rad,
                },
                "validation_metric_summary": evaluation["families"][selected_family]["profiles"][key],
            }
        )
    return {
        "schema_version": S2_QUALIFICATION_SCHEMA_VERSION,
        "qualification_id": qualification_plan["qualification_id"],
        "branch": qualification_plan["branch"],
        "commit_sha": qualification_plan["commit_sha"],
        "selected_method_family": selected_family,
        "selection_rule": SELECTION_RULE,
        "calibration_plan_hash": qualification_plan["role_plan_hashes"]["calibration"],
        "validation_plan_hash": qualification_plan["role_plan_hashes"]["validation"],
        "sealed_plan_commitment_hash": commitment["commitment_hash"],
        "candidate_search_code_sha256": code_hash,
        "camera_contract": qualification_plan["camera_contract"],
        "observation_mode": "ideal_reference_point",
        "output_frame": BASELINE_CENTER_FRD_FRAME,
        "output_axes": "forward_right_down",
        "position_unit": "m",
        "matrix_direction_convention": MATRIX_DIRECTION,
        "quaternion_order": "xyzw",
        "profile_catalog_role": PROFILE_CATALOG_ROLE,
        "runtime_selection_source": "not_implemented",
        "deployment_ready": False,
        "profile_count": len(entries),
        "profiles": entries,
    }


def fit_select_freeze(
    *,
    qualification_root: Path,
    calibration_sweep_root: Path,
    validation_sweep_root: Path,
    expected_branch: str,
    expected_commit_sha: str,
) -> dict[str, Any]:
    root = Path(qualification_root)
    _require_sha(expected_commit_sha)
    status = qualification_status(root)
    if status["state"] != "PLANS_PREPARED" or (root / "QUALIFICATION_PROFILE_FROZEN").exists():
        raise ValueError("qualification output is not eligible for first freeze")
    source_marker = root / "QUALIFICATION_PLANS_PREPARED"
    if not source_marker.is_file() or not (root / "SEALED_EVALUATION_NOT_EXECUTED").is_file():
        raise ValueError("qualification plan or sealed-unopened marker is missing")
    _assert_no_sealed_runtime(root)
    ledger = DataAccessLedger(root / "data_access_ledger.jsonl")
    plan_path = root / "qualification_plan.json"
    ledger.record(stage="freeze", role="calibration", path=plan_path, operation="read_qualification_plan")
    qualification_plan = json.loads(plan_path.read_text(encoding="utf-8"), parse_constant=_reject_json_constant)
    if qualification_plan.get("branch") != expected_branch or qualification_plan.get("commit_sha") != expected_commit_sha:
        raise ValueError("qualification plan provenance mismatch")
    commitment_path = root / "sealed_evaluation_commitment.json"
    ledger.record(stage="freeze", role="sealed_evaluation", path=commitment_path, operation="read_commitment")
    commitment = json.loads(commitment_path.read_text(encoding="utf-8"), parse_constant=_reject_json_constant)
    validate_sealed_commitment(
        commitment,
        expected_branch=expected_branch,
        expected_commit_sha=expected_commit_sha,
        expected_qualification_id=qualification_plan["qualification_id"],
    )
    sealed_plan_path = root / "sealed_plan" / "sealed_sweep_plan.json"
    sealed_cases_path = root / "sealed_plan" / "sealed_sweep_cases.jsonl"
    for path in (sealed_plan_path, sealed_cases_path):
        ledger.record(stage="freeze", role="sealed_evaluation", path=path, operation="read_sealed_plan")
    sealed_plan = json.loads(sealed_plan_path.read_text(encoding="utf-8"), parse_constant=_reject_json_constant)
    if sealed_plan.get("plan_hash") != commitment["sealed_plan_hash"] or sha256_file(sealed_cases_path) != commitment["sealed_cases_file_sha256"]:
        raise ValueError("sealed plan differs from its commitment")
    calibration = _load_role_dataset(
        root=Path(calibration_sweep_root), planned_dir=root / "calibration_plan", role="calibration",
        expected_branch=expected_branch, expected_sha=expected_commit_sha, ledger=ledger,
        warmup_count=qualification_plan["warmup_eligible_samples"], analysis_count=qualification_plan["analysis_eligible_samples"],
    )
    validation = _load_role_dataset(
        root=Path(validation_sweep_root), planned_dir=root / "validation_plan", role="validation",
        expected_branch=expected_branch, expected_sha=expected_commit_sha, ledger=ledger,
        warmup_count=qualification_plan["warmup_eligible_samples"], analysis_count=qualification_plan["analysis_eligible_samples"],
    )
    representatives = [item for item in calibration if isinstance(item, CalibrationRepresentative)]
    validation_cases = [item for item in validation if isinstance(item, ValidationCase)]
    if len(representatives) != 450 or len(validation_cases) != 225:
        raise ValueError("Calibration/Validation case counts do not match the frozen plan")
    if {item.case_id for item in representatives} & {item.case_id for item in validation_cases}:
        raise ValueError("Calibration and Validation case IDs overlap")
    candidates = fit_candidate_families(representatives)
    evaluation = evaluate_candidate_families(candidates, validation_cases)
    selected_family = select_candidate_family(evaluation)
    candidate_payload = _candidate_results_payload(candidates, evaluation)
    candidate_path = root / "candidate_family_results.json"
    atomic_write_json(candidate_path, candidate_payload)
    scorecard_path = root / "validation_scorecard.csv"
    _write_validation_scorecard(scorecard_path, evaluation)
    code_hash = sha256_file(Path(__file__))
    catalog = _frozen_catalog(
        qualification_plan=qualification_plan,
        commitment=commitment,
        selected_family=selected_family,
        candidates=candidates,
        evaluation=evaluation,
        validation_cases=validation_cases,
        code_hash=code_hash,
    )
    validate_frozen_catalog(catalog, expected_branch=expected_branch, expected_commit_sha=expected_commit_sha)
    catalog_path = root / "frozen_correction_catalog.json"
    atomic_write_json(catalog_path, catalog)
    validate_access_ledger(root / "data_access_ledger.jsonl")
    manifest = {
        "schema_version": S2_QUALIFICATION_SCHEMA_VERSION,
        "qualification_id": qualification_plan["qualification_id"],
        "branch": expected_branch,
        "commit_sha": expected_commit_sha,
        "selected_method_family": selected_family,
        "selection_rule": SELECTION_RULE,
        "qualification_plan_sha256": sha256_file(plan_path),
        "calibration_sweep_summary_sha256": sha256_file(Path(calibration_sweep_root) / "sweep_summary.json"),
        "validation_sweep_summary_sha256": sha256_file(Path(validation_sweep_root) / "sweep_summary.json"),
        "sealed_commitment_sha256": sha256_file(commitment_path),
        "candidate_family_results_sha256": sha256_file(candidate_path),
        "validation_scorecard_sha256": sha256_file(scorecard_path),
        "frozen_correction_catalog_sha256": sha256_file(catalog_path),
        "data_access_ledger_sha256": sha256_file(root / "data_access_ledger.jsonl"),
        "sealed_metrics_present": False,
        "sealed_runtime_access_count": 0,
    }
    manifest_path = root / "frozen_selection_manifest.json"
    atomic_write_json(manifest_path, manifest)
    state = {
        "schema_version": S2_QUALIFICATION_SCHEMA_VERSION,
        "qualification_id": qualification_plan["qualification_id"],
        "state": "PROFILE_FROZEN",
        "branch": expected_branch,
        "commit_sha": expected_commit_sha,
        "selected_method_family": selected_family,
        "frozen_catalog_sha256": sha256_file(catalog_path),
        "sealed_commitment_sha256": sha256_file(commitment_path),
        "frozen_selection_manifest_sha256": sha256_file(manifest_path),
    }
    atomic_write_json(root / "qualification_state.json", state)
    os.replace(source_marker, root / "QUALIFICATION_PROFILE_FROZEN")
    if not (root / "SEALED_EVALUATION_NOT_EXECUTED").is_file():
        raise ValueError("sealed Evaluation unopened marker is missing")
    return {"state": "PROFILE_FROZEN", "selected_method_family": selected_family, "frozen_catalog_sha256": sha256_file(catalog_path), "sealed_commitment_sha256": sha256_file(commitment_path), "profile_count": 25}


__all__ = [
    "ANGLE_TIE_TOLERANCE", "CANDIDATE_FAMILIES", "COMPLEXITY_RANK",
    "DATASET_ROLES", "FREEZE_ANCHOR_REPOSITORY_PATH", "FREEZE_ANCHOR_SCHEMA_VERSION",
    "IDENTITY_MATRIX", "LEDGER_STAGES", "MATRIX_DIRECTION",
    "PROFILE_CATALOG_ROLE", "QUALIFICATION_STATES", "S2_QUALIFICATION_SCHEMA_VERSION",
    "SELECTION_RULE", "CalibrationRepresentative", "CandidatePrediction",
    "CandidateProfile", "DataAccessLedger", "QualificationSpec", "ValidationCase",
    "WahbaFitResult", "WahbaObservation", "authorize_unseal",
    "build_calibration_representative", "build_qualification_bundle",
    "build_validation_case", "build_wahba_observations", "evaluate_candidate",
    "evaluate_candidate_families", "fit_candidate_families", "fit_select_freeze",
    "fit_wahba", "predict_candidate", "prepare_plans", "profile_key",
    "qualification_status", "resolve_freeze_anchor_commit", "score_prediction",
    "select_candidate_family",
    "spatial_group_key", "validate_access_ledger", "validate_sealed_commitment",
    "validate_freeze_anchor", "validate_frozen_catalog", "validation_gate_contract",
    "load_freeze_anchor", "verify_freeze_anchor", "verify_freeze_anchor_artifacts",
]
