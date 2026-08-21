"""Owner/controller for native optical-pose evidence and immutable profiles.

The controller is intentionally the boundary between presentation-only UI and
geometry.  Native images are copied from :class:`CameraPane`, persisted as
lossless PNG evidence, and then frozen.  Every pixel observation is interpreted
as an OpenCV native-image pixel (u right, v down) using the exact operating
camera K,D.  Absolute camera poses use world NED and optical RDF; derived stereo
profiles retain OpenCV's ``X_right = R_right_from_left X_left + T`` contract.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
import time
import uuid
from dataclasses import dataclass, fields as _dataclass_fields
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from PyQt5 import QtCore, QtGui

from Workstation.workstation_core.calibration import (
    StereoExtrinsics,
    StereoExtrinsicsProfile,
    compose_stereo_calibration,
)
from Workstation.workstation_core.native_frame_store import NativeFrameArtifact, NativeFrameStore
from Workstation.workstation_core.optical_pose_auto_select import (
    MountainAutoSelection,
    AutoSelectionResult,
    auto_select_horizon,
    auto_select_mountain,
)
from Workstation.workstation_core.operating_intrinsics import (
    OperatingCameraEvidence,
    OperatingCameraIntrinsics,
    OperatingIntrinsicsProfile,
    evaluate_intrinsics_applicability,
)

from Workstation.workstation_core.optical_pose import (
    CameraPoseNed,
    OpticalPoseCandidate,
    OpticalPoseSelection,
    POSE_DOF_NAMES,
    PoseDofMask,
    PoseParametersNed,
    apply_selected_dofs,
    boresight_relative_yaw_right_minus_left_deg,
    estimate_drone_yaw_deg,
    geometric_horizon_dip_rad,
    pose_parameters_from_camera_pose,
    stereo_pose_from_absolute,
)
from Workstation.workstation_core.baseline_alignment import baseline_from_gps
from Workstation.workstation_core.ideal_disparity import target_distance_from_gps
from Workstation.workstation_core.pose_solvers import (
    MountainRotationConfig,
    PnpCorrespondences,
    fit_normalized_horizon_line,
    fixed_center_orientation_ray_residuals,
    pose_target_ray_residuals,
    pose_reprojection_residuals,
    solve_constrained_pose_least_squares,
    solve_mountain_relative_rotation_robust,
    solve_pnp_ensemble,
    solve_spherical_horizon_roll_pitch,
)
from Workstation.workstation_core.recording_session import session_id_now
from Workstation.workstation_core.run_evidence import canonical_json_bytes
from Workstation.workstation_core.rtk_geometry import (
    GpsPositionState,
    estimate_rtk_baseline,
    ned_delta_m,
)
from Workstation.workstation_core.terrain import TerrainMetadata, inspect_geotiff
from Workstation.wiris_runtime_workers import WirisPnPPhotoCaptureWorker


OPTICAL_POSE_CONTROLLER_METHODS = (
    "workstation_optical_pose_snapshot",
    "register_workstation_optical_pose_candidate",
    "apply_workstation_optical_pose_candidate",
    "revert_workstation_optical_pose",
    "capture_workstation_optical_pose_burst",
    "import_workstation_optical_pose_burst",
    "import_workstation_optical_pose_terrain",
    "solve_workstation_optical_pose_horizon",
    "solve_workstation_optical_pose_terrain",
    "solve_workstation_optical_pose_gnss_mountain",
    "solve_workstation_optical_pose_drone_yaw",
    "solve_workstation_optical_pose_drone_rtk_multiview",
    "solve_workstation_optical_pose_combined",
    "prepare_workstation_optical_pose_auto_selection",
    "commit_workstation_optical_pose_auto_selection",
    "auto_select_workstation_optical_pose_horizon",
    "auto_select_workstation_optical_pose_mountain",
    "accept_workstation_optical_pose_suggestion",
    "record_workstation_optical_pose_annotation",
    "workstation_optical_pose_candidate_poses",
    "register_workstation_optical_pose_reference_candidate",
    "set_workstation_optical_pose_mountain_ridge_margin",
    "set_workstation_optical_pose_mountain_max_points",
    "clear_workstation_optical_pose_auto_suggestions",
    "capture_workstation_optical_pose_burst_wiris_ftp",
    "_handle_optical_pose_wiris_ftp_capture_completed",
    "_handle_optical_pose_wiris_ftp_capture_failed",
    "_clear_optical_pose_wiris_ftp_capture_worker",
    "capture_workstation_optical_pose_burst_wiris_ftp_oz",
    "_handle_optical_pose_wiris_ftp_oz_capture_completed",
    "_handle_optical_pose_wiris_ftp_oz_capture_failed",
    "_clear_optical_pose_wiris_ftp_oz_capture_worker",
)


_ALL_DOF = PoseDofMask.from_names(POSE_DOF_NAMES)
_HORIZON_DOF = PoseDofMask(roll_deg=True, pitch_deg=True, yaw_deg=False)

# ``app-receive`` frame and GNSS timestamps share the host monotonic clock,
# but a shared clock alone does not make arbitrarily distant samples
# synchronized.  Keep the capture-evidence gate explicit and conservative;
# callers may retain the samples as WARNING evidence when either bound fails.
DRONE_RTK_FRAME_PAIR_MAX_DELTA_S = 0.350
DRONE_RTK_FRAME_GNSS_MAX_ABS_DELTA_S = 0.350
_YAW_DOF = PoseDofMask(roll_deg=False, pitch_deg=False, yaw_deg=True)
# A single RTK target/ray pair only constrains yaw (see estimate_drone_yaw_deg's
# closed form, which fixes roll/pitch). Two or more non-degenerate target/ray
# pairs over-determine the full rotation with the optical center still fixed,
# so the joint least-squares path below optimizes all three rotation DOF.
_DRONE_ROTATION_DOF = PoseDofMask(roll_deg=True, pitch_deg=True, yaw_deg=True)
_TRANSLATION_DOF = PoseDofMask(roll_deg=False, pitch_deg=False, yaw_deg=False, north_m=True, east_m=True, down_m=True)


@dataclass(frozen=True)
class OpticalPoseAutoSelectJob:
    """Immutable owner snapshot for one background auto-selection run."""

    controller: "OpticalPoseController"
    mode: str
    payload: dict[str, Any]

    def run(self) -> dict[str, Any]:
        return self.controller._compute_auto_selection(self.mode, self.payload)


def _mat_transpose(matrix: Sequence[Sequence[float]]) -> tuple[tuple[float, float, float], ...]:
    return tuple(tuple(float(matrix[row][column]) for row in range(3)) for column in range(3))


def _matmul(
    left: Sequence[Sequence[float]],
    right: Sequence[Sequence[float]],
) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        tuple(
            sum(float(left[row][index]) * float(right[index][column]) for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    )


def _matvec(
    matrix: Sequence[Sequence[float]],
    vector: Sequence[float],
) -> tuple[float, float, float]:
    return tuple(
        sum(float(matrix[row][index]) * float(vector[index]) for index in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def _subtract(left: Sequence[float], right: Sequence[float]) -> tuple[float, float, float]:
    return tuple(float(a) - float(b) for a, b in zip(left, right))  # type: ignore[return-value]


def _finite_vector(values: Iterable[Any], *, length: int, name: str) -> tuple[float, ...]:
    parsed = tuple(float(value) for value in values)
    if len(parsed) != length or not all(math.isfinite(value) for value in parsed):
        raise ValueError(f"{name} must contain {length} finite values")
    return parsed


def _vector_value(value: Any, *, name: str) -> tuple[float, float, float]:
    if isinstance(value, Mapping):
        value = value.get("ned_m", value.get("vector_m", value))
        if isinstance(value, Mapping):
            value = (value.get("north_m"), value.get("east_m"), value.get("down_m"))
    parsed = _finite_vector(value or (), length=3, name=name)
    return parsed  # type: ignore[return-value]


def _mask_observability(mask: PoseDofMask) -> tuple[tuple[str, bool], ...]:
    return tuple((name, bool(getattr(mask, name))) for name in POSE_DOF_NAMES)


def _payload_sha256(value: Mapping[str, Any]) -> str:
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()


_DRONE_RTK_OBSERVATION_SCHEMA = "optical-pose-drone-rtk-capture-observation-v1"


def _gps_state_observation_mapping(value: Any, *, role: str) -> dict[str, Any] | None:
    """JSON-safe deep snapshot of one live ``GpsPositionState``.

    Returns ``None`` when no live sample exists for this role -- the caller
    must not fabricate a position.  ``state.raw`` (the live mavlink message
    object) is deliberately excluded: observation history must never hold a
    reference back into mutable/live telemetry state, only a plain-value copy
    taken at this instant.
    """

    if isinstance(value, Mapping):
        snapshot = dict(value)
        snapshot.setdefault("role", role)
        return snapshot
    if not isinstance(value, GpsPositionState):
        return None
    return {
        "role": role,
        "camera": value.camera,
        "namespace": value.namespace,
        "topic": value.topic,
        "latitude_deg": value.latitude_deg,
        "longitude_deg": value.longitude_deg,
        "altitude_m": value.altitude_m,
        "altitude_ellipsoid_m": value.altitude_ellipsoid_m,
        "fix_type": value.fix_type,
        "rtk_label": value.rtk_label,
        "satellites_used": value.satellites_used,
        "eph_m": value.eph_m,
        "epv_m": value.epv_m,
        "timestamp_s": float(value.timestamp_s),
        "time_source": value.time_source,
        "px4_timestamp_s": value.px4_timestamp_s,
        "px4_timestamp_us": value.px4_timestamp_us,
        "px4_timestamp_sample_us": value.px4_timestamp_sample_us,
        "gps_utc_us": value.gps_utc_us,
        "gps_timestamp_time_relative_us": value.gps_timestamp_time_relative_us,
        "velocity_n_m_s": value.velocity_n_m_s,
        "velocity_e_m_s": value.velocity_e_m_s,
        "velocity_d_m_s": value.velocity_d_m_s,
        "velocity_ned_valid": value.velocity_ned_valid,
        "hdop": value.hdop,
        "vdop": value.vdop,
        "device_id": value.device_id,
        "has_position": value.has_position,
    }


_GPS_STATE_FIELD_NAMES = frozenset(field.name for field in _dataclass_fields(GpsPositionState))


def _gps_state_from_capture_mapping(row: Any) -> GpsPositionState | None:
    """Reconstruct one immutable ``GpsPositionState`` from stored evidence.

    ``row`` is the JSON-safe deep snapshot produced by
    ``_gps_state_observation_mapping`` for a *past* Capture, never live
    telemetry. Returns ``None`` when no usable lat/lon/alt was recorded --
    the caller must not fabricate a position from a partial snapshot.
    """

    if not isinstance(row, Mapping):
        return None
    if row.get("latitude_deg") is None or row.get("longitude_deg") is None or row.get("altitude_m") is None:
        return None
    kwargs = {key: value for key, value in row.items() if key in _GPS_STATE_FIELD_NAMES}
    return GpsPositionState(**kwargs)


def _frame_clock_domain_verified(artifact: NativeFrameArtifact) -> bool:
    """True only when ``artifact.timestamp_s`` is provably host-monotonic.

    ``timestamp_reference`` is set by
    ``camera_pane.infer_camera_timestamp_reference``
    (workstation_core/camera_pane.py:13-28) and resolves to "app-receive"
    only when the frame producer supplied no NTP/source timestamp -- i.e.
    ``timestamp_s`` was stamped with ``time.monotonic()`` at
    ``CameraPane._show_frame`` (workstation_core/camera_pane.py:210). Every
    live ``GpsPositionState.timestamp_s`` in this codebase is stamped with
    that same ``time.monotonic()`` call, in the same process, at MAVLink
    message receipt (``PixhawkTelemetrySubscriber._handle_mavlink_message``,
    workstation_core/pixhawk_attitude.py:861-905). Only when both sides carry
    that "app-receive" marker are the two timestamps known to share a clock
    and therefore safe to subtract.
    """

    return artifact.timestamp_reference == "app-receive"


def _drone_rtk_sync_mapping(
    *,
    left_artifact: NativeFrameArtifact,
    right_artifact: NativeFrameArtifact,
    drone_state: GpsPositionState | None,
) -> dict[str, Any]:
    bounds = {
        "left_right_frame_max_delta_s": DRONE_RTK_FRAME_PAIR_MAX_DELTA_S,
        "frame_to_drone_gnss_max_abs_delta_s": DRONE_RTK_FRAME_GNSS_MAX_ABS_DELTA_S,
    }

    def unverified(reason: str) -> dict[str, Any]:
        return {
            "state": "SYNC_UNVERIFIED",
            "left_frame_to_drone_gnss_s": None,
            "right_frame_to_drone_gnss_s": None,
            "left_right_frame_delta_s": None,
            **bounds,
            "reason": reason,
        }

    if drone_state is None:
        return unverified("no Drone GNSS sample was available at capture")
    if not (
        _frame_clock_domain_verified(left_artifact)
        and _frame_clock_domain_verified(right_artifact)
    ):
        return unverified(
            "frame/GNSS clock-domain identity not confirmed "
            f"(left.timestamp_reference={left_artifact.timestamp_reference!r}, "
            f"right.timestamp_reference={right_artifact.timestamp_reference!r})"
        )
    try:
        drone_timestamp_s = float(drone_state.timestamp_s)
        left_timestamp_s = float(left_artifact.timestamp_s)
        right_timestamp_s = float(right_artifact.timestamp_s)
    except (TypeError, ValueError, OverflowError):
        return unverified("frame/GNSS timestamps are not finite numbers")
    if not all(
        math.isfinite(value)
        for value in (drone_timestamp_s, left_timestamp_s, right_timestamp_s)
    ):
        return unverified("frame/GNSS timestamps are not finite numbers")

    left_right_delta_s = right_timestamp_s - left_timestamp_s
    if abs(left_right_delta_s) > DRONE_RTK_FRAME_PAIR_MAX_DELTA_S:
        return unverified(
            f"left/right frame delta {abs(left_right_delta_s):.6f}s exceeds "
            f"{DRONE_RTK_FRAME_PAIR_MAX_DELTA_S:.6f}s"
        )
    left_delta_s = drone_timestamp_s - left_timestamp_s
    right_delta_s = drone_timestamp_s - right_timestamp_s
    if max(abs(left_delta_s), abs(right_delta_s)) > DRONE_RTK_FRAME_GNSS_MAX_ABS_DELTA_S:
        return unverified(
            "frame/Drone GNSS delta exceeds "
            f"{DRONE_RTK_FRAME_GNSS_MAX_ABS_DELTA_S:.6f}s"
        )
    return {
        "state": "SYNC_OK",
        "left_frame_to_drone_gnss_s": left_delta_s,
        "right_frame_to_drone_gnss_s": right_delta_s,
        "left_right_frame_delta_s": left_right_delta_s,
        **bounds,
        "reason": "",
    }


def _drone_rtk_observation_state(
    *,
    left_gnss: dict[str, Any] | None,
    right_gnss: dict[str, Any] | None,
    drone_gnss: dict[str, Any] | None,
    sync: Mapping[str, Any],
) -> str:
    """Evidence-completeness state -- never a claim about pose accuracy."""

    for row in (left_gnss, right_gnss, drone_gnss):
        if row is None or not row.get("has_position"):
            return "INCOMPLETE"
    all_fixed = all(
        row.get("rtk_label") == "RTK_FIXED" for row in (left_gnss, right_gnss, drone_gnss)
    )
    if all_fixed and sync.get("state") == "SYNC_OK":
        return "EVIDENCE_READY"
    return "WARNING"


def _load_json_mapping(path_value: object, *, label: str) -> tuple[dict[str, Any], str]:
    source = Path(str(path_value or "")).expanduser().resolve(strict=True)
    if not source.is_file():
        raise ValueError(f"{label} must be a regular JSON file")
    raw = source.read_bytes()
    try:
        value = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"{label} is not valid UTF-8 JSON: {exc}") from exc
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} root must be a JSON object")
    return dict(value), hashlib.sha256(raw).hexdigest()


def _pose_mapping(pose: CameraPoseNed) -> dict[str, Any]:
    parameters = pose_parameters_from_camera_pose(pose)
    return {
        "rotation_ned_from_camera_rdf": [list(row) for row in pose.rotation_ned_from_camera_rdf],
        "center_ned_m": list(pose.center_ned_m),
        "virtual_frd_rpy_deg": [parameters.roll_deg, parameters.pitch_deg, parameters.yaw_deg],
        "coordinate_contract": "X_NED=R_NED<-optical_RDF*X_RDF+C_NED; center [north,east,down] metres",
    }


def _candidate_mapping(candidate: OpticalPoseCandidate) -> dict[str, Any]:
    left = pose_parameters_from_camera_pose(candidate.left_pose_ned)
    right = pose_parameters_from_camera_pose(candidate.right_pose_ned)
    residuals = dict(candidate.quality_metrics)
    side_observability = {
        side: dict(values) for side, values in candidate.observability_by_side
    }
    common_observability = {
        name: all(side_observability[side].get(name, False) for side in ("left", "right"))
        for name in POSE_DOF_NAMES
    }
    return {
        "schema": "optical-pose-candidate-v1",
        "candidate_id": candidate.candidate_id,
        "method": candidate.method,
        "left_pose": _pose_mapping(candidate.left_pose_ned),
        "right_pose": _pose_mapping(candidate.right_pose_ned),
        "left_delta_rpy_deg": [left.roll_deg, left.pitch_deg, left.yaw_deg],
        "right_delta_rpy_deg": [right.roll_deg, right.pitch_deg, right.yaw_deg],
        "relative_delta_yaw_deg": candidate.relative_boresight_yaw_right_minus_left_deg,
        "residual": (
            residuals.get("angular_residual_rad")
            if residuals.get("angular_residual_rad") is not None
            else residuals.get("reprojection_error_px")
            if residuals.get("reprojection_error_px") is not None
            else residuals.get("sampson_rms_px_equivalent")
            if residuals.get("sampson_rms_px_equivalent") is not None
            else residuals.get("rms_px_equivalent")
        ),
        "observable_dof": common_observability,
        "observable_dof_by_side": side_observability,
        "applicable": True,
        "state": "WARNING" if candidate.warnings else "CANDIDATE",
        "warnings": list(candidate.warnings),
        "source_observation_ids": list(candidate.source_observation_ids),
        "input_hashes": dict(candidate.input_hashes),
        "quality_metrics": residuals,
        "uncertainty_metrics": dict(candidate.uncertainty_metrics),
        "created_utc_ns": candidate.created_utc_ns,
    }


class OpticalPoseController:
    """Keep native observations, priors, candidates, and selections auditable."""

    _MOUNTAIN_RIDGE_MARGIN_DEFAULT = 0.06
    _MOUNTAIN_RIDGE_MARGIN_MIN = 0.02
    _MOUNTAIN_RIDGE_MARGIN_MAX = 0.15
    _MOUNTAIN_MAX_POINTS_DEFAULT = 12
    _MOUNTAIN_MAX_POINTS_MIN = 4
    _MOUNTAIN_MAX_POINTS_MAX = 40

    def __init__(self, panel: object) -> None:
        self.panel = panel
        self._candidates: dict[str, OpticalPoseCandidate] = {}
        self._candidate_priors: dict[
            str, tuple[CameraPoseNed, CameraPoseNed]
        ] = {}
        self._candidate_solver_context: dict[str, dict[str, PnpCorrespondences]] = {}
        self._candidate_drone_rotation_context: dict[str, dict[str, dict[str, Any]]] = {}
        self._active_selection: OpticalPoseSelection | None = None
        self._active_poses: tuple[CameraPoseNed, CameraPoseNed] | None = None
        self._selection_history: list[OpticalPoseSelection] = []
        self._profile_restore_stack: list[dict[str, Any]] = []
        self._frozen_frames: dict[str, dict[str, Any]] = {}
        self._frozen_intrinsics_profile: OperatingIntrinsicsProfile | None = None
        self._frozen_prior_poses: tuple[CameraPoseNed, CameraPoseNed] | None = None
        self._frozen_capture_monotonic_s: float | None = None
        self._terrain_metadata: TerrainMetadata | None = None
        self._terrain_snapshot: dict[str, Any] | None = None
        # Automatic selections are reviewable evidence, not annotations.  The
        # UI receives these mappings for a separate overlay and only a later
        # explicit accept command is allowed to copy points into annotations.
        self._auto_suggestions: dict[str, dict[str, Any]] = {}
        self._last_accepted_auto_suggestion: dict[str, Any] | None = None
        self._annotation_review: dict[str, Any] | None = None
        # Append-only Drone RTK capture-observation evidence: one immutable
        # entry per live or FTP optical-pose capture, holding the left/right
        # frame identity plus the Left/Right/Drone GNSS snapshot selected for
        # that capture request. A later Capture must never rewrite or drop an
        # earlier entry.
        self._drone_rtk_observations: list[dict[str, Any]] = []
        # Operator-tunable Mountain ridge-boundary match margin (fraction of
        # image height); see optical_pose_auto_select.detect_mountain. An
        # in-session control, not a persisted AppSettings field -- it is
        # meant to be observed and adjusted alongside the frozen photo, not
        # configured ahead of time.
        self._mountain_ridge_margin_fraction = self._MOUNTAIN_RIDGE_MARGIN_DEFAULT
        # Operator-tunable cap on how many spread correspondences Mountain
        # keeps for the final suggestion; same in-session, non-persisted
        # nature as the ridge margin above.
        self._mountain_max_points = self._MOUNTAIN_MAX_POINTS_DEFAULT
        # Carries the caller's dof/annotation-mode/etc. payload from
        # capture_workstation_optical_pose_burst_wiris_ftp across the
        # background capture, so the completion handler can forward it into
        # import_workstation_optical_pose_burst.
        self._pending_wiris_ftp_payload: dict[str, Any] = {}

    def register_workstation_optical_pose_candidate(
        self,
        candidate: OpticalPoseCandidate,
        *,
        prior_poses: tuple[CameraPoseNed, CameraPoseNed] | None = None,
    ) -> dict[str, Any]:
        if not isinstance(candidate, OpticalPoseCandidate):
            raise TypeError("candidate must be OpticalPoseCandidate")
        if candidate.candidate_id in self._candidates:
            raise ValueError(f"duplicate optical pose candidate: {candidate.candidate_id}")
        resolved_prior = prior_poses or self._resolve_prior_poses({}, prefer_frozen=True)
        if (
            not isinstance(resolved_prior, tuple)
            or len(resolved_prior) != 2
            or not all(isinstance(pose, CameraPoseNed) for pose in resolved_prior)
        ):
            raise TypeError("candidate prior_poses must contain left/right CameraPoseNed")
        self._candidates[candidate.candidate_id] = candidate
        self._candidate_priors[candidate.candidate_id] = resolved_prior
        mapping = self._candidate_mapping_with_prior(candidate, resolved_prior)
        self._append_event("pose_candidate_created", mapping)
        return mapping

    def workstation_optical_pose_candidate_poses(self) -> tuple[dict[str, Any], ...]:
        """Return the most recently solved candidate per method, unapplied included.

        This is a read-only preview accessor for geometry-comparison displays;
        it never mutates ``active_intrinsics``/``selected_extrinsics_profile``.
        """

        latest_by_method: dict[str, OpticalPoseCandidate] = {}
        for candidate in self._candidates.values():
            current = latest_by_method.get(candidate.method)
            if current is None or candidate.created_utc_ns > current.created_utc_ns:
                latest_by_method[candidate.method] = candidate
        return tuple(
            {
                "candidate_id": candidate.candidate_id,
                "method": candidate.method,
                "left_pose_ned": candidate.left_pose_ned,
                "right_pose_ned": candidate.right_pose_ned,
                # Axes this solve actually optimized; every other axis in
                # left/right_pose_ned is copied verbatim from the prior pose
                # (see ``_resolve_prior_poses``), not measured by this
                # candidate. Callers must not present inherited axes as if
                # this method observed them.
                "optimized_dofs": candidate.optimized_dofs.selected_names,
            }
            for candidate in latest_by_method.values()
        )

    @staticmethod
    def _mask(payload: Mapping[str, Any]) -> PoseDofMask:
        values = payload.get("dof")
        values = values if isinstance(values, Mapping) else payload
        aliases = {
            "roll_deg": "roll",
            "pitch_deg": "pitch",
            "yaw_deg": "yaw",
            "north_m": "north",
            "east_m": "east",
            "down_m": "down",
        }
        kwargs = {
            internal: bool(values.get(external, values.get(internal, False)))
            for internal, external in aliases.items()
        }
        return PoseDofMask(**kwargs)

    def apply_workstation_optical_pose_candidate(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        candidate_id = str(payload.get("candidate_id") or "")
        candidate = self._candidates.get(candidate_id)
        if candidate is None:
            raise ValueError(f"unknown optical pose candidate: {candidate_id}")
        mask = self._mask(payload)
        if not mask.selected_names:
            raise ValueError("at least one pose DOF must be selected")
        side_observable = {
            side: dict(values) for side, values in candidate.observability_by_side
        }
        unsupported = tuple(
            name
            for name in mask.selected_names
            if not getattr(candidate.optimized_dofs, name)
            or not all(side_observable[side].get(name, False) for side in ("left", "right"))
        )
        if unsupported:
            raise ValueError(
                "selected DOF were not optimized/observable for this candidate: "
                + ", ".join(unsupported)
            )
        applicability = self._intrinsics_snapshot(use_frozen=False)
        if not bool(applicability["applicability"]["geometry_allowed"]):
            raise RuntimeError(
                "exact operating K,D is not applicable: "
                + ", ".join(applicability["applicability"]["reasons"])
            )
        expected_profile_hash = dict(candidate.input_hashes).get(
            "operating_intrinsics_profile_sha256",
            "",
        )
        active_profile_hash = str(applicability["applicability"]["profile_sha256"])
        if expected_profile_hash and expected_profile_hash != active_profile_hash:
            raise RuntimeError(
                "operating K,D changed after candidate creation; capture/solve again"
            )
        prior_left, prior_right = self._candidate_priors[candidate_id]
        left_parameters, right_parameters = self._apply_candidate_with_constraints(
            candidate,
            mask,
            prior_left,
            prior_right,
        )
        active = (left_parameters.to_camera_pose_ned(), right_parameters.to_camera_pose_ned())
        next_pair_sequence = len(getattr(self.panel, "distance_recording_rows", ()) or ())
        previous_profile = getattr(self.panel, "selected_extrinsics_profile", None)
        previous = str(getattr(previous_profile, "profile_id", "") or "")
        selection = OpticalPoseSelection(
            selection_id=f"pose-selection-{uuid.uuid4().hex}",
            candidate_id=candidate_id,
            selected_dofs=mask,
            effective_from_pair_sequence=next_pair_sequence,
            applied_utc_ns=time.time_ns(),
            previous_profile_id=previous,
            quality_snapshot=candidate.quality_metrics,
        )
        self._profile_restore_stack.append(self._capture_profile_state())
        profile = self._install_selected_profile(
            selection,
            candidate,
            active,
            applicability,
        )
        self._active_poses = active
        self._active_selection = selection
        self._selection_history.append(selection)
        self._invalidate_measurement_cache()
        result = self._selection_mapping(selection, active)
        result["selected_extrinsics_profile_id"] = profile.profile_id
        result["selected_extrinsics_content_sha256"] = profile.content_sha256
        result["effective_boundary"] = "next accepted pair (zero-based sequence equals current row count)"
        self._append_event("pose_candidate_applied", result)
        return result

    def _apply_candidate_with_constraints(
        self,
        candidate: OpticalPoseCandidate,
        mask: PoseDofMask,
        prior_left: CameraPoseNed,
        prior_right: CameraPoseNed,
    ) -> tuple[PoseParametersNed, PoseParametersNed]:
        """Apply a candidate without splicing coupled PnP Euler components.

        Horizon and Drone-yaw observations are axis-specific by construction.
        Terrain PnP is coupled, so a partial DOF mask is re-solved against the
        retained raw correspondences with unchecked values fixed at the prior.
        This keeps the UI's explicit mask mathematically meaningful.
        """

        context = self._candidate_solver_context.get(candidate.candidate_id, {})
        if candidate.method == "terrain-pnp-ensemble" and mask.selected_names != candidate.optimized_dofs.selected_names:
            if not context:
                raise RuntimeError(
                    "terrain candidate lacks retained 2-D/3-D observations for constrained re-solve"
                )
            outputs: list[PoseParametersNed] = []
            for side, prior, fallback in (
                ("left", prior_left, candidate.left_pose_ned),
                ("right", prior_right, candidate.right_pose_ned),
            ):
                correspondences = context.get(side)
                if correspondences is None:
                    outputs.append(pose_parameters_from_camera_pose(prior))
                    continue
                result = solve_constrained_pose_least_squares(
                    pose_parameters_from_camera_pose(prior),
                    mask,
                    lambda parameters, corr=correspondences: pose_reprojection_residuals(
                        parameters, corr
                    ),
                )
                if not result.success or not all(math.isfinite(value) for value in result.residuals):
                    raise RuntimeError(
                        f"terrain constrained re-solve failed for {side}: {result.message}"
                    )
                outputs.append(result.parameters)
            return outputs[0], outputs[1]
        left_parameters = apply_selected_dofs(
            pose_parameters_from_camera_pose(prior_left),
            pose_parameters_from_camera_pose(candidate.left_pose_ned),
            mask,
        )
        right_parameters = apply_selected_dofs(
            pose_parameters_from_camera_pose(prior_right),
            pose_parameters_from_camera_pose(candidate.right_pose_ned),
            mask,
        )
        return left_parameters, right_parameters

    def revert_workstation_optical_pose(self, _payload: Mapping[str, Any] | None = None) -> dict[str, Any]:
        previous = self._active_selection
        if self._profile_restore_stack:
            state = self._profile_restore_stack.pop()
            self._restore_profile_state(state)
        else:
            self._active_selection = None
            self._active_poses = None
        self._invalidate_measurement_cache()
        result = {
            "schema": "optical-pose-selection-v1",
            "state": "REVERTED",
            "previous_selection_id": "" if previous is None else previous.selection_id,
            "reverted_utc_ns": time.time_ns(),
        }
        self._append_event("pose_selection_reverted", result)
        return result

    def _capture_drone_rtk_observation(
        self,
        *,
        session_id: str,
        burst_id: str,
        frozen: Mapping[str, dict[str, Any]],
        gnss_snapshot: Mapping[str, Any] | None = None,
        gnss_snapshot_source: str = "live_capture_request",
    ) -> dict[str, Any]:
        """Snapshot Left/Right/Drone GNSS against the just-frozen native burst.

        This is evidence capture only: it never touches candidates, the
        active selection, or any solver. Missing/degraded GNSS is recorded
        as such (state INCOMPLETE/WARNING); it never raises, so a live or FTP
        Horizon/Mountain/Target Capture with no GNSS still succeeds.
        """

        raw_gps = (
            gnss_snapshot
            if gnss_snapshot is not None
            else getattr(self.panel, "latest_gps", {})
        )
        raw_gps = raw_gps if isinstance(raw_gps, Mapping) else {}
        left_artifact: NativeFrameArtifact = frozen["left"]["artifact"]
        right_artifact: NativeFrameArtifact = frozen["right"]["artifact"]
        left_state = raw_gps.get("left")
        right_state = raw_gps.get("right")
        drone_state = raw_gps.get("drone")
        left_gnss = _gps_state_observation_mapping(left_state, role="left")
        right_gnss = _gps_state_observation_mapping(right_state, role="right")
        drone_gnss = _gps_state_observation_mapping(drone_state, role="drone")
        sync = _drone_rtk_sync_mapping(
            left_artifact=left_artifact,
            right_artifact=right_artifact,
            drone_state=(
                drone_state
                if isinstance(drone_state, GpsPositionState)
                else _gps_state_from_capture_mapping(drone_state)
            ),
        )
        state = _drone_rtk_observation_state(
            left_gnss=left_gnss,
            right_gnss=right_gnss,
            drone_gnss=drone_gnss,
            sync=sync,
        )
        observation = {
            "schema": _DRONE_RTK_OBSERVATION_SCHEMA,
            "observation_id": f"drone-rtk-obs-{uuid.uuid4().hex}",
            "sequence": len(self._drone_rtk_observations),
            "session_id": session_id,
            "burst_id": burst_id,
            "frames": {
                "left": left_artifact.to_mapping(),
                "right": right_artifact.to_mapping(),
            },
            "gnss": {
                "left": left_gnss,
                "right": right_gnss,
                "drone": drone_gnss,
            },
            "gnss_snapshot_source": gnss_snapshot_source,
            "synchronization": sync,
            "state": state,
            "created_utc_ns": time.time_ns(),
        }
        self._drone_rtk_observations.append(observation)
        self._append_event("optical_pose_drone_rtk_observation_captured", observation)
        return observation

    def _capture_observation_for_frozen_burst(self) -> dict[str, Any] | None:
        """Return the Drone-RTK observation captured with the frozen burst.

        Matches on ``burst_id`` (== the frozen left artifact's ``frame_id``)
        so a later live-GPS change or a later Capture can never be silently
        substituted for the GNSS evidence that belonged to *this* frozen
        image. Returns ``None`` for an imported burst (which never appends
        an observation) or an empty history.
        """

        left_frozen = self._frozen_frames.get("left")
        artifact = left_frozen.get("artifact") if isinstance(left_frozen, Mapping) else None
        burst_id = getattr(artifact, "frame_id", None)
        if not burst_id:
            return None
        for observation in reversed(self._drone_rtk_observations):
            if observation.get("burst_id") == burst_id:
                return observation
        return None

    def capture_workstation_optical_pose_burst(
        self,
        payload: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Persist one unannotated native frame per side and freeze its state."""

        values = dict(payload or {})
        session_id = str(getattr(self.panel, "recording_session_id", "") or "")
        if not session_id:
            session_id = session_id_now()
            setattr(self.panel, "recording_session_id", session_id)
        resolver = getattr(self.panel, "_recording_session_dir", None)
        if not callable(resolver):
            raise RuntimeError("recording session directory owner is unavailable")
        root = Path(resolver(session_id))
        root.mkdir(parents=True, exist_ok=True)
        ensure = getattr(self.panel, "ensure_workstation_run_evidence", None)
        if callable(ensure):
            ensure(
                "optical_pose_capture_started",
                {"session_id": session_id},
                source="OpticalPoseController",
                coordinate_contract="native OpenCV pixels (u right, v down)",
            )

        captured_monotonic_s = time.monotonic()
        profile = self._operating_intrinsics_profile()
        prior: tuple[CameraPoseNed, CameraPoseNed] | None
        try:
            prior = self._resolve_prior_poses(values, prefer_frozen=False)
        except RuntimeError:
            prior = None
        burst_id = f"pose-burst-{time.time_ns()}-{uuid.uuid4().hex[:8]}"
        store = NativeFrameStore(root)
        frozen: dict[str, dict[str, Any]] = {}
        for side, pane_name in (("left", "left_pane"), ("right", "right_pane")):
            pane = getattr(self.panel, pane_name, None)
            getter = getattr(pane, "latest_native_measurement_frame", None)
            if not callable(getter):
                raise RuntimeError(f"{side} native frame source is unavailable")
            result = getter()
            if not isinstance(result, tuple) or len(result) != 3:
                raise RuntimeError(f"{side} native frame is not available yet")
            timestamp_s, image, raw_metadata = result
            if not isinstance(image, QtGui.QImage) or image.isNull():
                raise RuntimeError(f"{side} native frame is null")
            metadata = dict(raw_metadata) if isinstance(raw_metadata, Mapping) else {}
            metadata.setdefault("timestamp_s", float(timestamp_s))
            metadata.setdefault("native_width", image.width())
            metadata.setdefault("native_height", image.height())
            metadata.setdefault("overlay_applied", False)
            zoom = self._zoom_mapping(side)
            metadata["capture_zoom_index"] = zoom.get("index")
            metadata["capture_zoom_ratio"] = zoom.get("ratio")
            metadata["capture_zoom_updated_monotonic_s"] = zoom.get("updated_monotonic_s")
            artifact = store.persist(
                side=side,
                purpose="optical-pose-burst",
                frame_id=burst_id,
                image=image.copy(),
                metadata=metadata,
            )
            frozen[side] = {
                "image": image.copy(),
                "metadata": metadata,
                "artifact": artifact,
                "zoom": dict(zoom),
            }

        self._frozen_frames = frozen
        self._frozen_intrinsics_profile = profile
        self._frozen_prior_poses = prior
        self._frozen_capture_monotonic_s = captured_monotonic_s
        # A new frozen identity invalidates automatic proposals and accepted
        # annotation provenance from the previous burst.  The append-only
        # event log remains the historical record of that previous burst.
        self._auto_suggestions.clear()
        self._last_accepted_auto_suggestion = None
        self._annotation_review = None
        event_payload = {
            "schema": "optical-pose-frozen-burst-v1",
            "burst_id": burst_id,
            "session_id": session_id,
            "frames": {
                side: frozen[side]["artifact"].to_mapping() for side in ("left", "right")
            },
            "operating_intrinsics_profile_sha256": "" if profile is None else profile.profile_sha256,
            "prior_pose_available": prior is not None,
            "coordinate_contract": "native decoded pixels; u right, v down; no resize/overlay",
        }
        self._append_event("optical_pose_burst_captured", event_payload)
        # Horizon/Mountain/Target Capture is shared evidence acquisition; GNSS
        # availability must never gate it. This only records whatever
        # Left/Right/Drone GNSS happens to be live right now -- it never
        # raises on missing/degraded GNSS.
        self._capture_drone_rtk_observation(session_id=session_id, burst_id=burst_id, frozen=frozen)
        return self.workstation_optical_pose_snapshot()

    def capture_workstation_optical_pose_burst_wiris_ftp(
        self,
        payload: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Trigger a WIRIS HR still + FTP download, then freeze it as a burst.

        A second source alongside ``capture_workstation_optical_pose_burst``,
        not a replacement: once both stills are downloaded, they are handed to
        ``import_workstation_optical_pose_burst`` (``sources.left/right.path``),
        so every downstream solve is unaffected by which source produced the
        frozen burst. Capture and download happen on a background thread, so
        this call only starts the sequence -- the frozen-burst preview updates
        once ``_handle_optical_pose_wiris_ftp_capture_completed`` runs.
        """

        if self.panel.optical_pose_wiris_ftp_thread is not None:
            raise RuntimeError("a WIRIS FTP alignment capture is already in progress")
        if self.panel.optical_pose_wiris_ftp_oz_thread is not None:
            raise RuntimeError("a WIRIS FTP (OZ) alignment capture is already in progress")
        settings = getattr(self.panel, "settings", None)
        if not bool(getattr(settings, "wiris_ftp_enabled", False)):
            raise RuntimeError('WIRIS FTP capture requires "Enable FTP download" in Settings')
        target_infos = self.panel._camera_photo_target_infos()
        if len(target_infos) != 2:
            raise RuntimeError("WIRIS FTP capture failed: both WIRIS control hosts are required")
        session_id = str(getattr(self.panel, "recording_session_id", "") or "")
        if not session_id:
            session_id = session_id_now()
            setattr(self.panel, "recording_session_id", session_id)
        resolver = getattr(self.panel, "_recording_session_dir", None)
        if not callable(resolver):
            raise RuntimeError("recording session directory owner is unavailable")
        root = Path(resolver(session_id)) / "frames"
        capture_id = f"align-{time.time_ns()}-{uuid.uuid4().hex[:8]}"
        kwargs = self.panel._wiris_ftp_worker_kwargs(output_dir=root)
        thread = QtCore.QThread(self.panel)
        worker = WirisPnPPhotoCaptureWorker(
            target_infos,
            capture_id=capture_id,
            capture_channel="align",
            completion_timeout_s=10.0,
            **kwargs,
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._handle_optical_pose_wiris_ftp_capture_completed)
        worker.failed.connect(self.panel._handle_optical_pose_wiris_ftp_capture_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_optical_pose_wiris_ftp_capture_worker)
        self.panel.optical_pose_wiris_ftp_thread = thread
        self.panel.optical_pose_wiris_ftp_worker = worker
        pending_payload = dict(payload or {})
        raw_gps = getattr(self.panel, "latest_gps", {})
        raw_gps = raw_gps if isinstance(raw_gps, Mapping) else {}
        pending_payload["_ftp_request_gnss_snapshot"] = {
            side: _gps_state_observation_mapping(raw_gps.get(side), role=side)
            for side in ("left", "right", "drone")
        }
        pending_payload["_ftp_request_zoom_snapshot"] = {
            side: self._zoom_mapping(side) for side in ("left", "right")
        }
        pending_payload["_ftp_request_monotonic_s"] = time.monotonic()
        self._pending_wiris_ftp_payload = pending_payload
        self.panel._log(f"[정렬] WIRIS FTP Capture {capture_id}: 좌/우 카메라 내부 저장 중")
        thread.start()
        return self.workstation_optical_pose_snapshot()

    def _handle_optical_pose_wiris_ftp_capture_completed(self, result: object) -> None:
        record = result if isinstance(result, dict) else {}
        cameras = record.get("cameras")
        cameras = cameras if isinstance(cameras, dict) else {}
        capture_id = str(record.get("capture_id") or "")
        failures = self._ftp_capture_failures(cameras)
        if set(cameras) != {"left", "right"} or failures:
            self._handle_optical_pose_wiris_ftp_capture_failed(
                "; ".join(failures) or "camera result is missing left/right entries"
            )
            return
        payload = dict(getattr(self, "_pending_wiris_ftp_payload", None) or {})
        payload.pop("sources", None)
        payload["sources"] = self._ftp_import_sources(
            capture_id=capture_id,
            cameras=cameras,
            pending_payload=payload,
        )
        self.panel._log(f"[정렬] WIRIS FTP Capture {capture_id}: 다운로드 완료, 프리즈 프레임 저장 중")
        try:
            self.import_workstation_optical_pose_burst(payload)
        except Exception as exc:
            self.panel._log(f"[정렬] WIRIS FTP Capture {capture_id}: 프리즈 프레임 저장 실패 - {exc}")

    def _handle_optical_pose_wiris_ftp_capture_failed(self, error: str) -> None:
        self.panel._log(f"[정렬] WIRIS FTP Capture 실패: {error}")

    def _clear_optical_pose_wiris_ftp_capture_worker(self) -> None:
        self.panel.optical_pose_wiris_ftp_thread = None
        self.panel.optical_pose_wiris_ftp_worker = None

    @staticmethod
    def _ftp_capture_failures(cameras: dict[object, object]) -> list[str]:
        failures: list[str] = []
        for side, raw_item in cameras.items():
            item = raw_item if isinstance(raw_item, dict) else {}
            state = str(item.get("state") or "failed")
            if state != "completed":
                reason = str(item.get("error") or state)
            elif not bool(item.get("ftp_downloaded")):
                reason = str(item.get("ftp_error") or "FTP download failed")
            else:
                continue
            failures.append(f"{side}: {reason}")
        return failures

    @staticmethod
    def _ftp_import_sources(
        *,
        capture_id: str,
        cameras: Mapping[object, object],
        pending_payload: Mapping[str, Any],
    ) -> dict[str, dict[str, Any]]:
        """Build imported-still provenance without claiming an exposure time.

        CPTR request/completion and FTP publication times bracket the still,
        but none is the camera exposure timestamp.  Preserve those audit
        fields while marking the capture clock unknown.  A request-time GZVV
        snapshot may establish zoom; the later live value must not be used.
        """

        zoom_snapshot = pending_payload.get("_ftp_request_zoom_snapshot")
        zoom_snapshot = zoom_snapshot if isinstance(zoom_snapshot, Mapping) else {}
        request_monotonic_s = pending_payload.get("_ftp_request_monotonic_s")
        sources: dict[str, dict[str, Any]] = {}
        for side in ("left", "right"):
            raw_item = cameras.get(side)
            item = raw_item if isinstance(raw_item, Mapping) else {}
            raw_zoom = zoom_snapshot.get(side)
            zoom = raw_zoom if isinstance(raw_zoom, Mapping) else {}
            source: dict[str, Any] = {
                "path": str(item.get("ftp_local_path") or ""),
                "metadata": {
                    "timestamp_reference": "unknown",
                    "capture_timestamp_known": False,
                    "ftp_capture_id": capture_id,
                    "ftp_remote_path": str(item.get("ftp_remote_path") or ""),
                    "capture_requested_unix_ns": item.get("capture_requested_unix_ns"),
                    "capture_completed_unix_ns": item.get("capture_completed_unix_ns"),
                    "ftp_download_finished_unix_ns": item.get("finished_unix_ns"),
                    "capture_request_monotonic_s": request_monotonic_s,
                },
            }
            if zoom.get("index") is not None:
                source["zoom_index"] = zoom.get("index")
            if zoom.get("ratio") is not None:
                source["zoom_ratio"] = zoom.get("ratio")
            if zoom.get("updated_monotonic_s") is not None:
                source["zoom_updated_monotonic_s"] = zoom.get("updated_monotonic_s")
            sources[side] = source
        return sources

    def capture_workstation_optical_pose_burst_wiris_ftp_oz(
        self,
        payload: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Same as ``capture_workstation_optical_pose_burst_wiris_ftp``, on OZ.

        The WIRIS normal visible operating-zoom still (``SIVI TRUE``) instead
        of the HR still -- lower resolution, faster, no camera reboot risk.
        Downloads land in a separate ``align_oz_capture`` subfolder so they
        never mix with HR downloads, and feed the same
        ``import_workstation_optical_pose_burst`` freeze path.
        """

        if self.panel.optical_pose_wiris_ftp_oz_thread is not None:
            raise RuntimeError("a WIRIS FTP (OZ) alignment capture is already in progress")
        if self.panel.optical_pose_wiris_ftp_thread is not None:
            raise RuntimeError("a WIRIS FTP alignment capture is already in progress")
        settings = getattr(self.panel, "settings", None)
        if not bool(getattr(settings, "wiris_ftp_enabled", False)):
            raise RuntimeError('WIRIS FTP capture requires "Enable FTP download" in Settings')
        target_infos = self.panel._camera_photo_target_infos()
        if len(target_infos) != 2:
            raise RuntimeError("WIRIS FTP capture failed: both WIRIS control hosts are required")
        session_id = str(getattr(self.panel, "recording_session_id", "") or "")
        if not session_id:
            session_id = session_id_now()
            setattr(self.panel, "recording_session_id", session_id)
        resolver = getattr(self.panel, "_recording_session_dir", None)
        if not callable(resolver):
            raise RuntimeError("recording session directory owner is unavailable")
        root = Path(resolver(session_id)) / "frames"
        capture_id = f"align-oz-{time.time_ns()}-{uuid.uuid4().hex[:8]}"
        kwargs = self.panel._wiris_ftp_worker_kwargs(output_dir=root)
        thread = QtCore.QThread(self.panel)
        worker = WirisPnPPhotoCaptureWorker(
            target_infos,
            capture_id=capture_id,
            image_enable_command="SIVI TRUE",
            capture_channel="align_oz",
            completion_timeout_s=10.0,
            **kwargs,
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._handle_optical_pose_wiris_ftp_oz_capture_completed)
        worker.failed.connect(self.panel._handle_optical_pose_wiris_ftp_oz_capture_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_optical_pose_wiris_ftp_oz_capture_worker)
        self.panel.optical_pose_wiris_ftp_oz_thread = thread
        self.panel.optical_pose_wiris_ftp_oz_worker = worker
        pending_payload = dict(payload or {})
        raw_gps = getattr(self.panel, "latest_gps", {})
        raw_gps = raw_gps if isinstance(raw_gps, Mapping) else {}
        pending_payload["_ftp_request_gnss_snapshot"] = {
            side: _gps_state_observation_mapping(raw_gps.get(side), role=side)
            for side in ("left", "right", "drone")
        }
        pending_payload["_ftp_request_zoom_snapshot"] = {
            side: self._zoom_mapping(side) for side in ("left", "right")
        }
        pending_payload["_ftp_request_monotonic_s"] = time.monotonic()
        self._pending_wiris_ftp_payload = pending_payload
        self.panel._log(f"[정렬] WIRIS FTP (OZ) Capture {capture_id}: 좌/우 카메라 내부 저장 중")
        thread.start()
        return self.workstation_optical_pose_snapshot()

    def _handle_optical_pose_wiris_ftp_oz_capture_completed(self, result: object) -> None:
        record = result if isinstance(result, dict) else {}
        cameras = record.get("cameras")
        cameras = cameras if isinstance(cameras, dict) else {}
        capture_id = str(record.get("capture_id") or "")
        failures = self._ftp_capture_failures(cameras)
        if set(cameras) != {"left", "right"} or failures:
            self._handle_optical_pose_wiris_ftp_oz_capture_failed(
                "; ".join(failures) or "camera result is missing left/right entries"
            )
            return
        payload = dict(getattr(self, "_pending_wiris_ftp_payload", None) or {})
        payload.pop("sources", None)
        payload["sources"] = self._ftp_import_sources(
            capture_id=capture_id,
            cameras=cameras,
            pending_payload=payload,
        )
        self.panel._log(f"[정렬] WIRIS FTP (OZ) Capture {capture_id}: 다운로드 완료, 프리즈 프레임 저장 중")
        try:
            self.import_workstation_optical_pose_burst(payload)
        except Exception as exc:
            self.panel._log(f"[정렬] WIRIS FTP (OZ) Capture {capture_id}: 프리즈 프레임 저장 실패 - {exc}")

    def _handle_optical_pose_wiris_ftp_oz_capture_failed(self, error: str) -> None:
        self.panel._log(f"[정렬] WIRIS FTP (OZ) Capture 실패: {error}")

    def _clear_optical_pose_wiris_ftp_oz_capture_worker(self) -> None:
        self.panel.optical_pose_wiris_ftp_oz_thread = None
        self.panel.optical_pose_wiris_ftp_oz_worker = None

    def import_workstation_optical_pose_burst(
        self,
        payload: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Freeze a burst from existing image files instead of the live RTSP pane.

        Sets the exact same ``_frozen_frames``/``_frozen_intrinsics_profile``/
        ``_frozen_prior_poses`` state as ``capture_workstation_optical_pose_burst``,
        so every solve (Horizon/Terrain/Drone-yaw/Combined) works unchanged on
        top of an imported burst. An FTP completion payload may additionally
        carry a request-time GNSS snapshot, which is attached to the imported
        burst as its capture observation. Two uses: reuse an FTP-downloaded
        HR/OZ still (see ``wiris_ftp_*`` settings) instead of a fresh live
        capture, or run offline accuracy evaluation against local test images
        with known ground truth.

        Imported files do not inherit the current RTSP timestamp or zoom.
        Callers may provide capture-time values explicitly; otherwise the
        artifact records only its import-receive time and remains
        ``SYNC_UNVERIFIED`` / geometrically inapplicable until provenance is
        supplied. Camera serial/stabilization identity may still come from
        the declared live rig because those values are not capture clocks.
        """

        values = dict(payload or {})
        sources = values.get("sources")
        sources = sources if isinstance(sources, Mapping) else {}
        missing_sides = [side for side in ("left", "right") if not isinstance(sources.get(side), Mapping)]
        if missing_sides:
            raise ValueError(
                "import requires sources.left/right, each with a 'path': missing "
                + ", ".join(missing_sides)
            )
        session_id = str(getattr(self.panel, "recording_session_id", "") or "")
        if not session_id:
            session_id = session_id_now()
            setattr(self.panel, "recording_session_id", session_id)
        resolver = getattr(self.panel, "_recording_session_dir", None)
        if not callable(resolver):
            raise RuntimeError("recording session directory owner is unavailable")
        root = Path(resolver(session_id))
        root.mkdir(parents=True, exist_ok=True)
        ensure = getattr(self.panel, "ensure_workstation_run_evidence", None)
        if callable(ensure):
            ensure(
                "optical_pose_capture_started",
                {"session_id": session_id},
                source="OpticalPoseController",
                coordinate_contract="native OpenCV pixels (u right, v down)",
            )

        captured_monotonic_s = time.monotonic()
        profile = self._operating_intrinsics_profile()
        try:
            prior = self._resolve_prior_poses(values, prefer_frozen=False)
        except RuntimeError:
            prior = None
        burst_id = f"pose-burst-import-{time.time_ns()}-{uuid.uuid4().hex[:8]}"
        store = NativeFrameStore(root)
        frozen: dict[str, dict[str, Any]] = {}
        source_records: dict[str, dict[str, Any]] = {}
        for side in ("left", "right"):
            row = sources[side]
            source_path = str(row.get("path") or "").strip()
            if not source_path:
                raise ValueError(f"{side}: sources.{side}.path is required")
            resolved_source = Path(source_path).expanduser()
            if not resolved_source.is_file():
                raise ValueError(f"{side}: image file not found: {resolved_source}")
            image = QtGui.QImage(str(resolved_source))
            if image.isNull():
                raise ValueError(f"{side}: failed to decode image: {resolved_source}")
            source_sha256 = hashlib.sha256(resolved_source.read_bytes()).hexdigest()
            # Inherit only stable rig identity.  Copying the complete live
            # RTSP row would incorrectly attach that unrelated preview's
            # timestamp/source clock to an FTP or offline still.
            live_metadata = getattr(self.panel, "camera_frame_metadata", {}) or {}
            live_row = live_metadata.get(side, {}) if isinstance(live_metadata, Mapping) else {}
            metadata: dict[str, Any] = {}
            if isinstance(live_row, Mapping):
                for key in (
                    "camera_serial",
                    "serial",
                    "device_serial",
                    "camera_model",
                    "stabilization_state",
                    "stabilization",
                ):
                    if live_row.get(key) not in (None, ""):
                        metadata[key] = live_row[key]
            overrides = row.get("metadata")
            if isinstance(overrides, Mapping):
                metadata.update(overrides)

            raw_timestamp = row.get("timestamp_s", metadata.get("timestamp_s"))
            raw_reference = row.get(
                "timestamp_reference",
                metadata.get("timestamp_reference"),
            )
            try:
                capture_timestamp_s = (
                    None if raw_timestamp is None else float(raw_timestamp)
                )
            except (TypeError, ValueError, OverflowError):
                capture_timestamp_s = None
            reference = str(raw_reference or "").strip().lower()
            if capture_timestamp_s is None or not math.isfinite(capture_timestamp_s):
                # NativeFrameArtifact currently requires a numeric timestamp.
                # Store the truthful import-receive instant in its own clock
                # domain; never label it as the exposure/app-receive time.
                metadata["timestamp_s"] = captured_monotonic_s
                metadata["timestamp_reference"] = "import-receive"
                metadata["capture_timestamp_known"] = False
                metadata["import_received_monotonic_s"] = captured_monotonic_s
                if raw_timestamp is not None:
                    metadata["capture_timestamp_error"] = "non-finite capture timestamp"
            else:
                metadata["timestamp_s"] = capture_timestamp_s
                metadata["timestamp_reference"] = reference or "unknown"
                metadata["capture_timestamp_known"] = bool(reference) and reference not in {
                    "unknown",
                    "import-receive",
                }
                metadata["import_received_monotonic_s"] = captured_monotonic_s
            metadata["native_width"] = image.width()
            metadata["native_height"] = image.height()
            metadata["overlay_applied"] = False
            metadata["imported_source_path"] = str(resolved_source)
            metadata["imported_source_sha256"] = source_sha256

            # Zoom is capture evidence.  Only an explicit source value (for
            # FTP, the request-time GZVV snapshot) may populate it.
            raw_zoom_index = row.get("zoom_index", metadata.get("capture_zoom_index"))
            raw_zoom_ratio = row.get("zoom_ratio", metadata.get("capture_zoom_ratio"))
            raw_zoom_updated = row.get(
                "zoom_updated_monotonic_s",
                metadata.get("capture_zoom_updated_monotonic_s"),
            )
            try:
                zoom_index = None if raw_zoom_index is None else int(raw_zoom_index)
            except (TypeError, ValueError, OverflowError):
                zoom_index = None
            try:
                zoom_ratio = None if raw_zoom_ratio is None else float(raw_zoom_ratio)
            except (TypeError, ValueError, OverflowError):
                zoom_ratio = None
            if zoom_ratio is not None and not math.isfinite(zoom_ratio):
                zoom_ratio = None
            try:
                zoom_updated = None if raw_zoom_updated is None else float(raw_zoom_updated)
            except (TypeError, ValueError, OverflowError):
                zoom_updated = None
            if zoom_updated is not None and not math.isfinite(zoom_updated):
                zoom_updated = None
            metadata["capture_zoom_index"] = zoom_index
            metadata["capture_zoom_ratio"] = zoom_ratio
            metadata["capture_zoom_updated_monotonic_s"] = zoom_updated
            metadata["capture_zoom_known"] = zoom_index is not None
            artifact = store.persist(
                side=side,
                purpose="optical-pose-burst",
                frame_id=burst_id,
                image=image.copy(),
                metadata=metadata,
            )
            frozen[side] = {
                "image": image.copy(),
                "metadata": metadata,
                "artifact": artifact,
                "zoom": {
                    "index": metadata["capture_zoom_index"],
                    "ratio": metadata["capture_zoom_ratio"],
                    "updated_monotonic_s": metadata["capture_zoom_updated_monotonic_s"],
                },
            }
            source_records[side] = {
                "path": str(resolved_source),
                "sha256": source_sha256,
                "capture_timestamp_known": metadata["capture_timestamp_known"],
                "timestamp_reference": metadata["timestamp_reference"],
                "capture_zoom_known": metadata["capture_zoom_known"],
            }

        self._frozen_frames = frozen
        self._frozen_intrinsics_profile = profile
        self._frozen_prior_poses = prior
        self._frozen_capture_monotonic_s = captured_monotonic_s
        self._auto_suggestions.clear()
        self._last_accepted_auto_suggestion = None
        self._annotation_review = None
        event_payload = {
            "schema": "optical-pose-frozen-burst-v1",
            "burst_id": burst_id,
            "session_id": session_id,
            "frames": {
                side: frozen[side]["artifact"].to_mapping() for side in ("left", "right")
            },
            "operating_intrinsics_profile_sha256": "" if profile is None else profile.profile_sha256,
            "prior_pose_available": prior is not None,
            "import_sources": source_records,
            "coordinate_contract": "native decoded pixels; u right, v down; no resize/overlay",
        }
        self._append_event("optical_pose_burst_imported", event_payload)
        ftp_gnss_snapshot = values.get("_ftp_request_gnss_snapshot")
        if isinstance(ftp_gnss_snapshot, Mapping):
            self._capture_drone_rtk_observation(
                session_id=session_id,
                burst_id=burst_id,
                frozen=frozen,
                gnss_snapshot=ftp_gnss_snapshot,
                gnss_snapshot_source="ftp_capture_request",
            )
        return self.workstation_optical_pose_snapshot()

    @staticmethod
    def _auto_suggestion_side_mapping(
        side: str,
        result: AutoSelectionResult,
        frozen: Mapping[str, Any],
        *,
        annotation_key: str,
    ) -> dict[str, Any]:
        artifact: NativeFrameArtifact = frozen["artifact"]
        value = result.to_mapping()
        value.update(
            {
                "side": side,
                "frame_id": artifact.frame_id,
                "image_sha256": artifact.sha256,
                "source_width_px": artifact.width,
                "source_height_px": artifact.height,
                "annotation_key": annotation_key,
                # ``points`` is the stable generic selector schema.  The
                # annotation key is included so consumers can route it to the
                # existing Horizon/Ridge buffers without guessing.
                annotation_key: list(value.get("points", ())),
            }
        )
        return value

    def _record_auto_suggestion(
        self,
        suggestion: Mapping[str, Any],
    ) -> dict[str, Any]:
        value = dict(suggestion)
        suggestion_id = str(value.get("suggestion_id") or "")
        if not suggestion_id:
            raise ValueError("automatic suggestion_id is required")
        self._auto_suggestions[suggestion_id] = value
        self._append_event(
            "optical_pose_auto_suggestion_created",
            {
                "schema": "optical-pose-auto-suggestion-v1",
                "suggestion_id": suggestion_id,
                "mode": value.get("mode"),
                "state": value.get("state"),
                "detection_method": value.get("detection_method"),
                "confidence": value.get("confidence"),
                "inlier_count": value.get("inlier_count"),
                "residual_px": value.get("residual_px"),
                "source_frames": value.get("source_frames"),
                "input_hashes": value.get("input_hashes"),
                "failure_reason": value.get("failure_reason", ""),
                "eligible": value.get("eligible", False),
                # Preserve the complete native-point proposal in the
                # append-only event, not only its review summary.
                "suggestion": value,
            },
        )
        return self.workstation_optical_pose_snapshot()

    def _auto_select_inputs(
        self, payload: Mapping[str, Any]
    ) -> tuple[OperatingIntrinsicsProfile | None, tuple[CameraPoseNed, CameraPoseNed] | None, list[str]]:
        """Best-effort K,D/prior for Auto-select: degrade quality, never block.

        ``detect_horizon``/``detect_mountain`` already tolerate ``None``
        camera_matrix/distortion/expected_gravity/baseline (skip
        undistortion, skip expected-direction/epipolar weighting) -- the
        only thing that used to force a hard failure here was this wrapper
        insisting on having *something* before it would even try. A frozen
        burst is still required (there is no image to propose points on
        without one); K,D and prior pose are not.
        """

        missing = [side for side in ("left", "right") if side not in self._frozen_frames]
        if missing:
            raise RuntimeError(
                "capture a frozen native left/right burst before solving; missing "
                + ", ".join(missing)
            )
        warnings: list[str] = []
        profile: OperatingIntrinsicsProfile | None = None
        if self._frozen_intrinsics_profile is None:
            warnings.append(
                "no operating K,D was available when this burst was frozen; "
                "detection ran without a camera model (no undistortion, no directional bias)"
            )
        else:
            profile = self._frozen_intrinsics_profile
            applicability = self._intrinsics_snapshot(use_frozen=True)["applicability"]
            if not applicability["geometry_allowed"]:
                warnings.append("non-exact operating K,D: " + ", ".join(applicability["reasons"]))
        priors: tuple[CameraPoseNed, CameraPoseNed] | None = None
        try:
            priors = self._resolve_prior_poses(payload, prefer_frozen=True)
        except RuntimeError:
            warnings.append("no prior pose available; directional/epipolar bias disabled")
        return profile, priors, warnings

    def _resolution_safe_camera(
        self, side: str, profile: OperatingIntrinsicsProfile | None
    ) -> OperatingCameraIntrinsics | None:
        """This side's calibrated camera, or ``None`` if it was calibrated at
        a different resolution than the frozen burst's native frame.

        Undistortion is not merely less accurate when the calibration
        resolution does not match the image being undistorted -- the
        principal point and focal length are only meaningful in the pixel
        grid they were calibrated in, so ``cv2.undistort`` warps the frame
        into something unrelated to the original picture (large black
        borders, the real content compressed into a fisheye-like remainder).
        A resolution-mismatched K,D must be treated exactly like a missing
        camera model, not like a merely non-exact one.
        """

        if profile is None or side not in self._frozen_frames:
            return None
        camera: OperatingCameraIntrinsics = getattr(profile, side)
        image = self._frozen_frames[side]["image"]
        if int(image.width()) != int(camera.width_px) or int(image.height()) != int(camera.height_px):
            return None
        return camera

    def set_workstation_optical_pose_mountain_ridge_margin(
        self, payload: Mapping[str, Any]
    ) -> dict[str, Any]:
        """Set the Mountain ridge-boundary match margin for later Auto-select runs.

        A fraction of image height (see ``detect_mountain``'s
        ``ridge_margin_fraction``); does not itself re-run detection.
        """

        raw = payload.get("ridge_margin_fraction")
        try:
            fraction = float(raw)
        except (TypeError, ValueError) as exc:
            raise ValueError("ridge_margin_fraction must be a finite number") from exc
        if not math.isfinite(fraction):
            raise ValueError("ridge_margin_fraction must be a finite number")
        clamped = min(
            self._MOUNTAIN_RIDGE_MARGIN_MAX,
            max(self._MOUNTAIN_RIDGE_MARGIN_MIN, fraction),
        )
        self._mountain_ridge_margin_fraction = clamped
        return self.workstation_optical_pose_snapshot()

    def set_workstation_optical_pose_mountain_max_points(
        self, payload: Mapping[str, Any]
    ) -> dict[str, Any]:
        """Set how many spread correspondences Mountain keeps for later Auto-select runs.

        See ``detect_mountain``'s ``max_points``; does not itself re-run detection.
        """

        raw = payload.get("max_points")
        try:
            count = int(raw)
        except (TypeError, ValueError) as exc:
            raise ValueError("max_points must be an integer") from exc
        clamped = min(
            self._MOUNTAIN_MAX_POINTS_MAX,
            max(self._MOUNTAIN_MAX_POINTS_MIN, count),
        )
        self._mountain_max_points = clamped
        return self.workstation_optical_pose_snapshot()

    def clear_workstation_optical_pose_auto_suggestions(
        self, payload: Mapping[str, Any] | None = None
    ) -> dict[str, Any]:
        """Discard automatic Horizon/Mountain suggestions.

        Mirrors the UI's "Clear" action: an operator clearing the points
        they are looking at should also discard whatever automatic
        suggestion produced them, not just the manual overlay copy, or a
        stale suggestion keeps reappearing in the snapshot after the manual
        points are gone. With no ``mode`` (or an empty one) every suggestion
        is discarded; with a ``mode`` ("horizon" or "mountain") only that
        mode's suggestions are.
        """

        values = dict(payload or {})
        mode = str(values.get("mode") or "").strip().lower()
        if mode:
            self._auto_suggestions = {
                suggestion_id: value
                for suggestion_id, value in self._auto_suggestions.items()
                if str(value.get("mode") or "") != mode
            }
            if (
                self._last_accepted_auto_suggestion is not None
                and str(self._last_accepted_auto_suggestion.get("mode") or "") == mode
            ):
                self._last_accepted_auto_suggestion = None
        else:
            self._auto_suggestions.clear()
            self._last_accepted_auto_suggestion = None
        return self.workstation_optical_pose_snapshot()

    def _clone_for_auto_select(self) -> "OpticalPoseController":
        """Copy only immutable frozen evidence needed by a background selector."""

        clone = OpticalPoseController(self.panel)
        clone._frozen_frames = {}
        for side in ("left", "right"):
            row = self._frozen_frames.get(side)
            if not isinstance(row, Mapping):
                continue
            image = row.get("image")
            if not isinstance(image, QtGui.QImage) or image.isNull():
                raise RuntimeError(f"{side} frozen native frame is unavailable")
            clone._frozen_frames[side] = {
                "image": image.copy(),
                "metadata": copy.deepcopy(dict(row.get("metadata") or {})),
                "artifact": row.get("artifact"),
                "zoom": copy.deepcopy(dict(row.get("zoom") or {})),
            }
        clone._frozen_intrinsics_profile = self._frozen_intrinsics_profile
        clone._frozen_prior_poses = self._frozen_prior_poses
        clone._active_poses = self._active_poses
        clone._frozen_capture_monotonic_s = self._frozen_capture_monotonic_s
        clone._drone_rtk_observations = copy.deepcopy(self._drone_rtk_observations)
        clone._mountain_ridge_margin_fraction = self._mountain_ridge_margin_fraction
        clone._mountain_max_points = self._mountain_max_points
        return clone

    def prepare_workstation_optical_pose_auto_selection(
        self,
        mode: str,
        payload: Mapping[str, Any] | None = None,
    ) -> OpticalPoseAutoSelectJob:
        """Freeze the current owner evidence before background computation."""

        normalized_mode = str(mode or "").strip().lower()
        if normalized_mode not in {"horizon", "mountain"}:
            raise ValueError(f"unsupported optical-pose auto-selection mode: {mode!r}")
        missing = [side for side in ("left", "right") if side not in self._frozen_frames]
        if missing:
            raise RuntimeError(
                "capture a frozen native left/right burst before solving; missing "
                + ", ".join(missing)
            )
        values = dict(payload or {})
        values["auto_mode"] = normalized_mode
        return OpticalPoseAutoSelectJob(
            controller=self._clone_for_auto_select(),
            mode=normalized_mode,
            payload=values,
        )

    def _compute_auto_selection(
        self,
        mode: str,
        payload: Mapping[str, Any],
    ) -> dict[str, Any]:
        normalized_mode = str(mode or "").strip().lower()
        if normalized_mode == "horizon":
            return self._compute_auto_select_horizon(payload)
        if normalized_mode == "mountain":
            return self._compute_auto_select_mountain(payload)
        raise ValueError(f"unsupported optical-pose auto-selection mode: {mode!r}")

    def commit_workstation_optical_pose_auto_selection(
        self,
        suggestion: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Commit a completed background result on the owner/UI thread only."""

        value = dict(suggestion)
        source_frames = value.get("source_frames")
        if not isinstance(source_frames, Mapping):
            raise ValueError("auto-selection result is missing source frame identities")
        for side in ("left", "right"):
            frozen = self._frozen_frames.get(side)
            artifact = frozen.get("artifact") if isinstance(frozen, Mapping) else None
            expected = source_frames.get(side)
            if not isinstance(expected, Mapping) or artifact is None:
                raise RuntimeError("auto-selection result is stale; capture a new frozen burst")
            if (
                str(expected.get("frame_id") or "") != str(getattr(artifact, "frame_id", ""))
                or str(expected.get("image_sha256") or "") != str(getattr(artifact, "sha256", ""))
            ):
                raise RuntimeError("auto-selection result is stale; capture a new frozen burst")
        return self._record_auto_suggestion(value)

    def auto_select_workstation_optical_pose_horizon(
        self,
        payload: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Create and immediately commit a Horizon suggestion for legacy callers."""

        suggestion = self._compute_auto_select_horizon(dict(payload or {}))
        return self.commit_workstation_optical_pose_auto_selection(suggestion)

    def _compute_auto_select_horizon(
        self,
        payload: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Create a reviewed Horizon suggestion from the frozen native burst."""

        values = dict(payload or {})
        profile, priors, applicability_warnings = self._auto_select_inputs(values)
        results: dict[str, AutoSelectionResult] = {}
        for side_index, side in enumerate(("left", "right")):
            prior = None if priors is None else priors[side_index]
            camera = self._resolution_safe_camera(side, profile)
            expected_gravity = (
                None
                if prior is None
                else _matvec(_mat_transpose(prior.rotation_ned_from_camera_rdf), (0.0, 0.0, 1.0))
            )
            results[side] = auto_select_horizon(
                self._frozen_frames[side]["image"],
                camera_matrix=None if camera is None else camera.camera_matrix,
                distortion_coefficients=None if camera is None else camera.distortion_coefficients,
                expected_gravity_camera_rdf=expected_gravity,
            )
        side_mappings = {
            side: self._auto_suggestion_side_mapping(
                side,
                result,
                self._frozen_frames[side],
                annotation_key="horizon_line",
            )
            for side, result in results.items()
        }
        both_eligible = set(results) == {"left", "right"} and all(
            results[side].eligible for side in ("left", "right")
        )
        any_points = [result for result in results.values() if result.points]
        state = "GOOD" if both_eligible else "LOW_CONFIDENCE" if any_points else "NO_DATA"
        confidence = max((result.confidence for result in results.values()), default=0.0)
        inliers = sum(result.inlier_count for result in results.values())
        residuals = [result.residual_px for result in results.values() if result.residual_px is not None]
        failure_reasons = [result.reason for result in results.values() if result.reason != "ok"]
        suggestion_id = f"auto-horizon-{uuid.uuid4().hex}"
        suggestion = {
            "schema": "optical-pose-auto-suggestion-v1",
            "suggestion_id": suggestion_id,
            "mode": "horizon",
            "state": state,
            "provenance_state": "AUTO_PROPOSED",
            "detection_method": " / ".join(sorted({result.detection_method for result in results.values()})),
            "confidence": confidence,
            "inlier_count": inliers,
            "residual_px": max(residuals) if residuals else None,
            "eligible": both_eligible,
            "failure_reason": "; ".join(dict.fromkeys(failure_reasons)),
            "warnings": [
                *applicability_warnings,
                "automatic Horizon selection proposes endpoints only; no solver was executed",
                "flat horizon observes Roll/Pitch, not Yaw",
            ],
            "sides": side_mappings,
            "source_frames": {
                side: {
                    "frame_id": self._frozen_frames[side]["artifact"].frame_id,
                    "image_sha256": self._frozen_frames[side]["artifact"].sha256,
                }
                for side in ("left", "right")
            },
            "input_hashes": {
                **(
                    {"operating_intrinsics_profile_sha256": profile.profile_sha256}
                    if profile is not None
                    else {}
                ),
                **(
                    {
                        "prior_pose_sha256": _payload_sha256(
                            {"left": _pose_mapping(priors[0]), "right": _pose_mapping(priors[1])}
                        )
                    }
                    if priors is not None
                    else {}
                ),
            },
            "created_utc_ns": time.time_ns(),
        }
        return suggestion

    def auto_select_workstation_optical_pose_mountain(
        self,
        payload: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Create and immediately commit a Mountain suggestion for legacy callers."""

        suggestion = self._compute_auto_select_mountain(dict(payload or {}))
        return self.commit_workstation_optical_pose_auto_selection(suggestion)

    def _compute_auto_select_mountain(
        self,
        payload: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Create a reviewed stereo-feature Mountain suggestion.

        The selector may still return image matches when a baseline is
        temporarily unavailable so the operator can inspect the evidence, but
        solver approval remains blocked until the existing GNSS baseline
        contract is satisfied.
        """

        values = dict(payload or {})
        profile, priors, applicability_warnings = self._auto_select_inputs(values)
        baseline_ned: tuple[float, float, float] | None = None
        baseline_details: dict[str, Any] = {}
        baseline_warning = ""
        try:
            baseline_ned, baseline_details = self._resolve_gnss_baseline_for_mountain(values)
        except (RuntimeError, ValueError) as exc:
            baseline_warning = str(exc)
        prior_rotation = (
            None
            if priors is None
            else stereo_pose_from_absolute(*priors).rotation_right_from_left_rdf
        )
        left_camera = self._resolution_safe_camera("left", profile)
        right_camera = self._resolution_safe_camera("right", profile)
        result: MountainAutoSelection = auto_select_mountain(
            self._frozen_frames["left"]["image"],
            self._frozen_frames["right"]["image"],
            left_camera_matrix=None if left_camera is None else left_camera.camera_matrix,
            right_camera_matrix=None if right_camera is None else right_camera.camera_matrix,
            left_distortion_coefficients=None if left_camera is None else left_camera.distortion_coefficients,
            right_distortion_coefficients=None if right_camera is None else right_camera.distortion_coefficients,
            baseline_ned_m=baseline_ned,
            prior_rotation_right_from_left_rdf=prior_rotation,
            ridge_margin_fraction=self._mountain_ridge_margin_fraction,
            max_points=self._mountain_max_points,
        )
        sides = {
            "left": self._auto_suggestion_side_mapping(
                "left", result.left, self._frozen_frames["left"], annotation_key="ridge_polyline"
            ),
            "right": self._auto_suggestion_side_mapping(
                "right", result.right, self._frozen_frames["right"], annotation_key="ridge_polyline"
            ),
        }
        warnings = [*applicability_warnings, *result.warnings]
        if baseline_warning:
            warnings.append(f"GNSS baseline unavailable during suggestion: {baseline_warning}")
        suggestion_id = f"auto-mountain-{uuid.uuid4().hex}"
        input_hashes: dict[str, str] = {}
        if profile is not None:
            input_hashes["operating_intrinsics_profile_sha256"] = profile.profile_sha256
        if priors is not None:
            input_hashes["prior_pose_sha256"] = _payload_sha256(
                {"left": _pose_mapping(priors[0]), "right": _pose_mapping(priors[1])}
            )
        if baseline_details:
            input_hashes["gnss_baseline_sha256"] = _payload_sha256(baseline_details)
        suggestion = {
            "schema": "optical-pose-auto-suggestion-v1",
            "suggestion_id": suggestion_id,
            "mode": "mountain",
            "state": result.state,
            "provenance_state": "AUTO_PROPOSED",
            "detection_method": result.detection_method,
            "confidence": result.confidence,
            "inlier_count": result.inlier_count,
            "match_count": result.match_count,
            "residual_px": result.residual_px,
            "eligible": bool(result.eligible and baseline_ned is not None),
            "failure_reason": result.failure_reason or baseline_warning,
            "warnings": warnings,
            "sides": sides,
            "source_frames": {
                side: {
                    "frame_id": self._frozen_frames[side]["artifact"].frame_id,
                    "image_sha256": self._frozen_frames[side]["artifact"].sha256,
                }
                for side in ("left", "right")
            },
            "input_hashes": input_hashes,
            "gnss_baseline": dict(baseline_details),
            "created_utc_ns": time.time_ns(),
        }
        return suggestion

    def accept_workstation_optical_pose_suggestion(
        self,
        payload: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Record explicit review approval; the UI then copies points locally."""

        suggestion_id = str(payload.get("suggestion_id") or "")
        if not suggestion_id:
            raise ValueError("Use suggestion requires an explicit suggestion_id")
        suggestion = self._auto_suggestions.get(suggestion_id)
        if suggestion is None:
            raise ValueError(f"unknown automatic optical-pose suggestion: {suggestion_id}")
        if not bool(suggestion.get("eligible", False)) or str(suggestion.get("state", "")) != "GOOD":
            raise ValueError(
                "automatic suggestion is not eligible; low-confidence or incomplete suggestions "
                "remain overlay-only and do not replace manual annotations"
            )
        accepted = json.loads(json.dumps(suggestion, ensure_ascii=False))
        accepted["provenance_state"] = "AUTO_ACCEPTED"
        accepted["accepted_utc_ns"] = time.time_ns()
        self._last_accepted_auto_suggestion = accepted
        accepted_annotations = self._annotation_mappings_from_suggestion(accepted)
        if accepted_annotations:
            self._annotation_review = self._build_annotation_review(
                suggestion_id=suggestion_id,
                provenance_state="AUTO_ACCEPTED",
                annotation_mode=str(accepted.get("mode") or ""),
                annotations=accepted_annotations,
            )
        self._append_event(
            "optical_pose_auto_suggestion_accepted",
            {
                "schema": "optical-pose-auto-suggestion-acceptance-v1",
                "suggestion_id": suggestion_id,
                "mode": accepted.get("mode"),
                "provenance_state": "AUTO_ACCEPTED",
                "source_frames": accepted.get("source_frames"),
                "input_hashes": accepted.get("input_hashes"),
                "suggestion": accepted,
                "annotation_review": self._annotation_review,
            },
        )
        return self.workstation_optical_pose_snapshot()

    @staticmethod
    def _annotation_mappings_from_suggestion(
        suggestion: Mapping[str, Any],
    ) -> dict[str, dict[str, Any]]:
        """Convert accepted suggestion sides to the existing annotation schema."""

        mode = str(suggestion.get("mode") or "").lower()
        annotation_key = {
            "horizon": "horizon_line",
            "mountain": "ridge_polyline",
        }.get(mode)
        if annotation_key is None:
            return {}
        sides = suggestion.get("sides")
        if not isinstance(sides, Mapping):
            return {}
        annotations: dict[str, dict[str, Any]] = {}
        for side in ("left", "right"):
            row = sides.get(side)
            if not isinstance(row, Mapping):
                continue
            points = row.get("points", row.get(annotation_key, ()))
            if not isinstance(points, (tuple, list)):
                points = []
            annotation: dict[str, Any] = {
                "frame_id": str(row.get("frame_id") or ""),
                "image_sha256": str(row.get("image_sha256") or ""),
                "source_width_px": row.get("source_width_px"),
                "source_height_px": row.get("source_height_px"),
                "horizon_line": [],
                "ridge_polyline": [],
                "target_point": [],
            }
            annotation[annotation_key] = list(points)
            annotations[side] = annotation
        return annotations

    def _build_annotation_review(
        self,
        *,
        suggestion_id: str,
        provenance_state: str,
        annotation_mode: str,
        annotations: Mapping[str, Mapping[str, Any]],
    ) -> dict[str, Any]:
        normalized: dict[str, dict[str, Any]] = {}
        for side in ("left", "right"):
            annotation = annotations.get(side)
            if not isinstance(annotation, Mapping):
                continue
            self._validate_annotation_identity(side, annotation)
            normalized[side] = json.loads(
                json.dumps(dict(annotation), ensure_ascii=False)
            )
        if not normalized:
            raise ValueError("optical-pose annotation review requires a left or right annotation")
        return {
            "schema": "optical-pose-annotation-review-v1",
            "suggestion_id": str(suggestion_id or ""),
            "provenance_state": str(provenance_state or "MANUAL").upper(),
            "annotation_mode": str(annotation_mode or ""),
            "annotations": normalized,
            "annotation_hashes": {
                side: _payload_sha256(annotation)
                for side, annotation in normalized.items()
            },
            "updated_utc_ns": time.time_ns(),
        }

    def record_workstation_optical_pose_annotation(
        self,
        payload: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Persist the current committed annotation and its provenance state."""

        values = dict(payload or {})
        annotations = self._annotation_root(values)
        if not annotations:
            raise ValueError("optical-pose annotation update requires at least one annotation")
        provenance_state = str(values.get("provenance_state") or "MANUAL").upper()
        if provenance_state not in {"MANUAL", "AUTO_ACCEPTED", "MANUAL_EDITED"}:
            raise ValueError(f"unsupported optical-pose annotation provenance: {provenance_state}")
        review = self._build_annotation_review(
            suggestion_id=str(values.get("suggestion_id") or ""),
            provenance_state=provenance_state,
            annotation_mode=str(values.get("annotation_mode") or ""),
            annotations=annotations,
        )
        self._annotation_review = review
        self._append_event("optical_pose_annotations_updated", review)
        return self.workstation_optical_pose_snapshot()

    def import_workstation_optical_pose_terrain(
        self,
        payload: Mapping[str, Any],
    ) -> dict[str, Any]:
        source_text = str(payload.get("path") or payload.get("source_path") or "").strip()
        if not source_text:
            raise ValueError("GeoTIFF path is required")
        vertical_datum = str(payload.get("vertical_datum") or "unknown")
        metadata = inspect_geotiff(Path(source_text), vertical_datum=vertical_datum)
        importer = getattr(self.panel, "import_workstation_session_artifact", None)
        artifact: Mapping[str, Any] | None = None
        if callable(importer):
            artifact = importer(
                source_text,
                role="terrain/geotiff",
                media_type="image/tiff; application=geotiff",
                metadata={
                    "sha256": metadata.sha256,
                    "horizontal_datum": metadata.horizontal_datum,
                    "vertical_datum": metadata.vertical_datum,
                    "crs_wkt": metadata.crs_wkt,
                },
            )
        self._terrain_metadata = metadata
        self._terrain_snapshot = {
            **{key: value for key, value in metadata.to_mapping().items() if key != "path"},
            "source_name": Path(source_text).name,
            "artifact_relative_path": "" if artifact is None else str(artifact.get("relative_path") or ""),
        }
        self._append_event("terrain_geotiff_imported", self._terrain_snapshot)
        return self.workstation_optical_pose_snapshot()

    _HORIZON_CLICK_UNCERTAINTY_PX = 1.0
    _HORIZON_MIN_ENDPOINT_SEPARATION_FRACTION = 0.25
    _HORIZON_MODELS = ("flat_local", "spherical_geometric")

    def _solve_horizon_roll_pitch(
        self,
        normalized_points: Sequence[tuple[float, float]],
        *,
        horizon_model: str,
        dip_rad: float | None,
        prior: CameraPoseNed,
    ) -> tuple[float, float, float, float]:
        """Return ``(roll_deg, pitch_deg, rms_normalized, max_abs_normalized)``.

        ``flat_local`` treats the horizon as ``g.r=0`` (a flat/unrefracted
        line fit); the prior's expected gravity only resolves the ``g`` vs
        ``-g`` projective sign ambiguity, it never supplies the Roll/Pitch
        value itself. ``spherical_geometric`` instead solves ``g.r=sin(dip)``
        per sampled ray with a fixed prior Yaw -- Yaw and position are never
        variables of this fit.
        """

        before = pose_parameters_from_camera_pose(prior)
        if horizon_model == "spherical_geometric":
            assert dip_rad is not None
            rays = tuple((x, y, 1.0) for x, y in normalized_points)
            result = solve_spherical_horizon_roll_pitch(
                rays,
                dip_rad=dip_rad,
                prior_roll_deg=before.roll_deg,
                prior_pitch_deg=before.pitch_deg,
                prior_yaw_deg=before.yaw_deg,
            )
            return result.roll_deg, result.pitch_deg, result.rms_normalized, result.max_abs_normalized
        expected_gravity = _matvec(
            _mat_transpose(prior.rotation_ned_from_camera_rdf),
            (0.0, 0.0, 1.0),
        )
        fit = fit_normalized_horizon_line(
            normalized_points,
            expected_gravity_camera_rdf=expected_gravity,
        )
        return fit.roll_deg, fit.pitch_deg, fit.rms_normalized, fit.max_abs_normalized

    def _horizon_endpoint_sensitivity_deg(
        self,
        *,
        raw_points: Sequence[tuple[float, float]],
        camera: Any,
        horizon_model: str,
        dip_rad: float | None,
        prior: CameraPoseNed,
        nominal_roll_deg: float,
        nominal_pitch_deg: float,
    ) -> tuple[float, float]:
        """Deterministic one-axis-at-a-time +/-1px endpoint perturbation.

        This is a worst-case manual-click sensitivity, not a statistical
        1-sigma uncertainty: each of the two endpoints is perturbed along u
        and v independently by ``_HORIZON_CLICK_UNCERTAINTY_PX``, the same
        model is re-solved, and the maximum absolute Roll/Pitch delta from
        the nominal solve is retained.
        """

        max_roll_delta = 0.0
        max_pitch_delta = 0.0
        step = self._HORIZON_CLICK_UNCERTAINTY_PX
        for endpoint_index in range(2):
            for axis in (0, 1):
                for sign in (1.0, -1.0):
                    perturbed = list(raw_points)
                    u, v = perturbed[endpoint_index]
                    if axis == 0:
                        u = u + sign * step
                    else:
                        v = v + sign * step
                    perturbed[endpoint_index] = (u, v)
                    sampled = self._sample_horizon_pixels(tuple(perturbed))
                    normalized = self._undistort_pixels(camera, sampled)
                    roll_deg, pitch_deg, _rms, _max_abs = self._solve_horizon_roll_pitch(
                        normalized,
                        horizon_model=horizon_model,
                        dip_rad=dip_rad,
                        prior=prior,
                    )
                    max_roll_delta = max(max_roll_delta, abs(roll_deg - nominal_roll_deg))
                    max_pitch_delta = max(max_pitch_delta, abs(pitch_deg - nominal_pitch_deg))
        return max_roll_delta, max_pitch_delta

    def solve_workstation_optical_pose_horizon(
        self,
        payload: Mapping[str, Any],
    ) -> dict[str, Any]:
        profile, applicability = self._require_frozen_intrinsics()
        priors = self._resolve_prior_poses(payload, prefer_frozen=True)
        annotations = self._annotation_root(payload)
        horizon_model = str(payload.get("horizon_model") or "flat_local")
        if horizon_model not in self._HORIZON_MODELS:
            raise ValueError(f"unsupported horizon_model: {horizon_model!r}")
        dip_rad: float | None = None
        camera_height_m: float | None = None
        if horizon_model == "spherical_geometric":
            height_raw = payload.get("camera_height_above_reference_surface_m")
            if height_raw is None:
                raise ValueError(
                    "spherical_geometric horizon requires an explicit "
                    "camera_height_above_reference_surface_m; GNSS ellipsoid altitude "
                    "is not automatically substituted"
                )
            try:
                camera_height_m = float(height_raw)
            except (TypeError, ValueError) as exc:
                raise ValueError("camera_height_above_reference_surface_m must be numeric") from exc
            if not math.isfinite(camera_height_m) or camera_height_m < 0.0:
                raise ValueError("camera_height_above_reference_surface_m must be a finite, non-negative metres value")
            dip_rad = geometric_horizon_dip_rad(camera_height_m)

        solved: dict[str, dict[str, Any]] = {}
        for side, prior, camera in (
            ("left", priors[0], profile.left),
            ("right", priors[1], profile.right),
        ):
            annotation = annotations.get(side)
            if not isinstance(annotation, Mapping):
                continue
            self._validate_annotation_identity(side, annotation)
            points = self._annotation_points(annotation, "horizon_line")
            if len(points) < 2:
                continue
            sensitivity: tuple[float, float] | None = None
            if len(points) == 2:
                width = float(annotation["source_width_px"])
                separation = math.hypot(points[1][0] - points[0][0], points[1][1] - points[0][1])
                minimum_separation = self._HORIZON_MIN_ENDPOINT_SEPARATION_FRACTION * width
                if separation < minimum_separation:
                    raise ValueError(
                        f"{side}: horizon endpoints are only {separation:.1f}px apart; "
                        f"at least {minimum_separation:.1f}px ({self._HORIZON_MIN_ENDPOINT_SEPARATION_FRACTION:.0%} "
                        "of image width) is required so +/-1px click noise stays bounded"
                    )
            sampled = self._sample_horizon_pixels(points)
            normalized = self._undistort_pixels(camera, sampled)
            roll_deg, pitch_deg, rms_normalized, max_abs_normalized = self._solve_horizon_roll_pitch(
                normalized,
                horizon_model=horizon_model,
                dip_rad=dip_rad,
                prior=prior,
            )
            if not (math.isfinite(roll_deg) and math.isfinite(pitch_deg)):
                raise ValueError(f"{side}: horizon solve returned a non-finite Roll/Pitch")
            if len(points) == 2:
                sensitivity = self._horizon_endpoint_sensitivity_deg(
                    raw_points=points,
                    camera=camera,
                    horizon_model=horizon_model,
                    dip_rad=dip_rad,
                    prior=prior,
                    nominal_roll_deg=roll_deg,
                    nominal_pitch_deg=pitch_deg,
                )
            before = pose_parameters_from_camera_pose(prior)
            updated = PoseParametersNed(
                roll_deg,
                pitch_deg,
                before.yaw_deg,
                before.north_m,
                before.east_m,
                before.down_m,
            ).to_camera_pose_ned()
            solved[side] = {
                "pose": updated,
                "rms_normalized": rms_normalized,
                "max_abs_normalized": max_abs_normalized,
                "sensitivity_deg": sensitivity,
                "annotation": annotation,
            }
        if not solved:
            raise ValueError(
                "frozen horizon annotations require at least two native points on left or right"
            )
        left = solved.get("left", {}).get("pose", priors[0])
        right = solved.get("right", {}).get("pose", priors[1])
        quality: list[tuple[str, float]] = []
        source_ids: list[str] = []
        input_hashes: list[tuple[str, str]] = [
            ("operating_intrinsics_profile_sha256", profile.profile_sha256),
            ("horizon_model", horizon_model),
        ]
        if camera_height_m is not None:
            input_hashes.append(
                ("camera_height_above_reference_surface_m", f"{camera_height_m:.6f}")
            )
        for side, row in solved.items():
            approximate_px = float(row["rms_normalized"]) * math.sqrt(
                float(getattr(profile, side).camera_matrix[0][0])
                * float(getattr(profile, side).camera_matrix[1][1])
            )
            quality.extend(
                (
                    (f"{side}_rms_normalized", float(row["rms_normalized"])),
                    (f"{side}_max_abs_normalized", float(row["max_abs_normalized"])),
                    (f"{side}_rms_px_equivalent", approximate_px),
                )
            )
            sensitivity = row["sensitivity_deg"]
            if sensitivity is not None:
                roll_sensitivity, pitch_sensitivity = sensitivity
                quality.extend(
                    (
                        (f"{side}_roll_sensitivity_1px_deg", float(roll_sensitivity)),
                        (f"{side}_pitch_sensitivity_1px_deg", float(pitch_sensitivity)),
                    )
                )
            source_ids.append(str(row["annotation"]["frame_id"]))
            input_hashes.append((f"{side}_annotation_sha256", _payload_sha256(row["annotation"])))
        warnings: list[str] = [
            "yaw and optical-center position are unobservable from a horizon",
            "the prior expected gravity resolves only the g vs -g projective sign; it is not "
            "the source of the recovered Roll/Pitch value",
            "rms_normalized is a line/model fit-quality metric, not a pose uncertainty; see "
            "*_roll_sensitivity_1px_deg / *_pitch_sensitivity_1px_deg for a worst-case "
            "+/-1px manual endpoint-click sensitivity",
            "use only a physical/geometric horizon compatible with the selected model; a "
            "mountain ridge, building skyline, or tree line is not a valid horizon input "
            "(use the Mountain method for a ridge)",
        ]
        if horizon_model == "flat_local":
            warnings.append("flat/unrefracted horizon model; Earth curvature dip is not applied")
        else:
            warnings.append(
                "geometric spherical-Earth horizon; atmospheric refraction is not modeled"
            )
        candidate = OpticalPoseCandidate(
            candidate_id=f"horizon-{uuid.uuid4().hex}",
            method="flat-horizon-roll-pitch" if horizon_model == "flat_local" else "spherical-horizon-roll-pitch",
            left_pose_ned=left,
            right_pose_ned=right,
            optimized_dofs=_HORIZON_DOF,
            observability=_mask_observability(_HORIZON_DOF),
            source_observation_ids=tuple(dict.fromkeys(source_ids)),
            input_hashes=tuple(input_hashes),
            quality_metrics=tuple(quality),
            warnings=tuple(warnings),
            created_utc_ns=time.time_ns(),
            observability_by_side=tuple(
                (
                    side,
                    _mask_observability(
                        _HORIZON_DOF if side in solved else PoseDofMask.from_names(())
                    ),
                )
                for side in ("left", "right")
            ),
        )
        self.register_workstation_optical_pose_candidate(candidate, prior_poses=priors)
        return self.workstation_optical_pose_snapshot()

    def solve_workstation_optical_pose_terrain(
        self,
        payload: Mapping[str, Any],
    ) -> dict[str, Any]:
        if self._terrain_metadata is None:
            raise RuntimeError("import and inspect a GeoTIFF before terrain PnP")
        if not self._terrain_metadata.full_pnp_height_ready:
            raise RuntimeError(
                "terrain PnP requires an explicitly declared vertical datum; current value is unknown"
            )
        profile, _applicability = self._require_frozen_intrinsics()
        priors = self._resolve_prior_poses(payload, prefer_frozen=True)
        annotations = self._annotation_root(payload)
        correspondence_root = payload.get("correspondences")
        correspondence_root = correspondence_root if isinstance(correspondence_root, Mapping) else {}
        correspondence_file_sha256 = ""
        if not correspondence_root and payload.get("correspondences_path"):
            loaded, correspondence_file_sha256 = _load_json_mapping(
                payload.get("correspondences_path"),
                label="terrain correspondence JSON",
            )
            correspondence_root = loaded.get("correspondences", loaded)
            if not isinstance(correspondence_root, Mapping):
                raise ValueError(
                    "terrain correspondence JSON must contain a left/right object map"
                )
        solved: dict[str, Any] = {}
        correspondence_context: dict[str, PnpCorrespondences] = {}
        failures: list[str] = []
        for side, camera in (("left", profile.left), ("right", profile.right)):
            annotation = annotations.get(side)
            row = correspondence_root.get(side)
            if not isinstance(annotation, Mapping) or not isinstance(row, Mapping):
                continue
            self._validate_annotation_identity(side, annotation)
            image_points = self._annotation_points(annotation, "ridge_polyline")
            object_points = row.get("object_points_ned_m")
            if not isinstance(object_points, (tuple, list)):
                failures.append(
                    f"{side}: provide object_points_ned_m matched one-to-one with frozen ridge pixels"
                )
                continue
            if len(image_points) != len(object_points):
                failures.append(
                    f"{side}: ridge/object correspondence counts differ "
                    f"({len(image_points)} != {len(object_points)})"
                )
                continue
            try:
                correspondences = PnpCorrespondences.from_raw_camera_pixels(
                    object_points_ned_m=object_points,
                    image_points_px=image_points,
                    camera_matrix=camera.camera_matrix,
                    distortion_coefficients=camera.distortion_coefficients,
                )
                ensemble = solve_pnp_ensemble(correspondences)
                best = ensemble.best_candidate
                if best is None:
                    failures.append(f"{side}: all PnP solvers failed: {ensemble.failures}")
                    continue
                solved[side] = (best, ensemble, annotation)
                correspondence_context[side] = correspondences
            except Exception as exc:
                failures.append(f"{side}: {exc}")
        if not solved:
            detail = "; ".join(failures) or (
                "provide per-side frozen ridge annotations plus explicit object_points_ned_m "
                "matched one-to-one; a skyline polyline alone is not a 3D/2D PnP correspondence set"
            )
            raise ValueError(f"terrain PnP correspondence evidence is incomplete: {detail}")
        left = solved.get("left", (None,))[0].pose_ned if "left" in solved else priors[0]
        right = solved.get("right", (None,))[0].pose_ned if "right" in solved else priors[1]
        full_rank = all(row[0].jacobian_diagnostics.full_rank for row in solved.values())
        observable = _ALL_DOF if full_rank else PoseDofMask.from_names(())
        quality: list[tuple[str, float]] = []
        uncertainty: list[tuple[str, float]] = []
        source_ids: list[str] = [f"terrain:{self._terrain_metadata.sha256}"]
        hashes: list[tuple[str, str]] = [
            ("operating_intrinsics_profile_sha256", profile.profile_sha256),
            ("terrain_geotiff_sha256", self._terrain_metadata.sha256),
        ]
        if correspondence_file_sha256:
            hashes.append(("terrain_correspondence_json_sha256", correspondence_file_sha256))
        for side, (best, _ensemble, annotation) in solved.items():
            quality.extend(
                (
                    (f"{side}_reprojection_rmse_px", best.reprojection_rmse),
                    (f"{side}_reprojection_p95_px", best.reprojection_p95),
                    (f"{side}_positive_depth_ratio", best.positive_depth_ratio),
                )
            )
            condition = float(best.jacobian_diagnostics.condition_number)
            if math.isfinite(condition) and condition >= 0.0:
                uncertainty.append((f"{side}_jacobian_condition", condition))
            source_ids.append(str(annotation["frame_id"]))
            hashes.append((f"{side}_annotation_sha256", _payload_sha256(annotation)))
        quality.append(
            ("reprojection_error_px", max(row[0].reprojection_rmse for row in solved.values()))
        )
        warnings = list(failures)
        if not full_rank:
            warnings.append("PnP Jacobian is rank deficient; no DOF is marked applicable")
        candidate = OpticalPoseCandidate(
            candidate_id=f"terrain-pnp-{uuid.uuid4().hex}",
            method="terrain-pnp-ensemble",
            left_pose_ned=left,
            right_pose_ned=right,
            optimized_dofs=_ALL_DOF,
            observability=_mask_observability(observable),
            source_observation_ids=tuple(dict.fromkeys(source_ids)),
            input_hashes=tuple(hashes),
            quality_metrics=tuple(quality),
            uncertainty_metrics=tuple(uncertainty),
            warnings=tuple(warnings),
            created_utc_ns=time.time_ns(),
            observability_by_side=tuple(
                (
                    side,
                    _mask_observability(
                        observable if side in solved else PoseDofMask.from_names(())
                    ),
                )
                for side in ("left", "right")
            ),
        )
        self.register_workstation_optical_pose_candidate(candidate, prior_poses=priors)
        self._candidate_solver_context[candidate.candidate_id] = correspondence_context
        return self.workstation_optical_pose_snapshot()

    def _resolve_gnss_baseline_for_mountain(
        self,
        payload: Mapping[str, Any],
    ) -> tuple[tuple[float, float, float], dict[str, Any]]:
        """Resolve one auditable right-minus-left GNSS baseline in world NED.

        Priority is (1) an explicit payload override -- always tagged as an
        override, never silently mistaken for capture evidence -- then (2)
        the Left/Right GNSS snapshot captured with *this exact* frozen burst
        (``self._drone_rtk_observations``, matched by ``burst_id``). There is
        no fallback to ``panel.latest_gps``/``panel.latest_rtk_baseline``: a
        frozen Mountain image must never be paired with whatever GNSS
        happens to be live on the panel at solve time.
        """

        raw = payload.get("gnss_baseline_ned_m")
        source = "payload.gnss_baseline_ned_m (explicit override)"
        details: dict[str, Any] = {}
        if raw is None:
            raw = payload.get("baseline_ned_m")
            if raw is not None:
                source = "payload.baseline_ned_m (explicit override)"

        if raw is not None:
            if isinstance(raw, Mapping):
                vector = raw.get("ned_m", raw.get("vector_m"))
                if vector is None:
                    vector = (
                        raw.get("north_m"),
                        raw.get("east_m"),
                        raw.get("down_m"),
                    )
                details.update(dict(raw))
            else:
                vector = raw
            baseline_ned = _finite_vector(
                vector or (),
                length=3,
                name="gnss_baseline_ned_m",
            )
            details["capture_observation_id"] = None
            details.setdefault("fix_quality", payload.get("gnss_fix_quality", "UNKNOWN"))
            details.setdefault("left_rtk_label", payload.get("gnss_left_rtk_label", "UNKNOWN"))
            details.setdefault("right_rtk_label", payload.get("gnss_right_rtk_label", "UNKNOWN"))
            details.setdefault("left_age_s", payload.get("gnss_left_age_s"))
            details.setdefault("right_age_s", payload.get("gnss_right_age_s"))
        else:
            observation = self._capture_observation_for_frozen_burst()
            if observation is None:
                raise RuntimeError(
                    "frozen Mountain image has no capture-associated GNSS baseline evidence; "
                    "capture a burst with GNSS evidence, or supply an explicit "
                    "gnss_baseline_ned_m override"
                )
            gnss = observation.get("gnss")
            gnss = gnss if isinstance(gnss, Mapping) else {}
            left_row = gnss.get("left")
            right_row = gnss.get("right")
            left_state = _gps_state_from_capture_mapping(left_row)
            right_state = _gps_state_from_capture_mapping(right_row)
            if left_state is None or right_state is None:
                raise RuntimeError(
                    "frozen Mountain image has no capture-associated GNSS baseline evidence "
                    f"(capture observation {observation.get('observation_id')} is missing "
                    "left/right GNSS position)"
                )
            delta = ned_delta_m(left_state, right_state)
            if delta is None:
                raise RuntimeError(
                    "frozen Mountain image has no capture-associated GNSS baseline evidence "
                    "(left/right GNSS position incomplete)"
                )
            baseline_ned = delta
            source = f"capture_observation:{observation.get('observation_id')}"
            left_label = str((left_row or {}).get("rtk_label") or "UNKNOWN")
            right_label = str((right_row or {}).get("rtk_label") or "UNKNOWN")
            details = {
                "capture_observation_id": observation.get("observation_id"),
                "left_rtk_label": left_label,
                "right_rtk_label": right_label,
                "fix_quality": (
                    "RTK_FIXED"
                    if left_label == "RTK_FIXED" and right_label == "RTK_FIXED"
                    else "DEGRADED"
                ),
                "left_timestamp_s": (left_row or {}).get("timestamp_s"),
                "right_timestamp_s": (right_row or {}).get("timestamp_s"),
            }

        baseline_m = math.sqrt(sum(value * value for value in baseline_ned))
        if baseline_m <= 1.0e-9:
            raise ValueError("GNSS + Mountain requires a non-zero RTK baseline")
        fix_quality = str(details.get("fix_quality") or "UNKNOWN").upper()
        apply_eligible = (
            str(details.get("left_rtk_label") or "").upper() == "RTK_FIXED"
            and str(details.get("right_rtk_label") or "").upper() == "RTK_FIXED"
        )
        details.update(
            {
                "source": source,
                "ned_m": list(baseline_ned),
                "baseline_m": baseline_m,
                "fix_quality": fix_quality,
                "apply_eligible": apply_eligible,
                "coordinate_contract": "right GNSS antenna minus left GNSS antenna; world NED metres",
            }
        )
        return baseline_ned, details

    @staticmethod
    def _resolve_mountain_lever_arm(
        payload: Mapping[str, Any],
    ) -> tuple[tuple[float, float, float], str]:
        """Resolve an explicit optical-center correction, never an estimate.

        The correction is the vector to *add* to the antenna-to-antenna GNSS
        baseline (already expressed in Left camera RDF) to obtain the
        optical-center-to-optical-center baseline. This module never
        estimates it: without an explicit
        ``lever_arm_baseline_correction_left_rdf_m`` payload override the
        status is ``UNMODELED`` and the correction is the zero vector, which
        must be surfaced to the caller as an assumption, not a measurement.
        """

        raw = payload.get("lever_arm_baseline_correction_left_rdf_m")
        if raw is None:
            return (0.0, 0.0, 0.0), "UNMODELED"
        correction = _finite_vector(
            raw, length=3, name="lever_arm_baseline_correction_left_rdf_m"
        )
        return correction, "CALIBRATED"

    def solve_workstation_optical_pose_gnss_mountain(
        self,
        payload: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Build metric stereo R,t from GNSS and paired Mountain ``(u,v)`` clicks.

        No GeoTIFF, terrain height, or 3-D object correspondence is used.
        Same-order ridge clicks on the frozen left/right frames provide the
        normalized bearing pairs; the *direction* of the GNSS baseline
        conditions the relative rotation fit (its metric length never enters
        the rotation residual), while the GNSS baseline's metric length
        supplies the relative translation.  The prior absolute left pose
        remains the world-frame anchor: pixels plus a stereo baseline
        direction determine relative R,t, not an independent absolute world
        yaw, so this method never measures the Left absolute pose.
        """

        profile, _applicability = self._require_frozen_intrinsics()
        priors = self._resolve_prior_poses(payload, prefer_frozen=True)
        annotations = self._annotation_root(payload)
        points_by_side: dict[str, tuple[tuple[float, float], ...]] = {}
        input_hashes: list[tuple[str, str]] = [
            ("operating_intrinsics_profile_sha256", profile.profile_sha256)
        ]
        source_ids: list[str] = []
        for side in ("left", "right"):
            annotation = annotations.get(side)
            if not isinstance(annotation, Mapping):
                continue
            self._validate_annotation_identity(side, annotation)
            points = self._annotation_points(annotation, "ridge_polyline")
            if points:
                points_by_side[side] = points
                source_ids.append(str(annotation["frame_id"]))
                input_hashes.append(
                    (f"{side}_mountain_uv_sha256", _payload_sha256(annotation))
                )
        if set(points_by_side) != {"left", "right"}:
            raise ValueError(
                "GNSS + Mountain requires frozen left/right Mountain u,v clicks"
            )
        left_points = points_by_side["left"]
        right_points = points_by_side["right"]
        if len(left_points) != len(right_points):
            raise ValueError(
                "GNSS + Mountain requires the same number of left/right clicks; "
                "click corresponding mountain features in the same order"
            )
        if len(left_points) < 4:
            raise ValueError(
                "GNSS + Mountain requires at least four left/right u,v correspondence pairs"
            )
        self._validate_mountain_point_distribution(left_points, side="left")
        self._validate_mountain_point_distribution(right_points, side="right")

        baseline_ned, gnss_details = self._resolve_gnss_baseline_for_mountain(payload)
        baseline_left_rdf = _matvec(
            _mat_transpose(priors[0].rotation_ned_from_camera_rdf),
            baseline_ned,
        )
        lever_arm_correction, lever_arm_status = self._resolve_mountain_lever_arm(payload)
        optical_baseline_left_rdf = tuple(
            baseline_left_rdf[index] + lever_arm_correction[index] for index in range(3)
        )
        left_bearings = tuple(
            (*normalized, 1.0)
            for normalized in self._undistort_pixels(profile.left, left_points)
        )
        right_bearings = tuple(
            (*normalized, 1.0)
            for normalized in self._undistort_pixels(profile.right, right_points)
        )
        prior_stereo = stereo_pose_from_absolute(*priors)
        left = priors[0]
        left_anchor_sha256 = _payload_sha256(_pose_mapping(left))
        rotation_config = MountainRotationConfig()
        try:
            solve = solve_mountain_relative_rotation_robust(
                left_bearings_rdf=left_bearings,
                right_bearings_rdf=right_bearings,
                baseline_direction_left_rdf=optical_baseline_left_rdf,
                prior_rotation_right_from_left_rdf=prior_stereo.rotation_right_from_left_rdf,
                config=rotation_config,
            )
        except ValueError as exc:
            raise ValueError(f"GNSS + Mountain rotation solve failed: {exc}") from exc

        right_rotation_ned = _matmul(
            left.rotation_ned_from_camera_rdf,
            _mat_transpose(solve.rotation_right_from_left_rdf),
        )
        optical_baseline_ned = _matvec(
            left.rotation_ned_from_camera_rdf,
            optical_baseline_left_rdf,
        )
        right = CameraPoseNed(
            right_rotation_ned,
            tuple(left.center_ned_m[index] + optical_baseline_ned[index] for index in range(3)),
        )

        focal_scale = math.sqrt(
            float(profile.left.camera_matrix[0][0])
            * float(profile.left.camera_matrix[1][1])
            * float(profile.right.camera_matrix[0][0])
            * float(profile.right.camera_matrix[1][1])
        ) ** 0.25
        rotation_apply_eligible = bool(solve.apply_eligible)
        gnss_apply_eligible = bool(gnss_details.get("apply_eligible"))
        observable = PoseDofMask(
            roll_deg=rotation_apply_eligible,
            pitch_deg=rotation_apply_eligible,
            yaw_deg=rotation_apply_eligible,
            north_m=gnss_apply_eligible,
            east_m=gnss_apply_eligible,
            down_m=gnss_apply_eligible,
        )
        uv_hash = _payload_sha256(
            {
                "left": [
                    {"u_px": point[0], "v_px": point[1]}
                    for point in left_points
                ],
                "right": [
                    {"u_px": point[0], "v_px": point[1]}
                    for point in right_points
                ],
                "contract": "same-order paired frozen native OpenCV pixels; u right, v down",
            }
        )
        input_hashes.extend(
            (
                ("mountain_uv_pairs_sha256", uv_hash),
                ("gnss_baseline_sha256", _payload_sha256(gnss_details)),
                ("left_anchor_pose_sha256", left_anchor_sha256),
                ("lever_arm_status", lever_arm_status),
                ("pose_component_sources.left_rotation", "prior_anchor"),
                ("pose_component_sources.left_position", "prior_anchor"),
                ("pose_component_sources.relative_rotation", "mountain_correspondences"),
                ("pose_component_sources.relative_translation", "capture_gnss_baseline"),
                ("pose_component_sources.right_rotation", "derived"),
                ("pose_component_sources.right_position", "derived"),
                ("pose_component_sources.lever_arm", lever_arm_status),
            )
        )
        warnings: list[str] = [
            "GNSS is authoritative for right-minus-left metric translation in world NED",
            "Mountain uses only same-order left/right frozen u,v clicks; no GeoTIFF or terrain height",
            "relative R is conditioned on the selected Left absolute anchor; changing the anchor "
            "changes/invalidates this candidate even for identical pixels and GNSS baseline",
            "Left absolute pose is not measured by Mountain; it remains the prior/anchor",
            f"GNSS fix quality={gnss_details['fix_quality']}",
            (
                "lever arm CALIBRATED: an explicit optical-center correction was applied"
                if lever_arm_status == "CALIBRATED"
                else "lever arm UNMODELED: GNSS antenna baseline is used as an optical-baseline "
                "proxy with zero correction; this is an assumption, not a measurement"
            ),
        ]
        if not gnss_apply_eligible:
            warnings.append(
                "GNSS + Mountain apply eligibility requires RTK_FIXED on both Left and Right; "
                f"left={gnss_details.get('left_rtk_label')} right={gnss_details.get('right_rtk_label')}"
            )
        if not rotation_apply_eligible:
            warnings.append(
                "Mountain rotation is not apply-eligible: " + "; ".join(solve.ineligibility_reasons)
            )
        uncertainty: list[tuple[str, float]] = []
        if math.isfinite(solve.jacobian_condition):
            uncertainty.append(("rotation_jacobian_condition", float(solve.jacobian_condition)))
        quality: list[tuple[str, float]] = [
            ("mountain_uv_pair_count", float(len(left_points))),
            ("inlier_count", float(len(solve.inlier_indices))),
            ("outlier_count", float(len(solve.outlier_indices))),
            (
                "inlier_ratio",
                float(len(solve.inlier_indices)) / float(solve.total_count) if solve.total_count else 0.0,
            ),
            ("gnss_baseline_m", float(gnss_details["baseline_m"])),
        ]
        if math.isfinite(solve.sampson_rms_normalized) and math.isfinite(solve.sampson_p95_normalized):
            quality.extend(
                (
                    ("sampson_rms_normalized", float(solve.sampson_rms_normalized)),
                    ("sampson_p95_normalized", float(solve.sampson_p95_normalized)),
                    ("sampson_rms_px_equivalent", float(solve.sampson_rms_normalized) * focal_scale),
                    ("sampson_p95_px_equivalent", float(solve.sampson_p95_normalized) * focal_scale),
                )
            )
        else:
            warnings.append("Sampson residual metrics are unavailable (non-finite; too few inliers)")
        candidate = OpticalPoseCandidate(
            candidate_id=f"gnss-mountain-uv-{uuid.uuid4().hex}",
            method="gnss-constrained-mountain-uv",
            left_pose_ned=left,
            right_pose_ned=right,
            optimized_dofs=_ALL_DOF,
            observability=_mask_observability(observable),
            source_observation_ids=tuple(
                dict.fromkeys((*source_ids, f"gnss-baseline:{_payload_sha256(gnss_details)}"))
            ),
            input_hashes=tuple(input_hashes),
            quality_metrics=tuple(quality),
            uncertainty_metrics=tuple(uncertainty),
            warnings=tuple(warnings),
            created_utc_ns=time.time_ns(),
            observability_by_side=tuple(
                (
                    side,
                    _mask_observability(observable),
                )
                for side in ("left", "right")
            ),
        )
        self.register_workstation_optical_pose_candidate(candidate, prior_poses=priors)
        return self.workstation_optical_pose_snapshot()

    @staticmethod
    def _coerce_gps_state(value: Any, *, side: str, now_s: float) -> GpsPositionState | None:
        """Accept live ``GpsPositionState`` or a serialized telemetry row."""

        if isinstance(value, GpsPositionState):
            return value
        if not isinstance(value, Mapping):
            return None
        latitude = value.get("latitude_deg", value.get("lat"))
        longitude = value.get("longitude_deg", value.get("lon"))
        altitude = value.get("altitude_m", value.get("alt"))
        if latitude is None or longitude is None or altitude is None:
            return None
        timestamp = value.get("timestamp_s")
        if timestamp is None:
            age = value.get("age_s")
            timestamp = float(now_s) - max(0.0, float(age or 0.0))
        try:
            fix_value = value.get("fix_type")
            fix_type = None if fix_value in (None, "") else int(fix_value)
            return GpsPositionState(
                camera=side,
                namespace=str(value.get("namespace") or "telemetry"),
                timestamp_s=float(timestamp),
                topic=str(value.get("topic") or "telemetry/gps"),
                latitude_deg=float(latitude),
                longitude_deg=float(longitude),
                altitude_m=float(altitude),
                fix_type=fix_type,
                satellites_used=(
                    None
                    if value.get("satellites_used", value.get("satellites")) in (None, "")
                    else int(value.get("satellites_used", value.get("satellites")))
                ),
            )
        except (TypeError, ValueError):
            return None

    def _resolve_telemetry_geometry(
        self,
        payload: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Resolve fresh P1/P2/Drone GNSS into one local world-NED frame."""

        now_s = float(payload.get("now_s", time.monotonic()))
        raw_states = payload.get("telemetry_gps")
        if not isinstance(raw_states, Mapping):
            snapshot = payload.get("telemetry")
            if isinstance(snapshot, Mapping):
                raw_states = {
                    "left": snapshot.get("left_position"),
                    "right": snapshot.get("right_position"),
                    "drone": snapshot.get("position"),
                }
        if not isinstance(raw_states, Mapping):
            raw_states = getattr(self.panel, "latest_gps", {}) or {}
        if not isinstance(raw_states, Mapping):
            raw_states = {}

        states = {
            side: self._coerce_gps_state(raw_states.get(side), side=side, now_s=now_s)
            for side in ("left", "right", "drone")
        }
        missing = [side for side, state in states.items() if state is None or not state.has_position]
        if missing:
            raise RuntimeError(
                "Telemetry geometry requires valid left/right/Drone GNSS positions; "
                "missing " + ", ".join(missing)
            )
        age_limit = max(0.0, float(payload.get("maximum_gps_age_s", 2.0)))
        ages = {
            side: max(0.0, now_s - float(states[side].timestamp_s))  # type: ignore[union-attr]
            for side in states
        }
        stale = [side for side, age in ages.items() if age > age_limit]
        if stale:
            detail = ", ".join(f"{side}={ages[side]:.2f}s" for side in stale)
            raise RuntimeError(
                f"Telemetry geometry requires GNSS newer than {age_limit:.2f}s; stale {detail}"
            )

        left = states["left"]
        right = states["right"]
        drone = states["drone"]
        baseline = baseline_from_gps(left, right)  # type: ignore[arg-type]
        baseline_ned = ned_delta_m(left, right)  # type: ignore[arg-type]
        target_distance = target_distance_from_gps(left, right, drone)  # type: ignore[arg-type]
        if baseline is None or baseline_ned is None or target_distance is None:
            raise RuntimeError("Telemetry geometry could not derive a non-zero GNSS geometry")
        if baseline.length_m <= 1.0e-9:
            raise RuntimeError("Telemetry geometry requires a non-zero P1/P2 baseline")

        return {
            "now_s": now_s,
            "states": states,
            "ages": ages,
            "baseline": baseline,
            "baseline_ned": tuple(float(value) for value in baseline_ned),
            "target_distance_m": float(target_distance),
            "coordinate_contract": "origin=left GNSS; world NED metres; right=right-left",
        }

    def solve_workstation_optical_pose_drone_rtk_multiview(
        self,
        payload: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Solve full per-camera R,t from several Drone RTK target observations.

        The imported set contains paired ``observed_ray_camera_rdf`` (or
        native ``pixel_uv``) and ``target_point_ned_m`` rows for each side.
        The target point is the Drone RTK position in the same local-NED frame
        as the camera prior.  Camera centers and orientation are both solver
        variables; current GNSS is used only as a better initial center when it
        is fresh and available.
        """

        profile, _applicability = self._require_frozen_intrinsics()
        priors = self._resolve_prior_poses(payload, prefer_frozen=True)
        root: Mapping[str, Any] | None = None
        source_file_sha256 = ""
        raw_root = payload.get("drone_rtk_multiview")
        if isinstance(raw_root, Mapping):
            root = raw_root
        elif isinstance(payload.get("multiview"), Mapping):
            root = payload.get("multiview")  # type: ignore[assignment]
        elif payload.get("drone_rtk_multiview_path"):
            loaded, source_file_sha256 = _load_json_mapping(
                payload.get("drone_rtk_multiview_path"),
                label="Drone RTK multi-view observation JSON",
            )
            loaded_root = loaded.get("drone_rtk_multiview", loaded)
            if isinstance(loaded_root, Mapping):
                root = loaded_root
        if root is None:
            raise ValueError(
                "Drone RTK Multi-view requires an observation JSON or a drone_rtk_multiview payload"
            )
        observation_root = root.get("observations", root)
        if not isinstance(observation_root, Mapping):
            raise ValueError("Drone RTK multi-view observations must contain left/right arrays")

        live_geometry: dict[str, Any] | None = None
        try:
            live_geometry = self._resolve_telemetry_geometry(payload)
        except (RuntimeError, ValueError):
            # An offline/imported set may already contain an explicit local-NED
            # origin and camera centers.  Do not make the live link mandatory.
            live_geometry = None
        initial_centers = {
            "left": tuple(priors[0].center_ned_m),
            "right": tuple(priors[1].center_ned_m),
        }
        if live_geometry is not None:
            initial_centers = {
                "left": (0.0, 0.0, 0.0),
                "right": tuple(live_geometry["baseline_ned"]),
            }
        center_root = root.get("camera_centers_ned_m")
        if isinstance(center_root, Mapping):
            for side in ("left", "right"):
                if side in center_root:
                    initial_centers[side] = _vector_value(
                        center_root[side],
                        name=f"camera_centers_ned_m.{side}",
                    )

        origin_state: GpsPositionState | None = None
        if live_geometry is not None:
            origin_state = live_geometry["states"]["left"]
        elif root.get("origin_gps") is not None:
            origin_state = self._coerce_gps_state(
                root.get("origin_gps"),
                side="origin",
                now_s=time.monotonic(),
            )

        solved: dict[str, tuple[CameraPoseNed, Any, tuple[dict[str, Any], ...]]] = {}
        source_ids: list[str] = []
        input_hashes: list[tuple[str, str]] = [
            ("operating_intrinsics_profile_sha256", profile.profile_sha256),
        ]
        if source_file_sha256:
            input_hashes.append(("drone_rtk_multiview_json_sha256", source_file_sha256))
        warning_set: list[str] = [
            "Drone RTK target positions and camera rays are solved jointly for full R,t",
            "camera pose coordinate contract: X_NED = R_NED<-optical_RDF X_RDF + C_NED",
        ]
        for side, prior, camera in (
            ("left", priors[0], profile.left),
            ("right", priors[1], profile.right),
        ):
            raw_rows = observation_root.get(side)
            if not isinstance(raw_rows, (tuple, list)):
                raise ValueError(f"Drone RTK multi-view requires a {side} observation array")
            if len(raw_rows) < 4:
                raise ValueError(f"{side}: Drone RTK multi-view requires at least four observations")
            rays: list[tuple[float, float, float]] = []
            targets: list[tuple[float, float, float]] = []
            rows_for_hash: list[dict[str, Any]] = []
            side_source_ids: list[str] = []
            for index, raw_row in enumerate(raw_rows):
                if not isinstance(raw_row, Mapping):
                    raise ValueError(f"{side}[{index}] must be an object")
                row = dict(raw_row)
                ray_value = row.get("observed_ray_camera_rdf")
                if ray_value is None:
                    pixel = row.get("pixel_uv", row.get("image_point_px", row.get("target_pixel")))
                    if isinstance(pixel, Mapping):
                        pixel = (pixel.get("u_px", pixel.get("u")), pixel.get("v_px", pixel.get("v")))
                    pixel_uv = _finite_vector(pixel or (), length=2, name=f"{side}[{index}].pixel_uv")
                    normalized = self._undistort_pixels(camera, (tuple(pixel_uv),))[0]
                    ray_value = (normalized[0], normalized[1], 1.0)
                ray = _finite_vector(
                    ray_value,
                    length=3,
                    name=f"{side}[{index}].observed_ray_camera_rdf",
                )
                target_value = row.get("target_point_ned_m", row.get("drone_position_ned_m"))
                if target_value is None:
                    target_gps = row.get("drone_gps", row.get("target_gps"))
                    if target_gps is None or origin_state is None:
                        raise ValueError(
                            f"{side}[{index}] requires target_point_ned_m or Drone RTK GPS plus origin_gps"
                        )
                    target_state = self._coerce_gps_state(
                        target_gps,
                        side="drone",
                        now_s=time.monotonic(),
                    )
                    target_value = None if target_state is None else ned_delta_m(origin_state, target_state)
                target = _vector_value(target_value, name=f"{side}[{index}].target_point_ned_m")
                rays.append(ray)
                targets.append(target)
                observation_id = str(row.get("observation_id") or row.get("frame_id") or f"{side}-{index:04d}")
                side_source_ids.append(f"drone-rtk:{side}:{observation_id}")
                if row.get("timestamp_alignment_verified") is not True:
                    warning_set.append(
                        f"{side}[{index}]: image/Drone RTK timestamp alignment is not verified"
                    )
                rows_for_hash.append(row)
            before_pose = CameraPoseNed(
                prior.rotation_ned_from_camera_rdf,
                initial_centers[side],
            )
            before = pose_parameters_from_camera_pose(before_pose)

            def _residuals(
                parameters: PoseParametersNed,
                *,
                _rays=tuple(rays),
                _targets=tuple(targets),
            ) -> tuple[float, ...]:
                return pose_target_ray_residuals(
                    parameters,
                    observed_rays_camera_rdf=_rays,
                    target_points_ned_m=_targets,
                )

            result = solve_constrained_pose_least_squares(
                before,
                _ALL_DOF,
                _residuals,
                parameter_scales=(1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
            )
            if not result.success:
                raise ValueError(
                    f"{side}: Drone RTK multi-view R,t solve did not converge ({result.message})"
                )
            if not result.jacobian_diagnostics.full_rank:
                raise ValueError(
                    f"{side}: Drone RTK multi-view observations are geometrically degenerate; "
                    "use targets with varied 3-D positions"
                )
            solved[side] = (result.parameters.to_camera_pose_ned(), result, tuple(rows_for_hash))
            source_ids.extend(side_source_ids)
            input_hashes.append(
                (f"{side}_drone_rtk_multiview_sha256", _payload_sha256({"observations": rows_for_hash}))
            )

        left = solved["left"][0]
        right = solved["right"][0]
        quality: list[tuple[str, float]] = []
        uncertainty: list[tuple[str, float]] = []
        for side, (_pose, result, rows) in solved.items():
            residuals = tuple(float(value) for value in result.residuals)
            rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))
            quality.extend(
                (
                    (f"{side}_observation_count", float(len(rows))),
                    (f"{side}_angular_rms_chord", rms),
                )
            )
            condition = float(result.jacobian_diagnostics.condition_number)
            if math.isfinite(condition):
                uncertainty.append((f"{side}_jacobian_condition", condition))
        input_hashes.append(
            (
                "multiview_coordinate_contract_sha256",
                _payload_sha256(
                    {
                        "camera_origin": "local NED origin from imported origin_gps/current left GNSS or prior",
                        "target": "Drone RTK position in same local NED frame",
                        "ray": "camera optical RDF",
                    }
                ),
            )
        )
        candidate = OpticalPoseCandidate(
            candidate_id=f"drone-rtk-multiview-{uuid.uuid4().hex}",
            method="drone-rtk-multiview-r-t",
            left_pose_ned=left,
            right_pose_ned=right,
            optimized_dofs=_ALL_DOF,
            observability=_mask_observability(_ALL_DOF),
            source_observation_ids=tuple(source_ids),
            input_hashes=tuple(input_hashes),
            quality_metrics=tuple(quality),
            uncertainty_metrics=tuple(uncertainty),
            warnings=tuple(dict.fromkeys(warning_set)),
            created_utc_ns=time.time_ns(),
        )
        self.register_workstation_optical_pose_candidate(candidate, prior_poses=priors)
        return self.workstation_optical_pose_snapshot()

    def solve_workstation_optical_pose_drone_yaw(
        self,
        payload: Mapping[str, Any],
    ) -> dict[str, Any]:
        profile, _applicability = self._require_frozen_intrinsics()
        priors = self._resolve_prior_poses(payload, prefer_frozen=True)
        annotations = self._annotation_root(payload)
        root = payload.get("drone_yaw")
        root = root if isinstance(root, Mapping) else payload.get("observations")
        root = root if isinstance(root, Mapping) else {}
        drone_file_sha256 = ""
        if not root and payload.get("drone_yaw_path"):
            loaded, drone_file_sha256 = _load_json_mapping(
                payload.get("drone_yaw_path"),
                label="Drone RTK yaw observation JSON",
            )
            root = loaded.get("drone_yaw", loaded)
            if not isinstance(root, Mapping):
                raise ValueError(
                    "Drone RTK yaw JSON must contain a left/right observation object map"
                )
        solved: dict[str, tuple[CameraPoseNed, PoseDofMask, tuple[tuple[str, float], ...]]] = {}
        observation_counts: dict[str, int] = {}
        warnings: list[str] = []
        rotation_context: dict[str, dict[str, Any]] = {}
        hashes: list[tuple[str, str]] = [
            ("operating_intrinsics_profile_sha256", profile.profile_sha256)
        ]
        if drone_file_sha256:
            hashes.append(("drone_yaw_json_sha256", drone_file_sha256))
        for side, prior, camera in (
            ("left", priors[0], profile.left),
            ("right", priors[1], profile.right),
        ):
            raw_row = root.get(side)
            if isinstance(raw_row, Mapping):
                observation_rows: tuple[Mapping[str, Any], ...] = (raw_row,)
            elif isinstance(raw_row, (list, tuple)):
                observation_rows = tuple(item for item in raw_row if isinstance(item, Mapping))
            else:
                observation_rows = ()
            if not observation_rows:
                continue
            observation_counts[side] = len(observation_rows)
            annotation = annotations.get(side)
            before = pose_parameters_from_camera_pose(prior)

            if len(observation_rows) == 1:
                # A single RTK target/ray pair only constrains yaw (roll/pitch
                # held at the prior); use the existing closed-form solve.
                row = observation_rows[0]
                ray_value = row.get("observed_ray_camera_rdf")
                if ray_value is None:
                    if not isinstance(annotation, Mapping):
                        raise ValueError(
                            f"{side}: provide observed_ray_camera_rdf or a frozen target_point annotation"
                        )
                    self._validate_annotation_identity(side, annotation)
                    points = self._annotation_points(annotation, "target_point")
                    if len(points) != 1:
                        raise ValueError(f"{side}: exactly one frozen target pixel is required")
                    normalized = self._undistort_pixels(camera, points)[0]
                    ray = (normalized[0], normalized[1], 1.0)
                    hashes.append((f"{side}_annotation_sha256", _payload_sha256(annotation)))
                else:
                    ray = _finite_vector(ray_value, length=3, name=f"{side}.observed_ray_camera_rdf")
                los_value = row.get("expected_los_ned")
                if los_value is None:
                    camera_center = _finite_vector(
                        row.get("camera_center_ned_m", ()),
                        length=3,
                        name=f"{side}.camera_center_ned_m",
                    )
                    target_point = _finite_vector(
                        row.get("target_point_ned_m", ()),
                        length=3,
                        name=f"{side}.target_point_ned_m",
                    )
                    los = _subtract(target_point, camera_center)
                else:
                    los = _finite_vector(los_value, length=3, name=f"{side}.expected_los_ned")
                estimate = estimate_drone_yaw_deg(
                    ray,
                    los,
                    roll_deg=before.roll_deg,
                    pitch_deg=before.pitch_deg,
                )
                if not estimate.observable or estimate.yaw_deg is None:
                    raise ValueError(f"{side}: Drone RTK yaw is unobservable ({estimate.reason})")
                updated = PoseParametersNed(
                    before.roll_deg,
                    before.pitch_deg,
                    estimate.yaw_deg,
                    before.north_m,
                    before.east_m,
                    before.down_m,
                ).to_camera_pose_ned()
                solved[side] = (
                    updated,
                    _YAW_DOF,
                    ((f"{side}_angular_residual_rad", float(estimate.angular_residual_rad)),),
                )
                rotation_context[side] = {
                    "mode": "yaw_only",
                    "observed_ray_camera_rdf": tuple(float(value) for value in ray),
                    "expected_los_ned": tuple(float(value) for value in los),
                    "timestamp_alignment_verified": row.get("timestamp_alignment_verified") is True,
                }
                hashes.append((f"{side}_drone_input_sha256", _payload_sha256(row)))
                if row.get("timestamp_alignment_verified") is not True:
                    warnings.append(
                        f"{side}: camera pixel and target/camera RTK timestamp alignment is not verified"
                    )
                continue

            # Two or more RTK target/ray pairs over-determine the full
            # rotation (roll, pitch, yaw) with the optical center still fixed
            # at the prior; position is never inferred from this solve.
            rays: list[tuple[float, float, float]] = []
            targets: list[tuple[float, float, float]] = []
            for index, row in enumerate(observation_rows):
                ray_value = row.get("observed_ray_camera_rdf")
                if ray_value is None:
                    raise ValueError(
                        f"{side}[{index}]: multi-observation Drone yaw requires "
                        "observed_ray_camera_rdf explicitly per observation; frozen "
                        "annotations only support a single observation"
                    )
                rays.append(
                    _finite_vector(ray_value, length=3, name=f"{side}[{index}].observed_ray_camera_rdf")
                )
                los_value = row.get("expected_los_ned")
                if los_value is None:
                    target_point = _finite_vector(
                        row.get("target_point_ned_m", ()),
                        length=3,
                        name=f"{side}[{index}].target_point_ned_m",
                    )
                else:
                    los = _finite_vector(los_value, length=3, name=f"{side}[{index}].expected_los_ned")
                    target_point = tuple(prior.center_ned_m[axis] + los[axis] for axis in range(3))
                targets.append(target_point)
                if row.get("timestamp_alignment_verified") is not True:
                    warnings.append(
                        f"{side}[{index}]: camera pixel and target/camera RTK timestamp alignment "
                        "is not verified"
                    )
            hashes.append(
                (
                    f"{side}_drone_multi_input_sha256",
                    _payload_sha256({"observations": [dict(row) for row in observation_rows]}),
                )
            )

            def _residuals(
                parameters: PoseParametersNed,
                *,
                _center=prior.center_ned_m,
                _rays=tuple(rays),
                _targets=tuple(targets),
            ) -> tuple[float, ...]:
                return fixed_center_orientation_ray_residuals(
                    (parameters.roll_deg, parameters.pitch_deg, parameters.yaw_deg),
                    camera_center_ned_m=_center,
                    observed_rays_camera_rdf=_rays,
                    target_points_ned_m=_targets,
                )

            result = solve_constrained_pose_least_squares(before, _DRONE_ROTATION_DOF, _residuals)
            if not result.success:
                raise ValueError(
                    f"{side}: Drone RTK multi-observation rotation solve did not converge "
                    f"({result.message})"
                )
            updated = result.parameters.to_camera_pose_ned()
            rms_chord_residual = math.sqrt(
                sum(value * value for value in result.residuals) / len(result.residuals)
            )
            solved[side] = (
                updated,
                _DRONE_ROTATION_DOF,
                (
                    (f"{side}_rotation_rms_chord_residual", rms_chord_residual),
                    (f"{side}_observation_count", float(len(observation_rows))),
                ),
            )
            rotation_context[side] = {
                "mode": "full_rotation",
                "observation_count": len(observation_rows),
            }
        if not solved:
            raise ValueError(
                "Drone yaw requires explicit per-side target/camera LOS and ray or frozen target pixel"
            )
        left = solved.get("left", (priors[0],))[0]
        right = solved.get("right", (priors[1],))[0]
        quality = tuple(item for (_pose, _mask, metrics) in solved.values() for item in metrics)
        combined_mask = (
            _DRONE_ROTATION_DOF
            if any(mask is _DRONE_ROTATION_DOF for _pose, mask, _metrics in solved.values())
            else _YAW_DOF
        )
        candidate = OpticalPoseCandidate(
            candidate_id=f"drone-yaw-{uuid.uuid4().hex}",
            method="drone-rtk-target-yaw",
            left_pose_ned=left,
            right_pose_ned=right,
            optimized_dofs=combined_mask,
            observability=_mask_observability(combined_mask),
            source_observation_ids=tuple(
                f"{side}:explicit-drone-rtk-los×{observation_counts[side]}" for side in solved
            ),
            input_hashes=tuple(hashes),
            quality_metrics=quality,
            warnings=tuple(warnings),
            created_utc_ns=time.time_ns(),
            observability_by_side=tuple(
                (
                    side,
                    _mask_observability(
                        solved[side][1] if side in solved else PoseDofMask.from_names(())
                    ),
                )
                for side in ("left", "right")
            ),
        )
        self.register_workstation_optical_pose_candidate(candidate, prior_poses=priors)
        self._candidate_drone_rotation_context[candidate.candidate_id] = rotation_context
        return self.workstation_optical_pose_snapshot()

    def register_workstation_optical_pose_reference_candidate(
        self,
        payload: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Wrap already-known Checkerboard/Pixhawk+RTK evidence as a candidate.

        This never re-derives geometry: it packages an existing extrinsics
        source into the same ``OpticalPoseCandidate`` shape a PnP solve
        produces, anchored to the current prior's left pose, so
        ``solve_workstation_optical_pose_combined`` can mix specific axes
        from it alongside Horizon/Mountain/Drone-yaw solves (e.g. take only
        Checkerboard's translation while Horizon supplies roll/pitch).
        """

        source = str(payload.get("source") or "")
        priors = self._resolve_prior_poses(payload, prefer_frozen=True)
        left = priors[0]

        if source == "checkerboard":
            profile = getattr(self.panel, "checkerboard_extrinsics_profile", None)
            if profile is None:
                raise ValueError("no checkerboard stereoCalibrate profile is available")
            rotation_right_from_left = [
                [float(value) for value in row] for row in profile.extrinsics.rotation_left_to_right
            ]
            translation_left_to_right_m = _finite_vector(
                profile.extrinsics.translation_left_to_right_m,
                length=3,
                name="checkerboard.translation_left_to_right_m",
            )
            baseline_ned = _matvec(left.rotation_ned_from_camera_rdf, translation_left_to_right_m)
            right_rotation_ned = _matmul(
                left.rotation_ned_from_camera_rdf,
                _mat_transpose(rotation_right_from_left),
            )
            right = CameraPoseNed(
                right_rotation_ned,
                tuple(left.center_ned_m[index] + baseline_ned[index] for index in range(3)),
            )
            mask = _ALL_DOF
            method = "reference/checkerboard"
            hashes: tuple[tuple[str, str], ...] = (
                ("checkerboard_extrinsics_profile_sha256", profile.content_sha256),
            )
            warnings: tuple[str, ...] = (
                "packaged from the stored stereoCalibrate profile, not re-measured here",
            )
        elif source == "pixhawk_rtk":
            baseline = getattr(self.panel, "latest_rtk_baseline", None)
            if baseline is None:
                raise ValueError("no live RTK GNSS baseline is available")
            usable, reason = self.panel._gps_baseline_usable_for_translation(baseline)
            if not usable:
                raise ValueError(f"live RTK GNSS baseline is not usable: {reason}")
            right = CameraPoseNed(
                left.rotation_ned_from_camera_rdf,
                (
                    left.center_ned_m[0] + baseline.north_m,
                    left.center_ned_m[1] + baseline.east_m,
                    left.center_ned_m[2] + baseline.down_m,
                ),
            )
            # Rotation is intentionally not packaged from Pixhawk here: the
            # attitude quaternion is body-FRD, not camera-RDF, and mapping it
            # through the mount rotation is a separate piece of validated
            # geometry this candidate does not attempt. Only the live RTK
            # translation is exposed as a mixable axis.
            mask = _TRANSLATION_DOF
            method = "reference/pixhawk-rtk-translation"
            hashes = (
                ("rtk_baseline_fix_quality", baseline.fix_quality),
            )
            warnings = (
                "translation only; live RTK baseline is not anchored to a re-derived rotation",
            )
        else:
            raise ValueError(f"unknown reference candidate source: {source!r}")

        candidate = OpticalPoseCandidate(
            candidate_id=f"reference-{source}-{uuid.uuid4().hex}",
            method=method,
            left_pose_ned=left,
            right_pose_ned=right,
            optimized_dofs=mask,
            observability=_mask_observability(mask),
            source_observation_ids=(f"reference:{source}",),
            input_hashes=hashes,
            quality_metrics=(),
            warnings=warnings,
            created_utc_ns=time.time_ns(),
        )
        self.register_workstation_optical_pose_candidate(candidate, prior_poses=priors)
        return self.workstation_optical_pose_snapshot()

    def solve_workstation_optical_pose_combined(
        self,
        payload: Mapping[str, Any],
    ) -> dict[str, Any]:
        raw_ids = payload.get("candidate_ids")
        candidate_ids = [str(value) for value in raw_ids] if isinstance(raw_ids, (tuple, list)) else []
        if len(candidate_ids) < 2:
            raise ValueError(
                "combined pose requires at least two explicitly selected compatible candidate IDs"
            )
        candidates = []
        for candidate_id in candidate_ids:
            candidate = self._candidates.get(candidate_id)
            if candidate is None:
                raise ValueError(f"unknown combined source candidate: {candidate_id}")
            candidates.append(candidate)
        prior = self._candidate_priors[candidate_ids[0]]
        if any(self._candidate_priors[candidate_id] != prior for candidate_id in candidate_ids[1:]):
            raise ValueError("combined candidates must share the same explicit prior poses")
        raw_candidate_dofs = payload.get("candidate_dofs")
        candidate_dofs = raw_candidate_dofs if isinstance(raw_candidate_dofs, Mapping) else {}
        used: set[str] = set()
        left_parameters = pose_parameters_from_camera_pose(prior[0])
        right_parameters = pose_parameters_from_camera_pose(prior[1])
        for candidate in candidates:
            # By default a source contributes every axis it optimized. An
            # operator may instead cherry-pick a subset per candidate_id (e.g.
            # take only translation from a full-pose Checkerboard/reference
            # candidate so it can be mixed with a rotation-only Horizon
            # candidate without an overlap error).
            override = candidate_dofs.get(candidate.candidate_id)
            if override is None:
                selected_names = candidate.optimized_dofs.selected_names
            else:
                selected_names = tuple(str(name) for name in override)
                unknown = set(selected_names) - set(candidate.optimized_dofs.selected_names)
                if unknown:
                    raise ValueError(
                        f"candidate_dofs[{candidate.candidate_id}] selects DOF this candidate "
                        "did not optimize: " + ", ".join(sorted(unknown))
                    )
            side_observable = {
                side: dict(values) for side, values in candidate.observability_by_side
            }
            unobservable = tuple(
                name
                for name in selected_names
                if not all(
                    side_observable[side].get(name, False)
                    for side in ("left", "right")
                )
            )
            if unobservable:
                raise ValueError(
                    f"candidate {candidate.candidate_id} axes are not observable on both sides: "
                    + ", ".join(unobservable)
                )
            overlap = used.intersection(selected_names)
            if overlap:
                raise ValueError(
                    "combined candidates optimize overlapping DOF: " + ", ".join(sorted(overlap))
                )
            used.update(selected_names)
            candidate_mask = PoseDofMask.from_names(selected_names)
            left_parameters = apply_selected_dofs(
                left_parameters,
                pose_parameters_from_camera_pose(candidate.left_pose_ned),
                candidate_mask,
            )
            right_parameters = apply_selected_dofs(
                right_parameters,
                pose_parameters_from_camera_pose(candidate.right_pose_ned),
                candidate_mask,
            )
        mask = PoseDofMask.from_names(used)
        profile_hashes = {
            dict(candidate.input_hashes).get("operating_intrinsics_profile_sha256", "")
            for candidate in candidates
        }
        profile_hashes.discard("")
        if len(profile_hashes) > 1:
            raise ValueError("combined candidates were solved with different operating K,D profiles")
        quality: list[tuple[str, float]] = []
        for index, candidate in enumerate(candidates):
            quality.extend(
                (f"source_{index}_{key}", value) for key, value in candidate.quality_metrics
            )
        hashes = [(f"source_candidate_{index}", _payload_sha256(_candidate_mapping(candidate))) for index, candidate in enumerate(candidates)]
        if profile_hashes:
            hashes.append(("operating_intrinsics_profile_sha256", next(iter(profile_hashes))))
        combined = OpticalPoseCandidate(
            candidate_id=f"combined-{uuid.uuid4().hex}",
            method="combined-independent-axis-evidence",
            left_pose_ned=left_parameters.to_camera_pose_ned(),
            right_pose_ned=right_parameters.to_camera_pose_ned(),
            optimized_dofs=mask,
            observability=_mask_observability(mask),
            source_observation_ids=tuple(candidate_ids),
            input_hashes=tuple(hashes),
            quality_metrics=tuple(quality),
            warnings=("independent-axis composition; source candidates and shared prior retained",),
            created_utc_ns=time.time_ns(),
        )
        self.register_workstation_optical_pose_candidate(combined, prior_poses=prior)
        return self.workstation_optical_pose_snapshot()

    def workstation_optical_pose_snapshot(
        self,
        *,
        include_frames: bool = True,
    ) -> dict[str, Any]:
        actual_zoom = getattr(self.panel, "actual_visible_zoom", {}) or {}
        active_profile_id = "" if self._active_selection is None else self._active_selection.selection_id
        active_candidate_id = "" if self._active_selection is None else self._active_selection.candidate_id
        selected_extrinsics = getattr(self.panel, "selected_extrinsics_profile", None)
        intrinsics = self._intrinsics_snapshot(use_frozen=False)
        geometry_allowed = bool(intrinsics["applicability"]["geometry_allowed"])
        current_profile_hash = str(intrinsics["applicability"]["profile_sha256"])
        candidate_rows: list[dict[str, Any]] = []
        for candidate in self._candidates.values():
            row = self._candidate_mapping_with_prior(
                candidate,
                self._candidate_priors[candidate.candidate_id],
            )
            expected = dict(candidate.input_hashes).get(
                "operating_intrinsics_profile_sha256",
                "",
            )
            row["applicable"] = geometry_allowed and (
                not expected or expected == current_profile_hash
            )
            if not row["applicable"]:
                row["state"] = "BLOCKED"
            candidate_rows.append(row)
        frozen_previews: dict[str, str] = {}
        frozen_native_frames: dict[str, dict[str, Any]] = {}
        frozen_frame_artifacts: dict[str, dict[str, Any]] = {}
        for side, row in self._frozen_frames.items():
            artifact: NativeFrameArtifact = row["artifact"]
            frozen_frame_artifacts[side] = artifact.to_mapping()
            frozen_previews[side] = (
                f"{artifact.frame_id} · {artifact.width}×{artifact.height} · "
                f"sha256:{artifact.sha256[:12]}"
            )
            if include_frames:
                frozen_native_frames[side] = {
                    "image": row["image"].copy(),
                    "frame_id": artifact.frame_id,
                    "image_sha256": artifact.sha256,
                    "metadata": dict(row["metadata"]),
                    "relative_path": artifact.relative_path,
                }
        terrain_state = "NOT SET"
        if self._terrain_snapshot is not None:
            terrain_state = (
                "READY" if self._terrain_metadata and self._terrain_metadata.full_pnp_height_ready
                else "BLOCKED · vertical datum unknown"
            )
        mountain_uv_state = "REQUIRES paired frozen left/right u,v clicks"
        if any(
            str(row.get("method") or "") == "gnss-constrained-mountain-uv"
            for row in candidate_rows
        ):
            mountain_uv_state = "USED · paired frozen left/right u,v clicks"
        active_candidate = next(
            (
                row
                for row in candidate_rows
                if str(row.get("candidate_id") or "") == active_candidate_id
            ),
            None,
        )
        active_selected_dof = (
            {
                name.removesuffix("_deg").removesuffix("_m"): bool(
                    getattr(self._active_selection.selected_dofs, name)
                )
                for name in POSE_DOF_NAMES
            }
            if self._active_selection is not None
            else {}
        )
        return {
            "state": "ACTIVE" if active_profile_id else "CANDIDATE" if self._candidates else "NO DATA",
            "active_profile_id": active_profile_id,
            "active_candidate_id": active_candidate_id,
            "active_selection_id": active_profile_id,
            "active_selected_dof": active_selected_dof,
            "active_candidate": active_candidate,
            "mountain_ridge_margin_fraction": self._mountain_ridge_margin_fraction,
            "mountain_max_points": self._mountain_max_points,
            "readiness": {
                "state": "ACTIVE" if active_profile_id else "CANDIDATE" if self._candidates else "NO DATA",
                "operating_zoom": f"{self._zoom(actual_zoom, 'left')} / {self._zoom(actual_zoom, 'right')}",
                "intrinsics": (
                    "READY · exact operating K,D"
                    if geometry_allowed
                    else "BLOCKED · " + ", ".join(intrinsics["applicability"]["reasons"])
                ),
                "base_extrinsics": "LOADED" if selected_extrinsics is not None else "NOT SELECTED",
                "terrain": terrain_state,
                "mountain_uv": mountain_uv_state,
                "target_reference": (
                    "GNSS + Mountain: paired frozen left/right u,v + numeric GNSS baseline; "
                    "Drone yaw: explicit target RTK + pixel/ray"
                ),
            },
            "intrinsics_applicability": intrinsics,
            "terrain_asset": dict(self._terrain_snapshot or {}),
            "frozen_previews": frozen_previews,
            # UI projection key.  QImages never enter the append-only event log.
            "frozen_frames": frozen_native_frames if include_frames else {},
            "frozen_native_frames": frozen_native_frames if include_frames else {},
            # Persistent identity/path projection; unlike the UI frame keys,
            # this is safe to store in session_static.json.
            "frozen_frame_artifacts": frozen_frame_artifacts,
            "candidates": candidate_rows,
            "selection_history": [selection.selection_id for selection in self._selection_history],
            # Automatic proposals are audit/UI evidence only.  They contain
            # native points and frame identities, never QImages or live-frame
            # references, and are not copied into ``annotations`` here.
            "auto_suggestions": [dict(value) for value in self._auto_suggestions.values()],
            "accepted_auto_suggestion": (
                None
                if self._last_accepted_auto_suggestion is None
                else dict(self._last_accepted_auto_suggestion)
            ),
            "annotation_review": (
                None if self._annotation_review is None else dict(self._annotation_review)
            ),
            # Append-only Drone RTK capture-observation evidence (deep-copied
            # so a caller mutating the returned list/dicts cannot corrupt the
            # stored history). Solve/apply/candidate state above is entirely
            # unaffected by this key.
            "drone_rtk_capture_observations": [
                copy.deepcopy(observation) for observation in self._drone_rtk_observations
            ],
        }

    @staticmethod
    def _zoom(values: object, side: str) -> str:
        row = values.get(side, {}) if isinstance(values, Mapping) else {}
        index = row.get("index") if isinstance(row, Mapping) else None
        return "unknown" if index is None else str(index)

    @staticmethod
    def _pose_from_mapping(value: Mapping[str, Any], *, name: str) -> CameraPoseNed:
        rotation = value.get("rotation_ned_from_camera_rdf")
        center = value.get("center_ned_m")
        if rotation is not None and center is not None:
            return CameraPoseNed(tuple(tuple(row) for row in rotation), tuple(center))
        rpy = value.get("virtual_frd_rpy_deg")
        if rpy is None or center is None:
            raise ValueError(
                f"{name} requires rotation_ned_from_camera_rdf or virtual_frd_rpy_deg and center_ned_m"
            )
        parsed_rpy = _finite_vector(rpy, length=3, name=f"{name}.virtual_frd_rpy_deg")
        return CameraPoseNed.from_virtual_frd_rpy_deg(
            center_ned_m=center,
            roll_deg=parsed_rpy[0],
            pitch_deg=parsed_rpy[1],
            yaw_deg=parsed_rpy[2],
        )

    def _resolve_prior_poses(
        self,
        payload: Mapping[str, Any],
        *,
        prefer_frozen: bool,
    ) -> tuple[CameraPoseNed, CameraPoseNed]:
        explicit = payload.get("prior_poses")
        if isinstance(explicit, Mapping):
            left = explicit.get("left")
            right = explicit.get("right")
            if not isinstance(left, Mapping) or not isinstance(right, Mapping):
                raise ValueError("prior_poses must contain explicit left/right pose mappings")
            return (
                self._pose_from_mapping(left, name="prior_poses.left"),
                self._pose_from_mapping(right, name="prior_poses.right"),
            )
        if prefer_frozen and self._frozen_prior_poses is not None:
            return self._frozen_prior_poses
        if self._active_poses is not None:
            return self._active_poses
        declared = getattr(self.panel, "optical_pose_nominal_poses", None)
        if (
            isinstance(declared, tuple)
            and len(declared) == 2
            and all(isinstance(pose, CameraPoseNed) for pose in declared)
        ):
            return declared
        profile = getattr(self.panel, "selected_extrinsics_profile", None)
        extrinsics = getattr(profile, "extrinsics", None)
        if extrinsics is None:
            active = getattr(self.panel, "active_intrinsics", None)
            extrinsics = getattr(active, "extrinsics", None)
        if extrinsics is None:
            raise RuntimeError(
                "an explicit prior pose or selected base stereo R,t is required; "
                "candidate values cannot be reused as their own prior"
            )
        left = CameraPoseNed.from_virtual_frd_rpy_deg(
            center_ned_m=(0.0, 0.0, 0.0),
            roll_deg=0.0,
            pitch_deg=0.0,
            yaw_deg=0.0,
        )
        rotation_right_from_left = tuple(
            tuple(float(value) for value in row)
            for row in extrinsics.rotation_left_to_right
        )
        rotation_ned_from_right = _matmul(
            left.rotation_ned_from_camera_rdf,
            _mat_transpose(rotation_right_from_left),
        )
        translation = _finite_vector(
            extrinsics.translation_left_to_right_m,
            length=3,
            name="base.translation_left_to_right_m",
        )
        translated_ned = _matvec(rotation_ned_from_right, translation)
        right = CameraPoseNed(
            rotation_ned_from_right,
            tuple(-value for value in translated_ned),
        )
        return left, right

    @staticmethod
    def _candidate_mapping_with_prior(
        candidate: OpticalPoseCandidate,
        prior: tuple[CameraPoseNed, CameraPoseNed],
    ) -> dict[str, Any]:
        mapping = _candidate_mapping(candidate)
        deltas: list[list[float]] = []
        for pose, base in zip((candidate.left_pose_ned, candidate.right_pose_ned), prior):
            current = pose_parameters_from_camera_pose(pose)
            previous = pose_parameters_from_camera_pose(base)
            deltas.append(
                [
                    current.roll_deg - previous.roll_deg,
                    current.pitch_deg - previous.pitch_deg,
                    ((current.yaw_deg - previous.yaw_deg + 180.0) % 360.0) - 180.0,
                ]
            )
        mapping["left_delta_rpy_deg"] = deltas[0]
        mapping["right_delta_rpy_deg"] = deltas[1]
        mapping["prior_pose_sha256"] = _payload_sha256(
            {"left": _pose_mapping(prior[0]), "right": _pose_mapping(prior[1])}
        )
        return mapping

    def _zoom_mapping(self, side: str) -> dict[str, Any]:
        values = getattr(self.panel, "actual_visible_zoom", {}) or {}
        row = values.get(side, {}) if isinstance(values, Mapping) else {}
        return dict(row) if isinstance(row, Mapping) else {}

    def _run_condition(self) -> dict[str, Any]:
        snapshot = getattr(self.panel, "workstation_run_setup_snapshot", None)
        if not callable(snapshot):
            return {}
        try:
            value = snapshot()
        except Exception:
            return {}
        condition = value.get("condition") if isinstance(value, Mapping) else None
        return dict(condition) if isinstance(condition, Mapping) else {}

    def _declared_profile_value(self, side: str, kind: str) -> str | None:
        identities = getattr(self.panel, "operating_intrinsics_identity", {}) or {}
        row = identities.get(side, {}) if isinstance(identities, Mapping) else {}
        if isinstance(row, Mapping):
            value = row.get(kind)
            if value not in (None, ""):
                return str(value)
        condition = self._run_condition()
        aliases = (
            f"{side}_{kind}",
            f"{side}_camera_{kind}",
            f"camera_{kind}_{side}",
        )
        for key in aliases:
            value = condition.get(key)
            if value not in (None, ""):
                return str(value)
        return None

    def _operating_intrinsics_profile(self) -> OperatingIntrinsicsProfile | None:
        for name in ("operating_intrinsics_profile", "_operating_intrinsics_profile"):
            value = getattr(self.panel, name, None)
            if isinstance(value, OperatingIntrinsicsProfile):
                return value
        active = getattr(self.panel, "active_intrinsics", None)
        if active is None:
            return None
        source_zoom = getattr(self.panel, "_active_intrinsics_source_zoom_level", None)
        source_hash = str(getattr(self.panel, "_active_intrinsics_profile_hash", "") or "")
        if len(source_hash) != 64:
            source_hash = ""
        return OperatingIntrinsicsProfile.from_stereo_result(
            active,
            source_zoom_index=source_zoom,
            left_camera_serial=self._declared_profile_value("left", "serial"),
            right_camera_serial=self._declared_profile_value("right", "serial"),
            left_stabilization_state=self._declared_profile_value("left", "stabilization_state"),
            right_stabilization_state=self._declared_profile_value("right", "stabilization_state"),
            profile_id=(f"active-kd-{source_hash[:12]}" if source_hash else "active-kd-memory"),
            source_path=str(getattr(self.panel, "_active_intrinsics_profile_path", "") or ""),
            source_content_sha256=source_hash,
            provenance={
                "source": "Workstation active_intrinsics immutable copy",
                "serial_and_stabilization": "explicit run/profile declarations; never inferred",
            },
        )

    @staticmethod
    def _metadata_text(metadata: Mapping[str, Any], *names: str) -> str | None:
        for name in names:
            value = metadata.get(name)
            if value not in (None, ""):
                return str(value)
        return None

    def _camera_evidence(self, side: str, *, use_frozen: bool) -> OperatingCameraEvidence:
        if use_frozen and side in self._frozen_frames:
            frozen = self._frozen_frames[side]
            metadata = dict(frozen["metadata"])
            zoom = dict(frozen["zoom"])
            image = frozen["image"]
            width, height = image.width(), image.height()
        else:
            values = getattr(self.panel, "camera_frame_metadata", {}) or {}
            row = values.get(side, {}) if isinstance(values, Mapping) else {}
            metadata = dict(row) if isinstance(row, Mapping) else {}
            zoom = self._zoom_mapping(side)
            width_value = metadata.get("native_width", metadata.get("width"))
            height_value = metadata.get("native_height", metadata.get("height"))
            try:
                width = int(width_value) if width_value is not None else None
                height = int(height_value) if height_value is not None else None
            except (TypeError, ValueError):
                width = height = None
        identity = getattr(self.panel, "camera_identity_evidence", {}) or {}
        identity_row = identity.get(side, {}) if isinstance(identity, Mapping) else {}
        identity_row = identity_row if isinstance(identity_row, Mapping) else {}
        serial = self._metadata_text(
            metadata,
            "camera_serial",
            "serial",
            "device_serial",
        ) or self._metadata_text(identity_row, "camera_serial", "serial") or self._declared_profile_value(side, "serial")
        stabilization = self._metadata_text(
            metadata,
            "stabilization_state",
            "stabilization",
        ) or self._metadata_text(identity_row, "stabilization_state", "stabilization") or self._declared_profile_value(side, "stabilization_state")
        return OperatingCameraEvidence(
            side=side,
            zoom_index=zoom.get("index"),
            zoom_ratio=zoom.get("ratio"),
            zoom_updated_monotonic_s=zoom.get("updated_monotonic_s"),
            frame_width_px=width,
            frame_height_px=height,
            camera_serial=serial,
            stabilization_state=stabilization,
        )

    def _intrinsics_snapshot(self, *, use_frozen: bool) -> dict[str, Any]:
        profile = (
            self._frozen_intrinsics_profile
            if use_frozen
            else self._operating_intrinsics_profile()
        )
        left = self._camera_evidence("left", use_frozen=use_frozen)
        right = self._camera_evidence("right", use_frozen=use_frozen)
        now = (
            self._frozen_capture_monotonic_s
            if use_frozen and self._frozen_capture_monotonic_s is not None
            else time.monotonic()
        )
        extrinsics = getattr(self.panel, "selected_extrinsics_profile", None)
        result = evaluate_intrinsics_applicability(
            profile,
            left,
            right,
            now_monotonic_s=now,
            extrinsics_source_zoom_index=getattr(extrinsics, "source_zoom_level", None),
        )
        return {
            "schema": "optical-pose-intrinsics-snapshot-v1",
            "evidence_scope": "frozen_capture" if use_frozen else "current_live",
            "profile": None if profile is None else profile.to_mapping(),
            "applicability": result.to_mapping(),
            "camera_evidence": {
                "left": dict(left.__dict__),
                "right": dict(right.__dict__),
            },
        }

    def _require_frozen_intrinsics(
        self,
    ) -> tuple[OperatingIntrinsicsProfile, Mapping[str, Any]]:
        missing = [side for side in ("left", "right") if side not in self._frozen_frames]
        if missing:
            raise RuntimeError(
                "capture a frozen native left/right burst before solving; missing "
                + ", ".join(missing)
            )
        snapshot = self._intrinsics_snapshot(use_frozen=True)
        applicability = snapshot["applicability"]
        if not applicability["geometry_allowed"]:
            raise RuntimeError(
                "frozen burst lacks applicable exact operating K,D: "
                + ", ".join(applicability["reasons"])
            )
        assert self._frozen_intrinsics_profile is not None
        return self._frozen_intrinsics_profile, applicability

    @staticmethod
    def _annotation_root(payload: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
        raw = payload.get("annotations")
        raw = raw if isinstance(raw, Mapping) else {}
        result: dict[str, Mapping[str, Any]] = {}
        for side in ("left", "right"):
            value = raw.get(side)
            if value is None:
                value = payload.get(f"{side}_annotation")
            to_mapping = getattr(value, "to_mapping", None)
            if callable(to_mapping):
                value = to_mapping()
            if isinstance(value, Mapping):
                result[side] = dict(value)
        return result

    def _validate_annotation_identity(
        self,
        side: str,
        annotation: Mapping[str, Any],
    ) -> None:
        frozen = self._frozen_frames.get(side)
        if frozen is None:
            raise RuntimeError(f"{side}: no frozen native frame")
        artifact: NativeFrameArtifact = frozen["artifact"]
        if str(annotation.get("frame_id") or "") != artifact.frame_id:
            raise ValueError(f"{side}: annotation frame_id does not match frozen evidence")
        if str(annotation.get("image_sha256") or "") != artifact.sha256:
            raise ValueError(f"{side}: annotation image_sha256 does not match frozen PNG evidence")
        width = int(annotation.get("source_width_px", -1))
        height = int(annotation.get("source_height_px", -1))
        if (width, height) != (artifact.width, artifact.height):
            raise ValueError(f"{side}: annotation native dimensions do not match frozen evidence")

    def _annotation_points(
        self,
        annotation: Mapping[str, Any],
        key: str,
    ) -> tuple[tuple[float, float], ...]:
        raw = annotation.get(key)
        if not isinstance(raw, (tuple, list)):
            return ()
        width = int(annotation["source_width_px"])
        height = int(annotation["source_height_px"])
        parsed: list[tuple[float, float]] = []
        for index, point in enumerate(raw):
            if isinstance(point, Mapping):
                value = (point.get("u_px"), point.get("v_px"))
            else:
                value = point
            u, v = _finite_vector(value, length=2, name=f"{key}[{index}]")
            if not (0.0 <= u <= width - 1 and 0.0 <= v <= height - 1):
                raise ValueError(f"{key}[{index}] is outside the native image")
            parsed.append((u, v))
        return tuple(parsed)

    @staticmethod
    def _validate_mountain_point_distribution(
        points: Sequence[tuple[float, float]],
        *,
        side: str,
    ) -> None:
        """Fail closed for duplicate or nearly one-dimensional Mountain clicks."""

        if len(points) < 4:
            raise ValueError(
                f"GNSS + Mountain requires at least four left/right u,v correspondence pairs ({side})"
            )
        unique: list[tuple[float, float]] = []
        for point in points:
            if all(
                math.hypot(point[0] - other[0], point[1] - other[1]) >= 1.0
                for other in unique
            ):
                unique.append(point)
        if len(unique) < 4:
            raise ValueError(
                f"GNSS + Mountain {side} points contain duplicate/near-duplicate observations"
            )
        mean_u = sum(point[0] for point in unique) / len(unique)
        mean_v = sum(point[1] for point in unique) / len(unique)
        xx = sum((point[0] - mean_u) ** 2 for point in unique)
        yy = sum((point[1] - mean_v) ** 2 for point in unique)
        xy = sum((point[0] - mean_u) * (point[1] - mean_v) for point in unique)
        trace = xx + yy
        determinant = max(0.0, xx * yy - xy * xy)
        if trace <= 1.0e-12:
            raise ValueError(f"GNSS + Mountain {side} points are coincident")
        discriminant = max(0.0, trace * trace - 4.0 * determinant)
        minimum = 0.5 * (trace - math.sqrt(discriminant))
        maximum = 0.5 * (trace + math.sqrt(discriminant))
        if maximum <= 1.0e-12 or minimum / maximum < 1.0e-3:
            raise ValueError(
                f"GNSS + Mountain {side} points are near-collinear; distribute at least four features over the image"
            )

    @staticmethod
    def _sample_horizon_pixels(
        points: Sequence[tuple[float, float]],
    ) -> tuple[tuple[float, float], ...]:
        if len(points) != 2:
            return tuple(points)
        first, second = points
        return tuple(
            (
                first[0] + (second[0] - first[0]) * index / 32.0,
                first[1] + (second[1] - first[1]) * index / 32.0,
            )
            for index in range(33)
        )

    @staticmethod
    def _undistort_pixels(camera: Any, points: Sequence[Sequence[float]]) -> tuple[tuple[float, float], ...]:
        try:
            import cv2
            import numpy as np
        except Exception as exc:  # pragma: no cover - runtime dependency.
            raise RuntimeError("OpenCV and NumPy are required for optical-pose solving") from exc
        raw = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
        normalized = cv2.undistortPoints(
            raw,
            np.asarray(camera.camera_matrix, dtype=np.float64),
            np.asarray(camera.distortion_coefficients, dtype=np.float64),
        ).reshape(-1, 2)
        return tuple((float(point[0]), float(point[1])) for point in normalized)

    _PROFILE_ATTRIBUTE_NAMES = (
        "selected_extrinsics_profile",
        "active_intrinsics",
        "_selected_extrinsics_profile_path",
        "_selected_extrinsics_profile_hash",
        "_active_extrinsics_profile_path",
        "_active_extrinsics_profile_hash",
        "_active_extrinsics_source_zoom_level",
        "_active_calibration_selection_mode",
        "_active_optical_pose_profile_hash",
        "_active_optical_pose_selection_id",
        "_active_optical_pose_effective_from_pair_sequence",
    )

    def _capture_profile_state(self) -> dict[str, Any]:
        return {
            "attributes": {
                name: (hasattr(self.panel, name), getattr(self.panel, name, None))
                for name in self._PROFILE_ATTRIBUTE_NAMES
            },
            "active_selection": self._active_selection,
            "active_poses": self._active_poses,
        }

    def _restore_profile_state(self, state: Mapping[str, Any]) -> None:
        attributes = state.get("attributes")
        if isinstance(attributes, Mapping):
            for name, row in attributes.items():
                existed, value = row
                if existed:
                    setattr(self.panel, name, value)
                elif hasattr(self.panel, name):
                    delattr(self.panel, name)
        self._active_selection = state.get("active_selection")
        self._active_poses = state.get("active_poses")

    def _install_selected_profile(
        self,
        selection: OpticalPoseSelection,
        candidate: OpticalPoseCandidate,
        poses: tuple[CameraPoseNed, CameraPoseNed],
        intrinsics_snapshot: Mapping[str, Any],
    ) -> StereoExtrinsicsProfile:
        stereo = stereo_pose_from_absolute(*poses)
        previous = getattr(self.panel, "selected_extrinsics_profile", None)
        previous_extrinsics = getattr(previous, "extrinsics", None)
        quality = dict(candidate.quality_metrics)
        reprojection_error = quality.get("reprojection_error_px")
        if reprojection_error is None:
            reprojection_error = float(
                getattr(previous_extrinsics, "reprojection_error_px", 0.0)
            )
        operating_zoom = intrinsics_snapshot["applicability"].get("operating_zoom_index")
        translation_zoom = (
            operating_zoom
            if any(
                getattr(candidate.optimized_dofs, name)
                for name in ("north_m", "east_m", "down_m")
            )
            else getattr(previous, "source_zoom_level", None)
        )
        rotation_zoom = (
            operating_zoom
            if any(
                getattr(candidate.optimized_dofs, name)
                for name in ("roll_deg", "pitch_deg", "yaw_deg")
            )
            else getattr(previous, "source_zoom_level", None)
        )
        known_sources = {value for value in (rotation_zoom, translation_zoom) if value is not None}
        source_zoom = next(iter(known_sources)) if len(known_sources) == 1 else None
        operating_profile = self._operating_intrinsics_profile()
        assert operating_profile is not None
        extrinsics = StereoExtrinsics(
            rotation_left_to_right=[list(row) for row in stereo.rotation_right_from_left_rdf],
            translation_left_to_right_m=list(stereo.translation_right_from_left_rdf_m),
            reprojection_error_px=float(reprojection_error),
            pair_count=len(candidate.source_observation_ids),
        )
        profile = StereoExtrinsicsProfile(
            extrinsics=extrinsics,
            source_zoom_level=source_zoom,
            method=f"optical_pose/{candidate.method}",
            source_intrinsics_path=operating_profile.source_path,
            source_intrinsics_sha256=operating_profile.intrinsics_math_sha256,
            profile_id=selection.selection_id,
            metadata={
                "schema": "optical-pose-selected-extrinsics-v1",
                "candidate_id": candidate.candidate_id,
                "effective_from_pair_sequence": selection.effective_from_pair_sequence,
                "selected_dofs": list(selection.selected_dofs.selected_names),
                "rotation_observation_zoom_index": rotation_zoom,
                "translation_source_zoom_index": translation_zoom,
                "operating_intrinsics_profile_sha256": operating_profile.profile_sha256,
                "previous_profile_id": selection.previous_profile_id,
                "coordinate_contract": "OpenCV optical RDF; X_right=R_right_from_left*X_left+T metres",
            },
        )
        setattr(self.panel, "selected_extrinsics_profile", profile)
        # Kept separate from ``selected_extrinsics_profile`` (which a later
        # checkerboard load would overwrite) so the geometry-comparison
        # display can always show the last-applied PnP profile on its own.
        setattr(self.panel, "optical_pose_extrinsics_profile", profile)
        setattr(self.panel, "_selected_extrinsics_profile_path", f"memory:{profile.profile_id}")
        setattr(self.panel, "_selected_extrinsics_profile_hash", profile.content_sha256)
        setattr(self.panel, "_active_extrinsics_profile_path", f"memory:{profile.profile_id}")
        setattr(self.panel, "_active_extrinsics_profile_hash", profile.content_sha256)
        setattr(self.panel, "_active_optical_pose_profile_hash", profile.content_sha256)
        setattr(self.panel, "_active_optical_pose_selection_id", selection.selection_id)
        setattr(
            self.panel,
            "_active_optical_pose_effective_from_pair_sequence",
            selection.effective_from_pair_sequence,
        )
        setattr(self.panel, "_active_extrinsics_source_zoom_level", profile.source_zoom_level)
        setattr(
            self.panel,
            "_active_calibration_selection_mode",
            "manual_independent_zoom" if source_zoom is None else "manual_same_zoom",
        )
        current_intrinsics = getattr(self.panel, "active_intrinsics", None)
        if current_intrinsics is None:
            raise RuntimeError("active exact K,D disappeared before profile installation")
        setattr(
            self.panel,
            "active_intrinsics",
            compose_stereo_calibration(current_intrinsics, profile),
        )
        return profile

    def _selection_mapping(
        self,
        selection: OpticalPoseSelection,
        poses: tuple[CameraPoseNed, CameraPoseNed],
    ) -> dict[str, Any]:
        stereo = stereo_pose_from_absolute(*poses)
        value = {
            "schema": "optical-pose-selection-v1",
            "state": "ACTIVE",
            "selection_id": selection.selection_id,
            "candidate_id": selection.candidate_id,
            "selected_dof": {
                name.removesuffix("_deg").removesuffix("_m"): bool(getattr(selection.selected_dofs, name))
                for name in POSE_DOF_NAMES
            },
            "effective_from_pair_sequence": selection.effective_from_pair_sequence,
            "applied_utc_ns": selection.applied_utc_ns,
            "previous_profile_id": selection.previous_profile_id,
            "left_pose": _pose_mapping(poses[0]),
            "right_pose": _pose_mapping(poses[1]),
            "relative_boresight_yaw_right_minus_left_deg": boresight_relative_yaw_right_minus_left_deg(*poses),
            "derived_stereo": {
                "R_right_from_left_rdf": [list(row) for row in stereo.rotation_right_from_left_rdf],
                "T_right_from_left_rdf_m": list(stereo.translation_right_from_left_rdf_m),
                "contract": "X_right_RDF=R_right<-left*X_left_RDF+T_right<-left metres",
            },
            "quality_snapshot": dict(selection.quality_snapshot),
        }
        value["profile_sha256"] = hashlib.sha256(canonical_json_bytes(value)).hexdigest()
        return value

    def _invalidate_measurement_cache(self) -> None:
        for name, value in (
            ("_last_measurement_attempt_key", None),
            ("_last_measurement_attempt", None),
            ("_last_distance_history_key", None),
        ):
            if hasattr(self.panel, name):
                setattr(self.panel, name, value)

    def _append_event(self, event_type: str, payload: Mapping[str, Any]) -> None:
        writer = getattr(self.panel, "append_workstation_run_event", None)
        if not callable(writer):
            return
        writer(
            event_type,
            payload,
            source="OpticalPoseController",
            coordinate_contract="OpenCV optical RDF absolute pose in world NED; derived virtual FRD RPY",
        )


__all__ = [
    "OPTICAL_POSE_CONTROLLER_METHODS",
    "OpticalPoseAutoSelectJob",
    "OpticalPoseController",
]
