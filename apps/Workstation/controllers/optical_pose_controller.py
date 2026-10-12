"""Owner/controller for native optical-pose evidence and immutable profiles.

The controller is intentionally the boundary between presentation-only UI and
geometry.  Native images are copied from :class:`CameraPane`, persisted as
lossless PNG evidence, and then frozen.  Every pixel observation is interpreted
as an OpenCV native-image pixel (u right, v down) using the exact operating
camera K,D.  Absolute camera poses use world NED and optical RDF; derived stereo
profiles retain OpenCV's ``X_right = R_right_from_left X_left + T`` contract.
"""

from __future__ import annotations

from Workstation.optical_pose.capture import (
    OpticalCaptureOwner,
    FrozenCaptureInputs,
    FrozenNativeFrame,
    CAPTURE_RESOURCE_FIELDS,
)
from Workstation.optical_pose.selection import OpticalReviewState
from Workstation.optical_pose.presentation import optical_pose_snapshot, measurement_evidence_snapshot, zoom
from Workstation.optical_pose.application import (
    OpticalProfileApplication,
    OpticalApplicationInputs,
    OpticalApplicationEffects,
    pose_mask,
    apply_candidate_with_constraints,
    active_intrinsics_for_apply,
    prepare_selected_profile,
    selection_mapping,
    profile_geometry_values,
    _PreparedOpticalPoseApplication as _PreparedOpticalPoseApplication,
    _KD_INDEPENDENT_CANDIDATE_METHODS as _KD_INDEPENDENT_CANDIDATE_METHODS,
)
from Workstation.optical_pose.artifacts import (
    _load_json_mapping,
    accepted_suggestion_alignment_row as accepted_suggestion_alignment_row,
)
from Workstation.optical_pose.observations import (
    _gps_state_observation_mapping,
    _frame_clock_domain_verified as _frame_clock_domain_verified,
    _drone_rtk_sync_mapping as _drone_rtk_sync_mapping,
    _drone_rtk_observation_state as _drone_rtk_observation_state,
)
from Workstation.optical_pose.drone import solve_drone_multiview_candidate, solve_drone_yaw_candidate
from Workstation.optical_pose.observations import coerce_gps_state, resolve_mountain_baseline, frozen_pair_simultaneity
from Workstation.optical_pose.mountain import solve_mountain_candidate, resolve_mountain_lever_arm
from Workstation.optical_pose.geometry_values import _matmul
from Workstation.optical_pose.policy import (
    _ALL_DOF,
    _TRANSLATION_DOF,
)
from Workstation.optical_pose.terrain_solver import solve_terrain_candidate
from Workstation.optical_pose.horizon import HorizonPoseSolver
from Workstation.optical_pose.observations import (
    validate_annotation_identity,
    annotation_points,
    sample_horizon_pixels,
    undistort_pixels,
)
from Workstation.optical_pose.geometry_values import _finite_vector, _mask_observability
from Workstation.calibration_workflow.calibration_geometry_state import apply_geometry_values, restore_geometry_values
from Workstation.optical_pose.candidates import (
    OpticalCandidateRegistry,
    _candidate_mapping,
    candidate_mapping_with_prior,
)

from Workstation.optical_pose.auto_selection import (
    AutoSelectionDetectors,
    FrozenAutoSelectionFrame,
    FrozenAutoSelectionInputs,
    OpticalAutoSelection,
    OpticalPoseAutoSelectJob,
)
from Workstation.optical_pose.geometry_values import (
    _mat_transpose,
    _matvec,
    _payload_sha256,
    _pose_mapping as _pose_mapping,
)
from Workstation.workstation_core.immutable_json import FrozenJsonObject

import copy
import math
import time
import uuid
from pathlib import Path
from typing import Any, Mapping, Sequence

from PyQt5 import QtCore, QtGui

from Workstation.workstation_core.native_frame_store import NativeFrameStore
from Workstation.workstation_core.optical_pose_auto_select import (
    HORIZON_MIN_ENDPOINT_SEPARATION_FRACTION,
    auto_select_horizon,
    auto_select_mountain,
    validate_mountain_point_distribution,
)
from Workstation.workstation_core.operating_intrinsics import (
    OperatingCameraEvidence,
    OperatingIntrinsicsProfile,
    build_nominal_operating_intrinsics_profile,
    evaluate_intrinsics_applicability,
)

from Workstation.workstation_core.optical_pose import (
    CameraPoseNed,
    OpticalPoseCandidate,
    PoseDofMask,
    apply_selected_dofs,
    pose_parameters_from_camera_pose,
)
from Workstation.workstation_core.baseline_alignment import baseline_from_gps
from Workstation.workstation_core.ideal_disparity import target_distance_from_gps
from Workstation.controllers.run_evidence_controller import persistence_failure_error
from Workstation.workstation_core.rtk_geometry import (
    ned_delta_m,
)
from Workstation.workstation_core.terrain import inspect_geotiff
from Workstation.wiris_workers.capture import WirisPnPPhotoCaptureWorker


OPTICAL_POSE_CONTROLLER_METHODS = (
    "reset_workstation_optical_pose_session",
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
    "auto_select_workstation_optical_pose_cloud",
    "solve_workstation_optical_pose_cloud_rotation",
    "accept_workstation_optical_pose_suggestion",
    "record_workstation_optical_pose_annotation",
    "workstation_optical_pose_candidate_poses",
    "workstation_optical_pose_measurement_evidence",
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


# ``app-receive`` frame and GNSS timestamps share the host monotonic clock,
# but a shared clock alone does not make arbitrarily distant samples
# synchronized.  Keep the capture-evidence gate explicit and conservative;
# callers may retain the samples as WARNING evidence when either bound fails.
# Cloud far-field correspondences are only meaningful when both frozen frames
# were captured together; reuse the Drone RTK frame-pair bound.
# A single RTK target/ray pair only constrains yaw (see estimate_drone_yaw_deg's
# closed form, which fixes roll/pitch). Two or more non-degenerate target/ray
# pairs over-determine the full rotation with the optical center still fixed,
# so the joint least-squares path below optimizes all three rotation DOF.
# ``register_workstation_optical_pose_reference_candidate`` packages these two
# methods from evidence that never touches the operating K,D profile
# (stereoCalibrate's own stored extrinsics; a live RTK GNSS baseline), unlike
# every PnP/Horizon/Mountain/Drone-yaw candidate, which calls
# ``_require_frozen_intrinsics``/embeds ``operating_intrinsics_profile_sha256``
# because their pose *is* derived through K,D. Apply's "K,D changed since this
# candidate was created" staleness check is therefore meaningless for these
# two methods and is skipped for them (the separate "K,D is currently
# applicable at all" gate still applies to every candidate, since Apply always
# needs a valid live K,D to build the resulting active stereo geometry).


def _candidate_state(name: str) -> property:
    """Named compatibility fields reference one registry; never copy its state."""
    return property(
        lambda owner: getattr(owner._candidate_registry, name),
        lambda owner, value: setattr(owner._candidate_registry, name, value),
    )


def _owned_optical_state(path, name):
    def target(controller):
        value = controller
        for field in path:
            value = getattr(value, field)
        return value

    return property(
        lambda controller: getattr(target(controller), name),
        lambda controller, value: setattr(target(controller), name, value),
    )


class OpticalPoseController:
    """Keep native observations, priors, candidates, and selections auditable."""

    _frozen_frames = _owned_optical_state(("capture_owner",), "_frozen_frames")
    _frozen_intrinsics_profile = _owned_optical_state(("capture_owner",), "_frozen_intrinsics_profile")
    _frozen_prior_poses = _owned_optical_state(("capture_owner",), "_frozen_prior_poses")
    _frozen_yaw90_reference = _owned_optical_state(("capture_owner",), "_frozen_yaw90_reference")
    _frozen_capture_monotonic_s = _owned_optical_state(("capture_owner",), "_frozen_capture_monotonic_s")
    _pending_wiris_ftp_payload = _owned_optical_state(("capture_owner",), "_pending_wiris_ftp_payload")
    _drone_rtk_observations = _owned_optical_state(("capture_owner", "observations"), "_drone_rtk_observations")
    _drone_rtk_multiview_samples = _owned_optical_state(
        ("capture_owner", "observations"), "_drone_rtk_multiview_samples"
    )
    _auto_suggestions = _owned_optical_state(("_review",), "_auto_suggestions")
    _last_accepted_auto_suggestion = _owned_optical_state(("_review",), "_last_accepted_auto_suggestion")
    _annotation_review = _owned_optical_state(("_review",), "_annotation_review")
    _mountain_ridge_margin_fraction = _owned_optical_state(("_review",), "_mountain_ridge_margin_fraction")
    _mountain_max_points = _owned_optical_state(("_review",), "_mountain_max_points")
    _profile_restore_stack = _owned_optical_state(("_application",), "_profile_restore_stack")
    _terrain_metadata = _owned_optical_state(("capture_owner", "artifacts"), "terrain_metadata")
    _terrain_snapshot = _owned_optical_state(("capture_owner", "artifacts"), "terrain_snapshot")

    _candidates = _candidate_state("_candidates")
    _candidate_priors = _candidate_state("_candidate_priors")
    _candidate_drone_rotation_context = _candidate_state("_candidate_drone_rotation_context")
    _active_selection = _candidate_state("_active_selection")
    _active_poses = _candidate_state("_active_poses")
    _selection_history = _candidate_state("_selection_history")

    _MOUNTAIN_RIDGE_MARGIN_DEFAULT = 0.06
    _MOUNTAIN_RIDGE_MARGIN_MIN = 0.02
    _MOUNTAIN_RIDGE_MARGIN_MAX = 0.15
    # 0 = keep every RANSAC inlier (no display/storage sampling); a positive
    # value caps the spread selection at that many points (minimum 4).
    _MOUNTAIN_MAX_POINTS_DEFAULT = 0
    _MOUNTAIN_MAX_POINTS_MIN = 4
    _MOUNTAIN_MAX_POINTS_MAX = 500

    def __init__(self, panel: object) -> None:
        self.panel = panel
        self._candidate_registry = OpticalCandidateRegistry()
        self._review = OpticalReviewState(
            lambda name, value: self._append_event(name, value),
            lambda accepted: self._persist_accepted_suggestion_alignment_row(accepted),
        )
        self.capture_owner = OpticalCaptureOwner(
            review=self._review,
            emit=lambda name, value: self._append_event(name, value),
            log=lambda message: self.panel._log(message),
            parent=panel if isinstance(panel, QtCore.QObject) else None,
            publish_legacy=self._publish_capture_resources,
        )
        for name in CAPTURE_RESOURCE_FIELDS:
            self.capture_owner._set_resource(name, getattr(panel, name, None))
        self._application = OpticalProfileApplication(self._candidate_registry)

    def _publish_capture_resources(self, owner):
        # Production fields are descriptors on the shared feature state. Legacy
        # embedders still receive the same identities at this finite outer edge.
        for name, value in owner.resource_inventory().items():
            setattr(self.panel, name, value)

    def resource_inventory(self):
        return self.capture_owner.resource_inventory()

    def request_shutdown(self):
        return self.capture_owner.request_shutdown()

    def _frozen_artifacts(self):
        return {side: row["artifact"] for side, row in self._frozen_frames.items()}

    def _require_open_session_id(self) -> str:
        if not bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", True)):
            return "alignment"
        session_id = str(
            getattr(self.panel, "_measurement_session_id", "") or getattr(self.panel, "recording_session_id", "") or ""
        )
        session_state = str(getattr(self.panel, "_measurement_session_state", "IDLE") or "IDLE").upper()
        hierarchy_enabled = bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", False))
        if not session_id or (hierarchy_enabled and session_state != "OPEN"):
            raise RuntimeError("Open Session is required before Alignment capture")
        return session_id

    def reset_workstation_optical_pose_session(self) -> None:
        """Clear the current app-lifetime Alignment state explicitly."""

        if not bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", True)):
            writer = getattr(self.panel, "append_workstation_data", None)
            if callable(writer):
                writer(
                    "alignment",
                    {
                        "schema": "workstation-alignment-event-v1",
                        "event_type": "alignment_reset",
                        "reset_utc_ns": time.time_ns(),
                        "previous_active_selection_id": (
                            None if self._active_selection is None else self._active_selection.selection_id
                        ),
                        "previous_active_profile_hash": str(
                            getattr(
                                self.panel,
                                "_active_optical_pose_profile_hash",
                                "",
                            )
                            or ""
                        ),
                    },
                    priority=True,
                )

        self._application.reset()
        self.capture_owner.reset()
        self._candidates.clear()
        self._candidate_priors.clear()
        self._candidate_drone_rotation_context.clear()
        self._active_selection = None
        self._active_poses = None
        self._selection_history.clear()
        setattr(self.panel, "selected_extrinsics_profile", None)
        setattr(self.panel, "_active_optical_pose_selection_id", "")
        setattr(self.panel, "_active_optical_pose_profile_hash", "")
        setattr(self.panel, "_active_optical_pose_effective_from_pair_sequence", None)
        setattr(self.panel, "_active_alignment_snapshot_path", "")
        setattr(self.panel, "_active_alignment_snapshot_sha256", "")
        self._invalidate_measurement_cache()

    def _alignment_storage_context(self) -> tuple[str, Path]:
        """Resolve the Alignment persistence boundary without changing state."""

        if not bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", True)):
            resolver = getattr(self.panel, "_recording_session_dir", None)
            if not callable(resolver):
                raise RuntimeError("measurement data directory owner is unavailable")
            return "alignment", Path(resolver("alignment")) / "alignment"
        session_id = self._require_open_session_id()
        resolver = getattr(self.panel, "_measurement_session_dir", None)
        if not callable(resolver):
            resolver = getattr(self.panel, "_recording_session_dir", None)
        if not callable(resolver):
            raise RuntimeError("measurement session directory owner is unavailable")
        session_root = Path(resolver(session_id))
        if session_root.exists() and not session_root.is_dir():
            raise RuntimeError("measurement session storage owner is not a directory")
        return session_id, session_root / "alignment"

    def _persist_session_alignment_snapshot(self) -> Path:
        session_id, alignment_dir = self._alignment_storage_context()
        publication = self.capture_owner.artifacts.persist_alignment(
            hierarchy_enabled=bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", True)),
            session_id=session_id,
            alignment_dir=alignment_dir,
            snapshot=self.workstation_optical_pose_snapshot(include_frames=False),
            writer=getattr(self.panel, "append_workstation_data", None),
        )
        setattr(self.panel, "_active_alignment_snapshot_path", str(publication.version_path))
        setattr(self.panel, "_active_alignment_snapshot_sha256", publication.sha256)
        return publication.target

    def register_workstation_optical_pose_candidate(
        self, candidate: OpticalPoseCandidate, *, prior_poses: tuple[CameraPoseNed, CameraPoseNed] | None = None
    ) -> dict[str, Any]:
        self._candidate_registry.validate_new(candidate)
        resolved_prior = prior_poses or self._resolve_prior_poses({}, prefer_frozen=True)
        mapping = self._candidate_registry.register(candidate, resolved_prior)
        self._append_event("pose_candidate_created", mapping)
        return mapping

    def workstation_optical_pose_candidate_poses(self) -> tuple[dict[str, Any], ...]:
        return self._candidate_registry.latest_poses()

    def workstation_optical_pose_measurement_evidence(self):
        return measurement_evidence_snapshot(self._candidate_registry, self._review)

    _mask = staticmethod(pose_mask)

    def apply_workstation_optical_pose_candidate(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        # Resolve storage before publishing any active field. Application owns
        # validation, preparation, selection and the restore token lifetime.
        inputs = OpticalApplicationInputs(
            intrinsics_snapshot=self._intrinsics_snapshot(use_frozen=False),
            current_intrinsics=getattr(self.panel, "active_intrinsics", None),
            active_intrinsics_path=str(getattr(self.panel, "_active_intrinsics_profile_path", "") or ""),
            active_intrinsics_hash=str(getattr(self.panel, "_active_intrinsics_profile_hash", "") or ""),
            previous_profile=getattr(
                self.panel, "active_extrinsics_profile", getattr(self.panel, "selected_extrinsics_profile", None)
            ),
            next_pair_sequence=len(getattr(self.panel, "distance_recording_rows", ()) or ()),
        )
        result = self._application.apply(payload, inputs, self._application_effects())
        self._append_event("pose_candidate_applied", result)
        return result

    def _application_effects(self):
        return OpticalApplicationEffects(
            capture=self._capture_profile_state,
            commit_geometry=lambda values: apply_geometry_values(self.panel, values),
            restore=self._restore_profile_state,
            invalidate=self._invalidate_measurement_cache,
            persist=self._persist_session_alignment_snapshot,
            revision=self._geometry_revision,
            preflight=self._alignment_storage_context,
        )

    def _geometry_revision(self):
        application = getattr(self.panel, "application", None)
        store = getattr(application, "geometry_store", None)
        if store is None:
            store = getattr(getattr(self.panel, "state", None), "geometry_store", None)
        if store is not None:
            return (id(store), store.snapshot().revision)
        # Legacy embedders have no revision owner. Compare every geometry
        # identity, including K,D and optical selection, before rollback.
        return tuple(
            (name, hasattr(self.panel, name), id(getattr(self.panel, name, None)))
            for name in self._PROFILE_ATTRIBUTE_NAMES
            if name not in {"_active_alignment_snapshot_path", "_active_alignment_snapshot_sha256"}
        )

    _apply_candidate_with_constraints = staticmethod(apply_candidate_with_constraints)

    def revert_workstation_optical_pose(self, _payload: Mapping[str, Any] | None = None) -> dict[str, Any]:
        result = self._application.revert(self._application_effects())
        self._append_event("pose_selection_reverted", result)
        return result

    def _capture_drone_rtk_observation(
        self, *, session_id, burst_id, frozen, gnss_snapshot=None, gnss_snapshot_source="live_capture_request"
    ):
        raw = getattr(self.panel, "latest_gps", {}) if gnss_snapshot is None else gnss_snapshot
        return self.capture_owner.observations.capture(
            session_id=session_id,
            burst_id=burst_id,
            frozen=frozen,
            gnss_snapshot=raw,
            gnss_snapshot_source=gnss_snapshot_source,
        )

    def _capture_observation_for_frozen_burst(self):
        return self.capture_owner.observations.for_burst(self._frozen_frames)

    def _record_drone_rtk_multiview_sample(self, review):
        return self.capture_owner.observations.record_sample(
            review,
            frozen=self._frozen_frames,
            profile=self._frozen_intrinsics_profile,
            intrinsics_snapshot=self._intrinsics_snapshot(use_frozen=True),
        )

    def _accumulated_drone_rtk_multiview_root(self):
        return self.capture_owner.observations.collection(profile=self._frozen_intrinsics_profile)

    def _capture_yaw90_reference(self) -> dict[str, Any]:
        from Workstation.workstation_core.optical_yaw90_reference import build_reference

        now, utc = time.monotonic(), time.time_ns()
        selector = getattr(self.panel, "_selected_baseline_gps_states", None)
        raw = getattr(self.panel, "latest_gps", {}) or {}
        positions = selector() if callable(selector) else (raw.get("left"), raw.get("right"))
        gps = {
            side: _gps_state_observation_mapping(value, role=side) for side, value in zip(("left", "right"), positions)
        }
        live = getattr(self.panel, "latest_extrinsics", {}) or {}
        attitudes = {}
        for side in ("left", "right"):
            sample = live.get(side)
            row = {}
            for key in ("roll_deg", "pitch_deg", "yaw_deg", "timestamp_s"):
                value = getattr(sample, key, None)
                row[key] = float(value) if isinstance(value, (int, float)) and math.isfinite(value) else None
            row["source"] = str(getattr(sample, "topic", "ATTITUDE"))
            attitudes[side] = row
        settings = getattr(self.panel, "settings", None)
        reader = getattr(self.panel, "_workstation_setting", lambda _key, default=None: default)
        reverse = bool(
            reader("measurement_baseline_yaw_reverse", getattr(settings, "measurement_baseline_yaw_reverse", False))
        )
        source_reader = getattr(self.panel, "_pixhawk_gnss_source", lambda _side: "GPS1")
        return build_reference(
            gps=gps,
            attitudes=attitudes,
            reverse=reverse,
            requested_monotonic_s=now,
            requested_utc_ns=utc,
            sources={side: source_reader(side) for side in ("left", "right")},
        )

    def capture_workstation_optical_pose_burst(self, payload=None):
        values = dict(payload or {})
        inputs = self._capture_inputs(values)
        frames = {}
        for side in ("left", "right"):
            pane = getattr(self.panel, f"{side}_pane", None)
            getter = getattr(pane, "latest_native_measurement_frame", None)
            if not callable(getter):
                raise RuntimeError(f"{side} native frame source is unavailable")
            result = getter()
            if not isinstance(result, tuple) or len(result) != 3:
                raise RuntimeError(f"{side} native frame is not available yet")
            timestamp, image, metadata = result
            if not isinstance(image, QtGui.QImage) or image.isNull():
                raise RuntimeError(f"{side} native frame is null")
            frames[side] = FrozenNativeFrame(timestamp, image, metadata if isinstance(metadata, Mapping) else {})
        self.capture_owner.capture_native(inputs, frames, store_factory=NativeFrameStore)
        return self.workstation_optical_pose_snapshot()

    def _capture_inputs(self, values, *, imported=False):
        if self.capture_owner.closing:
            raise RuntimeError("Optical capture is shutting down")
        session_id = self._require_open_session_id()
        resolver = getattr(self.panel, "_recording_session_dir", None)
        if not callable(resolver):
            raise RuntimeError("recording session directory owner is unavailable")
        root = Path(resolver(session_id))
        root.mkdir(parents=True, exist_ok=True)
        ensure = getattr(self.panel, "ensure_workstation_run_evidence", None)
        if callable(ensure):
            result = ensure(
                "optical_pose_capture_started",
                {"session_id": session_id},
                source="OpticalPoseController",
                coordinate_contract="native OpenCV pixels (u right, v down)",
            )
            error = persistence_failure_error(result)
            if error is not None:
                raise OSError(f"optical-pose evidence setup failed: {error}")
        captured_monotonic_s = time.monotonic()
        yaw90 = copy.deepcopy(values.get("_capture_yaw90_reference")) if imported else self._capture_yaw90_reference()
        profile = self.operating_intrinsics_profile_for_geometry()
        try:
            prior = self._resolve_prior_poses(values, prefer_frozen=False)
        except RuntimeError:
            prior = None
        raw_gps = values.get("_ftp_request_gnss_snapshot") if imported else getattr(self.panel, "latest_gps", {})
        gnss = (
            {side: _gps_state_observation_mapping(raw_gps.get(side), role=side) for side in ("left", "right", "drone")}
            if isinstance(raw_gps, Mapping)
            else None
        )
        return FrozenCaptureInputs(
            session_id,
            root,
            captured_monotonic_s,
            profile,
            prior,
            {
                "yaw90": yaw90,
                "gnss": gnss,
                "zoom": {side: self._zoom_mapping(side) for side in ("left", "right")},
                "metadata": getattr(self.panel, "camera_frame_metadata", {}) or {},
                "camera_identity": getattr(self.panel, "camera_identity_evidence", {}) or {},
                "declared_identity": getattr(self.panel, "operating_intrinsics_identity", {}) or {},
                "extrinsics_source_zoom_index": getattr(
                    self._frozen_or_live_extrinsics_profile(prefer_frozen=False), "source_zoom_level", None
                ),
            },
        )

    def capture_workstation_optical_pose_burst_wiris_ftp(self, payload=None):
        return self._start_ftp_capture("hr", payload)

    def _start_ftp_capture(self, mode, payload):
        ordered_modes = ("oz", "hr") if mode == "oz" else ("hr", "oz")
        for other in ordered_modes:
            prefix = "optical_pose_wiris_ftp_oz_" if other == "oz" else "optical_pose_wiris_ftp_"
            if getattr(self.capture_owner, prefix + "thread") is not None:
                label = " (OZ)" if other == "oz" else ""
                raise RuntimeError(f"a WIRIS FTP{label} alignment capture is already in progress")
        settings = getattr(self.panel, "settings", None)
        if not bool(getattr(settings, "wiris_ftp_enabled", False)):
            raise RuntimeError('WIRIS FTP capture requires "Enable FTP download" in Settings')
        targets = tuple(self.panel._camera_photo_target_infos())
        if len(targets) != 2:
            raise RuntimeError("WIRIS FTP capture failed: both WIRIS control hosts are required")
        values = copy.deepcopy(dict(payload or {}))
        inputs = self._capture_inputs(values)
        channel = "align_oz" if mode == "oz" else "align"
        prefix = "align-oz" if mode == "oz" else "align"
        capture_id = f"{prefix}-{time.time_ns()}-{uuid.uuid4().hex[:8]}"
        kwargs = copy.deepcopy(self.panel._wiris_ftp_worker_kwargs(output_dir=inputs.root / "frames"))
        if mode == "oz":
            kwargs["image_enable_command"] = "SIVI TRUE"
        values.update(
            {
                "_capture_yaw90_reference": inputs.evidence("yaw90"),
                "_ftp_request_gnss_snapshot": inputs.evidence("gnss"),
                "_ftp_request_zoom_snapshot": inputs.evidence("zoom"),
                "_ftp_request_monotonic_s": time.monotonic(),
            }
        )
        self.capture_owner.start_ftp(
            mode=mode,
            capture_id=capture_id,
            inputs=inputs,
            payload=values,
            worker_factory=lambda: WirisPnPPhotoCaptureWorker(
                targets, capture_id=capture_id, capture_channel=channel, completion_timeout_s=10.0, **kwargs
            ),
        )
        return self.workstation_optical_pose_snapshot()

    def _handle_optical_pose_wiris_ftp_capture_completed(self, result: object) -> None:
        if self.capture_owner.has_admitted:
            return
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
        if self.capture_owner.has_admitted:
            return
        self.panel._log(f"[정렬] WIRIS FTP Capture 실패: {error}")

    def _clear_optical_pose_wiris_ftp_capture_worker(self):
        self.capture_owner.clear_legacy("hr")

    _ftp_capture_failures = staticmethod(OpticalCaptureOwner.ftp_capture_failures)

    _ftp_import_sources = staticmethod(OpticalCaptureOwner.ftp_import_sources)

    def capture_workstation_optical_pose_burst_wiris_ftp_oz(self, payload=None):
        return self._start_ftp_capture("oz", payload)

    def _handle_optical_pose_wiris_ftp_oz_capture_completed(self, result: object) -> None:
        if self.capture_owner.has_admitted:
            return
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
        if self.capture_owner.has_admitted:
            return
        self.panel._log(f"[정렬] WIRIS FTP (OZ) Capture 실패: {error}")

    def _clear_optical_pose_wiris_ftp_oz_capture_worker(self):
        self.capture_owner.clear_legacy("oz")

    def import_workstation_optical_pose_burst(self, payload=None):
        values = dict(payload or {})
        sources = values.get("sources")
        sources = sources if isinstance(sources, Mapping) else {}
        missing = [side for side in ("left", "right") if not isinstance(sources.get(side), Mapping)]
        if missing:
            raise ValueError("import requires sources.left/right, each with a 'path': missing " + ", ".join(missing))
        inputs = self._capture_inputs(values, imported=True)
        self.capture_owner.import_burst(inputs, copy.deepcopy(dict(sources)), store_factory=NativeFrameStore)
        return self.workstation_optical_pose_snapshot()

    def _record_auto_suggestion(self, payload=None):
        self._review.record(payload or {})
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
            raise RuntimeError("capture a frozen native left/right burst before solving; missing " + ", ".join(missing))
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

    def set_workstation_optical_pose_mountain_ridge_margin(self, payload=None):
        self._review.set_ridge_margin(payload or {})
        return self.workstation_optical_pose_snapshot()

    def set_workstation_optical_pose_mountain_max_points(self, payload=None):
        self._review.set_max_points(payload or {})
        return self.workstation_optical_pose_snapshot()

    def clear_workstation_optical_pose_auto_suggestions(self, payload=None):
        self._review.clear(payload or {})
        return self.workstation_optical_pose_snapshot()

    def _capture_auto_selection_inputs(
        self, values: Mapping[str, Any], *, include_baseline: bool
    ) -> FrozenAutoSelectionInputs:
        profile, priors, warnings = self._auto_select_inputs(values)
        frames = tuple(
            FrozenAutoSelectionFrame(side, self._frozen_frames[side]["image"], self._frozen_frames[side]["artifact"])
            for side in ("left", "right")
        )
        baseline = None
        details = {}
        baseline_error = ""
        if include_baseline:
            try:
                baseline, details = self._resolve_gnss_baseline_for_mountain(values)
            except (RuntimeError, ValueError) as exc:
                baseline_error = str(exc)
        return FrozenAutoSelectionInputs(
            frames=frames,
            profile=profile,
            priors=priors,
            applicability_warnings=tuple(warnings),
            baseline_ned_m=baseline,
            baseline_details=FrozenJsonObject(details),
            baseline_error=baseline_error,
            ridge_margin_fraction=self._mountain_ridge_margin_fraction,
            max_points=self._mountain_max_points,
        )

    def prepare_workstation_optical_pose_auto_selection(
        self,
        mode: str,
        payload: Mapping[str, Any] | None = None,
    ) -> OpticalPoseAutoSelectJob:
        """Freeze the current owner evidence before background computation."""

        normalized_mode = str(mode or "").strip().lower()
        if normalized_mode not in {"horizon", "mountain", "cloud"}:
            raise ValueError(f"unsupported optical-pose auto-selection mode: {mode!r}")
        missing = [side for side in ("left", "right") if side not in self._frozen_frames]
        if missing:
            raise RuntimeError("capture a frozen native left/right burst before solving; missing " + ", ".join(missing))
        values = dict(payload or {})
        values["auto_mode"] = normalized_mode
        return OpticalPoseAutoSelectJob(
            inputs=self._capture_auto_selection_inputs(values, include_baseline=normalized_mode != "horizon"),
            mode=normalized_mode,
            detectors=AutoSelectionDetectors(auto_select_horizon, auto_select_mountain),
        )

    def _compute_auto_selection(self, mode: str, payload: Mapping[str, Any]) -> dict[str, Any]:
        return self.prepare_workstation_optical_pose_auto_selection(mode, payload).run()

    def commit_workstation_optical_pose_auto_selection(self, payload):
        self._review.commit(payload, artifacts=self._frozen_artifacts())
        return self.workstation_optical_pose_snapshot()

    def auto_select_workstation_optical_pose_horizon(
        self,
        payload: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Create and immediately commit a Horizon suggestion for legacy callers."""

        suggestion = self._compute_auto_select_horizon(dict(payload or {}))
        return self.commit_workstation_optical_pose_auto_selection(suggestion)

    def _compute_auto_select_horizon(self, payload: Mapping[str, Any] | None = None) -> dict[str, Any]:
        return self.prepare_workstation_optical_pose_auto_selection("horizon", payload).run()

    def auto_select_workstation_optical_pose_mountain(
        self,
        payload: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Create and immediately commit a Mountain suggestion for legacy callers."""

        suggestion = self._compute_auto_select_mountain(dict(payload or {}))
        return self.commit_workstation_optical_pose_auto_selection(suggestion)

    def auto_select_workstation_optical_pose_cloud(
        self,
        payload: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Create and immediately commit a Cloud far-field suggestion."""

        suggestion = self._compute_auto_select_cloud(dict(payload or {}))
        return self.commit_workstation_optical_pose_auto_selection(suggestion)

    def _compute_auto_select_cloud(self, payload: Mapping[str, Any] | None = None) -> dict[str, Any]:
        return self.prepare_workstation_optical_pose_auto_selection("cloud", payload).run()

    def _compute_auto_select_mountain(
        self, payload: Mapping[str, Any] | None = None, *, far_field_only: bool = False, mode: str = "mountain"
    ) -> dict[str, Any]:
        values = dict(payload or {})
        inputs = self._capture_auto_selection_inputs(values, include_baseline=True)
        computation = OpticalAutoSelection(inputs, AutoSelectionDetectors(auto_select_horizon, auto_select_mountain))
        return computation._compute_auto_select_mountain({}, far_field_only=far_field_only, mode=mode)

    _append_auto_copy_quality_warning = staticmethod(OpticalReviewState._append_auto_copy_quality_warning)

    def _validated_horizon_suggestion_for_acceptance(self, suggestion):
        return self._review._validated_horizon_suggestion_for_acceptance(suggestion, artifacts=self._frozen_artifacts())

    def _validated_mountain_suggestion_for_acceptance(self, suggestion, payload):
        return self._review._validated_mountain_suggestion_for_acceptance(
            suggestion, payload, artifacts=self._frozen_artifacts()
        )

    def accept_workstation_optical_pose_suggestion(self, payload):
        self._review.accept(payload, artifacts=self._frozen_artifacts())
        return self.workstation_optical_pose_snapshot()

    def _persist_accepted_suggestion_alignment_row(self, accepted: Mapping[str, Any]) -> None:
        """Append one compact ``alignment`` row for an accepted suggestion.

        Acceptance used to live only in memory plus one bulky run event, so
        nothing reached ``alignment.jsonl`` or the ``04_Alignment`` sheet.
        The row carries the evidence needed to audit a Measurement's camera
        relative rotation without re-parsing the event payload.
        """

        writer = getattr(self.panel, "append_workstation_data", None)
        if not callable(writer):
            return
        try:
            writer("alignment", accepted_suggestion_alignment_row(accepted), priority=True)
        except (OSError, RuntimeError, TypeError, ValueError):
            pass

    _annotation_mappings_from_suggestion = staticmethod(OpticalReviewState._annotation_mappings_from_suggestion)

    def _build_annotation_review(self, *, suggestion_id, provenance_state, annotation_mode, annotations):
        return self._review._build_annotation_review(
            suggestion_id=suggestion_id,
            provenance_state=provenance_state,
            annotation_mode=annotation_mode,
            annotations=annotations,
            artifacts=self._frozen_artifacts(),
        )

    def record_workstation_optical_pose_annotation(self, payload):
        generation = (self.capture_owner._generation, self.capture_owner._burst_revision)
        review = self._review.record_annotation(payload, artifacts=self._frozen_artifacts())
        if generation == (self.capture_owner._generation, self.capture_owner._burst_revision):
            self._record_drone_rtk_multiview_sample(review)
        return self.workstation_optical_pose_snapshot()

    def import_workstation_optical_pose_terrain(self, payload):
        value = self.capture_owner.artifacts.import_terrain(
            payload, importer=getattr(self.panel, "import_workstation_session_artifact", None), inspect=inspect_geotiff
        )
        self._append_event("terrain_geotiff_imported", value)
        return self.workstation_optical_pose_snapshot()

    _HORIZON_CLICK_UNCERTAINTY_PX = 1.0
    _HORIZON_MIN_ENDPOINT_SEPARATION_FRACTION = HORIZON_MIN_ENDPOINT_SEPARATION_FRACTION
    _HORIZON_MODELS = ("flat_local", "spherical_geometric")

    def solve_workstation_optical_pose_horizon(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        profile, _applicability = self._require_frozen_intrinsics()
        priors = self._resolve_prior_poses(payload, prefer_frozen=True)
        annotations = copy.deepcopy(self._annotation_root(payload))
        artifacts = {side: row["artifact"] for side, row in self._frozen_frames.items()}
        candidate = HorizonPoseSolver().solve(
            dict(payload),
            profile=profile,
            priors=priors,
            annotations=annotations,
            artifacts=artifacts,
        )
        self.register_workstation_optical_pose_candidate(candidate, prior_poses=priors)
        return self.workstation_optical_pose_snapshot()

    def solve_workstation_optical_pose_terrain(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        if self._terrain_metadata is None:
            raise RuntimeError("import and inspect a GeoTIFF before terrain PnP")
        if not self._terrain_metadata.full_pnp_height_ready:
            raise RuntimeError("terrain PnP requires an explicitly declared vertical datum; current value is unknown")
        (profile, _applicability) = self._require_frozen_intrinsics()
        priors = self._resolve_prior_poses(payload, prefer_frozen=True)
        annotations = self._annotation_root(payload)
        correspondence_root = payload.get("correspondences")
        correspondence_root = correspondence_root if isinstance(correspondence_root, Mapping) else {}
        correspondence_file_sha256 = ""
        if not correspondence_root and payload.get("correspondences_path"):
            (loaded, correspondence_file_sha256) = _load_json_mapping(
                payload.get("correspondences_path"), label="terrain correspondence JSON"
            )
            correspondence_root = loaded.get("correspondences", loaded)
            if not isinstance(correspondence_root, Mapping):
                raise ValueError("terrain correspondence JSON must contain a left/right object map")
        candidate = solve_terrain_candidate(
            profile=profile,
            priors=priors,
            annotations=copy.deepcopy(annotations),
            artifacts={side: row["artifact"] for (side, row) in self._frozen_frames.items()},
            correspondence_root=copy.deepcopy(correspondence_root),
            correspondence_file_sha256=correspondence_file_sha256,
            terrain_metadata=self._terrain_metadata,
        )
        self.register_workstation_optical_pose_candidate(candidate, prior_poses=priors)
        return self.workstation_optical_pose_snapshot()

    def _resolve_gnss_baseline_for_mountain(self, payload):
        return resolve_mountain_baseline(payload, observation=self._capture_observation_for_frozen_burst())

    def _frozen_pair_simultaneity(self):
        return frozen_pair_simultaneity(self._frozen_frames)

    _resolve_mountain_lever_arm = staticmethod(resolve_mountain_lever_arm)

    def solve_workstation_optical_pose_gnss_mountain(
        self,
        payload: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Build metric stereo R,t from GNSS and paired Mountain ``(u,v)`` clicks."""

        return self._solve_gnss_mountain(payload, rotation_only=False)

    def solve_workstation_optical_pose_cloud_rotation(
        self,
        payload: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Rotation-only Cloud solve from far-field sky/cloud correspondences.

        Same paired ridge-annotation ``(u,v)`` inputs and Mountain rotation
        solver, but the candidate never observes translation (the Cloud
        measurement method fills N/E/D from live GNSS) and the frozen pair
        must be simultaneous for the rotation to be apply-eligible.
        """

        return self._solve_gnss_mountain(payload, rotation_only=True)

    def _solve_gnss_mountain(self, payload: Mapping[str, Any], *, rotation_only: bool) -> dict[str, Any]:
        profile, _applicability = self._require_frozen_intrinsics()
        from Workstation.workstation_core.optical_yaw90_reference import reference_poses

        reference = copy.deepcopy(self._frozen_yaw90_reference)
        if reference is None:
            raise RuntimeError("Yaw90 capture reference missing; capture again with P1/GNSS inputs")
        priors = reference_poses(reference)
        candidate = solve_mountain_candidate(
            {
                key: copy.deepcopy(payload[key])
                for key in ("lever_arm_baseline_correction_left_rdf_m",)
                if key in payload
            },
            profile=profile,
            priors=priors,
            reference=reference,
            annotations=copy.deepcopy(self._annotation_root(payload)),
            artifacts={side: row["artifact"] for side, row in self._frozen_frames.items()},
            rotation_only=rotation_only,
            simultaneity=self._frozen_pair_simultaneity() if rotation_only else (None, None, ""),
        )
        self.register_workstation_optical_pose_candidate(candidate, prior_poses=priors)
        return self.workstation_optical_pose_snapshot()

    _coerce_gps_state = staticmethod(coerce_gps_state)

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
                "Telemetry geometry requires valid left/right/Drone GNSS positions; missing " + ", ".join(missing)
            )
        age_limit = max(0.0, float(payload.get("maximum_gps_age_s", 2.0)))
        ages = {
            side: max(0.0, now_s - float(states[side].timestamp_s))  # type: ignore[union-attr]
            for side in states
        }
        stale = [side for side, age in ages.items() if age > age_limit]
        if stale:
            detail = ", ".join(f"{side}={ages[side]:.2f}s" for side in stale)
            raise RuntimeError(f"Telemetry geometry requires GNSS newer than {age_limit:.2f}s; stale {detail}")

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
            root = self._accumulated_drone_rtk_multiview_root()
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
        candidate = solve_drone_multiview_candidate(
            profile=profile,
            priors=priors,
            root=copy.deepcopy(root),
            source_file_sha256=source_file_sha256,
            live_geometry=live_geometry,
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
                raise ValueError("Drone RTK yaw JSON must contain a left/right observation object map")
        candidate, rotation_context = solve_drone_yaw_candidate(
            profile=profile,
            priors=priors,
            root=copy.deepcopy(root),
            drone_file_sha256=drone_file_sha256,
            annotations=copy.deepcopy(annotations),
            artifacts={side: row["artifact"] for side, row in self._frozen_frames.items()},
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
            hashes: tuple[tuple[str, str], ...] = (("checkerboard_extrinsics_profile_sha256", profile.content_sha256),)
            warnings: tuple[str, ...] = ("packaged from the stored stereoCalibrate profile, not re-measured here",)
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
            hashes = (("rtk_baseline_fix_quality", baseline.fix_quality),)
            warnings = ("translation only; live RTK baseline is not anchored to a re-derived rotation",)
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
            raise ValueError("combined pose requires at least two explicitly selected compatible candidate IDs")
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
            side_observable = {side: dict(values) for side, values in candidate.observability_by_side}
            unobservable = tuple(
                name
                for name in selected_names
                if not all(side_observable[side].get(name, False) for side in ("left", "right"))
            )
            if unobservable:
                raise ValueError(
                    f"candidate {candidate.candidate_id} axes are not observable on both sides: "
                    + ", ".join(unobservable)
                )
            overlap = used.intersection(selected_names)
            if overlap:
                raise ValueError("combined candidates optimize overlapping DOF: " + ", ".join(sorted(overlap)))
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
            dict(candidate.input_hashes).get("operating_intrinsics_profile_sha256", "") for candidate in candidates
        }
        profile_hashes.discard("")
        if len(profile_hashes) > 1:
            raise ValueError("combined candidates were solved with different operating K,D profiles")
        quality: list[tuple[str, float]] = []
        for index, candidate in enumerate(candidates):
            quality.extend((f"source_{index}_{key}", value) for key, value in candidate.quality_metrics)
        hashes = [
            (f"source_candidate_{index}", _payload_sha256(_candidate_mapping(candidate)))
            for index, candidate in enumerate(candidates)
        ]
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

    def workstation_optical_pose_snapshot(self, *, include_frames=True):
        return optical_pose_snapshot(
            self.capture_owner,
            self._candidate_registry,
            self._review,
            intrinsics=self._intrinsics_snapshot(use_frozen=False),
            actual_zoom=getattr(self.panel, "actual_visible_zoom", {}) or {},
            selected_extrinsics=getattr(
                self.panel, "active_extrinsics_profile", getattr(self.panel, "selected_extrinsics_profile", None)
            ),
            include_frames=include_frames,
        )

    _zoom = staticmethod(zoom)

    @staticmethod
    def _pose_from_mapping(value: Mapping[str, Any], *, name: str) -> CameraPoseNed:
        rotation = value.get("rotation_ned_from_camera_rdf")
        center = value.get("center_ned_m")
        if rotation is not None and center is not None:
            return CameraPoseNed(tuple(tuple(row) for row in rotation), tuple(center))
        rpy = value.get("virtual_frd_rpy_deg")
        if rpy is None or center is None:
            raise ValueError(f"{name} requires rotation_ned_from_camera_rdf or virtual_frd_rpy_deg and center_ned_m")
        parsed_rpy = _finite_vector(rpy, length=3, name=f"{name}.virtual_frd_rpy_deg")
        return CameraPoseNed.from_virtual_frd_rpy_deg(
            center_ned_m=center,
            roll_deg=parsed_rpy[0],
            pitch_deg=parsed_rpy[1],
            yaw_deg=parsed_rpy[2],
        )

    def _frozen_or_live_extrinsics_profile(self, *, prefer_frozen: bool) -> Any:
        """Read during owner-side input capture; workers never call this method."""
        return getattr(
            self.panel, "active_extrinsics_profile", getattr(self.panel, "selected_extrinsics_profile", None)
        )

    def _frozen_or_live_nominal_poses(self, *, prefer_frozen: bool) -> Any:
        return getattr(self.panel, "optical_pose_nominal_poses", None)

    def _frozen_or_live_active_intrinsics(self, *, prefer_frozen: bool) -> Any:
        return getattr(self.panel, "active_intrinsics", None)

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
        if prefer_frozen and self.capture_owner.frozen_inputs is not None:
            raise RuntimeError(
                "an explicit prior pose or selected base stereo R,t is required; "
                "no prior pose was available when this burst was frozen"
            )
        if self._active_poses is not None:
            return self._active_poses
        declared = self._frozen_or_live_nominal_poses(prefer_frozen=prefer_frozen)
        if (
            isinstance(declared, tuple)
            and len(declared) == 2
            and all(isinstance(pose, CameraPoseNed) for pose in declared)
        ):
            return declared
        profile = self._frozen_or_live_extrinsics_profile(prefer_frozen=prefer_frozen)
        extrinsics = getattr(profile, "extrinsics", None)
        if extrinsics is None:
            active = self._frozen_or_live_active_intrinsics(prefer_frozen=prefer_frozen)
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
            tuple(float(value) for value in row) for row in extrinsics.rotation_left_to_right
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

    _candidate_mapping_with_prior = staticmethod(candidate_mapping_with_prior)

    def _zoom_mapping(self, side: str) -> dict[str, Any]:
        values = getattr(self.panel, "actual_visible_zoom", {}) or {}
        row = values.get(side, {}) if isinstance(values, Mapping) else {}
        return dict(row) if isinstance(row, Mapping) else {}

    def _declared_profile_value(self, side: str, kind: str, *, use_frozen=False) -> str | None:
        inputs = self.capture_owner.frozen_inputs if use_frozen else None
        identities = (
            inputs.evidence("declared_identity", {})
            if inputs is not None
            else getattr(self.panel, "operating_intrinsics_identity", {}) or {}
        )
        row = identities.get(side, {}) if isinstance(identities, Mapping) else {}
        if isinstance(row, Mapping):
            value = row.get(kind)
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
            return self._nominal_operating_intrinsics_profile(use_frozen=False)
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

    def _nominal_operating_intrinsics_profile(self, *, use_frozen: bool) -> OperatingIntrinsicsProfile | None:
        left = self._camera_evidence("left", use_frozen=use_frozen)
        right = self._camera_evidence("right", use_frozen=use_frozen)
        if (
            left.zoom_index is None
            or right.zoom_index is None
            or left.zoom_index != right.zoom_index
            or left.frame_width_px is None
            or left.frame_height_px is None
            or right.frame_width_px is None
            or right.frame_height_px is None
        ):
            return None
        return build_nominal_operating_intrinsics_profile(
            zoom_index=left.zoom_index,
            left_width_px=left.frame_width_px,
            left_height_px=left.frame_height_px,
            right_width_px=right.frame_width_px,
            right_height_px=right.frame_height_px,
            left_zoom_ratio=left.zoom_ratio,
            right_zoom_ratio=right.zoom_ratio,
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
        inputs = self.capture_owner.frozen_inputs if use_frozen else None
        identity = (
            inputs.evidence("camera_identity", {})
            if inputs is not None
            else getattr(self.panel, "camera_identity_evidence", {}) or {}
        )
        identity_row = identity.get(side, {}) if isinstance(identity, Mapping) else {}
        identity_row = identity_row if isinstance(identity_row, Mapping) else {}
        serial = (
            self._metadata_text(
                metadata,
                "camera_serial",
                "serial",
                "device_serial",
            )
            or self._metadata_text(identity_row, "camera_serial", "serial")
            or self._declared_profile_value(side, "serial", use_frozen=use_frozen)
        )
        stabilization = (
            self._metadata_text(
                metadata,
                "stabilization_state",
                "stabilization",
            )
            or self._metadata_text(identity_row, "stabilization_state", "stabilization")
            or self._declared_profile_value(side, "stabilization_state", use_frozen=use_frozen)
        )
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
        profile = self._frozen_intrinsics_profile if use_frozen else self._operating_intrinsics_profile()
        left = self._camera_evidence("left", use_frozen=use_frozen)
        right = self._camera_evidence("right", use_frozen=use_frozen)
        now = (
            self._frozen_capture_monotonic_s
            if use_frozen and self._frozen_capture_monotonic_s is not None
            else time.monotonic()
        )
        inputs = self.capture_owner.frozen_inputs if use_frozen else None
        source_zoom = (
            inputs.evidence("extrinsics_source_zoom_index")
            if inputs is not None
            else getattr(self._frozen_or_live_extrinsics_profile(prefer_frozen=use_frozen), "source_zoom_level", None)
        )
        result = evaluate_intrinsics_applicability(
            profile,
            left,
            right,
            now_monotonic_s=now,
            extrinsics_source_zoom_index=source_zoom,
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

    def operating_intrinsics_profile_for_geometry(self) -> OperatingIntrinsicsProfile | None:
        snapshot = self._intrinsics_snapshot(use_frozen=False)
        if not bool(snapshot["applicability"].get("geometry_allowed")):
            return None
        profile = snapshot.get("profile")
        return OperatingIntrinsicsProfile.from_mapping(profile) if isinstance(profile, Mapping) else None

    def _require_frozen_intrinsics(
        self,
    ) -> tuple[OperatingIntrinsicsProfile, Mapping[str, Any]]:
        missing = [side for side in ("left", "right") if side not in self._frozen_frames]
        if missing:
            raise RuntimeError("capture a frozen native left/right burst before solving; missing " + ", ".join(missing))
        snapshot = self._intrinsics_snapshot(use_frozen=True)
        applicability = snapshot["applicability"]
        if not applicability["geometry_allowed"]:
            raise RuntimeError(
                "frozen burst lacks applicable exact operating K,D: " + ", ".join(applicability["reasons"])
            )
        assert self._frozen_intrinsics_profile is not None
        return self._frozen_intrinsics_profile, applicability

    _annotation_root = staticmethod(OpticalReviewState._annotation_root)

    def _validate_annotation_identity(self, side: str, annotation: Mapping[str, Any]) -> None:
        frozen = self._frozen_frames.get(side)
        validate_annotation_identity(None if frozen is None else frozen["artifact"], side, annotation)

    _annotation_points = staticmethod(annotation_points)

    @staticmethod
    def _validate_mountain_point_distribution(
        points: Sequence[tuple[float, float]],
        *,
        side: str,
    ) -> None:
        """Fail closed for duplicate or nearly one-dimensional Mountain clicks."""

        validate_mountain_point_distribution(points, side=side)

    _sample_horizon_pixels = staticmethod(sample_horizon_pixels)

    _undistort_pixels = staticmethod(undistort_pixels)

    _PROFILE_ATTRIBUTE_NAMES = (
        "selected_extrinsics_profile",
        "active_extrinsics_profile",
        "optical_pose_extrinsics_profile",
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
        "_active_alignment_snapshot_path",
        "_active_alignment_snapshot_sha256",
    )
    _MEASUREMENT_CACHE_ATTRIBUTE_NAMES = (
        "_last_measurement_attempt_key",
        "_last_measurement_attempt",
        "_last_distance_history_key",
    )

    def _active_intrinsics_for_apply(self, intrinsics_snapshot):
        return active_intrinsics_for_apply(
            intrinsics_snapshot,
            current=getattr(self.panel, "active_intrinsics", None),
            active_path=str(getattr(self.panel, "_active_intrinsics_profile_path", "") or ""),
            active_hash=str(getattr(self.panel, "_active_intrinsics_profile_hash", "") or ""),
        )

    def _capture_profile_state(self) -> dict[str, Any]:
        return {
            "attributes": {
                name: (hasattr(self.panel, name), getattr(self.panel, name, None))
                for name in self._PROFILE_ATTRIBUTE_NAMES
            },
            "active_selection": self._active_selection,
            "active_poses": self._active_poses,
            "selection_history": tuple(self._selection_history),
            "measurement_cache": {
                name: (hasattr(self.panel, name), getattr(self.panel, name, None))
                for name in self._MEASUREMENT_CACHE_ATTRIBUTE_NAMES
            },
        }

    def _restore_profile_state(self, state: Mapping[str, Any]) -> None:
        attributes = state.get("attributes")
        if isinstance(attributes, Mapping):
            restore_geometry_values(self.panel, attributes)
        self._candidate_registry.restore_selection(state)
        cache = state.get("measurement_cache")
        if isinstance(cache, Mapping):
            for name, row in cache.items():
                existed, value = row
                if existed:
                    setattr(self.panel, name, value)
                elif hasattr(self.panel, name):
                    delattr(self.panel, name)

    def _prepare_selected_profile(
        self, selection, candidate, poses, intrinsics_snapshot, *, current_intrinsics, operating_profile
    ):
        return prepare_selected_profile(
            selection,
            candidate,
            poses,
            intrinsics_snapshot,
            current_intrinsics=current_intrinsics,
            operating_profile=operating_profile,
            previous=getattr(
                self.panel, "active_extrinsics_profile", getattr(self.panel, "selected_extrinsics_profile", None)
            ),
        )

    def _commit_selected_profile(self, prepared):
        apply_geometry_values(self.panel, profile_geometry_values(prepared))
        self._candidate_registry.select(prepared.selection, prepared.poses)

    _selection_mapping = staticmethod(selection_mapping)

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
