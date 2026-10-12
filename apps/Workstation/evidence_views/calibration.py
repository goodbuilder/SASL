"""Pure calibration quality/profile/capture and optical RDF projections.

Inputs contain selected values, never live profile objects. Intrinsics use
pixels; X_right = R_right_from_left X_left + T, right origin = -R.T @ T.
"""

import math
from collections.abc import Mapping
from dataclasses import dataclass

from Workstation.workstation_core.calibration import independent_zoom_selection_mode
from Workstation.workstation_core.coordinate_contract import (
    BASELINE_TRANSLATION_FRAME,
    CAMERA_FRAME,
    MEASUREMENT_AXIS_CONTRACT,
    MEASUREMENT_FRAME,
)

from .snapshot import snapshot_mapping, thaw_snapshot
from .values import _integer, _number

CAMERA_FIELDS = ("width", "height", "reprojection_error_px", "image_count", "distortion_coefficients")
EXTRINSICS_FIELDS = (
    "translation_left_to_right_m",
    "rotation_left_to_right",
    "pair_count",
    "reprojection_error_px",
    "frame_contract",
)
PROFILE_FIELDS = ("source_zoom_level", "profile_id", "method", "metadata")


@dataclass(frozen=True)
class CalibrationInputs:
    geometry: Mapping
    profiles: Mapping
    capture: Mapping
    display: Mapping
    validation: Mapping

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            object.__setattr__(self, name, snapshot_mapping(getattr(self, name)))

    def project(self):
        return thaw_snapshot(
            _project(
                geometry=self.geometry,
                profiles=self.profiles,
                capture=self.capture,
                display=self.display,
                validation=self.validation,
            )
        )


def project_camera_intrinsics(camera: object | None) -> dict[str, object]:
    if camera is None:
        return {"state": "NO DATA"}
    return {
        "state": "AVAILABLE",
        "width_px": _integer(camera.get("width", None)),
        "height_px": _integer(camera.get("height", None)),
        "fx_px": camera.get("fx_px"),
        "fy_px": camera.get("fy_px"),
        "cx_px": camera.get("cx_px"),
        "cy_px": camera.get("cy_px"),
        "reprojection_error_px": _number(camera.get("reprojection_error_px", None)),
        "image_count": _integer(camera.get("image_count", None)),
        "distortion_coefficients": tuple(camera.get("distortion_coefficients", ()) or ()),
        "source": "StereoIntrinsicsResult / CameraIntrinsics",
        "frame_contract": f"{CAMERA_FRAME}; camera matrix units are pixels",
    }


def _project(*, geometry, profiles, capture, display, validation):
    active = bool(geometry.get("active"))
    candidate_available = bool(geometry.get("candidate_available"))
    result_available = bool(geometry.get("result_available"))
    profile = profiles.get("selected")
    calculation_active = bool(display.get("calculation_active"))
    calibration_status = display.get("calibration_status", "")
    calculation_status = display.get("calculation_status", "")
    status_label = calculation_status if calculation_active and calculation_status else calibration_status
    active_profile_path = str(profiles.get("active_intrinsics_path", "") or "")
    candidate_profile_path = str(profiles.get("candidate_intrinsics_path", "") or "")
    profile_path = active_profile_path if active else candidate_profile_path
    if not profile_path and result_available:
        profile_path = "memory/active-calibration" if active else "memory/calibration-candidate"
    zoom = (
        _integer(profiles.get("active_intrinsics_zoom", None))
        if active
        else _integer(profiles.get("candidate_intrinsics_zoom", None))
    )
    if zoom is None:
        zoom = _integer(display.get("zoom"))
    extrinsics = geometry.get("extrinsics")
    extrinsics_payload: dict[str, object] = {}
    if extrinsics is not None:
        translation = tuple((extrinsics or {}).get("translation_left_to_right_m", ()) or ())
        rotation = tuple(tuple(row) for row in (extrinsics or {}).get("rotation_left_to_right", ()) or ())
        right_origin = None
        if len(rotation) == 3 and all(len(row) == 3 for row in rotation) and (len(translation) == 3):
            right_origin = tuple(
                -sum(float(rotation[column][row]) * float(translation[column]) for column in range(3))
                for row in range(3)
            )
        extrinsics_payload = {
            "state": "AVAILABLE",
            "pair_count": _integer((extrinsics or {}).get("pair_count", None)),
            "reprojection_error_px": _number((extrinsics or {}).get("reprojection_error_px", None)),
            "rotation_left_to_right": rotation,
            "translation_left_to_right_m": translation,
            "right_camera_origin_in_left_rdf_m": right_origin,
            "frame_contract": str((extrinsics or {}).get("frame_contract", "") or f"{CAMERA_FRAME}; left→right R/T"),
            "baseline_m": sum(float(value) * float(value) for value in translation) ** 0.5 if translation else None,
        }
    active_intrinsics_zoom = _integer(profiles.get("active_intrinsics_zoom", None))
    if active_intrinsics_zoom is None:
        active_intrinsics_zoom = zoom
    active_extrinsics_zoom = _integer(profiles.get("active_extrinsics_zoom", None))
    selected_extrinsics_zoom = _integer((profile or {}).get("source_zoom_level", None))
    extrinsics_source_zoom = (
        active_extrinsics_zoom if active and active_extrinsics_zoom is not None else selected_extrinsics_zoom
    )
    extrinsics_profile_path = str(
        profiles.get("active_extrinsics_path", "") if active else profiles.get("selected_extrinsics_path", "") or ""
    )
    if not extrinsics_profile_path and profile is not None:
        profile_id = str((profile or {}).get("profile_id", "") or "stereo_extrinsics")
        extrinsics_profile_path = f"memory:{profile_id}"
    extrinsics_method = str((profile or {}).get("method", "") or "")
    extrinsics_profile_id = str((profile or {}).get("profile_id", "") or "")
    extrinsics_metadata = dict((profile or {}).get("metadata", {}) or {})
    cross_zoom = (
        active_intrinsics_zoom is not None
        and extrinsics_source_zoom is not None
        and (active_intrinsics_zoom != extrinsics_source_zoom)
    )
    selection_mode = independent_zoom_selection_mode(active_intrinsics_zoom, extrinsics_source_zoom)
    selection_label = {
        "manual_cross_zoom": "cross-zoom/manual",
        "manual_same_zoom": "same-zoom/manual",
        "manual_independent_zoom": "independent/manual; source zoom unknown",
    }[selection_mode]
    combination = (
        f"K,D zoom {active_intrinsics_zoom} + R,t zoom {extrinsics_source_zoom} ({selection_label})"
        if extrinsics is not None
        else f"K,D zoom {active_intrinsics_zoom}; R,t not selected"
    )
    actual_zoom = display.get("actual_zoom", {}) or {}
    actual_parts: list[str] = []
    for side in ("left", "right"):
        state = actual_zoom.get(side, {}) if isinstance(actual_zoom, Mapping) else {}
        index = state.get("index") if isinstance(state, Mapping) else None
        ratio = _number(state.get("ratio")) if isinstance(state, Mapping) else None
        if index is None:
            readback = "unknown"
        elif ratio is not None and math.isfinite(ratio) and (ratio > 0.0):
            readback = f"{index} ({ratio:g}×)"
        else:
            readback = str(index)
        actual_parts.append(f"{side}={readback}")
    zoom_readback = ", ".join(actual_parts)
    auto_capture_active = bool(capture.get("active"))
    auto_capture_saved_pairs = _integer(capture.get("saved_pairs"))
    valid_pair_count = extrinsics_payload.get("pair_count")
    if auto_capture_saved_pairs is not None and (auto_capture_active or auto_capture_saved_pairs > 0):
        valid_pair_count = auto_capture_saved_pairs
    selected_validation_mode = str(validation.get("selected_mode", "pixhawk") or "pixhawk")
    result_validation_mode = str(validation.get("result_mode", "") or "")
    validation_mode = (
        result_validation_mode if result_available and result_validation_mode else selected_validation_mode
    )
    validation_run_results = validation.get("runs", ()) or ()
    if not isinstance(validation_run_results, (tuple, list)):
        validation_run_results = ()
    raw_progress = display.get("progress", {}) or {}
    if isinstance(raw_progress, Mapping):
        progress = dict(raw_progress)
    else:
        progress = {}
    try:
        progress_percent = float(progress.get("percent", 0.0))
    except (TypeError, ValueError):
        progress_percent = 0.0
    progress_text = str(progress.get("message") or progress.get("phase") or "Idle")
    if progress:
        progress_text = f"{progress_text} ({max(0.0, min(100.0, progress_percent)):.0f}%)"
    calibration_quality = validation.get("quality", None)
    if not isinstance(calibration_quality, Mapping) and result_available:
        calibration_quality = validation.get("result_quality")
    calibration_quality = (
        dict(calibration_quality)
        if isinstance(calibration_quality, Mapping)
        else {"state": "UNVERIFIED", "reason_codes": ["quality_snapshot_unavailable"]}
    )
    capture_status = capture.get("status", {})
    return {
        "state": "CALCULATING"
        if calculation_active
        else "ACTIVE"
        if active
        else "AVAILABLE"
        if result_available
        else "NO DATA",
        "active": active,
        "calculation_active": calculation_active,
        "profile_path": profile_path,
        "active_profile_path": active_profile_path,
        "candidate_profile_path": candidate_profile_path,
        "candidate_available": candidate_available,
        "extrinsics_candidate_profile_id": str(profiles.get("candidate_id", "") or ""),
        "intrinsics_profile_hash": str(profiles.get("intrinsics_hash", "") or ""),
        "zoom": zoom,
        "intrinsics_source_zoom": active_intrinsics_zoom,
        "extrinsics_profile_path": extrinsics_profile_path,
        "extrinsics_profile_id": extrinsics_profile_id,
        "extrinsics_method": extrinsics_method,
        "extrinsics_metadata": extrinsics_metadata,
        "extrinsics_profile_hash": str(
            profiles.get("active_extrinsics_hash", "") if active else profiles.get("selected_extrinsics_hash", "") or ""
        ),
        "extrinsics_source_zoom": extrinsics_source_zoom,
        "combination": combination,
        "cross_zoom": cross_zoom,
        "selection_mode": str(
            profiles.get("selection_mode", "") or ("manual_candidate" if extrinsics is not None else "intrinsics_only")
        ),
        "validation_gate": "disabled",
        "quality_gate": "calibration_quality_v1",
        "calibration_quality": calibration_quality,
        "capture_status": capture_status,
        "zoom_readback": zoom_readback,
        "status_label": status_label,
        "progress": progress_text,
        "valid_pair_count": valid_pair_count,
        "auto_capture_active": auto_capture_active,
        "auto_capture_saved_pairs": auto_capture_saved_pairs,
        "source": "checkerboard calculation worker + live auto-capture controller + active_intrinsics/last_calibration_result",
        "left": project_camera_intrinsics(geometry.get("left")),
        "right": project_camera_intrinsics(geometry.get("right")),
        "extrinsics": extrinsics_payload,
        "validation_mode": validation_mode,
        "validation_summary": dict(validation.get("summary", {}) or {}),
        "validation_run_results": tuple(dict(row) for row in validation_run_results if isinstance(row, Mapping)),
        "frame_contract": f"{CAMERA_FRAME}; calibration intrinsics/extrinsics stay in OpenCV optical RDF",
        "measurement_contract": f"public output {MEASUREMENT_FRAME} [{MEASUREMENT_AXIS_CONTRACT}]; baseline {BASELINE_TRANSLATION_FRAME}",
    }
