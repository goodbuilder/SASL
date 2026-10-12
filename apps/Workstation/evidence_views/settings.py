"""Pure display normalization of captured measurement/calibration controls.

Inputs are resolved widget/settings values. Pixel, second, millisecond and
optical RDF/frame-source labels retain the existing display contract.
"""

from Workstation.workstation_core.coordinate_contract import CAMERA_FRAME, MEASUREMENT_AXIS_CONTRACT, MEASUREMENT_FRAME
from Workstation.workstation_core.settings import normalize_live_distance_time_basis

from .values import _integer, _number


def project_detection_config(controls):
    weights = controls["weights"]
    device = controls["device"]
    device = str(device or "cpu")
    gate_names = []
    gate_values = {
        "class": controls["class"],
        "time": controls["time"],
        "Δt": controls["Δt"],
        "vertical Δ": controls["vertical Δ"],
    }
    gate_names.extend(name for (name, enabled) in gate_values.items() if enabled)
    return {
        "weights_path": weights,
        "device": device,
        "input_size_px": _integer(controls["input_size_px"]),
        "confidence": _number(controls["confidence"]),
        "iou": _number(controls["iou"]),
        "class_gate": controls["class_gate"],
        "gate_enabled": tuple(gate_names),
        "max_delta_s": min(0.1, max(0.0, _number(controls["max_delta_s"], 0.1))),
        "time_basis": normalize_live_distance_time_basis(controls["time_basis"]),
        "time_basis_role": "preferred_with_app_receive_fallback",
        "ntp_evidence_role": "preferred_when_selected_and_verified",
        "max_vertical_delta_px": _number(controls["max_vertical_delta_px"]),
        "source": "AppSettings + existing YOLO configuration widgets",
    }


def project_measurement_settings(controls):
    intrinsics_source = controls["intrinsics_source"]
    distance_mode = controls["distance_mode"]
    return {
        "intrinsics_source": str(intrinsics_source or "checkerboard"),
        "manual_focal_override": controls["manual_focal_override"],
        "focal_px": _number(controls["focal_px"], 900.0),
        "distance_mode": str(distance_mode or "pixhawk"),
        "measurement_preferred_method": str(controls["measurement_preferred_method"]),
        "measurement_method_fallback_order": list(controls["measurement_method_fallback_order"] or ()),
        "measurement_baseline_yaw_reverse": bool(controls["measurement_baseline_yaw_reverse"]),
        "pixhawk_left_gnss_source": str(controls["pixhawk_left_gnss_source"]),
        "pixhawk_right_gnss_source": str(controls["pixhawk_right_gnss_source"]),
        "yolo_input_size_px": _integer(controls["yolo_input_size_px"]),
        "yolo_confidence": _number(controls["yolo_confidence"]),
        "yolo_iou": _number(controls["yolo_iou"]),
        "class_gate_enabled": controls["class_gate_enabled"],
        "class_gate": controls["class_gate"],
        "matching_gate_enabled": True,
        "time_basis": normalize_live_distance_time_basis(controls["time_basis"] or "app_receive_monotonic"),
        "time_basis_role": "preferred_with_app_receive_fallback",
        "ntp_evidence_role": "preferred_when_selected_and_verified",
        "time_delta_gate_enabled": True,
        "max_pair_delta_s": min(0.1, max(0.0, _number(controls["max_pair_delta_s"], 0.1))),
        "vertical_delta_gate_enabled": controls["vertical_delta_gate_enabled"],
        "max_vertical_delta_px": _number(controls["max_vertical_delta_px"]),
        "camera_sync_history_window_s": _number(controls["camera_sync_history_window_s"]),
        "camera_sync_receive_pair_max_delta_ms": _number(controls["camera_sync_receive_pair_max_delta_ms"]),
        "camera_sync_rtcp_pair_max_delta_ms": _number(controls["camera_sync_rtcp_pair_max_delta_ms"]),
        "camera_sync_max_sr_age_s": _number(controls["camera_sync_max_sr_age_s"]),
        "camera_sync_max_pc_offset_ms": _number(controls["camera_sync_max_pc_offset_ms"]),
        "camera_sync_max_pc_offset_role": "diagnostic-only",
        "camera_sync_clock_rate_error_pct": _number(controls["camera_sync_clock_rate_error_pct"]),
        "camera_sync_required_sr_count": _integer(controls["camera_sync_required_sr_count"]),
        "camera_sync_warmup_timeout_s": _number(controls["camera_sync_warmup_timeout_s"]),
        "camera_sync_mode": str(controls["camera_sync_mode"] or "manual"),
        "camera_sync_interval_s": _number(controls["camera_sync_interval_s"]) * 60.0,
        "source": "existing Measurement/Time-Sync owner widgets + AppSettings",
        "coordinate_contract": f"intrinsics/extrinsics source selectors; public output {MEASUREMENT_FRAME} [{MEASUREMENT_AXIS_CONTRACT}]",
    }


def project_calibration_setup(controls):
    return {
        "zoom_level": _integer(controls["zoom_level"]),
        "inner_cols": _integer(controls["inner_cols"]),
        "inner_rows": _integer(controls["inner_rows"]),
        "square_size_mm": _number(controls["square_size_mm"]),
        "min_images": _integer(controls["min_images"]),
        "calculation_pair_count": _integer(controls["calculation_pair_count"]),
        "pair_max_delta_ms": _number(controls["pair_max_delta_ms"]),
        "auto_interval_s": _number(controls["auto_interval_s"]),
        "calibration_root": controls["calibration_root"],
        "calibration_root_resolved": controls["calibration_root_resolved"],
        "validation_mode": str(controls["validation_mode"] or "pixhawk"),
        "validation_root": controls["validation_root"],
        "validation_json": controls["validation_json"],
        "validation_runs": _integer(controls["validation_runs"]),
        "validation_pair_count": _integer(controls["validation_pair_count"]),
        "validation_seed": _integer(controls["validation_seed"]),
        "source": "existing calibration setup widgets",
        "frame_contract": f"checkerboard image pixels/{CAMERA_FRAME}; stereo R/T remains left-camera optical RDF → right-camera optical RDF",
    }
