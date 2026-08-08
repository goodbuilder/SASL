from __future__ import annotations

"""Read-only projections of the existing StereoWIRIS panel state.

The workstation is a presentation shell.  This module deliberately keeps the
projection boundary boring: it reads existing model objects and widgets,
retains their timestamp/frame contracts, and never performs a measurement,
coordinate conversion, save, or calibration operation.
"""

import time
import csv
import json
from pathlib import Path
from typing import Any

from wiris_stereo_distance.coordinate_contract import (
    BASELINE_TRANSLATION_FRAME,
    BODY_FRAME,
    CAMERA_FRAME,
    MEASUREMENT_AXIS_CONTRACT,
    MEASUREMENT_FRAME,
    WORLD_FRAME,
)


def _widget_text(widget: object | None, default: str = "") -> str:
    method = getattr(widget, "text", None)
    if callable(method):
        try:
            return str(method())
        except Exception:
            return default
    return default


def _widget_value(widget: object | None, default: object = None) -> object:
    method = getattr(widget, "value", None)
    if callable(method):
        try:
            return method()
        except Exception:
            return default
    return default


def _widget_checked(widget: object | None, default: bool = False) -> bool:
    method = getattr(widget, "isChecked", None)
    if callable(method):
        try:
            return bool(method())
        except Exception:
            return default
    return default


def _combo_data(widget: object | None, default: object = None) -> object:
    method = getattr(widget, "currentData", None)
    if callable(method):
        try:
            return method()
        except Exception:
            return default
    return default


def _number(value: object, default: float | None = None) -> float | None:
    if value in (None, ""):
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _integer(value: object, default: int | None = None) -> int | None:
    if value in (None, ""):
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _age(timestamp_s: object, now_s: float) -> float | None:
    value = _number(timestamp_s)
    if value is None:
        return None
    return max(0.0, now_s - value)


def _safe_len(value: object) -> int:
    try:
        return len(value)  # type: ignore[arg-type]
    except (TypeError, AttributeError):
        return 0


def _matrix_value(matrix: object, row: int, column: int) -> float | None:
    try:
        return float(matrix[row][column])  # type: ignore[index]
    except (IndexError, KeyError, TypeError, ValueError):
        return None


class EvidenceAdapter:
    """Expose source-backed, UI-safe snapshots for one panel owner."""

    def __init__(self, panel: object) -> None:
        self.panel = panel

    def _app_root(self) -> Path:
        """Return the data root owned by this panel's application runtime."""

        configured = getattr(self.panel, "app_root", None)
        if configured not in (None, ""):
            return Path(configured)
        # Compatibility panels created by older tests and the Legacy adapter
        # retain the original StereoWIRIS root.
        from wiris_stereo_distance.settings import APP_ROOT

        return Path(APP_ROOT)

    def attitude(self, side: str, *, now_s: float | None = None) -> dict[str, object]:
        now = time.monotonic() if now_s is None else float(now_s)
        states = getattr(self.panel, "latest_extrinsics", {}) or {}
        state = states.get(side) if isinstance(states, dict) else None
        if state is None:
            return {
                "side": side,
                "state": "WAITING",
                "age_s": None,
                "frame_contract": "PX4 vehicle attitude quaternion rotates body FRD to world NED",
                "source": "DualPixhawkAttitudeSubscriber/MAVLink extrinsic_changed",
            }
        return {
            "side": side,
            "state": "LIVE",
            "age_s": _age(getattr(state, "timestamp_s", None), now),
            "roll_deg": _number(getattr(state, "roll_deg", None)),
            "pitch_deg": _number(getattr(state, "pitch_deg", None)),
            "yaw_deg": _number(getattr(state, "yaw_deg", None)),
            "q_wxyz": tuple(getattr(state, "q", ()) or ()),
            "namespace": str(getattr(state, "namespace", "") or ""),
            "topic": str(getattr(state, "topic", "") or ""),
            "app_timestamp_s": _number(getattr(state, "timestamp_s", None)),
            "px4_timestamp_s": _number(getattr(state, "px4_timestamp_s", None)),
            "px4_timestamp_us": _integer(getattr(state, "px4_timestamp_us", None)),
            "px4_timestamp_sample_us": _integer(getattr(state, "px4_timestamp_sample_us", None)),
            "source": "CameraExtrinsicState / attitude subscriber",
            "clock": "app receive monotonic; PX4 timestamp when present",
            "frame_contract": "PX4 vehicle attitude quaternion rotates body FRD to world NED",
            "orientation_contract": f"orientation: {BODY_FRAME} → {WORLD_FRAME}; roll/pitch/yaw are degrees",
        }

    def gps(self, side: str, *, now_s: float | None = None) -> dict[str, object]:
        now = time.monotonic() if now_s is None else float(now_s)
        states = getattr(self.panel, "latest_gps", {}) or {}
        state = states.get(side) if isinstance(states, dict) else None
        if state is None:
            return {
                "side": side,
                "state": "NO DATA",
                "source": "GpsPositionState / GPS receiver",
                "position_contract": "WGS84 latitude/longitude degrees; altitude metres",
            }
        fix_type = _integer(getattr(state, "fix_type", None))
        rtk_label = str(getattr(state, "rtk_label", "UNKNOWN") or "UNKNOWN")
        return {
            "side": side,
            "state": rtk_label,
            "rtk_label": rtk_label,
            "fix_type": fix_type,
            "satellites": _integer(getattr(state, "satellites_used", None)),
            "eph_m": _number(getattr(state, "eph_m", None)),
            "epv_m": _number(getattr(state, "epv_m", None)),
            "latitude_deg": _number(getattr(state, "latitude_deg", None)),
            "longitude_deg": _number(getattr(state, "longitude_deg", None)),
            "altitude_m": _number(getattr(state, "altitude_m", None)),
            "altitude_ellipsoid_m": _number(getattr(state, "altitude_ellipsoid_m", None)),
            "heading_deg": _number(getattr(state, "heading_deg", None)),
            "timestamp_s": _number(getattr(state, "timestamp_s", None)),
            "age_s": _age(getattr(state, "timestamp_s", None), now),
            "px4_timestamp_s": _number(getattr(state, "px4_timestamp_s", None)),
            "px4_timestamp_us": _integer(getattr(state, "px4_timestamp_us", None)),
            "px4_timestamp_sample_us": _integer(getattr(state, "px4_timestamp_sample_us", None)),
            "gps_utc_us": _integer(getattr(state, "gps_utc_us", None)),
            "namespace": str(getattr(state, "namespace", "") or ""),
            "topic": str(getattr(state, "topic", "") or ""),
            "time_source": str(getattr(state, "time_source", "") or ""),
            "source": "GpsPositionState / GPS receiver",
            "clock": "GPS app receive monotonic; PX4/GPS source timestamps retained",
            "position_contract": "WGS84 latitude/longitude degrees; altitude metres",
        }

    def baseline(self) -> dict[str, object]:
        baseline = getattr(self.panel, "latest_rtk_baseline", None)
        if baseline is None:
            return {
                "state": "NO DATA",
                "source": "RtkBaselineEstimate from left/right GpsPositionState",
                "axis_contract": "NED = north / east / down; metres",
            }
        return {
            "state": str(getattr(baseline, "fix_quality", "UNKNOWN") or "UNKNOWN"),
            "north_m": _number(getattr(baseline, "north_m", None)),
            "east_m": _number(getattr(baseline, "east_m", None)),
            "down_m": _number(getattr(baseline, "down_m", None)),
            "length_m": _number(getattr(baseline, "baseline_m", None)),
            "left_age_s": _number(getattr(baseline, "left_age_s", None)),
            "right_age_s": _number(getattr(baseline, "right_age_s", None)),
            "warnings": tuple(getattr(baseline, "warnings", ()) or ()),
            "source": "RtkBaselineEstimate from left/right GpsPositionState",
            "axis_contract": "NED = north / east / down; metres",
            "position_frame": "GPS/RTK WGS84 delta projected to local NED; not camera or FRD",
        }

    def detection(self, side: str, *, now_s: float | None = None) -> dict[str, object]:
        now = time.monotonic() if now_s is None else float(now_s)
        observation = getattr(self.panel, f"latest_{side}", None)
        stats_all = getattr(self.panel, "camera_stream_stats", {}) or {}
        stats = stats_all.get(side, {}) if isinstance(stats_all, dict) else {}
        stats = stats if isinstance(stats, dict) else {}
        result: dict[str, object] = {
            "side": side,
            "runtime_state": "TARGET PRESENT" if observation is not None else "NO TARGET",
            "enabled": bool(getattr(self.panel, "yolo_enabled", False)),
            "decoded_hz": _number(stats.get("decoded_hz")),
            "inferred_hz": _number(stats.get("inferred_hz")),
            "displayed_hz": _number(stats.get("displayed_hz")),
            "actual_device": str(stats.get("yolo_actual_device", "") or ""),
            "requested_device": str(stats.get("yolo_requested_device", "") or ""),
            "cpu_fallback": bool(stats.get("yolo_cpu_fallback", False)),
            "source": "DetectionObservation / YoloStreamWorker stats",
            "clock": "detection app receive monotonic; source timestamp metadata retained",
            "pixel_contract": "bbox x1/y1/x2/y2 and center are image pixels (u/v), not metres",
        }
        if observation is None:
            return result
        metadata = getattr(observation, "timestamp_metadata", {}) or {}
        result.update(
            {
                "timestamp_s": _number(getattr(observation, "timestamp_s", None)),
                "age_s": _age(getattr(observation, "timestamp_s", None), now),
                "width_px": _integer(getattr(observation, "width", None)),
                "height_px": _integer(getattr(observation, "height", None)),
                "x1_px": _number(getattr(observation, "x1", None)),
                "y1_px": _number(getattr(observation, "y1", None)),
                "x2_px": _number(getattr(observation, "x2", None)),
                "y2_px": _number(getattr(observation, "y2", None)),
                "center_u_px": _number(getattr(observation, "center", (None, None))[0]),
                "center_v_px": _number(getattr(observation, "center", (None, None))[1]),
                "confidence": _number(getattr(observation, "confidence", None)),
                "class_id": _integer(getattr(observation, "class_id", None)),
                "class_name": str(getattr(observation, "class_name", "") or ""),
                "candidate_count": _safe_len(getattr(observation, "candidate_class_ids", ())),
                "source_timestamp_s": _number(metadata.get("source_timestamp_s")),
                "timestamp_source": str(metadata.get("timestamp_source", "") or ""),
                "timestamp_reference": str(metadata.get("timestamp_reference", "") or ""),
            }
        )
        return result

    def detection_config(self) -> dict[str, object]:
        settings = getattr(self.panel, "settings", None)

        def setting(name: str, default: object = None) -> object:
            value = getattr(settings, name, default)
            return default if value is None else value

        weights = _widget_text(getattr(self.panel, "weights_edit", None), str(setting("yolo_weights_path", "")))
        device = _combo_data(getattr(self.panel, "device_combo", None), setting("yolo_device", "cpu"))
        device = str(device or "cpu")
        gate_names = []
        gate_values = {
            "class": _widget_checked(getattr(self.panel, "yolo_gate_class_checkbox", None), bool(setting("yolo_gate_class_enabled", False))),
            "time": _widget_checked(getattr(self.panel, "yolo_gate_matching_checkbox", None), bool(setting("yolo_gate_matching_enabled", False))),
            "Δt": _widget_checked(getattr(self.panel, "yolo_gate_time_delta_checkbox", None), bool(setting("yolo_gate_time_delta_enabled", False))),
            "vertical Δ": _widget_checked(getattr(self.panel, "yolo_gate_vertical_delta_checkbox", None), bool(setting("yolo_gate_vertical_delta_enabled", False))),
        }
        gate_names.extend(name for name, enabled in gate_values.items() if enabled)
        return {
            "weights_path": weights,
            "device": device,
            "input_size_px": _integer(_widget_value(getattr(self.panel, "input_size_spin", None), setting("yolo_input_size"))),
            "confidence": _number(_widget_value(getattr(self.panel, "conf_spin", None), setting("yolo_conf_threshold"))),
            "iou": _number(_widget_value(getattr(self.panel, "iou_spin", None), setting("yolo_iou_threshold"))),
            "class_gate": _widget_text(getattr(self.panel, "yolo_gate_class_edit", None), str(setting("yolo_gate_class_text", ""))),
            "gate_enabled": tuple(gate_names),
            "max_delta_s": _number(_widget_value(getattr(self.panel, "yolo_gate_time_delta_spin", None), setting("yolo_gate_max_time_delta_s"))),
            "time_basis": str(_combo_data(getattr(self.panel, "yolo_gate_time_basis_combo", None), setting("yolo_gate_time_basis", "")) or ""),
            "max_vertical_delta_px": _number(_widget_value(getattr(self.panel, "yolo_gate_vertical_delta_spin", None), setting("yolo_gate_max_vertical_delta_px"))),
            "source": "AppSettings + existing YOLO configuration widgets",
        }

    def measurement_settings(self) -> dict[str, object]:
        """Project editable measurement/synchronization settings.

        The values are read from the panel's existing controls and settings
        object.  Workstation may send this same shape back through the
        backend facade, but this projection never mutates the owner.
        Units are explicit: focal values are pixels, timing thresholds
        are seconds or milliseconds as labelled, and calibration/extrinsic
        selectors name their frame source rather than converting coordinates.
        """

        settings = getattr(self.panel, "settings", None)

        def setting(name: str, default: object = None) -> object:
            value = getattr(settings, name, default)
            return default if value is None else value

        intrinsics_source = _combo_data(
            getattr(self.panel, "intrinsics_source_combo", None),
            "manual" if _widget_checked(getattr(self.panel, "manual_focal_checkbox", None), bool(setting("manual_focal_override", False))) else "checkerboard",
        )
        distance_mode = _combo_data(
            getattr(self.panel, "distance_mode_combo", None),
            setting("triangulation_extrinsics_source", "pixhawk"),
        )
        return {
            "intrinsics_source": str(intrinsics_source or "checkerboard"),
            "manual_focal_override": _widget_checked(
                getattr(self.panel, "manual_focal_checkbox", None),
                bool(setting("manual_focal_override", False)),
            ),
            "focal_px": _number(
                _widget_value(getattr(self.panel, "focal_spin", None), None),
                900.0,
            ),
            "distance_mode": str(distance_mode or "pixhawk"),
            "yolo_input_size_px": _integer(
                _widget_value(getattr(self.panel, "input_size_spin", None), setting("yolo_input_size"))
            ),
            "yolo_confidence": _number(
                _widget_value(getattr(self.panel, "conf_spin", None), setting("yolo_conf_threshold"))
            ),
            "yolo_iou": _number(
                _widget_value(getattr(self.panel, "iou_spin", None), setting("yolo_iou_threshold"))
            ),
            "class_gate_enabled": _widget_checked(
                getattr(self.panel, "yolo_gate_class_checkbox", None),
                bool(setting("yolo_gate_class_enabled", False)),
            ),
            "class_gate": _widget_text(
                getattr(self.panel, "yolo_gate_class_edit", None),
                str(setting("yolo_gate_class_text", "")),
            ),
            "matching_gate_enabled": _widget_checked(
                getattr(self.panel, "yolo_gate_matching_checkbox", None),
                bool(setting("yolo_gate_matching_enabled", False)),
            ),
            "time_basis": str(
                _combo_data(
                    getattr(self.panel, "yolo_gate_time_basis_combo", None),
                    setting("yolo_gate_time_basis", "camera_rtcp"),
                )
                or "camera_rtcp"
            ),
            "time_delta_gate_enabled": _widget_checked(
                getattr(self.panel, "yolo_gate_time_delta_checkbox", None),
                bool(setting("yolo_gate_time_delta_enabled", False)),
            ),
            "max_pair_delta_s": _number(
                _widget_value(getattr(self.panel, "yolo_gate_time_delta_spin", None), setting("yolo_gate_max_time_delta_s"))
            ),
            "vertical_delta_gate_enabled": _widget_checked(
                getattr(self.panel, "yolo_gate_vertical_delta_checkbox", None),
                bool(setting("yolo_gate_vertical_delta_enabled", False)),
            ),
            "max_vertical_delta_px": _number(
                _widget_value(
                    getattr(self.panel, "yolo_gate_vertical_delta_spin", None),
                    setting("yolo_gate_max_vertical_delta_px"),
                )
            ),
            "camera_sync_history_window_s": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_history_window_spin", None),
                    setting("camera_sync_history_window_s"),
                )
            ),
            "camera_sync_receive_pair_max_delta_ms": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_receive_pair_max_delta_spin", None),
                    _number(setting("camera_sync_receive_pair_max_delta_s"), 0.0) * 1000.0,
                )
            ),
            "camera_sync_rtcp_pair_max_delta_ms": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_rtcp_pair_max_delta_spin", None),
                    _number(setting("camera_sync_rtcp_pair_max_delta_s"), 0.0) * 1000.0,
                )
            ),
            "camera_sync_max_sr_age_s": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_max_sr_age_spin", None),
                    setting("camera_sync_max_sr_age_s"),
                )
            ),
            "camera_sync_max_pc_offset_ms": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_max_pc_offset_spin", None),
                    _number(setting("camera_sync_max_pc_offset_s"), 0.0) * 1000.0,
                )
            ),
            "camera_sync_clock_rate_error_pct": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_clock_rate_error_spin", None),
                    _number(setting("camera_sync_max_clock_rate_error"), 0.0) * 100.0,
                )
            ),
            "camera_sync_required_sr_count": _integer(
                _widget_value(
                    getattr(self.panel, "camera_sync_required_sr_count_spin", None),
                    setting("camera_sync_required_sr_count"),
                )
            ),
            "camera_sync_warmup_timeout_s": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_warmup_timeout_spin", None),
                    setting("camera_sync_warmup_timeout_s"),
                )
            ),
            "source": "existing measurement parameter widgets + AppSettings",
            "coordinate_contract": f"intrinsics/extrinsics source selectors; public output {MEASUREMENT_FRAME} [{MEASUREMENT_AXIS_CONTRACT}]",
        }

    def calibration_setup(self) -> dict[str, object]:
        """Project editable calibration setup fields without changing them."""

        return {
            "zoom_level": _integer(_widget_value(getattr(self.panel, "zoom_level_spin", None))),
            "inner_cols": _integer(_widget_value(getattr(self.panel, "calib_cols_spin", None))),
            "inner_rows": _integer(_widget_value(getattr(self.panel, "calib_rows_spin", None))),
            "square_size_mm": _number(_widget_value(getattr(self.panel, "calib_square_spin", None))),
            "min_images": _integer(_widget_value(getattr(self.panel, "calib_min_images_spin", None))),
            "calibration_root": _widget_text(getattr(self.panel, "calib_dir_edit", None)),
            "validation_mode": str(
                _combo_data(getattr(self.panel, "checkerboard_validation_mode_combo", None), "pixhawk")
                or "pixhawk"
            ),
            "validation_root": _widget_text(getattr(self.panel, "checkerboard_validation_root_edit", None)),
            "validation_json": _widget_text(getattr(self.panel, "checkerboard_validation_json_edit", None)),
            "validation_runs": _integer(
                _widget_value(getattr(self.panel, "checkerboard_validation_random_runs_spin", None))
            ),
            "validation_pair_count": _integer(
                _widget_value(getattr(self.panel, "checkerboard_validation_random_pair_count_spin", None))
            ),
            "validation_seed": _integer(
                _widget_value(getattr(self.panel, "checkerboard_validation_seed_spin", None))
            ),
            "source": "existing calibration setup widgets",
            "frame_contract": f"checkerboard image pixels/{CAMERA_FRAME}; stereo R/T remains left-camera optical RDF → right-camera optical RDF",
        }

    def session_catalog(self) -> tuple[dict[str, object], ...]:
        """List finalized/retryable session directories for review only."""

        sessions_root = self._app_root() / "recordings" / "sessions"
        if not sessions_root.is_dir():
            return ()
        entries: list[dict[str, object]] = []
        for path in sorted((item for item in sessions_root.iterdir() if item.is_dir()), key=lambda item: item.name, reverse=True):
            manifest_path = path / "session_manifest.json"
            manifest: dict[str, object] = {}
            if manifest_path.is_file():
                try:
                    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
                    if isinstance(payload, dict):
                        manifest = payload
                except (OSError, ValueError):
                    manifest = {}
            published = path / "published"
            published_target = ""
            if published.is_symlink():
                try:
                    published_target = str(published.resolve().relative_to(path.resolve()))
                except (OSError, ValueError):
                    published_target = str(published)
            resource_files = manifest.get("resource_files", {})
            if not isinstance(resource_files, dict):
                resource_files = {}
            artifact_roles = manifest.get("artifact_roles", {})
            if not isinstance(artifact_roles, dict):
                artifact_roles = {}
            artifact_states = manifest.get("artifacts", {})
            if not isinstance(artifact_states, dict):
                artifact_states = {}
            artifact_names = {
                str(name)
                for name in (*resource_files.values(), *artifact_roles.keys(), *artifact_states.keys())
                if name not in (None, "")
            }
            artifacts = tuple(
                sorted(artifact_names)
            )
            entries.append(
                {
                    "session_id": path.name,
                    "path": str(path),
                    "status": str(manifest.get("status", "unknown") or "unknown"),
                    "phase": str(manifest.get("phase", "") or ""),
                    "generation_id": str(manifest.get("generation_id", "") or ""),
                    "error": str(manifest.get("error", "") or ""),
                    "retryable": bool(manifest.get("retryable", False)),
                    "published_target": published_target,
                    "artifacts": artifacts,
                    "source": "session_manifest.json + published generation pointer",
                    "time_axis": str(manifest.get("time_axis", "recording_time_ns") or "recording_time_ns"),
                }
            )
        return tuple(entries)

    def session_artifact_rows(
        self,
        session_id: str,
        artifact_name: str,
        *,
        limit: int = 100,
    ) -> dict[str, object]:
        """Read stored CSV rows without reinterpreting timestamps or frames."""

        sessions_root = (self._app_root() / "recordings" / "sessions").resolve()
        output_dir = (sessions_root / Path(str(session_id))).resolve()
        try:
            output_dir.relative_to(sessions_root)
        except ValueError:
            return {"columns": (), "rows": (), "error": "session path outside recordings root"}
        candidate = output_dir / Path(str(artifact_name)).name
        if not candidate.exists() and (output_dir / "published").is_symlink():
            candidate = output_dir / "published" / Path(str(artifact_name)).name
        if not candidate.is_file():
            return {"columns": (), "rows": (), "error": f"artifact not found: {artifact_name}"}
        if candidate.suffix.lower() != ".csv":
            return {
                "columns": (),
                "rows": (),
                "error": "only CSV artifacts are previewed; binary/video/XLSX outputs remain unchanged",
                "path": str(candidate),
            }
        try:
            with candidate.open("r", encoding="utf-8-sig", newline="") as handle:
                reader = csv.DictReader(handle)
                columns = tuple(reader.fieldnames or ())
                rows = tuple(dict(row) for _, row in zip(range(max(0, int(limit))), reader))
        except (OSError, UnicodeError, csv.Error) as exc:
            return {"columns": (), "rows": (), "error": str(exc), "path": str(candidate)}
        return {
            "columns": columns,
            "rows": rows,
            "error": "",
            "path": str(candidate),
            "time_axis": "recording_time_ns; source columns preserved",
            "coordinate_contract": "stored CSV fields are displayed as saved; no NED/ENU/camera-frame conversion",
        }

    def recording(self, *, now_s: float | None = None) -> dict[str, object]:
        now = time.monotonic() if now_s is None else float(now_s)
        session_id = str(getattr(self.panel, "recording_session_id", "") or "")
        output_dir: Path | None = None
        if session_id:
            method = getattr(self.panel, "_recording_session_dir", None)
            if callable(method):
                try:
                    output_dir = Path(method(session_id))
                except (OSError, TypeError, ValueError):
                    output_dir = None
        paths = getattr(self.panel, "video_recording_paths", {}) or {}
        video_paths = {str(side): str(path) for side, path in paths.items()} if isinstance(paths, dict) else {}
        artifacts: list[str] = []
        if output_dir is not None and output_dir.exists():
            try:
                artifacts = sorted(path.name for path in output_dir.iterdir() if path.is_file())[:64]
            except OSError:
                artifacts = []
        active = bool(getattr(self.panel, "distance_recording_enabled", False) or getattr(self.panel, "video_recording_enabled", False))
        active_method = getattr(self.panel, "_session_recording_active", None)
        if callable(active_method):
            try:
                active = bool(active_method())
            except Exception:
                pass
        elapsed_method = getattr(self.panel, "_recording_elapsed_s", None)
        elapsed = None
        if callable(elapsed_method):
            try:
                elapsed = _number(elapsed_method(now))
            except Exception:
                elapsed = None
        status_label = _widget_text(getattr(self.panel, "recording_status_label", None), "")
        save_thread = getattr(self.panel, "_session_save_thread", None)
        retry = bool(getattr(self.panel, "_session_save_retry_required", False))
        finalized = bool(getattr(self.panel, "_session_save_finalized", False))
        if save_thread is not None:
            finalization = "FINALIZING"
        elif retry:
            finalization = "RETRYABLE"
        elif finalized:
            finalization = "FINALIZED"
        elif active:
            finalization = "ACTIVE"
        else:
            finalization = "IDLE"
        return {
            "state": finalization,
            "active": active,
            "session_id": session_id,
            "output_dir": str(output_dir) if output_dir is not None else "",
            "video_paths": video_paths,
            "artifacts": tuple(artifacts),
            "elapsed_s": elapsed,
            "status_label": status_label,
            "retry_required": retry,
            "finalized": finalized,
            "sample_counts": {
                "pairs": _safe_len(getattr(self.panel, "distance_recording_rows", ())),
                "camera_left": _safe_len((getattr(self.panel, "camera_recording_rows", {}) or {}).get("left", ())),
                "camera_right": _safe_len((getattr(self.panel, "camera_recording_rows", {}) or {}).get("right", ())),
                "px4_left": _safe_len((getattr(self.panel, "px4_recording_rows", {}) or {}).get("left", ())),
                "px4_right": _safe_len((getattr(self.panel, "px4_recording_rows", {}) or {}).get("right", ())),
                "gnss_base": _safe_len(getattr(self.panel, "gnss_base_recording_rows", ())),
            },
            "recovered": getattr(self.panel, "_recovered_session_metadata", None) is not None,
            "source": "session spool rows + recording session/save worker",
            "clock": "session monotonic anchor; wall-clock ID retained",
        }

    @staticmethod
    def _camera_intrinsics(camera: object | None) -> dict[str, object]:
        if camera is None:
            return {"state": "NO DATA"}
        matrix = getattr(camera, "camera_matrix", None)
        return {
            "state": "AVAILABLE",
            "width_px": _integer(getattr(camera, "width", None)),
            "height_px": _integer(getattr(camera, "height", None)),
            "fx_px": _matrix_value(matrix, 0, 0),
            "fy_px": _matrix_value(matrix, 1, 1),
            "cx_px": _matrix_value(matrix, 0, 2),
            "cy_px": _matrix_value(matrix, 1, 2),
            "reprojection_error_px": _number(getattr(camera, "reprojection_error_px", None)),
            "image_count": _integer(getattr(camera, "image_count", None)),
            "distortion_coefficients": tuple(getattr(camera, "distortion_coefficients", ()) or ()),
            "source": "StereoIntrinsicsResult / CameraIntrinsics",
            "frame_contract": f"{CAMERA_FRAME}; camera matrix units are pixels",
        }

    def calibration(self) -> dict[str, object]:
        active = getattr(self.panel, "active_intrinsics", None)
        latest = getattr(self.panel, "last_calibration_result", None)
        result = active or latest
        status_label = _widget_text(getattr(self.panel, "calibration_status_label", None), "")
        profile_path = str(getattr(self.panel, "_active_intrinsics_profile_path", "") or "")
        if not profile_path and result is not None:
            profile_path = "memory/latest-calibration"
        zoom = _integer(getattr(self.panel, "last_calibration_zoom_level", None))
        if zoom is None:
            zoom_widget = getattr(self.panel, "zoom_level_spin", None)
            zoom = _integer(_widget_value(zoom_widget))
        extrinsics = None if result is None else getattr(result, "extrinsics", None)
        extrinsics_payload: dict[str, object] = {}
        if extrinsics is not None:
            translation = tuple(getattr(extrinsics, "translation_left_to_right_m", ()) or ())
            extrinsics_payload = {
                "state": "AVAILABLE",
                "pair_count": _integer(getattr(extrinsics, "pair_count", None)),
                "reprojection_error_px": _number(getattr(extrinsics, "reprojection_error_px", None)),
                "translation_left_to_right_m": translation,
                "frame_contract": str(getattr(extrinsics, "frame_contract", "") or f"{CAMERA_FRAME}; left→right R/T"),
                "baseline_m": sum(float(value) * float(value) for value in translation) ** 0.5 if translation else None,
            }
        return {
            "state": "ACTIVE" if active is not None else "AVAILABLE" if result is not None else "NO DATA",
            "active": active is not None,
            "profile_path": profile_path,
            "zoom": zoom,
            "status_label": status_label,
            "source": "active_intrinsics/last_calibration_result",
            "left": self._camera_intrinsics(None if result is None else getattr(result, "left", None)),
            "right": self._camera_intrinsics(None if result is None else getattr(result, "right", None)),
            "extrinsics": extrinsics_payload,
            "validation_mode": str(getattr(self.panel, "checkerboard_validation_result_mode", "") or ""),
            "validation_summary": dict(getattr(self.panel, "checkerboard_validation_summary", {}) or {}),
            "frame_contract": f"{CAMERA_FRAME}; calibration intrinsics/extrinsics stay in OpenCV optical RDF",
            "measurement_contract": f"public output {MEASUREMENT_FRAME} [{MEASUREMENT_AXIS_CONTRACT}]; baseline {BASELINE_TRANSLATION_FRAME}",
        }

    def summary(self, *, now_s: float | None = None) -> dict[str, object]:
        now = time.monotonic() if now_s is None else float(now_s)
        return {
            "attitude": {side: self.attitude(side, now_s=now) for side in ("left", "right")},
            "gps": {side: self.gps(side, now_s=now) for side in ("left", "right")},
            "baseline": self.baseline(),
            "detection_config": self.detection_config(),
            "detection": {side: self.detection(side, now_s=now) for side in ("left", "right")},
            "recording": self.recording(now_s=now),
            "calibration": self.calibration(),
        }
