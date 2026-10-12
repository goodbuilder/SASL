"""Geometry feature: owned state/views and explicit dependency ports.

Coordinate values retain their producer contracts (NED, body FRD, optical RDF,
and baseline-center FRD). This ownership change performs no axis conversion.
"""
from __future__ import annotations

from Workstation.panel.panel_feature_runtime import PanelFeature
import json
import math
import time
from PyQt5 import (
    QtCore,
    QtGui,
    QtWidgets,
)
from Workstation.panel.panel_support import MAX_MANUAL_FOCAL_PX, WIRIS_VISIBLE_ZOOM_FX_TOOLTIP
from Workstation.workstation_core.calibration import (
    StereoIntrinsicsResult,
    load_intrinsics_json,
)
from Workstation.workstation_core.coordinate_contract import (
    BODY_FRAME,
    CAMERA_FRAME,
    CAMERA_MOUNT_PROFILE,
    WORLD_FRAME,
)
from Workstation.workstation_core.measurement_methods import (
    MEASUREMENT_METHOD_LABELS,
)
from Workstation.workstation_core.measurement_widgets import (
    Triangulation3DView,
)
from Workstation.workstation_core.pixhawk_attitude import (
    CameraExtrinsicState,
)
from Workstation.workstation_core.settings import (
    resolve_settings_path,
)
from Workstation.workstation_core.sync import (
    ATTITUDE_STALE_AGE_S,
    ATTITUDE_STALE_PAIR_DELTA_S,
    ATTITUDE_TIMING_POLICY,
    ATTITUDE_VERY_STALE_AGE_S,
    ATTITUDE_VERY_STALE_PAIR_DELTA_S,
)
from Workstation.workstation_core.triangulation import (
    invert_left_to_right_camera_transform,
)
from datetime import (
    datetime,
)
from pathlib import (
    Path,
)


class GeometryFeature(PanelFeature):
    feature_name = 'geometry'
    dependency_names = (
        '_active_baseline_m',
        '_build_toggle_button',
        '_candidate_intrinsics_profile_path',
        '_checkerboard_translation_right_in_left_m',
        '_clear_measurement_pair_display',
        '_clear_triangulation_quality',
        '_format_mount_assumption_summary',
        '_intrinsics_storage_base_path',
        '_intrinsics_storage_path',
        '_latest_translation_right_in_left_m_for_diagnostics',
        '_log',
        '_measurement_selected_method_id',
        '_mount_assumption_status_label',
        '_refresh_distance',
        '_refresh_operation_header',
        '_refresh_optical_alignment_view',
        '_set_combo_data',
        '_workstation_setting',
        '_workstation_setting_overrides',
        '_yolo_pair_max_delta_s',
        '_yolo_vertical_max_delta_px',
        'active_intrinsics',
        'app_root',
        'apply_intrinsics_to_distance',
        'baseline_label',
        'baseline_spin',
        'calibration_image_diagnostics',
        'calibration_status_label',
        'camera_sync_clock_rate_error_spin',
        'camera_sync_history_window_spin',
        'camera_sync_interval_spin',
        'camera_sync_max_pc_offset_spin',
        'camera_sync_max_sr_age_spin',
        'camera_sync_mode_combo',
        'camera_sync_receive_pair_max_delta_spin',
        'camera_sync_required_sr_count_spin',
        'camera_sync_rtcp_pair_max_delta_spin',
        'camera_sync_settings_group',
        'camera_sync_warmup_timeout_spin',
        'current_parameters_group',
        'extrinsics_settings_group',
        'gps_baseline_spins',
        'intrinsics_settings_group',
        'intrinsics_source_combo',
        'intrinsics_status_label',
        'last_calibration_result',
        'last_calibration_zoom_level',
        'latest_extrinsics',
        'latest_rtk_baseline',
        'left_extrinsics_text',
        'left_intrinsics_text',
        'right_extrinsics_text',
        'right_intrinsics_text',
        'settings',
        'stereo_parameter_splitter',
        'triangulation_quality_attitude_label',
        'yolo_gate_class_checkbox',
        'yolo_gate_class_edit',
        'zoom_intrinsics_compare_list',
        'zoom_intrinsics_compare_paths',
        'zoom_intrinsics_path_label',
        'zoom_intrinsics_status_label',
        'zoom_left_intrinsics_text',
        'zoom_level_spin',
        'zoom_right_intrinsics_text',
    )
    writable_dependencies = frozenset(('_candidate_intrinsics_profile_path', 'baseline_label', 'baseline_spin', 'calibration_image_diagnostics', 'camera_sync_clock_rate_error_spin', 'camera_sync_history_window_spin', 'camera_sync_interval_spin', 'camera_sync_max_pc_offset_spin', 'camera_sync_max_sr_age_spin', 'camera_sync_mode_combo', 'camera_sync_receive_pair_max_delta_spin', 'camera_sync_required_sr_count_spin', 'camera_sync_rtcp_pair_max_delta_spin', 'camera_sync_settings_group', 'camera_sync_warmup_timeout_spin', 'current_parameters_group', 'extrinsics_settings_group', 'gps_baseline_spins', 'intrinsics_settings_group', 'intrinsics_source_combo', 'intrinsics_status_label', 'last_calibration_result', 'last_calibration_zoom_level', 'left_extrinsics_text', 'left_intrinsics_text', 'right_extrinsics_text', 'right_intrinsics_text', 'stereo_parameter_splitter', 'triangulation_quality_attitude_label'))

    def initialize_state(self, *, auto_start=False, auto_start_pixhawk=False, app_root=None, settings_path=None):
        pass

    def _set_focal_override_enabled(self, enabled: bool) -> None:
        self.focal_spin.setEnabled(bool(enabled))
        if hasattr(self, "intrinsics_source_combo"):
            desired = "manual" if enabled else "checkerboard"
            if self.intrinsics_source_combo.currentData() != desired:
                self._set_combo_data(self.intrinsics_source_combo, desired)
        self._update_distance_settings_visibility()
        self._refresh_camera_parameters_tab()

    def _handle_intrinsics_source_changed(self, _index: int) -> None:
        manual = self._intrinsics_source() == "manual"
        if self.manual_focal_checkbox.isChecked() != manual:
            self.manual_focal_checkbox.setChecked(manual)
            return
        self._update_distance_settings_visibility()
        self._refresh_camera_parameters_tab()

    def _handle_distance_mode_changed(self, _index: int) -> None:
        mode = self._distance_mode()
        self._distance_mode_last_source = mode
        self._update_distance_settings_visibility()
        self._refresh_camera_parameters_tab()
        self._refresh_optical_alignment_view()

    def _handle_measurement_display_mode_changed(self, _index: int) -> None:
        if self._measurement_display_mode() == "preview":
            self._clear_measurement_pair_display()
        else:
            self._refresh_distance()
        self._refresh_camera_parameters_tab()

    def _intrinsics_source(self) -> str:
        if not hasattr(self, "intrinsics_source_combo"):
            return "manual" if self.manual_focal_checkbox.isChecked() else "checkerboard"
        source = str(self.intrinsics_source_combo.currentData() or "checkerboard")
        return "manual" if source == "manual" else "checkerboard"

    def _update_distance_settings_visibility(self) -> None:
        if not hasattr(self, "intrinsics_source_combo"):
            return
        manual_intrinsics = self._intrinsics_source() == "manual"
        for widget in (self.manual_focal_label, self.focal_spin):
            widget.setVisible(manual_intrinsics)

        for widget in (self.distance_mode_label, self.distance_mode_combo):
            widget.setVisible(True)
        for widget in (self.baseline_label, self.baseline_spin):
            widget.setVisible(False)

        if self._distance_mode() == "checkerboard":
            self.mount_assumption_label.setText(
                "장착 가정: 체커보드로 계산한 고정 좌/우 카메라 기하를 사용합니다. "
                "체커보드 R/T는 좌측 optical RDF에서 우측 optical RDF로 변환해 적용합니다."
            )
        else:
            self.mount_assumption_label.setText(
                "장착 가정: GPS/RTK baseline only. "
                "수동 baseline 입력은 사용하지 않고 좌/우 Pixhawk GPS baseline과 자세만 계산에 사용합니다."
            )

        source_text = "수동 fallback" if manual_intrinsics else "체커보드 보정값"
        if self.active_intrinsics is not None:
            left_fx = self.active_intrinsics.left.camera_matrix[0][0]
            right_fx = self.active_intrinsics.right.camera_matrix[0][0]
            detail = f"적용됨, fx 평균={(left_fx + right_fx) / 2.0:.3f} px"
        elif self.last_calibration_result is not None:
            detail = "불러옴/계산됨, 아직 적용 전"
        else:
            detail = "NOMINAL_UNVERIFIED 자동 fallback · fresh zoom/raster readback 필요"
        self.intrinsics_status_label.setText(f"내부 파라미터: {source_text} | {detail}")

    def _configure_collapsible_group(
        self, group: QtWidgets.QGroupBox, *, checked: bool = True, collapsed_height: int = 34
    ) -> None:
        group.setCheckable(True)
        group.setChecked(bool(checked))

        def set_group_visible(current: bool, target: QtWidgets.QGroupBox = group) -> None:
            self._set_group_children_visible(target, current)
            target.setMaximumHeight(16777215 if current else collapsed_height)

        group.toggled.connect(set_group_visible)
        set_group_visible(bool(checked))

    def _set_group_children_visible(self, group: QtWidgets.QGroupBox, visible: bool) -> None:
        for child in group.findChildren(QtWidgets.QWidget):
            child.setVisible(visible)

    def _build_calibration_preview_label(self, text: str) -> QtWidgets.QLabel:
        label = QtWidgets.QLabel(text)
        label.setObjectName("calibrationPreview")
        label.setMinimumSize(320, 180)
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        return label

    def _build_parameter_text(self) -> QtWidgets.QPlainTextEdit:
        editor = QtWidgets.QPlainTextEdit()
        editor.setReadOnly(True)
        editor.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        editor.setMinimumHeight(180)
        font = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont)
        editor.setFont(font)
        return editor

    def _format_matrix(self, matrix: list[list[float]]) -> str:
        return "\n".join("  [" + ", ".join(f"{value:12.6f}" for value in row) + "]" for row in matrix)

    def _intrinsics_source_label(self) -> str:
        if self.active_intrinsics is not None:
            return "active calibration result"
        if self.last_calibration_result is not None:
            return "latest calibration result (not applied)"
        return "not loaded"

    def _focal_source_label(self) -> str:
        if self._intrinsics_source() == "manual":
            return f"manual focal spinbox ({self.focal_spin.value():.6f} px)"
        if self.active_intrinsics is not None:
            left_fx = self.active_intrinsics.left.camera_matrix[0][0]
            right_fx = self.active_intrinsics.right.camera_matrix[0][0]
            return f"calibrated intrinsics average fx ({(left_fx + right_fx) / 2.0:.6f} px)"
        if self.last_calibration_result is not None:
            return f"latest calibration result not applied; spinbox fallback ({self.focal_spin.value():.6f} px)"
        return f"spinbox fallback ({self.focal_spin.value():.6f} px)"

    def _baseline_source_label(self, baseline_source: str, baseline_m: float) -> str:
        if baseline_source == "gps-rtk-waiting":
            return "waiting for live GPS/RTK baseline"
        if baseline_source == "gps-rtk":
            return f"live GPS/RTK baseline ({baseline_m:.6f} m)"
        if baseline_source == "gps-rtk-blocked":
            return "live GPS/RTK baseline blocked"
        if baseline_source.startswith("RTK"):
            return f"{baseline_source} from Pixhawk/GPS"
        return baseline_source

    def _extrinsics_source_label(self, extrinsic: CameraExtrinsicState | None) -> str:
        if extrinsic is None:
            return "none (Pixhawk disconnected or waiting)"
        topic = extrinsic.topic.strip()
        if topic.startswith("mavlink/"):
            return f"MAVLink ({topic})"
        return f"MAVLink ({topic})" if topic else "MAVLink telemetry"

    def _distance_mode(self) -> str:
        if not hasattr(self, "distance_mode_combo"):
            return "pixhawk"
        mode = str(self.distance_mode_combo.currentData() or "pixhawk")
        return mode if mode in {"pixhawk", "checkerboard"} else "pixhawk"

    def _measurement_display_mode(self) -> str:
        value = self._workstation_setting_overrides.get("measurement_display_mode", "detection")
        return str(value) if value in {"detection", "preview"} else "detection"

    def _distance_mode_label(self) -> str:
        selected = str(getattr(self, "_measurement_selected_method_id", "") or "")
        if selected in MEASUREMENT_METHOD_LABELS:
            return str(MEASUREMENT_METHOD_LABELS[selected])
        if self._distance_mode() == "checkerboard":
            return "체커보드 Stereo 외부 파라미터"
        return "Pixhawk 자세 + GPS/RTK baseline"

    def _triangulation_extrinsics_source(self) -> str:
        return self._distance_mode()

    def _measurement_path_label(self) -> str:
        if self.active_intrinsics is None:
            return "3D ray fallback: 내부 파라미터 대기 중"
        selected = str(getattr(self, "_measurement_selected_method_id", "") or "")
        if selected in MEASUREMENT_METHOD_LABELS:
            return f"3D ray 거리: {MEASUREMENT_METHOD_LABELS[selected]} 사용"
        if self._distance_mode() == "checkerboard":
            if self._checkerboard_translation_right_in_left_m() is None:
                return "3D ray fallback: 체커보드 stereo 외부 파라미터 대기 중"
            return "3D ray 거리: 체커보드 Stereo 외부 파라미터 사용"
        left_ext = self.latest_extrinsics.get("left")
        right_ext = self.latest_extrinsics.get("right")
        if left_ext is None or right_ext is None:
            return "3D ray fallback: Pixhawk 자세 대기 중"
        if self._latest_translation_right_in_left_m_for_diagnostics() is None:
            return "3D ray fallback: GPS/RTK baseline 대기 중"
        return "3D ray 거리: Pixhawk 자세 + GPS/RTK baseline 사용"

    def _triangulation_extrinsics_source_label(self) -> str:
        selected = str(getattr(self, "_measurement_selected_method_id", "") or "")
        if selected in MEASUREMENT_METHOD_LABELS:
            return str(MEASUREMENT_METHOD_LABELS[selected])
        if self._distance_mode() == "checkerboard":
            return "체커보드 Stereo 외부 파라미터"
        return "Pixhawk 자세 + GPS/RTK baseline"

    def _distance_parameters_summary(self) -> str:
        path = self._measurement_path_label()
        focal = self._focal_source_label()
        baseline_m, baseline_source, _warnings = self._active_baseline_m()
        geometry = f"baseline={self._baseline_source_label(baseline_source, baseline_m)}"
        external = f"장착={self._mount_assumption_status_label()}"
        return (
            f"계산 파라미터: {path} | 초점거리={focal} | "
            f"위치/자세 출처={self._triangulation_extrinsics_source_label()} | {geometry} | {external}"
        )

    def _refresh_distance_parameters_label(self) -> None:
        if hasattr(self, "distance_parameters_label"):
            self.distance_parameters_label.setText(self._distance_parameters_summary())

    def _format_intrinsics_text(
        self,
        camera: str,
        intrinsics,
        status: str,
        source_label: str | None = None,
    ) -> str:
        title = "Left Camera Intrinsics" if camera == "left" else "Right Camera Intrinsics"
        resolved_source = source_label or self._intrinsics_source_label()
        if intrinsics is None:
            return (
                f"{title}\n"
                f"status: not loaded\n"
                f"source: {resolved_source}\n\n"
                "먼저 내부 파라미터 보정 결과를 불러오거나 실행하세요."
            )
        return (
            f"{title}\n"
            f"status: {status}\n"
            f"source: {resolved_source}\n"
            f"image_size: {intrinsics.width} x {intrinsics.height}\n"
            f"valid_images: {intrinsics.image_count}\n"
            f"reprojection_error: {intrinsics.reprojection_error_px:.6f} px\n\n"
            f"camera_matrix K:\n{self._format_matrix(intrinsics.camera_matrix)}\n\n"
            f"fx: {intrinsics.camera_matrix[0][0]:.6f}\n"
            f"fy: {intrinsics.camera_matrix[1][1]:.6f}\n"
            f"cx: {intrinsics.camera_matrix[0][2]:.6f}\n"
            f"cy: {intrinsics.camera_matrix[1][2]:.6f}\n\n"
            "distortion_coefficients:\n"
            + ", ".join(f"{value:.9g}" for value in intrinsics.distortion_coefficients)
        )

    def _current_zoom_intrinsics_result(self) -> tuple[StereoIntrinsicsResult | None, str, str, Path | None]:
        zoom_level = self.zoom_level_spin.value()
        if self.last_calibration_result is not None and self.last_calibration_zoom_level == zoom_level:
            candidate_path_text = str(
                getattr(self, "_candidate_intrinsics_profile_path", "") or ""
            )
            candidate_path = (
                Path(candidate_path_text)
                if candidate_path_text and not candidate_path_text.startswith("memory:")
                else None
            )
            candidate_name = (
                candidate_path.name if candidate_path is not None else "in-memory result"
            )
            return (
                self.last_calibration_result,
                f"calibration candidate (zoom {zoom_level:02d})",
                f"calibration candidate for zoom {zoom_level:02d} ({candidate_name})",
                candidate_path,
            )
        zoom_path = self._intrinsics_storage_path(zoom_level)
        if zoom_path.exists():
            try:
                return (
                    load_intrinsics_json(zoom_path),
                    f"saved zoom calibration ({zoom_path.name})",
                    f"zoom {zoom_level:02d} file",
                    zoom_path,
                )
            except Exception as exc:
                return None, f"failed to load {zoom_path.name}: {exc}", f"zoom {zoom_level:02d} file", zoom_path
        latest_path = self._intrinsics_storage_base_path()
        if latest_path.exists():
            try:
                latest_zoom_level = self._intrinsics_file_zoom_level(latest_path)
                latest_zoom_label = (
                    f", zoom {latest_zoom_level:02d}" if latest_zoom_level is not None else ""
                )
                return (
                    load_intrinsics_json(latest_path),
                    f"latest alias fallback ({latest_path.name}{latest_zoom_label})",
                    f"latest alias fallback ({latest_path.name}{latest_zoom_label})",
                    latest_path,
                )
            except Exception as exc:
                return None, f"failed to load {latest_path.name}: {exc}", f"latest alias fallback ({latest_path.name})", latest_path
        return None, f"not loaded for zoom {zoom_level:02d}", f"zoom {zoom_level:02d}", None

    def _configured_intrinsics_profile_path(self) -> Path | None:
        raw_path = str(
            self._workstation_setting("intrinsics_path", self.settings.intrinsics_path) or ""
        )
        if not raw_path.strip():
            return None
        return resolve_settings_path(raw_path, base_dir=self.app_root)

    def _load_intrinsics_profile_for_exact_zoom(
        self,
        path: Path,
        zoom_level: int,
    ) -> tuple[StereoIntrinsicsResult | None, str]:
        if not path.is_file():
            return None, f"intrinsic profile is not a regular file: {path}"
        try:
            result = load_intrinsics_json(path)
        except Exception as exc:
            return None, f"invalid intrinsic profile: {path}: {exc}"
        metadata_zoom_level = self._intrinsics_file_zoom_level(path)
        if metadata_zoom_level is None:
            return None, f"intrinsic profile has no valid metadata.zoom_level: {path}"
        if metadata_zoom_level != int(zoom_level):
            return (
                None,
                "intrinsic profile zoom mismatch: "
                f"configured={metadata_zoom_level:02d}, operating={int(zoom_level):02d}, path={path}",
            )
        return result, ""

    def _auto_load_intrinsics_candidate(
        self,
        zoom_level: int,
    ) -> tuple[StereoIntrinsicsResult | None, Path | None, str]:
        """Resolve the exact persisted profile without silently changing operator intent.

        An existing configured profile is authoritative.  If it is malformed or belongs to a
        different zoom, automatic loading fails closed instead of selecting another file.  The
        generated per-zoom/base storage paths are compatibility fallbacks only when the missing
        configured path is the calibration root's base alias.  A missing explicit external path
        never falls through to unrelated storage bytes.
        """

        configured_path = self._configured_intrinsics_profile_path()
        zoom_path = self._intrinsics_storage_path(zoom_level)
        base_path = self._intrinsics_storage_base_path()
        if configured_path is not None and configured_path.exists():
            result, error = self._load_intrinsics_profile_for_exact_zoom(
                configured_path,
                zoom_level,
            )
            return result, configured_path, error

        if configured_path is not None and configured_path != base_path:
            return (
                None,
                configured_path,
                f"configured intrinsic profile is missing: {configured_path}; "
                "automatic storage fallback is disabled for an explicit profile path",
            )

        candidates: list[Path] = []
        for candidate in (zoom_path, base_path):
            if candidate not in candidates:
                candidates.append(candidate)
        for candidate in candidates:
            if not candidate.exists():
                continue
            result, error = self._load_intrinsics_profile_for_exact_zoom(candidate, zoom_level)
            return result, candidate, error

        if configured_path is not None:
            return (
                None,
                configured_path,
                f"configured intrinsic profile is missing: {configured_path}; "
                f"no zoom {int(zoom_level):02d} storage profile exists at {zoom_path}",
            )
        return (
            None,
            zoom_path,
            f"no configured intrinsic profile and no zoom {int(zoom_level):02d} "
            f"storage profile exists at {zoom_path}",
        )

    def _load_current_zoom_intrinsics_for_apply(self) -> StereoIntrinsicsResult | None:
        zoom_level = self.zoom_level_spin.value()
        zoom_path = self._intrinsics_storage_path(zoom_level)
        if zoom_path.exists():
            return load_intrinsics_json(zoom_path)
        if self.last_calibration_result is not None and self.last_calibration_zoom_level == zoom_level:
            return self.last_calibration_result
        latest_path = self._intrinsics_storage_base_path()
        if latest_path.exists() and self._intrinsics_file_zoom_level(latest_path) == zoom_level:
            return load_intrinsics_json(latest_path)
        return None

    def _intrinsics_file_zoom_level(self, path: Path) -> int | None:
        try:
            payload = json.loads(Path(path).read_text())
        except Exception:
            return None
        metadata = payload.get("metadata") or {}
        try:
            zoom_level = metadata.get("zoom_level")
            return None if zoom_level is None else int(zoom_level)
        except (TypeError, ValueError):
            return None

    def _intrinsics_file_metadata(self, path: Path) -> dict:
        try:
            payload = json.loads(Path(path).read_text())
        except Exception:
            return {}
        metadata = payload.get("metadata") or {}
        return metadata if isinstance(metadata, dict) else {}

    def _format_zoom_intrinsics_comparison_text(self, path: Path, result: StereoIntrinsicsResult) -> str:
        metadata = self._intrinsics_file_metadata(path)
        zoom_value = metadata.get("zoom_level")
        zoom_text = "--" if zoom_value is None else f"{int(zoom_value):02d}"
        timestamp_text = ""
        try:
            timestamp_s = float(metadata.get("timestamp_s"))
            timestamp_text = datetime.fromtimestamp(timestamp_s).strftime("%Y-%m-%d %H:%M:%S")
        except (TypeError, ValueError, OSError):
            timestamp_text = "unknown time"
        left = result.left
        right = result.right
        lines = [
            f"{path.name} | zoom {zoom_text} | {timestamp_text}",
            str(path),
            (
                "Left  "
                f"{left.width}x{left.height} fx={left.camera_matrix[0][0]:.3f} "
                f"fy={left.camera_matrix[1][1]:.3f} cx={left.camera_matrix[0][2]:.3f} "
                f"cy={left.camera_matrix[1][2]:.3f} err={left.reprojection_error_px:.4f}px "
                f"n={left.image_count}"
            ),
            (
                "Right "
                f"{right.width}x{right.height} fx={right.camera_matrix[0][0]:.3f} "
                f"fy={right.camera_matrix[1][1]:.3f} cx={right.camera_matrix[0][2]:.3f} "
                f"cy={right.camera_matrix[1][2]:.3f} err={right.reprojection_error_px:.4f}px "
                f"n={right.image_count}"
            ),
        ]
        return "\n".join(lines)

    def add_zoom_intrinsics_comparison_files(self) -> None:
        start_path = self._intrinsics_storage_base_path().parent
        selected, _filter = QtWidgets.QFileDialog.getOpenFileNames(
            self.widget,
            "Add intrinsics JSON files",
            str(start_path),
            "Intrinsics JSON (*.json);;All files (*)",
        )
        added = 0
        for raw_path in selected:
            if self._add_zoom_intrinsics_comparison_path(Path(raw_path)):
                added += 1
        if added:
            self.calibration_status_label.setText(f"비교 목록에 intrinsic 파일 {added}개를 추가했습니다.")

    def remove_selected_zoom_intrinsics_comparison_files(self) -> None:
        if not hasattr(self, "zoom_intrinsics_compare_list"):
            return
        rows = sorted({index.row() for index in self.zoom_intrinsics_compare_list.selectedIndexes()}, reverse=True)
        for row in rows:
            item = self.zoom_intrinsics_compare_list.takeItem(row)
            if item is None:
                continue
            raw_path = item.data(QtCore.Qt.UserRole)
            try:
                self.zoom_intrinsics_compare_paths.remove(Path(str(raw_path)))
            except ValueError:
                pass
        if rows:
            self.calibration_status_label.setText(f"비교 목록에서 intrinsic 파일 {len(rows)}개를 제거했습니다.")

    def apply_selected_zoom_intrinsics_comparison_file(self) -> None:
        if not hasattr(self, "zoom_intrinsics_compare_list"):
            return
        item = self.zoom_intrinsics_compare_list.currentItem()
        if item is None:
            selected = self.zoom_intrinsics_compare_list.selectedItems()
            item = selected[0] if selected else None
        if item is None:
            self.calibration_status_label.setText("적용할 비교 목록 파일을 선택하세요.")
            return
        path = Path(str(item.data(QtCore.Qt.UserRole)))
        try:
            result = load_intrinsics_json(path)
        except Exception as exc:
            self.calibration_status_label.setText(f"비교 파일 적용 실패: {path} ({exc})")
            return
        zoom_level = self._intrinsics_file_zoom_level(path)
        self.last_calibration_result = result
        self.calibration_image_diagnostics = tuple(result.image_diagnostics)
        self.last_calibration_zoom_level = zoom_level
        self._candidate_intrinsics_profile_path = str(path)
        if self.apply_intrinsics_to_distance() is False:
            self._refresh_zoom_intrinsics_tab()
            self._refresh_operation_header()
            return
        if zoom_level is not None:
            self.zoom_level_spin.setValue(int(zoom_level))
            zoom_text = f"zoom {int(zoom_level):02d}"
        else:
            zoom_text = "zoom unknown"
        self.calibration_status_label.setText(f"Applied comparison intrinsics ({zoom_text}): {path}")
        self._log(f"[캘리브레이션] 비교 목록 intrinsic 파일을 적용했습니다 ({zoom_text}): {path}")
        self._refresh_zoom_intrinsics_tab()
        self._refresh_operation_header()

    def _add_zoom_intrinsics_comparison_path(self, path: Path) -> bool:
        if not hasattr(self, "zoom_intrinsics_compare_list"):
            return False
        resolved = path.expanduser()
        try:
            resolved = resolved.resolve()
        except OSError:
            pass
        if any(existing == resolved for existing in self.zoom_intrinsics_compare_paths):
            return False
        try:
            result = load_intrinsics_json(resolved)
        except Exception as exc:
            self.calibration_status_label.setText(f"비교 파일 로드 실패: {resolved} ({exc})")
            return False
        item = QtWidgets.QListWidgetItem(self._format_zoom_intrinsics_comparison_text(resolved, result))
        item.setData(QtCore.Qt.UserRole, str(resolved))
        item.setSizeHint(QtCore.QSize(0, 118))
        item.setFlags(item.flags() | QtCore.Qt.ItemIsDragEnabled | QtCore.Qt.ItemIsDropEnabled)
        self.zoom_intrinsics_compare_list.addItem(item)
        self.zoom_intrinsics_compare_paths.append(resolved)
        self.zoom_intrinsics_compare_list.clearSelection()
        self.zoom_intrinsics_compare_list.setCurrentItem(item)
        return True

    def _refresh_zoom_intrinsics_tab(self) -> None:
        if not hasattr(self, "zoom_left_intrinsics_text"):
            return
        result, status, source_label, path = self._current_zoom_intrinsics_result()
        zoom_level = self.zoom_level_spin.value()
        self.zoom_intrinsics_status_label.setText(f"줌 {zoom_level:02d} | 상태: {status}")
        path_text = f"파일: {path}" if path is not None else f"파일: {self._intrinsics_storage_path(zoom_level)}"
        self.zoom_intrinsics_path_label.setText(path_text)
        self.zoom_left_intrinsics_text.setPlainText(
            self._format_intrinsics_text("left", None if result is None else result.left, status, source_label)
        )
        self.zoom_right_intrinsics_text.setPlainText(
            self._format_intrinsics_text("right", None if result is None else result.right, status, source_label)
        )

    def _format_live_pixhawk_extrinsic_text(
        self,
        camera: str,
        extrinsic: CameraExtrinsicState | None,
        *,
        title_prefix: str = "",
    ) -> str:
        title = f"{title_prefix}{'Left Live Extrinsic' if camera == 'left' else 'Right Live Extrinsic'}"
        if extrinsic is None:
            return (
                f"{title}\n"
                "status: waiting\n\n"
                f"source: {self._extrinsics_source_label(None)}\n\n"
                "실시간 카메라 자세를 보려면 MAVLink로 픽스호크를 연결하세요."
            )
        age_ms = max(0.0, time.monotonic() - extrinsic.timestamp_s) * 1000.0
        px4_timestamp = "n/a" if extrinsic.px4_timestamp_s is None else f"{extrinsic.px4_timestamp_s:.6f}"
        return (
            f"{title}\n"
            f"status: live\n"
            f"source: {self._extrinsics_source_label(extrinsic)}\n"
            f"age: {age_ms:.0f} ms\n"
            f"px4_timestamp: {px4_timestamp}\n\n"
            f"roll:  {extrinsic.roll_deg:.6f} deg\n"
            f"pitch: {extrinsic.pitch_deg:.6f} deg\n"
            f"yaw:   {extrinsic.yaw_deg:.6f} deg\n\n"
            "quaternion q(w,x,y,z):\n"
            + ", ".join(f"{value:.9g}" for value in extrinsic.q)
        )

    def _format_checkerboard_effective_extrinsics_text(
        self,
        camera: str,
        live_extrinsic: CameraExtrinsicState | None,
    ) -> str:
        result = self.active_intrinsics
        extrinsics = None if result is None else result.extrinsics
        title = "Left Effective Extrinsic" if camera == "left" else "Right Effective Extrinsic"
        live_source = self._extrinsics_source_label(live_extrinsic)
        diagnostic = (
            "\n\nLive Pixhawk diagnostic only\n"
            f"source: {live_source}\n"
            "used_for_triangulation: false"
        )
        if extrinsics is None:
            return (
                f"{title}\n"
                "mode: checkerboard stereo R/T\n"
                "status: waiting for applied checkerboard stereo extrinsics\n"
                "used_for_triangulation: pending\n"
                f"frame_contract: {CAMERA_FRAME}\n"
                f"{diagnostic}"
            )
        rotation_right_to_left, translation_right_in_left_m = invert_left_to_right_camera_transform(
            extrinsics.rotation_left_to_right,
            extrinsics.translation_left_to_right_m,
        )
        if camera == "left":
            return (
                f"{title}\n"
                "mode: checkerboard stereo R/T\n"
                "status: effective reference camera\n"
                "source: applied checkerboard stereo extrinsics\n"
                "used_for_triangulation: true\n"
                f"frame_contract: {extrinsics.frame_contract}\n"
                "origin_in_left_rdf_m: 0.000000000, 0.000000000, 0.000000000\n"
                "rotation_left_to_left:\n"
                + self._format_matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
                + diagnostic
            )
        baseline_m = math.sqrt(sum(value * value for value in translation_right_in_left_m))
        return (
            f"{title}\n"
            "mode: checkerboard stereo R/T\n"
            "status: effective right camera in left RDF\n"
            "source: applied checkerboard stereo extrinsics\n"
            "used_for_triangulation: true\n"
            f"frame_contract: {extrinsics.frame_contract}\n"
            f"baseline_m: {baseline_m:.9f}\n"
            "translation_right_in_left_m: "
            + ", ".join(f"{value:.9f}" for value in translation_right_in_left_m)
            + "\n"
            "rotation_right_to_left:\n"
            + self._format_matrix(rotation_right_to_left)
            + diagnostic
        )

    def _format_extrinsics_text(self, camera: str, extrinsic: CameraExtrinsicState | None) -> str:
        if self._distance_mode() == "checkerboard":
            return self._format_checkerboard_effective_extrinsics_text(camera, extrinsic)
        return self._format_live_pixhawk_extrinsic_text(camera, extrinsic)

    def _format_stereo_geometry_text(self) -> str:
        active = self.active_intrinsics
        if active is not None:
            intrinsics_state = "active"
        elif self.last_calibration_result is not None:
            intrinsics_state = "latest loaded/calculated, not applied"
        else:
            intrinsics_state = "not loaded"
        baseline_m, baseline_source, warnings = self._active_baseline_m()
        translation = self._latest_translation_right_in_left_m_for_diagnostics()
        translation_text = "대기 중"
        if translation is not None:
            translation_text = f"{translation[0]:.6f}, {translation[1]:.6f}, {translation[2]:.6f} m"
        rtk = self.latest_rtk_baseline
        rtk_text = "대기 중"
        if rtk is not None:
            rtk_text = (
                f"{rtk.baseline_m:.6f} m, N/E/D={rtk.north_m:.6f}/"
                f"{rtk.east_m:.6f}/{rtk.down_m:.6f} m, quality={rtk.fix_quality}"
            )
        warning_text = "none" if not warnings else "; ".join(warnings)
        return (
            "Stereo Geometry\n"
            f"intrinsics: {intrinsics_state}\n"
            f"intrinsics_source: {self._intrinsics_source_label()}\n"
            f"distance_method: {self._measurement_path_label()}\n"
            f"distance_mode: {self._distance_mode_label()}\n"
            "use_3d_distance: true\n"
            f"pose_source: {self._triangulation_extrinsics_source_label()}\n"
            f"world_frame: {WORLD_FRAME}\n"
            f"body_frame: {BODY_FRAME}\n"
            f"camera_frame: {CAMERA_FRAME}\n"
            f"camera_mount_profile: {CAMERA_MOUNT_PROFILE}\n"
            "mount_assumption_model: gps-rtk-baseline-only-v1\n"
            f"mount_assumption_status: {self._mount_assumption_status_label()}\n"
            f"manual_focal_reference: {self.focal_spin.value():.6f} px\n"
            f"focal_source: {self._focal_source_label()}\n"
            f"baseline_used: {baseline_m:.6f} m\n"
            f"baseline_source: {baseline_source}\n"
            f"baseline_source_detail: {self._baseline_source_label(baseline_source, baseline_m)}\n"
            f"parameter_summary: {self._distance_parameters_summary()}\n"
            f"baseline_warnings: {warning_text}\n\n"
            f"rtk_baseline: {rtk_text}\n"
            f"translation_right_in_left: {translation_text}\n"
            f"{self._format_checkerboard_stereo_extrinsics_summary()}\n\n"
            f"{self._format_mount_assumption_summary()}\n\n"
            f"max_detection_dt: {self._yolo_pair_max_delta_s():.6f} s\n"
            f"max_vertical_delta: {self._yolo_vertical_max_delta_px():.6f} px\n"
            "same_class_required: true (stereo correspondence invariant)\n"
            f"target_class_gate_enabled: {self.yolo_gate_class_checkbox.isChecked()}\n"
            f"target_class_gate: {self.yolo_gate_class_edit.text().strip() or 'none'}\n"
            "attitude_gate: false\n"
            f"attitude_timing_policy: {ATTITUDE_TIMING_POLICY}\n"
            f"attitude_stale_age_threshold_s: {ATTITUDE_STALE_AGE_S:.3f}\n"
            f"attitude_very_stale_age_threshold_s: {ATTITUDE_VERY_STALE_AGE_S:.3f}\n"
            f"attitude_stale_pair_delta_threshold_s: {ATTITUDE_STALE_PAIR_DELTA_S:.3f}\n"
            f"attitude_very_stale_pair_delta_threshold_s: {ATTITUDE_VERY_STALE_PAIR_DELTA_S:.3f}"
        )

    def _format_checkerboard_stereo_extrinsics_summary(self) -> str:
        result = self.active_intrinsics or self.last_calibration_result
        extrinsics = None if result is None else result.extrinsics
        if extrinsics is None:
            return "checkerboard_stereo_extrinsics: not computed"
        baseline_m = math.sqrt(sum(value * value for value in extrinsics.translation_left_to_right_m))
        rotation_rows = [
            "  [" + ", ".join(f"{value: .9f}" for value in row) + "]"
            for row in extrinsics.rotation_left_to_right
        ]
        return (
            "checkerboard_stereo_extrinsics: computed\n"
            "checkerboard_stereo_usage: selectable live triangulation source in parameter calculation settings\n"
            f"checkerboard_frame_contract: {extrinsics.frame_contract}\n"
            f"checkerboard_pair_count: {extrinsics.pair_count}\n"
            f"checkerboard_reprojection_error_px: {extrinsics.reprojection_error_px:.6f}\n"
            f"checkerboard_translation_left_to_right_m: "
            + ", ".join(f"{value:.9f}" for value in extrinsics.translation_left_to_right_m)
            + "\n"
            f"checkerboard_baseline_m: {baseline_m:.9f}\n"
            "checkerboard_rotation_left_to_right:\n"
            + "\n".join(rotation_rows)
        )

    def _refresh_camera_parameters_tab(self) -> None:
        if not hasattr(self, "left_intrinsics_text"):
            return
        self._update_distance_settings_visibility()
        result = self.active_intrinsics or self.last_calibration_result
        status = "active" if self.active_intrinsics is not None else "latest loaded/calculated, not applied"
        self.left_intrinsics_text.setPlainText(self._format_intrinsics_text("left", None if result is None else result.left, status))
        self.right_intrinsics_text.setPlainText(self._format_intrinsics_text("right", None if result is None else result.right, status))
        self.left_extrinsics_text.setPlainText(self._format_extrinsics_text("left", self.latest_extrinsics.get("left")))
        self.right_extrinsics_text.setPlainText(self._format_extrinsics_text("right", self.latest_extrinsics.get("right")))
        self.stereo_geometry_text.setPlainText(self._format_stereo_geometry_text())
        self._refresh_distance_parameters_label()
        self._refresh_zoom_intrinsics_tab()

    def build_geometry_settings(self, stereo_layout):
        stereo = QtWidgets.QGroupBox("3D 거리 계산 설정")
        stereo_form = QtWidgets.QGridLayout(stereo)
        stereo_form.setHorizontalSpacing(8)
        stereo_form.setColumnStretch(1, 1)
        stereo_form.setColumnStretch(3, 1)
        stereo_form.setColumnStretch(5, 1)
        self.focal_spin = QtWidgets.QDoubleSpinBox()
        self.focal_spin.setRange(1.0, MAX_MANUAL_FOCAL_PX)
        self.focal_spin.setValue(900.0)
        self.focal_spin.setSuffix(" px")
        self.focal_spin.valueChanged.connect(lambda _value: self._refresh_camera_parameters_tab())
        self.manual_focal_checkbox = self._build_toggle_button(
            "수동 초점거리 사용",
            self.settings.manual_focal_override,
        )
        self.manual_focal_checkbox.setToolTip(
            "체커보드가 없을 때 추정 fx(px)를 직접 입력합니다.\n"
            "체커보드 보정값을 쓰려면 이 옵션을 끄고 보정 결과를 적용하세요.\n\n"
            + WIRIS_VISIBLE_ZOOM_FX_TOOLTIP
        )
        self.manual_focal_checkbox.toggled.connect(self._set_focal_override_enabled)
        self.intrinsics_source_combo = QtWidgets.QComboBox()
        self.intrinsics_source_combo.addItem("체커보드 보정값", "checkerboard")
        self.intrinsics_source_combo.addItem("수동 fallback", "manual")
        self._set_combo_data(
            self.intrinsics_source_combo,
            "manual" if self.settings.manual_focal_override else "checkerboard",
        )
        self.intrinsics_source_combo.currentIndexChanged.connect(self._handle_intrinsics_source_changed)
        self.intrinsics_status_label = QtWidgets.QLabel("내부 파라미터: 대기 중")
        self.intrinsics_status_label.setObjectName("statusStrip")
        self.intrinsics_status_label.setWordWrap(True)
        self.baseline_spin = QtWidgets.QDoubleSpinBox()
        self.baseline_spin.setRange(0.001, 100.0)
        self.baseline_spin.setDecimals(3)
        self.baseline_spin.setValue(0.300)
        self.baseline_spin.setSuffix(" m")
        self.baseline_spin.valueChanged.connect(lambda _value: self._refresh_camera_parameters_tab())
        self.gps_baseline_spins: dict[str, dict[str, QtWidgets.QDoubleSpinBox]] = {}
        self.distance_label = QtWidgets.QLabel("거리: 좌/우 검출 매칭 대기 중")
        self.distance_label.setObjectName("distanceStatus")
        self.distance_label.setWordWrap(True)
        self.distance_mode_combo = QtWidgets.QComboBox()
        self.distance_mode_combo.setObjectName("distanceModeCombo")
        self.distance_mode_combo.setAccessibleName("외부 파라미터 계산 방식")
        self.distance_mode_combo.setToolTip(
            "3D ray 거리 계산에 사용할 외부 파라미터 출처를 고릅니다.\n"
            "Pixhawk 자세 + GPS/RTK baseline 또는 체커보드 stereo R/T를 사용합니다."
        )
        self.distance_mode_combo.addItem("Pixhawk 자세 + GPS/RTK baseline", "pixhawk")
        self.distance_mode_combo.addItem("체커보드 Stereo 외부 파라미터", "checkerboard")
        initial_distance_mode = str(self.settings.triangulation_extrinsics_source or "pixhawk")
        if initial_distance_mode == "calibration":
            initial_distance_mode = "checkerboard"
        if initial_distance_mode not in {"pixhawk", "checkerboard"}:
            initial_distance_mode = "pixhawk"
        self.distance_mode_combo.setCurrentIndex(self.distance_mode_combo.findData(initial_distance_mode))
        self._distance_mode_last_source = initial_distance_mode
        self.distance_mode_combo.currentIndexChanged.connect(self._handle_distance_mode_changed)
        self.distance_parameters_label = QtWidgets.QLabel("계산 파라미터: 대기 중")
        self.distance_parameters_label.setObjectName("statusStrip")
        self.distance_parameters_label.setWordWrap(True)
        self.camera_sync_history_window_spin = QtWidgets.QDoubleSpinBox()
        self.camera_sync_history_window_spin.setRange(0.1, 5.0)
        self.camera_sync_history_window_spin.setSingleStep(0.1)
        self.camera_sync_history_window_spin.setDecimals(1)
        self.camera_sync_history_window_spin.setValue(self.settings.camera_sync_history_window_s)
        self.camera_sync_history_window_spin.setSuffix(" s")
        self.camera_sync_receive_pair_max_delta_spin = QtWidgets.QDoubleSpinBox()
        self.camera_sync_receive_pair_max_delta_spin.setRange(0.0, 2000.0)
        self.camera_sync_receive_pair_max_delta_spin.setSingleStep(25.0)
        self.camera_sync_receive_pair_max_delta_spin.setDecimals(0)
        self.camera_sync_receive_pair_max_delta_spin.setValue(
            self.settings.camera_sync_receive_pair_max_delta_s * 1000.0
        )
        self.camera_sync_receive_pair_max_delta_spin.setSuffix(" ms")
        self.camera_sync_rtcp_pair_max_delta_spin = QtWidgets.QDoubleSpinBox()
        self.camera_sync_rtcp_pair_max_delta_spin.setRange(0.0, 1000.0)
        self.camera_sync_rtcp_pair_max_delta_spin.setSingleStep(5.0)
        self.camera_sync_rtcp_pair_max_delta_spin.setDecimals(0)
        self.camera_sync_rtcp_pair_max_delta_spin.setValue(
            self.settings.camera_sync_rtcp_pair_max_delta_s * 1000.0
        )
        self.camera_sync_rtcp_pair_max_delta_spin.setSuffix(" ms")
        self.camera_sync_rtcp_pair_max_delta_spin.setToolTip(
            "Left/Right RTCP clock 상대 차이의 성공/유지 기준입니다. 기본 100 ms입니다."
        )
        self.camera_sync_max_sr_age_spin = QtWidgets.QDoubleSpinBox()
        self.camera_sync_max_sr_age_spin.setRange(0.1, 60.0)
        self.camera_sync_max_sr_age_spin.setSingleStep(0.5)
        self.camera_sync_max_sr_age_spin.setDecimals(1)
        self.camera_sync_max_sr_age_spin.setValue(self.settings.camera_sync_max_sr_age_s)
        self.camera_sync_max_sr_age_spin.setSuffix(" s")
        self.camera_sync_max_pc_offset_spin = QtWidgets.QDoubleSpinBox()
        self.camera_sync_max_pc_offset_spin.setRange(0.0, 5000.0)
        self.camera_sync_max_pc_offset_spin.setSingleStep(10.0)
        self.camera_sync_max_pc_offset_spin.setDecimals(0)
        self.camera_sync_max_pc_offset_spin.setValue(self.settings.camera_sync_max_pc_offset_s * 1000.0)
        self.camera_sync_max_pc_offset_spin.setSuffix(" ms")
        self.camera_sync_max_pc_offset_spin.setToolTip(
            "SR-PC 절대값을 강조 표시할 진단 기준입니다. STUS 성공/실패, VERIFIED, Auto 또는 pair gate에 사용하지 않습니다."
        )
        self.camera_sync_clock_rate_error_spin = QtWidgets.QDoubleSpinBox()
        self.camera_sync_clock_rate_error_spin.setRange(0.0, 10.0)
        self.camera_sync_clock_rate_error_spin.setSingleStep(0.1)
        self.camera_sync_clock_rate_error_spin.setDecimals(2)
        self.camera_sync_clock_rate_error_spin.setValue(self.settings.camera_sync_max_clock_rate_error * 100.0)
        self.camera_sync_clock_rate_error_spin.setSuffix(" %")
        self.camera_sync_required_sr_count_spin = QtWidgets.QSpinBox()
        self.camera_sync_required_sr_count_spin.setRange(1, 20)
        self.camera_sync_required_sr_count_spin.setValue(self.settings.camera_sync_required_sr_count)
        self.camera_sync_required_sr_count_spin.setSuffix(" 회")
        self.camera_sync_warmup_timeout_spin = QtWidgets.QDoubleSpinBox()
        self.camera_sync_warmup_timeout_spin.setRange(1.0, 120.0)
        self.camera_sync_warmup_timeout_spin.setSingleStep(1.0)
        self.camera_sync_warmup_timeout_spin.setDecimals(0)
        self.camera_sync_warmup_timeout_spin.setValue(self.settings.camera_sync_warmup_timeout_s)
        self.camera_sync_warmup_timeout_spin.setSuffix(" s")
        self.camera_sync_mode_combo = QtWidgets.QComboBox()
        self.camera_sync_mode_combo.addItem("Manual", "manual")
        self.camera_sync_mode_combo.addItem("Auto", "auto")
        self._set_combo_data(
            self.camera_sync_mode_combo,
            getattr(self.settings, "camera_sync_mode", "manual"),
        )
        self.camera_sync_interval_spin = QtWidgets.QSpinBox()
        self.camera_sync_interval_spin.setRange(5, 1440)
        self.camera_sync_interval_spin.setValue(
            max(5, min(1440, round(float(getattr(self.settings, "camera_sync_interval_s", 1800.0)) / 60.0)))
        )
        self.camera_sync_interval_spin.setSuffix(" min")
        self.camera_sync_settings_group = QtWidgets.QGroupBox("카메라 RTCP 동기화")
        self.camera_sync_settings_group.setObjectName("cameraSyncSettingsGroup")
        camera_sync_settings_form = QtWidgets.QGridLayout(self.camera_sync_settings_group)
        camera_sync_settings_form.setHorizontalSpacing(8)
        camera_sync_settings_form.setVerticalSpacing(6)
        camera_sync_settings_form.setColumnStretch(1, 1)
        camera_sync_settings_form.setColumnStretch(3, 1)
        camera_sync_fields = (
            ("STUS mode", self.camera_sync_mode_combo),
            ("Auto interval", self.camera_sync_interval_spin),
            ("Frame history", self.camera_sync_history_window_spin),
            ("수신 skew 허용", self.camera_sync_receive_pair_max_delta_spin),
            ("좌우 RTCP/NTP Δt gate", self.camera_sync_rtcp_pair_max_delta_spin),
            ("SR stale", self.camera_sync_max_sr_age_spin),
            ("SR-PC 진단 기준(gate 아님)", self.camera_sync_max_pc_offset_spin),
            ("Clock rate 오차", self.camera_sync_clock_rate_error_spin),
            ("최소 SR 수", self.camera_sync_required_sr_count_spin),
            ("Warmup timeout", self.camera_sync_warmup_timeout_spin),
        )
        for index, (label_text, widget) in enumerate(camera_sync_fields):
            row = index // 2
            column = (index % 2) * 2
            label = QtWidgets.QLabel(label_text)
            label.setBuddy(widget)
            widget.setAccessibleName(label_text)
            camera_sync_settings_form.addWidget(label, row, column)
            camera_sync_settings_form.addWidget(widget, row, column + 1)
        self.current_parameters_group = QtWidgets.QGroupBox("현재 적용값")
        current_parameters_layout = QtWidgets.QVBoxLayout(self.current_parameters_group)
        current_parameters_layout.addWidget(self.distance_parameters_label)

        self.intrinsics_settings_group = QtWidgets.QGroupBox("기본: 내부 파라미터")
        intrinsics_form = QtWidgets.QGridLayout(self.intrinsics_settings_group)
        intrinsics_form.setHorizontalSpacing(8)
        intrinsics_form.setColumnStretch(1, 1)
        self.manual_focal_label = QtWidgets.QLabel("초점거리")
        intrinsics_form.addWidget(QtWidgets.QLabel("출처"), 0, 0)
        intrinsics_form.addWidget(self.intrinsics_source_combo, 0, 1)
        intrinsics_form.addWidget(self.manual_focal_label, 1, 0)
        intrinsics_form.addWidget(self.focal_spin, 1, 1)
        intrinsics_form.addWidget(self.intrinsics_status_label, 2, 0, 1, 2)
        self.manual_focal_checkbox.setVisible(False)
        self._configure_collapsible_group(self.intrinsics_settings_group, checked=True)

        self.extrinsics_settings_group = QtWidgets.QGroupBox("기본: 외부 파라미터 / Baseline")
        extrinsics_form = QtWidgets.QGridLayout(self.extrinsics_settings_group)
        extrinsics_form.setHorizontalSpacing(8)
        extrinsics_form.setColumnStretch(1, 1)
        self.distance_mode_label = QtWidgets.QLabel("계산 방식")
        self.baseline_label = QtWidgets.QLabel("Baseline")
        self.mount_assumption_label = QtWidgets.QLabel(
            "장착 가정: GPS/RTK baseline only. "
            "수동 baseline 입력은 사용하지 않고 좌/우 Pixhawk GPS baseline과 자세만 계산에 사용합니다."
        )
        self.mount_assumption_label.setObjectName("statusStrip")
        self.mount_assumption_label.setWordWrap(True)
        extrinsics_form.addWidget(self.distance_mode_label, 0, 0)
        extrinsics_form.addWidget(self.distance_mode_combo, 0, 1)
        self.baseline_label.setVisible(False)
        self.baseline_spin.setVisible(False)
        extrinsics_form.addWidget(self.mount_assumption_label, 1, 0, 1, 2)

        stereo_parameter_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        stereo_parameter_splitter.setObjectName("stereoParameterSplitter")
        stereo_parameter_splitter.setChildrenCollapsible(False)
        stereo_parameter_splitter.setHandleWidth(8)
        stereo_parameter_splitter.setOpaqueResize(True)
        self.stereo_parameter_splitter = stereo_parameter_splitter

        intrinsics_panel = QtWidgets.QWidget()
        intrinsics_panel_layout = QtWidgets.QVBoxLayout(intrinsics_panel)
        intrinsics_panel_layout.setContentsMargins(0, 0, 0, 0)
        intrinsics_panel_layout.setSpacing(0)
        intrinsics_panel_layout.addWidget(self.intrinsics_settings_group)

        extrinsics_panel = QtWidgets.QWidget()
        extrinsics_panel_layout = QtWidgets.QVBoxLayout(extrinsics_panel)
        extrinsics_panel_layout.setContentsMargins(0, 0, 0, 0)
        extrinsics_panel_layout.setSpacing(0)
        extrinsics_panel_layout.addWidget(self.extrinsics_settings_group)

        stereo_parameter_splitter.addWidget(intrinsics_panel)
        stereo_parameter_splitter.addWidget(extrinsics_panel)
        stereo_parameter_splitter.setStretchFactor(0, 1)
        stereo_parameter_splitter.setStretchFactor(1, 1)
        stereo_parameter_splitter.setSizes([1, 1])

        stereo_form.addWidget(self.current_parameters_group, 0, 0, 1, 6)
        stereo_form.addWidget(stereo_parameter_splitter, 1, 0, 1, 6)
        stereo_form.addWidget(self.distance_label, 2, 0, 1, 6)
        for widget in (
            self.camera_sync_history_window_spin,
            self.camera_sync_receive_pair_max_delta_spin,
            self.camera_sync_rtcp_pair_max_delta_spin,
            self.camera_sync_max_sr_age_spin,
            self.camera_sync_max_pc_offset_spin,
            self.camera_sync_clock_rate_error_spin,
            self.camera_sync_required_sr_count_spin,
            self.camera_sync_warmup_timeout_spin,
        ):
            widget.valueChanged.connect(lambda _value: self._refresh_distance())
        stereo_layout.addWidget(stereo)
        stereo_layout.addStretch(1)

    def build_parameter_views(self, extrinsic_parameters_layout, intrinsic_parameters_layout):
        intrinsic_group = QtWidgets.QGroupBox("내부 파라미터")
        intrinsic_layout = QtWidgets.QHBoxLayout(intrinsic_group)
        self.left_intrinsics_text = self._build_parameter_text()
        self.right_intrinsics_text = self._build_parameter_text()
        intrinsic_layout.addWidget(self.left_intrinsics_text, 1)
        intrinsic_layout.addWidget(self.right_intrinsics_text, 1)
        extrinsic_group = QtWidgets.QGroupBox("외부 파라미터 / 스테레오 기하")
        extrinsic_layout = QtWidgets.QHBoxLayout(extrinsic_group)
        self.left_extrinsics_text = self._build_parameter_text()
        self.right_extrinsics_text = self._build_parameter_text()
        self.stereo_geometry_text = self._build_parameter_text()
        extrinsic_layout.addWidget(self.left_extrinsics_text, 1)
        extrinsic_layout.addWidget(self.right_extrinsics_text, 1)
        extrinsic_layout.addWidget(self.stereo_geometry_text, 1)
        intrinsic_parameters_layout.addWidget(intrinsic_group, 1)
        extrinsic_parameters_layout.addWidget(extrinsic_group, 1)

    def build_measurement_quality(self, triangulation_quality_layout):
        quality_summary_group = QtWidgets.QGroupBox("현재 측정 품질")
        quality_summary_form = QtWidgets.QGridLayout(quality_summary_group)
        quality_summary_form.setHorizontalSpacing(8)
        quality_summary_form.setColumnStretch(1, 1)
        quality_summary_form.setColumnStretch(3, 1)
        self.triangulation_quality_status_label = QtWidgets.QLabel("3D ray triangulation 대기 중")
        self.triangulation_quality_status_label.setObjectName("statusStrip")
        self.triangulation_quality_status_label.setWordWrap(True)
        self.triangulation_quality_status_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.triangulation_quality_gap_label = QtWidgets.QLabel("gap: --")
        self.triangulation_quality_distance_label = QtWidgets.QLabel("distance: --")
        self.triangulation_quality_mode_label = QtWidgets.QLabel("mode: --")
        self.triangulation_quality_attitude_label = QtWidgets.QLabel("Attitude timing: --")
        self.triangulation_quality_attitude_label.setObjectName("statusStrip")
        self.triangulation_quality_attitude_label.setWordWrap(True)
        self.triangulation_quality_attitude_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.triangulation_quality_attitude_label.setToolTip(
            "표시 전용 비차단 진단: sample age > 2 s 또는 pair Δt > 1 s이면 STALE, "
            "sample age > 10 s 또는 pair Δt > 5 s이면 VERY_STALE입니다."
        )
        for label in (
            self.triangulation_quality_gap_label,
            self.triangulation_quality_distance_label,
            self.triangulation_quality_mode_label,
        ):
            label.setObjectName("statChip")
            label.setAlignment(QtCore.Qt.AlignCenter)
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        quality_summary_form.addWidget(self.triangulation_quality_status_label, 0, 0, 1, 4)
        quality_summary_form.addWidget(self.triangulation_quality_gap_label, 1, 0)
        quality_summary_form.addWidget(self.triangulation_quality_distance_label, 1, 1)
        quality_summary_form.addWidget(self.triangulation_quality_mode_label, 1, 2, 1, 2)
        quality_summary_form.addWidget(self.triangulation_quality_attitude_label, 2, 0, 1, 4)
        triangulation_quality_layout.addWidget(quality_summary_group)

        self.triangulation_quality_plot = Triangulation3DView()
        triangulation_quality_layout.addWidget(self.triangulation_quality_plot, 1)
        point_group = QtWidgets.QGroupBox("3D point")
        point_layout = QtWidgets.QGridLayout(point_group)
        point_layout.setColumnStretch(1, 1)
        self.triangulation_quality_raw_point_label = QtWidgets.QLabel("Raw baseline_center_frd point: --")
        self.triangulation_quality_raw_point_label.setObjectName("statusStrip")
        self.triangulation_quality_raw_point_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        point_layout.addWidget(self.triangulation_quality_raw_point_label, 0, 0, 1, 2)
        triangulation_quality_layout.addWidget(point_group)
        self._clear_triangulation_quality("3D ray triangulation 대기 중")

