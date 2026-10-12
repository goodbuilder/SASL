"""Detection feature: owned state/views and explicit dependency ports.

Coordinate values retain their producer contracts (NED, body FRD, optical RDF,
and baseline-center FRD). This ownership change performs no axis conversion.
"""
from __future__ import annotations

from Workstation.panel.panel_feature_runtime import PanelFeature
import math
from PyQt5 import (
    QtWidgets,
)
from Workstation.panel.panel_support import YOLO_CPU_DEVICE, YOLO_CUDA_DEVICE, decoder_mode_for_side
from Workstation.runtime_paths import (
    import_workstation_asset_file,
    portable_workstation_data_path_text,
    resolve_workstation_data_path,
)
from Workstation.workstation_core.settings import (
    normalize_live_distance_time_basis,
)
from Workstation.workstation_core.stereo import (
    DetectionObservation,
    detection_timestamp_for_basis,
)
from Workstation.workstation_core.sync import (
    TimedSampleBuffer,
)
from Workstation.workstation_core.media_stream.policy import DEFAULT_PREVIEW_DISPLAY_RATE_HZ, DEFAULT_YOLO_WEIGHTS
from Workstation.workstation_core.media_stream.configuration import default_yolo_max_rate_hz
from pathlib import (
    Path,
)


class DetectionFeature(PanelFeature):
    feature_name = 'detection'
    dependency_names = (
        '_camera_sync_transport_override',
        '_coordinated_pairing_result',
        '_log',
        '_measurement_view_controller',
        '_runtime_decoder_overrides',
        '_workstation_setting',
        'app_root',
        'camera_connected',
        'decoder_combo',
        'device_edit',
        'preview_width_spin',
        'rtsp_latency_spin',
        'rtsp_transport_combo',
        'settings',
        'stream_backend_combo',
    )
    writable_dependencies = frozenset(())

    def initialize_state(self, *, auto_start=False, auto_start_pixhawk=False, app_root=None, settings_path=None):
        self.latest_left: DetectionObservation | None = None
        self.latest_right: DetectionObservation | None = None
        self.detection_buffers = {
            "left": TimedSampleBuffer[DetectionObservation](maxlen=120),
            "right": TimedSampleBuffer[DetectionObservation](maxlen=120),
        }
        self.yolo_enabled = False
        self._last_correspondence_rejection_reason = ""

    def _set_yolo_device_combo(self, device: str) -> None:
        index = self.device_combo.findData(self._normalized_yolo_device(device))
        if index >= 0:
            self.device_combo.setCurrentIndex(index)

    def _yolo_device_uses_cuda(self, value: str) -> bool:
        return self._normalized_yolo_device(value) == YOLO_CUDA_DEVICE

    def _normalized_yolo_device(self, value: str) -> str:
        normalized = value.strip().lower()
        if normalized in {"cuda", "cuda:0", "0", "gpu"}:
            return YOLO_CUDA_DEVICE
        return YOLO_CPU_DEVICE

    def _refresh_yolo_device_toggle(self, _checked: bool | None = None) -> None:
        if not hasattr(self, "device_combo"):
            return
        device = self._normalized_yolo_device(str(self.device_combo.currentData() or YOLO_CPU_DEVICE))
        self.device_edit.setText(device)
        self.device_combo.setAccessibleName(f"추론 장치: {device}")

    def _refresh_yolo_gate_controls(self) -> None:
        if not hasattr(self, "yolo_gate_class_checkbox"):
            return
        class_enabled = self.yolo_gate_class_checkbox.isChecked()
        self.yolo_gate_matching_checkbox.setChecked(True)
        self.yolo_gate_time_delta_checkbox.setChecked(True)
        time_basis_enabled = True
        time_delta_enabled = True
        vertical_delta_enabled = self.yolo_gate_vertical_delta_checkbox.isChecked()
        class_target = self.yolo_gate_class_edit.text().strip()
        self.yolo_gate_class_edit.setEnabled(class_enabled)
        self.yolo_gate_time_basis_combo.setEnabled(True)
        self.yolo_gate_matching_checkbox.setEnabled(False)
        self.yolo_gate_time_delta_checkbox.setEnabled(False)
        self.yolo_gate_time_delta_spin.setEnabled(time_delta_enabled)
        self.yolo_gate_vertical_delta_spin.setEnabled(vertical_delta_enabled)
        enabled = []
        if class_enabled:
            enabled.append("class" if class_target else "class(target missing)")
        if time_basis_enabled:
            enabled.append("time-basis")
        if time_delta_enabled:
            enabled.append("time-delta")
        if vertical_delta_enabled:
            enabled.append("vertical-delta")
        requested_basis = self._yolo_time_basis()
        result = getattr(self, "_coordinated_pairing_result", None)
        timing = getattr(result, "timing", None)
        effective_basis = (
            str(timing.effective_basis)
            if timing is not None
            else requested_basis
        )
        requested_text = "RTCP/NTP" if requested_basis == "camera_rtcp" else "APP RECEIVE"
        effective_text = "RTCP/NTP" if effective_basis == "camera_rtcp" else "APP RECEIVE"
        fallback_text = (
            f" · fallback={timing.fallback_reason}"
            if timing is not None and timing.fallback_reason
            else ""
        )
        text = (
            f"Pair timing: requested={requested_text} · effective={effective_text}"
            f"{fallback_text} · max 100 ms"
            + (" · " + ", ".join(enabled) if enabled else "")
        )
        self.yolo_gate_status_label.setText(text)

    def _yolo_matching_gate_enabled(self) -> bool:
        return bool(
            hasattr(self, "yolo_gate_matching_checkbox")
            and self.yolo_gate_matching_checkbox.isChecked()
        )

    def _yolo_time_delta_gate_enabled(self) -> bool:
        return bool(
            hasattr(self, "yolo_gate_time_delta_checkbox")
            and self.yolo_gate_time_delta_checkbox.isChecked()
        )

    def _yolo_vertical_delta_gate_enabled(self) -> bool:
        return bool(
            hasattr(self, "yolo_gate_vertical_delta_checkbox")
            and self.yolo_gate_vertical_delta_checkbox.isChecked()
        )

    def _yolo_pair_max_delta_s(self) -> float:
        if self._yolo_time_delta_gate_enabled():
            return self.yolo_gate_time_delta_spin.value()
        return float("inf")

    def _yolo_time_basis(self) -> str:
        owner = self._measurement_view_controller()
        return normalize_live_distance_time_basis(owner.requested_time_basis())

    def _yolo_match_timestamp_s(self, observation: DetectionObservation) -> float | None:
        return detection_timestamp_for_basis(observation, self._effective_yolo_time_basis())

    def _effective_yolo_time_basis(self) -> str:
        result = getattr(self, "_coordinated_pairing_result", None)
        timing = getattr(result, "timing", None)
        if timing is not None:
            return str(timing.effective_basis)
        return self._yolo_time_basis()

    def _camera_sync_gate_active(self) -> bool:
        """Require verified RTCP/NTP once a live camera stream is running."""

        return bool(self.camera_connected)

    def _camera_rtc_timestamp_s_from_metadata(self, metadata: dict) -> float | None:
        value = metadata.get("rtcp_frame_ntp_unix_s")
        if value is None:
            return None
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(parsed):
            return None
        return parsed

    def _selected_pair_sync_summary(self, left: DetectionObservation, right: DetectionObservation) -> tuple[str, float | None]:
        left_rtc_s = self._camera_rtc_timestamp_s_from_metadata(left.timestamp_metadata)
        right_rtc_s = self._camera_rtc_timestamp_s_from_metadata(right.timestamp_metadata)
        camera_rtc_delta_s = None
        if left_rtc_s is not None and right_rtc_s is not None:
            camera_rtc_delta_s = abs(left_rtc_s - right_rtc_s)
        basis = (
            "RTCP/NTP"
            if self._effective_yolo_time_basis() == "camera_rtcp"
            else "APP RECEIVE"
        )
        return basis, camera_rtc_delta_s

    def _yolo_vertical_max_delta_px(self) -> float:
        if self._yolo_vertical_delta_gate_enabled():
            return self.yolo_gate_vertical_delta_spin.value()
        return float("inf")

    def yolo_kwargs(self, camera: str | None = None) -> dict:
        weights_path = resolve_workstation_data_path(
            self.weights_edit.text().strip() or DEFAULT_YOLO_WEIGHTS
        )
        device = self.device_edit.text().strip()
        return {
            "weights_path": str(weights_path),
            "device": device,
            "input_size": self.input_size_spin.value(),
            "conf_threshold": self.conf_spin.value(),
            "iou_threshold": self.iou_spin.value(),
            "class_gate_enabled": self.yolo_gate_class_checkbox.isChecked(),
            "class_gate": self.yolo_gate_class_edit.text().strip(),
            "max_rate_hz": default_yolo_max_rate_hz(device),
            "display_rate_hz": DEFAULT_PREVIEW_DISPLAY_RATE_HZ,
            "preview_max_width": self.preview_width_spin.value(),
            "rtsp_latency_ms": self.rtsp_latency_spin.value(),
            "rtsp_encoded_queue_mode": str(
                self._workstation_setting(
                    "rtsp_encoded_queue_mode",
                    getattr(self.settings, "rtsp_encoded_queue_mode", "latest"),
                )
            ),
            "event_queue_size": self.yolo_event_queue_size_spin.value(),
            "frame_queue_size": self.yolo_frame_queue_size_spin.value(),
            "decoder": decoder_mode_for_side(
                self.decoder_combo.currentData(),
                self._runtime_decoder_overrides,
                camera,
            ),
            "stream_backend": self.stream_backend_combo.currentData(),
            "rtsp_reconnect_policy": str(
                self._workstation_setting(
                    "rtsp_reconnect_policy",
                    getattr(self.settings, "rtsp_reconnect_policy", "manual"),
                )
            ),
            "rtsp_transport": (
                self._camera_sync_transport_override
                or self.rtsp_transport_combo.currentData()
            ),
        }

    def browse_yolo_weights(self) -> None:
        current_path = resolve_workstation_data_path(
            self.weights_edit.text().strip() or DEFAULT_YOLO_WEIGHTS
        )
        start_dir = current_path.parent if current_path.parent.exists() else Path.home()
        selected, _filter = QtWidgets.QFileDialog.getOpenFileName(
            self.widget,
            "Select YOLO weights",
            str(start_dir),
            "Model weights (*.pt *.onnx *.engine *.torchscript);;All files (*)",
        )
        if not selected:
            return
        try:
            imported = import_workstation_asset_file(
                selected,
                category="models",
                data_root=self.app_root,
            )
            portable = portable_workstation_data_path_text(
                imported,
                data_root=self.app_root,
                field_name="yolo_weights_path",
            )
        except (OSError, ValueError) as exc:
            self._log(f"[YOLO] weights portable import 실패: {selected}: {exc}")
            return
        self.weights_edit.setText(portable)
        self._log(f"[YOLO] weights 파일을 portable data root로 가져왔습니다: {portable}")

