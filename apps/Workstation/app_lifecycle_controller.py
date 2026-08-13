"""Workstation startup and shutdown resource lifecycle controller."""

from __future__ import annotations
from PyQt5 import QtCore, QtWidgets

APP_LIFECYCLE_METHODS = [
    "_shutdown_save_settings",
    "_shutdown_stop_thread",
    "_shutdown_stop_auxiliary_workers",
    "_shutdown_stop_gnss_base_serial",
    "shutdown_for_close",
    "closeEvent",
    "_stop_calibration_preview_worker"
]


class AppLifecycleController:
    def __init__(self, panel):
        self.panel = panel

    def _shutdown_save_settings(self) -> bool:
        try:
            self.panel._save_current_settings()
        except Exception as exc:
            self.panel._log(f"[종료] 설정 저장 실패: {exc}")
            return False
        return True

    def _shutdown_stop_thread(self, attr: str) -> bool:
        thread = getattr(self.panel, attr, None)
        if thread is None:
            return True
        try:
            thread.quit()
            if thread.isRunning():
                return False
        except RuntimeError:
            pass
        if getattr(self.panel, attr, None) is thread:
            setattr(self.panel, attr, None)
        return True

    def _shutdown_stop_auxiliary_workers(self) -> bool:
        auto_capture_stopped = self.panel.stop_auto_calibration_capture()
        preview_stopped = self.panel._stop_calibration_preview_worker(wait=False)
        lrf_stopped = self.panel.stop_wiris_lrf_polling(wait=False)
        attrs = (
            "camera_auto_detect_thread",
            "wiris_control_check_thread",
            "wiris_command_thread",
            "wiris_zoom_thread",
            "wiris_time_sync_thread",
            "wiris_stip_thread",
            "camera_network_discovery_thread",
            "camera_network_apply_thread",
            "interface_status_thread",
            "pixhawk_usb_discovery_thread",
            "pixhawk_host_ip_discovery_thread",
            "rtsp_check_thread",
            "rtsp_enable_thread",
            "gnss_base_detect_thread",
            "checkerboard_calibration_thread",
        )
        results = [self.panel._shutdown_stop_thread(attr) for attr in attrs]
        return bool(auto_capture_stopped and preview_stopped and lrf_stopped and all(results))

    def _shutdown_stop_gnss_base_serial(self) -> bool:
        self.panel.disconnect_gnss_base_serial()
        thread = getattr(self.panel, "gnss_base_serial_thread", None)
        return thread is None or not thread.isRunning()

    def shutdown_for_close(self) -> bool:
        """Run the one close sequence shared by every top-level window."""

        coordinator = getattr(self.panel, "_shutdown_coordinator", None)
        if coordinator is None:
            return False
        return bool(coordinator.run())

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt override.
        if not self.panel.shutdown_for_close():
            event.ignore()
            QtCore.QTimer.singleShot(250, self.panel.close)
            return
        QtWidgets.QWidget.closeEvent(self.panel, event)

    def _stop_calibration_preview_worker(self, *_args, wait: bool = True) -> bool:
        thread = getattr(self.panel, "_calibration_preview_thread", None)
        if thread is None:
            return True
        thread.quit()
        if thread.isRunning():
            if not wait or not thread.wait(1000):
                return False
        self.panel._calibration_preview_thread = None
        self.panel._calibration_preview_worker = None
        return True
