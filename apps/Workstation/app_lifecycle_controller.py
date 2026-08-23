"""Workstation startup and shutdown resource lifecycle controller."""

from __future__ import annotations
import time
from PyQt5 import QtCore, QtWidgets


SHUTDOWN_DEADLINE_S = 5.0
SHUTDOWN_FORCE_WAIT_S = 1.0

APP_LIFECYCLE_METHODS = [
    "_shutdown_save_settings",
    "_shutdown_stop_thread",
    "_shutdown_stop_auxiliary_workers",
    "_shutdown_stop_gnss_base_serial",
    "_shutdown_stop_target_drone",
    "_shutdown_stop_drone_telemetry",
    "shutdown_for_close",
    "closeEvent",
    "_stop_calibration_preview_worker"
]


class AppLifecycleController:
    def __init__(self, panel):
        self.panel = panel

    def _shutdown_save_settings(self) -> bool:
        flusher = getattr(self.panel, "flush_workstation_run_evidence", None)
        if callable(flusher):
            try:
                flusher()
            except Exception as exc:
                self.panel._log(f"[종료] run evidence flush 실패: {exc}")
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
        measurement_owner = getattr(
            self.panel, "_measurement_view_controller_owner", None
        )
        measurement_workers_stopped = (
            True
            if measurement_owner is None
            else bool(measurement_owner.close_method_evaluation_workers())
        )
        native_store_closer = getattr(
            self.panel,
            "_shutdown_native_frame_stores",
            None,
        )
        native_stores_stopped = (
            True
            if not callable(native_store_closer)
            else bool(native_store_closer())
        )
        attrs = (
            "camera_auto_detect_thread",
            "wiris_control_check_thread",
            "wiris_command_thread",
            "pnp_photo_capture_thread",
            "wiris_zoom_thread",
            "wiris_time_sync_thread",
            "wiris_stip_thread",
            "camera_network_discovery_thread",
            "camera_network_apply_thread",
            "interface_status_thread",
            "pixhawk_usb_discovery_thread",
            "pixhawk_host_ip_discovery_thread",
            "drone_telemetry_usb_discovery_thread",
            "rtsp_check_thread",
            "rtsp_enable_thread",
            "gnss_base_detect_thread",
            "checkerboard_calibration_thread",
            "calibration_wiris_ftp_thread",
            "calibration_wiris_ftp_oz_thread",
            "calibration_wiris_oz_capture_thread",
            "calibration_wiris_oz_sync_thread",
            "calibration_wiris_oz_delete_thread",
            "calibration_wiris_oz_format_thread",
            "calibration_wiris_oz_gilc_thread",
            "wiris_image_format_thread",
            "optical_pose_wiris_ftp_thread",
            "optical_pose_wiris_ftp_oz_thread",
            "_measurement_media_verification_thread",
        )
        results = [self.panel._shutdown_stop_thread(attr) for attr in attrs]
        return bool(
            auto_capture_stopped
            and preview_stopped
            and lrf_stopped
            and measurement_workers_stopped
            and native_stores_stopped
            and all(results)
        )

    def _shutdown_stop_gnss_base_serial(self) -> bool:
        self.panel.disconnect_gnss_base_serial()
        thread = getattr(self.panel, "gnss_base_serial_thread", None)
        return thread is None or not thread.isRunning()

    def _shutdown_stop_target_drone(self) -> bool:
        owner = getattr(self.panel, "_target_drone_controller_owner", None)
        if owner is None:
            return True
        snapshot = owner.target_drone_stream_snapshot()
        if not snapshot.get("worker_alive") and snapshot.get("phase") in {
            "UNCONFIGURED",
            "CONFIGURED",
            "DEFERRED",
            "STOPPED",
            "SEALED",
            "FAILED",
        }:
            return True
        try:
            result = owner.stop_target_drone_stream(timeout_s=1.0)
        except Exception as exc:
            self.panel._log(f"[종료] Target Drone drain/stop 실패: {exc}")
            return False
        artifact = result.get("artifact") if isinstance(result, dict) else None
        if isinstance(artifact, dict):
            importer = getattr(self.panel, "import_workstation_session_artifact", None)
            if callable(importer) and not artifact.get("ledger_imported"):
                try:
                    importer(
                        artifact["absolute_path"],
                        role="target_drone_raw_mavlink_jsonl",
                        media_type="application/x-ndjson",
                        metadata={
                            key: value
                            for key, value in artifact.items()
                            if key != "absolute_path"
                        },
                    )
                except Exception as exc:
                    self.panel._log(f"[종료] Target Drone artifact linkage 실패: {exc}")
                    return False
        return not bool(result.get("worker_alive"))

    def _shutdown_stop_drone_telemetry(self) -> bool:
        owner = getattr(self.panel, "_drone_telemetry_controller_owner", None)
        if owner is not None:
            try:
                owner.disconnect_drone_telemetry()
            except Exception as exc:
                self.panel._log(f"[종료] Drone Telemetry 연결 해제 실패: {exc}")
                return False
        thread = getattr(self.panel, "drone_telemetry_connection_thread", None)
        return thread is None or not thread.isRunning()

    def shutdown_for_close(self) -> bool:
        """Run the one close sequence shared by every top-level window."""

        coordinator = getattr(self.panel, "_shutdown_coordinator", None)
        if coordinator is None:
            return False
        now_s = time.monotonic()
        started_s = getattr(self.panel, "_shutdown_started_monotonic_s", None)
        if started_s is None:
            started_s = now_s
            self.panel._shutdown_started_monotonic_s = started_s
        try:
            complete = bool(coordinator.run())
        except Exception as exc:  # noqa: BLE001 - shutdown must still reach its deadline.
            self.panel._log(f"[종료] lifecycle 단계 예외: {exc}")
            complete = False
        if complete:
            self.panel._shutdown_started_monotonic_s = None
            return True
        if now_s - float(started_s) < SHUTDOWN_DEADLINE_S:
            return False
        self._force_shutdown_after_deadline()
        self.panel._shutdown_started_monotonic_s = None
        return True

    def _force_shutdown_after_deadline(self) -> None:
        """Bound the final close even when one device QThread ignores quit()."""

        logger = getattr(self.panel, "_log", None)
        if callable(logger):
            logger(
                f"[종료] {SHUTDOWN_DEADLINE_S:.1f}초 deadline 초과; "
                "남은 worker를 강제 종료합니다."
            )
        flusher = getattr(self.panel, "flush_workstation_run_evidence", None)
        if callable(flusher):
            try:
                flusher()
            except Exception as exc:  # noqa: BLE001 - force close continues.
                if callable(logger):
                    logger(f"[종료] run evidence 최종 flush 실패: {exc}")

        for side in ("left", "right"):
            pane = getattr(self.panel, f"{side}_pane", None)
            worker = getattr(pane, "worker", None)
            for method_name in ("request_stop", "terminate_child_process", "kill_child_process"):
                method = getattr(worker, method_name, None)
                if callable(method):
                    try:
                        method()
                    except Exception:
                        pass

        threads: list[object] = []
        seen: set[int] = set()

        def add_thread(candidate: object | None) -> None:
            if candidate is None or id(candidate) in seen:
                return
            seen.add(id(candidate))
            threads.append(candidate)

        for name in dir(self.panel):
            if name.endswith("_thread"):
                try:
                    add_thread(getattr(self.panel, name, None))
                except Exception:
                    pass
        finder = getattr(self.panel, "findChildren", None)
        if callable(finder):
            try:
                for thread in finder(QtCore.QThread):
                    add_thread(thread)
            except (RuntimeError, TypeError):
                pass

        running: list[object] = []
        for thread in threads:
            try:
                if not thread.isRunning():
                    continue
                interrupter = getattr(thread, "requestInterruption", None)
                if callable(interrupter):
                    interrupter()
                thread.quit()
                terminator = getattr(thread, "terminate", None)
                if callable(terminator):
                    terminator()
                running.append(thread)
            except (AttributeError, RuntimeError, TypeError):
                continue

        force_deadline_s = time.monotonic() + SHUTDOWN_FORCE_WAIT_S
        for thread in running:
            remaining_ms = max(0, int((force_deadline_s - time.monotonic()) * 1000.0))
            if remaining_ms <= 0:
                break
            waiter = getattr(thread, "wait", None)
            if callable(waiter):
                try:
                    waiter(min(100, remaining_ms))
                except (AttributeError, RuntimeError, TypeError):
                    pass

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
