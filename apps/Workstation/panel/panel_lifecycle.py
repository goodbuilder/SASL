"""Lifecycle feature: owned state/views and explicit dependency ports.

Coordinate values retain their producer contracts (NED, body FRD, optical RDF,
and baseline-center FRD). This ownership change performs no axis conversion.
"""
from __future__ import annotations

from Workstation.panel.panel_feature_runtime import PanelFeature
from PyQt5 import (
    QtCore,
)
from Workstation.workers.device_runtime_workers import InterfaceStatusWorker
from Workstation.panel.panel_support import STARTUP_CAMERA_STATUS, STARTUP_PIXHAWK_STATUS, STARTUP_READY_MESSAGE
from sasl_common.lifecycle import (
    LifecycleCoordinator,
    LifecycleStep,
)


class LifecycleFeature(PanelFeature):
    feature_name = 'lifecycle'
    dependency_names = (
        '_auto_load_intrinsics',
        '_camera_sync_timer',
        '_continue_rtsp_reactivation_after_stop',
        '_eth0_probe_timeout_fired',
        '_eth0_probe_timers',
        '_log',
        '_pixhawk_gnss_source',
        '_poll_camera_sync_state',
        '_poll_stus_schedule',
        '_recording_stop_retry_timer',
        '_retry_deferred_delete_shutdown',
        '_retry_legacy_recording_stop',
        '_rtsp_reactivation_stop_timer',
        '_shutdown_save_settings',
        '_shutdown_stop_auxiliary_workers',
        '_shutdown_stop_drone_telemetry',
        '_shutdown_stop_gnss_base_serial',
        '_shutdown_stop_target_drone',
        '_shutdown_verify_resources',
        '_start_startup_camera_sequence',
        '_stop_calibration_preview_worker',
        '_stop_rtcm_pipeline',
        '_stus_connection_generation',
        '_stus_schedule_timer',
        '_stus_scheduler',
        '_threadsafe_log_requested',
        '_wait_for_session_save',
        'alignment_tab_index',
        'alignment_tabs',
        'archive_session_recordings',
        'baseline_alignment_tab_index',
        'camera_connection_label',
        'close_workstation_data',
        'connect_mavlink_pixhawk',
        'destroyed',
        'gnss_monitor',
        'interface_edit',
        'interface_status_thread',
        'interface_status_worker',
        'pixhawk_connection_label',
        'setMinimumSize',
        'stop_all',
        'stop_gnss_network',
        'tabs',
    )
    writable_dependencies = frozenset(('interface_status_thread', 'interface_status_worker'))

    def initialize_state(self, *, auto_start=False, auto_start_pixhawk=False, app_root=None, settings_path=None):
        self._deferred_delete_shutdown_complete = False
        self._deferred_delete_retry_timer = QtCore.QTimer(self.widget)
        self._deferred_delete_retry_timer.setSingleShot(True)
        self._deferred_delete_retry_timer.setInterval(25)
        self._deferred_delete_retry_timer.timeout.connect(
            self._retry_deferred_delete_shutdown
        )
        self._threadsafe_log_requested.connect(
            self._log,
            QtCore.Qt.QueuedConnection,
        )
        self.setMinimumSize(900, 820)
        self.destroyed.connect(self._stop_calibration_preview_worker)
        for camera in ("left", "right"):
            timer = QtCore.QTimer(self.widget)
            timer.setSingleShot(True)
            timer.setInterval(2500)
            timer.timeout.connect(
                lambda camera=camera: self._eth0_probe_timeout_fired(camera)
            )
            self._eth0_probe_timers[camera] = timer
        self._rtsp_reactivation_stop_timer.setSingleShot(True)
        self._rtsp_reactivation_stop_timer.setInterval(100)
        self._rtsp_reactivation_stop_timer.timeout.connect(
            self._continue_rtsp_reactivation_after_stop
        )
        self._startup_camera_sequence_pending = False
        self.auto_start = auto_start
        self.auto_start_pixhawk = auto_start_pixhawk
        self._recording_stop_retry_timer.setSingleShot(True)
        self._recording_stop_retry_timer.setInterval(100)
        self._recording_stop_retry_timer.timeout.connect(
            self._retry_legacy_recording_stop
        )
        self._shutdown_in_progress = False
        self._stus_scheduler.reset_for_connection(self._stus_connection_generation)
        self._camera_sync_timer.setInterval(250)
        self._camera_sync_timer.timeout.connect(self._poll_camera_sync_state)
        self._stus_schedule_timer.setInterval(1000)
        self._stus_schedule_timer.timeout.connect(self._poll_stus_schedule)
        self._stus_schedule_timer.start()
        self.gnss_monitor.left_source = self._pixhawk_gnss_source("left")
        self.gnss_monitor.right_source = self._pixhawk_gnss_source("right")

    def _initial_status(self) -> None:
        self._start_interface_status_worker()
        self.camera_connection_label.setText(STARTUP_CAMERA_STATUS)
        self.pixhawk_connection_label.setText(STARTUP_PIXHAWK_STATUS)
        self._log(STARTUP_READY_MESSAGE)
        # Historical Session/retry evidence is file-manager-only and never
        # participates in new Measurement admission.
        if self.auto_start_pixhawk:
            QtCore.QTimer.singleShot(500, self._auto_start_pixhawk)
        if self.auto_start:
            self._log("[카메라] --start: WIRIS 활성화와 RTSP 프레임 확인 후 GStreamer를 시작합니다.")
            self._start_startup_camera_sequence()

    def _start_interface_status_worker(self) -> None:
        if bool(getattr(self, "_shutdown_in_progress", False)):
            return
        if self.interface_status_thread is not None:
            return
        thread = QtCore.QThread(self.widget)
        worker = InterfaceStatusWorker(self.interface_edit.text().strip())
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.ready.connect(self._apply_interface_status)
        worker.failed.connect(self._interface_status_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self._clear_interface_status_worker)
        self.interface_status_thread = thread
        self.interface_status_worker = worker
        thread.start()

    def _apply_interface_status(self, interface_name: str, status: str) -> None:
        self._log(f"[연결] 네트워크 인터페이스 상태 ({interface_name}):\n{status}")

    def _interface_status_failed(self, error: str) -> None:
        self._log(f"[연결] 네트워크 인터페이스 상태 확인 실패: {error}")

    def _clear_interface_status_worker(self) -> None:
        self.interface_status_thread = None
        self.interface_status_worker = None

    def _auto_start_pixhawk(self) -> None:
        self.tabs.setCurrentIndex(self.alignment_tab_index)
        self.alignment_tabs.setCurrentIndex(self.baseline_alignment_tab_index)
        self._log(
            "[픽스호크] --start-pixhawk: configured P1/P2/Drone MAVLink 연결을 시작합니다."
        )
        # The launcher flag is an explicit operator request to connect the
        # configured endpoints.  Keep the visible Pixhawk page selected, then
        # use the same guarded connection owner as the manual Connect action;
        # Shell RM3100 polling starts from that connection postcondition.
        self.connect_mavlink_pixhawk()

    def build_lifecycle(self):
        self._shutdown_coordinator = LifecycleCoordinator(
            [
                LifecycleStep("stop-target-drone", lambda: self._shutdown_stop_target_drone()),
                LifecycleStep("stop-gnss-network", lambda: self.stop_gnss_network()),
                LifecycleStep("stop-gnss-base-serial", lambda: self._shutdown_stop_gnss_base_serial()),
                LifecycleStep("flush-stop-rtcm-pipeline", lambda: self._stop_rtcm_pipeline()),
                LifecycleStep("stop-drone-telemetry", lambda: self._shutdown_stop_drone_telemetry()),
                LifecycleStep("stop-runtime-workers", lambda: self.stop_all(wait=False)),
                LifecycleStep("stop-auxiliary-workers", lambda: self._shutdown_stop_auxiliary_workers()),
                LifecycleStep("wait-session-save", lambda: self._wait_for_session_save()),
                LifecycleStep("verify-resource-cleanup", lambda: self._shutdown_verify_resources()),
                LifecycleStep("save-settings", lambda: self._shutdown_save_settings()),
                LifecycleStep(
                    "close-measurement-data-writer",
                    lambda: self.close_workstation_data(timeout_s=0.25),
                ),
                LifecycleStep(
                    "archive-session-recordings",
                    lambda: self.archive_session_recordings(),
                ),
            ]
        )


