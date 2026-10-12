"""Workstation QWidget composition root and legacy API compatibility boundary."""
from __future__ import annotations

from Workstation.panel.panel_compatibility_exports import *  # Explicit legacy export inventory.
from Workstation.panel.panel_application import PanelApplication, install_panel_compatibility
from Workstation.panel.panel_controller_bindings import _CONTROLLER_DELEGATE_BINDINGS


class WirisStereoDistancePanel(QtWidgets.QWidget):
    calibration_preview_requested = QtCore.pyqtSignal(object)
    calibration_preview_changed = QtCore.pyqtSignal(object)
    camera_preview_cleared = QtCore.pyqtSignal()
    wiris_command_response = QtCore.pyqtSignal(object)
    calibration_capture_event = QtCore.pyqtSignal(object)
    calibration_diagnostics_changed = QtCore.pyqtSignal(object)
    measurement_package_changed = QtCore.pyqtSignal(object)
    _threadsafe_log_requested = QtCore.pyqtSignal(str)

    def __init__(self, *, auto_start=False, auto_start_pixhawk=False, app_root=None, settings_path=None):
        super().__init__()
        self.application = PanelApplication(self)
        self.application.initialize(
            auto_start=auto_start, auto_start_pixhawk=auto_start_pixhawk,
            app_root=APP_ROOT if app_root is None else app_root,
            settings_path=settings_path, use_default_roots=app_root is None,
        )

    def _drain_deferred_delete_shutdown(self) -> bool:
        """Give short-lived child workers one bounded cooperative drain."""

        try:
            if self.shutdown_for_close():
                return True
        except (AttributeError, RuntimeError, TypeError, ValueError):
            return False
        deadline_s = time.monotonic() + DEFERRED_DELETE_COOPERATIVE_DRAIN_MS / 1000.0
        application = QtCore.QCoreApplication.instance()
        while time.monotonic() < deadline_s:
            if application is not None:
                application.processEvents(QtCore.QEventLoop.AllEvents, 10)
            QtCore.QThread.msleep(2)
            try:
                if self.shutdown_for_close():
                    return True
            except (AttributeError, RuntimeError, TypeError, ValueError):
                return False
        return False


    def event(self, event: QtCore.QEvent) -> bool:  # noqa: N802 - Qt override.
        if event.type() == QtCore.QEvent.DeferredDelete:
            # Embedded/test owners sometimes bypass closeEvent and call
            # deleteLater() directly. Consume that first DeferredDelete until
            # the same ordered lifecycle used by normal close has proven all
            # child QThreads stopped; deleting a running child thread aborts
            # the process in Qt.
            if bool(getattr(self, "_deferred_delete_shutdown_complete", False)):
                timer = getattr(self, "_deferred_delete_retry_timer", None)
                if timer is not None:
                    timer.stop()
                return super().event(event)
            complete = self._drain_deferred_delete_shutdown()
            if complete:
                self._deferred_delete_shutdown_complete = True
                timer = getattr(self, "_deferred_delete_retry_timer", None)
                if timer is not None:
                    timer.stop()
                return super().event(event)
            timer = getattr(self, "_deferred_delete_retry_timer", None)
            if timer is not None and not timer.isActive():
                timer.start()
            return True
        return super().event(event)


    def _retry_deferred_delete_shutdown(self) -> None:
        if bool(getattr(self, "_deferred_delete_shutdown_complete", False)):
            return
        try:
            complete = bool(self.shutdown_for_close())
        except (AttributeError, RuntimeError, TypeError, ValueError):
            complete = False
        if complete:
            self._deferred_delete_shutdown_complete = True
            self._deferred_delete_retry_timer.stop()
            self.deleteLater()
            return
        self._deferred_delete_retry_timer.start()


    def __del__(self) -> None:
        # This is a last-resort guard for wrappers collected before Qt delivers
        # DeferredDelete.  It deliberately touches only the calibration preview
        # worker; session/settings shutdown remains explicit in closeEvent.
        try:
            self._stop_calibration_preview_worker()
        except Exception:
            pass


    def _camera_time_sync_controller(self):
        owner = getattr(self, "_camera_time_sync_controller_owner", None)
        if owner is None:
            owner = CameraTimeSyncController(self)
            self._camera_time_sync_controller_owner = owner
        return owner


    def resizeEvent(self, event) -> None:  # noqa: N802 - Qt override.
        self._update_runtime_camera_settings_layout_mode(event.size().width())
        self._rescale_calibration_preview_pixmaps()
        super().resizeEvent(event)


    def _recording_controller(self):
        owner = getattr(self, "_recording_controller_owner", None)
        if owner is None:
            owner = RecordingController(self)
            self._recording_controller_owner = owner
        return owner


    def _run_evidence_controller(self):
        owner = getattr(self, "_run_evidence_controller_owner", None)
        if owner is None:
            owner = RunEvidenceController(self)
            self._run_evidence_controller_owner = owner
        return owner


    def _data_storage_controller(self):
        owner = getattr(self, "_data_storage_controller_owner", None)
        if owner is None:
            owner = DataStorageController(self)
            self._data_storage_controller_owner = owner
        return owner


    def _optical_pose_controller(self):
        owner = getattr(self, "_optical_pose_controller_owner", None)
        if owner is None:
            owner = OpticalPoseController(self)
            self._optical_pose_controller_owner = owner
        return owner


    def _target_drone_controller(self):
        owner = getattr(self, "_target_drone_controller_owner", None)
        if owner is None:
            owner = TargetDroneController(self)
            self._target_drone_controller_owner = owner
        return owner


    def _drone_telemetry_controller(self):
        owner = getattr(self, "_drone_telemetry_controller_owner", None)
        if owner is None:
            owner = DroneTelemetryController(self)
            self._drone_telemetry_controller_owner = owner
        return owner


    def _calibration_workflow_controller(self):
        owner = getattr(self, "_calibration_workflow_controller_owner", None)
        if owner is None:
            owner = CalibrationWorkflowController(self)
            self._calibration_workflow_controller_owner = owner
        return owner


    def _device_runtime_controller(self):
        owner = getattr(self, "_device_runtime_controller_owner", None)
        if owner is None:
            owner = DeviceRuntimeController(self)
            self._device_runtime_controller_owner = owner
        return owner


    def _gnss_runtime_controller(self):
        owner = getattr(self, "_gnss_runtime_controller_owner", None)
        if owner is None:
            owner = GnssRuntimeController(self)
            self._gnss_runtime_controller_owner = owner
        return owner


    def _measurement_view_controller(self):
        owner = getattr(self, "_measurement_view_controller_owner", None)
        if owner is None:
            owner = MeasurementViewController(self)
            self._measurement_view_controller_owner = owner
        return owner


    def _sensor_presentation_controller(self):
        owner = getattr(self, "_sensor_presentation_controller_owner", None)
        if owner is None:
            owner = SensorPresentationController(self)
            self._sensor_presentation_controller_owner = owner
        return owner


    def _camera_stream_lifecycle_controller(self):
        owner = getattr(self, "_camera_stream_lifecycle_controller_owner", None)
        if owner is None:
            owner = CameraStreamLifecycleController(self)
            self._camera_stream_lifecycle_controller_owner = owner
        return owner


    def _measurement_run_controller(self):
        owner = getattr(self, "_measurement_run_controller_owner", None)
        if owner is None:
            owner = MeasurementRunController(self)
            self._measurement_run_controller_owner = owner
        return owner


    def _wiris_control_controller(self):
        owner = getattr(self, "_wiris_control_controller_owner", None)
        if owner is None:
            owner = WirisControlController(self)
            self._wiris_control_controller_owner = owner
        return owner


    def _app_lifecycle_controller(self):
        owner = getattr(self, "_app_lifecycle_controller_owner", None)
        if owner is None:
            owner = AppLifecycleController(self)
            self._app_lifecycle_controller_owner = owner
        return owner


    def _workstation_settings_controller(self):
        owner = getattr(self, "_workstation_settings_controller_owner", None)
        if owner is None:
            owner = WorkstationSettingsController(self)
            self._workstation_settings_controller_owner = owner
        return owner



install_panel_compatibility(WirisStereoDistancePanel)

for _controller_delegate_binding in _CONTROLLER_DELEGATE_BINDINGS:
    bind_controller_delegates(WirisStereoDistancePanel, _controller_delegate_binding)

def _configure_gl_surface_format() -> None:
    """Request a Compatibility-profile GL 2.1 context before QApplication exists.

    pyqtgraph's opengl.GLViewWidget misparses some Mesa version strings (e.g.
    ``4.6 (Compatibility Profile) Mesa ...``) and refuses to initialize unless
    the default surface format is pinned explicitly first. Skipped entirely if
    PyOpenGL isn't installed, so this stays a no-op when the 3D quality view
    will fall back to its placeholder anyway.
    """

    try:
        import OpenGL  # noqa: F401
    except Exception:
        return
    try:
        surface_format = QtGui.QSurfaceFormat()
        surface_format.setVersion(2, 1)
        surface_format.setProfile(QtGui.QSurfaceFormat.CompatibilityProfile)
        QtGui.QSurfaceFormat.setDefaultFormat(surface_format)
    except Exception:
        pass


def _parse_ui_mode(argv: list[str]) -> tuple[str, list[str]]:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--ui", choices=("workstation",), default="workstation")
    parsed, remaining = parser.parse_known_args(argv[1:])
    return str(parsed.ui), [argv[0], *remaining]


def main() -> int:
    os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "1")
    try:
        faulthandler.register(signal.SIGUSR1)
    except Exception:
        pass
    _ui_mode, cleaned_argv = _parse_ui_mode(list(sys.argv))
    sys.argv[:] = cleaned_argv
    auto_start = "--start" in sys.argv
    if auto_start:
        sys.argv.remove("--start")
    auto_start_pixhawk = "--start-pixhawk" in sys.argv or "--connect-pixhawk" in sys.argv
    if "--start-pixhawk" in sys.argv:
        sys.argv.remove("--start-pixhawk")
    if "--connect-pixhawk" in sys.argv:
        sys.argv.remove("--connect-pixhawk")
    maximized = "--maximized" in sys.argv
    if maximized:
        sys.argv.remove("--maximized")
    _configure_gl_surface_format()
    app = QtWidgets.QApplication(sys.argv)
    from Workstation.__main__ import (
        _flush_window_session_state,
        _install_qt_exception_hook,
    )
    from Workstation.application.main_window import WorkstationMainWindow

    window_holder: dict[str, object] = {}
    _install_qt_exception_hook(
        lambda: _flush_window_session_state(window_holder.get("window"))
    )
    window = WorkstationMainWindow(
        auto_start=auto_start,
        auto_start_pixhawk=auto_start_pixhawk,
    )
    window_holder["window"] = window
    if maximized:
        window.showMaximized()
    else:
        window.show()
    return app.exec_()
