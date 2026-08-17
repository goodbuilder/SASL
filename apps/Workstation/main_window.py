from __future__ import annotations

from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.workstation_core.theme import (
    APP_BG,
    FONT_FAMILY,
    PANEL_ALT_BG,
    PANEL_BORDER,
    TEXT_MAIN,
    TEXT_MUTED,
)

from .standalone_backend import create_standalone_backend
from .command_registry import CommandRegistry, CommandSpec
from .components import StatusIndicator
from .inspector import InspectorWidget
from .navigation import NavigationTree, canonical_context_id
from .settings_workspace import SettingsWorkspace
from .status_model import BackendStatusAdapter, RuntimeStatus
from Workstation.workstation_core.px4_calibration_groups import PX4_CALIBRATION_GROUPS
from .workspaces import (
    AlignmentWorkspace,
    AdvancedWorkspace,
    BaseGnssWorkspace,
    CameraWorkspace,
    CalibrationWorkspace,
    DiagnosticsContent,
    MeasurementWorkspace,
    OverviewWorkspace,
    PixhawkWorkspace,
    RecordingWorkspace,
    TelemetryWorkspace,
    TimeWorkspace,
)


class WorkstationMainWindow(QtWidgets.QMainWindow):
    """Industrial engineering shell around the standalone Workstation owner."""

    def __init__(
        self,
        *,
        backend: object | None = None,
        auto_start: bool = False,
        auto_start_pixhawk: bool = False,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("StereoWIRIS Workstation")
        self.setObjectName("workstationMainWindow")
        self.setWindowFlags(
            self.windowFlags()
            | QtCore.Qt.WindowMinimizeButtonHint
            | QtCore.Qt.WindowMaximizeButtonHint
            | QtCore.Qt.WindowCloseButtonHint
        )
        self.resize(1600, 980)
        self.backend = backend or create_standalone_backend(
            auto_start=auto_start,
            auto_start_pixhawk=auto_start_pixhawk,
        )
        self.status_adapter = BackendStatusAdapter(self.backend, self)
        self.commands = CommandRegistry(self)
        self._current_context = "overview"
        self._closing = False
        self._build_commands()
        self._build_shell()
        self._build_workspaces()
        self.inspector.command_requested.connect(self._handle_inspector_command)
        self._apply_style()
        self._ui_settings = QtCore.QSettings("SASL", "StereoWIRIS Workstation")
        self._restore_ui_state()
        self.status_adapter.model.changed.connect(self._refresh_from_status)
        self.status_adapter.polled.connect(self._refresh_evidence_from_poll)
        self.commands.accepted.connect(self._show_command_accepted)
        self.commands.failed.connect(self._show_command_error)
        self.status_adapter.start()

    def _build_commands(self) -> None:
        self.commands.register(
            CommandSpec(
                "camera.toggle",
                "Preview",
                "Start/stop the camera preview through the existing CameraPane owner.",
                "camera",
                self._toggle_camera,
                shortcut="Ctrl+P",
            )
        )
        self.commands.register(
            CommandSpec(
                "detection.toggle",
                "Detection",
                "Enable/disable YOLO detection through the existing stream path.",
                "detection",
                self._toggle_detection,
                shortcut="Ctrl+D",
            )
        )
        self.commands.register(
            CommandSpec(
                "recording.toggle",
                "Record",
                "Start/stop the existing session spool and asynchronous finalization.",
                "recording",
                self._toggle_recording,
                shortcut="Ctrl+R",
            )
        )
        self.commands.register(
            CommandSpec(
                "session.save",
                "Save session",
                "Finalize the existing session files without changing the measurement schema.",
                "data",
                self._save_session,
                shortcut="Ctrl+S",
            )
        )
        for command_id, text, tooltip, category, handler in (
            (
                "camera.discover_host_ip",
                "Find camera network",
                "Discover the camera-network host/interface through the existing owner.",
                "camera",
                self._camera_discover_host_ip,
            ),
            (
                "camera.auto_connect",
                "Connect cameras",
                "Discover WIRIS control candidates, verify HIWS, enable RTSP, and apply the verified camera pair.",
                "camera",
                self._camera_auto_connect,
            ),
            (
                "camera.check_rtsp",
                "Check video",
                "Check RTSP ports and frame reception through the existing camera worker path.",
                "camera",
                self._camera_check_rtsp,
            ),
            (
                "camera.check_control",
                "Check control",
                "Check WIRIS control ports through the existing network path.",
                "camera",
                self._camera_check_control,
            ),
            (
                "camera.enable_rtsp",
                "Enable stream",
                "Enable WIRIS RTSP using the existing control and camera owner.",
                "camera",
                self._camera_enable_rtsp,
            ),
            (
                "camera.reactivate_left",
                "Reactivate CAM L",
                "Safely send SETH TRUE for the left WIRIS camera, verify frames, and reconnect only the left preview.",
                "camera",
                lambda: self._camera_reactivate("left"),
            ),
            (
                "camera.reactivate_right",
                "Reactivate CAM R",
                "Safely send SETH TRUE for the right WIRIS camera, verify frames, and reconnect only the right preview.",
                "camera",
                lambda: self._camera_reactivate("right"),
            ),
            (
                "camera.swap",
                "Swap L/R",
                "Swap the existing Left/Right camera mapping without changing coordinate contracts.",
                "camera",
                self._camera_swap,
            ),
            (
                "camera.save_settings",
                "Save camera",
                "Save the current camera settings through the existing settings owner.",
                "camera",
                self._camera_save_settings,
            ),
            (
                "pixhawk.toggle_mavlink",
                "Connect Pixhawk",
                "Legacy toggle for the existing MAVLink Pixhawk path.",
                "device",
                self._pixhawk_toggle_mavlink,
            ),
            (
                "pixhawk.connect_mavlink",
                "Connect Pixhawk",
                "Connect the configured P1/P2 MAVLink Pixhawk endpoints.",
                "device",
                self._pixhawk_connect_mavlink,
            ),
            (
                "pixhawk.disconnect_mavlink",
                "Disconnect Pixhawk",
                "Disconnect the configured P1/P2 MAVLink Pixhawk endpoints.",
                "device",
                self._pixhawk_disconnect_mavlink,
            ),
            (
                "pixhawk.reset_level_calibration",
                "Reset level cal.",
                "Reset only PX4 SENS_BOARD_X_OFF/Y_OFF to firmware defaults on connected Pixhawks.",
                "device",
                self._pixhawk_reset_level_calibration,
            ),
            (
                "pixhawk.refresh_parameters.p1",
                "Refresh P1 parameters",
                "Read P1 PX4 calibration parameters written by QGC and show their raw sensor-axis values.",
                "device",
                lambda: self._pixhawk_refresh_parameters("left"),
            ),
            (
                "pixhawk.refresh_parameters.p2",
                "Refresh P2 parameters",
                "Read P2 PX4 calibration parameters written by QGC and show their raw sensor-axis values.",
                "device",
                lambda: self._pixhawk_refresh_parameters("right"),
            ),
            (
                "pixhawk.recover_eth_mavlink.p1",
                "P1 ETH MAV",
                "Use the configured P1 USB control port to restart PX4 Ethernet MAVLink at runtime, then reconnect UDP telemetry.",
                "device",
                lambda: self._pixhawk_recover_eth_mavlink("left"),
            ),
            (
                "pixhawk.recover_eth_mavlink.p2",
                "P2 ETH MAV",
                "Use the configured P2 USB control port to restart PX4 Ethernet MAVLink at runtime, then reconnect UDP telemetry.",
                "device",
                lambda: self._pixhawk_recover_eth_mavlink("right"),
            ),
            (
                "pixhawk.discover_usb",
                "Find USB Pixhawks",
                "List USB serial candidates and MAVLink heartbeat evidence without auto-assigning P1/P2.",
                "device",
                self._pixhawk_discover_usb,
            ),
            (
                "pixhawk.discover_mavlink_candidates",
                "Find network links",
                "Generate Ethernet MAVLink URL candidates without changing the selected P1/P2 endpoints.",
                "device",
                self._pixhawk_discover_mavlink_candidates,
            ),
            (
                "pixhawk.discover_host_ip",
                "Find Pixhawk host IP",
                "List local IPv4 interfaces that can be used for Pixhawk Ethernet candidates.",
                "device",
                self._pixhawk_discover_host_ip,
            ),
            (
                "telemetry.apply",
                "Apply drone telemetry",
                "Apply the active Drone Pixhawk endpoint and RTCM forwarding settings.",
                "device",
                self._telemetry_apply,
            ),
            (
                "telemetry.connect",
                "Connect drone telemetry",
                "Connect the active Drone Pixhawk MAVLink endpoint and receive its GPS position.",
                "device",
                self._telemetry_connect,
            ),
            (
                "telemetry.disconnect",
                "Disconnect drone telemetry",
                "Disconnect the active Drone Pixhawk MAVLink endpoint.",
                "device",
                self._telemetry_disconnect,
            ),
            (
                "telemetry.save_settings",
                "Save drone telemetry",
                "Persist the active Drone Pixhawk endpoint and RTCM forwarding settings.",
                "device",
                self._telemetry_save_settings,
            ),
            (
                "telemetry.discover_usb",
                "Find USB telemetry radio",
                "Probe /dev/serial/by-id, ttyACM, and ttyUSB candidates for a MAVLink heartbeat and fill the URL.",
                "device",
                self._telemetry_discover_usb,
            ),
            (
                "gnss.base_discover",
                "Find base GNSS",
                "Discover GNSS Base serial candidates through the existing owner.",
                "gnss",
                self._gnss_base_discover,
            ),
            (
                "gnss.base_connect",
                "Connect base",
                "Connect the configured GNSS Base serial through the existing worker owner.",
                "gnss",
                self._gnss_base_connect,
            ),
            (
                "gnss.base_disconnect",
                "Disconnect base",
                "Stop the configured GNSS Base serial worker.",
                "gnss",
                self._gnss_base_disconnect,
            ),
            (
                "gnss.survey_start",
                "Start survey",
                "Start local GNSS Base Survey-in through the existing UBX command path.",
                "gnss",
                self._gnss_survey_start,
            ),
            (
                "gnss.survey_stop",
                "Stop survey",
                "Stop local GNSS Base Survey-in through the existing UBX command path.",
                "gnss",
                self._gnss_survey_stop,
            ),
            (
                "gnss.ntrip_start",
                "Start NTRIP",
                "Start the configured NTRIP operation through the existing GNSS owner.",
                "gnss",
                self._gnss_ntrip_start,
            ),
            (
                "gnss.ntrip_stop",
                "Stop NTRIP",
                "Stop the existing NTRIP operation and its correction path.",
                "gnss",
                self._gnss_ntrip_stop,
            ),
            (
                "gnss.ntrip_forget_password",
                "Forget NTRIP password",
                "Remove the saved NTRIP password from the OS secure credential store.",
                "gnss",
                self._gnss_ntrip_forget_password,
            ),
        ):
            self.commands.register(CommandSpec(command_id, text, tooltip, category, handler))
        for side_code, camera in (("p1", "left"), ("p2", "right")):
            for calibration in PX4_CALIBRATION_GROUPS:
                if not calibration.resettable:
                    continue
                command_id = f"pixhawk.reset_calibration.{side_code}.{calibration.key}"
                self.commands.register(
                    CommandSpec(
                        command_id,
                        f"Reset {side_code.upper()} {calibration.label}",
                        (
                            f"Reset only {', '.join(calibration.reset_parameter_names)} "
                            "to the connected PX4 firmware defaults."
                        ),
                        "device",
                        lambda camera=camera, group=calibration.key: self._pixhawk_reset_calibration(
                            camera, group
                        ),
                    )
                )
        for command_id, text, tooltip, handler in (
            (
                "time.set_pc",
                "Set camera clock",
                "Set the selected WIRIS camera clock through the existing owner.",
                self._time_set_pc,
            ),
            (
                "time.check_sync",
                "Check sync",
                "Refresh RTCP/NTP stream timing evidence through the existing owner.",
                self._time_check_sync,
            ),
            (
                "time.apply_settings",
                "Apply sync limits",
                "Apply Workstation camera synchronization limits through the existing owner.",
                self._time_apply_settings,
            ),
            (
                "time.save_settings",
                "Save sync limits",
                "Persist Workstation camera synchronization limits for the next run.",
                self._time_save_settings,
            ),
        ):
            self.commands.register(CommandSpec(command_id, text, tooltip, "time", handler))
        for command_id, text, tooltip, handler in (
            (
                "calibration.capture_images",
                "Capture",
                "Capture calibration images through the existing checkerboard owner.",
                self._calibration_capture_images,
            ),
            (
                "calibration.capture_images_wiris_ftp",
                "Capture FTP HR",
                "Trigger a WIRIS HR still on both cameras and download it via FTP "
                "into this calibration run, instead of the live RTSP feed.",
                self._calibration_capture_images_wiris_ftp,
            ),
            (
                "calibration.capture_images_wiris_ftp_oz",
                "Capture FTP OZ",
                "Same as Capture FTP HR, using the lower-resolution operating-zoom "
                "still (SIVI) instead of the HR still -- faster, no camera reboot risk.",
                self._calibration_capture_images_wiris_ftp_oz,
            ),
            (
                "calibration.auto_capture",
                "Auto capture",
                "Start/stop validated automatic checkerboard pair capture through the existing owner.",
                self._calibration_toggle_auto_capture,
            ),
            (
                "calibration.run_intrinsic",
                "Run calibration",
                "Run the existing intrinsic calibration pipeline without duplicating camera math.",
                self._calibration_run_intrinsic,
            ),
            (
                "calibration.run_extrinsics",
                "Calculate R,t only",
                "Estimate OpenCV optical RDF left-to-right R,t with the selected K,D fixed.",
                self._calibration_run_extrinsics,
            ),
            (
                "calibration.save_intrinsics",
                "Save intrinsics",
                "Save the current intrinsic profile through the existing calibration owner.",
                self._calibration_save_intrinsics,
            ),
            (
                "calibration.save_extrinsics",
                "Save R,t",
                "Save the selected stereo R,t as an independent profile.",
                self._calibration_save_extrinsics,
            ),
            (
                "calibration.load_intrinsics",
                "Load intrinsics",
                "Choose an intrinsic JSON file and apply it.",
                self._calibration_load_intrinsics,
            ),
            (
                "calibration.load_extrinsics",
                "Load R,t",
                "Load a standalone R,t profile or legacy intrinsics JSON with embedded extrinsics.",
                self._calibration_load_extrinsics,
            ),
            (
                "calibration.apply_bundle",
                "Apply K,D + R,t",
                "Apply the independently selected K,D and R,t; source-zoom mismatch is informational only.",
                self._calibration_apply_bundle,
            ),
        ):
            self.commands.register(CommandSpec(command_id, text, tooltip, "calibration", handler))
        for command_id, text, tooltip, handler in (
            (
                "measurement.apply_settings",
                "Apply",
                "Apply Workstation measurement fields through the existing owner.",
                self._measurement_apply_settings,
            ),
            (
                "measurement.save_settings",
                "Save",
                "Persist the current measurement settings without changing measurement schemas or coordinate frames.",
                self._measurement_save_settings,
            ),
            (
                "calibration.apply_setup",
                "Apply setup",
                "Apply checkerboard geometry, calculation mode, and capture path through the existing owner.",
                self._calibration_apply_setup,
            ),
            (
                "calibration.save_setup",
                "Save setup",
                "Persist the current Workstation calibration setup through the existing settings owner.",
                self._calibration_save_setup,
            ),
            (
                "alignment.scan_sources",
                "Find sources",
                "Refresh MAVLink alignment source evidence through the existing owner.",
                self._alignment_scan_sources,
            ),
            (
                "alignment.start_sampling",
                "Start sampling",
                "Start the existing GNSS baseline quality sampling workflow.",
                self._alignment_start_sampling,
            ),
            (
                "alignment.stop_sampling",
                "Stop sampling",
                "Stop the existing GNSS baseline quality sampling workflow.",
                self._alignment_stop_sampling,
            ),
            (
                "alignment.save_profile",
                "Save profile",
                "Save the existing alignment profile without changing NED or camera-frame contracts.",
                self._alignment_save_profile,
            ),
            (
                "alignment.load_profile",
                "Load profile",
                "Load the existing alignment profile through the owner.",
                self._alignment_load_profile,
            ),
            (
                "alignment.swap_pixhawks",
                "Swap P1/P2",
                "Swap P1/P2 mapping through the existing owner; coordinate labels remain explicit.",
                self._alignment_swap_pixhawks,
            ),
            (
                "alignment.query_lrf",
                "Refresh LRF",
                "Run one on-demand WIRIS GLRD read on both cameras instead of a continuous poll.",
                self._alignment_query_lrf,
            ),
        ):
            category = (
                "measurement"
                if command_id.startswith("measurement.")
                else "calibration"
                if command_id.startswith("calibration.")
                else "alignment"
            )
            self.commands.register(CommandSpec(command_id, text, tooltip, category, handler))
        for command_id, text, tooltip, handler in (
            (
                "settings.apply",
                "Apply settings",
                "Apply the complete Workstation settings payload through the existing panel owner.",
                self._settings_apply,
            ),
            (
                "settings.save",
                "Save settings",
                "Persist the complete Workstation settings payload without changing data or coordinate schemas.",
                self._settings_save,
            ),
        ):
            self.commands.register(CommandSpec(command_id, text, tooltip, "settings", handler))
        for command_id, text, tooltip, handler in (
            (
                "advanced.wiris.send",
                "Send WIRIS SDK command",
                "Send the selected raw WIRIS SDK command through the existing camera control owner.",
                self._advanced_wiris_send,
            ),
            (
                "advanced.zoom.in",
                "Zoom in",
                "Send the existing WIRIS zoom-in command.",
                lambda: self._advanced_zoom("in"),
            ),
            (
                "advanced.zoom.out",
                "Zoom out",
                "Send the existing WIRIS zoom-out command.",
                lambda: self._advanced_zoom("out"),
            ),
            (
                "advanced.zoom.stop",
                "Stop zoom",
                "Stop the existing WIRIS focus/zoom action.",
                lambda: self._advanced_zoom("stop"),
            ),
            (
                "advanced.zoom.set",
                "Set zoom level",
                "Send the existing absolute WIRIS visible zoom command.",
                lambda: self._advanced_zoom("set"),
            ),
            (
                "advanced.watch.add_mavlink",
                "Add MAVLink watch",
                "Add a MAVLink field watch to the existing Pixhawk telemetry timeline.",
                self._advanced_watch_add_mavlink,
            ),
            (
                "advanced.watch.discover_mavlink",
                "Discover MAVLink watch",
                "Open the existing MAVLink message/field browser.",
                self._advanced_watch_discover_mavlink,
            ),
            (
                "advanced.watch.remove",
                "Remove field watch",
                "Remove the selected custom field watch from the existing subscriber timeline.",
                self._advanced_watch_remove,
            ),
            (
                "advanced.baseline.scan",
                "Scan baseline fields",
                "Refresh MAVLink baseline field evidence through the existing owner.",
                self._advanced_baseline_scan,
            ),
            (
                "advanced.px4.refresh_mag",
                "Refresh PX4 MAG",
                "Read the selected PX4 magnetometer status through the existing parameter manager.",
                self._advanced_px4_refresh_mag,
            ),
            (
                "advanced.px4.refresh_attitude",
                "Refresh attitude calibration",
                "Read selected PX4 accelerometer/gyro/board-rotation calibration state.",
                self._advanced_px4_refresh_attitude,
            ),
            (
                "advanced.px4.shell",
                "Open PX4 Shell",
                "Open the existing per-Pixhawk PX4 Shell dialog.",
                self._advanced_px4_shell,
            ),
            (
                "advanced.px4.apply_mag",
                "Apply PX4 MAG priorities",
                "Apply selected CAL_MAG priority values through the existing guarded manager.",
                self._advanced_px4_apply_mag,
            ),
            (
                "advanced.px4.save_mag",
                "Save PX4 MAG parameters",
                "Save PX4 MAG parameters through the existing MAVLink/PX4 shell path.",
                self._advanced_px4_save_mag,
            ),
            (
                "advanced.px4.reboot",
                "Reboot PX4",
                "Reboot the selected Pixhawk through the existing guarded MAVLink path.",
                self._advanced_px4_reboot,
            ),
            (
                "advanced.preview.refresh",
                "Refresh calibration preview",
                "Refresh checkerboard preview through the existing lazy preview worker.",
                self._advanced_preview_refresh,
            ),
            (
                "advanced.media.refresh",
                "Refresh media artifacts",
                "Refresh session and media artifact evidence.",
                self._advanced_media_refresh,
            ),
            (
                "advanced.media.preview",
                "Preview selected CSV",
                "Preview stored CSV fields without coordinate or timestamp conversion.",
                self._advanced_media_preview,
            ),
            (
                "advanced.media.open_session",
                "Open session folder",
                "Open the selected session directory in the system file manager.",
                self._advanced_media_open_session,
            ),
            (
                "advanced.media.open_artifact",
                "Open artifact",
                "Open the selected saved artifact with its system association.",
                self._advanced_media_open_artifact,
            ),
            (
                "advanced.media.open_video",
                "Open video",
                "Open the selected recorded video with the system-associated player.",
                self._advanced_media_open_video,
            ),
        ):
            self.commands.register(CommandSpec(command_id, text, tooltip, "advanced", handler))
        self.commands.register(
            CommandSpec(
                "advanced.open",
                "Advanced",
                "Open Workstation advanced diagnostics and tools.",
                "view",
                lambda: self._select_context("tasks.advanced"),
            )
        )
        for command_id in (
            "camera.toggle",
            "detection.toggle",
            "recording.toggle",
            "session.save",
            "camera.discover_host_ip",
            "camera.auto_connect",
            "camera.check_rtsp",
            "camera.check_control",
            "camera.enable_rtsp",
            "camera.reactivate_left",
            "camera.reactivate_right",
            "time.set_pc",
            "time.check_sync",
            "pixhawk.toggle_mavlink",
            "pixhawk.connect_mavlink",
            "pixhawk.disconnect_mavlink",
            "pixhawk.reset_level_calibration",
            "pixhawk.refresh_parameters.p1",
            "pixhawk.refresh_parameters.p2",
            "pixhawk.recover_eth_mavlink.p1",
            "pixhawk.recover_eth_mavlink.p2",
            "pixhawk.discover_usb",
            "pixhawk.discover_host_ip",
            "gnss.base_discover",
            "gnss.base_connect",
            "gnss.base_disconnect",
            "gnss.survey_start",
            "gnss.survey_stop",
            "gnss.ntrip_start",
            "gnss.ntrip_stop",
            "calibration.capture_images",
            "calibration.run_intrinsic",
            "calibration.run_extrinsics",
            "calibration.load_intrinsics",
            "calibration.load_extrinsics",
            "advanced.wiris.send",
            "advanced.zoom.in",
            "advanced.zoom.out",
            "advanced.zoom.stop",
            "advanced.zoom.set",
            "advanced.watch.discover_mavlink",
            "advanced.px4.refresh_mag",
            "advanced.px4.refresh_attitude",
            "advanced.px4.shell",
            "advanced.px4.apply_mag",
            "advanced.px4.save_mag",
            "advanced.px4.reboot",
            "advanced.preview.refresh",
            "advanced.media.refresh",
            "advanced.media.preview",
            "advanced.media.open_session",
            "advanced.media.open_artifact",
            "advanced.media.open_video",
        ):
            self.commands.set_deferred_completion(command_id)
        for side_code in ("p1", "p2"):
            for calibration in PX4_CALIBRATION_GROUPS:
                if calibration.resettable:
                    self.commands.set_deferred_completion(
                        f"pixhawk.reset_calibration.{side_code}.{calibration.key}"
                    )

    def _build_shell(self) -> None:
        self.navigation_dock = QtWidgets.QDockWidget("Navigation", self)
        self.navigation_dock.setObjectName("navigationDock")
        self.navigation = NavigationTree(self.navigation_dock)
        self.navigation_dock.setWidget(self.navigation)
        self.addDockWidget(QtCore.Qt.LeftDockWidgetArea, self.navigation_dock)
        self.resizeDocks([self.navigation_dock], [240], QtCore.Qt.Horizontal)

        self.inspector_dock = QtWidgets.QDockWidget("Inspector", self)
        self.inspector_dock.setObjectName("inspectorDock")
        self.inspector_dock.setMinimumWidth(240)
        self.inspector_dock.setMaximumWidth(360)
        self.inspector = InspectorWidget(
            self.backend,
            lambda: self._select_context("tasks.advanced"),
        )
        self.inspector_scroll = self._scroll_container(self.inspector, "inspectorScroll", self.inspector_dock)
        self.inspector_dock.setWidget(self.inspector_scroll)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, self.inspector_dock)
        self.resizeDocks([self.inspector_dock], [310], QtCore.Qt.Horizontal)

        self.diagnostics_dock = QtWidgets.QDockWidget("Diagnostics", self)
        self.diagnostics_dock.setObjectName("diagnosticsDock")
        self.diagnostics_dock.setMinimumHeight(120)
        self.diagnostics_dock.setMaximumHeight(280)
        self.diagnostics = DiagnosticsContent(self.backend, self.status_adapter.model, self.diagnostics_dock)
        self.diagnostics_dock.setWidget(self.diagnostics)
        self.addDockWidget(QtCore.Qt.BottomDockWidgetArea, self.diagnostics_dock)
        self.resizeDocks([self.diagnostics_dock], [210], QtCore.Qt.Vertical)

        self.workspace_host = QtWidgets.QStackedWidget(self)
        self.workspace_host.setObjectName("workspaceHost")
        self.setCentralWidget(self.workspace_host)
        self.navigation.context_changed.connect(self._select_context)

        toolbar = QtWidgets.QToolBar("Primary", self)
        toolbar.setObjectName("primaryToolbar")
        toolbar.setMovable(False)
        toolbar.setToolButtonStyle(QtCore.Qt.ToolButtonTextOnly)
        self.addToolBar(QtCore.Qt.TopToolBarArea, toolbar)
        toolbar.addAction(self.commands.action("camera.toggle"))
        toolbar.addAction(self.commands.action("detection.toggle"))
        toolbar.addSeparator()
        toolbar.addAction(self.commands.action("recording.toggle"))
        toolbar.addAction(self.commands.action("session.save"))
        toolbar.addSeparator()
        toolbar.addAction(self.navigation_dock.toggleViewAction())
        toolbar.addAction(self.inspector_dock.toggleViewAction())
        toolbar.addAction(self.diagnostics_dock.toggleViewAction())
        self.toolbar = toolbar

        self._build_menus()
        self.status_widgets: dict[str, StatusIndicator] = {}
        for key, label in (
            ("camera.left", "CAM L"),
            ("camera.right", "CAM R"),
            ("pixhawk.left", "P1"),
            ("pixhawk.right", "P2"),
            ("gnss", "RTK"),
            ("gnss.ntrip", "NTRIP"),
            ("detection", "DET"),
            ("measurement", "MEAS"),
            ("recording", "REC"),
            ("fps", "FPS"),
        ):
            indicator = StatusIndicator(label, compact=True)
            self.statusBar().addPermanentWidget(indicator)
            self.status_widgets[key] = indicator
        self.statusBar().setSizeGripEnabled(True)

    def _build_menus(self) -> None:
        file_menu = self.menuBar().addMenu("File")
        file_menu.addAction(self.commands.action("session.save"))
        file_menu.addSeparator()
        file_menu.addAction("Quit", self.close, "Ctrl+Q")
        view_menu = self.menuBar().addMenu("View")
        view_menu.addAction(self.navigation_dock.toggleViewAction())
        view_menu.addAction(self.inspector_dock.toggleViewAction())
        view_menu.addAction(self.diagnostics_dock.toggleViewAction())
        device_menu = self.menuBar().addMenu("Device")
        device_menu.addAction(self.commands.action("camera.toggle"))
        device_menu.addAction(self.commands.action("camera.auto_connect"))
        device_menu.addAction(self.commands.action("pixhawk.connect_mavlink"))
        device_menu.addAction(self.commands.action("pixhawk.disconnect_mavlink"))
        device_menu.addAction(self.commands.action("pixhawk.reset_level_calibration"))
        device_menu.addAction(self.commands.action("pixhawk.recover_eth_mavlink.p1"))
        device_menu.addAction(self.commands.action("pixhawk.recover_eth_mavlink.p2"))
        device_menu.addAction(self.commands.action("pixhawk.discover_usb"))
        device_menu.addAction(self.commands.action("pixhawk.discover_mavlink_candidates"))
        measurement_menu = self.menuBar().addMenu("Run")
        measurement_menu.addAction(self.commands.action("detection.toggle"))
        measurement_menu.addAction(self.commands.action("recording.toggle"))
        tools_menu = self.menuBar().addMenu("Tools")
        tools_menu.addAction(self.commands.action("advanced.open"))
        tools_menu.addAction(self.commands.action("settings.apply"))
        tools_menu.addAction(self.commands.action("settings.save"))
        diagnostics_menu = self.menuBar().addMenu("Diagnostics")
        diagnostics_menu.addAction(self.diagnostics_dock.toggleViewAction())

    def _build_workspaces(self) -> None:
        self.overview = OverviewWorkspace(self.backend, self.status_adapter.model)
        self.camera = CameraWorkspace(self.backend, self.status_adapter.model, self.commands.invoke)
        self.time_workspace = TimeWorkspace(self.backend, self.commands.invoke)
        self.measurement = MeasurementWorkspace(
            self.backend,
            self.status_adapter.model,
            self.commands.invoke,
        )
        self.pixhawk = PixhawkWorkspace(
            self.backend,
            self.status_adapter.model,
            self.commands.invoke,
        )
        self.telemetry = TelemetryWorkspace(
            self.backend,
            self.commands.invoke,
        )
        self.base_gnss = BaseGnssWorkspace(
            self.backend,
            self.commands.invoke,
        )
        self.alignment = AlignmentWorkspace(
            self.backend,
            self.status_adapter.model,
            lambda: self._select_context("tasks.advanced"),
            self.commands.invoke,
        )
        self.pixhawk.inspection_requested.connect(
            lambda selection: self._inspect_selection("devices.pixhawk", selection)
        )
        self.camera.inspection_requested.connect(
            lambda selection: self._inspect_selection("devices.cameras", selection)
        )
        self.base_gnss.inspection_requested.connect(
            lambda selection: self._inspect_selection("devices.gnss", selection)
        )
        self.alignment.inspection_requested.connect(
            lambda selection: self._inspect_selection("processing.alignment", selection)
        )
        self.alignment.optical_pose_command_requested.connect(
            self._handle_optical_pose_command
        )
        self.measurement.inspection_requested.connect(
            lambda selection: self._inspect_selection("processing.measurement", selection)
        )
        self.calibration = CalibrationWorkspace(
            self.backend,
            self._open_advanced_zoom,
            self.commands.invoke,
        )
        self.recording = RecordingWorkspace(
            self.backend,
            self.status_adapter.model,
            lambda: self._select_context("tasks.advanced"),
            self.commands.invoke,
        )
        self.recording.hr_command_requested.connect(self._handle_hr_command)
        self.recording.ulog_command_requested.connect(self._handle_ulog_command)
        self.advanced = AdvancedWorkspace(
            self.backend,
            self.status_adapter.model,
            self.commands.invoke,
        )
        self.settings_workspace = SettingsWorkspace(
            self.backend,
            self.commands.invoke,
        )
        self._owner_host = QtWidgets.QWidget(self)
        self._owner_host.hide()
        owner_widget = getattr(self.backend, "owner_widget", None)
        if isinstance(owner_widget, QtWidgets.QWidget):
            owner_widget.setParent(self._owner_host)
            owner_widget.hide()
        self._workspace_indexes: dict[str, int] = {}
        self.workspace_scrolls: dict[str, QtWidgets.QScrollArea] = {}
        for context_id, widget in (
            ("overview", self.overview),
            ("camera", self.camera),
            ("time", self.time_workspace),
            ("measurement", self.measurement),
            ("pixhawk", self.pixhawk),
            ("telemetry", self.telemetry),
            ("base_gnss", self.base_gnss),
            ("alignment", self.alignment),
            ("calibration", self.calibration),
            ("recording", self.recording),
            ("advanced", self.advanced),
            ("settings", self.settings_workspace),
        ):
            scroll = self._scroll_container(widget, f"{context_id.replace('.', '_')}WorkspaceScroll", self.workspace_host)
            self.workspace_scrolls[context_id] = scroll
            self._workspace_indexes[context_id] = self.workspace_host.addWidget(scroll)
        self.overview.open_context.connect(self._select_context)
        self._select_context("overview")

    def _handle_hr_command(self, command_id: str, payload: object) -> None:
        """Route camera still work while keeping file selection outside the workspace.

        The dialog returns only a source path.  Exact-byte copy, decoding,
        hashing, serial checks, and append-only event creation remain owned by
        ``RecordingController`` behind the backend facade.
        """

        values = dict(payload) if isinstance(payload, dict) else {}
        try:
            if command_id in {"recording.hr.capture", "recording.oz.capture"}:
                capture_method = (
                    self.backend.capture_pnp_hr_photos
                    if command_id == "recording.hr.capture"
                    else self.backend.capture_pnp_oz_photos
                )
                try:
                    result = capture_method(str(values.get("purpose") or "unspecified"))
                except TypeError:
                    # Compatibility for test/legacy facades while the real
                    # Workstation owner records the typed purpose.
                    result = capture_method()
            elif command_id == "recording.hr.import":
                source_path, _selected_filter = QtWidgets.QFileDialog.getOpenFileName(
                    self,
                    f"Import {str(values.get('side') or '').title()} HR image",
                    "",
                    "Images (*.png *.jpg *.jpeg *.tif *.tiff);;All files (*)",
                )
                if not source_path:
                    self.statusBar().showMessage("HR import cancelled", 2500)
                    return
                result = self.backend.import_hr_files(
                    str(values.get("capture_id") or ""),
                    side=str(values.get("side") or ""),
                    path=source_path,
                    purpose=str(values.get("purpose") or "unspecified"),
                )
            else:
                raise RuntimeError(f"unknown camera still command: {command_id}")
        except Exception as exc:
            status_label = (
                self.recording.pnp_capture_status_label
                if command_id.startswith("recording.hr")
                else self.recording.oz_capture_status_label
            )
            status_label.setText(f"{command_id.rsplit('.', 2)[-2].upper()} ERROR · {exc}")
            self.statusBar().showMessage(f"{command_id}: {exc}", 6000)
            return
        state = "in progress" if command_id.endswith("capture") else "imported"
        if result is False:
            state = "rejected"
        elif isinstance(result, dict):
            state = str(result.get("state", state) or state)
        status_label = (
            self.recording.pnp_capture_status_label
            if command_id.startswith("recording.hr")
            else self.recording.oz_capture_status_label
        )
        channel_label = "HR" if command_id.startswith("recording.hr") else "OZ"
        status_label.setText(f"{channel_label} {state}")
        self.statusBar().showMessage(f"{command_id}: {state}", 3000)

    def _handle_ulog_command(self, command_id: str, payload: object) -> None:
        if command_id != "recording.ulog.import":
            return
        values = dict(payload) if isinstance(payload, dict) else {}
        source_path, _selected_filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            f"Import {str(values.get('source') or '').upper()} ULog",
            "",
            "PX4 ULog (*.ulg *.ulog);;All files (*)",
        )
        if not source_path:
            self.recording.ulog_status_label.setText("ULog import cancelled")
            return
        values["path"] = source_path
        try:
            result = self.backend.ulog_import(values)
        except Exception as exc:
            self.recording.ulog_status_label.setText(f"ULog ERROR · {exc}")
            self.statusBar().showMessage(f"ULog import: {exc}", 6000)
            return
        self.recording.apply_ulog_snapshot(result)
        self.statusBar().showMessage("ULog imported and hash-verified", 3500)

    def _handle_optical_pose_command(self, command_id: str, payload: object) -> None:
        """Route typed OpticalPosePane requests through backend owner facades."""

        values = dict(payload) if isinstance(payload, dict) else {}
        try:
            if command_id == "alignment.pose.apply_candidate":
                result = self.backend.optical_pose_apply_candidate(values)
            elif command_id == "alignment.pose.revert":
                result = self.backend.optical_pose_revert(values)
            elif command_id == "alignment.pose.capture_burst":
                # Native burst acquisition is retained as an explicit owner
                # boundary; capture cannot be faked from the downscaled UI.
                method = getattr(self.backend, "optical_pose_capture_burst", None)
                if not callable(method):
                    raise RuntimeError("native frozen-burst owner is not available")
                result = method(values)
            elif command_id == "alignment.pose.import_burst":
                method = getattr(self.backend, "optical_pose_import_burst", None)
                if not callable(method):
                    raise RuntimeError("frozen-burst import owner is not available")
                result = method(values)
            elif command_id == "alignment.pose.capture_burst_wiris_ftp":
                # Same native-owner boundary as capture_burst: the capture and
                # FTP download run on a background thread owned by the panel,
                # not the UI process.
                method = getattr(self.backend, "optical_pose_capture_burst_wiris_ftp", None)
                if not callable(method):
                    raise RuntimeError("WIRIS FTP frozen-burst capture owner is not available")
                result = method(values)
            elif command_id == "alignment.pose.capture_burst_wiris_ftp_oz":
                method = getattr(self.backend, "optical_pose_capture_burst_wiris_ftp_oz", None)
                if not callable(method):
                    raise RuntimeError("WIRIS FTP (OZ) frozen-burst capture owner is not available")
                result = method(values)
            elif command_id == "alignment.pose.auto_select_horizon":
                method = getattr(self.backend, "optical_pose_auto_select_horizon", None)
                if not callable(method):
                    raise RuntimeError("automatic Horizon selector owner is not available")
                result = method(values)
            elif command_id == "alignment.pose.auto_select_mountain":
                method = getattr(self.backend, "optical_pose_auto_select_mountain", None)
                if not callable(method):
                    raise RuntimeError("automatic Mountain selector owner is not available")
                result = method(values)
            elif command_id == "alignment.pose.use_suggestion":
                method = getattr(self.backend, "optical_pose_accept_suggestion", None)
                if not callable(method):
                    raise RuntimeError("automatic suggestion approval owner is not available")
                result = method(values)
            elif command_id == "alignment.pose.annotation_changed":
                method = getattr(self.backend, "optical_pose_record_annotation", None)
                if not callable(method):
                    raise RuntimeError("optical-pose annotation persistence owner is not available")
                result = method(values)
            elif command_id == "alignment.pose.clear_auto_suggestions":
                method = getattr(self.backend, "optical_pose_clear_auto_suggestions", None)
                if not callable(method):
                    raise RuntimeError("automatic suggestion clear owner is not available")
                result = method(values)
            elif command_id == "alignment.pose.register_reference":
                method = getattr(self.backend, "optical_pose_register_reference_candidate", None)
                if not callable(method):
                    raise RuntimeError("reference candidate owner is not available")
                result = method(values)
            elif command_id == "alignment.mountain_ridge_margin.set":
                method = getattr(self.backend, "optical_pose_set_mountain_ridge_margin", None)
                if not callable(method):
                    raise RuntimeError("Mountain ridge margin owner is not available")
                result = method(values)
            elif command_id == "alignment.mountain_max_points.set":
                method = getattr(self.backend, "optical_pose_set_mountain_max_points", None)
                if not callable(method):
                    raise RuntimeError("Mountain max points owner is not available")
                result = method(values)
            elif command_id == "alignment.pose.import_terrain":
                method = getattr(self.backend, "optical_pose_import_terrain", None)
                if not callable(method):
                    raise RuntimeError("terrain GeoTIFF import owner is not available")
                source_path, _selected_filter = QtWidgets.QFileDialog.getOpenFileName(
                    self,
                    "Import terrain GeoTIFF",
                    "",
                    "GeoTIFF (*.tif *.tiff);;All files (*)",
                )
                if not source_path:
                    self.statusBar().showMessage("Terrain GeoTIFF import cancelled", 2500)
                    return
                vertical_datum, confirmed = QtWidgets.QInputDialog.getText(
                    self,
                    "Terrain vertical datum",
                    "Vertical datum of this GeoTIFF's height values (e.g. EGM96, WGS84 "
                    "ellipsoid). Leave blank only if genuinely unknown -- Terrain PnP "
                    "solving stays blocked until this is set to something other than "
                    "unknown.",
                )
                if not confirmed:
                    self.statusBar().showMessage("Terrain GeoTIFF import cancelled", 2500)
                    return
                values["path"] = source_path
                values["vertical_datum"] = vertical_datum.strip() or "unknown"
                result = method(values)
            else:
                method_name = {
                    "alignment.pose.solve_horizon": "optical_pose_solve_horizon",
                    "alignment.pose.solve_terrain": "optical_pose_solve_terrain",
                    "alignment.pose.solve_gnss_mountain": "optical_pose_solve_gnss_mountain",
                    "alignment.pose.solve_drone_yaw": "optical_pose_solve_drone_yaw",
                    "alignment.pose.initialize_telemetry": "optical_pose_initialize_telemetry",
                    "alignment.pose.solve_drone_rtk_multiview": "optical_pose_solve_drone_rtk_multiview",
                    "alignment.pose.solve_combined": "optical_pose_solve_combined",
                }.get(command_id)
                method = getattr(self.backend, str(method_name), None)
                if not method_name or not callable(method):
                    raise RuntimeError(f"pose solver owner is not available: {command_id}")
                if command_id == "alignment.pose.solve_drone_yaw" and not (
                    values.get("drone_yaw") or values.get("observations")
                ):
                    source_path, _selected_filter = QtWidgets.QFileDialog.getOpenFileName(
                        self,
                        "Import Drone RTK yaw observation JSON",
                        "",
                        "JSON (*.json);;All files (*)",
                    )
                    if not source_path:
                        self.statusBar().showMessage(
                            "Drone RTK observation import cancelled", 2500
                        )
                        return
                    values["drone_yaw_path"] = source_path
                if command_id == "alignment.pose.solve_drone_rtk_multiview" and not (
                    values.get("drone_rtk_multiview")
                    or values.get("multiview")
                    or values.get("drone_rtk_multiview_path")
                ):
                    source_path, _selected_filter = QtWidgets.QFileDialog.getOpenFileName(
                        self,
                        "Import Drone RTK multi-view observation JSON",
                        "",
                        "JSON (*.json);;All files (*)",
                    )
                    if not source_path:
                        self.statusBar().showMessage(
                            "Drone RTK multi-view import cancelled", 2500
                        )
                        return
                    values["drone_rtk_multiview_path"] = source_path
                if command_id == "alignment.pose.solve_terrain" and not (
                    values.get("correspondences") or values.get("correspondences_path")
                ):
                    source_path, _selected_filter = QtWidgets.QFileDialog.getOpenFileName(
                        self,
                        "Import Terrain PnP object-point correspondences JSON",
                        "",
                        "JSON (*.json);;All files (*)",
                    )
                    if not source_path:
                        self.statusBar().showMessage(
                            "Terrain PnP correspondence import cancelled", 2500
                        )
                        return
                    values["correspondences_path"] = source_path
                result = method(values)
        except Exception as exc:
            # This is the *only* feedback when a command never produces a
            # suggestion/candidate at all (e.g. auto-select failing before
            # _require_frozen_intrinsics() clears) -- give it as much room as
            # the "not eligible" case below, not the default 3s toast.
            self.statusBar().showMessage(f"{command_id}: failed — {exc}", 10000)
            if command_id in {
                "alignment.pose.auto_select_horizon",
                "alignment.pose.auto_select_mountain",
            }:
                # The status bar toast fades in 10s; keep the reason visible
                # next to the Auto-select buttons until the next attempt.
                self.alignment.optical_pose_pane.set_auto_suggestion_error(command_id, str(exc))
            return
        snapshot = result if isinstance(result, dict) and "candidates" in result else self.backend.optical_pose_snapshot()
        if isinstance(snapshot, dict):
            self.alignment.optical_pose_pane.apply_snapshot(snapshot)
        self.inspector.refresh(self._current_context, self.status_adapter.model)
        message = f"{command_id}: accepted"
        duration_ms = 3000
        if command_id in {
            "alignment.pose.auto_select_horizon",
            "alignment.pose.auto_select_mountain",
        } and isinstance(snapshot, dict):
            suggestions = snapshot.get("auto_suggestions")
            latest = suggestions[-1] if isinstance(suggestions, list) and suggestions else None
            if isinstance(latest, dict):
                pose_state = str(latest.get("state") or "UNKNOWN")
                confidence = latest.get("confidence")
                confidence_text = f"{float(confidence):.2f}" if isinstance(confidence, (int, float)) else "--"
                if latest.get("eligible", False):
                    message = f"{command_id}: {pose_state} (confidence={confidence_text})"
                else:
                    reason = str(latest.get("failure_reason") or "no eligible detection")
                    message = f"{command_id}: {pose_state} — {reason}"
                    duration_ms = 8000
        self.statusBar().showMessage(message, duration_ms)

    def _handle_inspector_command(self, command_id: str, payload: object) -> None:
        """Route Inspector actions through the same owner/command boundaries."""

        if str(command_id).startswith("alignment.pose.") or command_id in (
            "alignment.mountain_ridge_margin.set",
            "alignment.mountain_max_points.set",
        ):
            self._handle_optical_pose_command(command_id, payload)
            return
        try:
            self.commands.invoke(str(command_id))
        except Exception as exc:
            self.statusBar().showMessage(f"{command_id}: {exc}", 6000)
            return
        self._refresh_visible_surfaces(self.status_adapter.model.snapshot())
        self.inspector.refresh(self._current_context, self.status_adapter.model)

    @staticmethod
    def _scroll_container(
        widget: QtWidgets.QWidget,
        object_name: str,
        parent: QtWidgets.QWidget | None = None,
    ) -> QtWidgets.QScrollArea:
        scroll = QtWidgets.QScrollArea(parent)
        scroll.setObjectName(object_name)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        scroll.setWidget(widget)
        return scroll

    def _restore_ui_state(self) -> None:
        geometry = self._ui_settings.value("geometry")
        if isinstance(geometry, (bytes, bytearray)):
            self.restoreGeometry(bytes(geometry))
        state = self._ui_settings.value("dock_state")
        if isinstance(state, (bytes, bytearray)):
            self.restoreState(bytes(state))
        context = str(self._ui_settings.value("last_context", "overview") or "overview")
        self.navigation.select_context(context)

    def _save_ui_state(self) -> None:
        self._ui_settings.setValue("geometry", self.saveGeometry())
        self._ui_settings.setValue("dock_state", self.saveState())
        self._ui_settings.setValue("last_context", self._current_context)
        self._ui_settings.sync()

    def _apply_style(self) -> None:
        self.setStyleSheet(
            f"""
            QWidget#workstationMainWindow {{ background: {APP_BG}; color: {TEXT_MAIN}; }}
            QMainWindow, QDockWidget, QToolBar, QStatusBar {{ font-family: {FONT_FAMILY}; }}
            QDockWidget::title {{ background: {PANEL_ALT_BG}; border: 1px solid {PANEL_BORDER}; padding: 4px 6px; }}
            QToolBar#primaryToolbar {{ spacing: 3px; padding: 2px 4px; border-bottom: 1px solid {PANEL_BORDER}; }}
            QToolButton {{ min-height: 26px; padding: 2px 8px; }}
            QTreeView {{ background: #ffffff; border: 1px solid {PANEL_BORDER}; }}
            QGroupBox {{ border: 1px solid {PANEL_BORDER}; margin-top: 7px; padding-top: 5px; }}
            QGroupBox::title {{ subcontrol-origin: margin; left: 7px; padding: 0 3px; color: {TEXT_MUTED}; }}
            QFrame#workstationHeader {{ border-bottom: 1px solid {PANEL_BORDER}; }}
            QLabel#engineeringLabel {{ color: {TEXT_MUTED}; }}
            QLabel#workspaceSubtitle,
            QLabel#engineeringMeta,
            QLabel#cameraPreviewMeta,
            QLabel#statusIndicatorDetail,
            QLabel#statusStrip,
            QLabel#compactContract,
            QLabel#protocolCheck {{ color: {TEXT_MUTED}; }}
            QFrame#cameraPreview {{ background: #161616; border: 1px solid #525252; }}
            QLabel#cameraPreviewTitle, QLabel#cameraPreviewMeta {{ color: #ffffff; }}
            QLabel#cameraPreviewImage {{ background: #262626; color: #c6c6c6; }}
            QLabel#measurementResultStatus {{ padding: 6px; border: 1px solid {PANEL_BORDER}; background: #ffffff; }}
            """
        )

    def _select_context(self, context_id: str) -> None:
        context_id = canonical_context_id(context_id)
        self._current_context = context_id
        if context_id == "overview":
            page = "overview"
        elif context_id.startswith("devices.cameras") or context_id in {"processing.detection"}:
            page = "camera"
        elif context_id == "devices.time":
            page = "time"
        elif context_id in {"processing.measurement"}:
            page = "measurement"
        elif context_id in {"tasks.recording"}:
            page = "recording"
        elif context_id in {"devices.pixhawk", "devices.pixhawk.p1", "devices.pixhawk.p2"}:
            page = "pixhawk"
        elif context_id == "devices.telemetry":
            page = "telemetry"
        elif context_id == "devices.gnss":
            page = "base_gnss"
        elif context_id == "processing.alignment":
            page = "alignment"
        elif context_id in {"tasks.calibration"}:
            page = "calibration"
        elif context_id.startswith("diagnostics"):
            self.diagnostics_dock.show()
            self.diagnostics.select_tab_for_context(context_id)
            page = self.workspace_host.currentWidget()
        elif context_id in {"tasks.advanced", "tasks.playback", "tasks.export"}:
            page = "advanced"
        elif context_id in {"tasks.settings"}:
            self.settings_workspace.refresh_from_backend()
            page = "settings"
        else:
            page = "overview"
        if isinstance(page, str):
            self.workspace_host.setCurrentIndex(self._workspace_indexes[page])
        if page == "alignment":
            self.alignment.select_tab_for_context(context_id)
        self._refresh_visible_surfaces(self.status_adapter.model.snapshot())
        self.inspector.refresh(context_id, self.status_adapter.model)
        if context_id.startswith("diagnostics"):
            self.diagnostics.select_tab_for_context(context_id)
        if context_id not in {"overview"} and self.navigation.currentIndex().data(QtCore.Qt.UserRole) != context_id:
            self.navigation.select_context(context_id)

    def _open_advanced_zoom(self) -> None:
        """Open the shared Advanced camera-control surface for physical zoom."""

        self._select_context("tasks.advanced")
        if hasattr(self, "advanced") and hasattr(self.advanced, "tabs"):
            self.advanced.tabs.setCurrentIndex(0)

    def _inspect_selection(self, context_id: str, selection: object) -> None:
        if not isinstance(selection, dict):
            return
        context_id = canonical_context_id(context_id)
        if context_id != self._current_context:
            return
        self.inspector.inspect_selection(context_id, selection, self.status_adapter.model)

    def _toggle_camera(self) -> object:
        return self.backend.toggle_camera()

    def _toggle_detection(self) -> object:
        return self.backend.toggle_detection()

    def _toggle_recording(self) -> object:
        return self.backend.toggle_recording()

    def _save_session(self) -> object:
        return self.backend.save_session()

    def _camera_discover_host_ip(self) -> object:
        return self.backend.camera_discover_host_ip()

    def _camera_auto_connect(self) -> object:
        return self.backend.camera_auto_connect()

    def _camera_check_rtsp(self) -> object:
        return self.backend.camera_check_rtsp()

    def _camera_check_control(self) -> object:
        return self.backend.camera_check_control()

    def _camera_enable_rtsp(self) -> object:
        return self.backend.camera_enable_rtsp()

    def _camera_reactivate(self, camera: str) -> object:
        return self.backend.camera_reactivate(camera)

    def _camera_swap(self) -> object:
        return self.backend.camera_swap()

    def _camera_save_settings(self) -> object:
        return self.backend.camera_save_settings()

    def _time_set_pc(self) -> object:
        return self.backend.time_set_pc()

    def _time_check_sync(self) -> object:
        return self.backend.time_check_sync()

    def _time_apply_settings(self) -> object:
        return self.backend.apply_measurement_settings(self.time_workspace.settings_payload())

    def _time_save_settings(self) -> object:
        return self.backend.save_measurement_settings()

    def _pixhawk_toggle_mavlink(self) -> object:
        return self.backend.pixhawk_toggle_mavlink()

    def _pixhawk_connect_mavlink(self) -> object:
        return self.backend.pixhawk_connect_mavlink()

    def _pixhawk_disconnect_mavlink(self) -> object:
        return self.backend.pixhawk_disconnect_mavlink()

    def _pixhawk_reset_level_calibration(self) -> object:
        return self.backend.pixhawk_reset_level_calibration()

    def _pixhawk_refresh_parameters(self, camera: str) -> object:
        return self.backend.advanced_refresh_px4_calibration_params(camera, read_params=True)

    def _pixhawk_reset_calibration(self, camera: str, group: str) -> object:
        return self.backend.pixhawk_reset_calibration(camera, group)

    def _pixhawk_recover_eth_mavlink(self, camera: str) -> object:
        return self.backend.pixhawk_recover_eth_mavlink(camera)

    def _pixhawk_discover_usb(self) -> object:
        return self.backend.pixhawk_discover_usb()

    def _pixhawk_discover_mavlink_candidates(self) -> object:
        return self.backend.pixhawk_discover_mavlink_candidates()

    def _pixhawk_discover_host_ip(self) -> object:
        return self.backend.pixhawk_discover_host_ip()

    def _telemetry_apply(self) -> object:
        return self.backend.drone_telemetry_configure(self.telemetry.payload())

    def _telemetry_connect(self) -> object:
        return self.backend.drone_telemetry_connect(self.telemetry.payload())

    def _telemetry_disconnect(self) -> object:
        return self.backend.drone_telemetry_disconnect()

    def _telemetry_save_settings(self) -> object:
        return self.backend.drone_telemetry_save_settings(self.telemetry.payload())

    def _telemetry_discover_usb(self) -> object:
        return self.backend.drone_telemetry_discover_usb()

    def _gnss_base_discover(self) -> object:
        return self.backend.gnss_base_discover()

    def _gnss_base_connect(self) -> object:
        return self.backend.gnss_base_connect()

    def _gnss_base_disconnect(self) -> object:
        return self.backend.gnss_base_disconnect()

    def _gnss_survey_start(self) -> object:
        return self.backend.gnss_survey_start()

    def _gnss_survey_stop(self) -> object:
        return self.backend.gnss_survey_stop()

    def _gnss_ntrip_start(self) -> object:
        return self.backend.gnss_ntrip_start()

    def _gnss_ntrip_stop(self) -> object:
        return self.backend.gnss_ntrip_stop()

    def _gnss_ntrip_forget_password(self) -> object:
        return self.backend.gnss_ntrip_forget_password()

    def _calibration_capture_images(self) -> object:
        return self.backend.calibration_capture_images()

    def _calibration_capture_images_wiris_ftp(self) -> object:
        return self.backend.calibration_capture_images_wiris_ftp()

    def _calibration_capture_images_wiris_ftp_oz(self) -> object:
        return self.backend.calibration_capture_images_wiris_ftp_oz()

    def _calibration_toggle_auto_capture(self) -> object:
        panel = getattr(self.backend, "panel", None)
        controller = getattr(panel, "_calibration_workflow_controller_owner", None)
        active = bool(getattr(controller, "_auto_capture_active", False))
        if not active:
            self.backend.calibration_apply_setup(self.calibration.setup_payload())
        return self.backend.calibration_toggle_auto_capture()

    def _calibration_run_intrinsic(self) -> object:
        self.backend.calibration_apply_setup(self.calibration.setup_payload())
        return self.backend.calibration_run_intrinsic()

    def _calibration_run_extrinsics(self) -> object:
        self.backend.calibration_apply_setup(self.calibration.setup_payload())
        return self.backend.calibration_run_extrinsics()

    def _calibration_save_intrinsics(self) -> object:
        return self.backend.calibration_save_intrinsics()

    def _calibration_save_extrinsics(self) -> object:
        return self.backend.calibration_save_extrinsics()

    def _calibration_load_intrinsics(self) -> object:
        return self.backend.calibration_load_intrinsics()

    def _calibration_load_extrinsics(self) -> object:
        return self.backend.calibration_load_extrinsics()

    def _calibration_apply_bundle(self) -> object:
        return self.backend.calibration_apply_bundle()

    def _measurement_apply_settings(self) -> object:
        return self.backend.apply_measurement_settings(self.measurement.settings_payload())

    def _measurement_save_settings(self) -> object:
        return self.backend.save_measurement_settings()

    def _settings_apply(self) -> object:
        # The NTRIP password field lives only on this workspace widget, never
        # in settings_payload()'s JSON-safe dict (see NetworkConfig /
        # ntrip_credentials.py). It needs its own explicit handoff here,
        # where both this workspace and the backend are in scope.
        self.backend.set_gnss_ntrip_session_password(self.settings_workspace.gnss_network_password_edit.text())
        return self.backend.apply_workstation_system_settings(self.settings_workspace.settings_payload())

    def _settings_save(self) -> object:
        return self.backend.save_workstation_system_settings()

    def _calibration_apply_setup(self) -> object:
        return self.backend.calibration_apply_setup(self.calibration.setup_payload())

    def _calibration_save_setup(self) -> object:
        self.backend.calibration_apply_setup(self.calibration.setup_payload())
        return self.backend.calibration_save_setup()

    def _alignment_scan_sources(self) -> object:
        return self.backend.alignment_scan_sources()

    def _alignment_start_sampling(self) -> object:
        return self.backend.alignment_start_sampling()

    def _alignment_stop_sampling(self) -> object:
        return self.backend.alignment_stop_sampling()

    def _alignment_save_profile(self) -> object:
        return self.backend.alignment_save_profile()

    def _alignment_load_profile(self) -> object:
        return self.backend.alignment_load_profile()

    def _alignment_swap_pixhawks(self) -> object:
        return self.backend.alignment_swap_pixhawks()

    def _alignment_query_lrf(self) -> object:
        return self.backend.query_lrf_once()

    def _advanced_wiris_send(self) -> object:
        return self.backend.advanced_send_wiris_command(
            str(self.advanced.wiris_target_combo.currentData() or "both"),
            self.advanced.wiris_command_edit.text(),
            str(self.advanced.wiris_terminator_combo.currentData() or "\n"),
        )

    def _advanced_zoom(self, action: str) -> object:
        level = self.advanced.zoom_level_spin.value() if action == "set" else None
        return self.backend.advanced_send_zoom(
            str(self.advanced.zoom_target_combo.currentData() or "both"),
            action,
            level,
        )

    def _advanced_watch_add_mavlink(self) -> object:
        result = self.backend.advanced_add_custom_mavlink_watch(
            self.advanced.watch_name_edit.text(),
            self.advanced.watch_message_edit.text(),
            self.advanced.watch_field_edit.text(),
        )
        self.advanced._refresh_custom_watch_selector()
        return result

    def _advanced_watch_discover_mavlink(self) -> object:
        result = self.backend.advanced_discover_custom_mavlink_watch()
        self.advanced._refresh_custom_watch_selector()
        return result

    def _advanced_watch_remove(self) -> object:
        key = str(self.advanced.watch_remove_combo.currentData() or "")
        result = self.backend.advanced_remove_custom_mavlink_watch(key)
        self.advanced._refresh_custom_watch_selector()
        return result

    def _advanced_baseline_scan(self) -> object:
        result = self.backend.advanced_refresh_baseline_fields()
        self.advanced.apply_baseline_snapshot(result)
        return result

    def _advanced_px4_refresh_mag(self) -> object:
        return self.backend.advanced_refresh_px4_mag(self.advanced._selected_px4_side(), read_params=True)

    def _advanced_px4_refresh_attitude(self) -> object:
        return self.backend.advanced_refresh_px4_attitude_cal(
            self.advanced._selected_px4_side(), read_params=True
        )

    def _advanced_px4_shell(self) -> object:
        return self.backend.advanced_open_px4_shell(self.advanced._selected_px4_side())

    def _advanced_px4_apply_mag(self) -> object:
        return self.backend.advanced_apply_px4_mag_priorities(
            self.advanced._selected_px4_side(), self.advanced.px4_priority_payload()
        )

    def _advanced_px4_save_mag(self) -> object:
        return self.backend.advanced_save_px4_mag_params(self.advanced._selected_px4_side())

    def _advanced_px4_reboot(self) -> object:
        return self.backend.advanced_reboot_px4(self.advanced._selected_px4_side())

    def _advanced_preview_refresh(self) -> object:
        result = self.backend.advanced_refresh_calibration_preview()
        self.advanced.apply_preview_snapshot(result)
        return result

    def _advanced_media_refresh(self) -> object:
        self.advanced.refresh_media()
        return True

    def _advanced_media_preview(self) -> object:
        return self.advanced.media_preview()

    def _advanced_media_open_session(self) -> object:
        return self.advanced.media_open_session()

    def _advanced_media_open_artifact(self) -> object:
        return self.advanced.media_open_artifact()

    def _advanced_media_open_video(self) -> object:
        return self.advanced.media_open_video()

    def _refresh_from_status(self, snapshot: object) -> None:
        if not isinstance(snapshot, dict):
            snapshot = self.status_adapter.model.snapshot()
        for key, indicator in self.status_widgets.items():
            indicator.set_status(snapshot.get(key))
        recording = snapshot.get("recording")
        recording_state = recording.state if recording else "IDLE"
        recording_locked = recording_state in {"ACTIVE", "FINALIZING"}
        panel = getattr(self.backend, "panel", self.backend)
        camera_running = bool(getattr(panel, "camera_connected", False)) or any(
            status is not None and status.state in {"LIVE", "STARTING"}
            for status in (snapshot.get("camera.left"), snapshot.get("camera.right"))
        )
        clock_change_active = getattr(panel, "wiris_time_sync_thread", None) is not None
        # These controls are safety gates shared by several hidden pages, so
        # keep their enabled state current without rebuilding those pages.
        self.measurement.refresh_run_controls(snapshot, panel)
        rtsp_operation_active = any(
            getattr(panel, attribute, None) is not None
            for attribute in ("_rtsp_reactivation_side", "rtsp_enable_thread", "rtsp_check_thread")
        )
        for command_id in (
            "camera.swap",
            "camera.save_settings",
        ):
            button = self.camera.operation_buttons.get(command_id)
            if button is not None:
                button.setEnabled(not recording_locked)
        for command_id in ("camera.reactivate_left", "camera.reactivate_right"):
            button = self.camera.operation_buttons.get(command_id)
            if button is not None:
                button.setEnabled(not recording_locked and not rtsp_operation_active)
        time_set_button = self.time_workspace.operation_buttons.get("time.set_pc")
        if time_set_button is not None:
            time_set_button.setEnabled(not recording_locked)
        swap_button = self.alignment.operation_buttons.get("alignment.swap_pixhawks")
        if swap_button is not None:
            swap_button.setEnabled(not recording_locked)
        reset_level_button = self.alignment.operation_buttons.get("pixhawk.reset_level_calibration")
        if reset_level_button is not None:
            reset_level_button.setEnabled(not recording_locked)
        self.commands.set_enabled("camera.toggle", not recording_locked)
        self.commands.set_enabled("camera.auto_connect", not recording_locked)
        self.commands.set_enabled("camera.swap", not recording_locked)
        self.commands.set_enabled("camera.save_settings", not recording_locked)
        self.commands.set_enabled("camera.reactivate_left", not recording_locked and not rtsp_operation_active)
        self.commands.set_enabled("camera.reactivate_right", not recording_locked and not rtsp_operation_active)
        self.commands.set_enabled("alignment.swap_pixhawks", not recording_locked)
        self.commands.set_enabled("pixhawk.toggle_mavlink", not recording_locked)
        pixhawk_states = {
            str(snapshot.get(key).state).upper()
            for key in ("pixhawk.left", "pixhawk.right")
            if snapshot.get(key) is not None
        }
        pixhawk_connected = bool(pixhawk_states & {"LIVE", "CONNECTED", "STARTING"})
        pixhawk_connecting = getattr(panel, "pixhawk_connection_thread", None) is not None
        pixhawk_active = pixhawk_connected or pixhawk_connecting
        self.commands.set_enabled("pixhawk.connect_mavlink", not recording_locked and not pixhawk_active)
        self.commands.set_enabled("pixhawk.disconnect_mavlink", not recording_locked and pixhawk_active)
        self.commands.set_enabled("pixhawk.reset_level_calibration", not recording_locked)
        for side_code in ("p1", "p2"):
            for calibration in PX4_CALIBRATION_GROUPS:
                if calibration.resettable:
                    self.commands.set_enabled(
                        f"pixhawk.reset_calibration.{side_code}.{calibration.key}",
                        not recording_locked,
                    )
        bootstrap_active = getattr(panel, "pixhawk_usb_bootstrap_thread", None) is not None
        for command_id in (
            "pixhawk.recover_eth_mavlink.p1",
            "pixhawk.recover_eth_mavlink.p2",
        ):
            self.commands.set_enabled(command_id, not recording_locked and not bootstrap_active)
        self.commands.set_enabled("time.set_pc", not recording_locked)
        # Recording may be armed before cameras are live; samples begin when
        # the source streams become available. Once a stream is running, a
        # mode switch would reopen the active session's video paths.
        self.commands.set_enabled("detection.toggle", not (recording_locked and camera_running))
        self.commands.set_enabled(
            "recording.toggle",
            recording_state not in {"FINALIZING", "RECOVERABLE"}
            and (recording_locked or not clock_change_active),
        )
        has_session_data = bool(getattr(panel, "recording_session_id", "")) or bool(getattr(panel, "distance_history", []))
        self.commands.set_enabled(
            "session.save",
            not recording_locked and (has_session_data or recording_state == "RECOVERABLE"),
        )
        camera_state = snapshot.get("camera.left")
        self.commands.set_text("camera.toggle", "Stop preview" if camera_state and camera_state.state in {"LIVE", "STARTING"} else "Preview")
        detection = snapshot.get("detection")
        self.commands.set_text("detection.toggle", "Stop detection" if detection and detection.state in {"RUNNING", "STARTING"} else "Detection")
        self.commands.set_text("recording.toggle", "Stop recording" if recording and recording.state in {"ACTIVE", "FINALIZING"} else "Record")
        self.commands.set_text(
            "pixhawk.toggle_mavlink",
            "Disconnect Pixhawk MAVLink" if pixhawk_connected else "Connect Pixhawk MAVLink",
        )

    def _refresh_evidence_from_poll(self, snapshot: object) -> None:
        if not isinstance(snapshot, dict):
            snapshot = self.status_adapter.model.snapshot()
        self._refresh_visible_surfaces(snapshot)
        self.inspector.refresh(self._current_context, self.status_adapter.model)

    def _refresh_visible_surfaces(self, snapshot: dict[str, RuntimeStatus]) -> None:
        """Refresh only UI surfaces that can currently be observed.

        Camera frames have their own signals, while status evidence arrives at
        4 Hz.  Rebuilding every hidden table on every status tick caused
        avoidable UI-thread work and repeated file scans during long runs.
        """

        container = self.workspace_host.currentWidget()
        current = container.widget() if isinstance(container, QtWidgets.QScrollArea) else container
        refresh = getattr(current, "refresh", None)
        if callable(refresh):
            refresh(snapshot)
        if self.diagnostics_dock.isVisible():
            self.diagnostics.refresh(snapshot)

    def _show_command_error(self, command_id: str, message: str) -> None:
        label = self.commands.action(command_id).text() if command_id in self.commands._actions else command_id
        self.statusBar().showMessage(f"{label}: {message}", 5000)

    def _show_command_accepted(self, command_id: str, _result: object) -> None:
        label = self.commands.action(command_id).text() if command_id in self.commands._actions else command_id
        self.statusBar().showMessage(f"{label}: started", 3000)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # noqa: N802
        if self._closing:
            event.ignore()
            return
        self._closing = True
        if not bool(self.backend.shutdown_for_close()):
            event.ignore()
            self._closing = False
            QtCore.QTimer.singleShot(250, self.close)
            return
        self._save_ui_state()
        self.status_adapter.stop()
        super().closeEvent(event)
