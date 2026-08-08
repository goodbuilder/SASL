from __future__ import annotations

from PyQt5 import QtCore, QtGui, QtWidgets

from wiris_stereo_distance.theme import (
    ACCENT,
    APP_BG,
    ERROR,
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
from .navigation import NavigationTree
from .settings_workspace import SettingsWorkspace
from .status_model import BackendStatusAdapter, RuntimeStatus
from .workspaces import (
    AlignmentWorkspace,
    AdvancedWorkspace,
    CameraWorkspace,
    CalibrationWorkspace,
    DiagnosticsContent,
    LegacyHandoffWorkspace,
    MeasurementWorkspace,
    OverviewWorkspace,
    RecordingWorkspace,
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
        self._apply_style()
        self._ui_settings = QtCore.QSettings("SASL", "StereoWIRIS Workstation")
        self._restore_ui_state()
        self.status_adapter.model.changed.connect(self._refresh_from_status)
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
        self.commands.register(
            CommandSpec(
                "pixhawk.toggle",
                "Pixhawk ROS2",
                "Connect/disconnect the existing ROS2 Pixhawk subscriber path.",
                "device",
                self._toggle_pixhawk,
            )
        )
        for command_id, text, tooltip, category, handler in (
            (
                "camera.discover_host_ip",
                "Discover camera IP",
                "Discover the camera-network host/interface through the existing owner.",
                "camera",
                self._camera_discover_host_ip,
            ),
            (
                "camera.check_rtsp",
                "Check RTSP",
                "Check RTSP ports and frame reception through the existing camera worker path.",
                "camera",
                self._camera_check_rtsp,
            ),
            (
                "camera.check_control",
                "Check WIRIS control",
                "Check WIRIS control ports through the existing network path.",
                "camera",
                self._camera_check_control,
            ),
            (
                "camera.enable_rtsp",
                "Enable RTSP",
                "Enable WIRIS RTSP using the existing control and camera owner.",
                "camera",
                self._camera_enable_rtsp,
            ),
            (
                "camera.swap",
                "Swap cameras",
                "Swap the existing Left/Right camera mapping without changing coordinate contracts.",
                "camera",
                self._camera_swap,
            ),
            (
                "camera.save_settings",
                "Save camera settings",
                "Save the current camera settings through the existing settings owner.",
                "camera",
                self._camera_save_settings,
            ),
            (
                "pixhawk.auto_detect",
                "Auto-detect Pixhawk",
                "Discover PX4 ROS2 namespaces through the existing Pixhawk owner.",
                "device",
                self._pixhawk_auto_detect,
            ),
            (
                "pixhawk.toggle_mavlink",
                "Pixhawk MAVLink",
                "Connect/disconnect the existing MAVLink Pixhawk path.",
                "device",
                self._pixhawk_toggle_mavlink,
            ),
            (
                "gnss.base_discover",
                "Discover Base serial",
                "Discover GNSS Base serial candidates through the existing owner.",
                "gnss",
                self._gnss_base_discover,
            ),
            (
                "gnss.base_connect",
                "Connect Base serial",
                "Connect the configured GNSS Base serial through the existing worker owner.",
                "gnss",
                self._gnss_base_connect,
            ),
            (
                "gnss.base_disconnect",
                "Disconnect Base serial",
                "Stop the configured GNSS Base serial worker.",
                "gnss",
                self._gnss_base_disconnect,
            ),
            (
                "gnss.survey_start",
                "Start Survey-in",
                "Start local GNSS Base Survey-in through the existing UBX command path.",
                "gnss",
                self._gnss_survey_start,
            ),
            (
                "gnss.survey_stop",
                "Stop Survey-in",
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
        ):
            self.commands.register(CommandSpec(command_id, text, tooltip, category, handler))
        for command_id, text, tooltip, handler in (
            (
                "time.set_pc",
                "Set WIRIS time from PC",
                "Set the selected WIRIS camera clock through the existing owner.",
                self._time_set_pc,
            ),
            (
                "time.run_photo_workflow",
                "Run photo clock workflow",
                "Run CPTR/ICPT, FTP download, and EXIF/GPS clock comparison through the existing owner.",
                self._time_run_photo_workflow,
            ),
            (
                "time.check_sync",
                "Check RTCP/NTP sync",
                "Refresh RTCP/NTP stream timing evidence through the existing owner.",
                self._time_check_sync,
            ),
        ):
            self.commands.register(CommandSpec(command_id, text, tooltip, "time", handler))
        for command_id, text, tooltip, handler in (
            (
                "calibration.capture_images",
                "Capture images",
                "Capture calibration images through the existing checkerboard owner.",
                self._calibration_capture_images,
            ),
            (
                "calibration.run_intrinsic",
                "Run intrinsic calibration",
                "Run the existing intrinsic calibration pipeline without duplicating camera math.",
                self._calibration_run_intrinsic,
            ),
            (
                "calibration.save_intrinsics",
                "Save intrinsics",
                "Save the current intrinsic profile through the existing calibration owner.",
                self._calibration_save_intrinsics,
            ),
            (
                "calibration.load_intrinsics",
                "Load intrinsics",
                "Load and apply the selected intrinsic profile through the existing owner.",
                self._calibration_load_intrinsics,
            ),
            (
                "calibration.apply_intrinsics",
                "Apply intrinsics",
                "Apply the existing intrinsic result to the distance path.",
                self._calibration_apply_intrinsics,
            ),
            (
                "calibration.load_validation",
                "Load validation JSON",
                "Load checkerboard validation evidence through the existing owner.",
                self._calibration_load_validation,
            ),
            (
                "calibration.run_validation",
                "Run validation",
                "Run the existing checkerboard random validation pipeline.",
                self._calibration_run_validation,
            ),
            (
                "calibration.save_validation",
                "Save validation",
                "Save the existing checkerboard validation result.",
                self._calibration_save_validation,
            ),
            (
                "calibration.apply_validation",
                "Apply validation",
                "Apply the existing checkerboard validation result.",
                self._calibration_apply_validation,
            ),
        ):
            self.commands.register(CommandSpec(command_id, text, tooltip, "calibration", handler))
        for command_id, text, tooltip, handler in (
            (
                "measurement.apply_settings",
                "Apply measurement settings",
                "Apply Workstation measurement and synchronization fields through the existing owner.",
                self._measurement_apply_settings,
            ),
            (
                "measurement.save_settings",
                "Save measurement settings",
                "Persist the current measurement settings without changing measurement schemas or coordinate frames.",
                self._measurement_save_settings,
            ),
            (
                "calibration.apply_setup",
                "Apply calibration setup",
                "Apply checkerboard geometry, zoom, path, and validation setup through the existing owner.",
                self._calibration_apply_setup,
            ),
            (
                "calibration.scan_validation",
                "Scan validation files",
                "Scan the selected calibration validation root through the existing owner.",
                self._calibration_scan_validation,
            ),
            (
                "calibration.save_setup",
                "Save calibration setup",
                "Persist the selected calibration/intrinsics path through the existing settings owner.",
                self._calibration_save_setup,
            ),
            (
                "alignment.scan_sources",
                "Scan alignment sources",
                "Refresh ROS2/MAVLink alignment source evidence through the existing owner.",
                self._alignment_scan_sources,
            ),
            (
                "alignment.start_sampling",
                "Start baseline sampling",
                "Start the existing GNSS baseline quality sampling workflow.",
                self._alignment_start_sampling,
            ),
            (
                "alignment.stop_sampling",
                "Stop baseline sampling",
                "Stop the existing GNSS baseline quality sampling workflow.",
                self._alignment_stop_sampling,
            ),
            (
                "alignment.save_profile",
                "Save alignment profile",
                "Save the existing alignment profile without changing NED or camera-frame contracts.",
                self._alignment_save_profile,
            ),
            (
                "alignment.load_profile",
                "Load alignment profile",
                "Load the existing alignment profile through the owner.",
                self._alignment_load_profile,
            ),
            (
                "alignment.swap_pixhawks",
                "Swap Pixhawk sides",
                "Swap P1/P2 mapping through the existing owner; coordinate labels remain explicit.",
                self._alignment_swap_pixhawks,
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
                "Apply all settings",
                "Apply the complete Workstation settings payload through the existing panel owner.",
                self._settings_apply,
            ),
            (
                "settings.save",
                "Save all settings",
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
                "advanced.watch.add",
                "Add field watch",
                "Add a ROS2 custom field watch to the existing Pixhawk subscriber timeline.",
                self._advanced_watch_add,
            ),
            (
                "advanced.watch.discover_ros2",
                "Discover ROS2 watch",
                "Open the existing ROS2 topic/field browser.",
                self._advanced_watch_discover_ros2,
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
                "Refresh ROS2/MAVLink baseline field evidence through the existing owner.",
                self._advanced_baseline_scan,
            ),
            (
                "advanced.baseline.describe",
                "Describe baseline fields",
                "Describe the selected ROS2 message type without changing its producer contract.",
                self._advanced_baseline_describe,
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
                "legacy.open",
                "Advanced owner tools",
                "Open the existing owner for low-level SDK, PX4, field-watch, calibration-preview, and media tools.",
                "view",
                self._open_legacy,
            )
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
        self.inspector = InspectorWidget(self.backend, self._open_legacy)
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
        toolbar.addAction(self.commands.action("pixhawk.toggle"))
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
        file_menu.addAction(self.commands.action("legacy.open"))
        file_menu.addSeparator()
        file_menu.addAction("Quit", self.close, "Ctrl+Q")
        view_menu = self.menuBar().addMenu("View")
        view_menu.addAction(self.navigation_dock.toggleViewAction())
        view_menu.addAction(self.inspector_dock.toggleViewAction())
        view_menu.addAction(self.diagnostics_dock.toggleViewAction())
        device_menu = self.menuBar().addMenu("Device")
        device_menu.addAction(self.commands.action("camera.toggle"))
        device_menu.addAction(self.commands.action("pixhawk.toggle"))
        measurement_menu = self.menuBar().addMenu("Measurement")
        measurement_menu.addAction(self.commands.action("detection.toggle"))
        measurement_menu.addAction(self.commands.action("recording.toggle"))
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
        self.alignment = AlignmentWorkspace(
            self.backend,
            self.status_adapter.model,
            lambda: self._select_context("tasks.advanced"),
            self.commands.invoke,
        )
        self.calibration = CalibrationWorkspace(
            self.backend,
            lambda: self._select_context("tasks.advanced"),
            self.commands.invoke,
        )
        self.recording = RecordingWorkspace(
            self.backend,
            self.status_adapter.model,
            lambda: self._select_context("tasks.advanced"),
            self.commands.invoke,
        )
        self.advanced = AdvancedWorkspace(
            self.backend,
            self.status_adapter.model,
            self.commands.invoke,
        )
        self.settings_workspace = SettingsWorkspace(
            self.backend,
            self.commands.invoke,
        )
        self.legacy_page = QtWidgets.QWidget()
        legacy_layout = QtWidgets.QVBoxLayout(self.legacy_page)
        legacy_layout.setContentsMargins(0, 0, 0, 0)
        owner_widget = getattr(self.backend, "owner_widget", None)
        if isinstance(owner_widget, QtWidgets.QWidget):
            legacy_scroll = QtWidgets.QScrollArea(self.legacy_page)
            legacy_scroll.setObjectName("legacyWorkflowScroll")
            legacy_scroll.setWidgetResizable(False)
            legacy_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
            legacy_scroll.setWidget(owner_widget)
            legacy_scroll.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
            legacy_layout.addWidget(legacy_scroll)
        else:
            legacy_layout.addWidget(
                LegacyHandoffWorkspace(
                    "Advanced owner workflow",
                    "No standalone owner widget was supplied by the backend.",
                    lambda: None,
                )
            )
        self._workspace_indexes: dict[str, int] = {}
        self.workspace_scrolls: dict[str, QtWidgets.QScrollArea] = {}
        for context_id, widget in (
            ("overview", self.overview),
            ("camera", self.camera),
            ("time", self.time_workspace),
            ("measurement", self.measurement),
            ("alignment", self.alignment),
            ("calibration", self.calibration),
            ("recording", self.recording),
            ("advanced", self.advanced),
            ("settings", self.settings_workspace),
            ("legacy", self.legacy_page),
        ):
            scroll = self._scroll_container(widget, f"{context_id.replace('.', '_')}WorkspaceScroll", self.workspace_host)
            self.workspace_scrolls[context_id] = scroll
            self._workspace_indexes[context_id] = self.workspace_host.addWidget(scroll)
        self.overview.open_context.connect(self._select_context)
        self._select_context("overview")

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
            QLabel#workspaceTitle {{ font-size: 16px; font-weight: 600; }}
            QLabel#workspaceSubtitle, QLabel#engineeringMeta, QLabel#cameraPreviewMeta {{ color: {TEXT_MUTED}; }}
            QLabel#engineeringValue {{ font-size: 13px; font-weight: 600; }}
            QFrame#cameraPreview {{ background: #161616; border: 1px solid #525252; }}
            QLabel#cameraPreviewTitle, QLabel#cameraPreviewMeta {{ color: #ffffff; }}
            QLabel#cameraPreviewImage {{ background: #262626; color: #c6c6c6; }}
            QLabel#measurementResultStatus {{ padding: 6px; border: 1px solid {PANEL_BORDER}; background: #ffffff; }}
            """
        )

    def _select_context(self, context_id: str) -> None:
        self._current_context = context_id
        if context_id == "overview":
            page = "overview"
        elif context_id.startswith("devices.cameras") or context_id in {"processing.detection"}:
            page = "camera"
        elif context_id == "devices.time":
            page = "time"
        elif context_id in {"processing.measurement", "processing.synchronization", "tasks.recording"}:
            page = "measurement" if context_id != "tasks.recording" else "recording"
        elif context_id in {"devices.pixhawk", "devices.pixhawk.p1", "devices.pixhawk.p2", "devices.gnss", "processing.alignment"}:
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
        self.inspector.refresh(context_id, self.status_adapter.model)
        if context_id.startswith("diagnostics"):
            self.diagnostics.select_tab_for_context(context_id)
        if context_id not in {"overview"} and self.navigation.currentIndex().data(QtCore.Qt.UserRole) != context_id:
            self.navigation.select_context(context_id)

    def _open_legacy_context(self, context_id: str = "") -> None:
        if context_id:
            selector = getattr(self.backend, "select_owner_context", None)
            if callable(selector):
                selector(context_id)
        self.workspace_host.setCurrentIndex(self._workspace_indexes["legacy"])

    def _open_legacy(self) -> None:
        self._select_context("tasks.advanced")

    def _toggle_camera(self) -> object:
        return self.backend.toggle_camera()

    def _toggle_detection(self) -> object:
        return self.backend.toggle_detection()

    def _toggle_recording(self) -> object:
        return self.backend.toggle_recording()

    def _save_session(self) -> object:
        return self.backend.save_session()

    def _toggle_pixhawk(self) -> object:
        return self.backend.toggle_pixhawk()

    def _camera_discover_host_ip(self) -> object:
        return self.backend.camera_discover_host_ip()

    def _camera_check_rtsp(self) -> object:
        return self.backend.camera_check_rtsp()

    def _camera_check_control(self) -> object:
        return self.backend.camera_check_control()

    def _camera_enable_rtsp(self) -> object:
        return self.backend.camera_enable_rtsp()

    def _camera_swap(self) -> object:
        return self.backend.camera_swap()

    def _camera_save_settings(self) -> object:
        return self.backend.camera_save_settings()

    def _time_set_pc(self) -> object:
        return self.backend.time_set_pc()

    def _time_run_photo_workflow(self) -> object:
        return self.backend.time_run_photo_workflow()

    def _time_check_sync(self) -> object:
        return self.backend.time_check_sync()

    def _pixhawk_auto_detect(self) -> object:
        return self.backend.pixhawk_auto_detect()

    def _pixhawk_toggle_mavlink(self) -> object:
        return self.backend.pixhawk_toggle_mavlink()

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

    def _calibration_capture_images(self) -> object:
        return self.backend.calibration_capture_images()

    def _calibration_run_intrinsic(self) -> object:
        return self.backend.calibration_run_intrinsic()

    def _calibration_save_intrinsics(self) -> object:
        return self.backend.calibration_save_intrinsics()

    def _calibration_load_intrinsics(self) -> object:
        return self.backend.calibration_load_intrinsics()

    def _calibration_apply_intrinsics(self) -> object:
        return self.backend.calibration_apply_intrinsics()

    def _calibration_load_validation(self) -> object:
        return self.backend.calibration_load_validation()

    def _calibration_run_validation(self) -> object:
        return self.backend.calibration_run_validation()

    def _calibration_save_validation(self) -> object:
        return self.backend.calibration_save_validation()

    def _calibration_apply_validation(self) -> object:
        return self.backend.calibration_apply_validation()

    def _measurement_apply_settings(self) -> object:
        return self.backend.apply_measurement_settings(self.measurement.settings_payload())

    def _measurement_save_settings(self) -> object:
        return self.backend.save_measurement_settings()

    def _settings_apply(self) -> object:
        return self.backend.apply_workstation_system_settings(self.settings_workspace.settings_payload())

    def _settings_save(self) -> object:
        return self.backend.save_workstation_system_settings()

    def _calibration_apply_setup(self) -> object:
        return self.backend.calibration_apply_setup(self.calibration.setup_payload())

    def _calibration_scan_validation(self) -> object:
        return self.backend.calibration_scan_validation()

    def _calibration_save_setup(self) -> object:
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

    def _advanced_watch_add(self) -> object:
        result = self.backend.advanced_add_custom_topic_watch(
            self.advanced.watch_name_edit.text(),
            self.advanced.watch_topic_edit.text(),
            self.advanced.watch_field_edit.text(),
        )
        self.advanced._refresh_custom_watch_selector()
        return result

    def _advanced_watch_discover_ros2(self) -> object:
        result = self.backend.advanced_discover_custom_topic_watch()
        self.advanced._refresh_custom_watch_selector()
        return result

    def _advanced_watch_discover_mavlink(self) -> object:
        result = self.backend.advanced_discover_custom_mavlink_watch()
        self.advanced._refresh_custom_watch_selector()
        return result

    def _advanced_watch_remove(self) -> object:
        key = str(self.advanced.watch_remove_combo.currentData() or "")
        result = self.backend.advanced_remove_custom_topic_watch(key)
        self.advanced._refresh_custom_watch_selector()
        return result

    def _advanced_baseline_scan(self) -> object:
        result = self.backend.advanced_refresh_baseline_fields()
        self.advanced.apply_baseline_snapshot(result)
        return result

    def _advanced_baseline_describe(self) -> object:
        topic_type = self.advanced.selected_baseline_topic_type()
        result = self.backend.advanced_describe_baseline_fields(topic_type)
        self.advanced.apply_baseline_field_list(result)
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
        self.overview.refresh(snapshot)
        self.camera.refresh(snapshot)
        self.time_workspace.refresh(snapshot)
        self.measurement.refresh(snapshot)
        self.alignment.refresh(snapshot)
        self.calibration.refresh(snapshot)
        self.recording.refresh(snapshot)
        self.advanced.refresh(snapshot)
        self.diagnostics.refresh(snapshot)
        self.inspector.refresh(self._current_context, self.status_adapter.model)
        for key, indicator in self.status_widgets.items():
            indicator.set_status(snapshot.get(key))
        recording = snapshot.get("recording")
        recording_state = recording.state if recording else "IDLE"
        recording_locked = recording_state in {"ACTIVE", "FINALIZING"}
        self.commands.set_enabled("camera.toggle", not recording_locked)
        # The legacy workflow intentionally permits starting a session before
        # cameras are live; it then records when streams become available.
        self.commands.set_enabled("detection.toggle", True)
        self.commands.set_enabled("recording.toggle", recording_state not in {"FINALIZING", "RECOVERABLE"})
        panel = getattr(self.backend, "panel", None)
        has_session_data = bool(getattr(panel, "recording_session_id", "")) or bool(getattr(panel, "distance_history", []))
        self.commands.set_enabled(
            "session.save",
            not recording_locked and (has_session_data or recording_state == "RECOVERABLE"),
        )
        self.commands.set_enabled("pixhawk.toggle", True)
        camera_state = snapshot.get("camera.left")
        self.commands.set_text("camera.toggle", "Stop preview" if camera_state and camera_state.state in {"LIVE", "STARTING"} else "Preview")
        detection = snapshot.get("detection")
        self.commands.set_text("detection.toggle", "Stop detection" if detection and detection.state in {"RUNNING", "STARTING"} else "Detection")
        self.commands.set_text("recording.toggle", "Stop recording" if recording and recording.state in {"ACTIVE", "FINALIZING"} else "Record")

    def _show_command_error(self, command_id: str, message: str) -> None:
        self.statusBar().showMessage(f"{command_id}: {message}", 5000)

    def _show_command_accepted(self, command_id: str, _result: object) -> None:
        self.statusBar().showMessage(f"{command_id}: accepted; waiting for runtime evidence", 3000)

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
