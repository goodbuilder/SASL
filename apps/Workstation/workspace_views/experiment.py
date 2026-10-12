"""Experiment preparation and readiness presentation."""

from __future__ import annotations

from PyQt5 import QtCore, QtWidgets
from Workstation.ui.components import WorkspaceHeader
from Workstation.application.status_model import RuntimeStatus
from .shared import _evidence


class ExperimentWorkspace(QtWidgets.QWidget):
    """Guided view of the field protocol using existing owner evidence."""

    open_context = QtCore.pyqtSignal(str)

    def __init__(self, backend: object, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.backend = backend
        layout = QtWidgets.QVBoxLayout(self)
        self._layout = layout
        self._run_preparation_controller: object | None = None
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(
            WorkspaceHeader(
                "Protocol",
                "0 external prerequisite → 1–11 field protocol · evidence follows the operator order",
            )
        )
        prerequisite_row = QtWidgets.QHBoxLayout()
        note = QtWidgets.QLabel(
            "QGC Pixhawk Sensor Calibration is completed outside Workstation. "
            "Camera intrinsic/stereo calibration remains available in Calibration before the run."
        )
        note.setObjectName("compactContract")
        note.setWordWrap(True)
        prerequisite_row.addWidget(note, 1)
        self.pre_run_calibration_button = QtWidgets.QPushButton("Open pre-run calibration")
        self.pre_run_calibration_button.clicked.connect(
            lambda _checked=False: self.open_context.emit("tasks.calibration")
        )
        prerequisite_row.addWidget(self.pre_run_calibration_button)
        layout.addLayout(prerequisite_row)

        protocol_group = QtWidgets.QGroupBox("Protocol")
        protocol_layout = QtWidgets.QGridLayout(protocol_group)
        protocol_layout.setColumnStretch(1, 1)
        protocol_layout.setColumnStretch(2, 3)
        protocol_layout.addWidget(QtWidgets.QLabel("Step"), 0, 0)
        protocol_layout.addWidget(QtWidgets.QLabel("State"), 0, 1)
        protocol_layout.addWidget(QtWidgets.QLabel("Evidence / action"), 0, 2)
        protocol_layout.addWidget(QtWidgets.QLabel("Open"), 0, 3)
        self.step_states: dict[int, QtWidgets.QLabel] = {}
        self.step_details: dict[int, QtWidgets.QLabel] = {}
        self.step_buttons: dict[int, QtWidgets.QPushButton] = {}
        self.step_action_buttons: dict[int, tuple[QtWidgets.QPushButton, ...]] = {}
        steps = (
            (0, "QGC Pixhawk calibration", "External", None),
            (
                1,
                "Connect and verify WIRIS / Pixhawk / Base GNSS",
                "",
                (
                    ("Cameras", "devices.cameras"),
                    ("Pixhawk", "devices.pixhawk"),
                    ("Base", "devices.gnss"),
                ),
            ),
            (2, "Start Base Survey-in or NTRIP", "", "devices.gnss"),
            (3, "Initial WIRIS time synchronization", "", "devices.time"),
            (
                4,
                "RTCM injection and numeric fix_type",
                "",
                (("Base", "devices.gnss"), ("Pixhawk", "devices.pixhawk")),
            ),
            (5, "Physical baseline alignment", "", "processing.alignment"),
            (6, "Physical Roll / Pitch / Yaw alignment", "", "processing.alignment"),
            (7, "Repeat physical alignment evidence", "", "processing.alignment"),
            (8, "Final time-sync quality check / resync", "", "devices.time"),
            (9, "YOLO and measurement settings", "", "processing.measurement"),
            (10, "Start measurement", "", "processing.measurement"),
            (11, "Save measured data", "", "tasks.recording"),
        )
        for row, (number, label, initial_state, context) in enumerate(steps, start=1):
            protocol_layout.addWidget(QtWidgets.QLabel(f"{number}. {label}"), row, 0)
            state = QtWidgets.QLabel(initial_state or "WAIT")
            state.setObjectName("protocolState")
            self.step_states[number] = state
            protocol_layout.addWidget(state, row, 1)
            detail = QtWidgets.QLabel("--")
            detail.setObjectName("protocolCheck")
            detail.setWordWrap(True)
            self.step_details[number] = detail
            protocol_layout.addWidget(detail, row, 2)
            if context is None:
                button = QtWidgets.QPushButton("External")
                button.setEnabled(False)
                action_widget: QtWidgets.QWidget = button
                action_buttons = (button,)
            elif isinstance(context, tuple):
                action_widget = QtWidgets.QWidget()
                action_layout = QtWidgets.QHBoxLayout(action_widget)
                action_layout.setContentsMargins(0, 0, 0, 0)
                action_layout.setSpacing(4)
                buttons: list[QtWidgets.QPushButton] = []
                for button_label, target in context:
                    action_button = QtWidgets.QPushButton(button_label)
                    action_button.clicked.connect(
                        lambda _checked=False, target=target: self.open_context.emit(target)
                    )
                    action_layout.addWidget(action_button)
                    buttons.append(action_button)
                action_buttons = tuple(buttons)
                button = action_buttons[0]
            else:
                button = QtWidgets.QPushButton("Open")
                button.clicked.connect(lambda _checked=False, target=context: self.open_context.emit(target))
                action_widget = button
                action_buttons = (button,)
            self.step_buttons[number] = button
            self.step_action_buttons[number] = action_buttons
            protocol_layout.addWidget(action_widget, row, 3)
        layout.addWidget(protocol_group)
        self._protocol_group = protocol_group
        contract = QtWidgets.QLabel(
            "Fix evidence is numeric PX4/MAVLink fix_type; RTK_FIXED=6 is reported as a ratio and is not assumed for every run. "
            "Raw baseline remains NED [north,east,down]; alignment view is P1 body FRD [forward,right,down]."
        )
        contract.setObjectName("compactContract")
        contract.setWordWrap(True)
        layout.addWidget(contract)
        layout.addStretch(1)

    def attach_run_preparation(
        self,
        preflight_group: QtWidgets.QWidget,
        controller: object,
    ) -> None:
        """Place the owner-backed preflight controls in the Protocol workflow."""

        index = self._layout.indexOf(self._protocol_group)
        self._layout.insertWidget(index, preflight_group)
        self._run_preparation_controller = controller

    @staticmethod
    def _state(status: RuntimeStatus | None) -> str:
        return "UNKNOWN" if status is None else str(status.state or "UNKNOWN").upper()

    def _set_step(self, number: int, state: str, detail: str) -> None:
        self.step_states[number].setText(state)
        self.step_details[number].setText(detail)

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        refresh_preparation = getattr(
            self._run_preparation_controller,
            "refresh_run_preparation",
            None,
        )
        if callable(refresh_preparation):
            refresh_preparation(snapshot)
        evidence = _evidence(self.backend)
        camera_states = (self._state(snapshot.get("camera.left")), self._state(snapshot.get("camera.right")))
        pixhawk_states = (self._state(snapshot.get("pixhawk.left")), self._state(snapshot.get("pixhawk.right")))
        gps = {side: evidence.gps(side) for side in ("left", "right")}
        gnss = evidence.gnss_operation()
        routes = gnss.get("routes") if isinstance(gnss.get("routes"), dict) else {}
        physical = evidence.physical_alignment()
        quality = physical.get("quality") if isinstance(physical.get("quality"), dict) else {}
        calibration = evidence.calibration()
        sync_state = self._state(snapshot.get("synchronization"))
        checkpoints = getattr(getattr(self.backend, "panel", self.backend), "workstation_time_sync_checkpoints", ())
        final_sync = any(
            isinstance(item, dict)
            and str(item.get("phase", "")) == "final"
            and "VERIFIED" in str(item.get("status", "")).upper()
            for item in checkpoints or ()
        )

        connected = all(state in {"LIVE", "CONNECTED", "STARTING"} for state in (*camera_states, *pixhawk_states))
        gnss_state = self._state(snapshot.get("gnss"))
        gps_present = (
            gnss_state in {"FIX", "FLOAT", "NO FIX"}
            and all(data.get("fix_type") is not None for data in gps.values())
        )
        mode = str(gnss.get("mode", "UNKNOWN") or "UNKNOWN")
        base_serial = gnss.get("base_serial") if isinstance(gnss.get("base_serial"), dict) else {}
        ntrip = gnss.get("ntrip") if isinstance(gnss.get("ntrip"), dict) else {}
        if mode in {"LOCAL_SURVEY_IN", "NETWORK_ASSISTED_FIXED_BASE"}:
            base_link_ready = bool(base_serial.get("connected", False))
            base_link_text = "CONNECTED" if base_link_ready else "DISCONNECTED"
        elif mode == "NETWORK_RTK_DIRECT":
            base_link_ready = bool(ntrip.get("caster_host") and ntrip.get("mountpoint"))
            base_link_text = "NTRIP CONFIGURED" if base_link_ready else "NTRIP NOT CONFIGURED"
        else:
            base_link_ready = False
            base_link_text = f"MODE {mode}"
        self._set_step(
            1,
            "READY" if connected and gps_present and base_link_ready else "WAIT",
            f"CAM L/R={camera_states[0]}/{camera_states[1]} · P1/P2={pixhawk_states[0]}/{pixhawk_states[1]} · "
            f"BASE={base_link_text} · GPS fix_type={gps['left'].get('fix_type', '--')}/{gps['right'].get('fix_type', '--')}",
        )
        base_frames = int(gnss.get("base_frames") or 0)
        operation_state = str(gnss.get("state", "UNKNOWN") or "UNKNOWN")
        survey_or_ntrip = operation_state in {
            "BASE_SURVEYING",
            "NTRIP_ACTIVE",
            "LOCAL_RTCM_ACTIVE",
            "ROVERS_CONVERGING",
            "READY",
            "MEASURING",
        }
        self._set_step(2, "READY" if survey_or_ntrip or base_frames > 0 else "WAIT", f"operation={operation_state} · base RTCM frames={base_frames}")
        self._set_step(3, "READY" if sync_state == "SYNCED" else "WAIT", f"RTCP/NTP state={sync_state}")
        route_ready = all(
            isinstance(routes.get(side), dict)
            and int(routes[side].get("packets") or 0) > 0
            for side in ("left", "right")
        )
        self._set_step(
            4,
            "READY" if route_ready and gps_present else "WAIT",
            f"RTCM route L/R={'OK' if route_ready else 'WAIT'} · numeric fix_type="
            f"{gps['left'].get('fix_type', '--')}/{gps['right'].get('fix_type', '--')} · RTK_FIXED ratio="
            f"{float(quality.get('fixed_ratio', 0.0)) * 100.0:.1f}%" if quality.get("fixed_ratio") not in (None, "") else
            f"RTCM route L/R={'OK' if route_ready else 'WAIT'} · numeric fix_type="
            f"{gps['left'].get('fix_type', '--')}/{gps['right'].get('fix_type', '--')}",
        )
        alignment_state = str(physical.get("state", "WAITING") or "WAITING")
        relative = physical.get("relative_rpy_deg")
        relative_text = "R/P/Y=--"
        if isinstance(relative, (tuple, list)) and len(relative) == 3:
            relative_text = "R/P/Y=" + "/".join(f"{float(value):+.2f}°" for value in relative)
        sampling_active = bool(physical.get("sampling_active"))
        quality_reasons = {
            str(reason)
            for reason in (quality.get("reasons") or ())
        }
        fix_reasons = {"fix_type_missing", "fix_type_below_minimum", "fixed_ratio_below_minimum"}
        fix_ok = not bool(quality_reasons & fix_reasons)
        baseline_error = quality.get("baseline_error_m")
        baseline_evaluated = isinstance(baseline_error, (tuple, list)) and len(baseline_error) == 3
        baseline_ok = baseline_evaluated and fix_ok and "baseline_frd_out_of_tolerance" not in quality_reasons
        rpy_error = quality.get("relative_rpy_error_deg")
        rpy_evaluated = isinstance(rpy_error, (tuple, list)) and len(rpy_error) == 3
        rpy_ok = rpy_evaluated and fix_ok and "relative_rpy_out_of_tolerance" not in quality_reasons

        def vector_text(value: object, *, suffix: str) -> str:
            if not isinstance(value, (tuple, list)) or len(value) != 3:
                return "--"
            return "/".join(f"{float(item):+.3f}{suffix}" for item in value)

        samples = physical.get("sample_count", quality.get("sample_count", "--"))
        self._set_step(
            5,
            "SAMPLING" if sampling_active else "READY" if baseline_ok else "CHECK" if baseline_evaluated else "WAIT",
            f"P1→P2 FRD error={vector_text(baseline_error, suffix='m')} · samples={samples}",
        )
        self._set_step(
            6,
            "SAMPLING" if sampling_active else "READY" if rpy_ok else "CHECK" if rpy_evaluated else "WAIT",
            f"{relative_text} · error={vector_text(rpy_error, suffix='°')} · samples={samples}",
        )
        iterations = physical.get("iterations")
        iteration_count = len(iterations) if isinstance(iterations, (tuple, list)) else 0
        repeat_ready = iteration_count >= 2 and baseline_ok and rpy_ok
        self._set_step(
            7,
            "SAMPLING" if sampling_active else "READY" if repeat_ready else "CHECK" if iteration_count else "WAIT",
            f"completed iterations={iteration_count}/2 · latest state={alignment_state}",
        )
        self._set_step(8, "READY" if final_sync else "CHECK", f"current={sync_state} · final checkpoint={'recorded' if final_sync else 'not recorded'}")
        calibration_ready = bool(calibration.get("active"))
        measurement_config = evidence.measurement_settings()
        self._set_step(9, "READY" if calibration_ready else "WAIT", f"active calibration={'yes' if calibration_ready else 'no'} · mode={measurement_config.get('distance_mode', '--')}")
        measurement_state = self._state(snapshot.get("measurement"))
        detection_state = self._state(snapshot.get("detection"))
        if measurement_state == "VALID":
            run_state = "VALID"
        elif detection_state in {"RUNNING", "STARTING"}:
            run_state = "RUNNING"
        else:
            run_state = "WAIT"
        self._set_step(
            10,
            run_state,
            f"detection={detection_state} · measurement result={measurement_state}",
        )
        panel = getattr(self.backend, "panel", self.backend)
        has_data = bool(getattr(panel, "recording_session_id", "")) or bool(getattr(panel, "distance_history", ()))
        recording_state = self._state(snapshot.get("recording"))
        if recording_state == "SAVED":
            save_state, save_detail = "SAVED", "session artifacts finalized"
        elif recording_state == "FINALIZING":
            save_state, save_detail = "SAVING", "session save worker running"
        elif recording_state == "RECOVERABLE":
            save_state, save_detail = "RETRY", "retryable spool preserved; save again"
        elif recording_state == "ACTIVE" and bool(
            getattr(panel, "_automatic_session_recording", False)
        ):
            save_state, save_detail = "AUTO", "session is recording with Detection"
        elif has_data:
            save_state, save_detail = "READY", "session details are handled automatically"
        else:
            save_state, save_detail = "WAIT", "session starts with Detection"
        self._set_step(11, save_state, save_detail)
