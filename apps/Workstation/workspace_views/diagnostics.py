"""Read-only system, camera, GNSS and measurement diagnostics."""

from __future__ import annotations

from collections.abc import Mapping
from PyQt5 import QtGui, QtWidgets
from Workstation.workstation_core.coordinate_contract import (
    BASELINE_TRANSLATION_FRAME,
    BODY_FRAME,
    CAMERA_FRAME,
    MEASUREMENT_AXIS_CONTRACT,
    MEASUREMENT_FRAME,
    WORLD_FRAME,
)
from Workstation.workstation_core.runtime_camera_status import (
    decoder_presentation,
    format_camera_input_metrics,
)
from Workstation.application.status_model import RuntimeStatus, StatusModel
from .shared import _display_class, _display_number, _evidence, _sync_table_rows


class DiagnosticsContent(QtWidgets.QWidget):
    def __init__(self, backend: object, status_model: StatusModel, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.backend = backend
        self.status_model = status_model
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        self.tabs = QtWidgets.QTabWidget()
        self.system_model = QtGui.QStandardItemModel(0, 6, self)
        self.system_model.setHorizontalHeaderLabels(["Key", "State", "Level", "Source", "Age", "Detail"])
        self.system_table = QtWidgets.QTableView()
        self.system_table.setModel(self.system_model)
        self.system_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.system_table.verticalHeader().setVisible(False)
        for column, width in ((0, 180), (1, 110), (2, 70), (3, 360), (4, 80)):
            self.system_table.setColumnWidth(column, width)
        self.system_table.horizontalHeader().setStretchLastSection(True)
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self._log_pending_count = 0
        self.log_new_label = QtWidgets.QLabel("")
        self.log_new_label.setObjectName("statusStrip")
        self.log_new_label.hide()
        self.log_follow_button = QtWidgets.QPushButton("Follow latest")
        self.log_follow_button.setObjectName("secondaryAction")
        self.log_follow_button.clicked.connect(self._follow_latest_log)
        self.log_follow_button.hide()
        log_page = QtWidgets.QWidget()
        log_layout = QtWidgets.QVBoxLayout(log_page)
        log_layout.setContentsMargins(0, 0, 0, 0)
        log_toolbar = QtWidgets.QHBoxLayout()
        log_toolbar.addWidget(self.log_new_label)
        log_toolbar.addStretch(1)
        log_toolbar.addWidget(self.log_follow_button)
        log_layout.addLayout(log_toolbar)
        log_layout.addWidget(self.log_view)
        self.measurement_view = QtWidgets.QPlainTextEdit()
        self.measurement_view.setReadOnly(True)
        self.measurement_view.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self.camera_view = self._diagnostic_text_view()
        self.pixhawk_view = self._diagnostic_text_view()
        self.gnss_view = self._diagnostic_text_view()
        self.performance_view = self._diagnostic_text_view()
        self.ftp_view = self._diagnostic_text_view()
        self.tabs.addTab(self.system_table, "System")
        self.tabs.addTab(self.camera_view, "Camera")
        self.tabs.addTab(self.pixhawk_view, "Pixhawk")
        self.tabs.addTab(self.gnss_view, "GNSS")
        self.tabs.addTab(log_page, "Log")
        self.tabs.addTab(self.ftp_view, "FTP")
        self.tabs.addTab(self.measurement_view, "Measurement")
        self.tabs.addTab(self.performance_view, "Performance")
        layout.addWidget(self.tabs)

    @staticmethod
    def _diagnostic_text_view() -> QtWidgets.QPlainTextEdit:
        view = QtWidgets.QPlainTextEdit()
        view.setReadOnly(True)
        view.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        return view

    def select_tab_for_context(self, context_id: str) -> None:
        tab_indexes = {
            "diagnostics.camera": 1,
            "diagnostics.pixhawk": 2,
            "diagnostics.gnss": 3,
            "diagnostics.logs": 4,
            "diagnostics.ftp": 5,
            "diagnostics.measurement": 6,
            "diagnostics.performance": 7,
        }
        self.tabs.setCurrentIndex(tab_indexes.get(context_id, 0))

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        refreshers = (
            self._refresh_system,
            self._refresh_camera,
            self._refresh_pixhawk,
            self._refresh_gnss,
            self._refresh_log,
            self._refresh_ftp,
            self._refresh_measurement,
            self._refresh_performance,
        )
        index = self.tabs.currentIndex()
        if 0 <= index < len(refreshers):
            refreshers[index](snapshot)

    def _refresh_system(self, snapshot: dict[str, RuntimeStatus]) -> None:
        rows = tuple(
            (
                status.key,
                status.state,
                status.severity.value,
                status.source,
                "--" if status.age_s is None else f"{status.age_s:.2f}s",
                status.detail,
            )
            for status in snapshot.values()
        )
        _sync_table_rows(self.system_model, rows)

    def _refresh_log(self, _snapshot: dict[str, RuntimeStatus]) -> None:
        backend = getattr(self.backend, "diagnostic_text", None)
        if callable(backend):
            self._set_log_text(str(backend()))

    def _refresh_ftp(self, _snapshot: dict[str, RuntimeStatus]) -> None:
        backend = getattr(self.backend, "ftp_diagnostic_text", None)
        if callable(backend):
            self._set_diagnostic_text(self.ftp_view, str(backend()), move_to_end=True)
        else:
            self._set_diagnostic_text(self.ftp_view, "FTP diagnostic log is unavailable.")

    def _follow_latest_log(self) -> None:
        scrollbar = self.log_view.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
        self._log_pending_count = 0
        self.log_new_label.clear()
        self.log_new_label.hide()
        self.log_follow_button.hide()

    def _set_log_text(self, text: str) -> None:
        if text == self.log_view.toPlainText():
            return
        scrollbar = self.log_view.verticalScrollBar()
        was_at_bottom = scrollbar.value() >= scrollbar.maximum() - 4
        old_line_count = self.log_view.blockCount()
        old_scroll_value = scrollbar.value()
        self.log_view.setPlainText(text)
        if was_at_bottom:
            scrollbar.setValue(scrollbar.maximum())
            self._log_pending_count = 0
            self.log_new_label.clear()
            self.log_new_label.hide()
            self.log_follow_button.hide()
        else:
            scrollbar.setValue(min(old_scroll_value, scrollbar.maximum()))
            added_lines = max(1, self.log_view.blockCount() - old_line_count)
            self._log_pending_count += added_lines
            self.log_new_label.setText(f"New logs: {self._log_pending_count}")
            self.log_new_label.show()
            self.log_follow_button.show()

    def _refresh_camera(self, snapshot: dict[str, RuntimeStatus]) -> None:
        evidence = _evidence(self.backend)
        config = evidence.detection_config()
        detection_lines = [
            "Detection configuration (AppSettings / existing controls):",
            f"weights={config.get('weights_path', '--')}",
            f"device={config.get('device', '--')} input={config.get('input_size_px', '--')} px "
            f"conf/IoU={config.get('confidence', '--')}/{config.get('iou', '--')}",
            f"gates={', '.join(str(item) for item in config.get('gate_enabled', ()) or ()) or 'OFF'}",
        ]
        target_lines = ["Target projection (bbox fields are image pixels u/v):"]
        for side in ("left", "right"):
            target = evidence.detection(side)
            target_lines.append(
                f"{side}: {target.get('runtime_state', 'NO TARGET')} class="
                f"{_display_class(target)} "
                f"conf={_display_number(target.get('confidence'), digits=3)} "
                f"age={_display_number(target.get('age_s'), digits=2, suffix='s')}"
            )
        text = self._status_text(
            snapshot,
            ("camera.left", "camera.right", "detection", "synchronization"),
        )
        self._set_diagnostic_text(
            self.camera_view,
            "\n\n".join((text, "\n".join(detection_lines + target_lines))),
        )

    def _refresh_pixhawk(self, snapshot: dict[str, RuntimeStatus]) -> None:
        evidence = _evidence(self.backend)
        attitude_lines = ["Attitude source projection (orientation only; PX4 body FRD → world NED):"]
        for side in ("left", "right"):
            data = evidence.attitude(side)
            attitude_lines.append(
                f"{side}: state={data.get('state')} rpy_deg="
                f"{_display_number(data.get('roll_deg'))}/"
                f"{_display_number(data.get('pitch_deg'))}/"
                f"{_display_number(data.get('yaw_deg'))}; "
                f"age={_display_number(data.get('age_s'), digits=2, suffix='s')}; "
                f"px4_t={_display_number(data.get('px4_timestamp_s'), digits=6)}; "
                "transport=MAVLink"
            )
        text = self._status_text(snapshot, ("pixhawk.left", "pixhawk.right"))
        self._set_diagnostic_text(self.pixhawk_view, "\n\n".join((text, "\n".join(attitude_lines))))

    def _refresh_gnss(self, snapshot: dict[str, RuntimeStatus]) -> None:
        evidence = _evidence(self.backend)
        baseline = evidence.baseline()
        operation = evidence.gnss_operation()
        base_serial = operation.get("base_serial") if isinstance(operation.get("base_serial"), dict) else {}
        survey = operation.get("survey_in") if isinstance(operation.get("survey_in"), dict) else {}
        ntrip = operation.get("ntrip") if isinstance(operation.get("ntrip"), dict) else {}
        lines = [
            # "gnss.ntrip" is the actually-registered status key (see
            # status_model.py's _refresh_ntrip) -- this used to ask for a
            # plain "gnss" key that was never registered, so the detailed
            # NTRIP status line (phase/GGA age/RTCM age/reconnects/error)
            # silently rendered as "NO DATA" here no matter what NTRIP was
            # actually doing.
            self._status_text(snapshot, ("gnss.base", "gnss.ntrip")),
            "Base operation (Base receiver / Survey-in / NTRIP / RTCM):",
            f"mode={operation.get('mode', 'UNKNOWN')} state={operation.get('state', 'UNKNOWN')} "
            f"serial={'CONNECTED' if base_serial.get('connected') else 'DISCONNECTED'} "
            f"device={base_serial.get('device') or '--'} discovery={operation.get('base_discovery_status', 'NOT RUN')}",
            f"survey active/valid={bool(survey.get('active'))}/{bool(survey.get('valid'))} "
            f"duration={survey.get('duration_s', 0)} s accuracy={_display_number(survey.get('mean_accuracy_m'), digits=4)} m",
            f"NTRIP={'CONNECTED' if ntrip.get('connected') else 'DISCONNECTED'} "
            f"caster={ntrip.get('caster_host') or '--'}:{ntrip.get('caster_port') or '--'}/"
            f"{ntrip.get('mountpoint') or '--'}; RTCM frames={operation.get('base_frames', 0)} "
            f"age={_display_number(operation.get('last_frame_age_s'), digits=2, suffix='s')}",
            "",
            "Rover receiver projection (WGS84 geodetic; not NED baseline):",
        ]
        for side in ("left", "right"):
            data = evidence.gps(side)
            lines.append(
                f"{side}: {data.get('rtk_label', data.get('state', 'NO DATA'))}; "
                f"fix={data.get('fix_type', '--')} sats={data.get('satellites', '--')} "
                f"eph/epv={_display_number(data.get('eph_m'))}/{_display_number(data.get('epv_m'))} m; "
                f"age={_display_number(data.get('age_s'), digits=2, suffix='s')}; "
                f"lat/lon={_display_number(data.get('latitude_deg'), digits=7)}/"
                f"{_display_number(data.get('longitude_deg'), digits=7)}"
            )
        lines.extend(
            [
                "",
                "Baseline source projection (NED north/east/down metres):",
                f"state={baseline.get('state', 'NO DATA')} "
                f"N/E/D={_display_number(baseline.get('north_m'))}/"
                f"{_display_number(baseline.get('east_m'))}/"
                f"{_display_number(baseline.get('down_m'))}; "
                f"length={_display_number(baseline.get('length_m'))} m",
                "P1→P2 @ P1 body FRD [F,R,D]="
                f"state={baseline.get('p1_body_frd_state', 'WAITING')} "
                f"{_display_number(baseline.get('p1_body_frd_forward_m'))}/"
                f"{_display_number(baseline.get('p1_body_frd_right_m'))}/"
                f"{_display_number(baseline.get('p1_body_frd_down_m'))} m; "
                f"attitude_age={_display_number(baseline.get('p1_body_frd_attitude_age_s'), digits=2, suffix='s')}",
            ]
        )
        self._set_diagnostic_text(self.gnss_view, "\n".join(lines))

    def _refresh_measurement(self, _snapshot: dict[str, RuntimeStatus]) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        attempt = getattr(panel, "_last_measurement_attempt", None)
        pair_delta = getattr(panel, "selected_runtime_frame_pair_delta_s", None)
        pair_basis = getattr(panel, "selected_runtime_frame_pair_basis", "") or "--"
        rtc_delta = getattr(panel, "selected_runtime_frame_pair_camera_rtc_delta_s", None)
        baseline = getattr(panel, "latest_rtk_baseline", None)
        body_frd_projection = getattr(panel, "latest_p1_body_frd_baseline", None)
        if baseline is None:
            baseline_text = "NO DATA"
        else:
            baseline_text = (
                "NED="
                f"{float(getattr(baseline, 'north_m', 0.0)):+.3f}/"
                f"{float(getattr(baseline, 'east_m', 0.0)):+.3f}/"
                f"{float(getattr(baseline, 'down_m', 0.0)):+.3f} m; "
                f"length={float(getattr(baseline, 'baseline_m', 0.0)):.3f} m; "
                f"quality={getattr(baseline, 'fix_quality', 'UNKNOWN')}"
            )
        if body_frd_projection is None:
            body_frd_text = "WAITING"
        else:
            body_frd_text = (
                f"{float(getattr(body_frd_projection, 'forward_m', 0.0)):+.3f}/"
                f"{float(getattr(body_frd_projection, 'right_m', 0.0)):+.3f}/"
                f"{float(getattr(body_frd_projection, 'down_m', 0.0)):+.3f} m"
            )
        lines = [
            f"status: {getattr(panel, 'last_distance_status', '--')}",
            f"quality: {getattr(panel, 'last_distance_quality', '--')}",
            f"sync_status: {getattr(panel, 'camera_sync_status', '--')}",
            f"sync_reason: {getattr(panel, 'camera_sync_reason', '--')}",
            f"selected_pair_delta_s: {pair_delta if pair_delta is not None else '--'}",
            f"selected_pair_basis: {pair_basis}",
            f"selected_pair_rtcp_ntp_delta_s: {rtc_delta if rtc_delta is not None else '--'}",
            f"rtk_baseline_ned_m: {baseline_text}",
            f"rtk_baseline_p1_body_frd_m: {body_frd_text}",
            f"measurement_frame: {MEASUREMENT_FRAME}",
            f"axis_contract: {MEASUREMENT_AXIS_CONTRACT}",
            f"world_frame: {WORLD_FRAME}",
            f"body_frame: {BODY_FRAME}",
            f"camera_frame: {CAMERA_FRAME}",
            f"baseline_translation_frame: {BASELINE_TRANSLATION_FRAME}",
            f"attempt: {attempt!r}",
        ]
        self._set_diagnostic_text(self.measurement_view, "\n".join(lines))

    def _refresh_performance(self, snapshot: dict[str, RuntimeStatus]) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        identity = getattr(panel, "nvidia_runtime_identity", {}) or {}
        selection = getattr(panel, "decoder_selection_status", {}) or {}
        active_decoders = getattr(panel, "active_stream_decoders", {}) or {}
        fallback_reasons = getattr(panel, "decoder_fallback_reasons", {}) or {}
        runtime_overrides = getattr(panel, "_runtime_decoder_overrides", {}) or {}
        requested = selection.get("requested") or getattr(
            getattr(panel, "settings", None), "rtsp_decoder", "auto"
        )
        lines = [
            self._status_text(snapshot, ("fps",)),
            "",
            "NVIDIA / H.264 decoder evidence:",
            f"GPU model: {identity.get('model') or '--'}",
            f"Driver: {identity.get('driver_version') or '--'}",
            f"GPU UUID: {identity.get('gpu_uuid') or '--'}",
            f"Bus: {identity.get('bus_location') or '--'}",
            f"NVDEC available: {selection.get('nvidia_available') if selection.get('nvidia_available') is not None else '--'}",
            f"Requested mode: {requested}",
            f"Selected candidate: {selection.get('effective') or '--'}",
            f"Candidates: {', '.join(str(item) for item in tuple(selection.get('candidates') or ())) or '--'}",
            f"Allowed/degraded: {selection.get('allowed')}/{bool(selection.get('degraded', False))}",
            f"Selection reason: {selection.get('reason') or '--'}",
        ]
        for side, label in (("left", "Left"), ("right", "Right")):
            presentation = decoder_presentation(
                requested_mode=requested,
                active_decoder=(active_decoders.get(side) if isinstance(active_decoders, Mapping) else None),
                runtime_override=(
                    runtime_overrides.get(side)
                    if isinstance(runtime_overrides, Mapping) and side in runtime_overrides
                    else getattr(panel, "_runtime_decoder_override", None)
                ),
                fallback_reason=(fallback_reasons.get(side) if isinstance(fallback_reasons, Mapping) else ""),
            )
            lines.append(f"{label}: {presentation.value_text} | {presentation.detail}")
        stream_stats_by_side = getattr(panel, "camera_stream_stats", {}) or {}
        if not isinstance(stream_stats_by_side, Mapping):
            stream_stats_by_side = {}
        lines.extend(
            (
                "",
                "RTP / H.264 input measurements (worker rolling rates):",
                "RTP missing = sequence gap estimated missing packets; -- = no metric yet.",
            )
        )
        for side, label in (("left", "Left"), ("right", "Right")):
            side_stats = stream_stats_by_side.get(side, {})
            side_stats = dict(side_stats) if isinstance(side_stats, Mapping) else {}
            metrics_text = format_camera_input_metrics(side, side_stats).replace("\n", " | ")
            lines.append(
                f"{label}: {metrics_text}"
            )
        lines.append("GPU utilization is diagnostic only; it is not a readiness or success gate.")
        self._set_diagnostic_text(self.performance_view, "\n".join(lines))

    def _status_text(self, snapshot: dict[str, RuntimeStatus], keys: tuple[str, ...]) -> str:
        return "\n".join(self._format_status(snapshot.get(key)) for key in keys)

    @staticmethod
    def _set_diagnostic_text(
        view: QtWidgets.QPlainTextEdit,
        text: str,
        *,
        move_to_end: bool = False,
    ) -> None:
        if text == view.toPlainText():
            return
        view.setPlainText(text)
        if move_to_end:
            view.moveCursor(QtGui.QTextCursor.End)

    @staticmethod
    def _format_status(status: RuntimeStatus | None) -> str:
        if status is None:
            return "NO DATA"
        age = "--" if status.age_s is None else f"{status.age_s:.2f}s"
        return (
            f"{status.label} | state={status.state} | severity={status.severity.value} | age={age}\n"
            f"From: {status.source} | Clock: {status.clock}\n"
            f"detail={status.detail}"
        )
