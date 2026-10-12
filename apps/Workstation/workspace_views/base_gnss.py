"""Base GNSS connection, correction sources and route evidence."""

from __future__ import annotations

import time
from typing import Callable
from PyQt5 import QtCore, QtWidgets
from Workstation.ui.components import EngineeringValue, WorkspaceHeader
from Workstation.application.status_model import RuntimeStatus
from .shared import (
    _append_table_row,
    _command_button,
    _display_number,
    _evidence,
    _live_inspection,
    _sync_table_rows,
    _table_model,
)


class BaseGnssWorkspace(QtWidgets.QWidget):
    """Current Base GNSS, Survey-in, NTRIP, and RTCM distribution state."""

    inspection_requested = QtCore.pyqtSignal(object)

    MODE_LABELS = {
        "LOCAL_SURVEY_IN": "Local Survey-in",
        "NETWORK_ASSISTED_FIXED_BASE": "NTRIP-assisted Fixed Base",
        "NETWORK_RTK_DIRECT": "Network RTK Direct",
    }

    # Ordered acquisition chain for NETWORK_ASSISTED_FIXED_BASE / NETWORK_RTK_DIRECT:
    # position in this tuple is the stage number shown to the user. States outside
    # this chain (fault/terminal/other-mode states) are labelled but not numbered.
    ACQUISITION_STAGES: tuple[tuple[str, str], ...] = (
        # DISCONNECTED/CONFIGURED describe the operation-state-machine
        # session, not the Base serial link -- the serial link is shown
        # separately in the "Base receiver" panel's Connection field, and
        # can already be CONNECTED while this still reads stage 1: Start
        # NTRIP is what actually advances the state machine past here.
        ("DISCONNECTED", "GNSS 세션 미시작 (NTRIP Start 대기)"),
        ("CONFIGURED", "운용 설정 적용됨 · NTRIP Start 대기"),
        ("NTRIP_CONNECTING", "NTRIP 접속 중"),
        ("NTRIP_ACTIVE", "NTRIP 보정 수신 중"),
        # These two stage names ("float"/"fixed") name what the app is
        # WAITING for, not the receiver's current carrier-solution state --
        # BASE_AS_ROVER_FLOAT starts unconditionally right after NTRIP_ACTIVE,
        # before the receiver has any RTK solution at all. Check the
        # "GNSS Fixed 단계 (수신기 raw)" field for the receiver's actual
        # live fix quality (NO FIX/3D FIX/RTK FLOAT/RTK FIXED).
        ("BASE_AS_ROVER_FLOAT", "수신기 RTK Fix 수렴 대기 중"),
        ("BASE_AS_ROVER_FIXED", "수신기 RTK Fixed 획득 · 좌표 수집 시작"),
        ("FIXED_COORDINATE_COLLECTING", "좌표 안정성 검증 중"),
        ("FIXED_COORDINATE_VALIDATED", "좌표 검증 완료"),
        ("BASE_RECONFIGURING", "Base를 고정 기준점으로 재설정 중"),
        ("LOCAL_RTCM_ACTIVE", "Base 활성화 완료 · 자체 RTCM 송출 중"),
    )
    _ACQUISITION_STAGE_INDEX: dict[str, int] = {
        name: index for index, (name, _label) in enumerate(ACQUISITION_STAGES, start=1)
    }
    _ACQUISITION_STAGE_LABEL: dict[str, str] = dict(ACQUISITION_STAGES)
    _ACQUISITION_STAGE_TOTAL = len(ACQUISITION_STAGES)

    OTHER_STATE_LABELS: dict[str, str] = {
        "INITIALIZING": "초기화 중",
        "BASE_SURVEYING": "자체 Survey-in 진행 중",
        "ROVERS_CONVERGING": "로버 수렴 대기 중",
        "ALIGNMENT_COLLECTING": "정렬 데이터 수집 중",
        "ALIGNMENT_VALID": "정렬 완료",
        "READY": "측정 준비 완료",
        "MEASURING": "측정 중",
        "DEGRADED": "성능 저하",
        "FAULT": "오류 발생",
        "INVALIDATED": "무효화됨",
        "STOPPED": "중지됨",
    }

    def __init__(
        self,
        backend: object,
        invoke_command: Callable[[str], bool] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self._ntrip_was_connected = False
        self._ntrip_connect_monotonic_s: float | None = None
        self._ntrip_pre_connect_fix_text = "--"
        self._ntrip_fixed_after_connect_s: float | None = None
        self._last_state_text = "--"
        self._last_base_fix: bool | None = None
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(WorkspaceHeader("Base GNSS"))

        # Fixed 3-column x 2-row action grid. Survey-in and NTRIP start/stop
        # share a cell with their counterpart because refresh() never makes
        # both visible at once (local_mode vs network_mode are exclusive),
        # so the visible grid always stays a clean, fully-packed 3x2 layout
        # regardless of which operation mode is active.
        actions = QtWidgets.QGridLayout()
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        for command_id, label, row, col in (
            ("gnss.base_discover", "Find Base", 0, 0),
            ("gnss.base_connect", "Connect", 0, 1),
            ("gnss.base_disconnect", "Disconnect", 0, 2),
            ("gnss.survey_start", "Start Survey-in", 1, 0),
            ("gnss.ntrip_start", "Start NTRIP", 1, 0),
            ("gnss.survey_stop", "Stop Survey-in", 1, 1),
            ("gnss.ntrip_stop", "Stop NTRIP", 1, 1),
            ("gnss.ntrip_forget_password", "Forget NTRIP password", 1, 2),
        ):
            button = _command_button(label, command_id, invoke_command)
            self.operation_buttons[command_id] = button
            actions.addWidget(button, row, col)
        layout.addLayout(actions)

        summary_group = QtWidgets.QGroupBox("Current state")
        summary_layout = QtWidgets.QGridLayout(summary_group)
        self.summary_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("progress", "진행 단계"),
                ("mode", "Mode"),
                ("state", "Operation (raw)"),
                ("source", "Correction source"),
                ("rtcm", "RTCM stream"),
            )
        ):
            value = EngineeringValue(label)
            self.summary_values[key] = value
            summary_layout.addWidget(value, index // 2, index % 2)
        layout.addWidget(summary_group)

        self.serial_group = QtWidgets.QGroupBox("Base receiver")
        serial_layout = QtWidgets.QGridLayout(self.serial_group)
        self.serial_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (("connection", "Connection"), ("device", "Device"), ("baud", "Baud"), ("satellites", "Satellites"))
        ):
            value = EngineeringValue(label)
            self.serial_values[key] = value
            serial_layout.addWidget(value, index // 4, index % 4)
        layout.addWidget(self.serial_group)

        self.candidate_group = QtWidgets.QGroupBox("Discovered Base receivers")
        candidate_layout = QtWidgets.QVBoxLayout(self.candidate_group)
        self.discovery_status = EngineeringValue("Discovery")
        candidate_layout.addWidget(self.discovery_status)
        self.candidate_model = _table_model(self, ("Port", "Target", "Baud", "Status", "UBX messages"))
        self.candidate_table = QtWidgets.QTableView()
        self.candidate_table.setModel(self.candidate_model)
        self.candidate_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.candidate_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.candidate_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.candidate_table.verticalHeader().setVisible(False)
        self.candidate_table.setMaximumHeight(132)
        self.candidate_table.horizontalHeader().setStretchLastSection(True)
        self.candidate_table.clicked.connect(self._inspect_candidate_row)
        self.candidate_table.selectionModel().selectionChanged.connect(self._refresh_candidate_action)
        candidate_layout.addWidget(self.candidate_table)
        candidate_actions = QtWidgets.QHBoxLayout()
        self.apply_candidate_button = QtWidgets.QPushButton("Use selected receiver")
        self.apply_candidate_button.setEnabled(False)
        self.apply_candidate_button.clicked.connect(self._apply_selected_candidate)
        candidate_actions.addWidget(self.apply_candidate_button)
        candidate_actions.addStretch(1)
        candidate_layout.addLayout(candidate_actions)
        self.candidate_feedback = QtWidgets.QLabel("")
        self.candidate_feedback.setObjectName("statusStrip")
        self.candidate_feedback.setWordWrap(True)
        candidate_layout.addWidget(self.candidate_feedback)
        layout.addWidget(self.candidate_group)
        self._candidate_signature: tuple[tuple[object, ...], ...] = ()

        self.survey_group = QtWidgets.QGroupBox("Survey-in (수신기 자체 관측치)")
        survey_layout = QtWidgets.QGridLayout(self.survey_group)
        self.survey_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("status", "Status"),
                ("duration", "Duration"),
                ("accuracy", "Mean accuracy"),
                ("observations", "Observations"),
                ("ecef", "Mean ECEF X/Y/Z"),
            )
        ):
            value = EngineeringValue(label)
            self.survey_values[key] = value
            survey_layout.addWidget(value, index // 3, index % 3)
        layout.addWidget(self.survey_group)

        self.ntrip_group = QtWidgets.QGroupBox("NTRIP")
        ntrip_layout = QtWidgets.QGridLayout(self.ntrip_group)
        self.ntrip_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("connection", "Connection"),
                ("health", "Health"),
                ("caster", "Caster"),
                ("mountpoint", "Mountpoint"),
                ("gga", "GGA policy"),
                ("gga_transmission", "GGA 전송 상태"),
                ("status", "Caster response"),
                ("transport", "Transport"),
                ("rtcm", "RTCM"),
                ("coverage", "RTCM 완전성 (관측치 포함?)"),
                ("reconnects", "Reconnects"),
                ("error", "Last error"),
            )
        ):
            value = EngineeringValue(label)
            self.ntrip_values[key] = value
            ntrip_layout.addWidget(value, index // 3, index % 3)
        layout.addWidget(self.ntrip_group)

        self.fixed_group = QtWidgets.QGroupBox("Fixed Base acquisition")
        fixed_layout = QtWidgets.QGridLayout(self.fixed_group)
        self.fixed_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("position", "WGS84 position"),
                ("height", "Height"),
                ("gnss_fix_stage", "GNSS Fixed 단계 (수신기 raw)"),
                ("fix", "Base fix (임시 · 좌표 확보 단계)"),
                ("samples", "Samples"),
                ("scatter", "Scatter H/V"),
                ("reconfigure", "Apply verification"),
                ("ntrip_delta", "NTRIP 연결 전/후"),
            )
        ):
            value = EngineeringValue(label)
            self.fixed_values[key] = value
            fixed_layout.addWidget(value, index // 3, index % 3)
        layout.addWidget(self.fixed_group)

        self.checklist_group = QtWidgets.QGroupBox("Base-assisted checklist (informational only)")
        checklist_layout = QtWidgets.QGridLayout(self.checklist_group)
        self.checklist_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("base_coordinate", "1. Base coordinate confirmed"),
                ("base_rtcm", "2. Base RTCM output"),
                ("left_fixed", "3. Left Rover RTK Fixed"),
                ("right_fixed", "4. Right Rover RTK Fixed"),
                ("stabilized", "5. L/R Fixed stabilized"),
                ("rtcm_complete", "6. RTCM 관측치(MSM) 포함"),
            )
        ):
            value = EngineeringValue(label)
            self.checklist_values[key] = value
            checklist_layout.addWidget(value, index // 2, index % 2)
        checklist_note = QtWidgets.QLabel("표시 전용 상태 · 이 항목들은 측정 시작/진행을 차단하지 않습니다.")
        checklist_note.setObjectName("statusStrip")
        checklist_note.setWordWrap(True)
        checklist_layout.addWidget(checklist_note, 3, 0, 1, 2)
        layout.addWidget(self.checklist_group)

        route_group = QtWidgets.QGroupBox("RTCM distribution")
        route_layout = QtWidgets.QVBoxLayout(route_group)
        self.route_model = _table_model(self, ("Target", "Frames", "Packets", "Bytes", "Failures", "Last delivery"))
        self.route_table = QtWidgets.QTableView()
        self.route_table.setModel(self.route_model)
        self.route_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.route_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.route_table.verticalHeader().setVisible(False)
        self.route_table.setMaximumHeight(136)
        self.route_table.horizontalHeader().setStretchLastSection(True)
        self.route_table.clicked.connect(self._inspect_route_row)
        route_layout.addWidget(self.route_table)
        layout.addWidget(route_group)

        for group_name, group_values in (
            ("summary", self.summary_values),
            ("serial", self.serial_values),
            ("survey", self.survey_values),
            ("ntrip", self.ntrip_values),
            ("fixed", self.fixed_values),
            ("checklist", self.checklist_values),
        ):
            for key, value in group_values.items():
                value.activated.connect(
                    lambda group_name=group_name, key=key: self._inspect_value(group_name, key)
                )

        layout.addStretch(1)

    @staticmethod
    def _dict(value: object) -> dict[str, object]:
        return dict(value) if isinstance(value, dict) else {}

    def refresh(self, _snapshot: dict[str, RuntimeStatus]) -> None:
        data = _evidence(self.backend).gnss_operation()
        mode = str(data.get("mode", "UNKNOWN") or "UNKNOWN")
        state = str(data.get("state", "UNKNOWN") or "UNKNOWN")
        source = self._dict(data.get("source_identity"))
        serial = self._dict(data.get("base_serial"))
        survey = self._dict(data.get("survey_in"))
        ntrip = self._dict(data.get("ntrip"))
        coordinate = self._dict(data.get("base_coordinate"))
        fixed = self._dict(data.get("fixed_base"))
        profile = self._dict(fixed.get("profile"))
        base_discovery_status = str(data.get("base_discovery_status", "NOT RUN") or "NOT RUN")
        self.discovery_status.set_value(
            base_discovery_status,
            valid=False if base_discovery_status.startswith("ERROR") else None,
        )

        candidates = data.get("base_candidates")
        candidates = tuple(row for row in candidates if isinstance(row, dict)) if isinstance(candidates, (tuple, list)) else ()
        candidate_signature = tuple(
            (
                row.get("port", ""),
                row.get("target", ""),
                row.get("baud"),
                row.get("status", ""),
                row.get("messages", ""),
                bool(row.get("ubx", False)),
            )
            for row in candidates
        )
        if candidate_signature != self._candidate_signature:
            selected_port = self._selected_candidate_port()
            self.candidate_model.removeRows(0, self.candidate_model.rowCount())
            for row in candidates:
                _append_table_row(
                    self.candidate_model,
                    (
                        row.get("port", ""),
                        row.get("target", ""),
                        row.get("baud") or "--",
                        row.get("status", ""),
                        row.get("messages", ""),
                    ),
                )
            self._candidate_signature = candidate_signature
            if selected_port:
                for row_index, row in enumerate(candidates):
                    if str(row.get("port", "")) == selected_port:
                        self.candidate_table.selectRow(row_index)
                        break

        source_text = str(source.get("source_kind", "") or "--")
        if source.get("caster_host"):
            source_text = f"{source.get('caster_host')}:{source.get('caster_port', '--')}/{source.get('mountpoint', '')}"
        elif source.get("base_device"):
            source_text = str(source.get("base_device"))
        frame_age = data.get("last_frame_age_s")
        frame_text = (
            f"{data.get('base_frames', 0)} frames · {_display_number(data.get('average_frame_rate_hz'), digits=2)} Hz"
            f" · age {_display_number(frame_age, digits=2, suffix='s')}"
        )
        stage_index = self._ACQUISITION_STAGE_INDEX.get(state)
        if stage_index is not None:
            stage_label = self._ACQUISITION_STAGE_LABEL[state]
            progress_text = f"{stage_index}/{self._ACQUISITION_STAGE_TOTAL} · {stage_label}"
            progress_valid = True if state == "LOCAL_RTCM_ACTIVE" else None
        else:
            progress_text = self.OTHER_STATE_LABELS.get(state, state)
            progress_valid = False if state in {"FAULT", "INVALIDATED"} else None
        self.summary_values["progress"].set_value(progress_text, valid=progress_valid)
        self.summary_values["mode"].set_value(self.MODE_LABELS.get(mode, mode))
        self.summary_values["state"].set_value(state, valid=False if state in {"FAULT", "INVALIDATED"} else None)
        self.summary_values["source"].set_value(source_text)
        self.summary_values["rtcm"].set_value(frame_text)

        serial_connected = bool(serial.get("connected", False))
        self.serial_values["connection"].set_value(
            "CONNECTED" if serial_connected else "DISCONNECTED",
            valid=True if serial_connected else None,
        )
        self.serial_values["device"].set_value(serial.get("device") or "--")
        self.serial_values["baud"].set_value(serial.get("baud") or "--")
        satellites = self._dict(data.get("satellites"))
        sat_used = satellites.get("used")
        sat_tracked = satellites.get("tracked")
        sat_count = satellites.get("count")
        if sat_used is not None or sat_tracked is not None:
            sat_text = f"used={_display_number(sat_used, digits=0)} / tracked={_display_number(sat_tracked, digits=0)}"
        elif sat_count is not None:
            sat_text = f"{sat_count} ({satellites.get('count_source') or '--'})"
        else:
            sat_text = "--"
        self.serial_values["satellites"].set_value(
            sat_text,
            valid=True if (sat_used or sat_count or 0) >= 12 else False if serial_connected else None,
        )

        survey_active = bool(survey.get("active", False))
        survey_valid = bool(survey.get("valid", False))
        survey_state = "VALID" if survey_valid else "ACTIVE" if survey_active else "NOT RUNNING"
        self.survey_values["status"].set_value(survey_state, valid=True if survey_valid else None)
        self.survey_values["duration"].set_value(
            f"{survey.get('duration_s', 0)} / {_display_number(survey.get('target_duration_s'), digits=0)}",
            unit="s",
        )
        self.survey_values["accuracy"].set_value(
            f"{_display_number(survey.get('mean_accuracy_m'), digits=4)} / {_display_number(survey.get('target_accuracy_m'), digits=4)}",
            unit="m",
        )
        self.survey_values["observations"].set_value(survey.get("observations") or "--")
        ecef = survey.get("mean_ecef_m")
        ecef_text = "--"
        if isinstance(ecef, (tuple, list)) and len(ecef) == 3 and all(value is not None for value in ecef):
            ecef_text = "/".join(f"{float(value):.3f}" for value in ecef)
        self.survey_values["ecef"].set_value(ecef_text, unit="m", frame="ECEF [X,Y,Z]")

        ntrip_connected = bool(ntrip.get("connected", False))
        ntrip_running = bool(ntrip.get("running", False))
        ntrip_health = str(ntrip.get("health_state", "DISCONNECTED") or "DISCONNECTED")
        self.ntrip_values["connection"].set_value(
            "CONNECTED" if ntrip_connected else "DISCONNECTED",
            valid=True if ntrip_connected else None,
        )
        self.ntrip_values["health"].set_value(
            ntrip_health,
            valid=True if ntrip_health == "RTCM_ACTIVE" else False if ntrip_health in {"ERROR", "STALE", "RTCM_STALE"} else None,
        )
        tls_suffix = " · TLS" if ntrip.get("tls") else ""
        self.ntrip_values["caster"].set_value(f"{ntrip.get('caster_host', '--')}:{ntrip.get('caster_port', '--')}{tls_suffix}")
        self.ntrip_values["mountpoint"].set_value(ntrip.get("mountpoint") or "--")
        self.ntrip_values["gga"].set_value(ntrip.get("gga_policy") or "--")
        # A VRS caster (e.g. NGII RTS2/VRS-RTCM32) withholds RTCM without
        # periodic GGA while the transport itself still reports "connected",
        # so this is the only place that failure mode becomes visible.
        last_gga_age = ntrip.get("last_gga_age_s")
        gga_skip_reason = str(ntrip.get("gga_skip_reason") or "")
        gga_skip_age = ntrip.get("gga_skip_age_s")
        gga_skip_reason_text = gga_skip_reason.removeprefix("[GNSS][NTRIP] GGA skipped: ")
        currently_blocked = bool(gga_skip_reason) and (
            last_gga_age is None or (gga_skip_age is not None and gga_skip_age <= last_gga_age)
        )
        if not ntrip.get("connected") and not ntrip.get("running"):
            gga_transmission_text, gga_transmission_valid = "--", None
        elif currently_blocked:
            gga_transmission_text = (
                f"차단 · {gga_skip_reason_text} "
                f"({_display_number(gga_skip_age, digits=1, suffix='s')} 째)"
            )
            gga_transmission_valid = False
        elif last_gga_age is not None:
            gga_transmission_text = f"전송 중 · last={_display_number(last_gga_age, digits=1, suffix='s')}"
            gga_transmission_valid = True
        else:
            gga_transmission_text, gga_transmission_valid = "전송 대기", None
        self.ntrip_values["gga_transmission"].set_value(gga_transmission_text, valid=gga_transmission_valid)
        self.ntrip_values["status"].set_value(ntrip.get("status") or "--")
        transport_age = ntrip.get("last_chunk_age_s")
        self.ntrip_values["transport"].set_value(
            f"chunks={ntrip.get('chunks_received', 0)} · bytes={ntrip.get('bytes_received', 0)} · "
            f"rate={_display_number(ntrip.get('byte_rate_bps'), digits=1)} B/s · "
            f"last={_display_number(transport_age, digits=2, suffix='s')}",
            source="NtripStreamStats",
        )
        rtcm_age = ntrip.get("rtcm_age_s")
        message_types = ntrip.get("rtcm_message_types")
        type_text = "--"
        if isinstance(message_types, dict) and message_types:
            # No truncation: a stream can look fine at a glance (fresh, high
            # frame rate) while still lacking the MSM observables an RTK
            # solution actually needs -- that gap only shows up by seeing
            # every message type present, not just the lowest-numbered ones.
            type_text = ", ".join(
                f"{key}:{value}"
                for key, value in sorted(message_types.items(), key=lambda item: int(item[0]) if str(item[0]).lstrip("-").isdigit() else 0)
            )
        self.ntrip_values["rtcm"].set_value(
            f"frames={ntrip.get('rtcm_frames', data.get('base_frames', 0))} · "
            f"age={_display_number(rtcm_age, digits=2, suffix='s')} · types={type_text}",
            source="Base RTCM parser",
        )
        # A fresh, high-rate RTCM stream can still be unusable for RTK: an
        # RTK solution needs a station-coordinate message (1005/1006) AND at
        # least one MSM observation message (1070-1129). Ephemeris/antenna
        # messages (1019/1020/1033/1042/1007/...) alone will never converge
        # -- this is the gap that silently stalls acquisition at "3D FIX".
        base_position_present = bool(ntrip.get("rtcm_base_position_present"))
        msm_present = bool(ntrip.get("rtcm_msm_present"))
        if not ntrip.get("connected") or not message_types:
            coverage_text, coverage_valid = "--", None
        elif base_position_present and msm_present:
            coverage_text, coverage_valid = "OK (station + MSM 관측치 포함)", True
        elif base_position_present and not msm_present:
            coverage_text, coverage_valid = "부족 — MSM 관측치 없음, RTK 불가", False
        elif msm_present and not base_position_present:
            coverage_text, coverage_valid = "부족 — station 좌표(1005/1006) 없음, RTK 불가", False
        else:
            coverage_text, coverage_valid = "부족 — station·MSM 모두 없음, RTK 불가", False
        self.ntrip_values["coverage"].set_value(coverage_text, valid=coverage_valid)
        self.ntrip_values["reconnects"].set_value(ntrip.get("reconnect_count", 0))
        self.ntrip_values["error"].set_value(
            ntrip.get("last_error") or "--",
            valid=False if ntrip.get("last_error") else None,
        )

        latitude = coordinate.get("latitude_deg")
        longitude = coordinate.get("longitude_deg")
        position_text = "--" if latitude is None or longitude is None else f"{float(latitude):.7f}, {float(longitude):.7f}"
        self.fixed_values["position"].set_value(position_text, frame="WGS84 geodetic latitude/longitude degrees")
        self.fixed_values["height"].set_value(
            _display_number(coordinate.get("selected_height_m")),
            unit="m",
            frame=str(coordinate.get("height_type", fixed.get("height_type", "UNKNOWN"))),
        )
        fix_stage_label = str(coordinate.get("fix_stage_label") or "--")
        self.fixed_values["gnss_fix_stage"].set_value(
            fix_stage_label,
            valid=True if fix_stage_label == "RTK FIXED" else None,
        )
        base_fix = coordinate.get("fixed")
        self.fixed_values["fix"].set_value("--" if base_fix is None else "FIXED" if base_fix else "NOT FIXED", valid=base_fix)
        self.fixed_values["samples"].set_value(fixed.get("sample_count", 0))
        # profile is only populated once the coordinate is fully validated
        # AND applied to the receiver (LOCAL_RTCM_ACTIVE); live_profile
        # carries the same shape while still collecting/validating, so use
        # it as the in-progress preview instead of leaving this blank.
        live_profile = self._dict(fixed.get("live_profile"))
        scatter_source = profile if profile else live_profile
        self.fixed_values["scatter"].set_value(
            f"{_display_number(scatter_source.get('horizontal_scatter_m'))} / {_display_number(scatter_source.get('vertical_scatter_m'))}",
            unit="m",
            source="live (collecting)" if not profile and live_profile else "",
        )
        verification = "/".join(
            "OK" if fixed.get(key) else "WAIT"
            for key in ("reconfigure_ack", "reconfigure_readback", "reconfigure_rtcm")
        )
        self.fixed_values["reconfigure"].set_value(verification)

        now_monotonic_s = time.monotonic()
        if ntrip_connected and not self._ntrip_was_connected:
            self._ntrip_connect_monotonic_s = now_monotonic_s
            self._ntrip_pre_connect_fix_text = (
                "--" if self._last_base_fix is None else "FIXED" if self._last_base_fix else "NOT FIXED"
            )
            self._ntrip_fixed_after_connect_s = None
        elif not ntrip_connected:
            self._ntrip_connect_monotonic_s = None
            self._ntrip_fixed_after_connect_s = None
        if (
            ntrip_connected
            and base_fix
            and self._ntrip_connect_monotonic_s is not None
            and self._ntrip_fixed_after_connect_s is None
        ):
            self._ntrip_fixed_after_connect_s = now_monotonic_s - self._ntrip_connect_monotonic_s
        self._ntrip_was_connected = ntrip_connected
        self._last_state_text = state
        self._last_base_fix = base_fix
        if self._ntrip_connect_monotonic_s is None:
            ntrip_delta_text = f"NTRIP 미연결 · 최근 FIX={self._ntrip_pre_connect_fix_text}"
            ntrip_delta_valid = None
        elif self._ntrip_fixed_after_connect_s is not None:
            ntrip_delta_text = (
                f"연결 전={self._ntrip_pre_connect_fix_text} → 연결 후 {self._ntrip_fixed_after_connect_s:.0f}s만에 FIXED"
            )
            ntrip_delta_valid = True
        else:
            elapsed_since_connect = now_monotonic_s - self._ntrip_connect_monotonic_s
            ntrip_delta_text = (
                f"연결 전={self._ntrip_pre_connect_fix_text} → 연결 후 {elapsed_since_connect:.0f}s 경과, FIX 대기중"
            )
            ntrip_delta_valid = None
        self.fixed_values["ntrip_delta"].set_value(ntrip_delta_text, valid=ntrip_delta_valid)

        route_rows: list[tuple[object, ...]] = []
        routes = data.get("routes")
        routes = routes if isinstance(routes, dict) else {}
        for side, label in (("left", "P1"), ("right", "P2"), ("drone", "T")):
            route = self._dict(routes.get(side))
            route_rows.append(
                (
                    label,
                    route.get("frames", 0),
                    route.get("packets", 0),
                    route.get("bytes", 0),
                    route.get("fail", 0),
                    _display_number(route.get("delivery_age_s"), digits=2, suffix="s"),
                ),
            )
        _sync_table_rows(self.route_model, tuple(route_rows))

        readiness_limits = self._dict(data.get("readiness_limits"))
        rover = self._dict(data.get("rover"))
        left_rover = self._dict(rover.get("left"))
        right_rover = self._dict(rover.get("right"))
        coordinate_confirmed = bool(profile)
        try:
            correction_age_limit = max(
                0.0,
                float(readiness_limits.get("maximum_correction_age_s", 3.0)),
            )
        except (TypeError, ValueError):
            correction_age_limit = 3.0
        try:
            base_rtcm_active = (
                int(data.get("base_frames") or 0) > 0
                and frame_age not in (None, "")
                and float(frame_age) <= correction_age_limit
            )
        except (TypeError, ValueError):
            base_rtcm_active = False
        left_fixed = left_rover.get("rtk_fixed") is True
        right_fixed = right_rover.get("rtk_fixed") is True
        try:
            fixed_duration_target = max(
                0.0,
                float(readiness_limits.get("minimum_fixed_duration_s", 30.0)),
            )
        except (TypeError, ValueError):
            fixed_duration_target = 30.0
        try:
            fixed_stabilized = (
                left_fixed
                and right_fixed
                and left_rover.get("fixed_age_s") not in (None, "")
                and right_rover.get("fixed_age_s") not in (None, "")
                and float(left_rover.get("fixed_age_s")) >= fixed_duration_target
                and float(right_rover.get("fixed_age_s")) >= fixed_duration_target
            )
        except (TypeError, ValueError):
            fixed_stabilized = False
        checklist_statuses = {
            "base_coordinate": (coordinate_confirmed, "validated Fixed Base profile"),
            "base_rtcm": (base_rtcm_active, "fresh Base RTCM frame"),
            "left_fixed": (
                None if left_rover.get("fix_type") is None else left_fixed,
                "Left GPS fix_type=6",
            ),
            "right_fixed": (
                None if right_rover.get("fix_type") is None else right_fixed,
                "Right GPS fix_type=6",
            ),
            "stabilized": (
                None
                if left_rover.get("fix_type") is None or right_rover.get("fix_type") is None
                else fixed_stabilized,
                f"both fixed for ≥{fixed_duration_target:.0f}s",
            ),
            "rtcm_complete": (
                None if not ntrip.get("connected") or not message_types else (base_position_present and msm_present),
                "station 1005/1006 + MSM 1070-1129 present",
            ),
        }
        for key, (satisfied, source_text) in checklist_statuses.items():
            self.checklist_values[key].set_value(
                "SATISFIED" if satisfied is True else "WAITING",
                source=source_text,
                valid=satisfied if satisfied is not None else None,
            )

        local_mode = mode == "LOCAL_SURVEY_IN"
        assisted_mode = mode == "NETWORK_ASSISTED_FIXED_BASE"
        network_mode = assisted_mode or mode == "NETWORK_RTK_DIRECT"
        self.serial_group.setVisible(local_mode or assisted_mode)
        self.candidate_group.setVisible(local_mode or assisted_mode)
        # Survey-in (NAV-SVIN) is a receiver-native TMODE3 feature that is
        # only ever commanded in LOCAL_SURVEY_IN mode (start_gnss_base_survey_in
        # sends CFG-TMODE3 survey-in start). NETWORK_ASSISTED_FIXED_BASE never
        # enables it -- that mode tracks convergence via its own NAV-PVT
        # sample count/scatter instead (see fixed_values["scatter"]/["samples"]
        # below) -- so showing this panel there would just display a
        # permanently-idle/frozen NAV-SVIN poll and look broken.
        self.survey_group.setVisible(local_mode)
        self.ntrip_group.setVisible(network_mode)
        self.fixed_group.setVisible(assisted_mode)
        self.checklist_group.setVisible(assisted_mode)
        for command_id in ("gnss.base_discover", "gnss.base_connect", "gnss.base_disconnect"):
            self.operation_buttons[command_id].setVisible(local_mode or assisted_mode)
        for command_id in ("gnss.survey_start", "gnss.survey_stop"):
            self.operation_buttons[command_id].setVisible(local_mode)
        for command_id in ("gnss.ntrip_start", "gnss.ntrip_stop"):
            self.operation_buttons[command_id].setVisible(network_mode)
        self.operation_buttons["gnss.base_connect"].setEnabled((local_mode or assisted_mode) and not serial_connected)
        self.operation_buttons["gnss.base_discover"].setEnabled(
            (local_mode or assisted_mode) and not bool(data.get("base_discovery_active", False))
        )
        self.operation_buttons["gnss.base_disconnect"].setEnabled((local_mode or assisted_mode) and serial_connected)
        self.operation_buttons["gnss.survey_start"].setEnabled(local_mode and serial_connected and not survey_active)
        self.operation_buttons["gnss.survey_stop"].setEnabled(local_mode and serial_connected and survey_active)
        self.operation_buttons["gnss.ntrip_start"].setEnabled(
            network_mode and not ntrip_running and (not assisted_mode or serial_connected)
        )
        self.operation_buttons["gnss.ntrip_stop"].setEnabled(network_mode and ntrip_running)
        self._refresh_candidate_action()

    def _selected_candidate_row(self) -> int:
        selected = self.candidate_table.selectionModel().selectedRows()
        return selected[0].row() if selected else -1

    def _selected_candidate_port(self) -> str:
        row = self._selected_candidate_row()
        item = self.candidate_model.item(row, 0) if row >= 0 else None
        return item.text() if item is not None else ""

    def _refresh_candidate_action(self, *_args: object) -> None:
        self.apply_candidate_button.setEnabled(self._selected_candidate_row() >= 0)

    def _apply_selected_candidate(self) -> None:
        row = self._selected_candidate_row()
        method = getattr(self.backend, "gnss_base_apply_candidate", None)
        if row < 0 or not callable(method):
            self.candidate_feedback.setText("ERROR: select a Base receiver")
            return
        try:
            applied = method(row)
        except Exception as exc:
            self.candidate_feedback.setText(f"ERROR: {exc}")
            return
        if applied is False:
            self.candidate_feedback.setText("ERROR: Base receiver was not applied")
            return
        self.candidate_feedback.setText(f"Applied Base receiver row {row + 1}")
        self.refresh({})

    def _inspect_candidate_row(self, index: QtCore.QModelIndex) -> None:
        row = index.row()
        self.inspection_requested.emit(
            _live_inspection(
                self._candidate_inspection(row),
                lambda row=row: self._candidate_inspection(row),
            )
        )

    def _candidate_inspection(self, row: int) -> dict[str, object]:
        candidates = _evidence(self.backend).gnss_operation().get("base_candidates")
        candidates = candidates if isinstance(candidates, (tuple, list)) else ()
        candidate = self._dict(candidates[row]) if 0 <= row < len(candidates) else {}
        return {
            "selection": f"Base receiver candidate · {candidate.get('port', '--')}",
            "value": (
                f"target={candidate.get('target', '--')} · baud={candidate.get('baud', '--')} · "
                f"messages={candidate.get('messages', '--')}"
            ),
            "freshness": "Latest completed Base serial discovery",
            "contract": "Serial endpoint discovery metadata; contains no coordinates or rover baseline",
            "verdict": candidate.get("status") or ("UBX DETECTED" if candidate.get("ubx") else "UNVERIFIED"),
            "source": "GnssBaseSerialDetectWorker",
        }

    def _inspect_value(self, group_name: str, key: str) -> None:
        self.inspection_requested.emit(
            _live_inspection(
                self._value_inspection(group_name, key),
                lambda group_name=group_name, key=key: self._value_inspection(group_name, key),
            )
        )

    def _value_inspection(self, group_name: str, key: str) -> dict[str, object]:
        data = _evidence(self.backend).gnss_operation()
        widget_groups = {
            "summary": self.summary_values,
            "serial": self.serial_values,
            "survey": self.survey_values,
            "ntrip": self.ntrip_values,
            "fixed": self.fixed_values,
            "checklist": self.checklist_values,
        }
        widget = widget_groups[group_name][key]
        displayed = f"{widget.value.text()} {widget.unit.text()}".strip()
        freshness = "current operation configuration"
        contract = "Base GNSS operation state; no rover baseline or Pixhawk attitude"
        verdict = widget.meta.text() or "Informational"
        source = "GnssOperationManager"
        if group_name == "survey":
            survey = self._dict(data.get("survey_in"))
            freshness = f"duration={survey.get('duration_s', 0)}s · observations={survey.get('observations', '--')}"
            contract = (
                "UBX NAV-SVIN mean ECEF [X,Y,Z] metres"
                if key == "ecef"
                else "Survey-in duration seconds and mean accuracy metres"
            )
            verdict = "VALID" if survey.get("valid") else "ACTIVE" if survey.get("active") else "NOT RUNNING"
            source = "UBX NAV-SVIN"
        elif group_name == "ntrip":
            ntrip = self._dict(data.get("ntrip"))
            freshness = str(ntrip.get("status", "No NTRIP status") or "No NTRIP status")
            contract = "NTRIP connection metadata and RTCM transport; no coordinate-frame conversion"
            verdict = "CONNECTED" if ntrip.get("connected") else "DISCONNECTED"
            source = "NTRIP worker + GnssOperationConfig"
        elif group_name == "fixed":
            coordinate = self._dict(data.get("base_coordinate"))
            freshness = f"age={_display_number(coordinate.get('age_s'), digits=2, suffix='s')}"
            contract = "WGS84 latitude/longitude degrees; height metres with explicit height type"
            verdict = "FIXED" if coordinate.get("fixed") else "NOT FIXED" if coordinate.get("fixed") is not None else "NO DATA"
            source = "UBX NAV-PVT / validated Fixed Base profile"
        elif group_name == "checklist":
            freshness = "current GNSS operation evidence"
            contract = "Informational checklist only; it does not block measurement"
            verdict = "SATISFIED" if widget.value.text() == "SATISFIED" else "WAITING"
            source = "Base-assisted GNSS checklist projection"
        elif group_name == "serial":
            serial = self._dict(data.get("base_serial"))
            verdict = "CONNECTED" if serial.get("connected") else "DISCONNECTED"
            source = "Base serial worker"
        elif key == "rtcm":
            freshness = f"last frame age={_display_number(data.get('last_frame_age_s'), digits=2, suffix='s')}"
            contract = "RTCM frame/byte counters are transport evidence, not position coordinates"
            verdict = "FRESH" if data.get("last_frame_age_s") is not None else "NO FRAME"
            source = "RTCM parser counters"
        elif key == "state":
            verdict = str(data.get("invalidation_reason") or data.get("state", "UNKNOWN"))
        return {
            "selection": f"{group_name.title()} · {widget.label.text()}",
            "value": displayed,
            "freshness": freshness,
            "contract": contract,
            "verdict": verdict,
            "source": source,
        }

    def _inspect_route_row(self, index: QtCore.QModelIndex) -> None:
        route_targets = (("left", "P1"), ("right", "P2"), ("drone", "T"))
        if not 0 <= index.row() < len(route_targets):
            return
        side, label = route_targets[index.row()]
        self.inspection_requested.emit(
            _live_inspection(
                self._route_inspection(side, label),
                lambda side=side, label=label: self._route_inspection(side, label),
            )
        )

    def _route_inspection(self, side: str, label: str | None = None) -> dict[str, object]:
        target_label = label or {"left": "P1", "right": "P2", "drone": "T"}.get(side, side)
        data = _evidence(self.backend).gnss_operation()
        routes = data.get("routes") if isinstance(data.get("routes"), dict) else {}
        route = self._dict(routes.get(side))
        failures = int(route.get("fail") or 0)
        return {
            "selection": f"RTCM route · {target_label}",
            "value": (
                f"frames={route.get('frames', 0)} · packets={route.get('packets', 0)} · "
                f"bytes={route.get('bytes', 0)} · failures={failures}"
            ),
            "freshness": f"last delivery age={_display_number(route.get('delivery_age_s'), digits=2, suffix='s')}",
            "contract": "RTCM base stream → MAVLink GPS_RTCM_DATA; transport counters only",
            "verdict": "DELIVERY FAILURE" if failures else "DELIVERING" if route.get("packets") else "NO DELIVERY",
            "source": "RTCM injection route counters",
        }
