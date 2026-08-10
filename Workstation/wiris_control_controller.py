"""Workstation WIRIS command, time, and optical-zoom controller."""

from __future__ import annotations

import math
import time
from collections.abc import Sequence
from datetime import datetime, timezone

from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.wiris_runtime_workers import (
    WirisClockCommandWorker,
    WirisCommandWorker,
    WirisZoomWorker,
)
from Workstation.workstation_core.pixhawk_attitude import (
    discover_mavlink_messages,
    mavlink_graphable_fields_for_message,
)
from Workstation.workstation_core.wiris_zoom import (
    DEFAULT_ZOOM_IN_TEMPLATE,
    DEFAULT_ZOOM_OUT_TEMPLATE,
    DEFAULT_ZOOM_SET_TEMPLATE,
    DEFAULT_ZOOM_STOP_TEMPLATE,
    LINE_TERMINATOR,
    NUL_TERMINATOR,
)

WIRIS_COMMAND_CATALOG = (
    ("Activation", "Check SDK activation", "IACT", LINE_TERMINATOR, "SDK 활성화 상태를 TRUE/FALSE로 읽습니다."),
    ("Activation", "Activate SDK", "ACTV <ACTIVATION-CODE>", LINE_TERMINATOR, "활성화 코드로 SDK를 활성화합니다. 전송 전에 실제 코드로 바꾸세요."),
    ("Activation", "Get Ethernet Mode", "GETH", LINE_TERMINATOR, "Ethernet/RTSP 모드 상태를 TRUE/FALSE로 읽습니다."),
    ("Activation", "Set Ethernet Mode ON", "SETH TRUE", LINE_TERMINATOR, "Ethernet/RTSP 모드를 켜서 영상 송출을 시작합니다."),
    ("Activation", "Set Ethernet Mode OFF", "SETH FALSE", LINE_TERMINATOR, "Ethernet/RTSP 모드를 끕니다."),
    ("Activation", "Set SDK Port", "SSDK <port>", LINE_TERMINATOR, "SDK 제어 TCP 포트를 변경합니다."),
    ("Basic", "Check connection", "HIWS", LINE_TERMINATOR, "연결 확인 명령입니다. 정상 응답은 OK입니다."),
    ("Basic", "Set delimiter NULL", "SDLM NULL", LINE_TERMINATOR, "명령 구분자를 NULL로 설정합니다."),
    ("Basic", "Set delimiter LINE", "SDLM LINE", LINE_TERMINATOR, "명령 구분자를 줄바꿈 문자로 설정합니다."),
    ("Basic", "Get delimiter", "GDLM", LINE_TERMINATOR, "현재 명령 구분자 설정을 읽습니다."),
    ("Basic", "Get serial number", "GSRN", LINE_TERMINATOR, "카메라 시리얼 번호를 읽습니다."),
    ("Basic", "Get article number", "GATN", LINE_TERMINATOR, "카메라 article 번호를 읽습니다."),
    ("Basic", "Get firmware version", "GFWV", LINE_TERMINATOR, "카메라 펌웨어 버전을 읽습니다."),
    ("Basic", "Get CPU temperature", "GTCU", LINE_TERMINATOR, "CPU 온도를 읽습니다."),
    ("Basic", "Get internal core temperature", "GTIC", LINE_TERMINATOR, "카메라 내부 core 온도를 읽습니다."),
    ("Basic", "Get inside temperature", "GTIN", LINE_TERMINATOR, "카메라 내부 온도를 읽습니다."),
    ("Basic", "Get fan power", "GFPW", LINE_TERMINATOR, "팬 출력 상태를 읽습니다."),
    ("Date/Time", "Get date/time", "GDTI", LINE_TERMINATOR, "현재 카메라 날짜/시간을 읽습니다."),
    (
        "Date/Time",
        "Set Unix time",
        "STUS <unix_seconds> <microseconds>",
        LINE_TERMINATOR,
        "카메라 시간을 Unix epoch 초+마이크로초로 설정합니다. 재부팅 후 유지되지 않을 수 있습니다.",
    ),
    (
        "Date/Time",
        "Set date/time text",
        "SDTI yyyy/MM/dd-hh:mm:ss",
        LINE_TERMINATOR,
        "카메라 시간을 날짜 문자열로 설정합니다. 재부팅 후 유지되지 않을 수 있습니다.",
    ),
    ("Menu", "Move up", "MOVE UP", LINE_TERMINATOR, "카메라 메뉴에서 위로 이동합니다."),
    ("Menu", "Move down", "MOVE DOWN", LINE_TERMINATOR, "카메라 메뉴에서 아래로 이동합니다."),
    ("Menu", "Move OK", "MOVE OK", LINE_TERMINATOR, "카메라 메뉴에서 현재 항목을 확인합니다."),
    ("Menu", "Move cancel", "MOVE CANCEL", LINE_TERMINATOR, "카메라 메뉴에서 취소하거나 뒤로 이동합니다."),
    ("GPS", "Get GPS coordinates", "GGPS", LINE_TERMINATOR, "사용 가능할 때 카메라 GPS 좌표를 읽습니다."),
    ("Appearance", "Set main camera thermal", "SMCA THERMO", LINE_TERMINATOR, "열화상 카메라를 메인 카메라로 설정합니다."),
    ("Appearance", "Set main camera visible", "SMCA VISIBLE", LINE_TERMINATOR, "가시광 카메라를 메인 카메라로 설정합니다."),
    ("Appearance", "Get main camera", "GMCA", LINE_TERMINATOR, "현재 메인 카메라 설정을 읽습니다."),
    ("Appearance", "Set GPS info position", "SGPI POSITION", LINE_TERMINATOR, "화면 GPS 정보 표시를 위치 정보로 설정합니다."),
    ("Appearance", "Set GPS info altitude/speed", "SGPI ALTITUDE_SPEED", LINE_TERMINATOR, "화면 GPS 정보 표시를 고도/속도 정보로 설정합니다."),
    ("Appearance", "Get GPS info mode", "GGPI", LINE_TERMINATOR, "현재 GPS 정보 표시 모드를 읽습니다."),
    ("Zoom", "Zoom in", "SZIN", LINE_TERMINATOR, "선택한 카메라를 줌 인합니다."),
    ("Zoom", "Zoom out", "SZOT", LINE_TERMINATOR, "선택한 카메라를 줌 아웃합니다."),
    ("Zoom", "Stop focus/zoom action", "SGFA", LINE_TERMINATOR, "지원되는 경우 진행 중인 포커스/줌 동작을 중지합니다."),
    ("Zoom", "Set simultaneous zoom ON", "SZSM TRUE", LINE_TERMINATOR, "열화상/가시광 카메라를 동시에 줌하도록 설정합니다."),
    ("Zoom", "Set simultaneous zoom OFF", "SZSM FALSE", LINE_TERMINATOR, "열화상/가시광 동시 줌을 끕니다."),
    ("Zoom", "Get simultaneous zoom", "GZSM", LINE_TERMINATOR, "동시 줌 설정 상태를 읽습니다."),
    ("Zoom", "Set thermal zoom", "SZTN <index>", LINE_TERMINATOR, "열화상 줌 값을 설정합니다. <index>를 지원 값으로 바꾸세요."),
    ("Zoom", "Get thermal zoom", "GZTV", LINE_TERMINATOR, "현재 열화상 줌 값을 읽습니다."),
    ("Zoom", "Get thermal zoom list", "GZTL", LINE_TERMINATOR, "지원되는 열화상 줌 값 목록을 읽습니다."),
    ("Zoom", "Set visible zoom", "SZVN <index>", LINE_TERMINATOR, "가시광 줌 값을 설정합니다. <index>를 지원 값으로 바꾸세요."),
    ("Zoom", "Get visible zoom", "GZVV", LINE_TERMINATOR, "현재 가시광 줌 값을 읽습니다."),
    ("Capture/Record", "Recording start", "RCRS", LINE_TERMINATOR, "카메라 자체 녹화를 시작합니다."),
    ("Capture/Record", "Recording finish", "RCRF", LINE_TERMINATOR, "카메라 자체 녹화를 종료합니다."),
    ("Capture/Record", "Is recording", "IRCR", LINE_TERMINATOR, "카메라 녹화 상태를 읽습니다."),
    ("Capture/Record", "Set periodic capture", "SPIC <period>", LINE_TERMINATOR, "주기 캡처 값을 설정합니다. <period>를 SDK 지원 값으로 바꾸세요."),
    ("Capture/Record", "Get periodic capture", "GPIC", LINE_TERMINATOR, "현재 주기 캡처 설정을 읽습니다."),
    ("Stream", "Get thermal resolution", "GTRE", LINE_TERMINATOR, "열화상 스트림 해상도를 읽습니다."),
    ("Stream", "Set encoder bitrate", "SEBR VISIBLE <bitrate>", LINE_TERMINATOR, "가시광 스트림/녹화 인코더 bitrate를 설정합니다."),
    ("Stream", "Get encoder bitrate", "GEBR VISIBLE", LINE_TERMINATOR, "가시광 인코더 bitrate 설정을 읽습니다."),
    ("Stream", "Set GOP size", "SEGS VISIBLE <gop>", LINE_TERMINATOR, "가시광 인코더 GOP 크기를 설정합니다."),
    ("Stream", "Get GOP size", "GEGS VISIBLE", LINE_TERMINATOR, "가시광 인코더 GOP 크기를 읽습니다."),
    ("Stream", "Set IDR interval", "SEIR VISIBLE <idr>", LINE_TERMINATOR, "가시광 인코더 IDR 간격을 설정합니다."),
    ("Stream", "Get IDR interval", "GEIR VISIBLE", LINE_TERMINATOR, "가시광 인코더 IDR 간격을 읽습니다."),
    ("Stream", "Set quantization quality", "SEQZ VISIBLE <qp>", LINE_TERMINATOR, "가시광 인코더 QP 값을 설정합니다. 범위는 0-51입니다."),
    ("Stream", "Get quantization quality", "GEQZ VISIBLE", LINE_TERMINATOR, "가시광 인코더 QP 설정을 읽습니다."),
    ("Trigger", "Set trigger correction", "STRG CORRECTION", LINE_TERMINATOR, "외부 trigger 입력을 보정 동작으로 설정합니다."),
    ("Trigger", "Set trigger capture", "STRG CAPTURE", LINE_TERMINATOR, "외부 trigger 입력을 이미지 캡처 동작으로 설정합니다."),
    ("Trigger", "Set trigger record", "STRG RECORD", LINE_TERMINATOR, "외부 trigger 입력을 녹화 제어 동작으로 설정합니다."),
    ("Trigger", "Disable trigger action", "STRG NA", LINE_TERMINATOR, "외부 trigger 동작을 사용하지 않도록 설정합니다."),
    ("Trigger", "Get trigger action", "GTRG", LINE_TERMINATOR, "현재 외부 trigger 동작 설정을 읽습니다."),
    ("System", "Set IP address", "STIP <IPv4>", LINE_TERMINATOR, "카메라 IP 주소를 변경합니다."),
    ("System", "Set subnet mask", "STMS <mask>", LINE_TERMINATOR, "카메라 서브넷 마스크를 변경합니다."),
    ("System", "Get subnet mask", "GTMS", LINE_TERMINATOR, "현재 서브넷 마스크를 읽습니다."),
    ("System", "Set gateway", "STGW <gateway>", LINE_TERMINATOR, "카메라 gateway 주소를 변경합니다."),
    ("System", "Get gateway", "GTGW", LINE_TERMINATOR, "현재 gateway 주소를 읽습니다."),
    ("System", "Manual gateway ON", "SMGW TRUE", LINE_TERMINATOR, "수동 gateway 설정을 사용합니다."),
    ("System", "Manual gateway OFF", "SMGW FALSE", LINE_TERMINATOR, "수동 gateway 설정을 끕니다."),
    ("System", "Get manual gateway", "GMGW", LINE_TERMINATOR, "수동 gateway 사용 여부를 읽습니다."),
    ("System", "Reboot", "REBT", LINE_TERMINATOR, "카메라를 재부팅합니다."),
    ("Custom", "Custom SDK command", "", LINE_TERMINATOR, "SDK 명령을 직접 입력합니다."),
)

WIRIS_DANGEROUS_COMMAND_PREFIXES = (
    "ACTV",
    "SSDK",
    "SETH FALSE",
    "STIP",
    "STMS",
    "STGW",
    "SMGW",
    "STUS",
    "SDTI",
    "REBT",
    "RCRS",
    "RCRF",
)

WIRIS_CONTROL_METHODS = [
    "zoom_command_text",
    "selected_zoom_target",
    "zoom_hosts",
    "_control_port_spin_for_camera",
    "_control_port_for_camera",
    "_set_control_port_for_camera",
    "send_zoom",
    "_apply_wiris_zoom_result",
    "_wiris_zoom_failed",
    "_clear_wiris_zoom_worker",
    "workstation_wiris_command_catalog",
    "workstation_send_wiris_command",
    "workstation_send_zoom",
    "_apply_selected_wiris_command",
    "_append_wiris_command_log",
    "_wiris_command_needs_confirmation",
    "_stus_command_from_unix_time",
    "set_selected_wiris_time_from_pc",
    "_apply_wiris_time_sync_result",
    "_wiris_time_sync_failed",
    "_clear_wiris_time_sync_worker",
    "send_selected_wiris_command",
    "_start_wiris_command_worker",
    "_apply_wiris_command_result",
    "_wiris_command_failed",
    "_clear_wiris_command_worker"
]


class WirisControlController:
    def __init__(self, panel):
        self.panel = panel

    def zoom_command_text(self, action: str) -> str:
        if action == "in":
            return DEFAULT_ZOOM_IN_TEMPLATE
        if action == "out":
            return DEFAULT_ZOOM_OUT_TEMPLATE
        if action == "stop":
            return DEFAULT_ZOOM_STOP_TEMPLATE
        if action == "set":
            return DEFAULT_ZOOM_SET_TEMPLATE
        raise ValueError(f"Unknown zoom action: {action}")

    def selected_zoom_target(self) -> str:
        return str(self.panel.zoom_target_combo.currentData() or "both")

    def zoom_hosts(self, target: str) -> list[tuple[str, str]]:
        if target == "left":
            return [("left", self.panel.left_zoom_host_edit.text())]
        if target == "right":
            return [("right", self.panel.right_zoom_host_edit.text())]
        if target == "both":
            return [("left", self.panel.left_zoom_host_edit.text()), ("right", self.panel.right_zoom_host_edit.text())]
        raise ValueError(f"Unknown zoom target: {target}")

    def _control_port_spin_for_camera(self, camera: str) -> QtWidgets.QSpinBox:
        if camera == "left":
            return self.panel.left_control_port_spin
        if camera == "right":
            return self.panel.right_control_port_spin
        raise ValueError(f"Unknown camera: {camera}")

    def _control_port_for_camera(self, camera: str) -> int:
        return int(self.panel._control_port_spin_for_camera(camera).value())

    def _set_control_port_for_camera(self, camera: str, port: int) -> None:
        if 1 <= int(port) <= 65535:
            self.panel._control_port_spin_for_camera(camera).setValue(int(port))

    def send_zoom(self, target: str, action: str) -> None:
        if self.panel.wiris_zoom_thread is not None:
            self.panel._log("[카메라] 줌 명령이 이미 진행 중입니다.")
            return
        command_text = self.panel.zoom_command_text(action)
        level = self.panel.zoom_level_spin.value() if action == "set" else None
        targets = [
            (label, host, self.panel._control_port_for_camera(label))
            for label, host in self.panel.zoom_hosts(target)
        ]
        self.panel.zoom_status_label.setText("Zoom command: sending...")
        self.panel.wiris_zoom_thread = QtCore.QThread(self.panel)
        self.panel.wiris_zoom_worker = WirisZoomWorker(
            targets,
            template=command_text,
            level=level,
            timeout_s=self.panel.zoom_timeout_spin.value(),
        )
        self.panel.wiris_zoom_worker.moveToThread(self.panel.wiris_zoom_thread)
        self.panel.wiris_zoom_thread.started.connect(self.panel.wiris_zoom_worker.run)
        self.panel.wiris_zoom_worker.completed.connect(
            lambda statuses, target=target, action=action: self.panel._apply_wiris_zoom_result(
                target, action, statuses
            )
        )
        self.panel.wiris_zoom_worker.failed.connect(self.panel._wiris_zoom_failed)
        self.panel.wiris_zoom_worker.finished.connect(self.panel.wiris_zoom_thread.quit)
        self.panel.wiris_zoom_worker.finished.connect(self.panel.wiris_zoom_worker.deleteLater)
        self.panel.wiris_zoom_thread.finished.connect(self.panel.wiris_zoom_thread.deleteLater)
        self.panel.wiris_zoom_thread.finished.connect(self.panel._clear_wiris_zoom_worker)
        self.panel.wiris_zoom_thread.start()

    def _apply_wiris_zoom_result(self, target: str, action: str, result: object) -> None:
        statuses: list[str] = []
        for item in list(result or []):
            if not isinstance(item, dict):
                continue
            label = str(item.get("label") or "")
            if item.get("state") == "sent":
                response = str(item.get("response") or "")
                suffix = f" 응답={response}" if response else " 전송됨"
                statuses.append(f"{self.panel._camera_name_ko(label)}: {item.get('command', '')}{suffix}")
            else:
                statuses.append(f"{self.panel._camera_name_ko(label)}: 실패: {item.get('detail', 'unknown')}")
        status_text = " | ".join(statuses)
        self.panel.zoom_status_label.setText(status_text)
        self.panel._log(f"[카메라] 줌 명령 target={target}, action={action}: {status_text}")

    def _wiris_zoom_failed(self, error: str) -> None:
        self.panel.zoom_status_label.setText(f"Zoom command: failed ({error})")
        self.panel._log(f"[카메라] 줌 명령 실패: {error}")

    def _clear_wiris_zoom_worker(self) -> None:
        self.panel.wiris_zoom_thread = None
        self.panel.wiris_zoom_worker = None

    def workstation_wiris_command_catalog(self) -> tuple[dict[str, str], ...]:
        return tuple(
            {
                "category": str(category),
                "label": str(label),
                "command": str(command),
                "terminator": str(terminator),
                "description": str(description),
            }
            for category, label, command, terminator, description in WIRIS_COMMAND_CATALOG
        )

    def workstation_send_wiris_command(self, target: str, command: str, terminator: str) -> None:
        target = str(target or "both")
        if target not in {"both", "left", "right"}:
            raise ValueError(f"Unknown WIRIS target: {target}")
        command = str(command or "").strip()
        if not command:
            raise ValueError("WIRIS SDK command is empty")
        terminator = str(terminator or LINE_TERMINATOR)
        if terminator not in {LINE_TERMINATOR, NUL_TERMINATOR}:
            raise ValueError("Unknown WIRIS command terminator")
        self.panel._set_combo_data(self.panel.wiris_command_target_combo, target)
        self.panel.wiris_command_edit.setText(command)
        self.panel._set_combo_data(self.panel.wiris_command_terminator_combo, terminator)
        self.panel.send_selected_wiris_command()

    def workstation_send_zoom(self, target: str, action: str, level: int | None = None) -> None:
        target = str(target or "both")
        if target not in {"both", "left", "right"}:
            raise ValueError(f"Unknown zoom target: {target}")
        action = str(action or "")
        if action not in {"in", "out", "stop", "set"}:
            raise ValueError(f"Unknown zoom action: {action}")
        if action == "set":
            if level is None:
                raise ValueError("zoom level is required for set")
            self.panel.zoom_level_spin.setValue(int(level))
        self.panel.send_zoom(target, action)

    def _apply_selected_wiris_command(self, *_args) -> None:
        if not hasattr(self.panel, "wiris_command_combo"):
            return
        data = self.panel.wiris_command_combo.currentData()
        if not data:
            return
        command, terminator, description = data
        self.panel.wiris_command_edit.setText(str(command))
        self.panel._set_combo_data(self.panel.wiris_command_terminator_combo, str(terminator))
        self.panel.wiris_command_status_label.setText(str(description))
        self.panel.wiris_command_status_label.setToolTip(str(description))
        self.panel.wiris_command_combo.setToolTip(str(description))
        self.panel.wiris_command_edit.setToolTip(str(description))

    def _append_wiris_command_log(self, text: str) -> None:
        self.panel.wiris_command_log.appendPlainText(text)
        self.panel.wiris_command_log.moveCursor(QtGui.QTextCursor.End)

    def _wiris_command_needs_confirmation(self, command_text: str) -> bool:
        normalized = " ".join(command_text.upper().split())
        return any(normalized.startswith(prefix) for prefix in WIRIS_DANGEROUS_COMMAND_PREFIXES)

    def _stus_command_from_unix_time(self, unix_time_s: float) -> str:
        seconds = int(unix_time_s)
        microseconds = int(round((float(unix_time_s) - seconds) * 1_000_000))
        if microseconds >= 1_000_000:
            seconds += 1
            microseconds -= 1_000_000
        return f"STUS {seconds} {microseconds}"

    def set_selected_wiris_time_from_pc(self, target: str | None = None) -> bool | None:
        if self.panel._session_recording_active():
            status_text = "세션 기록 중에는 WIRIS 시간을 변경할 수 없습니다."
            self.panel.wiris_command_status_label.setText(status_text)
            if hasattr(self.panel, "clock_sync_status_label"):
                self.panel.clock_sync_status_label.setText(status_text)
            self.panel._log(f"[시계] {status_text}")
            return False
        if target is None:
            if hasattr(self.panel, "clock_wiris_time_target_combo"):
                target = str(self.panel.clock_wiris_time_target_combo.currentData() or "both")
            else:
                target = str(self.panel.wiris_command_target_combo.currentData() or "both")
        target = str(target or "both")
        if self.panel.wiris_time_sync_thread is not None:
            self.panel._log("[카메라] WIRIS RTC PC UTC 설정이 이미 진행 중입니다.")
            return
        target_infos = [
            (label, host, self.panel._control_port_for_camera(label), self.panel.zoom_timeout_spin.value())
            for label, host in self.panel.zoom_hosts(target)
        ]
        if not target_infos:
            status_text = "시간을 설정할 WIRIS 대상이 없습니다."
            self.panel.wiris_command_status_label.setText(status_text)
            if hasattr(self.panel, "clock_sync_status_label"):
                self.panel.clock_sync_status_label.setText(status_text)
            return
        self.panel.wiris_set_pc_time_button.setEnabled(False)
        self.panel.wiris_command_status_label.setText("PC UTC 설정: sending...")
        if hasattr(self.panel, "clock_sync_status_label"):
            self.panel.clock_sync_status_label.setText("PC UTC 설정: sending...")
        self.panel.wiris_time_sync_thread = QtCore.QThread(self.panel)
        self.panel.wiris_time_sync_worker = WirisClockCommandWorker(
            target_infos,
            command_text="STUS",
            shared_utc=True,
        )
        self.panel.wiris_time_sync_worker.moveToThread(self.panel.wiris_time_sync_thread)
        self.panel.wiris_time_sync_thread.started.connect(self.panel.wiris_time_sync_worker.run)
        self.panel.wiris_time_sync_worker.completed.connect(
            lambda result, target=target, target_infos=target_infos: self.panel._apply_wiris_time_sync_result(
                target, target_infos, result
            )
        )
        self.panel.wiris_time_sync_worker.failed.connect(self.panel._wiris_time_sync_failed)
        self.panel.wiris_time_sync_worker.finished.connect(self.panel.wiris_time_sync_thread.quit)
        self.panel.wiris_time_sync_worker.finished.connect(self.panel.wiris_time_sync_worker.deleteLater)
        self.panel.wiris_time_sync_thread.finished.connect(self.panel.wiris_time_sync_thread.deleteLater)
        self.panel.wiris_time_sync_thread.finished.connect(self.panel._clear_wiris_time_sync_worker)
        self.panel.wiris_time_sync_thread.start()
        return None

    def _apply_wiris_time_sync_result(
        self,
        target: str,
        target_infos: Sequence[tuple[str, str, int, float]],
        result: object,
    ) -> None:
        data = dict(result or {})
        results = dict(data.get("results") or {})
        requested_utc_by_camera: dict[str, float] = {}
        raw_requested_by_camera = dict(data.get("requested_utc_by_camera") or {})
        for label, *_ in target_infos:
            value = raw_requested_by_camera.get(label)
            if value is None:
                value = dict(results.get(label) or {}).get("stus_requested_utc_s")
            try:
                value = float(value)
            except (TypeError, ValueError):
                continue
            if math.isfinite(value):
                requested_utc_by_camera[label] = value

        rtt_by_camera: dict[str, float | None] = {}
        one_way_delay_by_camera: dict[str, float | None] = {}
        compensation_by_camera: dict[str, float | None] = {}
        timing_text_parts: list[str] = []
        statuses = {label: str(dict(results.get(label) or {}).get("response") or "") for label, *_ in target_infos}
        for label, _host, port, _timeout_s in target_infos:
            row = dict(results.get(label) or {})
            response = statuses.get(label, "")
            command = str(row.get("command") or "STUS")
            requested_utc_s = requested_utc_by_camera.get(label, data.get("requested_utc_s"))
            utc_text = ""
            if requested_utc_s is not None:
                utc_text = datetime.fromtimestamp(float(requested_utc_s), tz=timezone.utc).isoformat(timespec="microseconds")
            utc_suffix = f" [pc_utc={utc_text.replace('+00:00', 'Z')}]" if utc_text else ""
            self.panel._append_wiris_command_log(
                f"> {self.panel._camera_name_ko(label)} {row.get('host', '')}:{port} {command}"
                f"{utc_suffix}\n{response}\n"
            )

            try:
                rtt_median_s = float(row.get("stus_rtt_median_s"))
                if not math.isfinite(rtt_median_s) or rtt_median_s < 0.0:
                    raise ValueError
            except (TypeError, ValueError):
                rtt_median_s = None
            try:
                one_way_delay_s = float(row.get("stus_one_way_delay_s"))
                if not math.isfinite(one_way_delay_s) or one_way_delay_s < 0.0:
                    raise ValueError
            except (TypeError, ValueError):
                one_way_delay_s = None
            try:
                compensation_s = float(row.get("stus_compensation_s"))
                if not math.isfinite(compensation_s) or compensation_s < 0.0:
                    raise ValueError
            except (TypeError, ValueError):
                compensation_s = None
            rtt_by_camera[label] = rtt_median_s
            one_way_delay_by_camera[label] = one_way_delay_s
            compensation_by_camera[label] = compensation_s
            if rtt_median_s is None:
                timing_text_parts.append(f"{self.panel._camera_name_ko(label)} RTT=측정 실패(보정 0ms)")
            else:
                compensation_ms = 0.0 if compensation_s is None else compensation_s * 1000.0
                timing_text_parts.append(
                    f"{self.panel._camera_name_ko(label)} RTT={rtt_median_s * 1000.0:.2f}ms"
                    f"→STUS 보정 {compensation_ms:.2f}ms"
                )
        command_times = dict(data.get("command_times") or {})
        tcp_start_times = dict(data.get("tcp_start_times") or {})
        ordered_command_times = [
            (label, float(command_times[label]))
            for label, *_ in target_infos
            if label in command_times
        ]
        ordered_tcp_times = [
            (label, float(tcp_start_times[label]))
            for label, *_ in target_infos
            if label in tcp_start_times
        ]
        command_skew_ms = (
            (ordered_command_times[-1][1] - ordered_command_times[0][1]) * 1000.0
            if len(ordered_command_times) >= 2
            else None
        )
        tcp_start_skew_ms = (
            (ordered_tcp_times[-1][1] - ordered_tcp_times[0][1]) * 1000.0
            if len(ordered_tcp_times) >= 2
            else None
        )
        skew_text = ""
        if command_skew_ms is not None:
            skew_text = (
                f" | 명령 UTC 차이 {self.panel._camera_name_ko(ordered_command_times[0][0])}→"
                f"{self.panel._camera_name_ko(ordered_command_times[-1][0])}: {command_skew_ms:.3f} ms"
            )
        if tcp_start_skew_ms is not None:
            skew_text += f", TCP 호출 시작 차이: {tcp_start_skew_ms:.3f} ms"
        timing_text = ""
        if timing_text_parts:
            timing_text = " | STUS RTT 보정: " + ", ".join(timing_text_parts)
        status_text = " | ".join(
            f"{self.panel._camera_name_ko(label)}: {statuses.get(label, '<no response>')}"
            for label, *_ in target_infos
        ) + skew_text + timing_text
        self.panel.wiris_command_status_label.setText(status_text)
        if hasattr(self.panel, "clock_sync_status_label"):
            self.panel.clock_sync_status_label.setText(f"PC UTC 설정 전송: {status_text}")
            self.panel.clock_sync_summary_text.setPlainText(
                "STUS target = PC send UTC + camera HIWS RTT/2 + 2ms parse margin.\n"
                "Run RTCP/NTP 동기화 확인 after stream frames arrive to verify left/right RTC and PC UTC offsets.\n"
                f"{status_text}"
            )
        successful_targets = [
            label
            for label, *_ in target_infos
            if statuses.get(label, "").strip().upper() == "OK"
        ]
        requested_utc_s = data.get("requested_utc_s")
        result_state = "success" if len(successful_targets) == len(target_infos) else (
            "partial" if successful_targets else "failed"
        )
        self.panel._last_wiris_pc_utc_sync = {
            "command": "STUS",
            "target": target,
            "result": result_state,
            "requested_utc_s": requested_utc_s,
            "requested_utc_by_camera": requested_utc_by_camera,
            "successful_targets": successful_targets,
            "responses": statuses,
            "command_skew_ms": command_skew_ms,
            "tcp_start_skew_ms": tcp_start_skew_ms,
            "compensation_method": data.get("compensation_method"),
            "stus_parse_margin_s": data.get("stus_parse_margin_s"),
            "stus_rtt_by_camera": rtt_by_camera,
            "stus_one_way_delay_by_camera": one_way_delay_by_camera,
            "stus_compensation_by_camera": compensation_by_camera,
            "stus_rtt_probe_by_camera": dict(data.get("stus_rtt_by_camera") or {}),
            "recorded_at_unix_s": time.time(),
        }
        self.panel._camera_sync_requested_utc_by_camera = requested_utc_by_camera
        self.panel._begin_camera_sync_after_stus(
            target=target,
            result=result_state,
            requested_utc_s=None if requested_utc_s is None else float(requested_utc_s),
            target_infos=target_infos,
        )
        self.panel._log(f"[카메라] WIRIS RTC PC UTC 설정 target={target}: {status_text}")

    def _wiris_time_sync_failed(self, error: str) -> None:
        self.panel.wiris_command_status_label.setText(f"PC UTC 설정: failed ({error})")
        if hasattr(self.panel, "clock_sync_status_label"):
            self.panel.clock_sync_status_label.setText(f"PC UTC 설정: failed ({error})")
        self.panel._log(f"[카메라] WIRIS RTC PC UTC 설정 실패: {error}")

    def _clear_wiris_time_sync_worker(self) -> None:
        self.panel.wiris_time_sync_thread = None
        self.panel.wiris_time_sync_worker = None
        self.panel.wiris_set_pc_time_button.setEnabled(True)

    def send_selected_wiris_command(self) -> None:
        command_text = self.panel.wiris_command_edit.text().strip()
        if not command_text:
            self.panel.wiris_command_status_label.setText("전송할 명령이 비어 있습니다.")
            return
        if "<" in command_text or ">" in command_text or "{" in command_text or "}" in command_text:
            self.panel.wiris_command_status_label.setText("placeholder를 실제 값으로 바꾼 뒤 전송하세요.")
            return
        if self.panel._wiris_command_needs_confirmation(command_text):
            answer = QtWidgets.QMessageBox.warning(
                self.panel,
                "Send WIRIS SDK Command",
                (
                    f"이 명령은 카메라 설정, 네트워크, 녹화, 전원 상태를 바꿀 수 있습니다.\n\n"
                    f"{command_text}\n\n"
                    "대상 카메라가 확실할 때만 전송하세요."
                ),
                QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel,
                QtWidgets.QMessageBox.Cancel,
            )
            if answer != QtWidgets.QMessageBox.Ok:
                self.panel.wiris_command_status_label.setText("명령 전송이 취소되었습니다.")
                return

        target = str(self.panel.wiris_command_target_combo.currentData() or "both")
        terminator = str(self.panel.wiris_command_terminator_combo.currentData() or LINE_TERMINATOR)
        target_infos = [
            (label, host, self.panel._control_port_for_camera(label), self.panel.zoom_timeout_spin.value())
            for label, host in self.panel.zoom_hosts(target)
        ]
        if not target_infos:
            self.panel.wiris_command_status_label.setText("명령을 전송할 WIRIS 대상이 없습니다.")
            return

        self.panel._start_wiris_command_worker(
            target=target,
            target_infos=target_infos,
            command_text=command_text,
            terminator=terminator,
        )

    def _start_wiris_command_worker(
        self,
        *,
        target: str,
        target_infos: Sequence[tuple[str, str, int, float]],
        command_text: str,
        terminator: str,
    ) -> None:
        if self.panel.wiris_command_thread is not None:
            status_text = "WIRIS SDK command: another command is already in progress"
            self.panel._log("[카메라] WIRIS SDK 명령이 이미 진행 중입니다.")
            self._publish_wiris_command_response(
                {
                    "phase": "busy",
                    "target": target,
                    "command": command_text,
                    "status_text": status_text,
                }
            )
            return
        self.panel.wiris_command_status_label.setText("WIRIS SDK command: sending...")
        self._publish_wiris_command_response(
            {
                "phase": "sending",
                "target": target,
                "command": command_text,
                "status_text": "WIRIS SDK command: sending...",
            }
        )
        self.panel.wiris_command_send_button.setEnabled(False)
        self.panel.wiris_command_thread = QtCore.QThread(self.panel)
        self.panel.wiris_command_worker = WirisCommandWorker(
            target_infos,
            command_text=command_text,
            terminator=terminator,
        )
        self.panel.wiris_command_worker.moveToThread(self.panel.wiris_command_thread)
        self.panel.wiris_command_thread.started.connect(self.panel.wiris_command_worker.run)
        self.panel.wiris_command_worker.completed.connect(
            lambda result, target=target, command_text=command_text: self.panel._apply_wiris_command_result(
                target, command_text, result
            )
        )
        self.panel.wiris_command_worker.failed.connect(
            lambda error, target=target, command_text=command_text: self.panel._wiris_command_failed(
                error, target, command_text
            )
        )
        self.panel.wiris_command_worker.finished.connect(self.panel.wiris_command_thread.quit)
        self.panel.wiris_command_worker.finished.connect(self.panel.wiris_command_worker.deleteLater)
        self.panel.wiris_command_thread.finished.connect(self.panel.wiris_command_thread.deleteLater)
        self.panel.wiris_command_thread.finished.connect(self.panel._clear_wiris_command_worker)
        self.panel.wiris_command_thread.start()

    def _apply_wiris_command_result(self, target: str, command_text: str, result: object) -> None:
        data = dict(result or {}) if isinstance(result, dict) else {}
        statuses = {str(key): str(value) for key, value in dict(data.get("statuses") or {}).items()}
        send_times = {str(key): float(value) for key, value in dict(data.get("send_times") or {}).items()}
        target_infos = tuple(data.get("target_infos") or ())
        for log_text in dict(data.get("command_logs") or {}).values():
            self.panel._append_wiris_command_log(str(log_text))
        skew_text = ""
        ordered_times = [
            (str(label), send_times[str(label)])
            for label, _host, _port, _timeout_s in target_infos
            if str(label) in send_times
        ]
        if len(ordered_times) >= 2:
            first_label, first_time_s = ordered_times[0]
            last_label, last_time_s = ordered_times[-1]
            skew_text = (
                f" | 전송 시각 차이 {self.panel._camera_name_ko(first_label)}→"
                f"{self.panel._camera_name_ko(last_label)}: {(last_time_s - first_time_s) * 1000.0:.3f} ms"
            )
        status_text = " | ".join(
            f"{self.panel._camera_name_ko(str(label))}: {statuses.get(str(label), '<no response>')}"
            for label, _host, _port, _timeout_s in target_infos
        ) + skew_text
        self.panel.wiris_command_status_label.setText(status_text)
        self.panel._log(f"[카메라] WIRIS SDK 명령 target={target}, command={command_text}: {status_text}")
        self._publish_wiris_command_response(
            {
                **data,
                "phase": "completed",
                "target": target,
                "command": command_text,
                "status_text": status_text,
            }
        )

    def _wiris_command_failed(self, error: str, target: str = "", command_text: str = "") -> None:
        status_text = f"WIRIS SDK command: failed ({error})"
        self.panel.wiris_command_status_label.setText(status_text)
        self.panel._log(f"[카메라] WIRIS SDK 명령 실패: {error}")
        self._publish_wiris_command_response(
            {
                "phase": "failed",
                "target": target,
                "command": command_text,
                "status_text": status_text,
                "error": str(error),
            }
        )

    def _publish_wiris_command_response(self, payload: dict[str, object]) -> None:
        signal = getattr(self.panel, "wiris_command_response", None)
        emit = getattr(signal, "emit", None)
        if callable(emit):
            emit(dict(payload))

    def _clear_wiris_command_worker(self) -> None:
        self.panel.wiris_command_thread = None
        self.panel.wiris_command_worker = None
        self.panel.wiris_command_send_button.setEnabled(True)
