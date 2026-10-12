"""Camera endpoint candidates: owned rows, selection and controls, no device I/O."""

from dataclasses import dataclass, field
from PyQt5 import QtCore, QtWidgets
from Workstation.workers.panel_camera_workers import _control_candidate_value


@dataclass
class CameraCandidateState:
    rows: list = field(default_factory=list)
    status: str = "NOT RUN"


CANDIDATE_VIEW_FIELDS = (
    "apply_candidate_left_button",
    "apply_candidate_nic_button",
    "apply_candidate_right_button",
    "camera_candidate_table",
    "clear_camera_candidate_button",
    "connection_sequence_label",
)


class CameraCandidateView(QtCore.QObject):
    _camera_candidate_rows = property(
        lambda view: view.state.rows, lambda view, value: setattr(view.state, "rows", value)
    )
    workstation_camera_discovery_status = property(
        lambda view: view.state.status, lambda view, value: setattr(view.state, "status", value)
    )

    def __init__(self, state: CameraCandidateState, *, apply_nic, apply_side, log, parent=None):
        super().__init__(parent)
        if not all(callable(callback) for callback in (apply_nic, apply_side, log)):
            raise TypeError("Candidate view requires explicit intent/log callbacks")
        self.state = state
        self._log = log
        self.camera_candidate_table = QtWidgets.QTableWidget(0, 6)
        self.camera_candidate_table.setHorizontalHeaderLabels(["종류", "주소", "포트", "상태", "인터페이스", "PC IP"])
        self.camera_candidate_table.verticalHeader().setVisible(False)
        self.camera_candidate_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.camera_candidate_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.camera_candidate_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.camera_candidate_table.setMinimumHeight(140)
        self.connection_sequence_label = QtWidgets.QLabel(
            "1 PC IP 적용  |  2 Telnet port 탐색  |  3 영상 송출 활성화  |  4 RTSP port/주소 탐색  |  5 영상 수신 확인  |  6 카메라 Preview 연결"
        )
        self.connection_sequence_label.setObjectName("statusStrip")
        self.connection_sequence_label.setWordWrap(True)
        self.apply_candidate_nic_button = QtWidgets.QPushButton("Assign Camera NIC")
        self.apply_candidate_nic_button.clicked.connect(apply_nic)
        self.apply_candidate_left_button = QtWidgets.QPushButton("Apply to Left")
        self.apply_candidate_left_button.clicked.connect(lambda _checked=False: apply_side("left"))
        self.apply_candidate_right_button = QtWidgets.QPushButton("Apply to Right")
        self.apply_candidate_right_button.clicked.connect(lambda _checked=False: apply_side("right"))
        self.clear_camera_candidate_button = QtWidgets.QPushButton("Clear")
        self.clear_camera_candidate_button.clicked.connect(self.clear_camera_candidate_history)
        self.clear_camera_candidate_button.setEnabled(False)
        self.clear_camera_candidate_button.setToolTip("WIRIS 탐색 후보 내역을 지웁니다.")
        self.camera_candidate_table.itemSelectionChanged.connect(self._refresh_camera_candidate_actions)
        self._camera_candidate_rows: list[dict[str, object]] = []
        self.workstation_camera_discovery_status = "NOT RUN"

        if isinstance(parent, QtWidgets.QWidget):
            for name in CANDIDATE_VIEW_FIELDS:
                getattr(self, name).setParent(parent)

    def _set_camera_candidate_rows(
        self,
        host_interfaces: list[object],
        rtsp_candidates: list[object],
        control_candidates: list[object],
        verified_control_candidates: list[object],
    ) -> None:
        rows: list[dict[str, object]] = []
        for candidate in host_interfaces:
            interface = getattr(candidate, "interface", None)
            if interface is None:
                continue
            neighbor_hosts = tuple(getattr(candidate, "neighbor_hosts", ()) or ())
            status = f"neighbors={', '.join(neighbor_hosts)}" if neighbor_hosts else "candidate"
            rows.append(
                {
                    "kind": "NIC",
                    "address": getattr(interface, "name", ""),
                    "port": "",
                    "status": status,
                    "interface": interface,
                    "host_ip": str(getattr(interface, "address", "")),
                    "candidate": candidate,
                }
            )
        for candidate in rtsp_candidates:
            interface = getattr(candidate, "interface", None)
            rows.append(
                {
                    "kind": "RTSP",
                    "address": str(getattr(candidate, "url", "")),
                    "host": str(getattr(candidate, "host", "")),
                    "port": str(getattr(candidate, "port", "")),
                    "status": "port open",
                    "interface": interface,
                    "host_ip": str(getattr(interface, "address", "")) if interface is not None else "",
                    "candidate": candidate,
                }
            )
        verified_keys: set[tuple[str, int]] = set()
        for candidate in verified_control_candidates:
            host = str(_control_candidate_value(candidate, "host", "") or "")
            try:
                port = int(_control_candidate_value(candidate, "port", 0) or 0)
            except (TypeError, ValueError):
                port = 0
            verified_keys.add((host, port))
            interface = _control_candidate_value(candidate, "interface")
            response = str(_control_candidate_value(candidate, "response", "") or "")
            ok = bool(_control_candidate_value(candidate, "ok"))
            rows.append(
                {
                    "kind": "WIRIS",
                    "address": host,
                    "host": host,
                    "port": str(port) if port else "",
                    "status": f"HIWS {'OK' if ok else 'FAIL'} {response}".strip(),
                    "interface": interface,
                    "host_ip": str(getattr(interface, "address", "")) if interface is not None else "",
                    "candidate": candidate,
                }
            )
        for candidate in control_candidates:
            host = str(_control_candidate_value(candidate, "host", "") or "")
            try:
                port = int(_control_candidate_value(candidate, "port", 0) or 0)
            except (TypeError, ValueError):
                port = 0
            if (host, port) in verified_keys:
                continue
            interface = _control_candidate_value(candidate, "interface")
            rows.append(
                {
                    "kind": "WIRIS",
                    "address": host,
                    "host": host,
                    "port": str(port) if port else "",
                    "status": "TCP open",
                    "interface": interface,
                    "host_ip": str(getattr(interface, "address", "")) if interface is not None else "",
                    "candidate": candidate,
                }
            )
        self._set_camera_candidate_table_rows(rows)

    def _set_camera_candidate_table_rows(self, rows: list[dict[str, object]]) -> None:
        self._camera_candidate_rows = rows
        if rows:
            self.workstation_camera_discovery_status = f"FOUND {len(rows)} ENDPOINTS"
        self.camera_candidate_table.setRowCount(len(rows))
        for row, entry in enumerate(rows):
            for column, key in enumerate(("kind", "address", "port", "status", "interface", "host_ip")):
                value = entry.get(key, "")
                if key == "interface" and value:
                    value = getattr(value, "name", "")
                item = QtWidgets.QTableWidgetItem(str(value))
                self.camera_candidate_table.setItem(row, column, item)
        self.camera_candidate_table.resizeColumnsToContents()
        self._refresh_camera_candidate_actions()

    def clear_camera_candidate_history(self) -> None:
        if not self._camera_candidate_rows:
            return
        self._set_camera_candidate_table_rows([])
        self._log("[카메라] WIRIS 탐색 후보 내역을 지웠습니다.")

    def _refresh_camera_candidate_actions(self) -> None:
        row = self.camera_candidate_table.currentRow()
        kind = ""
        has_selection = 0 <= row < len(self._camera_candidate_rows)
        if has_selection:
            kind = str(self._camera_candidate_rows[row].get("kind", ""))
        self.clear_camera_candidate_button.setEnabled(bool(self._camera_candidate_rows))
        self.clear_camera_candidate_button.setToolTip("WIRIS 탐색 후보 내역을 지웁니다.")
        self.apply_candidate_nic_button.setEnabled(kind == "NIC")
        self.apply_candidate_nic_button.setToolTip(
            "선택한 물리 NIC에 Camera 역할의 주소/경로를 확인 후 적용합니다."
            if kind == "NIC"
            else "NIC 후보를 선택하면 적용할 수 있습니다."
        )
        side_enabled = kind in {"RTSP", "WIRIS"}
        for button in (self.apply_candidate_left_button, self.apply_candidate_right_button):
            button.setEnabled(side_enabled)
            if kind == "RTSP":
                button.setToolTip("선택한 RTSP 후보를 WIRIS 영상 주소에 적용합니다.")
            elif kind == "WIRIS":
                button.setToolTip("선택한 WIRIS 제어 후보를 Control Host와 RTSP host에 적용합니다.")
            else:
                button.setToolTip("RTSP 또는 WIRIS 후보를 선택하면 Left/Right에 적용할 수 있습니다.")

    def _selected_camera_candidate(self) -> dict[str, object] | None:
        row = self.camera_candidate_table.currentRow()
        if row < 0 or row >= len(self._camera_candidate_rows):
            self._log("[카메라] 탐색 후보를 먼저 선택하세요.")
            return None
        return self._camera_candidate_rows[row]

    def build_group(self):
        candidate_group = QtWidgets.QGroupBox("탐색 후보")
        candidate_layout = QtWidgets.QVBoxLayout(candidate_group)
        candidate_layout.addWidget(self.camera_candidate_table)
        candidate_actions = QtWidgets.QHBoxLayout()
        for button in (
            self.apply_candidate_nic_button,
            self.apply_candidate_left_button,
            self.apply_candidate_right_button,
            self.clear_camera_candidate_button,
        ):
            candidate_actions.addWidget(button, 1)
        candidate_layout.addLayout(candidate_actions)
        self.group = candidate_group
        return candidate_group
