"""Read-only, pair-consistent Method results. Display deadlines never stop workers."""
from __future__ import annotations

import json
import math
import time
from datetime import datetime

from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_LABELS


COORDINATES = {
    "original": (("range_m", "forward_m", "right_m", "down_m"),
                 ("Rg (m)", "Forward (m)", "Right (m)", "Down (m)"),
                 "baseline_center_frd · 원점: Method 카메라 중점 · 축: 기준선과 광축"),
    "level": (("target_level_range_m", "target_level_forward_m", "target_level_right_m", "target_level_down_m"),
              ("수평 Rg (m)", "수평 Forward (m)", "수평 Right (m)", "Down (m)"),
              "baseline_center_local_level_frd · 원점: Method별 카메라 중점(서로 다를 수 있음) · 전방: pair P1 수평 전방 · Down: NED 아래"),
    "wgs84": (("target_latitude_deg", "target_longitude_deg", "target_altitude_ellipsoid_m", "target_altitude_msl_m"),
              ("위도 (deg)", "경도 (deg)", "타원체고 (m)", "MSL (m)"),
              "WGS84 · 위도/경도 · 타원체고와 MSL은 별도 기준 · 변환 입력이 없으면 미확보"),
}
SUCCESS = {"VALID", "ESTIMATED"}


def number(value, digits=3):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return "—"
    return f"{value:.{digits}f}"


def stored_pose(value):
    try:
        return json.loads(value or "{}")
    except (TypeError, ValueError):
        return {"reason": "저장된 자세 근거를 읽을 수 없음"}


class PairCursor:
    def __init__(self):
        self.measurement_id = ""
        self.pair_id = ""
        self.started_s = 0.
        self.shown_terminal = False

    def choose(self, snapshot, now_s):
        run = snapshot.get("measurement_id", "")
        latest = snapshot.get("latest_pair_id", "")
        if run != self.measurement_id:
            self.measurement_id, self.pair_id = run, ""
            self.shown_terminal = False
        if latest and (not self.pair_id or self.shown_terminal or now_s - self.started_s >= 1.):
            if latest != self.pair_id:
                self.pair_id, self.started_s = latest, now_s
                self.shown_terminal = False
        return self.pair_id


class MeasurementComparisonPane(QtWidgets.QWidget):
    def __init__(self, reader, parent=None, *, clock=time.monotonic):
        super().__init__(parent)
        self.reader, self.clock = reader, clock
        self.cursor = PairCursor()
        self.snapshot = {}
        self._method_order = []
        self._selected_method = ""
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.header = QtWidgets.QLabel("기록 결과 없음 · 영상 쌍 대기")
        self.header.setObjectName("statusStrip")
        self.header.setWordWrap(True)
        self.header.setFixedHeight(self.fontMetrics().height() * 4 + 8)
        layout.addWidget(self.header)
        self.coordinate_combo = QtWidgets.QComboBox()
        for label, key in (("기존 F/R/D", "original"), ("수평 F/R/D", "level"), ("WGS84", "wgs84")):
            self.coordinate_combo.addItem(label, key)
        layout.addWidget(self.coordinate_combo)
        self.frame_label = QtWidgets.QLabel()
        self.frame_label.setWordWrap(True)
        self.frame_label.setFixedHeight(self.fontMetrics().height() * 3 + 4)
        layout.addWidget(self.frame_label)
        self.model = QtGui.QStandardItemModel(self)
        self.table = QtWidgets.QTableView()
        self.table.setModel(self.model)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSortingEnabled(False)
        self.table.setWordWrap(False)
        self.table.verticalHeader().hide()
        self.table.setMinimumHeight(170)
        self.table.setMaximumHeight(330)
        self.table.setHorizontalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
        self.model.setColumnCount(9)
        for col, width in enumerate((260, 135, 145, 145, 145, 135, 135, 120, 300)):
            self.table.setColumnWidth(col, width)
        layout.addWidget(self.table)
        details_group = QtWidgets.QGroupBox("선택 Method · Quality / Result / R,t")
        details_layout = QtWidgets.QVBoxLayout(details_group)
        self.details = QtWidgets.QPlainTextEdit()
        self.details.setReadOnly(True)
        self.details.setMinimumHeight(170)
        self.details.setMaximumHeight(300)
        details_layout.addWidget(self.details)
        layout.addWidget(details_group)
        self.coordinate_combo.currentIndexChanged.connect(self.render)
        self.table.selectionModel().selectionChanged.connect(self._select)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(250)
        self.timer.timeout.connect(self.refresh_results)
        self.render()

    def showEvent(self, event):
        super().showEvent(event)
        self.timer.start()
        self.refresh_results()

    def hideEvent(self, event):
        self.timer.stop()
        super().hideEvent(event)

    def refresh_results(self):
        try:
            snapshot = self.reader(self.cursor.measurement_id, self.cursor.pair_id or None)
            old_run = self.cursor.measurement_id
            selected = self.cursor.choose(snapshot, self.clock())
            if selected and snapshot.get("pair_id") != selected:
                snapshot = self.reader(self.cursor.measurement_id, selected)
                if snapshot.get("measurement_id") != self.cursor.measurement_id:
                    self.cursor.choose(snapshot, self.clock())
            if old_run != self.cursor.measurement_id:
                self._method_order, self._selected_method = [], ""
            self.snapshot = snapshot
            self.cursor.shown_terminal = bool(snapshot.get("all_terminal"))
            self.render()
        except Exception as exc:
            self.snapshot = {"rows": [], "reason": f"결과 조회 실패: {exc}"}
            self.render()

    def _select(self, *_args):
        selected = self.table.selectionModel().selectedRows()
        if selected and selected[0].row() < len(self._method_order):
            self._selected_method = self._method_order[selected[0].row()]
        self.render_details()

    @staticmethod
    def _values(row):
        outcome = row.get("outcome") or {}
        # Both projections came from the same immutable outcome in the record.
        return {**(outcome.get("result") or {}), **(row.get("derived") or {})}

    def render(self, *_args):
        mode = self.coordinate_combo.currentData() or "original"
        fields, headers, frame = COORDINATES[mode]
        self.frame_label.setText(frame)
        self.model.setHorizontalHeaderLabels(("Method", "계산 상태", "좌표 상태", *headers, "Ray gap (m)", "경고 / 차단 사유"))
        rows = {row["method_id"]: row for row in self.snapshot.get("rows", ())}
        if not rows:
            self._method_order = []
        self._method_order.extend(k for k in rows if k not in self._method_order)
        vertical = self.table.verticalScrollBar().value()
        horizontal = self.table.horizontalScrollBar().value()
        blocker = QtCore.QSignalBlocker(self.table.selectionModel())
        self.model.setRowCount(len(self._method_order))
        for index, method_id in enumerate(self._method_order):
            row = rows.get(method_id, {"status": "N/A"})
            data = self._values(row)
            status = row.get("status", "N/A")
            successful = status in SUCCESS
            values = [number(data.get(k), 9 if k.endswith("_deg") else 3) if successful else "—" for k in fields]
            available = sum(v != "—" for v in values)
            coordinate_state = "AVAILABLE" if available == 4 else "PARTIAL" if available else "미확보"
            warnings = list(data.get("warnings") or (row.get("outcome") or {}).get("warnings") or ())
            if mode == "level" and successful and available == 0:
                warnings.append(data.get("target_level_reason") or "P1 자세/기준선 입력 없음")
            if mode == "wgs84" and successful and available < 4:
                evidence = data.get("target_coordinate_evidence") or {}
                warnings.append((evidence.get("reason") or "월드 기준 입력 없음") if available == 0 else "MSL 없음")
            reason = " · ".join(str(v) for v in (row.get("reason"), *warnings) if v)
            cells = (MEASUREMENT_METHOD_LABELS.get(method_id, method_id), status, coordinate_state,
                     *values, number(data.get("ray_gap_m")) if successful else "—", reason)
            for col, text in enumerate(cells):
                item = self.model.item(index, col)
                if item is None:
                    item = QtGui.QStandardItem()
                    self.model.setItem(index, col, item)
                if item.text() != str(text):
                    item.setText(str(text))
                item.setToolTip(str(text))
                if 3 <= col <= 7:
                    item.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        if self._selected_method not in self._method_order:
            self._selected_method = next(iter(self._method_order), "")
        if self._selected_method:
            self.table.selectRow(self._method_order.index(self._selected_method))
        self.table.verticalScrollBar().setValue(vertical)
        self.table.horizontalScrollBar().setValue(horizontal)
        del blocker
        stamp = self.snapshot.get("created_unix_ns")
        time_text = datetime.fromtimestamp(stamp / 1e9).astimezone().isoformat(timespec="milliseconds") if stamp else "—"
        done = sum(r.get("status") != "PENDING" for r in rows.values())
        age = self.snapshot.get("created_monotonic_ns")
        age_text = number(max(0., self.clock() - age / 1e9), 1) if age else "—"
        self.header.setText(
            f"run={self.snapshot.get('measurement_id', '—')} · pair={self.snapshot.get('pair_id') or '대기'} · {time_text}\n"
            f"결과 {done}/{len(rows)} · 입력 {age_text}s 전 · 화면 대기 최대 1초 (계산/저장은 계속)"
            + (f" · {self.snapshot['reason']}" if self.snapshot.get("reason") else ""))
        if not rows and not self.snapshot.get("reason"):
            self.header.setText(self.header.text() + (
                " · 참여 Method 없음" if self.snapshot.get("pair_id") else " · 기록 결과 없음 · 첫 영상 쌍 대기"))
        self.header.setToolTip(self.header.text())
        self.render_details()

    def render_details(self):
        row = next((r for r in self.snapshot.get("rows", ()) if r["method_id"] == self._selected_method), None)
        if row is None:
            self.details.setPlainText(self.snapshot.get("reason") or "기록 결과 없음")
            return
        data = self._values(row)
        outcome = row.get("outcome") or {}
        attempt = outcome.get("attempt_payload") or {}
        lines = [f"{MEASUREMENT_METHOD_LABELS.get(self._selected_method, self._selected_method)} · pair={self.snapshot.get('pair_id')}",
                 f"Quality: {row.get('status')} · {row.get('reason') or ''}",
                 f"Ray gap={number(data.get('ray_gap_m'))} m · 영상 Δt={number(data.get('time_delta_s'))} s"]
        for side in ("left", "right"):
            lines.append(f"{side} 자세 age={number(attempt.get(side + '_attitude_age_s'))} s · "
                         f"GPS={json.dumps(attempt.get(side + '_gps'), ensure_ascii=False, default=str)}")
        for name, (fields, _, frame) in COORDINATES.items():
            lines.append(f"Result [{name}] · {frame}")
            lines.append(" · ".join(f"{field}={number(data.get(field), 9 if field.endswith('_deg') else 3) if row.get('status') in SUCCESS else '—'}" for field in fields))
        lines.append("수평 좌표 상태: " + str(data.get("target_level_status") or "미확보") + " · " + str(data.get("target_level_reason") or ""))
        world = data.get("target_coordinate_evidence") or {}
        lines.append("WGS84 근거: " + str(world.get("reason") or "") + " · MSL: " + str(world.get("msl_reason") or "—"))
        lines.extend(("Warnings: " + json.dumps(data.get("warnings") or outcome.get("warnings") or [], ensure_ascii=False),
                      "R,t / sources:", json.dumps({"R": data.get("rotation"), "t": data.get("translation"),
                        "frame": "X_right_RDF = R * X_left_RDF + t; metres",
                        "raw_point_left_RDF": (outcome.get("result", {}).get("triangulated") or {}).get("source_point_left_rdf_m"),
                        "pose_evidence": stored_pose(attempt.get("measurement_pose_evidence_json")),
                        "method_evidence": row.get("evidence"),
                        "level_evidence": data.get("target_level_evidence"),
                        "wgs84_evidence": data.get("target_coordinate_evidence")}, ensure_ascii=False, indent=2, default=str)))
        text = "\n".join(lines)
        if text != self.details.toPlainText():
            scroll = self.details.verticalScrollBar().value()
            self.details.setPlainText(text)
            self.details.verticalScrollBar().setValue(scroll)
