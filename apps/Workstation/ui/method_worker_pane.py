"""Compact read-model/command surface for explicit method calculation recovery."""

from __future__ import annotations

from collections.abc import Callable, Mapping
import math

from PyQt5 import QtCore, QtWidgets

from Workstation.workstation_core.measurement_methods import (
    MEASUREMENT_METHOD_IDS, MEASUREMENT_METHOD_LABELS,
)


class MethodWorkerRecoveryPane(QtWidgets.QGroupBox):
    def __init__(self, snapshot_provider: Callable | None, restart_command: Callable | None,
                 parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__("Method 계산 상태", parent)
        self._snapshot_provider = snapshot_provider
        self._restart_command = restart_command
        self._snapshot: Mapping = {}
        self._selected_row: dict[str, object] = {}
        self._full_status = ""
        self._full_feedback = ""
        layout = QtWidgets.QGridLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        self.method_combo = QtWidgets.QComboBox()
        self.method_combo.setAccessibleName("상태를 확인할 측정 방법")
        self.method_combo.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.method_combo.setMinimumContentsLength(18)
        for method in MEASUREMENT_METHOD_IDS:
            self.method_combo.addItem(MEASUREMENT_METHOD_LABELS[method], method)
        self.status_label = QtWidgets.QLabel("--")
        self.feedback_label = QtWidgets.QLabel("경과 시간만으로 계산 실패를 판정하지 않습니다.")
        for label in (self.status_label, self.feedback_label):
            label.setObjectName("statusStrip")
            label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
            label.setFixedHeight(label.fontMetrics().lineSpacing() + 8)
        self.restart_button = QtWidgets.QPushButton("선택 계산 중단·재시작")
        self.restart_button.setAccessibleName("선택 방법의 현재 계산을 중단하고 재시작")
        self.restart_button.setToolTip(
            "표시된 방법의 현재 계산을 FAILED로 기록하고 대기 중인 계산을 계속합니다. "
            "다른 방법의 계산과 관측은 계속됩니다."
        )
        self.restart_button.setEnabled(False)
        layout.addWidget(self.method_combo, 0, 0)
        layout.addWidget(self.status_label, 0, 1)
        layout.addWidget(self.restart_button, 0, 2)
        layout.addWidget(self.feedback_label, 1, 0, 1, 3)
        layout.setColumnStretch(1, 1)
        self.method_combo.currentIndexChanged.connect(self._render)
        self.restart_button.clicked.connect(self._restart)
        self.refresh()

    def refresh(self) -> None:
        try:
            value = self._snapshot_provider() if callable(self._snapshot_provider) else {}
            if not isinstance(value, Mapping):
                raise ValueError("계산 상태 응답 형식 오류")
            self._snapshot = value
        except Exception as exc:
            self._snapshot = {"state": "UNAVAILABLE", "error": str(exc)}
        self._render()

    def _render(self) -> None:
        method = self.method_combo.currentData()
        self.method_combo.setToolTip(self.method_combo.currentText())
        rows = self._snapshot.get("workers", ())
        if not isinstance(rows, (list, tuple)):
            rows = ()
        self._selected_row = next((dict(row) for row in rows
                                   if isinstance(row, Mapping) and row.get("method_id") == method), {})
        row = self._selected_row
        runtime_state = str(self._snapshot.get("state") or "NOT_STARTED")
        state = str(row.get("state") or "NOT_STARTED") if runtime_state == "OPEN" else runtime_state
        try:
            elapsed = float(row.get("elapsed_s") or 0.0)
            seconds = max(0, int(elapsed)) if math.isfinite(elapsed) else 0
            pending = max(0, int(row.get("pending_jobs") or 0))
        except (TypeError, ValueError, OverflowError):
            seconds, pending = 0, 0
        hours, remainder = divmod(seconds, 3600)
        minutes, seconds = divmod(remainder, 60)
        elapsed_text = f"{hours:02d}:{minutes:02d}:{seconds:02d}" if hours < 1000 else ">999 h"
        self._full_status = f"{state} · {elapsed_text} · pending {pending}"
        detail = f"Run: {row.get('measurement_id') or '--'}\nPair: {row.get('pair_id') or '--'}"
        self.status_label.setToolTip(self._full_status + "\n" + detail)
        self.status_label.setAccessibleDescription(self._full_status + "\n" + detail)
        last = row.get("last_restart")
        last = last if isinstance(last, Mapping) else {}
        if row.get("restart_pending"):
            self._full_feedback = "재시작 요청 대기 · 완료 상태를 기다리는 중"
        elif last:
            label = {"RESTARTED": "재시작 완료", "REJECTED": "재시작 거부", "FAILED": "재시작 실패"}.get(str(last.get("status")), "상태 확인 중")
            self._full_feedback = f"{label} · {last.get('reason', '')}"
        else:
            self._full_feedback = str(self._snapshot.get("error") or row.get("error") or
                                      "경과 시간만으로 계산 실패를 판정하지 않습니다.")
        self.restart_button.setEnabled(
            callable(self._restart_command) and runtime_state == "OPEN"
            and state in {"RUNNING", "DISPATCHED", "UNAVAILABLE"}
            and not bool(row.get("restart_pending"))
        )
        self._fit_labels()

    def _fit_labels(self) -> None:
        for label, text in ((self.status_label, self._full_status), (self.feedback_label, self._full_feedback)):
            label.setText(label.fontMetrics().elidedText(text, QtCore.Qt.ElideRight, max(0, label.width() - 4)))
        self.feedback_label.setToolTip(self._full_feedback)
        self.feedback_label.setAccessibleDescription(self._full_feedback)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._fit_labels()

    def _restart(self) -> None:
        if not self.restart_button.isEnabled() or not callable(self._restart_command):
            return
        # Use the identity actually displayed. A fresh read here could silently
        # replace it with a different job before the operator's click is sent.
        row = self._selected_row
        payload = {
            "method_id": row.get("method_id"),
            "expected_worker_generation": row.get("worker_generation"),
            "expected_pair_id": row.get("pair_id"),
            "expected_request_generation": row.get("request_generation"),
        }
        self.restart_button.setEnabled(False)
        try:
            result = self._restart_command(payload)
            if not isinstance(result, Mapping):
                raise ValueError("재시작 응답 형식 오류")
            self.refresh()
            prefix = "재시작 요청 대기" if result.get("accepted") else "재시작 거부"
            self._full_feedback = f"{prefix} · {result.get('reason', '')}"
        except Exception as exc:
            self._full_feedback = f"재시작 실패 · {exc}"
        self._fit_labels()
