"""Render recording status, bounded sheet previews, and graph visibility.

Graph hiding never deletes an append-only measurement evidence row."""
from __future__ import annotations
from collections import deque
from collections.abc import Sequence
from PyQt5 import QtCore, QtWidgets
from Workstation.workstation_core.measurement_widgets import DistanceHistoryPoint
from Workstation.workstation_core.measurement_data_store import LiveDataRows
from Workstation.workstation_core.session_storage.spool import SpoolRows
from Workstation.workstation_core.session_storage.projection import compact_distance_measurement_rows
from Workstation.workstation_core.session_storage.writers import session_sheet_previews
from .contracts import (RECORDING_PREVIEW_LIMIT)
from .state import (RecordingRowsState, SessionSaveState, RecordingSessionState, VideoSegmentState)
from .ports import PresentationPort
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .metadata import Metadata
    from .video_segments import VideoSegments


class Presentation:
    def __init__(self, *,
        rows: RecordingRowsState,
        save_state: SessionSaveState,
        session: RecordingSessionState,
        video_state: VideoSegmentState,
        boundary: PresentationPort,
    ) -> None:
        self.rows = rows
        self.save_state = save_state
        self.session = session
        self.video_state = video_state
        self.boundary = boundary

    def bind_collaborators(self, *,
        metadata: "Metadata",
        video_segments: "VideoSegments",
    ) -> None:
        self.metadata = metadata
        self.video_segments = video_segments

    def _refresh_recording_status_label(self) -> None:
        if not bool(getattr(self.boundary, "_measurement_session_hierarchy_enabled", True)):
            active = self.boundary._session_recording_active()
            mode = str(getattr(self.session, "_recording_mode", "manual") or "manual")
            video_state = str(
                self.video_segments.video_recording_start_snapshot().get("state") or "IDLE"
            )
            storage_reader = getattr(self.boundary, "data_storage_snapshot", None)
            try:
                storage = storage_reader() if callable(storage_reader) else {}
            except Exception:
                storage = {}
            storage_state = str(storage.get("state") or "UNAVAILABLE")
            dropped = int(storage.get("dropped_records") or 0)
            self.boundary.recording_status_label.setText(
                f"데이터 기록: {'진행 중' if active else '중지'} · mode={mode} · "
                f"video={video_state if active else getattr(self.video_state, '_last_recording_presence_state', 'IDLE')} · "
                f"storage={storage_state} · "
                f"pairs={len(self.rows.distance_recording_rows)}"
                + (f" · dropped={dropped}" if dropped else "")
            )
            return
        if self.save_state._session_save_thread is not None:
            self.boundary.recording_status_label.setText(
                f"세션 기록: 저장 중, session={self.session.recording_session_id or 'none'}"
            )
            return
        state = "진행 중" if self.boundary._session_recording_active() else "중지"
        session = f", session={self.session.recording_session_id}" if self.session.recording_session_id else ""
        self.boundary.recording_status_label.setText(
            "세션 기록: "
            f"{state}, 매칭 프레임쌍={len(self.rows.distance_recording_rows)}{session}"
        )

    def _recording_table_value(self, row: dict[str, object], field: str) -> str:
        value = row.get(field, "")
        if value is None:
            return ""
        return str(value)

    def _recording_preview_rows(self) -> list[dict[str, object]]:
        rows = self.rows.distance_recording_rows
        if isinstance(rows, (SpoolRows, LiveDataRows)):
            return rows.preview_rows
        return list(rows[-RECORDING_PREVIEW_LIMIT:])

    def _recording_sheet_names(self) -> tuple[str, ...]:
        """Workbook sheet names, in write order, without needing session rows."""

        return tuple(sheet.name for sheet in session_sheet_previews({}, []))

    def _recording_sheet_kind(self, sheet_name: str) -> str | None:
        return next(
            (sheet.kind for sheet in session_sheet_previews({}, []) if sheet.name == sheet_name),
            None,
        )

    def _active_recording_sheet_name(self) -> str | None:
        if not hasattr(self.boundary, "recording_sheet_tabs"):
            return None
        index = self.boundary.recording_sheet_tabs.currentIndex()
        if index < 0:
            return None
        return self.boundary.recording_sheet_tabs.tabText(index)

    def _append_recording_table_row(self, _row: dict[str, object]) -> None:
        """Refresh the visible sheet after one accepted pair was recorded.

        Sheet columns are projected from every recorded row, so a new sample
        can widen them; rebuilding the visible sheet is simpler than patching
        one row into each of the seven sheets.
        """

        self._refresh_active_recording_sheet(scroll_to_bottom=True)

    def _refresh_active_recording_sheet(self, *, scroll_to_bottom: bool = False) -> None:
        sheet_name = self._active_recording_sheet_name()
        if sheet_name is None:
            return
        table = self.boundary.recording_sheet_tables.get(sheet_name)
        if table is None:
            return
        rows = self._recording_preview_rows()
        # Only the key/value sheets read metadata, and building it runs the
        # clock-sync and camera-timing probes — skip it on the sample sheets,
        # which are refreshed once per recorded pair.
        needs_metadata = sheet_name in {"Summary", "CalibrationStatic"}
        metadata = self.metadata._recording_session_metadata() if needs_metadata else {}
        preview = next(
            (sheet for sheet in session_sheet_previews(metadata, rows) if sheet.name == sheet_name),
            None,
        )
        if preview is None:
            return
        table.blockSignals(True)
        try:
            if preview.kind == "key_value":
                self._fill_recording_sheet(table, ("key", "value"), [
                    (str(key), "" if value is None else str(value)) for key, value in preview.entries
                ])
            else:
                display_rows = (
                    compact_distance_measurement_rows(
                        rows,
                        {
                            "session_started_unix_ns": self.session.recording_session_started_unix_ns,
                            "session_started_monotonic_ns": self.session.recording_session_started_monotonic_ns,
                        },
                    )
                    if sheet_name == "DistanceMeasurements"
                    else []
                )
                self._fill_recording_sheet(
                    table,
                    preview.columns,
                    [
                        tuple(self._recording_table_value(row, field) for field in preview.columns)
                        for row in display_rows
                    ],
                )
        finally:
            table.blockSignals(False)
        if scroll_to_bottom and table.rowCount():
            table.scrollToBottom()
            table.selectRow(table.rowCount() - 1)
        self._refresh_recording_sample_detail()

    def _fill_recording_sheet(
        self,
        table: QtWidgets.QTableWidget,
        headers: Sequence[str],
        rows: Sequence[Sequence[str]],
    ) -> None:
        normalized_headers = tuple(str(header) for header in headers)
        current_headers = tuple(
            table.horizontalHeaderItem(index).text()
            if table.horizontalHeaderItem(index) is not None
            else ""
            for index in range(table.columnCount())
        )
        if table.columnCount() != len(normalized_headers):
            table.setColumnCount(len(normalized_headers))
        if current_headers != normalized_headers:
            table.setHorizontalHeaderLabels(list(normalized_headers))
        if table.rowCount() != len(rows):
            table.setRowCount(len(rows))
        for row_index, values in enumerate(rows):
            for column_index, value in enumerate(values):
                text = str(value)
                item = table.item(row_index, column_index)
                if item is None:
                    item = QtWidgets.QTableWidgetItem(text)
                    item.setFlags(QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable)
                    table.setItem(row_index, column_index, item)
                elif item.text() != text:
                    item.setText(text)

    def _refresh_recording_table(self) -> None:
        if not hasattr(self.boundary, "recording_sheet_tabs"):
            return
        self._refresh_active_recording_sheet()
        if not self.rows.distance_recording_rows:
            self.boundary.recording_sample_detail.clear()

    def _refresh_recording_sample_detail(self) -> None:
        if not hasattr(self.boundary, "recording_sheet_tabs"):
            return
        sheet_name = self._active_recording_sheet_name()
        table = self.boundary.recording_sheet_tables.get(sheet_name) if sheet_name else None
        if table is None:
            return
        if sheet_name is not None and self._recording_sheet_kind(sheet_name) == "key_value":
            self.boundary.recording_sample_detail.clear()
            return
        selected_rows = table.selectionModel().selectedRows()
        preview_rows = self._recording_preview_rows()
        if not selected_rows:
            self.boundary.recording_sample_detail.clear()
            return
        row_index = selected_rows[0].row()
        if row_index < 0 or row_index >= len(preview_rows):
            self.boundary.recording_sample_detail.clear()
            return
        row = preview_rows[row_index]
        lines = [f"{key}: {value}" for key, value in row.items()]
        self.boundary.recording_sample_detail.setPlainText("\n".join(lines))

    @staticmethod
    def _widget_text(widget: object | None) -> str | None:
        reader = getattr(widget, "text", None)
        if not callable(reader):
            return None
        try:
            return str(reader())
        except (AttributeError, RuntimeError, TypeError):
            return None

    def _remove_distance_history_point(self, point: DistanceHistoryPoint) -> None:
        point_key = self.boundary._distance_history_key(point)
        filtered_history: deque[DistanceHistoryPoint] = deque(maxlen=self.boundary.distance_history.maxlen)
        removed_history = False
        for candidate in self.boundary.distance_history:
            if self.boundary._distance_history_key(candidate) == point_key:
                removed_history = True
                continue
            filtered_history.append(candidate)
        if not removed_history:
            self.boundary._log("[Depth] 거리 그래프 숨김 요청이 현재 뷰와 일치하지 않았습니다.")
            return

        self.boundary.distance_history = filtered_history
        append_event = getattr(self.boundary, "append_workstation_run_event", None)
        if callable(append_event):
            try:
                append_event(
                    "measurement_point_hidden",
                    {
                        "sample_key": list(point_key),
                        "timestamp_s": point.timestamp_s,
                        "distance_m": point.distance_m,
                        "quality": point.quality,
                        "method": point.method,
                        "view_only": True,
                    },
                    source="RecordingController",
                    coordinate_contract="baseline_center_frd view annotation; canonical pair remains append-only",
                )
            except Exception as exc:
                self.boundary._log(f"[Depth] 그래프 숨김 감사 이벤트 기록 실패: {exc}")
        self._refresh_recording_status_label()
        self.boundary._refresh_operation_header()
        self.boundary._log(
            f"[Depth] 거리 그래프 점을 뷰에서만 숨겼습니다: {point.distance_m:.2f} m "
            f"({point.method}, quality={point.quality}); canonical 원본 pair/spool은 보존됩니다."
        )
