"""Recording View feature: owned state/views and explicit dependency ports.

Coordinate values retain their producer contracts (NED, body FRD, optical RDF,
and baseline-center FRD). This ownership change performs no axis conversion.
"""
from __future__ import annotations

from Workstation.panel.panel_feature_runtime import PanelFeature
import time
import uuid
from PyQt5 import (
    QtCore,
    QtWidgets,
)
from Workstation.panel.panel_support import RECORDING_PREVIEW_LIMIT
from Workstation.recording.contracts import (
    new_measurement_attempt_generation_id as _new_measurement_attempt_generation_id,
)
from Workstation.runtime_paths import (
    WORKSTATION_MEASUREMENT_DATA_ROOT,
    resolve_workstation_data_path,
)
from Workstation.workers.session_workers import SessionSaveWorker
from Workstation.workstation_core.measurement_attempt import (
    AttitudeSnapshot,
    GpsSnapshot,
    MeasurementAttempt,
)
from Workstation.workstation_core.measurement_data_store import (
    LiveDataRows,
)
from Workstation.workstation_core.measurement_widgets import (
    DistanceHistoryPoint,
)
from Workstation.workstation_core.session_storage.spool import SpoolRows
from Workstation.workstation_core.session_storage.paths import format_video_paths, resolve_session_dir
from Workstation.workstation_core.settings import (
    AppSettings,
)
from collections import (
    deque,
)
from datetime import (
    datetime,
    timezone,
)
from pathlib import (
    Path,
)


class RecordingViewFeature(PanelFeature):
    feature_name = 'recording_view'
    dependency_names = (
        '_build_clock_sync_tab',
        '_log',
        '_polish_controls',
        '_recording_sheet_names',
        '_refresh_active_recording_sheet',
        '_refresh_operation_header',
        '_refresh_recording_sample_detail',
        '_refresh_recording_status_label',
        '_refresh_recording_table',
        '_workstation_setting',
        'allocate_measurement_video_segment',
        'app_root',
        'data_storage_snapshot',
        'gnss_base_recording_rows',
        'measurement_storage_context',
        'save_current_settings_from_ui',
        'save_distance_history_csv',
        'save_history_button',
        'save_settings_button',
    )
    writable_dependencies = frozenset(('save_history_button', 'save_settings_button'))

    def initialize_state(self, *, auto_start=False, auto_start_pixhawk=False, app_root=None, settings_path=None):
        self.distance_history: deque[DistanceHistoryPoint] = deque(maxlen=600)
        self.geometry_comparison_rows: tuple[dict[str, object], ...] = ()
        self.distance_recording_rows: SpoolRows | list[dict[str, object]] = SpoolRows(
            preview_limit=RECORDING_PREVIEW_LIMIT,
            supports_updates=True,
        )
        self.distance_recording_keys: list[tuple[object, ...]] = []
        self.external_parameter_recording_rows: deque[dict[str, object]] = deque(maxlen=256)
        self.camera_recording_rows: dict[str, SpoolRows] = {
            "left": SpoolRows(),
            "right": SpoolRows(),
        }
        self.px4_recording_rows: dict[str, SpoolRows] = {
            "left": SpoolRows(),
            "right": SpoolRows(),
        }
        self.extrinsics_comparison_recording_rows: SpoolRows = SpoolRows()
        self.method_evaluation_bundle_rows: SpoolRows = SpoolRows()
        self._session_save_thread: QtCore.QThread | None = None
        self._session_save_worker: SessionSaveWorker | None = None
        self._session_save_retry_required = False
        self._session_save_finalized = False
        self._automatic_session_recording = False
        self._manual_recording_active = False
        self._method_evaluation_stop_retry_scheduled = False
        self._recording_stop_retry_generation = 0
        self._recording_stop_retry_request: dict[str, object] | None = None
        self._recording_stop_retry_timer = QtCore.QTimer(self.widget)
        self._recording_mode = "manual"
        self._measurement_package_explicit_lifecycle = False
        self._measurement_session_hierarchy_enabled = False
        self._measurement_session = None
        self._measurement_session_id = ""
        self._measurement_session_state = "IDLE"
        self._measurement_id = ""
        self._measurement_app_run_id = (
            f"run-{datetime.now(timezone.utc):%Y%m%dT%H%M%S.%fZ}-{uuid.uuid4().hex[:8]}"
        )
        self._measurement_package = None
        self._measurement_package_state = "IDLE"
        self._measurement_package_detail = "Ready for Measurement"
        self._measurement_package_error = ""
        self._measurement_package_detection_started = False
        self._measurement_package_detection_completed = False
        self._measurement_package_finalize_requested = False
        self._measurement_package_finalize_retry_scheduled = False
        self._measurement_package_registered_paths: set[tuple[str, str]] = set()
        self._measurement_package_inventory_errors: list[str] = []
        self._measurement_package_revision = 0
        self._recovered_session_metadata: dict[str, object] | None = None
        self.distance_recording_enabled = False
        self.distance_recording_started_monotonic_s: float | None = None
        self.recording_session_id = ""
        self._measurement_attempt_generation_id = _new_measurement_attempt_generation_id()
        self.recording_session_started_wall_s: float | None = None
        self.recording_session_started_monotonic_s: float | None = None
        self.recording_session_started_unix_ns: int | None = None
        self.recording_session_started_monotonic_ns: int | None = None
        self.recording_session_anchor_uncertainty_ns: int | None = None
        self._recording_clock_sync_start_observation: dict[str, object] | None = None
        self._recording_clock_sync_end_observation: dict[str, object] | None = None
        self.recording_reference_attitudes: dict[str, AttitudeSnapshot] = {}
        self.recording_reference_gps: dict[str, GpsSnapshot] = {}
        self.recording_origin_gps: GpsSnapshot | None = None
        self.recording_reference_baseline_ned_m: tuple[float, float, float] | None = None
        self._recording_pair_row_index: dict[tuple[object, ...], int] = {}
        self._recording_pending_native_request_rows: dict[
            str, tuple[str, int]
        ] = {}
        self.video_recording_enabled = False
        self.video_recording_paths: dict[str, Path] = {}
        self._video_recording_allocation_sizes: dict[str, int] = {}
        self._video_recording_start_snapshot: dict[str, object] = {
            "schema": "workstation-video-start-v1",
            "state": "IDLE",
            "detail": "video recording is not requested",
            "sides": {},
        }
        self._last_distance_history_key: tuple[object, ...] | None = None
        self._measurement_attempt_counter = 0
        self._last_measurement_attempt_key: tuple[object, ...] | None = None
        self._last_measurement_attempt: MeasurementAttempt | None = None
        self.last_distance_quality = "WARNING"
        self.last_distance_status = "거리: 좌/우 검출 매칭 대기 중"

    def _video_recording_kwargs(self, camera: str) -> dict[str, object]:
        if not self.video_recording_enabled:
            return {}
        path = self.video_recording_paths.get(camera)
        if path is None:
            return {}
        return {
            "record_video_path": str(path),
            "record_video_fps": 30.0,
        }

    def _recording_session_dir(self, stamp: str | None = None) -> Path:
        context_reader = getattr(self, "measurement_storage_context", None)
        if callable(context_reader):
            context = context_reader(stamp)
            if context.get("root"):
                return Path(str(context["root"]))
        reader = getattr(self, "data_storage_snapshot", None)
        if callable(reader):
            snapshot = reader()
            if isinstance(snapshot, dict) and snapshot.get("root"):
                return Path(str(snapshot["root"]))
        configured = str(
            self._workstation_setting(
                "measurement_data_root",
                AppSettings.measurement_data_root,
            )
            or ""
        )
        return (
            resolve_workstation_data_path(configured)
            if configured
            else WORKSTATION_MEASUREMENT_DATA_ROOT
        )

    def _measurement_session_dir(self, session_id: str | None = None) -> Path:
        selected = session_id or self._measurement_session_id or self.recording_session_id
        return resolve_session_dir(
            self.app_root / "recordings",
            str(selected or ""),
        )

    def _measurement_sessions_root(self) -> Path:
        return self.app_root / "recordings"

    def _prepare_video_recording_paths(self) -> None:
        created_utc_ns = time.time_ns()
        paths: dict[str, Path] = {}
        for side in ("left", "right"):
            try:
                allocator = getattr(self, "allocate_measurement_video_segment", None)
                path = allocator(side, utc_ns=created_utc_ns) if callable(allocator) else None
                if path is None:
                    raise RuntimeError("video segment allocator is unavailable")
                paths[side] = Path(path)
            except (OSError, RuntimeError, TypeError, ValueError) as exc:
                self._log(
                    f"[저장] {side} video path unavailable; Measurement continues: {exc}"
                )
        self.video_recording_paths = paths

    def _format_video_recording_paths(self) -> str:
        return format_video_paths(self.video_recording_paths)

    def _session_recording_active(self) -> bool:
        return self.distance_recording_enabled or self.video_recording_enabled

    def _distance_history_key(self, point: DistanceHistoryPoint) -> tuple[object, ...]:
        return point.sample_key or (point.timestamp_s, point.distance_m, point.method)

    def clear_distance_history(self) -> None:
        if self._session_save_thread is not None:
            self.recording_status_label.setText("데이터 기록: 저장 중에는 기록을 초기화할 수 없습니다")
            return
        live_row_owners = (
            self.distance_recording_rows,
            *self.camera_recording_rows.values(),
            *self.px4_recording_rows.values(),
            self.gnss_base_recording_rows,
            self.extrinsics_comparison_recording_rows,
            self.method_evaluation_bundle_rows,
        )
        if self.distance_recording_enabled and any(
            isinstance(rows, LiveDataRows) for rows in live_row_owners
        ):
            self.recording_status_label.setText(
                "데이터 기록: 활성 Measurement/Manual record 중에는 기록을 초기화할 수 없습니다"
            )
            return
        self.distance_history.clear()
        self.distance_recording_rows.clear()
        self.distance_recording_keys.clear()
        self.external_parameter_recording_rows.clear()
        self.camera_recording_rows["left"].clear()
        self.camera_recording_rows["right"].clear()
        self.px4_recording_rows["left"].clear()
        self.px4_recording_rows["right"].clear()
        self.gnss_base_recording_rows.clear()
        self.extrinsics_comparison_recording_rows.clear()
        self.method_evaluation_bundle_rows.clear()
        if self.distance_recording_enabled and self.recording_session_id:
            spool_root = self._recording_session_dir(self.recording_session_id) / ".spool"
            if isinstance(self.distance_recording_rows, SpoolRows):
                self.distance_recording_rows.bind(spool_root / "pair_rows.jsonl")
            for side, rows in self.camera_recording_rows.items():
                if isinstance(rows, SpoolRows):
                    rows.bind(spool_root / f"camera_{side}.jsonl")
            for side, rows in self.px4_recording_rows.items():
                if isinstance(rows, SpoolRows):
                    rows.bind(spool_root / f"px4_{side}.jsonl")
            if isinstance(self.gnss_base_recording_rows, SpoolRows):
                self.gnss_base_recording_rows.bind(spool_root / "gnss_base.jsonl")
            if isinstance(self.extrinsics_comparison_recording_rows, SpoolRows):
                self.extrinsics_comparison_recording_rows.bind(
                    spool_root / "extrinsics_comparison.jsonl"
                )
            if isinstance(self.method_evaluation_bundle_rows, SpoolRows):
                self.method_evaluation_bundle_rows.bind(
                    spool_root / "method_evaluation_bundles.jsonl"
                )
        self._recording_pair_row_index.clear()
        self._recording_pending_native_request_rows.clear()
        if not self.distance_recording_enabled:
            self.distance_recording_started_monotonic_s = None
        self._last_distance_history_key = None
        self.distance_plot.clear()
        self._refresh_recording_table()
        self._refresh_recording_status_label()
        self._log("[Depth] 거리 그래프와 미저장 기록을 초기화했습니다.")
        self._refresh_operation_header()

    def build_recording_preview(self, save_layout, time_layout):
        save_group = QtWidgets.QGroupBox("데이터 / 설정")
        save_form = QtWidgets.QGridLayout(save_group)
        self.save_history_button = QtWidgets.QPushButton("데이터 flush")
        self.save_history_button.clicked.connect(self.save_distance_history_csv)
        self.save_history_button.setToolTip(
            "백그라운드 append writer에 flush를 요청합니다. "
            "저장 성공 여부는 다음 측정 시작을 차단하지 않습니다."
        )
        self.save_settings_button = QtWidgets.QPushButton("전체 설정 저장")
        self.save_settings_button.clicked.connect(self.save_current_settings_from_ui)
        self.save_history_button.setText("데이터 flush")
        self.save_settings_button.setText("전체 설정 저장")
        self.session_summary_label = QtWidgets.QLabel("아직 기록된 매칭 프레임쌍이 없습니다.")
        self.session_summary_label.setObjectName("statusStrip")
        self.session_summary_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.session_summary_label.setWordWrap(True)
        self.recording_status_label = QtWidgets.QLabel("데이터 기록: 중지")
        self.recording_status_label.setObjectName("statusStrip")
        self.recording_status_label.setWordWrap(True)
        self.recording_status_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        save_form.addWidget(self.save_history_button, 0, 0)
        save_form.addWidget(self.save_settings_button, 0, 1)
        save_form.addWidget(self.session_summary_label, 1, 0, 1, 2)
        save_form.addWidget(self.recording_status_label, 2, 0, 1, 2)
        save_layout.addWidget(save_group)
        self.save_history_button.hide()

        self.recording_table_group = QtWidgets.QGroupBox("현재 기록 메모리 미리보기")
        recording_table_layout = QtWidgets.QVBoxLayout(self.recording_table_group)
        self.recording_sheet_tabs = QtWidgets.QTabWidget()
        self.recording_sheet_tabs.setDocumentMode(True)
        self.recording_sheet_tables: dict[str, QtWidgets.QTableWidget] = {}
        for sheet_name in self._recording_sheet_names():
            table = QtWidgets.QTableWidget(0, 0)
            table.setObjectName("recordingSampleTable")
            table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
            table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
            table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
            table.setAlternatingRowColors(True)
            table.verticalHeader().setVisible(False)
            table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
            table.itemSelectionChanged.connect(self._refresh_recording_sample_detail)
            self.recording_sheet_tables[sheet_name] = table
            self.recording_sheet_tabs.addTab(table, sheet_name)
        self.recording_sheet_tabs.currentChanged.connect(
            lambda _index: self._refresh_active_recording_sheet()
        )
        recording_table_layout.addWidget(self.recording_sheet_tabs, 2)
        self.recording_sample_detail = QtWidgets.QPlainTextEdit()
        self.recording_sample_detail.setObjectName("recordingSampleDetail")
        self.recording_sample_detail.setReadOnly(True)
        self.recording_sample_detail.setMaximumBlockCount(300)
        self.recording_sample_detail.setPlaceholderText("테이블에서 샘플을 선택하면 전체 내부 기록 필드가 표시됩니다.")
        recording_table_layout.addWidget(self.recording_sample_detail, 1)
        save_layout.addWidget(self.recording_table_group, 1)

        self._build_clock_sync_tab(time_layout)
        self._polish_controls()

