"""CalibrationInputPicker owns its widgets and transient presentation state.

Domain formulas, native raster/timing admission and calibration solves remain with
the existing backend and core contracts. No workspace host is accepted.
"""

from __future__ import annotations

import json
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Sequence
from PyQt5 import QtCore, QtGui, QtWidgets
from Workstation.workstation_core.settings import DEFAULT_CALIBRATION_ROOT
from Workstation.workstation_core.calibration_selection import (
    CALIBRATION_IMAGE_SUFFIXES, CALIBRATION_SELECTION_SCHEMA,
    empty_calibration_selection, load_calibration_selection_sidecar,
    portable_calibration_selection_paths, resolve_calibration_selection_paths,
)
from Workstation.workstation_core.calibration_timing import (
    INTRINSICS_ONLY, INVALID_METADATA, LEGACY_UNVERIFIED, load_calibration_pair_usage,
)
from Workstation.calibration_workflow.calibration_sessions import discover_calibration_sessions

from .photo_evidence import CalibrationPhotoEvidence
from .inventory import (
    path_stamp, directory_stamp, metadata_directory_stamp,
    pair_capture_time_ns, image_candidates, paired_keys, read_thumbnail,
)


class CalibrationInputPicker(QtCore.QObject):
    def __init__(self, *, read_setup, apply_setup_command=lambda: None, setup_payload=None,
                 root_widget=None, commands=None, selection_changed=None, default_root=DEFAULT_CALIBRATION_ROOT, parent=None):
        super().__init__(parent)
        if not callable(read_setup) or not callable(apply_setup_command):
            raise TypeError("Calibration inputs require setup and apply-command readers")
        self._read_setup = read_setup
        self._apply_setup_command = apply_setup_command
        self.default_root = default_root
        self.setup_payload = setup_payload or (lambda: {})
        self._root_widget = root_widget or (lambda: None)
        self.commands = commands
        self._selection_changed = selection_changed or (lambda: None)
        self._dialog_parent = parent
        self._calibration_pair_picker_signature: tuple[object, ...] | None = None
        self._calibration_pair_picker_thumbnail_generation = 0
        self._calibration_pair_picker_thumbnail_paths: list[tuple[str, Path]] = []
        self._calibration_pair_picker_thumbnail_index = 0
        self._calibration_pair_picker_items: dict[
            str, QtWidgets.QListWidgetItem
        ] = {}
        self._calibration_pair_picker_visible_keys: set[str] = set()
        self._calibration_pair_picker_session_started_unix_ns = time.time_ns()
        self._calibration_session_catalog_signature: tuple[object, ...] | None = None
        self._calibration_session_explicit_root = ""

    def selection_snapshot(self):
        return self._load_calibration_pair_selection()

    def refresh_selection(self):
        self._sync_calibration_file_selection(force=True)

    def cancel_pending_thumbnails(self):
        self._calibration_pair_picker_thumbnail_generation += 1
        self._calibration_pair_picker_thumbnail_paths = []

    def _build_calibration_session_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Calibration session")
        layout = QtWidgets.QGridLayout(group)
        layout.setHorizontalSpacing(8)
        self.calibration_session_combo = QtWidgets.QComboBox()
        self.calibration_session_combo.setObjectName("calibrationSessionSelector")
        self.calibration_session_combo.currentIndexChanged.connect(
            self._on_calibration_session_selected
        )
        self.calibration_session_refresh_button = QtWidgets.QPushButton("Refresh sessions")
        self.calibration_session_refresh_button.clicked.connect(
            lambda _checked=False: self._refresh_calibration_session_selector(force=True)
        )
        self.calibration_session_path_label = QtWidgets.QLabel("No saved session selected")
        self.calibration_session_path_label.setObjectName("statusStrip")
        self.calibration_session_path_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.calibration_session_result_label = QtWidgets.QLabel(
            "Results: beside the selected left/right photo set"
        )
        self.calibration_session_result_label.setObjectName("compactContract")
        self.calibration_session_result_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(QtWidgets.QLabel("Photo set"), 0, 0)
        layout.addWidget(self.calibration_session_combo, 0, 1)
        layout.addWidget(self.calibration_session_refresh_button, 0, 2)
        layout.addWidget(self.calibration_session_path_label, 1, 0, 1, 3)
        layout.addWidget(self.calibration_session_result_label, 2, 0, 1, 3)
        self._refresh_calibration_session_selector(force=True)
        return group


    def _refresh_calibration_session_selector(self, *, force: bool = False) -> None:
        combo = getattr(self, "calibration_session_combo", None)
        if combo is None:
            return
        configured = str(
            self._read_setup().get("calibration_root_resolved") or ""
        )
        default_base = Path(self.default_root)
        catalog_roots = [default_base]
        configured_base = Path(configured) if configured else None
        if (
            configured_base is not None
            and configured_base.is_dir()
            and configured_base.resolve() != default_base.resolve()
        ):
            catalog_roots.append(configured_base)
        sessions_by_root = {}
        for catalog_root in catalog_roots:
            for session in discover_calibration_sessions(catalog_root):
                sessions_by_root[str(session.root)] = session
        sessions = tuple(
            sorted(
                sessions_by_root.values(),
                key=lambda session: (session.updated_ns, str(session.root)),
                reverse=True,
            )
        )
        signature = (
            tuple(str(root) for root in catalog_roots),
            tuple((str(session.root), session.pair_count, session.updated_ns) for session in sessions),
        )
        if not force and signature == self._calibration_session_catalog_signature:
            return
        self._calibration_session_catalog_signature = signature
        selected_root = self._calibration_session_explicit_root
        combo.blockSignals(True)
        try:
            combo.clear()
            combo.addItem("Select a photo set…", None)
            for session in sessions:
                combo.addItem(session.display_name, str(session.root))
            selected_index = combo.findData(selected_root) if selected_root else 0
            if selected_index > 0:
                combo.setCurrentIndex(selected_index)
                self.calibration_session_path_label.setText(
                    str(combo.currentData() or "No saved session selected")
                )
                self.calibration_session_result_label.setText(
                    f"Results: {combo.currentData()}/intrinsics_zoom_##.json and intrinsics.json"
                )
            else:
                if self._calibration_session_explicit_root:
                    self.cancel_pending_thumbnails()
                self._calibration_session_explicit_root = ""
                combo.setCurrentIndex(0)
                self.calibration_session_path_label.setText("No saved session selected")
                self.calibration_session_result_label.setText(
                    "Select a Photo set before loading calculation inputs."
                )
        finally:
            combo.blockSignals(False)
        self._update_calibration_compute_selection_state()


    def _on_calibration_session_selected(self, index: int) -> None:
        combo = getattr(self, "calibration_session_combo", None)
        if combo is None or index < 0:
            return
        root = str(combo.itemData(index) or "").strip()
        if root != self._calibration_session_explicit_root:
            self.cancel_pending_thumbnails()
        if not root:
            self._calibration_session_explicit_root = ""
            self.calibration_session_path_label.setText("No saved session selected")
            self.calibration_session_result_label.setText(
                "Select a Photo set before loading calculation inputs."
            )
            self._calibration_file_picker_signature = None
            self._update_calibration_compute_selection_state()
            self._sync_calibration_file_selection()
            return
        root_widget = self._root_widget()
        if isinstance(root_widget, QtWidgets.QLineEdit):
            root_widget.setText(root)
        apply_setup = self._apply_setup_command()
        if callable(apply_setup):
            try:
                apply_setup(self.setup_payload())
            except Exception as exc:
                self._calibration_session_explicit_root = ""
                self.calibration_session_path_label.setText(f"Session apply failed: {exc}")
                self._update_calibration_compute_selection_state()
                return
        self._calibration_session_explicit_root = root
        self.calibration_session_path_label.setText(root)
        self.calibration_session_result_label.setText(
            f"Results: {root}/intrinsics_zoom_##.json and intrinsics.json"
        )
        self._calibration_pair_picker_signature = None
        self._calibration_file_picker_signature = None
        self._update_calibration_compute_selection_state()
        self._sync_calibration_file_selection()
        self._selection_changed()


    def apply_selected_calibration_session(self) -> bool:
        """Apply the visible session choice immediately before a compute action."""

        self._last_calibration_session_apply_performed = False
        combo = getattr(self, "calibration_session_combo", None)
        if combo is None:
            return False
        root = str(combo.currentData() or "").strip()
        if not root or root != self._calibration_session_explicit_root:
            self.calibration_session_path_label.setText(
                "Select a Photo set before running calibration."
            )
            self._update_calibration_compute_selection_state()
            return False
        root_widget = self._root_widget()
        configured_root = str(
            self._read_setup().get("calibration_root_resolved") or ""
        )
        needs_apply = configured_root != root
        if isinstance(root_widget, QtWidgets.QLineEdit) and root_widget.text().strip() != root:
            root_widget.setText(root)
            needs_apply = True
        self._normalize_legacy_calibration_selection(Path(root))
        if not needs_apply:
            return True
        apply_setup = self._apply_setup_command()
        if not callable(apply_setup):
            return True
        try:
            apply_setup(self.setup_payload())
        except Exception as exc:
            self.calibration_session_path_label.setText(f"Session apply failed: {exc}")
            return False
        self._last_calibration_session_apply_performed = True
        self.calibration_session_path_label.setText(root)
        self.calibration_session_result_label.setText(
            f"Results: {root}/intrinsics_zoom_##.json and intrinsics.json"
        )
        self._calibration_file_picker_signature = None
        self._sync_calibration_file_selection()
        self._selection_changed()
        return True


    def _update_calibration_compute_selection_state(self) -> None:
        combo = getattr(self, "calibration_session_combo", None)
        current_root = str(combo.currentData() or "").strip() if combo is not None else ""
        enabled = bool(
            current_root
            and self._calibration_session_explicit_root
            and current_root == self._calibration_session_explicit_root
        )
        tooltip = "" if enabled else "Select a Photo set before running calibration."
        for command_id in ("calibration.run_intrinsic", "calibration.run_extrinsics"):
            button = self.commands.primary.get(command_id) if self.commands is not None else None
            if button is not None:
                button.setEnabled(enabled)
                button.setToolTip(tooltip)


    @staticmethod
    def _normalize_legacy_calibration_selection(root: Path) -> None:
        """Retain the hook without rewriting unsupported or malformed evidence."""

        _ = root


    def _build_calibration_file_selection_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("계산에 사용할 사진")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(5)

        controls = QtWidgets.QGridLayout()
        controls.setHorizontalSpacing(8)
        controls.setVerticalSpacing(5)
        self.calibration_left_files_button = QtWidgets.QPushButton("Left 사진 선택…")
        self.calibration_right_files_button = QtWidgets.QPushButton("Right 사진 선택…")
        self.calibration_clear_files_button = QtWidgets.QPushButton("선택 초기화")
        self.calibration_left_files_button.clicked.connect(
            lambda _checked=False: self._choose_calibration_files("left")
        )
        self.calibration_right_files_button.clicked.connect(
            lambda _checked=False: self._choose_calibration_files("right")
        )
        self.calibration_clear_files_button.clicked.connect(
            self._clear_calibration_file_selection
        )
        self.calibration_file_selection_summary = QtWidgets.QLabel(
            "파일창에서 좌/우 사진을 각각 선택하세요."
        )
        self.calibration_file_selection_summary.setObjectName("statusStrip")
        self.calibration_file_selection_summary.setWordWrap(True)
        controls.addWidget(self.calibration_left_files_button, 0, 0)
        controls.addWidget(self.calibration_right_files_button, 0, 1)
        controls.addWidget(self.calibration_clear_files_button, 0, 2)
        controls.addWidget(self.calibration_file_selection_summary, 1, 0, 1, 3)
        layout.addLayout(controls)

        self.calibration_selected_files_table = QtWidgets.QTableWidget(0, 6)
        self.calibration_selected_files_table.setObjectName("calibrationSelectedFilesTable")
        self.calibration_selected_files_table.setHorizontalHeaderLabels(
            ("#", "Left", "Right", "Left 정보", "Right 정보", "Pair")
        )
        self.calibration_selected_files_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.calibration_selected_files_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.calibration_selected_files_table.verticalHeader().setVisible(False)
        self.calibration_selected_files_table.setMinimumHeight(100)
        self.calibration_selected_files_table.setMaximumHeight(230)
        header = self.calibration_selected_files_table.horizontalHeader()
        header.setStretchLastSection(True)
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(2, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(3, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(4, QtWidgets.QHeaderView.ResizeToContents)
        layout.addWidget(self.calibration_selected_files_table)

        hint = QtWidgets.QLabel(
            "intrinsic은 좌/우 목록을 각각 사용하고, Checkerboard R,t는 같은 행의 좌/우 파일을 pair로 사용합니다. "
            "사진 정보의 시각은 exact RTCP/NTP frame → app capture record/request → legacy file mtime "
            "순으로 표시하며 출처를 함께 적습니다. 선택 목록은 세션 폴더의 "
            "calibration_pair_selection.json에 저장됩니다."
        )
        hint.setObjectName("compactContract")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        self._calibration_file_picker_signature = None
        return group


    def _resolve_calibration_selection_paths(
        self,
        raw_paths: object,
        root: Path,
        *,
        stored_root: object = None,
    ) -> list[Path]:
        paths, _errors = resolve_calibration_selection_paths(
            raw_paths,
            root,
            stored_root=stored_root,
        )
        return paths


    def _calibration_file_selection_for_root(self, root: Path) -> tuple[list[Path], list[Path], bool]:
        selection_path = root / "calibration_pair_selection.json"
        data = load_calibration_selection_sidecar(selection_path).to_mapping()
        if str(data.get("_sidecar_state") or "") == "INVALID":
            self._calibration_file_selection_path_errors = tuple(
                str(value) for value in data.get("_sidecar_errors") or ()
            )
            return [], [], True
        manual = bool(data.get("manual_mode"))
        stored_root = data.get("root")
        left, left_errors = resolve_calibration_selection_paths(
            data.get("left_paths"), root, stored_root=stored_root
        )
        right, right_errors = resolve_calibration_selection_paths(
            data.get("right_paths"), root, stored_root=stored_root
        )
        self._calibration_file_selection_path_errors = (*left_errors, *right_errors)
        return left, right, manual


    def _choose_calibration_files(self, side: str) -> None:
        if side not in {"left", "right"}:
            return
        root_text = str(self._read_setup().get("calibration_root_resolved") or "")
        root = Path(root_text) if root_text else Path(self.default_root)
        current_left, current_right, manual = self._calibration_file_selection_for_root(root)
        current = current_left if side == "left" and manual else current_right if side == "right" and manual else []
        start = str(current[0].parent if current else root / side)
        files, _selected_filter = QtWidgets.QFileDialog.getOpenFileNames(
            self._dialog_parent,
            f"{side.title()} calibration images 선택",
            start,
            "Calibration images (*.png *.jpg *.jpeg *.tif *.tiff *.bmp);;All files (*)",
        )
        if not files:
            return
        selected = [Path(path).expanduser().resolve() for path in files if Path(path).is_file()]
        if not selected:
            return
        other = current_right if side == "left" and manual else current_left if side == "right" and manual else []
        candidate_paths = [*selected, *other]
        roots = {self._calibration_file_root(path) for path in candidate_paths}
        if len(roots) > 1:
            self.calibration_file_selection_summary.setText(
                "좌/우 사진은 같은 calibration session 폴더에서 선택해야 합니다."
            )
            return
        selected_root = next(iter(roots), root).resolve()
        left = selected if side == "left" else other
        right = selected if side == "right" else other
        root_widget = self._root_widget()
        if isinstance(root_widget, QtWidgets.QLineEdit):
            root_widget.setText(str(selected_root))
        apply_setup = self._apply_setup_command()
        if callable(apply_setup):
            try:
                apply_setup(self.setup_payload())
            except Exception as exc:
                self.calibration_file_selection_summary.setText(f"사진 root 적용 실패: {exc}")
                return
        self._save_calibration_file_selection(selected_root, left, right)
        if str(selected_root) != self._calibration_session_explicit_root:
            self.cancel_pending_thumbnails()
        self._calibration_session_explicit_root = str(selected_root)
        self._refresh_calibration_session_selector(force=True)
        self._calibration_file_picker_signature = None
        self._sync_calibration_file_selection()


    def _save_calibration_file_selection(
        self,
        root: Path,
        left: Sequence[Path],
        right: Sequence[Path],
    ) -> None:
        root = Path(root).expanduser().resolve(strict=False)
        left_absolute = [Path(path).expanduser().resolve(strict=False) for path in left]
        right_absolute = [Path(path).expanduser().resolve(strict=False) for path in right]
        left_paths = portable_calibration_selection_paths(root, left_absolute)
        right_paths = portable_calibration_selection_paths(root, right_absolute)
        common_keys = [
            self._calibration_file_key(left_path, "left")
            for left_path in left_absolute
            if any(
                self._calibration_file_key(right_path, "right")
                == self._calibration_file_key(left_path, "left")
                for right_path in right_absolute
            )
        ]
        previous = self._load_calibration_pair_selection()
        previous_left, left_errors = resolve_calibration_selection_paths(
            previous.get("excluded_left_paths"),
            root,
            stored_root=previous.get("root"),
        )
        previous_right, right_errors = resolve_calibration_selection_paths(
            previous.get("excluded_right_paths"),
            root,
            stored_root=previous.get("root"),
        )
        if left_errors or right_errors:
            self.calibration_file_selection_summary.setText(
                "기존 제외 경로를 복구할 수 없어 선택 목록을 덮어쓰지 않았습니다: "
                + ", ".join((*left_errors, *right_errors))
            )
            return
        selected_left = set(left_absolute)
        selected_right = set(right_absolute)
        excluded_left = sorted(path for path in previous_left if path in selected_left)
        excluded_right = sorted(path for path in previous_right if path in selected_right)
        payload = {
            "schema": CALIBRATION_SELECTION_SCHEMA,
            "root": ".",
            "manual_mode": bool(left_paths or right_paths),
            "left_paths": left_paths,
            "right_paths": right_paths,
            "included_keys": sorted(set(common_keys)),
            "excluded_left_paths": portable_calibration_selection_paths(root, excluded_left),
            "excluded_right_paths": portable_calibration_selection_paths(root, excluded_right),
            "excluded_pair_keys": list(previous.get("excluded_pair_keys") or ()),
        }
        path = root / "calibration_pair_selection.json"
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
        except OSError as exc:
            self.calibration_file_selection_summary.setText(f"선택 목록 저장 실패: {exc}")


    def _clear_calibration_file_selection(self) -> None:
        root_text = str(self._read_setup().get("calibration_root_resolved") or "")
        root = Path(root_text) if root_text else Path(self.default_root)
        try:
            path = root / "calibration_pair_selection.json"
            data = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
            if not isinstance(data, dict):
                data = {}
            data.update(
                {
                    "schema": CALIBRATION_SELECTION_SCHEMA,
                    "root": ".",
                    "manual_mode": False,
                    "left_paths": [],
                    "right_paths": [],
                    "included_keys": [],
                    "excluded_left_paths": [],
                    "excluded_right_paths": [],
                    "excluded_pair_keys": [],
                }
            )
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
        except (OSError, ValueError, TypeError) as exc:
            self.calibration_file_selection_summary.setText(f"선택 초기화 실패: {exc}")
            return
        self._calibration_file_picker_signature = None
        self._sync_calibration_file_selection()


    def _sync_calibration_file_selection(self, *, force: bool = False) -> None:
        table = getattr(self, "calibration_selected_files_table", None)
        if table is None:
            return
        if not self._calibration_session_explicit_root:
            table.setRowCount(0)
            self._calibration_file_picker_signature = None
            self.calibration_file_selection_summary.setText(
                "Photo set을 선택하면 계산 후보가 표시됩니다."
            )
            return
        root = Path(self._calibration_session_explicit_root)
        left, right, manual = self._calibration_file_selection_for_root(root)
        selection = self._load_calibration_pair_selection()
        excluded_left_paths, excluded_left_errors = resolve_calibration_selection_paths(
            selection.get("excluded_left_paths"),
            root,
            stored_root=selection.get("root"),
        )
        excluded_right_paths, excluded_right_errors = resolve_calibration_selection_paths(
            selection.get("excluded_right_paths"),
            root,
            stored_root=selection.get("root"),
        )
        path_errors = (
            *getattr(self, "_calibration_file_selection_path_errors", ()),
            *excluded_left_errors,
            *excluded_right_errors,
        )
        if path_errors:
            table.setRowCount(0)
            self._calibration_file_picker_signature = None
            self.calibration_file_selection_summary.setText(
                "선택/제외 경로 복구 실패 · 전체 후보 fallback 차단: "
                + ", ".join(path_errors)
            )
            return
        excluded_left = {str(path) for path in excluded_left_paths}
        excluded_right = {str(path) for path in excluded_right_paths}
        excluded_pair_keys = {str(key) for key in selection.get("excluded_pair_keys") or ()}
        if not manual:
            # Before the operator opens a file dialog, show the available
            # matching session pairs as a review-only preview. The controller
            # retains its legacy even-spread selection contract until explicit
            # files are saved here.
            left_candidates = image_candidates(root / "left")
            right_candidates = image_candidates(root / "right")
            left_by_key = {
                self._calibration_file_key(path, "left"): path
                for path in left_candidates
            }
            right_by_key = {
                self._calibration_file_key(path, "right"): path
                for path in right_candidates
            }
            keys = sorted(left_by_key.keys() & right_by_key.keys())
            if keys:
                left = [left_by_key[key] for key in keys]
                right = [right_by_key[key] for key in keys]
            else:
                left = left_candidates
                right = right_candidates
        signature = (
            str(root),
            manual,
            tuple(self._calibration_file_signature(path) for path in left),
            tuple(self._calibration_file_signature(path) for path in right),
            tuple(sorted(excluded_left)),
            tuple(sorted(excluded_right)),
            tuple(sorted(excluded_pair_keys)),
        )
        if not force and signature == getattr(self, "_calibration_file_picker_signature", None):
            return
        previous_signature = getattr(self, "_calibration_file_picker_signature", None)
        preserve_view = bool(previous_signature and previous_signature[0] == str(root))
        vertical_scroll = table.verticalScrollBar().value() if preserve_view else 0
        horizontal_scroll = table.horizontalScrollBar().value() if preserve_view else 0
        current_row = table.currentRow() if preserve_view else -1
        current_column = table.currentColumn() if preserve_view else -1
        self._calibration_file_picker_signature = signature
        table.setRowCount(0)
        row_count = max(len(left), len(right))
        table.setRowCount(row_count)
        for index in range(row_count):
            left_path = left[index] if index < len(left) else None
            right_path = right[index] if index < len(right) else None
            left_key = self._calibration_file_key(left_path, "left") if left_path is not None else ""
            right_key = self._calibration_file_key(right_path, "right") if right_path is not None else ""
            is_excluded = (
                (left_path is not None and str(left_path) in excluded_left)
                or (right_path is not None and str(right_path) in excluded_right)
                or (left_key and left_key == right_key and left_key in excluded_pair_keys)
            )
            pair_usage = (
                load_calibration_pair_usage(
                    root / "capture_metadata" / f"capture_{left_key}.json"
                )
                if left_key and left_key == right_key
                else None
            )
            if is_excluded:
                usage_state = "EXCLUDED"
            elif pair_usage is not None and not pair_usage.stereo_eligible:
                usage_state = (
                    "INTRINSICS ONLY"
                    if pair_usage.state == INTRINSICS_ONLY
                    else "INVALID METADATA"
                )
            elif pair_usage is not None and pair_usage.state == LEGACY_UNVERIFIED:
                usage_state = "matched · timing unverified"
            elif left_path is not None and right_path is not None and left_key == right_key:
                usage_state = "matched"
            elif left_path is not None and right_path is not None:
                usage_state = "same row"
            else:
                usage_state = "incomplete"
            values = (
                str(index + 1),
                left_path.name if left_path is not None else "--",
                right_path.name if right_path is not None else "--",
                self._calibration_file_info(left_path) if left_path is not None else "--",
                self._calibration_file_info(right_path) if right_path is not None else "--",
                usage_state,
            )
            for column, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value)
                tooltip_path = (
                    left_path
                    if column in {1, 3}
                    else right_path
                    if column in {2, 4}
                    else None
                )
                if tooltip_path is not None:
                    tooltip = self._calibration_file_tooltip(tooltip_path)
                elif column == 5 and pair_usage is not None:
                    usage_contract = (
                        "K,D input: allowed\nStereo R,t: excluded"
                        if not pair_usage.stereo_eligible
                        else "K,D input: allowed\nStereo R,t: allowed"
                    )
                    tooltip = f"{pair_usage.detail}\n{usage_contract}"
                else:
                    tooltip = value
                item.setToolTip(tooltip)
                table.setItem(index, column, item)
        table.resizeRowsToContents()
        if preserve_view:
            if 0 <= current_row < table.rowCount() and current_column >= 0:
                table.setCurrentCell(current_row, current_column)
            table.verticalScrollBar().setValue(vertical_scroll)
            table.horizontalScrollBar().setValue(horizontal_scroll)
        if manual:
            pair_count = min(len(left), len(right))
            self.calibration_file_selection_summary.setText(
                f"지정 파일: left={len(left)}, right={len(right)}, 계산 행 pair={pair_count}"
            )
        else:
            self.calibration_file_selection_summary.setText(
                f"자동 후보 미리보기: left={len(left)}, right={len(right)} · 파일창에서 고정 선택 가능"
            )


    def _build_calibration_pair_picker_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("계산용 사진 선택")
        layout = QtWidgets.QVBoxLayout(group)
        self.calibration_manual_selection_checkbox = QtWidgets.QCheckBox("수동 선택 사용")
        self.calibration_manual_selection_checkbox.setToolTip(
            "끄면 기존처럼 균등 분산 자동 선택을 사용합니다. 켜면 아래에서 체크한 쌍만 계산에 사용합니다."
        )
        self.calibration_manual_selection_checkbox.toggled.connect(
            self._on_calibration_pair_selection_changed
        )
        layout.addWidget(self.calibration_manual_selection_checkbox)
        filter_row = QtWidgets.QHBoxLayout()
        self.calibration_show_all_pairs_checkbox = QtWidgets.QCheckBox("전체 사진 보기")
        self.calibration_show_all_pairs_checkbox.setToolTip(
            "기본값은 Workstation 실행 이후 저장된 pair만 표시합니다. "
            "켜면 calibration root의 기존 pair까지 모두 표시합니다."
        )
        self.calibration_show_all_pairs_checkbox.toggled.connect(
            self._handle_calibration_pair_picker_filter_changed
        )
        filter_row.addWidget(self.calibration_show_all_pairs_checkbox)
        self.calibration_pair_picker_summary = QtWidgets.QLabel("앱 실행 후 캡처 대기")
        self.calibration_pair_picker_summary.setObjectName("statusStrip")
        filter_row.addWidget(self.calibration_pair_picker_summary, 1)
        layout.addLayout(filter_row)
        self.calibration_pair_picker_list = QtWidgets.QListWidget()
        self.calibration_pair_picker_list.setObjectName("calibrationPairPickerList")
        self.calibration_pair_picker_list.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.calibration_pair_picker_list.setIconSize(QtCore.QSize(64, 36))
        self.calibration_pair_picker_list.setMinimumHeight(90)
        self.calibration_pair_picker_list.setMaximumHeight(220)
        self.calibration_pair_picker_list.setToolTip(
            "체크된 쌍만(수동 선택 사용 시) Run calibration / Calculate R,t 에 사용됩니다."
        )
        self.calibration_pair_picker_list.itemChanged.connect(
            self._on_calibration_pair_selection_changed
        )
        layout.addWidget(self.calibration_pair_picker_list)
        self._calibration_pair_picker_root: Path | None = None
        return group


    def _calibration_pair_selection_path(self) -> Path | None:
        root_text = str(self._read_setup().get("calibration_root_resolved") or "")
        if not root_text:
            return None
        return Path(root_text) / "calibration_pair_selection.json"


    def _load_calibration_pair_selection(self) -> dict[str, object]:
        path = self._calibration_pair_selection_path()
        if path is None:
            return {
                **empty_calibration_selection(),
                "_sidecar_state": "ABSENT",
                "_sidecar_errors": [],
            }
        return load_calibration_selection_sidecar(path).to_mapping()


    def _on_calibration_pair_selection_changed(self, *_args: object) -> None:
        path = self._calibration_pair_selection_path()
        if path is None:
            return
        selection = self._load_calibration_pair_selection()
        included = {str(key) for key in selection.get("included_keys") or ()}
        visible_keys = set(getattr(self, "_calibration_pair_picker_visible_keys", set()))
        for index in range(self.calibration_pair_picker_list.count()):
            item = self.calibration_pair_picker_list.item(index)
            key = str(item.data(QtCore.Qt.UserRole))
            if key not in visible_keys:
                continue
            if item.checkState() == QtCore.Qt.Checked:
                included.add(key)
            else:
                included.discard(key)
        payload = {
            "schema": CALIBRATION_SELECTION_SCHEMA,
            "root": ".",
            "manual_mode": bool(self.calibration_manual_selection_checkbox.isChecked()),
            "included_keys": sorted(included),
            "left_paths": list(selection.get("left_paths") or ()),
            "right_paths": list(selection.get("right_paths") or ()),
            "excluded_left_paths": list(selection.get("excluded_left_paths") or ()),
            "excluded_right_paths": list(selection.get("excluded_right_paths") or ()),
            "excluded_pair_keys": list(selection.get("excluded_pair_keys") or ()),
        }
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        except OSError:
            pass


    def _handle_calibration_pair_picker_filter_changed(self, _checked: bool) -> None:
        self._calibration_pair_picker_signature = None
        self._sync_calibration_pair_picker()


    def _sync_calibration_pair_picker(self) -> None:
        picker = getattr(self, "calibration_pair_picker_list", None)
        if picker is None:
            return
        root_text = str(self._read_setup().get("calibration_root_resolved") or "")
        if not root_text:
            return
        root = Path(root_text)
        left_dir, right_dir = root / "left", root / "right"

        selection_path = root / "calibration_pair_selection.json"
        show_all_widget = getattr(self, "calibration_show_all_pairs_checkbox", None)
        show_all = bool(show_all_widget.isChecked()) if show_all_widget is not None else True
        session_started_ns = int(
            getattr(self, "_calibration_pair_picker_session_started_unix_ns", 0) or 0
        )
        signature = (
            str(root),
            directory_stamp(left_dir, "left_"),
            directory_stamp(right_dir, "right_"),
            metadata_directory_stamp(root / "capture_metadata"),
            path_stamp(selection_path),
            show_all,
            session_started_ns,
        )
        if signature == self._calibration_pair_picker_signature:
            return

        all_keys = paired_keys(root)
        metadata_dir = root / "capture_metadata"

        if show_all or session_started_ns <= 0:
            keys = all_keys
        else:
            keys = [key for key in all_keys if pair_capture_time_ns(root, key) >= session_started_ns]
        self._calibration_pair_picker_visible_keys = set(keys)
        summary_label = getattr(self, "calibration_pair_picker_summary", None)
        if summary_label is not None:
            scope = "전체" if show_all else "앱 실행 후"
            summary_label.setText(f"{scope} 표시: {len(keys)} / 전체 {len(all_keys)} pair")
        selection = self._load_calibration_pair_selection()
        included = set(selection.get("included_keys") or ())
        same_root = bool(
            self._calibration_pair_picker_signature
            and self._calibration_pair_picker_signature[0] == str(root)
        )
        current_item = picker.currentItem() if same_root else None
        current_key = current_item.data(QtCore.Qt.UserRole) if current_item is not None else None
        scroll_value = picker.verticalScrollBar().value() if same_root else 0
        picker.blockSignals(True)
        self.calibration_manual_selection_checkbox.blockSignals(True)
        try:
            self.calibration_manual_selection_checkbox.setChecked(bool(selection.get("manual_mode")))
            picker.clear()
            self._calibration_pair_picker_items = {}
            thumbnail_paths: list[tuple[str, Path]] = []
            for key in keys:
                origin = "manual"
                metadata_path = metadata_dir / f"capture_{key}.json"
                pair_usage = load_calibration_pair_usage(metadata_path)
                if metadata_path.exists():
                    try:
                        metadata_payload = json.loads(
                            metadata_path.read_text(encoding="utf-8")
                        )
                        if isinstance(metadata_payload, Mapping):
                            origin = str(metadata_payload.get("capture_origin", "manual"))
                    except (OSError, ValueError, TypeError):
                        pass
                left_path = left_dir / f"left_{key}.png"
                right_path = right_dir / f"right_{key}.png"
                left_ns, left_source, _left_evidence = self._calibration_file_capture_time(
                    left_path
                )
                right_ns, right_source, _right_evidence = self._calibration_file_capture_time(
                    right_path
                )
                left_time = self._format_calibration_file_capture_time(left_ns, left_source)
                right_time = self._format_calibration_file_capture_time(right_ns, right_source)
                usage_suffix = (
                    " · INTRINSICS ONLY"
                    if pair_usage.state == INTRINSICS_ONLY
                    else " · INVALID METADATA"
                    if pair_usage.state == INVALID_METADATA
                    else " · LEGACY TIMING UNVERIFIED"
                    if pair_usage.state == LEGACY_UNVERIFIED
                    else ""
                )
                item = QtWidgets.QListWidgetItem(
                    f"{key} · {origin}{usage_suffix} · L {left_time} · R {right_time}"
                )
                usage_contract = (
                    "K,D input: allowed\nStereo R,t: excluded"
                    if not pair_usage.stereo_eligible
                    else "K,D input: allowed\nStereo R,t: allowed"
                )
                item.setToolTip(
                    "Left\n"
                    + self._calibration_file_tooltip(left_path)
                    + "\n\nRight\n"
                    + self._calibration_file_tooltip(right_path)
                    + f"\n\nPair usage\n{pair_usage.detail}\n{usage_contract}"
                )
                item.setData(QtCore.Qt.UserRole, key)
                if pair_usage.stereo_eligible:
                    item.setFlags(
                        item.flags()
                        | QtCore.Qt.ItemIsUserCheckable
                        | QtCore.Qt.ItemIsEnabled
                    )
                    item.setCheckState(
                        QtCore.Qt.Checked
                        if not included or key in included
                        else QtCore.Qt.Unchecked
                    )
                else:
                    item.setCheckState(QtCore.Qt.Unchecked)
                    item.setFlags(item.flags() & ~QtCore.Qt.ItemIsUserCheckable)
                picker.addItem(item)
                self._calibration_pair_picker_items[key] = item
                thumbnail_paths.append((key, left_path))
        finally:
            picker.blockSignals(False)
            self.calibration_manual_selection_checkbox.blockSignals(False)
        if same_root:
            restored = self._calibration_pair_picker_items.get(current_key)
            if restored is not None:
                picker.setCurrentItem(restored)
            picker.verticalScrollBar().setValue(scroll_value)
        self._calibration_pair_picker_signature = signature
        self._calibration_pair_picker_thumbnail_generation += 1
        self._calibration_pair_picker_thumbnail_paths = thumbnail_paths
        self._calibration_pair_picker_thumbnail_index = 0
        generation = self._calibration_pair_picker_thumbnail_generation
        QtCore.QTimer.singleShot(0, lambda generation=generation: self._load_next_calibration_pair_thumbnail(generation))


    def _load_next_calibration_pair_thumbnail(self, generation: int) -> None:
        """Load one picker thumbnail between normal UI event-loop turns."""

        if generation != self._calibration_pair_picker_thumbnail_generation:
            return
        index = self._calibration_pair_picker_thumbnail_index
        paths = self._calibration_pair_picker_thumbnail_paths
        if index >= len(paths):
            return
        key, left_path = paths[index]
        self._calibration_pair_picker_thumbnail_index = index + 1
        item = self._calibration_pair_picker_items.get(key)
        if item is not None:
            try:
                thumbnail = read_thumbnail(left_path)
                if not thumbnail.isNull():
                    item.setIcon(QtGui.QIcon(QtGui.QPixmap.fromImage(thumbnail)))
            except RuntimeError:
                # A refresh may have replaced the QListWidget while a
                # previously queued thumbnail callback was still pending.
                return
        QtCore.QTimer.singleShot(
            0,
            lambda generation=generation: self._load_next_calibration_pair_thumbnail(generation),
        )


    # File evidence has a separate I/O boundary; these names preserve the old helpers.
    _calibration_file_root = staticmethod(CalibrationPhotoEvidence._calibration_file_root)
    _calibration_file_key = staticmethod(CalibrationPhotoEvidence._calibration_file_key)
    _calibration_capture_metadata_path = staticmethod(CalibrationPhotoEvidence._calibration_capture_metadata_path)
    _calibration_file_capture_time = staticmethod(CalibrationPhotoEvidence._calibration_file_capture_time)
    _format_calibration_file_capture_time = staticmethod(CalibrationPhotoEvidence._format_calibration_file_capture_time)
    _calibration_file_info = staticmethod(CalibrationPhotoEvidence._calibration_file_info)
    _calibration_file_tooltip = staticmethod(CalibrationPhotoEvidence._calibration_file_tooltip)
    _calibration_file_signature = staticmethod(CalibrationPhotoEvidence._calibration_file_signature)
