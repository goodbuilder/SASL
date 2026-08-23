from __future__ import annotations

import json
import os
from pathlib import Path
from types import SimpleNamespace

from PyQt5 import QtGui, QtWidgets

from Workstation.workspaces import CalibrationWorkspace


def _write_pair(root: Path, key: str = "001") -> tuple[Path, Path]:
    (root / "left").mkdir(parents=True, exist_ok=True)
    (root / "right").mkdir(parents=True, exist_ok=True)
    image = QtGui.QImage(32, 18, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("black"))
    left = root / "left" / f"left_{key}.png"
    right = root / "right" / f"right_{key}.png"
    assert image.save(str(left))
    assert image.save(str(right))
    return left, right


def test_calibration_photo_time_prefers_side_specific_rtcp_metadata(tmp_path: Path) -> None:
    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    left, right = _write_pair(tmp_path)
    metadata_dir = tmp_path / "capture_metadata"
    metadata_dir.mkdir()
    metadata_path = metadata_dir / "capture_001.json"
    metadata_path.write_text(
        json.dumps(
            {
                "capture_origin": "auto",
                "capture_recorded_unix_ns": 1_700_000_010_000_000_000,
                "left_rtcp_frame_ntp_unix_s": 1_700_000_000.125,
                "right_rtcp_frame_ntp_unix_s": 1_700_000_000.375,
            }
        ),
        encoding="utf-8",
    )

    left_ns, left_source, left_evidence = CalibrationWorkspace._calibration_file_capture_time(
        left
    )
    right_ns, right_source, right_evidence = CalibrationWorkspace._calibration_file_capture_time(
        right
    )

    assert left_ns == 1_700_000_000_125_000_000
    assert right_ns == 1_700_000_000_375_000_000
    assert left_source == right_source == "RTCP/NTP frame"
    assert left_evidence == right_evidence == str(metadata_path)
    assert "RTCP/NTP frame" in CalibrationWorkspace._calibration_file_info(left)


def test_ftp_photo_does_not_mislabel_live_preview_rtcp_as_still_time(tmp_path: Path) -> None:
    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    left, _right = _write_pair(tmp_path)
    metadata_dir = tmp_path / "capture_metadata"
    metadata_dir.mkdir()
    (metadata_dir / "capture_001.json").write_text(
        json.dumps(
            {
                "capture_origin": "ftp_oz",
                "capture_recorded_unix_ns": 1_700_000_020_250_000_000,
                "left_rtcp_frame_ntp_unix_s": 1_700_000_000.125,
            }
        ),
        encoding="utf-8",
    )

    unix_ns, source, _evidence = CalibrationWorkspace._calibration_file_capture_time(left)

    assert unix_ns == 1_700_000_020_250_000_000
    assert source == "app capture record"


def test_calibration_photo_time_labels_malformed_legacy_fallback_as_mtime(
    tmp_path: Path,
) -> None:
    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    left, _right = _write_pair(tmp_path)
    metadata_dir = tmp_path / "capture_metadata"
    metadata_dir.mkdir()
    (metadata_dir / "capture_001.json").write_text("{not-json", encoding="utf-8")
    mtime_ns = 1_600_000_000_625_000_000
    os.utime(left, ns=(mtime_ns, mtime_ns))

    unix_ns, source, evidence = CalibrationWorkspace._calibration_file_capture_time(left)

    assert unix_ns == mtime_ns
    assert source == "file mtime"
    assert evidence == str(left)
    assert "file mtime" in CalibrationWorkspace._calibration_file_tooltip(left)


def test_calibration_photo_time_format_does_not_crash_on_out_of_range_timestamp() -> None:
    text = CalibrationWorkspace._format_calibration_file_capture_time(
        10**40,
        "malformed metadata",
    )

    assert "시각 해석 불가" in text
    assert "malformed metadata" in text


def test_compute_photo_table_displays_left_and_right_times_and_refreshes_for_metadata(
    tmp_path: Path,
    monkeypatch,
) -> None:
    import Workstation.workspaces as workspaces_module

    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    left, right = _write_pair(tmp_path)
    metadata_dir = tmp_path / "capture_metadata"
    metadata_dir.mkdir()
    metadata_path = metadata_dir / "capture_001.json"
    metadata_path.write_text(
        json.dumps(
            {
                "capture_origin": "manual",
                "left_rtcp_frame_ntp_unix_s": 1_700_000_000.125,
                "right_rtcp_frame_ntp_unix_s": 1_700_000_000.375,
            }
        ),
        encoding="utf-8",
    )
    evidence = SimpleNamespace(
        calibration_setup=lambda: {"calibration_root_resolved": str(tmp_path)}
    )
    monkeypatch.setattr(workspaces_module, "_evidence", lambda _backend: evidence)
    workspace = CalibrationWorkspace.__new__(CalibrationWorkspace)
    workspace.backend = SimpleNamespace()
    workspace.calibration_selected_files_table = QtWidgets.QTableWidget(0, 6)
    workspace.calibration_file_selection_summary = QtWidgets.QLabel()
    workspace._calibration_file_picker_signature = None
    workspace._calibration_session_explicit_root = str(tmp_path)

    workspace._sync_calibration_file_selection()

    table = workspace.calibration_selected_files_table
    assert table.rowCount() == 1
    assert "RTCP/NTP frame" in table.item(0, 3).text()
    assert "RTCP/NTP frame" in table.item(0, 4).text()
    assert "사진 시각:" in table.item(0, 1).toolTip()
    initial_signature = workspace._calibration_file_picker_signature

    updated_metadata_ns = metadata_path.stat().st_mtime_ns + 1_000_000_000
    os.utime(metadata_path, ns=(updated_metadata_ns, updated_metadata_ns))
    workspace._sync_calibration_file_selection()

    assert workspace._calibration_file_picker_signature != initial_signature
    assert left.name in table.item(0, 1).text()
    assert right.name in table.item(0, 2).text()
    table.deleteLater()
    workspace.calibration_file_selection_summary.deleteLater()
    _app.processEvents()
