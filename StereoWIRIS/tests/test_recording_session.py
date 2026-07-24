import csv
import json
from pathlib import Path
from xml.etree import ElementTree
from zipfile import ZipFile

from wiris_stereo_distance.recording_session import (
    SpoolRows,
    discover_retryable_sessions,
    format_video_paths,
    load_retryable_session,
    raw_video_paths,
    save_session_files,
    session_dir,
    session_id_now,
    write_recovery_metadata,
)


def test_recording_session_paths_use_stable_session_layout() -> None:
    root = Path("/tmp/recordings")

    assert session_dir(root, "20260605_120000") == (
        root / "sessions" / "20260605_120000"
    )
    assert raw_video_paths(root, "20260605_120000") == {
        "left": root / "sessions" / "20260605_120000" / "video" / "raw_left.mp4",
        "right": root / "sessions" / "20260605_120000" / "video" / "raw_right.mp4",
    }


def test_format_video_paths_handles_empty_and_populated_paths() -> None:
    assert format_video_paths({}) == "저장 경로 없음"
    assert (
        format_video_paths({"left": Path("/tmp/session/video/raw_left.mp4")})
        == "left=/tmp/session/video/raw_left.mp4"
    )


def _xlsx_sheet_names_and_row_counts(path: Path) -> tuple[list[str], list[int]]:
    spreadsheet_namespace = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
    relationship_namespace = (
        "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
    )
    package_relationship_namespace = (
        "http://schemas.openxmlformats.org/package/2006/relationships"
    )
    with ZipFile(path) as archive:
        workbook = ElementTree.fromstring(archive.read("xl/workbook.xml"))
        relationships = ElementTree.fromstring(archive.read("xl/_rels/workbook.xml.rels"))
        targets = {
            relationship.attrib["Id"]: relationship.attrib["Target"]
            for relationship in relationships.findall(
                f"{{{package_relationship_namespace}}}Relationship"
            )
        }
        sheets = workbook.find(f"{{{spreadsheet_namespace}}}sheets")
        assert sheets is not None
        names: list[str] = []
        row_counts: list[int] = []
        for sheet in sheets:
            names.append(sheet.attrib["name"])
            relationship_id = sheet.attrib[f"{{{relationship_namespace}}}id"]
            target = targets[relationship_id].lstrip("/")
            worksheet_path = target if target.startswith("xl/") else f"xl/{target}"
            worksheet = ElementTree.fromstring(archive.read(worksheet_path))
            sheet_data = worksheet.find(f"{{{spreadsheet_namespace}}}sheetData")
            assert sheet_data is not None
            row_counts.append(len(sheet_data.findall(f"{{{spreadsheet_namespace}}}row")))
    return names, row_counts


def _xlsx_dynamic_axis_values(path: Path) -> list[list[tuple[str, str]]]:
    spreadsheet_namespace = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
    axes: list[list[tuple[str, str]]] = []
    with ZipFile(path) as archive:
        for sheet_index in range(2, 7):
            worksheet = ElementTree.fromstring(
                archive.read(f"xl/worksheets/sheet{sheet_index}.xml")
            )
            sheet_data = worksheet.find(f"{{{spreadsheet_namespace}}}sheetData")
            assert sheet_data is not None
            values: list[tuple[str, str]] = []
            for row in sheet_data.findall(f"{{{spreadsheet_namespace}}}row")[1:]:
                cells = row.findall(f"{{{spreadsheet_namespace}}}c")[:2]
                numeric = [
                    cell.find(f"{{{spreadsheet_namespace}}}v")
                    for cell in cells
                ]
                assert all(value is not None and value.text is not None for value in numeric)
                values.append((numeric[0].text, numeric[1].text))  # type: ignore[union-attr]
            axes.append(values)
    return axes


def test_save_session_files_writes_wide_csv_and_projected_xlsx(tmp_path) -> None:
    output_dir = tmp_path / "sessions" / "20260605_120000"
    pair_rows = [
        {
            "sample_index": 7,
            "session_elapsed_s": 1.25,
            "distance_m": 5.4,
            "left_gps_latitude_deg": 37.1,
            "left_roll_deg": 0.1,
            "baseline_m": 1.8,
            "pair_quality_status": "matched",
        },
        {
            "sample_index": 8,
            "session_elapsed_s": 1.30,
            "distance_m": 5.3,
            "left_gps_latitude_deg": 37.2,
            "left_roll_deg": 0.2,
            "baseline_m": 1.8,
            "pair_quality_status": "matched",
        },
    ]
    saved = save_session_files(
        output_dir,
        metadata={
            "session_id": "20260605_120000",
            "pair_count": 2,
            "calibration": {"profile": "night"},
        },
        pair_rows=pair_rows,
        camera_rows={
            "left": [
                {
                    "sequence": 0,
                    "session_id": "20260605_120000",
                    "side": "left",
                    "sample_type": "gui_camera_frame",
                    "sample_monotonic_ns": 1_000_000_000,
                    "source_utc_ns": 1_800_000_000_000_000_000,
                }
            ]
        },
        px4_rows={
            "right": [
                {
                    "sequence": 0,
                    "session_id": "20260605_120000",
                    "side": "right",
                    "sample_type": "pixhawk_attitude",
                    "sample_monotonic_ns": 1_100_000_000,
                    "px4_timestamp_sample_us": 900_000,
                }
            ]
        },
    )

    assert saved.outputs == [
        saved.csv,
        saved.xlsx,
        saved.camera_left_csv,
        saved.camera_right_csv,
        saved.px4_left_csv,
        saved.px4_right_csv,
    ]
    assert saved.csv == output_dir / "20260605_120000.csv"
    assert saved.xlsx == output_dir / "20260605_120000.xlsx"
    assert sorted(path.name for path in output_dir.iterdir()) == [
        ".generations",
        "20260605_120000.csv",
        "20260605_120000.xlsx",
        "camera_left.csv",
        "camera_right.csv",
        "published",
        "px4_left.csv",
        "px4_right.csv",
        "session_manifest.json",
    ]
    with saved.csv.open("r", newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 2
    assert rows[0]["sample_index"] == "7"
    assert rows[1]["session_elapsed_s"] == "1.3"
    assert set(rows[0]) == set(pair_rows[0])

    sheet_names, sheet_row_counts = _xlsx_sheet_names_and_row_counts(saved.xlsx)
    assert sheet_names == [
        "Summary",
        "Measurements",
        "GPS",
        "Attitude",
        "Baseline",
        "Quality",
        "Calibration",
    ]
    dynamic_counts = dict(zip(sheet_names, sheet_row_counts, strict=True))
    assert {dynamic_counts[name] for name in sheet_names[1:6]} == {len(pair_rows) + 1}
    axes = _xlsx_dynamic_axis_values(saved.xlsx)
    assert axes == [[("7", "1.25"), ("8", "1.3")]] * 5
    with saved.camera_left_csv.open("r", newline="", encoding="utf-8") as handle:
        camera_rows = list(csv.DictReader(handle))
    assert camera_rows[0]["sample_type"] == "gui_camera_frame"
    assert camera_rows[0]["sample_monotonic_ns"] == "1000000000"
    with saved.px4_right_csv.open("r", newline="", encoding="utf-8") as handle:
        px4_rows = list(csv.DictReader(handle))
    assert px4_rows[0]["sample_type"] == "pixhawk_attitude"
    assert px4_rows[0]["px4_timestamp_sample_us"] == "900000"
    assert json.loads(saved.manifest.read_text(encoding="utf-8"))["status"] == "finalized"


def test_save_session_files_creates_headers_for_empty_pair_rows(tmp_path) -> None:
    output_dir = tmp_path / "empty_session"

    saved = save_session_files(
        output_dir,
        metadata={"session_id": "empty_session"},
        pair_rows=[],
    )

    with saved.csv.open("r", newline="", encoding="utf-8") as handle:
        assert next(csv.reader(handle)) == ["sample_index", "session_elapsed_s"]
    for path in (
        saved.camera_left_csv,
        saved.camera_right_csv,
        saved.px4_left_csv,
        saved.px4_right_csv,
    ):
        with path.open("r", newline="", encoding="utf-8") as handle:
            assert next(csv.reader(handle))[:4] == [
                "sequence",
                "session_id",
                "side",
                "sample_type",
            ]
    sheet_names, sheet_row_counts = _xlsx_sheet_names_and_row_counts(saved.xlsx)
    dynamic_counts = dict(zip(sheet_names, sheet_row_counts, strict=True))
    assert {dynamic_counts[name] for name in sheet_names[1:6]} == {1}


def test_xlsx_failure_keeps_all_authoritative_csvs_and_retry_manifest(tmp_path, monkeypatch) -> None:
    import wiris_stereo_distance.recording_session as recording_session

    def fail_xlsx(*_args, **_kwargs):
        raise OSError("workbook locked")

    monkeypatch.setattr(recording_session, "_write_xlsx", fail_xlsx)
    saved = save_session_files(
        tmp_path / "session",
        metadata={"session_id": "session"},
        pair_rows=[{"sample_index": 0, "session_elapsed_s": 0.0}],
        camera_rows={"left": [{"sequence": 0, "side": "left"}]},
        px4_rows={"right": [{"sequence": 0, "side": "right"}]},
    )

    assert saved.finalized is False
    assert "workbook locked" in saved.error
    assert all(path.is_file() for path in (
        saved.csv, saved.camera_left_csv, saved.camera_right_csv,
        saved.px4_left_csv, saved.px4_right_csv,
    ))
    manifest = json.loads(saved.manifest.read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert manifest["retryable"] is True
    assert manifest["artifacts"]["session.xlsx"] == "failed"


def test_csv_prepare_failure_does_not_replace_any_authoritative_csv(tmp_path, monkeypatch) -> None:
    import wiris_stereo_distance.recording_session as recording_session

    output_dir = tmp_path / "session"
    output_dir.mkdir()
    names = ["session.csv", "camera_left.csv", "camera_right.csv", "px4_left.csv", "px4_right.csv"]
    for name in names:
        (output_dir / name).write_text(f"old-{name}\n", encoding="utf-8")
    original_write = recording_session.write_dynamic_csv

    def fail_third(output, rows, **kwargs):
        if output.name == "camera_right.csv":
            raise OSError("disk full")
        return original_write(output, rows, **kwargs)

    monkeypatch.setattr(recording_session, "write_dynamic_csv", fail_third)
    try:
        save_session_files(
            output_dir,
            metadata={"session_id": "session"},
            pair_rows=[{"sample_index": 0, "session_elapsed_s": 0.0}],
        )
    except OSError as exc:
        assert "disk full" in str(exc)
    else:
        raise AssertionError("expected CSV preparation failure")

    for name in names:
        assert (output_dir / name).read_text(encoding="utf-8") == f"old-{name}\n"
    manifest = json.loads((output_dir / "session_manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert manifest["phase"] == "prepare_csv"
    assert all(state == "prepared" for state in manifest["artifacts"].values())


def test_spool_rows_preserve_all_rows_with_bounded_preview(tmp_path) -> None:
    rows = SpoolRows(preview_limit=4)
    rows.bind(tmp_path / "rows.jsonl")
    for sequence in range(25):
        rows.append({"sequence": sequence})

    assert len(rows) == 25
    assert rows.preview_count == 4
    assert [row["sequence"] for row in rows] == list(range(25))
    assert rows[-1]["sequence"] == 24


def test_session_ids_do_not_collide_within_the_same_clock_tick(monkeypatch) -> None:
    import wiris_stereo_distance.recording_session as recording_session

    monkeypatch.setattr(recording_session.time, "time_ns", lambda: 1_800_000_000_123_456_789)
    first = session_id_now()
    second = session_id_now()

    assert first != second
    assert first[:15] == second[:15]


def test_publish_failure_keeps_previous_generation_visible(tmp_path, monkeypatch) -> None:
    import wiris_stereo_distance.recording_session as recording_session

    output_dir = tmp_path / "session"
    first = save_session_files(
        output_dir,
        metadata={"session_id": "session"},
        pair_rows=[{"sample_index": 0, "session_elapsed_s": 0.0, "distance_m": 1.0}],
    )
    previous_pointer = (output_dir / "published").readlink()
    previous_csv = first.csv.read_text(encoding="utf-8")
    original_publish = recording_session._publish_generation

    monkeypatch.setattr(
        recording_session,
        "_publish_generation",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(OSError("pointer fault")),
    )
    try:
        save_session_files(
            output_dir,
            metadata={"session_id": "session"},
            pair_rows=[{"sample_index": 0, "session_elapsed_s": 0.0, "distance_m": 9.0}],
        )
    except OSError as exc:
        assert "pointer fault" in str(exc)
    else:
        raise AssertionError("expected publish failure")

    assert (output_dir / "published").readlink() == previous_pointer
    assert first.csv.read_text(encoding="utf-8") == previous_csv
    manifest = json.loads(first.manifest.read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert manifest["phase"] == "publish"
    assert len(list((output_dir / ".generations").iterdir())) == 2

    monkeypatch.setattr(recording_session, "_publish_generation", original_publish)
    save_session_files(
        output_dir,
        metadata={"session_id": "session"},
        pair_rows=[{"sample_index": 0, "session_elapsed_s": 0.0, "distance_m": 2.0}],
    )
    generations = list((output_dir / ".generations").iterdir())
    assert len(generations) == 1
    assert generations[0].name == (output_dir / "published").readlink().name


def test_spool_iteration_does_not_stop_later_durable_appends(tmp_path) -> None:
    path = tmp_path / "rows.jsonl"
    rows = SpoolRows(preview_limit=3)
    rows.bind(path)
    assert list(rows) == []
    for sequence in range(12):
        rows.append({"sequence": sequence})
    rows.close()

    recovered = SpoolRows(preview_limit=3)
    recovered.open_existing(path)
    assert [row["sequence"] for row in recovered] == list(range(12))
    assert recovered.preview_count == 3


def test_retryable_session_discovery_and_pair_row_recovery(tmp_path) -> None:
    recordings_root = tmp_path / "recordings"
    output_dir = recordings_root / "sessions" / "recover-me"
    pair_rows = SpoolRows(preview_limit=2, supports_updates=True)
    pair_rows.bind(output_dir / ".spool" / "pair_rows.jsonl")
    for sequence in range(5):
        pair_rows.append({"sample_index": sequence, "distance_m": float(sequence)})
    pair_rows.close()
    write_recovery_metadata(
        output_dir,
        {"session_id": "recover-me", "session_started_unix_ns": 123},
    )
    (output_dir / "session_manifest.json").write_text(
        json.dumps({"status": "failed", "retryable": True}), encoding="utf-8"
    )

    assert discover_retryable_sessions(recordings_root) == [output_dir]
    recovered = load_retryable_session(output_dir)
    assert len(recovered.pair_rows) == 5
    assert [row["sample_index"] for row in recovered.pair_rows] == list(range(5))
    assert recovered.metadata["session_id"] == "recover-me"


def test_retry_cleans_finalize_and_preparing_orphans(tmp_path) -> None:
    output_dir = tmp_path / "session"
    (output_dir / ".finalize-old").mkdir(parents=True)
    (output_dir / ".generations" / "old.preparing").mkdir(parents=True)

    save_session_files(
        output_dir,
        metadata={"session_id": "session"},
        pair_rows=[],
    )

    assert not (output_dir / ".finalize-old").exists()
    assert not (output_dir / ".generations" / "old.preparing").exists()


def test_legacy_regular_outputs_are_migrated_to_stable_generation_links(tmp_path) -> None:
    output_dir = tmp_path / "session"
    output_dir.mkdir()
    (output_dir / "session.csv").write_text("legacy\n", encoding="utf-8")

    saved = save_session_files(
        output_dir,
        metadata={"session_id": "session"},
        pair_rows=[{"sample_index": 0, "session_elapsed_s": 0.0}],
    )

    assert saved.csv.is_symlink()
    assert saved.csv.is_file()
    assert (output_dir / "published").is_symlink()
