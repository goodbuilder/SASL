from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pyarrow as pa
import pyarrow.parquet as pq
import pytest

from Workstation.workstation_core import session_parquet
from Workstation.workstation_core.session_parquet import (
    CAMERA_TIMELINE_FILENAME,
    GEOMETRY_COMPARISON_FILENAME,
    GNSS_BASE_TIMELINE_FILENAME,
    MEASUREMENT_PAIRS_FILENAME,
    PX4_TIMELINE_FILENAME,
    ParquetFinalizationError,
    finalize_camera_timeline,
    finalize_geometry_comparison,
    finalize_gnss_base_timeline,
    finalize_measurement_pairs,
    finalize_px4_timeline,
    finalize_session_parquet_artifacts,
    read_lossless_rows,
    verify_parquet_artifact,
)


def _canonical(value: object) -> str:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )


def test_measurement_pairs_are_typed_and_dynamic_fields_round_trip_losslessly(
    tmp_path: Path,
) -> None:
    output = tmp_path / MEASUREMENT_PAIRS_FILENAME
    rows = [
        {
            "sample_index": "41",
            "recording_time_ns": 9_007_199_254_740_993,
            "measurement_attempt_id": "attempt-0041",
            "measurement_valid": "true",
            "left_cx": "101.25",
            "distance_m": 17.5,
            "baseline_length_m": 2,
            "odd_nested": {
                "unicode": "산",
                "values": [1, False, None, {"new_solver_field": 4.25}],
            },
            "odd_native_path": Path("frames/left-0041.png"),
        },
        {
            "sample_index": 42,
            "pair_id": "pair-0042",
            "measurement_valid": False,
            "warning_code": "SYNC_STALE",
            "future_field": ["preserve", {"all": "of it"}],
        },
    ]

    metadata = finalize_measurement_pairs(output, rows)

    table = pq.read_table(output)
    assert table.schema.field("sample_index").type == pa.int64()
    assert table.schema.field("recording_time_ns").type == pa.int64()
    assert table.schema.field("measurement_valid").type == pa.bool_()
    assert table.schema.field("distance_m").type == pa.float64()
    assert table.schema.field("pair_id").type == pa.string()
    assert table.schema.field("row_json").type == pa.large_string()
    assert table.schema.field("row_json").nullable is False
    assert table.column("sample_index").to_pylist() == [41, 42]
    assert table.column("pair_id").to_pylist() == ["attempt-0041", "pair-0042"]
    assert table.column("measurement_valid").to_pylist() == [True, False]
    assert table.column("left_bbox_center_x_px").to_pylist() == [101.25, None]
    assert table.column("baseline_m").to_pylist() == [2.0, None]

    expected_rows = [
        {
            **rows[0],
            "odd_native_path": "frames/left-0041.png",
        },
        rows[1],
    ]
    assert table.column("row_json").to_pylist() == [
        _canonical(row) for row in expected_rows
    ]
    assert read_lossless_rows(output, "measurement_pairs") == expected_rows

    assert metadata.path == output
    assert metadata.role == "measurement_pairs_parquet"
    assert metadata.row_count == 2
    assert metadata.size_bytes == output.stat().st_size
    assert metadata.sha256 == hashlib.sha256(output.read_bytes()).hexdigest()
    assert metadata.schema["id"] == "workstation-measurement-pairs-parquet-v1"
    assert metadata.schema["version"] == 1
    assert len(metadata.schema_sha256) == 64
    assert metadata.authoritative is False
    assert metadata.verified is True
    assert metadata.to_mapping(relative_to=tmp_path)["path"] == MEASUREMENT_PAIRS_FILENAME


def test_camera_partitions_are_combined_with_non_nullable_side(
    tmp_path: Path,
) -> None:
    output = tmp_path / CAMERA_TIMELINE_FILENAME
    metadata = finalize_camera_timeline(
        output,
        {
            "left": [
                {
                    "sequence": 1,
                    "image_width_px": "1920",
                    "odd_camera": {"rtcp": [1, 2]},
                }
            ],
            "right": [
                {
                    "side": "right",
                    "sequence": 2,
                    "image_height_px": 1080,
                    "future_timing": "kept",
                }
            ],
        },
    )

    table = pq.read_table(output)
    assert table.schema.field("side").type == pa.string()
    assert table.schema.field("side").nullable is False
    assert table.column("side").to_pylist() == ["left", "right"]
    assert table.column("image_width_px").to_pylist() == [1920, None]
    assert read_lossless_rows(output, "camera_timeline") == [
        {
            "side": "left",
            "sequence": 1,
            "image_width_px": "1920",
            "odd_camera": {"rtcp": [1, 2]},
        },
        {
            "side": "right",
            "sequence": 2,
            "image_height_px": 1080,
            "future_timing": "kept",
        },
    ]
    assert metadata.row_count == 2


def test_px4_partitions_are_combined_with_source_id_and_keep_payload(
    tmp_path: Path,
) -> None:
    output = tmp_path / PX4_TIMELINE_FILENAME
    metadata = finalize_px4_timeline(
        output,
        {
            "left": [
                {
                    "sequence": 3,
                    "sample_type": "pixhawk_attitude",
                    "roll_deg": "1.25",
                    "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                }
            ],
            "drone": [
                {
                    "source_id": "drone",
                    "sequence": 4,
                    "lat_deg": 37.1,
                    "lon_deg": 127.2,
                    "mavlink_payload": {"custom_mode": 6},
                }
            ],
        },
    )

    table = pq.read_table(output)
    assert table.schema.field("source_id").nullable is False
    assert table.column("source_id").to_pylist() == ["left", "drone"]
    assert table.column("roll_deg").to_pylist() == [1.25, None]
    assert table.column("latitude_deg").to_pylist() == [None, 37.1]
    assert table.column("longitude_deg").to_pylist() == [None, 127.2]
    recovered = read_lossless_rows(output, "pixhawk_timeline")
    assert recovered[0]["quaternion_wxyz"] == [1.0, 0.0, 0.0, 0.0]
    assert recovered[1]["mavlink_payload"] == {"custom_mode": 6}
    assert metadata.row_count == 2


def test_gnss_and_geometry_have_fixed_types_even_when_empty(
    tmp_path: Path,
) -> None:
    gnss = tmp_path / GNSS_BASE_TIMELINE_FILENAME
    geometry = tmp_path / GEOMETRY_COMPARISON_FILENAME

    gnss_metadata = finalize_gnss_base_timeline(gnss, [])
    geometry_metadata = finalize_geometry_comparison(geometry, [])

    gnss_schema = pq.read_schema(gnss)
    geometry_schema = pq.read_schema(geometry)
    assert gnss_schema.field("session_utc_ns").type == pa.int64()
    assert gnss_schema.field("latitude_deg").type == pa.float64()
    assert gnss_schema.field("fixed").type == pa.bool_()
    assert geometry_schema.field("pair_sequence").type == pa.int64()
    assert geometry_schema.field("method").type == pa.string()
    assert geometry_schema.field("range_m").type == pa.float64()
    assert gnss_metadata.row_count == 0
    assert geometry_metadata.row_count == 0
    assert verify_parquet_artifact(gnss, "gnss_base_timeline").verified is True
    assert verify_parquet_artifact(geometry, "geometry_comparison").verified is True


def test_standard_finalizer_writes_exactly_five_named_artifacts(
    tmp_path: Path,
) -> None:
    artifacts = finalize_session_parquet_artifacts(
        tmp_path,
        measurement_pairs=[{"pair_id": "p1", "distance_m": 8.0}],
        camera_rows={"left": [], "right": []},
        px4_rows={"left": [], "right": [], "drone": []},
        gnss_base_rows=[{"sequence": 1, "fixed": True}],
        geometry_comparison_rows=[
            {
                "pair_sequence": 1,
                "method": "checkerboard",
                "range_m": 8.0,
                "solver_private": {"iterations": 3},
            }
        ],
    )

    expected = {
        MEASUREMENT_PAIRS_FILENAME,
        CAMERA_TIMELINE_FILENAME,
        PX4_TIMELINE_FILENAME,
        GNSS_BASE_TIMELINE_FILENAME,
        GEOMETRY_COMPARISON_FILENAME,
    }
    assert {path.name for path in tmp_path.iterdir()} == expected
    mapping = artifacts.to_mapping(relative_to=tmp_path)
    assert set(mapping) == {
        "measurement_pairs",
        "camera_timeline",
        "pixhawk_timeline",
        "gnss_base_timeline",
        "geometry_comparison",
    }
    assert mapping["geometry_comparison"]["row_count"] == 1
    assert read_lossless_rows(
        tmp_path / GEOMETRY_COMPARISON_FILENAME,
        "geometry_comparison",
    )[0]["solver_private"] == {"iterations": 3}


def test_readback_failure_keeps_previous_destination_and_removes_temp_file(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    output = tmp_path / MEASUREMENT_PAIRS_FILENAME
    previous = b"previous-complete-generation"
    output.write_bytes(previous)

    def fail_verification(*args, **kwargs):
        raise ParquetFinalizationError("injected readback failure")

    monkeypatch.setattr(session_parquet, "_verify_table", fail_verification)

    with pytest.raises(ParquetFinalizationError, match="injected readback failure"):
        finalize_measurement_pairs(output, [{"sample_index": 1}])

    assert output.read_bytes() == previous
    assert list(tmp_path.glob(f".{output.name}.*.tmp")) == []


@pytest.mark.parametrize(
    ("writer", "rows", "message"),
    (
        (
            lambda path, value: finalize_camera_timeline(path, value),
            {"left": [{"side": "right", "sequence": 1}]},
            "side conflicts with partition key",
        ),
        (
            lambda path, value: finalize_px4_timeline(path, value),
            {"left": [{"source_id": "drone", "sequence": 1}]},
            "source_id conflicts with partition key",
        ),
        (
            lambda path, value: finalize_measurement_pairs(path, value),
            [{"sample_index": "not-an-integer"}],
            "sample_index must be an integer",
        ),
        (
            lambda path, value: finalize_measurement_pairs(path, value),
            [{"future_number": float("nan")}],
            "non-finite number",
        ),
    ),
)
def test_invalid_source_data_is_rejected_before_publication(
    tmp_path: Path,
    writer,
    rows,
    message: str,
) -> None:
    output = tmp_path / "rejected.parquet"
    with pytest.raises((TypeError, ValueError), match=message):
        writer(output, rows)
    assert not output.exists()


def test_verifier_detects_typed_column_that_disagrees_with_row_json(
    tmp_path: Path,
) -> None:
    valid_path = tmp_path / "valid.parquet"
    corrupt_path = tmp_path / "corrupt.parquet"
    finalize_geometry_comparison(
        valid_path,
        [{"pair_sequence": 7, "method": "horizon", "range_m": 22.5}],
    )
    table = pq.read_table(valid_path)
    records = table.to_pylist()
    records[0]["range_m"] = 999.0
    pq.write_table(pa.Table.from_pylist(records, schema=table.schema), corrupt_path)

    with pytest.raises(
        ParquetFinalizationError,
        match="typed column disagrees with row_json",
    ):
        verify_parquet_artifact(corrupt_path, "geometry_comparison")


def test_unknown_artifact_key_is_rejected(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="unknown Parquet artifact key"):
        verify_parquet_artifact(tmp_path / "anything.parquet", "unknown")


def test_large_iterable_is_written_and_verified_in_bounded_row_groups(
    tmp_path: Path,
) -> None:
    output = tmp_path / MEASUREMENT_PAIRS_FILENAME

    def rows():
        for index in range(session_parquet.PARQUET_BATCH_ROWS * 2 + 17):
            yield {"sample_index": index, "pair_id": f"pair-{index}"}

    metadata = finalize_measurement_pairs(output, rows())

    assert metadata.row_count == session_parquet.PARQUET_BATCH_ROWS * 2 + 17
    assert pq.ParquetFile(output).metadata.num_row_groups == 3
