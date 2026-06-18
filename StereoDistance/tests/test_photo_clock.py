from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

from wiris_stereo_distance.photo_clock import (
    compare_photo_clocks,
    compare_photo_folders,
    read_photo_clock_metadata,
)


def _ascii(text: str) -> bytes:
    return text.encode("ascii") + b"\x00"


def _rational_values(values: list[tuple[int, int]]) -> bytes:
    return b"".join(n.to_bytes(4, "little") + d.to_bytes(4, "little") for n, d in values)


def _ifd(entries: list[tuple[int, int, int, bytes]], start_offset: int) -> bytes:
    header_size = 2 + len(entries) * 12 + 4
    entry_bytes = bytearray()
    data_bytes = bytearray()
    for tag, field_type, count, value in entries:
        entry_bytes += tag.to_bytes(2, "little")
        entry_bytes += field_type.to_bytes(2, "little")
        entry_bytes += count.to_bytes(4, "little")
        if len(value) <= 4:
            entry_bytes += value.ljust(4, b"\x00")
        else:
            value_offset = start_offset + header_size + len(data_bytes)
            entry_bytes += value_offset.to_bytes(4, "little")
            data_bytes += value
    return (
        len(entries).to_bytes(2, "little")
        + bytes(entry_bytes)
        + (0).to_bytes(4, "little")
        + bytes(data_bytes)
    )


def _jpeg_with_exif(
    *,
    date_original: str,
    subsecond: str | None = None,
    gps_date: str | None = None,
    gps_time: list[tuple[int, int]] | None = None,
) -> bytes:
    ifd0_count = 2 if gps_date and gps_time else 1
    ifd0_len = 2 + ifd0_count * 12 + 4
    exif_offset = 8 + ifd0_len
    exif_entries = [(0x9003, 2, len(_ascii(date_original)), _ascii(date_original))]
    if subsecond is not None:
        exif_entries.append((0x9291, 2, len(_ascii(subsecond)), _ascii(subsecond)))
    exif = _ifd(exif_entries, exif_offset)

    gps = b""
    gps_offset = exif_offset + len(exif)
    if gps_date and gps_time:
        gps_entries = [
            (0x0001, 2, 2, _ascii("N")),
            (0x0002, 5, 3, _rational_values([(37, 1), (25, 1), (0, 1)])),
            (0x0003, 2, 2, _ascii("E")),
            (0x0004, 5, 3, _rational_values([(127, 1), (0, 1), (0, 1)])),
            (0x0007, 5, 3, _rational_values(gps_time)),
            (0x001D, 2, len(_ascii(gps_date)), _ascii(gps_date)),
        ]
        gps = _ifd(gps_entries, gps_offset)

    ifd0_entries = [(0x8769, 4, 1, exif_offset.to_bytes(4, "little"))]
    if gps:
        ifd0_entries.append((0x8825, 4, 1, gps_offset.to_bytes(4, "little")))
    ifd0 = _ifd(ifd0_entries, 8)
    tiff = b"II*\x00" + (8).to_bytes(4, "little") + ifd0 + exif + gps
    app1 = b"Exif\x00\x00" + tiff
    return b"\xff\xd8" + b"\xff\xe1" + (len(app1) + 2).to_bytes(2, "big") + app1 + b"\xff\xd9"


def test_reads_camera_exif_timestamp_with_subsecond(tmp_path: Path) -> None:
    path = tmp_path / "left.jpg"
    path.write_bytes(_jpeg_with_exif(date_original="2026:05:16 12:00:10", subsecond="123"))

    metadata = read_photo_clock_metadata(path)

    assert metadata.capture_time == datetime(2026, 5, 16, 12, 0, 10, 123000)
    assert metadata.capture_time_source == "DateTimeOriginal"
    assert metadata.has_subsecond is True


def test_prefers_gps_timestamp_and_reads_gps_coordinate(tmp_path: Path) -> None:
    path = tmp_path / "left.jpg"
    path.write_bytes(
        _jpeg_with_exif(
            date_original="2026:05:16 12:00:10",
            subsecond="123",
            gps_date="2026:05:16",
            gps_time=[(3, 1), (0, 1), (10250, 1000)],
        )
    )

    metadata = read_photo_clock_metadata(path)

    assert metadata.capture_time == datetime(2026, 5, 16, 3, 0, 10, 250000, tzinfo=timezone.utc)
    assert metadata.capture_time_source == "GPSDateStamp/GPSTimeStamp"
    assert metadata.has_subsecond is True
    assert metadata.gps_latitude == 37.416666666666664
    assert metadata.gps_longitude == 127.0


def test_compares_right_minus_left_offset(tmp_path: Path) -> None:
    left = tmp_path / "left.jpg"
    right = tmp_path / "right.jpg"
    left.write_bytes(
        _jpeg_with_exif(
            date_original="2026:05:16 12:00:10",
            gps_date="2026:05:16",
            gps_time=[(3, 1), (0, 1), (10000, 1000)],
        )
    )
    right.write_bytes(
        _jpeg_with_exif(
            date_original="2026:05:16 12:00:10",
            gps_date="2026:05:16",
            gps_time=[(3, 1), (0, 1), (10250, 1000)],
        )
    )

    result = compare_photo_clocks(left, right)

    assert result.offset_seconds == 0.25


def test_compares_matching_filenames_in_folders(tmp_path: Path) -> None:
    left_dir = tmp_path / "left"
    right_dir = tmp_path / "right"
    left_dir.mkdir()
    right_dir.mkdir()
    for directory, gps_ms in ((left_dir, 10000), (right_dir, 10100)):
        (directory / "pair001.jpg").write_bytes(
            _jpeg_with_exif(
                date_original="2026:05:16 12:00:10",
                gps_date="2026:05:16",
                gps_time=[(3, 1), (0, 1), (gps_ms, 1000)],
            )
        )

    result = compare_photo_folders(left_dir, right_dir)

    assert result.pairing_mode == "matching filenames"
    assert len(result.pairs) == 1
    assert result.median_offset_seconds == 0.1
