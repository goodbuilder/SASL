from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from statistics import mean, median
from typing import Any

EXIF_TAG = 0x8769
GPS_TAG = 0x8825

TAG_DATE_TIME = 0x0132
TAG_DATE_TIME_ORIGINAL = 0x9003
TAG_SUBSEC_TIME = 0x9290
TAG_SUBSEC_TIME_ORIGINAL = 0x9291
TAG_OFFSET_TIME_ORIGINAL = 0x9011
TAG_MAKE = 0x010F
TAG_MODEL = 0x0110

GPS_LATITUDE_REF = 0x0001
GPS_LATITUDE = 0x0002
GPS_LONGITUDE_REF = 0x0003
GPS_LONGITUDE = 0x0004
GPS_TIME_STAMP = 0x0007
GPS_DATE_STAMP = 0x001D

TIFF_TYPE_SIZES = {
    1: 1,  # BYTE
    2: 1,  # ASCII
    3: 2,  # SHORT
    4: 4,  # LONG
    5: 8,  # RATIONAL
    7: 1,  # UNDEFINED
    9: 4,  # SLONG
    10: 8,  # SRATIONAL
}

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".tif", ".tiff"}


class PhotoMetadataError(ValueError):
    pass


@dataclass(frozen=True)
class PhotoClockMetadata:
    path: Path
    capture_time: datetime | None
    capture_time_source: str
    has_subsecond: bool
    gps_latitude: float | None
    gps_longitude: float | None
    camera_make: str
    camera_model: str
    warnings: tuple[str, ...] = ()


@dataclass(frozen=True)
class ClockOffsetResult:
    left: PhotoClockMetadata
    right: PhotoClockMetadata
    offset_seconds: float | None
    warnings: tuple[str, ...] = ()


@dataclass(frozen=True)
class ClockFolderResult:
    pairs: tuple[ClockOffsetResult, ...]
    pairing_mode: str
    warnings: tuple[str, ...] = ()

    @property
    def valid_offsets(self) -> tuple[float, ...]:
        return tuple(result.offset_seconds for result in self.pairs if result.offset_seconds is not None)

    @property
    def median_offset_seconds(self) -> float | None:
        offsets = self.valid_offsets
        return None if not offsets else float(median(offsets))

    @property
    def mean_offset_seconds(self) -> float | None:
        offsets = self.valid_offsets
        return None if not offsets else float(mean(offsets))


def read_photo_clock_metadata(path: str | Path) -> PhotoClockMetadata:
    photo_path = Path(path)
    if not photo_path.exists():
        raise PhotoMetadataError(f"file does not exist: {photo_path}")
    data = photo_path.read_bytes()
    if data.startswith(b"\xff\xd8"):
        ifd0, exif_ifd, gps_ifd = _read_jpeg_exif(data)
    elif data.startswith((b"II*\x00", b"MM\x00*")):
        ifd0, exif_ifd, gps_ifd = _read_tiff_exif(data)
    else:
        raise PhotoMetadataError(f"unsupported image format: {photo_path}")

    warnings: list[str] = []
    capture_time, source, has_subsecond = _best_capture_time(ifd0, exif_ifd, gps_ifd, warnings)
    gps_latitude = _gps_coordinate(gps_ifd.get(GPS_LATITUDE), gps_ifd.get(GPS_LATITUDE_REF), warnings)
    gps_longitude = _gps_coordinate(gps_ifd.get(GPS_LONGITUDE), gps_ifd.get(GPS_LONGITUDE_REF), warnings)
    if capture_time is None:
        warnings.append("no EXIF/GPS capture timestamp")
    if source != "GPSDateStamp/GPSTimeStamp" and (gps_latitude is not None or gps_longitude is not None):
        warnings.append("GPS coordinates present but GPS timestamp missing")

    return PhotoClockMetadata(
        path=photo_path,
        capture_time=capture_time,
        capture_time_source=source,
        has_subsecond=has_subsecond,
        gps_latitude=gps_latitude,
        gps_longitude=gps_longitude,
        camera_make=str(ifd0.get(TAG_MAKE) or ""),
        camera_model=str(ifd0.get(TAG_MODEL) or ""),
        warnings=tuple(warnings),
    )


def compare_photo_clocks(left_path: str | Path, right_path: str | Path) -> ClockOffsetResult:
    left = read_photo_clock_metadata(left_path)
    right = read_photo_clock_metadata(right_path)
    warnings = [*left.warnings, *right.warnings]
    offset_seconds: float | None = None
    if left.capture_time is None or right.capture_time is None:
        warnings.append("cannot compute offset without both capture timestamps")
    elif _is_aware(left.capture_time) != _is_aware(right.capture_time):
        warnings.append("cannot compare timezone-aware GPS time with timezone-less camera time")
    else:
        left_time = _normalise_for_delta(left.capture_time)
        right_time = _normalise_for_delta(right.capture_time)
        offset_seconds = (right_time - left_time).total_seconds()
        if left.capture_time_source != right.capture_time_source:
            warnings.append(
                f"timestamp sources differ: left={left.capture_time_source}, right={right.capture_time_source}"
            )
        if not (left.has_subsecond and right.has_subsecond):
            warnings.append("one or both timestamps are second-resolution only")
    return ClockOffsetResult(left=left, right=right, offset_seconds=offset_seconds, warnings=tuple(warnings))


def compare_photo_folders(
    left_dir: str | Path,
    right_dir: str | Path,
    *,
    max_pairs: int = 200,
) -> ClockFolderResult:
    left_root = Path(left_dir)
    right_root = Path(right_dir)
    if not left_root.is_dir():
        raise PhotoMetadataError(f"left folder does not exist: {left_root}")
    if not right_root.is_dir():
        raise PhotoMetadataError(f"right folder does not exist: {right_root}")

    left_files = _image_files(left_root)
    right_files = _image_files(right_root)
    warnings: list[str] = []
    if not left_files:
        warnings.append("left folder has no supported images")
    if not right_files:
        warnings.append("right folder has no supported images")

    left_by_name = {path.name: path for path in left_files}
    right_by_name = {path.name: path for path in right_files}
    common_names = sorted(set(left_by_name) & set(right_by_name))
    if common_names:
        pair_paths = [(left_by_name[name], right_by_name[name]) for name in common_names]
        pairing_mode = "matching filenames"
    else:
        pair_paths = list(zip(left_files, right_files))
        pairing_mode = "sorted order"
        if left_files and right_files:
            warnings.append("no common filenames; paired by sorted order")

    if len(pair_paths) > max_pairs:
        pair_paths = pair_paths[:max_pairs]
        warnings.append(f"limited analysis to first {max_pairs} pairs")

    results: list[ClockOffsetResult] = []
    for left_path, right_path in pair_paths:
        try:
            results.append(compare_photo_clocks(left_path, right_path))
        except PhotoMetadataError as exc:
            empty_left = PhotoClockMetadata(left_path, None, "missing", False, None, None, "", "", (str(exc),))
            empty_right = PhotoClockMetadata(right_path, None, "missing", False, None, None, "", "", (str(exc),))
            results.append(ClockOffsetResult(empty_left, empty_right, None, (str(exc),)))
    return ClockFolderResult(pairs=tuple(results), pairing_mode=pairing_mode, warnings=tuple(warnings))


def format_offset_seconds(offset_seconds: float | None) -> str:
    if offset_seconds is None:
        return "n/a"
    direction = "right ahead" if offset_seconds > 0 else "right behind" if offset_seconds < 0 else "aligned"
    return f"{offset_seconds:+.6f} s ({direction})"


def format_photo_timestamp(metadata: PhotoClockMetadata) -> str:
    if metadata.capture_time is None:
        return "missing"
    suffix = " subsec" if metadata.has_subsecond else " sec-only"
    return f"{metadata.capture_time.isoformat()} [{metadata.capture_time_source}, {suffix}]"


def _image_files(root: Path) -> list[Path]:
    return sorted(
        path
        for path in root.rglob("*")
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    )


def _read_jpeg_exif(data: bytes) -> tuple[dict[int, Any], dict[int, Any], dict[int, Any]]:
    position = 2
    while position + 4 <= len(data):
        if data[position] != 0xFF:
            break
        marker = data[position + 1]
        position += 2
        if marker in {0xD8, 0xD9}:
            continue
        if marker == 0xDA:
            break
        if position + 2 > len(data):
            break
        segment_length = int.from_bytes(data[position : position + 2], "big")
        segment_start = position + 2
        segment_end = position + segment_length
        if segment_length < 2 or segment_end > len(data):
            break
        segment = data[segment_start:segment_end]
        if marker == 0xE1 and segment.startswith(b"Exif\x00\x00"):
            return _read_tiff_exif(segment[6:])
        position = segment_end
    raise PhotoMetadataError("EXIF APP1 segment not found")


def _read_tiff_exif(data: bytes) -> tuple[dict[int, Any], dict[int, Any], dict[int, Any]]:
    if data[:2] == b"II":
        byte_order = "little"
    elif data[:2] == b"MM":
        byte_order = "big"
    else:
        raise PhotoMetadataError("invalid TIFF byte order")
    if _u16(data, 2, byte_order) != 42:
        raise PhotoMetadataError("invalid TIFF magic")
    ifd0_offset = _u32(data, 4, byte_order)
    ifd0 = _read_ifd(data, ifd0_offset, byte_order)
    exif_ifd = _read_ifd(data, int(ifd0[EXIF_TAG]), byte_order) if EXIF_TAG in ifd0 else {}
    gps_ifd = _read_ifd(data, int(ifd0[GPS_TAG]), byte_order) if GPS_TAG in ifd0 else {}
    return ifd0, exif_ifd, gps_ifd


def _read_ifd(data: bytes, offset: int, byte_order: str) -> dict[int, Any]:
    if offset <= 0 or offset + 2 > len(data):
        return {}
    count = _u16(data, offset, byte_order)
    entries: dict[int, Any] = {}
    for index in range(count):
        entry_offset = offset + 2 + index * 12
        if entry_offset + 12 > len(data):
            break
        tag = _u16(data, entry_offset, byte_order)
        field_type = _u16(data, entry_offset + 2, byte_order)
        item_count = _u32(data, entry_offset + 4, byte_order)
        type_size = TIFF_TYPE_SIZES.get(field_type)
        if type_size is None:
            continue
        value_size = item_count * type_size
        value_field = data[entry_offset + 8 : entry_offset + 12]
        if value_size <= 4:
            raw_value = value_field[:value_size]
        else:
            value_offset = int.from_bytes(value_field, byte_order, signed=False)
            if value_offset < 0 or value_offset + value_size > len(data):
                continue
            raw_value = data[value_offset : value_offset + value_size]
        entries[tag] = _decode_tiff_value(raw_value, field_type, item_count, byte_order)
    return entries


def _decode_tiff_value(raw_value: bytes, field_type: int, item_count: int, byte_order: str) -> Any:
    if field_type == 2:
        return raw_value.rstrip(b"\x00").decode("utf-8", errors="replace").strip()
    if field_type == 3:
        values = tuple(_u16(raw_value, index * 2, byte_order) for index in range(item_count))
    elif field_type == 4:
        values = tuple(_u32(raw_value, index * 4, byte_order) for index in range(item_count))
    elif field_type == 5:
        values = tuple(
            (_u32(raw_value, index * 8, byte_order), _u32(raw_value, index * 8 + 4, byte_order))
            for index in range(item_count)
        )
    elif field_type == 9:
        values = tuple(_s32(raw_value, index * 4, byte_order) for index in range(item_count))
    elif field_type == 10:
        values = tuple(
            (_s32(raw_value, index * 8, byte_order), _s32(raw_value, index * 8 + 4, byte_order))
            for index in range(item_count)
        )
    else:
        values = tuple(raw_value)
    return values[0] if len(values) == 1 else values


def _best_capture_time(
    ifd0: dict[int, Any],
    exif_ifd: dict[int, Any],
    gps_ifd: dict[int, Any],
    warnings: list[str],
) -> tuple[datetime | None, str, bool]:
    gps_time = _gps_datetime(gps_ifd.get(GPS_DATE_STAMP), gps_ifd.get(GPS_TIME_STAMP), warnings)
    if gps_time is not None:
        return gps_time, "GPSDateStamp/GPSTimeStamp", _datetime_has_subseconds(gps_time)

    for tag, subsec_tag, source in (
        (TAG_DATE_TIME_ORIGINAL, TAG_SUBSEC_TIME_ORIGINAL, "DateTimeOriginal"),
        (TAG_DATE_TIME, TAG_SUBSEC_TIME, "DateTime"),
    ):
        value = exif_ifd.get(tag) if tag in exif_ifd else ifd0.get(tag)
        if value:
            subsec = exif_ifd.get(subsec_tag) if subsec_tag in exif_ifd else ifd0.get(subsec_tag)
            offset = exif_ifd.get(TAG_OFFSET_TIME_ORIGINAL)
            parsed = _camera_datetime(str(value), None if subsec is None else str(subsec), None if offset is None else str(offset))
            if parsed is not None:
                return parsed, source, _datetime_has_subseconds(parsed)
            warnings.append(f"cannot parse {source}: {value}")
    return None, "missing", False


def _gps_datetime(date_value: Any, time_value: Any, warnings: list[str]) -> datetime | None:
    if not date_value or not time_value:
        return None
    try:
        date_part = datetime.strptime(str(date_value), "%Y:%m:%d")
        time_parts = _rational_values(time_value)
        if len(time_parts) < 3:
            raise ValueError("GPS timestamp needs hour, minute, second")
        hour = int(time_parts[0])
        minute = int(time_parts[1])
        second_float = float(time_parts[2])
        second = int(second_float)
        microsecond = int(round((second_float - second) * 1_000_000))
        if microsecond >= 1_000_000:
            second += 1
            microsecond -= 1_000_000
        return datetime(
            date_part.year,
            date_part.month,
            date_part.day,
            hour,
            minute,
            second,
            microsecond,
            tzinfo=timezone.utc,
        )
    except Exception as exc:
        warnings.append(f"cannot parse GPS timestamp: {exc}")
        return None


def _camera_datetime(value: str, subsecond: str | None, offset: str | None) -> datetime | None:
    try:
        parsed = datetime.strptime(value.strip(), "%Y:%m:%d %H:%M:%S")
    except ValueError:
        return None
    digits = "".join(char for char in (subsecond or "") if char.isdigit())
    if digits:
        parsed = parsed.replace(microsecond=int((digits[:6]).ljust(6, "0")))
    if offset:
        try:
            sign = 1 if offset[0] == "+" else -1
            hours = int(offset[1:3])
            minutes = int(offset[4:6])
            parsed = parsed.replace(tzinfo=timezone(sign * timedelta(hours=hours, minutes=minutes)))
        except Exception:
            pass
    return parsed


def _gps_coordinate(value: Any, reference: Any, warnings: list[str]) -> float | None:
    if value is None or reference is None:
        return None
    try:
        degrees, minutes, seconds = _rational_values(value)[:3]
        coordinate = float(degrees) + float(minutes) / 60.0 + float(seconds) / 3600.0
        ref = str(reference).upper()
        if ref in {"S", "W"}:
            coordinate *= -1.0
        return coordinate
    except Exception as exc:
        warnings.append(f"cannot parse GPS coordinate: {exc}")
        return None


def _rational_values(value: Any) -> tuple[float, ...]:
    values = value if isinstance(value, tuple) and value and isinstance(value[0], tuple) else (value,)
    parsed: list[float] = []
    for numerator, denominator in values:
        parsed.append(0.0 if denominator == 0 else float(numerator) / float(denominator))
    return tuple(parsed)


def _datetime_has_subseconds(value: datetime) -> bool:
    return value.microsecond != 0


def _is_aware(value: datetime) -> bool:
    return value.tzinfo is not None and value.utcoffset() is not None


def _normalise_for_delta(value: datetime) -> datetime:
    if _is_aware(value):
        return value.astimezone(timezone.utc).replace(tzinfo=None)
    return value


def _u16(data: bytes, offset: int, byte_order: str) -> int:
    return int.from_bytes(data[offset : offset + 2], byte_order, signed=False)


def _u32(data: bytes, offset: int, byte_order: str) -> int:
    return int.from_bytes(data[offset : offset + 4], byte_order, signed=False)


def _s32(data: bytes, offset: int, byte_order: str) -> int:
    return int.from_bytes(data[offset : offset + 4], byte_order, signed=True)
