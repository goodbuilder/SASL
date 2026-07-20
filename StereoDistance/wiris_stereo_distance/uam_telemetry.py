from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable, Sequence

from sasl_common.truth_coordinates import (
    GLOBAL_LLA_AXIS_CONTRACT,
    GLOBAL_LLA_FRAME,
    LOCAL_NED_AXIS_CONTRACT,
    LOCAL_NED_FRAME,
    local_ned_from_lla,
    local_ned_from_xyz,
    scaled_altitude_m,
    scaled_degrees,
)
from wiris_stereo_distance.coordinate_contract import (
    FRD_COMPARISON_AXIS_CONTRACT,
    MEASUREMENT_AXIS_CONTRACT,
    MEASUREMENT_FRAME,
    MIXED_FRAME,
    TRUTH_AXIS_CONTRACT,
    TRUTH_FRAME,
)

TRUTH_SOURCE_AUTO = "auto"
TRUTH_SOURCE_LOCAL_NED = "local_ned"
TRUTH_SOURCE_GLOBAL_POSITION = "global_position"
TRUTH_SOURCE_GPS_POSITION = "gps_position"
TRUTH_SOURCE_TYPES = (
    TRUTH_SOURCE_AUTO,
    TRUTH_SOURCE_LOCAL_NED,
    TRUTH_SOURCE_GLOBAL_POSITION,
    TRUTH_SOURCE_GPS_POSITION,
)


@dataclass(frozen=True)
class UamTelemetrySample:
    timestamp_s: float
    x_m: float
    y_m: float
    z_m: float
    source: str
    timestamp_reference: str = "px4-boot"
    utc_unix_s: float | None = None
    gps_reference_lat_deg: float | None = None
    gps_reference_lon_deg: float | None = None
    gps_reference_alt_m: float | None = None
    gps_reference_kind: str = ""
    gps_altitude_reference: str = ""
    source_frame: str = LOCAL_NED_FRAME
    source_axis_contract: str = LOCAL_NED_AXIS_CONTRACT
    raw_lat_deg: float | None = None
    raw_lon_deg: float | None = None
    raw_alt_m: float | None = None
    conversion_status: str = "ok"

    @property
    def distance_m(self) -> float:
        return math.sqrt(self.x_m * self.x_m + self.y_m * self.y_m + self.z_m * self.z_m)


class ULogTelemetryError(RuntimeError):
    pass


class UamAlignmentError(RuntimeError):
    pass


@dataclass(frozen=True)
class _UlogUtcMapper:
    offset_s: float
    source: str
    sample_count: int

    def utc_unix_s(self, timestamp_s: float) -> float:
        return float(timestamp_s) + self.offset_s


@dataclass(frozen=True)
class UamAlignmentProfile:
    profile_id: str
    rotation_matrix: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
    origin_offset_m: tuple[float, float, float]
    sample_count: int
    rmse_m: float
    max_error_m: float
    created_unix_s: float
    method: str = "rigid-horn-v1"
    measurement_frame: str = MEASUREMENT_FRAME
    truth_frame: str = TRUTH_FRAME
    truth_source_type: str = ""
    truth_source_frame: str = ""
    truth_axis_contract: str = ""
    truth_time_basis: str = ""
    truth_origin_kind: str = ""

    def apply(self, point_m: tuple[float, float, float]) -> tuple[float, float, float]:
        rotated = _mat_vec(self.rotation_matrix, point_m)
        return _add3(rotated, self.origin_offset_m)

    def inverse_apply(self, point_m: tuple[float, float, float]) -> tuple[float, float, float]:
        centered = _sub3(point_m, self.origin_offset_m)
        return _mat_vec(_transpose3(self.rotation_matrix), centered)

    def to_dict(self) -> dict[str, object]:
        return {
            "profile_id": self.profile_id,
            "method": self.method,
            "measurement_frame": self.measurement_frame,
            "truth_frame": self.truth_frame,
            "rotation_matrix": [list(row) for row in self.rotation_matrix],
            "origin_offset_m": list(self.origin_offset_m),
            "sample_count": self.sample_count,
            "rmse_m": self.rmse_m,
            "max_error_m": self.max_error_m,
            "created_unix_s": self.created_unix_s,
            "truth_source_type": self.truth_source_type,
            "truth_source_frame": self.truth_source_frame,
            "truth_axis_contract": self.truth_axis_contract,
            "truth_time_basis": self.truth_time_basis,
            "truth_origin_kind": self.truth_origin_kind,
        }

    @classmethod
    def from_dict(cls, payload: dict[str, object]) -> "UamAlignmentProfile":
        matrix_raw = payload["rotation_matrix"]
        offset_raw = payload["origin_offset_m"]
        if not isinstance(matrix_raw, Sequence) or len(matrix_raw) != 3:
            raise ValueError("rotation_matrix must be 3x3")
        rows: list[tuple[float, float, float]] = []
        for row in matrix_raw:
            if not isinstance(row, Sequence) or len(row) != 3:
                raise ValueError("rotation_matrix must be 3x3")
            rows.append((float(row[0]), float(row[1]), float(row[2])))
        if not isinstance(offset_raw, Sequence) or len(offset_raw) != 3:
            raise ValueError("origin_offset_m must have three values")
        return cls(
            profile_id=str(payload["profile_id"]),
            method=str(payload.get("method") or "rigid-horn-v1"),
            measurement_frame=str(payload.get("measurement_frame") or MEASUREMENT_FRAME),
            truth_frame=str(payload.get("truth_frame") or TRUTH_FRAME),
            rotation_matrix=(rows[0], rows[1], rows[2]),
            origin_offset_m=(float(offset_raw[0]), float(offset_raw[1]), float(offset_raw[2])),
            sample_count=int(payload.get("sample_count") or 0),
            rmse_m=float(payload.get("rmse_m") or 0.0),
            max_error_m=float(payload.get("max_error_m") or 0.0),
            created_unix_s=float(payload.get("created_unix_s") or 0.0),
            truth_source_type=str(payload.get("truth_source_type") or ""),
            truth_source_frame=str(payload.get("truth_source_frame") or ""),
            truth_axis_contract=str(payload.get("truth_axis_contract") or ""),
            truth_time_basis=str(payload.get("truth_time_basis") or ""),
            truth_origin_kind=str(payload.get("truth_origin_kind") or ""),
        )


def truth_source_type_from_sample(sample: UamTelemetrySample) -> str:
    source = (sample.source or "").lower()
    if "vehicle_global_position" in source or "mavlink_global" in source:
        return TRUTH_SOURCE_GLOBAL_POSITION
    if "vehicle_gps_position" in source or "sensor_gps" in source or "mavlink_gps" in source:
        return TRUTH_SOURCE_GPS_POSITION
    if sample.source_frame == GLOBAL_LLA_FRAME:
        return TRUTH_SOURCE_GPS_POSITION
    return TRUTH_SOURCE_LOCAL_NED


def _normalize_truth_source_type(value: str | None) -> str:
    normalized = (value or TRUTH_SOURCE_AUTO).strip().lower().replace("-", "_")
    aliases = {
        "auto": TRUTH_SOURCE_AUTO,
        "local": TRUTH_SOURCE_LOCAL_NED,
        "local_ned": TRUTH_SOURCE_LOCAL_NED,
        "global": TRUTH_SOURCE_GLOBAL_POSITION,
        "global_position": TRUTH_SOURCE_GLOBAL_POSITION,
        "gps": TRUTH_SOURCE_GPS_POSITION,
        "gps_position": TRUTH_SOURCE_GPS_POSITION,
    }
    result = aliases.get(normalized)
    if result is None:
        raise ValueError(f"unsupported truth_source_type: {value}")
    return result


def _coerce_float(value: object) -> float | None:
    if value is None:
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _add3(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub3(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _norm3(v: tuple[float, float, float]) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _cross3(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _centroid(points: Sequence[tuple[float, float, float]]) -> tuple[float, float, float]:
    count = float(len(points))
    return (
        sum(point[0] for point in points) / count,
        sum(point[1] for point in points) / count,
        sum(point[2] for point in points) / count,
    )


def _mat_vec(
    matrix: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )


def _transpose3(
    matrix: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    return (
        (matrix[0][0], matrix[1][0], matrix[2][0]),
        (matrix[0][1], matrix[1][1], matrix[2][1]),
        (matrix[0][2], matrix[1][2], matrix[2][2]),
    )


def _quat_to_matrix(q: Sequence[float]) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    w, x, y, z = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0.0:
        raise UamAlignmentError("invalid zero-length alignment quaternion")
    w, x, y, z = (w / norm, x / norm, y / norm, z / norm)
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
        (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
        (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
    )


def _largest_eigenvector_symmetric(matrix: Sequence[Sequence[float]]) -> tuple[float, float, float, float]:
    size = 4
    a = [[float(matrix[row][col]) for col in range(size)] for row in range(size)]
    vectors = [[1.0 if row == col else 0.0 for col in range(size)] for row in range(size)]
    for _ in range(80):
        p, q = 0, 1
        max_offdiag = abs(a[p][q])
        for row in range(size):
            for col in range(row + 1, size):
                value = abs(a[row][col])
                if value > max_offdiag:
                    max_offdiag = value
                    p, q = row, col
        if max_offdiag < 1e-12:
            break
        if abs(a[p][q]) < 1e-30:
            continue
        tau = (a[q][q] - a[p][p]) / (2.0 * a[p][q])
        sign = 1.0 if tau >= 0.0 else -1.0
        tangent = sign / (abs(tau) + math.sqrt(1.0 + tau * tau))
        cosine = 1.0 / math.sqrt(1.0 + tangent * tangent)
        sine = tangent * cosine
        app = a[p][p]
        aqq = a[q][q]
        apq = a[p][q]
        a[p][p] = app - tangent * apq
        a[q][q] = aqq + tangent * apq
        a[p][q] = 0.0
        a[q][p] = 0.0
        for index in range(size):
            if index in (p, q):
                continue
            aip = a[index][p]
            aiq = a[index][q]
            a[index][p] = cosine * aip - sine * aiq
            a[p][index] = a[index][p]
            a[index][q] = sine * aip + cosine * aiq
            a[q][index] = a[index][q]
        for index in range(size):
            vip = vectors[index][p]
            viq = vectors[index][q]
            vectors[index][p] = cosine * vip - sine * viq
            vectors[index][q] = sine * vip + cosine * viq
    best_index = max(range(size), key=lambda index: a[index][index])
    eigenvector = tuple(vectors[row][best_index] for row in range(size))
    norm = math.sqrt(sum(value * value for value in eigenvector))
    if norm <= 0.0:
        raise UamAlignmentError("failed to estimate alignment rotation")
    normalized = tuple(value / norm for value in eigenvector)
    if normalized[0] < 0.0:
        normalized = tuple(-value for value in normalized)
    return normalized  # type: ignore[return-value]


def _has_non_collinear_geometry(points: Sequence[tuple[float, float, float]], *, tolerance: float = 1e-6) -> bool:
    if len(points) < 3:
        return False
    origin = points[0]
    vectors = [_sub3(point, origin) for point in points[1:]]
    for first_index, first in enumerate(vectors):
        if _norm3(first) <= tolerance:
            continue
        for second in vectors[first_index + 1 :]:
            if _norm3(_cross3(first, second)) > tolerance:
                return True
    return False


def _first_float_field(row: dict[str, object], names: Sequence[str]) -> float | None:
    for name in names:
        value = _coerce_float(row.get(name))
        if value is not None:
            return value
    return None


def _timestamp_to_seconds(value: object) -> float | None:
    timestamp = _coerce_float(value)
    if timestamp is None:
        return None
    if timestamp > 1_000_000.0:
        return timestamp / 1_000_000.0
    return timestamp


def _field_values(data: dict[str, object], name: str) -> Sequence[object] | None:
    values = data.get(name)
    if values is None:
        return None
    if hasattr(values, "tolist"):
        values = values.tolist()
    if isinstance(values, Sequence) and not isinstance(values, (str, bytes, bytearray)):
        return values
    return None


def _median(values: Sequence[float]) -> float:
    ordered = sorted(values)
    midpoint = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[midpoint]
    return (ordered[midpoint - 1] + ordered[midpoint]) * 0.5


def _timestamp_sample_to_seconds(value: object) -> float | None:
    timestamp_s = _timestamp_to_seconds(value)
    if timestamp_s is None or timestamp_s <= 0.0:
        return None
    return timestamp_s


def _scaled_degrees(value: object) -> float | None:
    return scaled_degrees(value)


def _scaled_altitude_m(value: object) -> float | None:
    return scaled_altitude_m(value)


def _gps_ned_delta_m(
    *,
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    reference_lat_deg: float,
    reference_lon_deg: float,
    reference_alt_m: float,
) -> tuple[float, float, float]:
    converted = local_ned_from_lla(
        lat=lat_deg,
        lon=lon_deg,
        alt=alt_m,
        reference_lat_deg=reference_lat_deg,
        reference_lon_deg=reference_lon_deg,
        reference_alt_m=reference_alt_m,
        source="gps_lla_to_ned",
        origin_kind="compat",
    )
    if converted is None:
        return math.nan, math.nan, math.nan
    return converted.north_m, converted.east_m, converted.down_m


def _row_timestamp_s(data: dict[str, object], index: int) -> tuple[float | None, str]:
    sample_timestamps = _field_values(data, "timestamp_sample")
    if sample_timestamps is not None and index < len(sample_timestamps):
        sample_timestamp_s = _timestamp_sample_to_seconds(sample_timestamps[index])
        if sample_timestamp_s is not None:
            return sample_timestamp_s, "px4-timestamp-sample"
    timestamps = _field_values(data, "timestamp")
    if timestamps is not None and index < len(timestamps):
        timestamp_s = _timestamp_to_seconds(timestamps[index])
        if timestamp_s is not None:
            return timestamp_s, "px4-boot"
    return None, "px4-boot"


def _build_ulog_utc_mapper(datasets: dict[str, object]) -> _UlogUtcMapper | None:
    for topic_name in ("vehicle_global_position", "vehicle_gps_position", "sensor_gps"):
        dataset = datasets.get(topic_name)
        data = getattr(dataset, "data", {}) if dataset is not None else {}
        timestamp_values = _field_values(data, "timestamp")
        utc_values = _field_values(data, "time_utc_usec")
        if timestamp_values is None or utc_values is None:
            continue
        count = min(len(timestamp_values), len(utc_values))
        offsets: list[float] = []
        for index in range(count):
            timestamp_s, _reference = _row_timestamp_s(data, index)
            utc_usec = _coerce_float(utc_values[index])
            if timestamp_s is None or utc_usec is None or utc_usec <= 0.0:
                continue
            utc_unix_s = utc_usec / 1_000_000.0
            if utc_unix_s < 946_684_800.0:
                continue
            offsets.append(utc_unix_s - timestamp_s)
        if offsets:
            return _UlogUtcMapper(
                offset_s=_median(offsets),
                source=f"{topic_name}.time_utc_usec",
                sample_count=len(offsets),
            )
    return None


def _first_valid_gps_reference(
    data: dict[str, object],
) -> tuple[float, float, float] | None:
    lat_values = _field_values(data, "lat")
    lon_values = _field_values(data, "lon")
    alt_values = _field_values(data, "alt")
    if lat_values is None or lon_values is None or alt_values is None:
        return None
    count = min(len(lat_values), len(lon_values), len(alt_values))
    for index in range(count):
        lat_deg = _scaled_degrees(lat_values[index])
        lon_deg = _scaled_degrees(lon_values[index])
        alt_m = _scaled_altitude_m(alt_values[index])
        if None not in (lat_deg, lon_deg, alt_m):
            return float(lat_deg), float(lon_deg), float(alt_m)
    return None


def _first_scaled_field_value(
    data: dict[str, object],
    name: str,
    scaler: Callable[[object], float | None],
) -> float | None:
    values = _field_values(data, name)
    if values is None:
        return scaler(data.get(name))
    for value in values:
        parsed = scaler(value)
        if parsed is not None:
            return float(parsed)
    return None


def _ulog_gps_reference(
    datasets: dict[str, object],
    gps_data: dict[str, object],
    *,
    first_sample_origin_kind: str,
) -> tuple[float, float, float, str, str] | None:
    local_position = datasets.get("vehicle_local_position")
    local_data = getattr(local_position, "data", {}) if local_position is not None else {}
    ref_lat = _first_scaled_field_value(local_data, "ref_lat", _scaled_degrees)
    ref_lon = _first_scaled_field_value(local_data, "ref_lon", _scaled_degrees)
    ref_alt = _first_scaled_field_value(local_data, "ref_alt", _scaled_altitude_m)
    if None not in (ref_lat, ref_lon, ref_alt):
        return float(ref_lat), float(ref_lon), float(ref_alt), "vehicle_local_position_ref", "unknown"

    home_position = datasets.get("home_position")
    home_data = getattr(home_position, "data", {}) if home_position is not None else {}
    home_ref = _first_valid_gps_reference(home_data)
    if home_ref is not None:
        return home_ref[0], home_ref[1], home_ref[2], "home_position", "unknown"

    first_gps_ref = _first_valid_gps_reference(gps_data)
    if first_gps_ref is not None:
        return first_gps_ref[0], first_gps_ref[1], first_gps_ref[2], first_sample_origin_kind, "unknown"
    return None


def _load_ulog_gps_fallback_samples(
    datasets: dict[str, object],
    *,
    utc_mapper: _UlogUtcMapper | None,
    topic_names: Sequence[str] = ("vehicle_global_position", "vehicle_gps_position", "sensor_gps"),
) -> tuple[list[UamTelemetrySample], str]:
    for topic_name in topic_names:
        dataset = datasets.get(topic_name)
        data = getattr(dataset, "data", {}) if dataset is not None else {}
        timestamps = _field_values(data, "timestamp")
        lat_values = _field_values(data, "lat")
        lon_values = _field_values(data, "lon")
        alt_values = _field_values(data, "alt")
        if timestamps is None or lat_values is None or lon_values is None or alt_values is None:
            continue
        first_sample_origin_kind = (
            "first_valid_global_sample" if topic_name == "vehicle_global_position" else "first_valid_gps_sample"
        )
        reference = _ulog_gps_reference(
            datasets,
            data,
            first_sample_origin_kind=first_sample_origin_kind,
        )
        if reference is None:
            continue
        ref_lat, ref_lon, ref_alt, ref_kind, altitude_reference = reference
        count = min(len(timestamps), len(lat_values), len(lon_values), len(alt_values))
        samples: list[UamTelemetrySample] = []
        for index in range(count):
            timestamp_s, timestamp_reference = _row_timestamp_s(data, index)
            lat_deg = _scaled_degrees(lat_values[index])
            lon_deg = _scaled_degrees(lon_values[index])
            alt_m = _scaled_altitude_m(alt_values[index])
            if None in (timestamp_s, lat_deg, lon_deg, alt_m):
                continue
            converted = local_ned_from_lla(
                lat=lat_deg,
                lon=lon_deg,
                alt=alt_m,
                reference_lat_deg=ref_lat,
                reference_lon_deg=ref_lon,
                reference_alt_m=ref_alt,
                source=f"ulog:{topic_name}:gps_lla_to_ned:{ref_kind}",
                origin_kind=ref_kind,
                altitude_reference=altitude_reference,
            )
            if converted is None:
                continue
            samples.append(
                UamTelemetrySample(
                    timestamp_s=float(timestamp_s),
                    x_m=converted.north_m,
                    y_m=converted.east_m,
                    z_m=converted.down_m,
                    source=converted.source,
                    timestamp_reference=timestamp_reference,
                    utc_unix_s=None if utc_mapper is None else utc_mapper.utc_unix_s(float(timestamp_s)),
                    gps_reference_lat_deg=converted.origin_lat_deg,
                    gps_reference_lon_deg=converted.origin_lon_deg,
                    gps_reference_alt_m=converted.origin_alt_m,
                    gps_reference_kind=converted.origin_kind,
                    gps_altitude_reference=converted.altitude_reference,
                    source_frame=converted.source_frame,
                    source_axis_contract=converted.source_axis_contract,
                    raw_lat_deg=converted.raw_lat_deg,
                    raw_lon_deg=converted.raw_lon_deg,
                    raw_alt_m=converted.raw_alt_m,
                    conversion_status=converted.conversion_status,
                )
            )
        samples.sort(key=lambda sample: sample.timestamp_s)
        if samples:
            return samples, f"{topic_name}:gps_lla_to_ned:{ref_kind}"
    return [], ""


def load_ulog_local_position_samples(
    path: str | Path,
    *,
    truth_source_type: str = TRUTH_SOURCE_AUTO,
) -> tuple[list[UamTelemetrySample], str]:
    selected_truth_source_type = _normalize_truth_source_type(truth_source_type)
    try:
        from pyulog import ULog
    except Exception as exc:  # pragma: no cover - environment dependent.
        raise ULogTelemetryError(f"pyulog를 사용할 수 없습니다: {exc}") from exc

    ulog_path = Path(path).expanduser()
    try:
        ulog = ULog(str(ulog_path))
    except Exception as exc:
        raise ULogTelemetryError(f"ULog 파일을 열 수 없습니다: {ulog_path} ({exc})") from exc

    topic_priority = (
        "vehicle_local_position",
        "vehicle_odometry",
        "vehicle_visual_odometry",
    )
    datasets = {
        getattr(dataset, "name", ""): dataset
        for dataset in getattr(ulog, "data_list", [])
    }
    selected_name = next((name for name in topic_priority if name in datasets), "")
    utc_mapper = _build_ulog_utc_mapper(datasets)

    if selected_truth_source_type == TRUTH_SOURCE_GLOBAL_POSITION:
        samples, gps_topic_name = _load_ulog_gps_fallback_samples(
            datasets,
            utc_mapper=utc_mapper,
            topic_names=("vehicle_global_position",),
        )
        if samples:
            return samples, gps_topic_name
        raise ULogTelemetryError("ULog에서 vehicle_global_position truth source를 찾지 못했습니다.")

    if selected_truth_source_type == TRUTH_SOURCE_GPS_POSITION:
        samples, gps_topic_name = _load_ulog_gps_fallback_samples(
            datasets,
            utc_mapper=utc_mapper,
            topic_names=("vehicle_gps_position", "sensor_gps"),
        )
        if samples:
            return samples, gps_topic_name
        raise ULogTelemetryError("ULog에서 vehicle_gps_position 또는 sensor_gps truth source를 찾지 못했습니다.")

    if not selected_name:
        if selected_truth_source_type == TRUTH_SOURCE_LOCAL_NED:
            available = ", ".join(sorted(name for name in datasets if name))
            raise ULogTelemetryError(
                "ULog에서 selected local_ned truth source를 찾지 못했습니다. "
                f"필요 topic={', '.join(topic_priority)}; available={available or 'none'}"
            )
        samples, gps_topic_name = _load_ulog_gps_fallback_samples(datasets, utc_mapper=utc_mapper)
        if samples:
            return samples, gps_topic_name
        available = ", ".join(sorted(name for name in datasets if name))
        raise ULogTelemetryError(
            "ULog에서 local position topic 또는 GPS fallback topic을 찾지 못했습니다. "
            f"필요 local topic={', '.join(topic_priority)}; GPS topic=vehicle_global_position/vehicle_gps_position/sensor_gps; "
            f"available={available or 'none'}"
        )

    data = getattr(datasets[selected_name], "data", {})
    timestamps = _field_values(data, "timestamp")
    x_values = _field_values(data, "x")
    y_values = _field_values(data, "y")
    z_values = _field_values(data, "z")
    if timestamps is None or x_values is None or y_values is None or z_values is None:
        if selected_truth_source_type == TRUTH_SOURCE_LOCAL_NED:
            raise ULogTelemetryError(f"{selected_name}에 timestamp/x/y/z 필드가 없습니다.")
        samples, gps_topic_name = _load_ulog_gps_fallback_samples(datasets, utc_mapper=utc_mapper)
        if samples:
            return samples, gps_topic_name
        raise ULogTelemetryError(f"{selected_name}에 timestamp/x/y/z 필드가 없고 GPS fallback도 사용할 수 없습니다.")

    count = min(len(timestamps), len(x_values), len(y_values), len(z_values))
    samples: list[UamTelemetrySample] = []
    for index in range(count):
        timestamp_s, timestamp_reference = _row_timestamp_s(data, index)
        x_m = _coerce_float(x_values[index])
        y_m = _coerce_float(y_values[index])
        z_m = _coerce_float(z_values[index])
        if None in (timestamp_s, x_m, y_m, z_m):
            continue
        converted = local_ned_from_xyz(x_m, y_m, z_m, source=f"ulog:{selected_name}")
        if converted is None:
            continue
        samples.append(
            UamTelemetrySample(
                timestamp_s=float(timestamp_s),
                x_m=converted.north_m,
                y_m=converted.east_m,
                z_m=converted.down_m,
                source=converted.source,
                timestamp_reference=timestamp_reference,
                utc_unix_s=None if utc_mapper is None else utc_mapper.utc_unix_s(float(timestamp_s)),
                source_frame=converted.source_frame,
                source_axis_contract=converted.source_axis_contract,
            )
        )
    samples.sort(key=lambda sample: sample.timestamp_s)
    if not samples:
        raise ULogTelemetryError(f"{selected_name}에서 유효한 timestamp/x/y/z 샘플을 찾지 못했습니다.")
    return samples, selected_name


def _measurement_time_s(row: dict[str, object]) -> float | None:
    for field in ("bbox_mid_timestamp_s", "timestamp_s"):
        parsed = _coerce_float(row.get(field))
        if parsed is not None:
            return parsed
    return None


def _measurement_utc_unix_s(row: dict[str, object]) -> float | None:
    parsed = _coerce_float(row.get("bbox_mid_utc_unix_s"))
    if parsed is not None:
        return parsed
    left_utc_s = _coerce_float(row.get("left_bbox_utc_unix_s"))
    right_utc_s = _coerce_float(row.get("right_bbox_utc_unix_s"))
    if left_utc_s is not None and right_utc_s is not None:
        return (left_utc_s + right_utc_s) * 0.5
    return left_utc_s if left_utc_s is not None else right_utc_s


def _row_measurement(
    row: dict[str, object],
) -> tuple[tuple[float, float, float], str, str, str] | None:
    explicit_frame = str(row.get("measurement_frame") or "").strip()
    explicit_axis = str(row.get("measurement_axis_contract") or "").strip()
    field_sets = (
        (
            ("raw_right_m", "raw_down_m", "raw_forward_m"),
            MEASUREMENT_FRAME,
            MEASUREMENT_AXIS_CONTRACT,
            "raw-stereo-center",
        ),
        (
            ("corrected_north_m", "corrected_east_m", "corrected_down_m"),
            TRUTH_FRAME,
            TRUTH_AXIS_CONTRACT,
            "truth-frame-corrected",
        ),
        (
            ("measurement_north_m", "measurement_east_m", "measurement_down_m"),
            TRUTH_FRAME,
            TRUTH_AXIS_CONTRACT,
            "truth-frame-measurement",
        ),
        (
            ("raw_stereo_center_right_m", "raw_stereo_center_down_m", "raw_stereo_center_forward_m"),
            MEASUREMENT_FRAME,
            MEASUREMENT_AXIS_CONTRACT,
            "raw-stereo-center",
        ),
        (
            ("raw_x_m", "raw_y_m", "raw_z_m"),
            MEASUREMENT_FRAME,
            MEASUREMENT_AXIS_CONTRACT,
            "raw-stereo-center",
        ),
        (
            ("stereo_center_right_m", "stereo_center_down_m", "stereo_center_forward_m"),
            MEASUREMENT_FRAME,
            MEASUREMENT_AXIS_CONTRACT,
            "stereo-center",
        ),
        (
            ("corrected_x_m", "corrected_y_m", "corrected_z_m"),
            explicit_frame or MEASUREMENT_FRAME,
            explicit_axis or MEASUREMENT_AXIS_CONTRACT,
            "corrected",
        ),
        (
            ("x_m", "y_m", "z_m"),
            explicit_frame or MEASUREMENT_FRAME,
            explicit_axis or MEASUREMENT_AXIS_CONTRACT,
            "legacy",
        ),
    )
    for fields, frame, axis_contract, value_kind in field_sets:
        values = tuple(_coerce_float(row.get(field)) for field in fields)
        if None not in values:
            return values, frame, axis_contract, value_kind  # type: ignore[return-value]
    return None


def _row_xyz(row: dict[str, object]) -> tuple[float, float, float] | None:
    measurement = _row_measurement(row)
    return None if measurement is None else measurement[0]


def _row_distance_m(row: dict[str, object], xyz: tuple[float, float, float], value_kind: str = "") -> float:
    if value_kind.startswith("raw"):
        parsed = _coerce_float(row.get("raw_range_m"))
        if parsed is not None:
            return parsed
        parsed = _coerce_float(row.get("raw_distance_m"))
        if parsed is not None:
            return parsed
        return math.sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2])
    parsed = _coerce_float(row.get("corrected_distance_m"))
    if parsed is not None:
        return parsed
    parsed = _coerce_float(row.get("distance_m"))
    if parsed is not None:
        return parsed
    return math.sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2])


def _embedded_truth_sample(row: dict[str, object], fallback_time_s: float) -> UamTelemetrySample | None:
    values = (
        _first_float_field(row, ("drone_north_m", "drone_x_m")),
        _first_float_field(row, ("drone_east_m", "drone_y_m")),
        _first_float_field(row, ("drone_down_m", "drone_z_m")),
    )
    if None in values:
        return None
    timestamp_s = _coerce_float(row.get("drone_timestamp_s"))
    if timestamp_s is None:
        age_s = _coerce_float(row.get("drone_age_s"))
        timestamp_s = fallback_time_s - age_s if age_s is not None else fallback_time_s
    utc_unix_s = _coerce_float(row.get("drone_utc_unix_s"))
    return UamTelemetrySample(
        timestamp_s=float(timestamp_s),
        x_m=float(values[0]),
        y_m=float(values[1]),
        z_m=float(values[2]),
        source="live-recorded-row",
        timestamp_reference=str(row.get("drone_timestamp_reference") or "app-receive"),
        utc_unix_s=utc_unix_s,
    )


def _fit_alignment_profile(
    source_points: Sequence[tuple[float, float, float]],
    target_points: Sequence[tuple[float, float, float]],
    *,
    profile_id: str | None = None,
    truth_source_type: str = "",
    truth_source_frame: str = "",
    truth_axis_contract: str = "",
    truth_time_basis: str = "",
    truth_origin_kind: str = "",
) -> UamAlignmentProfile:
    if len(source_points) != len(target_points):
        raise UamAlignmentError("measurement/truth pair count mismatch")
    if len(source_points) < 3:
        raise UamAlignmentError("axis/origin alignment requires at least 3 matched samples")
    if not _has_non_collinear_geometry(source_points) or not _has_non_collinear_geometry(target_points):
        raise UamAlignmentError("matched samples are degenerate; move the target across at least 3 non-collinear positions")

    source_centroid = _centroid(source_points)
    target_centroid = _centroid(target_points)
    h = [[0.0, 0.0, 0.0] for _ in range(3)]
    for source, target in zip(source_points, target_points):
        centered_source = _sub3(source, source_centroid)
        centered_target = _sub3(target, target_centroid)
        for row in range(3):
            for col in range(3):
                h[row][col] += centered_source[row] * centered_target[col]
    hxx, hxy, hxz = h[0]
    hyx, hyy, hyz = h[1]
    hzx, hzy, hzz = h[2]
    trace = hxx + hyy + hzz
    horn = (
        (trace, hyz - hzy, hzx - hxz, hxy - hyx),
        (hyz - hzy, hxx - hyy - hzz, hxy + hyx, hzx + hxz),
        (hzx - hxz, hxy + hyx, -hxx + hyy - hzz, hyz + hzy),
        (hxy - hyx, hzx + hxz, hyz + hzy, -hxx - hyy + hzz),
    )
    rotation = _quat_to_matrix(_largest_eigenvector_symmetric(horn))
    rotated_centroid = _mat_vec(rotation, source_centroid)
    origin_offset = _sub3(target_centroid, rotated_centroid)
    residuals = [
        _norm3(_sub3(_add3(_mat_vec(rotation, source), origin_offset), target))
        for source, target in zip(source_points, target_points)
    ]
    rmse = math.sqrt(sum(value * value for value in residuals) / len(residuals))
    return UamAlignmentProfile(
        profile_id=profile_id or f"uam_alignment_{time.strftime('%Y%m%d_%H%M%S')}",
        rotation_matrix=rotation,
        origin_offset_m=origin_offset,
        sample_count=len(source_points),
        rmse_m=rmse,
        max_error_m=max(residuals),
        created_unix_s=time.time(),
        truth_source_type=truth_source_type,
        truth_source_frame=truth_source_frame,
        truth_axis_contract=truth_axis_contract,
        truth_time_basis=truth_time_basis,
        truth_origin_kind=truth_origin_kind,
    )


def estimate_alignment_profile_from_rows(
    comparison_rows: Sequence[dict[str, object]],
    *,
    profile_id: str | None = None,
    min_samples: int = 3,
) -> UamAlignmentProfile:
    source_points: list[tuple[float, float, float]] = []
    target_points: list[tuple[float, float, float]] = []
    truth_source_types: set[str] = set()
    truth_source_frames: set[str] = set()
    truth_axis_contracts: set[str] = set()
    truth_time_bases: set[str] = set()
    truth_origin_kinds: set[str] = set()
    for row in comparison_rows:
        measurement = (
            _first_float_field(row, ("measurement_right_m", "stereo_center_right_m", "measurement_x_m", "x_m")),
            _first_float_field(row, ("measurement_down_m", "stereo_center_down_m", "measurement_y_m", "y_m")),
            _first_float_field(row, ("measurement_forward_m", "stereo_center_forward_m", "measurement_z_m", "z_m")),
        )
        if str(row.get("truth_frame") or "") == MEASUREMENT_FRAME:
            truth = (
                _first_float_field(row, ("truth_right_m", "truth_x_m")),
                _first_float_field(row, ("truth_down_m", "truth_y_m")),
                _first_float_field(row, ("truth_forward_m", "truth_z_m")),
            )
        else:
            truth = (
                _first_float_field(row, ("truth_north_m", "truth_x_m")),
                _first_float_field(row, ("truth_east_m", "truth_y_m")),
                _first_float_field(row, ("truth_down_m", "truth_z_m")),
            )
        if None in measurement or None in truth:
            continue
        source_points.append((float(measurement[0]), float(measurement[1]), float(measurement[2])))
        target_points.append((float(truth[0]), float(truth[1]), float(truth[2])))
        truth_source_type = str(row.get("truth_source_type") or "").strip()
        if truth_source_type:
            truth_source_types.add(truth_source_type)
        truth_source_frame = str(row.get("truth_source_frame") or "").strip()
        if truth_source_frame:
            truth_source_frames.add(truth_source_frame)
        truth_axis_contract = str(row.get("truth_source_axis_contract") or row.get("truth_axis_contract") or "").strip()
        if truth_axis_contract:
            truth_axis_contracts.add(truth_axis_contract)
        truth_time_basis = str(row.get("truth_time_basis") or "").strip()
        if truth_time_basis:
            truth_time_bases.add(truth_time_basis)
        truth_origin_kind = str(row.get("truth_gps_reference_kind") or "").strip()
        if truth_origin_kind:
            truth_origin_kinds.add(truth_origin_kind)
    if len(source_points) < int(min_samples):
        raise UamAlignmentError(
            f"axis/origin alignment requires at least {int(min_samples)} matched samples; got {len(source_points)}"
        )
    if len(truth_source_types) > 1:
        raise UamAlignmentError(
            "matched rows mix multiple truth source types; build one profile per selected truth source"
        )
    if len(truth_source_frames) > 1 or len(truth_axis_contracts) > 1:
        raise UamAlignmentError(
            "matched rows mix multiple truth frame/axis contracts; build one profile per coordinate contract"
        )
    return _fit_alignment_profile(
        source_points,
        target_points,
        profile_id=profile_id,
        truth_source_type=next(iter(truth_source_types), ""),
        truth_source_frame=next(iter(truth_source_frames), ""),
        truth_axis_contract=next(iter(truth_axis_contracts), ""),
        truth_time_basis="mixed" if len(truth_time_bases) > 1 else next(iter(truth_time_bases), ""),
        truth_origin_kind="mixed" if len(truth_origin_kinds) > 1 else next(iter(truth_origin_kinds), ""),
    )


def _format_optional_float(value: float | None, precision: int = 6) -> str:
    if value is None:
        return ""
    number = float(value)
    if abs(number) < 0.5 * 10 ** (-precision):
        number = 0.0
    return f"{number:.{precision}f}"


def _sample_at_time_basis(
    samples: Sequence[UamTelemetrySample],
    timestamp_s: float,
    *,
    max_dt_s: float,
    get_timestamp_s: Callable[[UamTelemetrySample], float | None],
    timestamp_reference: str | None = None,
    interpolation_suffix: str = "interp",
) -> tuple[UamTelemetrySample, float] | None:
    timed_samples = [
        (sample, sample_time_s)
        for sample in samples
        if (sample_time_s := get_timestamp_s(sample)) is not None
    ]
    timed_samples.sort(key=lambda item: item[1])
    if not timed_samples:
        return None
    if len(timed_samples) == 1:
        sample, sample_time_s = timed_samples[0]
        dt_s = timestamp_s - sample_time_s
        return (sample, dt_s) if abs(dt_s) <= max_dt_s else None

    before: tuple[UamTelemetrySample, float] | None = None
    after: tuple[UamTelemetrySample, float] | None = None
    for sample, sample_time_s in timed_samples:
        if sample_time_s <= timestamp_s:
            before = (sample, sample_time_s)
            continue
        after = (sample, sample_time_s)
        break

    if before is not None and after is not None and before[1] != after[1]:
        before_sample, before_time_s = before
        after_sample, after_time_s = after
        if timestamp_s - before_time_s <= max_dt_s and after_time_s - timestamp_s <= max_dt_s:
            ratio = (timestamp_s - before_time_s) / (after_time_s - before_time_s)
            utc_unix_s = None
            if before_sample.utc_unix_s is not None and after_sample.utc_unix_s is not None:
                utc_unix_s = before_sample.utc_unix_s + (after_sample.utc_unix_s - before_sample.utc_unix_s) * ratio
            interpolated = UamTelemetrySample(
                timestamp_s=before_sample.timestamp_s + (after_sample.timestamp_s - before_sample.timestamp_s) * ratio,
                x_m=before_sample.x_m + (after_sample.x_m - before_sample.x_m) * ratio,
                y_m=before_sample.y_m + (after_sample.y_m - before_sample.y_m) * ratio,
                z_m=before_sample.z_m + (after_sample.z_m - before_sample.z_m) * ratio,
                source=f"{before_sample.source}:{interpolation_suffix}",
                timestamp_reference=timestamp_reference or before_sample.timestamp_reference,
                utc_unix_s=utc_unix_s,
                gps_reference_lat_deg=before_sample.gps_reference_lat_deg,
                gps_reference_lon_deg=before_sample.gps_reference_lon_deg,
                gps_reference_alt_m=before_sample.gps_reference_alt_m,
                gps_reference_kind=before_sample.gps_reference_kind,
                gps_altitude_reference=before_sample.gps_altitude_reference,
                source_frame=before_sample.source_frame,
                source_axis_contract=before_sample.source_axis_contract,
                raw_lat_deg=before_sample.raw_lat_deg,
                raw_lon_deg=before_sample.raw_lon_deg,
                raw_alt_m=before_sample.raw_alt_m,
                conversion_status=before_sample.conversion_status,
            )
            return interpolated, 0.0

    nearest, nearest_time_s = min(timed_samples, key=lambda item: abs(item[1] - timestamp_s))
    dt_s = timestamp_s - nearest_time_s
    return (nearest, dt_s) if abs(dt_s) <= max_dt_s else None


def sample_at_time(
    samples: Sequence[UamTelemetrySample],
    timestamp_s: float,
    *,
    max_dt_s: float,
) -> tuple[UamTelemetrySample, float] | None:
    return _sample_at_time_basis(
        samples,
        timestamp_s,
        max_dt_s=max_dt_s,
        get_timestamp_s=lambda sample: sample.timestamp_s,
    )


def sample_at_utc_time(
    samples: Sequence[UamTelemetrySample],
    utc_unix_s: float,
    *,
    max_dt_s: float,
) -> tuple[UamTelemetrySample, float] | None:
    return _sample_at_time_basis(
        samples,
        utc_unix_s,
        max_dt_s=max_dt_s,
        get_timestamp_s=lambda sample: sample.utc_unix_s,
        timestamp_reference="utc-unix",
        interpolation_suffix="utc-interp",
    )


def compare_measurement_rows(
    rows: Iterable[dict[str, object]],
    telemetry_samples: Sequence[UamTelemetrySample],
    *,
    truth_source: str,
    max_dt_s: float,
    time_offset_s: float = 0.0,
    allow_embedded_truth: bool = True,
    alignment_profile: UamAlignmentProfile | None = None,
) -> list[dict[str, object]]:
    comparisons: list[dict[str, object]] = []
    sorted_samples = sorted(telemetry_samples, key=lambda sample: sample.timestamp_s)
    sorted_utc_samples = sorted(
        (sample for sample in telemetry_samples if sample.utc_unix_s is not None),
        key=lambda sample: float(sample.utc_unix_s or 0.0),
    )
    for index, row in enumerate(rows, start=1):
        measurement_time_s = _measurement_time_s(row)
        measurement_utc_unix_s = _measurement_utc_unix_s(row)
        measurement = _row_measurement(row)
        if measurement is None or (measurement_time_s is None and measurement_utc_unix_s is None):
            continue
        measurement_xyz, measurement_frame, measurement_axis_contract, measurement_value_kind = measurement
        matched: tuple[UamTelemetrySample, float] | None = None
        truth_time_basis = ""
        measurement_time_basis = ""
        target_truth_time_s = None if measurement_time_s is None else measurement_time_s + float(time_offset_s)
        target_truth_utc_unix_s = (
            None if measurement_utc_unix_s is None else measurement_utc_unix_s + float(time_offset_s)
        )
        if target_truth_utc_unix_s is not None and sorted_utc_samples:
            matched = sample_at_utc_time(
                sorted_utc_samples,
                target_truth_utc_unix_s,
                max_dt_s=float(max_dt_s),
            )
            if matched is not None:
                truth_time_basis = "utc-unix"
                measurement_time_basis = "bbox_mid_utc_unix_s"
        if matched is None and target_truth_time_s is not None and sorted_samples:
            matched = sample_at_time(sorted_samples, target_truth_time_s, max_dt_s=float(max_dt_s))
            if matched is not None:
                truth_time_basis = matched[0].timestamp_reference or "px4-boot"
                measurement_time_basis = "bbox_mid_timestamp_s"
        if matched is None and allow_embedded_truth:
            embedded_fallback_time_s = target_truth_time_s if target_truth_time_s is not None else 0.0
            embedded = _embedded_truth_sample(row, embedded_fallback_time_s)
            if embedded is not None:
                if target_truth_utc_unix_s is not None and embedded.utc_unix_s is not None:
                    matched = (embedded, target_truth_utc_unix_s - embedded.utc_unix_s)
                    truth_time_basis = "embedded-utc-unix"
                    measurement_time_basis = "bbox_mid_utc_unix_s"
                elif target_truth_time_s is not None:
                    matched = (embedded, target_truth_time_s - embedded.timestamp_s)
                    truth_time_basis = embedded.timestamp_reference or "embedded-row"
                    measurement_time_basis = "bbox_mid_timestamp_s"
        if matched is None:
            continue
        truth, dt_s = matched
        measurement_distance_m = _row_distance_m(row, measurement_xyz, measurement_value_kind)
        truth_frame = str(row.get("truth_frame") or TRUTH_FRAME)
        truth_axis_contract = str(row.get("truth_axis_contract") or TRUTH_AXIS_CONTRACT)
        aligned_xyz: tuple[float, float, float] | None = None
        truth_rdf_xyz: tuple[float, float, float] | None = None
        baseline_center_ned: tuple[float, float, float] | None = None
        alignment_profile_id = ""
        alignment_method = ""
        if measurement_frame == truth_frame:
            aligned_xyz = measurement_xyz
            truth_rdf_xyz = (truth.x_m, truth.y_m, truth.z_m)
            alignment_method = "already-truth-frame"
        elif alignment_profile is not None:
            aligned_xyz = alignment_profile.apply(measurement_xyz)
            truth_rdf_xyz = alignment_profile.inverse_apply((truth.x_m, truth.y_m, truth.z_m))
            baseline_center_ned = alignment_profile.origin_offset_m
            alignment_profile_id = alignment_profile.profile_id
            alignment_method = alignment_profile.method
        comparison_frame = truth_frame if aligned_xyz is not None else MIXED_FRAME
        error_basis_warning = ""
        if comparison_frame == MIXED_FRAME:
            error_basis_warning = (
                "measurement is stereo-center camera data while truth is PX4 local North/East/Down; "
                "apply alignment profile before treating axis errors as final"
            )
        truth_source_text = truth.source or truth_source
        truth_source_type = truth_source_type_from_sample(truth)
        measurement_bias_policy = ""
        if measurement_value_kind.startswith("raw") and str(row.get("bias_profile_id") or "").strip():
            measurement_bias_policy = "recorded live bias ignored; raw stereo-center measurement used"
        error_basis = aligned_xyz if aligned_xyz is not None else measurement_xyz
        error_x = error_basis[0] - truth.x_m
        error_y = error_basis[1] - truth.y_m
        error_z = error_basis[2] - truth.z_m
        rdf_error: tuple[float, float, float] | None = None
        if truth_rdf_xyz is not None:
            rdf_error = _sub3(measurement_xyz, truth_rdf_xyz)
        measurement_frd_xyz = (measurement_xyz[2], measurement_xyz[0], measurement_xyz[1])
        truth_frd_xyz: tuple[float, float, float] | None = None
        frd_error: tuple[float, float, float] | None = None
        if truth_rdf_xyz is not None:
            truth_frd_xyz = (truth_rdf_xyz[2], truth_rdf_xyz[0], truth_rdf_xyz[1])
            frd_error = _sub3(measurement_frd_xyz, truth_frd_xyz)
        position_error_m = _norm3((error_x, error_y, error_z))
        aligned_distance_m = None if aligned_xyz is None else _norm3(aligned_xyz)
        truth_rdf_range_m = None if truth_rdf_xyz is None else _norm3(truth_rdf_xyz)
        truth_frd_range_m = None if truth_frd_xyz is None else _norm3(truth_frd_xyz)
        truth_range_m = (
            _norm3(_sub3((truth.x_m, truth.y_m, truth.z_m), baseline_center_ned))
            if baseline_center_ned is not None
            else truth.distance_m
        )
        range_error_m = measurement_distance_m - truth_range_m
        error_distance = position_error_m if aligned_xyz is not None else range_error_m
        measurement_north = None if aligned_xyz is None else aligned_xyz[0]
        measurement_east = None if aligned_xyz is None else aligned_xyz[1]
        measurement_down = None if aligned_xyz is None else aligned_xyz[2]
        comparisons.append(
            {
                "row_index": index,
                "measurement_timestamp_s": _format_optional_float(measurement_time_s),
                "measurement_utc_unix_s": _format_optional_float(measurement_utc_unix_s),
                "measurement_time_basis": measurement_time_basis,
                "camera_time_sync_status": str(row.get("camera_time_sync_status") or "unrecorded"),
                "camera_time_sync_basis": str(row.get("camera_time_sync_basis") or ""),
                "camera_time_sync_warning": str(row.get("camera_time_sync_warning") or ""),
                "rtcp_pair_delta_s": str(row.get("rtcp_pair_delta_s") or ""),
                "rtcp_pc_offset_max_ms": str(row.get("rtcp_pc_offset_max_ms") or ""),
                "truth_timestamp_s": f"{truth.timestamp_s:.6f}",
                "truth_utc_unix_s": _format_optional_float(truth.utc_unix_s),
                "truth_dt_s": f"{dt_s:.6f}",
                "truth_time_basis": truth_time_basis,
                "truth_timestamp_reference": truth.timestamp_reference,
                "truth_source": truth_source_text,
                "truth_source_type": truth_source_type,
                "truth_source_frame": truth.source_frame,
                "truth_source_axis_contract": truth.source_axis_contract,
                "truth_conversion_status": truth.conversion_status,
                "truth_raw_lat_deg": _format_optional_float(truth.raw_lat_deg, precision=9),
                "truth_raw_lon_deg": _format_optional_float(truth.raw_lon_deg, precision=9),
                "truth_raw_alt_m": _format_optional_float(truth.raw_alt_m),
                "truth_gps_reference_lat_deg": _format_optional_float(truth.gps_reference_lat_deg, precision=9),
                "truth_gps_reference_lon_deg": _format_optional_float(truth.gps_reference_lon_deg, precision=9),
                "truth_gps_reference_alt_m": _format_optional_float(truth.gps_reference_alt_m),
                "truth_gps_reference_kind": truth.gps_reference_kind,
                "truth_gps_altitude_reference": truth.gps_altitude_reference,
                "measurement_frame": measurement_frame,
                "measurement_axis_contract": measurement_axis_contract,
                "camera_comparison_axis_contract": FRD_COMPARISON_AXIS_CONTRACT,
                "measurement_value_kind": measurement_value_kind,
                "measurement_bias_policy": measurement_bias_policy,
                "measurement_x_m": f"{measurement_xyz[0]:.6f}",
                "measurement_y_m": f"{measurement_xyz[1]:.6f}",
                "measurement_z_m": f"{measurement_xyz[2]:.6f}",
                "measurement_right_m": f"{measurement_xyz[0]:.6f}",
                "measurement_down_m": f"{measurement_xyz[1]:.6f}",
                "measurement_forward_m": f"{measurement_xyz[2]:.6f}",
                "measurement_distance_m": f"{measurement_distance_m:.6f}",
                "estimated_range_m": f"{measurement_distance_m:.6f}",
                "measurement_frd_forward_m": _format_optional_float(measurement_frd_xyz[0]),
                "measurement_frd_right_m": _format_optional_float(measurement_frd_xyz[1]),
                "measurement_frd_down_m": _format_optional_float(measurement_frd_xyz[2]),
                "truth_rdf_right_m": _format_optional_float(None if truth_rdf_xyz is None else truth_rdf_xyz[0]),
                "truth_rdf_down_m": _format_optional_float(None if truth_rdf_xyz is None else truth_rdf_xyz[1]),
                "truth_rdf_forward_m": _format_optional_float(None if truth_rdf_xyz is None else truth_rdf_xyz[2]),
                "truth_rdf_range_m": _format_optional_float(truth_rdf_range_m),
                "error_rdf_right_m": _format_optional_float(None if rdf_error is None else rdf_error[0]),
                "error_rdf_down_m": _format_optional_float(None if rdf_error is None else rdf_error[1]),
                "error_rdf_forward_m": _format_optional_float(None if rdf_error is None else rdf_error[2]),
                "truth_frd_forward_m": _format_optional_float(None if truth_frd_xyz is None else truth_frd_xyz[0]),
                "truth_frd_right_m": _format_optional_float(None if truth_frd_xyz is None else truth_frd_xyz[1]),
                "truth_frd_down_m": _format_optional_float(None if truth_frd_xyz is None else truth_frd_xyz[2]),
                "truth_frd_range_m": _format_optional_float(truth_frd_range_m),
                "error_frd_forward_m": _format_optional_float(None if frd_error is None else frd_error[0]),
                "error_frd_right_m": _format_optional_float(None if frd_error is None else frd_error[1]),
                "error_frd_down_m": _format_optional_float(None if frd_error is None else frd_error[2]),
                "measurement_north_m": _format_optional_float(measurement_north),
                "measurement_east_m": _format_optional_float(measurement_east),
                "measurement_down_ned_m": _format_optional_float(measurement_down),
                "measurement_n_m": _format_optional_float(measurement_north),
                "measurement_e_m": _format_optional_float(measurement_east),
                "measurement_d_m": _format_optional_float(measurement_down),
                "estimated_north_m": _format_optional_float(measurement_north),
                "estimated_east_m": _format_optional_float(measurement_east),
                "estimated_down_m": _format_optional_float(measurement_down),
                "estimated_n_m": _format_optional_float(measurement_north),
                "estimated_e_m": _format_optional_float(measurement_east),
                "estimated_d_m": _format_optional_float(measurement_down),
                "measurement_ned_distance_m": _format_optional_float(aligned_distance_m),
                "truth_frame": truth_frame,
                "truth_axis_contract": truth_axis_contract,
                "truth_x_m": f"{truth.x_m:.6f}",
                "truth_y_m": f"{truth.y_m:.6f}",
                "truth_z_m": f"{truth.z_m:.6f}",
                "truth_north_m": f"{truth.x_m:.6f}",
                "truth_east_m": f"{truth.y_m:.6f}",
                "truth_down_m": f"{truth.z_m:.6f}",
                "truth_n_m": f"{truth.x_m:.6f}",
                "truth_e_m": f"{truth.y_m:.6f}",
                "truth_d_m": f"{truth.z_m:.6f}",
                "truth_distance_m": f"{truth.distance_m:.6f}",
                "truth_range_m": f"{truth_range_m:.6f}",
                "comparison_frame": comparison_frame,
                "error_x_m": _format_optional_float(error_x),
                "error_y_m": _format_optional_float(error_y),
                "error_z_m": _format_optional_float(error_z),
                "error_north_m": _format_optional_float(error_x if aligned_xyz is not None else None),
                "error_east_m": _format_optional_float(error_y if aligned_xyz is not None else None),
                "error_down_m": _format_optional_float(error_z if aligned_xyz is not None else None),
                "position_error_m": _format_optional_float(position_error_m if aligned_xyz is not None else None),
                "range_error_m": _format_optional_float(range_error_m),
                "range_error_basis": "baseline_center" if baseline_center_ned is not None else "truth_origin",
                "error_distance_m": _format_optional_float(error_distance),
                "error_distance_kind": "position_error_m" if aligned_xyz is not None else "range_difference_m",
                "error_frame": comparison_frame,
                "error_basis_warning": error_basis_warning,
                "alignment_profile_id": alignment_profile_id,
                "alignment_method": alignment_method,
                "alignment_origin_offset_north_m": (
                    "" if alignment_profile is None else f"{alignment_profile.origin_offset_m[0]:.6f}"
                ),
                "alignment_origin_offset_east_m": (
                    "" if alignment_profile is None else f"{alignment_profile.origin_offset_m[1]:.6f}"
                ),
                "alignment_origin_offset_down_m": (
                    "" if alignment_profile is None else f"{alignment_profile.origin_offset_m[2]:.6f}"
                ),
                "baseline_center_north_m": (
                    "" if baseline_center_ned is None else f"{baseline_center_ned[0]:.6f}"
                ),
                "baseline_center_east_m": (
                    "" if baseline_center_ned is None else f"{baseline_center_ned[1]:.6f}"
                ),
                "baseline_center_down_m": (
                    "" if baseline_center_ned is None else f"{baseline_center_ned[2]:.6f}"
                ),
            }
        )
    return comparisons


def comparison_summary(rows: Sequence[dict[str, object]]) -> dict[str, float]:
    errors = [_coerce_float(row.get("error_distance_m")) for row in rows]
    values = [abs(value) for value in errors if value is not None]
    if not values:
        return {}
    values.sort()
    median = values[len(values) // 2]
    p95 = values[min(len(values) - 1, int(math.ceil(len(values) * 0.95)) - 1)]
    rmse = math.sqrt(sum(value * value for value in values) / len(values))
    return {"count": float(len(values)), "median_abs_m": median, "p95_abs_m": p95, "rmse_m": rmse}


def estimate_time_offset_s(
    rows: Sequence[dict[str, object]],
    telemetry_samples: Sequence[UamTelemetrySample],
    *,
    max_dt_s: float,
    initial_offset_s: float = 0.0,
    search_radius_s: float = 10.0,
    step_s: float | None = None,
) -> dict[str, float] | None:
    samples = sorted(telemetry_samples, key=lambda sample: sample.timestamp_s)
    if not samples:
        return None

    measurement_utc_times = [
        timestamp_s
        for row in rows
        if (timestamp_s := _measurement_utc_unix_s(row)) is not None and _row_xyz(row) is not None
    ]
    utc_truth_times = [float(sample.utc_unix_s) for sample in samples if sample.utc_unix_s is not None]
    if measurement_utc_times and utc_truth_times:
        measurement_times = measurement_utc_times
        truth_times = sorted(utc_truth_times)
    else:
        measurement_times = [
            timestamp_s
            for row in rows
            if (timestamp_s := _measurement_time_s(row)) is not None and _row_xyz(row) is not None
        ]
        truth_times = [sample.timestamp_s for sample in samples]
    if not measurement_times or not truth_times:
        return None

    measurement_times.sort()
    midpoint_index = len(measurement_times) // 2
    truth_midpoint_index = len(truth_times) // 2
    base_offsets = {
        float(initial_offset_s),
        truth_times[0] - measurement_times[0],
        truth_times[-1] - measurement_times[-1],
        truth_times[truth_midpoint_index] - measurement_times[midpoint_index],
        ((truth_times[0] + truth_times[-1]) * 0.5) - ((measurement_times[0] + measurement_times[-1]) * 0.5),
    }

    step = float(step_s) if step_s is not None else max(0.01, min(0.25, float(max_dt_s) * 0.5))
    radius = max(float(search_radius_s), float(max_dt_s), step)
    offsets: set[float] = set()
    steps_each_side = max(1, int(math.ceil(radius / step)))
    for base in base_offsets:
        for index in range(-steps_each_side, steps_each_side + 1):
            offsets.add(round(base + index * step, 6))

    best: tuple[int, float, float, float, list[dict[str, object]]] | None = None
    for offset_s in sorted(offsets):
        comparisons = compare_measurement_rows(
            rows,
            samples,
            truth_source="offset-estimate",
            max_dt_s=max_dt_s,
            time_offset_s=offset_s,
            allow_embedded_truth=False,
        )
        if not comparisons:
            continue
        errors = [_coerce_float(row.get("error_distance_m")) for row in comparisons]
        abs_errors = [abs(value) for value in errors if value is not None]
        if not abs_errors:
            continue
        dt_values = [_coerce_float(row.get("truth_dt_s")) for row in comparisons]
        abs_dt_values = [abs(value) for value in dt_values if value is not None]
        rmse = math.sqrt(sum(value * value for value in abs_errors) / len(abs_errors))
        mean_abs_dt = sum(abs_dt_values) / len(abs_dt_values) if abs_dt_values else 0.0
        candidate = (len(comparisons), rmse, mean_abs_dt, float(offset_s), comparisons)
        if best is None:
            best = candidate
            continue
        if candidate[0] > best[0] or (
            candidate[0] == best[0]
            and (candidate[1] < best[1] or (math.isclose(candidate[1], best[1]) and candidate[2] < best[2]))
        ):
            best = candidate

    if best is None:
        return None
    count, rmse, mean_abs_dt, offset_s, _comparisons = best
    return {
        "time_offset_s": offset_s,
        "matched_count": float(count),
        "rmse_m": rmse,
        "mean_abs_dt_s": mean_abs_dt,
    }
