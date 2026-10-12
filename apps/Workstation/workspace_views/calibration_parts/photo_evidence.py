"""Photo metadata reader; clock priority and native-grid evidence are unchanged."""
from __future__ import annotations
import json
import math
import time
from pathlib import Path
from collections.abc import Mapping
from PyQt5 import QtGui


class CalibrationPhotoEvidence:
    @staticmethod
    def _calibration_file_root(path: Path) -> Path:
        resolved = path.expanduser().resolve()
        if resolved.parent.name.lower() in {"left", "right"}:
            return resolved.parent.parent
        return resolved.parent

    @staticmethod
    def _calibration_file_key(path: Path, side: str) -> str:
        prefix = f"{side}_"
        return path.stem.removeprefix(prefix) if path.stem.startswith(prefix) else path.stem

    @classmethod
    def _calibration_capture_metadata_path(cls, path: Path) -> Path:
        side = path.parent.name.lower()
        key = cls._calibration_file_key(path, side)
        return cls._calibration_file_root(path) / "capture_metadata" / f"capture_{key}.json"

    @classmethod
    def _calibration_file_capture_time(
        cls,
        path: Path,
    ) -> tuple[int | None, str, str]:
        """Resolve one photo's wall-clock time and its independent provenance.

        Camera RTCP/NTP frame time is the strongest source because it belongs
        to the exact left or right image. Pair-level application record/request
        time is retained as a fallback; legacy external files use mtime and are
        labelled as such instead of being presented as a camera timestamp.
        """

        metadata_path = cls._calibration_capture_metadata_path(path)
        try:
            payload = json.loads(metadata_path.read_text(encoding="utf-8"))
        except (OSError, ValueError, TypeError):
            payload = {}
        if isinstance(payload, Mapping):
            side = path.parent.name.lower()
            capture_origin = str(payload.get("capture_origin") or "").strip().lower()
            nested = payload.get("frame_grid_by_side")
            side_payload = (
                nested.get(side, {})
                if isinstance(nested, Mapping) and isinstance(nested.get(side), Mapping)
                else {}
            )
            # FTP/OZ metadata historically copied the contemporaneous live
            # preview RTCP time, not the downloaded still's exposure time.
            # Never label that value as the selected FTP photo's timestamp.
            rtcp_is_bound_to_saved_frame = capture_origin in {"manual", "auto"} or (
                not capture_origin and bool(side_payload.get("native_lossless_source"))
            )
            if rtcp_is_bound_to_saved_frame:
                for container, field in (
                    (side_payload, "rtcp_frame_ntp_unix_s"),
                    (payload, f"{side}_rtcp_frame_ntp_unix_s"),
                ):
                    try:
                        seconds = float(container.get(field))
                    except (AttributeError, TypeError, ValueError):
                        continue
                    if math.isfinite(seconds) and seconds > 0.0:
                        whole_seconds = int(seconds)
                        fractional_ns = int(
                            round((seconds - whole_seconds) * 1_000_000_000)
                        )
                        if fractional_ns >= 1_000_000_000:
                            whole_seconds += 1
                            fractional_ns -= 1_000_000_000
                        return (
                            whole_seconds * 1_000_000_000 + fractional_ns,
                            "RTCP/NTP frame",
                            str(metadata_path),
                        )
            for field, source in (
                ("capture_recorded_unix_ns", "app capture record"),
                ("capture_requested_unix_ns", "capture request"),
            ):
                try:
                    unix_ns = int(payload.get(field))
                except (TypeError, ValueError):
                    continue
                if unix_ns > 0:
                    return unix_ns, source, str(metadata_path)
        try:
            return int(path.stat().st_mtime_ns), "file mtime", str(path)
        except OSError:
            return None, "unavailable", str(path)

    @staticmethod
    def _format_calibration_file_capture_time(unix_ns: int | None, source: str) -> str:
        if unix_ns is None or unix_ns <= 0:
            return "시각 없음 · unavailable"
        seconds, nanoseconds = divmod(int(unix_ns), 1_000_000_000)
        try:
            local_time = time.localtime(seconds)
        except (OverflowError, OSError, ValueError):
            return f"시각 해석 불가 ({unix_ns} ns) · {source}"
        timezone_name = time.strftime("%Z", local_time).strip()
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", local_time)
        timestamp += f".{nanoseconds // 1_000_000:03d}"
        if timezone_name:
            timestamp += f" {timezone_name}"
        return f"{timestamp} · {source}"

    @classmethod
    def _calibration_file_info(cls, path: Path) -> str:
        try:
            size_kib = path.stat().st_size / 1024.0
        except OSError:
            size_kib = 0.0
        size = QtGui.QImageReader(str(path)).size()
        dimensions = f"{size.width()}×{size.height()}" if size.isValid() else "?×?"
        unix_ns, source, _evidence_path = cls._calibration_file_capture_time(path)
        captured_at = cls._format_calibration_file_capture_time(unix_ns, source)
        return f"{dimensions} · {size_kib:.1f} KiB\n{captured_at}"

    @classmethod
    def _calibration_file_tooltip(cls, path: Path) -> str:
        unix_ns, source, evidence_path = cls._calibration_file_capture_time(path)
        captured_at = cls._format_calibration_file_capture_time(unix_ns, source)
        return (
            f"{path}\n"
            f"사진 시각: {captured_at}\n"
            f"시각 근거: {evidence_path}"
        )

    @classmethod
    def _calibration_file_signature(cls, path: Path) -> tuple[object, ...]:
        try:
            image_stat = path.stat()
            image_stamp = (int(image_stat.st_mtime_ns), int(image_stat.st_size))
        except OSError:
            image_stamp = (0, 0)
        metadata_path = cls._calibration_capture_metadata_path(path)
        try:
            metadata_stat = metadata_path.stat()
            metadata_stamp = (int(metadata_stat.st_mtime_ns), int(metadata_stat.st_size))
        except OSError:
            metadata_stamp = (0, 0)
        return str(path), *image_stamp, str(metadata_path), *metadata_stamp
