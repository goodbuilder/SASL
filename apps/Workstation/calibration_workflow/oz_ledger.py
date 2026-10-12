"""OZ trigger/download ledger and strict one-to-one remote-file pairing.

Camera-side names remain device paths. This owner neither starts a worker nor
changes widgets; batches are returned only after the existing all-or-nothing
count/validity admission, keeping originals on failure.
"""
from __future__ import annotations
import json
import time
from pathlib import Path
from typing import Mapping
from PyQt5 import QtGui


class OzCaptureLedger:
    def read(self, path: Path) -> dict[str, object]:
        empty: dict[str, object] = {"rounds": [], "consumed_remote_names": {"left": [], "right": []}}
        if not path.exists():
            return empty
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return empty
        if not isinstance(data, dict):
            return empty
        data.setdefault("rounds", [])
        data.setdefault("consumed_remote_names", {"left": [], "right": []})
        return data


    def write(self, path: Path, ledger: dict[str, object]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(ledger, indent=2), encoding="utf-8")


    @staticmethod
    def _ftp_calibration_capture_metadata(
        *,
        capture_id: str,
        cameras: Mapping[object, object],
        frames: Mapping[str, QtGui.QImage | None],
    ) -> dict[str, dict[str, object]]:
        """Describe FTP stills without borrowing the live RTSP clock.

        CPTR/ICPT and FTP timestamps are lifecycle evidence, not exposure
        timestamps.  Keeping the reference explicitly unknown prevents
        ``capture_calibration_images`` from falling back to the panel's
        unrelated current preview metadata and forming a false stereo pair.
        """

        result: dict[str, dict[str, object]] = {}
        for side in ("left", "right"):
            raw_item = cameras.get(side)
            item = raw_item if isinstance(raw_item, Mapping) else {}
            image = frames.get(side)
            result[side] = {
                "timestamp_reference": "unknown",
                "capture_timestamp_known": False,
                "ftp_capture_id": capture_id,
                "ftp_remote_path": str(item.get("ftp_remote_path") or ""),
                "capture_requested_unix_ns": item.get("capture_requested_unix_ns"),
                "capture_requested_monotonic_ns": item.get(
                    "capture_requested_monotonic_ns"
                ),
                "capture_pairing_contract": item.get("capture_pairing_contract"),
                "capture_completed_unix_ns": item.get("capture_completed_unix_ns"),
                "ftp_download_finished_unix_ns": item.get("finished_unix_ns"),
                "native_width": image.width() if isinstance(image, QtGui.QImage) else 0,
                "native_height": image.height() if isinstance(image, QtGui.QImage) else 0,
                "native_lossless_source": False,
            }
        return result


    @staticmethod
    def _ftp_capture_failures(cameras: dict[object, object]) -> list[str]:
        failures: list[str] = []
        for side, raw_item in cameras.items():
            item = raw_item if isinstance(raw_item, dict) else {}
            state = str(item.get("state") or "failed")
            if state != "completed":
                reason = str(item.get("error") or state)
            elif not bool(item.get("ftp_downloaded")):
                reason = str(item.get("ftp_error") or "FTP download failed")
            else:
                continue
            failures.append(f"{side}: {reason}")
        return failures


    def record_capture(self, ledger, *, capture_id, cameras):
        left_ok = bool((cameras.get("left") or {}).get("stored_on_camera"))
        right_ok = bool((cameras.get("right") or {}).get("stored_on_camera"))
        synchronized = left_ok and right_ok
        rounds = ledger["rounds"]
        rounds.append(
            {
                "capture_id": capture_id,
                "requested_at_unix_ns": min(
                    (
                        int((cameras.get(side) or {}).get("capture_requested_unix_ns"))
                        for side in ("left", "right")
                        if (cameras.get(side) or {}).get("capture_requested_unix_ns")
                    ),
                    default=time.time_ns(),
                ),
                "left": {
                    "stored_on_camera": left_ok,
                    "state": (cameras.get("left") or {}).get("state"),
                    "capture_requested_unix_ns": (cameras.get("left") or {}).get(
                        "capture_requested_unix_ns"
                    ),
                    "capture_requested_monotonic_ns": (cameras.get("left") or {}).get(
                        "capture_requested_monotonic_ns"
                    ),
                    "capture_pairing_contract": (cameras.get("left") or {}).get(
                        "capture_pairing_contract"
                    ),
                },
                "right": {
                    "stored_on_camera": right_ok,
                    "state": (cameras.get("right") or {}).get("state"),
                    "capture_requested_unix_ns": (cameras.get("right") or {}).get(
                        "capture_requested_unix_ns"
                    ),
                    "capture_requested_monotonic_ns": (cameras.get("right") or {}).get(
                        "capture_requested_monotonic_ns"
                    ),
                    "capture_pairing_contract": (cameras.get("right") or {}).get(
                        "capture_pairing_contract"
                    ),
                },
                "synchronized": synchronized,
                "downloaded": False,
                "left_remote_name": None,
                "right_remote_name": None,
            }
        )
        pending = sum(1 for row in rounds if row.get("synchronized") and not row.get("downloaded"))
        return left_ok, right_ok, synchronized, pending

    def assemble_sync_batch(self, ledger, record, round_ids):
        left = record.get("left") if isinstance(record.get("left"), dict) else {}
        right = record.get("right") if isinstance(record.get("right"), dict) else {}
        left_items = list(left.get("items") or ())
        right_items = list(right.get("items") or ())
        messages = []
        left_invalid = [str(item.get("name")) for item in left_items if not item.get("valid")]
        right_invalid = [str(item.get("name")) for item in right_items if not item.get("valid")]
        if len(left_items) != len(round_ids) or len(right_items) != len(round_ids) or left_invalid or right_invalid:
            messages.append(
                "[캘리브레이션] WIRIS FTP 동기화 실패: 좌우 회차 수/유효성이 요청과 일치하지 않아 "
                f"배치 전체를 건너뜁니다 (요청={len(round_ids)}, left={len(left_items)}개"
                f"(무효 {left_invalid}), right={len(right_items)}개(무효 {right_invalid})). "
                "카메라 원본은 삭제하지 않았습니다 -- 다른 캡처 채널이 같은 원격 디렉토리에 "
                "파일을 남기지 않았는지 확인 후 다시 시도하세요."
            )
            status = (
                "WIRIS FTP sync failed: 좌우 회차가 정확히 매칭되지 않아 중단했습니다 (로그 참고)."
            )
            return None, messages, status
        rounds_by_id = {str(row.get("capture_id")): row for row in ledger["rounds"]}
        consumed = ledger.setdefault("consumed_remote_names", {"left": [], "right": []})
        queue: list[
            tuple[str, dict[str, QtGui.QImage], dict[str, dict[str, object]]]
        ] = []
        delete_names: dict[str, list[str]] = {"left": [], "right": []}
        for index, capture_id in enumerate(round_ids):
            left_name, right_name = str(left_items[index]["name"]), str(right_items[index]["name"])
            row = rounds_by_id.get(capture_id)
            if row is not None:
                row["downloaded"] = True
                row["left_remote_name"] = left_name
                row["right_remote_name"] = right_name
            consumed.setdefault("left", []).append(left_name)
            consumed.setdefault("right", []).append(right_name)
            delete_names["left"].append(left_name)
            delete_names["right"].append(right_name)
            frames = {
                "left": QtGui.QImage(str(left_items[index]["path"])),
                "right": QtGui.QImage(str(right_items[index]["path"])),
            }
            lifecycle = {
                "left": {
                    "ftp_remote_path": left_name,
                    "capture_requested_unix_ns": None if row is None else (
                        (row.get("left") or {}).get("capture_requested_unix_ns")
                        or row.get("requested_at_unix_ns")
                    ),
                    "capture_requested_monotonic_ns": None if row is None else (
                        (row.get("left") or {}).get("capture_requested_monotonic_ns")
                    ),
                    "capture_pairing_contract": None if row is None else (
                        (row.get("left") or {}).get("capture_pairing_contract")
                    ),
                },
                "right": {
                    "ftp_remote_path": right_name,
                    "capture_requested_unix_ns": None if row is None else (
                        (row.get("right") or {}).get("capture_requested_unix_ns")
                        or row.get("requested_at_unix_ns")
                    ),
                    "capture_requested_monotonic_ns": None if row is None else (
                        (row.get("right") or {}).get("capture_requested_monotonic_ns")
                    ),
                    "capture_pairing_contract": None if row is None else (
                        (row.get("right") or {}).get("capture_pairing_contract")
                    ),
                },
            }
            queue.append(
                (
                    capture_id,
                    frames,
                    self._ftp_calibration_capture_metadata(
                        capture_id=capture_id,
                        cameras=lifecycle,
                        frames=frames,
                    ),
                )
            )
        return (queue, delete_names), messages, ""

