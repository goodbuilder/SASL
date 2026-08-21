"""Small, filesystem-backed calibration-session catalog.

The calibration UI and the owner controller both treat a session root as the
unit of capture, computation, and result persistence.  Discovery is tolerant
of older layouts: any directory containing matching ``left_*.png`` and
``right_*.png`` pairs is a selectable offline photo set.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path


SESSION_MANIFEST_NAME = "calibration_session.json"


@dataclass(frozen=True)
class CalibrationSession:
    root: Path
    session_id: str
    pair_count: int
    updated_ns: int
    manifest_path: Path

    @property
    def display_name(self) -> str:
        return f"{self.session_id} · {self.pair_count} pair"


def _pair_count(root: Path) -> int:
    left = {
        path.stem.removeprefix("left_")
        for path in (root / "left").glob("left_*.png")
    }
    right = {
        path.stem.removeprefix("right_")
        for path in (root / "right").glob("right_*.png")
    }
    return len(left & right)


def discover_calibration_sessions(base_root: str | Path) -> tuple[CalibrationSession, ...]:
    base = Path(base_root).expanduser()
    if not base.is_dir():
        return ()
    candidates: set[Path] = set()
    for left_dir in base.rglob("left"):
        if not left_dir.is_dir() or not (left_dir.parent / "right").is_dir():
            continue
        if not any(left_dir.glob("left_*.png")) or not any(
            (left_dir.parent / "right").glob("right_*.png")
        ):
            continue
        candidates.add(left_dir.parent)
    sessions: list[CalibrationSession] = []
    for root in candidates:
        try:
            updated_ns = max(
                path.stat().st_mtime_ns
                for path in (root / "left").glob("left_*.png")
            )
        except (OSError, ValueError):
            continue
        sessions.append(
            CalibrationSession(
                root=root,
                session_id=root.name,
                pair_count=_pair_count(root),
                updated_ns=int(updated_ns),
                manifest_path=root / SESSION_MANIFEST_NAME,
            )
        )
    return tuple(sorted(sessions, key=lambda item: (item.updated_ns, str(item.root)), reverse=True))


def write_calibration_session_manifest(
    root: str | Path,
    *,
    session_kind: str = "calibration",
    source: str = "",
) -> Path:
    session_root = Path(root)
    path = session_root / SESSION_MANIFEST_NAME
    previous: dict[str, object] = {}
    if path.exists():
        try:
            loaded = json.loads(path.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                previous = loaded
        except (OSError, ValueError, TypeError):
            previous = {}
    payload = {
        "schema": "workstation-calibration-session-v1",
        "session_id": session_root.name,
        "session_kind": session_kind,
        "created_unix_ns": int(previous.get("created_unix_ns") or time.time_ns()),
        "updated_unix_ns": time.time_ns(),
        "root": str(session_root),
        "last_source": str(source or previous.get("last_source") or ""),
        "pair_count": _pair_count(session_root),
        "source_roots": {
            "HR": str(session_root / "calib_capture" / "HR"),
            "OZ": str(session_root / "calib_capture" / "OZ"),
            "AutoCapture": str(session_root / "calib_capture" / "AutoCapture"),
        },
    }
    session_root.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    return path
