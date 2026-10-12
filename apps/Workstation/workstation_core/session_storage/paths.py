"""Session paths responsibility extracted without changing saved data contracts.

Position/attitude frames, axis aliases, units, timestamps, schema and hashes
retain the producer/recording contract. This module does not convert camera
origins, optical RDF, baseline-center FRD or world NED implicitly.
"""
from __future__ import annotations

import threading
import time
from datetime import datetime
from pathlib import Path


_SESSION_ID_LOCK = threading.Lock()


_LAST_SESSION_ID_NS = 0


def session_id_now() -> str:
    """Return a sortable session id that cannot collide within one process."""
    global _LAST_SESSION_ID_NS
    with _SESSION_ID_LOCK:
        now_ns = max(time.time_ns(), _LAST_SESSION_ID_NS + 1)
        _LAST_SESSION_ID_NS = now_ns
    stamp = datetime.fromtimestamp(now_ns / 1_000_000_000)
    return f"{stamp:%Y%m%d_%H%M%S}_{now_ns % 1_000_000_000:09d}"


def session_dir(root: Path, session_id: str | None = None) -> Path:
    """Return the canonical v2 session directory directly below ``root``.

    Older Workstation builds inserted an extra ``sessions`` component.  New
    writers intentionally use ``recordings/<session-id>``; read/discovery
    paths keep explicit legacy fallback support instead of perpetuating the
    extra directory for new evidence.
    """

    selected = str(session_id or session_id_now())
    if (
        selected in {"", ".", "..", "sessions"}
        or selected.startswith(".")
        or Path(selected).name != selected
        or "\\" in selected
        or "\x00" in selected
    ):
        raise ValueError("session_id must be one safe non-reserved path component")
    return root / selected


def legacy_session_dir(root: Path, session_id: str) -> Path:
    """Return the pre-v2 ``recordings/sessions/<session-id>`` location."""

    return root / "sessions" / session_id


def resolve_session_dir(root: Path, session_id: str) -> Path:
    """Resolve an existing v2/legacy session or the canonical new location.

    A duplicate ID across the two layouts and any session-directory symlink
    are rejected rather than selecting an ambiguous evidence owner.
    """

    root = Path(root)
    legacy_root = root / "sessions"
    if root.is_symlink() or legacy_root.is_symlink():
        raise ValueError("recordings and legacy container directories must not be symlinks")
    canonical = session_dir(root, session_id)
    legacy = legacy_session_dir(root, session_id)
    for candidate in (canonical, legacy):
        if candidate.is_symlink():
            raise ValueError(f"session directories must not be symlinks: {candidate}")
    canonical_exists = canonical.exists()
    legacy_exists = legacy.exists()
    if canonical_exists and legacy_exists:
        raise FileExistsError(
            f"direct/legacy session ID collision: {session_id}"
        )
    if canonical_exists:
        return canonical
    if legacy_exists:
        return legacy
    return canonical


def raw_video_paths(root: Path, session_id: str | None = None) -> dict[str, Path]:
    output_dir = session_dir(root, session_id)
    return {
        "left": output_dir / "video" / "raw_left.mp4",
        "right": output_dir / "video" / "raw_right.mp4",
    }


def format_video_paths(paths: dict[str, Path]) -> str:
    if not paths:
        return "저장 경로 없음"
    return ", ".join(f"{camera}={path}" for camera, path in paths.items())
