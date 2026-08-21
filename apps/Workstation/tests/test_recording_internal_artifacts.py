from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

from Workstation.recording_controller import RecordingController


def test_session_owned_artifact_prefers_reference_registrar_with_producer_integrity(
    tmp_path: Path,
) -> None:
    calls: list[tuple[object, dict[str, object]]] = []

    def register(path: object, **kwargs: object) -> dict[str, object]:
        calls.append((path, dict(kwargs)))
        return {"relative_path": "ulog/left/flight.ulg"}

    def unexpected_import(_path: object, **_kwargs: object) -> None:
        raise AssertionError("session-owned bytes must not be copied through external import")

    panel = SimpleNamespace(
        register_workstation_session_artifact=register,
        import_workstation_session_artifact=unexpected_import,
    )
    path = tmp_path / "session" / "ulog" / "left" / "flight.ulg"

    result = RecordingController(panel)._register_session_owned_artifact(
        path,
        role="ulog/left",
        media_type="application/vnd.px4.ulog",
        metadata={"source_id": "left"},
        expected_size_bytes=123,
        expected_sha256="a" * 64,
    )

    assert result == {"relative_path": "ulog/left/flight.ulg"}
    assert calls == [
        (
            path,
            {
                "role": "ulog/left",
                "media_type": "application/vnd.px4.ulog",
                "metadata": {"source_id": "left"},
                "expected_size_bytes": 123,
                "expected_sha256": "a" * 64,
            },
        )
    ]


def test_session_owned_artifact_keeps_legacy_external_import_fallback(tmp_path: Path) -> None:
    calls: list[tuple[object, dict[str, object]]] = []

    def legacy_import(path: object, **kwargs: object) -> str:
        calls.append((path, dict(kwargs)))
        return "legacy"

    panel = SimpleNamespace(import_workstation_session_artifact=legacy_import)
    path = tmp_path / "session" / "raw" / "target.jsonl"

    result = RecordingController(panel)._register_session_owned_artifact(
        path,
        role="target_drone_raw_mavlink_jsonl",
        media_type="application/x-ndjson",
        metadata={"source_id": "target_drone"},
        expected_size_bytes=42,
        expected_sha256="b" * 64,
    )

    assert result == "legacy"
    assert calls == [
        (
            path,
            {
                "role": "target_drone_raw_mavlink_jsonl",
                "media_type": "application/x-ndjson",
                "metadata": {"source_id": "target_drone"},
            },
        )
    ]
