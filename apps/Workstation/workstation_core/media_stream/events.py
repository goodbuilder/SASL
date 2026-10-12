"""events owner extracted from the Workstation media runtime.

Existing queues, locks, clocks, source-frame identity and policies are preserved.
Native pixels use image u/v and RGB/BGR/NV12; no world/body transform is introduced.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any
from .configuration import normalize_stream_backend, recording_integrity_contract


def _emit_status(output_queue, text: str) -> None:
    output_queue.put(("status", text))


def _emit_terminal_exit(output_queue, reason: str) -> None:
    output_queue.put(("terminal_exit", {"reason": str(reason)}))


def _emit_decoder_fallback(
    output_queue,
    *,
    reason: str,
    recording_restart_required: bool,
) -> None:
    output_queue.put(
        (
            "decoder_fallback",
            {
                "reason": str(reason),
                "recording_restart_required": bool(recording_restart_required),
            },
        )
    )


def _emit_recording_finalized(
    output_queue,
    *,
    path: str,
    backend: str,
    success: bool,
    detail: str,
    contract: dict[str, str | bool] | None = None,
    metrics: dict[str, int | float | str | bool] | None = None,
) -> None:
    payload: dict[str, Any] = {
        "path": str(Path(path).expanduser()),
        "stream_backend": normalize_stream_backend(backend),
        "success": bool(success),
        "detail": str(detail),
        **(contract or recording_integrity_contract(backend)),
    }
    if metrics:
        payload.update(metrics)
    output_queue.put(("recording_finalized", payload))
