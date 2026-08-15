"""Asynchronous workstation recording-session persistence workers."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

from PyQt5 import QtCore

from Workstation.workstation_core.recording_session import SpoolRows, save_session_files


class SessionSaveWorker(QtCore.QObject):
    saved = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        output_dir: Path,
        metadata: dict[str, object],
        pair_rows: Sequence[dict[str, object]],
        camera_rows: dict[str, SpoolRows],
        px4_rows: dict[str, SpoolRows],
        gnss_base_rows: SpoolRows | None = None,
        extrinsics_comparison_rows: SpoolRows | None = None,
    ) -> None:
        super().__init__()
        self.output_dir = output_dir
        self.metadata = metadata
        self.pair_rows = pair_rows
        self.camera_rows = camera_rows
        self.px4_rows = px4_rows
        self.gnss_base_rows = gnss_base_rows
        self.extrinsics_comparison_rows = extrinsics_comparison_rows

    def _emit_if_alive(self, signal_name: str, *args: object) -> bool:
        """Drop a completed save notification only after Qt application teardown."""

        try:
            getattr(self, signal_name).emit(*args)
        except RuntimeError:
            return False
        return True

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            saved = save_session_files(
                self.output_dir,
                metadata=self.metadata,
                pair_rows=self.pair_rows,
                camera_rows=self.camera_rows,
                px4_rows=self.px4_rows,
                gnss_base_rows=self.gnss_base_rows,
                extrinsics_comparison_rows=self.extrinsics_comparison_rows,
            )
        except Exception as exc:
            self._emit_if_alive("failed", str(exc))
        else:
            self._emit_if_alive("saved", saved)
        finally:
            self._emit_if_alive("finished")
