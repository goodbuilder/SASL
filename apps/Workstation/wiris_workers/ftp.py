"""WIRIS ftp workers with their original state, Qt signals and I/O lifetime.

Left/right source identity, image bytes, metadata and source clocks retain
their producer contracts; no NED/FRD/RDF reinterpretation is performed here.
"""
from __future__ import annotations

import ftplib
import os
from collections.abc import Mapping, Sequence
from pathlib import Path
from PyQt5 import QtCore, QtGui


class WirisOzFtpSyncWorker(QtCore.QObject):
    """Download the oldest not-yet-synced OZ stills from both cameras.

    Unlike ``WirisPnPPhotoCaptureWorker``, this worker does not trigger a
    capture -- it only lists each camera's FTP directory (sorted by that
    camera's own MDTM order, since the two camera clocks can differ by
    hours) and downloads the next ``fetch_counts[label]`` files that are not
    already in ``downloaded_remote_names[label]``. Pairing left/right by
    shared trigger command is the caller's job (see the capture ledger in
    ``calibration_controller.py``): this worker only guarantees each side's
    files come back in that side's own upload order, so the caller can align
    the k-th new left file with the k-th new right file.

    This worker never deletes the camera's copy. Each downloaded file is
    checked for a decodable image (a truncated/corrupt transfer must not
    cost the only remaining copy), and the result reports ``valid`` per file
    so the caller can decide whether the whole batch is trustworthy before
    ever asking a camera to delete anything (see
    ``WirisFtpBatchDeleteWorker``, a separate, later step).
    """

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    #: WIRIS Enterprise names the OZ visible still with this exact suffix
    #: (radiometric/superresolution/HR stills get their own distinct
    #: suffixes). CPTR can produce several files per trigger depending on
    #: which SIxx save-format toggles are on, so filtering by extension
    #: alone is not enough -- this pins the count-per-round invariant.
    OZ_VISIBLE_SUFFIX = "-visible.jpg"

    def __init__(
        self,
        target_infos: Sequence[tuple[str, str, int, float]],
        *,
        ftp_settings_by_label: Mapping[str, Mapping[str, object]],
        storage_root_by_label: Mapping[str, str],
        ftp_output_dir: str | os.PathLike[str],
        fetch_counts: Mapping[str, int],
        downloaded_remote_names: Mapping[str, Sequence[str]] | None = None,
    ) -> None:
        super().__init__()
        self.target_infos = tuple(
            (str(label), str(host), int(port), max(0.1, float(timeout_s)))
            for label, host, port, timeout_s in target_infos
        )
        self.ftp_settings_by_label: dict[str, dict[str, object]] = {
            str(label): dict(config) for label, config in ftp_settings_by_label.items()
        }
        # WIRIS Enterprise has no fixed image directory: stills land under
        # /<storage root>/<session folder>/img/, and a new session folder
        # appears every time Ethernet Mode (re)starts or the camera reboots.
        # storage_root_by_label carries the SD_CARD/SSD/FLASH_DRIVE-derived
        # FTP root (e.g. "ssd") queried via GILC just before this worker
        # runs; session folders under it are discovered fresh each sync.
        self.storage_root_by_label: dict[str, str] = {
            str(label): str(root) for label, root in storage_root_by_label.items()
        }
        self.ftp_output_dir = Path(ftp_output_dir)
        self.fetch_counts = {str(label): max(0, int(count)) for label, count in fetch_counts.items()}
        # Identifiers are "<session>/<name>", not bare filenames -- two
        # different session folders can otherwise contain a same-named file.
        self.downloaded_remote_names: dict[str, set[str]] = {
            str(label): {str(name) for name in names}
            for label, names in (downloaded_remote_names or {}).items()
        }

    def _session_folders(self, ftp: ftplib.FTP, storage_root: str) -> list[str]:
        try:
            ftp.cwd(f"/{storage_root}")
        except ftplib.all_errors:
            return []
        # Session folder names are "YY-MM-DD_HH-MM", so lexicographic sort
        # is chronological -- no reliance on either camera's own clock.
        return sorted(ftp.nlst())

    def _sorted_remote_files_across_sessions(
        self, ftp: ftplib.FTP, storage_root: str
    ) -> list[tuple[str, str]]:
        """Return (session, name) pairs in camera-side upload order.

        Concatenates every session folder's ``img/`` listing, oldest session
        first, instead of trusting only the newest session folder -- a
        capture batch spanning a camera reboot has its earlier rounds sitting
        in a now-previous session folder, which a "just look at the latest
        folder" sync would silently miss.
        """

        pairs: list[tuple[str, str]] = []
        for session in self._session_folders(ftp, storage_root):
            try:
                ftp.cwd(f"/{storage_root}/{session}/img")
            except ftplib.all_errors:
                continue

            def _modified_at(name: str) -> str:
                try:
                    response = ftp.voidcmd(f"MDTM {name}")
                    return response.strip().rsplit(" ", 1)[-1]
                except ftplib.all_errors:
                    return ""

            names = sorted(
                (name for name in ftp.nlst() if name.lower().endswith(self.OZ_VISIBLE_SUFFIX)),
                key=_modified_at,
            )
            pairs.extend((session, name) for name in names)
        return pairs

    def _sync_one(self, label: str, host: str, port: int, timeout_s: float) -> dict[str, object]:
        take = self.fetch_counts.get(label, 0)
        already = self.downloaded_remote_names.get(label, set())
        storage_root = self.storage_root_by_label.get(label, "")
        if take <= 0 or not storage_root:
            return {"items": []}
        config = self.ftp_settings_by_label.get(label, {})
        ftp_port = int(config.get("port", 21))
        user = str(config.get("user") or "anonymous")
        password = str(config.get("password", ""))
        ftp_timeout_s = max(0.1, float(config.get("timeout_s", timeout_s)))
        self.ftp_output_dir.mkdir(parents=True, exist_ok=True)
        side_output_dir = self.ftp_output_dir / label
        side_output_dir.mkdir(parents=True, exist_ok=True)
        items: list[dict[str, object]] = []
        with ftplib.FTP() as ftp:
            ftp.connect(host, ftp_port, timeout=ftp_timeout_s)
            ftp.login(user=user, passwd=password)
            all_files = self._sorted_remote_files_across_sessions(ftp, storage_root)
            pending = [
                (session, name)
                for session, name in all_files
                if f"{session}/{name}" not in already
            ]
            for session, name in pending[:take]:
                ftp.cwd(f"/{storage_root}/{session}/img")
                identifier = f"{session}/{name}"
                destination = side_output_dir / f"{session}_{name}"
                with destination.open("wb") as handle:
                    ftp.retrbinary(f"RETR {name}", handle.write)
                valid = not QtGui.QImage(str(destination)).isNull()
                items.append({"name": identifier, "path": str(destination), "valid": valid})
        return {"items": items}

    @QtCore.pyqtSlot()
    def run(self) -> None:
        results: dict[str, dict[str, object]] = {}
        try:
            for label, host, port, timeout_s in self.target_infos:
                try:
                    results[label] = self._sync_one(label, host, port, timeout_s)
                except Exception as exc:
                    results[label] = {"items": [], "error": str(exc)}
            self.completed.emit(results)
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            self.finished.emit()


class WirisFtpBatchDeleteWorker(QtCore.QObject):
    """Delete a specific, already-confirmed set of remote files per camera.

    Deliberately separate from ``WirisOzFtpSyncWorker``: deletion is only
    ever issued for names the caller has already downloaded *and* validated
    (decodable image) *and* successfully paired left/right for the same
    trigger round. This never deletes on the strength of the download alone.
    """

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        target_infos: Sequence[tuple[str, str, int, float]],
        *,
        ftp_settings_by_label: Mapping[str, Mapping[str, object]],
        names_by_label: Mapping[str, Sequence[str]],
    ) -> None:
        super().__init__()
        self.target_infos = tuple(
            (str(label), str(host), int(port), max(0.1, float(timeout_s)))
            for label, host, port, timeout_s in target_infos
        )
        self.ftp_settings_by_label: dict[str, dict[str, object]] = {
            str(label): dict(config) for label, config in ftp_settings_by_label.items()
        }
        self.names_by_label: dict[str, list[str]] = {
            str(label): [str(name) for name in names] for label, names in names_by_label.items()
        }

    def _delete_one(self, label: str, host: str, port: int, timeout_s: float) -> dict[str, object]:
        names = self.names_by_label.get(label) or []
        if not names:
            return {"deleted_names": [], "delete_errors": {}}
        config = self.ftp_settings_by_label.get(label, {})
        ftp_port = int(config.get("port", 21))
        user = str(config.get("user") or "anonymous")
        password = str(config.get("password", ""))
        remote_dir = str(config.get("remote_dir") or "/")
        ftp_timeout_s = max(0.1, float(config.get("timeout_s", timeout_s)))
        deleted_names: list[str] = []
        delete_errors: dict[str, str] = {}
        with ftplib.FTP() as ftp:
            ftp.connect(host, ftp_port, timeout=ftp_timeout_s)
            ftp.login(user=user, passwd=password)
            if remote_dir:
                ftp.cwd(remote_dir)
            for name in names:
                try:
                    ftp.delete(name)
                    deleted_names.append(name)
                except ftplib.all_errors as exc:
                    delete_errors[name] = str(exc)
        return {"deleted_names": deleted_names, "delete_errors": delete_errors}

    @QtCore.pyqtSlot()
    def run(self) -> None:
        results: dict[str, dict[str, object]] = {}
        try:
            for label, host, port, timeout_s in self.target_infos:
                try:
                    results[label] = self._delete_one(label, host, port, timeout_s)
                except Exception as exc:
                    results[label] = {"deleted_names": [], "delete_errors": {}, "error": str(exc)}
            self.completed.emit(results)
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            self.finished.emit()

# Keep the named legacy class identity; implementation ownership is here.
WirisOzFtpSyncWorker.__module__ = "Workstation.wiris_runtime_workers"
WirisFtpBatchDeleteWorker.__module__ = "Workstation.wiris_runtime_workers"
