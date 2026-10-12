"""WIRIS capture workers with their original state, Qt signals and I/O lifetime.

Left/right source identity, image bytes, metadata and source clocks retain
their producer contracts; no NED/FRD/RDF reinterpretation is performed here.
"""
from __future__ import annotations

import ftplib
import hashlib
import os
import posixpath
import re
import tempfile
import threading
import time
from collections.abc import Mapping, Sequence
from pathlib import Path, PurePosixPath
from PyQt5 import QtCore
from Workstation.workstation_core.wiris_zoom import LINE_TERMINATOR, build_wiris_command
from . import io as _wiris_io
from . import policy as _wire_policy


class WirisPnPPhotoCaptureWorker(QtCore.QObject):
    """Enable one visible still channel and wait for camera-local storage.

    The WIRIS camera uses the same ``CPTR``/``ICPT`` lifecycle for the normal
    visible operating-zoom still and the visible HR still.  The channel's
    image-storage command is injected so the two evidence paths can remain
    separate while sharing the timing and completion contract.
    """

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()
    progress = QtCore.pyqtSignal(object)

    #: JPG/TIFF are the only still formats WIRIS Enterprise stores for CPTR.
    FTP_IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".tif", ".tiff")
    FTP_STORAGE_ROOTS = ("ssd", "sdcard", "flashd")
    FTP_CONNECT_ATTEMPTS = 2

    def __init__(
        self,
        target_infos: Sequence[tuple[str, str, int, float]],
        *,
        capture_id: str,
        image_enable_command: str = "SIHR TRUE",
        capture_channel: str = "hr",
        capture_purpose: str = "unspecified",
        completion_timeout_s: float = 10.0,
        ftp_enabled: bool = False,
        ftp_settings_by_label: Mapping[str, Mapping[str, object]] | None = None,
        ftp_output_dir: str | os.PathLike[str] | None = None,
    ) -> None:
        super().__init__()
        self.target_infos = tuple(
            (str(label), str(host), int(port), max(0.1, float(timeout_s)))
            for label, host, port, timeout_s in target_infos
        )
        self.capture_id = str(capture_id)
        if (
            not self.capture_id
            or Path(self.capture_id).name != self.capture_id
            or self.capture_id in {".", ".."}
            or "\\" in self.capture_id
            or "\x00" in self.capture_id
        ):
            raise ValueError("capture_id must be one safe path component")
        self.image_enable_command = str(image_enable_command).strip()
        self.capture_channel = str(capture_channel).strip().lower() or "hr"
        self.capture_purpose = str(capture_purpose or "unspecified")
        self.completion_timeout_s = max(0.0, float(completion_timeout_s))
        # FTP reaches the same WIRIS Enterprise host as the TCP command
        # channel above; only the port differs. Downloading is best-effort:
        # a failure here never turns an otherwise-successful CPTR capture
        # into a failed one, it only leaves ftp_downloaded False so the
        # operator falls back to the existing manual import path.
        # Each camera runs its own local FTP account, so the connection
        # settings are looked up per label (``left``/``right``) rather than
        # shared across both cameras.
        self.ftp_enabled = bool(ftp_enabled)
        self.ftp_settings_by_label: dict[str, dict[str, object]] = {
            str(label): dict(config) for label, config in (ftp_settings_by_label or {}).items()
        }
        capture_family = (
            "HR"
            if "SIHR" in self.image_enable_command.upper()
            else "OZ"
            if "SIVI" in self.image_enable_command.upper()
            else ""
        )
        if self.capture_channel in {"calib", "calib_oz"} and capture_family:
            capture_root = f"calib_capture/{capture_family}"
        else:
            capture_root = f"{self.capture_channel}_capture"
        self.ftp_session_root = (
            None
            if ftp_output_dir is None
            else Path(ftp_output_dir).expanduser().resolve(strict=False)
        )
        self.ftp_output_dir = (
            None
            if self.ftp_session_root is None
            else self.ftp_session_root / capture_root
        )

    @staticmethod
    def _capture_status(response: str) -> str:
        """Classify one ICPT response without treating garbage as success.

        WIRIS firmwares return either a bare boolean/0/1 or echo ``ICPT``
        before it.  Completion must be explicit; empty, error, and malformed
        responses are invalid rather than implicitly idle.
        """

        text = str(response or "").replace("\x00", "").strip().upper()
        if not text:
            return "invalid"
        tokens = re.findall(r"[A-Z]+|[01]", text)
        if any(
            token in {"ERR", "ERROR", "FAILED", "FAIL", "INVALID", "UNKNOWN", "NAK", "NACK"}
            for token in tokens
        ):
            return "invalid"
        while tokens and tokens[0] in {"OK", "ICPT"}:
            tokens.pop(0)
        if len(tokens) != 1:
            return "invalid"
        if tokens[0] in {"0", "FALSE", "IDLE", "DONE", "READY", "COMPLETE", "COMPLETED"}:
            return "complete"
        if tokens[0] in {"1", "TRUE", "BUSY", "CAPTURING"}:
            return "busy"
        return "invalid"

    @classmethod
    def _capture_busy(cls, response: str) -> bool:
        return cls._capture_status(response) == "busy"

    @staticmethod
    def _send(host: str, port: int, timeout_s: float, command_text: str) -> str:
        return _wiris_io.send_tcp_line(
            build_wiris_command(
                host=host,
                port=port,
                command=command_text,
                timeout_s=timeout_s,
                terminator=LINE_TERMINATOR,
            )
        )

    def _preferred_ftp_suffix(self) -> str | None:
        command = self.image_enable_command.upper()
        if "SIHR" in command:
            return "-highres"
        if "SIVI" in command:
            return "-visible"
        return None

    def _all_ftp_image_names(self, names: Sequence[object]) -> list[str]:
        return [
            PurePosixPath(str(name)).name
            for name in names
            if Path(str(name)).suffix.lower() in self.FTP_IMAGE_EXTENSIONS
        ]

    def _image_names(self, names: Sequence[object]) -> list[str]:
        image_names = self._all_ftp_image_names(names)
        preferred_suffix = self._preferred_ftp_suffix()
        if preferred_suffix is None:
            return image_names
        preferred = [
            name
            for name in image_names
            if Path(name).stem.lower().endswith(preferred_suffix)
        ]
        # Some firmware versions use a generic filename.  Preserve the
        # existing extension-only fallback when the channel-specific suffix
        # is not present in the selected directory.
        return preferred or image_names

    @staticmethod
    def _ftp_mtime(ftp: ftplib.FTP, name: str) -> str:
        try:
            response = ftp.voidcmd(f"MDTM {name}")
            return response.strip().rsplit(" ", 1)[-1]
        except ftplib.all_errors:
            return ""

    def _latest_ftp_file_name(self, ftp: ftplib.FTP) -> str:
        names = self._image_names(ftp.nlst())
        if not names:
            raise FileNotFoundError("WIRIS FTP remote directory has no JPG/TIFF images")

        return max(names, key=lambda name: self._ftp_mtime(ftp, name))

    def _connect_ftp(
        self,
        host: str,
        port: int,
        user: str,
        password: str,
        timeout_s: float,
    ) -> ftplib.FTP:
        last_error: Exception | None = None
        for attempt in range(self.FTP_CONNECT_ATTEMPTS):
            ftp = ftplib.FTP()
            try:
                ftp.connect(host, port, timeout=timeout_s)
                ftp.login(user=user, passwd=password)
                return ftp
            except Exception as exc:
                last_error = exc
                try:
                    ftp.close()
                except Exception:
                    pass
                if attempt + 1 < self.FTP_CONNECT_ATTEMPTS:
                    # WIRIS FTP can take one full control timeout to become
                    # ready immediately after CPTR; a second connection
                    # attempt avoids turning that transient delay into a
                    # false download failure.
                    time.sleep(0.25)
        if last_error is not None:
            raise last_error
        raise RuntimeError("WIRIS FTP connection failed without an exception")

    def _ftp_image_candidates(
        self,
        ftp: ftplib.FTP,
        remote_dir: str,
    ) -> list[tuple[str, int, str, str]]:
        requested_dir = posixpath.normpath(str(remote_dir or "/").strip() or "/")
        if not requested_dir.startswith("/"):
            requested_dir = f"/{requested_dir}"
        locations: list[tuple[str, list[str]]] = []
        if requested_dir == "/":
            # WIRIS stores captures below /<storage>/<session>/img rather
            # than directly in /.  A fixed /ssd/... path becomes stale after
            # an Ethernet restart, so discover current sessions when the
            # operator leaves Remote dir at the default root.
            for storage_root in self.FTP_STORAGE_ROOTS:
                try:
                    ftp.cwd(f"/{storage_root}")
                    sessions = ftp.nlst()
                except TimeoutError:
                    raise
                except ftplib.all_errors:
                    continue
                for session in sessions:
                    session_name = PurePosixPath(str(session)).name
                    if not session_name or session_name in {".", ".."}:
                        continue
                    image_dir = f"/{storage_root}/{session_name}/img"
                    try:
                        ftp.cwd(image_dir)
                        names = self._all_ftp_image_names(ftp.nlst())
                    except TimeoutError:
                        raise
                    except ftplib.all_errors:
                        continue
                    if names:
                        locations.append((image_dir, names))
        else:
            ftp.cwd(requested_dir)
            locations.append((requested_dir, self._all_ftp_image_names(ftp.nlst())))

        candidates: list[tuple[str, int, str, str]] = []
        for order, (image_dir, names) in enumerate(locations):
            ftp.cwd(image_dir)
            for name in names:
                candidates.append(
                    (self._ftp_mtime(ftp, name), order, image_dir, name)
                )
        if not candidates:
            if requested_dir == "/":
                raise FileNotFoundError(
                    "WIRIS FTP storage sessions contain no JPG/TIFF images "
                    "under /ssd/*/img, /sdcard/*/img, or /flashd/*/img"
                )
            raise FileNotFoundError(
                f"WIRIS FTP remote directory has no JPG/TIFF images: {requested_dir}"
            )
        return candidates

    @staticmethod
    def _ftp_candidate_token(candidate: tuple[str, int, str, str]) -> str:
        mtime, _order, image_dir, name = candidate
        return f"{image_dir.rstrip('/')}/{name}|{mtime}"

    def _latest_ftp_file_location(
        self,
        ftp: ftplib.FTP,
        remote_dir: str,
        *,
        baseline_tokens: set[str] | None = None,
    ) -> tuple[str, str]:
        candidates = self._ftp_image_candidates(ftp, remote_dir)
        if baseline_tokens is not None:
            candidates = [
                candidate
                for candidate in candidates
                if self._ftp_candidate_token(candidate) not in baseline_tokens
            ]
            if not candidates:
                raise FileNotFoundError(
                    "WIRIS FTP has no new JPG/TIFF image after CPTR; refusing to reuse a stale file"
                )
        preferred_suffix = self._preferred_ftp_suffix()
        preferred_candidates = (
            [
                row
                for row in candidates
                if Path(row[3]).stem.lower().endswith(preferred_suffix)
            ]
            if preferred_suffix is not None
            else []
        )
        selected_candidates = preferred_candidates or candidates
        _mtime, _order, image_dir, name = max(
            selected_candidates,
            key=lambda row: (row[0], row[1]),
        )
        return image_dir, name

    def _snapshot_ftp_files(self, host: str, label: str) -> set[str]:
        config = self.ftp_settings_by_label.get(label, {})
        port = int(config.get("port", 21))
        user = str(config.get("user") or "anonymous")
        password = str(config.get("password", ""))
        remote_dir = str(config.get("remote_dir") or "/")
        timeout_s = max(0.1, float(config.get("timeout_s", 10.0)))
        ftp = self._connect_ftp(host, port, user, password, timeout_s)
        try:
            try:
                candidates = self._ftp_image_candidates(ftp, remote_dir)
            except FileNotFoundError:
                return set()
            return {self._ftp_candidate_token(candidate) for candidate in candidates}
        finally:
            try:
                ftp.quit()
            except Exception:
                ftp.close()

    def _download_latest_ftp_file(
        self,
        host: str,
        label: str,
        *,
        baseline_tokens: set[str] | None = None,
    ) -> tuple[Path, str]:
        if self.ftp_output_dir is None:
            raise RuntimeError("no local FTP download directory was configured")
        if label not in {"left", "right"}:
            raise ValueError("FTP capture side must be left or right")
        output_root = self.ftp_output_dir.resolve(strict=False)
        output_root.mkdir(parents=True, exist_ok=True)
        config = self.ftp_settings_by_label.get(label, {})
        port = int(config.get("port", 21))
        user = str(config.get("user") or "anonymous")
        password = str(config.get("password", ""))
        remote_dir = str(config.get("remote_dir") or "/")
        timeout_s = max(0.1, float(config.get("timeout_s", 10.0)))
        deadline_s = time.monotonic() + max(5.0, timeout_s * 2.0)
        last_error: Exception | None = None
        while True:
            ftp = None
            try:
                ftp = self._connect_ftp(host, port, user, password, timeout_s)
                image_dir, latest_name = self._latest_ftp_file_location(
                    ftp,
                    remote_dir,
                    baseline_tokens=baseline_tokens,
                )
                ftp.cwd(image_dir)
                side_output_dir = output_root / label
                side_output_dir.mkdir(parents=True, exist_ok=True)
                resolved_side = side_output_dir.resolve(strict=True)
                resolved_side.relative_to(output_root)
                destination = resolved_side / f"{self.capture_id}_{latest_name}"
                digest = hashlib.sha256()
                size_bytes = 0
                descriptor, temporary_name = tempfile.mkstemp(
                    dir=resolved_side,
                    prefix=f".{destination.name}.",
                    suffix=".part",
                )
                temporary = Path(temporary_name)
                try:
                    with os.fdopen(descriptor, "wb") as handle:
                        def write_chunk(chunk: bytes) -> None:
                            nonlocal size_bytes
                            written = handle.write(chunk)
                            if written != len(chunk):
                                raise OSError("short WIRIS FTP download write")
                            digest.update(chunk)
                            size_bytes += len(chunk)

                        ftp.retrbinary(f"RETR {latest_name}", write_chunk)
                        handle.flush()
                        os.fsync(handle.fileno())
                    if size_bytes <= 0:
                        raise OSError("WIRIS FTP downloaded an empty still image")
                    expected_sha256 = digest.hexdigest()
                    if os.path.lexists(destination):
                        if destination.is_symlink() or not destination.is_file():
                            raise OSError("WIRIS FTP destination is not a regular file")
                        existing = destination.read_bytes()
                        if (
                            len(existing) != size_bytes
                            or hashlib.sha256(existing).hexdigest()
                            != expected_sha256
                        ):
                            raise OSError("WIRIS FTP capture destination collision")
                        temporary.unlink()
                    else:
                        os.replace(temporary, destination)
                        directory_descriptor = os.open(resolved_side, os.O_RDONLY)
                        try:
                            os.fsync(directory_descriptor)
                        finally:
                            os.close(directory_descriptor)
                    sealed = destination.read_bytes()
                    if (
                        len(sealed) != size_bytes
                        or hashlib.sha256(sealed).hexdigest() != expected_sha256
                    ):
                        raise OSError("WIRIS FTP sealed-byte verification failed")
                finally:
                    temporary.unlink(missing_ok=True)
                return destination, f"{image_dir.rstrip('/')}/{latest_name}"
            except Exception as exc:
                last_error = exc
            finally:
                if ftp is not None:
                    try:
                        ftp.quit()
                    except Exception:
                        ftp.close()
            if time.monotonic() >= deadline_s:
                break
            time.sleep(0.5)
        if last_error is not None:
            raise last_error
        raise RuntimeError("WIRIS FTP download ended without an exception")

    @QtCore.pyqtSlot()
    def run(self) -> None:
        results: dict[str, dict[str, object]] = {}
        capture_barrier = (
            threading.Barrier(len(self.target_infos))
            if len(self.target_infos) >= 2
            else None
        )

        def capture_one(label: str, host: str, port: int, timeout_s: float) -> None:
            item: dict[str, object] = {
                "state": "failed",
                "side": label,
                "capture_id": self.capture_id,
                "purpose": self.capture_purpose,
                f"{self.capture_channel}_enabled": False,
                "capture_complete": False,
                "stored_on_camera": False,
                "ftp_downloaded": False,
            }
            ftp_baseline_tokens: set[str] | None = None
            ftp_baseline_error = ""

            def send_capture_command(command_text: str) -> str:
                self.progress.emit(
                    {
                        "source": "calibration",
                        "phase": "command",
                        "side": label,
                        "command": command_text,
                        "state": "sending",
                    }
                )
                try:
                    response = self._send(host, port, timeout_s, command_text)
                except Exception as exc:
                    self.progress.emit(
                        {
                            "source": "calibration",
                            "phase": "response",
                            "side": label,
                            "command": command_text,
                            "state": "failed",
                            "response": f"ERROR: {exc}",
                        }
                    )
                    raise
                self.progress.emit(
                    {
                        "source": "calibration",
                        "phase": "response",
                        "side": label,
                        "command": command_text,
                        "state": "received",
                        "response": response or "<no response>",
                    }
                )
                return response

            if self.ftp_enabled:
                try:
                    ftp_baseline_tokens = self._snapshot_ftp_files(host, label)
                except Exception as exc:
                    ftp_baseline_error = str(exc)
            try:
                item[f"{self.capture_channel}_enable_response"] = send_capture_command(
                    self.image_enable_command
                )
                item[f"{self.capture_channel}_enabled"] = True
                # CPTR is the closest timestamp the SDK exposes for a stored
                # still. Synchronize the host dispatches, but never relabel
                # this proxy as the unknown camera exposure timestamp.
                if capture_barrier is not None:
                    _wire_policy._wait_for_peer_or_continue(
                        capture_barrier,
                        timeout_s=max(1.0, timeout_s),
                    )
                item["capture_requested_monotonic_ns"] = time.monotonic_ns()
                item["capture_requested_unix_ns"] = time.time_ns()
                item["capture_pairing_contract"] = (
                    "same_capture_id_and_synchronized_cptr_dispatch_v1"
                )
                item["capture_response"] = send_capture_command("CPTR")
                deadline_s = time.monotonic() + self.completion_timeout_s
                while True:
                    response = send_capture_command("ICPT")
                    item["capture_status_response"] = response
                    capture_status = self._capture_status(response)
                    if capture_status == "complete":
                        item["capture_complete"] = True
                        item["stored_on_camera"] = True
                        item["state"] = "completed"
                        item["capture_completed_monotonic_s"] = time.monotonic()
                        item["capture_completed_unix_ns"] = time.time_ns()
                        break
                    if capture_status == "invalid":
                        item["state"] = "failed"
                        item["error"] = f"invalid ICPT response: {response!r}"
                        break
                    if time.monotonic() >= deadline_s:
                        item["state"] = "timeout"
                        item["error"] = "ICPT completion timeout"
                        break
                    time.sleep(0.25)
            except Exception as exc:
                _wire_policy._abort_barrier(capture_barrier)
                item["error"] = str(exc)
            if item.get("stored_on_camera") and self.ftp_enabled:
                if ftp_baseline_tokens is None:
                    item["ftp_error"] = (
                        "pre-capture FTP snapshot failed"
                        + (f": {ftp_baseline_error}" if ftp_baseline_error else "")
                    )
                else:
                    try:
                        self.progress.emit(
                            {
                                "source": "calibration",
                                "phase": "ftp",
                                "side": label,
                                "state": "downloading",
                                "response": "waiting for new FTP image",
                            }
                        )
                        downloaded, remote_path = self._download_latest_ftp_file(
                            host,
                            label,
                            baseline_tokens=ftp_baseline_tokens,
                        )
                        item["ftp_downloaded"] = True
                        item["ftp_local_path"] = str(downloaded)
                        item["ftp_local_path_usage_contract"] = (
                            "historical_provenance_only"
                        )
                        if self.ftp_session_root is not None:
                            item["ftp_local_relative_path"] = downloaded.relative_to(
                                self.ftp_session_root
                            ).as_posix()
                        item["ftp_remote_path"] = remote_path
                        item["ftp_remote_filename"] = PurePosixPath(
                            remote_path
                        ).name
                        item["ftp_size_bytes"] = downloaded.stat().st_size
                        item["ftp_sha256"] = hashlib.sha256(
                            downloaded.read_bytes()
                        ).hexdigest()
                        item["zoom_readback"] = None
                        item["zoom_readback_status"] = (
                            "not_available_in_capture_worker"
                        )
                        item["ftp_byte_verification"] = (
                            "size_and_sha256_after_atomic_rename"
                        )
                        self.progress.emit(
                            {
                                "source": "calibration",
                                "phase": "ftp",
                                "side": label,
                                "state": "downloaded",
                                "response": f"remote={remote_path}; local={downloaded}",
                                "path": str(downloaded),
                            }
                        )
                    except Exception as exc:
                        item["ftp_downloaded"] = False
                        item["ftp_error"] = str(exc)
                        self.progress.emit(
                            {
                                "source": "calibration",
                                "phase": "ftp",
                                "side": label,
                                "state": "failed",
                                "response": str(exc),
                            }
                        )
            item["finished_unix_ns"] = time.time_ns()
            results[label] = item

        threads = [
            threading.Thread(
                target=capture_one,
                args=(label, host, port, timeout_s),
                name=f"wiris-pnp-photo-{self.capture_id}-{label}",
            )
            for label, host, port, timeout_s in self.target_infos
        ]
        try:
            for thread in threads:
                thread.start()
            max_ftp_timeout_s = max(
                (float(config.get("timeout_s", 10.0)) for config in self.ftp_settings_by_label.values()),
                default=10.0,
            )
            ftp_wait_budget_s = max_ftp_timeout_s * 2.0 if self.ftp_enabled else 0.0
            maximum_wait_s = self.completion_timeout_s + ftp_wait_budget_s + sum(
                max(1.0, timeout_s) * 3.0
                for _label, _host, _port, timeout_s in self.target_infos[:1]
            )
            for thread in threads:
                thread.join(timeout=maximum_wait_s)
            for label, _host, _port, _timeout_s in self.target_infos:
                results.setdefault(
                    label,
                    {
                        "state": "failed",
                        "error": "capture worker thread timeout",
                        f"{self.capture_channel}_enabled": False,
                        "capture_complete": False,
                        "stored_on_camera": False,
                        "ftp_downloaded": False,
                        "finished_unix_ns": time.time_ns(),
                    },
                )
            camera_items = list(results.values())
            request_times = {
                label: int(item["capture_requested_monotonic_ns"])
                for label, item in results.items()
                if item.get("capture_requested_monotonic_ns") is not None
            }
            request_delta_s = (
                (request_times["right"] - request_times["left"]) * 1.0e-9
                if {"left", "right"} <= set(request_times)
                else None
            )
            self.completed.emit(
                {
                    "capture_id": self.capture_id,
                    "capture_channel": self.capture_channel,
                    "image_enable_command": self.image_enable_command,
                    "commands": (self.image_enable_command, "CPTR", "ICPT"),
                    "pair_time_basis": "host-cptr-dispatch-monotonic",
                    "pair_request_delta_s": request_delta_s,
                    "camera_exposure_timestamp_known": False,
                    "cameras": results,
                    "stored_on_camera": bool(camera_items)
                    and all(item.get("stored_on_camera") for item in camera_items),
                    "ftp_downloaded": self.ftp_enabled
                    and bool(camera_items)
                    and all(item.get("ftp_downloaded") for item in camera_items),
                }
            )
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            self.finished.emit()


class WirisOZPhotoCaptureWorker(WirisPnPPhotoCaptureWorker):
    """Capture a normal visible operating-zoom still on both cameras."""

    def __init__(
        self,
        target_infos: Sequence[tuple[str, str, int, float]],
        *,
        capture_id: str,
        completion_timeout_s: float = 10.0,
        ftp_enabled: bool = False,
        ftp_settings_by_label: Mapping[str, Mapping[str, object]] | None = None,
        ftp_output_dir: str | os.PathLike[str] | None = None,
    ) -> None:
        super().__init__(
            target_infos,
            capture_id=capture_id,
            image_enable_command="SIVI TRUE",
            capture_channel="oz",
            completion_timeout_s=completion_timeout_s,
            ftp_enabled=ftp_enabled,
            ftp_settings_by_label=ftp_settings_by_label,
            ftp_output_dir=ftp_output_dir,
        )

# Keep the named legacy class identity; implementation ownership is here.
WirisPnPPhotoCaptureWorker.__module__ = "Workstation.wiris_runtime_workers"
WirisOZPhotoCaptureWorker.__module__ = "Workstation.wiris_runtime_workers"
