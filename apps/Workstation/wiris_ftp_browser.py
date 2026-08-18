"""Standalone FTP browser dialog for exploring a WIRIS camera's onboard storage.

Lets an operator navigate the camera's FTP directory tree, download individual
files on demand, and copy the currently browsed path back into Settings as the
"FTP remote dir" value -- instead of guessing that path blind.
"""

from __future__ import annotations

import ftplib
import posixpath
import re

from PyQt5 import QtCore, QtWidgets

_UNIX_LIST_RE = re.compile(
    r"^(?P<perms>[dl\-])\S*\s+\d+\s+\S+\s+\S+\s+(?P<size>\d+)\s+\S+\s+\S+\s+\S+\s+(?P<name>.+)$"
)
_DOS_LIST_RE = re.compile(r"^(?P<date>\S+)\s+(?P<time>\S+)\s+(?P<dirmark><DIR>|\d+)\s+(?P<name>.+)$")


def _parse_list_line(line: str) -> dict[str, object] | None:
    line = line.rstrip("\r\n")
    if not line.strip():
        return None
    match = _UNIX_LIST_RE.match(line)
    if match:
        name = match.group("name").strip()
        if " -> " in name:
            name = name.split(" -> ", 1)[0]
        if name in (".", ".."):
            return None
        return {"name": name, "is_dir": match.group("perms") == "d", "size": int(match.group("size"))}
    match = _DOS_LIST_RE.match(line)
    if match:
        name = match.group("name").strip()
        if name in (".", ".."):
            return None
        is_dir = match.group("dirmark") == "<DIR>"
        size = None if is_dir else int(match.group("dirmark"))
        return {"name": name, "is_dir": is_dir, "size": size}
    return None


class _FtpListWorker(QtCore.QObject):
    """Connects fresh, lists one directory, and disconnects."""

    listed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, *, host: str, port: int, user: str, password: str, timeout_s: float, path: str) -> None:
        super().__init__()
        self.host = host
        self.port = port
        self.user = user
        self.password = password
        self.timeout_s = timeout_s
        self.path = path

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            with ftplib.FTP() as ftp:
                ftp.connect(self.host, self.port, timeout=self.timeout_s)
                ftp.login(user=self.user, passwd=self.password)
                if self.path:
                    ftp.cwd(self.path)
                current_path = ftp.pwd()
                entries = self._list(ftp)
            self.listed.emit({"path": current_path, "entries": entries})
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            self.finished.emit()

    @staticmethod
    def _list(ftp: ftplib.FTP) -> list[dict[str, object]]:
        try:
            entries = []
            for name, facts in ftp.mlsd():
                if name in (".", ".."):
                    continue
                size_text = str(facts.get("size", ""))
                entries.append(
                    {
                        "name": name,
                        "is_dir": facts.get("type") in ("dir", "cdir", "pdir"),
                        "size": int(size_text) if size_text.isdigit() else None,
                    }
                )
            if entries:
                return entries
        except Exception:
            pass
        lines: list[str] = []
        try:
            ftp.retrlines("LIST", lines.append)
        except Exception:
            lines = []
        parsed = [entry for entry in (_parse_list_line(line) for line in lines) if entry is not None]
        if parsed:
            return parsed
        try:
            names = ftp.nlst()
        except Exception:
            names = []
        return [{"name": name, "is_dir": None, "size": None} for name in names if name not in (".", "..")]


class _FtpDownloadWorker(QtCore.QObject):
    downloaded = QtCore.pyqtSignal(str)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        host: str,
        port: int,
        user: str,
        password: str,
        timeout_s: float,
        remote_path: str,
        local_path: str,
    ) -> None:
        super().__init__()
        self.host = host
        self.port = port
        self.user = user
        self.password = password
        self.timeout_s = timeout_s
        self.remote_path = remote_path
        self.local_path = local_path

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            remote_dir = posixpath.dirname(self.remote_path) or "/"
            remote_name = posixpath.basename(self.remote_path)
            with ftplib.FTP() as ftp:
                ftp.connect(self.host, self.port, timeout=self.timeout_s)
                ftp.login(user=self.user, passwd=self.password)
                ftp.cwd(remote_dir)
                with open(self.local_path, "wb") as handle:
                    ftp.retrbinary(f"RETR {remote_name}", handle.write)
            self.downloaded.emit(self.local_path)
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            self.finished.emit()


class WirisFtpBrowserDialog(QtWidgets.QDialog):
    """Browse and download files from a WIRIS camera's onboard FTP server."""

    def __init__(
        self,
        parent: QtWidgets.QWidget | None = None,
        *,
        hosts: dict[str, str] | None = None,
        credentials: dict[str, dict[str, object]] | None = None,
        initial_camera: str | None = None,
        port: int = 21,
        user: str = "anonymous",
        password: str = "",
        remote_dir: str = "/",
        timeout_s: float = 10.0,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("WIRIS FTP Browser")
        self.resize(640, 480)
        self.selected_remote_dir: str | None = None
        self.selected_camera_label: str | None = None

        self._hosts = dict(hosts or {})
        # Each WIRIS camera can have its own FTP account, so switching the
        # "Camera" combo also swaps in that camera's own port/user/password
        # instead of leaving the previously-selected camera's credentials in
        # place.
        self._credentials = {label: dict(config) for label, config in (credentials or {}).items()}
        self._thread: QtCore.QThread | None = None
        self._worker: QtCore.QObject | None = None
        self._current_path = remote_dir or "/"

        layout = QtWidgets.QVBoxLayout(self)

        connection_form = QtWidgets.QFormLayout()
        self.camera_combo = QtWidgets.QComboBox()
        for label in self._hosts:
            self.camera_combo.addItem(label)
        self.camera_combo.addItem("Custom")
        self.camera_combo.currentTextChanged.connect(self._apply_camera_selection)
        connection_form.addRow("Camera", self.camera_combo)

        self.host_edit = QtWidgets.QLineEdit(next(iter(self._hosts.values()), ""))
        self.port_spin = QtWidgets.QSpinBox()
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(int(port))
        self.user_edit = QtWidgets.QLineEdit(user)
        self.password_edit = QtWidgets.QLineEdit(password)
        self.password_edit.setEchoMode(QtWidgets.QLineEdit.Password)
        self.timeout_spin = QtWidgets.QDoubleSpinBox()
        self.timeout_spin.setRange(0.5, 60.0)
        self.timeout_spin.setValue(float(timeout_s))
        self.timeout_spin.setSuffix(" s")

        connection_form.addRow("Host", self.host_edit)
        connection_form.addRow("Port", self.port_spin)
        connection_form.addRow("User", self.user_edit)
        connection_form.addRow("Password", self.password_edit)
        connection_form.addRow("Timeout", self.timeout_spin)
        layout.addLayout(connection_form)

        path_row = QtWidgets.QHBoxLayout()
        self.path_edit = QtWidgets.QLineEdit(self._current_path)
        self.up_button = QtWidgets.QPushButton("Up")
        self.up_button.clicked.connect(self._go_up)
        self.connect_button = QtWidgets.QPushButton("Connect / List")
        self.connect_button.clicked.connect(self._start_listing)
        path_row.addWidget(self.path_edit, 1)
        path_row.addWidget(self.up_button)
        path_row.addWidget(self.connect_button)
        layout.addLayout(path_row)

        self.entries_list = QtWidgets.QListWidget()
        self.entries_list.itemActivated.connect(self._activate_item)
        layout.addWidget(self.entries_list, 1)

        self.status_label = QtWidgets.QLabel("")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

        button_row = QtWidgets.QHBoxLayout()
        self.download_button = QtWidgets.QPushButton("Download selected...")
        self.download_button.clicked.connect(self._download_selected)
        self.use_path_button = QtWidgets.QPushButton("Use this path")
        self.use_path_button.setToolTip("Copy the currently browsed directory into the FTP remote dir setting.")
        self.use_path_button.clicked.connect(self._use_current_path)
        self.close_button = QtWidgets.QPushButton("Close")
        self.close_button.clicked.connect(self.reject)
        button_row.addWidget(self.download_button)
        button_row.addWidget(self.use_path_button)
        button_row.addStretch(1)
        button_row.addWidget(self.close_button)
        layout.addLayout(button_row)

        if self._hosts:
            initial_index = self.camera_combo.findText(initial_camera or "")
            self.camera_combo.setCurrentIndex(max(0, initial_index))
            self._apply_camera_selection(self.camera_combo.currentText())
        if self.host_edit.text().strip():
            self._start_listing()

    def _apply_camera_selection(self, label: str) -> None:
        host = self._hosts.get(label)
        if host:
            self.host_edit.setText(host)
        config = self._credentials.get(label)
        if config is not None:
            self.port_spin.setValue(int(config.get("port", self.port_spin.value())))
            self.user_edit.setText(str(config.get("user", self.user_edit.text())))
            self.password_edit.setText(str(config.get("password", self.password_edit.text())))
            self.timeout_spin.setValue(float(config.get("timeout_s", self.timeout_spin.value())))
            remote_dir = str(config.get("remote_dir") or "/")
            self._current_path = remote_dir
            self.path_edit.setText(remote_dir)

    def _busy(self, busy: bool) -> None:
        for widget in (
            self.connect_button,
            self.up_button,
            self.download_button,
            self.use_path_button,
            self.close_button,
            self.camera_combo,
            self.host_edit,
            self.port_spin,
            self.user_edit,
            self.password_edit,
            self.timeout_spin,
            self.path_edit,
        ):
            widget.setEnabled(not busy)

    def done(self, result: int) -> None:
        # QThread aborts the whole process if its C++ object is destroyed
        # while still running (e.g. the dialog closing mid-request). Block
        # until any in-flight FTP worker thread actually finishes -- bounded
        # by ftplib's own connect/read timeout -- before allowing Qt to tear
        # the dialog (and its child QThread) down.
        if self._thread is not None and self._thread.isRunning():
            self._thread.quit()
            self._thread.wait()
        super().done(result)

    def _clear_thread(self) -> None:
        self._thread = None
        self._worker = None
        self._busy(False)

    def _start_listing(self) -> None:
        if self._thread is not None:
            return
        path = self.path_edit.text().strip() or "/"
        self._busy(True)
        self.status_label.setText("Listing...")
        self._thread = QtCore.QThread(self)
        self._worker = _FtpListWorker(
            host=self.host_edit.text().strip(),
            port=self.port_spin.value(),
            user=self.user_edit.text(),
            password=self.password_edit.text(),
            timeout_s=self.timeout_spin.value(),
            path=path,
        )
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.listed.connect(self._on_listed)
        self._worker.failed.connect(self._on_list_failed)
        self._worker.finished.connect(self._thread.quit)
        self._worker.finished.connect(self._worker.deleteLater)
        self._thread.finished.connect(self._thread.deleteLater)
        self._thread.finished.connect(self._clear_thread)
        self._thread.start()

    def _on_listed(self, result: object) -> None:
        data = dict(result or {})
        self._current_path = str(data.get("path") or self._current_path)
        self.path_edit.setText(self._current_path)
        entries = list(data.get("entries") or [])
        entries.sort(key=lambda entry: (entry.get("is_dir") is not True, str(entry.get("name") or "").lower()))
        self.entries_list.clear()
        for entry in entries:
            name = str(entry.get("name") or "")
            is_dir = entry.get("is_dir")
            size = entry.get("size")
            if is_dir is True:
                text = f"[DIR]  {name}"
            elif isinstance(size, int):
                text = f"{name}   ({size:,} bytes)"
            else:
                text = name
            item = QtWidgets.QListWidgetItem(text)
            item.setData(QtCore.Qt.UserRole, entry)
            self.entries_list.addItem(item)
        self.status_label.setText(f"{len(entries)} item(s) in {self._current_path}")

    def _on_list_failed(self, message: str) -> None:
        self.path_edit.setText(self._current_path)
        self.status_label.setText(f"Failed to list directory: {message}")

    def _go_up(self) -> None:
        parent = posixpath.dirname(self._current_path.rstrip("/")) or "/"
        self.path_edit.setText(parent)
        self._start_listing()

    def _activate_item(self, item: QtWidgets.QListWidgetItem) -> None:
        entry = dict(item.data(QtCore.Qt.UserRole) or {})
        name = str(entry.get("name") or "")
        if not name:
            return
        is_dir = entry.get("is_dir")
        target = posixpath.join(self._current_path.rstrip("/") or "/", name)
        if is_dir is False:
            self._download_entry(target, name)
            return
        # is_dir True or unknown (NLST-only fallback): try to navigate into it.
        # If it turns out to actually be a file, the CWD fails server-side and
        # _on_list_failed reports it -- no separate "is this a file" probe needed.
        self.path_edit.setText(target)
        self._start_listing()

    def _download_selected(self) -> None:
        item = self.entries_list.currentItem()
        if item is None:
            self.status_label.setText("Select a file to download first.")
            return
        entry = dict(item.data(QtCore.Qt.UserRole) or {})
        name = str(entry.get("name") or "")
        if not name:
            return
        target = posixpath.join(self._current_path.rstrip("/") or "/", name)
        self._download_entry(target, name)

    def _download_entry(self, remote_path: str, suggested_name: str) -> None:
        if self._thread is not None:
            return
        local_path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save downloaded file", suggested_name)
        if not local_path:
            return
        self._busy(True)
        self.status_label.setText(f"Downloading {remote_path}...")
        self._thread = QtCore.QThread(self)
        self._worker = _FtpDownloadWorker(
            host=self.host_edit.text().strip(),
            port=self.port_spin.value(),
            user=self.user_edit.text(),
            password=self.password_edit.text(),
            timeout_s=self.timeout_spin.value(),
            remote_path=remote_path,
            local_path=local_path,
        )
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.downloaded.connect(self._on_downloaded)
        self._worker.failed.connect(self._on_download_failed)
        self._worker.finished.connect(self._thread.quit)
        self._worker.finished.connect(self._worker.deleteLater)
        self._thread.finished.connect(self._thread.deleteLater)
        self._thread.finished.connect(self._clear_thread)
        self._thread.start()

    def _on_downloaded(self, local_path: str) -> None:
        self.status_label.setText(f"Downloaded to {local_path}")

    def _on_download_failed(self, message: str) -> None:
        self.status_label.setText(f"Download failed: {message}")

    def _use_current_path(self) -> None:
        self.selected_remote_dir = self._current_path
        self.selected_camera_label = self.camera_combo.currentText() or None
        self.accept()
