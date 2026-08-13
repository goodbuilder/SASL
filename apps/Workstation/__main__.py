"""Direct launcher for the standalone Workstation application."""

from __future__ import annotations

import argparse
import faulthandler
import os
import signal
import sys
import tempfile
from pathlib import Path


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Start the SASL Workstation UI.")
    parser.add_argument("--check", action="store_true", help="Verify runtime imports without opening Qt.")
    parser.add_argument("--maximized", action="store_true", help="Open the Workstation maximized.")
    parser.add_argument(
        "--start",
        dest="start",
        action="store_true",
        default=True,
        help="Enable WIRIS/RTSP and start the camera preview after launch (default).",
    )
    parser.add_argument(
        "--no-start",
        dest="start",
        action="store_false",
        help="Open the Workstation without starting camera discovery or preview.",
    )
    parser.add_argument(
        "--start-pixhawk",
        "--connect-pixhawk",
        dest="start_pixhawk",
        action="store_true",
        help="Connect the configured Pixhawk MAVLink endpoints after launch.",
    )
    return parser


def _runtime_check() -> int:
    import cv2
    import numpy as np
    import pyarrow
    import xlsxwriter
    import yaml
    from PyQt5 import QtCore

    import sasl_common.lifecycle
    import Workstation.main_window
    import Workstation.standalone_gui

    print("SASL Workstation runtime check ok")
    print(f"python={sys.executable}")
    print(f"repository={Path(__file__).resolve().parents[2]}")
    print(
        f"qt={QtCore.QT_VERSION_STR}, opencv={cv2.__version__}, numpy={np.__version__}, "
        f"yaml={yaml.__version__}, xlsxwriter={xlsxwriter.__version__}, pyarrow={pyarrow.__version__}"
    )
    return 0


def _workstation_lock_path() -> Path:
    uid = os.getuid() if hasattr(os, "getuid") else "user"
    runtime_dir = os.environ.get("XDG_RUNTIME_DIR", "").strip()
    base = Path(runtime_dir) if runtime_dir else Path(tempfile.gettempdir())
    return base / f"sasl-workstation-{uid}.lock"


def _acquire_single_instance_lock(lock_path: Path | None = None):
    """Keep one Workstation process per OS user to protect camera SDK ports."""

    from PyQt5 import QtCore

    lock = QtCore.QLockFile(str(lock_path or _workstation_lock_path()))
    lock.setStaleLockTime(30_000)
    return lock if lock.tryLock(0) else None


def main(argv: list[str] | None = None) -> int:
    os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "1")
    try:
        faulthandler.register(signal.SIGUSR1)
    except (AttributeError, OSError, RuntimeError):
        pass
    args, qt_args = _parser().parse_known_args(sys.argv[1:] if argv is None else argv)
    if args.check:
        return _runtime_check()

    from PyQt5 import QtWidgets

    from .main_window import WorkstationMainWindow
    from .standalone_gui import _configure_gl_surface_format

    _configure_gl_surface_format()
    app = QtWidgets.QApplication([sys.argv[0], *qt_args])
    instance_lock = _acquire_single_instance_lock()
    if instance_lock is None:
        print("SASL Workstation is already running; duplicate launch was blocked.", file=sys.stderr)
        return 0
    window = WorkstationMainWindow(
        auto_start=bool(args.start),
        auto_start_pixhawk=bool(args.start_pixhawk),
    )
    if args.maximized:
        window.showMaximized()
    else:
        window.show()
    return int(app.exec_())


if __name__ == "__main__":
    raise SystemExit(main())
