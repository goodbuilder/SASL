"""Media leaves retain serialized contracts and execute through real spawn."""

from __future__ import annotations

import base64
import importlib
import json
import multiprocessing
from pathlib import Path
import pickle
import queue
import subprocess
import sys
import textwrap
import time

import pytest

from Workstation.workstation_core import yolo_stream
from Workstation.workstation_core.media_stream import detections, rtp_timing
from Workstation.workstation_core.media_stream.child_runtime import _run_yolo_process
from Workstation.workstation_core.media_stream.ipc import CameraIpcCounters, _BoundedProcessOutputQueue


_FIXTURE = json.loads((Path(__file__).parent / "fixtures/media_stream_legacy_types.json").read_text())


@pytest.mark.parametrize("row", _FIXTURE["values"], ids=lambda row: row["name"])
def test_existing_media_pickle_bytes_and_class_identity_are_preserved(row):
    owner = detections if row["name"] == "DetectionBox" else rtp_timing
    cls = getattr(owner, row["name"])
    legacy_bytes = base64.b64decode(row["pickle_base64"])
    assert getattr(yolo_stream, row["name"]) is cls
    assert type(pickle.loads(legacy_bytes)) is cls
    assert pickle.dumps(cls(**row["fields"]), protocol=_FIXTURE["protocol"]) == legacy_bytes


def test_media_owners_import_without_gui_model_or_legacy_facade():
    apps = Path(__file__).resolve().parents[2]
    script = textwrap.dedent("""
        import importlib, importlib.abc, sys
        from pathlib import Path
        apps = Path(sys.argv[1])
        sys.path[:0] = [str(apps), str(apps.parent / 'common/python')]
        class Guard(importlib.abc.MetaPathFinder):
            def find_spec(self, fullname, path=None, target=None):
                if fullname.startswith(('PyQt5', 'torch', 'ultralytics', 'cv2', 'gi')) or fullname == 'Workstation.workstation_core.yolo_stream':
                    raise AssertionError('premature runtime import: ' + fullname)
                return None
        sys.meta_path.insert(0, Guard())
        for name in ('detections', 'rtp_timing', 'rates', 'ipc', 'recorders', 'child_runtime'):
            importlib.import_module('Workstation.workstation_core.media_stream.' + name)
        try:
            importlib.import_module('Workstation.workstation_core.yolo_stream')
        except AssertionError:
            pass
        else:
            raise AssertionError('forbidden import fixture was accepted')
        print('independent media owners')
    """)
    result = subprocess.run(
        [sys.executable, "-I", "-B", "-c", script, str(apps)], check=True, text=True, capture_output=True, timeout=30
    )
    assert result.stdout.strip() == "independent media owners"


@pytest.mark.parametrize("valid_source", [False, True])
def test_real_child_entrypoint_spawn_decodes_or_reports_failure_and_exits(tmp_path, valid_source):
    path = tmp_path / "native.png"
    if valid_source:
        import cv2
        import numpy as np

        native = np.empty((24, 32, 3), dtype=np.uint8)
        native[:] = (31, 73, 151)  # Input BGR; IPC preview must be exact RGB.
        assert cv2.imwrite(str(path), native)
    context = multiprocessing.get_context("spawn")
    control = context.Queue(maxsize=32)
    frames = context.Queue(maxsize=2)
    metadata = context.Queue(maxsize=8)
    counters = CameraIpcCounters(context)
    output = _BoundedProcessOutputQueue(control, frames, metadata, counters)
    stop = context.Event()
    process = context.Process(
        target=_run_yolo_process,
        args=(
            {
                "url": str(path),
                "enable_detection": False,
                "stream_backend": "opencv",
                "preview_max_width": 0,
                "rtsp_frame_timeout_s": 0.5,
                "worker_cpu_threads": 1,
            },
            output,
            stop,
        ),
    )
    messages = []
    try:
        process.start()
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            try:
                message = frames.get(timeout=0.05)
            except queue.Empty:
                try:
                    message = control.get(timeout=0.05)
                except queue.Empty:
                    if not process.is_alive():
                        break
                    continue
            messages.append(message)
            if message[0] == "frame":
                stop.set()
                break
        stop.set()
        process.join(timeout=3.0)
        assert not process.is_alive()
        assert process.exitcode == 0
        assert counters.snapshot()["control_dropped"] == 0
        if valid_source:
            frame = next(message for message in messages if message[0] == "frame")
            assert frame[2:4] == (32, 24)
            assert frame[4] == bytes((151, 73, 31)) * (32 * 24)
            assert frame[1]["native_width"] == 32
            assert frame[1]["native_height"] == 24
            assert frame[1]["app_receive_monotonic_ns"] > 0
        else:
            assert any(message[0] == "status" and "스트림 열기 실패" in message[1] for message in messages)
    finally:
        stop.set()
        if process.is_alive():
            process.terminate()
            process.join(timeout=3.0)
        for channel in (control, frames, metadata):
            channel.close()
            channel.join_thread()
