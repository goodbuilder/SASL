"""Worker identity and one-way imports after WIRIS responsibility separation."""
from __future__ import annotations

import ast
import importlib
from pathlib import Path
import subprocess
import sys
import textwrap

import pytest

from Workstation import wiris_runtime_workers as legacy


WORKER_OWNERS = {
    "clock": ("WirisClockCommandWorker",),
    "network": ("WirisStipWorker", "WirisRtspEnableWorker"),
    "capture": ("WirisPnPPhotoCaptureWorker", "WirisOZPhotoCaptureWorker"),
    "ftp": ("WirisOzFtpSyncWorker", "WirisFtpBatchDeleteWorker"),
    "commands": ("WirisCommandWorker", "WirisImageFormatWorker", "WirisLrfPollWorker", "WirisZoomWorker"),
    "probe": ("RtspStreamCheckWorker",),
}


@pytest.mark.parametrize("module_name", WORKER_OWNERS)
def test_legacy_worker_types_are_the_actual_state_and_signal_owners(module_name):
    module = importlib.import_module(f"Workstation.wiris_workers.{module_name}")
    defined = {
        node.name for node in ast.parse(Path(module.__file__).read_text()).body
        if isinstance(node, ast.ClassDef)
    }
    for name in WORKER_OWNERS[module_name]:
        assert name in defined
        actual = getattr(module, name)
        assert getattr(legacy, name) is actual
        assert actual.__module__ == "Workstation.wiris_runtime_workers"
        assert hasattr(actual, "finished")


def test_oz_capture_reuses_the_same_capture_transaction_owner():
    from Workstation.wiris_workers.capture import WirisOZPhotoCaptureWorker, WirisPnPPhotoCaptureWorker

    assert WirisOZPhotoCaptureWorker.__bases__ == (WirisPnPPhotoCaptureWorker,)
    assert not any(
        isinstance(node, (ast.FunctionDef, ast.ClassDef))
        for node in ast.parse(Path(legacy.__file__).read_text()).body
    )


@pytest.mark.parametrize("module_name", ["clock", "network", "capture", "ftp", "commands"])
def test_worker_owner_import_does_not_pull_in_facade_gui_or_media_backend(module_name):
    apps = Path(__file__).resolve().parents[2]
    script = textwrap.dedent("""
        import importlib, importlib.abc, sys
        from pathlib import Path
        apps = Path(sys.argv[1])
        sys.path[:0] = [str(apps), str(apps.parent / 'common/python')]
        forbidden = (
            'Workstation.wiris_runtime_workers',
            'Workstation.standalone_gui',
            'Workstation.workstation_core.yolo_stream',
        )
        class Guard(importlib.abc.MetaPathFinder):
            def find_spec(self, fullname, path=None, target=None):
                if fullname in forbidden:
                    raise AssertionError('worker back-imported ' + fullname)
                return None
        sys.meta_path.insert(0, Guard())
        module = importlib.import_module('Workstation.wiris_workers.' + sys.argv[2])
        assert not set(forbidden).intersection(sys.modules)
        if sys.argv[2] == 'clock':
            worker = module.WirisClockCommandWorker((), command_text='STUS')
            worker.cancel()
            try:
                worker._raise_if_cancelled()
            except InterruptedError:
                pass
            else:
                raise AssertionError('cancellation lost')
        print('owner imported independently')
    """)
    result = subprocess.run(
        [sys.executable, "-I", "-B", "-c", script, str(apps), module_name],
        capture_output=True, text=True, check=True, timeout=30,
    )
    assert result.stdout.strip() == "owner imported independently"
