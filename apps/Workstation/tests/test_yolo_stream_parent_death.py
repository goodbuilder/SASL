from __future__ import annotations

import signal

import pytest

from Workstation.workstation_core import yolo_stream


class _FakePrctl:
    def __init__(self, result: int = 0) -> None:
        self.result = int(result)
        self.calls: list[tuple[int, int, int, int, int]] = []
        self.argtypes = None
        self.restype = None

    def __call__(self, *args: int) -> int:
        self.calls.append(tuple(int(value) for value in args))
        return self.result


class _FakeLibc:
    def __init__(self, result: int = 0) -> None:
        self.prctl = _FakePrctl(result)


def test_linux_camera_child_arms_sigterm_on_parent_death(monkeypatch) -> None:
    libc = _FakeLibc()
    monkeypatch.setattr(yolo_stream.sys, "platform", "linux")
    monkeypatch.setattr(yolo_stream.os, "getppid", lambda: 42)
    monkeypatch.setattr(yolo_stream.ctypes, "CDLL", lambda *_args, **_kwargs: libc)

    assert yolo_stream._arm_linux_parent_death_signal() is True
    assert libc.prctl.calls == [(1, int(signal.SIGTERM), 0, 0, 0)]


def test_linux_camera_child_exits_when_parent_dies_during_setup(monkeypatch) -> None:
    libc = _FakeLibc()
    parent_ids = iter((42, 1))
    monkeypatch.setattr(yolo_stream.sys, "platform", "linux")
    monkeypatch.setattr(yolo_stream.os, "getppid", lambda: next(parent_ids))
    monkeypatch.setattr(yolo_stream.ctypes, "CDLL", lambda *_args, **_kwargs: libc)
    monkeypatch.setattr(
        yolo_stream.os,
        "_exit",
        lambda code: (_ for _ in ()).throw(SystemExit(code)),
    )

    with pytest.raises(SystemExit) as excinfo:
        yolo_stream._arm_linux_parent_death_signal()

    assert excinfo.value.code == 128 + int(signal.SIGTERM)


def test_parent_death_signal_is_noop_off_linux(monkeypatch) -> None:
    monkeypatch.setattr(yolo_stream.sys, "platform", "darwin")
    monkeypatch.setattr(
        yolo_stream.ctypes,
        "CDLL",
        lambda *_args, **_kwargs: pytest.fail("CDLL must not be used off Linux"),
    )

    assert yolo_stream._arm_linux_parent_death_signal() is False
