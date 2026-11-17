#!/usr/bin/env python3
"""Wait for the visible RQT main window and maximize it through EWMH."""

from __future__ import annotations

import argparse
import ctypes
import ctypes.util
import os
import sys
import time
from typing import Any


class _ClientMessageData(ctypes.Union):
    _fields_ = [
        ("b", ctypes.c_char * 20),
        ("s", ctypes.c_short * 10),
        ("l", ctypes.c_long * 5),
    ]


class _ClientMessageEvent(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int),
        ("serial", ctypes.c_ulong),
        ("send_event", ctypes.c_int),
        ("display", ctypes.c_void_p),
        ("window", ctypes.c_ulong),
        ("message_type", ctypes.c_ulong),
        ("format", ctypes.c_int),
        ("data", _ClientMessageData),
    ]


class _XEvent(ctypes.Union):
    _fields_ = [("xclient", _ClientMessageEvent), ("pad", ctypes.c_long * 24)]


def _load_x11() -> Any:
    library = ctypes.util.find_library("X11")
    if not library:
        raise RuntimeError("libX11 is unavailable")
    x11 = ctypes.CDLL(library)
    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
    x11.XDefaultRootWindow.restype = ctypes.c_ulong
    x11.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    x11.XInternAtom.restype = ctypes.c_ulong
    x11.XGetWindowProperty.argtypes = [
        ctypes.c_void_p,
        ctypes.c_ulong,
        ctypes.c_ulong,
        ctypes.c_long,
        ctypes.c_long,
        ctypes.c_int,
        ctypes.c_ulong,
        ctypes.POINTER(ctypes.c_ulong),
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_ulong),
        ctypes.POINTER(ctypes.c_ulong),
        ctypes.POINTER(ctypes.c_void_p),
    ]
    x11.XGetWindowProperty.restype = ctypes.c_int
    x11.XFetchName.argtypes = [
        ctypes.c_void_p,
        ctypes.c_ulong,
        ctypes.POINTER(ctypes.c_void_p),
    ]
    x11.XFetchName.restype = ctypes.c_int
    x11.XSendEvent.argtypes = [
        ctypes.c_void_p,
        ctypes.c_ulong,
        ctypes.c_int,
        ctypes.c_long,
        ctypes.POINTER(_XEvent),
    ]
    x11.XSendEvent.restype = ctypes.c_int
    x11.XFlush.argtypes = [ctypes.c_void_p]
    x11.XFree.argtypes = [ctypes.c_void_p]
    return x11


def _property_values(x11: Any, display: Any, window: int, atom: int) -> list[int]:
    actual_type = ctypes.c_ulong()
    actual_format = ctypes.c_int()
    item_count = ctypes.c_ulong()
    bytes_after = ctypes.c_ulong()
    data = ctypes.c_void_p()
    status = x11.XGetWindowProperty(
        display,
        window,
        atom,
        0,
        4096,
        0,
        0,
        ctypes.byref(actual_type),
        ctypes.byref(actual_format),
        ctypes.byref(item_count),
        ctypes.byref(bytes_after),
        ctypes.byref(data),
    )
    if status != 0 or not data.value or actual_format.value != 32:
        if data.value:
            x11.XFree(data)
        return []
    try:
        values = ctypes.cast(data, ctypes.POINTER(ctypes.c_ulong))
        return [int(values[index]) for index in range(item_count.value)]
    finally:
        x11.XFree(data)


def _window_title(x11: Any, display: Any, window: int) -> str:
    title = ctypes.c_void_p()
    if not x11.XFetchName(display, window, ctypes.byref(title)) or not title.value:
        return ""
    try:
        return ctypes.string_at(title.value).decode("utf-8", errors="replace")
    finally:
        x11.XFree(title)


def _maximize(x11: Any, display: Any, root: int, window: int) -> None:
    event = _XEvent()
    event.xclient.type = 33  # ClientMessage
    event.xclient.send_event = 1
    event.xclient.display = display
    event.xclient.window = window
    event.xclient.message_type = x11.XInternAtom(display, b"_NET_WM_STATE", 0)
    event.xclient.format = 32
    event.xclient.data.l[0] = 1  # _NET_WM_STATE_ADD
    event.xclient.data.l[1] = x11.XInternAtom(display, b"_NET_WM_STATE_MAXIMIZED_VERT", 0)
    event.xclient.data.l[2] = x11.XInternAtom(display, b"_NET_WM_STATE_MAXIMIZED_HORZ", 0)
    event.xclient.data.l[3] = 1  # application request
    mask = (1 << 19) | (1 << 20)  # SubstructureNotify | SubstructureRedirect
    if not x11.XSendEvent(display, root, 0, mask, ctypes.byref(event)):
        raise RuntimeError("window manager rejected the maximize request")
    x11.XFlush(display)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parser.add_argument("--interval-s", type=float, default=0.25)
    parser.add_argument("--title-contains", default="rqt")
    args = parser.parse_args()
    if args.timeout_s <= 0.0 or args.interval_s <= 0.0:
        parser.error("timeout and interval must be positive")
    if not os.environ.get("DISPLAY"):
        print("DISPLAY is unset; cannot find the RQT window", file=sys.stderr)
        return 2

    try:
        x11 = _load_x11()
        display = x11.XOpenDisplay(None)
        if not display:
            raise RuntimeError(f"cannot open X display {os.environ['DISPLAY']}")
    except Exception as exc:
        print(f"RQT maximize setup failed: {exc}", file=sys.stderr)
        return 2

    root = x11.XDefaultRootWindow(display)
    clients_atom = x11.XInternAtom(display, b"_NET_CLIENT_LIST", 0)
    state_atom = x11.XInternAtom(display, b"_NET_WM_STATE", 0)
    vertical_atom = x11.XInternAtom(display, b"_NET_WM_STATE_MAXIMIZED_VERT", 0)
    horizontal_atom = x11.XInternAtom(display, b"_NET_WM_STATE_MAXIMIZED_HORZ", 0)
    deadline = time.monotonic() + args.timeout_s
    match = args.title_contains.casefold()
    last_title = ""
    try:
        while time.monotonic() < deadline:
            for window in _property_values(x11, display, root, clients_atom):
                title = _window_title(x11, display, window)
                lowered = title.casefold()
                if match not in lowered or "selection owner" in lowered:
                    continue
                last_title = title
                try:
                    _maximize(x11, display, root, window)
                except RuntimeError:
                    continue
                states = _property_values(x11, display, window, state_atom)
                if vertical_atom in states and horizontal_atom in states:
                    print(f"RQT window maximized: id=0x{window:x}, title={title}")
                    return 0
            time.sleep(args.interval_s)
    finally:
        x11.XFlush(display)

    detail = f"; last matching title: {last_title}" if last_title else ""
    print(
        f"RQT main window did not become maximized within {args.timeout_s:g}s{detail}",
        file=sys.stderr,
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
