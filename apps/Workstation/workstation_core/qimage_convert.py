"""Pixel-format conversions between raw native buffers and Qt images.

Kept in its own module so Qt is not pulled into otherwise Qt-agnostic
modules (e.g. ``native_frame_store.py``) purely for this conversion.
"""

from __future__ import annotations

from PyQt5 import QtGui


def qimage_from_bgr(data: bytes, *, width: int, height: int) -> QtGui.QImage:
    """Wrap a tightly-packed BGR24 buffer as a QImage, copying the pixels."""

    bytes_per_line = int(width) * 3
    bgr_format = getattr(QtGui.QImage, "Format_BGR888", None)
    if bgr_format is not None:
        return QtGui.QImage(
            data,
            int(width),
            int(height),
            bytes_per_line,
            bgr_format,
        ).copy()
    return QtGui.QImage(
        data,
        int(width),
        int(height),
        bytes_per_line,
        QtGui.QImage.Format_RGB888,
    ).rgbSwapped()


__all__ = ["qimage_from_bgr"]
