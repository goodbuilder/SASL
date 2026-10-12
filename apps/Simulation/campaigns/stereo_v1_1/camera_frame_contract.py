"""Explicit Gazebo camera-link FLU to OpenCV optical RDF conversion."""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np


# Gazebo's camera optical_link orientation is reported in its SDF link frame:
# x=forward, y=left, z=up (FLU). The image/calibration frame is OpenCV RDF:
# x=right, y=down, z=forward. Columns are the RDF basis vectors expressed in
# link FLU coordinates, so R_WC = R_WL @ R_LC.
R_CAMERA_LINK_FLU_FROM_OPTICAL_RDF = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
], dtype=float)


def quaternion_xyzw_to_matrix(values: Sequence[float]) -> np.ndarray:
    """Convert a Gazebo world-from-link XYZW quaternion to a rotation matrix."""
    if len(values) != 4:
        raise ValueError("Gazebo camera-link quaternion must contain XYZW")
    x, y, z, w = (float(value) for value in values)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("Gazebo camera-link quaternion must be finite and nonzero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


def optical_rwc_from_entity_quaternion_xyzw(values: Sequence[float]) -> np.ndarray:
    """Return optical RDF → world ENU R_WC from the recorded SDF link pose."""
    world_from_camera_link_flu = quaternion_xyzw_to_matrix(values)
    return world_from_camera_link_flu @ R_CAMERA_LINK_FLU_FROM_OPTICAL_RDF
