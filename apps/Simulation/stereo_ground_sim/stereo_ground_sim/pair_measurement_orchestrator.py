"""Pair-level ordering for observation acquisition and stereo ray variants.

Coordinate math belongs to injected pipelines. This orchestrator only ensures
that Nominal, Optical, GPS-baseline and diagnostic rays consume the same pixel
pair, image dimensions, pair timestamp, and acceptance-time pose snapshot.
"""

import time

import numpy as np


class PairMeasurementOrchestrator:
    def __init__(self, dependencies, *, monotonic=time.monotonic):
        self.dependencies = dependencies
        self.monotonic = monotonic

    def compute(self, pair, phase, device=None):
        d = self.dependencies
        started = self.monotonic()
        camera_snapshot = d["camera_snapshot"]()
        operational_snapshot = d["operational_snapshot"](pair.pair_id)
        mode = d["observation_mode"]()
        if mode == "ideal_reference_point":
            width = int(getattr(pair.left_message, "width", 0))
            height = int(getattr(pair.left_message, "height", 0))
            alignment = d["alignment"](pair.pair_id)
            if width <= 0 or height <= 0:
                return self._result(
                    started, phase, camera_snapshot,
                    ideal_error="invalid_image_dimensions",
                )
            point = None if alignment is None else alignment.position_world_enu_m
            left, right = (
                (None, None)
                if point is None
                else d["ideal_detection"](point, width, height, camera_snapshot)
            )
            blank = np.zeros((height, width, 3), dtype=np.uint8)
            if left is None or right is None:
                return self._result(
                    started, phase, camera_snapshot,
                    left_image=blank, right_image=blank,
                    ideal_error="projection_invalid_or_out_of_fov",
                )
            rays = self._rays(
                pair, left, right, width, height, camera_snapshot,
                operational_snapshot,
            )
            return self._result(
                started, phase, camera_snapshot,
                left_image=blank, right_image=blank, left=left, right=right,
                ideal_error=None, **rays,
            )

        left_image = d["decode"](pair.left_message)
        right_image = d["decode"](pair.right_message)
        if left_image is None or right_image is None:
            return self._result(
                started, phase, camera_snapshot,
                left_image=left_image, right_image=right_image,
            )
        if device is None:
            device = d["device"]()
        left, right = d["detect"](left_image, right_image, device)
        rays = {}
        if left is not None and right is not None:
            rays = self._rays(
                pair, left, right, left_image.shape[1], left_image.shape[0],
                camera_snapshot, operational_snapshot,
            )
        return self._result(
            started, phase, camera_snapshot,
            left_image=left_image, right_image=right_image,
            left=left, right=right, **rays,
        )

    def _rays(self, pair, left, right, width, height, camera_snapshot, operational_snapshot):
        d = self.dependencies
        args = (left["cx"], left["cy"], right["cx"], right["cy"], width, height)
        return {
            "raw": d["nominal"](*args),
            "corrected": d["optical"](*args, camera_snapshot),
            "gps_baselines": d["gps"](*args),
            "diagnostic_rays": d["diagnostics"](
                *args,
                camera_snapshot,
                operational_snapshot,
                pair.pair_midpoint_stamp_ns,
            ),
        }

    def _result(self, started, phase, camera_snapshot, **values):
        result = {
            "started": started,
            "finished": self.monotonic(),
            "phase": phase,
            "left_image": None, "right_image": None,
            "left": None, "right": None, "raw": None, "corrected": None,
            "gps_baselines": {}, "diagnostic_rays": {},
            "camera_oracle_snapshot": camera_snapshot,
        }
        result.update(values)
        return result
