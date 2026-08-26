"""Latest-only off-GUI geometry and optical-alignment computation."""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass
from typing import Mapping

from Workstation.workstation_core.measurement_attempt import MeasurementAttempt
from Workstation.workstation_core.measurement_controller import (
    MeasurementGeometryResult,
    calculate_measurement_geometry,
)


@dataclass(frozen=True)
class PreviewMeasurementRequest:
    key: str
    attempt_payload: dict[str, object]
    optical_payload: dict[str, object]
    submitted_monotonic_s: float


@dataclass(frozen=True)
class PreviewMeasurementResult:
    key: str
    attempt: MeasurementAttempt | None
    geometry_result: MeasurementGeometryResult | None
    optical_metrics: dict[str, float | None]
    error: str
    submitted_monotonic_s: float
    completed_monotonic_s: float


def calculate_optical_metrics(
    attempt: MeasurementAttempt,
    payload: Mapping[str, object],
) -> dict[str, float | None]:
    left_center = tuple(float(value) for value in payload["left_center"])
    right_center = tuple(float(value) for value in payload["right_center"])
    observed_dx_px = left_center[0] - right_center[0]
    observed_dy_px = left_center[1] - right_center[1]
    expected_dy_raw = payload.get("expected_dy_px")
    expected_dy_px = (
        None if expected_dy_raw is None else float(expected_dy_raw)
    )
    mean_fy_raw = payload.get("mean_fy_px")
    mean_fy_px = None if mean_fy_raw is None else float(mean_fy_raw)
    mean_fx_raw = payload.get("mean_fx_px")
    mean_fx_px = None if mean_fx_raw is None else float(mean_fx_raw)
    target_distance_m = float(payload.get("target_distance_m") or 0.0)
    left_timestamp_raw = payload.get("left_pair_timestamp_s")
    right_timestamp_raw = payload.get("right_pair_timestamp_s")
    camera_delta_s = (
        None
        if left_timestamp_raw is None or right_timestamp_raw is None
        else abs(float(left_timestamp_raw) - float(right_timestamp_raw))
    )
    dy_residual_px = (
        None if expected_dy_px is None else observed_dy_px - expected_dy_px
    )
    vertical_error_mrad = (
        None
        if dy_residual_px is None or mean_fy_px is None or mean_fy_px <= 0.0
        else dy_residual_px / mean_fy_px * 1000.0
    )
    expected_disparity_px = None
    yaw_residual_px = None
    yaw_error_mrad = None
    if (
        mean_fx_px is not None
        and mean_fx_px > 0.0
        and attempt.baseline_m > 0.0
        and target_distance_m > 0.0
    ):
        magnitude = mean_fx_px * attempt.baseline_m / target_distance_m
        expected_disparity_px = magnitude if observed_dx_px >= 0.0 else -magnitude
        yaw_residual_px = observed_dx_px - expected_disparity_px
        yaw_error_mrad = yaw_residual_px / mean_fx_px * 1000.0
    return {
        "observed_dx_px": observed_dx_px,
        "observed_dy_px": observed_dy_px,
        "expected_dy_px": expected_dy_px,
        "dy_residual_px": dy_residual_px,
        "vertical_error_mrad": vertical_error_mrad,
        "expected_disparity_px": expected_disparity_px,
        "yaw_residual_px": yaw_residual_px,
        "yaw_error_mrad": yaw_error_mrad,
        "camera_delta_s": camera_delta_s,
    }


class PreviewMeasurementCoordinator:
    """One pending/latest request; stale preview calculations are replaceable."""

    def __init__(self) -> None:
        self._requests: queue.Queue[PreviewMeasurementRequest | None] = queue.Queue(
            maxsize=1
        )
        self._results: queue.Queue[PreviewMeasurementResult] = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._submitted = 0
        self._request_replaced = 0
        self._result_replaced = 0
        self._completed = 0
        self._failed = 0
        self._thread = threading.Thread(
            target=self._run,
            name="preview-measurement-coordinator",
            daemon=True,
        )
        self._thread.start()

    def submit(
        self,
        *,
        key: str,
        attempt: MeasurementAttempt,
        optical_payload: Mapping[str, object],
    ) -> None:
        request = PreviewMeasurementRequest(
            key=str(key),
            attempt_payload=attempt.canonical_payload(),
            optical_payload=dict(optical_payload),
            submitted_monotonic_s=time.monotonic(),
        )
        try:
            self._requests.put_nowait(request)
        except queue.Full:
            try:
                self._requests.get_nowait()
                with self._lock:
                    self._request_replaced += 1
            except queue.Empty:
                pass
            try:
                self._requests.put_nowait(request)
            except queue.Full:
                with self._lock:
                    self._request_replaced += 1
        with self._lock:
            self._submitted += 1

    def _publish(self, result: PreviewMeasurementResult) -> None:
        try:
            self._results.put_nowait(result)
            return
        except queue.Full:
            pass
        try:
            self._results.get_nowait()
            with self._lock:
                self._result_replaced += 1
        except queue.Empty:
            pass
        try:
            self._results.put_nowait(result)
        except queue.Full:
            with self._lock:
                self._result_replaced += 1

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                request = self._requests.get(timeout=0.1)
            except queue.Empty:
                continue
            if request is None:
                return
            try:
                attempt = MeasurementAttempt.from_canonical_payload(
                    request.attempt_payload
                )
                geometry_result = calculate_measurement_geometry(attempt)
                optical_metrics = calculate_optical_metrics(
                    attempt,
                    request.optical_payload,
                )
                error = ""
            except Exception as exc:  # noqa: BLE001 - result stays explicit.
                attempt = None
                geometry_result = None
                optical_metrics = {}
                error = f"{type(exc).__name__}: {exc}"
            result = PreviewMeasurementResult(
                key=request.key,
                attempt=attempt,
                geometry_result=geometry_result,
                optical_metrics=optical_metrics,
                error=error,
                submitted_monotonic_s=request.submitted_monotonic_s,
                completed_monotonic_s=time.monotonic(),
            )
            with self._lock:
                self._completed += 1
                if error:
                    self._failed += 1
            self._publish(result)

    def poll_latest(self) -> PreviewMeasurementResult | None:
        latest = None
        while True:
            try:
                latest = self._results.get_nowait()
            except queue.Empty:
                return latest

    def metrics(self) -> dict[str, int]:
        with self._lock:
            return {
                "submitted": self._submitted,
                "request_replaced": self._request_replaced,
                "result_replaced": self._result_replaced,
                "completed": self._completed,
                "failed": self._failed,
                "request_capacity": 1,
                "result_capacity": 1,
            }

    def close(self) -> bool:
        self._stop.set()
        try:
            self._requests.put_nowait(None)
        except queue.Full:
            try:
                self._requests.get_nowait()
            except queue.Empty:
                pass
            try:
                self._requests.put_nowait(None)
            except queue.Full:
                pass
        self._thread.join(timeout=1.0)
        return not self._thread.is_alive()


__all__ = [
    "PreviewMeasurementCoordinator",
    "PreviewMeasurementRequest",
    "PreviewMeasurementResult",
    "calculate_optical_metrics",
]
