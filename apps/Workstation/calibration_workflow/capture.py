"""Capture cadence and native-frame admission own their timer and inflight state.

Snapshots carry optical image grids and capture clocks; decoded host receipt is
never promoted to exposure truth. Stopping cadence does not cancel an admitted
CalibrationJobs worker or discard its already-persisted image result.
"""
from __future__ import annotations
import time
from dataclasses import dataclass
from typing import Mapping, Callable
from PyQt5 import QtCore, QtGui
from Workstation.workstation_core.calibration_quality import CalibrationViewGeometry, extract_view_geometry, pose_novelty
from Workstation.workstation_core.native_frame_ring import FrameKey
from .timing import CaptureTiming

CALIBRATION_AUTO_DETECTION_MAX_AGE_S = 2.0
CAPTURE_FIELDS = ('_auto_capture_timer', '_auto_capture_active', '_auto_capture_saved_pairs', '_auto_capture_last_pair_key', '_auto_capture_inflight_pair_key', '_auto_capture_last_attempt_s', '_capture_origin', '_auto_capture_pose_views', '_auto_capture_pending_pose_views', '_auto_capture_pending_pose_novelty')


@dataclass(frozen=True)
class LiveCaptureInputs:
    busy: bool
    detections: Mapping
    allow_preview_fallback: bool
    native_frame_getters: Mapping
    pattern_size: tuple[int, int]
    pair_max_delta_s: float


def is_nal1_non_idr_auto_capture_metadata(metadata: object) -> bool:
    if not isinstance(metadata, Mapping):
        return False
    nal_types = metadata.get("h264_access_unit_nal_types")
    if not isinstance(nal_types, (list, tuple)):
        return False
    try:
        normalized_nal_types = {int(value) for value in nal_types}
        pts_ns = int(metadata["h264_access_unit_pts_ns"])
        decoded_pts_ns = int(metadata["decoded_buffer_pts_ns"])
    except (KeyError, TypeError, ValueError, OverflowError):
        return False
    return bool(
        metadata.get("h264_access_unit_kind") == "nal1_non_idr_intra"
        and metadata.get("h264_nal1_non_idr_intra") is True
        and metadata.get("h264_access_unit_is_delta") is False
        and metadata.get("h264_random_access_guaranteed") is False
        and pts_ns >= 0
        and decoded_pts_ns == pts_ns
        and 1 in normalized_nal_types
        and 5 not in normalized_nal_types
    )


class CalibrationCapture(QtCore.QObject):
    def __init__(self, *, read_live_inputs=None, submit_capture=None, present_status=None, parent=None):
        super().__init__(parent)
        self.read_live_inputs = read_live_inputs
        self.submit_capture = submit_capture
        self.present_status = present_status
        self._auto_capture_timer = QtCore.QTimer(self)
        self._auto_capture_timer.timeout.connect(self.poll)
        self._auto_capture_active = False
        self._auto_capture_saved_pairs = 0
        self._auto_capture_last_pair_key = None
        self._auto_capture_inflight_pair_key = None
        self._auto_capture_last_attempt_s = 0.0
        self._capture_origin = "manual"
        self._auto_capture_pose_views = {"left": [], "right": []}
        self._auto_capture_pending_pose_views = None
        self._auto_capture_pending_pose_novelty = None

    def bind(self, *, read_live_inputs, submit_capture, present_status):
        if not all(callable(callback) for callback in (read_live_inputs, submit_capture, present_status)):
            raise TypeError("Capture requires input, submission and presentation operations")
        self.read_live_inputs = read_live_inputs
        self.submit_capture = submit_capture
        self.present_status = present_status

    def poll(self):
        if not callable(self.read_live_inputs):
            raise RuntimeError("Capture runtime is not composed")
        self.tick(self.read_live_inputs())

    def start(self, interval_s):
        if not all(callable(callback) for callback in (self.read_live_inputs, self.submit_capture, self.present_status)):
            raise RuntimeError("Capture runtime is not composed")
        self._auto_capture_active = True
        self._auto_capture_saved_pairs = 0
        self._auto_capture_last_pair_key = None
        self._auto_capture_inflight_pair_key = None
        self._auto_capture_pose_views = {"left": [], "right": []}
        self._auto_capture_pending_pose_views = None
        self._auto_capture_pending_pose_novelty = None
        self._auto_capture_last_attempt_s = 0.0
        self._auto_capture_timer.start(max(100, int(interval_s * 1000.0)))

    def stop(self):
        was_active = self._auto_capture_active or self._auto_capture_timer.isActive()
        self._auto_capture_timer.stop()
        self._auto_capture_active = False
        self._auto_capture_inflight_pair_key = None
        self._auto_capture_pending_pose_views = None
        self._auto_capture_pending_pose_novelty = None
        return was_active

    def resource_inventory(self):
        return {"_auto_capture_timer": self._auto_capture_timer}

    def tick(self, inputs: LiveCaptureInputs) -> None:
        if not self._auto_capture_active:
            return
        if inputs.busy:
            return

        detections = inputs.detections
        left = detections.get("left") if isinstance(detections, dict) else None
        right = detections.get("right") if isinstance(detections, dict) else None
        if not isinstance(left, dict) or not isinstance(right, dict):
            self.present_status("자동 촬영 대기: 좌·우 체커보드 검출 결과를 기다리는 중")
            return
        if not bool(left.get("found")) or not bool(right.get("found")):
            self.present_status("자동 촬영 대기: 좌·우 화면에서 체커보드가 모두 검출되어야 합니다")
            return
        left_image = left.get("image")
        right_image = right.get("image")
        if not isinstance(left_image, QtGui.QImage) or not isinstance(right_image, QtGui.QImage):
            self.present_status("자동 촬영 대기: 좌·우 최신 프레임을 기다리는 중")
            return

        try:
            left_request_id = int(left.get("request_id") or 0)
            right_request_id = int(right.get("request_id") or 0)
        except (TypeError, ValueError):
            return
        if left_request_id <= 0 or right_request_id <= 0:
            return
        pair_key = (left_request_id, right_request_id)
        if pair_key == self._auto_capture_last_pair_key:
            return

        preview_metadata = {
            "left": dict(left.get("metadata") or {}) if isinstance(left.get("metadata"), dict) else {},
            "right": dict(right.get("metadata") or {}) if isinstance(right.get("metadata"), dict) else {},
        }
        allow_preview_fallback = bool(
            inputs.allow_preview_fallback
        )
        if not allow_preview_fallback:
            missing_nal1_sides = [
                side
                for side in ("left", "right")
                if not is_nal1_non_idr_auto_capture_metadata(
                    preview_metadata[side]
                )
            ]
            if missing_nal1_sides:
                self.present_status(
                    "자동 촬영 대기: "
                    + ", ".join(missing_nal1_sides)
                    + " NAL1 non-delta frame을 기다리는 중"
                )
                return
        native_boundary_available = all(
            callable(
                inputs.native_frame_getters.get(side)
            )
            for side in ("left", "right")
        )
        corners_available = all(bool(row.get("corners")) for row in (left, right))
        try:
            pose_views = {
                "left": extract_view_geometry(
                    left.get("corners") or (),
                    (left_image.width(), left_image.height()),
                    pattern_size=inputs.pattern_size,
                    view_id=f"preview-left-{left_request_id}",
                ),
                "right": extract_view_geometry(
                    right.get("corners") or (),
                    (right_image.width(), right_image.height()),
                    pattern_size=inputs.pattern_size,
                    view_id=f"preview-right-{right_request_id}",
                ),
            } if corners_available else {}
        except (TypeError, ValueError) as exc:
            self.present_status(
                f"자동 촬영 대기: checkerboard pose 분석 실패 ({exc})"
            )
            return
        if (native_boundary_available or not allow_preview_fallback) and not pose_views:
            self.present_status(
                "자동 촬영 대기: checkerboard corner geometry가 없습니다"
            )
            return
        reference_views = getattr(
            self,
            "_auto_capture_pose_views",
            {"left": [], "right": []},
        )
        novelty = {
            side: pose_novelty(pose_views[side], reference_views.get(side, ()))
            for side in ("left", "right")
        } if pose_views else {"left": 2.0, "right": 2.0}
        if any(value <= 1.0 for value in novelty.values()):
            duplicate_sides = ", ".join(
                side for side, value in novelty.items() if value <= 1.0
            )
            self.present_status(
                f"자동 촬영 대기: {duplicate_sides} checkerboard pose가 이전과 거의 같습니다. "
                "위치·거리·yaw/pitch/roll을 바꾸세요."
            )
            return

        native_frames: dict[str, QtGui.QImage] = {}
        metadata_by_camera: dict[str, dict[str, object]] = {}
        for side in ("left", "right"):
            native_getter = inputs.native_frame_getters.get(side)
            if not callable(native_getter):
                if not allow_preview_fallback:
                    self.present_status(
                        f"자동 촬영 대기: {side} exact FrameKey API가 없습니다"
                    )
                    return
                preview_image = left_image if side == "left" else right_image
                native_frames[side] = preview_image
                metadata_by_camera[side] = dict(preview_metadata[side])
                continue
            try:
                frame_key = FrameKey.from_mapping(preview_metadata[side])
            except (TypeError, ValueError):
                self.present_status(
                    f"자동 촬영 대기: {side} exact FrameKey가 없습니다"
                )
                return
            native = native_getter(
                frame_key,
                maximum_age_s=CALIBRATION_AUTO_DETECTION_MAX_AGE_S,
            )
            if native is None:
                self.present_status(
                    f"자동 촬영 대기: {side} exact native FrameKey가 만료되어 새 NAL1 frame을 기다리는 중"
                )
                return
            frame_id, native_image, native_metadata = native
            preview_pts_ns = preview_metadata[side].get("h264_access_unit_pts_ns")
            if (
                str(frame_id) != frame_key.text
                or str(native_metadata.get("frame_key") or "") != frame_key.text
                or not is_nal1_non_idr_auto_capture_metadata(native_metadata)
                or preview_pts_ns is None
                or int(native_metadata["h264_access_unit_pts_ns"]) != int(preview_pts_ns)
            ):
                self.present_status(
                    f"자동 촬영 대기: {side} NAL1/FrameKey provenance가 일치하지 않습니다"
                )
                return
            native_frames[side] = native_image
            metadata_by_camera[side] = {
                **dict(native_metadata),
                "frame_id": str(frame_id),
                "calibration_source": "native_frame_cache",
                "preview_width": left_image.width() if side == "left" else right_image.width(),
                "preview_height": left_image.height() if side == "left" else right_image.height(),
            }
        left_image = native_frames["left"]
        right_image = native_frames["right"]
        now_s = time.monotonic()
        for camera in ("left", "right"):
            timestamp_s = metadata_by_camera[camera].get("timestamp_s")
            try:
                age_s = now_s - float(timestamp_s)
            except (TypeError, ValueError):
                self.present_status(
                    "자동 촬영 대기: 좌·우 프레임의 수신 시각이 없습니다"
                )
                return
            if age_s < 0.0 or age_s > CALIBRATION_AUTO_DETECTION_MAX_AGE_S:
                self.present_status(
                    "자동 촬영 대기: 체커보드 검출 프레임이 오래되어 새 프레임을 기다리는 중"
                )
                return

        pair_timing_ok, _pair_time_basis, _pair_delta_s, pair_timing_detail = (
            CaptureTiming()._checkerboard_capture_pair_timing(metadata_by_camera, maximum_delta_s=inputs.pair_max_delta_s)
        )

        self._auto_capture_last_pair_key = pair_key
        self._auto_capture_last_attempt_s = now_s
        self._auto_capture_inflight_pair_key = pair_key
        self._auto_capture_pending_pose_views = pose_views
        self._auto_capture_pending_pose_novelty = novelty
        started = self.submit_capture(
            frames={"left": left_image, "right": right_image},
            metadata_by_camera=metadata_by_camera,
            capture_origin="auto",
        )
        if not started:
            self._auto_capture_inflight_pair_key = None
            self._auto_capture_pending_pose_views = None
            self._auto_capture_pending_pose_novelty = None
            return
        usage = (
            "stereo pair 후보"
            if pair_timing_ok
            else f"intrinsics-only 후보 ({pair_timing_detail})"
        )
        self.present_status(
            f"자동 촬영 중: {usage} | 유효 stereo pair "
            f"{self._auto_capture_saved_pairs}개 | Capture worker가 보드 검증 중"
        )



    def fail_auto(self):
        self._auto_capture_inflight_pair_key = None
        self._auto_capture_pending_pose_views = None
        self._auto_capture_pending_pose_novelty = None
        self._capture_origin = "manual"

    def complete_auto(self, saved_sides, pair_saved):
        self._auto_capture_inflight_pair_key = None
        if pair_saved and self._auto_capture_active:
            self._auto_capture_saved_pairs += 1
        if self._auto_capture_active and isinstance(self._auto_capture_pending_pose_views, dict):
            for side in saved_sides:
                view = self._auto_capture_pending_pose_views.get(side)
                if isinstance(view, CalibrationViewGeometry):
                    self._auto_capture_pose_views.setdefault(side, []).append(view)
        self.fail_auto()
