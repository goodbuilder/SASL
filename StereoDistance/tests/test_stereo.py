from __future__ import annotations

from wiris_stereo_distance.stereo import (
    DetectionObservation,
    StereoCalibration,
    detection_timestamp_for_basis,
    match_stereo_observations,
    stereo_estimate_from_match,
)
from wiris_stereo_distance.camera_pane import CameraPane


def observation(camera: str, *, cx: float, cy: float, timestamp_s: float = 10.0, class_id: int = 0):
    return DetectionObservation(
        camera=camera,
        timestamp_s=timestamp_s,
        width=1280,
        height=720,
        x1=cx - 5.0,
        y1=cy - 5.0,
        x2=cx + 5.0,
        y2=cy + 5.0,
        confidence=0.9,
        class_id=class_id,
        class_name="drone",
    )


def test_pairing_accepts_equal_horizontal_pixels_for_rotated_ray_geometry() -> None:
    match = match_stereo_observations(
        observation("left", cx=640.0, cy=360.0),
        observation("right", cx=640.0, cy=360.0),
        StereoCalibration(),
    )

    assert match is not None
    assert match.time_delta_s == 0.0
    assert match.vertical_delta_px == 0.0


def test_pairing_rejects_time_vertical_and_class_contract_violations() -> None:
    left = observation("left", cx=650.0, cy=360.0)
    limits = StereoCalibration(max_time_delta_s=0.1, max_vertical_delta_px=10.0)

    assert match_stereo_observations(
        left,
        observation("right", cx=630.0, cy=360.0, timestamp_s=10.2),
        limits,
    ) is None
    assert match_stereo_observations(
        left,
        observation("right", cx=630.0, cy=380.0),
        limits,
    ) is None
    assert match_stereo_observations(
        left,
        observation("right", cx=630.0, cy=360.0, class_id=2),
        limits,
    ) is None


def test_estimate_carries_ray_range_and_pair_metadata() -> None:
    match = match_stereo_observations(
        observation("left", cx=650.0, cy=360.0),
        observation("right", cx=630.0, cy=361.0),
        StereoCalibration(),
        sync_quality="GOOD",
        relative_roll_deg=0.1,
        relative_pitch_deg=-0.2,
        relative_yaw_deg=0.3,
    )

    assert match is not None
    estimate = stereo_estimate_from_match(match, range_m=42.5)
    assert estimate is not None
    assert estimate.range_m == 42.5
    assert estimate.sync_quality == "GOOD"
    assert estimate.relative_yaw_deg == 0.3


def test_camera_rtcp_timestamp_basis_is_explicit() -> None:
    sample = observation("left", cx=640.0, cy=360.0)
    assert detection_timestamp_for_basis(sample, "camera_rtc_required") is None

    timed = DetectionObservation(
        **{
            **sample.__dict__,
            "timestamp_metadata": {"rtcp_frame_ntp_unix_s": 1234.5},
        }
    )
    assert detection_timestamp_for_basis(timed, "camera_rtc_required") == 1234.5


def test_camera_detection_label_reports_only_raw_bbox_center() -> None:
    class Label:
        text = ""

        def setText(self, value):  # noqa: N802 - Qt-compatible test double.
            self.text = value

    pane = type("Pane", (), {"detection_label": Label()})()
    CameraPane._set_detection_label(pane, observation("left", cx=641.5, cy=359.25))

    assert "center=(641.50, 359.25) px" in pane.detection_label.text
    assert "correct" not in pane.detection_label.text.lower()
