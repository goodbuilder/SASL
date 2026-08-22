from Workstation.workstation_core.stereo import (
    DetectionObservation,
    StereoCalibration,
    match_stereo_observations,
)


def _observation(camera: str, *, timestamp_s: float, y: float, class_id: int = 0):
    return DetectionObservation(
        camera=camera,
        timestamp_s=timestamp_s,
        width=1280,
        height=720,
        x1=500.0 if camera == "left" else 450.0,
        y1=y,
        x2=600.0 if camera == "left" else 550.0,
        y2=y + 40.0,
        confidence=0.9,
        class_id=class_id,
    )


def test_field_pair_keeps_time_and_vertical_quality_as_advisories() -> None:
    match = match_stereo_observations(
        _observation("left", timestamp_s=10.0, y=100.0),
        _observation("right", timestamp_s=11.0, y=300.0),
        StereoCalibration(
            focal_length_px=1000.0,
            baseline_m=1.0,
            max_time_delta_s=0.1,
            max_vertical_delta_px=20.0,
            quality_gates_advisory=True,
        ),
    )

    assert match is not None
    assert any("camera pair delta" in warning for warning in match.warnings)
    assert any("vertical bbox delta" in warning for warning in match.warnings)


def test_field_pair_keeps_correspondence_identity_hard() -> None:
    match = match_stereo_observations(
        _observation("left", timestamp_s=10.0, y=100.0, class_id=0),
        _observation("right", timestamp_s=10.01, y=100.0, class_id=1),
        StereoCalibration(
            focal_length_px=1000.0,
            baseline_m=1.0,
            quality_gates_advisory=True,
        ),
    )

    assert match is None


def test_field_pair_falls_back_from_required_rtcp_to_app_receive_time() -> None:
    match = match_stereo_observations(
        _observation("left", timestamp_s=10.0, y=100.0),
        _observation("right", timestamp_s=10.01, y=100.0),
        StereoCalibration(
            focal_length_px=1000.0,
            baseline_m=1.0,
            timestamp_basis="camera_rtc_required",
            quality_gates_advisory=True,
        ),
    )

    assert match is not None
    assert any("app receive time used" in warning for warning in match.warnings)
