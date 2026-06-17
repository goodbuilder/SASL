import math

from wiris_stereo_distance.attitude_bias import (
    RelativeAttitudeBias,
    RelativeAttitudeBiasCapture,
    apply_relative_bias,
    relative_attitude,
)


def yaw_quaternion(degrees: float):
    radians = math.radians(degrees)
    return (math.cos(radians / 2.0), 0.0, 0.0, math.sin(radians / 2.0))


def test_relative_attitude_reports_yaw_difference() -> None:
    relative = relative_attitude(yaw_quaternion(10), yaw_quaternion(15))

    assert round(relative.yaw_deg, 3) == 5.0
    assert round(relative.roll_deg, 3) == 0.0
    assert round(relative.pitch_deg, 3) == 0.0


def test_apply_relative_bias_removes_captured_yaw_offset() -> None:
    relative = relative_attitude(yaw_quaternion(10), yaw_quaternion(15))
    corrected = apply_relative_bias(
        relative,
        RelativeAttitudeBias(roll_deg=0.0, pitch_deg=0.0, yaw_deg=5.0, sample_count=1, captured_timestamp_s=1.0),
    )

    assert round(corrected.yaw_deg, 3) == 0.0


def test_relative_attitude_bias_capture_averages_samples() -> None:
    capture = RelativeAttitudeBiasCapture(start_timestamp_s=0.0, duration_s=1.0)
    capture.add(yaw_quaternion(0), yaw_quaternion(4))
    capture.add(yaw_quaternion(0), yaw_quaternion(6))

    bias = capture.finish(captured_timestamp_s=1.1)

    assert bias is not None
    assert bias.sample_count == 2
    assert round(bias.yaw_deg, 3) == 5.0
