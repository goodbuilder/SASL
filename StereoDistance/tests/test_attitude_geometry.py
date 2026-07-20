import math

from wiris_stereo_distance.attitude_geometry import relative_attitude


def yaw_quaternion(degrees: float):
    radians = math.radians(degrees)
    return (math.cos(radians / 2.0), 0.0, 0.0, math.sin(radians / 2.0))


def test_relative_attitude_reports_yaw_difference() -> None:
    relative = relative_attitude(yaw_quaternion(10), yaw_quaternion(15))

    assert round(relative.yaw_deg, 3) == 5.0
    assert round(relative.roll_deg, 3) == 0.0
    assert round(relative.pitch_deg, 3) == 0.0
