import json
import math

import pytest

from Workstation.workstation_core.target_coordinates import (
    TARGET_COORDINATE_FIELDS, ecef_to_geodetic_m, ned_to_wgs84, target_wgs84_from_attempt,
)


def attempt(q=(1., 0., 0., 0.)):
    return {"triangulation_source": "pixhawk", "left_gps": {
        "latitude_deg": 0., "longitude_deg": 0., "altitude_ellipsoid_m": 100., "fix_type": 6,
    }, "left_attitude": {"quaternion_wxyz": q}}


def test_ecef_axis_and_equatorial_east_independent_expected():
    # At lat=lon=0: ECEF X=up, Y=east, Z=north.
    assert ecef_to_geodetic_m((6378137+123, 0, 0)) == pytest.approx((0, 0, 123), abs=1e-8)
    b = 6356752.314245179
    assert ecef_to_geodetic_m((0, 0, b+123)) == pytest.approx((90, 0, 123), abs=1e-8)
    east = 1000
    radius = 6378137+100
    assert ned_to_wgs84((0, 0, 100), (0, east, 0)) == pytest.approx(
        (0, math.degrees(math.atan2(east, radius)), math.hypot(radius, east)-6378137), abs=1e-8)


def test_optical_down_and_altitude_sign():
    result = target_wgs84_from_attempt(attempt(), (0, 20, 0))
    assert result["target_altitude_ellipsoid_m"] == pytest.approx(80)
    assert result["target_altitude_msl_m"] is None


def test_world_target_invariant_under_camera_tilt():
    original = target_wgs84_from_attempt(attempt(), (12, -30, 1000))
    theta = math.radians(25)
    # Independent inverse pitch rotation on body [F,R,D]=[1000,12,-30].
    f = math.cos(theta)*1000 - math.sin(theta)*(-30)
    d = math.sin(theta)*1000 + math.cos(theta)*(-30)
    changed = target_wgs84_from_attempt(attempt((math.cos(theta/2), 0, math.sin(theta/2), 0)), (12, d, f))
    for name in TARGET_COORDINATE_FIELDS[:3]:
        assert changed[name] == pytest.approx(original[name], abs=1e-8)


def test_geometry_quaternion_wins_over_raw_attitude():
    data = attempt()
    data["geometry_left_quaternion_wxyz"] = (math.sqrt(.5), 0, 0, math.sqrt(.5))
    result = target_wgs84_from_attempt(data, (0, 0, 100))
    assert result["target_longitude_deg"] > 0
    assert abs(result["target_latitude_deg"]) < 1e-10


def test_relative_pose_requires_explicit_anchor_and_uses_its_rotation():
    data = attempt()
    data["triangulation_source"] = "checkerboard"
    assert target_wgs84_from_attempt(data, (0, 0, 100))["target_latitude_deg"] is None
    data["measurement_pose_evidence_json"] = json.dumps({
        "target_origin_model": "P1_GNSS_as_left_optical_center",
        "left_pose": {"rotation_ned_from_camera_rdf": [[1,0,0],[0,0,1],[0,-1,0]]},
    })
    result = target_wgs84_from_attempt(data, (0, 0, 100))
    assert result["target_longitude_deg"] > 0
    assert abs(result["target_latitude_deg"]) < 1e-10


@pytest.mark.parametrize("key,value", [("altitude_ellipsoid_m", None), ("latitude_deg", float('nan')), ("longitude_deg", 181), ("fix_type", 0)])
def test_unavailable_origin_does_not_fabricate_zero(key, value):
    data = attempt()
    data["left_gps"][key] = value
    result = target_wgs84_from_attempt(data, (0,0,100))
    assert all(result[name] is None for name in TARGET_COORDINATE_FIELDS)
    assert result["target_coordinate_evidence"]["reason"]


def test_antimeridian_and_pole_are_finite():
    lat, lon, height = ned_to_wgs84((0,179.9999,20), (0,100,0))
    assert -180 <= lon < -179
    assert all(math.isfinite(x) for x in ned_to_wgs84((90,0,20), (100,100,0)))


def test_midlatitude_against_independent_ellipsoid_normal_fixture():
    # Hand construction with WGS84 flattening, independent of production forward conversion.
    a, flattening = 6378137., 1/298.257223563
    e2 = flattening*(2-flattening)
    for latitude in (-89.999, -45., 35., 89.999):
        phi, lam, height = math.radians(latitude), math.radians(127), 1234.
        radius = a / math.sqrt(1-e2*math.sin(phi)**2)
        xyz = ((radius+height)*math.cos(phi)*math.cos(lam),
               (radius+height)*math.cos(phi)*math.sin(lam),
               (radius*(1-e2)+height)*math.sin(phi))
        assert ecef_to_geodetic_m(xyz) == pytest.approx((latitude,127,height), abs=1e-8)
