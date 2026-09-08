"""Target geolocation from frozen left optical RDF, never baseline-center FRD.

The P1 GNSS position is the left optical-origin proxy; camera/body mounting
rotation is retained. No geoid model is assumed, so target MSL stays unknown.
"""
from __future__ import annotations

import json
import math
from collections.abc import Mapping

from Workstation.workstation_core.attitude_geometry import rotate_vector_by_quaternion
from Workstation.workstation_core.gnss_profile import (
    WGS84_SEMI_MAJOR_M, WGS84_FIRST_ECCENTRICITY_SQUARED, geodetic_to_ecef_m,
)
from Workstation.workstation_core.triangulation import camera_vector_to_body_frd

TARGET_COORDINATE_FIELDS = (
    "target_latitude_deg", "target_longitude_deg",
    "target_altitude_ellipsoid_m", "target_altitude_msl_m",
)


def _number(value):
    if isinstance(value, bool):
        raise ValueError("boolean coordinate")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("non-finite coordinate")
    return result


def _vector(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("expected three coordinate components")
    return tuple(_number(v) for v in value)


def ecef_to_geodetic_m(value):
    """ECEF metres -> WGS84 latitude/longitude degrees, ellipsoid metres."""
    x, y, z = _vector(value)
    a, e2 = WGS84_SEMI_MAJOR_M, WGS84_FIRST_ECCENTRICITY_SQUARED
    p = math.hypot(x, y)
    if math.hypot(p, z) < 1.0:
        raise ValueError("geodetic position undefined near Earth center")
    if p < 1e-9:
        return math.copysign(90.0, z), 0.0, abs(z) - a * math.sqrt(1.0 - e2)
    latitude = math.atan2(z, p * (1.0 - e2))
    for _ in range(20):
        n = a / math.sqrt(1.0 - e2 * math.sin(latitude) ** 2)
        updated = math.atan2(z + e2 * n * math.sin(latitude), p)
        if abs(updated - latitude) < 1e-15:
            latitude = updated
            break
        latitude = updated
    # Projection onto the ellipsoid normal remains well-conditioned at poles.
    height = p * math.cos(latitude) + z * math.sin(latitude) - a * math.sqrt(
        1.0 - e2 * math.sin(latitude) ** 2
    )
    return math.degrees(latitude), math.degrees(math.atan2(y, x)), height


def ned_to_wgs84(origin, ned):
    """origin [lat deg, lon deg, ellipsoid m] + [N,E,D] m -> geodetic."""
    lat, lon, height = _vector(origin)
    if not -90 <= lat <= 90 or not -180 <= lon <= 180:
        raise ValueError("invalid geodetic origin")
    north, east, down = _vector(ned)
    phi, lam = math.radians(lat), math.radians(lon)
    sp, cp, sl, cl = math.sin(phi), math.cos(phi), math.sin(lam), math.cos(lam)
    delta = (-sp * cl * north - sl * east - cp * cl * down,
             -sp * sl * north + cl * east - cp * sl * down,
             cp * north - sp * down)
    base = geodetic_to_ecef_m(lat, lon, height)
    return ecef_to_geodetic_m(tuple(a + b for a, b in zip(base, delta)))


def target_wgs84_from_attempt(attempt: Mapping, source_point_left_rdf_m) -> dict:
    """Return nullable public fields and replayable evidence from one attempt.

    Missing georeferencing affects only these extra outputs. It never blocks
    the existing camera-relative triangulation result.
    """
    output = dict.fromkeys(TARGET_COORDINATE_FIELDS)
    evidence = {
        "schema": "target-geolocation-v1", "reason": "",
        "origin_model": "P1_GNSS_as_left_optical_center",
        "pixhawk_optical_center_offset_m": [0.0, 0.0, 0.0],
        "msl_reason": "target_geoid_height_unavailable",
    }
    output["target_coordinate_evidence"] = evidence
    try:
        point = _vector(source_point_left_rdf_m)
        gps = attempt.get("left_gps")
        if not isinstance(gps, Mapping):
            raise ValueError("left_GNSS_origin_unavailable")
        if int(gps.get("fix_type") or 0) < 3:
            raise ValueError("left_GNSS_position_fix_unavailable")
        origin = _vector((gps.get("latitude_deg"), gps.get("longitude_deg"), gps.get("altitude_ellipsoid_m")))
        if attempt.get("triangulation_source") == "pixhawk":
            attitude = attempt.get("left_attitude") or {}
            q = attempt.get("geometry_left_quaternion_wxyz") or attitude.get("quaternion_wxyz")
            if q is None:
                raise ValueError("world_rotation_unavailable")
            body = camera_vector_to_body_frd(point)
            ned = rotate_vector_by_quaternion(body, q)
            evidence["body_to_ned_quaternion_wxyz"] = list(q)
        else:
            poses = json.loads(attempt.get("measurement_pose_evidence_json") or "{}")
            if poses.get("target_origin_model") != "P1_GNSS_as_left_optical_center":
                raise ValueError("relative_profile_world_anchor_unavailable")
            rotation = poses.get("left_pose", {}).get("rotation_ned_from_camera_rdf")
            if not isinstance(rotation, (tuple, list)) or len(rotation) != 3:
                raise ValueError("world_rotation_unavailable")
            rotation = tuple(_vector(row) for row in rotation)
            # Validate a proper rotation before accepting an external profile.
            for i in range(3):
                for j in range(3):
                    if abs(sum(rotation[i][k] * rotation[j][k] for k in range(3)) - (i == j)) > 1e-6:
                        raise ValueError("invalid_world_rotation")
            cross = (rotation[0][1]*rotation[1][2]-rotation[0][2]*rotation[1][1],
                     rotation[0][2]*rotation[1][0]-rotation[0][0]*rotation[1][2],
                     rotation[0][0]*rotation[1][1]-rotation[0][1]*rotation[1][0])
            if abs(sum(cross[i]*rotation[2][i] for i in range(3))-1) > 1e-6:
                raise ValueError("invalid_world_rotation")
            ned = tuple(sum(row[i] * point[i] for i in range(3)) for row in rotation)
            evidence["rotation_ned_from_left_optical"] = rotation
        lat, lon, height = ned_to_wgs84(origin, ned)
        output.update(zip(TARGET_COORDINATE_FIELDS[:3], (lat, lon, height)))
        evidence.update(origin_wgs84=origin, target_from_left_ned_m=ned,
                        source_point_left_rdf_m=point)
    except (TypeError, ValueError, KeyError, OverflowError, AttributeError) as exc:
        evidence["reason"] = str(exc)
    return output


def target_coordinates_from_attempt(attempt: Mapping, source_point_left_rdf_m) -> dict:
    """Independent additive WGS84 and method-midpoint level coordinates."""
    from Workstation.workstation_core.target_level_coordinates import target_level_from_attempt
    return {
        **target_wgs84_from_attempt(attempt, source_point_left_rdf_m),
        **target_level_from_attempt(attempt, source_point_left_rdf_m),
    }
