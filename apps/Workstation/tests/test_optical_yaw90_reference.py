import copy
import json
import math

import pytest

from Workstation.workstation_core.optical_yaw90_reference import build_reference, reference_poses
from Workstation.workstation_core.optical_pose import stereo_pose_from_absolute


def inputs():
    return dict(gps={
        "left": dict(latitude_deg=0., longitude_deg=0., altitude_m=10., fix_type=6, timestamp_s=99.),
        "right": dict(latitude_deg=0., longitude_deg=0.00001, altitude_m=10., fix_type=6, timestamp_s=99.),
    }, attitudes={"left": dict(roll_deg=0., pitch_deg=0., yaw_deg=0., timestamp_s=99.),
                  "right": dict(yaw_deg=0., timestamp_s=99.)}, reverse=False,
        requested_monotonic_s=100., requested_utc_ns=1000000000,
        sources={"left": "GPS1", "right": "GPS2"})


def test_north_facing_east_baseline_has_negative_opencv_x_translation():
    args = inputs()
    original = copy.deepcopy(args)
    ref = build_reference(**args)
    assert args == original
    left, right = reference_poses(ref)
    assert left.boresight_ned == pytest.approx((1, 0, 0))
    stereo = stereo_pose_from_absolute(left, right)
    assert stereo.rotation_right_from_left_rdf == ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))
    expected_length = math.radians(.00001) * 6378137.
    assert stereo.translation_right_from_left_rdf_m == pytest.approx((-expected_length, 0, 0))
    args["attitudes"]["left"]["roll_deg"] = 80.
    assert ref["p1_roll_pitch_deg"] == [0., 0.]
    assert json.loads(json.dumps(ref, allow_nan=False)) == ref


@pytest.mark.parametrize("missing", ["gps", "attitude", "horizontal"])
def test_missing_required_geometry_blocks(missing):
    args = inputs()
    if missing == "gps": args["gps"]["left"]["altitude_m"] = float("nan")
    elif missing == "attitude": args["attitudes"]["left"]["roll_deg"] = None
    else: args["gps"]["right"]["longitude_deg"] = 0.
    ref = build_reference(**args)
    assert ref["state"] == "BLOCKED"
    json.dumps(ref, allow_nan=False)
    with pytest.raises(RuntimeError): reference_poses(ref)


def test_reverse_and_aged_low_quality_inputs_are_audited():
    args = inputs()
    args["reverse"] = True
    args["gps"]["left"].update(fix_type=2, timestamp_s=1.)
    args["attitudes"]["right"] = {}
    ref = build_reference(**args)
    assert ref["state"] == "READY"
    assert ref["selection"]["selected_yaw_deg"] == 180.
    assert ref["warnings"]
    assert ref["gps"]["left"]["age_at_request_s"] == 99.
