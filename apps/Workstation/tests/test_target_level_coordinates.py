import math
import pytest
from Workstation.workstation_core.target_level_coordinates import target_level_from_attempt, LEVEL_COORDINATE_FIELDS


def attempt(q=(1., 0., 0., 0.), baseline=(2., 0., 0.)):
    return {"left_attitude": {"quaternion_wxyz": q, "timestamp_s": 1.},
            "baseline": {"translation_right_in_left_rdf_m": baseline}}


def test_level_midpoint_and_height_difference():
    result = target_level_from_attempt(attempt(baseline=(4., 6., 2.)), (12., 23., 101.))
    assert tuple(result[k] for k in LEVEL_COORDINATE_FIELDS) == pytest.approx((100., 10., 20., math.sqrt(10500)))


def test_eight_degree_down_tilt():
    t = math.radians(-8)
    result = target_level_from_attempt(attempt((math.cos(t/2), 0., math.sin(t/2), 0.)), (1., 0., 100.))
    assert tuple(result[k] for k in LEVEL_COORDINATE_FIELDS) == pytest.approx((100*math.cos(t), 0., -100*math.sin(t), 100.))


def test_roll_and_common_heading_ignore_method_attitude():
    a = attempt((math.sqrt(.5), math.sqrt(.5), 0., 0.))
    a['geometry_left_quaternion_wxyz'] = (0., 0., 0., 1.)
    result = target_level_from_attempt(a, (11., 20., 100.))
    assert tuple(result[k] for k in LEVEL_COORDINATE_FIELDS) == pytest.approx((100., -20., 10., math.sqrt(10500)))


def test_yaw_does_not_rotate_relative_horizontal_components():
    a = attempt((math.sqrt(.5), 0., 0., math.sqrt(.5)))
    result = target_level_from_attempt(a, (11., 20., 100.))
    assert tuple(result[k] for k in LEVEL_COORDINATE_FIELDS) == pytest.approx((100., 10., 20., math.sqrt(10500)))


@pytest.mark.parametrize('q', [None, (0,0,0,0), (float('nan'),0,0,0), (math.sqrt(.5),0,math.sqrt(.5),0)])
def test_bad_or_vertical_attitude_keeps_outputs_unknown(q):
    result = target_level_from_attempt(attempt(q), (1,2,3))
    assert all(result[k] is None for k in LEVEL_COORDINATE_FIELDS)
    assert result['target_level_status'] == 'UNAVAILABLE'
    assert result['target_level_reason']


def test_independent_of_gnss_and_preserves_method_origin():
    a = target_level_from_attempt(attempt(baseline=(2,0,0)), (10,0,100))
    b = target_level_from_attempt(attempt(baseline=(4,0,0)), (10,0,100))
    assert a['target_level_right_m'] == 9
    assert b['target_level_right_m'] == 8
    assert a['target_level_evidence']['forward_axis_ned'] == b['target_level_evidence']['forward_axis_ned']


def test_relative_method_uses_its_actual_rt_midpoint_and_preserves_range():
    from dataclasses import replace
    from Workstation.tests.measurement_fixtures import checkerboard_attempt, pixhawk_attempt, frozen_snapshot, outcome_for
    a = replace(checkerboard_attempt(), left_attitude=pixhawk_attempt().left_attitude)
    result = outcome_for(a, frozen_snapshot(a)).to_record()['result']
    assert result['target_level_status'] == 'AVAILABLE'
    assert result['target_level_range_m'] == pytest.approx(result['triangulated']['range_m'], abs=1e-10)
    # T=(-.4,0,0), R is a one-degree Y rotation: right origin is -R.T @ T.
    t = math.radians(1)
    assert result['target_level_evidence']['origin_left_rdf_m'] == pytest.approx((.2*math.cos(t), 0, .2*math.sin(t)))


def test_missing_attitude_does_not_change_original_worker_geometry():
    from dataclasses import replace
    from Workstation.tests.measurement_fixtures import checkerboard_attempt, pixhawk_attempt, frozen_snapshot, outcome_for
    a = checkerboard_attempt()
    b = replace(a, left_attitude=pixhawk_attempt().left_attitude)
    missing = outcome_for(a, frozen_snapshot(a)).to_record()['result']
    available = outcome_for(b, frozen_snapshot(b)).to_record()['result']
    assert missing['triangulated'] == available['triangulated']
    assert missing['target_level_status'] == 'UNAVAILABLE'
    assert available['target_level_status'] == 'AVAILABLE'


@pytest.mark.parametrize('point,baseline', [(None, (2,0,0)), ((1,2,float('inf')), (2,0,0)), ((1,2,3), None), ((1,2,3), (0,0,0))])
def test_missing_or_invalid_point_and_origin(point, baseline):
    result = target_level_from_attempt(attempt(baseline=baseline), point)
    assert result['target_level_status'] == 'UNAVAILABLE'
    assert all(result[k] is None for k in LEVEL_COORDINATE_FIELDS)
