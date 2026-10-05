from copy import deepcopy
from dataclasses import replace
import math
from types import SimpleNamespace

import pytest

from Workstation.alignment_rt_controller import alignment_live_rt_snapshot, build_live_rt_rows
from Workstation.workstation_core.rtk_geometry import GpsPositionState


def panel():
    left = GpsPositionState('left', 'left', 99.9, 'GPS_RAW_INT', latitude_deg=0., longitude_deg=0., altitude_m=50., fix_type=6)
    right = replace(left, camera='right', longitude_deg=math.degrees(2/6378137), altitude_m=49.75)
    attitude = SimpleNamespace(q=(1.,0.,0.,0.), roll_deg=0., pitch_deg=0., yaw_deg=0., timestamp_s=99.8)
    return SimpleNamespace(latest_extrinsics={'left':deepcopy(attitude),'right':deepcopy(attitude)},
        latest_gps={'left':left,'right':right}, settings=SimpleNamespace(),
        _workstation_setting_overrides={}, _measurement_latest_valid_attitude={'untouched':True})


def test_same_math_with_hand_checked_optical_transform_and_no_side_effects():
    p = panel()
    before = deepcopy(vars(p))
    rows = alignment_live_rt_snapshot(p, now_s=100)['rows']
    assert vars(p) == before
    for row in rows:
        assert row['state'] == 'LIVE'
        assert row['translation_right_from_left_rdf_m'] == pytest.approx((-2,-.25,0))
        assert row['right_origin_in_left_rdf_m'] == pytest.approx((2,.25,0))
        assert row['rotation_right_from_left_rdf'] == [[1,0,0],[0,1,0],[0,0,1]]
        assert row['age_s'] == pytest.approx(.2)


def test_actual_attitude_and_yaw90_are_independent_and_reverse_is_live():
    p = panel()
    p.latest_extrinsics['right'].q = (math.sqrt(.5),0,0,math.sqrt(.5))
    p.latest_extrinsics['right'].yaw_deg = 90
    rows = alignment_live_rt_snapshot(p, now_s=100)['rows']
    r, t = rows[0]['rotation_right_from_left_rdf'], rows[0]['translation_right_from_left_rdf_m']
    # A world point [N,E,D]=[4,5,6] seen from the two poses has known RDF coordinates.
    left_point = (5,6,4)
    assert [sum(r[i][j]*left_point[j] for j in range(3))+t[i] for i in range(3)] == pytest.approx((-4,5.75,3))
    assert rows[0]['relative_rpy_deg'][2] == pytest.approx(90)
    assert rows[1]['relative_rpy_deg'][2] == pytest.approx(0)
    p._workstation_setting_overrides['measurement_baseline_yaw_reverse'] = True
    reversed_rows = alignment_live_rt_snapshot(p, now_s=100)['rows']
    assert reversed_rows[0]['translation_right_from_left_rdf_m'] == rows[0]['translation_right_from_left_rdf_m']
    assert reversed_rows[1]['translation_right_from_left_rdf_m'] == pytest.approx((2,-.25,0))


def test_selected_gps2_is_used_instead_of_cached_baseline():
    p = panel()
    p.latest_rtk_baseline = SimpleNamespace(north_m=999,east_m=999,down_m=999)
    left, right = p.latest_gps.values()
    p._selected_baseline_gps_states = lambda: (left, replace(right, longitude_deg=math.degrees(5/6378137)))
    assert alignment_live_rt_snapshot(p, now_s=100)['rows'][0]['translation_right_from_left_rdf_m'][0] == pytest.approx(-5)


def test_stale_data_keeps_values_and_missing_data_keeps_two_rows():
    p = panel()
    for row in alignment_live_rt_snapshot(p, now_s=105)['rows']:
        assert row['state'] == 'STALE'
        assert row['age_s'] == pytest.approx(5.2)
        assert row['rotation_right_from_left_rdf'] is not None
    p.latest_gps.pop('right')
    rows = alignment_live_rt_snapshot(p, now_s=105)['rows']
    assert len(rows) == 2
    assert all(row['state']=='WAITING' and row['translation_right_from_left_rdf_m'] is None for row in rows)


def test_invalid_quaternion_does_not_fabricate_live_matrix():
    p = panel()
    p.latest_extrinsics['right'].q = (0,0,0,0)
    assert alignment_live_rt_snapshot(p, now_s=100)['rows'][0]['state'] == 'WAITING'


def test_vertical_baseline_only_prevents_yaw90():
    p = panel()
    p.latest_gps['right'] = replace(p.latest_gps['right'], longitude_deg=0)
    rows = alignment_live_rt_snapshot(p, now_s=100)['rows']
    assert rows[0]['state'] == 'LIVE'
    assert rows[1]['state'] == 'WAITING'


def test_future_time_and_float_fix_are_visible_quality_states():
    p = panel()
    p.latest_gps['left'] = replace(p.latest_gps['left'], fix_type=5)
    assert alignment_live_rt_snapshot(p, now_s=100)['rows'][0]['state'] == 'WARNING'
    p.latest_extrinsics['left'].timestamp_s = 110
    assert alignment_live_rt_snapshot(p, now_s=100)['rows'][0]['state'] == 'STALE'
