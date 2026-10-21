"""Conservative source bounds and original-image projection, not accuracy tests."""
import numpy as np
import pytest
from campaigns.stereo_v1_1.mesh_enclosure_audit import corners, enclosing_radius, project_enclosure, PositionSeries


def camera():
    return {'width':200,'height':200,'k':[100,0,100,0,100,100,0,0,1],'d':[]}


def test_hand_projection_of_axis_aligned_cube():
    result=project_enclosure([0,0,10],1,[0,0,0],np.eye(3),camera(),.1,1000,32)
    assert result['status']=='ENCLOSED'
    assert result['minimum_optical_depth_m']==9
    assert result['maximum_optical_depth_m']==11
    assert result['projected_enclosure_xyxy_px']==pytest.approx([100-100/9,100-100/9,100+100/9,100+100/9])


def test_radius_encloses_rotated_source_bounds_all_attitudes():
    low,high=[-2,-1,-.5],[1,3,1]
    radius=enclosing_radius(low,high)
    assert radius==pytest.approx(np.sqrt(14))
    for angle in [0,.2,1.5,3.1]:
        c,s=np.cos(angle),np.sin(angle)
        rotation=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        assert np.all(np.abs(corners(low,high)@rotation)<=radius+1e-12)
    rng=np.random.default_rng(123)
    for _ in range(8):
        rotation,_=np.linalg.qr(rng.normal(size=(3,3)))
        assert np.all(np.abs(corners(low,high)@rotation)<=radius+1e-12)


def test_no_gt_bracket_never_extrapolates_or_crosses_epoch():
    def row(stamp,pos):return {'position_frame':'gazebo_world_enu','reference_link':'vehicle_reference_link','simulation_epoch':0,'source_timestamp_ns':stamp,'position_world_enu_m':pos}
    series=PositionSeries([row(0,[0,0,0]),row(40_000_000,[4,0,0])])
    p,gap=series.align(0,20_000_000)
    assert p.tolist()==[2,0,0] and gap==40_000_000
    for epoch,stamp in [(1,20_000_000),(0,50_000_000),(0,-1)]:
        with pytest.raises(ValueError,match='bracket'):series.align(epoch,stamp)
    series=PositionSeries([row(0,[0,0,0]),row(40_000_001,[4,0,0])])
    with pytest.raises(ValueError,match='bracket'):series.align(0,20_000_000)


def test_behind_camera_far_clip_and_invalid_k_fail():
    assert project_enclosure([0,0,0],1,[0,0,0],np.eye(3),camera(),.1,1000,32)['status']=='BEHIND_OR_NEAR_CLIPPED'
    assert project_enclosure([0,0,1000],1,[0,0,0],np.eye(3),camera(),.1,1000,32)['status']=='FAR_CLIPPED'
    info=camera();info['k'][0]=float('nan')
    with pytest.raises(ValueError,match='CameraInfo'):project_enclosure([0,0,10],1,[0,0,0],np.eye(3),info,.1,1000,32)
