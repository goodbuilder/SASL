#!/usr/bin/env python3
"""Offline conservative all-attitude mesh enclosure FOV/declared-clip audit.

Geometry QA only: no image detections, position errors, fits or path rankings.
A sphere enclosing all full-rotor-sweep source bounds is invariant to vehicle
attitude. A world-ENU cube enclosing that sphere therefore contains the mesh
at each linearly aligned reference position, conditional on source/model
identity and the stated zero model/base/reference origin contract.
"""
from __future__ import annotations

import argparse
import bisect
import hashlib
import itertools
import json
import math
from pathlib import Path
import sys
import numpy as np

SIM_ROOT = Path(__file__).resolve().parents[2]
if str(SIM_ROOT) not in sys.path:
    sys.path.insert(0, str(SIM_ROOT))
from campaigns.stereo_v1_1.camera_frame_contract import optical_rwc_from_entity_quaternion_xyzw


def corners(low, high):
    lo, hi = np.asarray(low, dtype=float), np.asarray(high, dtype=float)
    if lo.shape != (3,) or hi.shape != (3,) or not np.isfinite([lo, hi]).all() or np.any(lo > hi):
        raise ValueError('invalid enclosing bounds')
    return np.array(list(itertools.product(*zip(lo, hi))))


def enclosing_radius(low, high):
    radius = float(np.linalg.norm(corners(low, high), axis=1).max())
    if radius <= 0:
        raise ValueError('mesh enclosure radius must be positive')
    return radius


class PositionSeries:
    """World ENU metre positions, unique simulation ns; no orientation needed."""
    def __init__(self, rows):
        self.rows = {}
        for row in rows:
            if row.get('position_frame') != 'gazebo_world_enu' or row.get('reference_link') != 'vehicle_reference_link':
                raise ValueError('GT must declare world ENU vehicle_reference_link')
            point = np.array(row['position_world_enu_m'], dtype=float)
            if point.shape != (3,) or not np.isfinite(point).all():
                raise ValueError('invalid world ENU position')
            self.rows.setdefault(row['simulation_epoch'], []).append((int(row['source_timestamp_ns']), point))
        self.times = {}
        for epoch, rows in self.rows.items():
            rows.sort(key=lambda r:r[0])
            self.times[epoch] = [r[0] for r in rows]
            if len(set(self.times[epoch])) != len(rows):
                raise ValueError('duplicate GT timestamp')

    def align(self, epoch, stamp):
        times = self.times.get(epoch, [])
        i = bisect.bisect_left(times, stamp)
        if i < len(times) and times[i] == stamp:
            return self.rows[epoch][i][1], 0
        if i == 0 or i == len(times) or times[i]-times[i-1] > 40_000_000:
            raise ValueError('no same-epoch GT bracket within 0.04s; extrapolation forbidden')
        before, after = self.rows[epoch][i-1], self.rows[epoch][i]
        alpha = (stamp-before[0])/(after[0]-before[0])
        return before[1]*(1-alpha)+after[1]*alpha, after[0]-before[0]


def project_enclosure(position, radius, camera_center, rwc, info, near, far, margin):
    """p_C=R_WC.T(p_W-C_W); camera RDF metres -> original u/v pixels."""
    k = np.asarray(info['k'], dtype=float).reshape(3,3)
    if (not np.isfinite(k).all() or k[0,0] <= 0 or k[1,1] <= 0
            or not np.allclose(k[2], [0,0,1], rtol=0, atol=1e-12)
            or abs(k[0,1])+abs(k[1,0]) > 1e-12
            or any(not math.isfinite(float(v)) or abs(v)>1e-12 for v in info.get('d', []))
            or info['width'] <= 0 or info['height'] <= 0):
        raise ValueError('invalid or unsupported CameraInfo K/distortion/dimensions')
    if not all(math.isfinite(v) for v in [radius,near,far,margin]) or not 0 < radius or not 0 <= near < far or margin < 0:
        raise ValueError('invalid radius, clip planes or margin')
    p = np.asarray(position, dtype=float)
    c = np.asarray(camera_center, dtype=float)
    rotation = np.asarray(rwc, dtype=float)
    if (p.shape != (3,) or c.shape != (3,) or rotation.shape != (3,3)
            or not np.isfinite(p).all() or not np.isfinite(c).all() or not np.isfinite(rotation).all()
            or not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-8)
            or not math.isclose(float(np.linalg.det(rotation)),1,abs_tol=1e-8)):
        raise ValueError('invalid world ENU point or optical RDF-to-world rotation')
    optical = (corners(p-radius,p+radius)-c)@rotation
    depths = optical[:,2]
    result = {'status':'BEHIND_OR_NEAR_CLIPPED','minimum_optical_depth_m':float(depths.min()),
              'maximum_optical_depth_m':float(depths.max()),'projected_enclosure_xyxy_px':None,
              'minimum_edge_margin_px':None}
    if depths.min() <= near:
        return result
    pixels = optical@k.T
    uv = pixels[:,:2]/pixels[:,2,None]
    lo,hi = uv.min(axis=0),uv.max(axis=0)
    edge = float(min(lo[0],lo[1],info['width']-hi[0],info['height']-hi[1]))
    result.update(projected_enclosure_xyxy_px=[float(lo[0]),float(lo[1]),float(hi[0]),float(hi[1])],
                  minimum_edge_margin_px=edge)
    result['status'] = 'FAR_CLIPPED' if depths.max() >= far else 'OUTSIDE_IMAGE' if edge < 0 else 'MARGIN_VIOLATION' if edge < margin else 'ENCLOSED'
    return result


def rows(path):
    with path.open() as handle:
        for line in handle:
            yield json.loads(line)


def static_camera_poses(path, truth, epoch, start, end):
    """Verify recorded diagnostics stay at the selected static pose and bracket science."""
    coverage = {side:[] for side in ('left','right')}
    for row in rows(path):
        if row.get('simulation_epoch') != epoch:
            continue
        side = row['camera_id']
        payload = row['payload']
        expected = truth[side]
        position = payload['observed_optical_position_world_enu_m']
        q = np.array(payload['observed_optical_orientation_world_from_entity_xyzw'],dtype=float)
        q0 = np.array(expected['observed_optical_orientation_world_from_entity_xyzw'],dtype=float)
        if (not np.allclose(position,expected['observed_optical_position_world_enu_m'],rtol=0,atol=1e-6)
                or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-6
                or abs(abs(float(q@q0))-1)>1e-8):
            raise ValueError('camera is not static within recorded pose tolerance')
        coverage[side].append(payload['gazebo_stamp_ns'])
    if any(not stamps or min(stamps)>start or max(stamps)<end for stamps in coverage.values()):
        raise ValueError('recorded static camera pose does not bracket science window')


def audit(attempt: Path, geometry_path: Path, margin=32.):
    load=lambda name:json.loads((attempt/name).read_text())
    geometry=json.loads(geometry_path.read_text())
    manifest=load('run_manifest.json')
    if (geometry.get('status')!='PASS_SOURCE_GEOMETRY_MEASUREMENT'
            or geometry.get('model_id')!=manifest.get('vehicle_model_id')
            or 'zero transform' not in geometry.get('reference_definition','')
            or manifest.get('vehicle_reference_link')!='vehicle_reference_link'):
        raise ValueError('geometry model/reference evidence does not match attempt')
    bounds=geometry['visual_full_rotor_sweep_enclosing_bounds']
    radius=enclosing_radius(bounds['minimum_xyz_m'],bounds['maximum_xyz_m'])
    events=list(rows(attempt/'events.jsonl'))
    starts=[r for r in events if r['event']=='science_start'];ends=[r for r in events if r['event']=='science_end']
    if len(starts)!=1 or len(ends)!=1 or starts[0]['simulation_epoch']!=ends[0]['simulation_epoch']:
        raise ValueError('one complete same-epoch science window required')
    start,end=starts[0]['simulation_timestamp_ns'],ends[0]['simulation_timestamp_ns'];epoch=starts[0]['simulation_epoch']
    if start>=end:raise ValueError('invalid science window')
    info=load('image_index_summary.json')['camera_info'];truth=load('camera_truth.json')['camera_payloads']
    static_camera_poses(attempt/'camera_pose_diagnostics.jsonl',truth,epoch,start,end)
    config={}
    for line in (attempt/'effective_scenario.env').read_text().splitlines():
        if line and not line.startswith('#') and '=' in line:
            key,value=line.split('=',1);config[key]=value.strip().strip('"').strip("'")
    near,far=float(config['CAMERA_CLIP_NEAR_M']),float(config['CAMERA_CLIP_FAR_M'])
    series=PositionSeries(rows(attempt/'ground_truth.jsonl'))
    cameras={};all_pass=True
    index=list(rows(attempt/'image_index.jsonl'))
    for side in ('left','right'):
        if info[side]['message_count']<=0 or len(info[side]['variants'])!=1:
            raise ValueError('one stable recorded CameraInfo required')
        ci=info[side]['variants'][0];pose=truth[side]
        rwc=optical_rwc_from_entity_quaternion_xyzw(pose['observed_optical_orientation_world_from_entity_xyzw'])
        frames=[]
        for image in index:
            if image['camera_id']!=side or image['simulation_epoch']!=epoch or not start<=image['image_timestamp_ns']<=end:continue
            if image['image_width_px']!=ci['width'] or image['image_height_px']!=ci['height']:raise ValueError('source image and CameraInfo dimensions differ')
            try:
                point,gap=series.align(epoch,image['image_timestamp_ns'])
            except ValueError as exc:
                frames.append({'frame_id':image['frame_id'],'status':'NO_GT_BRACKET','reason':str(exc)});continue
            result=project_enclosure(point,radius,pose['observed_optical_position_world_enu_m'],rwc,ci,near,far,margin)
            frames.append({'frame_id':image['frame_id'],'image_timestamp_ns':image['image_timestamp_ns'],'gt_bracketing_gap_ns':gap,**result})
        passed=bool(frames and all(r['status']=='ENCLOSED' for r in frames));all_pass &= passed
        margins=[r['minimum_edge_margin_px'] for r in frames if r.get('minimum_edge_margin_px') is not None]
        depths=[r for r in frames if 'minimum_optical_depth_m' in r]
        cameras[side]={'status':'PASS_CONDITIONAL_ENCLOSURE' if passed else 'FAILED','science_frame_count':len(frames),
            'minimum_edge_margin_px':min(margins) if margins else None,
            'minimum_optical_depth_m':min(r['minimum_optical_depth_m'] for r in depths) if depths else None,
            'maximum_optical_depth_m':max(r['maximum_optical_depth_m'] for r in depths) if depths else None,
            'failures':[r for r in frames if r['status']!='ENCLOSED']}
    files=['run_manifest.json','events.jsonl','image_index.jsonl','image_index_summary.json','ground_truth.jsonl',
           'camera_truth.json','camera_pose_diagnostics.jsonl','effective_scenario.env','raw_capture_summary.json']
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    return {'method':'full_rotor_mesh_all_attitude_enclosure_v1','status':'PASS_CONDITIONAL_ENCLOSURE' if all_pass else 'FAILED',
        'uses_position_accuracy_metrics':False,'changes_science_selection':False,'model_id':geometry['model_id'],
        'geometry_report_sha256':digest(geometry_path),'source_model_sdf_sha256':geometry['model_sdf_sha256'],
        'source_mesh_assets':[{'uri':m['uri'],'sha256':m['sha256']} for m in geometry['meshes']],
        'historical_loaded_model_hash_verified':False,'enclosing_radius_m':radius,
        'construction':'max corner norm about zero model/base/reference origin; world ENU cube plus/minus radius contains all rotated mesh orientations',
        'minimum_margin_px':margin,'margin_source':'engineering_guard_not_protocol_threshold',
        'declared_near_clip_m':near,'declared_far_clip_m':far,'clip_plane_source':'effective_scenario.env; not a measurement of actual renderer clip planes',
        'camera_pose_contract':'recorded Gazebo link FLU XYZW converted to world-from-optical RDF; static trace checked',
        'gt_alignment':'same-epoch position linear interpolation, bracket <=0.04s, no extrapolation; orientation unnecessary',
        'assumptions':['measured source mesh bounds apply to the historically loaded model; model ID alone does not verify historical asset hashes',
            'model origin and verified zero base_link/vehicle_reference_link origin coincide',
            'recorded static camera pose and pinhole CameraInfo describe renderer; no distortion',
            'GT linear interpolation represents reference position at exposure; no error bound on intersample dynamics',
            'declared clip planes were applied by renderer; not independently observed here',
            'enclosure only; no visibility/occlusion/contrast or full silhouette claim'],
        'input_sha256':{name:digest(attempt/name) for name in files},'audit_code_sha256':digest(Path(__file__)), 'cameras':cameras}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--attempt-dir',type=Path,required=True);parser.add_argument('--geometry-report',type=Path,required=True)
    parser.add_argument('--report',type=Path,required=True);parser.add_argument('--minimum-margin-px',type=float,default=32.)
    args=parser.parse_args()
    if args.report.resolve().is_relative_to(args.attempt_dir.resolve()):raise ValueError('report must be outside immutable attempt')
    if args.report.exists():raise FileExistsError(args.report)
    report=audit(args.attempt_dir,args.geometry_report,args.minimum_margin_px)
    with args.report.open('x') as handle:json.dump(report,handle,indent=2,allow_nan=False);handle.write('\n')
    print(json.dumps({'status':report['status'],'cameras':report['cameras']}))
    return 0 if report['status']=='PASS_CONDITIONAL_ENCLOSURE' else 1


if __name__=='__main__':raise SystemExit(main())
