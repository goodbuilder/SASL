#!/usr/bin/env python3
"""Create reference paths and validate this protocol. Does not run Gazebo or a flight controller.
Python 3 standard library only. Usage: python3 05_generate_reference.py --output-dir ./reference_generated
"""
import argparse, bisect, json, math
from pathlib import Path

def point(s, p, angle):
    D=p['D_start_m']+(p['D_end_m']-p['D_start_m'])*s
    amp=p['amplitude_m']
    x=0.0 if p['shape']=='straight' else amp*math.sin(math.pi*s)**2 if p['shape']=='bow' else amp*math.sin(2*math.pi*s)**3
    return [x,D,D*math.tan(angle)]

def tangent_and_second(s,p,angle):
    amp=p['amplitude_m']; dy=p['D_end_m']-p['D_start_m']
    if p['shape']=='straight': dx=0.0; ddx=0.0
    elif p['shape']=='bow':
        dx=amp*math.pi*math.sin(2*math.pi*s)
        ddx=2*amp*math.pi**2*math.cos(2*math.pi*s)
    else:
        k=2*math.pi; ss=math.sin(k*s); cc=math.cos(k*s)
        dx=3*amp*k*ss*ss*cc
        ddx=3*amp*k*k*(2*ss*cc*cc-ss**3)
    return [dx,dy,dy*math.tan(angle)],[ddx,0.0,0.0]

def make_path(p, angle, vmax, tr, hz):
    n=10000
    pts=[point(i/n,p,angle) for i in range(n+1)]
    arc=[0.0]
    for a,b in zip(pts,pts[1:]): arc.append(arc[-1]+math.dist(a,b))
    L=arc[-1]
    if L < vmax*tr: raise ValueError('Path too short for requested ramp and cruise')
    T=L/vmax+tr
    times=[i/hz for i in range(int(math.floor(T*hz))+1)]
    if times[-1]<T: times.append(T)
    rows=[]
    for t in times:
        if t<tr:
            dist=vmax*.5*(t-tr/math.pi*math.sin(math.pi*t/tr))
            speed=vmax*.5*(1-math.cos(math.pi*t/tr)); speed_dot=vmax*.5*math.pi/tr*math.sin(math.pi*t/tr)
        elif t<=T-tr:
            dist=vmax*.5*tr+vmax*(t-tr)
            speed=vmax; speed_dot=0.0
        else:
            q=t-(T-tr)
            dist=L-vmax*.5*tr+vmax*.5*(q+tr/math.pi*math.sin(math.pi*q/tr))
            speed=vmax*.5*(1+math.cos(math.pi*q/tr)); speed_dot=-vmax*.5*math.pi/tr*math.sin(math.pi*q/tr)
        dist=max(0,min(L,dist))
        i=max(1,min(n,bisect.bisect_left(arc,dist)))
        f=(dist-arc[i-1])/(arc[i]-arc[i-1])
        s=((i-1)+f)/n
        tangent,second=tangent_and_second(s,p,angle); norm=math.sqrt(sum(x*x for x in tangent))
        dot=sum(x*y for x,y in zip(tangent,second))
        velocity=[x/norm*speed for x in tangent]
        accel=[tangent[j]/norm*speed_dot+(second[j]/norm**2-tangent[j]*dot/norm**4)*speed**2 for j in range(3)]
        rows.append({'t_science_s':round(t,8),'s':s,'position_world_enu_m':point(s,p,angle),
                     'velocity_world_enu_mps':velocity,'acceleration_world_enu_mps2':accel,'heading_deg_clockwise_from_north':180})
    return {'length_3d_m':L,'science_duration_s':T,'reference_update_hz':hz,'samples':rows}

def matmul(a,b): return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def pose(psi,R0):
    q=math.radians(-psi); c=math.cos(q); s=math.sin(q)
    return matmul([[c,-s,0],[s,c,0],[0,0,1]],R0)

def validate(spec,manifest,refs):
    runs=manifest['runs']; assert len(runs)==39
    assert len({r['run_id'] for r in runs})==39
    assert sum(r['rig_group']=='G0' for r in runs)==30
    assert sum(r['rig_group']=='G1' for r in runs)==9
    assert len({r['seed_requested'] for r in runs})==39
    cam=spec['camera']; R0=spec['coordinates']['R_WC_nominal']
    report={'status':'REFERENCE_GEOMETRY_PASS','not_a_gazebo_execution':True,'checks':[]}
    report['motion_checks']=[]
    for name,ref in refs.items():
        max_speed=max(math.sqrt(sum(q*q for q in row['velocity_world_enu_mps'])) for row in ref['samples'])
        max_accel=max(math.sqrt(sum(q*q for q in row['acceleration_world_enu_mps2'])) for row in ref['samples'])
        assert max_speed<=spec['motion']['cruise_speed_3d_mps']+1e-8
        assert max_accel<=spec['motion']['preflight_desired_acceleration_limit_mps2']
        p=spec['trajectories'][name]
        assert math.dist(ref['samples'][0]['position_world_enu_m'],point(0,p,math.radians(20)))<1e-8
        assert math.dist(ref['samples'][-1]['position_world_enu_m'],point(1,p,math.radians(20)))<1e-8
        report['motion_checks'].append({'path':name,'max_reference_speed_mps':max_speed,'max_reference_acceleration_mps2':max_accel})
    for group,vals in cam['physical_pose_groups'].items():
        for name,ref in refs.items():
            if group=='G1' and name not in ['P0','P4','V0']: continue
            best=1e9
            for eye in ['left','right']:
                C=cam[eye+'_center_world_m']; R=pose(vals['heading_'+eye+'_deg'],R0)
                for row in ref['samples']:
                    v=[row['position_world_enu_m'][i]-C[i] for i in range(3)]
                    pc=[sum(R[k][i]*v[k] for k in range(3)) for i in range(3)]
                    assert pc[2]>0
                    u=cam['fx_px']*pc[0]/pc[2]+cam['cx_px']; vv=cam['fy_px']*pc[1]/pc[2]+cam['cy_px']
                    best=min(best,u,vv,cam['image_width_px']-u,cam['image_height_px']-vv)
            assert best>=32, (group,name,best)
            report['checks'].append({'rig':group,'path':name,'minimum_reference_point_edge_margin_px':round(best,3)})
    report['limitation']='Reference-point geometry only. Verify actual mesh/bbox extent, dynamics, tracking and raw-image recording in student Gazebo.'
    return report

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--output-dir',default='reference_generated'); args=ap.parse_args()
    here=Path(__file__).resolve().parent
    spec=json.loads((here/'03_experiment_spec.json').read_text(encoding='utf-8'))
    manifest=json.loads((here/'04_run_manifest.json').read_text(encoding='utf-8'))
    a=math.radians(spec['scope']['current_glide_slope_deg']); m=spec['motion']
    refs={k:make_path(v,a,m['cruise_speed_3d_mps'],m['speed_ramp_s'],m['reference_update_hz']) for k,v in spec['trajectories'].items()}
    dest=Path(args.output_dir);dest.mkdir(parents=True,exist_ok=True)
    for k,v in refs.items(): (dest/(k+'_reference.json')).write_text(json.dumps(v,ensure_ascii=False,indent=2),encoding='utf-8')
    report=validate(spec,manifest,refs)
    (dest/'reference_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'reference_validation':report['status'],'gazebo_executed':False,'runs_planned':39,'paths':{k:{q:v[q] for q in ['length_3d_m','science_duration_s']} for k,v in refs.items()}},ensure_ascii=False,indent=2))
if __name__=='__main__': main()
