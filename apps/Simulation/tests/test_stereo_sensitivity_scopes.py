"""Synthetic OFAT scope parity; no runtime flights or real V0 outcomes."""
import csv
import json
import sys
from collections import Counter
from pathlib import Path
import pytest

SIM_ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(SIM_ROOT))
from campaigns.stereo_v1_1 import analyze_campaign as analysis


@pytest.fixture
def sensitivity_fixture(tmp_path,monkeypatch):
    spec=json.loads((SIM_ROOT/'campaigns/stereo_v1_1/protocol/03_experiment_spec.json').read_text())
    scenario=analysis._sensitivity_scenarios(spec)[0]
    monkeypatch.setattr(analysis,'_sensitivity_scenarios',lambda spec:[scenario])
    slots=[dict(run_id=f'G0-{path}-R01',rig_group='G0',path_id=path,repetition=1,
                attempts=[{'attempt_id':'a01'}]) for path in ['P0','V0']]
    output=tmp_path/'analysis';output.mkdir()
    counts=Counter();fit_calls=[];evaluation_calls=[];sizes={'calibration':250,'evaluation':250}
    sensor={'shared_sensor_key':'one-physical-realization'}
    def make_sensor(*args,**kwargs):
        counts['sensor']+=1
        return sensor,{}
    def pairs(root,slot,scenario,trial):
        counts['pairs_'+slot['path_id']]+=1
        n=sizes['evaluation'] if slot['path_id']=='V0' else sizes['calibration']+1
        rows=[dict(pair_id=f'p{i:04d}',timestamp_ns=i*50_000_000,simulation_epoch=0,
                   left_uv_px=[100+i*.01,200],right_uv_px=[90+i*.01,200]) for i in range(n)]
        return rows,{'nominal_science_pair_count':n,'both_detected_science_pair_count':n,'valid_solver_pair_count':n},None
    def rtk(root,slot,rows,scenario,trial,input_dir):
        counts['rtk']+=1
        selected=rows[1:]  # one predeclared alignment failure shared by BOTH scopes
        rtk_rows=[{'paired_id':r['pair_id']} for r in selected]
        input_dir.mkdir(parents=True,exist_ok=True)
        (input_dir/'drone_rtk_observations.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rtk_rows))
        (input_dir/'drone_rtk_generation_provenance.json').write_text('{}')
        return selected,rtk_rows,{'synthetic_alignment_mask':'drop_first_unaligned_pair'}
    def solve(**kwargs):
        fit_calls.append(kwargs)
        result=analysis._not_run_result(kwargs['method'],kwargs['attitude_variant'],len(kwargs['image_pairs']),
                                        'synthetic fixture',kwargs['trial_config'])
        result['convergence_status']='CONVERGED'
        return result
    monkeypatch.setattr(analysis,'make_sensitivity_observation',make_sensor)
    monkeypatch.setattr(analysis,'_sensitivity_pairs',pairs)
    monkeypatch.setattr(analysis,'_sensitivity_rtk_inputs',rtk)
    monkeypatch.setattr(analysis,'solve_calibration',solve)
    monkeypatch.setattr(analysis,'_attempt_dir',lambda *args:tmp_path)
    monkeypatch.setattr(analysis,'_camera_truth',lambda *args:{})
    monkeypatch.setattr(analysis,'_read_jsonl',lambda *args:[])
    monkeypatch.setattr(analysis,'GroundTruthSeries',lambda *args:None)
    def evaluate(row,eval_id,scope,summary,pairs,*args):
        expected = 200 if scope == 'matched_200' else sizes['evaluation']
        assert summary['nominal_science_pair_count'] == expected
        assert summary['both_detected_science_pair_count'] == expected
        evaluation_calls.append((scope,[r['pair_id'] for r in pairs]))
        return dict(method=row['method'],attitude_variant=row['attitude_variant'],
                    calibration_status=row['convergence_status'],mc_seed=row['mc_seed'],
                    evaluation_status='EVALUATED',metrics={'position_3d_rmse_m':1.0,
                    'valid_position_estimate_count':len(pairs)})
    monkeypatch.setattr(analysis,'_evaluation_record',evaluate)
    def calibrate():return analysis._run_sensitivity_calibrations(tmp_path,slots,spec,{'selected_path_id':'P0'},output,mc_trials=1)
    def evaluate_manifest(manifest):
        (output/'analysis_freeze.json').write_text('{}')
        return analysis._evaluate_sensitivity_cases(tmp_path,slots,spec,output,manifest)
    return dict(root=tmp_path,output=output,spec=spec,slots=slots,counts=counts,fit_calls=fit_calls,
                evaluation_calls=evaluation_calls,sizes=sizes,calibrate=calibrate,evaluate=evaluate_manifest)


@pytest.mark.parametrize('count',[200,250])
def test_both_scopes_share_one_realization_and_exact_common_200_mask(sensitivity_fixture,count):
    f=sensitivity_fixture;f['sizes']['calibration']=count
    manifest=f['calibrate']()
    assert len(manifest['cases'])==2
    assert f['counts']['sensor']==f['counts']['rtk']==f['counts']['pairs_P0']==1
    by_scope={}
    for call in f['fit_calls']:
        scope=call['trial_config']['sample_scope']
        ids=[p['pair_id'] for p in call['image_pairs']]
        assert len(ids)==(200 if scope=='matched_200' else count)
        assert 'p0000' not in ids
        by_scope.setdefault(scope,[]).append(ids)
        if call['method']!='C_no_drone_rtk_joint':
            assert [r['paired_id'] for r in call['drone_rtk_observations']]==ids
        else:assert call['drone_rtk_observations'] is None
    assert all(len(v)==6 for v in by_scope.values())
    assert all(ids==by_scope['matched_200'][0] for ids in by_scope['matched_200'])
    folders={case['data_scope']:f['output']/case['case_path'] for case in manifest['cases']}
    assert (folders['all_data']/'sensor_observations.json').stat().st_ino==(folders['matched_200']/'sensor_observations.json').stat().st_ino
    for scope,folder in folders.items():
        assert json.loads((folder/'trial_config.json').read_text())['sample_scope']==scope


def test_199_aligned_pairs_produce_six_unmet_results_without_fake_frames(sensitivity_fixture):
    f=sensitivity_fixture;f['sizes']['calibration']=199
    manifest=f['calibrate']()
    assert len(f['fit_calls'])==6
    case=next(c for c in manifest['cases'] if c['data_scope']=='matched_200')
    bundle=json.loads((f['output']/case['case_path']/'calibration_results.json').read_text())
    assert len(bundle['results'])==6
    assert all(r['convergence_status']=='NOT_RUN_INSUFFICIENT_INPUT' for r in bundle['results'])
    assert all(c['trial_config']['sample_scope']=='all_data' for c in f['fit_calls'])
    assert bundle['matched_200_selection']['available_unique_pair_count']==199
    assert bundle['matched_200_selection']['status']=='INSUFFICIENT_UNIQUE_PAIRS'
    assert all(r['selected_pair_ids']==[] for r in bundle['results'])


def test_resume_detects_tampered_scope_bundle(sensitivity_fixture):
    f=sensitivity_fixture;manifest=f['calibrate']()
    case=next(c for c in manifest['cases'] if c['data_scope']=='matched_200')
    path=f['output']/case['case_path']/'calibration_results.json'
    path.write_text(path.read_text()+' ')
    with pytest.raises((ValueError,RuntimeError)):
        f['calibrate']()


def test_evaluation_and_main_y0_baselines_stay_in_the_same_scope(sensitivity_fixture):
    f=sensitivity_fixture;manifest=f['calibrate']()
    for scope,value in [('all_data',10.),('matched_200',20.)]:
        path=analysis._analysis_case_dir(f['output'],f['slots'][0],0,'Y0',scope)/'evaluation_results.json'
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps({'data_scope':scope,'results':[dict(method=m,attitude_variant=v,calibration_status='CONVERGED',
            metrics={'position_3d_rmse_m':value,'valid_position_estimate_count':200 if scope=='matched_200' else 250})
            for m in analysis.METHODS for v in analysis.VARIANTS]}))
    summary=f['evaluate'](manifest)
    assert summary['case_count']==2
    for case in manifest['cases']:
        scope=case['data_scope'];payload=json.loads((f['output']/case['case_path']/'evaluation_results.json').read_text())
        assert payload['data_scope']==scope
        for row in payload['results']:
            assert row['main_y0_baseline']['position_3d_rmse_m']==(20. if scope=='matched_200' else 10.)
    assert Counter(scope for scope,_ in f['evaluation_calls'])=={'all_data':6,'matched_200':6}
    assert all(len(ids)==(200 if scope=='matched_200' else 250) for scope,ids in f['evaluation_calls'])
    with (f['output']/'sensitivity_evaluation_metrics.csv').open() as handle:rows=list(csv.DictReader(handle))
    assert all(r['baseline_data_scope']==r['data_scope'] for r in rows)


def test_short_matched_v0_serializes_empty_payload_and_six_explicit_unmet_csv_rows(sensitivity_fixture):
    f=sensitivity_fixture;f['sizes']['evaluation']=199
    manifest=f['calibrate']();f['evaluate'](manifest)
    case=next(c for c in manifest['cases'] if c['data_scope']=='matched_200')
    payload=json.loads((f['output']/case['case_path']/'evaluation_results.json').read_text())
    assert payload['results']==[]
    assert payload['evaluation_status']=='MATCHED_200_UNMET'
    assert all(scope=='all_data' for scope,_ in f['evaluation_calls'])
    with (f['output']/'sensitivity_evaluation_metrics.csv').open() as handle:rows=list(csv.DictReader(handle))
    unmet=[r for r in rows if r['data_scope']=='matched_200']
    assert len(unmet)==6
    assert all(r['evaluation_status']=='MATCHED_200_UNMET' and not r['position_3d_rmse_m'] for r in unmet)


def test_wrong_scope_baseline_payload_is_rejected_even_at_expected_path(sensitivity_fixture):
    f=sensitivity_fixture;manifest=f['calibrate']()
    path=analysis._analysis_case_dir(f['output'],f['slots'][0],0,'Y0','all_data')/'evaluation_results.json'
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps({'data_scope':'matched_200','results':[]}))
    with pytest.raises(ValueError,match='baseline scope mismatch'):
        f['evaluate'](manifest)
