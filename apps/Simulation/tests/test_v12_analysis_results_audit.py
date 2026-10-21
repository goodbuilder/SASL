import json
from pathlib import Path
import pytest
from campaigns.stereo_v1_1 import audit_v12_analysis_results as a

SPEC = json.loads((Path(__file__).resolve().parents[1] / '작업지시서/stereo_calibration_39run_package_v1_2/03_experiment_spec.json').read_text())


def test_pending_and_bad_freeze_never_open_evaluation(tmp_path, monkeypatch):
    assert a.audit(tmp_path, tmp_path)['status'] == 'PENDING'
    folder = tmp_path / 'analysis_versions/test'
    folder.mkdir(parents=True)
    pointer = {'analysis_id': 'test', 'analysis_summary': 'analysis_versions/test/analysis_summary.json',
               'analysis_freeze_sha256': 'bad'}
    (tmp_path / 'analysis_latest.json').write_text(json.dumps(pointer))
    assert a.audit(tmp_path, tmp_path)['status'] == 'PENDING'
    (folder / 'analysis_freeze.json').write_text('{}')
    original = a.read
    def guarded(path):
        assert 'evaluation' not in path.name and path.name != 'analysis_summary.json'
        return original(path)
    monkeypatch.setattr(a, 'read', guarded)
    result = a.audit(tmp_path, tmp_path)
    assert result['status'] == 'FAIL' and result['evaluation_files_opened'] is False


def test_declared_full_matrix_and_sensitivity_levels():
    slots = [{'run_id': f'{g}-{p}-R{r:02d}', 'rig_group': g, 'path_id': p, 'repetition': r}
             for g, paths, repeats in [('G0', ['P0','P1','P2','P3','P4','V0'], 5),
                                       ('G1', ['P0','P4','V0'], 3)]
             for r in range(1,repeats+1) for p in paths]
    keys = list(a.expected_main(slots, SPEC, 100))
    assert len(keys) == len(set(keys)) == 42400
    assert ('G0-P3-R05',99,'Y6','matched_200') in keys
    assert ('G1-P4-R03',99,'G1_PRIMARY_ZERO','all_data') in keys
    assert sum(a.expected_sensitivity(SPEC).values()) == 25


def _case(tmp_path):
    for name in ['trial_config.json', 'sensor_observations.json']:
        (tmp_path / name).write_text(json.dumps({'calibration_run_id': 'G0-P1-R02',
                    'evaluation_run_id': 'G0-V0-R02', 'mc_seed': 99}))
    rows = []
    for method, variant in a.MAIN:
        row = dict.fromkeys(SPEC['calibration_contract']['result_fields'])
        row.update(method=method, attitude_variant=variant, calibration_run_id='G0-P1-R02',
                   evaluation_run_id='G0-V0-R02', mc_seed=99,
                   convergence_status='NOT_RUN_INSUFFICIENT_INPUT', failure_reason='no detections')
        rows.append(row)
    (tmp_path / 'calibration_results.json').write_text(json.dumps({'results': rows}))
    evaluated = [{'method': m, 'attitude_variant': v, 'calibration_run_id':'G0-P1-R02',
                  'evaluation_run_id':'G0-V0-R02','mc_seed':99,
                  'evaluation_status':'NOT_EVALUATED_CALIBRATION_FAILED'}
                 for m,v in a.MAIN | {('M0_sensor_only','INITIAL'),('ORACLE_true_camera_pose','ORACLE')}]
    payload = {'calibration_run_id':'G0-P1-R02','evaluation_run_id':'G0-V0-R02',
               'trial_index':99,'freeze_sha256':'frozen','results':evaluated}
    (tmp_path / 'evaluation_results.json').write_text(json.dumps(payload))
    return payload


def test_case_accepts_explicit_scientific_failure_but_rejects_missing_control(tmp_path):
    payload = _case(tmp_path)
    assert a.audit_case(tmp_path,SPEC,'G0-P1-R02','G0-V0-R02',99,'frozen') == 'REPORTED'
    payload['results'] = [r for r in payload['results'] if r['method'] != 'ORACLE_true_camera_pose']
    (tmp_path / 'evaluation_results.json').write_text(json.dumps(payload))
    with pytest.raises(ValueError,match='missing evaluation'):
        a.audit_case(tmp_path,SPEC,'G0-P1-R02','G0-V0-R02',99,'frozen')


def test_matched200_unmet_is_explicit_outcome_not_coverage_failure(tmp_path):
    payload = _case(tmp_path)
    payload.update(evaluation_status='MATCHED_200_UNMET',data_scope='matched_200',
                   required_pair_count=200,pair_count_available=57,results=[])
    (tmp_path / 'evaluation_results.json').write_text(json.dumps(payload))
    assert a.audit_case(tmp_path,SPEC,'G0-P1-R02','G0-V0-R02',99,'frozen') == 'MATCHED_200_UNMET'


def test_reject_nonfinite_and_duplicate_results_and_escape(tmp_path):
    p=tmp_path/'bad.json';p.write_text('{"x":1e999}')
    with pytest.raises(ValueError,match='nonfinite'): a.read(p)
    with pytest.raises(ValueError,match='duplicate'): a.rows_by_method([{'method':'A','attitude_variant':'V'}]*2)
    with pytest.raises(ValueError,match='escapes'): a.owned(tmp_path,'../outside')


def test_case_rejects_wrong_yaw_scope_labels(tmp_path):
    _case(tmp_path)
    config_path = tmp_path / 'trial_config.json'
    config = json.loads(config_path.read_text())
    config.update(yaw_case='Y0', sample_scope='all_data')
    config_path.write_text(json.dumps(config))
    bundle_path = tmp_path / 'calibration_results.json'
    bundle = json.loads(bundle_path.read_text())
    bundle.update(source_run_id='G0-P1-R02', trial_index=99, yaw_case='Y1', data_scope='all_data')
    bundle_path.write_text(json.dumps(bundle))
    with pytest.raises(ValueError,match='case labels'):
        a.audit_case(tmp_path,SPEC,'G0-P1-R02','G0-V0-R02',99,'frozen',yaw='Y0',scope='all_data')


def test_sensitivity_matrix_requires_all_repetitions_trials_and_same_v0():
    slots = [{'run_id': f'G0-{path}-R{rep:02d}', 'rig_group': 'G0',
              'path_id': path, 'repetition': rep}
             for rep in range(1, 6) for path in ('P0', 'P3', 'V0')]
    keys = a.expected_sensitivity_cases(slots, {'P0', 'P3'}, [f'S{i}' for i in range(25)], 100)
    assert len(keys) == 50000
    assert ('S24', 'G0-P3-R05', 99, 'matched_200') in keys
    cases = [{'scenario_id': scenario, 'calibration_run_id': run_id, 'trial_index': trial,
              'path_id': run_id.split('-')[1], 'evaluation_run_id': 'G0-V0-' + run_id.split('-')[-1],
              'data_scope': scope, 'case_path': f'sensitivities/{scenario}/{run_id}/trial_{trial:03d}/{scope}'}
             for scenario, run_id, trial, scope in keys]
    manifest = {'calibration_paths': ['P0', 'P3'], 'mc_trials_per_sensitivity_case': 100,
                'sample_scopes': ['all_data', 'matched_200'], 'cases': cases}
    a.validate_sensitivity_matrix(manifest, slots, {'P0', 'P3'}, [f'S{i}' for i in range(25)], 100)
    manifest['cases'] = cases[:-1]
    with pytest.raises(ValueError, match='sensitivity matrix mismatch'):
        a.validate_sensitivity_matrix(manifest, slots, {'P0', 'P3'}, [f'S{i}' for i in range(25)], 100)
    manifest['cases'] = cases
    cases[0]['evaluation_run_id'] = 'G0-V0-R99'
    with pytest.raises(ValueError, match='sensitivity pairing mismatch'):
        a.validate_sensitivity_matrix(manifest, slots, {'P0', 'P3'}, [f'S{i}' for i in range(25)], 100)


def _scoped_sensitivity_case(tmp_path, *, available=240, selected=200, eval_available=240):
    payload = _case(tmp_path)
    scope='matched_200'
    evidence={'requested_pair_count':200,'available_unique_pair_count':available,
              'selected_pair_count':selected,
              'status':'SELECTED' if available>=200 else 'INSUFFICIENT_UNIQUE_PAIRS'}
    config_path=tmp_path/'trial_config.json'
    config=json.loads(config_path.read_text())
    config.update(sample_scope=scope,matched_200_selection=evidence)
    config_path.write_text(json.dumps(config))
    bundle_path=tmp_path/'calibration_results.json'
    bundle=json.loads(bundle_path.read_text())
    bundle.update(calibration_run_id='G0-P1-R02',evaluation_run_id='G0-V0-R02',trial_index=99,
                  data_scope=scope,matched_200_selection=evidence)
    for row in bundle['results']:
        row.update(data_scope=scope,observation_count=selected)
        if selected:
            row.update(convergence_status='RANK_DEFICIENT',selected_pair_ids=[f'p{i}' for i in range(selected)])
    bundle_path.write_text(json.dumps(bundle))
    eval_evidence={**evidence,'available_unique_pair_count':eval_available,
                   'selected_pair_count':200 if eval_available>=200 else 0,
                   'status':'SELECTED' if eval_available>=200 else 'INSUFFICIENT_UNIQUE_PAIRS'}
    payload.update(data_scope=scope,baseline_data_scope=scope,matched_200_selection=eval_evidence)
    for row in payload['results']:
        row.update(data_scope=scope,comparison_to_main_y0={'data_scope':scope})
    if eval_available<200:
        payload.update(evaluation_status='MATCHED_200_UNMET',results=[],
                       required_pair_count=200,pair_count_available=eval_available)
    (tmp_path/'evaluation_results.json').write_text(json.dumps(payload))
    return payload


def test_sensitivity_matched_requires200_when_feasible_and_accepts_explicit_shortfall(tmp_path):
    _scoped_sensitivity_case(tmp_path)
    assert a.audit_case(tmp_path,SPEC,'G0-P1-R02','G0-V0-R02',99,'frozen',
                        sensitivity=True,scope='matched_200') == 'REPORTED'
    _scoped_sensitivity_case(tmp_path,available=240,selected=0)
    with pytest.raises(ValueError,match='feasible matched selection'):
        a.audit_case(tmp_path,SPEC,'G0-P1-R02','G0-V0-R02',99,'frozen',sensitivity=True,scope='matched_200')
    _scoped_sensitivity_case(tmp_path,available=99,selected=0,eval_available=57)
    assert a.audit_case(tmp_path,SPEC,'G0-P1-R02','G0-V0-R02',99,'frozen',
                        sensitivity=True,scope='matched_200') == 'MATCHED_200_UNMET'


@pytest.mark.parametrize('target', ['bundle_scope','eval_scope','baseline_scope','row_scope','comparison_scope'])
def test_sensitivity_rejects_cross_scope_results(tmp_path,target):
    payload=_scoped_sensitivity_case(tmp_path)
    if target=='bundle_scope':
        path=tmp_path/'calibration_results.json'; data=json.loads(path.read_text());data['data_scope']='all_data'
    else:
        path=tmp_path/'evaluation_results.json';data=payload
        if target=='eval_scope':data['data_scope']='all_data'
        elif target=='baseline_scope':data['baseline_data_scope']='all_data'
        elif target=='row_scope':data['results'][0]['data_scope']='all_data'
        else:data['results'][0]['comparison_to_main_y0']['data_scope']='all_data'
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='scope'):
        a.audit_case(tmp_path,SPEC,'G0-P1-R02','G0-V0-R02',99,'frozen',sensitivity=True,scope='matched_200')


def test_sensitivity_missing_entire_matched_scope_is_not_complete():
    slots=[{'run_id':'G0-P0-R01','rig_group':'G0','path_id':'P0','repetition':1}]
    case={'scenario_id':'s','calibration_run_id':'G0-P0-R01','evaluation_run_id':'G0-V0-R01',
          'path_id':'P0','trial_index':0,'data_scope':'all_data',
          'case_path':'sensitivities/s/G0-P0-R01/trial_000/all_data'}
    manifest={'calibration_paths':['P0'],'mc_trials_per_sensitivity_case':1,
              'sample_scopes':['all_data','matched_200'],'cases':[case]}
    with pytest.raises(ValueError,match='sensitivity matrix mismatch'):
        a.validate_sensitivity_matrix(manifest,slots,{'P0'},['s'],1)
    assert len(a.expected_sensitivity_cases(
        [{'run_id':f'G0-P0-R{i:02d}','rig_group':'G0','path_id':'P0','repetition':i} for i in range(1,6)],
        {'P0'},[f'S{i}' for i in range(25)],100))==25000


def test_sensitivity_rejects_wrong_scenario_identity(tmp_path):
    _scoped_sensitivity_case(tmp_path)
    config_path=tmp_path/'trial_config.json'
    config=json.loads(config_path.read_text());config['yaw_case']='SENSITIVITY_S0'
    config_path.write_text(json.dumps(config))
    bundle_path=tmp_path/'calibration_results.json'
    bundle=json.loads(bundle_path.read_text());bundle['scenario_id']='S1'
    bundle_path.write_text(json.dumps(bundle))
    with pytest.raises(ValueError,match='scenario mismatch'):
        a.audit_case(tmp_path,SPEC,'G0-P1-R02','G0-V0-R02',99,'frozen',
                     sensitivity=True,scope='matched_200',scenario_id='S0')
