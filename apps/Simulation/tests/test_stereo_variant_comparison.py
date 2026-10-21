import copy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from campaigns.stereo_v1_1.analysis_variant_comparison import (
    METHODS, FIELDS, paired_variant_rows, VariantComparisonSummary,
)


def payload(*, rpy=1., yaw=2., count_r=200, count_y=200, trial=0, rep=1, scenario=None):
    result={'calibration_run_id':f'G0-P0-R{rep:02d}','evaluation_run_id':f'G0-V0-R{rep:02d}',
            'trial_index':trial,'data_scope':'all_data','results':[]}
    result.update({'scenario_id':scenario} if scenario else {'yaw_case':'Y0'})
    for method in METHODS:
        for variant, value, count in [('RPY_IMU',rpy,count_r),('YAW_ONLY',yaw,count_y)]:
            result['results'].append({'method':method,'attitude_variant':variant,
                'calibration_run_id':result['calibration_run_id'],'evaluation_run_id':result['evaluation_run_id'],
                'mc_seed':trial,'data_scope':'all_data','calibration_status':'CONVERGED',
                'evaluation_status':'EVALUATED','metrics':{'position_3d_rmse_m':value,
                    'position_3d_p95_m':None if value is None else value*1.5,
                    'valid_position_estimate_count':count,'valid_measurement_rate':0.8,'detection_rate':0.9}})
    return result


@pytest.mark.parametrize('rpy,yaw,direction,outcome',[(1.,2.,'LOWER','IMPROVED'),(3.,2.,'HIGHER','REGRESSED'),(2.,2.,'EQUAL','NO_CHANGE')])
def test_paired_directions_have_no_accuracy_threshold(rpy,yaw,direction,outcome):
    rows=list(paired_variant_rows([payload(rpy=rpy,yaw=yaw)]))
    assert len(rows)==3
    for row in rows:
        assert set(row)==set(FIELDS)
        assert row['comparison_status']=='COMPARABLE_CASE_METRIC'
        assert row['rmse_delta_rpy_minus_yaw_m']==rpy-yaw
        assert row['rmse_direction']==direction and row['outcome']==outcome
    json.dumps(rows,allow_nan=False)


def test_nonconvergence_and_different_valid_counts_are_descriptive_only():
    case=payload(rpy=1.,yaw=2.)
    case['results'][0]['calibration_status']='RANK_DEFICIENT'
    row=list(paired_variant_rows([case]))[0]
    assert row['comparison_status']=='CALIBRATION_NOT_CONVERGED'
    assert row['rmse_direction']=='LOWER' and row['outcome']=='NOT_COMPARABLE'
    row=list(paired_variant_rows([payload(count_r=100,count_y=200)]))[0]
    assert row['comparison_status']=='DIFFERENT_VALID_COUNTS'
    assert row['rmse_delta_rpy_minus_yaw_m']==-1 and row['outcome']=='NOT_COMPARABLE'


def test_zero_valid_and_missing_metrics_remain_visible():
    for case in (payload(rpy=None,yaw=None,count_r=0,count_y=0),payload(count_r=0,count_y=0)):
        rows=list(paired_variant_rows([case]))
        assert len(rows)==3
        assert all(row['comparison_status']=='METRIC_UNAVAILABLE' for row in rows)
    case=payload();case['results'][0].update(calibration_status='NOT_RUN_INSUFFICIENT_INPUT',
              evaluation_status='NOT_EVALUATED_CALIBRATION_FAILED',metrics=None,failure_reason='no paired detections')
    row=list(paired_variant_rows([case]))[0]
    assert row['comparison_status']=='CALIBRATION_NOT_RUN_OR_UNAVAILABLE'
    assert row['rpy_failure_reason']=='no paired detections'


def test_top_unmet_emits_three_pairs_without_invented_calibration_statuses():
    case=payload(scenario='right_time_plus')
    case.update(data_scope='matched_200',evaluation_status='MATCHED_200_UNMET',results=[])
    rows=list(paired_variant_rows([case]))
    assert len(rows)==3 and all(row['family']=='sensitivity' for row in rows)
    assert all(row['rpy_calibration_status']=='UNKNOWN' and row['yaw_calibration_status']=='UNKNOWN' for row in rows)
    assert all(row['comparison_status']=='MATCHED_200_UNMET' and row['rpy_rmse_m'] is None for row in rows)
    case['data_scope']='all_data'
    with pytest.raises(ValueError,match='unmet'):list(paired_variant_rows([case]))


@pytest.mark.parametrize('defect',['missing','duplicate','calrun','evalrun','trial','scope','condition','nan','inf'])
def test_bad_pairs_fail_closed(defect):
    case=payload();row=case['results'][0]
    if defect=='missing':case['results'].pop()
    elif defect=='duplicate':case['results'].append(copy.deepcopy(row))
    elif defect=='calrun':row['calibration_run_id']='G0-P4-R01'
    elif defect=='evalrun':row['evaluation_run_id']='G0-V0-R02'
    elif defect=='trial':row['mc_seed']=1
    elif defect=='scope':row['data_scope']='matched_200'
    elif defect=='condition':row['yaw_case']='Y6'
    else:row['metrics']['position_3d_rmse_m']=float(defect)
    with pytest.raises(ValueError):list(paired_variant_rows([case]))


def test_main_calibration_rows_validate_status_and_exclude_controls():
    case=payload()
    case['calibration_results']=[{**row,'convergence_status':'CONVERGED'} for row in case['results']]
    case['results'].append({'method':'ORACLE_true_camera_pose','attitude_variant':'ORACLE'})
    case['calibration_results'].append({'method':'A_rtk_independent','attitude_variant':'YAW_IDEAL'})
    assert len(list(paired_variant_rows([case])))==3
    case['calibration_results'][0]['convergence_status']='OPTIMIZER_FAILED'
    with pytest.raises(ValueError,match='status mismatch'):list(paired_variant_rows([case]))


def test_summary_separates_mc_physical_runs_condition_scope_and_family():
    summary=VariantComparisonSummary()
    cases=[payload(trial=t,rep=r) for r in (1,2) for t in (0,1,2)]
    cases += [payload(scenario='imu_sigma'),payload()]
    cases[-1]['yaw_case']='GPS_YAW90'
    matched=payload();matched['data_scope']='matched_200'
    for row in matched['results']:row['data_scope']='matched_200'
    cases.append(matched)
    for row in paired_variant_rows(iter(cases)):summary.add(row)
    output=summary.to_dict()
    assert output['paired_case_count']==27 and len(output['groups'])==12
    group=next(g for g in output['groups'] if g['condition_id']=='Y0' and g['method']==METHODS[0] and g['data_scope']=='all_data')
    assert group['paired_case_count']==6
    assert group['physical_calibration_run_count']==group['physical_repetition_count']==2
    assert group['physical_evaluation_run_count']==2 and group['mc_trial_count']==3
    assert group['comparable_delta_means']['rmse_delta_rpy_minus_yaw_m']=={'mean':-1.,'case_count':6}
    assert group['outcome_counts']=={'IMPROVED':6}
    json.dumps(output,allow_nan=False)


def test_generator_is_lazy_and_summary_does_not_retain_payload_rows():
    consumed=[]
    def stream():
        for trial in range(4):
            consumed.append(trial)
            yield payload(trial=trial)
    rows=paired_variant_rows(stream())
    first=next(rows)
    assert consumed==[0]
    summary=VariantComparisonSummary();summary.add(first)
    for row in rows:summary.add(row)
    assert consumed==[0,1,2,3]
    assert summary.to_dict()['paired_case_count']==12
    for group in summary.groups.values():
        assert not any(isinstance(value,list) for value in group.values())


def test_missing_explicit_calibration_and_invalid_summary_values_fail_closed():
    case=payload();case['calibration_results']=[]
    with pytest.raises(ValueError,match='missing paired calibration'):
        list(paired_variant_rows([case]))
    summary=VariantComparisonSummary()
    row=next(paired_variant_rows([payload()]))
    bad={**row,'rpy_rmse_m':float('inf')}
    with pytest.raises(ValueError,match='nonfinite'):summary.add(bad)
    assert summary.to_dict()['groups']==[]


@pytest.mark.parametrize('bad_prefix',[False,True])
def test_sensitivity_producer_namespaced_yaw_and_raw_scenario_ids(bad_prefix):
    case=payload(scenario='camera_imu_sigma_0p05deg')
    for row in case['results']:
        row.update(yaw_case='SENSITIVITY_camera_imu_sigma_0p05deg',
                   sensitivity_scenario_id='camera_imu_sigma_0p05deg',
                   scenario_id='camera_imu_sigma_0p05deg')
    case['calibration_results']=[{**row,'convergence_status':'CONVERGED'} for row in case['results']]
    if bad_prefix:
        case['calibration_results'][0]['yaw_case']='camera_imu_sigma_0p05deg'
        with pytest.raises(ValueError,match='cross-condition'):
            list(paired_variant_rows([case]))
    else:
        rows=list(paired_variant_rows([case]))
        assert len(rows)==3
        assert all(row['condition_id']=='camera_imu_sigma_0p05deg' and
                   row['family']=='sensitivity' for row in rows)
