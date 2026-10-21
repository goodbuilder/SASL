#!/usr/bin/env python3
"""Audit v1.2 offline coverage/contracts, never fit or rank numerical accuracy.

Freeze identity and source binding are verified before any evaluation is opened.
Explicit unsuccessful scientific outcomes count as reported outcomes. Output is
exclusive and outside immutable analysis/raw directories. Uses only stdlib.
"""
from __future__ import annotations
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

METHODS = ('A_rtk_independent', 'B_rtk_joint', 'C_no_drone_rtk_joint')
VARIANTS = ('RPY_IMU', 'YAW_ONLY')
MAIN = {(m, v) for m in METHODS for v in VARIANTS}


def read(path):
    def bad(value):
        raise ValueError(f'nonfinite JSON constant: {path}: {value}')
    value = json.loads(path.read_text(encoding='utf-8'), parse_constant=bad)
    def finite(item):
        if isinstance(item, float) and not math.isfinite(item):
            raise ValueError(f'nonfinite JSON number: {path}')
        if isinstance(item, dict):
            for child in item.values(): finite(child)
        elif isinstance(item, list):
            for child in item: finite(child)
    finite(value)
    return value


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''): h.update(block)
    return h.hexdigest()


def owned(root, relative):
    path = (root / relative).resolve()
    if Path(relative).is_absolute() or not path.is_relative_to(root.resolve()):
        raise ValueError(f'path escapes artifact root: {relative}')
    return path


def require(test, reason):
    if not test: raise ValueError(reason)


def verify_hashes(root, mapping):
    for name, digest in mapping.items():
        require(sha(owned(root, name)) == digest, f'hash mismatch: {name}')


def expected_main(slots, spec, trials):
    for row in slots:
        if row['path_id'] == 'V0': continue
        cases = ([x['id'] for x in spec['reprocessing']['yaw_bias_cases_deg']]
                 if row['rig_group'] == 'G0' else ['G1_PRIMARY_ZERO'])
        for trial in range(trials):
            for case in [*cases, 'GPS_YAW90']:
                for scope in ('all_data', 'matched_200'):
                    yield row['run_id'], trial, case, scope


def expected_sensitivity_cases(slots, paths, scenario_ids, trials):
    return {(scenario, row['run_id'], trial, scope)
            for row in slots if row['rig_group'] == 'G0' and row['path_id'] in paths
            for scenario in scenario_ids for trial in range(trials)
            for scope in ('all_data', 'matched_200')}


def validate_sensitivity_matrix(manifest, slots, paths, scenario_ids, trials):
    require(set(manifest['calibration_paths']) == paths
            and manifest['mc_trials_per_sensitivity_case'] == trials
            and manifest.get('sample_scopes') == ['all_data', 'matched_200'],
            'sensitivity path/trial policy mismatch')
    cases = manifest['cases']
    keys = [(c['scenario_id'], c['calibration_run_id'], c['trial_index'], c.get('data_scope')) for c in cases]
    require(len(keys) == len(set(keys))
            and set(keys) == expected_sensitivity_cases(slots, paths, scenario_ids, trials),
            'sensitivity matrix mismatch: missing/duplicate repetition, MC trial or sample scope')
    by_id = {row['run_id']: row for row in slots}
    for case in cases:
        row = by_id[case['calibration_run_id']]
        require(case['path_id'] == row['path_id']
                and case['evaluation_run_id'] == f"G0-V0-R{int(row['repetition']):02d}",
                'sensitivity pairing mismatch')
        require(case.get('case_path') == (f"sensitivities/{case['scenario_id']}/{case['calibration_run_id']}/"
                                        f"trial_{case['trial_index']:03d}/{case['data_scope']}"),
                'sensitivity case path/scope mismatch')


def level_key(value):
    if isinstance(value, dict): return tuple((k, level_key(v)) for k, v in sorted(value.items()))
    if isinstance(value, list): return tuple(level_key(v) for v in value)
    return value


def expected_sensitivity(spec):
    r = spec['reprocessing']; levels = r['rtk_sigma_xyz_m']; values = []
    for factor in ('camera_rtk_sigma_xyz_m', 'drone_rtk_sigma_xyz_m'):
        values += [(factor, levels[i]) for i in (0, 2)]
    values += [('camera_imu_roll_pitch_sigma_deg', spec['calibration_contract']['imu_prior']['stress_sigma_deg'])]
    for magnitude in r['yaw_extension_abs_deg']:
        values += [('initial_yaw_bias_deg', {'left': a*magnitude, 'right': b*magnitude})
                   for a, b in ((1,1),(-1,-1),(1,-1),(-1,1))]
    values += [('ideal_projection_pixel_sigma_px', v) for v in r['pixel_iid_sigma_px']]
    for offset in r['timestamp_offsets_s']:
        if offset:
            values += [(factor, offset) for factor in ('drone_rtk_timestamp_offset_s', 'right_camera_timestamp_offset_s')]
    values += [('drone_rtk_shared_enu_bias', {'sigma_xyz_m': levels[1]})]
    return Counter((factor, level_key(value)) for factor, value in values)


def rows_by_method(rows):
    keys = [(r['method'], r['attitude_variant']) for r in rows]
    require(len(keys) == len(set(keys)), 'duplicate method/variant result')
    return dict(zip(keys, rows))


def validate_matched_selection(payload, spec, label):
    evidence = payload.get('matched_200_selection')
    require(isinstance(evidence, dict), f'{label} lacks matched selection evidence')
    count = spec['reprocessing']['matched_sample_comparison_n']
    available, selected = evidence.get('available_unique_pair_count'), evidence.get('selected_pair_count')
    require(isinstance(available, int) and not isinstance(available, bool) and available >= 0,
            f'{label} has invalid available pair count')
    require(evidence.get('requested_pair_count') == count, f'{label} matched target differs from specification')
    require(isinstance(selected, int) and not isinstance(selected, bool), f'{label} has invalid selected pair count')
    require(selected == (count if available >= count else 0), f'{label} feasible matched selection omitted or wrong count')
    require(evidence.get('status') == ('SELECTED' if available >= count else 'INSUFFICIENT_UNIQUE_PAIRS'),
            f'{label} matched selection status contradicts pair count')
    return available, selected


def audit_case(directory, spec, run_id, eval_id, trial, freeze_hash, *, sensitivity=False, yaw=None, scope=None, scenario_id=None):
    # Calibration result_fields belong to the calibration results; the writer's
    # evaluation schema references them by run/method/variant, not duplicated fields.
    for name in spec['calibration_contract']['derived_trial_files']:
        require((directory / name).is_file(), f'missing derived trial file: {directory}/{name}')
    config = read(directory / 'trial_config.json')
    read(directory / 'sensor_observations.json')
    require(config['calibration_run_id'] == run_id and config['evaluation_run_id'] == eval_id,
            'trial configuration run pairing mismatch')
    require(config['mc_seed'] == trial, 'trial configuration MC identity mismatch')
    bundle = read(directory / 'calibration_results.json')
    if yaw is not None:
        require(bundle.get('yaw_case') == config.get('yaw_case') == yaw
                and bundle.get('data_scope') == config.get('sample_scope') == scope
                and bundle.get('trial_index') == trial and bundle.get('source_run_id') == run_id,
                'calibration case labels mismatch')
    calibration_matched = None
    if sensitivity:
        require(scope in ('all_data', 'matched_200')
                and config.get('sample_scope') == bundle.get('data_scope') == scope
                and bundle.get('calibration_run_id') == run_id
                and bundle.get('evaluation_run_id') == eval_id
                and bundle.get('trial_index') == trial, 'sensitivity calibration scope/identity mismatch')
        if scenario_id is not None:
            require(bundle.get('scenario_id') == scenario_id
                    and config.get('yaw_case') == f'SENSITIVITY_{scenario_id}',
                    'sensitivity calibration scenario mismatch')
        if scope == 'matched_200':
            calibration_matched = validate_matched_selection(bundle, spec, 'calibration')
            require(config.get('matched_200_selection') == bundle.get('matched_200_selection'),
                    'sensitivity config/bundle selection evidence mismatch')
    calibration = rows_by_method(bundle['results'])
    require(MAIN <= calibration.keys(), 'missing main calibration method/variant')
    for key in MAIN:
        row = calibration[key]
        require(set(spec['calibration_contract']['result_fields']) <= row.keys(), 'missing calibration result fields')
        require(row['calibration_run_id'] == run_id and row['evaluation_run_id'] == eval_id
                and row['mc_seed'] == trial, 'calibration identity mismatch')
        require(bool(row.get('convergence_status')), 'missing calibration status')
        if sensitivity:
            require(row.get('data_scope') == scope, 'sensitivity calibration row scope mismatch')
        if calibration_matched is not None:
            available, selected = calibration_matched
            if not selected:
                require(row['convergence_status'] == 'NOT_RUN_INSUFFICIENT_INPUT',
                        'calibration fitted despite unmatched sample count')
            require(row.get('observation_count') == selected, 'calibration row sample count mismatch')
            if row['convergence_status'] in ('CONVERGED', 'OPTIMIZER_FAILED', 'RANK_DEFICIENT'):
                pair_ids = row.get('selected_pair_ids', [])
                require(len(pair_ids) == len(set(pair_ids)) == selected, 'calibration sample IDs missing or duplicated')
        if row['convergence_status'] == 'NOT_RUN_INSUFFICIENT_INPUT':
            require(bool(row.get('failure_reason')), 'missing unsuccessful calibration reason')
    evaluation = read(directory / 'evaluation_results.json')
    if yaw is not None:
        require(evaluation.get('yaw_case') == yaw and evaluation.get('data_scope') == scope,
                'evaluation case labels mismatch')
    evaluation_matched = None
    if sensitivity:
        require(evaluation.get('data_scope') == scope, 'sensitivity evaluation scope mismatch')
        require(evaluation.get('baseline_data_scope') == scope, 'sensitivity baseline scope mismatch')
        if scenario_id is not None:
            require(evaluation.get('scenario_id') == scenario_id, 'sensitivity evaluation scenario mismatch')
        if scope == 'matched_200':
            evaluation_matched = validate_matched_selection(evaluation, spec, 'evaluation')
            if evaluation_matched[1] == 0:
                require(evaluation.get('evaluation_status') == 'MATCHED_200_UNMET',
                        'evaluation omitted explicit matched-count failure')
    require(evaluation['freeze_sha256'] == freeze_hash, 'evaluation freeze mismatch')
    require(evaluation['calibration_run_id'] == run_id and evaluation['evaluation_run_id'] == eval_id
            and evaluation['trial_index'] == trial, 'evaluation identity mismatch')
    require(evaluation.get('v0_recalibration_performed', False) is False, 'V0 recalibration was performed')
    if evaluation.get('evaluation_status') == 'MATCHED_200_UNMET':
        require(evaluation.get('data_scope') == 'matched_200'
                and evaluation.get('required_pair_count') == 200
                and 0 <= evaluation.get('pair_count_available', -1) < 200
                and evaluation.get('results') == [], 'invalid matched-200 failure accounting')
        if evaluation_matched is not None:
            require(evaluation['pair_count_available'] == evaluation_matched[0],
                    'evaluation unmatched availability evidence mismatch')
        return 'MATCHED_200_UNMET'
    evaluated = rows_by_method(evaluation['results'])
    required = MAIN if sensitivity else MAIN | {('M0_sensor_only', 'INITIAL'), ('ORACLE_true_camera_pose', 'ORACLE')}
    require(required <= evaluated.keys(), 'missing evaluation method/variant/control')
    for row in evaluated.values():
        require(row.get('calibration_run_id') == run_id and row.get('evaluation_run_id') == eval_id
                and row.get('mc_seed') == trial, 'evaluation row identity mismatch')
        require(bool(row.get('evaluation_status')), 'missing evaluation outcome status')
        if sensitivity:
            require(row.get('data_scope') == scope, 'sensitivity evaluation row scope mismatch')
            require((row.get('comparison_to_main_y0') or {}).get('data_scope') == scope,
                    'sensitivity comparison baseline scope mismatch')
    return 'REPORTED'


def audit(output_root, source_root):
    output_root, source_root = Path(output_root).resolve(), Path(source_root).resolve()
    pointer_file = output_root / 'analysis_latest.json'
    if not pointer_file.is_file(): return {'status': 'PENDING', 'reason': 'analysis_latest.json absent', 'evaluation_files_opened': False}
    opened = False
    try:
        pointer = read(pointer_file)
        summary_path = owned(output_root, pointer['analysis_summary'])
        require(summary_path.name == 'analysis_summary.json', 'pointer must reference analysis_summary.json')
        analysis = summary_path.parent
        freeze_path = analysis / 'analysis_freeze.json'
        if not freeze_path.is_file():
            return {'status': 'PENDING', 'reason': 'analysis freeze absent', 'evaluation_files_opened': False}
        freeze_hash = sha(freeze_path)
        require(freeze_hash == pointer['analysis_freeze_sha256'], 'pointer freeze hash mismatch')
        freeze = read(freeze_path)
        require(freeze['status'] == 'FROZEN_BEFORE_V0_POSITION_EVALUATION'
                and freeze['v0_position_error_read_before_freeze'] is False, 'invalid freeze contract')
        require(pointer['analysis_id'] == analysis.name, 'analysis pointer identity mismatch')
        manifest = read(analysis / 'analysis_run_manifest.json')
        require(manifest['analysis_id'] == pointer['analysis_id'], 'run manifest identity mismatch')
        status = read(source_root / 'run_status.json'); slots = status['run_slots']
        require(status['valid_run_count'] == 39 and len(slots) == 39
                and len({r['run_id'] for r in slots}) == 39
                and all(r['status'] in ('VALID', 'VALID_WITH_WARNINGS') for r in slots), 'source39 not complete')
        bound = manifest['source_campaign_manifest_sha256']
        required = {'frozen_protocol/input_parameters.xlsx', 'workbook_campaign_plan.json',
                    'schema_mapping.json', 'frozen_protocol/v1_2_specification.json', 'run_status.json'}
        require(required <= bound.keys(), 'missing frozen source bindings')
        verify_hashes(source_root, bound)
        spec = read(source_root / 'frozen_protocol/v1_2_specification.json')
        require(spec['document_version'] == '1.2' and spec['schema_version'] == 'stereo_calibration_campaign_v1_2', 'not a v1.2 specification')
        protocol = manifest['source_protocol']
        require(protocol == freeze['source_protocol'] and protocol['sha256'] == bound['frozen_protocol/v1_2_specification.json'], 'actual specification hash mismatch')
        plan = read(source_root / 'workbook_campaign_plan.json')
        require(plan['workbook_sha256'] == bound['frozen_protocol/input_parameters.xlsx'] == status['workbook_sha256'], 'frozen XLSX binding mismatch')
        encoded = json.dumps(plan, sort_keys=True, ensure_ascii=False, separators=(',', ':'), allow_nan=False).encode()
        require(hashlib.sha256(encoded).hexdigest() == status['campaign_plan_sha256'], 'frozen plan digest mismatch')
        require({r['run_id'] for r in plan['planned_runs']} == {r['run_id'] for r in slots}
                and len(plan['planned_runs']) == 39, 'source slots differ from frozen plan')
        frozen_rows = {r['run_id']: r for r in plan['planned_runs']}
        for row in slots:
            require(all(row[k] == frozen_rows[row['run_id']][k] for k in ('rig_group', 'path_id', 'repetition')),
                    'source run identity differs from frozen plan')
        require(sha(analysis / 'path_selection.json') == freeze['path_selection_sha256'], 'path selection freeze hash mismatch')
        require(manifest['algorithm_sha256'] == freeze['algorithm_sha256'], 'algorithm freeze mismatch')
        require(manifest['mc_trial_count'] == freeze['mc_trial_count'] == spec['reprocessing']['mc_repeats'] == 100, 'main MC trial count must be100')
        for name, key in [('calibration_freeze_manifest.json', 'primary_calibration_checksums_manifest_sha256'),
                          ('sensitivity_calibration_manifest.json', 'sensitivity_calibration_manifest_sha256'),
                          ('sensitivity_calibration_checksums.json', 'sensitivity_calibration_checksums_sha256')]:
            require(sha(analysis / name) == freeze[key], f'freeze child hash mismatch: {name}')
        for mapping_name in ('calibration_freeze_manifest.json', 'sensitivity_calibration_checksums.json'):
            mapping = read(analysis / mapping_name)
            require(all(Path(n).name == 'calibration_checksums.json' for n in mapping),
                    'invalid pre-freeze checksum inventory')
            verify_hashes(analysis, mapping)
            for name in mapping:
                require(Path(name).name == 'calibration_checksums.json', 'invalid pre-freeze checksum inventory')
                checks = owned(analysis, name)
                children = read(checks)
                require(all(not any(part.startswith('evaluation') for part in Path(n).parts) for n in children), 'evaluation file in pre-freeze calibration inventory')
                verify_hashes(checks.parent, children)
        # Only after all pre-evaluation gates may any evaluation bytes be read.
        opened = True
        checksum_rows = {}
        for line in (analysis / 'analysis_checksums.sha256').read_text().splitlines():
            digest, name = line.split('  ', 1)
            require(name not in checksum_rows, 'duplicate analysis checksum path')
            checksum_rows[name] = digest
        actual_files = {str(p.relative_to(analysis)) for p in analysis.rglob('*')
                        if p.is_file() and p.name != 'analysis_checksums.sha256'}
        require(actual_files == set(checksum_rows), 'analysis checksum inventory mismatch')
        verify_hashes(analysis, checksum_rows)
        expected = set(expected_main(slots, spec, 100))
        index = read(analysis / 'calibration_run_index.json')
        indexed = [(r['run_id'], r['trial_index'], r['yaw_case'], r['data_scope']) for r in index['batches']]
        require(len(indexed) == len(set(indexed)) and set(indexed) == expected, 'main matrix missing/duplicate/unexpected cases')
        by_id = {r['run_id']: r for r in slots}; outcomes = Counter(); directories = set()
        for run_id, trial, yaw, scope in sorted(expected):
            slot = by_id[run_id]; eval_id = f"{slot['rig_group']}-V0-R{int(slot['repetition']):02d}"
            require(eval_id in by_id, 'matching V0 slot absent')
            case = analysis / 'derived' / run_id / slot['attempts'][-1]['attempt_id'] / f'trial_{trial:03d}' / yaw / scope
            directories.add(case)
            outcomes[audit_case(case, spec, run_id, eval_id, trial, freeze_hash, yaw=yaw, scope=scope)] += 1
        require({p.parent for p in (analysis / 'derived').rglob('calibration_results.json')} == directories,
                'main case directories do not match declared matrix')
        sensitivity = read(analysis / 'sensitivity_calibration_manifest.json')
        scenarios = sensitivity['scenarios']; ids = [s['scenario_id'] for s in scenarios]
        observed_levels = Counter((s['factor'], level_key(s['factor_value'])) for s in scenarios)
        require(len(ids) == len(set(ids)) == 25 and observed_levels == expected_sensitivity(spec), 'sensitivity factor levels mismatch')
        paths = {'P0'} | ({freeze['selected_path_id']} if freeze['selected_path_id'] else set())
        validate_sensitivity_matrix(sensitivity, slots, paths, ids, spec['reprocessing']['mc_repeats'])
        cases = sensitivity['cases']
        shared_sensors = {}
        for case in cases:
            directory = owned(analysis, case['case_path'])
            audit_case(directory, spec, case['calibration_run_id'], case['evaluation_run_id'],
                       case['trial_index'], freeze_hash, sensitivity=True, scope=case['data_scope'],
                       scenario_id=case['scenario_id'])
            key = (case['scenario_id'], case['calibration_run_id'], case['trial_index'])
            sensor_hash = sha(directory / 'sensor_observations.json')
            require(shared_sensors.setdefault(key, sensor_hash) == sensor_hash,
                    'sensitivity scopes use different camera sensor realization')
        summary = read(summary_path)
        require(summary['analysis_id'] == pointer['analysis_id'] and summary['analysis_freeze_sha256'] == freeze_hash
                and summary['v0_position_errors_evaluated_after_freeze'] is True
                and summary['v0_recalibration_performed'] is False, 'summary identity/freeze mismatch')
        return {'status': 'PASS', 'analysis_id': pointer['analysis_id'], 'main_batches': len(expected),
                'sensitivity_cases': len(cases), 'outcomes': dict(outcomes), 'evaluation_files_opened': True,
                'scope': 'coverage/schema/provenance; no numerical accuracy threshold or raw39 recertification'}
    except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:
        return {'status': 'FAIL', 'reason': f'{type(exc).__name__}: {exc}', 'evaluation_files_opened': opened}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--source-root', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    report = args.report.resolve()
    require(not report.is_relative_to(args.output_root.resolve() / 'analysis_versions')
            and not report.is_relative_to(args.source_root.resolve() / '원본_실행별'), 'report must be outside immutable analysis/raw')
    result = audit(args.output_root, args.source_root)
    with report.open('x', encoding='utf-8') as f: json.dump(result, f, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result['status'] == 'PASS' else 2

if __name__ == '__main__':
    raise SystemExit(main())
