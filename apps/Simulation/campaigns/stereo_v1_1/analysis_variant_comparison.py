"""Pure, streaming RPY_IMU/YAW_ONLY comparisons; caller owns post-freeze access.

Comparability is conditional on case identity, successful fits, finite RMSE and
positive equal valid counts. Equal counts do NOT prove identical valid frame IDs.
No statistical significance, accuracy threshold or flight independence is inferred.
"""
from __future__ import annotations
from collections import Counter
from fractions import Fraction
import math
import re

METHODS = ('A_rtk_independent', 'B_rtk_joint', 'C_no_drone_rtk_joint')
VARIANTS = ('RPY_IMU', 'YAW_ONLY')
GROUP_FIELDS = ('family', 'method', 'rig_group', 'path_id', 'condition_kind', 'condition_id', 'data_scope')
IDENTITY_FIELDS = ('calibration_run_id', 'evaluation_run_id', 'physical_repetition', 'mc_trial')
VALUE_FIELDS = ('rmse_m', 'p95_m', 'valid_count', 'valid_measurement_rate', 'detection_rate')
NUMERIC_FIELDS = tuple(f'{prefix}_{name}' for prefix in ('rpy', 'yaw') for name in VALUE_FIELDS) + (
    'rmse_delta_rpy_minus_yaw_m', 'p95_delta_rpy_minus_yaw_m')
FIELDS = (*GROUP_FIELDS, *IDENTITY_FIELDS,
          'rpy_calibration_status', 'yaw_calibration_status',
          'rpy_evaluation_status', 'yaw_evaluation_status',
          'rpy_failure_reason', 'yaw_failure_reason', *NUMERIC_FIELDS,
          'comparison_status', 'outcome', 'rmse_direction', 'p95_direction')
CONTRACT = ('Case-level conditional RMSE comparison; equal valid counts do not establish identical valid '
            'frame IDs. No accuracy threshold, p-values or statistical independence claim. '
            'MC cases and distinct physical run/repetition identities are counted separately.')
_RUN = re.compile(r'^(G[01])-(P[0-4]|V0)-R([0-9]+)$')


def _require(condition, reason):
    if not condition:
        raise ValueError(reason)


def _integer(value, label):
    _require(isinstance(value, int) and not isinstance(value, bool) and value >= 0,
             f'{label} must be a nonnegative integer')
    return value


def _number(value, label):
    if value is None:
        return None
    _require(isinstance(value, (int, float)) and not isinstance(value, bool), f'{label} must be numeric or null')
    try:
        number = float(value)
    except OverflowError as exc:
        raise ValueError(f'{label} is nonfinite') from exc
    _require(math.isfinite(number), f'{label} is nonfinite')
    return number


def _identity(payload):
    cal, eva = payload.get('calibration_run_id'), payload.get('evaluation_run_id')
    c = _RUN.fullmatch(cal) if isinstance(cal, str) else None
    e = _RUN.fullmatch(eva) if isinstance(eva, str) else None
    _require(c is not None and e is not None and c[2] != 'V0' and e[2] == 'V0'
             and c[1] == e[1] and int(c[3]) == int(e[3]) > 0, 'invalid calibration/evaluation run pairing')
    trial = _integer(payload.get('trial_index'), 'trial_index')
    scope = payload.get('data_scope')
    _require(scope in ('all_data', 'matched_200'), 'invalid case data_scope')
    kind = 'scenario_id' if 'scenario_id' in payload else 'yaw_case'
    condition = payload.get(kind)
    _require(isinstance(condition, str) and bool(condition), f'missing {kind}')
    return {'family': 'sensitivity' if kind == 'scenario_id' else 'primary',
            'rig_group': c[1], 'path_id': c[2], 'condition_kind': kind, 'condition_id': condition,
            'data_scope': scope, 'calibration_run_id': cal, 'evaluation_run_id': eva,
            'physical_repetition': int(c[3]), 'mc_trial': trial}


def _index(rows, identity, *, calibration=False):
    _require(isinstance(rows, list), 'result rows must be a list')
    indexed = {}
    for row in rows:
        _require(isinstance(row, dict), 'result row must be an object')
        key = (row.get('method'), row.get('attitude_variant'))
        if key[0] not in METHODS or key[1] not in VARIANTS:
            continue  # M0, ORACLE and YAW_IDEAL are not this paired comparison.
        _require(key not in indexed, 'duplicate method/variant in case')
        for name in ('calibration_run_id', 'evaluation_run_id'):
            _require(row.get(name) == identity[name], f'cross-case {name}')
        _require(_integer(row.get('mc_seed'), 'mc_seed') == identity['mc_trial'], 'cross-case mc_seed')
        if 'trial_index' in row:
            _require(row['trial_index'] == identity['mc_trial'], 'cross-case trial_index')
        # Some legitimate NOT_RUN calibration records rely on their enclosing
        # bundle for scope. Evaluation records always carry an explicit scope.
        if not calibration or 'data_scope' in row:
            _require(row.get('data_scope') == identity['data_scope'], 'cross-scope result')
        for name in ('yaw_case', 'sensitivity_scenario_id', 'scenario_id'):
            if name in row:
                expected = (f"SENSITIVITY_{identity['condition_id']}"
                            if name == 'yaw_case' and identity['family'] == 'sensitivity'
                            else identity['condition_id'])
                _require(row[name] == expected, 'cross-condition result')
        field = 'convergence_status' if calibration else 'calibration_status'
        _require(isinstance(row.get(field), str) and bool(row[field]), f'missing {field}')
        if not calibration:
            _require(isinstance(row.get('evaluation_status'), str) and bool(row['evaluation_status']),
                     'missing evaluation_status')
        indexed[key] = row
    return indexed


def _metrics(row, prefix):
    metrics = row.get('metrics')
    _require(metrics is None or isinstance(metrics, dict), 'metrics must be an object or null')
    metrics = metrics or {}
    values = {}
    for name, source in (('rmse_m', 'position_3d_rmse_m'), ('p95_m', 'position_3d_p95_m'),
                         ('valid_measurement_rate', 'valid_measurement_rate'), ('detection_rate', 'detection_rate')):
        value = _number(metrics.get(source), source)
        _require(value is None or value >= 0, f'negative {source}')
        if name.endswith('rate'):
            _require(value is None or value <= 1, f'{source} exceeds one')
        values[f'{prefix}_{name}'] = value
    count = metrics.get('valid_position_estimate_count')
    values[f'{prefix}_valid_count'] = None if count is None else _integer(count, 'valid_position_estimate_count')
    return values


def _direction(delta):
    return 'UNAVAILABLE' if delta is None else 'LOWER' if delta < 0 else 'HIGHER' if delta > 0 else 'EQUAL'


def paired_variant_rows(payloads):
    """Yield three comparison rows per exact primary/sensitivity case.

    Input is an iterable of saved evaluation payloads, optionally enriched with
    calibration_results. No files, GT or fits are accessed. Missing variants
    fail closed except explicit top-level MATCHED_200_UNMET with empty results.
    """
    for payload in payloads:
        identity = _identity(payload)
        unmet = payload.get('evaluation_status') == 'MATCHED_200_UNMET'
        rows = payload.get('results')
        if unmet:
            _require(identity['data_scope'] == 'matched_200' and rows == [], 'invalid top-level matched-200 unmet case')
        evaluation = _index(rows, identity)
        cal_rows = payload.get('calibration_results')
        calibration = _index(cal_rows, identity, calibration=True) if cal_rows is not None else {}
        required = {(method, variant) for method in METHODS for variant in VARIANTS}
        if not unmet:
            _require(evaluation.keys() == required, 'missing paired evaluation variant')
        if cal_rows is not None and (not unmet or cal_rows):
            _require(calibration.keys() == required, 'missing paired calibration variant')
        for method in METHODS:
            output = {**identity, 'method': method, **dict.fromkeys(NUMERIC_FIELDS)}
            for prefix, variant in (('rpy', 'RPY_IMU'), ('yaw', 'YAW_ONLY')):
                ev = evaluation.get((method, variant))
                cal = calibration.get((method, variant))
                if ev is not None and cal is not None:
                    _require(ev['calibration_status'] == cal['convergence_status'], 'calibration/evaluation status mismatch')
                output[f'{prefix}_calibration_status'] = (cal['convergence_status'] if cal is not None
                    else ev['calibration_status'] if ev is not None else 'UNKNOWN')
                output[f'{prefix}_evaluation_status'] = 'MATCHED_200_UNMET' if unmet else ev['evaluation_status']
                output[f'{prefix}_failure_reason'] = ((cal or {}).get('failure_reason') or (ev or {}).get('failure_reason'))
                if ev is not None:
                    output.update(_metrics(ev, prefix))
            for metric in ('rmse', 'p95'):
                left, right = output[f'rpy_{metric}_m'], output[f'yaw_{metric}_m']
                delta = _number(left-right, f'{metric} delta') if left is not None and right is not None else None
                output[f'{metric}_delta_rpy_minus_yaw_m'] = delta
                output[f'{metric}_direction'] = _direction(delta)
            cs = (output['rpy_calibration_status'], output['yaw_calibration_status'])
            es = (output['rpy_evaluation_status'], output['yaw_evaluation_status'])
            r_count, y_count = output['rpy_valid_count'], output['yaw_valid_count']
            if unmet:
                status = 'MATCHED_200_UNMET'
            elif any(value != 'CONVERGED' for value in cs):
                status = ('CALIBRATION_NOT_CONVERGED' if any(value in ('OPTIMIZER_FAILED', 'RANK_DEFICIENT') for value in cs)
                          else 'CALIBRATION_NOT_RUN_OR_UNAVAILABLE')
            elif any(value != 'EVALUATED' for value in es):
                status = 'EVALUATION_UNAVAILABLE'
            elif output['rmse_delta_rpy_minus_yaw_m'] is None or r_count is None or y_count is None or min(r_count, y_count) == 0:
                status = 'METRIC_UNAVAILABLE'
            elif r_count != y_count:
                status = 'DIFFERENT_VALID_COUNTS'
            else:
                status = 'COMPARABLE_CASE_METRIC'
            output['comparison_status'] = status
            output['outcome'] = ({'LOWER':'IMPROVED', 'HIGHER':'REGRESSED', 'EQUAL':'NO_CHANGE'}[output['rmse_direction']]
                                 if status == 'COMPARABLE_CASE_METRIC' else 'NOT_COMPARABLE')
            yield output


class VariantComparisonSummary:
    """Bounded per-group counters/exact sums plus distinct run/trial identities."""
    def __init__(self):
        self.groups = {}

    def add(self, row):
        key = tuple(row[name] for name in GROUP_FIELDS)
        # Validate all numeric values before changing aggregation state.
        values = {name:_number(row.get(name), name) for name in NUMERIC_FIELDS}
        trial = _integer(row['mc_trial'], 'mc_trial')
        repetition = _integer(row['physical_repetition'], 'physical_repetition')
        group = self.groups.setdefault(key, {'count':0, 'statuses':Counter(), 'outcomes':Counter(),
            'cal_runs':set(), 'eval_runs':set(), 'repetitions':set(), 'trials':set(),
            'sums':{}, 'comparable_sums':{}})
        group['count'] += 1
        group['statuses'][row['comparison_status']] += 1
        group['outcomes'][row['outcome']] += 1
        group['cal_runs'].add(row['calibration_run_id']); group['eval_runs'].add(row['evaluation_run_id'])
        group['repetitions'].add((row['rig_group'], repetition)); group['trials'].add(trial)
        for name, value in values.items():
            if value is None: continue
            total, count = group['sums'].get(name, (Fraction(0), 0))
            group['sums'][name] = total + Fraction(value), count + 1
            if row['comparison_status'] == 'COMPARABLE_CASE_METRIC' and name.endswith('_delta_rpy_minus_yaw_m'):
                total, count = group['comparable_sums'].get(name, (Fraction(0), 0))
                group['comparable_sums'][name] = total + Fraction(value), count + 1

    def to_dict(self):
        rows = []
        for key, group in sorted(self.groups.items()):
            def means(values):
                return {name:{'mean':float(total/count), 'case_count':count} for name,(total,count) in sorted(values.items())}
            rows.append({**dict(zip(GROUP_FIELDS,key)), 'paired_case_count':group['count'],
                'comparison_status_counts':dict(sorted(group['statuses'].items())),
                'outcome_counts':dict(sorted(group['outcomes'].items())),
                'physical_calibration_run_count':len(group['cal_runs']),
                'physical_evaluation_run_count':len(group['eval_runs']),
                'physical_repetition_count':len(group['repetitions']), 'mc_trial_count':len(group['trials']),
                'all_finite_metric_means':means(group['sums']),
                'comparable_delta_means':means(group['comparable_sums'])})
        return {'schema_version':'stereo_paired_variant_summary_v1', 'comparison_metric':'position_3d_rmse_m',
                'delta_definition':'RPY_IMU minus YAW_ONLY; negative RMSE delta is lower',
                'interpretation':CONTRACT, 'paired_case_count':sum(g['count'] for g in self.groups.values()),
                'groups':rows}
