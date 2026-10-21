"""Identity admission must fail before the coordinator opens V0 errors."""
import copy
import json

import pytest

from campaigns.stereo_v1_1 import analyze_campaign as analysis


@pytest.fixture
def admission_case(tmp_path, monkeypatch):
    root, output = tmp_path / 'raw', tmp_path / 'analysis'
    root.mkdir(); output.mkdir()
    environment = root / 'environment_report.json'
    environment.write_text('{"status":"PREFLIGHT_PASS"}')
    (root / 'run_status.json').write_text('{"valid_run_count":39}')
    checksum = root / 'raw/a01/checksums.sha256'
    checksum.parent.mkdir(parents=True)
    checksum.write_text('original checksum inventory')
    protocol = {'source': 'fixture_spec', 'sha256': 'frozen-protocol'}
    manifest = {
        'algorithm_sha256': {'solver.py': 'start-version'}, 'dependencies': {'numpy': 'fixture'},
        'source_protocol': protocol, 'bundled_legacy_protocol_sha256': {'spec': 'legacy'},
        'mc_trial_count': 100, 'pixel_sigma_px': 1., 'huber_f_scale': 1.345,
        'environment_report_sha256': analysis._sha256(environment),
        'source_campaign_manifest_sha256': {'run_status.json': analysis._sha256(root / 'run_status.json')},
        'source_attempt_checksums_sha256': {'G0-P0-R01/a01': analysis._sha256(checksum)},
        'source_identity_sha256': 'initial-source-identity',
    }
    freeze = {key: copy.deepcopy(manifest[key]) for key in (
        'algorithm_sha256', 'dependencies', 'source_protocol', 'bundled_legacy_protocol_sha256',
        'mc_trial_count', 'pixel_sigma_px', 'huber_f_scale')}
    for filename, field in (
        ('path_selection.json', 'path_selection_sha256'),
        ('calibration_freeze_manifest.json', 'primary_calibration_checksums_manifest_sha256'),
        ('sensitivity_calibration_manifest.json', 'sensitivity_calibration_manifest_sha256'),
        ('sensitivity_calibration_checksums.json', 'sensitivity_calibration_checksums_sha256'),
    ):
        (output / filename).write_text('{}')
        freeze[field] = analysis._sha256(output / filename)
    analysis._write_immutable(output / 'analysis_run_manifest.json', manifest)
    analysis._write_immutable(output / 'analysis_freeze.json', freeze)
    monkeypatch.setattr(analysis, '_analysis_hashes', lambda: manifest['algorithm_sha256'])
    monkeypatch.setattr(analysis, '_analysis_dependencies', lambda: manifest['dependencies'])
    calls = []
    def qc(path):
        calls.append(path)
        return {'valid_run_count': 39}, [{'run_id': 'G0-P0-R01',
            'attempts': [{'attempt_id': 'a01', 'recording_path': 'raw/a01'}]}]
    monkeypatch.setattr(analysis, '_status_slots', qc)
    monkeypatch.setattr(analysis, '_load_analysis_protocol', lambda root,status: ({}, protocol))
    return root, output, manifest, freeze, environment, calls


def test_matching_identity_rechecks_raw_and_persists_unchanged_admission(admission_case):
    args = admission_case[:5]
    result = analysis._admit_frozen_evaluation(*args)
    assert result['status'] == 'PRE_V0_IDENTITY_CHECK_PASSED'
    assert result['analysis_freeze_sha256'] == analysis._sha256(args[1] / 'analysis_freeze.json')
    assert admission_case[5] == [args[0]]
    before = (args[1] / 'evaluation_admission.json').read_bytes()
    assert analysis._admit_frozen_evaluation(*args) == result
    assert (args[1] / 'evaluation_admission.json').read_bytes() == before


@pytest.mark.parametrize('field', ['algorithm_sha256', 'dependencies', 'source_protocol',
                                 'mc_trial_count', 'pixel_sigma_px', 'huber_f_scale'])
def test_start_and_freeze_mismatch_is_rejected_before_raw_or_v0(admission_case, field):
    root, output, manifest, freeze, env, calls = admission_case
    freeze[field] = 'changed'
    (output / 'analysis_freeze.json').write_text(json.dumps(freeze))
    with pytest.raises(RuntimeError, match='identity changed'):
        analysis._admit_frozen_evaluation(root, output, manifest, freeze, env)
    assert calls == [] and not (output / 'evaluation_admission.json').exists()


def test_current_source_version_drift_is_rejected(admission_case, monkeypatch):
    root, output, manifest, freeze, env, calls = admission_case
    monkeypatch.setattr(analysis, '_analysis_hashes', lambda: {'solver.py': 'new-version'})
    with pytest.raises(RuntimeError, match='current algorithm/dependency'):
        analysis._admit_frozen_evaluation(root, output, manifest, freeze, env)
    assert calls == []


@pytest.mark.parametrize('filename', ['environment_report.json', 'run_status.json', 'raw/a01/checksums.sha256'])
def test_source_artifact_change_cannot_admit_evaluation(admission_case, filename):
    root, output, manifest, freeze, env, _ = admission_case
    (root / filename).write_text('changed source artifact')
    with pytest.raises(RuntimeError, match='changed before V0'):
        analysis._admit_frozen_evaluation(root, output, manifest, freeze, env)
    assert not (output / 'evaluation_admission.json').exists()


def test_fresh_protocol_identity_is_checked(admission_case, monkeypatch):
    root, output, manifest, freeze, env, _ = admission_case
    monkeypatch.setattr(analysis, '_load_analysis_protocol', lambda *args: ({}, {'sha256': 'changed'}))
    with pytest.raises(RuntimeError, match='source protocol identity'):
        analysis._admit_frozen_evaluation(root, output, manifest, freeze, env)


@pytest.mark.parametrize('filename', ['path_selection.json', 'calibration_freeze_manifest.json',
                                    'sensitivity_calibration_manifest.json', 'sensitivity_calibration_checksums.json'])
def test_frozen_calibration_inventory_drift_is_rejected(admission_case, filename):
    root, output, manifest, freeze, env, _ = admission_case
    (output / filename).write_text('{"changed":true}')
    with pytest.raises(RuntimeError, match='frozen calibration/selection artifact'):
        analysis._admit_frozen_evaluation(root, output, manifest, freeze, env)
    assert not (output / 'evaluation_admission.json').exists()
