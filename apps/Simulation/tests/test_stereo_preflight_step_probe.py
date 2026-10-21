import json
from pathlib import Path
import subprocess
import pytest
from campaigns.stereo_v1_1 import preflight_capture as module


def test_probe_uses_workbook_world_isolated_backend_and_hashes_report(tmp_path, monkeypatch):
    parent_env = {'UNCHANGED': 'yes'}
    def run(command, **kwargs):
        assert command[command.index('--world-name') + 1] == 'declared_world'
        assert kwargs['env']['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] == 'python'
        assert kwargs['check'] and kwargs['timeout'] == 30
        Path(command[command.index('--report') + 1]).write_text(json.dumps({'status': 'PASS'}))
    monkeypatch.setattr(module.subprocess, 'run', run)
    source = {'environment_contract': [{'setting_id': 'world_name', 'declared_value': 'declared_world'}]}
    result = module._run_step_probe(Path('/usr/bin/python3'), tmp_path, source, parent_env)
    assert result['status'] == 'PASS' and len(result['sha256']) == 64
    assert parent_env == {'UNCHANGED': 'yes'}


def test_probe_process_failure_propagates(tmp_path, monkeypatch):
    def run(command, **kwargs):
        raise subprocess.CalledProcessError(1, command)
    monkeypatch.setattr(module.subprocess, 'run', run)
    with pytest.raises(subprocess.CalledProcessError):
        module._run_step_probe(Path('/usr/bin/python3'), tmp_path, {}, {})
