"""Derived storage ownership at the real analysis copy boundary."""
from pathlib import Path

import pytest

from campaigns.stereo_v1_1.analyze_campaign import _copy_verified


def test_owned_generated_input_keeps_all_bytes_and_shares_only_derived_inode(tmp_path):
    analysis = tmp_path / 'analysis'
    source = analysis / 'generated_inputs' / 'rtk.jsonl'
    source.parent.mkdir(parents=True)
    source.write_bytes(b'{"observation":1}\n')
    target = analysis / 'case' / 'drone_rtk_observations.jsonl'
    digest = _copy_verified(source, target, owned_root=analysis)
    assert source.stat().st_ino == target.stat().st_ino
    assert source.read_bytes() == target.read_bytes() == b'{"observation":1}\n'
    assert _copy_verified(source, target, owned_root=analysis) == digest


def test_external_raw_input_remains_an_independent_copy(tmp_path):
    analysis = tmp_path / 'analysis'
    analysis.mkdir()
    raw = tmp_path / '원본_실행별' / 'sensor.json'
    raw.parent.mkdir()
    raw.write_bytes(b'original')
    target = analysis / 'case' / 'sensor.json'
    _copy_verified(raw, target, owned_root=analysis)
    assert raw.read_bytes() == target.read_bytes()
    assert raw.stat().st_ino != target.stat().st_ino
    raw.write_bytes(b'changed external source')
    assert target.read_bytes() == b'original'
    with pytest.raises(FileExistsError):
        _copy_verified(raw, target, owned_root=analysis)


def test_owned_source_cannot_link_to_another_owner(tmp_path):
    analysis = tmp_path / 'analysis'
    analysis.mkdir()
    source = analysis / 'input.json'
    source.write_text('{}')
    with pytest.raises(ValueError, match='same derived analysis root'):
        _copy_verified(source, tmp_path / 'outside.json', owned_root=analysis)
    assert not (tmp_path / 'outside.json').exists()


def test_public_analysis_uses_exactly_the_normalized_locked_root(tmp_path, monkeypatch):
    from campaigns.stereo_v1_1 import analyze_campaign as analysis
    quoted = Path('~/analysis-output')
    canonical = (tmp_path / 'actual-analysis').resolve()
    expand = Path.expanduser
    monkeypatch.setattr(Path, 'expanduser',
        lambda value: canonical if value == quoted else expand(value))
    def run(output, trials, source, *, workers):
        assert output == canonical
        assert workers == 2
        with pytest.raises(RuntimeError, match='active coordinator'):
            with analysis.analysis_output_lock(output):
                pass
        return {'owned_root': str(output)}
    monkeypatch.setattr(analysis, '_analyze_locked', run)
    assert analysis.analyze(quoted, workers=2) == {'owned_root': str(canonical)}
    with analysis.analysis_output_lock(canonical):
        pass
