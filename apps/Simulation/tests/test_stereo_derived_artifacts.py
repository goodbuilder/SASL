"""Real filesystem sharing boundaries; no campaign data is accessed."""
import errno
import hashlib
import os
from pathlib import Path
import pytest
from campaigns.stereo_v1_1 import derived_artifacts as module


def setup_files(tmp_path):
    root = tmp_path / 'analysis'
    root.mkdir()
    source = root / 'generated' / 'source.jsonl'
    source.parent.mkdir()
    source.write_bytes(b'full immutable bytes\x00\n' * 1000)
    return root, source, root / 'derived' / 'copy.jsonl'


def test_real_hardlink_preserves_all_bytes_and_sha(tmp_path):
    root, source, target = setup_files(tmp_path)
    expected = source.read_bytes()
    digest = module.link_immutable_derived(source, target, root)
    assert digest == hashlib.sha256(expected).hexdigest()
    assert target.read_bytes() == expected
    assert (source.stat().st_dev, source.stat().st_ino) == (target.stat().st_dev, target.stat().st_ino)


def test_existing_equal_accepted_and_different_never_overwritten(tmp_path):
    root, source, target = setup_files(tmp_path)
    target.parent.mkdir()
    target.write_bytes(source.read_bytes())
    inode = target.stat().st_ino
    module.link_immutable_derived(source, target, root)
    assert target.stat().st_ino == inode
    target.write_bytes(b'conflicting evidence')
    with pytest.raises(ValueError, match='differs'):
        module.link_immutable_derived(source, target, root)
    assert target.read_bytes() == b'conflicting evidence'


def test_outside_raw_and_symlink_escape_refused(tmp_path):
    root, source, target = setup_files(tmp_path)
    outside = tmp_path / 'external'
    outside.mkdir()
    other = outside / 'input'
    other.write_bytes(b'outside')
    (root / 'escape').symlink_to(outside, target_is_directory=True)
    for src, dst in [(other,target), (source,outside/'copy'),
                     (source,root/'escape'/'copy'), (root/'escape'/'input',target),
                     (source,root/'raw_images_or_lossless_bag'/'copy'),
                     (source,root/'원본_실행별'/'copy')]:
        with pytest.raises(ValueError):
            module.link_immutable_derived(src,dst,root)
    assert not target.exists()


def test_exdev_propagates_without_fallback_or_artifact_loss(tmp_path, monkeypatch):
    root, source, target = setup_files(tmp_path)
    original = source.read_bytes()
    def exdev(*args):
        raise OSError(errno.EXDEV,'different devices')
    monkeypatch.setattr(module.os,'link',exdev)
    with pytest.raises(OSError) as caught:
        module.link_immutable_derived(source,target,root)
    assert caught.value.errno == errno.EXDEV
    assert source.read_bytes() == original
    assert not target.exists()


def test_mutation_during_operation_detected_and_evidence_retained(tmp_path, monkeypatch):
    root, source, target = setup_files(tmp_path)
    real_link = os.link
    def changing_link(src,dst):
        real_link(src,dst)
        source.write_bytes(b'prohibited source mutation')
    monkeypatch.setattr(module.os,'link',changing_link)
    with pytest.raises(ValueError,match='changed'):
        module.link_immutable_derived(source,target,root)
    assert source.exists() and target.exists()
    assert target.read_bytes() == b'prohibited source mutation'


def test_nonregular_source_refused(tmp_path):
    root, source, target = setup_files(tmp_path)
    with pytest.raises(ValueError,match='regular'):
        module.link_immutable_derived(source.parent,target,root)


def test_raw_directory_cannot_be_relabelled_as_owned_analysis_root(tmp_path):
    raw = tmp_path / 'raw_images_or_lossless_bag'
    raw.mkdir()
    source = raw / 'original.db3'
    source.write_bytes(b'raw')
    with pytest.raises(ValueError, match='raw recording'):
        module.link_immutable_derived(source,raw/'copy',raw)
    assert list(raw.iterdir()) == [source]
