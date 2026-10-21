"""Lightweight harness boundaries only; no ROS process is launched by these tests."""
from pathlib import Path
import pytest
from campaigns.stereo_v1_1.verify_capture_recovery import owned_bag_argv, snapshot_files, run


def test_bag_target_requires_ros2_record_and_exact_owned_output(tmp_path):
    output = str(tmp_path / 'raw_images_or_lossless_bag' / 'left')
    argv = ['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'bag', 'record', '-o', output]
    assert owned_bag_argv(argv, tmp_path)
    assert not owned_bag_argv(argv[:-1]+[output+'-unrelated'], tmp_path)
    assert not owned_bag_argv(['rm', 'bag', 'record', '-o', output], tmp_path)
    assert not owned_bag_argv(['/opt/ros/humble/bin/ros2', 'bag', 'play', '-o', output], tmp_path)
    assert not owned_bag_argv(['ros2', 'bag', 'record', '-o'], tmp_path)


def test_snapshot_detects_modified_removed_and_added_files(tmp_path):
    raw = tmp_path / 'raw'
    raw.mkdir()
    file = raw / 'image.db3'
    file.write_bytes(b'original')
    original = snapshot_files(tmp_path)
    assert snapshot_files(tmp_path) == original
    file.write_bytes(b'modified')
    assert snapshot_files(tmp_path) != original
    file.unlink()
    assert snapshot_files(tmp_path) != original


def test_domain_zero_refused_before_output_or_subprocess(tmp_path):
    with pytest.raises(ValueError, match='isolated nonzero'):
        run(tmp_path / 'new', 0)
    assert not (tmp_path / 'new').exists()


def test_existing_output_refused_without_process_launch(tmp_path, monkeypatch):
    import campaigns.stereo_v1_1.verify_capture_recovery as module
    monkeypatch.setattr(module, 'refuse_sitl', lambda: None)
    with pytest.raises(FileExistsError):
        run(tmp_path, 89)
    assert list(tmp_path.iterdir()) == []
