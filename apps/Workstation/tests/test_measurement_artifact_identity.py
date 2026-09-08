from Workstation.workstation_core.measurement_artifacts import (
    measurement_start_stamp,
    reserve_measurement_context,
)


def test_start_stamp_does_not_round_into_the_next_second():
    assert measurement_start_stamp(999_999_999) == "19700101_090000_999999999+0900"
    assert measurement_start_stamp(1_000_000_000) == "19700101_090001_000000000+0900"


def test_collision_reserves_a_whole_run_and_artifacts_keep_start(tmp_path):
    first = reserve_measurement_context(tmp_path, "run-a", 1, 123)
    second = reserve_measurement_context(tmp_path, "run-b", 1, 124)
    assert second.file_stamp == first.file_stamp + "-01"
    paths = [first.allocate("media", "left", ".mp4") for _ in range(2)]
    assert paths[0] != paths[1]
    assert all(first.file_stamp in p.name and p.parent.parent == first.root for p in paths)
    assert first.file_stamp in first.workbook_path.name
    assert first.file_stamp in first.stream_path("measurements").name
