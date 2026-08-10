from __future__ import annotations

from Simulation.stereo_ground_sim.stereo_ground_sim.time_alignment import StereoImagePairer


def _run(events):
    pairer = StereoImagePairer(max_buffer_size=8)
    pairs = []
    for side, stamp in events:
        pairs.extend(pairer.add(side, stamp, f"{side}:{stamp}"))
    pairs.extend(pairer.flush())
    return pairer, pairs


def test_exact_and_signed_nearest_pairing_without_reuse() -> None:
    pairer, pairs = _run(
        [("left", 100), ("right", 100), ("left", 200), ("right", 202), ("left", 300), ("right", 299)]
    )
    assert [(pair.pairing_mode, pair.pair_delta_ns, pair.pair_midpoint_stamp_ns) for pair in pairs] == [
        ("exact", 0, 100),
        ("nearest", 2, 201),
        ("nearest", -1, 299),
    ]
    assert pairer.paired_count == 3
    assert len({pair.left_message for pair in pairs}) == 3
    assert len({pair.right_message for pair in pairs}) == 3
    assert [pair.pair_id for pair in pairs] == [1, 2, 3]


def test_callback_side_order_produces_same_stamp_pairs() -> None:
    first = _run([("left", 100), ("right", 102), ("left", 200), ("right", 201)])[1]
    second = _run([("right", 102), ("left", 100), ("right", 201), ("left", 200)])[1]
    contract = lambda pairs: [(p.left_stamp_ns, p.right_stamp_ns, p.pair_delta_ns) for p in pairs]
    assert contract(first) == contract(second)


def test_older_nearest_pair_is_emitted_before_newer_exact_pair() -> None:
    _, pairs = _run([("left", 100), ("right", 102), ("left", 200), ("right", 200)])
    assert [(pair.pair_midpoint_stamp_ns, pair.pairing_mode, pair.pair_id) for pair in pairs] == [
        (101, "nearest", 1),
        (200, "exact", 2),
    ]


def test_duplicate_and_extra_frame_are_accounted() -> None:
    pairer, pairs = _run([("left", 10), ("left", 10), ("right", 10), ("right", 20)])
    assert len(pairs) == 2
    assert sum(pair.pairing_mode == "exact" for pair in pairs) == 1
    assert pairer.unmatched_left_count + pairer.unmatched_right_count == 0


def test_bounded_overflow_counts_unmatched() -> None:
    pairer = StereoImagePairer(max_buffer_size=2)
    pairer.add("left", 1, object())
    pairer.add("left", 2, object())
    pairer.add("left", 3, object())
    assert pairer.unmatched_left_count == 1
