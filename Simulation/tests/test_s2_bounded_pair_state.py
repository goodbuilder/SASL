from Simulation.stereo_ground_sim.stereo_ground_sim.time_alignment import BoundedIdHistory


def test_bounded_id_history_stays_bounded_for_ten_thousand_pairs() -> None:
    history = BoundedIdHistory(128)
    for pair_id in range(1, 10_001):
        assert history.add(pair_id)
        assert not history.add(pair_id)
    assert len(history) == 128
    assert 10_000 in history
    assert 9_872 not in history


def test_bounded_history_accepts_out_of_order_late_gt_ids() -> None:
    history = BoundedIdHistory(4)
    for pair_id in (10, 12, 11, 9):
        assert history.add(pair_id)
    assert len(history) == 4
    assert all(pair_id in history for pair_id in (9, 10, 11, 12))
