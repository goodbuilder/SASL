"""Single-owner state for joining measurements with Ground Truth.

This registry stores objects only.  It performs no coordinate conversion:
measurement vectors retain their producer-declared baseline-center FRD frame,
while alignments retain world ENU until the evaluation pipeline converts them.
"""

from dataclasses import dataclass, field

from stereo_ground_sim.time_alignment import BoundedIdHistory


@dataclass
class PairEvaluationRegistry:
    """Own the three-way pair/measurement/alignment join and tombstones."""

    history_capacity: int
    pairs: dict = field(default_factory=dict)
    measurements: dict = field(default_factory=dict)
    alignments: dict = field(default_factory=dict)
    evaluated_ids: BoundedIdHistory = field(init=False)
    failed_ids: BoundedIdHistory = field(init=False)
    published_gt_ids: BoundedIdHistory = field(init=False)

    def __post_init__(self):
        capacity = int(self.history_capacity)
        self.evaluated_ids = BoundedIdHistory(capacity)
        self.failed_ids = BoundedIdHistory(capacity)
        self.published_gt_ids = BoundedIdHistory(capacity)

    def is_terminal(self, pair_id):
        return pair_id in self.evaluated_ids or pair_id in self.failed_ids

    def store_measurement(self, pair_id, metrics):
        if self.is_terminal(pair_id):
            return False
        self.measurements[pair_id] = metrics
        return True

    def store_alignment(self, pair_id, alignment):
        if pair_id not in self.pairs or self.is_terminal(pair_id):
            return False
        self.alignments[pair_id] = alignment
        return True

    def ready(self, pair_id):
        if self.is_terminal(pair_id):
            return None
        pair = self.pairs.get(pair_id)
        measurement = self.measurements.get(pair_id)
        alignment = self.alignments.get(pair_id)
        if pair is None or measurement is None or alignment is None:
            return None
        return pair, measurement, alignment

    def finalize(self, pair_id, *, failed=False):
        (self.failed_ids if failed else self.evaluated_ids).add(pair_id)
        result = (
            self.pairs.pop(pair_id, None),
            self.measurements.pop(pair_id, None),
            self.alignments.pop(pair_id, None),
        )
        return result
