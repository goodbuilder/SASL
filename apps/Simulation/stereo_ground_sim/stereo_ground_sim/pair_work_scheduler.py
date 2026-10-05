"""Fair in-memory scheduling of measurement and GT-alignment work."""

from collections import deque


class PairWorkScheduler:
    def __init__(self):
        self.measurements = deque()
        self.alignments = deque()
        self.prefer_alignment_next = False

    @property
    def empty(self):
        return not self.measurements and not self.alignments

    def pop_next(self):
        """Alternate ready work classes so continuous images cannot starve GT."""

        if self.prefer_alignment_next and self.alignments:
            self.prefer_alignment_next = False
            return "alignment", self.alignments.popleft()
        if self.measurements:
            self.prefer_alignment_next = True
            return "measurement", self.measurements.popleft()
        if self.alignments:
            self.prefer_alignment_next = False
            return "alignment", self.alignments.popleft()
        return None, None

    def remove_pair(self, pair_id):
        pair_id = int(pair_id)
        self.measurements = deque(
            pair for pair in self.measurements if int(pair.pair_id) != pair_id
        )
        self.alignments = deque(
            item for item in self.alignments if int(item[0].pair_id) != pair_id
        )

    def clear(self):
        dropped = len(self.measurements) + len(self.alignments)
        self.measurements.clear()
        self.alignments.clear()
        self.prefer_alignment_next = False
        return dropped
