"""V0 final evaluation is not opened by structural GT alignment."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest


SIM_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.align_gt import align_attempt


def test_v0_tracking_reveal_requires_later_freeze(tmp_path: Path):
    attempt = tmp_path / "attempt"
    attempt.mkdir()
    (attempt / "run_manifest.json").write_text(json.dumps({"path_id": "V0"}))
    output = tmp_path / "derived"
    with pytest.raises(ValueError, match="remain closed"):
        align_attempt(attempt, output, reveal_tracking=True)
    assert not output.exists()
