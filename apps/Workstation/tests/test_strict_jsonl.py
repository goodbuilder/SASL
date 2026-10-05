from __future__ import annotations

import json
from pathlib import Path

import pytest

from Workstation.workstation_core.strict_jsonl import (
    append_jsonl_record,
    prepare_jsonl_append,
)
from Workstation.workstation_core.run_evidence import RunEvidenceLedger


def test_torn_final_record_is_quarantined_without_valid_prefix_loss(
    tmp_path: Path,
) -> None:
    path = tmp_path / "events.jsonl"
    prefix = b'{"event":1}\n{"event":2}\n'
    torn = b'{"event":'
    path.write_bytes(prefix + torn)

    recovery = prepare_jsonl_append(path)

    assert recovery.repaired is True
    assert recovery.row_count == 2
    assert path.read_bytes() == prefix
    assert recovery.quarantined_tail is not None
    assert recovery.quarantined_tail.read_bytes() == torn


def test_complete_unterminated_record_only_gets_newline(tmp_path: Path) -> None:
    path = tmp_path / "events.jsonl"
    path.write_bytes(b'{"event":1}\n{"event":2}')

    recovery = prepare_jsonl_append(path)

    assert recovery.normalized_newline is True
    assert recovery.row_count == 2
    assert path.read_bytes() == b'{"event":1}\n{"event":2}\n'


def test_malformed_middle_record_is_hard_failure_without_mutation(
    tmp_path: Path,
) -> None:
    path = tmp_path / "events.jsonl"
    original = b'{"event":1}\nnot-json\n{"event":3}\n'
    path.write_bytes(original)

    with pytest.raises(OSError, match="line=2"):
        prepare_jsonl_append(path)

    assert path.read_bytes() == original


def test_append_is_canonical_fsynced_json_and_rejects_symlink(tmp_path: Path) -> None:
    path = tmp_path / "events.jsonl"
    append_jsonl_record(path, {"z": 2, "a": "산"})

    assert path.read_bytes() == '{"a":"산","z":2}\n'.encode("utf-8")
    assert json.loads(path.read_text()) == {"a": "산", "z": 2}
    outside = tmp_path / "outside.jsonl"
    outside.write_text("", encoding="utf-8")
    link = tmp_path / "link.jsonl"
    link.symlink_to(outside)
    with pytest.raises(OSError, match="symlink"):
        append_jsonl_record(link, {"escape": True})
    assert outside.read_bytes() == b""


def test_run_evidence_ledger_repairs_tail_once_then_appends(tmp_path: Path) -> None:
    root = tmp_path / "session"
    root.mkdir()
    path = root / "run_events.jsonl"
    path.write_bytes(b'{"schema":"old"}\n{"torn":')
    if callable(getattr(RunEvidenceLedger, "apply_setup", None)):
        from Workstation.workstation_core.run_evidence import RunMode, RunSetup

        setup = RunSetup(
            experiment_id="exp",
            run_id="run",
            condition_id="condition",
            mode=RunMode.DRONE,
            condition=(),
            requested_zoom_left=1,
            requested_zoom_right=1,
        )
        (root / "run_setup.json").write_text(
            json.dumps(setup.to_mapping()),
            encoding="utf-8",
        )
    ledger = RunEvidenceLedger(root)
    event = ledger.append_event(
        "capture",
        {"value": 1},
        source="test",
    )

    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert rows[0] == {"schema": "old"}
    assert rows[1]["event_id"] == event["event_id"]
    assert ledger._ledger_recovery["run_events"].quarantined_tail is not None
