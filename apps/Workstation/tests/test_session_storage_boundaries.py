"""Storage composition guards for the method registry and one spool owner."""
from __future__ import annotations

import importlib
from pathlib import Path
import subprocess
import sys
import textwrap
from typing import get_type_hints

import pytest

from Workstation.workstation_core import recording_session as legacy
from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS
from Workstation.workstation_core.session_storage import models, schema, spool
from Workstation.workstation_core.session_storage import projection


def test_every_registered_method_has_its_authoritative_result_stream():
    # The runtime registry is independent of the storage catalog. Every
    # method needs a path even when no pair was ready to run that method.
    for method_id in MEASUREMENT_METHOD_IDS:
        assert schema.AUTHORITATIVE_JSONL_STREAMS[f"method_result_{method_id}"] == (
            f"raw/method_results/{method_id}",
            f"data/raw/method_results/{method_id}.jsonl",
        )
    assert legacy.AUTHORITATIVE_JSONL_STREAMS is schema.AUTHORITATIVE_JSONL_STREAMS


def test_storage_types_and_legacy_imports_share_exact_identity():
    assert legacy.SpoolRows is spool.SpoolRows
    assert legacy.SavedSessionFiles is models.SavedSessionFiles
    assert legacy.RetryableSession is models.RetryableSession
    assert legacy.SessionSheetPreview is models.SessionSheetPreview
    assert get_type_hints(models.RetryableSession)["pair_rows"] is spool.SpoolRows
    assert spool.SpoolRows.__module__ == "Workstation.workstation_core.recording_session"


def test_legacy_save_and_recovery_entrypoints_target_actual_owners():
    for name, module_name in (
        ("save_session_files", "finalize"),
        ("rebuild_session_review", "review"),
        ("load_retryable_session", "recovery"),
        ("write_recovery_metadata", "recovery"),
    ):
        owner = importlib.import_module(f"Workstation.workstation_core.session_storage.{module_name}")
        assert getattr(legacy, name) is getattr(owner, name)


def test_spool_is_usable_without_the_facade_gui_or_derived_writers(tmp_path):
    apps = Path(__file__).resolve().parents[2]
    script = textwrap.dedent("""
        import importlib.abc, sys
        from pathlib import Path
        apps = Path(sys.argv[1])
        sys.path[:0] = [str(apps), str(apps.parent / 'common/python')]
        forbidden = ('PyQt5', 'pyarrow', 'xlsxwriter', 'Workstation.workstation_core.recording_session')
        class Guard(importlib.abc.MetaPathFinder):
            def find_spec(self, fullname, path=None, target=None):
                if any(fullname == name or fullname.startswith(name + '.') for name in forbidden):
                    raise AssertionError('spool imported a UI/derived writer: ' + fullname)
                return None
        sys.meta_path.insert(0, Guard())
        from Workstation.workstation_core.session_storage.spool import SpoolRows
        rows = SpoolRows(preview_limit=1)
        rows.bind(Path(sys.argv[2]) / 'rows.jsonl')
        try:
            rows.append({'sample_index': 0, 'north_m': 11.0, 'east_m': -23.0, 'down_m': 37.0})
            rows.append({'sample_index': 1, 'north_m': 41.0, 'east_m': -53.0, 'down_m': 67.0})
            assert rows.preview_count == 1
            assert [row['north_m'] for row in rows] == [11.0, 41.0]
        finally:
            rows.close()
        assert not any(name in sys.modules for name in forbidden)
        print('spool owner used independently')
    """)
    result = subprocess.run(
        [sys.executable, "-I", "-B", "-c", script, str(apps), str(tmp_path)],
        capture_output=True, text=True, check=True, timeout=30,
    )
    assert result.stdout.strip() == "spool owner used independently"


@pytest.mark.parametrize(
    ("project", "original", "added"),
    [
        (projection._iter_project_pair_typed_fields,
         {"session_elapsed_ns": 17, "recorded_unix_ns": 123, "selected_pair_delta_ns": 7},
         {"recording_time_ns": 17, "estimated_utc_time_ns": 123, "sync_delta_ns": 7}),
        (projection._iter_project_camera_typed_fields,
         {"session_elapsed_s": 0.0000000015, "rtcp_sr_age_s": 0.25},
         {"recording_time_ns": 2, "rtcp_sr_age_ns": 250_000_000}),
        (projection._iter_project_px4_typed_fields,
         {"session_elapsed_s": 1.25, "source_utc_ns": 1_700_000_000_123_456_789},
         {"recording_time_ns": 1_250_000_000, "gps_utc_time_ns": 1_700_000_000_123_456_789}),
    ],
)
def test_derived_projection_consumes_one_copy_at_a_time(project, original, added):
    pulled = []
    source = {**original, "baseline_center_forward_m": 11.0,
              "baseline_center_right_m": -23.0, "baseline_center_down_m": 37.0}
    before = dict(source)

    def copies():
        for index in range(3):
            pulled.append(index)
            yield dict(source)

    rows = project(copies())
    assert pulled == []
    first = next(rows)
    assert pulled == [0]
    assert first == {**before, **added}
    assert source == before
    assert len(list(rows)) == 2
    assert source == before
