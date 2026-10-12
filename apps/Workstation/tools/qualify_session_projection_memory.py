#!/usr/bin/env python3
"""Offline Linux RSS comparison of eager versus lazy derived projections.

Synthetic rows only; no devices, live app settings or user recordings. The
eager reference restores the previous projection allocation in this process.
Run each row count in a fresh process. Optional profiles cover unique geometry
contexts, camera epochs, method-result indexing, mutable-spool replay and Excel.
No memory pass threshold is invented here: JSON reports measured growth for
the existing change packet's same-row-size, increasing-row-count review.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import resource
import sys
import tempfile
import time

SASL_ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(SASL_ROOT / "apps"), str(SASL_ROOT / "common" / "python")]

from Workstation.workstation_core.session_storage import projection
from Workstation.workstation_core.session_storage.finalize import save_session_files


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rows", type=int, default=10_000)
    parser.add_argument("--eager-reference", action="store_true", help="restore eager Parquet projection copies only; not the historical whole-save algorithm")
    parser.add_argument("--include-review", action="store_true")
    parser.add_argument("--unique-contexts", action="store_true")
    parser.add_argument("--epoch-per-row", action="store_true")
    parser.add_argument("--method-bundles", action="store_true")
    parser.add_argument("--spool-updates", action="store_true")
    parser.add_argument("--payload-bytes", type=int, default=0)
    parser.add_argument("--compare-rows", type=int, nargs="+", help="run these sizes in fresh child processes and report RSS deltas")
    args = parser.parse_args()
    if args.rows < 1 or args.payload_bytes < 0 or not sys.platform.startswith("linux"):
        parser.error("use a positive row count and nonnegative payload bytes on Linux")
    if args.compare_rows:
        import subprocess
        values = []
        flags = [name for name, enabled in (
            ("--include-review", args.include_review), ("--unique-contexts", args.unique_contexts),
            ("--epoch-per-row", args.epoch_per_row), ("--method-bundles", args.method_bundles),
            ("--spool-updates", args.spool_updates), ("--eager-reference", args.eager_reference),
        ) if enabled]
        for count in args.compare_rows:
            completed = subprocess.run([sys.executable, "-B", str(Path(__file__).resolve()), "--rows", str(count),
                                        "--payload-bytes", str(args.payload_bytes), *flags], check=True, capture_output=True, text=True)
            values.append(json.loads(completed.stdout))
        print(json.dumps({"runs": values,
            "peak_rss_growth_mib": round(values[-1]["peak_rss_mib"] - values[0]["peak_rss_mib"], 3),
            "assessment": "measurement only; compare identical row size and review/index profile; no relaxed pass threshold"}, sort_keys=True))
        return
    if args.eager_reference:
        for kind in ("pair", "camera", "px4"):
            eager = getattr(projection, f"_project_{kind}_typed_fields")
            setattr(projection, f"_iter_project_{kind}_typed_fields", lambda rows, eager=eager: eager(list(rows)))

    def pairs():
        for index in range(args.rows):
            yield {
                "pair_id": f"synthetic-{index}", "sample_index": index,
                "session_elapsed_ns": index * 100_000_000,
                "recorded_unix_ns": 1_700_000_000_000_000_000 + index * 100_000_000,
                "baseline_center_forward_m": 50.0 + index * 0.01,
                "baseline_center_right_m": -2.0, "baseline_center_down_m": 3.0,
                "distance_m": 60.0, "baseline_m": 10.0,
                "pair_status": "VALID", "class_name": "offline-fixture",
                "measurement_method_id": "pixhawk_rtk_live",
                "measurement_valid": True,
                "intrinsics_profile_sha256": f"{index if args.unique_contexts else 0:064x}",
                "left_app_receive_monotonic_ns": 1_000_000_000_000 + index * 100_000_000,
                "right_app_receive_monotonic_ns": 1_000_000_000_000 + index * 100_000_000,
                "left_rtp_ssrc": index + 1 if args.epoch_per_row else 101,
                "right_rtp_ssrc": index + 1 if args.epoch_per_row else 101,
                "synthetic_payload": "x" * args.payload_bytes,
                "selected_pair_delta_ns": 1_000_000,
            }

    def camera_rows():
        for index in range(args.rows):
            yield {
                "sequence": index, "session_elapsed_s": index / 10,
                "timestamp_s": 1000 + index / 10,
                "app_receive_monotonic_ns": 1_000_000_000_000 + index * 100_000_000,
                "host_receive_monotonic_ns": 1_000_000_000_000 + index * 100_000_000,
                "source_utc_ns": 1_700_000_000_000_000_000 + index * 100_000_000,
                "rtp_ssrc": index + 1 if args.epoch_per_row else 101,
                "stream_generation": 1, "rtcp_sr_received_count": 2,
                "session_utc_ns": 1_700_000_000_000_000_000 + index * 100_000_000,
                "rtcp_sr_age_s": 0.1, "rtcp_sr_pc_offset_s": 0.002,
                "rtcp_frame_receive_delay_s": 0.003, "rtcp_frame_transport_delay_s": 0.004,
            }

    def px4_rows():
        for index in range(args.rows):
            yield {
                "sequence": index, "session_elapsed_s": index / 10,
                "sample_type": "mavlink_attitude", "topic": "mavlink/ATTITUDE",
                "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0,
                "session_utc_ns": 1_700_000_000_000_000_000 + index * 100_000_000,
            }

    def bundles():
        if not args.method_bundles:
            return
        from Workstation.workstation_core.method_evaluation import MethodEvaluationBundle, MethodEvaluationOutcome
        from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS
        for index in range(args.rows):
            fields = dict(measurement_id="m000001", pair_id=f"synthetic-{index}", pair_sequence=index,
                          snapshot_sha256=f"{index:064x}", request_generation=1)
            yield MethodEvaluationBundle(**fields, outcomes=tuple(
                MethodEvaluationOutcome(**fields, method_id=method, method_instance_id="", status="BLOCKED", reasons=("offline memory fixture",))
                for method in MEASUREMENT_METHOD_IDS
            )).to_record()

    started = time.perf_counter()
    with tempfile.TemporaryDirectory(prefix="workstation-projection-memory-") as directory:
        output = Path(directory) / "offline-memory" / "measurements" / "m000001"
        pair_source = pairs()
        if args.spool_updates:
            from Workstation.workstation_core.session_storage.spool import SpoolRows
            pair_source = SpoolRows(preview_limit=2, supports_updates=True)
            pair_source.bind(output / ".spool/pair_rows.jsonl")
            for index, row in enumerate(pairs()):
                pair_source.append(row)
                if index % 1000 == 0:
                    pair_source.replace(index, row)
            pair_source.close()
        saved = save_session_files(
            output,
            metadata={"session_id": "offline-memory", "measurement_id": "m000001",
                      "purpose": "offline projection memory qualification only"},
            pair_rows=pair_source, camera_rows={"left": camera_rows(), "right": camera_rows()},
            method_evaluation_bundle_rows=bundles(),
            px4_rows={"left": px4_rows(), "right": px4_rows()},
        )
        assert saved.finalized and saved.review_pending
        if args.include_review:
            from Workstation.workstation_core.session_storage.review import rebuild_session_review
            review = rebuild_session_review(output.parent.parent, "offline-memory")
            assert review.is_file()
        manifest = json.loads(saved.manifest.read_text())
        context_count = manifest["artifact_details"]["geometry_contexts"]["row_count"]
        epoch_count = manifest["artifact_details"]["time_sync_epochs"]["row_count"]
        assert context_count == (args.rows if args.unique_contexts else 1)
        assert epoch_count == (args.rows * 2 if args.epoch_per_row else 2)
    print(json.dumps({
        "strategy": "eager-reference" if args.eager_reference else "streaming-finalization",
        "rows_per_stream": args.rows, "streams": 5,
        "elapsed_s": round(time.perf_counter() - started, 3),
        "peak_rss_mib": round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024, 3),
        "review": "rebuilt and verified" if args.include_review else "deferred; not part of this memory claim",
        "unique_contexts": args.unique_contexts, "geometry_context_count": context_count,
        "epoch_per_row": args.epoch_per_row, "time_sync_epoch_count": epoch_count,
        "method_bundles": args.method_bundles, "spool_updates": args.spool_updates,
        "payload_bytes": args.payload_bytes,
        "scope": "canonical source, disk indexes and Parquet; Excel only when include-review is true",
        "unbounded_inputs": "individual row size, metadata, dynamic schema fields and session measurement inventory",

    }, sort_keys=True))


if __name__ == "__main__":
    main()
