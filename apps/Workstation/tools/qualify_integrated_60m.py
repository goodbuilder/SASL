#!/usr/bin/env python3
"""Low-rate, non-consuming oracle for the Workstation 5+60 minute field run."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from typing import Any, Mapping

import pyatspi

APPS_ROOT = Path(__file__).resolve().parents[2]
if str(APPS_ROOT) not in sys.path:
    sys.path.insert(0, str(APPS_ROOT))

from Workstation.workstation_core.mavlink_liveness import MAVLINK_QUALIFICATION_S


SCHEMA = "workstation-integrated-qualification-v1"
DIAGNOSTIC_KEYS = (
    "camera.left",
    "camera.right",
    "pixhawk.left",
    "pixhawk.right",
    "gnss.ntrip",
    "gnss",
    "detection",
    "synchronization",
    "measurement",
    "recording",
    "fps",
)
CAMERA_KEYS = ("camera.left", "camera.right")
PIXHAWK_KEYS = ("pixhawk.left", "pixhawk.right")
CAMERA_NICS = ("enx588694fce07e", "enx0023a40a2e39")
MIN_FREE_BYTES = 100 * 1024**3


def _finite(value: float) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("numeric evidence must be finite")
    return result


def _proc_rows() -> dict[int, dict[str, int]]:
    rows: dict[int, dict[str, int]] = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / "stat").read_text(encoding="utf-8").split()
            status_lines = (entry / "status").read_text(encoding="utf-8").splitlines()
        except (OSError, UnicodeError):
            continue
        status: dict[str, str] = {}
        for line in status_lines:
            key, separator, value = line.partition(":")
            if separator:
                status[key] = value.strip()
        try:
            rows[int(entry.name)] = {
                "ppid": int(fields[3]),
                "cpu_ticks": int(fields[13]) + int(fields[14]),
                "rss_kib": int(status.get("VmRSS", "0 kB").split()[0]),
                "threads": int(status.get("Threads", "0")),
            }
        except (IndexError, TypeError, ValueError):
            continue
    return rows


def _process_tree(root_pid: int) -> tuple[int, ...]:
    rows = _proc_rows()
    if root_pid not in rows:
        return ()
    selected = {root_pid}
    changed = True
    while changed:
        changed = False
        for pid, row in rows.items():
            if pid not in selected and row["ppid"] in selected:
                selected.add(pid)
                changed = True
    return tuple(sorted(selected))


def _process_snapshot(root_pid: int) -> dict[str, Any]:
    rows = _proc_rows()
    pids = _process_tree(root_pid)
    fd_count = 0
    for pid in pids:
        try:
            fd_count += sum(1 for _ in (Path("/proc") / str(pid) / "fd").iterdir())
        except OSError:
            pass
    return {
        "root_alive": root_pid in rows,
        "pids": list(pids),
        "process_count": len(pids),
        "rss_kib": sum(rows[pid]["rss_kib"] for pid in pids if pid in rows),
        "threads": sum(rows[pid]["threads"] for pid in pids if pid in rows),
        "cpu_ticks": sum(rows[pid]["cpu_ticks"] for pid in pids if pid in rows),
        "fd_count": fd_count,
    }


def _matching_socket_lines(pids: tuple[int, ...]) -> dict[str, list[str]]:
    markers = tuple(f"pid={pid}," for pid in pids)
    result: dict[str, list[str]] = {"tcp": [], "udp": []}
    for kind, command in (
        ("tcp", ("ss", "-H", "-tpn")),
        ("udp", ("ss", "-H", "-upn")),
    ):
        try:
            output = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=3.0,
            ).stdout
        except (OSError, subprocess.TimeoutExpired):
            continue
        result[kind] = [
            line.strip()
            for line in output.splitlines()
            if any(marker in line for marker in markers)
        ]
    return result


def _gpu_snapshot() -> dict[str, Any]:
    query = (
        "utilization.gpu,utilization.memory,memory.used,memory.total,"
        "temperature.gpu,power.draw,encoder.stats.sessionCount,"
        "encoder.stats.averageFps,encoder.stats.averageLatency"
    )
    try:
        completed = subprocess.run(
            (
                "nvidia-smi",
                f"--query-gpu={query}",
                "--format=csv,noheader,nounits",
            ),
            check=False,
            capture_output=True,
            text=True,
            timeout=3.0,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"available": False, "error": type(exc).__name__}
    if completed.returncode != 0:
        return {"available": False, "error": completed.stderr.strip()[:500]}
    lines = completed.stdout.splitlines()
    if not lines:
        return {"available": False, "error": "nvidia-smi returned no rows"}
    values = [value.strip() for value in lines[0].split(",")]
    if len(values) != 9:
        return {"available": False, "error": "unexpected nvidia-smi field count"}
    names = (
        "gpu_util_pct",
        "memory_util_pct",
        "memory_used_mib",
        "memory_total_mib",
        "temperature_c",
        "power_w",
        "encoder_sessions",
        "encoder_average_fps",
        "encoder_average_latency_us",
    )
    parsed: dict[str, Any] = {"available": True}
    for name, value in zip(names, values):
        try:
            parsed[name] = _finite(float(value))
        except ValueError:
            parsed[name] = None
    return parsed


def _nic_snapshot() -> dict[str, dict[str, int | None]]:
    names = (
        "rx_bytes",
        "tx_bytes",
        "rx_packets",
        "tx_packets",
        "rx_errors",
        "tx_errors",
        "rx_dropped",
        "tx_dropped",
    )
    result: dict[str, dict[str, int | None]] = {}
    for interface in CAMERA_NICS:
        stats: dict[str, int | None] = {}
        root = Path("/sys/class/net") / interface / "statistics"
        for name in names:
            try:
                stats[name] = int((root / name).read_text(encoding="ascii").strip())
            except (OSError, ValueError):
                stats[name] = None
        result[interface] = stats
    return result


def _artifact_size(root: Path) -> tuple[int, int]:
    size = 0
    count = 0
    if not root.exists():
        return size, count
    for path in root.rglob("*"):
        try:
            if path.is_file():
                size += path.stat().st_size
                count += 1
        except OSError:
            continue
    return size, count


def _find_workstation_app():
    desktop = pyatspi.Registry.getDesktop(0)
    for index in range(desktop.childCount):
        app = desktop.getChildAtIndex(index)
        if (app.name or "") != "__main__.py":
            continue
        for child_index in range(app.childCount):
            child = app.getChildAtIndex(child_index)
            if child.getRoleName() == "frame" and (child.name or "") == "StereoWIRIS Workstation":
                return app
    raise RuntimeError("Workstation AT-SPI application is unavailable")


def _diagnostic_rows() -> tuple[dict[str, dict[str, str]], float]:
    started = time.monotonic()
    app = _find_workstation_app()
    frames = [
        app.getChildAtIndex(index)
        for index in range(app.childCount)
        if app.getChildAtIndex(index).getRoleName() == "frame"
        and (app.getChildAtIndex(index).name or "") == "StereoWIRIS Workstation"
    ]
    if len(frames) != 1:
        raise RuntimeError("Workstation top-level frame is unavailable")
    diagnostic_roots = [
        frames[0].getChildAtIndex(index)
        for index in range(frames[0].childCount)
        if frames[0].getChildAtIndex(index).getRoleName() == "frame"
        and (frames[0].getChildAtIndex(index).name or "") == "Diagnostics"
    ]
    if len(diagnostic_roots) != 1:
        raise RuntimeError("Workstation Diagnostics dock is unavailable")
    tables: list[Any] = []

    def visit(node) -> None:
        if node.getRoleName() == "table":
            try:
                table = node.queryTable()
            except Exception:
                table = None
            if table is not None and table.nColumns == 6:
                tables.append(table)
        for index in range(node.childCount):
            visit(node.getChildAtIndex(index))

    # Do not walk the complete multi-workspace GUI.  The Diagnostics dock owns
    # the status table and remains mounted regardless of the selected page.
    # Restricting traversal here avoids generating avoidable accessibility
    # traffic under the sustained qualification workload.
    visit(diagnostic_roots[0])
    for table in tables:
        rows: dict[str, dict[str, str]] = {}
        for row_index in range(table.nRows):
            values = [
                str(table.getAccessibleAt(row_index, column).name or "")
                for column in range(table.nColumns)
            ]
            if values[0] in DIAGNOSTIC_KEYS:
                rows[values[0]] = {
                    "state": values[1],
                    "severity": values[2],
                    "source": values[3],
                    "age": values[4],
                    "detail": values[5],
                }
        if all(key in rows for key in DIAGNOSTIC_KEYS):
            return rows, time.monotonic() - started
    raise RuntimeError("Workstation diagnostics status table is unavailable")


def qualification_failures(
    rows: Mapping[str, Mapping[str, str]],
    *,
    elapsed_s: float,
    phase: str,
    free_bytes: int,
) -> list[str]:
    failures: list[str] = []
    camera_states = [str(rows.get(key, {}).get("state", "")) for key in CAMERA_KEYS]
    if phase in {"preflight", "camera_pixhawk_baseline", "camera_pixhawk_ntrip"}:
        if any(state != "LIVE" for state in camera_states):
            failures.append(f"camera {phase} states={camera_states}")
    elif all(state != "LIVE" for state in camera_states):
        failures.append(f"both cameras unavailable states={camera_states}")

    pixhawk_states = [str(rows.get(key, {}).get("state", "")) for key in PIXHAWK_KEYS]
    allowed_pixhawk = {"QUALIFYING", "LIVE", "PARAM SYNC", "RECOVERING"}
    if any(state not in allowed_pixhawk for state in pixhawk_states):
        failures.append(f"pixhawk states={pixhawk_states}")
    live_deadline_s = MAVLINK_QUALIFICATION_S
    pixhawk_live_required = (
        phase in {"preflight", "soak", "camera_pixhawk_baseline", "camera_pixhawk_ntrip"}
        and elapsed_s >= live_deadline_s
    )
    if pixhawk_live_required and any(state != "LIVE" for state in pixhawk_states):
        deadline_label = (
            f"{int(live_deadline_s)}s"
            if live_deadline_s < 60.0
            else f"{int(live_deadline_s // 60)}m"
        )
        failures.append(
            f"pixhawk not LIVE after {deadline_label} states={pixhawk_states}"
        )

    ntrip = rows.get("gnss.ntrip", {})
    ntrip_state = str(ntrip.get("state", ""))
    ntrip_detail = str(ntrip.get("detail", ""))
    if phase == "camera_pixhawk_baseline":
        if ntrip_state != "DISCONNECTED":
            failures.append(f"baseline NTRIP must remain disconnected: {ntrip_state}")
    else:
        if ntrip_state in {"DISCONNECTED", "ERROR", "AUTH_FAILED", "STALE", "NO DATA", ""}:
            failures.append(f"ntrip state={ntrip_state}")
        if "RTCM age=--" in ntrip_detail or "connected=no" in ntrip_detail:
            failures.append("ntrip lacks current RTCM/connection evidence")

    if phase not in {"camera_pixhawk_baseline", "camera_pixhawk_ntrip"}:
        recording = rows.get("recording", {})
        recording_state = str(recording.get("state", ""))
        recording_detail = str(recording.get("detail", ""))
        if recording_state in {"IDLE", "ERROR", "FAILED", ""}:
            failures.append(f"recording state={recording_state}")
        if "storage=OK" not in recording_detail:
            failures.append("recording storage is not OK")
    if free_bytes < MIN_FREE_BYTES:
        failures.append(f"disk free below 100GiB: {free_bytes}")
    return failures


def _snapshot(
    *,
    root_pid: int,
    artifact_root: Path,
    started_ns: int,
    checkpoint_index: int,
    phase: str,
) -> dict[str, Any]:
    observed_ns = time.monotonic_ns()
    rows, gui_latency_s = _diagnostic_rows()
    process = _process_snapshot(root_pid)
    pids = tuple(int(pid) for pid in process["pids"])
    disk = shutil.disk_usage(artifact_root)
    artifact_bytes, artifact_files = _artifact_size(artifact_root)
    elapsed_s = (observed_ns - started_ns) * 1.0e-9
    failures = qualification_failures(
        rows,
        elapsed_s=elapsed_s,
        phase=phase,
        free_bytes=disk.free,
    )
    if not process["root_alive"]:
        failures.append("Workstation root process exited")
    if gui_latency_s > 2.0:
        failures.append(f"GUI traversal latency {gui_latency_s:.3f}s exceeds 2s")
    return {
        "schema": SCHEMA,
        "kind": "checkpoint",
        "phase": phase,
        "checkpoint_index": int(checkpoint_index),
        "observed_monotonic_ns": observed_ns,
        "observed_unix_ns": time.time_ns(),
        "elapsed_s": elapsed_s,
        "gui_traversal_latency_s": gui_latency_s,
        "statuses": rows,
        "process": process,
        "sockets": _matching_socket_lines(pids),
        "gpu": _gpu_snapshot(),
        "nics": _nic_snapshot(),
        "storage": {
            "root": str(artifact_root),
            "artifact_bytes": artifact_bytes,
            "artifact_files": artifact_files,
            "disk_free_bytes": disk.free,
            "disk_total_bytes": disk.total,
        },
        "failures": failures,
    }


def _append_jsonl(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )
    with path.open("a", encoding="utf-8") as handle:
        handle.write(encoded + "\n")
        handle.flush()
        os.fsync(handle.fileno())


def _print_payload(payload: Mapping[str, Any]) -> None:
    try:
        print(json.dumps(payload, ensure_ascii=False), flush=True)
    except BrokenPipeError:
        sys.stdout = open(os.devnull, "w", encoding="utf-8")


def _print_summary(payload: Mapping[str, Any], mode: str) -> None:
    if mode == "none":
        return
    if mode == "json":
        _print_payload(payload)
        return
    statuses = payload.get("statuses")
    statuses = statuses if isinstance(statuses, Mapping) else {}
    summary = {
        "kind": payload.get("kind"),
        "phase": payload.get("phase"),
        "checkpoint_index": payload.get("checkpoint_index"),
        "elapsed_s": payload.get("elapsed_s"),
        "camera": [
            (statuses.get(key) or {}).get("state")
            for key in CAMERA_KEYS
        ],
        "pixhawk": [
            (statuses.get(key) or {}).get("state")
            for key in PIXHAWK_KEYS
        ],
        "ntrip": (statuses.get("gnss.ntrip") or {}).get("state"),
        "fps": (statuses.get("fps") or {}).get("detail"),
        "gui_latency_s": payload.get("gui_traversal_latency_s"),
        "rss_kib": (payload.get("process") or {}).get("rss_kib"),
        "failures": payload.get("failures", []),
    }
    _print_payload(summary)


def run(args: argparse.Namespace) -> int:
    output = Path(args.output).expanduser().resolve()
    artifact_root = Path(args.artifact_root).expanduser().resolve()
    started_ns = time.monotonic_ns()
    start = {
        "schema": SCHEMA,
        "kind": "start",
        "phase": args.phase,
        "root_pid": int(args.pid),
        "duration_s": _finite(args.duration),
        "sample_interval_s": _finite(args.sample_interval),
        "checkpoint_interval_s": _finite(args.checkpoint_interval),
        "started_monotonic_ns": started_ns,
        "started_unix_ns": time.time_ns(),
        "credential_contract": "process_local_secret_not_recorded",
    }
    _append_jsonl(output, start)
    next_checkpoint_s = 0.0
    checkpoint_index = 0
    last_rows: dict[str, dict[str, str]] | None = None
    deadline_ns = started_ns + int(float(args.duration) * 1_000_000_000)

    while True:
        now_ns = time.monotonic_ns()
        elapsed_s = (now_ns - started_ns) * 1.0e-9
        if now_ns >= deadline_ns:
            break
        if elapsed_s + 1.0e-9 >= next_checkpoint_s:
            try:
                checkpoint = _snapshot(
                    root_pid=int(args.pid),
                    artifact_root=artifact_root,
                    started_ns=started_ns,
                    checkpoint_index=checkpoint_index,
                    phase=args.phase,
                )
            except Exception as exc:
                checkpoint = {
                    "schema": SCHEMA,
                    "kind": "checkpoint_error",
                    "phase": args.phase,
                    "checkpoint_index": checkpoint_index,
                    "elapsed_s": elapsed_s,
                    "error": f"{type(exc).__name__}: {exc}",
                }
            _append_jsonl(output, checkpoint)
            checkpoint_statuses = checkpoint.get("statuses")
            if isinstance(checkpoint_statuses, Mapping):
                last_rows = {
                    str(key): dict(value)
                    for key, value in checkpoint_statuses.items()
                    if isinstance(value, Mapping)
                }
            _print_summary(checkpoint, args.stdout)
            if checkpoint.get("failures") and bool(args.fail_fast):
                return 2
            checkpoint_index += 1
            next_checkpoint_s += float(args.checkpoint_interval)
        if not _process_tree(int(args.pid)):
            return 2
        time.sleep(min(float(args.sample_interval), max(0.0, (deadline_ns - time.monotonic_ns()) * 1e-9)))

    try:
        final_checkpoint = _snapshot(
            root_pid=int(args.pid),
            artifact_root=artifact_root,
            started_ns=started_ns,
            checkpoint_index=checkpoint_index,
            phase=args.phase,
        )
    except Exception as exc:
        final_checkpoint = {
            "schema": SCHEMA,
            "kind": "checkpoint_error",
            "phase": args.phase,
            "checkpoint_index": checkpoint_index,
            "elapsed_s": (time.monotonic_ns() - started_ns) * 1.0e-9,
            "error": f"{type(exc).__name__}: {exc}",
        }
    _append_jsonl(output, final_checkpoint)
    _print_summary(final_checkpoint, args.stdout)
    final_statuses = final_checkpoint.get("statuses")
    if isinstance(final_statuses, Mapping):
        last_rows = {
            str(key): dict(value)
            for key, value in final_statuses.items()
            if isinstance(value, Mapping)
        }
    terminal = {
        "schema": SCHEMA,
        "kind": "complete",
        "phase": args.phase,
        "elapsed_s": (time.monotonic_ns() - started_ns) * 1.0e-9,
        "last_statuses": last_rows or {},
        "completed_unix_ns": time.time_ns(),
    }
    _append_jsonl(output, terminal)
    if args.stdout == "json":
        _print_payload(terminal)
    return 2 if final_checkpoint.get("failures") else 0


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(description=__doc__)
    value.add_argument("--pid", type=int, required=True)
    value.add_argument("--duration", type=float, default=3600.0)
    value.add_argument("--sample-interval", type=float, default=5.0)
    value.add_argument("--checkpoint-interval", type=float, default=300.0)
    value.add_argument(
        "--phase",
        choices=(
            "preflight",
            "camera_pixhawk_baseline",
            "camera_pixhawk_ntrip",
            "soak",
        ),
        default="soak",
    )
    value.add_argument("--fail-fast", action="store_true")
    value.add_argument(
        "--stdout",
        choices=("summary", "json", "none"),
        default="summary",
    )
    value.add_argument("--output", required=True)
    value.add_argument(
        "--artifact-root",
        default=str(
            Path(__file__).resolve().parents[3]
            / ".local"
            / "workstation_data"
            / "measurement_data"
        ),
    )
    return value


def main() -> int:
    args = parser().parse_args()
    if args.pid <= 0:
        raise SystemExit("--pid must be positive")
    for name in ("duration", "sample_interval", "checkpoint_interval"):
        if _finite(getattr(args, name)) <= 0.0:
            raise SystemExit(f"--{name.replace('_', '-')} must be positive")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
