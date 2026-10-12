"""Selection sidecar I/O, using the existing portable-path schema and core parser.

Each operation receives its root and existing selection together; it never reads
live widgets, solves geometry, or treats diagnostic errors as runtime truth.
"""
from __future__ import annotations
import json
from collections.abc import Mapping
from pathlib import Path
from Workstation.workstation_core.calibration_selection import (
    CALIBRATION_SELECTION_SCHEMA, portable_calibration_selection_paths,
    resolve_calibration_selection_paths,
)


def write_outlier_selection(root, selection, diagnostic_rows, payload, *, restore):
    if not isinstance(payload, Mapping):
        return False, ""
    path_text = str(payload.get("path") or "").strip()
    side = str(payload.get("side") or "").strip().lower()
    if side not in {"left", "right"} or not path_text:
        return False, ""
    root = Path(root).expanduser().resolve(strict=False)
    path = Path(path_text).expanduser().resolve()
    excluded_left_paths, excluded_left_errors = resolve_calibration_selection_paths(
        selection.get("excluded_left_paths"),
        root,
        stored_root=selection.get("root"),
    )
    excluded_right_paths, excluded_right_errors = resolve_calibration_selection_paths(
        selection.get("excluded_right_paths"),
        root,
        stored_root=selection.get("root"),
    )
    selected_left_paths, selected_left_errors = resolve_calibration_selection_paths(
        selection.get("left_paths"),
        root,
        stored_root=selection.get("root"),
    )
    selected_right_paths, selected_right_errors = resolve_calibration_selection_paths(
        selection.get("right_paths"),
        root,
        stored_root=selection.get("root"),
    )
    path_errors = (
        *excluded_left_errors,
        *excluded_right_errors,
        *selected_left_errors,
        *selected_right_errors,
    )
    if path_errors:
        return False, "기존 선택/제외 경로를 복구할 수 없어 변경하지 않았습니다: " + ", ".join(path_errors)
    excluded_left = {str(item) for item in excluded_left_paths}
    excluded_right = {str(item) for item in excluded_right_paths}
    excluded_pairs = {
        str(item) for item in selection.get("excluded_pair_keys") or ()
    }
    capture_key = str(payload.get("capture_key") or "").strip()
    pair_left_paths = {str(path)} if side == "left" else set()
    pair_right_paths = {str(path)} if side == "right" else set()
    if capture_key:
        for row in diagnostic_rows:
            if (
                str(row.get("capture_key") or "") == capture_key
                and str(row.get("path") or "")
                and str(row.get("path")) != str(path)
            ):
                counterpart = str(Path(str(row["path"])).expanduser().resolve())
                if str(row.get("side") or "").lower() == "left":
                    pair_left_paths.add(counterpart)
                elif str(row.get("side") or "").lower() == "right":
                    pair_right_paths.add(counterpart)
    scope = str(payload.get("scope") or "image")
    if restore:
        if scope == "pair":
            excluded_pairs.discard(capture_key)
            excluded_left.difference_update(pair_left_paths)
            excluded_right.difference_update(pair_right_paths)
        elif side == "left":
            excluded_left.discard(str(path))
        else:
            excluded_right.discard(str(path))
        action_text = "복원"
    else:
        if scope == "pair":
            excluded_pairs.add(capture_key)
            excluded_left.update(pair_left_paths)
            excluded_right.update(pair_right_paths)
            action_text = "stereo pair 제외"
        elif side == "left":
            excluded_left.add(str(path))
            action_text = "Left 사진 제외"
        else:
            excluded_right.add(str(path))
            action_text = "Right 사진 제외"
    data = {
        "schema": CALIBRATION_SELECTION_SCHEMA,
        "root": ".",
        "manual_mode": bool(selection.get("manual_mode")),
        "left_paths": portable_calibration_selection_paths(
            root, selected_left_paths
        ),
        "right_paths": portable_calibration_selection_paths(
            root, selected_right_paths
        ),
        "included_keys": list(selection.get("included_keys") or ()),
        "excluded_left_paths": portable_calibration_selection_paths(
            root, [Path(item) for item in sorted(excluded_left)]
        ),
        "excluded_right_paths": portable_calibration_selection_paths(
            root, [Path(item) for item in sorted(excluded_right)]
        ),
        "excluded_pair_keys": sorted(key for key in excluded_pairs if key),
    }
    try:
        sidecar = root / "calibration_pair_selection.json"
        sidecar.parent.mkdir(parents=True, exist_ok=True)
        sidecar.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
    except OSError as exc:
        return False, f"제외 상태 저장 실패: {exc}"
    return True, (
        f"{action_text}: {Path(path_text).name} | 원본 파일은 삭제하지 않았습니다. 다시 계산하면 반영됩니다."
    )
