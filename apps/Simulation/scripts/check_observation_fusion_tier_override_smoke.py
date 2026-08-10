#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import check_stereodistance_observation_fusion_node_smoke as node_smoke  # noqa: E402


def load_manifest(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"manifest must be a JSON object: {path}")
    return payload


def tier_names(manifest: Mapping[str, object]) -> list[str]:
    tiers = manifest.get("tiers", [])
    if not isinstance(tiers, list):
        return []
    names: list[str] = []
    for tier in tiers:
        if isinstance(tier, Mapping):
            name = str(tier.get("tier", ""))
            if name:
                names.append(name)
    return names


def smoke_rows(*, manifest_json: Path, profile_json: Path | None = None) -> list[dict[str, object]]:
    manifest = load_manifest(manifest_json)
    default_tier = str(manifest.get("default_recommendation", ""))
    rows: list[dict[str, object]] = []
    for tier in tier_names(manifest):
        payload = node_smoke.run_smoke(
            manifest_json=manifest_json,
            profile_json=profile_json,
            tier=tier,
        )
        rows.append(
            {
                "tier": tier,
                "is_default": tier == default_tier,
                "status": payload.get("status", ""),
                "adapter_class": payload.get("adapter_class", ""),
                "observation_count": payload.get("observation_count", ""),
                "observation_fusion_status": payload.get("observation_fusion_status", ""),
                "estimated_range_m": payload.get("estimated_range_m", ""),
                "corrected_range_m": payload.get("corrected_range_m", ""),
                "runtime_truth_used": payload.get("runtime_truth_used", False),
                "failure_count": payload.get("failure_count", 0),
                "failures": ";".join(str(failure) for failure in payload.get("failures", [])),
                "coordinate_contract": payload.get("coordinate_contract", ""),
            }
        )
    return rows


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def summary_payload(*, manifest_json: Path, profile_json: Path | None, rows: Sequence[Mapping[str, object]]) -> dict[str, object]:
    failure_rows = [row for row in rows if row.get("status") != "PASS" or int(row.get("failure_count", 0) or 0) != 0]
    runtime_truth_used = any(str(row.get("runtime_truth_used", "")).lower() == "true" for row in rows)
    default_rows = [row for row in rows if row.get("is_default") is True]
    return {
        "mode": "observation_fusion_tier_override_smoke",
        "status": "PASS" if not failure_rows and rows and not runtime_truth_used and len(default_rows) == 1 else "FAIL",
        "manifest_json": str(manifest_json),
        "profile_json": "" if profile_json is None else str(profile_json),
        "tier_count": len(rows),
        "pass_count": sum(1 for row in rows if row.get("status") == "PASS" and int(row.get("failure_count", 0) or 0) == 0),
        "default_tier_count": len(default_rows),
        "runtime_truth_used": runtime_truth_used,
        "coordinate_contracts_explicit": True,
        "failure_count": len(failure_rows) + (0 if rows else 1) + (0 if len(default_rows) == 1 else 1),
        "failures": [
            f"tier:{row.get('tier', '')}:{row.get('failures', '') or row.get('status', '')}"
            for row in failure_rows
        ]
        + ([] if rows else ["tiers:empty"])
        + ([] if len(default_rows) == 1 else ["default_tier_count:not_one"]),
        "rows": list(rows),
    }


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Observation fusion tier override smoke",
        f"status: {payload.get('status', '')}",
        f"manifest_json: {payload.get('manifest_json', '')}",
        f"profile_json: {payload.get('profile_json', '')}",
        f"tier_count: {payload.get('tier_count', '')}",
        f"pass_count: {payload.get('pass_count', '')}",
        f"default_tier_count: {payload.get('default_tier_count', '')}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', False)}",
        "Coordinate contract: runtime image u/v/disparity -> stereo-center RDF; public output remains FRD+range.",
        "",
        "Rows:",
    ]
    for row in payload.get("rows", []):
        if not isinstance(row, Mapping):
            continue
        lines.append(
            "- {tier}: status={status}, default={is_default}, adapter={adapter}, observations={count}, fusion={fusion}".format(
                tier=row.get("tier", ""),
                status=row.get("status", ""),
                is_default=row.get("is_default", ""),
                adapter=row.get("adapter_class", ""),
                count=row.get("observation_count", ""),
                fusion=row.get("observation_fusion_status", ""),
            )
        )
    failures = payload.get("failures", [])
    if isinstance(failures, list) and failures:
        lines.append("")
        lines.append("Failures:")
        lines.extend(f"- {failure}" for failure in failures)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Smoke-test every observation-fusion manifest tier override.")
    parser.add_argument("--manifest-json", type=Path, required=True)
    parser.add_argument("--profile-json", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args(argv)


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    rows = smoke_rows(manifest_json=args.manifest_json, profile_json=args.profile_json)
    payload = summary_payload(manifest_json=args.manifest_json, profile_json=args.profile_json, rows=rows)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "observation_fusion_tier_override_smoke.csv", rows)
    (args.output_dir / "observation_fusion_tier_override_smoke.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    write_text(args.output_dir / "observation_fusion_tier_override_smoke.txt", payload)
    return payload


def main() -> None:
    payload = run()
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
