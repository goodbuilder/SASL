#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Sequence


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Concatenate candidate_sample_diagnostics.csv files across completed "
            "ROI/fusion reports. Missing inputs are reported as skipped, not evidence."
        )
    )
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def sample_path(path: Path) -> Path | None:
    if path.is_file() and path.name == "candidate_sample_diagnostics.csv":
        return path
    candidate = path / "candidate_sample_diagnostics.csv"
    if candidate.exists():
        return candidate
    return None


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    args = parse_args()
    output_rows: list[dict[str, object]] = []
    skipped: list[dict[str, object]] = []
    loaded: list[str] = []
    for item in args.inputs:
        path = sample_path(item)
        if path is None:
            skipped.append({"input": str(item), "reason": "missing_candidate_sample_diagnostics"})
            continue
        rows = read_rows(path)
        if not rows:
            skipped.append({"input": str(item), "reason": "empty_candidate_sample_diagnostics"})
            continue
        report_id = path.parent.name
        loaded.append(str(path))
        for row in rows:
            combined = dict(row)
            combined["source_diagnostics"] = str(path)
            combined["report_id"] = report_id
            output_rows.append(combined)

    write_csv(args.output_dir / "candidate_sample_diagnostics.csv", output_rows)
    write_csv(args.output_dir / "skipped_inputs.csv", skipped)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "metadata.json").write_text(
        json.dumps(
            {
                "loaded_input_count": len(loaded),
                "skipped_input_count": len(skipped),
                "sample_count": len(output_rows),
                "loaded_inputs": loaded,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"Loaded diagnostics: {len(loaded)}")
    print(f"Skipped inputs: {len(skipped)}")
    print(f"Samples: {len(output_rows)}")
    print(f"Output: {args.output_dir / 'candidate_sample_diagnostics.csv'}")
    return 0 if output_rows else 1


if __name__ == "__main__":
    raise SystemExit(main())
