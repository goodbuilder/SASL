#!/usr/bin/env python3
"""Convert an ASCII STL mesh to binary STL without changing coordinates."""

from __future__ import annotations

import argparse
import struct
from pathlib import Path


def convert(source: Path, destination: Path) -> int:
    triangles: list[tuple[tuple[float, float, float], tuple[tuple[float, float, float], ...]]] = []
    normal: tuple[float, float, float] | None = None
    vertices: list[tuple[float, float, float]] = []

    with source.open("r", encoding="ascii") as handle:
        for line_number, line in enumerate(handle, 1):
            fields = line.split()
            if not fields:
                continue
            if fields[0] == "facet" and len(fields) == 5 and fields[1] == "normal":
                normal = tuple(float(value) for value in fields[2:5])  # type: ignore[assignment]
                vertices = []
            elif fields[0] == "vertex" and len(fields) == 4:
                vertices.append(tuple(float(value) for value in fields[1:4]))  # type: ignore[arg-type]
            elif fields[0] == "endfacet":
                if normal is None or len(vertices) != 3:
                    raise ValueError(f"invalid facet ending at line {line_number}")
                triangles.append((normal, tuple(vertices)))
                normal = None
                vertices = []

    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("wb") as handle:
        header = b"SASL WIRIS Enterprise binary STL; coordinates preserved"
        handle.write(header.ljust(80, b" "))
        handle.write(struct.pack("<I", len(triangles)))
        for normal, vertices in triangles:
            handle.write(struct.pack("<3f", *normal))
            for vertex in vertices:
                handle.write(struct.pack("<3f", *vertex))
            handle.write(struct.pack("<H", 0))
    return len(triangles)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    count = convert(args.source, args.destination)
    print(f"converted {count} triangles: {args.source} -> {args.destination}")


if __name__ == "__main__":
    main()
