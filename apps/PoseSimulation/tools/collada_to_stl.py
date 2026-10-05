#!/usr/bin/env python3
"""Convert the triangle geometry in a COLLADA mesh to one binary STL.

The PoseSimulation RViz view intentionally uses STL ``MESH_RESOURCE`` markers.
This small dependency-free converter keeps the existing Gazebo COLLADA drone
asset as the source of truth while producing a portable RViz mesh asset.
"""

from __future__ import annotations

import argparse
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path


NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def _source_values(source: ET.Element) -> list[float]:
    array = source.find("c:float_array", NS)
    if array is None or not array.text:
        raise ValueError(f"source {source.get('id', '<unknown>')} has no float array")
    return [float(value) for value in array.text.split()]


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _normal(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    c: tuple[float, float, float],
) -> tuple[float, float, float]:
    normal = _cross(
        (b[0] - a[0], b[1] - a[1], b[2] - a[2]),
        (c[0] - a[0], c[1] - a[1], c[2] - a[2]),
    )
    length = math.sqrt(sum(component * component for component in normal))
    if length <= 1.0e-12:
        return (0.0, 0.0, 0.0)
    return tuple(component / length for component in normal)  # type: ignore[return-value]


def _triangles(root: ET.Element):
    for geometry in root.findall(".//c:library_geometries/c:geometry", NS):
        mesh = geometry.find("c:mesh", NS)
        if mesh is None:
            continue
        sources = {
            source.get("id"): _source_values(source)
            for source in mesh.findall("c:source", NS)
            if source.get("id")
        }
        vertices = mesh.find("c:vertices", NS)
        if vertices is None:
            continue
        position_source = next(
            (
                item.get("source", "").lstrip("#")
                for item in vertices.findall("c:input", NS)
                if item.get("semantic") == "POSITION"
            ),
            None,
        )
        if position_source is None or position_source not in sources:
            continue
        positions = sources[position_source]
        for triangles in mesh.findall("c:triangles", NS):
            inputs = triangles.findall("c:input", NS)
            vertex_input = next(
                (item for item in inputs if item.get("semantic") == "VERTEX"),
                None,
            )
            if vertex_input is None:
                continue
            stride = 1 + max(int(item.get("offset", "0")) for item in inputs)
            vertex_offset = int(vertex_input.get("offset", "0"))
            index_values = [
                int(value)
                for value in triangles.findtext("c:p", default="", namespaces=NS).split()
            ]
            for start in range(0, len(index_values), stride * 3):
                vertices_for_triangle = []
                for corner in range(3):
                    index = index_values[start + corner * stride + vertex_offset]
                    base = index * 3
                    vertices_for_triangle.append(tuple(positions[base : base + 3]))
                yield vertices_for_triangle[0], vertices_for_triangle[1], vertices_for_triangle[2]


def convert(source: Path, destination: Path) -> int:
    triangles = list(_triangles(ET.parse(source).getroot()))
    if not triangles:
        raise ValueError(f"no triangles found in {source}")
    header = b"PoseSimulation x500_15m STL converted from NXP-HGD-CF.dae"[:80].ljust(80, b" ")
    with destination.open("wb") as output:
        output.write(header)
        output.write(struct.pack("<I", len(triangles)))
        for first, second, third in triangles:
            output.write(struct.pack("<3f", *_normal(first, second, third)))
            for vertex in (first, second, third):
                output.write(struct.pack("<3f", *vertex))
            output.write(b"\0\0")
    return len(triangles)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    count = convert(args.source, args.destination)
    print(f"wrote {args.destination} ({count} triangles)")


if __name__ == "__main__":
    main()
