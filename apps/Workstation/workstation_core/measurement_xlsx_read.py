"""Minimal stdlib XLSX reader for Measurement workbooks (tests, reproduction).

Reads shared strings, inline strings, numbers and booleans from every sheet
and returns each sheet as a list of rows (lists). Cells are returned as
``str``, ``float``, ``int`` (when the stored number is integral), ``bool`` or
``None``. No formulas are evaluated; the workbooks never contain any.
"""

from __future__ import annotations

import re
import zipfile
from pathlib import Path
from xml.etree import ElementTree

_NS = {
    "main": "http://schemas.openxmlformats.org/spreadsheetml/2006/main",
    "rel": "http://schemas.openxmlformats.org/officeDocument/2006/relationships",
    "pkg": "http://schemas.openxmlformats.org/package/2006/relationships",
}
_CELL_REF = re.compile(r"([A-Z]+)(\d+)")


def _column_index(reference: str) -> int:
    match = _CELL_REF.match(reference)
    if match is None:
        raise ValueError(f"invalid cell reference: {reference}")
    letters = match.group(1)
    index = 0
    for character in letters:
        index = index * 26 + (ord(character) - ord("A") + 1)
    return index - 1


def _rich_text(element: ElementTree.Element) -> str:
    parts: list[str] = []
    for text in element.iter(f"{{{_NS['main']}}}t"):
        parts.append(text.text or "")
    return "".join(parts)


def _number(text: str) -> float | int:
    value = float(text)
    if value.is_integer() and "e" not in text.lower() and "." not in text:
        return int(text)
    return value


def read_workbook(path: Path | str) -> dict[str, list[list[object]]]:
    """Return ``{sheet_name: rows}`` for every worksheet in the workbook."""

    sheets: dict[str, list[list[object]]] = {}
    with zipfile.ZipFile(Path(path)) as archive:
        shared: list[str] = []
        if "xl/sharedStrings.xml" in archive.namelist():
            root = ElementTree.fromstring(archive.read("xl/sharedStrings.xml"))
            shared = [_rich_text(item) for item in root.findall("main:si", _NS)]
        workbook = ElementTree.fromstring(archive.read("xl/workbook.xml"))
        relationships = ElementTree.fromstring(archive.read("xl/_rels/workbook.xml.rels"))
        targets = {
            item.attrib["Id"]: item.attrib["Target"]
            for item in relationships.findall("pkg:Relationship", _NS)
        }
        for sheet in workbook.find("main:sheets", _NS).findall("main:sheet", _NS):
            name = sheet.attrib["name"]
            relationship_id = sheet.attrib[f"{{{_NS['rel']}}}id"]
            target = targets[relationship_id]
            member = target if target.startswith("xl/") else f"xl/{target.lstrip('/')}"
            sheets[name] = _read_sheet(archive.read(member), shared)
    return sheets


def _read_sheet(payload: bytes, shared: list[str]) -> list[list[object]]:
    root = ElementTree.fromstring(payload)
    data = root.find("main:sheetData", _NS)
    rows: list[list[object]] = []
    if data is None:
        return rows
    for row in data.findall("main:row", _NS):
        row_index = int(row.attrib.get("r", len(rows) + 1)) - 1
        while len(rows) < row_index:
            rows.append([])
        cells: list[object] = []
        for cell in row.findall("main:c", _NS):
            column = _column_index(cell.attrib.get("r", ""))
            while len(cells) < column:
                cells.append(None)
            cell_type = cell.attrib.get("t", "")
            value_element = cell.find("main:v", _NS)
            value: object
            if cell_type == "s":
                value = shared[int(value_element.text)] if value_element is not None else ""
            elif cell_type == "inlineStr":
                inline = cell.find("main:is", _NS)
                value = "" if inline is None else _rich_text(inline)
            elif cell_type == "b":
                value = bool(int(value_element.text)) if value_element is not None else None
            elif cell_type == "str":
                value = value_element.text if value_element is not None else ""
            else:
                value = None if value_element is None or value_element.text is None else _number(value_element.text)
            cells.append(value)
        rows.append(cells)
    return rows


def table_rows(rows: list[list[object]], header_row: int = 0) -> list[dict[str, object]]:
    """Turn a header + data grid into dicts until the first fully blank row."""

    if header_row >= len(rows):
        return []
    header = [str(value) if value is not None else "" for value in rows[header_row]]
    result: list[dict[str, object]] = []
    for row in rows[header_row + 1 :]:
        if not row or all(value in (None, "") for value in row):
            break
        padded = list(row) + [None] * (len(header) - len(row))
        result.append({name: padded[index] for index, name in enumerate(header) if name})
    return result


def find_marker_row(rows: list[list[object]], marker: str) -> int:
    for index, row in enumerate(rows):
        if row and row[0] == marker:
            return index
    raise KeyError(marker)


def table_rows_with_parts(sheets: dict[str, list[list[object]]], name: str) -> list[dict[str, object]]:
    """Read a machine table including numbered continuation worksheets."""
    return [row for sheet, grid in sheets.items() if sheet == name or re.fullmatch(re.escape(name) + r"_\d+", sheet)
            for row in table_rows(grid)]


__all__ = ["find_marker_row", "read_workbook", "table_rows"]
