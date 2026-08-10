from __future__ import annotations

import colorsys
import re
from pathlib import Path

from workstation_core import theme


HEX_COLOR_RE = re.compile(r"#[0-9a-fA-F]{6}")


def _is_neutral_or_red(value: str) -> bool:
    red, green, blue = (int(value[index : index + 2], 16) for index in (1, 3, 5))
    hue, saturation, _value = colorsys.rgb_to_hsv(red / 255, green / 255, blue / 255)
    return saturation < 0.04 or hue < 0.04 or hue > 0.96


def _hex_colors_under(*roots: Path) -> set[str]:
    return {
        match.group(0).lower()
        for root in roots
        for source_path in root.rglob("*.py")
        for match in HEX_COLOR_RE.finditer(source_path.read_text(encoding="utf-8"))
    }


def test_workstation_theme_palette_is_neutral_or_red() -> None:
    palette = {
        name: value
        for name, value in vars(theme).items()
        if name.isupper() and isinstance(value, str) and value.startswith("#")
    }

    assert palette
    assert all(_is_neutral_or_red(value) for value in palette.values()), palette


def test_workstation_python_color_literals_are_neutral_or_red() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    colors = _hex_colors_under(repo_root / "workstation", repo_root / "workstation_core")

    assert colors
    assert all(_is_neutral_or_red(value) for value in colors), sorted(colors)
