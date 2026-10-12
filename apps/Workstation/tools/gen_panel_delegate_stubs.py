#!/usr/bin/env python3
"""Regenerate the external typing stub for delegate-bound panel methods.

`WirisStereoDistancePanel` (in `standalone_gui.py`) exposes registered controller
methods by binding them onto the class at import time via `setattr`
(`bind_controller_delegates`, driven by `_CONTROLLER_DELEGATE_BINDINGS`).
Static type checkers and IDEs can't see attributes attached that way, so this
script generates a `TYPE_CHECKING`-guarded block in `standalone_gui.pyi` -
inert at runtime, visible to tooling - between two sentinel comments in the
class body.

Usage:
    python3 tools/gen_panel_delegate_stubs.py          # regenerate in place
    python3 tools/gen_panel_delegate_stubs.py --check  # exit 1 if stale, don't write
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
APPS_ROOT = REPO_ROOT / "apps"
STEREO_WIRIS_ROOT = APPS_ROOT / "StereoWIRIS"
COMMON_PYTHON_ROOT = REPO_ROOT / "common" / "python"
for _path in (REPO_ROOT, APPS_ROOT, COMMON_PYTHON_ROOT, STEREO_WIRIS_ROOT):
    _text = str(_path)
    if _text not in sys.path:
        sys.path.insert(0, _text)

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QT_OPENGL", "software")
os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")

TARGET_FILE = Path(__file__).resolve().parents[1] / "application" / "standalone_gui.pyi"
BEGIN_MARKER = "    # BEGIN GENERATED PANEL DELEGATE STUBS - see tools/gen_panel_delegate_stubs.py"
END_MARKER = "    # END GENERATED PANEL DELEGATE STUBS"
_INDENT = " " * 8


def render_stub_block() -> str:
    from Workstation.application import standalone_gui
    from Workstation.panel.panel_contracts import FIELD_OWNERS, METHOD_OWNERS
    from Workstation.measurement.state import MEASUREMENT_PANEL_FIELDS
    from Workstation.recording.state import RECORDING_STATE_FIELDS

    seen: set[str] = set()
    stub_lines: list[str] = [
        f"{_INDENT}{name}: Any"
        for name in sorted(set(FIELD_OWNERS) | set(MEASUREMENT_PANEL_FIELDS) | set(RECORDING_STATE_FIELDS))
    ]
    for binding in standalone_gui._CONTROLLER_DELEGATE_BINDINGS:
        for name in binding.method_names:
            if name in seen:
                continue
            seen.add(name)
            stub_lines.append(f"{_INDENT}def {name}(self, *args: Any, **kwargs: Any) -> Any: ...")

    for name in METHOD_OWNERS:
        if name not in seen:
            seen.add(name)
            stub_lines.append(f"{_INDENT}def {name}(self, *args: Any, **kwargs: Any) -> Any: ...")

    body = "\n".join(stub_lines) if stub_lines else f"{_INDENT}pass"
    lines = [
        BEGIN_MARKER,
        "    #",
        "    # These declarations are inert at runtime (TYPE_CHECKING is False when the",
        "    # module is imported); they exist only so IDEs/type checkers can see the",
        "    # registered methods and named fields attached at the compatibility boundary,",
        "    # which are otherwise invisible to static analysis. Do not hand-edit -",
        "    # regenerate with `python3 tools/gen_panel_delegate_stubs.py` after",
        "    # changing any controller registry or panel field/method contract.",
        "    if TYPE_CHECKING:",
        body,
        END_MARKER,
    ]
    return "\n".join(lines) + "\n"


def regenerate(*, check: bool) -> bool:
    """Returns True if the file is (now, or already) up to date."""

    original_text = TARGET_FILE.read_text()
    begin_index = original_text.index(BEGIN_MARKER)
    end_index = original_text.index(END_MARKER, begin_index) + len(END_MARKER)

    new_block = render_stub_block().rstrip("\n")
    new_text = original_text[:begin_index] + new_block + original_text[end_index:]

    if new_text == original_text:
        return True
    if check:
        return False
    TARGET_FILE.write_text(new_text)
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="exit nonzero if regenerating would change the committed file, without writing",
    )
    args = parser.parse_args()

    up_to_date = regenerate(check=args.check)
    if args.check and not up_to_date:
        print(
            f"{TARGET_FILE} delegate stub block is stale; "
            "run `python3 tools/gen_panel_delegate_stubs.py` to regenerate.",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
