"""Guard against drift between the generated stubs and delegate bindings.

`tools/gen_panel_delegate_stubs.py` regenerates a `TYPE_CHECKING`-guarded
block of method stubs so IDEs/type checkers can see the ~410 delegate-bound
methods on `WirisStereoDistancePanel`. If a `*_METHODS` registry changes
without re-running the generator, this test catches the mismatch.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

_GENERATOR_PATH = Path(__file__).resolve().parents[1] / "tools" / "gen_panel_delegate_stubs.py"
_spec = importlib.util.spec_from_file_location("gen_panel_delegate_stubs", _GENERATOR_PATH)
_gen_module = importlib.util.module_from_spec(_spec)
assert _spec.loader is not None
_spec.loader.exec_module(_gen_module)


def test_stub_block_matches_registered_delegate_bindings():
    assert _gen_module.regenerate(check=True), (
        "standalone_gui.py's generated delegate stub block is stale; "
        "run `python3 tools/gen_panel_delegate_stubs.py` to regenerate it."
    )
