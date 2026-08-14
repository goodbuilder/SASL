"""Guards against drift between `standalone_gui.py`'s generated stub block and
`_CONTROLLER_DELEGATE_BINDINGS`, and confirms the block is inert at runtime.

`tools/gen_panel_delegate_stubs.py` regenerates a `TYPE_CHECKING`-guarded
block of method stubs so IDEs/type checkers can see the ~410 delegate-bound
methods on `WirisStereoDistancePanel`. If a `*_METHODS` registry changes
without re-running the generator, this test catches the mismatch.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from typing import TYPE_CHECKING

from Workstation import standalone_gui

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


def test_stub_generation_is_idempotent():
    first = _gen_module.render_stub_block()
    second = _gen_module.render_stub_block()
    assert first == second


def test_type_checking_is_false_at_runtime():
    assert TYPE_CHECKING is False


def test_delegate_bound_method_does_not_originate_from_the_stub_block():
    """The stub block and the real delegate binding attach the same names;
    this proves the *stub* contributes nothing at runtime, by checking the
    live bound method actually comes from `controller_delegation.py`.
    """

    method = standalone_gui.WirisStereoDistancePanel.__dict__["toggle_session_recording"]
    assert method.__module__ == "Workstation.controller_delegation"
