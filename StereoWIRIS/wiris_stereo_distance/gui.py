"""Compatibility alias for the Legacy StereoWIRIS GUI.

The implementation lives in :mod:`wiris_stereo_distance.legacy_gui` so the
Workstation can keep its own copied owner in :mod:`Workstation.standalone_gui`.
Replacing this module object with
the implementation module preserves existing imports and, importantly,
keeps monkeypatches on ``wiris_stereo_distance.gui`` applied to the Legacy
owner during tests and downstream integrations.
"""

import sys as _sys

from . import legacy_gui as _legacy_gui

_sys.modules[__name__] = _legacy_gui
