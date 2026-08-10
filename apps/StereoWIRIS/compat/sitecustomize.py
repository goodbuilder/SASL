"""Repair Matplotlib toolkit discovery for the application launcher."""

from __future__ import annotations

import site
from pathlib import Path


try:
    import mpl_toolkits

    user_toolkits = Path(site.getusersitepackages()) / "mpl_toolkits"
    toolkit_path = str(user_toolkits)
    if user_toolkits.is_dir() and toolkit_path not in mpl_toolkits.__path__:
        mpl_toolkits.__path__.insert(0, toolkit_path)
except Exception:
    # This compatibility hook must never prevent the application from starting.
    pass
