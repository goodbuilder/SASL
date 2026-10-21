"""Load the checked-in Qt Designer forms without generated or local state.

Forms own widget geometry and static presentation. Their caller owns runtime
values, commands, models and custom views. Paths resolve from this package,
independently of the working directory or the application data directory.
"""

from pathlib import Path

from PyQt5 import uic


FORMS_ROOT = Path(__file__).resolve().parent / "forms"


def load_form(name: str, target):
    """Populate an existing QWidget/QMainWindow using a named bundled form."""
    if Path(name).name != name or not name.endswith(".ui"):
        raise ValueError(f"Expected a bundled .ui filename, got {name!r}")
    return uic.loadUi(str(FORMS_ROOT / name), target)
