"""Finite legacy aliases to camera-owned UI components."""

from types import SimpleNamespace
from .connection_view import CONNECTION_VIEW_FIELDS
from .candidates import CANDIDATE_VIEW_FIELDS
from .command_view import COMMAND_VIEW_FIELDS


class CameraViews(SimpleNamespace):
    def __init__(self):
        super().__init__(connection=None, candidates=None, commands=None)


def _view_field(owner, name):
    return property(
        lambda views: getattr(getattr(views, owner), name),
        lambda views, value: setattr(getattr(views, owner), name, value),
    )


for _owner, _names in (
    ("connection", CONNECTION_VIEW_FIELDS),
    ("candidates", CANDIDATE_VIEW_FIELDS),
    ("commands", COMMAND_VIEW_FIELDS),
):
    for _name in _names:
        setattr(CameraViews, _name, _view_field(_owner, _name))
del _owner, _names, _name
