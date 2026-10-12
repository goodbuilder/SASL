"""Command declarations preserve the independent pre-extraction catalog."""

from __future__ import annotations

import ast
from dataclasses import asdict, replace
import json
from pathlib import Path

import pytest

from Workstation.main_window import WorkstationMainWindow
from Workstation.command_bindings.camera import register_camera


class Registry:
    def __init__(self):
        self.specs = {}

    def register(self, spec):
        assert spec.command_id not in self.specs, "duplicate command binding"
        self.specs[spec.command_id] = spec

    def set_deferred_completion(self, command_id):
        assert command_id in self.specs
        self.specs[command_id] = replace(self.specs[command_id], deferred_completion=True)


class DeclaredActions:
    def __init__(self, names):
        self.commands = Registry()
        for name in names:
            setattr(self, name, lambda *args, _name=name, **kwargs: dict(handler=_name, args=list(args), kwargs=kwargs))


def test_command_metadata_order_completion_and_payload_match_original_catalog():
    fixture = json.loads((Path(__file__).parent / "fixtures/main_window_command_catalog.json").read_text())
    expected = fixture["commands"]
    actions = DeclaredActions({row["handler_result"]["handler"] for row in expected})
    WorkstationMainWindow._build_commands(actions)
    actual = []
    for spec in actions.commands.specs.values():
        row = asdict(spec)
        del row["handler"]
        row["handler_result"] = spec.handler()
        actual.append(row)
    assert actual == expected


def test_missing_required_group_action_fails_at_composition():
    with pytest.raises(TypeError, match="camera_auto_connect"):
        register_camera(Registry())


def _forbidden_imports(source):
    result = []
    for node in ast.walk(ast.parse(source)):
        modules = (
            [node.module]
            if isinstance(node, ast.ImportFrom)
            else [alias.name for alias in node.names]
            if isinstance(node, ast.Import)
            else []
        )
        result.extend(
            module
            for module in modules
            if module
            and (
                module.startswith(
                    ("Workstation.main_window", "Workstation.standalone_backend", "Workstation.workspace_views")
                )
            )
        )
    return result


def test_command_declarations_cannot_import_window_backend_or_screens():
    package = Path(__file__).resolve().parents[1] / "command_bindings"
    assert _forbidden_imports("from Workstation.main_window import WorkstationMainWindow")
    assert _forbidden_imports("import Workstation.standalone_backend")
    for path in package.glob("*.py"):
        assert not _forbidden_imports(path.read_text()), path.name
