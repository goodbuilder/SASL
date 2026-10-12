"""Workspace owners are independently importable and preserve the public boundary."""

from __future__ import annotations

import ast
import importlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest


APP_ROOT = Path(__file__).resolve().parents[1]
VIEW_ROOT = APP_ROOT / "workspace_views"
CLASS_OWNERS = {
    "AdvancedWorkspace": "advanced",
    "AlignmentWorkspace": "alignment",
    "BaseGnssWorkspace": "base_gnss",
    "CalibrationErrorPlot": "calibration_parts.widgets",
    "CalibrationWorkspace": "calibration",
    "CameraPreview": "camera_preview",
    "CameraPreviewPair": "camera_preview",
    "CameraWorkspace": "camera",
    "DiagnosticsContent": "diagnostics",
    "ExperimentWorkspace": "experiment",
    "MeasurementWorkspace": "measurement",
    "OpticalPosePane": "optical_pose",
    "OverviewWorkspace": "overview",
    "PixhawkWorkspace": "pixhawk",
    "RecordingWorkspace": "recording",
    "SessionWorkspace": "session",
    "TelemetryWorkspace": "telemetry",
    "TimeWorkspace": "time_sync",
}
VIEW_DEPENDENCIES = {
    # Explicit module imports only. Package initialization is accounted for
    # separately in the runtime closure, so re-exports do not fabricate cycles.
    "advanced": {"px4_widgets", "shared"},
    "alignment": {"optical_pose"},
    "base_gnss": {"shared"},
    "calibration": {
        "shared", "calibration_parts.widgets", "calibration_parts.commands", "calibration_parts.compatibility",
        "calibration_parts.capture", "calibration_parts.preview", "calibration_parts.inputs",
        "calibration_parts.quality", "calibration_parts.validation", "calibration_parts.contracts",
    },
    "calibration_parts": set(),
    "calibration_parts.capture": {"shared", "calibration_parts.widgets"},
    "calibration_parts.commands": set(),
    "calibration_parts.compatibility": set(),
    "calibration_parts.contracts": set(),
    "calibration_parts.inputs": {"calibration_parts.photo_evidence", "calibration_parts.inventory"},
    "calibration_parts.inventory": set(),
    "calibration_parts.photo_evidence": set(),
    "calibration_parts.preview": {"camera_preview"},
    "calibration_parts.quality": {"shared", "calibration_parts.selection_files", "calibration_parts.widgets"},
    "calibration_parts.selection_files": set(),
    "calibration_parts.validation": set(),
    "calibration_parts.widgets": set(),
    "camera": {"camera_preview", "network_helpers", "shared"},
    "camera_preview": set(),
    "diagnostics": {"shared"},
    "experiment": {"shared"},
    "measurement": {"camera_preview", "shared"},
    "network_helpers": set(),
    "optical_pose": {
        "optical_pose_parts.annotations", "optical_pose_parts.suggestions", "optical_pose_parts.candidates",
        "optical_pose_parts.methods", "optical_pose_parts.observations", "optical_pose_parts.live_reference",
        "optical_pose_parts.layouts",
    },
    "optical_pose_parts": set(),
    "optical_pose_parts.annotations": {"optical_pose_parts.layouts"},
    "optical_pose_parts.candidates": set(),
    "optical_pose_parts.layouts": set(),
    "optical_pose_parts.live_reference": set(),
    "optical_pose_parts.methods": {"optical_pose_parts.layouts"},
    "optical_pose_parts.observations": set(),
    "optical_pose_parts.suggestions": {"optical_pose_parts.annotations"},
    "overview": {"shared"},
    "pixhawk": {
        "shared", "pixhawk_parts.connections", "pixhawk_parts.attitude", "pixhawk_parts.gnss",
        "pixhawk_parts.magnetometers", "pixhawk_parts.geometry", "pixhawk_parts.contracts", "pixhawk_parts.formatting",
    },
    "pixhawk_parts": set(),
    "pixhawk_parts.attitude": {"shared"},
    "pixhawk_parts.connections": {"shared", "network_helpers", "pixhawk_parts.contracts"},
    "pixhawk_parts.contracts": set(),
    "pixhawk_parts.formatting": set(),
    "pixhawk_parts.geometry": {"shared", "pixhawk_parts.formatting"},
    "pixhawk_parts.gnss": {"shared", "pixhawk_parts.formatting"},
    "pixhawk_parts.magnetometers": {"shared", "px4_widgets", "pixhawk_parts.contracts", "pixhawk_parts.formatting"},
    "px4_widgets": set(),
    "recording": {"session", "shared", "target_reference"},
    "session": set(),
    "shared": set(),
    "telemetry": {"shared"},
    "target_reference": set(),
    "time_sync": {"shared"},
}


def test_compatibility_names_identify_the_actual_feature_classes():
    from Workstation import workspaces

    for name, owner in CLASS_OWNERS.items():
        module = importlib.import_module(f"Workstation.workspace_views.{owner}")
        actual = getattr(workspaces, name)
        assert actual is getattr(module, name)
        assert actual.__module__ == module.__name__
    assert workspaces.DataWorkspace is workspaces.RecordingWorkspace
    assert workspaces.SessionsWorkspace is workspaces.RecordingWorkspace


def test_facade_contains_only_explicit_imports_and_export_declarations():
    tree = ast.parse((APP_ROOT / "workspaces.py").read_text())
    for node in tree.body:
        if isinstance(node, ast.Expr):
            assert isinstance(node.value, ast.Constant) and isinstance(node.value.value, str)
        elif isinstance(node, ast.ImportFrom):
            assert node.module.startswith("workspace_views.")
            assert all(alias.name != "*" for alias in node.names)
        else:
            assert isinstance(node, ast.Assign)
            assert [target.id for target in node.targets] == ["__all__"]
            assert isinstance(node.value, ast.List)


def _view_dependencies(source, label="fixture", *, module="fixture", packages=(), modules=None):
    """Resolve explicit imports to full view-module suffixes.

    Parent package initialization belongs to runtime import closure, not an
    invented child->parent source edge: __init__ may re-export that same child.
    """
    dependencies = set()
    namespace = "Workstation.workspace_views"
    prefix = namespace + "."
    known = set(VIEW_DEPENDENCIES if modules is None else modules)
    package = namespace + (("." + module) if module in packages else
                           ("." + module.rsplit(".", 1)[0] if "." in module else ""))
    tree = ast.parse(source)
    importlib_names = {"importlib"}
    dynamic_loaders = {"eval", "exec", "__import__"}
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name == "importlib":
                    importlib_names.add(alias.asname or alias.name)
        elif isinstance(node, ast.ImportFrom) and node.module == "importlib":
            dynamic_loaders.update(alias.asname or alias.name for alias in node.names if alias.name == "import_module")

    def assert_allowed(name):
        parts = name.split(".")
        assert "workspaces" not in parts, label
        assert not {"standalone_backend", "standalone_gui", "panel_application"} & set(parts), label

    def record(name):
        if name.startswith(prefix):
            dependencies.add(name[len(prefix):])

    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            target = node.module or ""
            assert all(alias.name != "*" for alias in node.names), label
            if node.level:
                target = importlib.util.resolve_name("." * node.level + target, package)
            assert_allowed(target)
            for alias in node.names:
                assert_allowed(alias.name)
                child = f"{target}.{alias.name}"
                # Importing a named submodule differs from importing a value
                # exported by a module/package. Preserve the exact owner edge.
                if child.startswith(prefix) and child[len(prefix):] in known:
                    record(child)
                else:
                    record(target)
        elif isinstance(node, ast.Import):
            for alias in node.names:
                assert_allowed(alias.name)
                record(alias.name)
        elif isinstance(node, ast.Call):
            if isinstance(node.func, ast.Name):
                assert node.func.id not in dynamic_loaders, label
            elif isinstance(node.func, ast.Attribute) and node.func.attr == "import_module":
                assert not (isinstance(node.func.value, ast.Name) and node.func.value.id in importlib_names), label
            elif isinstance(node.func, ast.Call) and isinstance(node.func.func, ast.Name) and node.func.func.id == "getattr":
                arguments = node.func.args
                if len(arguments) >= 2 and isinstance(arguments[0], ast.Name) and arguments[0].id in importlib_names:
                    assert not (isinstance(arguments[1], ast.Constant) and arguments[1].value == "import_module"), label
    return dependencies


def _view_modules(root):
    modules, packages = {}, set()
    for path in root.rglob("*.py"):
        relative = path.relative_to(root)
        if path.name == "__init__.py":
            if relative.parent == Path("."):
                continue
            module = ".".join(relative.parent.parts)
            packages.add(module)
        else:
            module = ".".join(relative.with_suffix("").parts)
        modules[module] = path
    return modules, packages


def _runtime_dependency_closure(graph, module):
    loaded, pending = set(), [module]
    while pending:
        dependency = pending.pop()
        if dependency in loaded:
            continue
        assert dependency in graph, f"Unknown workspace dependency: {dependency}"
        loaded.add(dependency)
        pending.extend(graph[dependency])
        # Python also initializes every containing package. Traverse its real
        # re-exports without adding false cycles to the explicit source graph.
        parts = dependency.split(".")
        pending.extend(".".join(parts[:index]) for index in range(1, len(parts)))
    return loaded


def _assert_acyclic(graph):
    def visit(module, ancestors):
        assert module not in ancestors, (*ancestors, module)
        assert module in graph, f"Unknown workspace dependency: {module}"
        for dependency in graph[module]:
            visit(dependency, (*ancestors, module))

    for module in graph:
        visit(module, ())


def test_views_have_explicit_acyclic_dependencies_and_no_facade_imports():
    modules, packages = _view_modules(VIEW_ROOT)
    discovered = {
        module: _view_dependencies(path.read_text(), path, module=module, packages=packages, modules=modules)
        for module, path in modules.items()
    }
    assert discovered == VIEW_DEPENDENCIES
    _assert_acyclic(discovered)


@pytest.mark.parametrize("source", [
    "import Workstation.workspace_views.camera_preview as preview",
    "from Workstation.workspace_views import camera_preview as preview",
    "from Workstation.workspace_views.camera_preview import CameraPreview",
    "from .camera_preview import CameraPreview",
    "from . import camera_preview",
])
def test_dependency_gate_recognizes_each_supported_import_form(source):
    assert _view_dependencies(source) == {"camera_preview"}


@pytest.mark.parametrize("source,expected", [
    ("from ..shared import _display_number", {"shared"}),
    ("from .widgets import CalibrationErrorPlot", {"calibration_parts.widgets"}),
    ("from . import widgets", {"calibration_parts.widgets"}),
    ("from Workstation.workspace_views.calibration_parts import widgets", {"calibration_parts.widgets"}),
    ("import Workstation.workspace_views.optical_pose_parts.annotations as annotations", {"optical_pose_parts.annotations"}),
])
def test_dependency_gate_resolves_nested_modules_without_collapsing_package_names(source, expected):
    assert _view_dependencies(source, module="calibration_parts.quality") == expected


@pytest.mark.parametrize("source", [
    "import Workstation.workspaces as views",
    "from Workstation.workspaces import CameraPreview",
    "from Workstation import workspaces",
    "from .. import workspaces",
])
def test_dependency_gate_rejects_facade_back_imports(source):
    with pytest.raises(AssertionError):
        _view_dependencies(source)


@pytest.mark.parametrize("source", [
    "from ... import workspaces",
    "from Workstation import standalone_backend",
    "import Workstation.standalone_gui as legacy",
    "from Workstation.panel_application import PanelApplication",
    "import importlib as loader\nloader.import_module('Workstation.workspaces')",
    "from importlib import import_module as load\nload('Workstation.workspaces')",
    "import importlib\ngetattr(importlib, 'import_module')('Workstation.workspaces')",
    "eval('load_hidden_owner()')",
    "exec('import Workstation.workspaces')",
    "__import__('Workstation.workspaces')",
])
def test_nested_dependency_gate_rejects_backend_facade_and_dynamic_import_paths(source):
    with pytest.raises(AssertionError):
        _view_dependencies(source, module="calibration_parts.quality")


def test_dependency_gate_rejects_cycle_hidden_across_import_forms():
    graph = {
        "camera_preview": _view_dependencies("import Workstation.workspace_views.camera"),
        "camera": _view_dependencies("from Workstation.workspace_views import camera_preview"),
    }
    with pytest.raises(AssertionError):
        _assert_acyclic(graph)


def test_nested_cycle_is_rejected_across_relative_package_levels():
    modules = {"fixture_parts", "fixture_parts.first", "fixture_parts.nested", "fixture_parts.nested.second"}
    graph = {
        "fixture_parts": set(),
        "fixture_parts.nested": set(),
        "fixture_parts.first": _view_dependencies(
            "from .nested.second import value", module="fixture_parts.first", modules=modules),
        "fixture_parts.nested.second": _view_dependencies(
            "from .. import first", module="fixture_parts.nested.second", modules=modules),
    }
    assert graph["fixture_parts.first"] == {"fixture_parts.nested.second"}
    assert graph["fixture_parts.nested.second"] == {"fixture_parts.first"}
    with pytest.raises(AssertionError):
        _assert_acyclic(graph)


def test_package_reexport_does_not_create_a_false_child_to_parent_cycle():
    modules = {"fixture_parts", "fixture_parts.value", "shared"}
    graph = {
        "fixture_parts": _view_dependencies(
            "from .value import Value", module="fixture_parts", packages={"fixture_parts"}, modules=modules),
        "fixture_parts.value": _view_dependencies(
            "from ..shared import helper", module="fixture_parts.value", modules=modules),
        "shared": set(),
    }
    assert graph == {"fixture_parts": {"fixture_parts.value"}, "fixture_parts.value": {"shared"}, "shared": set()}
    _assert_acyclic(graph)
    assert _runtime_dependency_closure(graph, "fixture_parts.value") == modules


def test_recursive_inventory_keeps_nested_same_named_modules_distinct(tmp_path):
    for relative in ("__init__.py", "a/__init__.py", "a/widgets.py", "b/__init__.py", "b/widgets.py"):
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('"""fixture"""\n')
    modules, packages = _view_modules(tmp_path)
    assert set(modules) == {"a", "a.widgets", "b", "b.widgets"}
    assert packages == {"a", "b"}


@pytest.mark.parametrize("module", sorted(VIEW_DEPENDENCIES))
def test_owner_import_loads_only_expected_transitive_dependencies(module):
    loaded = _runtime_dependency_closure(VIEW_DEPENDENCIES, module)
    repo_root = APP_ROOT.parents[1]
    search_paths = [str(repo_root / "apps"), str(repo_root / "common/python")]
    script = (
        "import importlib, json, sys\n"
        f"sys.path[:0] = {search_paths!r}\n"
        f"importlib.import_module('Workstation.workspace_views.{module}')\n"
        "assert 'Workstation.workspaces' not in sys.modules\n"
        "print(json.dumps(sorted(name[len('Workstation.workspace_views.'):] for name in sys.modules "
        "if name.startswith('Workstation.workspace_views.'))))\n"
    )
    result = subprocess.run(
        [sys.executable, "-B", "-c", script],
        check=True, text=True, capture_output=True, timeout=30,
    )
    assert set(json.loads(result.stdout)) == loaded


def test_production_composition_imports_concrete_view_owners():
    for filename in ("main_window.py", "baseline_yaw90_pane.py"):
        tree = ast.parse((APP_ROOT / filename).read_text())
        imports = [node for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)]
        assert any("workspace_views." in (node.module or "") for node in imports)
        assert not any("workspaces" in (node.module or "").split(".") for node in imports)


def test_recording_owner_preserves_worker_recovery_wiring(workstation_qapplication):
    from Workstation.method_worker_pane import MethodWorkerRecoveryPane
    from Workstation.status_model import StatusModel
    from Workstation.workspace_views.recording import DataWorkspace

    worker = {
        "method_id": "pixhawk_rtk_live", "state": "RUNNING",
        "worker_generation": 7, "request_generation": 11,
        "measurement_id": "original-run", "pair_id": "original-run:pair-4",
        "elapsed_s": 12, "pending_jobs": 2,
    }
    state = {"state": "OPEN", "workers": [worker]}
    requests = []
    backend = SimpleNamespace(
        panel=SimpleNamespace(_measurement_session_hierarchy_enabled=False),
        measurement_worker_snapshot=lambda: state,
        restart_measurement_method_worker=lambda payload: requests.append(payload) or {"accepted": True},
    )
    view = DataWorkspace(backend, StatusModel(), lambda: None)
    try:
        pane = view.method_worker_pane
        assert isinstance(pane, MethodWorkerRecoveryPane)
        assert view.isAncestorOf(pane)
        pane.method_combo.setCurrentIndex(pane.method_combo.findData("pixhawk_rtk_live"))
        worker["elapsed_s"] = 72
        view._refresh_measurement_workbook()
        assert "00:01:12" in pane.status_label.toolTip()
        pane.restart_button.click()
        assert requests == [{
            "method_id": "pixhawk_rtk_live", "expected_worker_generation": 7,
            "expected_pair_id": "original-run:pair-4", "expected_request_generation": 11,
        }]
        state["state"] = "CLOSING"
        view._refresh_measurement_workbook()
        assert not pane.restart_button.isEnabled()
    finally:
        view.close()
        view.deleteLater()
        workstation_qapplication.processEvents()
