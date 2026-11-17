from __future__ import annotations

from pathlib import Path

from campaigns.stereo_v1_1.runtime_paths import root_python_path


def test_root_python_prefers_app_runtime_when_it_exists(tmp_path: Path) -> None:
    app_python = tmp_path / ".venv" / "bin" / "python"
    app_python.parent.mkdir(parents=True)
    app_python.touch()

    assert root_python_path(tmp_path, environ={}, fallback="/usr/bin/python3") == app_python


def test_root_python_preserves_explicit_absolute_override(tmp_path: Path) -> None:
    override = tmp_path / "custom-python"

    assert root_python_path(
        tmp_path,
        environ={"STEREO_V1_1_ROOT_PYTHON": str(override)},
        fallback="/usr/bin/python3",
    ) == override


def test_root_python_resolves_relative_override_from_sim_root(tmp_path: Path) -> None:
    assert root_python_path(
        tmp_path,
        environ={"STEREO_V1_1_ROOT_PYTHON": ".local/python"},
        fallback="/usr/bin/python3",
    ) == tmp_path / ".local/python"


def test_root_python_falls_back_when_app_runtime_is_absent(tmp_path: Path) -> None:
    assert root_python_path(
        tmp_path, environ={}, fallback="/usr/bin/python3"
    ) == Path("/usr/bin/python3")
