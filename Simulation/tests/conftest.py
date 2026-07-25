from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest


SIMULATION_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = SIMULATION_ROOT.parent


@pytest.fixture(scope="session")
def isolated_stereo_ground_sim_overlay(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """Build the installed-package contract without creating a repo-local overlay."""

    root = tmp_path_factory.mktemp("stereo_ground_sim_overlay")
    command = [
        "colcon",
        "--log-base",
        str(root / "log"),
        "build",
        "--base-paths",
        str(SIMULATION_ROOT / "stereo_ground_sim"),
        "--packages-select",
        "stereo_ground_sim",
        "--build-base",
        str(root / "build"),
        "--install-base",
        str(root / "install"),
        "--event-handlers",
        "console_direct+",
    ]
    result = subprocess.run(
        command,
        cwd=REPOSITORY_ROOT,
        env=dict(os.environ),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert result.returncode == 0, (
        "isolated stereo_ground_sim overlay build failed\n"
        f"stdout:\n{result.stdout[-4000:]}\n"
        f"stderr:\n{result.stderr[-4000:]}"
    )
    assert (root / "install" / "setup.bash").is_file()
    return root


@pytest.fixture(scope="class")
def isolated_operator_overlay_root(request, isolated_stereo_ground_sim_overlay: Path) -> None:
    request.cls.isolated_operator_overlay_root = isolated_stereo_ground_sim_overlay
