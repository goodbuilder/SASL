from __future__ import annotations

import os
import subprocess
from pathlib import Path


SIMULATION_ROOT = Path(__file__).resolve().parents[1]
INTEGRATION_ROOT = SIMULATION_ROOT.parent


def test_operator_view_consumes_only_the_canonical_px4_msgs_overlay() -> None:
    source = (SIMULATION_ROOT / "run_operator_view.sh").read_text(encoding="utf-8")

    assert 'SASL_ROS_WORKSPACE="${SASL_ROS_WORKSPACE:-$INTEGRATION_ROOT/workspace/main}"' in source
    assert 'source "$PX4_MSGS_OVERLAY_SETUP"' in source
    assert "86d8239e962f6939e05c3737784f60c02fa884db" in source
    assert "PX4_MSGS_OVERLAY_MANIFEST" in source
    assert '--packages-select stereo_ground_sim stereo_ground_sim_overlay' in source
    assert '--packages-select px4_msgs stereo_ground_sim' not in source
    assert '"$ROOT_DIR/px4_msgs"' not in source


def test_machine_local_px4_artifacts_are_ignored() -> None:
    ignore = (INTEGRATION_ROOT / ".gitignore").read_text(encoding="utf-8")

    assert ".venv-px4-*/" in ignore
    assert "Simulation/px4_msgs/" in ignore


def test_bootstrap_uses_a_detached_pinned_px4_msgs_checkout() -> None:
    source = (INTEGRATION_ROOT / "scripts" / "bootstrap_ros_workspace.sh").read_text(encoding="utf-8")

    assert 'git -C "$PX4_MSGS_DIR" checkout --detach "$PX4_MSGS_REF"' in source
    assert "86d8239e962f6939e05c3737784f60c02fa884db" in source
    assert 'rev-parse "${PX4_MSGS_REF}^{commit}"' in source
    assert 'ACTUAL_PX4_MSGS_COMMIT="$(git -C "$PX4_MSGS_DIR" rev-parse HEAD)"' in source
    assert "px4_msgs-overlay.env" in source


def test_bootstrap_installs_manifest_for_the_exact_source_commit(tmp_path: Path) -> None:
    upstream = tmp_path / "px4_msgs-upstream"
    upstream.mkdir()
    (upstream / "package.xml").write_text(
        "<package format=\"3\"><name>px4_msgs</name><version>0.0.0</version>"
        "<description>fixture</description><maintainer email=\"test@example.invalid\">test</maintainer>"
        "<license>Apache-2.0</license></package>\n",
        encoding="utf-8",
    )
    subprocess.run(["git", "init", "-q", str(upstream)], check=True)
    subprocess.run(["git", "-C", str(upstream), "add", "package.xml"], check=True)
    subprocess.run(
        [
            "git",
            "-C",
            str(upstream),
            "-c",
            "user.name=Workspace Test",
            "-c",
            "user.email=workspace-test@example.invalid",
            "commit",
            "-qm",
            "fixture",
        ],
        check=True,
    )
    commit = subprocess.run(
        ["git", "-C", str(upstream), "rev-parse", "HEAD"],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    ).stdout.strip()

    ros_setup = tmp_path / "ros-setup.bash"
    ros_setup.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    colcon = bin_dir / "colcon"
    colcon.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
    colcon.chmod(0o755)
    workspace = tmp_path / "workspace"
    env = os.environ.copy()
    env.update(
        {
            "PATH": f"{bin_dir}:{env['PATH']}",
            "SASL_ROS_WORKSPACE": str(workspace),
            "PX4_MSGS_URL": str(upstream),
            "PX4_MSGS_REF": commit,
            "ROS_SETUP": str(ros_setup),
            "SASL_ROS_CACHE": str(tmp_path / "cache"),
        }
    )
    result = subprocess.run(
        [str(INTEGRATION_ROOT / "scripts" / "bootstrap_ros_workspace.sh")],
        cwd=INTEGRATION_ROOT,
        env=env,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode == 0, result.stderr
    manifest = workspace / "install" / "share" / "sasl_ros_workspace" / "px4_msgs-overlay.env"
    assert manifest.read_text(encoding="utf-8").splitlines() == [
        f"PX4_MSGS_COMMIT={commit}",
        f"PX4_MSGS_REF={commit}",
    ]
