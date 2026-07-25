from __future__ import annotations

from pathlib import Path


def test_operator_launcher_discovers_machine_local_yolo_weights_before_bare_fallback() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_operator_view.sh").read_text(encoding="utf-8")

    assert '"$ROOT_DIR/$candidate"' in source
    assert '"$ROOT_DIR/../$candidate"' in source
    assert '"$HOME/Downloads/$candidate"' in source
    assert '"$HOME/다운로드/$candidate"' in source


def test_operator_launcher_auto_selects_a_kconfiglib_capable_px4_python() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_operator_view.sh").read_text(encoding="utf-8")

    assert '"$ROOT_DIR/.venv-px4-local/bin/python"' in source
    assert '"$ROOT_DIR/.venv-px4-v1.17.0/bin/python"' in source
    assert "-c 'import kconfiglib'" in source


def test_cleanup_kills_rqt_process_group_even_after_its_wrapper_exits() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_operator_view.sh").read_text(encoding="utf-8")

    assert "Signal the recorded process group even if that original wrapper PID has" in source
    assert 'kill -KILL "-$pid" >/dev/null 2>&1 || kill -KILL "$pid"' in source


def test_auto_mission_launcher_selects_a_pymavlink_capable_python() -> None:
    source = (Path(__file__).resolve().parents[1] / "run_auto_mission.sh").read_text(encoding="utf-8")

    assert "resolve_pymavlink_python" in source
    assert '"$ROOT_DIR/.venv-px4-local/bin/python"' in source
    assert '"$ROOT_DIR/.venv-px4-v1.17.0/bin/python"' in source
    assert "-c 'import pymavlink'" in source
