from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

from campaigns.stereo_v1_1 import control_cli, control_config, workbook_campaign
from campaigns.stereo_v1_1.control_config import load_control_config


ROOT = Path(__file__).resolve().parents[1]


def _config(tmp_path: Path, body: str) -> Path:
    path = tmp_path / "simulation.env"
    path.write_text(body, encoding="utf-8")
    return path


def test_human_control_block_defaults_to_manifest_plan_and_repo_relative_output(tmp_path):
    path = _config(tmp_path, "# rest of Simulation.env is not evaluated\n")
    config = load_control_config(path)
    assert config.action == "xlsx_plan"
    assert config.run_id == "G0-P0-R01"
    assert config.mc_trials is None
    assert config.output_root == ROOT / "recordings" / "stereo_v1_1" / "campaign_001"
    assert config.px4_dir_path == ROOT / "PX4-Autopilot"
    assert config.yolo_python_path == ROOT / ".venv" / "bin" / "python"
    assert config.input_root_setting == "output"
    assert config.input_root == config.output_root
    assert config.yolo_device == "cuda:0"
    assert config.workbook_path == ROOT / "config/campaign/v1_1/sweep_manifest_v1_1.xlsx"
    assert config.workbook_world_name == "stereo_ground_drone"
    assert len(config.workbook_sha256) == 64


def test_xlsx_action_ignores_offline_trial_setting_but_uses_workbook_world(tmp_path, monkeypatch):
    path = _config(tmp_path, "\n".join([
        "STEREO_V1_1_ACTION=xlsx_plan",
        "STEREO_V1_1_MC_TRIALS=not_an_integer",
    ]))
    config = load_control_config(path)
    monkeypatch.setenv("PX4_GZ_WORLD", "stale_shell_world")
    monkeypatch.setattr(control_cli, "_effective_px4_dir", lambda _config: "/pinned/px4")
    monkeypatch.setattr(control_cli, "_analysis_python", lambda _config: Path("/pinned/python"))

    runtime = control_cli._campaign_runtime_env(config)

    assert config.mc_trials is None
    assert runtime["PX4_GZ_WORLD"] == "stereo_ground_drone"


def test_resume_preflight_uses_environment_from_the_frozen_workbook(tmp_path, monkeypatch):
    frozen_workbook = tmp_path / "frozen_input.xlsx"
    captured = {}
    monkeypatch.setattr(control_cli, "_campaign_runtime_env",
                        lambda _config: {"PX4_GZ_WORLD": "current_workbook_world"})
    monkeypatch.setattr(workbook_campaign, "build_workbook_plan", lambda _path: {
        "environment_contract": [{"setting_id": "world_name", "declared_value": "frozen_world"}],
    })
    monkeypatch.setattr(control_cli, "_run", lambda command, *, env, label:
                        captured.update(command=command, env=env, label=label) or 0)
    config = SimpleNamespace(
        config_path=tmp_path / "simulation.env",
        output_root=tmp_path / "campaign",
    )

    assert control_cli._run_preflight(config, action="resume", workbook_path=frozen_workbook) == 0

    assert captured["env"]["PX4_GZ_WORLD"] == "frozen_world"
    assert str(frozen_workbook) in captured["command"]


def test_workbook_plan_action_validates_xlsx_and_routes_without_flight(tmp_path, monkeypatch):
    path = _config(tmp_path, "STEREO_V1_1_ACTION=xlsx_plan\n")
    calls = []
    monkeypatch.setattr(control_cli, "_run", lambda command, **kwargs: calls.append((command, kwargs)) or 0)

    assert control_cli.execute(path) == 0

    config = load_control_config(path)
    assert len(config.workbook_sha256) == 64
    command, options = calls[0]
    assert command[1].endswith("workbook_campaign.py")
    assert command[2] == "plan"
    assert Path(command[4]) == config.workbook_path
    assert options["label"] == "workbook-plan"


def test_workbook_action_does_not_require_the_static_run_manifest(tmp_path, monkeypatch):
    path = _config(tmp_path, "STEREO_V1_1_ACTION=xlsx_plan\n")

    def forbidden_static_manifest():
        raise AssertionError("XLSX actions must get their run count and IDs from workbook rows")

    monkeypatch.setattr(control_config, "_manifest_run_ids", forbidden_static_manifest)
    config = load_control_config(path)
    assert config.action == "xlsx_plan"
    assert len(config.workbook_sha256) == 64


def test_repo_local_venv_interpreter_symlink_stays_at_configured_path(tmp_path, monkeypatch):
    sim_root = tmp_path / "repo" / "apps" / "Simulation"
    python_path = sim_root / ".venv" / "bin" / "python"
    python_path.parent.mkdir(parents=True)
    python_path.symlink_to("/usr/bin/python3")
    monkeypatch.setattr(control_config, "SIM_ROOT", sim_root)
    monkeypatch.setattr(control_config, "REPO_ROOT", sim_root.parents[1])
    env = _config(tmp_path, "STEREO_V1_1_ACTION=plan\nSTEREO_V1_1_YOLO_PYTHON=.venv/bin/python")

    config = load_control_config(env)

    assert config.yolo_python_path == python_path
    assert config.yolo_python_path.is_file()
    monkeypatch.setattr(control_cli, "SIM_ROOT", sim_root)
    monkeypatch.setenv("STEREO_V1_1_YOLO_PYTHON", ".venv/bin/python")
    assert control_cli._analysis_python(SimpleNamespace(yolo_python_path=python_path)) == python_path


def test_data_root_output_uses_only_a_relative_campaign_id(tmp_path, monkeypatch):
    data_root = tmp_path / "user-data"
    data_root.mkdir()
    monkeypatch.setenv("SASL_WORKSTATION_DATA_ROOT", str(data_root))
    path = _config(tmp_path, "\n".join([
        "STEREO_V1_1_ACTION=reprocess",
        "STEREO_V1_1_OUTPUT_ID=existing_campaign_39",
        "STEREO_V1_1_OUTPUT_BASE=data_root",
    ]))
    config = load_control_config(path)
    assert config.output_root == data_root / "existing_campaign_39"


def test_data_root_output_requires_explicit_runtime_root(tmp_path, monkeypatch):
    monkeypatch.delenv("SASL_WORKSTATION_DATA_ROOT", raising=False)
    path = _config(tmp_path, "STEREO_V1_1_OUTPUT_BASE=data_root")
    config = load_control_config(path)
    with pytest.raises(ValueError, match="SASL_WORKSTATION_DATA_ROOT"):
        _ = config.output_root


def test_reprocess_input_root_resolves_relative_to_sasl_root_without_touching_output(tmp_path, monkeypatch):
    repo_root = tmp_path / "SASL"
    sim_root = repo_root / "apps" / "Simulation"
    source_root = tmp_path / "existing_39_run_campaign"
    source_root.mkdir()
    monkeypatch.setattr(control_config, "REPO_ROOT", repo_root)
    monkeypatch.setattr(control_config, "SIM_ROOT", sim_root)
    path = _config(tmp_path, "\n".join([
        "STEREO_V1_1_ACTION=reprocess",
        "STEREO_V1_1_INPUT_ROOT=../existing_39_run_campaign",
        "STEREO_V1_1_OUTPUT_BASE=repo",
        "STEREO_V1_1_OUTPUT_ID=analysis_output",
    ]))

    config = load_control_config(path)

    assert config.input_root == source_root
    assert config.output_root == ROOT / "recordings" / "stereo_v1_1" / "analysis_output"
    assert config.input_root != config.output_root


def test_reprocess_dispatch_passes_source_and_output_as_distinct_roots(tmp_path, monkeypatch):
    repo_root = tmp_path / "SASL"
    sim_root = repo_root / "apps" / "Simulation"
    source_root = tmp_path / "existing_39_run_campaign"
    source_root.mkdir()
    monkeypatch.setattr(control_config, "REPO_ROOT", repo_root)
    monkeypatch.setattr(control_config, "SIM_ROOT", sim_root)
    monkeypatch.setenv("STEREO_V1_1_YOLO_PYTHON", "/usr/bin/python3")
    path = _config(tmp_path, "\n".join([
        "STEREO_V1_1_ACTION=reprocess",
        "STEREO_V1_1_INPUT_ROOT=../existing_39_run_campaign",
        "STEREO_V1_1_OUTPUT_BASE=repo",
        "STEREO_V1_1_OUTPUT_ID=analysis_output",
        "STEREO_V1_1_MC_TRIALS=100",
    ]))
    config = load_control_config(path)
    captured = {}

    def fake_run(command, *, env, label):
        captured.update(command=command, env=env, label=label)
        return 0

    monkeypatch.setattr(control_cli, "_run", fake_run)

    assert control_cli._run_analysis(config, input_root=config.input_root) == 0
    assert captured["label"] == "reprocess"
    assert captured["command"][captured["command"].index("--output-root") + 1] == str(config.output_root)
    assert captured["command"][captured["command"].index("--input-root") + 1] == str(source_root)
    assert captured["command"][captured["command"].index("--mc-trials") + 1] == "100"


def test_reprocess_input_root_can_use_the_configured_data_root(tmp_path, monkeypatch):
    data_root = tmp_path / "readonly_campaign_root"
    data_root.mkdir()
    monkeypatch.setenv("SASL_WORKSTATION_DATA_ROOT", str(data_root))
    path = _config(tmp_path, "STEREO_V1_1_INPUT_ROOT=data_root")

    config = load_control_config(path)

    assert config.input_root == data_root


def test_control_values_are_literal_validated_tokens_and_manifest_ids(tmp_path):
    path = _config(tmp_path, "\n".join([
        "STEREO_V1_1_ACTION=all_results",
        "STEREO_V1_1_RUN_ID=G1-P4-R03",
        "STEREO_V1_1_OUTPUT_ID=notebook_run_2",
        "STEREO_V1_1_MC_TRIALS=100",
        "STEREO_V1_1_PX4_DIR=PX4-Autopilot",
        "YOLO_DEVICE=cpu",
    ]))
    config = load_control_config(path)
    assert config.action == "all_results"
    assert config.run_id == "G1-P4-R03"
    assert config.output_id == "notebook_run_2"
    assert config.yolo_device == "cpu"

    bad = _config(tmp_path, "STEREO_V1_1_ACTION=$(touch /tmp/should_not_run)")
    with pytest.raises(ValueError, match="plain literal token"):
        load_control_config(bad)


@pytest.mark.parametrize("line, message", [
    ("STEREO_V1_1_ACTION=unknown", "unsupported"),
    ("STEREO_V1_1_RUN_ID=G0-NOT-A-RUN", "not in the frozen"),
    ("STEREO_V1_1_OUTPUT_ID=../outside", "plain literal token"),
    ("STEREO_V1_1_MC_TRIALS=0", "between 1 and 1000"),
    ("STEREO_V1_1_PX4_DIR=../../../../tmp/PX4", "inside the SASL repository"),
    ("STEREO_V1_1_YOLO_PYTHON=../../../../tmp/python", "stay inside apps/Simulation"),
    ("STEREO_V1_1_INPUT_ROOT=../source;touch", "relative path literal"),
    ("YOLO_DEVICE=0;rm", "plain literal token"),
])
def test_invalid_control_values_fail_closed(tmp_path, line, message):
    body = f"STEREO_V1_1_ACTION=plan\n{line}" if line.startswith("STEREO_V1_1_RUN_ID=") else line
    if line.startswith("STEREO_V1_1_MC_TRIALS="):
        body = f"STEREO_V1_1_ACTION=reprocess\n{line}"
    path = _config(tmp_path, body)
    with pytest.raises(ValueError, match=message):
        load_control_config(path)


def test_duplicate_campaign_keys_fail_closed(tmp_path):
    path = _config(tmp_path, "STEREO_V1_1_ACTION=plan\nSTEREO_V1_1_ACTION=all\n")
    with pytest.raises(ValueError, match="duplicate"):
        load_control_config(path)
