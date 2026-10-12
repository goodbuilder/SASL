"""Strict reader for the human-edited v1.1 RQT control settings.

Only the listed control and device settings are interpreted. The rest of
``config/simulation.env`` remains owned by the existing shell launchers, so
this reader never evaluates shell code or expands variables.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
from dataclasses import dataclass
from pathlib import Path


SIM_ROOT = Path(__file__).resolve().parents[2]
REPO_ROOT = SIM_ROOT.parents[1]
DEFAULT_CONFIG = SIM_ROOT / "config" / "simulation.env"
OUTPUT_PARENT = SIM_ROOT / "recordings" / "stereo_v1_1"
SPEC_PATH = Path(__file__).resolve().parent / "protocol" / "03_experiment_spec.json"
MANIFEST_PATH = Path(__file__).resolve().parent / "protocol" / "04_run_manifest.json"

DEFAULTS = {
    "STEREO_V1_1_ACTION": "xlsx_plan",
    "STEREO_V1_1_RUN_ID": "G0-P0-R01",
    "STEREO_V1_1_OUTPUT_ID": "campaign_001",
    "STEREO_V1_1_OUTPUT_BASE": "repo",
    "STEREO_V1_1_INPUT_ROOT": "output",
    "STEREO_V1_1_WORKBOOK": "config/campaign/v1_1/sweep_manifest_v1_1.xlsx",
    "STEREO_V1_1_MC_TRIALS": "100",
    "STEREO_V1_1_PX4_DIR": "PX4-Autopilot",
    "STEREO_V1_1_YOLO_PYTHON": ".venv/bin/python",
    "YOLO_DEVICE": "cuda:0",
    "YOLO_TORCH_BACKEND": "cuda",
}
ALLOWED_ACTIONS = frozenset({
    "plan", "preflight", "development", "run", "all", "resume",
    "reprocess", "all_results", "qc", "legacy_sweep",
    "xlsx_plan", "xlsx_all", "xlsx_resume",
})
_KEYS = frozenset(DEFAULTS)
_LITERAL = re.compile(r"^[A-Za-z0-9_.:-]+$")
_REPO_RELATIVE_PATH = re.compile(r"^[A-Za-z0-9_./-]+$")
_OUTPUT_ID = re.compile(r"^[\w][\w.-]{0,127}$")
_YOLO_DEVICE = re.compile(r"^(cpu|cuda(?::[0-9]+)?)$")


@dataclass(frozen=True)
class CampaignControlConfig:
    action: str
    run_id: str
    output_id: str
    output_base: str
    input_root_setting: str
    workbook_path: Path
    workbook_sha256: str | None
    workbook_world_name: str | None
    mc_trials: int | None
    px4_dir_path: Path
    yolo_python_path: Path
    yolo_device: str
    yolo_torch_backend: str
    config_path: Path
    config_sha256: str

    @property
    def output_root(self) -> Path:
        if self.output_base == "repo":
            base = OUTPUT_PARENT.resolve()
        else:
            raw_root = os.environ.get("SASL_WORKSTATION_DATA_ROOT", "").strip()
            if not raw_root:
                raise ValueError("STEREO_V1_1_OUTPUT_BASE=data_root requires SASL_WORKSTATION_DATA_ROOT")
            base = Path(raw_root).expanduser().resolve()
        target = (base / self.output_id).resolve()
        if not target.is_relative_to(base) or target == base:
            raise ValueError("campaign output ID must remain a child of its selected data root")
        return target

    @property
    def input_root(self) -> Path:
        setting = self.input_root_setting
        if setting == "output":
            return self.output_root
        if setting == "repo":
            return REPO_ROOT.resolve()
        if setting == "data_root":
            raw_root = os.environ.get("SASL_WORKSTATION_DATA_ROOT", "").strip()
            if not raw_root:
                raise ValueError("STEREO_V1_1_INPUT_ROOT=data_root requires SASL_WORKSTATION_DATA_ROOT")
            return Path(raw_root).expanduser().resolve()
        return Path(os.path.abspath(REPO_ROOT / setting))


def _manifest_run_ids() -> set[str]:
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8-sig"))
    return {str(row["run_id"]) for row in manifest["runs"]}


def load_control_config(path: Path | str = DEFAULT_CONFIG) -> CampaignControlConfig:
    """Read and validate the RQT control block without sourcing the env file."""
    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f"Simulation.env not found: {config_path}")
    raw = config_path.read_bytes()
    text = raw.decode("utf-8-sig")
    values = dict(DEFAULTS)
    seen: set[str] = set()
    for number, line in enumerate(text.splitlines(), start=1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            continue
        key, value = stripped.split("=", 1)
        key = key.strip()
        if key not in _KEYS:
            continue
        if key in seen:
            raise ValueError(f"duplicate {key} at line {number}")
        seen.add(key)
        value = value.strip()
        if key == "STEREO_V1_1_INPUT_ROOT":
            allowed = None  # validated as a non-executable relative path below
        elif key in {"STEREO_V1_1_PX4_DIR", "STEREO_V1_1_YOLO_PYTHON", "STEREO_V1_1_WORKBOOK"}:
            allowed = _REPO_RELATIVE_PATH
        else:
            allowed = _LITERAL
        if allowed is not None and not allowed.fullmatch(value):
            raise ValueError(f"{key} at line {number} must be a plain literal token")
        values[key] = value

    action = values["STEREO_V1_1_ACTION"]
    if action not in ALLOWED_ACTIONS:
        raise ValueError(f"unsupported STEREO_V1_1_ACTION: {action}")
    run_id = values["STEREO_V1_1_RUN_ID"]
    if action not in {"xlsx_plan", "xlsx_all", "xlsx_resume"} and run_id not in _manifest_run_ids():
        raise ValueError(f"run id is not in the frozen 39-slot manifest: {run_id}")
    output_id = values["STEREO_V1_1_OUTPUT_ID"]
    if not _OUTPUT_ID.fullmatch(output_id) or output_id in (".", ".."):
        raise ValueError("STEREO_V1_1_OUTPUT_ID must be a short path-free name")
    output_base = values["STEREO_V1_1_OUTPUT_BASE"]
    if output_base not in {"repo", "data_root"}:
        raise ValueError("STEREO_V1_1_OUTPUT_BASE must be repo or data_root")
    input_root_setting = values["STEREO_V1_1_INPUT_ROOT"]
    if (not input_root_setting or Path(input_root_setting).is_absolute()
            or any(character in input_root_setting for character in "\x00\r\n=$`\\;|&<>")):
        raise ValueError("STEREO_V1_1_INPUT_ROOT must be output, repo, data_root, or a relative path literal")
    workbook_relative = values["STEREO_V1_1_WORKBOOK"]
    if (not _REPO_RELATIVE_PATH.fullmatch(workbook_relative)
            or Path(workbook_relative).is_absolute()):
        raise ValueError("STEREO_V1_1_WORKBOOK must be a repo-relative path without shell expansion")
    workbook_path = Path(os.path.abspath(SIM_ROOT / workbook_relative))
    if not workbook_path.is_relative_to(SIM_ROOT.resolve()):
        raise ValueError("STEREO_V1_1_WORKBOOK must stay inside apps/Simulation")
    workbook_sha256 = None
    workbook_world_name = None
    if action in {"xlsx_plan", "xlsx_all", "xlsx_resume"}:
        from campaigns.stereo_v1_1.workbook_campaign import build_workbook_plan
        compiled_workbook_plan = build_workbook_plan(workbook_path)
        workbook_sha256 = compiled_workbook_plan["workbook_sha256"]
        world_rows = [row["declared_value"] for row in compiled_workbook_plan["environment_contract"]
                      if row["setting_id"] == "world_name"]
        if world_rows:
            workbook_world_name = world_rows[0]
    mc_trials = None
    if action in {"resume", "all_results", "reprocess"}:
        try:
            mc_trials = int(values["STEREO_V1_1_MC_TRIALS"])
        except ValueError:
            raise ValueError("STEREO_V1_1_MC_TRIALS must be an integer") from None
        if not 1 <= mc_trials <= 1000:
            raise ValueError("STEREO_V1_1_MC_TRIALS must be between 1 and 1000")
    px4_relative = values["STEREO_V1_1_PX4_DIR"]
    if not _REPO_RELATIVE_PATH.fullmatch(px4_relative) or Path(px4_relative).is_absolute():
        raise ValueError("STEREO_V1_1_PX4_DIR must be a repo-relative path without shell expansion")
    px4_dir = (SIM_ROOT / px4_relative).resolve()
    if not px4_dir.is_relative_to(REPO_ROOT.resolve()):
        raise ValueError("STEREO_V1_1_PX4_DIR must resolve inside the SASL repository")
    yolo_relative = values["STEREO_V1_1_YOLO_PYTHON"]
    if not _REPO_RELATIVE_PATH.fullmatch(yolo_relative) or Path(yolo_relative).is_absolute():
        raise ValueError("STEREO_V1_1_YOLO_PYTHON must be a repo-relative path without shell expansion")
    # Keep the configured path lexical: venv/bin/python is commonly a symlink
    # to the system interpreter, while its isolated site-packages remain in
    # the repo-local venv. Resolve '..' for containment without following that
    # final interpreter symlink.
    yolo_python = Path(os.path.abspath(SIM_ROOT / yolo_relative))
    if not yolo_python.is_relative_to(SIM_ROOT.resolve()):
        raise ValueError("STEREO_V1_1_YOLO_PYTHON must stay inside apps/Simulation")
    yolo_device = values["YOLO_DEVICE"]
    if not _YOLO_DEVICE.fullmatch(yolo_device):
        raise ValueError("YOLO_DEVICE must be cpu or cuda[:index]")
    yolo_torch_backend = values["YOLO_TORCH_BACKEND"]
    if yolo_torch_backend not in {"cpu", "cuda", "auto"}:
        raise ValueError("YOLO_TORCH_BACKEND must be cpu, cuda, or auto")
    return CampaignControlConfig(
        action=action,
        run_id=run_id,
        output_id=output_id,
        output_base=output_base,
        input_root_setting=input_root_setting,
        workbook_path=workbook_path,
        workbook_sha256=workbook_sha256,
        workbook_world_name=workbook_world_name,
        mc_trials=mc_trials,
        px4_dir_path=px4_dir,
        yolo_python_path=yolo_python,
        yolo_device=yolo_device,
        yolo_torch_backend=yolo_torch_backend,
        config_path=config_path,
        config_sha256=hashlib.sha256(raw).hexdigest(),
    )
