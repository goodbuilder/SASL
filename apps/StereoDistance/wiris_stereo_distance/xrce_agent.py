from __future__ import annotations

import subprocess
from pathlib import Path


DEFAULT_MICRO_XRCE_AGENT_LOG = Path("/tmp/stereo_microxrceagent.log")


def micro_xrce_agent_command(port: int) -> list[str]:
    port = int(port)
    if port < 1 or port > 65535:
        raise ValueError(f"uXRCE port must be between 1 and 65535: {port}")
    return ["MicroXRCEAgent", "udp4", "-p", str(port)]


def is_micro_xrce_agent_running(port: int) -> bool:
    port = int(port)
    pattern = rf"MicroXRCEAgent.*udp4.*-p[ ]+{port}"
    try:
        result = subprocess.run(
            ["pgrep", "-f", pattern],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except FileNotFoundError:
        return False
    return result.returncode == 0


def ensure_micro_xrce_agent(
    port: int,
    log_path: Path = DEFAULT_MICRO_XRCE_AGENT_LOG,
) -> tuple[bool, str]:
    port = int(port)
    if is_micro_xrce_agent_running(port):
        return False, f"already running on UDP {port}"

    command = micro_xrce_agent_command(port)
    try:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open("ab") as log_file:
            process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
    except FileNotFoundError as exc:
        raise RuntimeError(
            "MicroXRCEAgent command was not found. Install Micro XRCE-DDS Agent or start it manually."
        ) from exc
    except OSError as exc:
        raise RuntimeError(f"MicroXRCEAgent failed to start: {exc}") from exc

    return True, f"started on UDP {port} (pid {process.pid}, log {log_path})"
