from __future__ import annotations

import json
from pathlib import Path

import pytest

from Workstation.workstation_core.pixhawk_firmware import (
    PINNED_PX_UPLOADER_SHA256,
    atomic_write_firmware_evidence,
    parse_px_uploader_output,
    read_px4_firmware_identity,
    verify_pinned_uploader,
)


def test_repository_firmware_and_pinned_uploader_have_verified_identity() -> None:
    root = Path(__file__).resolve().parents[1]
    firmware = read_px4_firmware_identity(root / "firmware/stock/px4_fmu-v6x_default.px4")

    assert firmware.board_id == 53
    assert firmware.board_revision == 0
    assert firmware.image_size == 1_959_920
    assert firmware.image_size <= firmware.image_maxsize
    assert firmware.sha256 == "8f2c42787163b71e00758205ab0bed9df1669fd26d7a9f42d6f995cbabba0d7f"
    assert verify_pinned_uploader(root / "tools/px_uploader.py") == PINNED_PX_UPLOADER_SHA256


def test_firmware_rejects_invalid_bundle_metadata(tmp_path: Path) -> None:
    invalid = tmp_path / "bad.px4"
    invalid.write_text(json.dumps({"magic": "PX4FWv1", "board_id": 53}), encoding="utf-8")

    with pytest.raises(ValueError, match="board/image metadata"):
        read_px4_firmware_identity(invalid)


def test_uploader_output_requires_independent_board_and_all_phases() -> None:
    parsed = parse_px_uploader_output(
        "Found board id: 53,0 bootloader protocol revision 5 on /dev/ttyACM0\n"
        "Erase  100%\nProgram 100%\nVerify 100%\nRebooting."
    )

    assert parsed == {
        "detected_board_id": 53,
        "detected_board_revision": 0,
        "bootloader_protocol_revision": 5,
        "erase": "PASS",
        "program": "PASS",
        "verify": "PASS",
        "reboot": "PASS",
    }


def test_firmware_evidence_atomic_write(tmp_path: Path) -> None:
    target = tmp_path / "flash" / "result.json"
    atomic_write_firmware_evidence(target, {"state": "FLASHED/VERIFIED", "board": 53})
    assert json.loads(target.read_text(encoding="utf-8"))["board"] == 53
    assert list(target.parent.glob(".*.tmp")) == []


def test_usb_flash_worker_verifies_role_and_invokes_pinned_uploader_without_force(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from Workstation import device_runtime_workers
    from Workstation.device_runtime_workers import PixhawkUsbFlashWorker
    from Workstation.workstation_core.pixhawk_attitude import UsbMavlinkIdentity

    root = Path(__file__).resolve().parents[1]
    calls: list[list[str]] = []
    identities = [
        UsbMavlinkIdentity("/dev/ttyACM9", "/dev/ttyACM9", 115200, 3, 1, 1.0),
        UsbMavlinkIdentity("/dev/ttyACM9", "/dev/ttyACM9", 115200, 3, 1, 2.0),
    ]
    monkeypatch.setattr(device_runtime_workers.os.path, "realpath", lambda _path: "/dev/ttyACM9")
    monkeypatch.setattr(
        device_runtime_workers,
        "probe_usb_mavlink_devices",
        lambda **_kwargs: [identities.pop(0)],
    )

    class _Process:
        def __init__(self, command, **_kwargs) -> None:
            calls.append(list(command))
            self.stdout = iter(
                [
                    "Found board id: 53,0 bootloader protocol revision 5 on /dev/ttyACM9\n",
                    "Erase Program Verify Rebooting.\n",
                ]
            )

        def poll(self):
            return 0

        def wait(self, timeout=None):
            return 0

        def terminate(self):
            return None

    monkeypatch.setattr(device_runtime_workers.subprocess, "Popen", _Process)
    worker = PixhawkUsbFlashWorker(
        camera="drone",
        expected_system_id=3,
        usb_device="/dev/ttyACM9",
        firmware_path=str(root / "firmware/stock/px4_fmu-v6x_default.px4"),
        uploader_path=str(root / "tools/px_uploader.py"),
    )
    completed: list[dict[str, object]] = []
    failures: list[str] = []
    worker.completed.connect(completed.append)
    worker.failed.connect(failures.append)

    worker.run()

    assert failures == []
    assert completed[0]["state"] == "FLASHED/VERIFIED"
    assert completed[0]["preflight"]["system_id"] == 3
    assert completed[0]["postflight"]["system_id"] == 3
    assert "--force" not in calls[0]
    assert "--force-erase" not in calls[0]
