from __future__ import annotations

from campaigns.stereo_v1_1.campaign_preflight import _short_probe_blockers


def _check(name: str, status: str, detail: dict | None = None, *, required: bool = True):
    return {
        "name": name,
        "status": status,
        "detail": detail or {},
        "required": required,
    }


def test_missing_cuda_does_not_block_no_yolo_storage_probe():
    checks = [
        _check("isolated_yolo_python_and_device", "FAIL", {
            "required_imports_exit": 0,
            "selected_device_available": False,
        }),
    ]

    assert _short_probe_blockers(checks) == []


def test_missing_yolo_packages_still_blocks_short_probe():
    checks = [_check("isolated_yolo_python_and_device", "FAIL", {
        "required_imports_exit": 1,
    })]

    assert _short_probe_blockers(checks) == ["isolated_yolo_python_and_device"]


def test_unrelated_required_failure_still_blocks_short_probe():
    checks = [
        _check("isolated_yolo_python_and_device", "FAIL", {
            "required_imports_exit": 0,
            "selected_device_available": False,
        }),
        _check("simulation_overlay", "FAIL"),
    ]

    assert _short_probe_blockers(checks) == ["simulation_overlay"]


def test_optional_warning_does_not_block_short_probe():
    checks = [_check("optional_diagnostic", "WARN", required=False)]

    assert _short_probe_blockers(checks) == []
