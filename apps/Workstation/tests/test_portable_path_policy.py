from __future__ import annotations

from tools.check_portable_paths import ROOT, repository_violations, scan_text


def test_portable_path_policy_rejects_user_home_and_dated_checkout() -> None:
    violations = scan_text(
        "apps/example.py",
        "\n".join(
            (
                'MODEL = "/home/operator/models/yolo.pt"',
                r'CACHE = "C:\Users\operator\cache"',
                'ROOT = "/path/to/SASL-20260821"',
            )
        ),
    )

    assert {violation.reason for violation in violations} == {
        "POSIX user-home absolute path",
        "Windows user-home absolute path",
        "dated SASL checkout name",
    }


def test_portable_path_policy_allows_system_and_historical_paths() -> None:
    assert scan_text(
        "apps/example.py",
        'ROS = "/opt/ros/humble"\nDEVICE = "/dev/ttyACM0"\nCACHE = "/tmp/app-cache"',
    ) == ()
    assert scan_text(
        "apps/Simulation/reports/historical.md",
        "captured at /home/operator/old-run",
    ) == ()


def test_current_repository_has_no_active_portability_violations() -> None:
    assert repository_violations(ROOT) == ()
