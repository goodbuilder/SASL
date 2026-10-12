#!/usr/bin/env python3
"""Require a real, unskipped synthetic NVENC recording test after driver repair.

No camera/network source is opened. The existing test runs inside the isolated
Workstation runner and verifies lossless decoded frames and finalized MP4.
"""
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    driver = subprocess.run(['nvidia-smi'], text=True, capture_output=True)
    print(driver.stdout or driver.stderr, end='')
    if driver.returncode:
        print('FAIL: NVIDIA driver is not ready; no NVENC pass is claimed.')
        return 1
    with tempfile.TemporaryDirectory(prefix='workstation-nvenc-check-') as directory:
        report = Path(directory) / 'result.xml'
        case = 'apps/Workstation/tests/test_camera_stream_recovery.py::test_decoded_nv12_recorder_finalizes_synthetic_lossless_mp4'
        result = subprocess.run([
            str(root / 'apps/Workstation/tools/run_isolated_tests.sh'),
            '-q', '-ra', '--tb=short', f'--junitxml={report}', case,
        ], cwd=root)
        if result.returncode:
            return result.returncode
        cases = ET.parse(report).getroot().findall('.//testcase')
        if len(cases) != 1 or any(case.find('skipped') is not None for case in cases):
            print('FAIL: NVENC validation was skipped; hardware acceptance requires execution.')
            return 1
        print('PASS: real NVENC lossless recording and MP4 finalization verified.')
        return 0


if __name__ == '__main__':
    raise SystemExit(main())
