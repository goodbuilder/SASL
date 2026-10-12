# Runtime environment repair checkpoint

Status: Python repair complete; awaiting reboot decision for NVIDIA.
User request: resolve the two skips and one warning. No reboot permission granted yet.

- Canonical .venv exists and is fully provisioned; do not delete it. User-site/ROS Python paths disabled in runner.
- NumPy1.24.4/SciPy1.8.0, pyproj3.7.1/rasterio1.4.4, Torch2.13.0/torchvision0.28.0; cffi2.1.1 added for inherited Ubuntu PyNaCl closure. pip check clean.
- Full canonical run2268PASS/1SKIP/0warnings234.56s, closed session12695. Remaining skip NVENC.
- Loaded kernel module595.91.07 vs libraries/on-disk module595.99.02, kernel6.8.0-138-generic, DKMS installed, SecureBoot disabled. No driver unload or reboot performed. Active desktop uses NVIDIA modules.
- Reboot interrupts applications/jobs. Ask only after all non-disruptive work/checks are ready. User can reboot manually or explicitly authorize the action. No timeout/default selection counts as permission.
- After reboot: run python3 tools/check_workstation_nvenc.py from repo root; require executed PASS, not skipped. If it fails, inspect actual driver and encoder errors before further changes. Update report/ledger when the remaining hardware criterion is met.
- Sources: requirements/dev.txt + optional-terrain.txt + constraints-workstation-py310.txt; tools/setup_workstation_test_env.sh; tools/check_workstation_nvenc.py; Workstation test runner; terrain test and NVENC probe. Product coordinate algorithms unchanged.
- Existing RCA/decomposition/Target and concurrent Simulation changes preserved; no commit/push requested.

Read next: report.md, verification.md, evidence/regression-before-reboot.log.

Pending user question: 직접 재부팅 후 알려주거나, 작업을 저장한 상태에서 즉시 재부팅을 명시적으로 승인할 수 있는가? 재부팅 시 실행 중 앱/작업이 종료되므로 별도 확인이 필요하다.
