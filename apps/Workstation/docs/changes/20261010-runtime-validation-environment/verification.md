# Runtime validation environment repair

User asks to resolve two skipped tests and one warning from Target reference verification. Prior test result2266PASS/2SKIP/1warning is historical evidence. Preserve all preceding app/Simulation work.

## Source of truth and scope
Declared requirements/runtime.txt controls NumPy>=1.23.5,<1.25 and SciPy>=1.8,<1.11. Terrain code requires both rasterio>=1.3 and pyproj>=3.5. Official runtime uses root .venv, system-site-packages and PYTHONNOUSERSITE=1. No global Python downgrade, system driver removal or source algorithm change is needed for Python dependency alignment.

Coordinate review: terrain dataset CRS → WGS84 lon/lat/ellipsoid height → ECEF(m) → local NED[north,east,down](m). Existing LocalNedFrame formula and axis order remain unchanged; validate with independent north/east/up perturbations.

## Baseline facts
- Root .venv absent. Shell user packages NumPy1.26.4 and system SciPy1.8.0 are outside SciPy declared compatibility. pyproj absent.
- nvidia-smi fails: loaded kernel module595.91.07 vs NVML/user libraries595.99.02. modinfo reports on-disk module595.99.02 for running kernel6.8.0-138-generic; DKMS reports installed. apt history records the corresponding driver upgrade.
- RTX4050 Laptop GPU is enumerated. Actual FFmpeg NVENC probes fail with unsupported device both on host and inside network isolation, at16×16 and160×64. Network isolation is not the distinguishing cause.
- Reboot or graphics-driver reload is disruptive to this active desktop. No reboot/module unload has been authorized or performed. Complete Python repair and prepare exact post-reboot validation first.

## Changes underway
requirements/optional-terrain.txt supplies reproducible terrain packages; dev requirements include it. The missing-dependency test now injects each ImportError rather than skipping when packages are installed. The NVENC capability probe uses the actual recorder test dimensions160×64. It still skips if encoding genuinely cannot initialize, and its frame/MP4/lossless checks remain intact.

## Evidence ledger
Baseline diagnosis PASS; Python install/compatibility/terrain/full regression PENDING; NVENC repair PENDING external driver activation.

## Python environment repair PASS / focused checks
Canonical .venv provisioned with user-site disabled and ambient PYTHONPATH removed. NumPy1.24.4/SciPy1.8.0 import and scipy.linalg.solve pass with UserWarning treated as error. pyproj3.7.1/rasterio1.4.4 installed. Torch2.13.0 uses matching torchvision0.28.0 instead of the incompatible inherited user pair. pip check initially found missing cffi for Ubuntu PyNaCl; declared cffi in dev requirements and installed2.1.1. pip check now: No broken requirements found.

Terrain+shared coordinate tests:25PASS in0.27s, no skips/warnings. Includes explicit missing-import tests for both terrain libraries. Full suite now running with the canonical, user-site-disabled runner.

GPU state is still unresolved: host and isolated NVENC both fail. On-disk595.99.02 DKMS modules installed for running kernel; initramfs rebuilt2026-10-10; SecureBoot disabled. Live NVIDIA display/modeset modules are in use. No module unload or reboot was performed. Ubuntu official guidance identifies reboot as activation for upgraded userspace/older loaded-module mismatch. Strict post-reboot tool tools/check_workstation_nvenc.py refuses to treat skips as a pass.

## Full canonical regression PASS except pending GPU capability
Command: apps/Workstation/tools/run_isolated_tests.sh -q -ra --tb=short. **2268PASS,1SKIP,0warnings,234.56s,exit0** including teardown. Only remaining skip is actual NVENC initialization failure. Raw evidence: evidence/regression-before-reboot.log. Two terrain dependency injection cases and the real pyproj coordinate case executed.

Setup helper executed successfully a second time; local pip freeze hash unchanged (7c5b28437b2e0085822ed091865f01082da7fc92c31c7b07ee1b159a9d807e3c). pip check clean. Current dependency profile/core versions retained in repo; .venv intentionally ignored and reproducible.

Strict GPU check currently exits1 on driver mismatch and makes no GPU pass claim. No reboot/system-driver action taken. Remaining dependency is explicit user reboot authorization or notice that manual reboot is complete.

Final pre-reboot gates PASS: portable paths,repository policy,whitespace,bash syntax,pip check. Source hashes stable; required setup/requirements/tests/evidence not ignored. Canonical .venv is intentionally ignored. Worktree remains uncommitted. Reboot decision is the only pending user action before actual GPU validation.
