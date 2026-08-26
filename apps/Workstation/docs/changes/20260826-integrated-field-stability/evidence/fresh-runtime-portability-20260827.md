# Fresh runtime and portability audit — 2026-08-27

## Scope

This is a current-host fresh-checkout/fresh-venv qualification, not evidence from a different clean
Ubuntu machine. Docker exists but the operator user cannot access its root-owned daemon and
passwordless sudo is unavailable, so the separate clean-OS/VM gate remains open.

## Repository portability

`python3 tools/check_portable_paths.py` initially reproduced 12 findings:

- four duplicated pattern findings in `apps/Simulation/goal.md` from two machine-specific examples;
- two findings for the active Legacy absolute model setting;
- six findings in calibration/change evidence whose original capture path is immutable provenance.

The active Simulation examples now derive the repository root, and Legacy selects tracked
`yolov8n_uav.pt` by a relative path. The policy now classifies the named calibration artifacts and
change-evidence tree as historical evidence rather than requiring their captured bytes to be
rewritten. Current result: `portable path policy: PASS`. Policy/settings/import regressions: 48
passed; expanded runtime/portable selection after the setuptools fix: 71 passed.

## Fresh checkout and venv

- Temporary checkout: `/tmp/sasl-fresh-checkout-20260827.7WeNXH`.
- Source: current worktree copied without `.git`, old venvs, `.local`, workspace build/install/log,
  recordings, caches, crash logs or Python bytecode.
- Venv creation: `/usr/bin/python3 -m venv --system-site-packages .venv`.
- `pyvenv.cfg`: Python 3.10.12, `include-system-site-packages = true`.
- The approved Workstation `yolov8m.pt` was provisioned separately exactly as the manifest requires:
  52,136,884 bytes and SHA-256
  `5d4a90cdc7a21786cc59cd19778e9eafff836df9e2da32524737c7ee6efe4fe5`.
- Legacy's tracked relative `yolov8n_uav.pt` needed no external checkout path.

The first two online installs timed out while downloading the 526.6 MB Torch wheel. This was a
network transport failure, not dependency resolution. Complete cached PyPI wheel bodies were then
exposed read-only as a temporary wheelhouse. Their internal `.dist-info/METADATA` and `WHEEL` tags
were inspected before use. The same manifest then installed successfully.

During this recreation, Torch selected `setuptools 84`, which conflicts with Ubuntu/ROS
`colcon-core <80`. The runtime manifest had no direct setuptools bound. It now declares
`setuptools>=77,<80`, the intersection of Torch 2.13's `>=77` and colcon's `<80`; the fresh install
selected 79.0.1, matching the working operator venv.

## Independent runtime checks

All of the following ran from inside the temporary checkout and exited zero:

1. `tools/check_operator_runtime.sh --deployment --decoder auto`;
2. `tools/check_operator_runtime.sh --execution --decoder auto`;
3. `apps/Workstation/run.sh --check`;
4. `apps/StereoWIRIS/run.sh --check --mavlink-only --ui legacy`.

The execution doctor verified 15 declared distributions. Workstation resolved the separately
provisioned model, all application imports and Auto→`nvh264dec`. Legacy resolved its relative model,
settings, all imports and Auto→`nvh264dec`. Workstation and Legacy were run again with isolated
`HOME`, `XDG_CONFIG_HOME` and Matplotlib cache and both again exited zero.

The current checkout's Workstation and Legacy checks also exit zero after the new setuptools bound.
The 5.9 GB temporary checkout and temporary cache-wheel symlink directory were deleted after the
hashes and text inventories below were captured; the listed `/tmp` logs remain.

## Artifact hashes

- Final manifest install log:
  `40016d0b802b80f1d365ccaf16e166b87e6b786d3d6af7506768435c659cba76`.
- Four-check log:
  `b1f689112d72ea6208484be308e8ede3ef67ea677553b5d7d33d37ec41ef7965`.
- Isolated-HOME application checks:
  `59bb0559ba20af8cb9bf92408e6794f156a45507994a7086425f3c8a951b119c`.
- Portability PASS log:
  `4e227bc8b985d1ab6f67f83f3d0dcf2084720452637bfcdcf54376f71accc920`.
- 359-line installed distribution inventory:
  `a7325dc5a67b83b5566bda70680ec2cf04950bfde86ee64a330da770313968bf`.
- Fresh `pyvenv.cfg`:
  `53467b25c279c8b93877e01077a0fd75b9b7144ce68fb7aaa8e806f28e8c2a40`.
- Runtime manifest:
  `8e0729347151eb7052373179579290f9fe7ba43e26fdebee3ebc264909cf17de`.

## Remaining R10 boundary

Manifest recreation is now proven in a fresh checkout/venv on the current Ubuntu 22.04.5 host, and
the repository portability gate is clean. A genuinely separate clean Ubuntu host/VM is still
required to prove that no unlisted OS state contributed. It must not be replaced by this same-host
result.
