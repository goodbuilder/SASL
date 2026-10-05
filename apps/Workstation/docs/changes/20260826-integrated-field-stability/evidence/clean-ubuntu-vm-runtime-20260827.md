# Clean Ubuntu VM runtime qualification — 2026-08-27

## Result and boundary

R10 is **PASS** on a separate clean Ubuntu OS guest. This closes repository/runtime portability;
it does not replace any camera, Pixhawk, NTRIP/RTK, STUS, MP4, cold-boot, or 60-minute field gate.
The guest had no device or GPU passthrough, so Auto decoder fallback was verified against software
GStreamer while the physical operator host remains the NVIDIA boundary.

## Independent OS and source identity

- Image: Ubuntu Jammy current cloud image,
  `jammy-server-cloudimg-amd64.img`, from
  `https://cloud-images.ubuntu.com/jammy/current/jammy-server-cloudimg-amd64.img`.
- Published and actual image SHA-256:
  `c0a5af17e6c0f76351fe07e2fffef3011dab1facb8a8ed5701dcf648dabd4f0a`.
- Guest: Ubuntu 22.04.5 LTS, kernel `5.15.0-190-generic`; cloud-init ended `done` with no errors or
  recoverable errors.
- Hypervisor: KVM/QEMU 6.2, 4 vCPU, 6 GiB RAM, 35 GiB copy-on-write guest disk. The guest root was
  its own ext4 filesystem; host source and cached package bytes were exposed read-only through 9p.
- Product source: pushed commit `132c18544158368fc89ac98dc457b8e7a1341f9c`, with local and remote
  branch heads equal before export.
- Commit archive: 75,985,539 bytes, SHA-256
  `f627c323e4ba03b588068209ed56cefaa1af94e6195ce1396d39aa856dfb8551`.
- The archive contained no `.git` directory. All 2,679 archived regular files were independently
  hashed after extraction and matched the `/opt/SASL` runtime tree; the file-inventory artifact
  SHA-256 is `b736d02783d65a25d85f1d9a2d4698aec287f35a9ac58634ee7efbf8e5737197`.

The two intentionally untracked models were provisioned as external artifacts, not inherited from
the host installation:

| Consumer | Bytes | SHA-256 | Tracked authority |
|---|---:|---|---|
| Workstation `yolov8m.pt` | 52,136,884 | `5d4a90cdc7a21786cc59cd19778e9eafff836df9e2da32524737c7ee6efe4fe5` | `apps/Workstation/model-manifest.json` |
| Legacy `yolov8n_uav.pt` | 6,269,475 | `8a2204bf2ed7c09b9dbabeaa1c1e984ab482c10325e2387150e23d8daa3cb777` | `apps/StereoWIRIS/model-manifest.json` |

## Clean installation and RED findings

The repository Ubuntu manifest was installed with apt, then a new Python 3.10.12 venv was created
with `include-system-site-packages = true`. Only `requirements/runtime.txt` was installed into that
venv. Network downloads of the 526.6 MB Torch wheel were unreliable, so complete cached wheel
bodies were exposed read-only. Their internal `.dist-info/METADATA` and every `WHEEL` tag were
inspected before 293 compatible aliases were presented to pip. The wheel-alias manifest SHA-256 is
`b6fc9fe752bdda494650407c61003b48b67183d16c5cddd28693652d23854810`.

The clean guest exposed two real deployment defects:

1. `pip check` reported `pygobject 3.42.1 requires pycairo, which is not installed`. The system
   manifest now owns `python3-cairo`; after installation, `pip check` reported no broken
   requirements.
2. Legacy failed because `yolov8n_uav.pt` was ignored and absent from the commit archive despite
   documentation calling it bundled. The bytes remain correctly external, but a tracked manifest,
   explicit provisioning step, and size/SHA-256 runtime validation now own the boundary.

The final runtime manifest SHA-256 is
`8e0729347151eb7052373179579290f9fe7ba43e26fdebee3ebc264909cf17de`; the final Ubuntu manifest
SHA-256 is `e26fc0738353a8d06b35f03ce738af0c83b24fc04d954bb77f4ae9cb2c953fbb`.
The pip installation log SHA-256 is
`bbea60e56fceae3188ae3cca0a015304fe7b8c3293113e8260d6899d6edfaf15`, and the 123-line installed
distribution inventory SHA-256 is
`4f71b5e5faba2ec91d1b77550cee54a45ff62d03a6549c3bc2d44970562ccf68`.

Key selected versions were PyQt5 5.15.11, NumPy 1.24.4, SciPy 1.10.1, OpenCV 4.11.0,
PyArrow 23.0.1, pymavlink 2.4.49, setuptools 79.0.1, packaging 26.3, Ultralytics 8.4.127, and
Torch 2.13.0.

## Final checks

All commands below ran from the exact final commit tree and exited zero:

1. `.venv/bin/python -m pip check`;
2. `tools/check_operator_runtime.sh --deployment --decoder software`;
3. `tools/check_operator_runtime.sh --execution --decoder software`;
4. `apps/Workstation/run.sh --check`;
5. `apps/StereoWIRIS/run.sh --check --mavlink-only --ui legacy`;
6. both application checks again with new isolated `HOME`, `XDG_CONFIG_HOME`, and Matplotlib cache.

The deployment doctor verified all 11 Ubuntu packages, including `python3-cairo`. The guest has no
NVIDIA driver or `nvh264dec`, so explicit Software selected `avdec_h264`, while each application
Auto check warned and selected the same working software decoder. This is expected degradation, not
a false NVIDIA PASS. Workstation verified the approved YOLOv8m bytes; Legacy verified the approved
YOLOv8n UAV bytes and every required application import.

As an independent negative boundary test, the first byte of the guest's Legacy model was changed
without changing its size. The real application checker returned exit 1 with
`YOLO model SHA-256 does not match model-manifest.json`. Restoring the read-only approved bytes made
the complete Legacy launcher pass. The tamper log SHA-256 is
`dd7374ff00154f925e4bace6fb06dc77cb074f457d75228be88c2bef350ecb12`.

The complete final-check log is 18,683 bytes with SHA-256
`268578b352e3db6a276fee3832ec9b12d6103154a4993361e3a692ed445ee27d`.
After evidence capture the guest powered off cleanly; the recoverable qcow2 overlay was retained.

## Regression evidence and residual risk

- New/adjacent model and runtime contract tests: **34 passed**.
- Legacy tests excluding the known monolithic native-Qt order residual: **421 passed**.
- Complete Workstation suite: **1466 passed, 1 skipped**.
- Portable-path audit: **PASS**, zero findings.

R10 is therefore closed. R1/R2/R3/R7 remain blocked by the physical FW 2.18.72 zero-IDR sender;
open-sky dual RTK Fixed, three independent cold boots, and final 60-minute integrated acceptance
remain external field work.
