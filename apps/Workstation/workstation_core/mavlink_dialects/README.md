# Reproducing the SASL MAVLink dialect

From the SASL repository root, using the pinned Workstation test environment:

```bash
.venv/bin/python -B apps/Workstation/tools/regenerate_mavlink_dialect.py --check
```

The command validates the repo-owned XML, generates into a temporary directory, and
compares every byte with `sasl_can_mag_dialect.py` and its recorded SHA-256. For a
preview file, use `--output /tmp/sasl_can_mag_dialect.py`. The canonical dialect name
is preserved even if a different output basename is selected.

`definitions/` contains the exact `common.xml`, `standard.xml`, `minimal.xml`, XSD
and COPYING notice recovered from the firmware source revision recorded in
[generation_manifest.json](generation_manifest.json). The original common XML diff
matches the tracked [SASL message patch](../../firmware/patches/px4-v1.17-sasl-can-mag-mavlink-dialect.patch).
The recorded PX4, MAVLink and bundled-generator revisions are provenance. No separate
PX4 checkout is needed to regenerate the Workstation artifact.

The generator is `pymavlink==2.4.49`, already pinned in the repository's Workstation
constraints. Its relevant Python sources are hash-checked. The recorded firmware XSD
is selected explicitly because the wheel's older XSD rejects `superseded` metadata.
XML validation stays enabled, including all includes; a disabled-validation fallback
fails this command. The wheel's own common XML cannot substitute for these inputs:
it has 205 messages, while this dialect has 235 and additional fields.

The current result is 1,919,084 bytes with SHA-256
`7ed861c4d01a48e1c972cd1936b4410e29f209f1e428178f704564868f4ea2ba`.
The two upstream zero-valued bitmask diagnostics (`CAMERA_TRACKING_STATUS_FLAGS_IDLE`
and `MAV_BOOL_FALSE`) remain visible; they do not disable XML validation.

`SASL_CAN_MAG` is message 180, CRC extra 233. `time_usec` is microseconds since
system boot; `xmag/ymag/zmag` are raw sensor-frame Gauss, without body-FRD or NED
rotation. `can_port=1/2` denotes CAN1/CAN2, independently of the node/device IDs.
All standard message/frame definitions remain byte-identical in the generated
runtime. Generation verifies the software contract; recorded firmware deployment
and hardware observations have their own evidence in the firmware manifests.

For a deliberate protocol change, update the XML and input hashes, generate a
preview, review the field/order/CRC and coordinate changes, then update the runtime
artifact and its manifest hash with corresponding protocol tests. Keep this generated
module cohesive rather than editing or splitting its emitted message classes.
