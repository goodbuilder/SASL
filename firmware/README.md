# PX4 Firmware DDS Export

This directory stores reproducible firmware work for the SASL Pixhawk setup.
Do not commit a full `PX4-Autopilot` checkout here. Keep the checkout local and
commit only patches, notes, small scripts, and selected flashable `.px4`
artifacts that need to be shared with the SASL project.

Tracked flashable artifacts:

- `firmware/artifacts/px4_fmu-v6x_default/px4_fmu-v6x_default.px4`
  - FMU v6X custom PX4 v1.17-alpha build with SASL GPS/MAG DDS export patches.
  - SHA256:
    `63410eca73fc864770afee773e6e41bd6b7787fd802a3df1f7ff73720b699ea8`

## PX4 1.16 GPS2/MAG DDS Patch

Patch:

```sh
firmware/patches/px4-1.16-uxrce-sensor-gps-mag-instances.patch
```

Purpose:

- Export raw GPS instances as DDS topics:
  - `/fmu/out/sensor_gps_0`
  - `/fmu/out/sensor_gps_1`
- Export raw magnetometer instances as DDS topics:
  - `/fmu/out/sensor_mag_0`
  - `/fmu/out/sensor_mag_1`
  - `/fmu/out/sensor_mag_2`
  - `/fmu/out/sensor_mag_3`
  - `/fmu/out/sensor_mag_4`
- Export selected fused/voted magnetometer data:
  - `/fmu/out/vehicle_magnetometer`

The patch extends PX4's uXRCE-DDS topic generator with optional `uorb_topic`
and `uorb_instance` keys. This allows one uORB multi-instance source such as
`sensor_mag` to be published under several DDS topic names.

## Apply

From a PX4-Autopilot 1.16 checkout:

```sh
git checkout release/1.16
git apply /home/moon/Desktop/SASL/firmware/patches/px4-1.16-uxrce-sensor-gps-mag-instances.patch
```

## Build And Flash v6X

The local machine uses the user-local ARM toolchain and the SASL simulation
venv Python:

```sh
cd /home/moon/Desktop/SASL/firmware/PX4-Autopilot
PATH=/home/moon/tools/gcc-arm-none-eabi-10.3-2021.10/bin:$PATH \
PYTHON_EXECUTABLE=/home/moon/Desktop/SASL/Simulation/.venv/bin/python \
make px4_fmu-v6x_default
```

Flash a USB-connected FMU v6X:

```sh
PATH=/home/moon/tools/gcc-arm-none-eabi-10.3-2021.10/bin:$PATH \
PYTHON_EXECUTABLE=/home/moon/Desktop/SASL/Simulation/.venv/bin/python \
make px4_fmu-v6x_default upload
```

Expected build check:

```sh
strings build/px4_fmu-v6x_default/px4_fmu-v6x_default.elf \
  | rg '/fmu/out/(sensor_gps|sensor_mag|vehicle_magnetometer)'
```

Expected entries include `sensor_gps_0`, `sensor_gps_1`,
`sensor_mag_0` through `sensor_mag_4`, and `vehicle_magnetometer`.

## Runtime Checks

PX4 shell checks:

```sh
uxrce_dds_client status
listener sensor_gps 2
listener sensor_mag 5
listener vehicle_magnetometer 1
```

ROS2 checks:

```sh
source /opt/ros/humble/setup.bash
source workspace/main/install/setup.bash
ros2 topic list | rg 'sensor_gps|sensor_mag|vehicle_magnetometer'
ros2 topic echo --once /uav_1/fmu/out/sensor_gps_1
ros2 topic echo --once /uav_1/fmu/out/sensor_mag_4
```

## 2026-06-21 Hardware Finding - Pixhawk 1

The USB-connected FMU v6X was successfully flashed with the patched firmware:

- PX4 git hash: `9ca30ebf2c1daef30c806c60e3be3008d5766e6b`
- PX4 branch: `release/1.16`
- Build datetime reported by PX4 shell: `Jun 21 2026 00:55:14`
- `listener sensor_gps 2` showed two GPS instances.
- `listener sensor_mag 5` showed five magnetometer instances, including two
  UAVCAN magnetometers at instances 3 and 4.

However, that USB board reported:

```text
uxrce_dds_client: Running, disconnected
Agent IP: 10.0.0.10
Agent port: 8888
```

The Ethernet ROS2/uXRCE traffic on the bench was coming from `10.0.0.2` and
`10.0.0.3`, not from the USB-flashed board. The Micro XRCE-DDS Agent log also
showed both Ethernet clients using the same client key:

```text
client_key: 0x00000002, address: 10.0.0.2:...
client_key: 0x00000002, address: 10.0.0.3:...
```

For stable dual-Pixhawk ROS2 operation, flash this patched firmware onto the
actual Ethernet-connected Pixhawks and set unique DDS keys, normally:

- left/Pixhawk 1: `UXRCE_DDS_KEY=1`, ROS namespace `/uav_0`
- right/Pixhawk 2: `UXRCE_DDS_KEY=2`, ROS namespace `/uav_1`

Then restart the Micro XRCE-DDS Agent and reboot or restart
`uxrce_dds_client` on both Pixhawks.

## 2026-06-21 Hardware Finding - Pixhawk 2

The USB-connected Pixhawk 2 was flashed with the same patched DDS export
firmware after entering the bootloader with `reboot -b`.

- PX4 git hash: `9ca30ebf2c1daef30c806c60e3be3008d5766e6b`
- PX4 branch: `release/1.16`
- Build datetime reported by PX4 shell: `Jun 21 2026 01:44:58`
- `UXRCE_DDS_KEY=2`
- uXRCE-DDS status: `Running, connected`
- Agent IP/port: `10.0.0.10:8888`
- ROS namespace: `/uav_1`

Runtime checks after flashing:

- `/uav_1/fmu/out/sensor_gps_0`: publisher present, `device_id=11141125`
- `/uav_1/fmu/out/sensor_gps_1`: publisher present, `device_id=11141181`
- `/uav_1/fmu/out/sensor_mag_3`: publisher present, `device_id=8944643`
- `/uav_1/fmu/out/sensor_mag_4`: publisher present, `device_id=8944387`
- `/uav_1/fmu/out/vehicle_magnetometer`: publisher present, selected
  `device_id=396817`

Pixhawk 2 uses RM3100 device ID `8944643` for the primary RM3100 where
Pixhawk 1 used `8944899`. The StereoWIRIS app treats both as known RM3100
primary IDs and maps them to alignment `CAN1`; `8944387` maps to `CAN2`.

## Coordinate Contract

`sensor_gps_*` uses PX4 `SensorGps` fields and units as exported by
`px4_msgs/msg/SensorGps`; latitude/longitude are geodetic degrees and altitude
fields are meters. `sensor_mag_*` uses PX4 `SensorMag` raw sensor axes and
`device_id`; the patch does not transform axes or convert frames. It only
exports existing PX4 uORB instances through DDS.

## PX4 v1.17 GPS2/MAG DDS Patch

Patch:

```sh
firmware/patches/px4-v1.17-uxrce-sensor-gps-mag-instances.patch
```

This is the v1.17-alpha/local-main equivalent of the 1.16 GPS2/MAG DDS
export patch. PX4 v1.17 already carries an `instance` path in the uXRCE-DDS
generator, but StereoWIRIS needs stable DDS names such as
`/fmu/out/sensor_gps_0` while subscribing to the underlying `sensor_gps`
uORB instance. The patch adds `uorb_topic` and `uorb_instance` aliases so DDS
topic names and uORB source names can differ.

Apply from any local PX4-Autopilot checkout:

```sh
cd /path/to/PX4-Autopilot
git apply /path/to/SASL/firmware/patches/px4-v1.17-uxrce-sensor-gps-mag-instances.patch
```

Build and flash:

```sh
make px4_fmu-v6x_default
make px4_fmu-v6x_default upload
```

Or flash the tracked custom artifact through QGroundControl's custom firmware
file flow:

```text
firmware/artifacts/px4_fmu-v6x_default/px4_fmu-v6x_default.px4
```

Expected build check:

```sh
strings build/px4_fmu-v6x_default/px4_fmu-v6x_default.elf \
  | rg '/fmu/out/(sensor_gps|sensor_mag|vehicle_magnetometer)'
```

Expected entries include `sensor_gps_0`, `sensor_gps_1`,
`sensor_mag_0` through `sensor_mag_4`, and `vehicle_magnetometer`.

## 2026-06-24 Hardware Finding - Pixhawk 1 v1.17 Patch

The USB-connected FMU v6X was flashed from:

- PX4 checkout: local PX4-Autopilot checkout previously stored under
  `/home/moontaehyung/Capstone/source/PX4-Autopilot`
- PX4 git describe: `v1.17.0-alpha1-1214-g7b1e54889d-dirty`
- Firmware artifact:
  `firmware/artifacts/px4_fmu-v6x_default/px4_fmu-v6x_default.px4`
- Build timestamp: `2026-06-24 06:10:22 KST`
- USB serial reported by uploader: `002100263433510636363338`

Build evidence:

- `strings build/px4_fmu-v6x_default/px4_fmu-v6x_default.elf | rg '/fmu/out/(sensor_gps|sensor_mag|vehicle_magnetometer)'`
  showed `/fmu/out/sensor_gps_0`, `/fmu/out/sensor_gps_1`,
  `/fmu/out/sensor_mag_0..4`, and `/fmu/out/vehicle_magnetometer`.

Runtime evidence after upload:

- `/uav_0/fmu/out/sensor_gps_0`
- `/uav_0/fmu/out/sensor_gps_1`
- `/uav_0/fmu/out/sensor_mag_0`
- `/uav_0/fmu/out/sensor_mag_1`
- `/uav_0/fmu/out/sensor_mag_2`
- `/uav_0/fmu/out/sensor_mag_3`
- `/uav_0/fmu/out/sensor_mag_4`
- `/uav_0/fmu/out/vehicle_magnetometer`

At that checkpoint `/uav_1` still showed only `vehicle_attitude`, so Pixhawk 2
still needed the same firmware upload.

## 2026-06-24 Hardware Finding - Pixhawk 2 v1.17 Patch

The USB-connected Pixhawk 2 did not enter bootloader through the normal
`make ... upload` reboot loop. It was reachable as MAVLink system `2`, so it
was put into bootloader with `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` parameter 1
set to `3`, then flashed directly with `Tools/px4_uploader.py`.

- USB serial reported by uploader: `0049002b3433510336363338`
- Firmware artifact:
  `firmware/artifacts/px4_fmu-v6x_default/px4_fmu-v6x_default.px4`
- Upload result: complete, 100% verified

Runtime evidence after upload:

- `/uav_1/fmu/out/sensor_gps_0`
- `/uav_1/fmu/out/sensor_gps_1`
- `/uav_1/fmu/out/sensor_mag_0`
- `/uav_1/fmu/out/sensor_mag_1`
- `/uav_1/fmu/out/sensor_mag_2`
- `/uav_1/fmu/out/sensor_mag_3`
- `/uav_1/fmu/out/sensor_mag_4`
- `/uav_1/fmu/out/vehicle_magnetometer`

Sample checks:

- `/uav_1/fmu/out/sensor_gps_1`: `device_id=11141181`, `fix_type=0`
  at the indoor/no-fix checkpoint.
- `/uav_1/fmu/out/sensor_mag_4`: `device_id=8944395`, raw vector
  `x=-0.99267578125`, `y=-1.3486328125`, `z=-0.70849609375`.
