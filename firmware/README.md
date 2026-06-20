# PX4 Firmware DDS Export

This directory stores reproducible firmware work for the SASL Pixhawk setup.
Do not commit a full `PX4-Autopilot` checkout here. Keep the checkout local and
commit only patches, notes, and small scripts.

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
source /home/moon/px4_msgs_ws/install/setup.bash
ros2 topic list | rg 'sensor_gps|sensor_mag|vehicle_magnetometer'
ros2 topic echo --once /uav_1/fmu/out/sensor_gps_1
ros2 topic echo --once /uav_1/fmu/out/sensor_mag_4
```

## 2026-06-21 Hardware Finding

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

## Coordinate Contract

`sensor_gps_*` uses PX4 `SensorGps` fields and units as exported by
`px4_msgs/msg/SensorGps`; latitude/longitude are geodetic degrees and altitude
fields are meters. `sensor_mag_*` uses PX4 `SensorMag` raw sensor axes and
`device_id`; the patch does not transform axes or convert frames. It only
exports existing PX4 uORB instances through DDS.
