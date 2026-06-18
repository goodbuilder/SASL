# Coordinate Contract

This document fixes the coordinate-frame and axis naming contract used by StereoDistance. The code source of truth is `wiris_stereo_distance/coordinate_contract.py`; this document is the human-readable version for experiments, reports, and CSV interpretation.

## Frames

| Frame id | Meaning | Axis contract | Use |
| --- | --- | --- | --- |
| `ned` | PX4 world/local frame | `N/E/D = North/East/Down` | ULog and PX4 local-position truth |
| `px4_frd` | PX4 body frame | `Forward/Right/Down` | Pixhawk attitude/body geometry |
| `opencv_optical` | Camera optical frame | `Right/Down/Forward` | Camera rays and per-camera optical vectors |
| `stereo_center_opencv_optical` | Stereo rig center optical frame | `Right/Down/Forward` | YOLO 3D measurement rows before UAM alignment |
| `stereo_center_frd` | Stereo rig center comparison frame | `Forward/Right/Down` | Simulation and UAM camera-axis comparison |
| `px4_local_ned` | UAM/Pixhawk local truth frame | `North/East/Down` | Truth and aligned estimated output |

## Fixed Mount

The nominal front camera mount profile is:

```text
front_camera_z_to_body_x_x_to_body_y
```

This means:

```text
camera +X Right   -> body +Y Right
camera +Y Down    -> body +Z Down
camera +Z Forward -> body +X Forward
```

Do not swap axes ad hoc in analysis code. Convert through the declared frame contract or through a saved UAM alignment profile.

## Measurement Fields

Raw YOLO stereo-center measurements are camera-frame values:

| Field | Meaning |
| --- | --- |
| `measurement_right_m`, `stereo_center_right_m`, legacy `x_m` | Camera/stereo-center Right |
| `measurement_down_m`, `stereo_center_down_m`, legacy `y_m` | Camera/stereo-center Down |
| `measurement_forward_m`, `stereo_center_forward_m`, legacy `z_m` | Camera/stereo-center Forward |

Legacy `x_m/y_m/z_m` fields remain for CSV compatibility only. In current UI and reports, describe them as `Right/Down/Forward`.

When an alignment profile is active, PX4 local truth can be transformed back
into the camera-center frame for per-axis FRD comparison:

| Field | Meaning |
| --- | --- |
| `measurement_frd_forward_m` | YOLO measurement in camera-center Forward |
| `measurement_frd_right_m` | YOLO measurement in camera-center Right |
| `measurement_frd_down_m` | YOLO measurement in camera-center Down |
| `truth_frd_forward_m` | Ground truth transformed to camera-center Forward |
| `truth_frd_right_m` | Ground truth transformed to camera-center Right |
| `truth_frd_down_m` | Ground truth transformed to camera-center Down |
| `error_frd_forward_m` | `measurement_frd_forward_m - truth_frd_forward_m` |
| `error_frd_right_m` | `measurement_frd_right_m - truth_frd_right_m` |
| `error_frd_down_m` | `measurement_frd_down_m - truth_frd_down_m` |
| `truth_rdf_right_m` | Ground truth transformed to camera-center Right |
| `truth_rdf_down_m` | Ground truth transformed to camera-center Down |
| `truth_rdf_forward_m` | Ground truth transformed to camera-center Forward |
| `error_rdf_right_m` | `measurement_right_m - truth_rdf_right_m` |
| `error_rdf_down_m` | `measurement_down_m - truth_rdf_down_m` |
| `error_rdf_forward_m` | `measurement_forward_m - truth_rdf_forward_m` |

Use the FRD fields for camera-frame axis-level error. The RDF fields remain
compatibility aliases for older analysis code. The scalar range metric remains
`estimated_range_m - truth_range_m`.

## Truth Fields

PX4 ULog, ROS2 `vehicle_local_position`, and MAVLink `LOCAL_POSITION_NED` expose local-position values as `x/y/z`, but StereoDistance interprets and displays them as:

| Field | Meaning |
| --- | --- |
| `truth_north_m`, legacy `truth_x_m` | PX4 local North |
| `truth_east_m`, legacy `truth_y_m` | PX4 local East |
| `truth_down_m`, legacy `truth_z_m` | PX4 local Down |

For recorded rows, `drone_north/east/down_m` are preferred over legacy `drone_x/y/z_m`.

Truth source collection and profile-source selection are separate:

- Collect every available source when possible: PX4 local NED,
  PX4/global LLA, and GPS LLA.
- Build or evaluate one profile with exactly one selected truth source contract:
  `local_ned`, `global_position`, or `gps_position`.
- Do not mix local/global/GPS truth rows inside one calibration profile.

Default analysis-source priority is:

1. ULog local NED: `vehicle_local_position`, then `vehicle_odometry`, then `vehicle_visual_odometry`.
2. ULog global/GPS LLA converted to local NED: `vehicle_global_position`,
   `vehicle_gps_position`, then `sensor_gps`.
3. Live local/global/GPS telemetry converted to local NED.
4. Recorded-row local NED fallback: `drone_north/east/down_m`.

GPS `lat/lon/alt` must never be compared directly with camera-frame data. It
must first be converted to local NED using an explicit reference origin. The
reference priority is PX4 local-position reference (`ref_lat/ref_lon/ref_alt`),
then `home_position`, then the first valid GPS/global sample. Rows produced
from GPS/global-derived truth must retain the reference metadata:

| Field | Meaning |
| --- | --- |
| `truth_gps_reference_lat_deg` | GPS-to-NED reference latitude |
| `truth_gps_reference_lon_deg` | GPS-to-NED reference longitude |
| `truth_gps_reference_alt_m` | GPS-to-NED reference altitude |
| `truth_gps_reference_kind` | `vehicle_local_position_ref`, `home_position`, `first_valid_gps_sample`, or `first_valid_global_sample` |
| `truth_gps_altitude_reference` | Altitude datum when known; otherwise `unknown` |

## UAM Comparison Metrics

Before coordinate alignment, YOLO `Right/Down/Forward` and truth `North/East/Down` are a `mixed-frame` comparison. Mixed-frame axis errors are diagnostic only.

After loading or estimating a UAM coordinate-alignment profile, final comparison fields are:

| Field | Meaning |
| --- | --- |
| `estimated_north_m` | YOLO point transformed into PX4 local North |
| `estimated_east_m` | YOLO point transformed into PX4 local East |
| `estimated_down_m` | YOLO point transformed into PX4 local Down |
| `error_north_m` | `estimated_north_m - truth_north_m` |
| `error_east_m` | `estimated_east_m - truth_east_m` |
| `error_down_m` | `estimated_down_m - truth_down_m` |
| `position_error_m` | Euclidean position error |
| `alignment_profile_id` | Applied axis/origin correction profile |

For PX4 local-frame reporting, use aligned `estimated_north/east/down_m` versus
`truth_north/east/down_m`. For camera-frame reporting, use
`error_frd_forward/right/down_m`.

## Alignment Profile Rules

A UAM alignment profile stores:

- rotation matrix: `camera Right/Down/Forward -> PX4 local North/East/Down`
- origin offset: `origin_offset_north/east/down_m`
- sample count, RMSE, max error, method, frame ids

Use a saved profile in the main experiment only when camera, zoom/intrinsics, Pixhawk/GNSS position, and UAM mounting geometry are unchanged. If any of those change, create a new profile.

At least three non-collinear matched samples are required. A line-only motion is degenerate and must not be used to lock a final axis/origin profile.
