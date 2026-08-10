# StereoWIRIS Coordinate Contract

This document fixes the coordinate-frame and axis naming contract used by
StereoWIRIS. The code source of truth is
`wiris_stereo_distance/coordinate_contract.py`.

## Active frames

| Frame id | Meaning | Axis contract | Active use |
| --- | --- | --- | --- |
| `ned` | PX4/GNSS navigation frame | `North/East/Down` | P1/P2 attitude and antenna-baseline geometry |
| `px4_frd` | PX4 body frame | `Forward/Right/Down` | P1/P2 body attitude and nominal camera mount |
| `camera_optical_rdf` | Each camera optical frame | `Right/Down/Forward` | Pixel rays before conversion into the left frame |
| `left_camera_rdf` | Left optical-center frame | `Right/Down/Forward` | Internal common geometry and explicit source-point diagnostics |
| `baseline_center_frd` | Midpoint of the left optical origin and selected right-origin estimate | `Forward/Right/Down` | Raw/public StereoWIRIS point and range |

`NED`, `FRD`, and optical `RDF` are not aliases. Values cross these frames
only through the explicit attitude, mount, and baseline transforms used by the
triangulation path.

## GNSS correction-operation contract

The three GNSS operation modes share one correction transport contract but
have different ownership of the Base receiver:

| Mode | Correction path | Coordinate operation | Measurement prerequisite |
| --- | --- | --- | --- |
| `LOCAL_SURVEY_IN` | Base serial RTCM output → selected rover MAVLink | Base position is established by u-blox Survey-in | Survey-in valid, RTCM routed, selected rovers RTK Fixed and fresh |
| `NETWORK_ASSISTED_FIXED_BASE` | NTRIP caster RTCM → Base serial → rover MAVLink | Base first acts as a corrected rover; repeated Fixed WGS84 coordinates are validated, then Base is set to fixed | `CFG-TMODE3` ACK, matching ECEF readback, fresh post-config Base RTCM, selected rovers RTK Fixed and fresh |
| `NETWORK_RTK_DIRECT` | NTRIP caster RTCM → selected rover MAVLink | No Base coordinate is inferred or written | RTCM routed, selected rovers RTK Fixed and fresh |

NTRIP RTCM is opaque correction data. It is never interpreted as a stereo
baseline and is never written into `ned`, `enu`, `baseline_center_frd`, or a
camera optical frame. Rover GPS positions remain WGS84 geodetic input fields;
the existing `rtk_geometry` path derives the selected rover antenna delta as
`North/East/Down` metres.

The fixed-base initializer accepts only repeated samples with one known height
datum. The current `CFG-TMODE3 fixed` adapter accepts WGS84 ellipsoid height
only and converts geodetic latitude/longitude/ellipsoid height to ECEF metres,
then to the u-blox ECEF centimetre and high-part fields. MSL samples are
reported but fail closed until a geoid conversion is supplied; they are never
silently treated as ellipsoid height.

Operation provenance records the selected mode, caster/mountpoint or serial
source identity, GGA policy, fixed-coordinate profile hash, and readiness
reasons. Raw u-blox `NAV-PVT` RTK Fixed means `fixType` 3/4 with
`gnssFixOK=1` and `carrSoln=2`; this is distinct from PX4/MAVLink rover
`fix_type=6`. A correction stream being connected is not itself
measurement-ready.

Mode, GGA policy, height datum, serial/caster identity, and rover routes are
latched at operation start. They require stop/restart to change. Every enabled
readiness threshold fails closed when its value is missing; explicitly disabling
the gate only allows an unasserted GNSS measurement and leaves the failed
readiness result plus `gate_disabled` warning in metadata.

## Active measurement method

The only runtime measurement method is `two_ray_closest_midpoint`.

- Purpose: estimate a 3-D target point from synchronized left/right detector
  pixels without Ground Truth or target telemetry.
- Pixel input: bbox-center pixels on the same pixel grids as the selected
  intrinsics, with distortion-aware ray construction when distortion is
  configured.
- Pose input: either checkerboard stereo `R/T`, or P1/P2 Pixhawk attitudes plus
  a GPS/RTK antenna baseline transformed through the nominal camera mount.
- Accepted approximation: the GPS/RTK antenna baseline may stand in for the
  optical-center baseline. Its source and quality remain recorded; this is not
  an exact lever-arm calibration claim.
- Algorithm: express the right ray and right optical-center origin, or its
  recorded GPS/RTK antenna-baseline proxy, in `left_camera_rdf`, find the
  closest points on the two positive rays, then transform the result into the
  attempt-frozen baseline-center frame.
- Baseline-center origin: midpoint of the left optical origin and selected
  right-origin estimate. Checkerboard `R/T` supplies the optical-center
  estimate; Pixhawk uses the explicitly recorded antenna-baseline proxy without
  claiming a calibrated lever arm.
- Baseline-center axes: `Right` is the normalized left-to-right center vector;
  `Forward` is the normalized mean of both optical-forward vectors projected
  orthogonal to `Right`; `Down = normalize(Forward × Right)`.
- Output: `[Forward, Right, Down, Range]` in `baseline_center_frd`, with
  `Range = norm([Forward, Right, Down])` from the midpoint origin.
- The transform uses metres, preserves ray gap/depth/angle geometry, and
  recomputes range after the origin change.
- Missing or mismatched intrinsics grids, missing pose/translation, zero
  baseline, near-parallel rays, non-positive depth, and non-finite range fail
  closed and retain a rejection reason.

StereoWIRIS has no measurement-correction or tracking-filter layer. The
former BBox reprojection, affine bbox-center, distance-binned 3-D bias, and
Kalman paths were removed. Runtime and newly saved measurement rows contain
the Raw `baseline_center_frd` result only. The corresponding
`source_left_camera_rdf_*` values may be retained as diagnostics; they are not
public output aliases.

### Frame instance scope

StereoWIRIS freezes one `baseline_center_frd` instance per immutable
`MeasurementAttempt`. This is necessary because the Pixhawk path selects a
left/right GPS baseline and two attitudes for the detection pair. Each v2 row
therefore records `baseline_frame_scope=measurement_attempt` and a
`baseline_frame_instance_id`. Coordinates from different rows must not be
subtracted as if they shared one world-fixed origin. Simulation uses the same
origin/axis construction formula but may keep one nominal frame for a whole
static-rig run.

### Coherent measurement input

Each newly selected detection pair receives a generation/session-scoped,
monotonically increasing `measurement_attempt_id`; `MeasurementAttempt.pair_id`
combines that generation identity and sequence instead of using a timestamp
alone. Before geometry runs, StereoWIRIS
copies the pair, nearest left/right attitude samples, selected left/right GPS
samples, applied left/right K/D, zoom/profile identity, and geometry settings
into one immutable `MeasurementAttempt`. Repeated GUI refreshes reuse a
geometry-ready attempt. An incomplete attempt is replaced only when the same
pair gains strictly more required inputs, after which the ready snapshot is
frozen.

Detection timestamp metadata is projected to
`DETECTION_METADATA_ALLOWLIST` and both the observation factory and direct
snapshot constructor use the same canonicalization. Stored values are limited
to immutable `None`, `str`, `bool`, `int`, or finite `float` scalars; unknown or
duplicate direct-constructor keys, mutable values, and NaN/Inf are rejected
when the snapshot is created. Each metadata item must be an exact two-element
`tuple` or `list`; mappings, sets, strings, bytes, and arbitrary iterables are
never interpreted as pairs. Direct construction accepts only `None`, a tuple,
or a list as the outer container, while the observation factory materializes a
runtime `Mapping.items()` sequence before applying the same pair validation.
The active payload adapter first validates `None`, `Mapping`, or exact
tuple/list outer containers and exact two-element tuple/list items, then
materializes a detached plain dictionary; it never passes an unchecked
iterable to `dict()`. A single whole-dictionary `deepcopy` deeply detaches
ordinary deepcopy-compatible/plain metadata graphs before snapshot projection.
Copy exceptions propagate fail-closed with no shallow fallback; arbitrary
custom `__deepcopy__` behavior is not an unconditional trust guarantee. The
snapshot canonicalizer independently revalidates the allowlist and scalar
contract and stores metadata in allowlist order.
`MeasurementAttempt` accepts only exact base `DetectionSnapshot` instances, and
the snapshot factory explicitly rejects subclasses so an overridden
`__post_init__()` cannot bypass provenance validation. The legacy attempt
canonical SHA-256 remains a full, profile-path-sensitive integrity hash of one
serialized attempt instance. The content SHA-256 excludes exactly the top-level
`attempt_id`, `attempt_session_id`, `created_monotonic_s`, and `created_wall_s`
values plus nested `intrinsics.selected_profile_path`; it is a path-portable
semantic hash, not a privacy boundary or physical detection-pair identity. The
instance SHA-256 binds that content hash to the full serialized instance.

Correspondence is fail-closed before attempt construction: left/right classes
must match, and either side with multiple candidates of that selected class is
rejected because this phase does not introduce a validated multi-object
matcher. Independent highest-confidence detections are not triangulated when
that correspondence is ambiguous.

For the Pixhawk path, the right ray is rotated into the selected left-camera
basis with the attempt's left quaternion, and the selected GPS pair's NED
baseline is transformed into `left_camera_rdf` with that exact same left
quaternion and selected app-monotonic attitude timestamp. The quaternion order
is `(w,x,y,z)`. The selected right attitude also places the right optical
forward axis in the same left-camera basis before the baseline-center FRD axes
are built. Geometry and valid-row recording do not re-read GUI latest
attitude, GPS baseline, or intrinsics after the attempt exists. Checkerboard
attempts instead use the applied stereo `R/T`; available Pixhawk attitude and
GPS antenna snapshots are retained only as recording provenance and are not
consumed by checkerboard geometry.

This coherence rule does not change validity policy: stale attitude remains
display-only and non-blocking, RTK Fixed is not newly required, and no ray-gap
or intersection-angle acceptance threshold is added in this phase.

S3-B4 migrates the public origin/axis contract to `baseline_center_frd` and the
paired-row schema to v2. This does not approve hardware reliability or exact
camera/GNSS lever arms. GPS age/skew, stale/RTK acceptance, recorder/session
isolation, and live hardware validation remain separate qualification gates.

## P1/P2 Pixhawk data boundary

P1/P2 Pixhawk data describes the stereo measurement rig. It may provide:

- body orientation in `px4_frd` relative to `ned`;
- GPS/RTK antenna positions used to form the left-to-right baseline;
- GNSS, magnetometer, connection, and timing quality diagnostics.

It does not provide target/UAM Ground Truth in the active StereoWIRIS
application. The application does not ingest a target PX4 local position,
load a target ULog, build a `left_camera_rdf -> px4_local_ned` comparison
profile, or calculate target truth/error fields.

### Attitude timing

For the Pixhawk geometry path, `left_attitude_age_s` and
`right_attitude_age_s` are absolute offsets between the bbox-pair midpoint on
the application monotonic receive-time axis and the nearest attitude samples.
`attitude_pair_delta_s` is the absolute left/right attitude-sample difference
on that same axis.

- `FRESH`: sample age at most 2 s and pair delta at most 1 s.
- `STALE`: either limit is exceeded.
- `VERY_STALE`: sample age exceeds 10 s or pair delta exceeds 5 s.

This `display_only_non_blocking` classification is provenance. The selected
attitudes are still geometry inputs, but the stale-status classification does
not add another rotation, translation, correction, or rejection step.
Checkerboard `R/T` does not consume Pixhawk attitude and is recorded as
`NOT_USED` (shown as `N/A` in the UI).

## Fixed mount

The nominal front-camera mount profile is:

```text
front_camera_z_to_body_x_x_to_body_y
```

Its axis mapping is:

```text
camera +X Right   -> body +Y Right
camera +Y Down    -> body +Z Down
camera +Z Forward -> body +X Forward
```

Do not swap axes ad hoc. Convert through the declared mount, attitude, and
baseline transforms.

## Measurement fields

Current v2 Raw measurements use the following meanings:

| Field | Meaning |
| --- | --- |
| `baseline_center_forward_m`, `raw_baseline_center_forward_m` | Forward from the selected baseline-frame midpoint |
| `baseline_center_right_m`, `raw_baseline_center_right_m` | Right from the selected baseline-frame midpoint |
| `baseline_center_down_m`, `raw_baseline_center_down_m` | Down from the selected baseline-frame midpoint |
| `range_m`, `baseline_center_range_m`, `raw_baseline_center_range_m` | Euclidean range from the selected baseline-frame midpoint |
| `source_left_camera_rdf_right_m/down_m/forward_m` | Diagnostic source point in the left optical RDF frame |

New rows include:

```text
schema_version=stereowiris-paired-frame-v2
measurement_frame=baseline_center_frd
measurement_axis_contract=baseline_center_frd:forward_right_down
public_vector_contract=baseline_center_frd:[forward,right,down,range]
legacy_xyz_alias_contract=not_emitted
baseline_frame_scope=measurement_attempt
baseline_frame_origin_model=checkerboard_stereo_rt_optical_centers
  or gps_rtk_antenna_baseline_proxy_no_lever_arm
```

Valid rows also include the attempt identity, legacy full/path-sensitive
canonical SHA-256, path-portable semantic content SHA-256, and instance SHA-256
that binds the content hash to the full serialized instance, plus the selected
detection timestamps, quaternion/GPS sample identities, selected NED baseline
and right-origin translation, K/D profile/content hash, and captured source
quality/warnings. These fields describe the same immutable inputs used for that
row's geometry.

V2 rows do not emit ambiguous `x_m/y_m/z_m`, old `left_camera_*`, or
`stereo_center_*` public aliases. V1 rows remain historical
`left_camera_rdf` records and are interpreted by their `schema_version` and
`measurement_frame`; they are not rewritten in place. New rows do not emit target truth,
aligned NED estimate, per-axis error, range error, position error, or UAM
alignment-profile fields.

Attitude timing fields are temporal provenance, not coordinates. Their values
do not change the public frame or axis order.

## Session recording frames and time axes

- WGS84 latitude/longitude/MSL altitude and optional ellipsoid altitude describe each P1/P2 GPS antenna.
- Session-local antenna positions and the left-to-right baseline use `ned` (`North, East, Down`) in metres, with the session-start left GPS antenna as the common origin.
- P1/P2 attitude quaternions rotate `px4_frd` into `ned` and use `(w,x,y,z)` order. Session-start attitude deltas are reported as RPY plus the shortest SO(3) rotation angle.
- Detector bbox corners and source rays remain in each camera's optical RDF convention; a successful public point is `baseline_center_frd`.
- `session_elapsed_s` is an app-monotonic ordering/presentation axis based on the matched frame-pair midpoint. Left/right RTCP/NTP Unix seconds/nanoseconds, PX4 boot/sample timestamps, SensorGps UTC, and app receive monotonic timestamps remain separate provenance clocks and are not coordinate values.
- Device timeline CSVs keep the clocks separate: `sample_monotonic_*` is the source/app receive monotonic clock, `session_utc_*` is derived only from the session host Unix/monotonic anchor, `source_utc_*` is populated only by valid RTCP/NTP or GNSS UTC, and `host_receive_*` is captured at the GUI callback. Camera timeline rows use image pixels with top-left origin, `u` right and `v` down. PX4 attitude rows explicitly retain the `px4_frd -> ned` quaternion contract; GPS velocity remains world NED.

## Historical artifacts

Recordings created by older versions may contain target telemetry, ULog
comparison, `px4_local_ned`, stereo-center, correction, or alignment fields.
Those files describe their historical producer and are not an active schema or
an implicit conversion rule for new measurements. StereoWIRIS does not
load those values into the current Raw measurement path.
