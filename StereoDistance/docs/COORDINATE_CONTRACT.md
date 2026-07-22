# Coordinate Contract

This document fixes the coordinate-frame and axis naming contract used by
StereoDistance. The code source of truth is
`wiris_stereo_distance/coordinate_contract.py`.

## Active frames

| Frame id | Meaning | Axis contract | Active use |
| --- | --- | --- | --- |
| `ned` | PX4/GNSS navigation frame | `North/East/Down` | P1/P2 attitude and antenna-baseline geometry |
| `px4_frd` | PX4 body frame | `Forward/Right/Down` | P1/P2 body attitude and nominal camera mount |
| `camera_optical_rdf` | Each camera optical frame | `Right/Down/Forward` | Pixel rays before conversion into the left frame |
| `left_camera_rdf` | Left optical-center frame | `Right/Down/Forward` | Raw/public StereoDistance point and range |

`NED`, `FRD`, and optical `RDF` are not aliases. Values cross these frames
only through the explicit attitude, mount, and baseline transforms used by the
triangulation path.

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
- Algorithm: express the right ray and right optical-center origin in
  `left_camera_rdf`, find the closest points on the two positive rays, and
  return their midpoint.
- Output: `[Right, Down, Forward, Range]` in `left_camera_rdf`, with the left
  optical center at `(0, 0, 0)` and
  `Range = norm([Right, Down, Forward])`.
- No baseline-midpoint translation or public FRD reorder is applied.
- Missing or mismatched intrinsics grids, missing pose/translation, zero
  baseline, near-parallel rays, non-positive depth, and non-finite range fail
  closed and retain a rejection reason.

StereoDistance has no measurement-correction or tracking-filter layer. The
former BBox reprojection, affine bbox-center, distance-binned 3-D bias, and
Kalman paths were removed. Runtime and newly saved measurement rows contain
the Raw `left_camera_rdf` result only.

### Coherent measurement input

Each newly selected detection pair receives a generation/session-scoped,
monotonically increasing `measurement_attempt_id`; `MeasurementAttempt.pair_id`
combines that generation identity and sequence instead of using a timestamp
alone. Before geometry runs, StereoDistance
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
when the snapshot is created. Metadata is stored in allowlist order. The
attempt canonical SHA-256 is an integrity hash of one serialized attempt
instance, including its generation, sequence, timestamps, and profile path; it
is not a physical detection-pair identity.

Correspondence is fail-closed before attempt construction: left/right classes
must match, and either side with multiple candidates of that selected class is
rejected because this phase does not introduce a validated multi-object
matcher. Independent highest-confidence detections are not triangulated when
that correspondence is ambiguous.

For the Pixhawk path, the right ray is rotated into the selected left-camera
basis with the attempt's left quaternion, and the selected GPS pair's NED
baseline is transformed into `left_camera_rdf` with that exact same left
quaternion and selected app-monotonic attitude timestamp. The quaternion order
is `(w,x,y,z)`. Geometry and valid-row recording do not re-read GUI latest
attitude, GPS baseline, or intrinsics after the attempt exists. Checkerboard
attempts instead copy the applied stereo `R/T` and do not consume Pixhawk or
GPS values.

This coherence rule does not change validity policy: stale attitude remains
display-only and non-blocking, RTK Fixed is not newly required, and no ray-gap
or intersection-angle acceptance threshold is added in this phase.

S3-B1 does not approve hardware reliability. GPS age/skew and stale/RTK
acceptance policy remain for S3-B2; atomic recorder/session isolation remains
for S3-B3; and the public `baseline_center_frd` origin/axis migration remains
for S3-B4. The current public output stays `left_camera_rdf`.

## P1/P2 Pixhawk data boundary

P1/P2 Pixhawk data describes the stereo measurement rig. It may provide:

- body orientation in `px4_frd` relative to `ned`;
- GPS/RTK antenna positions used to form the left-to-right baseline;
- GNSS, magnetometer, connection, and timing quality diagnostics.

It does not provide target/UAM Ground Truth in the active StereoDistance
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

This `display_only_non_blocking` state is provenance. It does not rotate,
translate, reject, or otherwise alter the `left_camera_rdf` result.
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

Current Raw measurements use the following meanings:

| Field | Meaning |
| --- | --- |
| `left_camera_right_m`, `raw_left_camera_right_m`, legacy `x_m`/`raw_x_m` | Right from the left optical center |
| `left_camera_down_m`, `raw_left_camera_down_m`, legacy `y_m`/`raw_y_m` | Down from the left optical center |
| `left_camera_forward_m`, `raw_left_camera_forward_m`, legacy `z_m`/`raw_z_m` | Forward from the left optical center |
| `distance_m`, `raw_range_m` | Euclidean range from the left optical center |

New rows include:

```text
measurement_frame=left_camera_rdf
measurement_axis_contract=left_camera_rdf:right_down_forward
public_vector_contract=left_camera_rdf:[right,down,forward,range]
legacy_xyz_alias_contract=x=right,y=down,z=forward
```

Valid rows also include the attempt identity and canonical SHA-256 plus the
selected detection timestamps, quaternion/GPS sample identities, selected NED
baseline and right-origin translation, K/D profile/content hash, and captured
source quality/warnings. These fields describe the same immutable inputs used
for that row's geometry.

Legacy `x_m/y_m/z_m` fields remain only as explicit aliases. New rows leave
`stereo_center_*` and `raw_stereo_center_*` empty instead of placing
left-origin values in midpoint-named fields. They do not emit target truth,
aligned NED estimate, per-axis error, range error, position error, or UAM
alignment-profile fields.

Attitude timing fields are temporal provenance, not coordinates. Their values
do not change the public frame or axis order.

## Historical artifacts

Recordings created by older versions may contain target telemetry, ULog
comparison, `px4_local_ned`, stereo-center, correction, or alignment fields.
Those files describe their historical producer and are not an active schema or
an implicit conversion rule for new measurements. StereoDistance does not
load those values into the current Raw measurement path.
