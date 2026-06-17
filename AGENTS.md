# AGENTS.md - 영상감시시스템 공통 작업 규칙

This file applies to `/home/moontaehyung/영상감시시스템` and every child
directory unless a deeper `AGENTS.md` adds more specific rules.

## Coordinate Contract Gate

Coordinate and frame contracts are safety-critical for this workspace.
Every change that touches measurement, triangulation, simulation,
telemetry, Pixhawk/GPS/ULog handling, WIRIS data, recording, plotting,
validation, CSV schemas, or documentation must include a coordinate
contract review.

Before editing:

- Identify every frame involved: NED, ENU, PX4/body FRD, OpenCV optical
  RDF, stereo-center, left-camera, right-camera, world, GPS/RTK, or ULog.
- Identify every axis order used by arrays, topics, CSV fields, and UI
  labels.
- Treat fields named `x/y/z` as ambiguous until their alias contract is
  verified from nearby schema, docs, tests, or producer code.
- Distinguish position frame from orientation frame. A Pixhawk attitude,
  GPS/RTK baseline, camera optical ray, and recorded CSV point may all
  use different contracts.

When editing:

- Do not introduce or move values between `north/east/down`,
  `forward/right/down`, `right/down/forward`, `x/y/z`,
  `stereo-center`, `left-camera`, or `right-camera` fields without an
  explicit conversion or an explicit alias contract.
- Do not write center-frame values into left-camera/right-camera fields,
  or NED values into camera-frame fields.
- Any changed formula must make its input frame, output frame, units, and
  sign convention clear in code, schema, or tests.
- Preserve legacy fields only when a clear `axis_contract` or alias field
  exists.

After editing:

- Run or add targeted tests for any changed coordinate transform,
  triangulation formula, topic/CSV schema, or validation path.
- Check documentation and UI labels against producer code and saved CSV
  fields.
- In the final response, include a short line:
  `Coordinate contract checked: <files/frames/tests>`.

## Experiment Method Discipline

For experiment or correction-method changes:

- Record the method name, purpose, input assumptions, output contract,
  and pass/fail criteria in the relevant README, report, or experiment
  log.
- Distinguish calibration-time truth from runtime inputs. Runtime methods
  must not depend on ground truth unless explicitly marked as offline
  validation only.
- Treat partial failures as information: report what improved, what
  regressed, and which conditions were outside the validated range.

## Verification

Do not claim a measurement, correction, or validation change is complete
without fresh evidence. Prefer targeted tests first, then smoke/runtime
checks when the change affects execution behavior.
