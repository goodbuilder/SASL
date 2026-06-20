# AGENTS.md - 영상감시시스템 공통 작업 규칙

This file applies to the `SASL` repository and every child directory
unless a deeper `AGENTS.md` adds more specific rules.

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

## Git Portability Discipline

The intended workflow is to push the `SASL` repository, clone or pull it
on a notebook, and continue work from the pushed files without relying on
untracked machine-local state.

When adding or changing dependencies, generated interfaces, firmware
helpers, scripts, parameters, or non-app files:

- Keep reproducible source, configuration, scripts, and documentation
  inside the `SASL` git tree when they are required to continue the work
  on another machine.
- Do not make required project state live only in absolute local paths
  such as `/home/moon/px4_msgs_ws`, `/opt/...`, `/tmp/...`, or a desktop
  download directory. If such a path is used for local execution, provide
  a repo-relative alternative or document the setup command that recreates
  it.
- Prefer repo-local workspaces such as `SASL/workspace/main/src/...` for
  ROS interface source packages. Build products such as `build/`,
  `install/`, and `log/` should normally be regenerated on the target
  machine instead of committed. If ROS tooling cannot build reliably from
  the checkout path, keep only the build/log cache outside the repository
  and leave the reproducible source plus setup script under `SASL`.
- If PX4 firmware source changes are required, keep the patch, fork,
  submodule, or setup script under `SASL` rather than only modifying a
  separate checkout. The notebook clone must make it clear which PX4
  source revision and DDS/uORB export changes are needed.
- Do not commit machine-specific caches, virtual environments, compiled
  outputs, large downloaded model files, or secrets unless the repository
  already tracks them intentionally and the user has confirmed that they
  belong in git.
- When a command, script, test, or app setting references a local absolute
  path, either convert it to a repo-relative path or record why the
  absolute path is intentionally machine-local.
- Before finalizing a change that affects portability, run `git status`
  from the `SASL` root and call out any required files that are untracked,
  ignored, or outside the repository.

## Verification

Do not claim a measurement, correction, or validation change is complete
without fresh evidence. Prefer targeted tests first, then smoke/runtime
checks when the change affects execution behavior.
