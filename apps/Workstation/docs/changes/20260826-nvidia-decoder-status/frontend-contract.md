# Frontend Contract

## Controls and inputs

Existing Settings decoder combo remains the only mode control. Proposed default/current value: Auto.

## Payloads and commands

No new command. UI consumes requested mode, per-side active decoder, fallback reason, driver and
element availability.

## Visible states and transitions

- stopped/no evidence → `--`
- Auto/NVIDIA + active NVDEC → `GPU · nvh264dec`
- explicit Software → `CPU · avdec_h264 (selected)`
- Auto fallback → `CPU fallback · avdec_h264` with reason
- explicit NVIDIA unavailable/failure → `Unavailable`/ERROR with recovery guidance

## Errors and recovery actions

Auto shows why fallback occurred. Explicit NVIDIA tells operator to select Auto/Software or repair
driver/plugin. Selector change clears runtime fallback override for the next start.

## Keyboard, focus, accessibility, and layout

No new modal/control. Existing combo keyboard/focus remains. States use text and stable extents.

## Cross-feature visible behavior and isolation

Camera decoder failure changes Camera/Performance evidence only. Pixhawk, GNSS, calibration,
recording artifacts, and Measurement history remain intact.
