# Frontend Contract

## Controls and inputs

No new control. Start/Stop Auto, Interval, and Max Live pair delta keep current ranges/defaults.

## Payloads and commands

The command/setup payload is unchanged. AU classification is runtime evidence, not a setting.

## Visible states and transitions

Idle -> waiting NAL1 on both sides -> waiting exact FrameKeys -> existing pose/timing evaluation ->
capture worker -> saved/waiting. Only active timing-eligible two-sided saves increment the count.

## Errors and recovery actions

Messages distinguish missing NAL1, missing/expired exact key, wrong generation, duplicate pose,
stale frame, timing-only candidate, and worker failure. Recovery is a later live candidate; manual
Live Pair and Stop stay available.

## Keyboard, focus, accessibility, and layout

Unchanged tab/focus order, widget geometry, wrapping, theme, selection, and scrolling.

## Cross-feature visible behavior and isolation

Preview, YOLO, Measurement, and recording continue while Auto waits/fails. Auto candidacy does not
change global camera severity.
