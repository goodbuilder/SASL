# UI Quality and Stability

The new screen is a dedicated RQT plugin with the existing Qt/RQT idiom: labels, standard `QPushButton`s, and a read-only `QPlainTextEdit`. It does not restyle the existing Operator/Recorder panels or introduce a new dashboard visual system.

The panel is a standalone RQT perspective because the v1.1 runner owns Gazebo/PX4 process startup and rejects an already active operator stack. Its perspective filename intentionally differs from `operator_rqt.perspective`, so the headless runner's legacy GUI cleanup does not close the control window.

Dynamic status is separated from bounded log output. The log keeps at most 2,000 blocks; the 39-slot status line uses fixed labels/count formatting and does not resize with each message. Buttons are disabled while a process runs, and Stop is enabled only during work. The action's display is refreshed from the file on demand; invalid settings show an error and disable Start.

Automated Qt offscreen construction verifies the widget can load `Simulation.env`, display the default plan action, and receive a status-poll timer tick. The standalone perspective was built and opened visibly; the latest screenshot at `/tmp/sasl-rqt-input-root-status.png` showed `reprocess`, the direct 39-run source path, a separate repository output path, MC=100, and `Source campaign status: 39 slots: valid=39, failed=0`. A QtTest click on **Run configured action** dispatched a 100-trial reprocess and received `ANALYSIS_COMPLETE`. DPI/locale testing, keyboard/focus behavior, dynamic resizing, and a native mouse click on the visible window remain NOT_RUN.

## Existing product-specific visual language and component tokens

## Affected screens, components, text, tables, plots, and conditional controls

## AI/template-like pattern rejection audit

## Dynamic value formats, precision, units, longest extents, and update rates

## Reserved geometry, overflow, wrapping, elision, and detail access

## Focus, selection, scroll, table-column, dock/splitter, and window stability

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

## Before/after screenshots and dynamic-state sequence evidence
