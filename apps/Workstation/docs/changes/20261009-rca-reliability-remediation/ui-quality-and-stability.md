# UI quality and stability

Preserve the existing dense engineering Data Status view, Qt tables/buttons and theme. Only the method execution/recovery area changes. Existing navigation, Inspector, plots, and camera widgets are preserved.

Verify idle/running/recovery/closing/error, long IDs, elapsed rollover, selection and focus during rapid refresh, and compact window sizes. Use offscreen actual-widget captures plus state assertions; hardware images are not required for this control.

No CPU/GPU allocation or throughput tuning claim. A queued actor restart must leave GUI callbacks responsive while process termination is in progress.
