"""Workstation-owned measurement, camera, telemetry, and recording core.

This package is intentionally separate from the Legacy package.  Legacy keeps
its historical package and import surface; Workstation imports this copy so
the two applications do not share runtime code, settings roots, or optional
middleware dependencies.
"""
