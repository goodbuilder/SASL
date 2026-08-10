"""Workstation MAVLink parameter-store adapters.

The adapters own wire-type handling only. They do not interpret telemetry or
coordinate frames.
"""


class MavlinkParameterStore:
    """Integer parameter adapter with mandatory write readback."""

    def __init__(self, subscriber, camera: str) -> None:
        self.subscriber = subscriber
        self.camera = camera

    def get_param(self, name: str) -> int | float | None:
        return self.subscriber.read_param_int(self.camera, name, timeout_s=1.5)

    def set_param(self, name: str, value: int | float) -> None:
        expected = int(value)
        actual = self.subscriber.set_param_int(
            self.camera, name, expected, timeout_s=2.0
        )
        if actual != expected:
            raise RuntimeError(
                f"{name} readback mismatch: expected={expected}, actual={actual}"
            )


class MavlinkAttitudeParameterStore:
    """Read-only type-aware adapter retaining genuine REAL32 values."""

    def __init__(self, subscriber, camera: str) -> None:
        self.subscriber = subscriber
        self.camera = camera

    def get_param(self, name: str) -> int | float | None:
        return self.subscriber.read_param(self.camera, name, timeout_s=1.5)
