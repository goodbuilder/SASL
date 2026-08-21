from __future__ import annotations

from types import SimpleNamespace

import pytest

from Workstation.measurement_view_controller import MeasurementViewController
from Workstation.standalone_backend import StandalonePanelBackend
from Workstation.workstation_core.measurement_methods import (
    CHECKERBOARD_STEREO,
    HORIZON_HYBRID,
    MEASUREMENT_FILL_LIVE_GNSS,
    MEASUREMENT_FILL_LIVE_PIXHAWK,
    MEASUREMENT_FILL_NONE,
    MeasurementMethodStatus,
    PIXHAWK_RTK_LIVE,
    normalize_measurement_method_fill_policy,
)


class _Package:
    def __init__(self, metadata=None) -> None:
        self.metadata = dict(metadata or {})
        self.updates: list[dict[str, object]] = []

    def snapshot(self):
        return {"metadata": dict(self.metadata)}

    def update_metadata(self, values):
        update = dict(values)
        self.updates.append(update)
        self.metadata.update(update)


def _panel(package: _Package | None = None) -> SimpleNamespace:
    return SimpleNamespace(
        settings=SimpleNamespace(
            measurement_preferred_method=PIXHAWK_RTK_LIVE,
            measurement_method_fallback_order=(PIXHAWK_RTK_LIVE, CHECKERBOARD_STEREO),
        ),
        _workstation_setting=lambda _name, default=None: default,
        _measurement_id="m000001" if package is not None else "",
        _measurement_package=package,
        _measurement_package_state="COLLECTING" if package is not None else "IDLE",
        _measurement_package_detection_started=False,
        _last_measurement_attempt_key=None,
        _last_measurement_attempt=None,
        append_workstation_run_event=lambda *_args, **_kwargs: None,
    )


def _horizon_status() -> MeasurementMethodStatus:
    components = []
    for side in ("left", "right"):
        components.extend(
            (
                (f"{side}.roll_deg", "candidate:horizon-1"),
                (f"{side}.pitch_deg", "candidate:horizon-1"),
                (f"{side}.yaw_deg", MEASUREMENT_FILL_LIVE_PIXHAWK),
                (f"{side}.north_m", MEASUREMENT_FILL_LIVE_GNSS),
                (f"{side}.east_m", MEASUREMENT_FILL_LIVE_GNSS),
                (f"{side}.down_m", MEASUREMENT_FILL_LIVE_GNSS),
            )
        )
    return MeasurementMethodStatus(
        method_id=HORIZON_HYBRID,
        instance_id="candidate:horizon-1",
        candidate_id="horizon-1",
        ready=True,
        observed_dofs=("roll_deg", "pitch_deg"),
        component_sources=tuple(components),
        requires_live_pixhawk=True,
        requires_live_gnss=True,
    )


def test_method_owned_dofs_are_locked_and_missing_dofs_are_toggleable() -> None:
    controller = MeasurementViewController(_panel(_Package()))
    controller._measurement_live_readiness = lambda: (True, True, ())

    rows = controller._measurement_component_controls(_horizon_status())

    assert rows["left.roll_deg"]["provided"] is True
    assert rows["left.roll_deg"]["locked"] is True
    assert rows["left.yaw_deg"]["editable"] is True
    assert rows["left.yaw_deg"]["compatible_fill_source"] == MEASUREMENT_FILL_LIVE_PIXHAWK
    assert rows["left.north_m"]["editable"] is True
    assert rows["left.north_m"]["compatible_fill_source"] == MEASUREMENT_FILL_LIVE_GNSS


def test_fill_toggle_is_measurement_owned_persisted_and_locked_after_detection() -> None:
    package = _Package()
    panel = _panel(package)
    controller = MeasurementViewController(panel)
    status = _horizon_status()
    controller._measurement_live_readiness = lambda: (True, True, ())
    controller._measurement_method_statuses = lambda: (status,)

    snapshot = controller.configure_workstation_measurement_method_fill(
        {
            "method_id": HORIZON_HYBRID,
            "component": "left.yaw_deg",
            "enabled": False,
        }
    )

    assert panel._measurement_method_fill_policy[HORIZON_HYBRID]["left.yaw_deg"] == MEASUREMENT_FILL_NONE
    assert package.updates[-1]["measurement_method_fill_policy"][HORIZON_HYBRID][
        "left.yaw_deg"
    ] == MEASUREMENT_FILL_NONE
    assert snapshot["fill_policy"][HORIZON_HYBRID]["left.yaw_deg"] == MEASUREMENT_FILL_NONE

    panel._measurement_package_detection_started = True
    with pytest.raises(RuntimeError, match="before Detection"):
        controller.configure_workstation_measurement_method_fill(
            {
                "method_id": HORIZON_HYBRID,
                "component": "left.yaw_deg",
                "enabled": True,
            }
        )


def test_recovery_loads_each_measurements_independent_fill_policy() -> None:
    first_policy = normalize_measurement_method_fill_policy(None)
    first_policy[HORIZON_HYBRID]["left.yaw_deg"] = MEASUREMENT_FILL_NONE
    first = _Package(
        {
            "measurement_preferred_method": HORIZON_HYBRID,
            "measurement_method_fallback_order": [HORIZON_HYBRID, PIXHAWK_RTK_LIVE],
            "measurement_method_fill_policy": first_policy,
        }
    )
    panel = _panel(first)
    controller = MeasurementViewController(panel)

    controller._measurement_method_state()
    assert panel._measurement_preferred_method_id == HORIZON_HYBRID
    assert controller._measurement_fallback_order()[:2] == (
        HORIZON_HYBRID,
        PIXHAWK_RTK_LIVE,
    )
    assert panel._measurement_method_fill_policy[HORIZON_HYBRID]["left.yaw_deg"] == MEASUREMENT_FILL_NONE

    second = _Package()
    panel._measurement_id = "m000002"
    panel._measurement_package = second
    controller._measurement_method_state()

    assert panel._measurement_preferred_method_id == PIXHAWK_RTK_LIVE
    assert controller._measurement_fallback_order()[:2] == (
        PIXHAWK_RTK_LIVE,
        CHECKERBOARD_STEREO,
    )
    assert (
        panel._measurement_method_fill_policy[HORIZON_HYBRID]["left.yaw_deg"]
        == MEASUREMENT_FILL_LIVE_PIXHAWK
    )


def test_disabling_required_horizon_fill_makes_only_that_method_unready() -> None:
    profile_hash = "a" * 64
    mask = SimpleNamespace(
        roll_deg=True,
        pitch_deg=True,
        yaw_deg=False,
        north_m=False,
        east_m=False,
        down_m=False,
    )
    candidate = SimpleNamespace(
        candidate_id="horizon-1",
        method="flat-horizon-roll-pitch",
        optimized_dofs=mask,
        observability=(("roll_deg", True), ("pitch_deg", True)),
        observability_by_side=(),
        input_hashes=(("operating_intrinsics_profile_sha256", profile_hash),),
        created_utc_ns=1,
        warnings=(),
    )
    owner = SimpleNamespace(
        _candidates={"horizon-1": candidate},
        _intrinsics_snapshot=lambda **_kwargs: {
            "applicability": {
                "geometry_allowed": True,
                "profile_sha256": profile_hash,
                "reasons": (),
            }
        },
    )
    panel = _panel(_Package())
    panel.active_intrinsics = object()
    panel.checkerboard_extrinsics_profile = None
    panel.optical_pose_extrinsics_profile = None
    panel._optical_pose_controller = lambda: owner
    controller = MeasurementViewController(panel)
    controller._measurement_live_readiness = lambda: (True, True, ())

    ready = next(
        row for row in controller._measurement_method_statuses() if row.method_id == HORIZON_HYBRID
    )
    assert ready.ready is True
    assert dict(ready.component_sources)["left.yaw_deg"] == MEASUREMENT_FILL_LIVE_PIXHAWK

    policy = controller._measurement_fill_policy()
    policy[HORIZON_HYBRID]["left.yaw_deg"] = MEASUREMENT_FILL_NONE
    panel._measurement_method_fill_policy = policy
    blocked = next(
        row for row in controller._measurement_method_statuses() if row.method_id == HORIZON_HYBRID
    )
    assert blocked.ready is False
    assert "left.yaw_deg has no compatible fill source" in blocked.reasons
    assert dict(blocked.component_sources)["left.yaw_deg"] == MEASUREMENT_FILL_NONE


def test_backend_forwards_typed_fill_toggle_payload() -> None:
    calls: list[dict[str, object]] = []
    panel = SimpleNamespace(
        configure_workstation_measurement_method_fill=lambda payload: calls.append(
            dict(payload)
        )
        or {"ok": True}
    )
    backend = StandalonePanelBackend(panel)

    result = backend.configure_measurement_method_fill(
        {
            "method_id": HORIZON_HYBRID,
            "component": "right.down_m",
            "enabled": False,
        }
    )

    assert result == {"ok": True}
    assert calls == [
        {
            "method_id": HORIZON_HYBRID,
            "component": "right.down_m",
            "enabled": False,
        }
    ]
