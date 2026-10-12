"""Independent contract evidence for the Measurement ownership decomposition."""

from __future__ import annotations

import ast
import dataclasses
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from Workstation.measurement.frozen_inputs import FrozenPairInputs
from Workstation.measurement.geometry import (
    GeometryActions,
    GeometryInputAdapter,
    GeometryInputs,
)
from Workstation.measurement.geometry_resolver import resolve_measurement_attempt
from Workstation.measurement.optical_inputs import OpticalMeasurementInputs
from Workstation.measurement.planning import plan_frozen_pair
from Workstation.measurement.ports import ValueInput
from Workstation.measurement.state import (
    GeometryState,
    MEASUREMENT_PANEL_FIELDS,
    MethodSelectionState,
    PairCaptureState,
    UNSET,
    measurement_state_for,
    measurement_state_property,
)
from Workstation.measurement_view_controller import MeasurementViewController
from Workstation.tests.test_measurement_geometry_policy import _attempt
from Workstation.workstation_core.calibration import (
    CameraIntrinsics,
    CheckerboardConfig,
    StereoExtrinsics,
    StereoIntrinsicsResult,
)
from Workstation.workstation_core.detection_pair_coordinator import PairTimingDecision
from Workstation.workstation_core.measurement_controller import (
    calculate_measurement_geometry,
)
from Workstation.workstation_core.measurement_methods import (
    CHECKERBOARD_STEREO,
    MeasurementMethodStatus,
)
from Workstation.workstation_core.method_evaluation import FrozenMeasurementPairSnapshot


def frozen_pair() -> FrozenPairInputs:
    attempt = _attempt()
    left, right = attempt.left_detection, attempt.right_detection
    snapshot = FrozenMeasurementPairSnapshot(
        measurement_id="frozen-run",
        pair_id="frozen-run:1",
        pair_sequence=1,
        created_monotonic_ns=1_020_000_000,
        created_unix_ns=2_000_000_000,
        timestamp_basis="app_receive_monotonic",
        left_detection=dataclasses.asdict(left),
        right_detection=dataclasses.asdict(right),
    )
    return FrozenPairInputs(
        physical_key=("pair", 1),
        snapshot=snapshot,
        requested_timestamp_basis="app_receive_monotonic",
        timestamp_basis="app_receive_monotonic",
        timing_fallback_reason="",
        pair_timing_decision=PairTimingDecision(
            requested_basis="app_receive_monotonic",
            effective_basis="app_receive_monotonic",
            left_timestamp_s=1.0,
            right_timestamp_s=1.01,
            signed_delta_ns=10_000_000,
            abs_delta_ns=10_000_000,
        ),
        left_pairing_timestamp_s=1.0,
        right_pairing_timestamp_s=1.01,
        created_monotonic_s=1.02,
        created_wall_s=2.0,
        detection_midpoint_s=1.005,
        left_detection=left,
        right_detection=right,
        left_attitude=None,
        right_attitude=None,
        left_attitude_age_s=None,
        right_attitude_age_s=None,
        left_gps=None,
        right_gps=None,
        measurement_start=0.5,
        reused_inputs=(),
    )


def geometry_capture():
    """Real input adapter with scalar ports; no widget, Qt app or panel exists."""
    values = {"focal": 900.0, "zoom": 1, "manual": False}
    camera = CameraIntrinsics(
        width=640,
        height=480,
        camera_matrix=[[900.0, 0.0, 320.0], [0.0, 900.0, 240.0], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.0] * 5,
        reprojection_error_px=0.2,
        image_count=20,
    )
    calibration = StereoIntrinsicsResult(
        left=camera,
        right=camera,
        board=CheckerboardConfig(9, 6, 0.03),
        extrinsics=StereoExtrinsics(
            rotation_left_to_right=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation_left_to_right_m=[-0.4, 0.0, 0.0],
            reprojection_error_px=0.3,
            pair_count=20,
        ),
    )
    status = MeasurementMethodStatus(
        method_id=CHECKERBOARD_STEREO, instance_id="checkerboard:fixed", ready=True
    )
    statuses = [status]
    missing = ValueInput(lambda default: default)
    ports = {member.name: missing for member in dataclasses.fields(GeometryInputs)}
    ports.update(
        optical=OpticalMeasurementInputs(
            **{
                member.name: missing
                for member in dataclasses.fields(OpticalMeasurementInputs)
            }
        ),
        focal_length_px=lambda: values["focal"],
        active_zoom=lambda: values["zoom"],
        manual_focal_override=lambda: values["manual"],
        _yolo_pair_max_delta_s=ValueInput(lambda default: lambda: 0.1),
        _yolo_vertical_max_delta_px=ValueInput(lambda default: lambda: 5.0),
        settings=ValueInput(lambda default: SimpleNamespace(require_rtk_fixed=False)),
        _workstation_setting=ValueInput(
            lambda default: lambda name, fallback: fallback
        ),
    )
    callbacks = {
        member.name: (
            missing
            if member.name.startswith("panel_")
            else lambda *args, **kwargs: None
        )
        for member in dataclasses.fields(GeometryActions)
    }
    callbacks["_measurement_method_statuses"] = lambda: tuple(statuses)
    methods_state = MethodSelectionState(
        _measurement_selected_method_id=CHECKERBOARD_STEREO,
        _measurement_selected_instance_id=status.instance_id,
        _measurement_selection_reason="frozen calibration",
    )
    adapter = GeometryInputAdapter(
        state=GeometryState(),
        inputs=GeometryInputs(**ports),
        actions=GeometryActions(**callbacks),
        capture_state=PairCaptureState(),
        methods_state=methods_state,
    )
    adapter._measurement_intrinsics_result_for_method = (
        lambda *args, **kwargs: calibration
    )
    return adapter, values, calibration, statuses, status


def test_real_geometry_and_plan_ignore_mutated_live_inputs_after_capture():
    adapter, values, calibration, statuses, status = geometry_capture()
    captured = frozen_pair()
    context = adapter.freeze_geometry_inputs(captured)
    assert not context.capture_error
    captured = dataclasses.replace(captured, geometry_inputs=context)
    first = resolve_measurement_attempt(captured, status, context)
    first_bytes = first.canonical_payload()
    first_plan = plan_frozen_pair(captured, enqueued_monotonic_ns=123)

    values.update(focal=123.0, zoom=99, manual=True)
    calibration.left.camera_matrix[0][0] = 123.0
    calibration.extrinsics.translation_left_to_right_m[0] = -99.0
    statuses.clear()
    adapter.methods_state._measurement_selection_reason = "new live selection"
    second = resolve_measurement_attempt(captured, status, context)
    second_plan = plan_frozen_pair(captured, enqueued_monotonic_ns=123)
    assert second.canonical_payload() == first_bytes
    assert second_plan.requests == first_plan.requests
    assert (
        second_plan.snapshot.canonical_sha256() == captured.snapshot.canonical_sha256()
    )
    assert second.geometry_config.focal_length_px == 900.0
    assert second.intrinsics.active_zoom_level == 1
    assert not second.geometry_config.manual_focal_override
    geometry = calculate_measurement_geometry(second)
    # Independent pinhole oracle: Z = f * B / disparity = 900 * .4 / 40 = 9.
    # Left RDF point (.2, 0, 9) -> baseline-center FRD (9, 0, 0).
    assert geometry.triangulated.point_m == pytest.approx((9.0, 0.0, 0.0), abs=1e-8)
    assert geometry.triangulated.source_point_left_rdf_m == pytest.approx(
        (0.2, 0.0, 9.0), abs=1e-8
    )
    assert geometry.triangulated.range_m == pytest.approx(9.0, abs=1e-8)


def test_frozen_contract_and_plan_exports_have_no_mutable_aliases():
    adapter, _values, _calibration, _statuses, status = geometry_capture()
    captured = frozen_pair()
    context = adapter.freeze_geometry_inputs(captured)
    captured = dataclasses.replace(captured, geometry_inputs=context)
    with pytest.raises(dataclasses.FrozenInstanceError):
        captured.created_wall_s = 999
    with pytest.raises(TypeError):
        context.methods[(status.method_id, status.instance_id)] = None
    plan = plan_frozen_pair(captured, enqueued_monotonic_ns=1)
    exported = plan.legacy_values()
    exported[2][CHECKERBOARD_STEREO]["ready"] = False
    assert plan.evidence[CHECKERBOARD_STEREO]["ready"] is True


def test_typed_pair_production_planner_has_no_live_input_callbacks():
    from Workstation.measurement.planning import (
        MethodPlanningActions,
        MethodPlanningInputs,
        MethodPlanningService,
    )
    from Workstation.measurement.state import MethodPlanningState

    adapter, *_rest = geometry_capture()
    captured = frozen_pair()
    captured = dataclasses.replace(
        captured, geometry_inputs=adapter.freeze_geometry_inputs(captured)
    )

    def reject(*args, **kwargs):
        raise AssertionError("live configuration was queried after pair capture")

    planner = MethodPlanningService(
        state=MethodPlanningState(),
        inputs=MethodPlanningInputs(),
        actions=MethodPlanningActions(lambda pair: captured, reject, reject),
    )
    requests, absent, evidence, snapshot = planner._plan_method_evaluations(object())
    assert [request.method_id for request in requests] == [CHECKERBOARD_STEREO]
    assert snapshot is captured.snapshot
    assert evidence[CHECKERBOARD_STEREO]["ready"] is True
    assert "pixhawk_rtk_live" in absent


def test_compatibility_properties_and_services_share_one_authoritative_state():
    class Host:
        pass

    for name in MEASUREMENT_PANEL_FIELDS:
        setattr(Host, name, measurement_state_property(name))
    host = Host()
    host._measurement_selected_method_id = "checkerboard_stereo"
    controller = MeasurementViewController(host)
    assert controller.services.methods.state is measurement_state_for(host).methods
    assert controller.services.capture.state is measurement_state_for(host).capture
    assert controller.services.commit.state is measurement_state_for(host).commit
    assert "_measurement_selected_method_id" not in vars(host)
    controller.services.methods.state._measurement_selected_method_id = "terrain_pnp"
    assert host._measurement_selected_method_id == "terrain_pnp"
    host._last_frozen_measurement_pair = "sentinel"
    assert controller.services.capture.state._last_frozen_measurement_pair == "sentinel"
    assert "_last_frozen_measurement_pair" not in vars(host)


def test_services_do_not_receive_panel_or_universal_context():
    root = Path(__file__).parents[1] / "measurement"
    for name in (
        "methods",
        "capture",
        "geometry",
        "geometry_resolver",
        "planning",
        "runtime",
        "commit",
        "projection",
        "presentation",
    ):
        source = (root / f"{name}.py").read_text()
        tree = ast.parse(source)
        assert "__getattr__" not in source
        for node in ast.walk(tree):
            if (
                isinstance(node, ast.Attribute)
                and isinstance(node.value, ast.Name)
                and node.value.id == "self"
            ):
                assert node.attr not in {"panel", "controller", "context", "services"}
    resolver = (root / "geometry_resolver.py").read_text()
    assert (
        "self." not in resolver
        and "PyQt" not in resolver
        and "ValueInput" not in resolver
    )
    assert ".setText(" not in (root / "geometry.py").read_text()
    assert ".value()" not in (root / "geometry.py").read_text()


def test_pure_resolver_import_has_no_qt_dependency():
    apps = Path(__file__).parents[2]
    code = """
import builtins
original = builtins.__import__
def guarded(name, *args, **kwargs):
    if name.startswith(('PyQt', 'PySide')):
        raise AssertionError('Qt import from pure geometry')
    return original(name, *args, **kwargs)
builtins.__import__ = guarded
from Workstation.measurement.geometry_resolver import resolve_measurement_attempt
from Workstation.measurement.planning import plan_frozen_pair
"""
    env = dict(
        os.environ,
        PYTHONPATH=os.pathsep.join(
            dict.fromkeys([str(apps), *(entry for entry in sys.path if entry)])
        ),
    )
    completed = subprocess.run(
        [sys.executable, "-c", code], env=env, capture_output=True, text=True
    )
    assert completed.returncode == 0, completed.stderr


def test_resource_inventory_includes_non_suffix_actor_and_queues():
    controller = MeasurementViewController(SimpleNamespace())
    actor, commands, results, timer = object(), object(), object(), object()
    owner = SimpleNamespace(_actor=actor, _commands=commands, _results=results)
    controller._method_evaluation_coordinator = owner
    controller._method_evaluation_timer = timer
    resources = controller.resource_inventory()
    assert resources["method.coordinator"] is owner
    assert resources["method.actor_thread"] is actor
    assert resources["method.commands"] is commands
    assert resources["method.results"] is results
    assert resources["method.timer"] is timer


def test_capture_preserves_preferred_second_method_before_default_build():
    from Workstation.workstation_core.measurement_methods import HORIZON_HYBRID

    adapter, _values, _calibration, statuses, first = geometry_capture()
    preferred = dataclasses.replace(
        first, method_id=HORIZON_HYBRID, instance_id="preferred-second"
    )
    statuses.append(preferred)
    adapter.methods_state._measurement_selected_method_id = UNSET
    adapter.methods_state._measurement_selected_instance_id = UNSET
    selections = []

    def choose_preferred():
        selections.append(preferred)
        return preferred

    adapter.actions = dataclasses.replace(
        adapter.actions, _select_measurement_method_for_attempt=choose_preferred
    )
    captured = frozen_pair()
    captured = dataclasses.replace(
        captured, geometry_inputs=adapter.freeze_geometry_inputs(captured)
    )
    assert selections == [preferred]
    assert captured.geometry_inputs.statuses == (first, preferred)
    assert captured.geometry_inputs.selected_method_id == HORIZON_HYBRID
    # Calling the default build after capture must not repeat the policy read.
    adapter.actions = dataclasses.replace(
        adapter.actions,
        _select_measurement_method_for_attempt=lambda: pytest.fail(
            "selection after capture"
        ),
    )
    result = adapter._build_measurement_attempt(
        object(), frozen_inputs=captured, use_cache=False
    )
    assert result.measurement_method_id == HORIZON_HYBRID
    assert result.measurement_method_instance_id == "preferred-second"


def test_capture_without_selected_method_preserves_hysteresis_wait():
    adapter, *_rest = geometry_capture()
    adapter.methods_state._measurement_selected_method_id = UNSET
    adapter.methods_state._measurement_selected_instance_id = UNSET
    adapter.actions = dataclasses.replace(
        adapter.actions, _select_measurement_method_for_attempt=lambda: None
    )
    captured = frozen_pair()
    captured = dataclasses.replace(
        captured, geometry_inputs=adapter.freeze_geometry_inputs(captured)
    )
    assert any(status.ready for status in captured.geometry_inputs.statuses)
    with pytest.raises(ValueError, match="no ready measurement method at pair capture"):
        adapter._build_measurement_attempt(
            object(), frozen_inputs=captured, use_cache=False
        )


def test_selection_failure_is_captured_as_evidence_without_dropping_pair():
    adapter, *_rest = geometry_capture()

    def failed_selection():
        raise ValueError("calibration selection unavailable")

    adapter.actions = dataclasses.replace(
        adapter.actions, _select_measurement_method_for_attempt=failed_selection
    )
    captured = frozen_pair()
    context = adapter.freeze_geometry_inputs(captured)
    captured = dataclasses.replace(captured, geometry_inputs=context)
    assert captured.snapshot.pair_id == "frozen-run:1"
    assert context.config is None
    assert "calibration selection unavailable" in context.capture_error
