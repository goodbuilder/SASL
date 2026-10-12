"""Pure magnetometer identity/unit/yaw projections from captured sensor values.

SCALED_IMU mG and Shell/CAN G remain producer sensor axes. Corrected yaw uses
existing PX4 calibration math; body FRD/world NED attitude is an explicit input.
"""

from collections.abc import Mapping
from dataclasses import dataclass

from Workstation.workstation_core.magnetometer_yaw import calibrated_magnetometer_yaw_deg, project_magnetometer_yaw
from Workstation.workstation_core.pixhawk_link.decode import MAVLINK_MAG_STREAMS, magnetometer_value_units
from Workstation.workstation_core.pixhawk_link.policy import MAVLINK_ATTITUDE_STALE_S, MAVLINK_IMU_STREAM_STALE_S
from Workstation.workstation_core.px4_mag_manager import (
    INTERNAL_MAG_IDS,
    MAG_SLOTS,
    RM3100_BACKUP_IDS,
    RM3100_PRIMARY_IDS,
    identify_mag_device,
    mag_priority_slot_key,
    mag_role_for_device_id,
)

from .snapshot import snapshot_mapping
from .values import _age, _integer, _number

MAG_SAMPLE_FIELDS = ("topic", "timestamp_s", "px4_timestamp_s", "x", "y", "z", "magnitude", "device_id", "node_id")
MAG_SLOT_FIELDS = ("slot", "device_id", "priority")


@dataclass(frozen=True)
class MagnetometerInputs:
    imu: Mapping
    can: Mapping
    slots: tuple
    active_device_id: object
    params: object
    board_params: object
    attitude: Mapping

    def __post_init__(self):
        for name in ("imu", "can", "attitude"):
            object.__setattr__(self, name, snapshot_mapping(getattr(self, name)))
        for name in ("params", "board_params"):
            value = getattr(self, name)
            object.__setattr__(self, name, None if value is None else snapshot_mapping(value))
        object.__setattr__(self, "slots", tuple(snapshot_mapping(value) for value in self.slots))


class MagnetometerProjection:
    def __init__(self, inputs, *, now):
        self.inputs, self.now = inputs, float(now)

    def _magnetometer_yaw_fields(
        self,
        side: str,
        *,
        xmag: object,
        ymag: object,
        sensor_device_id: int | None,
        sample_available: bool,
        now_s: float,
    ) -> dict[str, object]:
        """Return raw yaw plus an exact active-device PX4 reference."""
        active_device_id = _integer(self.inputs.active_device_id)
        if not sample_available:
            return {
                "raw_yaw_deg": None,
                "px4_yaw_deg": None,
                "delta_yaw_deg": None,
                "px4_yaw_state": "WAITING",
                "px4_yaw_reason": "MAG sample waiting",
                "raw_yaw_contract": "RAW sensor-axis azimuth = atan2(-y, x); no CAL_MAG, sensor rotation, board rotation, or tilt correction",
                "active_device_id": active_device_id,
                "px4_yaw_age_s": None,
            }
        attitude = self.inputs.attitude
        projection = project_magnetometer_yaw(
            x=xmag,
            y=ymag,
            sensor_device_id=sensor_device_id,
            active_device_id=active_device_id,
            px4_yaw_deg=attitude.get("yaw_deg"),
            px4_age_s=attitude.get("age_s"),
            px4_stale_s=MAVLINK_ATTITUDE_STALE_S,
        )
        return {
            **projection.to_mapping(),
            "active_device_id": active_device_id,
            "px4_yaw_age_s": attitude.get("age_s"),
            "px4_yaw_source": attitude.get("source", "MAVLink attitude telemetry"),
            "px4_yaw_frame_contract": attitude.get(
                "frame_contract", "PX4 vehicle attitude quaternion rotates body FRD to world NED"
            ),
        }

    def _magnetometer_corrected_yaw_fields(
        self,
        side: str,
        *,
        xmag: object,
        ymag: object,
        zmag: object,
        sensor_device_id: int | None,
        sample_available: bool,
    ) -> dict[str, object]:
        if not sample_available:
            return {
                "corrected_yaw_deg": None,
                "corrected_yaw_state": "WAITING",
                "corrected_yaw_reason": "MAG sample waiting",
                "corrected_yaw_contract": "CAL_MAG-corrected sensor vector; PARAM readback required",
            }
        if sensor_device_id is None:
            return {
                "corrected_yaw_deg": None,
                "corrected_yaw_state": "IDENTITY UNKNOWN",
                "corrected_yaw_reason": "sensor device ID unavailable; corrected yaw not calculated",
                "corrected_yaw_contract": "CAL_MAG-corrected sensor vector; device identity required",
            }
        params = self.inputs.params
        slot = None
        if params is not None:
            for candidate in range(4):
                if _integer(params.get(f"CAL_MAG{candidate}_ID")) == int(sensor_device_id):
                    slot = candidate
                    break
        board_params = self.inputs.board_params
        calibration = calibrated_magnetometer_yaw_deg(
            x=xmag,
            y=ymag,
            z=zmag,
            calibration_params=params,
            calibration_slot=slot,
            board_params=board_params,
            internal_sensor=int(sensor_device_id) in INTERNAL_MAG_IDS,
        )
        return {
            **calibration,
            "corrected_yaw_contract": "PX4 Magnetometer::Correct-equivalent: CAL_MAG offset/scale/off-diagonal/rotation; power compensation is withheld when no contemporaneous power sample exists",
        }

    def magnetometers(self, side: str, *, now_s: float | None = None) -> tuple[dict[str, object], ...]:
        """Project the ROS2-free MAVLink IMU magnetometer streams.

        ``SCALED_IMU*`` values retain the producer's xmag/ymag/zmag sensor
        axes and milligauss unit.  This adapter only presents those values; it
        does not reinterpret them as NED, ENU, PX4 FRD, or camera coordinates.
        """
        now = self.now if now_s is None else float(now_s)
        states_by_slot = self.inputs.imu
        rows: list[dict[str, object]] = []
        axis_contract = (
            "MAVLink SCALED_IMU* producer xmag/ymag/zmag sensor axes; Workstation applies no coordinate conversion"
        )
        for sensor_slot, imu_label, message_name in MAVLINK_MAG_STREAMS:
            expected_topic = f"mavlink/{message_name}"
            state = states_by_slot.get(sensor_slot) if isinstance(states_by_slot, Mapping) else None
            if state is None:
                yaw_fields = self._magnetometer_yaw_fields(
                    side, xmag=None, ymag=None, sensor_device_id=None, sample_available=False, now_s=now
                )
                rows.append(
                    {
                        "side": side,
                        "pixhawk": "P1" if side == "left" else "P2",
                        "sensor_slot": sensor_slot,
                        "imu": imu_label,
                        "state": "WAITING",
                        "xmag": None,
                        "ymag": None,
                        "zmag": None,
                        "norm": None,
                        "unit": "mG",
                        "age_s": None,
                        "timestamp_s": None,
                        "px4_timestamp_s": None,
                        "topic": expected_topic,
                        "device_id": None,
                        "axis_contract": axis_contract,
                        "source": "MAVLink SCALED_IMU telemetry",
                        **yaw_fields,
                        **self._magnetometer_corrected_yaw_fields(
                            side, xmag=None, ymag=None, zmag=None, sensor_device_id=None, sample_available=False
                        ),
                    }
                )
                continue
            topic = str(state.get("topic", "") or expected_topic)
            values = {
                "xmag": _number(state.get("x", None)),
                "ymag": _number(state.get("y", None)),
                "zmag": _number(state.get("z", None)),
                "norm": _number(state.get("magnitude", None)),
            }
            complete = all(values[key] is not None for key in ("xmag", "ymag", "zmag"))
            age_s = _age(state.get("timestamp_s", None), now)
            device_id = _integer(state.get("device_id", None))
            yaw_fields = self._magnetometer_yaw_fields(
                side,
                xmag=values["xmag"],
                ymag=values["ymag"],
                sensor_device_id=device_id,
                sample_available=True,
                now_s=now,
            )
            rows.append(
                {
                    "side": side,
                    "pixhawk": "P1" if side == "left" else "P2",
                    "sensor_slot": sensor_slot,
                    "imu": imu_label,
                    "state": "PARTIAL"
                    if not complete
                    else "STALE"
                    if age_s is None or age_s > MAVLINK_IMU_STREAM_STALE_S
                    else "LIVE",
                    **values,
                    "unit": magnetometer_value_units(topic) or "mG",
                    "age_s": age_s,
                    "timestamp_s": _number(state.get("timestamp_s", None)),
                    "px4_timestamp_s": _number(state.get("px4_timestamp_s", None)),
                    "topic": topic,
                    "device_id": device_id,
                    "axis_contract": axis_contract,
                    "source": "PX4 Shell sensor_mag readback"
                    if topic.lower().startswith("px4-shell/sensor_mag/")
                    else "MAVLink SCALED_IMU telemetry",
                    **yaw_fields,
                    **self._magnetometer_corrected_yaw_fields(
                        side,
                        xmag=values["xmag"],
                        ymag=values["ymag"],
                        zmag=values["zmag"],
                        sensor_device_id=device_id,
                        sample_available=complete,
                    ),
                }
            )
        return tuple(rows)

    def can_magnetometers(self, side: str, *, now_s: float | None = None) -> tuple[dict[str, object], ...]:
        """Project firmware CAN_MAG telemetry, with a CAL_MAG identity fallback.

        xmag/ymag/zmag/Norm are never derived from CAL_MAG parameters -- only
        PX4's DEBUG_VECT CAN_MAG compatibility message proves a CAN sensor is
        currently online and sampling. When that message is absent (older
        firmware, or simply no packet yet on the active transport --
        USB/radio telemetry/Ethernet all work the same way here), Node
        ID/Device ID are instead read from the last CAL_MAG*_ID parameter
        readout in ``panel._px4_mag_status_cache`` (populated by the PX4 MAG
        calibration "Read" action). That is a fixed configuration value, not
        a live sample, so the row is marked "CONFIG" rather than LIVE/STALE.
        """
        now = self.now if now_s is None else float(now_s)
        states_by_slot = self.inputs.can
        rows: list[dict[str, object]] = []
        axis_contract = "firmware CAN magnetometer xmag/ymag/zmag producer sensor axes; Workstation applies no coordinate conversion"

        def source_for(topic: str) -> str:
            if topic.lower().startswith("px4-shell/sensor_mag/"):
                return "PX4 Shell sensor_mag readback"
            return "PX4 DEBUG_VECT CAN_MAG compatibility telemetry"

        configured_rm: dict[str, tuple[int, int | None]] = {}
        for slot in self.inputs.slots:
            device_id = _integer(slot.get("device_id", None))
            if device_id in RM3100_PRIMARY_IDS:
                configured_rm["CAN1"] = (device_id, 125 if device_id == RM3100_PRIMARY_IDS[0] else 124)
            elif device_id in RM3100_BACKUP_IDS:
                configured_rm["CAN2"] = (device_id, 123 if device_id == RM3100_BACKUP_IDS[0] else None)
        for sensor_slot in ("CAN1", "CAN2"):
            state = states_by_slot.get(sensor_slot) if isinstance(states_by_slot, Mapping) else None
            if state is None:
                configured = configured_rm.get(sensor_slot)
                yaw_fields = self._magnetometer_yaw_fields(
                    side, xmag=None, ymag=None, sensor_device_id=None, sample_available=False, now_s=now
                )
                rows.append(
                    {
                        "side": side,
                        "pixhawk": "P1" if side == "left" else "P2",
                        "can_port": sensor_slot,
                        "node_id": None if configured is None else configured[1],
                        "device_id": None if configured is None else configured[0],
                        "state": "CONFIG" if configured is not None else "WAITING",
                        "xmag": None,
                        "ymag": None,
                        "zmag": None,
                        "norm": None,
                        "unit": "G",
                        "age_s": None,
                        "topic": "px4-shell/sensor_mag or mavlink/DEBUG_VECT",
                        "axis_contract": axis_contract,
                        "source": "PX4 CAL_MAG configuration; waiting for Shell sensor_mag sample"
                        if configured is not None
                        else "PX4 Shell sensor_mag / DEBUG_VECT compatibility telemetry",
                        **yaw_fields,
                        **self._magnetometer_corrected_yaw_fields(
                            side,
                            xmag=None,
                            ymag=None,
                            zmag=None,
                            sensor_device_id=None if configured is None else configured[0],
                            sample_available=False,
                        ),
                    }
                )
                continue
            topic = str(state.get("topic", "") or "mavlink/DEBUG_VECT")
            age_s = _age(state.get("timestamp_s", None), now)
            values = {
                "xmag": _number(state.get("x", None)),
                "ymag": _number(state.get("y", None)),
                "zmag": _number(state.get("z", None)),
                "norm": _number(state.get("magnitude", None)),
            }
            complete = all(values[key] is not None for key in ("xmag", "ymag", "zmag"))
            shell_source = topic.lower().startswith("px4-shell/sensor_mag/")
            if not complete:
                state_label = "PARTIAL"
            elif age_s is None or age_s > (15.0 if shell_source else 5.0):
                state_label = "DISCONNECTED"
            elif age_s > (6.0 if shell_source else 1.0):
                state_label = "STALE"
            else:
                state_label = "LIVE"
            device_id = _integer(state.get("device_id", None))
            yaw_fields = self._magnetometer_yaw_fields(
                side,
                xmag=values["xmag"],
                ymag=values["ymag"],
                sensor_device_id=device_id,
                sample_available=True,
                now_s=now,
            )
            rows.append(
                {
                    "side": side,
                    "pixhawk": "P1" if side == "left" else "P2",
                    "can_port": sensor_slot,
                    "node_id": _integer(state.get("node_id", None)),
                    "device_id": device_id,
                    "state": state_label,
                    **values,
                    "unit": magnetometer_value_units(topic) or "G",
                    "age_s": age_s,
                    "topic": topic,
                    "axis_contract": axis_contract,
                    "source": source_for(topic),
                    **yaw_fields,
                    **self._magnetometer_corrected_yaw_fields(
                        side,
                        xmag=values["xmag"],
                        ymag=values["ymag"],
                        zmag=values["zmag"],
                        sensor_device_id=device_id,
                        sample_available=complete,
                    ),
                }
            )
        return tuple(rows)

    def magnetometers_for_imu_view(self, side: str, *, now_s: float | None = None) -> tuple[dict[str, object], ...]:
        """Project the IMU-table view without relabeling CAN data as SCALED_IMU.

        The first three rows are the standard ``SCALED_IMU*`` streams.  The
        final two rows are the existing dedicated ``DEBUG_VECT``
        ``CAN1_MAG``/``CAN2_MAG`` compatibility samples from the stable PX4
        path.  Keeping the two sources separate in the evidence dictionaries
        preserves their topic, units, identity, and yaw-correlation rules.
        """
        rows = list(self.magnetometers(side, now_s=now_s))
        for can_row in self.can_magnetometers(side, now_s=now_s):
            can_port = str(can_row.get("can_port") or "CAN?")
            rows.append({**can_row, "imu": f"RM3100 {can_port}", "sensor_slot": can_port})
        return tuple(rows)

    def magnetometers_for_cal_mag_view(self, side: str, *, now_s: float | None = None) -> tuple[dict[str, object], ...]:
        """Project one row per PX4 ``CAL_MAGn`` slot in parameter order.

        ``sensor_mag`` instance numbers and MAVLink ``SCALED_IMU*`` stream
        numbers are transport/topology identities, not calibration-slot
        identities.  The operator-facing MAG table therefore uses the
        configured ``CAL_MAGn_ID`` as its row key and only attaches a live
        sample whose device ID matches that slot.
        """
        now = self.now if now_s is None else float(now_s)
        params = self.inputs.params or {}
        slots_by_number = {
            int(slot.get("slot")): slot for slot in self.inputs.slots if slot.get("slot", None) is not None
        }
        source_rows: dict[int, dict[str, object]] = {}
        for row in self.magnetometers(side, now_s=now):
            device_id = _integer(row.get("device_id"))
            if device_id is not None:
                source_rows[device_id] = dict(row)
        for row in self.can_magnetometers(side, now_s=now):
            device_id = _integer(row.get("device_id"))
            if device_id is None:
                continue
            existing = source_rows.get(device_id)
            if existing is None or str(row.get("state")) not in {"CONFIG", "WAITING"}:
                source_rows[device_id] = dict(row)
        rows: list[dict[str, object]] = []
        for slot_number in MAG_SLOTS:
            slot = slots_by_number.get(slot_number)
            configured_device_id = _integer(
                slot.get("device_id", None) if slot is not None else params.get(f"CAL_MAG{slot_number}_ID")
            )
            priority = _integer(
                slot.get("priority", None) if slot is not None else params.get(f"CAL_MAG{slot_number}_PRIO")
            )
            info = identify_mag_device(configured_device_id)
            role = mag_role_for_device_id(configured_device_id)
            source_row = source_rows.get(configured_device_id) if configured_device_id is not None else None
            if source_row is None:
                yaw_fields = self._magnetometer_yaw_fields(
                    side, xmag=None, ymag=None, sensor_device_id=configured_device_id, sample_available=False, now_s=now
                )
                row = {
                    "side": side,
                    "pixhawk": "P1" if side == "left" else "P2",
                    "cal_slot": slot_number,
                    "cal_mag": mag_priority_slot_key(slot_number),
                    "sensor": info.device_type if configured_device_id is not None else "N/A",
                    "sensor_role": role or "UNKNOWN",
                    "state": "CONFIG" if configured_device_id is not None else "WAITING",
                    "xmag": None,
                    "ymag": None,
                    "zmag": None,
                    "norm": None,
                    "unit": "G",
                    "age_s": None,
                    "topic": "CAL_MAG configuration",
                    "device_id": configured_device_id,
                    "priority": priority,
                    "source": "PX4 CAL_MAG configuration; waiting for matching sensor sample"
                    if configured_device_id is not None
                    else "PX4 CAL_MAG parameter readback waiting",
                    **yaw_fields,
                    **self._magnetometer_corrected_yaw_fields(
                        side,
                        xmag=None,
                        ymag=None,
                        zmag=None,
                        sensor_device_id=configured_device_id,
                        sample_available=False,
                    ),
                }
            else:
                row = dict(source_row)
                row.update(
                    {
                        "cal_slot": slot_number,
                        "cal_mag": mag_priority_slot_key(slot_number),
                        "sensor": info.device_type,
                        "sensor_role": role or "UNKNOWN",
                        "device_id": configured_device_id,
                        "priority": priority,
                    }
                )
            rows.append(row)
        return tuple(rows)
