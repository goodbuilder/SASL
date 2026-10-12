"""Value-only diagnostics text; QWidget effects stay in the outer adapter.

Sensor offsets preserve producer sensor axes and m/s² or rad/s units. Board
fine offsets are degrees. No sensor vector is re-labelled as NED or optical RDF.
"""


def parameter_messages(outcome, label):
    if outcome.kind == "mag_refresh":
        status = outcome.mag_status
        slots = ", ".join(
            f"CAL_MAG{getattr(slot, 'slot', '?')}="
            f"{getattr(slot, 'device_id', None)}/prio={getattr(slot, 'priority', None)}"
            for slot in (getattr(status, "slots", ()) or ())
        ) or "slots N/A"
        missing = outcome.missing_readback or ()
        if missing:
            preview = ", ".join(missing[:6]) + (" …" if len(missing) > 6 else "")
            return (f"[PX4 MAG] {label} CAL_MAG PARAM readback "
                    f"불완전 ({len(missing)}개 미수신: {preview}): {slots}",)
        return (f"[PX4 MAG] {label} CAL_MAG PARAM readback 완료: {slots}",)
    if outcome.kind == "mag_apply":
        lines = [f"[PX4 MAG] {label} Workstation PARAM 적용: {', '.join(outcome.applied) or 'none'}"]
        if outcome.missing_roles:
            lines.append(f"[PX4 MAG] {label} 미적용 role: {', '.join(outcome.missing_roles)}")
        return tuple(lines)
    return ()


def attitude_cal_status_text(label, status, *, format_value):
    lines = [f"{label} | PX4 자세 보정 상태"]
    for slot in status.accel_slots:
        if slot.device_id is None:
            continue
        offset_text = ", ".join(format_value(v) for v in slot.offset_m_s2)
        scale_text = ", ".join(format_value(v) for v in slot.scale)
        lines.append(
            f"ACC{slot.slot}: id={slot.device_id} prio={slot.priority} "
            f"calibrated={'Y' if slot.calibrated else 'N'} offset(m/s^2)=({offset_text}) scale=({scale_text})"
        )
    for slot in status.gyro_slots:
        if slot.device_id is None:
            continue
        offset_text = ", ".join(format_value(v) for v in slot.offset_rad_s)
        lines.append(
            f"GYRO{slot.slot}: id={slot.device_id} prio={slot.priority} "
            f"calibrated={'Y' if slot.calibrated else 'N'} offset(rad/s)=({offset_text})"
        )
    board = status.board_rotation
    level_text = ", ".join("N/A" if v is None else f"{format_value(v)}deg" for v in board.level_offset_deg)
    lines.append(f"BOARD_ROT: {board.rotation_label} level_offset=({level_text})")
    if status.warning_text:
        lines.append(status.warning_text)
    return "\n".join(lines)
