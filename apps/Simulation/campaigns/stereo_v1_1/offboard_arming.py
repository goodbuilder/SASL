"""v1.2 arming admission while preserving the held offboard setpoint stream.

The caller owns held PX4 local NED position/yaw and emergency landing. This
module sends no trajectory commands and never force-arms or changes parameters.
ACK indicates command handling; only a target ARMED+OFFBOARD heartbeat admits
flight. Recorded ACK loss/denial is evidence, not a diagnosis of PX4's reason.
"""
from __future__ import annotations

import math
import time

from pymavlink import mavutil


def px4_offboard_mode_evidence(message):
    """Decode PX4 custom main mode; display formatters are not admission gates.

    PX4 px4_custom_mode.h defines main_mode at bits 16..23 and sub_mode at
    bits 24..31. Only a PX4 heartbeat with CUSTOM_MODE_ENABLED is authoritative.
    """
    autopilot = int(message.autopilot)
    vehicle_type = int(message.type)
    base_mode = int(message.base_mode)
    custom_mode = int(message.custom_mode)
    main_mode = (custom_mode >> 16) & 0xff
    sub_mode = (custom_mode >> 24) & 0xff
    custom_enabled = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
    offboard = (autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4
                and custom_enabled and main_mode == mavutil.PX4_CUSTOM_MAIN_MODE_OFFBOARD)
    return {"offboard": offboard, "autopilot": autopilot, "vehicle_type": vehicle_type,
            "custom_main_mode": main_mode, "custom_sub_mode": sub_mode,
            "custom_mode_enabled": custom_enabled}


def arm_with_offboard_stream(master, send_hold, emit, check_stop, *,
                             hz=50.0, attempts=4, ack_timeout_s=5.0,
                             retry_delay_s=2.0):
    if not math.isfinite(hz) or hz <= 0 or attempts < 1:
        raise ValueError('offboard arming: invalid stream rate or attempt count')
    target = (int(master.target_system), int(master.target_component))
    if min(target) <= 0:
        raise ValueError('offboard arming: identified target system/component required')
    period = 1.0/hz
    next_send = time.monotonic()
    command = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    seen_offboard = False
    last_ack = None
    active_attempt = 0
    sent_count = 0
    last_send_ns = None
    max_send_gap_ns = 0

    def event(name, **details):
        emit(name, host_monotonic_ns=time.monotonic_ns(), **details)

    def stream_if_due():
        nonlocal next_send, sent_count, last_send_ns, max_send_gap_ns
        check_stop()
        now = time.monotonic()
        if now >= next_send:
            send_hold()
            sent_ns = time.monotonic_ns()
            if last_send_ns is not None:
                max_send_gap_ns = max(max_send_gap_ns, sent_ns-last_send_ns)
            last_send_ns = sent_ns
            sent_count += 1
            # Missed wall ticks are not replayed as bursts.
            next_send = max(next_send+period, now+period)

    def handle(msg, *, eligible):
        nonlocal seen_offboard, last_ack
        kind = msg.get_type()
        source = (msg.get_srcSystem(), msg.get_srcComponent())
        if source != target:
            event('offboard_arm_message_ignored', message_type=kind,
                  source_system=source[0], source_component=source[1], reason='wrong_source')
            return None
        if kind == 'STATUSTEXT':
            event('offboard_arm_statustext', severity=int(msg.severity), text=str(msg.text))
        elif kind == 'COMMAND_ACK':
            event('offboard_arm_ack', command=int(msg.command), result=int(msg.result),
                  attempt=active_attempt, eligible=eligible and int(msg.command)==command,
                  progress=getattr(msg,'progress',None), result_param2=getattr(msg,'result_param2',None))
            if eligible and int(msg.command)==command:
                last_ack = int(msg.result)
                if last_ack not in (mavutil.mavlink.MAV_RESULT_ACCEPTED,
                                    mavutil.mavlink.MAV_RESULT_IN_PROGRESS):
                    return 'rejected'
        elif kind == 'HEARTBEAT':
            mode_evidence = px4_offboard_mode_evidence(msg)
            mode = 'OFFBOARD' if mode_evidence['offboard'] else 'NOT_PX4_OFFBOARD'
            armed = bool(int(msg.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            event('offboard_arm_heartbeat', mode=mode, armed=armed,
                  base_mode=int(msg.base_mode), custom_mode=int(msg.custom_mode),
                  system_status=int(msg.system_status), attempt=active_attempt, eligible=eligible,
                  **mode_evidence)
            if eligible:
                if mode != 'OFFBOARD' and (seen_offboard or armed):
                    raise RuntimeError(f'offboard arming: OFFBOARD mode lost ({mode}, armed={armed})')
                seen_offboard = seen_offboard or mode=='OFFBOARD'
                if armed and mode=='OFFBOARD':
                    return 'armed'
        return None

    def pump_until(deadline, *, stop_on_rejection):
        while time.monotonic() < deadline:
            stream_if_due()
            timeout = max(0.0,min(period,next_send-time.monotonic(),deadline-time.monotonic()))
            message = master.recv_match(type=['COMMAND_ACK','STATUSTEXT','HEARTBEAT'],
                                        blocking=True,timeout=timeout)
            if message is not None:
                state = handle(message,eligible=True)
                if state=='armed' or (state=='rejected' and stop_on_rejection):
                    return state
        return None

    try:
        # Discard queued admission evidence from before this arm procedure while
        # retaining its diagnostics and continuing the exact held setpoint stream.
        drain_deadline = time.monotonic()+.2
        while time.monotonic()<drain_deadline:
            stream_if_due()
            msg = master.recv_match(type=['COMMAND_ACK','STATUSTEXT','HEARTBEAT'],blocking=False)
            if msg is None:
                break
            handle(msg,eligible=False)
        for active_attempt in range(1,attempts+1):
            stream_if_due()
            master.mav.command_long_send(target[0],target[1],command,0,
                                         1.0,0.0,0.0,0.0,0.0,0.0,0.0)
            event('offboard_arm_requested',attempt=active_attempt,force=False)
            outcome = pump_until(time.monotonic()+ack_timeout_s,stop_on_rejection=True)
            if outcome=='armed':
                event('offboard_arm_confirmed',attempt=active_attempt,last_ack_result=last_ack)
                return
            event('offboard_arm_attempt_incomplete',attempt=active_attempt,
                  outcome=outcome or 'armed_heartbeat_timeout',last_ack_result=last_ack)
            if active_attempt<attempts:
                if pump_until(time.monotonic()+retry_delay_s,stop_on_rejection=False)=='armed':
                    event('offboard_arm_confirmed',attempt=active_attempt,last_ack_result=last_ack)
                    return
        raise RuntimeError(f'offboard arming: target ARMED+OFFBOARD heartbeat not confirmed after {attempts} attempts; last ACK={last_ack}')
    finally:
        event('offboard_arm_stream_summary',setpoint_count=sent_count,
              requested_host_hz=hz,max_host_send_gap_ns=max_send_gap_ns)
