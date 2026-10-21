"""Arming retains offboard stream and requires actual target armed-mode evidence."""
import sys
from pathlib import Path
from types import SimpleNamespace
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from campaigns.stereo_v1_1 import offboard_arming as arming


class Message(SimpleNamespace):
    def get_type(self):return self.kind
    def get_srcSystem(self):return getattr(self,'source_system',1)
    def get_srcComponent(self):return getattr(self,'source_component',1)


def ack(result=0,**kw):return Message(kind='COMMAND_ACK',command=400,result=result,**kw)
def heartbeat(armed=True,mode='OFFBOARD',**kw):
    return Message(kind='HEARTBEAT',mode=mode,base_mode=(128 if armed else 0)|1,
                   custom_mode=393216 if mode=='OFFBOARD' else 196608,system_status=3,
                   autopilot=arming.mavutil.mavlink.MAV_AUTOPILOT_PX4,
                   type=arming.mavutil.mavlink.MAV_TYPE_QUADROTOR,**kw)


@pytest.fixture
def harness(monkeypatch):
    wall=[0.0];sent=[];events=[];commands=[];messages=[]
    monkeypatch.setattr(arming.time,'monotonic',lambda:wall[0])
    monkeypatch.setattr(arming.time,'monotonic_ns',lambda:round(wall[0]*1e9))
    def receive(*,type,blocking,timeout=0):
        messages.sort(key=lambda row:row[0])
        end=wall[0]+timeout if blocking else wall[0]
        if messages and messages[0][0]<=end:
            when,msg=messages.pop(0);wall[0]=max(wall[0],when);return msg
        wall[0]=end
        return None
    master=SimpleNamespace(target_system=1,target_component=1,
        mav=SimpleNamespace(command_long_send=lambda *a:commands.append((wall[0],a))),recv_match=receive)
    def run(**kw):
        arming.arm_with_offboard_stream(master,lambda:sent.append(wall[0]),
            lambda event,**details:events.append((event,details)),kw.pop('check_stop',lambda:None),**kw)
    return SimpleNamespace(wall=wall,sent=sent,events=events,commands=commands,messages=messages,run=run)


def assert_stream(h):
    assert len(h.sent)>2
    assert max(b-a for a,b in zip(h.sent,h.sent[1:]))<=.020000001
    assert all(c[1][4:]==(1.0,0.0,0.0,0.0,0.0,0.0,0.0) for c in h.commands)


def test_missing_ack_still_streams_until_actual_target_armed_heartbeat(harness):
    h=harness;h.messages.extend([(.5,heartbeat())]);h.run()
    assert_stream(h)
    assert len(h.commands)==1
    assert any(n=='offboard_arm_confirmed' for n,_ in h.events)


def test_wrong_source_ack_ignored_statustext_retained_and_inprogress_not_success(harness):
    h=harness
    h.messages.extend([(.1,ack(source_system=2)),(.15,Message(kind='STATUSTEXT',severity=4,text='Preflight check')),
                       (.2,ack(result=5)),(.5,heartbeat())])
    h.run()
    assert h.wall[0]==.5
    assert any(n=='offboard_arm_message_ignored' for n,_ in h.events)
    assert any(n=='offboard_arm_statustext' and d['text']=='Preflight check' for n,d in h.events)
    assert_stream(h)


def test_ack_accepted_without_armed_heartbeat_cannot_pass(harness):
    h=harness;h.messages.append((.1,ack()))
    with pytest.raises(RuntimeError,match='not confirmed'):
        h.run(attempts=1,ack_timeout_s=.3)
    assert_stream(h)


def test_retry_delays_stream_and_failure_preserves_original_bounds(harness):
    h=harness
    with pytest.raises(RuntimeError,match='after 4 attempts'):
        h.run()
    assert len(h.commands)==4
    assert h.wall[0]==pytest.approx(26)
    assert_stream(h)
    assert h.commands[1][0]-h.commands[0][0]==pytest.approx(7)


def test_temporary_rejection_delay_preserves_stream(harness):
    h=harness;h.messages.extend([(.1,ack(result=1)),(2.3,heartbeat())]);h.run()
    assert len(h.commands)==2
    assert h.commands[1][0]==pytest.approx(2.1)
    assert_stream(h)


def test_mode_loss_and_wrong_target_armed_heartbeat_do_not_admit(harness):
    h=harness;h.messages.extend([(.1,heartbeat(source_component=2)),(.2,heartbeat(False)),
                               (.3,heartbeat(False,mode='POSCTL'))])
    with pytest.raises(RuntimeError,match='OFFBOARD mode lost'):
        h.run()
    assert_stream(h)


def test_cancel_preserves_evidence_and_stops_commands(harness):
    h=harness
    def cancel():
        if h.wall[0]>=.1:raise KeyboardInterrupt('cancelled')
    with pytest.raises(KeyboardInterrupt):h.run(check_stop=cancel)
    assert len(h.commands)==1
    assert h.events[-1][0]=='offboard_arm_stream_summary'


def test_prequeued_armed_heartbeat_is_not_new_admission_evidence(harness):
    h=harness;h.messages.append((0,heartbeat()))
    with pytest.raises(RuntimeError,match='not confirmed'):
        h.run(attempts=1,ack_timeout_s=.1)


def parsed_wire_heartbeat(*,base_mode=145,custom_mode=393216,autopilot=None):
    mavlink = arming.mavutil.mavlink
    sender = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
    message = mavlink.MAVLink_heartbeat_message(
        type=mavlink.MAV_TYPE_QUADROTOR,
        autopilot=mavlink.MAV_AUTOPILOT_PX4 if autopilot is None else autopilot,
        base_mode=base_mode,custom_mode=custom_mode,system_status=4,mavlink_version=3)
    packet = message.pack(sender)
    parsed = mavlink.MAVLink(None).parse_char(packet)
    assert (parsed.get_srcSystem(),parsed.get_srcComponent()) == (1,1)
    return parsed


def test_real_parsed_px4_armed_offboard_heartbeat_is_authoritative(harness):
    h=harness
    parsed=parsed_wire_heartbeat()
    h.messages.append((.2,parsed))
    h.run()
    row=next(d for n,d in h.events if n=='offboard_arm_heartbeat')
    assert row['mode']=='OFFBOARD' and row['armed']
    assert row['custom_main_mode']==6 and row['custom_sub_mode']==0
    assert row['base_mode']==145 and row['custom_mode']==393216
    assert row['autopilot']==arming.mavutil.mavlink.MAV_AUTOPILOT_PX4


@pytest.mark.parametrize('override', [
    {'custom_mode':3<<16},
    {'base_mode':144},
    {'autopilot':arming.mavutil.mavlink.MAV_AUTOPILOT_GENERIC},
])
def test_real_parsed_negative_mode_contract_cannot_admit_armed_target(harness,override):
    h=harness;h.messages.append((.2,parsed_wire_heartbeat(**override)))
    with pytest.raises(RuntimeError,match='OFFBOARD mode lost'):
        h.run()
