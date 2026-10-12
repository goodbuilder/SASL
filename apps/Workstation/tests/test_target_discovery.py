"""Wire/OS oracles for Target discovery; never touches physical devices."""
import os
import socket
import threading
import time
import pytest
from Workstation.workstation_core.target_discovery import TargetReferenceSettings, discover_targets, ports_from
from Workstation.workstation_core.target_transport import ReceiveOnlyConnection

# MAVLink1 HEARTBEAT, seq7, SYS42/COMP1, payload 00000000/02/03/80/04/03.
# Independently X.25 checked with HEARTBEAT CRC_EXTRA=50: CRC=0xdf06.
HEARTBEAT = bytes.fromhex('fe09072a010000000000020380040306df')


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def test_preferences_validation_and_port_bounds():
    assert TargetReferenceSettings.from_mapping().mode == 'uam'
    assert ports_from('14570,14570,14552') == (14570,14552)
    for value in ('', '0', '65536', ','.join(map(str, range(1,18)))):
        with pytest.raises(ValueError): ports_from(value)
    with pytest.raises(ValueError): TargetReferenceSettings.from_mapping({'endpoint':'/tmp/not-a-device'})
    with pytest.raises(ValueError): TargetReferenceSettings.from_mapping({'expected_component_id':0})
    setting=TargetReferenceSettings.from_mapping({'mode':'drone','endpoint':'udpin:0.0.0.0:14570','expected_system_id':42})
    assert TargetReferenceSettings.from_mapping(setting.to_mapping()) == setting


def test_crc_identity_exclusive_bind_and_no_return_datagram():
    port=free_port()
    endpoint=f'udpin:127.0.0.1:{port}'
    with ReceiveOnlyConnection(endpoint) as connection, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
        sender.bind(('127.0.0.1',0));sender.settimeout(.05)
        with pytest.raises(OSError): ReceiveOnlyConnection(endpoint)
        corrupt=bytearray(HEARTBEAT);corrupt[8]^=1
        sender.sendto(corrupt,('127.0.0.1',port))
        assert connection.wait_heartbeat(timeout=.05) is None
        sender.sendto(HEARTBEAT,('127.0.0.1',port))
        msg=connection.wait_heartbeat(timeout=.2)
        assert msg.get_srcSystem()==42 and msg.get_srcComponent()==1
        assert connection.peer==sender.getsockname()
        with pytest.raises(socket.timeout):sender.recvfrom(1024)
    with ReceiveOnlyConnection(endpoint):pass


def test_selected_peer_filters_other_sender_even_with_identical_ids():
    port=free_port()
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as wanted, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as other:
        wanted.bind(('127.0.0.1',0));other.bind(('127.0.0.1',0))
        with ReceiveOnlyConnection(f'udpin:127.0.0.1:{port}',expected_peer=wanted.getsockname()) as receiver:
            other.sendto(HEARTBEAT,('127.0.0.1',port))
            assert receiver.wait_heartbeat(timeout=.05) is None
            wanted.sendto(HEARTBEAT,('127.0.0.1',port))
            assert receiver.wait_heartbeat(timeout=.2).get_srcSystem()==42
            assert receiver.rejected_peer_count==1


def test_passive_discovery_reports_non_three_identity_and_partial_occupied_port():
    port=free_port();done=threading.Event()
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as occupied, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
        occupied.bind(('127.0.0.1',0))
        def send():
            while not done.wait(.01): sender.sendto(HEARTBEAT,('127.0.0.1',port))
        thread=threading.Thread(target=send);thread.start()
        try:
            result=discover_targets('network',{'ports':[port,occupied.getsockname()[1]]},threading.Event(),duration_s=.2)
        finally:done.set();thread.join()
    assert result['state']=='PARTIAL' and len(result['errors'])==1
    assert result['candidates'][0]['expected_system_id']==42
    with ReceiveOnlyConnection(f'udpin:0.0.0.0:{port}'):pass


def test_cancel_and_active_owner_skip_release_ports():
    port=free_port();cancel=threading.Event();cancel.set()
    result=discover_targets('network',{'ports':[port]},cancel)
    assert result['state']=='CANCELLED'
    result=discover_targets('network',{'ports':[port]},threading.Event(),blocked=[{'endpoint':f'udp:0.0.0.0:{port}','owner':'P1'}])
    assert result['state']=='FAILED' and 'P1' in result['errors'][0]
    with ReceiveOnlyConnection(f'udpin:0.0.0.0:{port}'):pass


def test_usb_pty_reads_without_any_write_and_releases_device():
    import select
    master,slave=os.openpty();path=os.ttyname(slave)
    try:
        with ReceiveOnlyConnection(path) as connection:
            os.write(master,HEARTBEAT)
            assert connection.wait_heartbeat(timeout=.2).get_srcSystem()==42
            assert not select.select([master],[],[],.05)[0]
            with pytest.raises(OSError): ReceiveOnlyConnection(path)
        with ReceiveOnlyConnection(path):pass
    finally:os.close(master);os.close(slave)

from types import SimpleNamespace
from Workstation.target_discovery_controller import TargetDiscoveryController


def make_owner(discover=None, usage=None):
    calls=[]; preferences=TargetReferenceSettings()
    def save(value):
        nonlocal preferences
        preferences=value;calls.append(('save',value))
    owner=TargetDiscoveryController(connection_snapshot=lambda:{'phase':'STOPPED'},
        connect=lambda payload:calls.append(('connect',payload)) or {'phase':'STARTING'},
        stop=lambda **kw:calls.append(('stop',kw)),read_preferences=lambda:preferences,
        save_preferences=save,endpoint_usage=lambda:usage or {},
        **({'discover':discover} if discover else {}))
    return owner,calls


def wait_done(owner):
    deadline=time.monotonic()+2
    while owner.snapshot()['busy'] and time.monotonic()<deadline:time.sleep(.005)
    assert not owner.snapshot()['busy']


def test_cancel_duplicate_generation_and_no_background_apply():
    entered=threading.Event();release=threading.Event()
    def discover(*args,**kwargs):
        entered.set();release.wait(1)
        return {'state':'FOUND','candidates':[{'candidate_id':'old'}],'errors':[]}
    owner,calls=make_owner(discover)
    try:
        first=owner.discover('network',{'mode':'drone'})
        assert entered.wait(1)
        with pytest.raises(RuntimeError):owner.discover('usb',{'mode':'drone'})
        owner.cancel();release.set();wait_done(owner)
        result=owner.snapshot()
        assert result['generation']==first['generation']
        assert result['state']=='CANCELLED' and result['candidates']==[]
        assert not calls
    finally:release.set();assert owner.shutdown()


def test_candidate_selection_connect_pinning_and_saved_preferences_are_separate():
    row={'candidate_id':'observed','transport':'network','endpoint':'udpin:127.0.0.1:14570',
         'baudrate':57600,'expected_system_id':42,'expected_component_id':1,'peer':['127.0.0.1',33001],
         'heartbeat_count':2,'observed_monotonic_s':time.monotonic(),'identity_state':'OBSERVED'}
    owner,calls=make_owner(lambda *a,**k:{'state':'FOUND','candidates':[row],'errors':[]})
    try:
        snap=owner.discover('network',{'mode':'drone'});wait_done(owner)
        selected=owner.use_candidate('observed',snap['generation'])
        assert calls==[]
        payload={**selected,'mode':'drone','discovery_generation':snap['generation']}
        owner.save(payload)
        assert len(calls)==1 and calls[0][0]=='save'
        assert 'peer' not in owner.snapshot()['preferences']
        owner.connect(payload)
        assert calls[-1][1]['expected_peer']==row['peer']
        assert owner.snapshot()['candidates']==[]
    finally:assert owner.shutdown()


def test_busy_endpoint_rejection_and_uam_no_discovery():
    owner,calls=make_owner(usage={'endpoints':[{'endpoint':'udp:0.0.0.0:14570','owner':'Telemetry'}]})
    with pytest.raises(RuntimeError,match='Telemetry'):
        owner.connect({'mode':'drone','endpoint':'udpin:127.0.0.1:14570'})
    with pytest.raises(ValueError,match='UAM'):owner.discover('network',{'mode':'uam'})
    assert calls==[]
    assert owner.shutdown()


def test_target_preferences_persist_atomically_without_runtime_evidence(tmp_path,monkeypatch):
    from Workstation.workstation_core.settings import AppSettings, load_settings
    from Workstation.target_drone_controller import TargetDroneController
    import Workstation.workstation_core.settings as settings_module
    panel=SimpleNamespace(settings=AppSettings(),settings_path=tmp_path/'settings.json')
    owner=TargetDroneController(panel)
    payload={'mode':'drone','endpoint':'udpin:127.0.0.1:14570','expected_system_id':42}
    owner.target_reference_save(payload)
    assert load_settings(panel.settings_path).target_reference.expected_system_id==42
    old=panel.settings;raw=panel.settings_path.read_bytes()
    def fail(*args):raise OSError('disk full')
    monkeypatch.setattr(settings_module.os,'replace',fail)
    with pytest.raises(OSError):owner.target_reference_save({**payload,'expected_system_id':43})
    assert panel.settings is old and panel.settings_path.read_bytes()==raw


def test_production_receiver_udp_identity_gps_freshness_and_close(tmp_path):
    from Workstation.workstation_core.target_drone_stream import TargetDroneReceiver,TargetDroneWorker
    from Workstation.workstation_core.mavlink_sources import MavlinkSourceConfig
    from pymavlink.dialects.v20 import common
    port=free_port();done=threading.Event()
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
        sender.bind(('127.0.0.1',0))
        encoder=common.MAVLink(None,srcSystem=42,srcComponent=1)
        gps=encoder.gps_raw_int_encode(1700000000000000,6,375000000,1270000000,123000,10,10,0,0,15).pack(encoder)
        receiver=TargetDroneReceiver(MavlinkSourceConfig.target_drone(f'udpin:127.0.0.1:{port}',expected_system_id=42,expected_component_id=1),
            mode='drone',spool_path=tmp_path/'raw.jsonl',expected_peer=sender.getsockname())
        worker=TargetDroneWorker(receiver)
        def send():
            while not done.wait(.01):
                sender.sendto(HEARTBEAT,('127.0.0.1',port));sender.sendto(gps,('127.0.0.1',port))
        thread=threading.Thread(target=send);thread.start();worker.start()
        try:
            deadline=time.monotonic()+2
            while receiver.snapshot().accepted_message_count==0 and time.monotonic()<deadline:time.sleep(.01)
            assert receiver.reception_status()['samples']['GPS_RAW_INT']['state']=='VALID'
            assert receiver.reception_status()['samples']['GPS_RAW_INT']['rtk']=='RTK_FIXED'
            assert receiver.reception_status()['samples']['GPS2_RAW']['state']=='UNKNOWN'
            assert receiver.last_gps('GPS1').fields['latitude_deg_wgs84']==37.5
        finally:done.set();thread.join();worker.stop(timeout_s=1)
    with ReceiveOnlyConnection(f'udpin:127.0.0.1:{port}'):pass


def test_real_network_cancel_is_bounded_and_releases_listener():
    port=free_port();cancel=threading.Event();result=[]
    thread=threading.Thread(target=lambda:result.append(discover_targets('network',{'ports':[port]},cancel)))
    thread.start();time.sleep(.05);started=time.monotonic();cancel.set();thread.join(.5)
    assert not thread.is_alive() and time.monotonic()-started<.5
    assert result[0]['state']=='CANCELLED'
    with ReceiveOnlyConnection(f'udpin:0.0.0.0:{port}'):pass


def test_usb_present_without_heartbeat_is_visible_but_unverified():
    master,slave=os.openpty();path=os.ttyname(slave)
    try:
        result=discover_targets('usb',{'mode':'drone'},threading.Event(),devices=[path,path],duration_s=.05)
        assert result['state']=='PARTIAL'
        assert len(result['candidates'])==1
        assert result['candidates'][0]['identity_state']=='NO HEARTBEAT'
        assert result['candidates'][0]['expected_system_id'] is None
    finally:os.close(master);os.close(slave)


def test_parser_queue_has_bounded_overflow_under_packet_burst():
    port=free_port()
    with ReceiveOnlyConnection(f'udpin:127.0.0.1:{port}') as receiver:
        receiver._parse(HEARTBEAT*300,('127.0.0.1',40000))
        assert len(receiver._messages)==256 and receiver.dropped==44


def test_udp_exclusion_is_enforced_across_processes():
    import subprocess,sys
    port=free_port()
    with ReceiveOnlyConnection(f'udpin:127.0.0.1:{port}'):
        result=subprocess.run([sys.executable,'-c',
            "import socket,sys; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); "
            "s.bind(('127.0.0.1',int(sys.argv[1])))",str(port)],capture_output=True,text=True)
    assert result.returncode!=0 and 'Address already in use' in result.stderr


def test_expired_selection_is_rejected_and_new_discovery_ignores_old_transport_draft():
    row={'candidate_id':'expired','observed_monotonic_s':time.monotonic()-31,'identity_state':'OBSERVED'}
    requests=[]
    def discover(kind,prefs,*args,**kwargs):
        requests.append((kind,prefs))
        return {'state':'FOUND','candidates':[row],'errors':[]}
    owner,calls=make_owner(discover)
    try:
        snap=owner.discover('network',{'mode':'drone','transport':'usb','endpoint':'/dev/test-old-radio'})
        wait_done(owner)
        assert requests[0][1]['endpoint']==''
        with pytest.raises(ValueError,match='expired'):owner.use_candidate('expired',snap['generation'])
        assert not calls
    finally:assert owner.shutdown()


def test_existing_usb_and_gnss_probes_do_not_open_a_leased_device(monkeypatch):
    from Workstation.workstation_core.serial_endpoint_lease import acquire_serial_endpoint
    from Workstation.workstation_core.pixhawk_attitude import probe_usb_mavlink_devices
    from Workstation.workstation_core.gnss_protocol import probe_gnss_base_serial_ports
    from pymavlink import mavutil
    import serial
    opened=[]
    def forbidden(*args,**kwargs):
        opened.append(args)
        raise AssertionError('Busy device must not be opened')
    monkeypatch.setattr(mavutil,'mavlink_connection',forbidden)
    monkeypatch.setattr(serial,'Serial',forbidden)
    with acquire_serial_endpoint('/dev/target-test-exclusive'):
        assert probe_usb_mavlink_devices(['/dev/target-test-exclusive'],baudrates=(57600,))==[]
        rows=probe_gnss_base_serial_ports(['/dev/target-test-exclusive'],baud=57600)
        assert 'busy' in rows[0]['status']
    assert opened==[]
