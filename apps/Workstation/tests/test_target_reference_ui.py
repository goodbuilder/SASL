"""Real panel/backend/view tests with loopback MAVLink and no device startup."""
from pathlib import Path
import socket
import threading
import time
from types import SimpleNamespace
import pytest
from PyQt5 import QtCore,QtTest,QtWidgets
from Workstation.standalone_gui import WirisStereoDistancePanel
from Workstation.standalone_backend import StandalonePanelBackend
from Workstation.workspace_views.target_reference import TargetReferenceView
from Workstation.workstation_core.target_discovery import discover_targets

HEARTBEAT=bytes.fromhex('fe09072a010000000000020380040306df')


def no_startup(_self):pass


@pytest.fixture
def target(monkeypatch,tmp_path):
    monkeypatch.setattr(WirisStereoDistancePanel,'_initial_status',no_startup)
    monkeypatch.setattr(WirisStereoDistancePanel,'_auto_load_intrinsics',no_startup)
    panel=WirisStereoDistancePanel(app_root=tmp_path,auto_start=False,auto_start_pixhawk=False)
    app=QtWidgets.QApplication.instance();app.processEvents()
    backend=StandalonePanelBackend(panel)
    view=TargetReferenceView(backend)
    view.resize(1100,450);view.show();app.processEvents()
    yield panel,backend,view
    owner=getattr(panel,'_target_drone_controller_owner',None)
    if owner:
        owner.shutdown_discovery()
        if owner._worker:owner.stop_target_drone_stream(timeout_s=1)
    view.close();view.deleteLater();panel.close();panel.deleteLater();app.processEvents()


def wait_for(predicate,view,timeout=3):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        QtWidgets.QApplication.instance().processEvents();view.refresh(force=True)
        if predicate():return
        QtTest.QTest.qWait(10)
    raise AssertionError('Target UI condition timed out')


def test_real_button_discovery_select_connect_stop_save_and_freshness(target):
    panel,backend,view=target
    assert view.platform.currentData()=='uam' and not view.find_network.isEnabled()
    view.connect_button.click()
    assert backend.target_drone_snapshot()['phase']=='DEFERRED'
    assert backend.target_drone_snapshot()['worker_alive'] is False
    view.platform.setCurrentIndex(view.platform.findData('drone'))
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as reserve:
        reserve.bind(('127.0.0.1',0));port=reserve.getsockname()[1]
    view.ports.setText(str(port))
    owner=panel._target_drone_controller()
    owner._discovery()._discover=lambda *a,**kw:discover_targets(*a,**kw,duration_s=.2)
    done=threading.Event()
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
        sender.bind(('127.0.0.1',0));sender.settimeout(.05)
        def send():
            while not done.wait(.01):sender.sendto(HEARTBEAT,('127.0.0.1',port))
        thread=threading.Thread(target=send);thread.start()
        try:
            view.find_network.click()
            wait_for(lambda:view.model.rowCount()==1 and not view._busy,view)
            assert view.model.item(0,3).text()=='42'
            view.table.selectRow(0);view.use.click()
            assert view.system.value()==42 and view.component.value()==1
            assert backend.target_drone_snapshot()['phase']=='DEFERRED'
            assert panel.settings.target_reference.mode=='uam'
            view.connect_button.click()
            wait_for(lambda:backend.target_drone_snapshot()['stream_state']=='STREAMING',view)
            assert not view.platform.isEnabled()
            assert 'GPS1 UNKNOWN' in view.samples_label.text()
            assert backend.target_drone_snapshot()['reception']['peer']==sender.getsockname()
            with pytest.raises(socket.timeout):sender.recvfrom(1000)
            view.stop_button.click()
            wait_for(lambda:not view._busy and not backend.target_drone_snapshot()['worker_alive'],view)
            view.save_button.click()
            assert panel.settings.target_reference.expected_system_id==42
            assert panel.settings_path.is_file()
            from Workstation.workstation_core.settings import load_settings
            assert load_settings(panel.settings_path).target_reference.endpoint==f'udpin:0.0.0.0:{port}'
        finally:done.set();thread.join()


def test_refresh_keeps_manual_draft_focus_and_handles_long_errors(target):
    panel,backend,view=target
    view.platform.setCurrentIndex(view.platform.findData('drone'))
    view.endpoint.setText('udpin:127.0.0.1:19001');view.endpoint.setFocus()
    before=view.geometry();focus=QtWidgets.QApplication.focusWidget()
    for _ in range(3):view.refresh(force=True)
    assert view.endpoint.text()=='udpin:127.0.0.1:19001'
    assert QtWidgets.QApplication.focusWidget() is focus
    view._message('failed '+('very-long-detail '*200))
    assert view.geometry()==before and len(view.message.text())<=140
    assert len(view.message.toolTip())>1000


def test_active_other_owner_blocks_target_without_opening_it(target):
    panel,backend,view=target
    panel.mavlink_pixhawk=SimpleNamespace(urls={'left':'udpin:0.0.0.0:14570'},connections={'left':object()})
    try:
        with pytest.raises(RuntimeError,match='Pixhawk/left'):
            backend.target_reference_connect({'mode':'drone','endpoint':'udpin:127.0.0.1:14570'})
        assert not backend.target_drone_snapshot()['worker_alive']
    finally:panel.mavlink_pixhawk=None


def test_shutdown_cancels_discovery_and_inventory_tracks_thread(target):
    panel,backend,view=target
    entered=threading.Event()
    def observe(kind,prefs,cancel,**kw):
        entered.set();cancel.wait(1)
        return {'state':'FOUND','candidates':[],'errors':[]}
    owner=panel._target_drone_controller();owner._discovery()._discover=observe
    backend.target_reference_discover('network',{'mode':'drone'})
    assert entered.wait(1)
    assert panel.application.resource_inventory()['target_discovery_thread'].is_alive()
    assert owner.shutdown_discovery(timeout_s=1)
    assert not panel.application.resource_inventory()['target_discovery_thread'].is_alive()


def test_background_stop_publication_returns_to_gui_thread(target):
    panel,backend,view=target
    observed=[]
    panel.measurement_source_transition=lambda *args:observed.append((QtCore.QThread.currentThread(),args))
    owner=panel._target_drone_controller()
    thread=threading.Thread(target=lambda:owner._publish_measurement_source('LEFT','stopped'))
    thread.start();thread.join()
    wait_for(lambda:bool(observed),view)
    assert observed[0][0] is panel.thread()
    assert observed[0][1]==('target','LEFT','stopped')
