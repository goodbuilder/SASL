#!/usr/bin/env python3
"""Actual Target panel capture with loopback-only MAVLink. No physical devices.

Run from repository root in an unshared user/network namespace. QT_SCALE_FACTOR
may be set before launch. --width/--height are logical window dimensions.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
import traceback
from unittest.mock import patch


def skip_startup(_panel):
    pass


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--width',type=int,default=1366)
    parser.add_argument('--height',type=int,default=900)
    args=parser.parse_args()
    links=json.loads(subprocess.check_output(['ip','-j','link','show'],text=True))
    if [link['ifname'] for link in links]!=['lo']:
        raise SystemExit('Capture requires a loopback-only network namespace')
    subprocess.run(['ip','link','set','lo','up'],check=True)
    root=Path(__file__).resolve().parents[3]
    sys.path[:0]=[str(root/'apps'),str(root/'common/python')]
    os.environ['QT_QPA_PLATFORM']='offscreen'
    os.environ['QT_OPENGL']='software'
    os.environ['LIBGL_ALWAYS_SOFTWARE']='1'
    args.output.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='target-reference-capture-') as temporary:
        data=Path(temporary)
        for name,part in [('SASL_WORKSTATION_DATA_ROOT','data'),('XDG_CONFIG_HOME','config'),('XDG_CACHE_HOME','cache'),('MPLCONFIGDIR','mpl'),('QT_SHADER_CACHE_PATH','shader')]:
            os.environ[name]=str(data/part)
        from PyQt5 import QtCore,QtTest,QtWidgets
        from Workstation.application.standalone_gui import WirisStereoDistancePanel
        from Workstation.application.standalone_backend import StandalonePanelBackend
        from Workstation.application.main_window import WorkstationMainWindow
        from Workstation.controllers.app_lifecycle_controller import AppLifecycleController
        from Workstation.workstation_core.target_discovery import discover_targets
        errors=[];rendering_limits=[]
        def error_hook(kind,value,tb):
            known_gl=(isinstance(value,RuntimeError) and str(value).startswith('pyqtgraph.opengl: Requires >= OpenGL 2.1;')
                      and any(frame.filename.endswith('pyqtgraph/opengl/GLViewWidget.py') for frame in traceback.extract_tb(tb)))
            (rendering_limits if known_gl else errors).append(f'{kind.__name__}: {value}')
            traceback.print_exception(kind,value,tb)
        sys.excepthook=error_hook
        app=QtWidgets.QApplication([])
        with patch.object(WirisStereoDistancePanel,'_initial_status',skip_startup),patch.object(WirisStereoDistancePanel,'_auto_load_intrinsics',skip_startup):
            panel=WirisStereoDistancePanel(app_root=data/'data',auto_start=False,auto_start_pixhawk=False)
            app.processEvents()
        backend=StandalonePanelBackend(panel)
        window=WorkstationMainWindow(backend=backend)
        window.resize(args.width,args.height);window.show()
        window.navigation.select_context('tasks.data');QtTest.QTest.qWait(100)
        view=window.recording.target_reference
        observations=[]
        def capture(name):
            app.processEvents();view.refresh(force=True)
            assert window.grab().save(str(args.output/f'{name}.png'))
            observations.append({'name':name,'window_size':[window.width(),window.height()],
                'target_geometry':[view.x(),view.y(),view.width(),view.height()]})
        capture('uam')
        view.platform.setCurrentIndex(view.platform.findData('drone'));capture('drone-empty')
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as reserve:
            reserve.bind(('127.0.0.1',0));port=reserve.getsockname()[1]
        view.ports.setText(str(port))
        owner=panel._target_drone_controller()
        owner._discovery()._discover=lambda *a,**kw:discover_targets(*a,**kw,duration_s=.2)
        stop=threading.Event()
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
            def emit():
                while not stop.wait(.02):sender.sendto(bytes.fromhex('fe09072a010000000000020380040306df'),('127.0.0.1',port))
            thread=threading.Thread(target=emit);thread.start()
            try:
                view.find_network.click()
                deadline=time.monotonic()+3
                while time.monotonic()<deadline:
                    QtTest.QTest.qWait(20);view.refresh(force=True)
                    if view.model.rowCount() and not view._busy:break
                assert view.model.rowCount()==1 and not view._busy
                view.table.selectRow(0);view.use.click();capture('candidate-selected')
                view.connect_button.click()
                deadline=time.monotonic()+3
                while time.monotonic()<deadline:
                    QtTest.QTest.qWait(20);view.refresh(force=True)
                    if backend.target_drone_snapshot()['stream_state']=='STREAMING':break
                assert backend.target_drone_snapshot()['stream_state']=='STREAMING'
                capture('heartbeat-no-gps')
                view._message('Connection detail: '+('long-endpoint-or-error '*100))
                capture('long-detail')
                view.stop_button.click()
                deadline=time.monotonic()+3
                while time.monotonic()<deadline:
                    QtTest.QTest.qWait(20);view.refresh(force=True)
                    if not view._busy and not backend.target_drone_snapshot()['worker_alive']:break
                assert not view._busy and not backend.target_drone_snapshot()['worker_alive']
            finally:stop.set();thread.join()
        window.close();QtTest.QTest.qWait(100);app.processEvents()
        shutdown=AppLifecycleController(panel)._shutdown_resource_snapshot()
        summary={'profiles':observations,'scale':os.environ.get('QT_SCALE_FACTOR','1'),
                 'callback_errors':errors,'rendering_limitations':rendering_limits,'shutdown':shutdown}
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        assert not errors,errors
        assert shutdown['complete'],shutdown
        print(json.dumps(summary))


if __name__=='__main__':
    main()
