"""Instrument actual Qt Ctrl+Q, without replacing any shutdown operation.
Run inside an isolated user/network namespace from repository root.
"""
import faulthandler
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from unittest.mock import patch


def skip(_panel):pass


def main():
    links=json.loads(subprocess.check_output(['ip','-j','link','show'],text=True))
    assert [x['ifname'] for x in links]==['lo']
    root=Path(subprocess.check_output(['git','rev-parse','--show-toplevel'],text=True).strip())
    sys.path[:0]=[str(root/'apps'),str(root/'common/python')]
    faulthandler.register(signal.SIGUSR1)
    with tempfile.TemporaryDirectory(prefix='workstation-ctrl-q-fresh-') as temporary:
        temp=Path(temporary)
        os.environ.update(QT_QPA_PLATFORM='offscreen',SASL_WORKSTATION_DATA_ROOT=str(temp/'data'),
            XDG_CONFIG_HOME=str(temp/'config'),XDG_CACHE_HOME=str(temp/'cache'),MPLCONFIGDIR=str(temp/'mpl'))
        from PyQt5 import QtCore,QtTest,QtWidgets
        from Workstation.standalone_gui import WirisStereoDistancePanel
        from Workstation.standalone_backend import StandalonePanelBackend
        from Workstation.main_window import WorkstationMainWindow
        from Workstation.app_lifecycle_controller import AppLifecycleController
        app=QtWidgets.QApplication([])
        with patch.object(WirisStereoDistancePanel,'_initial_status',skip),patch.object(WirisStereoDistancePanel,'_auto_load_intrinsics',skip):
            panel=WirisStereoDistancePanel(app_root=temp/'data',auto_start=False,auto_start_pixhawk=False)
            app.processEvents()
        backend=StandalonePanelBackend(panel);window=WorkstationMainWindow(backend=backend)
        requested=None
        def log(kind,**details):
            print(json.dumps({'event':kind,'monotonic_s':time.monotonic(),'elapsed_s':None if requested is None else time.monotonic()-requested,**details}),flush=True)
        original=backend.shutdown_for_close
        def shutdown():
            coordinator=panel._shutdown_coordinator
            index=coordinator.index
            log('shutdown_enter',step=coordinator.steps[index].name if index<len(coordinator.steps) else 'done')
            value=original()
            index=coordinator.index
            log('shutdown_return',result=value,step=coordinator.steps[index].name if index<len(coordinator.steps) else 'done')
            return value
        backend.shutdown_for_close=shutdown
        class CloseEvents(QtCore.QObject):
            def eventFilter(self,obj,event):
                if event.type()==QtCore.QEvent.Close:log('close_event',closing=window._closing)
                if event.type()==QtCore.QEvent.Hide:log('window_hidden')
                return False
        observer=CloseEvents(window);window.installEventFilter(observer)
        action=next(a for a in window.menuBar().actions()[0].menu().actions() if a.text()=='Quit')
        action.triggered.connect(lambda *_:log('quit_action_triggered'))
        def press():
            nonlocal requested
            window.activateWindow();window.setFocus();app.processEvents()
            requested=time.monotonic();log('ctrl_q_sent',active=window.isActiveWindow())
            QtTest.QTest.keyClick(window,QtCore.Qt.Key_Q,QtCore.Qt.ControlModifier)
        def pulse():
            log('pulse',visible=window.isVisible(),closing=window._closing,step=panel._shutdown_coordinator.index)
        def watchdog():
            log('timeout',resources=AppLifecycleController(panel)._shutdown_resource_snapshot())
            app.exit(2)
        pulse_timer=QtCore.QTimer(window);pulse_timer.setInterval(500);pulse_timer.timeout.connect(pulse);pulse_timer.start()
        app.aboutToQuit.connect(lambda:log('about_to_quit'))
        window.show();QtCore.QTimer.singleShot(500,press);QtCore.QTimer.singleShot(8000,watchdog)
        code=app.exec_();log('app_exec_return',code=code,visible=window.isVisible())
        return code


if __name__=='__main__':raise SystemExit(main())
