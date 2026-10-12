"""Two fresh processes prove saved Target UI preferences without auto-connect.
Run from the repo root inside an isolated user/network namespace.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from unittest.mock import patch


def skip(_panel):
    pass


def phase(mode, root):
    repo=Path(subprocess.check_output(['git','rev-parse','--show-toplevel'],text=True).strip())
    sys.path[:0]=[str(repo/'apps'),str(repo/'common/python')]
    os.environ.update(QT_QPA_PLATFORM='offscreen',SASL_WORKSTATION_DATA_ROOT=str(root/'data'),
                      XDG_CONFIG_HOME=str(root/'config'),XDG_CACHE_HOME=str(root/'cache'),
                      MPLCONFIGDIR=str(root/'mpl'))
    from PyQt5 import QtWidgets
    from Workstation.standalone_gui import WirisStereoDistancePanel
    from Workstation.standalone_backend import StandalonePanelBackend
    from Workstation.workspace_views.target_reference import TargetReferenceView
    errors=[]
    sys.excepthook=lambda kind,value,tb:errors.append(str(value))
    app=QtWidgets.QApplication([])
    with patch.object(WirisStereoDistancePanel,'_initial_status',skip),patch.object(WirisStereoDistancePanel,'_auto_load_intrinsics',skip):
        panel=WirisStereoDistancePanel(app_root=root/'data',auto_start=False,auto_start_pixhawk=False)
        app.processEvents()
    backend=StandalonePanelBackend(panel)
    if mode=='write':
        backend.target_reference_save({'mode':'drone','endpoint':'udpin:127.0.0.1:14570','expected_system_id':42,'expected_component_id':1})
    view=TargetReferenceView(backend)
    assert view.platform.currentData()=='drone'
    assert view.endpoint.text()=='udpin:127.0.0.1:14570'
    assert view.system.value()==42 and view.component.value()==1
    snapshot=backend.target_reference_snapshot()
    assert not snapshot['connection']['configured']
    assert not snapshot['connection']['worker_alive']
    assert snapshot['discovery']['state']=='IDLE' and not snapshot['discovery']['candidates']
    view.close();panel.close();app.processEvents()
    assert not errors,errors
    print(json.dumps({'phase':mode,'restored':True,'auto_connect':False,'callback_errors':errors}))


if __name__=='__main__':
    links=json.loads(subprocess.check_output(['ip','-j','link','show'],text=True))
    assert [r['ifname'] for r in links]==['lo']
    if len(sys.argv)>1:
        phase(sys.argv[1],Path(sys.argv[2]))
    else:
        with tempfile.TemporaryDirectory(prefix='target-restart-smoke-') as temporary:
            for mode in ('write','read'):
                subprocess.run([sys.executable,'-B',__file__,mode,temporary],check=True)
