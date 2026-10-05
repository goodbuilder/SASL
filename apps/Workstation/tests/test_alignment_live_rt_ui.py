from dataclasses import replace
from types import SimpleNamespace

from PyQt5 import QtCore

from Workstation.workspaces import AlignmentWorkspace
from Workstation.status_model import StatusModel
from Workstation.alignment_rt_controller import alignment_live_rt_snapshot
from Workstation.tests.test_alignment_live_rt import panel


def test_live_updates_without_registration_and_preserves_candidate_selection(monkeypatch):
    import Workstation.workspaces as module
    monkeypatch.setattr(module.time, 'monotonic', lambda:100.)
    p = panel()
    view = AlignmentWorkspace(SimpleNamespace(panel=p), StatusModel(), lambda:None)
    pane = view.optical_pose_pane
    commands = []
    view.optical_pose_command_requested.connect(lambda *args:commands.append(args))
    view.refresh({})
    assert pane.live_rt_table.rowCount() == 2
    assert pane.live_rt_table.item(0,0).text() == 'Attitude + GNSS'
    assert 'R -2.000000' in pane.live_rt_table.item(0,2).text()
    assert pane.live_rt_table.item(1,3).text() == 'LIVE'
    assert not pane.live_rt_table.wordWrap()
    assert pane.live_rt_table.textElideMode() == QtCore.Qt.ElideNone
    assert pane.candidate_table.rowCount() == 0
    pane.set_candidates([{'candidate_id':'fixed-1', 'method':'test'}, {'candidate_id':'fixed-2','method':'test'}])
    pane.candidate_table.selectRow(1)
    pane.candidate_table.setCurrentCell(1,0)
    pane.combine_candidate_list.item(0).setCheckState(QtCore.Qt.Checked)
    selected = pane.selected_candidate_id()
    p.latest_gps['right'] = replace(p.latest_gps['right'], longitude_deg=p.latest_gps['right'].longitude_deg*2)
    for _ in range(3):
        view.refresh({})
    assert 'R -4.000000' in pane.live_rt_table.item(0,2).text()
    assert pane.selected_candidate_id() == selected == 'fixed-2'
    assert pane.combine_candidate_ids() == ('fixed-1',)
    assert commands == []
    assert 'References' not in [view.operation_tabs.tabText(i) for i in range(view.operation_tabs.count())]
    view.close()


def test_query_failure_retains_last_values_with_error_then_recovers(monkeypatch):
    import Workstation.workspaces as module
    monkeypatch.setattr(module.time, 'monotonic', lambda:100.)
    p = panel()
    backend = SimpleNamespace(panel=p, alignment_live_rt_snapshot=lambda:alignment_live_rt_snapshot(p, now_s=100))
    view = AlignmentWorkspace(backend,StatusModel(),lambda:None)
    view.refresh({})
    pane = view.optical_pose_pane
    old = pane.live_rt_table.item(0,1).text()
    backend.alignment_live_rt_snapshot=lambda:{'rows':[{'method_id':'pixhawk_rtk_live','rotation_right_from_left_rdf':[[float('nan')]]}]}
    monkeypatch.setattr(module.time, 'monotonic', lambda:110.)
    view.refresh({})
    assert pane.live_rt_table.item(0,1).text() == old
    assert pane.live_rt_table.item(0,3).text() == 'ERROR'
    assert pane.live_rt_table.item(0,4).text() == '10.20'
    backend.alignment_live_rt_snapshot=lambda:alignment_live_rt_snapshot(p, now_s=110)
    view.refresh({})
    assert pane.live_rt_table.item(0,3).text() == 'STALE'
    p.latest_gps.clear()
    view.refresh({})
    assert pane.live_rt_table.item(0,3).text() == 'WAITING'
    assert pane.live_rt_table.item(0,1).text() == '--'
    view.close()


def test_baseline_setup_keeps_live_rows_and_return_keeps_readout(monkeypatch):
    import Workstation.workspaces as module
    monkeypatch.setattr(module.time, 'monotonic', lambda:100.)
    p = panel()
    view = AlignmentWorkspace(SimpleNamespace(panel=p),StatusModel(),lambda:None)
    view.show()
    view.open_baseline_yaw90()
    view.refresh({})
    assert view.operation_tabs.currentWidget() is view.baseline_yaw90_pane
    pane = view.optical_pose_pane
    assert not pane.live_rt_group.isHidden()
    assert pane.layout().indexOf(pane.live_rt_group) > pane.layout().indexOf(pane.method_review_splitter)
    mountain_index = next(
        index for index in range(view.operation_tabs.count())
        if view.operation_tabs.tabText(index) == 'Mountain + GNSS'
    )
    view.operation_tabs.setCurrentIndex(mountain_index)
    view.refresh({})
    assert view.operation_tabs.currentWidget() is not view.baseline_yaw90_pane
    assert pane.layout().indexOf(pane.live_rt_group) < pane.layout().indexOf(pane.method_review_splitter)
    assert pane.live_rt_table.item(0,3).text() == 'LIVE'
    view.close()


def test_main_window_status_poll_updates_only_visible_alignment(workstation_qapplication):
    from Workstation.tests.test_workstation_calibration import _FakeBackend
    from Workstation.main_window import WorkstationMainWindow
    backend = _FakeBackend()
    p = panel()
    calls = []
    def read():
        calls.append(True)
        return alignment_live_rt_snapshot(p, now_s=100)
    backend.alignment_live_rt_snapshot = read
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    window.show()
    window.navigation.select_context('processing.alignment')
    workstation_qapplication.processEvents()
    before = len(calls)
    window.status_adapter.refresh()
    assert len(calls) > before
    assert window.alignment.optical_pose_pane.live_rt_table.item(0,3).text() == 'LIVE'
    window.navigation.select_context('processing.measurement')
    workstation_qapplication.processEvents()
    before = len(calls)
    window.status_adapter.refresh()
    assert len(calls) == before
    window.close()


def test_standalone_facade_delegates_without_registration():
    from Workstation.standalone_backend import StandalonePanelBackend
    from copy import deepcopy
    p = panel()
    before = deepcopy(vars(p))
    backend = StandalonePanelBackend(p)
    data = backend.alignment_live_rt_snapshot()
    assert vars(p) == before
    assert len(data['rows']) == 2
    assert data['rows'][0]['rotation_right_from_left_rdf'] is not None
    backend.deleteLater()
