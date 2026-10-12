import sys,time,json
from pathlib import Path
sys.path[:0]=[str(Path.cwd()/'apps'),str(Path.cwd()/'common/python')]
from PyQt5 import QtCore,QtWidgets
import pytest
from Workstation.tests.test_rtcm_pipeline_workers import _Subscriber
from Workstation.gnss_runtime_controller import GnssRuntimeController
_APP=None

def test_immediate_retirement_and_deleted_wrapper_reproduce_test_cleanup_gap():
 global _APP
 _APP=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
 panel=QtCore.QObject();panel.mavlink_pixhawk=_Subscriber();panel.pixhawk_rtcm_thread=None;panel.pixhawk_rtcm_worker=None;panel.rtcm_pipeline_worker=None
 for name in ('_refresh_gnss_base_rtcm_stats','_maybe_activate_gnss_fixed_base','_refresh_gnss_operation_readiness'):setattr(panel,name,lambda:None)
 controller=GnssRuntimeController(panel);controller._ensure_rtcm_pipeline=lambda:None;controller._wire_rtcm_pipeline_injection_worker=lambda _worker:None;panel._clear_pixhawk_rtcm_worker=controller._clear_pixhawk_rtcm_worker
 try:
  first=controller._ensure_pixhawk_rtcm_worker();first_thread=panel.pixhawk_rtcm_thread;real_quit=first_thread.quit
  def finish_before_retire_returns():
   real_quit();assert first_thread.wait(1000)
  first_thread.quit=finish_before_retire_returns
  panel.mavlink_pixhawk=_Subscriber();start=time.monotonic();second=controller._ensure_pixhawk_rtcm_worker();elapsed=time.monotonic()-start;second_thread=panel.pixhawk_rtcm_thread
  assert second is not None and second is not first and second_thread.isRunning()
  with pytest.raises(AssertionError):
   assert second is None  # Original timing-dependent assertion.
  # Recreate the test's early failure before the second thread is appended.
  original_cleanup_list=[first_thread]
  for thread in original_cleanup_list:
   if thread.isRunning():thread.quit();thread.wait(1000)
  _APP.processEvents()
  assert second_thread.isRunning()
  # A separately valid event-loop ordering deletes the retired QObject wrapper.
  QtCore.QCoreApplication.sendPostedEvents(None,QtCore.QEvent.DeferredDelete)
  with pytest.raises(RuntimeError,match='deleted'):
   first_thread.isRunning()
  assert second_thread.isRunning()
  print(json.dumps({'legacy_none_assertion_failed':True,'replacement_returned_synchronously':True,'elapsed_s':elapsed,'original_cleanup_misses_running_replacement':True,'retired_wrapper_deleted':True}))
 finally:
  panel._pixhawk_rtcm_restart_pending=None;panel.mavlink_pixhawk=None
  for thread in tuple(panel.findChildren(QtCore.QThread)):
   try:thread.quit();assert thread.wait(1000)
   except RuntimeError:pass
  _APP.processEvents();panel.deleteLater();QtCore.QCoreApplication.sendPostedEvents(None,QtCore.QEvent.DeferredDelete);_APP.processEvents()
