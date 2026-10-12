"""Time an explicitly selected Workstation X11 window's actual Ctrl+Q exit.
Usage: python3 visible_ctrl_q.py PID WINDOW_ID OUTPUT_JSON
Only sends the shortcut after validating title/PID/active-window identity.
"""
import json
from pathlib import Path
import sys
import time
from Xlib import X, XK, display, protocol
from Xlib.ext import xtest

pid=int(sys.argv[1]);wid=int(sys.argv[2],0);output=Path(sys.argv[3])
d=display.Display();root=d.screen().root;window=d.create_resource_object('window',wid)
owner=window.get_full_property(d.intern_atom('_NET_WM_PID'),X.AnyPropertyType)
assert owner is not None and int(owner.value[0])==pid
assert window.get_wm_name()=='StereoWIRIS Workstation'
children_file=Path(f'/proc/{pid}/task/{pid}/children')
children=list(map(int,children_file.read_text().split())) if children_file.exists() else []
root.send_event(protocol.event.ClientMessage(window=wid,client_type=d.intern_atom('_NET_ACTIVE_WINDOW'),data=(32,[2,X.CurrentTime,0,0,0])),event_mask=X.SubstructureRedirectMask|X.SubstructureNotifyMask)
d.flush();deadline=time.monotonic()+3
while True:
    active=root.get_full_property(d.intern_atom('_NET_ACTIVE_WINDOW'),X.AnyPropertyType)
    focus=d.get_input_focus().focus
    focus_id=getattr(focus,'id',focus)
    if active is not None and int(active.value[0])==wid and focus_id==wid:break
    if time.monotonic()>deadline:raise RuntimeError('Target window did not become active; no shortcut sent')
    time.sleep(.02)
time.sleep(.15)  # Deliver Qt activation/focus events before the shortcut.
assert d.get_input_focus().focus.id==wid, 'Focus changed; no shortcut sent'
control=d.keysym_to_keycode(XK.string_to_keysym('Control_L'));q=d.keysym_to_keycode(XK.string_to_keysym('q'))
started=time.monotonic()
xtest.fake_input(d,X.KeyPress,control);d.sync();time.sleep(.05)
try:
    assert d.get_input_focus().focus.id==wid, 'Focus changed before Q; aborting shortcut'
    xtest.fake_input(d,X.KeyPress,q);d.sync();time.sleep(.05)
    xtest.fake_input(d,X.KeyRelease,q)
finally:
    xtest.fake_input(d,X.KeyRelease,control);d.sync()
result={'method':'X11 XTEST Ctrl+Q with actual X input focus and Qt activation settling','pid':pid,'window':hex(wid),'children_before':children,'window_closed_s':None,'process_exited_s':None}

def alive(process):
    try:return Path(f'/proc/{process}/stat').read_text().split(') ',1)[1][0]!='Z'
    except FileNotFoundError:return False

reported=set()
while time.monotonic()-started<45:
    elapsed=time.monotonic()-started
    if result['window_closed_s'] is None:
        try:shown=window.get_attributes().map_state==X.IsViewable
        except Exception:shown=False
        if not shown:
            result['window_closed_s']=elapsed
            print('Window closed after',round(elapsed,3),'s',flush=True)
    if not alive(pid):
        result['process_exited_s']=elapsed
        break
    for milestone in (2,5,10,20,30):
        if elapsed>=milestone and milestone not in reported:
            print('Still waiting at',milestone,'s',flush=True);reported.add(milestone)
    time.sleep(.02)
result['remaining_children']=[p for p in children if alive(p)]
result['process_still_running']=alive(pid)
output.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result),flush=True)
d.close()
