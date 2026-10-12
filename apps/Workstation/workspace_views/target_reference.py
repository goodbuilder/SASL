"""Target reference operator controls; all I/O lives behind backend commands."""
from __future__ import annotations
import time
from PyQt5 import QtCore, QtGui, QtWidgets


class TargetReferenceView(QtWidgets.QGroupBox):
    def __init__(self, backend, parent=None):
        super().__init__('Target reference', parent)
        self.backend = backend
        self._selected = None
        self._updating = False
        self._rows = []
        self._generation = 0
        self._signature = None
        self._last_refresh = float('-inf')
        self._busy = False
        layout = QtWidgets.QGridLayout(self)
        layout.setColumnStretch(1,1)
        self.platform = QtWidgets.QComboBox()
        self.platform.addItem('Drone','drone');self.platform.addItem('UAM','uam')
        self.platform.setCurrentIndex(1)
        self.transport = QtWidgets.QComboBox()
        for label,value in (('Network UDP','network'),('USB / Radio','usb'),('Manual','manual')):
            self.transport.addItem(label,value)
        layout.addWidget(QtWidgets.QLabel('Target platform'),0,0)
        layout.addWidget(self.platform,0,1)
        layout.addWidget(QtWidgets.QLabel('Receive-only'),0,2)
        layout.addWidget(self.transport,0,3)
        self.endpoint = QtWidgets.QLineEdit()
        self.endpoint.setObjectName('targetDroneEndpoint')
        self.endpoint.setPlaceholderText('udpin:0.0.0.0:14570 or /dev/serial/by-id/...')
        self.baud = QtWidgets.QSpinBox();self.baud.setRange(1200,4000000);self.baud.setValue(57600)
        self.system = QtWidgets.QSpinBox();self.component = QtWidgets.QSpinBox()
        for spin in (self.system,self.component):spin.setRange(0,255);spin.setSpecialValueText('Any')
        ids=QtWidgets.QHBoxLayout()
        for label,widget in (('SYS',self.system),('COMP',self.component),('Baud',self.baud)):
            ids.addWidget(QtWidgets.QLabel(label));ids.addWidget(widget)
        layout.addWidget(QtWidgets.QLabel('Target MAVLink'),1,0)
        layout.addWidget(self.endpoint,1,1)
        layout.addLayout(ids,1,2,1,2)
        self.ports = QtWidgets.QLineEdit('14570,14550,14551,14552,14553,14554,14555,14556,14557,14558,14559')
        self.ports.setToolTip('PC UDP listen ports, comma separated (maximum 16). Target must already transmit here.')
        layout.addWidget(QtWidgets.QLabel('Listen ports'),2,0)
        layout.addWidget(self.ports,2,1,1,3)
        scan=QtWidgets.QHBoxLayout()
        self.find_network=QtWidgets.QPushButton('Find Network');self.find_usb=QtWidgets.QPushButton('Find USB')
        self.cancel=QtWidgets.QPushButton('Cancel')
        self.discovery_status=QtWidgets.QLabel('IDLE');self.discovery_status.setObjectName('statusStrip')
        self.discovery_status.setMinimumWidth(150)
        for widget in (self.find_network,self.find_usb,self.cancel,self.discovery_status):scan.addWidget(widget)
        scan.addStretch(1);layout.addLayout(scan,3,0,1,4)
        self.model=QtGui.QStandardItemModel(0,6,self)
        self.model.setHorizontalHeaderLabels(['State','Source / Device','Port / Baud','SYS','COMP','Heartbeat age'])
        self.table=QtWidgets.QTableView()
        self.table.setModel(self.model);self.table.setFixedHeight(112)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.verticalHeader().hide()
        self.table.horizontalHeader().setStretchLastSection(True)
        for index,width in enumerate((135,245,115,55,65,100)):self.table.setColumnWidth(index,width)
        layout.addWidget(self.table,4,0,1,4)
        actions=QtWidgets.QHBoxLayout()
        self.use=QtWidgets.QPushButton('Use selected');self.connect_button=QtWidgets.QPushButton('Connect target')
        self.stop_button=QtWidgets.QPushButton('Stop target');self.save_button=QtWidgets.QPushButton('Save')
        for widget in (self.use,self.connect_button,self.stop_button,self.save_button):actions.addWidget(widget)
        actions.addStretch(1);layout.addLayout(actions,5,0,1,4)
        self.reference_label=QtWidgets.QLabel()
        self.reference_label.setObjectName('compactContract')
        self.state_label=QtWidgets.QLabel('Target Drone: NOT CONFIGURED')
        self.state_label.setObjectName('compactContract')
        self.samples_label=QtWidgets.QLabel('HEARTBEAT -- | ATTITUDE --\nGPS1 -- | GPS2 --')
        self.samples_label.setMinimumHeight(self.samples_label.fontMetrics().lineSpacing()*2)
        self.message=QtWidgets.QLabel('Ready');self.message.setObjectName('statusStrip')
        for row,widget in enumerate((self.reference_label,self.state_label,self.samples_label,self.message),6):
            widget.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            widget.setMinimumWidth(0)
            widget.setSizePolicy(QtWidgets.QSizePolicy.Ignored,QtWidgets.QSizePolicy.Fixed)
            layout.addWidget(widget,row,0,1,4)
        self.platform.currentIndexChanged.connect(self._edited)
        self.transport.currentIndexChanged.connect(self._edited)
        self.endpoint.textEdited.connect(self._edited)
        self.ports.textEdited.connect(self._edited)
        self.system.valueChanged.connect(self._edited);self.component.valueChanged.connect(self._edited)
        self.baud.valueChanged.connect(self._edited)
        self.find_network.clicked.connect(lambda:self._discover('network'))
        self.find_usb.clicked.connect(lambda:self._discover('usb'))
        self.cancel.clicked.connect(lambda:self._command('target_reference_cancel'))
        self.use.clicked.connect(self._use)
        self.connect_button.clicked.connect(self.connect_target)
        self.stop_button.clicked.connect(self.stop_target)
        self.save_button.clicked.connect(self._save)
        self.table.selectionModel().selectionChanged.connect(lambda *_:self._controls())
        reader=getattr(backend,'target_reference_snapshot',None)
        if callable(reader):
            try:self._fill(dict(reader().get('discovery',{})).get('preferences',{}))
            except Exception as exc:self._message(str(exc))
        self.refresh_mode();self.refresh(force=True)

    def _message(self,text):
        self.message.setText(str(text).replace('\n',' ')[:140])
        self.message.setToolTip(str(text))

    def _edited(self,*_):
        if not self._updating:self._selected=None
        self.refresh_mode()

    def payload(self):
        data={'mode':self.platform.currentData(),'transport':self.transport.currentData(),
              'endpoint':self.endpoint.text().strip(),'baudrate':self.baud.value(),
              'expected_system_id':self.system.value() or None,'expected_component_id':self.component.value() or None,
              'ports':self.ports.text()}
        if self._selected is not None:
            data.update(candidate_id=self._selected['candidate_id'],discovery_generation=self._generation)
        return data

    def _fill(self,data):
        self._updating=True
        try:
            for name,widget in (('mode',self.platform),('transport',self.transport)):
                if name in data:widget.setCurrentIndex(max(0,widget.findData(data[name])))
            if 'endpoint' in data:self.endpoint.setText(str(data['endpoint']))
            if 'baudrate' in data:self.baud.setValue(int(data['baudrate']))
            for name,widget in (('expected_system_id',self.system),('expected_component_id',self.component)):
                if name in data:widget.setValue(int(data[name] or 0))
            if 'ports' in data:
                self.ports.setText(','.join(map(str,data['ports'])))
                self.ports.setCursorPosition(0)
        finally:self._updating=False

    def _command(self,name,*args):
        method=getattr(self.backend,name,None)
        if not callable(method):self._message('Target command unavailable');return None
        try:
            result=method(*args)
            self._message('Request accepted')
            self.refresh(force=True)
            return result
        except Exception as exc:self._message(str(exc));return None

    def _discover(self,kind):
        self._selected=None
        self._command('target_reference_discover',kind,self.payload())

    def _use(self):
        index=self.table.currentIndex().row()
        if not 0<=index<len(self._rows):return
        result=self._command('target_reference_use_candidate',self._rows[index]['candidate_id'],self._generation)
        if isinstance(result,dict):
            self._fill({**result,'mode':'drone','transport':'usb' if result['transport']=='usb' else 'network'})
            self._selected=result
            self._message('Candidate copied to draft; press Connect target to verify again')

    def connect_target(self):
        method='target_reference_connect' if callable(getattr(self.backend,'target_reference_connect',None)) else 'target_drone_connect'
        result=self._command(method,self.payload())
        if isinstance(result,dict):self.render_connection(result)

    def stop_target(self):
        if callable(getattr(self.backend,'target_reference_stop',None)):
            self._command('target_reference_stop')
        else:
            method=getattr(self.backend,'target_drone_stop',None)
            try:
                if callable(method):self.render_connection(method(timeout_s=3.0))
            except Exception as exc:self._message(str(exc))

    def _save(self):
        if self._command('target_reference_save',self.payload()) is not None:self._message('Connection preferences saved; reconnect explicitly')

    def refresh_mode(self):
        uam=self.platform.currentData()=='uam'
        self.reference_label.setText('Reference acquisition: deferred external UAM RTK (expected; not a live error)' if uam else
            'Reference acquisition: live target Drone Pixhawk / RTK when available · receive-only')
        self.reference_label.setToolTip(self.reference_label.text())
        self.connect_button.setText('Set UAM deferred' if uam else 'Connect target')
        self._controls()

    def _controls(self):
        live=getattr(self,'_active',False);busy=self._busy
        drone=self.platform.currentData()=='drone'
        self.platform.setEnabled(not live and not busy)
        for widget in (self.transport,self.endpoint,self.system,self.component,self.baud,self.ports):
            widget.setEnabled(drone and not live and not busy)
        self.find_network.setEnabled(drone and not live and not busy)
        self.find_usb.setEnabled(drone and not live and not busy)
        self.cancel.setEnabled(busy and getattr(self,'_operation','')!='STOPPING')
        self.use.setEnabled(drone and not live and not busy and self.table.currentIndex().isValid())
        self.connect_button.setEnabled(not live and not busy)
        self.stop_button.setEnabled(live and not busy)
        self.save_button.setEnabled(not live and not busy)

    def render_connection(self,data):
        self._active=bool(data.get('worker_alive') or data.get('phase') in ('STARTING','RUNNING'))
        text=f"Target Drone: {data.get('phase','UNKNOWN')} · {data.get('stream_state') or '--'} · accepted={data.get('accepted_message_count',0)} · recv-only"
        detail=f"{text}\nCurrent endpoint: {data.get('endpoint','')}\nVerified SYS/COMP: {data.get('verified_source_system')}/{data.get('verified_source_component')}\n{data.get('last_error') or ''}"
        self.state_label.setText(text[:150]);self.state_label.setToolTip(detail)
        samples=data.get('reception',{}).get('samples',{})
        values=[]
        for key,label in (('HEARTBEAT','HB'),('GPS_RAW_INT','GPS1'),('GPS2_RAW','GPS2'),('ATTITUDE','ATT')):
            row=samples.get(key,{})
            age=row.get('age_s');age_text='--' if age is None else '>999s' if age>999 else f'{age:.1f}s'
            suffix=f" {row.get('rtk','UNKNOWN')}" if label.startswith('GPS') and row.get('fix_type') is not None else ''
            values.append(f"{label} {row.get('state','UNKNOWN')} {age_text}{suffix}")
        self.samples_label.setText(' | '.join((values[0],values[3]))+'\n'+' | '.join(values[1:3]))
        self.samples_label.setToolTip('\n'.join(values))
        if data.get('last_error'):self._message(data['last_error'])
        self._controls()

    def refresh(self,*,force=False):
        now=time.monotonic()
        if not force and now-self._last_refresh<.2:return
        self._last_refresh=now
        reader=getattr(self.backend,'target_reference_snapshot',None)
        if not callable(reader):return
        try:
            data=reader();scan=data.get('discovery',{})
            self._busy=bool(scan.get('busy'));self._operation=str(scan.get('operation',''))
            self._generation=scan.get('generation',0)
            self.discovery_status.setText(f"{self._operation or scan.get('state','IDLE')} · {len(scan.get('candidates',[]))} candidates")
            self.discovery_status.setToolTip('\n'.join(scan.get('errors',[]))+f"\nOverflow: {scan.get('overflow',0)}; packets: {scan.get('packet_overflow',0)}")
            if scan.get('operation_error'):self._message(scan['operation_error'])
            rows=scan.get('candidates',[])
            signature=(self._generation,tuple((r['candidate_id'],r['heartbeat_count']) for r in rows))
            if signature!=self._signature:
                self._signature=signature;self._rows=rows;self.model.setRowCount(0)
                if scan.get('errors'):self._message('; '.join(scan['errors']))
                self.table.clearSelection();self.table.setCurrentIndex(QtCore.QModelIndex())
                for row in rows:
                    peer=row.get('peer')
                    source=f'{peer[0]}:{peer[1]}' if peer else row['endpoint']
                    values=[row['identity_state'],source,row['endpoint'].rsplit(':',1)[-1] if peer else row['baudrate'],row['expected_system_id'],row['expected_component_id'],'--']
                    items=[QtGui.QStandardItem(str(v)) for v in values]
                    for item in items:item.setToolTip(f"{row['endpoint']}\nSource: {source}")
                    self.model.appendRow(items)
            for index,row in enumerate(rows):
                age=max(0,now-row['observed_monotonic_s'])
                self.model.item(index,5).setText('--' if not row['heartbeat_count'] else 'EXPIRED' if age>30 else f'{age:.1f}s')
            self.render_connection(data.get('connection',{}))
        except Exception as exc:self._message(f'Target status unavailable: {exc}')
