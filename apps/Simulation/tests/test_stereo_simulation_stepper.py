"""Source-step service ACK is insufficient without observed reference dispatch."""
import sys
from pathlib import Path
from types import SimpleNamespace
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from campaigns.stereo_v1_1.simulation_stepper import SimulationStepper, check_step_barrier
from campaigns.stereo_v1_1.science_dispatch import ScienceDispatcher


class Request:
    pass


class Node:
    def __init__(self):
        self.calls=[]
        self.ok=True
    def service_info(self, service):
        return [object()]
    def request(self, service, req, request_type, response_type, timeout):
        self.calls.append((service, vars(req), timeout))
        return self.ok, SimpleNamespace(data=self.ok)


def test_persistent_service_only_pauses_steps_and_resumes_never_resets():
    node=Node()
    stepper=SimulationStepper('stereo_ground_drone',node=node,request_type=Request,response_type=object)
    stepper.pause()
    stepper.step(5)
    stepper.resume()
    assert [call[1] for call in node.calls]==[{'pause':True},{'pause':True,'multi_step':5},{'pause':False}]
    assert all(c[0]=='/world/stereo_ground_drone/control' and c[2]==200 for c in node.calls)
    assert not stepper.paused


def test_unacknowledged_pause_still_requires_resume_cleanup():
    node=Node();node.ok=False
    stepper=SimulationStepper('world',node=node,request_type=Request,response_type=object)
    with pytest.raises(RuntimeError,match='simulation step control:'):
        stepper.pause()
    assert stepper.paused
    node.ok=True
    stepper.resume()
    assert not stepper.paused


def test_step_limits_and_partial_endpoint_are_physics_grid_bounded():
    assert SimulationStepper.next_target(0,47_567_110_900)==(5,20_000_000)
    assert SimulationStepper.next_target(47_560_000_000,47_567_110_900)==(2,47_568_000_000)
    stepper=SimulationStepper('world',node=Node(),request_type=Request,response_type=object)
    with pytest.raises(ValueError):stepper.step(5)
    stepper.pause()
    for count in [0,6,1.5]:
        with pytest.raises(ValueError):stepper.step(count)


def test_clock_ack_without_dispatched_reference_cannot_advance_source():
    p={'observed_timestamp_ns':20_000_000,'last_reference_timestamp_ns':0,'finished':False}
    assert not check_step_barrier(p,20_000_000,100_000_000)
    p['last_reference_timestamp_ns']=20_000_000
    assert check_step_barrier(p,20_000_000,100_000_000)
    p['observed_timestamp_ns']=24_000_000
    with pytest.raises(RuntimeError,match='beyond'):
        check_step_barrier(p,20_000_000,100_000_000)


def test_observed_dispatch_barrier_preserves_current_clock_no_catchup():
    def snap(t):return SimpleNamespace(simulation_time_ns=t,simulation_epoch=0,xy_reset_counter=0,z_reset_counter=0)
    sends=[]
    d=ScienceDispatcher(snap(0),.047,50,lambda s,*args:sends.append(s.simulation_time_ns),lambda:None,0)
    d.on_clock(snap(0),0)
    for t in [20_000_000,40_000_000,48_000_000]:
        d.on_clock(snap(t),t)
        assert check_step_barrier(d.progress_snapshot(),t,47_000_000)
    assert sends==[0,20_000_000,40_000_000]
    assert d.progress_snapshot()['finished']


def test_ground_probe_observes_real_fake_source_progress_not_ack(monkeypatch):
    from campaigns.stereo_v1_1 import verify_simulation_steps as probe
    wall=[0.0]
    monkeypatch.setattr(probe.time,'monotonic',lambda:wall[0])
    monkeypatch.setattr(probe.time,'monotonic_ns',lambda:round(wall[0]*1e9))
    monkeypatch.setattr(probe.time,'sleep',lambda seconds:wall.__setitem__(0,wall[0]+seconds))
    class Physics:
        physics_dt_ns=4_000_000
        source_ns=1_000_000_000
        paused=False
        def warmup_continuous(self):self.source_ns+=self.physics_dt_ns
        def pause(self):self.paused=True
        def step(self,n):self.source_ns+=n*self.physics_dt_ns
        def resume(self):self.paused=False;self.source_ns+=self.physics_dt_ns
    physics=Physics()
    def observe():return SimpleNamespace(simulation_time_ns=physics.source_ns,simulation_epoch=2,
                                         xy_reset_counter=3,z_reset_counter=1)
    def read_clock():return dict(source_timestamp_ns=physics.source_ns,simulation_epoch=2,
                                 local_reset_counters=(3,1),fresh=True,receive_monotonic_ns=round(wall[0]*1e9))
    result=probe.run_probe(physics,observe,read_clock)
    assert result['status']=='PASS'
    assert [r['observed_delta_ns'] for r in result['records'] if r['action']=='step_observed']==[20_000_000,20_000_000,4_000_000]
    assert not physics.paused


def test_ground_probe_ack_without_observed_progress_times_out(monkeypatch):
    from campaigns.stereo_v1_1 import verify_simulation_steps as probe
    wall=[0.0]
    monkeypatch.setattr(probe.time,'monotonic',lambda:wall[0])
    monkeypatch.setattr(probe.time,'sleep',lambda seconds:wall.__setitem__(0,wall[0]+seconds))
    snapshot=SimpleNamespace(simulation_time_ns=0,simulation_epoch=0,xy_reset_counter=0,z_reset_counter=0)
    stepper=SimpleNamespace(physics_dt_ns=4_000_000,pause=lambda:None,step=lambda n:None,
        warmup_continuous=lambda:setattr(snapshot,'simulation_time_ns',4_000_000))
    def read_clock():return dict(source_timestamp_ns=snapshot.simulation_time_ns,simulation_epoch=0,
                                 local_reset_counters=(0,0),fresh=True,receive_monotonic_ns=round(wall[0]*1e9))
    with pytest.raises(RuntimeError,match='probe timeout: 5 physics steps'):
        probe.run_probe(stepper,lambda:SimpleNamespace(**vars(snapshot)),read_clock,timeout_s=.2)


def test_ground_probe_main_resumes_and_joins_on_failure(tmp_path,monkeypatch):
    from campaigns.stereo_v1_1 import verify_simulation_steps as probe
    calls=[]
    bridge=SimpleNamespace(last_error=None,diagnostic_shutdown_complete=True,close=lambda:calls.append('join'))
    stepper=SimpleNamespace(resume=lambda:calls.append('resume'))
    monkeypatch.setattr(probe,'RosObservationBridge',lambda cache:bridge)
    monkeypatch.setattr(probe,'SimulationStepper',lambda name:stepper)
    def fail(*args,**kwargs):raise RuntimeError('probe failed')
    monkeypatch.setattr(probe,'run_probe',fail)
    monkeypatch.setattr(sys,'argv',['probe','--report',str(tmp_path/'report.json')])
    assert probe.main()==1
    assert calls==['resume','join']
    import json
    assert json.loads((tmp_path/'report.json').read_text())['status']=='FAIL'


def test_missing_service_fails_before_any_control_request(monkeypatch):
    from campaigns.stereo_v1_1 import simulation_stepper as module
    node = Node()
    node.service_info = lambda service: []
    ticks = iter([0.0, 6.0])
    monkeypatch.setattr(module.time, 'monotonic', lambda: next(ticks))
    with pytest.raises(RuntimeError, match='service discovery timed out'):
        SimulationStepper('world', node=node, request_type=Request, response_type=object)
    assert node.calls == []


def test_real_service_warmup_precedes_pause_and_does_not_step():
    node=Node()
    stepper=SimulationStepper('world',node=node,request_type=Request,response_type=object)
    stepper.warmup_continuous()
    assert not stepper.paused
    stepper.pause()
    assert [c[1] for c in node.calls]==[{'pause':False},{'pause':True}]


def test_paused_clock_can_be_verified_independently_of_rejected_pair(monkeypatch):
    from campaigns.stereo_v1_1 import simulation_stepper as module
    wall=[0.0]
    monkeypatch.setattr(module.time,'monotonic',lambda:wall[0])
    monkeypatch.setattr(module.time,'sleep',lambda seconds:wall.__setitem__(0,wall[0]+seconds))
    evidence=dict(source_timestamp_ns=112100000000,simulation_epoch=0,
                  local_reset_counters=(4,2),fresh=True)
    def read_clock():return dict(evidence,receive_monotonic_ns=round(wall[0]*1e9))
    got=module.wait_clock_evidence(read_clock,0,(4,2),
        baseline_ns=112080000000,quiescent=True)
    assert got['source_timestamp_ns']==112100000000
    assert wall[0]>=.1
    # This proof says nothing about GT/local snapshot admission: the caller
    # must still reject the >100 ms pair; no historical snapshot substitutes.


def test_pause_recovery_clears_original_stability_and_second_failure_is_bounded():
    from campaigns.stereo_v1_1.simulation_stepper import PauseAdmissionRecovery
    calls=[]
    stepper=SimpleNamespace(resume=lambda:calls.append('resume'))
    samples=[1,2,3]
    recovery=PauseAdmissionRecovery()
    recovery.begin();recovery.recover(stepper,samples)
    assert samples==[] and calls==['resume']
    samples.extend([4,5])
    recovery.begin()
    with pytest.raises(RuntimeError,match='second pause admission'):
        recovery.recover(stepper,samples)
    assert samples==[] and calls==['resume','resume']


@pytest.mark.parametrize('change', ['epoch','reset','backwards'])
def test_clock_admission_generation_failures_are_fatal(change):
    from campaigns.stereo_v1_1.simulation_stepper import wait_clock_evidence
    evidence=dict(source_timestamp_ns=100,simulation_epoch=0,
                  local_reset_counters=(4,2),fresh=True)
    if change=='epoch':evidence['simulation_epoch']=1
    if change=='reset':evidence['local_reset_counters']=(5,2)
    if change=='backwards':evidence['source_timestamp_ns']=99
    with pytest.raises(RuntimeError,match='simulation step control:'):
        wait_clock_evidence(lambda:evidence,0,(4,2),baseline_ns=100,quiescent=True)


def test_warmup_ack_with_fresh_constant_clock_cannot_pass_as_quiescence(monkeypatch):
    from campaigns.stereo_v1_1 import simulation_stepper as module
    wall=[0.0]
    monkeypatch.setattr(module.time,'monotonic',lambda:wall[0])
    monkeypatch.setattr(module.time,'sleep',lambda seconds:wall.__setitem__(0,wall[0]+seconds))
    node=Node()
    stepper=module.SimulationStepper('world',node=node,request_type=Request,response_type=object)
    stepper.warmup_continuous()
    evidence=dict(source_timestamp_ns=100,simulation_epoch=0,local_reset_counters=(4,2),fresh=True)
    with pytest.raises(RuntimeError,match='continuous clock did not advance after warmup'):
        module.wait_clock_evidence(lambda:evidence,0,(4,2),baseline_ns=100,
                                   quiescent=False,timeout_s=.2)
    assert node.calls[-1][1]=={'pause':False}


def test_single_cached_fresh_clock_receipt_cannot_prove_pause(monkeypatch):
    from campaigns.stereo_v1_1 import simulation_stepper as module
    wall=[0.0]
    monkeypatch.setattr(module.time,'monotonic',lambda:wall[0])
    monkeypatch.setattr(module.time,'sleep',lambda seconds:wall.__setitem__(0,wall[0]+seconds))
    evidence=dict(source_timestamp_ns=100,simulation_epoch=0,
                  local_reset_counters=(4,2),fresh=True,receive_monotonic_ns=0)
    with pytest.raises(RuntimeError,match='received clock did not become quiescent'):
        module.wait_clock_evidence(lambda:evidence,0,(4,2),baseline_ns=100,
                                   quiescent=True,timeout_s=.2)
