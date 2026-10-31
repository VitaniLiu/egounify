"""Parquet replay offline tests: transport and clock are faked, never real ROS."""
import importlib.util
import math
from pathlib import Path
import pytest


@pytest.fixture
def endpoint():
    path = Path(__file__).with_name('chassis_endpoint.py')
    assert path.exists(), 'chassis endpoint has not been implemented'
    spec = importlib.util.spec_from_file_location('chassis_test_endpoint', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture
def client():
    path = Path(__file__).with_name('replay_chassis.py')
    assert path.exists(), '20 Hz loader has not been implemented'
    spec = importlib.util.spec_from_file_location('chassis_test_client', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def fixture_data():
    feature = {'shape': [3], 'names': ['vx_m_s', 'vy_m_s', 'wz_rad_s'], 'dtype': 'float32'}
    info = {'fps': 20, 'total_frames': 3, 'features': {
        'action.chassis': feature, 'observation.chassis': dict(feature)}}
    ts = [1_000_000_000, 1_050_000_000, 1_100_000_000]
    timeline = {'timestamp_ns': ts, 'native_replay': {'chassis_frame': 'base_link',
        'chassis_units': ['m/s', 'm/s', 'rad/s']},
        'aligned_sources': {'actions/chassis': {'timestamp_ns': [t-10_000_000 for t in ts]}}}
    columns = {'timestamp': [0., .05, .1], 'frame_index': [0,1,2],
        'action.chassis': [[.1,0.,.2],[-.1,0.,-.2],[.2,0.,0.]],
        'observation.chassis': [[0.,0.,0.],[.05,.001,.1],[-.05,0.,-.1]]}
    return info, timeline, columns


def test_plan_uses_action_not_feedback_and_holds_last_slot(client):
    p = client.make_plan(*fixture_data())
    assert p['offset_ns'] == [0,50_000_000,100_000_000]
    assert p['velocity'] == [[.1,0.,.2],[-.1,0.,-.2],[.2,0.,0.]]
    assert p['end_ns'] == 150_000_000
    assert p['source_timestamp_ns'] == [1_000_000_000,1_050_000_000,1_100_000_000]


@pytest.mark.parametrize('fault', ['nan','lateral','overspeed','rate','gap','order','future','units','frame','index','missing_frame_metadata'])
def test_bad_input_rejected(client, fault):
    i,t,c = fixture_data()
    if fault=='nan': c['action.chassis'][0][0] = math.nan
    if fault=='lateral': c['action.chassis'][0][1] = .1
    if fault=='overspeed': c['action.chassis'][0][0] = .401
    if fault=='rate': i['fps'] = 50
    if fault=='gap': t['timestamp_ns'][1] += 1
    if fault=='order': i['features']['action.chassis']['names'] = ['wz_rad_s','vy_m_s','vx_m_s']
    if fault=='future': t['aligned_sources']['actions/chassis']['timestamp_ns'][0] += 20_000_000
    if fault=='units': t['native_replay']['chassis_units'][0] = 'mm/s'
    if fault=='frame': t['native_replay']['chassis_frame'] = 'odom'
    if fault=='index': c['frame_index'][1] = 8
    if fault=='missing_frame_metadata': del t['native_replay']
    with pytest.raises(ValueError): client.make_plan(i,t,c)


def test_prefix_and_float32_roundoff(client):
    i,t,c = fixture_data(); c['action.chassis'][0][0] = .4000000059604645
    p = client.make_plan(i,t,c,seconds=.1)
    assert len(p['velocity']) == 2 and p['end_ns'] == 100_000_000
    assert p['velocity'][0][0] == .4000000059604645
    with pytest.raises(ValueError): client.make_plan(i,t,c,seconds=.075)


class Clock:
    def __init__(self): self.t = 0.
    def now(self): return self.t
    def sleep(self, dt): self.t += dt


class Transport:
    def __init__(self, clock, fail_at=None): self.clock=clock; self.sent=[]; self.stopped=False; self.fail_at=fail_at
    def check(self):
        if self.fail_at is not None and self.clock.now() >= self.fail_at: raise RuntimeError('feedback lost')
    def tracking(self, values): pass
    def publish(self, values, stamp=None): self.sent.append((self.clock.now(), list(values), stamp))
    def stop(self): self.stopped=True; return {'stop_confirmed': True}


def test_scheduler_preserves_time_values_and_appends_separate_stop(client, endpoint):
    p=client.make_plan(*fixture_data()); clock=Clock(); tr=Transport(clock)
    r=endpoint.run_schedule(p,tr,lambda: None,clock.now,clock.sleep)
    assert r['status']=='completed' and tr.stopped
    assert [v[0] for v in tr.sent] == pytest.approx([0,.05,.1])
    assert tr.sent[0][1] == [.1,0,0,0,0,.2]
    assert clock.now() == pytest.approx(.15)


def test_feedback_failure_stops_without_remaining_frames(client, endpoint):
    p=client.make_plan(*fixture_data()); clock=Clock(); tr=Transport(clock,fail_at=.07)
    r=endpoint.run_schedule(p,tr,lambda: None,clock.now,clock.sleep)
    assert r['status']=='failed' and tr.stopped
    assert len(tr.sent)==2


def test_host_loss_and_interrupt_stop(client, endpoint):
    for exc in [RuntimeError('host heartbeat lost'), KeyboardInterrupt()]:
        clock=Clock(); tr=Transport(clock)
        def guard():
            if clock.now()>=.02: raise exc
        r=endpoint.run_schedule(client.make_plan(*fixture_data()),tr,guard,clock.now,clock.sleep)
        assert r['status']=='failed' and tr.stopped and len(tr.sent)==1


def test_no_catchup_burst(client, endpoint):
    clock=Clock(); tr=Transport(clock)
    def oversleep(dt): clock.t += .2
    r=endpoint.run_schedule(client.make_plan(*fixture_data()),tr,lambda: None,clock.now,oversleep)
    assert r['status']=='failed' and tr.stopped and len(tr.sent)==1


def test_near_slot_delay_cannot_compress_two_frames(client, endpoint):
    clock=Clock(); tr=Transport(clock)
    def sleep(dt):
        if clock.t==0.: clock.t=.099
        else: clock.sleep(dt)
    r=endpoint.run_schedule(client.make_plan(*fixture_data()),tr,lambda:None,clock.now,sleep)
    assert r['status']=='failed' and tr.stopped and len(tr.sent)==1


def test_robot_report_is_saved_before_network_delivery(endpoint, tmp_path):
    import json
    p=tmp_path/'report.json'
    endpoint.save_report(p,{'source':'fixture'}, {'status':'failed','stop_confirmed':True,'sample':math.nan})
    saved=json.loads(p.read_text())
    assert saved['result']['stop_confirmed'] is True and saved['result']['sample']=='NaN'
    assert saved['plan']['source']=='fixture'


def test_unconfirmed_stop_cannot_be_success(client, endpoint):
    clock=Clock(); tr=Transport(clock); tr.stop=lambda: {'stop_confirmed': False}
    r=endpoint.run_schedule(client.make_plan(*fixture_data()),tr,lambda: None,clock.now,clock.sleep)
    assert r['status']=='failed' and r['stop_confirmed'] is False


def test_endpoint_revalidates_before_publishing(endpoint):
    p={'schema':'chassis_parquet_20hz_v1','offset_ns':[0,50_000_000],
       'end_ns':100_000_000,'velocity':[[0,.1,0],[0,0,0]]}
    clock=Clock(); tr=Transport(clock)
    with pytest.raises(ValueError): endpoint.run_schedule(p,tr,lambda:None,clock.now,clock.sleep)
    assert not tr.sent


def test_loader_verifies_real_parquet_and_rejects_tampering(client, tmp_path):
    import json
    import pyarrow as pa
    import pyarrow.parquet as pq
    i,t,c = fixture_data()
    (tmp_path/'meta').mkdir(); (tmp_path/'data').mkdir()
    (tmp_path/'meta/info.json').write_text(json.dumps(i))
    (tmp_path/'meta/source_timestamps.json').write_text(json.dumps(t))
    pq.write_table(pa.table(c), tmp_path/'data/episode.parquet')
    entries={str(p.relative_to(tmp_path)):client.sha(p) for p in tmp_path.rglob('*') if p.is_file()}
    (tmp_path/'meta/completion.json').write_text(json.dumps(entries))
    assert client.load_plan(tmp_path)['velocity']==c['action.chassis']
    (tmp_path/'meta/info.json').write_text('{}')
    with pytest.raises(ValueError, match='Integrity mismatch'): client.load_plan(tmp_path)


def test_host_lease_deadline_and_eof(endpoint, monkeypatch):
    import io
    clock=Clock(); monkeypatch.setattr(endpoint.time,'monotonic',clock.now)
    lease=endpoint.HostLease(); lease.check()
    clock.t=.501
    with pytest.raises(RuntimeError): lease.check()
    lease.read(io.StringIO('PING\n'))
    assert lease.last==.501
    with pytest.raises(RuntimeError): lease.check()


@pytest.mark.parametrize('fault', ['none','lateral','stale'])
def test_stop_requires_three_axis_fresh_stable_feedback(endpoint, monkeypatch, fault):
    from types import SimpleNamespace as N
    clock=Clock(); monkeypatch.setattr(endpoint.time,'monotonic',clock.now)
    monkeypatch.setattr(endpoint.time,'sleep',clock.sleep)
    tr=endpoint.RobotTransport.__new__(endpoint.RobotTransport); tr.started=True
    speed=N(linear=N(x=0.,y=.02 if fault=='lateral' else 0.),angular=N(z=0.))
    c=N(state={'message':N(speed=speed),'time':0.},samples=[],published=[],diagnostics=[],tracking_guard=N(events=[]))
    def publish(v):
        assert v==[0.]*6
        c.published.append(list(v))
        if fault!='stale': c.state['time']=clock.now()
    c.publish=publish; tr.c=c
    result=tr.stop()
    assert result['stop_confirmed']==(fault=='none')
    assert clock.now()>=.5 and c.published


def test_nonfinite_diagnostics_preserve_valid_failure_report(endpoint):
    import json
    report={'status':'failed','diagnostics':[{'data':[1.,math.nan,math.inf,-math.inf]}]}
    safe=endpoint.json_safe(report)
    assert json.loads(json.dumps(safe,allow_nan=False))['diagnostics'][0]['data']==[1.,'NaN','Infinity','-Infinity']


def test_default_cli_never_opens_ssh(client, monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(client, '__file__', str(tmp_path/'replay_chassis.py'))
    monkeypatch.setattr(client.sys, 'argv', ['replay_chassis.py'])
    monkeypatch.setattr(client, 'load_plan', lambda *_:client.make_plan(*fixture_data()))
    def forbid(*args, **kwargs): raise AssertionError('Offline must not spawn SSH')
    monkeypatch.setattr(client.subprocess,'Popen',forbid)
    assert client.main()==0
    assert 'OFFLINE_PASS' in capsys.readouterr().out
    assert len(list((tmp_path/'runs').glob('*/plan.json')))==1


def test_transport_needs_chassis_only_without_upper_body_ros(client, endpoint):
    from types import SimpleNamespace as N
    calls=[]
    class Chassis:
        def __init__(self, bag, rows):
            self.thread=N(is_alive=lambda:True)
            self.state={'message':N(speed=N(linear=N(y=0.)))}
        def check(self): calls.append('check')
        def ready(self): calls.append('ready')
        def tracking(self, values): calls.append(('tracking',values))
    tr=endpoint.RobotTransport({'CoordinatedChassis':Chassis},client.make_plan(*fixture_data()))
    tr.ready()
    assert 'ready' in calls and ('tracking',[0.]*6) in calls
    tr.c.thread.is_alive=lambda:False
    with pytest.raises(RuntimeError,match='executor'): tr.check()
    tr.c.thread.is_alive=lambda:True
    def stale(): raise RuntimeError('Chassis feedback stale >200ms')
    tr.c.check=stale
    with pytest.raises(RuntimeError,match='Chassis feedback stale'): tr.check()


@pytest.mark.parametrize('started,confirmed,warning', [(False,None,False),(True,False,True),(True,True,False),(None,None,True)])
def test_stop_warning_distinguishes_preflight_from_lost_motion(client, started, confirmed, warning):
    report={'status':'failed','motion_started':started,'stop_confirmed':confirmed}
    result=client.finish_remote_result(report,1,True)
    assert ('stop_warning' in result)==warning
    assert result['status']=='failed'
