"""Authenticated-stdio execution agent; isolated_controller is the only enabled profile."""
import ctypes,hashlib,json,os,queue,signal,subprocess,sys,threading,time
_LIBC=ctypes.CDLL(None)

def die_with_parent():
    if _LIBC.prctl(1,signal.SIGTERM)!=0:os._exit(125)
    if os.getppid()==1:os._exit(125)
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from unified_replay.integration.api import PREFIX,ROLES,verify_session
from unified_replay.integration.ros_adapter import RosAdapter


def main():
    role=sys.argv[1]
    if role not in ROLES:raise ValueError('Unknown role')
    def interrupted(*_):raise RuntimeError('Agent interrupted')
    signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGINT,interrupted)
    session=json.loads(sys.stdin.readline());plan=verify_session(session);ns=session['namespace']
    root=Path(__file__).parent;run=root/'runs'/session['run_id']/role;run.mkdir(parents=True,exist_ok=False)
    children=[];files=[];adapter=None;result=None
    def emit(kind,**values):print(PREFIX+json.dumps(dict(kind=kind,role=role,**values),allow_nan=False),flush=True)
    def launch(cmd,name):
        f=(run/(name+'.log')).open('w');files.append(f)
        child=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,preexec_fn=die_with_parent);children.append(child);return child
    commands=queue.Queue();eof=threading.Event()
    def reader():
        try:
            for line in sys.stdin:commands.put(json.loads(line))
        finally:eof.set()
    threading.Thread(target=reader,daemon=True).start()
    start=None;committed=False;last_hb=time.monotonic_ns();last_status=0;offset=0;injected=False
    log=[];reason=None;index={c:0 for c in ROLES[role]};clock=lambda:time.monotonic_ns()+offset
    try:
        launch([sys.executable,str(root/'fixtures.py'),role,ns],'fixture')
        if role=='robot':
            import yaml
            baseline=yaml.safe_load((root/'native_qp_parameters.yaml').read_text())['/**']['ros__parameters']
            params=dict(baseline)
            params.update(ik_request_topic='input/ik',joint_cmd_output_prefix='output')
            for key,value in list(params.items()):
                if isinstance(value,str) and value.startswith('/info/'):params[key]=ns+value
            cfg=run/'qp.yaml';cfg.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
            launch(['/opt/kernelmind/apex/install/marvin_qp_controller/lib/marvin_qp_controller/qp_controller',
                    '--ros-args','-r','__ns:='+ns,'--params-file',str(cfg)],'native_qp')
        adapter=RosAdapter(role,ns)
        deadline=time.monotonic()+25
        while not adapter.ready():
            adapter.spin(.02)
            if any(p.poll() is not None for p in children):raise RuntimeError('Fixture/controller startup failed; inspect '+str(run))
            if time.monotonic()>deadline:raise TimeoutError('ROS receivers/feedback not ready')
        adapter.preflight();adapter.initialize_controller()
        metadata={}
        if role=='robot':
            from rcl_interfaces.srv import GetParameters
            from rclpy.parameter import parameter_value_to_python
            client=adapter.node.create_client(GetParameters,ns+'/qp_controller/get_parameters')
            names=[k for k,v in params.items() if not isinstance(v,dict)]
            if not client.wait_for_service(timeout_sec=3):raise RuntimeError('QP parameter service missing')
            future=client.call_async(GetParameters.Request(names=names));until=time.monotonic()+3
            while not future.done() and time.monotonic()<until:adapter.spin(.01)
            if not future.done():raise RuntimeError('QP readback timeout')
            observed={k:parameter_value_to_python(v) for k,v in zip(names,future.result().values)}
            if any(observed[k]!=params[k] for k in names):raise RuntimeError('QP parameter readback mismatch')
            from unified_replay.integration.fixtures import model_home
            home=model_home();ordered=sum(GROUPS_VALUES(),[])
            expected=[home[k] for k in ordered];actual=adapter.feedback['body'][1]
            if max(abs(a-b) for a,b in zip(actual,expected))>.02:raise RuntimeError('Ideal feedback failed to reach controller home')
            metadata={'parameter_readback_verified':True,'parameter_changes':{k:[baseline.get(k),v] for k,v in params.items() if baseline.get(k)!=v},
                      'bootstrap':'diagnostic controller home, not recorded initial pose',
                      'recorded_initial_max_error':max(abs(a-b) for a,b in zip(actual,plan['initial_feedback']['body'])),
                      'controller_home':expected,
                      'controller_binary_sha256':hashlib.sha256(Path('/opt/kernelmind/apex/install/marvin_qp_controller/lib/marvin_qp_controller/qp_controller').read_bytes()).hexdigest(),
                      'parameter_snapshot_sha256':hashlib.sha256((root/'native_qp_parameters.yaml').read_bytes()).hexdigest()}
        else:
            for c in ROLES[role]:
                if max(abs(a-b) for a,b in zip(adapter.feedback[c][1],plan['initial_feedback'][c]))>.1:raise RuntimeError('Initial hand pose mismatch')
            metadata={'driver':'installed SharpaDriverNode','actuators':'MockSharpaHand only'}
        emit('READY',session_sha256=session['session_sha256'],plan_sha256=plan['plan_sha256'],metadata=metadata,feedback={k:v[1] for k,v in adapter.feedback.items()})
        while reason is None:
            adapter.spin(.001);now=time.monotonic_ns()
            if any(p.poll() is not None for p in children):raise RuntimeError('Fixture/controller exited')
            if eof.is_set():raise RuntimeError('Coordinator transport EOF')
            while not commands.empty():
                m=commands.get_nowait();kind=m['kind']
                if kind=='PING':
                    if not (session['scenario']=='clock_probe_loss' and role=='hands' and start is not None and clock()-start>=300_000_000):
                        emit('PONG',seq=m['seq'],local_ns=clock())
                elif kind=='HEARTBEAT':last_hb=now
                elif kind=='ARM':
                    if start is not None or m['start_ns']-clock()<300_000_000:raise ValueError('Late or duplicate ARM')
                    adapter.arm();start=m['start_ns'];emit('ARMED',session_sha256=session['session_sha256'])
                elif kind=='COMMIT':
                    if start is None or clock()>=start:raise RuntimeError('Invalid COMMIT')
                    committed=True;last_hb=now;emit('COMMITTED')
                elif kind=='STOP':reason=m['reason'];break
                else:raise ValueError('Unknown coordinator message')
            if reason is not None:break
            if start is not None and not committed and clock()>=start:raise RuntimeError('Commit timeout')
            if not committed:continue
            if now-last_hb>200_000_000:raise RuntimeError('Heartbeat timeout')
            elapsed=clock()-start
            scenario=session['scenario']
            if elapsed>=300_000_000 and not injected:
                if scenario in ('left_sdk_reject','right_sdk_reject') and role=='hands':adapter.inject(scenario);injected=True
                if scenario in ('body_feedback_loss','chassis_feedback_loss') and role=='robot':adapter.inject(scenario);injected=True
                if scenario=='clock_jump' and role=='hands':offset=20_000_000;injected=True
                if scenario=='robot_process_exit' and role=='robot':os._exit(23)
            if elapsed>=0:
                adapter.check()
                # At most one event per channel per iteration. Reject overdue work.
                for c in ROLES[role]:
                    i=index[c];events=plan['channels'][c]
                    if i<len(events) and clock()>=start+events[i]['offset_ns']:
                        delay=clock()-(start+events[i]['offset_ns'])
                        if delay>50_000_000:raise RuntimeError(c+':Schedule deadline missed')
                        adapter.dispatch(c,events[i],i)
                        log.append({'channel':c,'event_index':i,'offset_ns':events[i]['offset_ns'],'source_hash':events[i]['payload_sha256'],
                                    'local_ns':clock(),'lateness_ns':delay})
                        index[c]+=1
                if elapsed>=plan['duration_ns'] and all(index[c]==len(plan['channels'][c]) for c in ROLES[role]):
                    adapter.spin(.03)
                    if role=='robot' and any(len(adapter.outputs.get(k,[]))<=adapter.initial_outputs.get(k,0) for k in ('A','B','body','head')):raise RuntimeError('Missing native outputs during replay')
                    if adapter.pending:raise RuntimeError('Pending SDK receipts at end')
                    reason='completed';break
            if now-last_status>20_000_000:
                emit('STATUS',sent=index);last_status=now
    except BaseException as exc:reason=str(exc)
    finally:
        stop_result=adapter.stop() if adapter else {c:{'confirmed':False,'error':'adapter not initialized'} for c in ROLES[role]}
        result={'reason':reason,'stop':stop_result,'commands':log,'receipts':adapter.receipts if adapter else [],
                'controller_outputs':adapter.outputs if adapter else {},'control_calls':adapter.control_calls if adapter else [],
                'session_sha256':session['session_sha256'],'hardware_commands_sent':0,'physical_stop_verified':False,
                'role':role,'stop_local_ns':clock(),'logs':str(run)}
        (run/'result.json').write_text(json.dumps(result,indent=2))
        if not (session['scenario']=='hands_stop_ack_loss' and role=='hands'):
            try:emit('STOPPED',result=result)
            except (BrokenPipeError,OSError):pass
        if adapter:adapter.close()
        for child in reversed(children):
            if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
        for child in reversed(children):
            try:child.wait(5)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        for f in files:f.close()


def GROUPS_VALUES():
    from unified_replay.integration.api import GROUPS
    return [GROUPS[k] for k in ('A','B','body','head')]

if __name__=='__main__':main()
