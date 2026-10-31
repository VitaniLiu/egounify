"""Two-host SSH/stdio coordination. Endpoints are constrained to isolated ROS197."""
import hashlib,json,os,queue,shlex,subprocess,sys,threading,time,uuid
from pathlib import Path
from .api import PREFIX,ROLES,digest

SCENARIOS=('normal','left_sdk_reject','right_sdk_reject','body_feedback_loss','chassis_feedback_loss',
           'hands_heartbeat_loss','coordinator_loss','clock_jump','clock_probe_loss','robot_process_exit','hands_stop_ack_loss')
ROBOT_ROOT='/home/user/unified_replay_integration'
HAND_ROOT='/home/descfly/apex_n_sharpa/integrations/manus_sharpa'


def run(plan,scenario='normal',log_dir=None):
    if scenario not in SCENARIOS:raise ValueError('Unsupported controller integration scenario')
    if not plan['diagnostic']:raise ValueError('Controller lab requires diagnostic input')
    run_id=uuid.uuid4().hex[:12]
    session=dict(schema='unified_controller_session_v1',run_id=run_id,namespace='/unified_replay_lab_'+run_id,
                 profile='isolated_controller',physical_execution=False,scenario=scenario,plan=plan,
                 initialization_policy='diagnostic native model home; recorded start differences must be reported')
    session['coordinator_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    session['agent_hashes']={n:hashlib.sha256((Path(__file__).parent/n).read_bytes()).hexdigest() for n in ('api.py','agent.py','fixtures.py','ros_adapter.py','native_qp_parameters.yaml')}
    session['session_sha256']=digest(session)
    out=Path(log_dir);out.mkdir(parents=True,exist_ok=False);(out/'session.json').write_text(json.dumps(session))
    root=Path(__file__).parent;procs={};events=queue.Queue();logs=[];readers=[];ready={};stops={};sync={};probes={};seq=0;start=None;trigger=None;trigger_ns=None
    def emit(role,kind,**args):
        try:procs[role].stdin.write(json.dumps(dict(kind=kind,**args))+'\n');procs[role].stdin.flush();return True
        except (BrokenPipeError,OSError,ValueError):return False
    def reader(role,process,log):
        for line in process.stdout:
            log.write(line);log.flush()
            if line.startswith(PREFIX):
                try:events.put((time.monotonic_ns(),role,json.loads(line[len(PREFIX):])))
                except ValueError:events.put((time.monotonic_ns(),role,{'kind':'BAD_WIRE'}))
        events.put((time.monotonic_ns(),role,{'kind':'EOF'}))
    def receive(timeout=.002):
        try:t,role,m=events.get(timeout=timeout)
        except queue.Empty:return None
        if m['kind']=='READY':ready[role]=m
        if m['kind']=='STOPPED':stops[role]=m['result']
        if m['kind']=='PONG':
            t0=probes.pop((role,m['seq']),None)
            if t0 is not None:m.update(offset_ns=m['local_ns']-(t0+t)//2,uncertainty_ns=(t-t0)//2)
        return t,role,m
    def fail(reason):
        nonlocal trigger,trigger_ns
        if trigger is None:trigger=reason;trigger_ns=time.monotonic_ns()
        for role in procs:emit(role,'STOP',reason='coordinated:'+reason)
    try:
        env=dict(os.environ,ROS_DOMAIN_ID='197',ROS_LOCALHOST_ONLY='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                 RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
        env.pop('CYCLONEDDS_URI',None);env.pop('PYTHONPATH',None)
        hand=[HAND_ROOT+'/pixi','run','--manifest-path',HAND_ROOT+'/portable_deps/litchi_hardware/pixi.toml','-e','ros2',
              'python','-u',str(root/'agent.py'),'hands']
        command=('source /etc/apex/apex_ros_env.sh >/dev/null 2>&1; '
                 'export ROS_DOMAIN_ID=197 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4; '
                 'export ROS_LOG_DIR=/tmp/unified-replay-ros197 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1; '
                 'unset FASTRTPS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI; '
                 'exec python3 -u '+shlex.quote(ROBOT_ROOT+'/src/unified_replay/integration/agent.py')+' robot')
        robot=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','user@6.6.7.100','bash -c '+shlex.quote(command)]
        for role,cmd in [('robot',robot),('hands',hand)]:
            err=(out/(role+'.stderr.log')).open('w');log=(out/(role+'.stdout.log')).open('w');logs.extend([err,log])
            p=subprocess.Popen(cmd,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=err,text=True,bufsize=1,env=env)
            procs[role]=p;thread=threading.Thread(target=reader,args=(role,p,log),daemon=True);thread.start();readers.append(thread)
            p.stdin.write(json.dumps(session)+'\n');p.stdin.flush()
        until=time.monotonic()+40
        while len(ready)<2 and time.monotonic()<until:
            value=receive(.02)
            if value and value[2]['kind'] in ('STOPPED','EOF','BAD_WIRE'):raise RuntimeError('startup:'+str(value[1:]))
        if len(ready)!=2:raise RuntimeError('ready_timeout')
        if any(r['session_sha256']!=session['session_sha256'] for r in ready.values()):raise RuntimeError('Ready hash mismatch')
        samples={r:[] for r in ROLES}
        for _ in range(7):
            waiting=set(ROLES)
            for role in ROLES:
                seq+=1;probes[role,seq]=time.monotonic_ns();emit(role,'PING',seq=seq)
            until=time.monotonic()+.5
            while waiting and time.monotonic()<until:
                value=receive()
                if value and value[2]['kind']=='PONG' and 'offset_ns' in value[2]:samples[value[1]].append(value[2]);waiting.discard(value[1])
            if waiting:raise RuntimeError('Clock probe timeout')
        for role in ROLES:
            best=min(samples[role],key=lambda x:x['uncertainty_ns'])
            if best['uncertainty_ns']>5_000_000:raise RuntimeError('Clock uncertainty exceeds 5ms')
            sync[role]={'offset_ns':best['offset_ns'],'uncertainty_ns':best['uncertainty_ns']}
        start=time.monotonic_ns()+1_000_000_000
        for role in ROLES:emit(role,'ARM',start_ns=start+sync[role]['offset_ns'])
        armed=set()
        while len(armed)<2 and time.monotonic_ns()<start-300_000_000:
            value=receive()
            if value and value[2]['kind']=='ARMED':armed.add(value[1])
            if value and value[2]['kind']=='STOPPED':raise RuntimeError('Arm failed:'+value[2]['result']['reason'])
        if len(armed)!=2:raise RuntimeError('Arm barrier timeout')
        for role in ROLES:emit(role,'COMMIT')
        committed=set()
        while len(committed)<2 and time.monotonic_ns()<start-100_000_000:
            value=receive()
            if value and value[2]['kind']=='COMMITTED':committed.add(value[1])
        if len(committed)!=2:raise RuntimeError('Commit barrier timeout')
        hb=0;probe=0;clock_good={role:time.monotonic_ns() for role in ROLES};clock_bad={role:0 for role in ROLES}
        while len(stops)<2:
            now=time.monotonic_ns();elapsed=now-start;blackout=scenario=='coordinator_loss' and elapsed>=300_000_000
            if now-hb>20_000_000 and not blackout:
                for role in ROLES:
                    if not (scenario=='hands_heartbeat_loss' and role=='hands' and elapsed>=300_000_000):emit(role,'HEARTBEAT')
                hb=now
            if now-probe>50_000_000 and not blackout:
                for role in ROLES:
                    if role not in stops:
                        seq+=1;probes[role,seq]=now;emit(role,'PING',seq=seq)
                probe=now
            value=receive()
            if value:
                t,role,m=value
                if m['kind']=='STOPPED' and m['result']['reason']!='completed' and not blackout:fail(role+':'+m['result']['reason'])
                if m['kind']=='EOF' and role not in stops:fail(role+':missing_stop_ack')
                if m['kind']=='BAD_WIRE':fail(role+':invalid_wire_message')
                if m['kind']=='PONG' and 'offset_ns' in m and role not in stops and not blackout:
                    if m['uncertainty_ns']<=5_000_000:
                        clock_good[role]=now;clock_bad[role]=0
                        if abs(m['offset_ns']-sync[role]['offset_ns'])>5_000_000+m['uncertainty_ns']+sync[role]['uncertainty_ns']:
                            fail(role+':clock_mapping_changed')
                    else:
                        clock_bad[role]+=1
                        if clock_bad[role]>=3:fail(role+':clock_uncertainty_exceeded')
                if blackout and m['kind']=='STOPPED' and trigger is None:
                    trigger='coordinator_heartbeat_loss';trigger_ns=start+300_000_000
            if not blackout:
                for role in ROLES:
                    if role not in stops and now-clock_good[role]>300_000_000:fail(role+':clock_sync_stale')
            if trigger_ns is not None and now-trigger_ns>2_000_000_000:break
            if elapsed>plan['duration_ns']+2_000_000_000:fail('Completion timeout');break
    except BaseException as e:fail(str(e))
    finally:
        until=time.monotonic()+2
        while len(stops)<2 and time.monotonic()<until:receive(.01)
        for p in procs.values():
            try:p.stdin.close()
            except (OSError,ValueError):pass
        for p in procs.values():
            try:p.wait(8)
            except subprocess.TimeoutExpired:p.terminate();p.wait(5)
        for thread in readers:thread.join(timeout=2)
        for f in logs:f.close()
    confirmed=len(stops)==2 and all(all(s['confirmed'] for s in r['stop'].values()) for r in stops.values())
    normal=confirmed and trigger is None and all(s['reason']=='completed' for s in stops.values())
    exact=True;max_delay=0;max_error=0;first=[]
    for role,r in stops.items():
        for c in ROLES[role]:
            observed=[e for e in r['commands'] if e['channel']==c];expected=plan['channels'][c]
            for i,e in enumerate(observed):
                exact &= e['event_index']==i and e['source_hash']==expected[i]['payload_sha256'] and e['offset_ns']==expected[i]['offset_ns']
                max_delay=max(max_delay,e['lateness_ns'])
                if role in sync and start is not None:max_error=max(max_error,abs(e['local_ns']-sync[role]['offset_ns']-start-e['offset_ns']))
            if normal:exact &= len(observed)==len(expected)
            if observed:first.append(observed[0]['local_ns']-sync[role]['offset_ns'])
        if role=='hands':
            sent={(e['channel'],e['event_index']):e['source_hash'] for e in r['commands']}
            receipts={(e['channel'],e['event_index']):e['source_hash'] for e in r['receipts']}
            if normal:exact &= sent==receipts
    expected={'left_sdk_reject':'hands:left_hand:SDK rejected','right_sdk_reject':'hands:right_hand:SDK rejected',
              'body_feedback_loss':'robot:body:feedback_timeout','chassis_feedback_loss':'robot:chassis:feedback_timeout',
              'hands_heartbeat_loss':'hands:Heartbeat timeout','coordinator_loss':'coordinator_heartbeat_loss',
              'clock_jump':'hands:clock_mapping_changed','clock_probe_loss':'hands:clock_sync_stale','robot_process_exit':'robot:missing_stop_ack','hands_stop_ack_loss':'hands:missing_stop_ack'}
    passed=normal if scenario=='normal' else (trigger is not None and trigger.startswith(expected[scenario]))
    if scenario not in ('normal','robot_process_exit','hands_stop_ack_loss','body_feedback_loss','chassis_feedback_loss'):passed &= confirmed
    if scenario in ('body_feedback_loss','chassis_feedback_loss'):passed &= not confirmed and len(stops)==2
    if scenario in ('robot_process_exit','hands_stop_ack_loss'):passed &= not confirmed and len(stops)==1
    report=dict(schema='unified_controller_integration_v1',scenario=scenario,experiment_passed=bool(passed and exact and max_delay<=50_000_000),
                status='COMPLETED' if normal else ('STOPPED_ON_FAULT' if confirmed else 'STOP_UNCONFIRMED'),
                trigger=trigger,stop_confirmed=confirmed,missing_stop_ack=sorted(set(ROLES)-set(stops)),
                physical_execution=False,physical_stop_verified=False,hardware_commands_sent=0,
                namespace=session['namespace'],domain=197,transport='SSH stdio robot + local stdio hands; DDS localhost on each host',
                plan_sha256=plan['plan_sha256'],session_sha256=session['session_sha256'],ready=ready,clock_mapping=sync,
                source_payload_order_verified=bool(exact),max_lateness_ns=max_delay,max_common_schedule_error_ns=max_error,
                first_command_spread_ns=max(first)-min(first) if len(first)==4 else None,executors=stops,
                limits=['Ideal robot feedback and mock mux/chassis','Sharpa real ROS driver with fake actuators',
                        'Native model home initialization is a diagnostic fixture, not recorded-start acceptance','Physical execution remains disabled'])
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));return report
