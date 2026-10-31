"""Four subprocess mock executors over local pipes; never opens a device/socket.

This exercises timing/protocol logic, not ROS/QP dynamics or physical stopping.
Each executor has an independently offset monotonic clock and a local watchdog.
"""
import multiprocessing as mp
from multiprocessing.connection import wait
import os
import time

from .isolation import install
from .contract import CHANNELS, fingerprint, verify_plan, validate_start_feedback

HEARTBEAT_NS = 20_000_000
WATCHDOG_NS = 200_000_000
FEEDBACK_NS = 100_000_000
DEADLINE_NS = 50_000_000
CLOCK_UNCERTAINTY_NS = 5_000_000
CLOCK_DRIFT_NS = 5_000_000
LEAD_NS = 350_000_000
STOP_BOUND_NS = 400_000_000

SCENARIOS = ('normal', 'left_sdk_reject', 'right_sdk_reject', 'body_feedback_loss',
             'chassis_feedback_loss', 'left_heartbeat_loss', 'left_link_partition', 'coordinator_loss',
             'right_scheduler_stall', 'clock_jump', 'prepare_hash_mismatch',
             'missing_ready', 'start_pose_mismatch', 'body_process_exit', 'chassis_stop_ack_loss')


def _worker(pipe, name, events, duration, plan_hash, feedback, scenario, clock_offset):
    """The only output sink is an in-memory mock command log."""
    install()
    base = time.monotonic_ns()
    committed = False
    start = None
    index = 0
    last_heartbeat = base
    last_feedback = base
    last_status = base
    injected = False
    log = []
    state = 'PREPARED'
    stop_reason = None
    stop_ns = None
    jumped = 0
    pending_start = None
    local_start = None
    actual_start = None
    def clock():
        # A small rate skew plus large epoch skew; mappings must not use Unix time.
        now = time.monotonic_ns()
        return now + clock_offset + (now-base)*20//1_000_000 + jumped
    def send(kind, **values):
        pipe.send(dict(kind=kind, channel=name, **values))
    def stop(reason):
        nonlocal stop_reason, stop_ns, state
        if stop_reason is not None:
            return
        stop_reason = reason
        stop_ns = time.monotonic_ns()
        state = 'STOPPED'
        # Model an explicit zero-velocity/disable call, then acknowledge it.
        action = 'zero_velocity' if name == 'chassis' else 'disable'
        result = dict(reason=reason, real_stop_ns=stop_ns, local_stop_ns=clock(),
                      action=action, mock_stop_confirmed=True, commands=log,
                      actual_start_real_ns=actual_start, hardware_commands_sent=0,
                      process_id=os.getpid())
        if scenario == 'chassis_stop_ack_loss' and name == 'chassis':
            return
        send('STOPPED', **result)
    try:
        if scenario == 'start_pose_mismatch' and name == 'left_hand':
            feedback = [v+.2 for v in feedback]
        send('BOOT', plan_sha256=plan_hash, process_id=os.getpid(), feedback=feedback)
        while stop_reason is None:
            now = time.monotonic_ns()
            while pipe.poll():
                m = pipe.recv()
                kind = m['kind']
                if kind == 'PING':
                    send('PONG', seq=m['seq'], local_ns=clock())
                elif kind == 'HEARTBEAT':
                    last_heartbeat = now
                elif kind == 'ARM':
                    pending_start = m['start_local_ns']
                    if pending_start-clock() < LEAD_NS//2:
                        stop('start_deadline_invalid');break
                    if not (scenario == 'missing_ready' and name == 'right_hand'):
                        echo = 'bad-hash' if scenario == 'prepare_hash_mismatch' and name == 'left_hand' else plan_hash
                        send('ARMED', plan_sha256=echo)
                    state = 'ARMED'
                elif kind == 'COMMIT':
                    if state != 'ARMED' or pending_start is None or pending_start-clock() <= 0:
                        stop('invalid_commit');break
                    committed = True
                    local_start = pending_start
                    start = now + local_start-clock()
                    last_heartbeat = now
                    send('COMMITTED')
                elif kind == 'STOP':
                    stop(m['reason']);break
            if stop_reason is not None:
                break
            now = time.monotonic_ns()
            if committed:
                elapsed = clock()-local_start
                if elapsed >= 0 and actual_start is None:
                    actual_start = now
                    state = 'RUNNING'
                    send('STARTED', real_ns=now, local_ns=clock())
                active_fault = elapsed >= 300_000_000
                if active_fault and not injected:
                    if scenario == 'body_process_exit' and name == 'body':
                        os._exit(23)
                    if scenario == 'right_scheduler_stall' and name == 'right_hand':
                        time.sleep(.08);injected = True
                    if scenario == 'clock_jump' and name == 'left_hand':
                        jumped = 20_000_000;injected = True
                now = time.monotonic_ns()
                if now-last_heartbeat > WATCHDOG_NS:
                    stop('heartbeat_timeout');break
                lost_feedback = active_fault and ((scenario == 'body_feedback_loss' and name == 'body')
                    or (scenario == 'chassis_feedback_loss' and name == 'chassis'))
                if not lost_feedback:
                    last_feedback = now  # Mock driver feedback; never recorded feedback as live input.
                if now-last_feedback > FEEDBACK_NS:
                    stop('feedback_timeout');break
                if index < len(events) and clock() >= local_start+events[index]['offset_ns']:
                    event = events[index]
                    delay = clock()-(local_start+event['offset_ns'])
                    if delay > DEADLINE_NS:
                        stop('schedule_deadline_missed');break
                    if active_fault and scenario == name.replace('_hand','')+'_sdk_reject' and 'hand' in name:
                        stop('mock_sdk_rejected');break
                    # Check before dispatch; never catch up a backlog of late commands.
                    real = time.monotonic_ns()
                    log.append(dict(event_index=index, source_index=event['source_index'],
                                    source_timestamp_ns=event['source_timestamp_ns'],
                                    offset_ns=event['offset_ns'], payload_sha256=event['payload_sha256'],
                                    actual_real_ns=real, actual_local_ns=clock(), lateness_ns=delay,
                                    receipt='mock_sdk_accepted' if 'hand' in name else 'mock_published'))
                    index += 1
                if clock() >= local_start+duration and index == len(events):
                    stop('completed');break
                if now-last_status >= HEARTBEAT_NS:
                    send('STATUS', state=state, sent=index, feedback_age_ns=now-last_feedback)
                    last_status = now
            # No event may be sent merely because ARMED time has elapsed.
            elif pending_start is not None and clock() >= pending_start:
                stop('commit_timeout');break
            elif now-base > 8_000_000_000:
                stop('prepare_timeout');break
            time.sleep(.0005)
    except (EOFError, BrokenPipeError, OSError):
        try:stop('transport_closed')
        except (BrokenPipeError, OSError):pass
    except BaseException as exc:
        try:stop('worker_exception:'+repr(exc))
        except (BrokenPipeError, OSError):pass
    finally:
        pipe.close()


def simulate(plan, scenario='normal'):
    verify_plan(plan)
    if scenario not in SCENARIOS:
        raise ValueError('Unknown simulation scenario')
    ctx = mp.get_context('spawn')
    pipes, processes, stops, sync, states = {}, {}, {}, {}, {}
    offsets = {'body':0, 'left_hand':35_450_000_000, 'right_hand':-12_345_000_000, 'chassis':7_000_000_000}
    began = time.monotonic_ns()
    transition = [{'state':'PREPARING', 'real_ns':began}]
    starts = {}
    trigger = None
    trigger_ns = None
    common_start = None
    last_rx = {}
    probes = {}
    sequence = 0
    messages = []
    closed = set()
    write_closed = set()
    def send(name, kind, **value):
        if name in write_closed:
            return False
        if scenario=='left_link_partition' and name=='left_hand' and common_start is not None and time.monotonic_ns()-common_start>=300_000_000:
            return False
        try:pipes[name].send(dict(kind=kind, **value));return True
        except (BrokenPipeError, EOFError, OSError):write_closed.add(name);return False
    def receive(timeout=.002):
        result=[]
        active=[p for k,p in pipes.items() if k not in closed]
        for pipe in wait(active, timeout=timeout) if active else []:
            name=next(k for k,v in pipes.items() if v is pipe)
            try:
                while pipe.poll():
                    msg=pipe.recv();now=time.monotonic_ns()
                    if scenario=='left_link_partition' and name=='left_hand' and common_start is not None and now-common_start>=300_000_000:
                        continue
                    last_rx[name]=now
                    if msg['kind']=='STOPPED':stops[name]=msg
                    elif msg['kind']=='STARTED':starts[name]=msg['real_ns']
                    elif msg['kind']=='PONG':
                        t0=probes.pop((name,msg['seq']),None)
                        if t0 is not None:
                            bound=(now-t0)//2
                            estimate=msg['local_ns']-(t0+now)//2
                            msg.update(uncertainty_ns=bound, offset_ns=estimate)
                    result.append(msg)
            except (EOFError,OSError):closed.add(name)
        return result
    def fail(reason):
        nonlocal trigger,trigger_ns
        if trigger is None:
            trigger=reason;trigger_ns=time.monotonic_ns()
            transition.append({'state':'STOPPING','reason':reason,'real_ns':trigger_ns})
        for name in CHANNELS:
            send(name,'STOP',reason='coordinated_stop:'+reason)
    try:
        for name in CHANNELS:
            parent,child=ctx.Pipe()
            process=ctx.Process(target=_worker,args=(child,name,plan['channels'][name],plan['duration_ns'],
                plan['plan_sha256'],plan['initial_feedback'][name],scenario,offsets[name]),name='mock-replay-'+name)
            process.start();child.close();pipes[name]=parent;processes[name]=process
        deadline=time.monotonic_ns()+6_000_000_000
        boot=set();boot_feedback={}
        while len(boot)<4 and time.monotonic_ns()<deadline:
            for msg in receive(.01):
                if msg['kind']=='BOOT' and msg['plan_sha256']==plan['plan_sha256']:
                    boot.add(msg['channel']);boot_feedback[msg['channel']]=msg['feedback']
        if len(boot)!=4:raise RuntimeError('executor_boot_timeout')
        validate_start_feedback(plan,boot_feedback)
        # Repeated local round trips exercise the same epoch-independent mapping
        # needed by a future network transport. Bounds include process scheduling.
        samples={name:[] for name in CHANNELS}
        for _ in range(5):
            waiting=set(CHANNELS)
            for name in CHANNELS:
                sequence+=1;probes[name,sequence]=time.monotonic_ns();send(name,'PING',seq=sequence)
            until=time.monotonic_ns()+500_000_000
            while waiting and time.monotonic_ns()<until:
                for msg in receive():
                    if msg['kind']=='PONG' and 'offset_ns' in msg:
                        samples[msg['channel']].append(msg);waiting.discard(msg['channel'])
            if waiting:raise RuntimeError('clock_probe_timeout')
        for name in CHANNELS:
            best=min(samples[name],key=lambda m:m['uncertainty_ns'])
            if best['uncertainty_ns']>CLOCK_UNCERTAINTY_NS:raise RuntimeError('clock_uncertainty_exceeded')
            sync[name]={'offset_ns':best['offset_ns'],'uncertainty_ns':best['uncertainty_ns'],
                        'injected_epoch_offset_ns':offsets[name]}
        common_start=time.monotonic_ns()+LEAD_NS
        for name in CHANNELS:send(name,'ARM',start_local_ns=common_start+sync[name]['offset_ns'])
        armed=set()
        while len(armed)<4 and time.monotonic_ns()<common_start-150_000_000:
            for msg in receive():
                if msg['kind']=='ARMED':
                    if msg['plan_sha256']!=plan['plan_sha256']:raise RuntimeError('prepared_plan_hash_mismatch')
                    armed.add(msg['channel'])
        if len(armed)!=4:raise RuntimeError('ready_barrier_timeout')
        transition.append({'state':'ARMED','real_ns':time.monotonic_ns()})
        for name in CHANNELS:send(name,'COMMIT')
        committed=set()
        while len(committed)<4 and time.monotonic_ns()<common_start-50_000_000:
            for msg in receive():
                if msg['kind']=='COMMITTED':committed.add(msg['channel'])
        if len(committed)!=4:raise RuntimeError('commit_barrier_timeout')
        transition.append({'state':'COMMITTED','real_ns':time.monotonic_ns()})
        last_heartbeat=0;last_probe=0
        while len(stops)<4:
            now=time.monotonic_ns();elapsed=now-common_start
            blackout=scenario=='coordinator_loss' and elapsed>=300_000_000
            if now-last_heartbeat>=HEARTBEAT_NS and not blackout:
                for name in CHANNELS:
                    if not (scenario=='left_heartbeat_loss' and name=='left_hand' and elapsed>=300_000_000):
                        send(name,'HEARTBEAT')
                last_heartbeat=now
            if now-last_probe>=50_000_000 and not blackout:
                for name in CHANNELS:
                    if name not in stops:
                        sequence+=1;probes[name,sequence]=now;send(name,'PING',seq=sequence)
                last_probe=now
            for msg in receive():
                name=msg['channel']
                if msg['kind']=='STARTED' and not any(x['state']=='RUNNING' for x in transition):
                    transition.append({'state':'RUNNING','real_ns':msg['real_ns']})
                if msg['kind']=='STOPPED' and msg['reason']!='completed' and not blackout:
                    fail(name+':'+msg['reason'])
                if msg['kind']=='PONG' and 'offset_ns' in msg and not blackout and name not in stops:
                    change=abs(msg['offset_ns']-sync[name]['offset_ns'])
                    allowance=CLOCK_DRIFT_NS+msg['uncertainty_ns']+sync[name]['uncertainty_ns']
                    if msg['uncertainty_ns']>CLOCK_UNCERTAINTY_NS:fail(name+':clock_uncertainty_exceeded')
                    elif change>allowance:fail(name+':clock_mapping_changed')
            for name,process in processes.items():
                if process.exitcode is not None and name not in stops:
                    fail(name+':missing_stop_ack')
                elif name not in stops and now-last_rx.get(name,now)>WATCHDOG_NS+50_000_000 and not blackout:
                    fail(name+':executor_status_timeout')
            if blackout and any(s['reason']=='heartbeat_timeout' for s in stops.values()) and trigger is None:
                # Observe only: deliberately do not send a central STOP in this case.
                trigger='coordinator_heartbeat_loss';trigger_ns=common_start+300_000_000
                transition.append({'state':'STOPPING','reason':trigger,'real_ns':now})
            if trigger_ns is not None and now-trigger_ns>STOP_BOUND_NS+200_000_000:break
            if elapsed>plan['duration_ns']+STOP_BOUND_NS+500_000_000:
                fail('completion_timeout');break
        if len(stops)<4 and trigger is None:fail('stop_ack_timeout')
    except BaseException as exc:
        fail(str(exc))
    finally:
        # Collect stop acknowledgements, but never infer one from process exit.
        if len(stops)<4:
            until=time.monotonic_ns()+STOP_BOUND_NS
            while len(stops)<4 and time.monotonic_ns()<until:receive(.01)
        for name,process in processes.items():
            process.join(timeout=.3)
            if process.is_alive():process.terminate();process.join(timeout=1)
        for pipe in pipes.values():pipe.close()
    verified_stop=len(stops)==4 and all(s['mock_stop_confirmed'] for s in stops.values())
    complete=verified_stop and all(s['reason']=='completed' for s in stops.values()) and trigger is None
    log_matches=True;max_lateness=0;emitted=0;after_stop=0;max_schedule_error=0
    for name,s in stops.items():
        observed=s['commands'];expected=plan['channels'][name]
        for i,event in enumerate(observed):
            if i>=len(expected) or event['event_index']!=i or event['payload_sha256']!=expected[i]['payload_sha256'] or event['offset_ns']!=expected[i]['offset_ns']:
                log_matches=False
            max_lateness=max(max_lateness,event['lateness_ns'])
            if common_start is not None:
                max_schedule_error=max(max_schedule_error,abs(event['actual_real_ns']-common_start-event['offset_ns']))
            after_stop+=int(event['actual_real_ns']>s['real_stop_ns'])
        emitted+=len(observed)
        if complete and len(observed)!=len(expected):log_matches=False
    stop_latency=max((s['real_stop_ns']-trigger_ns for s in stops.values()),default=0) if trigger_ns else 0
    start_spread=max(starts.values())-min(starts.values()) if len(starts)==4 else None
    status='COMPLETED' if complete else ('STOPPED_ON_FAULT' if verified_stop else 'STOP_UNCONFIRMED')
    transition.append({'state':status,'real_ns':time.monotonic_ns()})
    report=dict(schema='unified_replay_simulation_v1',scenario=scenario,status=status,
        backend='local_pipe_subprocess_mock',hardware_initialized=False,hardware_commands_sent=0,
        physical_stop_verified=False,plan_sha256=plan['plan_sha256'],episode_index=plan['episode_index'],
        capture_id=plan['capture_id'],common_start_monotonic_ns=common_start,clock_mapping=sync,
        started_channels=list(starts),start_spread_ns=start_spread,trigger=trigger,
        trigger_monotonic_ns=trigger_ns,stop_latency_ns=max(0,stop_latency),
        mock_stop_confirmed=verified_stop,missing_stop_ack=sorted(set(CHANNELS)-set(stops)),
        payload_order_verified=log_matches,commands_after_local_stop=after_stop,
        max_lateness_ns=max_lateness,max_common_schedule_error_ns=max_schedule_error,mock_commands_emitted=emitted,
        thresholds_ns={'heartbeat_period':HEARTBEAT_NS,'watchdog':WATCHDOG_NS,'feedback':FEEDBACK_NS,
                       'schedule_lateness':DEADLINE_NS,'clock_uncertainty':CLOCK_UNCERTAINTY_NS,'stop_bound':STOP_BOUND_NS},
        transitions=transition,executors=stops,
        process_exitcodes={name:p.exitcode for name,p in processes.items()},
        limits=['No ROS/QP/SDK or device dynamics','No physical tracking or stop certification',
                'Clock-offset injection uses same-host subprocesses; real network not tested'])
    # This is an oracle for an injected experiment, not a claim the run succeeded.
    expected_trigger={
        'left_sdk_reject':'left_hand:mock_sdk_rejected', 'right_sdk_reject':'right_hand:mock_sdk_rejected',
        'body_feedback_loss':'body:feedback_timeout', 'chassis_feedback_loss':'chassis:feedback_timeout',
        'left_heartbeat_loss':'left_hand:heartbeat_timeout', 'left_link_partition':'left_hand:missing_stop_ack', 'coordinator_loss':'coordinator_heartbeat_loss',
        'right_scheduler_stall':'right_hand:schedule_deadline_missed', 'clock_jump':'left_hand:clock_mapping_changed',
        'prepare_hash_mismatch':'prepared_plan_hash_mismatch','missing_ready':'ready_barrier_timeout',
        'start_pose_mismatch':'left_hand: initial_pose_mismatch',
        'body_process_exit':'body:missing_stop_ack','chassis_stop_ack_loss':'chassis:missing_stop_ack'}
    if scenario=='normal':
        passed=complete and start_spread is not None and start_spread<=DEADLINE_NS
    else:
        passed=trigger==expected_trigger[scenario]
        if scenario in ('body_process_exit','chassis_stop_ack_loss','left_link_partition'):
            passed &= not verified_stop and len(stops)==3
        else:passed &= verified_stop and stop_latency<=STOP_BOUND_NS
        if scenario in ('prepare_hash_mismatch','missing_ready','start_pose_mismatch'):passed &= emitted==0 and not starts
    report['experiment_passed']=bool(passed and log_matches and after_stop==0 and max_lateness<=DEADLINE_NS)
    return report
