"""Strict LeRobot-only replay planner. No ROS, SSH, hardware or motion imports."""
import hashlib
import json
import re
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq
from unified_capture import HAND_NAMES, BODY_NAMES, SERIALS

IK_POSES = ('left', 'right', 'left_elbow', 'right_elbow', 'torso', 'head',
            'pelvis', 'left_foot', 'right_foot', 'left_knee', 'right_knee')
JOINT_NAMES = [f'Joint{i}_{side}' for side in ('L', 'R') for i in range(1, 8)] + BODY_NAMES
CHANNELS = ('body', 'left_hand', 'right_hand', 'chassis')
# Snapshot of installed Sharpa Wave schema, read as source without loading SDK.
HAND_LIMITS = [(-.1745,1.9199),(-.3491,.3491),(-.5236,1.3963),(-.3491,.3491),(0,1.7453),
              (-.17453293,1.5708),(-.3491,.3491),(0,1.7453),(0,1.3963),
              (-.17453293,1.5708),(-.3491,.3491),(0,1.7453),(0,1.3963),
              (-.17453293,1.5708),(-.3491,.3491),(0,1.7453),(0,1.3963),(0,.2618),
              (-.17453293,1.5708),(-.3491,.3491),(0,1.7453),(0,1.3963)]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def fingerprint(value):
    return hashlib.sha256(canonical(value)).hexdigest()


def file_hash(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def verify_episode(root):
    root = Path(root).absolute()
    require(root == root.resolve(), 'Dataset path/symlink invalid')
    require(root.is_dir(), 'Episode directory missing')
    require(not any(p.is_symlink() for p in root.rglob('*')), 'Dataset symlinks forbidden')
    completion = root / 'meta/completion.json'
    expected = json.loads(completion.read_text())
    actual = {str(p.relative_to(root)): file_hash(p) for p in root.rglob('*')
              if p.is_file() and p != completion}
    require(actual == expected and bool(actual), 'LeRobot completion hash/inventory mismatch')
    return file_hash(completion)


def stamps(values, label):
    require(isinstance(values, list) and len(values) >= 2 and all(type(t) is int for t in values),
            label + ': integer ns timestamps required')
    a = np.asarray(values, dtype='i8')
    require(np.all(a > 0) and np.all(np.diff(a) > 0), label + ': non-increasing timeline')
    return a


def numeric(values, shape, label):
    a = np.asarray(values, dtype='f8')
    require(a.shape == shape and np.isfinite(a).all(), label + ': invalid shape/nonfinite values')
    return a


def poses(values, n, label):
    a = numeric(values, (n, 9), label)
    x, y = a[:, 3:6], a[:, 6:9]
    require(np.allclose(np.linalg.norm(x, axis=1), 1, atol=2e-5, rtol=0)
            and np.allclose(np.linalg.norm(y, axis=1), 1, atol=2e-5, rtol=0)
            and np.allclose((x*y).sum(axis=1), 0, atol=2e-5, rtol=0), label + ': invalid rot6d')
    return a


def build_plan(root, expected_episode=None, allow_diagnostic=False):
    root = Path(root).absolute()
    before = verify_episode(root)
    read = lambda name: json.loads((root / 'meta' / name).read_text())
    info, capture, replay, timeline = [read(n) for n in
        ('info.json', 'capture.json', 'unified_replay.json', 'source_timestamps.json')]
    eid = info.get('source_episode_index')
    require(type(eid) is int and 0 <= eid < 100_000_000, 'Invalid original episode index')
    require(expected_episode is None or eid == expected_episode, 'Selected episode mismatch')
    require(root.name == f'episode_{eid:08d}' and info.get('episode_ids') == [eid], 'Episode path/index mismatch')
    require(info.get('profile') == 'tianji_unified78' and info.get('codebase_version') == 'v2.1', 'Unified78 v2.1 required')
    require(info.get('coordinate_frame') == 'base_link' and info.get('fps') == 20, 'Frame/fps mismatch')
    require(info.get('total_episodes') == 1 and info.get('encoded_action_dim') == 78, 'Single original 78D episode required')
    require(bool(re.fullmatch('[0-9a-f]{32}', str(capture.get('capture_id')))), 'Invalid capture ID')
    require(replay.get('schema') == 'tianji_unified_replay_v1', 'Unified replay data missing')
    require(capture.get('state') == 'COMPLETE' and capture.get('audit', {}).get('issues') == [], 'Capture incomplete/audit failed')
    for value in (info, capture, replay):
        require(value.get('capture_id') == capture['capture_id'], 'Capture identity mismatch')
        require(type(value.get('diagnostic')) is bool and value['diagnostic'] == capture['diagnostic'], 'Diagnostic identity mismatch')
    for value in (capture, replay):
        require(value.get('episode_index') == eid and value.get('episode_name') == f'episode_{eid:08d}', 'Replay episode mismatch')
        require(value.get('clock_domain') == 'robot_receive', 'Clock domain mismatch')
        require(value.get('serials') == SERIALS, 'Device serial mismatch')
    require(not capture['diagnostic'] or allow_diagnostic, 'Diagnostic data requires explicit offline/simulation opt-in')
    require(replay.get('gripper_commands_enabled') is False, 'Grippers must remain disabled')
    mask = [i not in (54,77) for i in range(78)]
    for key in ('state_mask','action_mask'):
        require(info.get(key) == mask, 'Only gripper slots 54/77 may be masked')
    expected_tail = BODY_NAMES + ['left_hand_'+n for n in HAND_NAMES] + ['left_gripper_mm'] + ['right_hand_'+n for n in HAND_NAMES] + ['right_gripper_mm']
    for key in ('observation.state','action','validity'):
        require(info['features'][key]['shape'] == [78], '78D feature required')
        require(info['features'][key].get('names', [])[27:] == expected_tail, '78D body/hand/gripper order mismatch')
    t = stamps(timeline['timestamp_ns'], 'training')
    require(len(t) == info['total_frames'] and np.all(np.diff(t) == 50_000_000), 'Training grid mismatch')
    origin, end = replay['origin_ns'], replay['end_ns']
    require(type(origin) is int and type(end) is int and origin == int(t[0]) and end == int(t[-1])
            and capture['start_ack_ns'] <= origin < end <= capture['stop_request_ns'], 'Replay window mismatch')
    require(timeline.get('source_episode_index') == eid and timeline.get('capture_id') == capture['capture_id'], 'Training timeline identity mismatch')
    table_path = root / f'data/chunk-{eid//1000:03d}/episode_{eid:06d}.parquet'
    columns = ['episode_index','frame_index','timestamp','observation.state','action','validity']
    table = pq.read_table(table_path, columns=columns).to_pydict()
    n = len(t)
    require(table['episode_index'] == [eid]*n and table['frame_index'] == list(range(n)), 'Parquet index mismatch')
    require(np.allclose(table['timestamp'], (t-t[0])/1e9, atol=2e-6, rtol=0), 'Parquet timestamp mismatch')
    validity = numeric(table['validity'], (n,78), 'validity')
    require(np.array_equal(validity, np.tile(mask, (n,1))), 'Per-frame mask mismatch')
    for key in ('observation.state','action'):
        a = numeric(table[key], (n,78), key)
        require(np.all(a[:, [54,77]] == 0), 'Masked gripper values must be zero')
        for k in (0,9,18): poses(a[:,k:k+9], n, key)
    native = replay['native_ik_chassis']
    require(native == timeline.get('native_replay'), 'Duplicated native replay sources disagree')
    require(native['schema'] == 'skye_native_replay_v1' and native['coordinate_frame'] == 'base_link', 'Native IK schema/frame mismatch')
    it = stamps(native['ik_timestamp_ns'], 'IK')
    require(replay['ik_offset_ns'] == (it-origin).tolist(), 'IK offsets mismatch')
    require(set(native['ik']) == {*IK_POSES, 'available', 'joint_positions'}, 'Incomplete IK fields')
    for key in IK_POSES: poses(native['ik'][key], len(it), 'IK/'+key)
    gate = np.asarray(native['ik']['available'])
    require(gate.shape == (len(it),11) and np.isin(gate,[0,1]).all(), 'Invalid IK availability')
    numeric(native['ik']['joint_positions'], (len(it),19), 'IK seed')
    require(native['initial_joint_names'] == JOINT_NAMES, 'Initial joint order mismatch')
    numeric(native['initial_joint_positions'], (19,), 'Initial joint feedback')
    streams = replay['streams']
    specs = {'observations/full_joints': (19,'m,rad', JOINT_NAMES),
             'observations/body_joints': (5,'m,rad,rad,rad,rad',BODY_NAMES),
             'actions/body_controller': (3,'m,rad,rad',BODY_NAMES[:3]),
             'actions/head_controller': (2,'rad,rad',BODY_NAMES[3:]),
             'observations/chassis': (3,'m/s,m/s,rad/s',None),
             'actions/chassis': (3,'m/s,m/s,rad/s',None)}
    specs.update({f'{role}/{side}_hand':(22,'rad',HAND_NAMES)
                  for role in ('observations','actions') for side in ('left','right')})
    topics = {'observations/full_joints':'/tj/joint_states', 'observations/body_joints':'/tj/joint_states',
              'actions/body_controller':'/tj/control/joint_cmd_body', 'actions/head_controller':'/tj/control/joint_cmd_head',
              'observations/chassis':'/recording/chassis/state', 'actions/chassis':'/recording/chassis/manual_move_cmd'}
    topics.update({f'{role}/{side}_hand':f'/hand_{side}/joint_'+('commands' if role=='actions' else 'states')
                   for role in ('observations','actions') for side in ('left','right')})
    for key,(width,unit,names) in specs.items():
        s = streams[key]; st = stamps(s['timestamp_ns'], key)
        values = numeric(s['value'], (len(st),width), key)
        require(s['unit'] == unit, key+': unit mismatch')
        require(s['source_topic'] == topics[key], key+': source topic mismatch')
        if names is not None:
            if 'joint_names' in s:
                recorded_names = json.loads(s['joint_names'])
            else:
                feature = {'actions/body_controller':'action.body_controller', 'actions/head_controller':'action.head_controller'}.get(key)
                recorded_names = info['features'].get(feature, {}).get('names')
            require(recorded_names == names, key+': joint order mismatch')
        require(s['offset_ns'] == (st-origin).tolist(), key+': offset mismatch')
        before_origin = np.flatnonzero(st <= origin)
        require(len(before_origin) > 0 and st[-1] >= end, key+': missing window coverage')
        initial = int(before_origin[-1])
        require(s['initial_source_index'] == initial and s['initial_timestamp_ns'] == int(st[initial])
                and np.array_equal(s['initial_value'], values[initial]), key+': initial reference mismatch')
        require(np.max(np.diff(st)) <= (200_000_000 if 'hand' in key else 100_000_000), key+': recorded gap too large')
        if 'hand' in key:
            side = key.split('/')[1].split('_')[0]
            require(s['serial'] == SERIALS[side] and s['session_id'] == capture['source_session_id'], key+': hand identity mismatch')
            stage = 'sdk_call_succeeded' if key.startswith('actions/') else 'measured_feedback'
            require(s['source_stage'] == stage, key+': wrong hand source stage')
            limits = np.asarray(HAND_LIMITS)
            require(np.all(values >= limits[:,0]) and np.all(values <= limits[:,1]), key+': SDK joint limits exceeded')
            require(len(s['seq']) == len(st) and all(type(x) is int for x in s['seq']) and np.all(np.diff(s['seq']) > 0), key+': hand sequence invalid')
    # Verify the 20Hz training body/hand values against their original-rate
    # replay samples and exact source indices, rather than trusting two copies.
    mapping = [('observations/body_joints','observations/body_joints','observation.state',27,5),
               ('actions/body_controller','actions/body_controller','action',27,3),
               ('actions/head_controller','actions/head_controller','action',30,2)]
    mapping += [(f'{role}/{side}_hand',f'{alias}/{side}',column,start,22)
                for role,alias,column in [('observations','state','observation.state'),('actions','action','action')]
                for side,start in [('left',32),('right',55)]]
    for stream,aligned,column,start,width in mapping:
        entry=timeline['aligned_sources'][aligned]
        indices=entry['source_index'];source=streams[stream]
        require(len(indices)==n and all(type(i) is int and 0<=i<len(source['value']) for i in indices), stream+': invalid aligned indices')
        times=np.asarray([source['timestamp_ns'][i] for i in indices],dtype='i8')
        require(entry['timestamp_ns']==times.tolist() and np.all(np.abs(times-t)<=50_000_000), stream+': aligned time mismatch')
        if stream.startswith('actions/'):require(np.all(times<=t), stream+': future action')
        require(np.allclose(np.asarray(table[column])[:,start:start+width],np.asarray(source['value'])[indices],atol=1e-6,rtol=1e-6), stream+': training/replay values disagree')
    ct = stamps(native['chassis_timestamp_ns'], 'chassis')
    cv = numeric(native['chassis_velocity'], (len(ct),3), 'chassis')
    require(native['chassis_frame'] == 'base_link' and native['chassis_units'] == ['m/s','m/s','rad/s'], 'Chassis frame/units mismatch')
    require(np.max(np.abs(cv[:,0])) <= .400001 and np.max(np.abs(cv[:,1])) <= 1e-8
            and np.max(np.abs(cv[:,2])) <= .500001, 'Chassis command bounds exceeded')
    require(ct.tolist() == streams['actions/chassis']['timestamp_ns'] and cv.tolist() == streams['actions/chassis']['value'], 'Chassis replay sources disagree')
    body = [{key: native['ik'][key][i] for key in (*IK_POSES,'available','joint_positions')} for i in range(len(it))]
    inputs = {'body':(it,body), 'chassis':(ct,cv.tolist())}
    for side in ('left','right'):
        s = streams[f'actions/{side}_hand'];inputs[side+'_hand'] = (np.array(s['timestamp_ns'],dtype='i8'),s['value'])
    channels = {}
    for name,(times,values) in inputs.items():
        prior = np.flatnonzero(times <= origin)
        require(len(prior) > 0 and times[-1] >= end, name+': missing window coverage')
        require(np.max(np.diff(times)) <= (200_000_000 if 'hand' in name else 100_000_000), name+': recorded gap too large')
        initial = int(prior[-1])
        # Seed each channel once at common t=0 with the causal active target.
        # Negative-time history stays in LeRobot; it is never burst-replayed.
        indices = [initial] + np.flatnonzero((times > origin) & (times <= end)).tolist()
        events = [{'offset_ns':0 if j == 0 else int(times[i]-origin),
                   'source_timestamp_ns':int(times[i]), 'source_index':i,
                   'initial':j == 0, 'payload':values[i], 'payload_sha256':fingerprint(values[i])}
                  for j,i in enumerate(indices)]
        require(origin-int(times[initial]) <= (200_000_000 if 'hand' in name else 100_000_000), name+': stale initial target')
        channels[name] = events
    initial_feedback = {name:streams[key]['initial_value'] for name,key in
        [('body','observations/full_joints'),('left_hand','observations/left_hand'),
         ('right_hand','observations/right_hand'),('chassis','observations/chassis')]}
    require(verify_episode(root) == before, 'Episode changed during planning')
    plan = dict(schema='unified_replay_plan_v1', backend='mock_only', hardware_execution_allowed=False,
                episode_index=eid, capture_id=capture['capture_id'], diagnostic=capture['diagnostic'],
                source=str(root), source_completion_sha256=before, origin_ns=origin, end_ns=end,
                duration_ns=end-origin, channels=channels, initial_feedback=initial_feedback,
                gripper_commands_enabled=False, serials=SERIALS, state_action_dimension=78,
                state_mask=mask, action_mask=mask,
                initial_policy='one causal active target at shared t0; no negative-offset replay',
                body_policy='original IK input; body/head controller output is reference only',
                unverified=['physical tracking','collision/reachability','real network timing','hardware stop confirmation'])
    plan['plan_sha256'] = fingerprint(plan)
    return plan


def verify_plan(plan):
    require(plan.get('backend') == 'mock_only' and plan.get('hardware_execution_allowed') is False,
            'Only hardware-free mock plans may run')
    require(plan.get('schema') == 'unified_replay_plan_v1' and plan.get('gripper_commands_enabled') is False,
            'Plan schema/gripper mismatch')
    require(plan.get('plan_sha256') == fingerprint({k:v for k,v in plan.items() if k!='plan_sha256'}), 'Plan hash mismatch')
    require(set(plan['channels']) == set(CHANNELS), 'All four channels required')
    for name, events in plan['channels'].items():
        require(events and events[0]['offset_ns'] == 0, 'Common initial target required')
        previous = -1
        for e in events:
            require(type(e['offset_ns']) is int and previous < e['offset_ns'] <= plan['duration_ns'], 'Invalid event order/window')
            require(e['payload_sha256'] == fingerprint(e['payload']), 'Payload hash mismatch')
            previous = e['offset_ns']


def validate_start_feedback(plan, feedback):
    require(set(feedback) == set(CHANNELS), 'Missing starting feedback')
    errors = {}
    for name in CHANNELS:
        expected = np.asarray(plan['initial_feedback'][name], dtype='f8')
        actual = numeric(feedback[name], expected.shape, name+': starting feedback')
        tolerance = np.full(expected.shape, .1 if 'hand' in name else .05)
        if name == 'body': tolerance[14] = .005  # prismatic waist, metres
        if name == 'chassis': tolerance[:] = .01
        error = np.abs(actual-expected)
        require(np.all(error <= tolerance), name+': initial_pose_mismatch')
        errors[name] = error.tolist()
    return errors
