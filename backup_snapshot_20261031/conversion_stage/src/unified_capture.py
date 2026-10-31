"""Certified single-episode Raw contracts, hand evidence and replay preservation.
No hardware, ROS control, or motion interfaces are imported here.
"""
import hashlib
import json
import re
from pathlib import Path
import numpy as np

PROFILE = 'tianji_unified78'
HAND_NAMES = 'thumb_CMC_FE thumb_CMC_AA thumb_MCP_FE thumb_MCP_AA thumb_IP index_MCP_FE index_MCP_AA index_PIP index_DIP middle_MCP_FE middle_MCP_AA middle_PIP middle_DIP ring_MCP_FE ring_MCP_AA ring_PIP ring_DIP pinky_CMC pinky_MCP_FE pinky_MCP_AA pinky_PIP pinky_DIP'.split()
BODY_NAMES = ['waist_joint_1','waist_joint_2','waist_joint_3','Head_J1_Joint','Head_J2_Joint']
SERIALS = {'left':'CE50903ECE53','right':'CC50933FCC53'}
CONTEXT='/recording/sharpa/context'
EVENT='/recording/capture_event'
TOPICS={f'/hand_{side}/joint_{suffix}':(side,role) for side in ('left','right') for role,suffix in [('state','states'),('action','commands')]}

def digest(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

def episode_index(value):
    if type(value) is int and 0<=value<100_000_000:return value
    m=re.fullmatch(r'(?:episode_)?([0-9]{8})',str(value))
    if not m:raise ValueError('Explicit eight-digit episode identity required')
    return int(m[1])

def validate_identity(manifest, expected=None):
    idx=episode_index(manifest.get('episode_index'))
    if manifest.get('episode_name')!=f'episode_{idx:08d}':raise ValueError('Episode name/index mismatch')
    if expected is not None and idx!=episode_index(expected):raise ValueError('Episode identity mismatch')
    if not re.fullmatch('[0-9a-f]{32}',manifest.get('capture_id','')):raise ValueError('Invalid capture identity')
    return idx

def verify_raw(root, allow_diagnostic=False, expected=None):
    root=Path(root).resolve()
    c=json.loads((root/'capture_completion.json').read_text())
    m=json.loads((root/'capture_manifest.json').read_text())
    if c.get('schema')!='apex_capture_completion_v1' or m.get('schema')!='apex_capture_manifest_v1':raise ValueError('Capture schema mismatch')
    if c.get('state')!='COMPLETE' or m.get('state')!='COMPLETE':raise ValueError('Capture is not COMPLETE')
    if digest(root/'capture_manifest.json')!=c.get('manifest_sha256'):raise ValueError('Capture manifest hash mismatch')
    if m.get('capture_id')!=c.get('capture_id') or type(m.get('diagnostic')) is not bool or m['diagnostic']!=c.get('diagnostic'):raise ValueError('Capture identity/diagnostic mismatch')
    if m['diagnostic'] and not allow_diagnostic:raise ValueError('Diagnostic capture rejected for production')
    idx=validate_identity(m,expected)
    if c.get('episode_index')!=idx:raise ValueError('Completion episode mismatch')
    if m.get('clock_domain')!='robot_receive':raise ValueError('Wrong capture clock domain')
    if not isinstance(m.get('raw_files'),dict) or not m['raw_files']:raise ValueError('No certified raw files')
    actual={str(p.relative_to(root)) for p in root.rglob('*') if p.is_file()}
    if any(p.is_symlink() for p in root.rglob('*')):raise ValueError('Raw symlinks forbidden')
    if actual!=set(m['raw_files'])|{'capture_manifest.json','capture_completion.json'}:raise ValueError('Raw inventory mismatch')
    for rel,e in m['raw_files'].items():
        p=(root/rel).resolve()
        if not p.is_relative_to(root) or p==root or p.stat().st_size!=e['size'] or digest(p)!=e['sha256']:raise ValueError('Raw file hash/size mismatch: '+rel)
    if m.get('audit',{}).get('issues')!=[]:raise ValueError('Capture audit not clean')
    if not (type(m.get('start_ack_ns')) is int and type(m.get('stop_request_ns')) is int and m['start_ack_ns']<m['stop_request_ns']):raise ValueError('Capture window invalid')
    return m

def hand_rows(root, manifest, expected_serials):
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    if manifest.get('serials')!=expected_serials:raise ValueError('Capture hand serial mismatch')
    evidence={};samples=[];events=[];context_log=[]
    for rel in sorted(manifest['raw_files']):
        if Path(rel).suffix!='.mcap' or 'video' in Path(rel).parts:continue
        with (Path(root)/rel).open('rb') as f:
            for _,channel,record,msg in make_reader(f,decoder_factories=[DecoderFactory()]).iter_decoded_messages(topics=[*TOPICS,CONTEXT,EVENT]):
                if channel.topic in (CONTEXT,EVENT):
                    e=json.loads(msg.data)
                    if channel.topic==CONTEXT:
                        candidates=[e];context_log.append((record.log_time,msg.data))
                    else:
                        if e.get('capture_id')!=manifest['capture_id']:raise ValueError('Foreign capture event')
                        events.append(e.get('kind'));candidates=e.get('snapshot',{}).get('context_history',[])
                    for e in candidates:
                        key=(e.get('session_id'),e.get('seq'))
                        if key in evidence and evidence[key]!=e:raise ValueError('Conflicting hand evidence')
                        if e.get('session_id')!=manifest['source_session_id'] or e.get('serials')!=expected_serials:raise ValueError('Foreign hand session/device')
                        if e.get('clock_domain')!='robot_receive':raise ValueError('Hand evidence clock mismatch')
                        if not e.get('live') and not manifest['diagnostic']:raise ValueError('Fake hand evidence in production')
                        evidence[key]=e
                else:samples.append((channel.topic,int(record.log_time),msg))
    if not {'start','stop'}.issubset(events):raise ValueError('Missing capture boundary events')
    rows={(side,role):[] for side in ('left','right') for role in ('state','action')}
    seen=set()
    for topic,stamp,msg in samples:
        side,role=TOPICS[topic];identity=msg.header.frame_id.split('/')
        if len(identity)!=3 or identity[0]!='manus_sharpa':raise ValueError('Hand sample has no event identity')
        key=(identity[1],int(identity[2]));e=evidence.get(key)
        if not e or e.get('side')!=side or e.get('kind')!=('state' if role=='state' else 'accepted'):raise ValueError('Hand sample lacks matching evidence')
        if (topic,key) in seen:raise ValueError('Duplicate hand event in Raw')
        seen.add((topic,key))
        if list(msg.name)!=HAND_NAMES or e.get('names')!=HAND_NAMES:raise ValueError('Hand joint order mismatch')
        q=np.asarray(msg.position,dtype='f8')
        if q.shape!=(22,) or not np.isfinite(q).all() or q.tolist()!=e.get('position'):raise ValueError('Hand values differ from evidence')
        header=int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec)
        if header!=e.get('received_robot_ns') or not 0<=stamp-header<=500_000_000:raise ValueError('Hand receive/log clock or queue delay invalid')
        if role=='action' and e.get('sdk_result',{}).get('status')!='sdk_call_succeeded':raise ValueError('Unaccepted hand action')
        rows[side,role].append((stamp,int(e['source_stamp_ns']),header,int(key[1]),q))
    for key,values in rows.items():
        values.sort(key=lambda v:v[0])
        if len(values)<2 or np.any(np.diff([v[0] for v in values])<=0):raise ValueError('Missing/non-increasing hand stream '+str(key))
    return rows,context_log

def add_hands(h,root,manifest,serials):
    import h5py
    rows,context=hand_rows(root,manifest,serials)
    h.attrs.update(unified_profile=PROFILE,episode_index=manifest['episode_index'],episode_name=manifest['episode_name'],
                   capture_id=manifest['capture_id'],diagnostic=manifest['diagnostic'],capture_manifest_sha256=digest(Path(root)/'capture_manifest.json'))
    capture=h.create_group('capture');capture.attrs['manifest_json']=json.dumps(manifest)
    evidence=h.create_group('evidence/sharpa')
    evidence.create_dataset('log_timestamp_ns',data=np.array([t for t,_ in context],dtype='i8'))
    evidence.create_dataset('context_json',data=[e for _,e in context],dtype=h5py.string_dtype('utf-8'))
    bounds=[]
    for (side,role),v in rows.items():
        prefix='observations' if role=='state' else 'actions'
        g=h.create_group(f'{prefix}/{side}_hand')
        for name,col in [('timestamp_ns',0),('source_timestamp_ns',1),('receive_timestamp_ns',2),('seq',3)]:
            g.create_dataset(name,data=np.array([r[col] for r in v],dtype='i8'))
        g.create_dataset('value',data=np.array([r[4] for r in v],dtype='f8'))
        g.attrs.update(joint_names=json.dumps(HAND_NAMES),source_role=role,source_stage='measured_feedback' if role=='state' else 'sdk_call_succeeded',
                       unit='rad',kind='vector',coordinate_frame='native_joint_coordinates',clock_domain='robot_receive',
                       source_topic=f'/hand_{side}/joint_'+('states' if role=='state' else 'commands'),serial=serials[side],session_id=manifest['source_session_id'])
        bounds.append((v[0][0],v[-1][0]))
    return bounds

def check_hdf_identity(h,source_id,allow_diagnostic=False):
    if h.attrs.get('unified_profile')!=PROFILE or not h.attrs.get('complete'):raise ValueError('Certified unified HDF5 required')
    m=json.loads(h['capture'].attrs['manifest_json']);idx=validate_identity(m,source_id)
    if h.attrs.get('episode_index')!=idx or h.attrs.get('capture_id')!=m['capture_id']:raise ValueError('HDF5 episode/capture identity mismatch')
    if bool(h.attrs.get('diagnostic'))!=m['diagnostic']:raise ValueError('HDF5 diagnostic marker mismatch')
    if m['diagnostic'] and not allow_diagnostic:raise ValueError('Diagnostic HDF5 rejected for production')
    if h.attrs.get('clock_domain')!='robot_receive':raise ValueError('HDF5 clock mismatch')
    return m

def replay_payload(h,origin_ns,end_ns,manifest,native):
    if native is None:raise ValueError('Complete native IK replay data required')
    streams={}
    for key in ['observations/full_joints','observations/body_joints','actions/body_controller','actions/head_controller',
                'observations/chassis','actions/chassis',
                *[f'{role}/{side}_hand' for role in ('observations','actions') for side in ('left','right')]]:
        g=h[key];t=g['timestamp_ns'][:];v=g['value'][:]
        if len(t)<2 or np.any(np.diff(t)<=0) or not np.isfinite(v).all():raise ValueError('Invalid replay stream: '+key)
        before=np.flatnonzero(t<=origin_ns)
        if not len(before):raise ValueError('Replay initial sample unavailable: '+key)
        initial=int(before[-1])
        streams[key]={'timestamp_ns':t.tolist(),'offset_ns':(t-origin_ns).tolist(),'value':v.tolist(),
                      'unit':str(g.attrs['unit']),'source_topic':str(g.attrs['source_topic']),
                      'initial_source_index':initial,'initial_timestamp_ns':int(t[initial]),
                      'initial_value':v[initial].tolist()}
        for attr in ('joint_names','serial','session_id','source_stage'):
            if attr in g.attrs:streams[key][attr]=str(g.attrs[attr])
        if 'seq' in g:streams[key]['seq']=g['seq'][:].tolist()
    return {'schema':'tianji_unified_replay_v1','episode_index':manifest['episode_index'],'episode_name':manifest['episode_name'],
            'capture_id':manifest['capture_id'],'diagnostic':manifest['diagnostic'],'clock_domain':'robot_receive',
            'origin_ns':int(origin_ns),'end_ns':int(end_ns),'serials':manifest['serials'],'gripper_commands_enabled':False,
            'streams':streams,'native_ik_chassis':native,
            'ik_offset_ns':(np.asarray(native['ik_timestamp_ns'],dtype='i8')-origin_ns).tolist(),
            'execution_contract':'data_only; negative offsets retain pre-start evidence, not new commands; executor must validate before motion'}
