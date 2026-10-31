"""Offline contract, causal scheduling, isolation and protocol regression tests."""
import copy
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from unified_replay import contract as c
from unified_replay.simulator import simulate


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


def seal(root):
    completion=root/'meta/completion.json'
    save(completion,{str(p.relative_to(root)):c.file_hash(p) for p in root.rglob('*') if p.is_file() and p!=completion})


def fixture(root):
    root=root/'episode_00000042';root.mkdir()
    origin=1_000_000_000;end=1_200_000_000
    shared=dict(capture_id='a'*32,episode_index=42,episode_name='episode_00000042',diagnostic=True,
                clock_domain='robot_receive',serials=c.SERIALS)
    capture=dict(shared,state='COMPLETE',audit={'issues':[]},source_session_id='b'*32,
                 start_ack_ns=origin-100_000_000,stop_request_ns=end+100_000_000)
    times=list(range(origin-50_000_000,end+75_000_000,25_000_000))
    streams={}
    specs={'observations/full_joints':(19,'m,rad',c.JOINT_NAMES),
           'observations/body_joints':(5,'m,rad,rad,rad,rad',c.BODY_NAMES),
           'actions/body_controller':(3,'m,rad,rad',c.BODY_NAMES[:3]),
           'actions/head_controller':(2,'rad,rad',c.BODY_NAMES[3:]),
           'observations/chassis':(3,'m/s,m/s,rad/s',None),
           'actions/chassis':(3,'m/s,m/s,rad/s',None)}
    specs.update({f'{role}/{side}_hand':(22,'rad',c.HAND_NAMES)
        for role in ('observations','actions') for side in ('left','right')})
    topics={'observations/full_joints':'/tj/joint_states','observations/body_joints':'/tj/joint_states',
            'actions/body_controller':'/tj/control/joint_cmd_body','actions/head_controller':'/tj/control/joint_cmd_head',
            'observations/chassis':'/recording/chassis/state','actions/chassis':'/recording/chassis/manual_move_cmd'}
    topics.update({f'{role}/{side}_hand':f'/hand_{side}/joint_'+('commands' if role=='actions' else 'states') for role in ('observations','actions') for side in ('left','right')})
    for key,(width,unit,names) in specs.items():
        st=times
        if key.endswith('left_hand'):st=list(range(origin-10_000_000,end+31_000_000,10_000_000))
        if key.endswith('right_hand'):st=list(range(origin-15_000_000,end+31_000_000,15_000_000))
        v=[[0.0]*width for _ in st]
        if 'hand' in key:v=[[.01+i*.001]*width for i in range(len(st))]
        initial=max(i for i,t in enumerate(st) if t<=origin)
        s=dict(timestamp_ns=st,offset_ns=[t-origin for t in st],value=v,unit=unit,
               initial_source_index=initial,initial_timestamp_ns=st[initial],initial_value=v[initial],source_topic=topics[key])
        if names:s['joint_names']=json.dumps(names)
        if 'hand' in key:
            side=key.split('/')[1].split('_')[0]
            s.update(serial=c.SERIALS[side],session_id='b'*32,seq=list(range(len(st))),
                     source_stage='sdk_call_succeeded' if key.startswith('actions/') else 'measured_feedback')
        streams[key]=s
    ik={name:[[0,0,0,1,0,0,0,1,0] for _ in times] for name in c.IK_POSES}
    ik.update(available=[[True]*11 for _ in times],joint_positions=[[0]*19 for _ in times])
    native=dict(schema='skye_native_replay_v1',coordinate_frame='base_link',ik_timestamp_ns=times,ik=ik,
                chassis_timestamp_ns=times,chassis_velocity=streams['actions/chassis']['value'],
                chassis_frame='base_link',chassis_units=['m/s','m/s','rad/s'],initial_joint_names=c.JOINT_NAMES,initial_joint_positions=[0]*19)
    replay=dict(shared,schema='tianji_unified_replay_v1',origin_ns=origin,end_ns=end,gripper_commands_enabled=False,
                streams=streams,native_ik_chassis=native,ik_offset_ns=[t-origin for t in times])
    mask=[i not in (54,77) for i in range(78)]
    names=[f'pose_{i}' for i in range(27)]+c.BODY_NAMES+['left_hand_'+n for n in c.HAND_NAMES]+['left_gripper_mm']+['right_hand_'+n for n in c.HAND_NAMES]+['right_gripper_mm']
    info=dict(shared,source_episode_index=42,episode_ids=[42],profile='tianji_unified78',codebase_version='v2.1',
              coordinate_frame='base_link',fps=20,total_episodes=1,encoded_action_dim=78,total_frames=5,
              state_mask=mask,action_mask=mask,features={k:{'shape':[78],'names':names} for k in ('observation.state','action','validity')})
    timeline=dict(timestamp_ns=list(range(origin,end+1,50_000_000)),source_episode_index=42,capture_id='a'*32,native_replay=native)
    for name,value in [('info',info),('capture',capture),('unified_replay',replay),('source_timestamps',timeline)]:save(root/'meta'/f'{name}.json',value)
    a=np.zeros((5,78));a[:,[3,7,12,16,21,25]]=1
    table={'episode_index':[42]*5,'frame_index':list(range(5)),'timestamp':[i/20 for i in range(5)],
           'observation.state':a.tolist(),'action':a.tolist(),'validity':[mask]*5}
    timeline['aligned_sources']={}
    mapping=[('observations/body_joints','observations/body_joints','observation.state',27,5),
             ('actions/body_controller','actions/body_controller','action',27,3),
             ('actions/head_controller','actions/head_controller','action',30,2)]
    mapping += [(f'{role}/{side}_hand',f'{alias}/{side}',column,start,22)
                for role,alias,column in [('observations','state','observation.state'),('actions','action','action')]
                for side,start in [('left',32),('right',55)]]
    for stream,aligned,column,start,width in mapping:
        source=streams[stream]
        ix=[max(i for i,t in enumerate(source['timestamp_ns']) if t<=grid) for grid in timeline['timestamp_ns']]
        timeline['aligned_sources'][aligned]={'source_index':ix,'timestamp_ns':[source['timestamp_ns'][i] for i in ix]}
        # Independent rows: don't alias state and action arrays.
        table[column]=copy.deepcopy(table[column])
        for row,i in enumerate(ix):table[column][row][start:start+width]=source['value'][i]
    save(root/'meta/source_timestamps.json',timeline)
    p=root/'data/chunk-000/episode_000042.parquet';p.parent.mkdir(parents=True);pq.write_table(pa.table(table),p)
    seal(root)
    return root


def mutate(root,name,fn):
    p=root/'meta'/f'{name}.json';v=json.loads(p.read_text());fn(v);save(p,v);seal(root)


def test_offline_causal_plan_keeps_rate_and_id(tmp_path):
    root=fixture(tmp_path);p=c.build_plan(root,42,True);c.verify_plan(p)
    assert p['episode_index']==42 and p['state_action_dimension']==78
    assert len({len(v) for v in p['channels'].values()})>1
    for name,events in p['channels'].items():
        assert events[0]['offset_ns']==0 and events[0]['source_timestamp_ns']<=p['origin_ns']
        assert all(e['offset_ns']>0 and e['source_timestamp_ns']==p['origin_ns']+e['offset_ns'] for e in events[1:])
        assert len({e['source_index'] for e in events})==len(events)
    assert p['hardware_execution_allowed'] is False
    assert p['action_mask'][54] is False and p['action_mask'][77] is False
    assert c.validate_start_feedback(p,p['initial_feedback'])


def test_default_diagnostic_rejection(tmp_path):
    with pytest.raises(ValueError,match='Diagnostic'):c.build_plan(fixture(tmp_path))


def test_episode_selection_mismatch(tmp_path):
    with pytest.raises(ValueError,match='Selected episode'):c.build_plan(fixture(tmp_path),3,True)


def test_completion_tamper_and_symlink(tmp_path):
    root=fixture(tmp_path);(root/'extra.txt').write_text('unexpected')
    with pytest.raises(ValueError,match='completion'):c.build_plan(root,42,True)
    (root/'extra.txt').unlink();(root/'link').symlink_to(root/'meta/info.json')
    with pytest.raises(ValueError,match='symlink'):c.build_plan(root,42,True)


@pytest.mark.parametrize('fault', ['capture','clock','mask','dimension','gripper','initial','hand_order','serial',
                                  'sdk_receipt','hand_limits','sequence','offset','native_disagreement','rotation','window','nan'])
def test_resigned_semantic_corruption_rejected(tmp_path,fault):
    root=fixture(tmp_path)
    def modify(r):
        if fault=='capture':r['capture_id']='c'*32
        elif fault=='clock':r['clock_domain']='av_source'
        elif fault=='gripper':r['gripper_commands_enabled']=True
        elif fault=='initial':r['streams']['actions/left_hand']['initial_source_index']=0
        elif fault=='hand_order':r['streams']['actions/left_hand']['joint_names']=json.dumps(list(reversed(c.HAND_NAMES)))
        elif fault=='serial':r['streams']['actions/left_hand']['serial']='wrong'
        elif fault=='sdk_receipt':r['streams']['actions/left_hand']['source_stage']='requested'
        elif fault=='hand_limits':r['streams']['actions/left_hand']['value'][-1][0]=10
        elif fault=='sequence':r['streams']['actions/left_hand']['seq'][2]=0
        elif fault=='offset':r['streams']['actions/right_hand']['offset_ns'][2]+=1
        elif fault=='native_disagreement':r['native_ik_chassis']['ik']['joint_positions'][0][0]=.01
        elif fault=='rotation':r['native_ik_chassis']['ik']['left'][0][3]=0
        elif fault=='window':r['end_ns']+=50_000_000
        elif fault=='nan':r['streams']['actions/head_controller']['value'][1][0]=float('nan')
    if fault=='mask':mutate(root,'info',lambda i:i['action_mask'].__setitem__(55,False))
    elif fault=='dimension':mutate(root,'info',lambda i:i['features']['action'].__setitem__('shape',[76]))
    else:mutate(root,'unified_replay',modify)
    with pytest.raises((ValueError,KeyError)):c.build_plan(root,42,True)


def test_parquet_mask_not_just_metadata(tmp_path):
    root=fixture(tmp_path);p=root/'data/chunk-000/episode_000042.parquet';d=pq.read_table(p).to_pydict();d['validity'][0][55]=False;pq.write_table(pa.table(d),p);seal(root)
    with pytest.raises(ValueError,match='Per-frame mask'):c.build_plan(root,42,True)


def test_plan_mutation_rejected(tmp_path):
    p=c.build_plan(fixture(tmp_path),42,True);p['channels']['left_hand'][0]['payload'][0]=2
    with pytest.raises(ValueError,match='hash'):c.verify_plan(p)


def test_start_pose_is_not_silently_moved(tmp_path):
    p=c.build_plan(fixture(tmp_path),42,True);q=copy.deepcopy(p['initial_feedback']);q['body'][14]=.1
    with pytest.raises(ValueError,match='initial_pose'):c.validate_start_feedback(p,q)


def test_execute_rejected_before_output_or_inputs(tmp_path):
    out=tmp_path/'output'
    r=subprocess.run([sys.executable,str(ROOT/'src/unified_replay/main.py'),'--task','none','--episode','3','--execute','--output-dir',str(out)],capture_output=True,text=True)
    assert r.returncode==2 and 'Hardware execution' in r.stderr and not out.exists()


def test_isolation_blocks_raw_network_and_hardware():
    script='''from unified_replay.isolation import install
import socket
install()
for op in [lambda:open('/tmp/a.mcap','wb'), lambda:open('/dev/ttyUSB0','rb'), lambda:socket.socket().connect(('127.0.0.1',9))]:
    try:op()
    except PermissionError:pass
    else:raise AssertionError('Isolation bypass')
'''
    env=dict(os.environ,PYTHONPATH=str(ROOT/'src'))
    r=subprocess.run([sys.executable,'-c',script],env=env,capture_output=True,text=True)
    assert r.returncode==0,r.stderr


def longer_plan(root):
    p=c.build_plan(root,42,True)
    # Extend a mock-only fixture without changing the source LeRobot data.
    p['duration_ns']=600_000_000;p['end_ns']=p['origin_ns']+p['duration_ns']
    for name,events in p['channels'].items():
        step={'body':25_000_000,'chassis':20_000_000,'left_hand':10_000_000,'right_hand':15_000_000}[name]
        prototype=copy.deepcopy(events[0]);p['channels'][name]=[]
        for i,t in enumerate(range(0,p['duration_ns']+1,step)):
            e=copy.deepcopy(prototype);e.update(offset_ns=t,source_timestamp_ns=p['origin_ns']+t,source_index=i,initial=i==0)
            p['channels'][name].append(e)
    p['plan_sha256']=c.fingerprint({k:v for k,v in p.items() if k!='plan_sha256'})
    return p


@pytest.mark.parametrize('scenario',['normal','right_sdk_reject','clock_jump','coordinator_loss','body_process_exit'])
def test_subprocess_stop_protocol(tmp_path,scenario):
    result=simulate(longer_plan(fixture(tmp_path)),scenario)
    assert result['experiment_passed'],{k:v for k,v in result.items() if k!='executors'}
    assert result['hardware_commands_sent']==0
    assert result['commands_after_local_stop']==0
    if scenario=='normal':
        assert len({s['process_id'] for s in result['executors'].values()})==4
        assert result['payload_order_verified']
    if scenario=='body_process_exit':
        assert result['status']=='STOP_UNCONFIRMED' and result['missing_stop_ack']==['body']
