"""av-pipeline client: LeRobot action -> native Skye IK replay over SSH.

Default performs live read-only preflight. --execute enables motion only when
all endpoint checks pass. Hands/grippers are always excluded, masks unchanged.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import threading
import time
import numpy as np
import pyarrow.parquet as pq


def no_raw(event,args):
    if event=='open' and isinstance(args[0],(str,bytes)):
        p=Path(os.fsdecode(args[0]))
        if set(p.parts)&{'raw_data','row_data','apex_data','sharpa_data','gento_replay_bags','hdf5','apex_hdf5'} or p.suffix in ('.mcap','.db3','.h5','.hdf5'):
            raise PermissionError('Only LeRobot trajectory sources are allowed: '+str(p))


IK_POSES = ('left', 'right', 'left_elbow', 'right_elbow', 'torso', 'head',
            'pelvis', 'left_foot', 'right_foot', 'left_knee', 'right_knee')
IK_STREAMS = {'action.ik_' + key: 'actions/ik_' + key
              for key in (*IK_POSES, 'available', 'joint_positions')}


def recorded_ik_targets(info, timeline, cleaning, columns, decoder):
    """Decode independently recorded final IK inputs, without teleop offset fitting."""
    ts = np.asarray(timeline['timestamp_ns'], dtype=np.int64)
    source_times = []
    for key, source in IK_STREAMS.items():
        feature = info['features'][key]
        provenance = cleaning['extra_streams'][key]
        if (feature.get('source_role') != 'action' or feature.get('source_stage') != 'ik_input'
                or provenance.get('source_role') != 'action' or provenance.get('source_stage') != 'ik_input'
                or provenance.get('source_topic') != '/tj/control/ik_request'):
            raise ValueError('Unverified recorded IK source: ' + key)
        raw_stamps = timeline['aligned_sources'][source]['timestamp_ns']
        if not all(type(t) is int for t in raw_stamps):raise ValueError('Invalid IK timestamps')
        stamps = np.asarray(raw_stamps, dtype=np.int64)
        if stamps.shape != ts.shape or np.any(stamps > ts) or np.any(ts-stamps > 50_000_000):
            raise ValueError('Invalid IK source timing')
        if np.any(np.diff(stamps) < 0):raise ValueError('IK timestamps moved backwards')
        source_times.append(stamps)
    if any(not np.array_equal(source_times[0], x) for x in source_times[1:]):
        raise ValueError('IK poses and availability must come from the same message')
    poses = {}
    for side in IK_POSES:
        key = 'action.ik_' + side
        feature = info['features'][key]
        if feature.get('coordinate_frame') != 'base_link' or feature.get('unit') != 'm':
            raise ValueError('Recorded IK poses require base_link/meters')
        if feature.get('names') != ['x','y','z','r1x','r1y','r1z','r2x','r2y','r2z']:
            raise ValueError('Wrong recorded IK pose layout')
        poses[side] = decoder.pose_matrix(columns[key])
        if len(poses[side]) != len(ts):raise ValueError('IK pose length mismatch')
        if cleaning['extra_streams'][key].get('source_coordinate_frame') != 'tracking_base_link':
            raise ValueError('Unverified original IK frame')
    seed_feature = info['features']['action.ik_joint_positions']
    if (seed_feature.get('names') != ['position_' + str(i) for i in range(19)]
            or seed_feature.get('unit') != 'm,rad'
            or seed_feature.get('coordinate_frame') != 'native_joint_coordinates'):
        raise ValueError('Invalid recorded IK seed layout')
    seed = np.asarray(columns['action.ik_joint_positions'])
    if seed.shape != (len(ts), 19) or not np.isfinite(seed).all():
        raise ValueError('Invalid recorded IK seed')
    poses['joint_positions'] = seed
    availability = info['features']['action.ik_available']
    if availability.get('unit') != 'bool' or availability.get('names') != ['left_hand','right_hand','left_elbow','right_elbow','torso','head','pelvis','left_foot','right_foot','left_knee','right_knee']:
        raise ValueError('Wrong IK availability layout/units')
    gate = np.asarray(columns['action.ik_available'])
    if gate.shape != (len(ts), 11) or not np.isin(gate, [0, 1]).all():
        raise ValueError('Invalid recorded IK availability')
    return poses, gate.astype(bool), {
        'schema': 'skye_recorded_ik_targets_v3', 'input_frame': 'base_link',
        'output_frame': 'base_link', 'body_head_policy': 'recorded_ik',
        'pose_fields': list(IK_POSES), 'joint_state_names': [],
        'source_topic': '/tj/control/ik_request'}


def prepare(episode,execute):
    spec=importlib.util.spec_from_file_location('lerobot_decoder',Path(__file__).parents[1]/'lerobot_replay/main.py')
    decoder=importlib.util.module_from_spec(spec);spec.loader.exec_module(decoder)
    fingerprint=decoder.verify_episode(episode)
    read=lambda name:json.loads((episode/'meta'/name).read_text())
    table=pq.read_table(episode/'data/chunk-000/episode_000000.parquet')
    info = read('info.json')
    present = set(IK_STREAMS) & set(table.column_names)
    if present and present != set(IK_STREAMS):raise ValueError('Incomplete recorded IK streams')
    columns={k:np.array(table[k].to_pylist()) for k in ['action','observation.state','validity','frame_index','timestamp',*decoder.AUX,*sorted(present)]}
    targets=decoder.decode(read('info.json'),read('source_timestamps.json'),read('chunk_frames.json') if info['coordinate_frame'] == 'chunk_first_frame_camera' else None,read('cleaning_report.json'),columns)
    if present:
        poses, gate, contract = recorded_ik_targets(info, read('source_timestamps.json'),
                                                   read('cleaning_report.json'), columns, decoder)
        for side in ('left', 'right'):
            targets['action_'+side+'_base_matrix'] = poses[side]
    else:
        contract=read('replay_contract.json')
        if contract.get('schema')!='skye_recorded_ik_mapping_v1' or contract.get('mapping_formula')!='p_ik=p_action+c; R_ik=R_action@C' or contract.get('body_head_policy')!='recorded_controller_FK':raise ValueError('Unverified replay contract')
        if contract['timestamp_ns']!=targets['timestamp_ns'].tolist():raise ValueError('Replay availability timeline mismatch')
        gate=np.array(contract['available'])
        if gate.shape!=(len(table),11) or gate.dtype!=bool:raise ValueError('Invalid IK availability')
        if np.any(np.array(contract['ik_log_timestamp_ns'])>targets['timestamp_ns']) or np.max(targets['timestamp_ns']-np.array(contract['ik_log_timestamp_ns']))>50_000_000:raise ValueError('Invalid control gate timestamps')
        for side in ('left','right'):
            c=np.asarray(contract['mapping'][side]['translation_offset'],float)
            rotation=np.asarray(contract['mapping'][side]['right_rotation_offset'],float)
            decoder.rotation(rotation)
            if c.shape!=(3,) or not np.isfinite(c).all():raise ValueError('Invalid mapping offset')
            t=targets['action_'+side+'_base_matrix']
            t[:,:3,3]+=c;t[:,:3,:3]=t[:,:3,:3]@rotation
    if decoder.verify_episode(episode)!=fingerprint:raise ValueError('LeRobot changed during loading')
    # Legacy poses use recorded mapping; new episodes carry final IK action streams.
    # Body/head use independently recorded controller actions, not masked body78.
    return {'schema':'skye_lerobot_client_v1','source':str(episode),'source_sha256':fingerprint,
        'offset_ns':targets['offset_ns'].tolist(),'left':targets['action_left_base_matrix'].tolist(),
        'right':targets['action_right_base_matrix'].tolist(),'body':targets['action_body_controller'].tolist(),
        'head':targets['action_head_controller'].tolist(),'ignored_slots':list(range(32,78)),
        'execute':execute,'available':gate.tolist(),'replay_contract':contract,
        'execution_target_policy':'robot_controller_fk_inverse_support_v1' if present else 'legacy',
        'ik_reference': {k:v.tolist() for k,v in poses.items()} if present else {},
        'validation_trajectory': {'left':targets['action_left_base_matrix'].tolist(),
            'right':targets['action_right_base_matrix'].tolist(), 'body':targets['action_body_controller'].tolist(),
            'available':gate.tolist()} if present else {},
        'excluded_channels':['hands','grippers','chassis'],
        'head_policy':'recorded_head_controller_only'}


def prepare_native(episode, execute):
    """Decode full-rate auxiliary LeRobot controls without opening raw/HDF5."""
    spec=importlib.util.spec_from_file_location('decoder',Path(__file__).parents[1]/'lerobot_replay/main.py')
    decoder=importlib.util.module_from_spec(spec);spec.loader.exec_module(decoder)
    fingerprint=decoder.verify_episode(episode)
    info=json.loads((episode/'meta/info.json').read_text())
    if info['features']['action']['shape'] != [78]:raise ValueError('Require 78D LeRobot')
    data=json.loads((episode/'meta/source_timestamps.json').read_text())['native_replay']
    if data['schema']!='skye_native_replay_v1' or data['coordinate_frame']!='base_link':raise ValueError('Unverified native replay schema/frame')
    ts=np.asarray(data['ik_timestamp_ns'],dtype=np.int64)
    ct=np.asarray(data['chassis_timestamp_ns'],dtype=np.int64)
    for times in (ts,ct):
        if len(times)<2 or np.any(np.diff(times)<=0) or np.max(np.diff(times))>100_000_000:raise ValueError('Invalid control timestamps')
    refs={k:decoder.pose_matrix(np.asarray(data['ik'][k],float)).tolist() for k in IK_POSES}
    for k in IK_POSES:
        if len(refs[k])!=len(ts):raise ValueError('Incomplete IK reference')
    seed=np.asarray(data['ik']['joint_positions'],float);gate=np.asarray(data['ik']['available'])
    cv=np.asarray(data['chassis_velocity'],float)
    if seed.shape!=(len(ts),19) or not np.isfinite(seed).all():raise ValueError('Invalid IK seed')
    if gate.shape!=(len(ts),11) or not np.isin(gate,[0,1]).all():raise ValueError('Invalid availability')
    if data['chassis_frame']!='base_link' or data['chassis_units']!=['m/s','m/s','rad/s']:raise ValueError('Invalid chassis frame/units')
    if cv.shape!=(len(ct),3) or not np.isfinite(cv).all() or np.max(np.abs(cv[:,0]))>.400001 or np.max(np.abs(cv[:,1]))>1e-8 or np.max(np.abs(cv[:,2]))>.500001 or np.max(np.abs(cv[-1]))>1e-8:raise ValueError('Invalid chassis commands')
    refs['joint_positions']=seed.tolist()
    origin=int(min(ts[0],ct[0]))
    if decoder.verify_episode(episode)!=fingerprint:raise ValueError('LeRobot changed during load')
    return dict(schema='skye_native_lerobot_v1',source=str(episode),source_sha256=fingerprint,
                execute=execute,offset_ns=(ts-origin).tolist(),ik_reference=refs,available=gate.astype(bool).tolist(),
                chassis_offset_ns=(ct-origin).tolist(),chassis_velocity=cv.tolist(),
                initial_joint_names=data['initial_joint_names'],initial_joint_positions=data['initial_joint_positions'],
                raw_or_hdf5_read=False)


def run_native(plan, plan_only=False):
    if plan_only:
        print(json.dumps(plan));return
    import shlex
    remote='/home/user/codex-robot-test/test_native_ik_replay.py'
    local=Path(__file__).parents[2]/'deployment/test_native_ik_replay.py'
    expected=hashlib.sha256(local.read_bytes()).hexdigest()
    actual=subprocess.check_output(['ssh','user@6.6.7.100','sha256sum',remote],text=True).split()[0]
    if actual!=expected:raise RuntimeError('Native endpoint hash mismatch')
    command='source /etc/apex/apex_ros_env.sh >/dev/null 2>&1; export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; exec python3 -B '+remote+' --lerobot-plan --with-chassis'+(' --execute' if plan['execute'] else '')
    proc=subprocess.Popen(['ssh','user@6.6.7.100','bash -c '+shlex.quote(command)],stdin=subprocess.PIPE,text=True)
    proc.stdin.write(json.dumps(plan)+'\n');proc.stdin.flush()
    def heartbeat():
        while proc.poll() is None:
            try:proc.stdin.write('heartbeat\n');proc.stdin.flush()
            except (BrokenPipeError,OSError):break
            time.sleep(.1)
    threading.Thread(target=heartbeat,daemon=True).start()
    rc=proc.wait()
    if rc:raise SystemExit(rc)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',required=True);p.add_argument('--episode',type=int,required=True)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--native-recorded',action='store_true',help='Replay complete original IK semantics and chassis from LeRobot auxiliary streams')
    p.add_argument('--plan-only',action='store_true')
    p.add_argument('--test-duration',type=float,choices=(2.0,),help='Two-second native LeRobot prefix test')
    p.add_argument('--max-frames',type=int,help='Replay only this many initial frames, minimum 2')
    p.add_argument('--test-home-feedback-tolerance-rad',type=float,choices=(0.01,0.015),default=0.01,help='Temporary home feedback tolerance. Default remains 0.01 rad.')
    p.add_argument('--full-trajectory-test',action='store_true',help='Explicitly allow the reviewed 314-frame test with the temporary 0.015 rad home tolerance')
    args=p.parse_args()
    if args.full_trajectory_test and args.max_frames is not None:p.error('Full trajectory test cannot be truncated')
    if args.full_trajectory_test and args.test_home_feedback_tolerance_rad!=0.015:p.error('Full trajectory test requires the explicit 0.015 rad test tolerance')
    if args.test_home_feedback_tolerance_rad!=0.01 and not args.full_trajectory_test and (args.max_frames is None or not 2<=args.max_frames<=6):p.error('Test tolerance requires --max-frames 2..6 or --full-trajectory-test')
    if not re.fullmatch(r'[\w-]+',args.task) or not 0<=args.episode<=99999999:raise ValueError('Invalid task/episode')
    sys.addaudithook(no_raw)
    mount=Path('/mnt/apex-data')
    if not mount.is_mount():raise ValueError('WD disk not mounted')
    device=subprocess.check_output(['findmnt','-rn','-M',str(mount),'-o','SOURCE'],text=True).strip()
    if 'WD-WX12A367D1A2' not in subprocess.check_output(['lsblk','-sn','-o','SERIAL',device],text=True).split():raise ValueError('Wrong disk')
    root=mount/'apex_n_sharpa/data';episode=root/'lerobot'/args.task/f'episode_{args.episode:08d}'
    if episode.resolve()!=episode:raise ValueError('Dataset symlinks forbidden')
    if args.native_recorded:
        if args.max_frames or args.full_trajectory_test:raise ValueError("Native full-rate mode cannot use training-frame truncation")
        plan=prepare_native(episode,args.execute)
        if args.test_duration:
            end=int(args.test_duration*1e9)
            n=sum(t<end for t in plan['offset_ns']);m=sum(t<end for t in plan['chassis_offset_ns'])
            plan['offset_ns']=plan['offset_ns'][:n];plan['available']=plan['available'][:n]
            plan['ik_reference']={k:v[:n] for k,v in plan['ik_reference'].items()}
            plan['chassis_offset_ns']=plan['chassis_offset_ns'][:m]+[end]
            plan['chassis_velocity']=plan['chassis_velocity'][:m]+[[0.,0.,0.]]
            plan['test_duration_s']=args.test_duration
        return run_native(plan,args.plan_only)
    if args.plan_only:raise ValueError("--plan-only requires --native-recorded")
    plan=prepare(episode,args.execute)
    plan['test_home_feedback_tolerance_rad']=args.test_home_feedback_tolerance_rad
    plan['full_trajectory_test']=args.full_trajectory_test
    if args.full_trajectory_test and len(plan['offset_ns'])!=314:p.error('Reviewed full trajectory must contain exactly 314 frames')
    if args.max_frames is not None:
        if not 2<=args.max_frames<=len(plan['offset_ns']):raise ValueError('Invalid frame limit')
        plan['source_total_frames']=len(plan['offset_ns'])
        for key in ('offset_ns','left','right','body','head','available'):
            plan[key]=plan[key][:args.max_frames]
        plan['ik_reference'] = {k:v[:args.max_frames] for k,v in plan['ik_reference'].items()}
        plan['selected_frame_range']=[0,args.max_frames]

    spec=importlib.util.spec_from_file_location('endpoint',Path(__file__).with_name('robot_endpoint.py'))
    endpoint=importlib.util.module_from_spec(spec);spec.loader.exec_module(endpoint);endpoint.validate_plan(plan)
    out=root/'replay_runs'/args.task/f'episode_{args.episode:08d}'/str(time.time_ns())
    if out.parent.resolve()!=out.parent:raise ValueError('Output symlinks forbidden')
    out.mkdir(parents=True)
    (out/'plan.json').write_text(json.dumps(plan)+'\n')
    # Robot-side code is deployed separately. Pin its contents before any execution.
    expected=hashlib.sha256(Path(__file__).with_name('robot_endpoint.py').read_bytes()).hexdigest()
    remote='/home/user/codex-robot-test/lerobot_endpoint.py'
    actual=subprocess.check_output(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','user@6.6.7.100','sha256sum',remote],text=True).split()[0]
    if actual!=expected:raise RuntimeError('Robot endpoint code differs; redeploy reviewed endpoint')
    command='export ROS_LOG_DIR=/tmp/lerobot-client-logs; source /etc/apex/apex_ros_env.sh; export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; exec python3 -u '+remote
    import shlex
    proc=subprocess.Popen(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','user@6.6.7.100','bash -c '+shlex.quote(command)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    stopped=threading.Event();errors=[]
    def feed():
        try:
            proc.stdin.write(json.dumps(plan)+'\n');proc.stdin.flush()
            while not stopped.wait(.1):proc.stdin.write('heartbeat\n');proc.stdin.flush()
        except (BrokenPipeError,OSError):pass
    def stderr():
        for line in proc.stderr:errors.append(line)
    sender=threading.Thread(target=feed,daemon=True);sender.start()
    threading.Thread(target=stderr,daemon=True).start()
    result=None
    try:
        for line in proc.stdout:
            if line.startswith('RESULT_JSON='):
                result=json.loads(line.split('=',1)[1]);(out/'report.json').write_text(json.dumps(result,indent=2)+'\n')
            else:print(line,end='',flush=True)
        rc=proc.wait(timeout=5)
    finally:
        stopped.set();sender.join(timeout=2)
        try:proc.stdin.close()
        except BrokenPipeError:pass
        if proc.poll() is None:
            # EOF is an endpoint stop signal; allow its finally path to restore mux.
            try:proc.wait(timeout=8)
            except subprocess.TimeoutExpired:proc.terminate()
        (out/'stderr.log').write_text(''.join(errors))
    print(json.dumps({'output':str(out),'result':{k:v for k,v in (result or {}).items() if k not in ('samples','home_samples')}},indent=2))
    if result is None or rc:raise SystemExit(rc or 1)


if __name__=='__main__':main()
