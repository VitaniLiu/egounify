"""Export reversible episode-first increments from verified LeRobot only.

Delta position is expressed in base_link; delta rotation left-multiplies the
anchor rotation. This is NOT a quaternion difference or a full SE(3) quotient.
No robot IO. Native Apex incremental-wire compatibility is not presumed.
"""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import importlib.util
import numpy as np
import pyarrow.parquet as pq
_spec=importlib.util.spec_from_file_location('incremental_lerobot_decoder',Path(__file__).with_name('main.py'))
decoder=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(decoder)


def pose_delta(poses):
    poses=np.asarray(poses,dtype=np.float64)
    if poses.ndim!=3 or poses.shape[1:]!=(4,4) or not len(poses) or not np.isfinite(poses).all():
        raise ValueError('Invalid poses')
    decoder.rotation(poses[:,:3,:3])
    if not np.allclose(poses[:,3],[0,0,0,1],atol=1e-9,rtol=0):raise ValueError('Invalid pose last row')
    anchor=poses[0].copy()
    delta=np.repeat(np.eye(4)[None],len(poses),axis=0)
    delta[:,:3,3]=poses[:,:3,3]-anchor[:3,3]
    delta[:,:3,:3]=poses[:,:3,:3]@anchor[:3,:3].T
    return delta,anchor


def restore_pose(delta,anchor):
    delta=np.asarray(delta,float);anchor=np.asarray(anchor,float)
    result=delta.copy()
    result[:,:3,3]=anchor[:3,3]+delta[:,:3,3]
    result[:,:3,:3]=delta[:,:3,:3]@anchor[:3,:3]
    return result


def convert(targets,mask):
    n=len(targets['timestamp_ns'])
    mask=np.asarray(mask)
    if mask.shape!=(n,78) or not np.isin(mask,[0,1]).all() or not mask[:,:27].all() or mask[:,27:].any():raise ValueError('Unsupported validity mask')
    out={'timestamp_ns':targets['timestamp_ns'].copy(),'offset_ns':targets['offset_ns'].copy(),
         'state_mask':mask.astype(bool),'action_mask':mask.astype(bool)}
    errors={}
    for role in ('action','state'):
        vectors=np.zeros((n,78),np.float64);anchors=[]
        for side,k in [('left',0),('right',9),('head',18)]:
            key=f'{role}_{side}_base_matrix'
            delta,anchor=pose_delta(targets[key]);anchors.append(anchor)
            vectors[:,k:k+9]=np.concatenate([delta[:,:3,3],delta[:,:3,0],delta[:,:3,1]],axis=1)
            out[f'{role}_{side}_delta_matrix']=delta
            errors[key]=float(np.max(np.abs(restore_pose(delta,anchor)-targets[key])))
        out[role+'_delta78']=vectors;out[role+'_anchor_base_matrix']=np.array(anchors)
    for key in ('action_body_controller','action_head_controller','observation_body_joints'):
        values=targets[key];anchor=values[0].copy();delta=values-anchor
        out[key+'_delta']=delta;out[key+'_anchor']=anchor
        if key+'_source_timestamp_ns' in targets:out[key+'_source_timestamp_ns']=targets[key+'_source_timestamp_ns'].copy()
        errors[key]=float(np.max(np.abs(delta+anchor-values)))
    if max(errors.values())>2e-5:raise ValueError('Incremental round-trip validation failed')
    return out,errors


def guard(event,args):
    if event=='open' and isinstance(args[0],(str,bytes)):
        path=Path(os.fsdecode(args[0]))
        if set(path.parts)&{'raw_data','row_data','apex_data','sharpa_data','gento_replay_bags','hdf5','apex_hdf5'} or path.suffix in ('.mcap','.db3','.h5','.hdf5'):
            raise PermissionError('Non-LeRobot trajectory input forbidden: '+str(path))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task',required=True);parser.add_argument('--episode',type=int,required=True)
    args=parser.parse_args()
    if not re.fullmatch(r'[\w-]+',args.task) or not 0<=args.episode<=99999999:raise ValueError('Invalid task/episode')
    sys.addaudithook(guard)
    mount=Path('/mnt/apex-data')
    if not mount.is_mount():raise ValueError('WD disk not mounted')
    device=subprocess.check_output(['findmnt','-rn','-M',str(mount),'-o','SOURCE'],text=True).strip()
    if 'WD-WX12A367D1A2' not in subprocess.check_output(['lsblk','-sn','-o','SERIAL',device],text=True).split():raise ValueError('Wrong storage device')
    root=mount/'apex_n_sharpa/data';source=root/'lerobot'/args.task/f'episode_{args.episode:08d}'
    if source.resolve()!=source:raise ValueError('Input symlink forbidden')
    fingerprint=decoder.verify_episode(source)
    read=lambda name:json.loads((source/'meta'/name).read_text())
    table=pq.read_table(source/'data/chunk-000/episode_000000.parquet')
    columns={k:np.array(table[k].to_pylist()) for k in ['action','observation.state','validity','frame_index','timestamp',*decoder.AUX]}
    targets=decoder.decode(read('info.json'),read('source_timestamps.json'),read('chunk_frames.json'),read('cleaning_report.json'),columns)
    output,errors=convert(targets,columns['validity'])
    out=root/'replay_incremental'/args.task/f'episode_{args.episode:08d}'
    if out.parent.resolve()!=out.parent or out.exists():raise ValueError('Output exists or symlink; refusing overwrite')
    out.parent.mkdir(parents=True,exist_ok=True);partial=out.with_name('.'+out.name+'.partial')
    partial.mkdir()
    try:
        np.savez_compressed(partial/'targets.npz',**output)
        # Verify the actual serialized artifact rather than just in-memory arrays.
        with np.load(partial/'targets.npz',allow_pickle=False) as saved:
            if set(saved.files)!=set(output) or any(not np.array_equal(saved[k],v) for k,v in output.items()):raise ValueError('Saved output differs')
        report={'schema':'skye_episode_first_incremental_targets_v1','source':str(source),
            'source_completion_sha256':fingerprint,'frames':len(table),'fps':20,
            'duration_s':int(targets['offset_ns'][-1])/1e9,'anchor_scope':'episode_first_frame; never reset at camera chunk boundaries',
            'position_delta':'p(t)-p(0), expressed in base_link',
            'rotation_delta':'R(t) @ R(0).T, first two matrix columns',
            'restore':'p(t)=p(0)+dp(t); R(t)=dR(t)@R(0)',
            'matrix_warning':'delta_matrix combines spatial position difference and rotation delta; do not multiply it as a full SE(3) quotient',
            'scalar_delta':'q(t)-q(0), original m/rad units; no wrap or unit conversion',
            'state_action_separation':'each role has its own first-frame anchor; state is diagnostic, never substituted for action',
            'first_frame':'zero translation / identity rotation / zero joint delta; rot6 identity is [1,0,0,0,1,0], not six zeros',
            'valid_78_indices':list(range(27)),'masked_78_indices':list(range(27,78)),
            'missing_hand_gripper_indices':list(range(32,78)),
            'hand_gripper_policy':'zero placeholders with false mask; never actuator commands',
            'head_execution_policy':'client may choose head_controller delta OR Cartesian head delta, never both',
            'body_joint_names':decoder.BODY,'head_joint_names':decoder.HEAD,
            'raw_or_hdf5_read':False,'roundtrip_max_abs_error':max(errors.values()),'roundtrip_errors':errors,
            'robot_commands_sent':0,'native_apex_incremental_wire_verified':False,'execution_ready':False,
            'artifact_type':'replay target NPZ with provenance, not a replacement standard LeRobot dataset',
            'excluded':'chassis; velocity commands are not pose increments',
            'limitations':['Incremental data does not prove native controller anchor/composition compatibility',
                'Reanchoring on live robot changes absolute path; original anchors preserve original data for validation']}
        (partial/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        if decoder.verify_episode(source)!=fingerprint:raise ValueError('Source changed during conversion')
        (partial/'completion.json').write_text(json.dumps({p.name:decoder.digest(p) for p in partial.iterdir()},indent=2)+'\n')
        partial.rename(out)
    except BaseException:
        shutil.rmtree(partial);raise
    print(json.dumps({'output':str(out),'frames':len(table),'roundtrip_max_abs_error':max(errors.values()),'robot_commands_sent':0},indent=2))


if __name__=='__main__':main()
