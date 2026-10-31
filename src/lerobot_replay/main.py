"""Strict LeRobot -> Skye Cartesian replay plan. No ROS, robot IO or IK guesses."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import numpy as np
import pyarrow.parquet as pq

BODY = ['waist_joint_1', 'waist_joint_2', 'waist_joint_3']
HEAD = ['Head_J1_Joint', 'Head_J2_Joint']
AUX = {
    'action.body_controller': ('actions/body_controller', BODY, 'm,rad,rad', 'action'),
    'action.head_controller': ('actions/head_controller', HEAD, 'rad,rad', 'action'),
    'observation.body_joints': ('observations/body_joints', BODY + HEAD, 'm,rad,rad,rad,rad', 'state'),
}


def require(ok, message):
    if not ok:
        raise ValueError(message)


def rotation(matrix):
    r = np.asarray(matrix, dtype=np.float64)
    require(r.shape[-2:] == (3, 3) and np.isfinite(r).all(), 'Invalid rotation shape/values')
    require(np.allclose(np.swapaxes(r, -1, -2) @ r, np.eye(3), atol=2e-5, rtol=0)
            and np.allclose(np.linalg.det(r), 1, atol=2e-5, rtol=0), 'Non-rigid rotation')
    # Remove float32 roundoff only, never repair a degenerate predicted rotation.
    u, _, vt = np.linalg.svd(r)
    return u @ vt


def pose_matrix(values):
    v = np.asarray(values, dtype=np.float64)
    require(v.ndim == 2 and v.shape[1] == 9 and np.isfinite(v).all(), 'Invalid XYZ+rot6 pose')
    r = np.stack((v[:, 3:6], v[:, 6:9], np.cross(v[:, 3:6], v[:, 6:9])), axis=-1)
    t = np.repeat(np.eye(4)[None], len(v), axis=0)
    t[:, :3, :3], t[:, :3, 3] = rotation(r), v[:, :3]
    return t


def quaternions(matrices):
    """Stable matrix -> xyzw, including pi rotations; continuous sign over time."""
    result = []
    for r in rotation(matrices):
        xx, yx, zx = r[:, 0]; xy, yy, zy = r[:, 1]; xz, yz, zz = r[:, 2]
        k = np.array([[xx-yy-zz, xy+yx, xz+zx, zy-yz],
                      [xy+yx, yy-xx-zz, yz+zy, xz-zx],
                      [xz+zx, yz+zy, zz-xx-yy, yx-xy],
                      [zy-yz, xz-zx, yx-xy, xx+yy+zz]]) / 3
        _, vectors = np.linalg.eigh(k)
        q = vectors[:, -1]
        if (result and np.dot(q, result[-1]) < 0) or (not result and q[3] < 0):
            q = -q
        result.append(q)
    return np.array(result)


def decode(info, timeline, chunk_info, cleaning, columns):
    """Pure decoder. All inputs originate in the same verified LeRobot episode."""
    dimension = info.get('encoded_action_dim')
    require(dimension in {27, 78}, 'Requires 27D or 78D schema')
    require(info.get('head_ee_definition') == 'eyes_link', 'Unverified head frame')
    frame = info.get('coordinate_frame')
    require(frame in {'base_link', 'chunk_first_frame_camera'}, 'Unsupported input frame')
    require(info.get('eef_rotation_representation') == 'rotation_matrix_first_two_columns', 'Wrong rot6 convention')
    require(info.get('state_action_units') == {'eef_position': 'm', 'joint_angle': 'rad', 'gripper': 'mm'}, 'Wrong units')
    n = info['total_frames']
    stamps_raw = timeline['timestamp_ns']
    require(all(type(x) is int for x in stamps_raw), 'Timestamps must be integer ns')
    ts = np.array(stamps_raw, dtype=np.int64)
    require(n >= 2 and ts.shape == (n,) and np.all(np.diff(ts) > 0), 'Invalid timeline')
    require(info['fps'] == 20 and np.all(np.diff(ts) == 50_000_000), 'Requires original 20Hz grid')
    require(np.array_equal(columns['frame_index'], np.arange(n)), 'Frame order changed')
    require(np.allclose(columns['timestamp'], (ts-ts[0])/1e9, atol=2e-6, rtol=0), 'Timestamp mismatch')
    mask = np.asarray(columns['validity'])
    require(mask.shape == (n, dimension) and np.isin(mask, [0, 1]).all(), 'Invalid validity mask')
    mask = mask.astype(bool)
    require(mask[:, :27].all(), 'Missing arm/head poses; no zero target allowed')
    require(not mask[:, 27:].any(), 'This Skye adapter does not execute active body78/hand/gripper slots')
    for key in ('state_mask', 'action_mask'):
        require(np.array_equal(np.asarray(info[key], dtype=bool), mask[0]) and np.all(mask == mask[0]), 'Mask metadata mismatch')
    if frame == 'chunk_first_frame_camera':
        require(chunk_info.get('head_frame') == 'eyes_link' and
                chunk_info.get('action_semantics') == 'absolute_target_in_fixed_chunk_frame', 'Wrong chunk/action semantics')
    sources = timeline['aligned_sources']
    expected_sources = [f'{role}/{side}' for role in ('observations', 'actions')
                        for side in ('left', 'right', 'active_camera')] + [v[0] for v in AUX.values()]
    all_times = [ts]
    for key in expected_sources:
        entry = sources[key]
        require(all(type(x) is int for x in entry['timestamp_ns']), 'Source time must be integer ns')
        st = np.array(entry['timestamp_ns'], dtype=np.int64)
        ix = np.asarray(entry['source_index'])
        require(st.shape == (n,) and ix.shape == (n,) and np.issubdtype(ix.dtype, np.integer)
                and (ix >= 0).all() and (np.diff(ix) >= 0).all(), 'Invalid source indices')
        require(np.all(np.abs(st-ts) <= 50_000_000), 'Source skew exceeds 50ms')
        if key.startswith('actions/'):
            require(np.all(st <= ts), 'Future action sample')
        all_times.append(st)
    require(np.all(np.ptp(np.stack(all_times), axis=0) <= 50_000_000), 'Cross-source skew exceeds 50ms')
    result = {'timestamp_ns': ts, 'offset_ns': ts-ts[0]}
    for role, col in [('state', 'observation.state'), ('action', 'action')]:
        v = np.asarray(columns[col], dtype=np.float64)
        require(v.shape == (n, dimension) and np.isfinite(v).all(), 'Invalid pose vector')
        require(np.all(v[~mask] == 0), 'Masked dimensions must be explicit zero placeholders')
        for name, k in [('left', 0), ('right', 9), ('head', 18)]:
            result[f'{role}_{name}_base_matrix'] = pose_matrix(v[:, k:k+9])
    if frame == 'chunk_first_frame_camera':
        end = 0
        for chunk in chunk_info['chunks']:
            start, stop = chunk['start_frame'], chunk['stop_frame']
            require(type(start) is int and type(stop) is int and start == end and start < stop <= n
                    and stop-start <= 32, 'Chunk gap, overlap or invalid length')
            t = np.array(chunk['base_from_chunk_camera'], dtype=float)
            require(t.shape == (4,4) and np.isfinite(t).all() and np.allclose(t[3], [0,0,0,1], atol=1e-9, rtol=0), 'Invalid anchor')
            rotation(t[:3, :3])
            require(np.allclose(result['state_head_base_matrix'][start], np.eye(4), atol=2e-5, rtol=0), 'Chunk first observed head is not identity')
            for key in list(result):
                if key.endswith('_base_matrix'):
                    result[key][start:stop] = t @ result[key][start:stop]
            end = stop
        require(end == n, 'Incomplete chunk coverage')
    for key in list(result):
        if key.endswith('_base_matrix'):
            t = result[key]
            result[key.replace('_matrix', '_xyz_xyzw')] = np.c_[t[:, :3, 3], quaternions(t[:, :3, :3])]
    for col, (source, names, unit, role) in AUX.items():
        feature, provenance = info['features'][col], cleaning['extra_streams'][col]
        require(feature['names'] == names and feature.get('unit') == unit and feature.get('source_role') == role,
                f'{col}: wrong joint order/units/role')
        stage = 'controller_output' if role == 'action' else 'measured_feedback'
        require(feature.get('source_stage') == stage and provenance['source_role'] == role
                and provenance['unit'] == unit and provenance['source_stage'] == stage, 'Unverified auxiliary source')
        value = np.asarray(columns[col], dtype=np.float64)
        require(value.shape == (n,len(names)) and np.isfinite(value).all(), 'Invalid auxiliary joints')
        result[col.replace('.', '_')] = value
        result[col.replace('.', '_') + '_source_timestamp_ns'] = np.array(sources[source]['timestamp_ns'], dtype=np.int64)
    return result


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''): h.update(block)
    return h.hexdigest()


def verify_episode(episode):
    require(not any(p.is_symlink() for p in episode.rglob('*')), 'Dataset symlinks forbidden')
    manifest = json.loads((episode/'meta/completion.json').read_text())
    actual = {str(p.relative_to(episode)):digest(p) for p in episode.rglob('*')
              if p.is_file() and p != episode/'meta/completion.json'}
    require(actual == manifest, 'LeRobot completion hash mismatch')
    return digest(episode/'meta/completion.json')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', required=True)
    parser.add_argument('--episode', required=True, type=int)
    args=parser.parse_args()
    import re
    require(re.fullmatch(r'[\w-]+',args.task) and 0 <= args.episode <= 99999999, 'Invalid task/episode')
    mount=Path(os.environ.get('APEX_STORAGE_MOUNT','/mnt/apex-data'))
    require(mount.is_absolute() and os.path.ismount(mount), 'WD disk not mounted')
    device=subprocess.check_output(['findmnt','-rn','-M',str(mount),'-o','SOURCE'],text=True).strip()
    require('WD-WX12A367D1A2' in subprocess.check_output(['lsblk','-sn','-o','SERIAL',device],text=True).split(), 'Wrong storage device')
    root=mount/'apex_n_sharpa/data'
    require(root.resolve()==root, 'Data root symlink forbidden')
    storage_device = mount.stat().st_dev
    episode=root/'lerobot'/args.task/f'episode_{args.episode:08d}'
    require(episode.resolve()==episode, 'Episode symlink forbidden')
    fingerprint=verify_episode(episode)
    meta=episode/'meta'
    read=lambda name:json.loads((meta/name).read_text())
    table=pq.read_table(episode/'data/chunk-000/episode_000000.parquet')
    wanted=['action','observation.state','validity','frame_index','timestamp',*AUX]
    columns={name:np.array(table[name].to_pylist()) for name in wanted}
    result=decode(read('info.json'),read('source_timestamps.json'),read('chunk_frames.json') if read('info.json')['coordinate_frame'] == 'chunk_first_frame_camera' else None,read('cleaning_report.json'),columns)
    out=root/'replay'/args.task/f'episode_{args.episode:08d}'
    require(out.parent.resolve()==out.parent and not out.exists(), 'Output exists or parent symlink; choose a clean output')
    require(os.path.ismount(mount) and mount.stat().st_dev == storage_device, 'Storage mount changed')
    out.parent.mkdir(parents=True,exist_ok=True)
    tmp=out.with_name('.'+out.name+'.partial')
    require(not tmp.exists(), 'Partial output exists; inspect before retry')
    tmp.mkdir()
    try:
        np.savez_compressed(tmp/'targets.npz',**result)
        report={'schema':'skye_lerobot_replay_targets_v1','source_episode':str(episode),
                'source_completion_sha256':fingerprint,'frames':len(result['timestamp_ns']), 'fps':20,
                'head_frame':'eyes_link','coordinate_frame':'base_link','quaternion_order':'xyzw',
                'position_unit':'m','joint_units':{'body':['m','rad','rad'],'head':['rad','rad']},
                'body_joint_names':BODY,'head_joint_names':HEAD,
                'targets':'action_* only; state_* is diagnostic feedback, never command substitution',
                'source_files':'LeRobot only; no MCAP/HDF5 reads','masked_78_slots_omitted':list(range(27,78)),
                'auxiliary_commands':'body/head controller targets are independently recorded valid features, not body78 slot values',
                'timing':'offset_ns is relative schedule; source timestamps remain provenance, never reuse old ROS headers',
                'robot_commands_sent':0,'execution_ready':False,
                'execution_requirements':['Skye IK with current joint seed, limits and collision checks',
                    'choose Cartesian head target OR recorded head controller target, never both',
                    'fresh robot feedback, verified input mux, start-pose transition and stop handling'],
                'limitations':['20Hz causally sampled targets cannot recover original higher-rate commands',
                    'Arm joints are not uniquely recoverable from EEF; no recorded joint trajectory is claimed',
                    'Chassis and hand/gripper execution not supported by this adapter']}
        (tmp/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        require(verify_episode(episode)==fingerprint, 'Input changed during adaptation')
        require(os.path.ismount(mount) and mount.stat().st_dev == storage_device, 'Storage mount changed')
        (tmp/'completion.json').write_text(json.dumps({p.name:digest(p) for p in tmp.iterdir() if p.is_file()},indent=2)+'\n')
        tmp.rename(out)
    except BaseException:
        shutil.rmtree(tmp)
        raise
    print(json.dumps({'output':str(out),'frames':len(result['timestamp_ns']),'robot_commands_sent':0},ensure_ascii=False))


if __name__=='__main__': main()
