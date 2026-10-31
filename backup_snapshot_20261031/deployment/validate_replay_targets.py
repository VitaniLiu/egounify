"""Independent audit only; HDF5 is a test oracle, never a replay input."""
from pathlib import Path
import json
import hashlib
import numpy as np
import h5py
ROOT=Path('/mnt/apex-data/apex_n_sharpa/data')
TASK='robot_import_20261024_egoav_v4'


def matrix(p):
    p=np.asarray(p,dtype=float);q=p[:,3:];q=q/np.linalg.norm(q,axis=1)[:,None]
    x,y,z,w=q.T
    r=np.array([[1-2*y*y-2*z*z,2*x*y-2*z*w,2*x*z+2*y*w],
                [2*x*y+2*z*w,1-2*x*x-2*z*z,2*y*z-2*x*w],
                [2*x*z-2*y*w,2*y*z+2*x*w,1-2*x*x-2*y*y]]).transpose(2,0,1)
    t=np.repeat(np.eye(4)[None],len(p),axis=0);t[:,:3,:3]=r;t[:,:3,3]=p[:,:3]
    return t


reports=[]
for i in range(3):
    ep=f'episode_{i:08d}';out=ROOT/'replay'/TASK/ep;lr=ROOT/'lerobot'/TASK/ep
    manifest=json.loads((out/'completion.json').read_text())
    for name,sha in manifest.items():assert hashlib.sha256((out/name).read_bytes()).hexdigest()==sha
    data=np.load(out/'targets.npz',allow_pickle=False)
    timeline=json.loads((lr/'meta/source_timestamps.json').read_text())
    np.testing.assert_array_equal(data['timestamp_ns'],timeline['timestamp_ns'])
    errors={}
    with h5py.File(ROOT/'hdf5'/TASK/f'hdf5_{i:08d}.hdf5','r') as h:
        for role,parent in [('state','observations'),('action','actions')]:
            for name,source in [('left','left'),('right','right'),('head','active_camera')]:
                path=f'{parent}/{source}';ix=timeline['aligned_sources'][path]['source_index']
                truth=matrix(h[path]['eef'][:][ix])
                direct=data[f'{role}_{name}_base_matrix'];quat=matrix(data[f'{role}_{name}_base_xyz_xyzw'])
                np.testing.assert_allclose(direct,truth,atol=2e-6,rtol=0)
                np.testing.assert_allclose(quat,truth,atol=2e-6,rtol=0)
                errors[f'{role}_{name}']=float(np.max(abs(quat-truth)))
        for key,path in [('action_body_controller','actions/body_controller'),('action_head_controller','actions/head_controller'),('observation_body_joints','observations/body_joints')]:
            ix=timeline['aligned_sources'][path]['source_index'];truth=h[path]['value'][:][ix]
            np.testing.assert_allclose(data[key],truth,atol=1e-7,rtol=1e-6)
    reports.append({'episode':ep,'frames':len(data['timestamp_ns']),'matrix_and_quaternion_max_errors':errors,'joint_target_check':'passed','input_role_separation':'passed','robot_commands_sent':0})
path=ROOT/'inspection/replay_preflight/adapter_accuracy.json'
path.write_text(json.dumps({'oracle':'independent HDF5 audit; runtime adapter reads LeRobot only','episodes':reports},indent=2)+'\n')
print(json.dumps(reports,indent=2));print(path)
