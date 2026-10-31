import copy
import importlib.util
from pathlib import Path
import numpy as np
import pytest

spec=importlib.util.spec_from_file_location('replay_adapter',Path(__file__).resolve().parents[1]/'src/lerobot_replay/main.py')
r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)


def sample():
    n=4;ts=np.arange(n,dtype=np.int64)*50_000_000+1_792_751_000_000_000_000
    identity=[0,0,0,1,0,0,0,1,0]
    state=np.zeros((n,78));action=state.copy()
    for k in (0,9,18):
        state[:,k:k+9]=identity;action[:,k:k+9]=identity
    state[:,0]=2;action[:,0]=3;action[:,18]=.4
    mask=np.zeros((n,78),dtype=np.uint8);mask[:,:27]=1
    col={'action':action,'observation.state':state,'validity':mask,'timestamp':np.arange(n)*.05,'frame_index':np.arange(n)}
    info={'encoded_action_dim':78,'head_ee_definition':'eyes_link','coordinate_frame':'chunk_first_frame_camera',
          'eef_rotation_representation':'rotation_matrix_first_two_columns',
          'state_action_units':{'eef_position':'m','joint_angle':'rad','gripper':'mm'},
          'total_frames':n,'fps':20,'state_mask':mask[0].tolist(),'action_mask':mask[0].tolist(),'features':{}}
    cleaning={'extra_streams':{}};sources={}
    for role in ('observations','actions'):
        for side in ('left','right','active_camera'):
            sources[f'{role}/{side}']={'timestamp_ns':ts.tolist(),'source_index':list(range(n))}
    for name,(key,names,unit,role) in r.AUX.items():
        stage='controller_output' if role=='action' else 'measured_feedback'
        info['features'][name]={'names':names,'unit':unit,'source_role':role,'source_stage':stage}
        cleaning['extra_streams'][name]={'unit':unit,'source_role':role,'source_stage':stage}
        sources[key]={'timestamp_ns':ts.tolist(),'source_index':list(range(n))}
        col[name]=np.full((n,len(names)),.2 if role=='action' else .1)
    t=np.array([[0,-1,0,1],[1,0,0,2],[0,0,1,3],[0,0,0,1]],float)
    t2=np.array([[-1,0,0,4],[0,-1,0,5],[0,0,1,6],[0,0,0,1]],float)
    chunks={'head_frame':'eyes_link','action_semantics':'absolute_target_in_fixed_chunk_frame','chunks':[
        {'start_frame':0,'stop_frame':2,'base_from_chunk_camera':t.tolist()},
        {'start_frame':2,'stop_frame':4,'base_from_chunk_camera':t2.tolist()}]}
    return info,{'timestamp_ns':ts.tolist(),'aligned_sources':sources},chunks,cleaning,col


def test_inverse_nontrivial_rotation_translation_and_roles():
    inputs=sample();a=r.decode(*inputs)
    np.testing.assert_allclose(a['action_left_base_xyz_xyzw'][0,:3],[1,5,3])
    np.testing.assert_allclose(a['state_left_base_xyz_xyzw'][0,:3],[1,4,3])
    np.testing.assert_allclose(a['action_left_base_xyz_xyzw'][2,:3],[1,5,6])
    np.testing.assert_allclose(a['action_head_base_xyz_xyzw'][0,:3],[1,2.4,3])
    np.testing.assert_array_equal(a['offset_ns'],[0,50_000_000,100_000_000,150_000_000])
    np.testing.assert_allclose(a['action_body_controller'],.2)
    np.testing.assert_allclose(a['observation_body_joints'],.1)
    # No masked zero slots emitted as actuator targets.
    assert not any('gripper' in key or 'hand' in key for key in a)
    np.testing.assert_allclose(inputs[-1]['action'][:,0],3)


@pytest.mark.parametrize('axis',[0,1,2])
def test_quaternion_pi_stability(axis):
    matrix=-np.eye(3);matrix[axis,axis]=1
    q=r.quaternions(np.array([matrix,matrix]))
    assert np.isfinite(q).all()
    np.testing.assert_allclose(abs(q[:,axis]),1,atol=1e-10)
    np.testing.assert_allclose(q[:,3],0,atol=1e-10)
    assert np.dot(q[0],q[1])>0


@pytest.mark.parametrize('fault',['mask','nonzero_masked','gap','overlap','reflection','rotation','future','skew','unit','role','names','head_frame','semantics','timeline','first_head','active_body'])
def test_bad_data_rejected(fault):
    info,time,chunks,clean,col=sample()
    if fault=='mask':col['validity'][0,0]=0
    if fault=='nonzero_masked':col['action'][0,32]=1
    if fault=='gap':chunks['chunks'][1]['start_frame']=3
    if fault=='overlap':chunks['chunks'][1]['start_frame']=1
    if fault=='reflection':chunks['chunks'][0]['base_from_chunk_camera'][2][2]=-1
    if fault=='rotation':col['action'][0,3:9]=0
    if fault=='future':time['aligned_sources']['actions/left']['timestamp_ns'][0]+=1
    if fault=='skew':time['aligned_sources']['actions/left']['timestamp_ns'][0]-=60_000_000
    if fault=='unit':info['features']['action.body_controller']['unit']='mm,deg,deg'
    if fault=='role':info['features']['action.body_controller']['source_role']='state'
    if fault=='names':info['features']['action.body_controller']['names']=r.BODY[::-1]
    if fault=='head_frame':info['head_ee_definition']='head_link3'
    if fault=='semantics':chunks['action_semantics']='relative_delta'
    if fault=='timeline':time['timestamp_ns'][2]=time['timestamp_ns'][1]
    if fault=='first_head':col['observation.state'][2,18]=1
    if fault=='active_body':col['validity'][:,27]=1
    with pytest.raises(ValueError):r.decode(info,time,chunks,clean,col)


@pytest.mark.parametrize('dimension', [27, 78])
def test_base_frame_needs_no_chunk_anchors(dimension):
    info, timeline, chunks, cleaning, columns = sample()
    info.update(coordinate_frame='base_link', encoded_action_dim=dimension)
    for key in ('action', 'observation.state', 'validity'):
        columns[key] = columns[key][:, :dimension]
    for key in ('action_mask', 'state_mask'):
        info[key] = info[key][:dimension]
    result = r.decode(info, timeline, None, cleaning, columns)
    np.testing.assert_allclose(result['action_left_base_matrix'][:, 0, 3], 3)
    np.testing.assert_allclose(result['state_left_base_matrix'][:, 0, 3], 2)
    np.testing.assert_allclose(result['action_head_base_matrix'][:, 0, 3], .4)
