import importlib.util
from pathlib import Path
import numpy as np
import pytest

spec=importlib.util.spec_from_file_location('incremental',Path(__file__).parents[1]/'src/lerobot_replay/export_incremental.py')
d=importlib.util.module_from_spec(spec);spec.loader.exec_module(d)


def poses():
    t=np.repeat(np.eye(4)[None],34,axis=0)
    rz=np.array([[0,-1,0],[1,0,0],[0,0,1]])
    rx=np.array([[1,0,0],[0,0,-1],[0,1,0]])
    t[:,:3,:3]=rz;t[1:,:3,:3]=rx@rz
    t[:,:3,3]=np.c_[np.arange(34)*.01+2,np.ones(34)*3,np.ones(34)*4]
    return t,rx


def test_base_frame_delta_noncommuting_rotation_and_nonzero_origin():
    original,rx=poses();delta,anchor=d.pose_delta(original)
    np.testing.assert_allclose(delta[0],np.eye(4))
    np.testing.assert_allclose(delta[1,:3,:3],rx)
    np.testing.assert_allclose(delta[1,:3,3],[.01,0,0])
    np.testing.assert_allclose(d.restore_pose(delta,anchor),original,atol=1e-12)
    # Spatial displacement is not the translation of T(t) @ inv(T(0)).
    assert not np.allclose(delta[1,:3,3],(original[1]@np.linalg.inv(anchor))[:3,3])


def test_chunk_boundary_not_reset_and_roles_masks_preserved():
    t,_=poses();n=len(t)
    targets={'timestamp_ns':np.arange(n,dtype=np.int64)*50_000_000+1792751767339605580,'offset_ns':np.arange(n,dtype=np.int64)*50_000_000}
    for role in ('state','action'):
        for side in ('left','right','head'):
            a=t.copy();a[:,:3,3]+=10 if role=='action' else 0
            targets[f'{role}_{side}_base_matrix']=a
    for key,width in [('action_body_controller',3),('action_head_controller',2),('observation_body_joints',5)]:
        targets[key]=np.arange(n*width).reshape(n,width)/100
    mask=np.zeros((n,78),np.uint8);mask[:,:27]=1
    out,errors=d.convert(targets,mask)
    assert max(errors.values())<1e-12
    assert out['action_delta78'][32,0]==pytest.approx(.32)
    np.testing.assert_array_equal(out['action_delta78'][0,:9],[0,0,0,1,0,0,0,1,0])
    assert not out['action_mask'][:,27:].any()
    assert not out['action_delta78'][:,27:].any()
    assert not np.array_equal(out['state_anchor_base_matrix'],out['action_anchor_base_matrix'])
    np.testing.assert_array_equal(out['timestamp_ns'],targets['timestamp_ns'])
    np.testing.assert_array_equal(out['action_body_controller_delta'][0],[0,0,0])


@pytest.mark.parametrize('bad',['nan','reflection','lastrow'])
def test_invalid_pose_rejected(bad):
    t,_=poses()
    if bad=='nan':t[0,0,3]=np.nan
    if bad=='reflection':t[0,:3,0]*=-1
    if bad=='lastrow':t[0,3,0]=1
    with pytest.raises(ValueError):d.pose_delta(t)
