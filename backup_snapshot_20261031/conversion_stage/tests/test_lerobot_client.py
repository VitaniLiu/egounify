import copy
import importlib.util
from pathlib import Path
import numpy as np
import pytest

spec=importlib.util.spec_from_file_location('endpoint',Path(__file__).parents[1]/'src/lerobot_client/robot_endpoint.py')
e=importlib.util.module_from_spec(spec);spec.loader.exec_module(e)


def plan():
    return dict(schema='skye_lerobot_client_v1',execute=False,offset_ns=[0,50_000_000],
        available=[[True]*11 for _ in range(2)],replay_contract={'schema':'skye_recorded_ik_mapping_v1'},
        left=[np.eye(4).tolist()]*2,right=[np.eye(4).tolist()]*2,
        body=[[.5,0,0]]*2,head=[[0,0]]*2,ignored_slots=list(range(32,78)))


@pytest.mark.parametrize('fault',['time','mask','nan','rotation','jump','body_jump'])
def test_bad_plan_rejected(fault):
    p=copy.deepcopy(plan())
    if fault=='time':p['offset_ns'][1]+=1
    if fault=='mask':p['ignored_slots'].remove(54)
    if fault=='nan':p['head'][0][0]=float('nan')
    if fault=='rotation':p['left'][0][0][0]=-1
    if fault=='jump':p['left']=[np.eye(4).tolist(),np.eye(4).tolist()];p['left'][1][0][3]=.2
    if fault=='body_jump':p['body']=[[.5,0,0],[.8,0,0]]
    with pytest.raises(ValueError):e.validate_plan(p)


def test_plan_ignores_uncollected_hands():
    assert e.validate_plan(plan())==2


def test_missing_gate_and_mapping_rejected():
    p=plan();p.pop('available')
    with pytest.raises(ValueError):e.validate_plan(p)
    p=plan();p.pop('replay_contract')
    with pytest.raises(ValueError):e.validate_plan(p)


def test_arm_enable_disagreement_rejected():
    p=plan();p['available'][0][1]=False
    with pytest.raises(ValueError):e.validate_plan(p)


@pytest.mark.parametrize('axis',np.eye(3))
def test_pi_rotation_quaternion(axis):
    q=e.quat(e.rot(axis,np.pi))
    assert abs(np.dot(q[:3],axis))==pytest.approx(1)
    assert abs(q[3])<1e-10


def test_mjcf_fk_rotation_slide_and_base_offset():
    xml='''<mujoco><compiler angle="radian"/><worldbody>
      <body name="base_link" pos="4 5 6"><body name="lift" pos="0 0 1">
      <joint name="slide" type="slide" axis="0 0 1" range="0 1"/>
      <body name="turn"><joint name="yaw" axis="0 0 1" range="-3.2 3.2"/>
      <body name="tool" pos="1 0 0"/></body></body></body></worldbody></mujoco>'''
    model=e.ControllerModel(xml)
    t=model.fk('tool',{'slide':.5,'yaw':np.pi/2})
    np.testing.assert_allclose(t[:3,3],[0,1,1.5],atol=1e-10)
    np.testing.assert_allclose(t[:3,:3],e.rot([0,0,1],np.pi/2),atol=1e-10)
    with pytest.raises(ValueError):model.fk('tool',{'slide':2,'yaw':0})


def test_urdf_fk():
    xml='''<robot><joint name="lift" type="prismatic"><parent link="base_link"/>
      <child link="tool"/><origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/>
      <axis xyz="1 0 0"/><limit lower="0" upper="1"/></joint></robot>'''
    np.testing.assert_allclose(e.Model(xml).fk('tool',{'lift':.5})[:3,3],[1,2.5,3])


def test_native_fixed_width_commands():
    assert e.active_command('body',[.5,.1,-.1,0,0,0])==[.5,.1,-.1]
    assert e.active_command('head',[.2,.3,0])==[.2,.3]
    with pytest.raises(ValueError):e.active_command('body',[.5,0,0])
    with pytest.raises(ValueError):e.active_command('head',[0,0,.2])
    with pytest.raises(ValueError):e.active_command('A',[float('nan')]*7)


def test_home_feedback_tolerance_default_and_override():
    p=plan()
    assert e.home_feedback_tolerance(p)==0.01
    p['test_home_feedback_tolerance_rad']=0.015
    assert e.validate_plan(p)==2
    assert e.home_feedback_tolerance(p)==0.015


@pytest.mark.parametrize('value',[0.15,0.016,0,float('nan'),True,'0.015'])
def test_invalid_home_feedback_override(value):
    p=plan();p['test_home_feedback_tolerance_rad']=value
    with pytest.raises(ValueError):e.validate_plan(p)


def test_home_feedback_override_rejects_long_plan():
    p=plan();p['test_home_feedback_tolerance_rad']=0.015;p['offset_ns']=list(range(0,350000000,50000000))
    with pytest.raises(ValueError,match='six frames'):e.validate_plan(p)


def test_full_trajectory_override_is_exactly_scoped():
    p=plan();n=314;p['offset_ns']=[i*50_000_000 for i in range(n)]
    for key in ('left','right'):p[key]=[np.eye(4).tolist()]*n
    p['body']=[[.5,0,0]]*n;p['head']=[[0,0]]*n;p['available']=[[True]*11 for _ in range(n)]
    p['test_home_feedback_tolerance_rad']=0.015;p['full_trajectory_test']=True
    assert e.validate_plan(p)==314
    p['offset_ns'].pop()
    with pytest.raises(ValueError,match='exactly 314'):e.validate_plan(p)


@pytest.mark.parametrize('fault', [None, 'future', 'unsynced', 'frame', 'source', 'gate'])
def test_recorded_ik_targets_without_fitted_mapping(fault):
    root = Path(__file__).parents[1] / 'src'
    def load(name, path):
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        return module
    client = load('client', root / 'lerobot_client/main.py')
    decoder = load('decoder', root / 'lerobot_replay/main.py')
    ts = [100_000_000, 150_000_000]
    info = {'features': {}}; cleaning = {'extra_streams': {}}
    timeline = {'timestamp_ns': ts, 'aligned_sources': {}}; columns = {}
    for key, source in client.IK_STREAMS.items():
        info['features'][key] = dict(source_role='action', source_stage='ik_input',
                                     coordinate_frame='base_link', unit='m')
        cleaning['extra_streams'][key] = dict(source_role='action', source_stage='ik_input',
                                             source_topic='/tj/control/ik_request', source_coordinate_frame='tracking_base_link')
        timeline['aligned_sources'][source] = {'timestamp_ns': ts.copy()}
        columns[key] = np.array([[1, 2, 3, 1, 0, 0, 0, 1, 0]] * 2, dtype=float)
    for side in client.IK_POSES:
        info['features']['action.ik_'+side]['names'] = ['x','y','z','r1x','r1y','r1z','r2x','r2y','r2z']
    info['features']['action.ik_available'].update(unit='bool', names=['left_hand','right_hand','left_elbow','right_elbow','torso','head','pelvis','left_foot','right_foot','left_knee','right_knee'])
    columns['action.ik_available'] = np.ones((2, 11))
    columns['action.ik_joint_positions'] = np.zeros((2, 19))
    info['features']['action.ik_joint_positions'].update(names=['position_'+str(i) for i in range(19)], unit='m,rad', coordinate_frame='native_joint_coordinates')
    if fault == 'future': timeline['aligned_sources']['actions/ik_left']['timestamp_ns'][0] += 1
    if fault == 'unsynced': timeline['aligned_sources']['actions/ik_left']['timestamp_ns'][0] -= 1
    if fault == 'frame': info['features']['action.ik_left']['coordinate_frame'] = 'tracking_base_link'
    if fault == 'source': cleaning['extra_streams']['action.ik_left']['source_topic'] = '/target_pose'
    if fault == 'gate': columns['action.ik_available'][0, 0] = .5
    if fault:
        with pytest.raises(ValueError): client.recorded_ik_targets(info, timeline, cleaning, columns, decoder)
    else:
        poses, gate, contract = client.recorded_ik_targets(info, timeline, cleaning, columns, decoder)
        np.testing.assert_allclose(poses['left'][:, :3, 3], [[1, 2, 3]] * 2)
        assert gate.dtype == bool and contract['schema'] == 'skye_recorded_ik_targets_v3'


def test_recorded_ik_endpoint_contract():
    p = plan()
    p['replay_contract'] = dict(schema='skye_recorded_ik_targets_v2', input_frame='base_link',
        output_frame='base_link', body_head_policy='recorded_controller_FK', source_topic='/tj/control/ik_request')
    assert e.validate_plan(p) == 2
    p['replay_contract']['input_frame'] = 'camera'
    with pytest.raises(ValueError): e.validate_plan(p)


@pytest.mark.parametrize('source,slot', [('foot_midpoint',7),('foot_midpoint',8),('pelvis',6)])
def test_missing_recorded_support_targets_rejected(source,slot):
    p=plan();p['replay_contract']['schema']='skye_recorded_ik_targets_v2'
    p['available']=[[False]*11 for _ in range(2)];p['available'][0][slot]=True
    with pytest.raises(RuntimeError,match='support-reference'):
        e.validate_reference_inputs(p,dict(support_frame_enabled=True,support_frame_source=source))
    e.validate_reference_inputs(p,dict(support_frame_enabled=False,support_frame_source=source))
    p['available'][0][slot]=False
    e.validate_reference_inputs(p,dict(support_frame_enabled=True,support_frame_source=source))


def complete_plan():
    p = plan()
    p['execution_target_policy'] = 'robot_controller_fk_inverse_support_v1'
    keys = [k for k, _, _ in e.IK_FIELDS]
    p['replay_contract'] = dict(schema='skye_recorded_ik_targets_v3', input_frame='base_link',
        output_frame='base_link', body_head_policy='recorded_ik', source_topic='/tj/control/ik_request',
        pose_fields=keys, joint_state_names=[])
    p['ik_reference'] = {k: [np.eye(4).tolist() for _ in range(2)] for k in keys}
    p['ik_reference']['joint_positions'] = [[0.0]*19 for _ in range(2)]
    p['validation_trajectory'] = {key:copy.deepcopy(p[key]) for key in ('left','right','body','available')}
    return p


@pytest.mark.parametrize('fault', [None, 'missing_foot', 'bad_rotation', 'bad_seed', 'arm_conflict'])
def test_complete_reference_integrity(fault):
    p = complete_plan()
    if fault == 'missing_foot': del p['ik_reference']['left_foot']
    if fault == 'bad_rotation': p['ik_reference']['pelvis'][0][0][0] = 2
    if fault == 'bad_seed': p['ik_reference']['joint_positions'][0].pop()
    if fault == 'arm_conflict': p['ik_reference']['left'][0][0][3] = .01
    if fault:
        with pytest.raises(ValueError): e.validate_plan(p)
    else:
        assert e.validate_plan(p) == 2
        e.validate_reference_inputs(p, dict(support_frame_enabled=True, support_frame_source='foot_midpoint'))


def test_complete_request_serializes_all_references_and_seed():
    from types import SimpleNamespace as S
    p = complete_plan()
    for i,(key,_,_) in enumerate(e.IK_FIELDS):
        p['ik_reference'][key][0][0][3] = i / 100
    p['ik_reference']['joint_positions'][0] = list(range(19))
    target = e.frame_target(p, 0, None, {}, {})
    msg = S(header=S(), available=[False]*11, joint_state=S())
    for _,_,field in e.IK_FIELDS:
        setattr(msg, field, S(position=S(), orientation=S()))
    e.fill_ik_request(msg, target, [True]*11)
    assert msg.header.frame_id == 'base_link' and all(msg.available)
    assert msg.left_foot_pose.position.x == .07
    assert msg.torso_pose.position.x == .04
    assert msg.joint_state.position == list(range(19))


def test_robot_target_inverse_support_preserves_recorded_references():
    p = complete_plan()
    for key in ('left_foot', 'right_foot'):
        p['ik_reference'][key][0][0][3] = .2
        p['ik_reference'][key][1][0][3] = .4
    original = copy.deepcopy(p)
    class Model:
        def fk(self, link, q):
            t = np.eye(4); t[:3,3] = [1,2,3] if link == 'torso_link' else [4,5,6]
            return t
    qp = dict(support_frame_enabled=True, support_frame_source='foot_midpoint',
              support_frame_filter_alpha=.2, support_frame_use_z=True, base_closure_enabled=False)
    frames = e.robot_target_frames(p, Model(), {}, {'body':['a','b','c'], 'head':['d','e']}, qp)
    for index, origin in enumerate((.2, .24)):
        np.testing.assert_allclose(frames[index]['torso'][:3,3]-[origin,0,0], [1,2,3])
        np.testing.assert_allclose(frames[index]['head'][:3,3]-[origin,0,0], [4,5,6])
        np.testing.assert_allclose(frames[index]['left'][:3,3]-[origin,0,0], [0,0,0], atol=1e-12)
        np.testing.assert_array_equal(frames[index]['left_foot'], original['ik_reference']['left_foot'][index])
    assert p == original
    p['available'][0][7] = False
    with pytest.raises(ValueError, match='origin'):
        e.robot_target_frames(p, Model(), {}, {'body':['a','b','c'], 'head':['d','e']}, qp)
    qp['base_closure_enabled'] = True
    with pytest.raises(ValueError, match='configuration'):
        e.robot_target_frames(original, Model(), {}, {}, qp)


def test_reach_bound_checks_full_trajectory_not_only_selected_prefix():
    p = complete_plan()
    p['validation_trajectory'] = {k:copy.deepcopy(v)+[copy.deepcopy(v[-1])] for k,v in p['validation_trajectory'].items()}
    for side in ('left','right'):
        p['validation_trajectory'][side][2][0][3] = .7
    class Model:
        def __init__(self):
            self.bodies = {}
            for label in ('L','R'):
                for i in range(2,8):
                    t = np.eye(4)
                    if i == 2: t[0,3] = .5
                    self.bodies['Link'+str(i)+'_'+label] = (None,t,None)
        def fk(self, link, positions):
            t = np.eye(4)
            if link.endswith('_tool'): t[0,3] = .1
            return t
    result = e.arm_reachability(p, Model(), {}, {'body':['a','b','c']})
    assert not result['passed'] and result['frames'] == 3
    assert result['arms']['left']['unreachable_frames'] == [2]
    assert result['arms']['left']['worst']['excess_m'] == pytest.approx(.1)


def test_selected_prefix_cannot_change_full_validation_targets():
    p = complete_plan(); p['validation_trajectory']['left'][0][0][3] = .1
    with pytest.raises(ValueError, match='Selected trajectory'):
        e.validate_plan(p)

@pytest.mark.parametrize('fault',[None,'chassis_speed','chassis_stop','ik_time','gate','pose','tamper'])
def test_native_lerobot_controls_and_integrity(tmp_path, fault):
    import json, hashlib
    spec=importlib.util.spec_from_file_location('native_client',Path(__file__).parents[1]/'src/lerobot_client/main.py')
    client=importlib.util.module_from_spec(spec);spec.loader.exec_module(client)
    meta=tmp_path/'meta';meta.mkdir()
    poses={k:[[0.,0.,0.,1.,0.,0.,0.,1.,0.]]*2 for k in client.IK_POSES}
    poses.update(available=[[1]*11]*2,joint_positions=[[0.]*19]*2)
    data=dict(schema='skye_native_replay_v1',coordinate_frame='base_link',ik_timestamp_ns=[100,20000100],ik=poses,
              chassis_timestamp_ns=[0,20000000,40000000],chassis_velocity=[[.1,0,0],[.1,0,0],[0,0,0]],
              chassis_frame='base_link',chassis_units=['m/s','m/s','rad/s'],
              initial_joint_names=[str(i) for i in range(19)],initial_joint_positions=[0.]*19)
    if fault=='chassis_speed':data['chassis_velocity'][0][0]=.41
    if fault=='chassis_stop':data['chassis_velocity'][-1][0]=.1
    if fault=='ik_time':data['ik_timestamp_ns'][1]=100
    if fault=='gate':data['ik']['available']=[[2]*11]*2
    if fault=='pose':data['ik']['left']=[[0.]*9]*2
    (meta/'info.json').write_text(json.dumps({'features':{'action':{'shape':[78]}}}))
    (meta/'source_timestamps.json').write_text(json.dumps({'native_replay':data}))
    (meta/'completion.json').write_text(json.dumps({str(p.relative_to(tmp_path)):hashlib.sha256(p.read_bytes()).hexdigest() for p in meta.iterdir()}))
    if fault=='tamper':(meta/'info.json').write_text('{}')
    if fault:
        with pytest.raises(ValueError):client.prepare_native(tmp_path,False)
    else:
        result=client.prepare_native(tmp_path,False)
        assert result['offset_ns']==[100,20000100]
        assert result['chassis_offset_ns']==[0,20000000,40000000]
        assert result['raw_or_hdf5_read'] is False
        assert np.array_equal(np.array(result['ik_reference']['left'])[0],np.eye(4))
