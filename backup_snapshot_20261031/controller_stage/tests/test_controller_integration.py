"""ROS-wire encoder and lab-only execution gates; no ROS or SDK import needed."""
import copy,json,math,sys,os,hashlib
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from unified_replay.integration import api as a


def header():return S(stamp=None,frame_id='')
def pose():return S(position=S(x=0.,y=0.,z=0.),orientation=S(x=0.,y=0.,z=0.,w=0.))
def payload():
    p={k:[.1,.2,.3,1.,0.,0.,0.,1.,0.] for k in a.POSE_FIELDS}
    p.update(available=[True,False]*5+[True],joint_positions=list(range(19)))
    return p


def test_ik_encoder_preserves_all_eleven_fields_and_seed():
    m=S(header=header(),available=[False]*11,joint_state=S(position=[]))
    for field in a.POSE_FIELDS.values():setattr(m,field,pose())
    p=payload();stamp=object();a.fill_ik(m,p,stamp)
    assert m.header.stamp is stamp and m.header.frame_id=='base_link'
    assert m.available==p['available'] and m.joint_state.position==p['joint_positions']
    for field in a.POSE_FIELDS.values():
        q=getattr(m,field);assert [q.position.x,q.position.y,q.position.z]==[.1,.2,.3]
        assert [q.orientation.x,q.orientation.y,q.orientation.z,q.orientation.w]==[0,0,0,1]


@pytest.mark.parametrize('axis',[0,1,2])
def test_pi_rotation_serialization(axis):
    r=-np.eye(3);r[axis,axis]=1
    values=[0,0,0]+r[:,0].tolist()+r[:,1].tolist();q=a.quaternion(values)
    assert abs(q[axis])==pytest.approx(1) and abs(q[3])<1e-10


@pytest.mark.parametrize('v',[[0]*9,[0]*8,[float('nan')]*9])
def test_bad_rotation_rejected(v):
    with pytest.raises(ValueError):a.quaternion(v)


def test_hand_encoder_no_gripper_slot_or_stale_stamp():
    m=S(header=header(),name=[],position=[]);stamp=object();a.fill_hand(m,[.03]*22,stamp,'session/17')
    assert m.name==a.HAND_NAMES and len(m.position)==22 and m.header.stamp is stamp
    assert m.header.frame_id=='session/17'
    with pytest.raises(ValueError):a.fill_hand(m,[0]*23,stamp,'bad')


@pytest.mark.parametrize('v',[[.41,0,0],[0,.01,0],[0,0,.51],[float('nan'),0,0]])
def test_chassis_invalid_commands_rejected(v):
    m=S(header=header(),twist=S(linear=S(x=0,y=0,z=0),angular=S(x=0,y=0,z=0)))
    with pytest.raises(ValueError):a.fill_chassis(m,v,object())


def test_chassis_three_to_ros_six_axis_mapping():
    m=S(header=header(),twist=S(linear=S(x=0,y=0,z=0),angular=S(x=0,y=0,z=0)))
    a.fill_chassis(m,[.2,0,-.3],object())
    assert m.twist.linear.x==.2 and m.twist.angular.z==-.3
    assert [m.twist.linear.y,m.twist.linear.z,m.twist.angular.x,m.twist.angular.y]==[0]*4


@pytest.mark.parametrize('fault',['status','clamp','value','stamp','names'])
def test_sdk_receipt_is_not_assumed_from_publish(fault):
    r=dict(schema='sharpa_command_result_v2',command_stamp_ns=123,status='sdk_call_succeeded',clamped_joints=[],target=[.03]*22,names=a.HAND_NAMES)
    if fault=='status':r['status']='rejected'
    if fault=='clamp':r['clamped_joints']=['thumb_CMC_FE']
    if fault=='value':r['target'][0]=0
    if fault=='stamp':r['command_stamp_ns']=124
    if fault=='names':r['names']=list(reversed(a.HAND_NAMES))
    with pytest.raises(ValueError):a.check_hand_receipt(r,123,[.03]*22)


@pytest.mark.parametrize('domain,local,ns,profile',[(29,'1','/unified_replay_lab_aabbccddeeff','isolated_controller'),
    (197,'0','/unified_replay_lab_aabbccddeeff','isolated_controller'),
    (197,'1','/tj','isolated_controller'),(197,'1','/unified_replay_lab_aabbccddeeff','live')])
def test_physical_domains_topics_and_profile_rejected(monkeypatch,domain,local,ns,profile):
    monkeypatch.setenv('ROS_DOMAIN_ID',str(domain));monkeypatch.setenv('ROS_LOCALHOST_ONLY',local)
    with pytest.raises(PermissionError):a.lab_guard(ns,profile)


def test_live_profile_not_enabled():assert a.LIVE_PROFILE['enabled'] is False


def test_session_rejects_physical_flag_before_ros(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','197');monkeypatch.setenv('ROS_LOCALHOST_ONLY','1')
    with pytest.raises(ValueError,match='Nonphysical'):a.verify_session({'schema':'unified_controller_session_v1','physical_execution':True})


def test_session_requires_pinned_agent_code(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','197');monkeypatch.setenv('ROS_LOCALHOST_ONLY','1')
    s=dict(schema='unified_controller_session_v1',physical_execution=False,namespace='/unified_replay_lab_aabbccddeeff',profile='isolated_controller',agent_hashes={'api.py':'0'*64})
    with pytest.raises(ValueError,match='version mismatch'):a.verify_session(s)
