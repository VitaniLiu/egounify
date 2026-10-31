"""Wire contract and real ROS message encoders, independent of ROS imports."""
import hashlib
import json
import math
import os
import re
from pathlib import Path

DOMAIN = 197
PREFIX = '@UNIFIED_REPLAY '
POSE_FIELDS = dict(left='left_hand_pose', right='right_hand_pose', left_elbow='left_elbow_pose',
    right_elbow='right_elbow_pose', torso='torso_pose', head='head_pose', pelvis='pelvis_pose',
    left_foot='left_foot_pose', right_foot='right_foot_pose', left_knee='left_knee_pose', right_knee='right_knee_pose')
HAND_NAMES = 'thumb_CMC_FE thumb_CMC_AA thumb_MCP_FE thumb_MCP_AA thumb_IP index_MCP_FE index_MCP_AA index_PIP index_DIP middle_MCP_FE middle_MCP_AA middle_PIP middle_DIP ring_MCP_FE ring_MCP_AA ring_PIP ring_DIP pinky_CMC pinky_MCP_FE pinky_MCP_AA pinky_PIP pinky_DIP'.split()
SERIALS = {'left':'CE50903ECE53','right':'CC50933FCC53'}
GROUPS = {'A':[f'Joint{i}_L' for i in range(1,8)],'B':[f'Joint{i}_R' for i in range(1,8)],
          'body':['waist_joint_1','waist_joint_2','waist_joint_3'],'head':['Head_J1_Joint','Head_J2_Joint']}
ROLES = {'robot':('body','chassis'),'hands':('left_hand','right_hand')}
LIVE_PROFILE = {'body_domain':29,'hand_domain':198,'chassis_domain':0,
    'body_topic':'/tj/control/ik_request/replay','body_mux':'/tj/control/ik_request_mux/select',
    'hand_topic':'/sharpa/{side}/request','hand_result':'/sharpa/{side}/command_result',
    'chassis_topic':'/move/ManualMoveCmd','enabled':False}


def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def lab_guard(namespace, profile='isolated_controller'):
    if profile!='isolated_controller':raise PermissionError('Live execution is not commissioned')
    if os.environ.get('ROS_DOMAIN_ID')!=str(DOMAIN) or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise PermissionError('Integration requires domain197 localhost only')
    if not re.fullmatch('/unified_replay_lab_[0-9a-f]{12}',namespace):raise PermissionError('Unsafe integration namespace')
    return namespace


def verify_session(session):
    if session.get('schema')!='unified_controller_session_v1' or session.get('physical_execution') is not False:
        raise ValueError('Nonphysical integration session required')
    lab_guard(session['namespace'],session['profile'])
    if not session.get('agent_hashes'):raise ValueError('Agent version binding missing')
    for name,sha in session['agent_hashes'].items():
        if Path(name).name!=name or name not in ('api.py','agent.py','fixtures.py','ros_adapter.py','native_qp_parameters.yaml'):raise ValueError('Invalid agent file binding')
        if hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest()!=sha:raise ValueError('Agent version mismatch:'+name)
    plan=session['plan']
    if plan.get('hardware_execution_allowed') is not False or plan.get('backend')!='mock_only':raise ValueError('Hardware plan rejected')
    if plan['plan_sha256']!=digest({k:v for k,v in plan.items() if k!='plan_sha256'}):raise ValueError('Plan hash mismatch')
    if not plan.get('diagnostic'):raise ValueError('Only diagnostic integration fixtures allowed')
    if set(plan.get('channels',{}))!={'body','chassis','left_hand','right_hand'} or type(plan.get('duration_ns')) is not int or not 0<plan['duration_ns']<=120_000_000_000:raise ValueError('Incomplete or unbounded integration plan')
    if plan.get('gripper_commands_enabled') is not False:raise ValueError('Gripper output forbidden')
    if session['session_sha256']!=digest({k:v for k,v in session.items() if k!='session_sha256'}):raise ValueError('Session hash mismatch')
    for channel,events in plan['channels'].items():
        prev=-1
        for event in events:
            t=event['offset_ns']
            if type(t) is not int or not prev<t<=plan['duration_ns'] or event['payload_sha256']!=digest(event['payload']):
                raise ValueError('Corrupt event')
            prev=t
        if not events or events[0]['offset_ns']!=0:raise ValueError('Missing initial target')
    return plan


def quaternion(pose):
    if len(pose)!=9 or not all(math.isfinite(v) for v in pose):raise ValueError('Invalid pose')
    x=pose[3:6];y=pose[6:9]
    if abs(sum(v*v for v in x)-1)>5e-5 or abs(sum(v*v for v in y)-1)>5e-5 or abs(sum(a*b for a,b in zip(x,y)))>5e-5:
        raise ValueError('Invalid rot6d')
    z=[x[1]*y[2]-x[2]*y[1],x[2]*y[0]-x[0]*y[2],x[0]*y[1]-x[1]*y[0]]
    r=[[x[i],y[i],z[i]] for i in range(3)];trace=sum(r[i][i] for i in range(3))
    if trace>0:
        s=math.sqrt(trace+1)*2;q=[(r[2][1]-r[1][2])/s,(r[0][2]-r[2][0])/s,(r[1][0]-r[0][1])/s,s/4]
    else:
        i=max(range(3),key=lambda k:r[k][k]);j=(i+1)%3;k=(i+2)%3;s=math.sqrt(1+r[i][i]-r[j][j]-r[k][k])*2
        q=[0.,0.,0.,0.];q[i]=s/4;q[j]=(r[j][i]+r[i][j])/s;q[k]=(r[k][i]+r[i][k])/s;q[3]=(r[k][j]-r[j][k])/s
    return q


def fill_ik(message, payload, stamp):
    if set(payload)!={*POSE_FIELDS,'available','joint_positions'}:raise ValueError('Incomplete IK payload')
    if len(payload['available'])!=11 or len(payload['joint_positions'])!=19:raise ValueError('IK shape')
    if not all(math.isfinite(v) for v in payload['joint_positions']):raise ValueError('IK seed nonfinite')
    message.header.stamp=stamp;message.header.frame_id='base_link'
    for i,(key,field) in enumerate(POSE_FIELDS.items()):
        v=payload[key];q=quaternion(v);p=getattr(message,field)
        p.position.x,p.position.y,p.position.z=map(float,v[:3]);p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=q
        if payload['available'][i] not in (0,1):raise ValueError('IK availability')
        message.available[i]=bool(payload['available'][i])
    message.joint_state.position=list(map(float,payload['joint_positions']))
    return message


def fill_hand(message, values, stamp, token):
    if len(values)!=22 or not all(math.isfinite(v) for v in values):raise ValueError('Hand shape/nonfinite')
    message.header.stamp=stamp;message.header.frame_id=token;message.name=list(HAND_NAMES);message.position=list(map(float,values));return message


def fill_chassis(message, values, stamp):
    if len(values)!=3 or not all(math.isfinite(v) for v in values) or abs(values[0])>.400001 or abs(values[1])>1e-8 or abs(values[2])>.500001:
        raise ValueError('Chassis bounds')
    message.header.stamp=stamp;message.header.frame_id='base_link'
    message.twist.linear.x=float(values[0]);message.twist.linear.y=float(values[1]);message.twist.angular.z=float(values[2]);return message


def check_hand_receipt(receipt, stamp, payload):
    if receipt.get('schema')!='sharpa_command_result_v2' or receipt.get('command_stamp_ns')!=stamp:raise ValueError('Receipt identity mismatch')
    if receipt.get('status')!='sdk_call_succeeded':raise ValueError('SDK rejected command')
    if receipt.get('clamped_joints') or receipt.get('target')!=list(payload) or receipt.get('names')!=HAND_NAMES:raise ValueError('SDK altered target')
