"""Robot-side bounded IK execution; trajectory arrives over SSH stdin only.

No recording/file reader. Uses live JointState and robot_description for FK.
"""
import json
import sys
import time
import threading
import signal
import math
import re
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def rot(axis, angle):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    x,y,z = axis
    k = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(angle)*k+(1-math.cos(angle))*(k@k)


class Model:
    def __init__(self, xml):
        self.joints = {}
        for j in ET.fromstring(xml).findall('joint'):
            def vector(e, key, default):
                v=j.find(e)
                return np.array([float(x) for x in (v.get(key,default) if v is not None else default).split()])
            origin=np.eye(4);origin[:3,3]=vector('origin','xyz','0 0 0')
            r,p,y=vector('origin','rpy','0 0 0')
            origin[:3,:3]=rot([0,0,1],y)@rot([0,1,0],p)@rot([1,0,0],r)
            lim=j.find('limit')
            self.joints[j.find('child').get('link')]=(j.find('parent').get('link'),j.get('name'),j.get('type'),origin,vector('axis','xyz','1 0 0'),
                None if lim is None else (float(lim.get('lower','-inf')),float(lim.get('upper','inf'))))

    def fk(self, link, positions):
        if link=='base_link':return np.eye(4)
        parent,name,kind,origin,axis,limits=self.joints[link]
        motion=np.eye(4)
        if kind!='fixed':
            value=positions[name]
            if not math.isfinite(value):raise ValueError('Nonfinite joint '+name)
            if limits and not limits[0]-1e-5<=value<=limits[1]+1e-5:raise ValueError('Joint limit '+name)
            if kind in ('revolute','continuous'):motion[:3,:3]=rot(axis,value)
            elif kind=='prismatic':motion[:3,3]=axis*value
            else:raise ValueError('Unsupported joint '+kind)
        return self.fk(parent,positions)@origin@motion


class ControllerModel:
    """FK of the installed QP MJCF; meshes are never opened.

    Support only this verified local, radian, zero joint-anchor model.
    Reject richer MJCF constructs rather than silently approximate them.
    """
    def __init__(self, xml):
        root=ET.fromstring(xml)
        compiler=root.find('compiler')
        if compiler is None or compiler.get('angle')!='radian':raise ValueError('MJCF radians required')
        self.bodies={}
        def walk(body,parent):
            if any(key in body.attrib for key in ('euler','axisangle','xyaxes','zaxis')):raise ValueError('Unsupported MJCF orientation')
            origin=np.eye(4);origin[:3,3]=np.fromstring(body.get('pos','0 0 0'),sep=' ')
            w,x,y,z=np.fromstring(body.get('quat','1 0 0 0'),sep=' ')
            norm=math.sqrt(w*w+x*x+y*y+z*z);w,x,y,z=[v/norm for v in (w,x,y,z)]
            origin[:3,:3]=np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
            joints=body.findall('joint')
            if len(joints)>1 or body.find('freejoint') is not None:raise ValueError('Unsupported MJCF joints')
            j=joints[0] if joints else None
            if j is not None and (j.get('type','hinge') not in ('hinge','slide') or float(j.get('ref','0'))!=0 or np.any(np.fromstring(j.get('pos','0 0 0'),sep=' '))):raise ValueError('Unsupported joint anchor/ref')
            self.bodies[body.get('name')]=(parent,origin,j)
            for child in body.findall('body'):walk(child,body.get('name'))
        for body in root.find('worldbody').findall('body'):walk(body,None)

    def fk(self,link,positions):
        def world(name):
            parent,origin,j=self.bodies[name];motion=np.eye(4)
            if j is not None:
                v=positions[j.get('name')];limits=np.fromstring(j.get('range','-inf inf'),sep=' ')
                if not math.isfinite(v) or not limits[0]-1e-5<=v<=limits[1]+1e-5:raise ValueError('Joint limit '+j.get('name'))
                axis=np.fromstring(j.get('axis','0 0 1'),sep=' ')
                if j.get('type','hinge')=='slide':motion[:3,3]=axis/np.linalg.norm(axis)*v
                else:motion[:3,:3]=rot(axis,v)
            return (np.eye(4) if parent is None else world(parent))@origin@motion
        return np.linalg.inv(world('base_link'))@world(link)


def angle(a,b):
    return math.acos(float(np.clip((np.trace(a[:3,:3].T@b[:3,:3])-1)/2,-1,1)))


def quat(r):
    # Stable conversion, including 180-degree rotations.
    x,y,z=r[:,0],r[:,1],r[:,2]
    k=np.array([[x[0]-y[1]-z[2],y[0]+x[1],z[0]+x[2],y[2]-z[1]],
        [y[0]+x[1],y[1]-x[0]-z[2],z[1]+y[2],z[0]-x[2]],
        [z[0]+x[2],z[1]+y[2],z[2]-x[0]-y[1],x[1]-y[0]],
        [y[2]-z[1],z[0]-x[2],x[1]-y[0],np.trace(r)]])/3
    return np.linalg.eigh(k)[1][:,-1]


def home_feedback_tolerance(plan):
    value=plan.get('test_home_feedback_tolerance_rad',0.01)
    if type(value) not in (int,float) or value not in (0.01,0.015):
        raise ValueError('Unsupported test home feedback tolerance')
    full=plan.get('full_trajectory_test',False)
    if type(full) is not bool:raise ValueError('Explicit boolean full trajectory test required')
    if full and (value!=0.015 or len(plan['offset_ns'])!=314):
        raise ValueError('Full trajectory test requires exactly 314 frames and 0.015 rad')
    if value!=0.01 and not full and not 2<=len(plan['offset_ns'])<=6:
        raise ValueError('Test home feedback tolerance requires at most six frames')
    return float(value)


IK_FIELDS = tuple(zip(
    ('left', 'right', 'left_elbow', 'right_elbow', 'torso', 'head', 'pelvis',
     'left_foot', 'right_foot', 'left_knee', 'right_knee'), range(11),
    ('left_hand_pose', 'right_hand_pose', 'left_elbow_pose', 'right_elbow_pose',
     'torso_pose', 'head_pose', 'pelvis_pose', 'left_foot_pose', 'right_foot_pose',
     'left_knee_pose', 'right_knee_pose')))


def frame_target(plan, index, model, positions, groups):
    """Reconstruct recorded IK inputs independently of transport and feedback."""
    if plan['replay_contract']['schema'] == 'skye_recorded_ik_targets_v3':
        return {k: np.asarray(v[index], float) for k,v in plan['ik_reference'].items()}
    q = dict(positions)
    q.update(zip(groups['body'], plan['body'][index]))
    q.update(zip(groups['head'], plan['head'][index]))
    return dict(left=np.array(plan['left'][index]), right=np.array(plan['right'][index]),
                torso=model.fk('torso_link', q), head=model.fk('eyes_link', q))


def robot_goal(plan, index, model, positions, groups):
    """Robot-space goals, distinct from native request-space references."""
    q = dict(positions)
    q.update(zip(groups['body'], plan['body'][index]))
    if 'head' in plan: q.update(zip(groups['head'], plan['head'][index]))
    return dict(left=np.asarray(plan['left'][index]), right=np.asarray(plan['right'][index]),
                torso=model.fk('torso_link',q), head=model.fk('eyes_link',q), pelvis=np.eye(4))


def arm_reachability(plan, model, positions, groups):
    """Necessary reach bound, independent of IK convergence and joint costs.

    Remove the fixed tool transform to locate the requested last-joint origin.
    Its distance from the first-joint origin cannot exceed the sum of intervening
    link lengths, even without joint limits. This does not certify collision or
    full IK feasibility when the bound passes.
    """
    trajectory = plan.get('validation_trajectory') or plan
    result = {'frames':len(trajectory['body']), 'scope':'Necessary wrist reach bound at recorded body goals; not a sufficient IK/collision test', 'arms':{}}
    for side, suffix, slot in [('left','L',0), ('right','R',1)]:
        fixed = np.linalg.inv(model.fk('Link7_'+suffix, positions)) @ model.fk(side+'_tool', positions)
        length = sum(float(np.linalg.norm(model.bodies['Link'+str(i)+'_'+suffix][1][:3,3])) for i in range(2,8))
        rows = []
        for i, available in enumerate(trajectory['available']):
            if not available[slot]: continue
            q = dict(positions); q.update(zip(groups['body'], trajectory['body'][i]))
            wrist = np.asarray(trajectory[side][i]) @ np.linalg.inv(fixed)
            shoulder = model.fk('Link1_'+suffix, q)
            distance = float(np.linalg.norm(wrist[:3,3] - shoulder[:3,3]))
            rows.append(dict(frame=i, requested_distance_m=distance, excess_m=max(0.,distance-length)))
        worst = max(rows, key=lambda row:row['excess_m'])
        result['arms'][side] = dict(reach_upper_bound_m=length, worst=worst,
            unreachable_frames=[row['frame'] for row in rows if row['excess_m'] > 1e-6])
    result['passed'] = not any(value['unreachable_frames'] for value in result['arms'].values())
    return result


def robot_target_frames(plan, model, positions, groups, qp):
    """Convert robot controller actions to requests before native support subtraction.

    Raw reference keypoints are retained. Source 3 in stationary mode applies the
    filtered foot translation before constructing torso-relative hand tasks.
    This inverse translation makes the effective targets robot-base poses.
    """
    if (plan['replay_contract']['schema'] != 'skye_recorded_ik_targets_v3'
            or not qp.get('support_frame_enabled')
            or qp.get('support_frame_source') != 'foot_midpoint'
            or qp.get('base_closure_enabled')):
        raise ValueError('Unverified robot target conversion configuration')
    alpha = qp.get('support_frame_filter_alpha')
    if not isinstance(alpha, (int,float)) or not 0 < alpha <= 1:
        raise ValueError('Invalid support origin filter')
    origin = None
    frames = []
    for index, available in enumerate(plan['available']):
        target = frame_target(plan, index, model, positions, groups)
        if available[7] and available[8]:
            reference = (target['left_foot'][:3,3] + target['right_foot'][:3,3]) / 2
            if not qp.get('support_frame_use_z'): reference[2] = 0
            origin = reference.copy() if origin is None else alpha*reference+(1-alpha)*origin
        if origin is None: raise ValueError('Support origin cannot be established from recorded feet')
        desired = robot_goal(plan, index, model, positions, groups)
        for key, matrix in desired.items():
            target[key] = matrix.copy()
            target[key][:3,3] += origin
        frames.append(target)
    return frames


def fill_ik_request(message, target, available):
    """Shared serializer for isolated and physical replay; no reference synthesis."""
    message.header.frame_id = 'base_link'
    for key, slot, field in IK_FIELDS:
        if key not in target:
            if available[slot]:
                raise ValueError('Missing active IK field: ' + key)
            continue
        a = target[key]; pose = getattr(message, field)
        pose.position.x, pose.position.y, pose.position.z = map(float, a[:3,3])
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, quat(a[:3,:3]))
        message.available[slot] = bool(available[slot])
    if 'joint_positions' in target:
        message.joint_state.position = list(map(float, target['joint_positions']))
    return message


def validate_plan(plan):
    if plan.get('schema')!='skye_lerobot_client_v1':raise ValueError('Wrong schema')
    if type(plan.get('execute')) is not bool:raise ValueError('Explicit boolean execute required')
    ts=plan['offset_ns'];n=len(ts)
    home_feedback_tolerance(plan)
    if not 2<=n<=2400 or ts[0]!=0 or any(type(t) is not int for t in ts):raise ValueError('Invalid timeline')
    if any(b-a!=50_000_000 for a,b in zip(ts,ts[1:])):raise ValueError('Requires exact 20 Hz')
    for key in ('left','right'):
        a=np.asarray(plan[key],float)
        if a.shape!=(n,4,4) or not np.isfinite(a).all():raise ValueError('Invalid pose')
        if not np.allclose(a[:,3],[0,0,0,1],atol=1e-8):raise ValueError('Invalid homogeneous pose')
        r=a[:,:3,:3]
        if not np.allclose(r.transpose(0,2,1)@r,np.eye(3),atol=2e-5) or not np.allclose(np.linalg.det(r),1,atol=2e-5):raise ValueError('Invalid rotation')
        if np.max(np.linalg.norm(np.diff(a[:,:3,3],axis=0),axis=1))>.08:raise ValueError('Arm target jumps >8cm/frame')
    for key,width in [('body',3),('head',2)]:
        a=np.asarray(plan[key],float)
        if a.shape!=(n,width) or not np.isfinite(a).all():raise ValueError('Invalid controller target')
        if np.max(np.abs(np.diff(a,axis=0)))>.10:raise ValueError('Joint target jumps >0.1/frame')
    if plan.get('ignored_slots')!=list(range(32,78)):raise ValueError('Hand/gripper exclusion required')
    gate=np.array(plan.get('available'))
    if gate.shape!=(n,11) or gate.dtype!=bool:raise ValueError('Recorded IK availability required')
    if not gate[:,0].any() or not np.array_equal(gate[:,0],gate[:,1]):raise ValueError('Paired arms gate required')
    if plan.get('replay_contract',{}).get('schema') not in {'skye_recorded_ik_mapping_v1', 'skye_recorded_ik_targets_v2', 'skye_recorded_ik_targets_v3'}:raise ValueError('Verified replay provenance required')
    contract = plan['replay_contract']
    if contract['schema'] == 'skye_recorded_ik_targets_v2' and (contract.get('input_frame') != 'base_link' or contract.get('output_frame') != 'base_link' or contract.get('body_head_policy') != 'recorded_controller_FK' or contract.get('source_topic') != '/tj/control/ik_request'):
        raise ValueError('Invalid recorded IK contract')
    if contract['schema'] == 'skye_recorded_ik_targets_v3':
        if plan.get('execution_target_policy') != 'robot_controller_fk_inverse_support_v1':
            raise ValueError('Explicit effective target policy required')
        keys = [key for key, _, _ in IK_FIELDS]
        if (contract.get('pose_fields') != keys or contract.get('body_head_policy') != 'recorded_ik'
                or contract.get('input_frame') != 'base_link' or contract.get('output_frame') != 'base_link'
                or contract.get('source_topic') != '/tj/control/ik_request' or contract.get('joint_state_names') != []):
            raise ValueError('Invalid complete IK contract')
        refs = plan.get('ik_reference', {})
        if set(refs) != set(keys + ['joint_positions']):
            raise ValueError('Incomplete recorded IK references')
        for key in keys:
            a = np.asarray(refs[key], float)
            if (a.shape != (n,4,4) or not np.isfinite(a).all()
                    or not np.allclose(a[:,3], [0,0,0,1], atol=1e-8)
                    or not np.allclose(a[:,:3,:3].transpose(0,2,1) @ a[:,:3,:3], np.eye(3), atol=2e-5)
                    or not np.allclose(np.linalg.det(a[:,:3,:3]), 1, atol=2e-5)):
                raise ValueError('Invalid IK reference pose: ' + key)
        for key in ('left', 'right'):
            if not np.array_equal(np.asarray(refs[key]), np.asarray(plan[key])):
                raise ValueError('Conflicting IK arm targets')
        validation = plan.get('validation_trajectory', {})
        if set(validation) != {'left','right','body','available'}:
            raise ValueError('Full source trajectory validation required')
        total = len(validation['body'])
        if not n <= total <= 2400: raise ValueError('Invalid full validation length')
        for key, shape in [('left',(total,4,4)),('right',(total,4,4)),('body',(total,3)),('available',(total,11))]:
            values = np.asarray(validation[key])
            if values.shape != shape or not np.isfinite(values).all():
                raise ValueError('Malformed full validation trajectory')
            if not np.array_equal(values[:n], np.asarray(plan[key])):
                raise ValueError('Selected trajectory differs from full validation trajectory')
        seed = np.asarray(refs['joint_positions'], float)
        if seed.shape != (n,19) or not np.isfinite(seed).all():
            raise ValueError('Invalid IK seed')
    return n



def validate_reference_inputs(plan, qp):
    """Reject incomplete recorded IK requests when QP needs omitted references."""
    if plan.get('replay_contract', {}).get('schema') != 'skye_recorded_ik_targets_v2':
        return
    if not qp.get('support_frame_enabled'):
        return
    source = qp.get('support_frame_source')
    slots = {'foot_midpoint': (7, 8), 'pelvis': (6,), 'none': ()}.get(source)
    if slots is None:
        raise RuntimeError('Unverified QP support-frame source')
    if any(row[slot] for row in plan['available'] for slot in slots):
        raise RuntimeError('LeRobot lacks recorded IK support-reference poses (' + str(source) +
                           '); active flags cannot substitute for missing targets')


def active_command(group, values):
    width, active = {'A': (7, 7), 'B': (7, 7), 'body': (6, 3), 'head': (3, 2)}[group]
    if len(values) != width or not all(math.isfinite(x) for x in values):
        raise ValueError('Malformed controller output ' + group)
    # Native fixed-width messages reserve unused body/head channels.
    if any(abs(x) > 1e-10 for x in values[active:]):
        raise ValueError('Unexpected unused controller channels ' + group)
    return values[:active]


def main():
    plan=json.loads(sys.stdin.readline());n=validate_plan(plan)
    alive={'time':time.monotonic(),'eof':False}
    def heartbeat():
        for line in sys.stdin:
            if line.strip()=='heartbeat':alive['time']=time.monotonic()
        alive['eof']=True
    threading.Thread(target=heartbeat,daemon=True).start()
    import rclpy
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int32,Int16MultiArray,Bool,String
    from marvin_msgs.msg import IKRequest,JointcmdArm,JointcmdBody,JointcmdHead
    from marvin_msgs.srv import Int
    from std_srvs.srv import Trigger
    rclpy.init(domain_id=29);node=rclpy.create_node('lerobot_replay_client')
    state={};commands={};qp_commands={};command_counts={};samples=[];attempted=False;initial_source=None
    report={'source':plan['source'],'source_sha256':plan['source_sha256'],'robot_commands_sent':0,'status':'preflight',
        'hand_gripper_commands_sent':0,'chassis_commands_sent':0,'frames':n,
        'home_feedback_tolerance_rad':home_feedback_tolerance(plan),
        'home_command_tolerance_rad':0.01,'home_lift_tolerance_m':0.002,
        'temporary_test_configuration':home_feedback_tolerance(plan)!=0.01,
        'head_execution':'FK of recorded action.head_controller, eyes_link; not simultaneous VR head target',
        'body_execution':'FK of recorded action.body_controller, torso_link',
        'raw_or_hdf5_read':False,'execution_ready':False,
        'selected_frame_range':plan.get('selected_frame_range',[0,n]),
        'source_total_frames':plan.get('source_total_frames',n)}
    def feedback(m):
        state.update(joints=m,joint_time=time.monotonic())
    node.create_subscription(JointState,'/tj/joint_states',feedback,qos_profile_sensor_data)
    transient=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for label in ('ik_request_mux','joint_cmd_mux'):
        node.create_subscription(Int32,'/tj/info/'+label+'/active_source',lambda m,k=label:state.update({k:m.data}),transient)
    for label, topic in [('body_pose_mode','/tj/info/body_pose_mode'),
                         ('teleop_motion_mode','/tj/info/teleop_motion_mode'),
                         ('base_mode','/info/base_teleop/active_mode')]:
        node.create_subscription(Int32, topic, lambda m,k=label: state.update({k:m.data}), transient)
    node.create_subscription(Bool,'/tj/info/vr_connected',lambda m:state.update(vr=m.data),qos_profile_sensor_data)
    node.create_subscription(Int16MultiArray,'/tj/info/robot_state',lambda m:state.update(robot_state=list(m.data),robot_state_time=time.monotonic()),qos_profile_sensor_data)
    node.create_subscription(Bool,'/tj/info/collision_statusA',lambda m:state.update(collision_A=m.data,collision_A_time=time.monotonic()),qos_profile_sensor_data)
    node.create_subscription(Bool,'/tj/info/collision_statusB',lambda m:state.update(collision_B=m.data,collision_B_time=time.monotonic()),qos_profile_sensor_data)
    node.create_subscription(String,'/tj/info/gento_replay/status',lambda m:state.update(native=json.loads(m.data),native_time=time.monotonic()),10)
    groups={'A':[f'Joint{i}_L' for i in range(1,8)],'B':[f'Joint{i}_R' for i in range(1,8)],
        'body':['waist_joint_1','waist_joint_2','waist_joint_3'],'head':['Head_J1_Joint','Head_J2_Joint']}
    def command(m, group, stage):
        key=stage+'/'+group;command_counts[key]=command_counts.get(key,0)+1
        (commands if stage=='mux' else qp_commands)[group]=(time.monotonic(),list(m.positions))
    for group,typ in [('A',JointcmdArm),('B',JointcmdArm),('body',JointcmdBody),('head',JointcmdHead)]:
        node.create_subscription(typ,'/tj/control/joint_cmd_'+group,lambda m,k=group:command(m,k,'mux'),qos_profile_sensor_data)
        node.create_subscription(typ,'/tj/control/qp_controller/joint_cmd_'+group,lambda m,k=group:command(m,k,'qp'),qos_profile_sensor_data)
    pub=node.create_publisher(IKRequest,'/tj/control/ik_request/replay',qos_profile_sensor_data)
    client=node.create_client(Int,'/tj/control/ik_request_mux/select')
    backend_stop=threading.Event()
    def backend_watch():
        import urllib.request
        while not backend_stop.wait(.05):
            try:
                payload=json.loads(urllib.request.urlopen('http://127.0.0.1:8080/api/v1/robot/status',timeout=.15).read())
                state.update(backend=payload,backend_time=time.monotonic(),backend_error=None)
            except BaseException as e:state.update(backend_error=str(e))
    threading.Thread(target=backend_watch,daemon=True).start()
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.005)
    def call(c,req):
        if not c.wait_for_service(timeout_sec=2):raise RuntimeError('Service unavailable '+c.srv_name)
        f=c.call_async(req);end=time.monotonic()+3
        while not f.done() and time.monotonic()<end:spin(.005)
        if not f.done():raise RuntimeError('Service timeout '+c.srv_name)
        return f.result()
    def params(name,keys):
        c=node.create_client(GetParameters,name+'/get_parameters');r=GetParameters.Request();r.names=keys
        v=call(c,r).values;node.destroy_client(c)
        return {k:(x.bool_value if x.type==1 else x.integer_value if x.type==2 else x.double_value if x.type==3 else x.string_value if x.type==4 else list(x.string_array_value) if x.type==9 else None) for k,x in zip(keys,v)}
    def select(value):
        r=Int.Request();r.data=value;response=call(client,r)
        if not response.success:raise RuntimeError('IK mux rejected: '+response.message)
        end=time.monotonic()+2
        while state.get('ik_request_mux')!=value and time.monotonic()<end:spin(.01)
        if state.get('ik_request_mux')!=value:raise RuntimeError('IK mux transition not confirmed')
    def current():return dict(zip(state['joints'].name,state['joints'].position))
    def check(owned=False):
        if alive['eof'] or time.monotonic()-alive['time']>.5:raise RuntimeError('Host heartbeat lost')
        if time.monotonic()-state.get('joint_time',0)>.2:raise RuntimeError('Joint feedback stale')
        m=state['joints']
        if len(m.name)!=19 or len(m.position)!=19 or len(m.velocity)!=19 or not all(math.isfinite(v) for v in [*m.position,*m.velocity]):raise RuntimeError('Invalid feedback')
        if time.monotonic()-state.get('robot_state_time',0)>.2:raise RuntimeError('Robot mode feedback stale')
        if state.get('robot_state')!=[2,2,1,1,1,0]:raise RuntimeError('Robot mode/fault state changed: '+repr(state.get('robot_state')))
        for side in ('A','B'):
            # motion_node publishes collision_status as an event, not a heartbeat.
            # Absence of a message is unknown; an observed true event is fatal.
            if state.get('collision_'+side) is True:raise RuntimeError('Collision status active '+side)
        if time.monotonic()-state.get('backend_time',0)>.3:raise RuntimeError('Backend health stale: '+repr(state.get('backend_error')))
        backend=state['backend']
        if not backend.get('ok') or not backend.get('data',{}).get('control_ready') or backend['data'].get('control_error'):
            raise RuntimeError('Backend control fault: '+json.dumps(backend))
        velocity_limits={'waist_joint_1':1.0,**{name:3.14 for name in m.name if name!='waist_joint_1'}}
        excess={name:abs(v) for name,v in zip(m.name,m.velocity) if abs(v)>velocity_limits[name]+1e-6}
        if excess:raise RuntimeError('Measured velocity limit exceeded: '+json.dumps(excess))
        if any(state.get(key) != 0 for key in ('body_pose_mode','teleop_motion_mode','base_mode')):
            raise RuntimeError('Controller modes differ from isolated validation')
        if state.get('vr') is not False:raise RuntimeError('VR connected or unknown')
        if state.get('joint_cmd_mux')!=0:raise RuntimeError('QP joint output is not selected')
        if owned and state.get('ik_request_mux')!=3:raise RuntimeError('IK ownership changed')
        if owned and n <= 6:
            anchor = report['replay_start_positions']
            if any(abs(value-anchor[name]) > (.005 if name=='waist_joint_1' else .10) for name,value in zip(m.name,m.position)):
                raise RuntimeError('Short replay cumulative displacement limit exceeded')
        if time.monotonic()-state.get('native_time',0)>2:raise RuntimeError('Native replay status stale')
        if state['native'].get('playback_active') or state['native'].get('recording'):raise RuntimeError('Native replay/recording active')
    def publish(target,available):
        m=IKRequest();m.header.frame_id='base_link';m.header.stamp=node.get_clock().now().to_msg()
        fill_ik_request(m, target, available)
        pub.publish(m);report['robot_commands_sent']+=1
    def interrupt(*_):raise KeyboardInterrupt('Interrupted')
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    try:
        spin(5);check();initial_source=state.get('ik_request_mux')
        if initial_source not in (0,1):raise RuntimeError('IK source already occupied')
        if max(abs(x) for x in state['joints'].velocity)>.02:raise RuntimeError('Robot is moving')
        qp=params('/tj/qp_controller',['use_incremental_hand_targets','ik_request_timeout','ik_request_default_frame','ik_tasks_file','publish_joint_cmd','support_frame_enabled','support_frame_source','support_frame_filter_alpha','support_frame_use_z','support_frame_use_yaw','base_closure_enabled','joint_cmd_output_ramp_duration'])
        report['qp_parameters']=qp
        report['controller_modes']={key:state.get(key) for key in ('body_pose_mode','teleop_motion_mode','base_mode')}
        if plan['replay_contract']['schema'] == 'skye_recorded_ik_targets_v3':
            report['head_execution']='recorded IK head reference'
            report['body_execution']='recorded IK torso and pelvis references'
        validate_reference_inputs(plan, qp)
        mux=params('/tj/ik_request_mux',['source_topics','clear_target_cache_on_switch','publish_cached_on_switch','publish_disable_on_switch'])
        if mux!={'source_topics':['','control/ik_request/vr','control/ik_request/vla','control/ik_request/replay'],'clear_target_cache_on_switch':True,'publish_cached_on_switch':False,'publish_disable_on_switch':True}:raise RuntimeError('Unverified mux configuration')
        if qp['ik_tasks_file']!='skye/ik_tasks.yaml' or qp['ik_request_default_frame']!='base_link' or not qp['publish_joint_cmd'] or not 0<qp['ik_request_timeout']<=.5:raise RuntimeError('Unsupported IK configuration')
        xml=params('/tj/robot_state_publisher',['robot_description'])['robot_description'];urdf=Model(xml)
        model_path=Path('/opt/kernelmind/apex/install/marvin_description/share/marvin_description/mjcf/Gento_Sky/Gento_Sky.xml')
        model_xml=model_path.read_text();model=ControllerModel(model_xml)
        report['controller_model_sha256']=hashlib.sha256(model_xml.encode()).hexdigest()
        report['kinematic_config_source']=str(model_path)
        positions=current();initial=dict(positions);targets=[]
        comparisons={}
        for link in ('eyes_link','flange_L','flange_R'):
            if link in urdf.joints and link in model.bodies:
                comparisons[link]=float(np.max(np.abs(urdf.fk(link,positions)-model.fk(link,positions))))
        report['urdf_mjcf_fk_errors']=comparisons
        if not comparisons or max(comparisons.values())>1e-4:raise RuntimeError('Live URDF and controller model disagree')
        goals = None
        if plan['replay_contract']['schema'] == 'skye_recorded_ik_targets_v3':
            targets = robot_target_frames(plan, model, positions, groups, qp)
            goals = [robot_goal(plan, i, model, positions, groups) for i in range(n)]
            report['reachability'] = arm_reachability(plan, model, positions, groups)
            report['head_execution'] = 'recorded head controller FK; inverse native support translation'
            report['body_execution'] = 'recorded body controller FK; inverse native support translation'
        else:
            targets = [frame_target(plan, i, model, positions, groups) for i in range(n)]
        if goals is None: goals = targets
        actual={k:model.fk(link,positions) for k,link in [('left','left_tool'),('right','right_tool'),('torso','torso_link'),('head','eyes_link')]}
        report['initial_errors']={k:{'translation_m':float(np.linalg.norm(actual[k][:3,3]-goals[0][k][:3,3])),'rotation_rad':angle(actual[k],goals[0][k])} for k in actual}
        blockers=[]
        if report.get('reachability',{}).get('passed') is False:
            blockers.append('Full trajectory exceeds the necessary arm wrist reach bound at recorded body goals')
        if plan['execute'] and plan['replay_contract']['schema'] == 'skye_recorded_ik_targets_v3':
            blockers.append('Effective-target conversion has not passed full offline and isolated validation; motion disabled')
        report['mapping_semantics']='robot-base arm goals and recorded body/head controller FK; inverse native support translation; original references retained'
        report['active_frames']=sum(bool(x[0]) for x in plan['available'])
        if any(v['translation_m']>.03 or v['rotation_rad']>.15 for v in report['initial_errors'].values()):blockers.append('Start pose differs >3cm/0.15rad; requires a separately verified start-pose transition')
        report['blockers']=blockers;report['mapping_preflight_passed']=not blockers
        if not plan['execute']:
            report['status']='preflight_passed' if not blockers else 'preflight_blocked'
        else:
            if blockers:raise RuntimeError('; '.join(blockers))
            if pub.get_subscription_count()<1:raise RuntimeError('No IK mux subscriber')
            # The native controller requires an explicit home initialization.
            # Only allow it when already close to its configured home, never a
            # blind long-distance recovery trajectory.
            key=ET.fromstring(model_xml).find("keyframe/key[@name='home']")
            if key is None:raise RuntimeError('Controller home definition missing')
            joint_order=[j.get('name') for j in ET.fromstring(model_xml).findall('.//worldbody//joint')]
            home_values=np.fromstring(key.get('qpos'),sep=' ')
            if len(joint_order)!=19 or len(home_values)!=19:raise RuntimeError('Unverified home joint mapping')
            home=dict(zip(joint_order,home_values));before=current()
            home_errors={k:abs(before[k]-float(v)) for k,v in home.items()}
            report['home_initialization_errors']=home_errors
            if any(e>(.005 if k=='waist_joint_1' else .05) for k,e in home_errors.items()):raise RuntimeError('Too far from home for automatic initialization')
            attempted=True
            report['home_attempts']=[];report['home_samples']=[]
            for home_attempt in range(1):
                home_started=time.monotonic()
                home_client=node.create_client(Trigger,'/tj/control/go_home')
                home_response=call(home_client,Trigger.Request());node.destroy_client(home_client)
                report['home_response']={'success':home_response.success,'message':home_response.message}
                if not home_response.success:raise RuntimeError('Native home initialization rejected')
                duration_match = re.search(r'duration=([0-9.]+)s', home_response.message)
                if duration_match is None:raise RuntimeError('Native home duration unavailable')
                home_duration = float(duration_match.group(1))
                if not math.isfinite(home_duration) or not 0 < home_duration <= 4:raise RuntimeError('Unverified native home duration')
                report['home_planned_duration_s'] = home_duration
                end=time.monotonic()+5;stable_since=None
                while time.monotonic()<end:
                    spin(.02);check()
                    q=current()
                    report['home_samples'].append({'attempt':home_attempt,'seconds':time.monotonic()-home_started,'position':q,'qp':{k:v[1] for k,v in qp_commands.items()},'mux':{k:v[1] for k,v in commands.items()}})
                    settled=all(abs(q[k]-v)<(.002 if k=='waist_joint_1' else home_feedback_tolerance(plan)) for k,v in home.items())
                    settled &= max(abs(v) for v in state['joints'].velocity)<.02
                    settled &= time.monotonic()-home_started >= home_duration
                    # Home is a finite command stream, not a continuous heartbeat.
                    # Require all four groups observed after this request and the
                    # final commanded values at home, plus fresh actual feedback.
                    for group,names in groups.items():
                        if group not in commands or commands[group][0]<home_started:
                            settled=False;continue
                        values=active_command(group,commands[group][1])
                        settled &= all(abs(v-home[k])<(.002 if k=='waist_joint_1' else .01) for k,v in zip(names,values))
                    stable_since=(stable_since or time.monotonic()) if settled else None
                    if stable_since and time.monotonic()-stable_since>.5:break
                errors={k:abs(current()[k]-float(v)) for k,v in home.items()}
                report['home_final_errors']=errors
                report['home_attempts'].append({'errors':errors,'confirmed':bool(stable_since and time.monotonic()-stable_since>.5)})
                if report['home_attempts'][-1]['confirmed']:break
                raise RuntimeError('Home initialization/output confirmation timed out')
            else:raise RuntimeError('Home initialization failed')
            report['home_confirmed']=True;report['execution_ready']=True
            report['replay_start_positions']=current()
            initial=dict(report['replay_start_positions'])
            attempted=True;select(3);launch=time.monotonic();bad_since=None
            report['max_measured_velocity_by_joint']={k:0.0 for k in state['joints'].name}
            report['max_command_feedback_error_by_joint']={k:0.0 for k in state['joints'].name}
            for i,target in enumerate(targets):
                deadline=launch+plan['offset_ns'][i]/1e9
                while time.monotonic()<deadline:spin(.005);check(True)
                check(True)
                if time.monotonic()-deadline>.10:raise RuntimeError('Schedule deadline missed')
                enabled=plan['available'][i]
                publish(target,enabled);spin(.005)
                q=current();bad=False
                for name,v in zip(state['joints'].name,state['joints'].velocity):report['max_measured_velocity_by_joint'][name]=max(report['max_measured_velocity_by_joint'][name],abs(v))
                for group,names in groups.items():
                    if not enabled[{'A':0,'B':1,'body':4,'head':5}[group]]:continue
                    if group not in commands or time.monotonic()-commands[group][0]>.2:
                        if time.monotonic()-launch>.3:raise RuntimeError('Controller output missing '+group)
                        continue
                    values=active_command(group,commands[group][1])
                    for name,v in zip(names,values):report['max_command_feedback_error_by_joint'][name]=max(report['max_command_feedback_error_by_joint'][name],abs(q[name]-v))
                    bad |= any(abs(q[name]-v)>(.04 if name=='waist_joint_1' else .25) for name,v in zip(names,values))
                bad_since=(bad_since or time.monotonic()) if bad else None
                if bad_since and time.monotonic()-bad_since>.5:raise RuntimeError('Sustained tracking error')
                samples.append({'frame':i,'position':q,'enabled':bool(enabled[0]),'delay_ms':(time.monotonic()-deadline)*1000})
            report['status']='stream_completed'
    except BaseException as e:report['status']='failed';report['error']=str(e)
    finally:
        backend_stop.set()
        if attempted:
            try:
                # Idle clears active IK targets before restoring the original input.
                select(0);spin(.7)
                if time.monotonic()-state.get('joint_time',0)>.2 or max(abs(v) for v in state['joints'].velocity)>.02:raise RuntimeError('Stop not confirmed')
                report['stop_confirmed']=True
                if state.get('vr') is False:select(initial_source)
                report['original_source_restored']=state.get('ik_request_mux')==initial_source
            except BaseException as e:report['stop_error']=str(e)
        report['command_counts']=command_counts
        report['last_commands']={stage+'/'+k:v[1] for stage,values in [('qp',qp_commands),('mux',commands)] for k,v in values.items()}
        report['last_command_age_seconds']={stage+'/'+k:time.monotonic()-v[0] for stage,values in [('qp',qp_commands),('mux',commands)] for k,v in values.items()}
        report['samples']=samples;report['sent_frames']=report['robot_commands_sent']
        report['collision_events_observed']={side:state.get('collision_'+side) for side in ('A','B')}
        report['physical_replay_verified']=report['status']=='stream_completed' and report.get('stop_confirmed',False) and report.get('original_source_restored',False) and bool(samples) and any(abs(s['position'][k]-initial[k])>.01 for s in samples for k in initial)
        if attempted and (not report.get('stop_confirmed') or not report.get('original_source_restored')):
            report['status']='failed'
        print('RESULT_JSON='+json.dumps(report),flush=True)
        node.destroy_node();rclpy.shutdown()
    if report['status'] in ('failed','preflight_blocked'):raise SystemExit(2)


if __name__=='__main__':main()
