"""Explicit original-bag replay through the native service, with live guards."""
import argparse, json, math, re, signal, sys, threading, time, urllib.request
import xml.etree.ElementTree as ET
import numpy as np
from pathlib import Path
import rclpy, rosbag2_py
from rclpy.serialization import deserialize_message
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32, Int16MultiArray, Bool
from marvin_msgs.msg import JointcmdArm, JointcmdBody, JointcmdHead, IKRequest
from marvin_msgs.srv import GentoReplay, Int
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters

parser=argparse.ArgumentParser()
parser.add_argument('--execute',action='store_true')
parser.add_argument('--with-chassis',action='store_true')
parser.add_argument('--lerobot-plan',action='store_true')
parser.add_argument('--bag',choices=['my-bag-26-10-23-18-38-53'],default='my-bag-26-10-23-18-38-53')
a=parser.parse_args(); bag=a.bag
lerobot_plan=None
if a.lerobot_plan:
    lerobot_plan=json.loads(sys.stdin.readline())
    if lerobot_plan.get('schema')!='skye_native_lerobot_v1' or lerobot_plan.get('execute')!=a.execute:raise ValueError('Unverified LeRobot plan')
    def no_raw(event,args):
        if event=='open' and isinstance(args[0],(str,bytes)):
            path=Path(args[0].decode() if isinstance(args[0],bytes) else args[0])
            if set(path.parts)&{'raw_data','gento_replay_bags','hdf5','apex_hdf5'} or path.suffix in ('.mcap','.db3','.h5','.hdf5'):raise PermissionError('LeRobot-only replay')
    sys.addaudithook(no_raw)
    recorded=dict(zip(lerobot_plan['initial_joint_names'],lerobot_plan['initial_joint_positions']))
    if len(recorded)!=19 or not all(math.isfinite(v) for v in recorded.values()):raise ValueError('Invalid initial feedback')
else:
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri='/home/user/gento_replay_bags/'+bag,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    recorded=None
    while reader.has_next():
        topic,data,stamp=reader.read_next()
        if topic=='/tj/joint_states':
            msg=deserialize_message(data,JointState);recorded=dict(zip(msg.name,msg.position));break
    if recorded is None or len(recorded)!=19:raise RuntimeError('Recorded initial joints unavailable')
    del reader
rclpy.init(domain_id=29);node=rclpy.create_node('monitored_original_raw_replay')
state={};commands={};samples=[];events=[];start=time.monotonic();attempted=False;initial_source=None
report=dict(bag=bag,mode='native_original_bag_IK_replay',start_unix_ns=time.time_ns(),start_calls=0,status='preflight')
chassis=None;timeline=[];event_index=0
stop_watch=threading.Event();heartbeat={'time':time.monotonic(),'eof':False}
groups={'A':[f'Joint{i}_L' for i in range(1,8)],'B':[f'Joint{i}_R' for i in range(1,8)],'body':['waist_joint_1','waist_joint_2','waist_joint_3'],'head':['Head_J1_Joint','Head_J2_Joint']}
def heartbeat_reader():
    for line in sys.stdin:heartbeat['time']=time.monotonic()
    heartbeat['eof']=True
threading.Thread(target=heartbeat_reader,daemon=True).start()
def backend_reader():
    while not stop_watch.is_set():
        try:
            state['backend']=json.loads(urllib.request.urlopen('http://127.0.0.1:8080/api/v1/robot/status',timeout=.15).read());state['backend_time']=time.monotonic()
        except Exception as exc:state['backend_error']=str(exc)
        stop_watch.wait(.05)
threading.Thread(target=backend_reader,daemon=True).start()
def joints(m):
    state.update(joints=m,feedback_time=time.monotonic())
    if not samples or time.monotonic()-start-samples[-1]['t']>.02:
        samples.append(dict(t=time.monotonic()-start,names=list(m.name),position=list(m.position),velocity=list(m.velocity)))
def native_status(m):
    state.update(native=json.loads(m.data),native_time=time.monotonic())
    event={k:state['native'].get(k) for k in ('playback_active','active_playback_bag_uri','last_message')}
    if not events or events[-1]['status']!=event:events.append(dict(t=time.monotonic()-start,status=event))
node.create_subscription(JointState,'/tj/joint_states',joints,qos_profile_sensor_data)
node.create_subscription(String,'/tj/info/gento_replay/status',native_status,10)
node.create_subscription(Bool,'/tj/info/vr_connected',lambda m:state.update(vr=m.data),qos_profile_sensor_data)
node.create_subscription(Int16MultiArray,'/tj/info/robot_state',lambda m:state.update(mode=list(m.data),mode_time=time.monotonic()),qos_profile_sensor_data)
for side in ('A','B'):
    node.create_subscription(Bool,'/tj/info/collision_status'+side,lambda m,k=side:state.update({'collision_'+k:m.data}),qos_profile_sensor_data)
latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
for key in ('ik_request_mux','joint_cmd_mux'):
    node.create_subscription(Int32,'/tj/info/'+key+'/active_source',lambda m,k=key:state.update({k:m.data}),latched)
for group,typ in [('A',JointcmdArm),('B',JointcmdArm),('body',JointcmdBody),('head',JointcmdHead)]:
    node.create_subscription(typ,'/tj/control/joint_cmd_'+group,lambda m,k=group:commands.update({k:(time.monotonic(),list(m.positions))}),qos_profile_sensor_data)
def replay_request(m):
    state['replay_messages']=state.get('replay_messages',0)+1
    enabled=any(m.available)
    if enabled and not state.get('replay_enabled'):state['enabled_since']=time.monotonic()
    state['replay_enabled']=enabled
node.create_subscription(IKRequest,'/tj/control/ik_request/replay',replay_request,qos_profile_sensor_data)
client=node.create_client(GentoReplay,'/tj/control/gento_replay/playback_named')
selector=node.create_client(Int,'/tj/control/ik_request_mux/select')
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=min(.001,max(0,end-time.monotonic())))
def invoke(client,request):
    if not client.wait_for_service(timeout_sec=2):raise RuntimeError('Service unavailable')
    future=client.call_async(request);end=time.monotonic()+3
    while not future.done() and time.monotonic()<end:spin(.005)
    if not future.done():raise RuntimeError('Service timeout')
    return future.result()
def native(value):
    req=GentoReplay.Request();req.data=value;req.name=bag;r=invoke(client,req)
    return dict(success=r.success,error_code=r.error_code,message=r.message)
def select(value):
    req=Int.Request();req.data=value;invoke(selector,req);end=time.monotonic()+2
    while state.get('ik_request_mux')!=value and time.monotonic()<end:spin(.01)
    if state.get('ik_request_mux')!=value:raise RuntimeError('Mux transition not confirmed')
def check(allow_idle=False):
    now=time.monotonic()
    if heartbeat['eof'] or now-heartbeat['time']>.5:raise RuntimeError('Host heartbeat lost')
    if now-state.get('feedback_time',0)>.2:raise RuntimeError('Joint feedback stale')
    m=state['joints']
    if len(m.name)!=19 or len(m.position)!=19 or len(m.velocity)!=19 or not all(math.isfinite(v) for v in [*m.position,*m.velocity]):raise RuntimeError('Invalid joint feedback')
    if now-state.get('mode_time',0)>.2 or state.get('mode')!=[2,2,1,1,1,0]:raise RuntimeError('Robot mode/fault changed')
    if state.get('vr') is not False:raise RuntimeError('VR connected or unknown')
    if state.get('joint_cmd_mux')!=0:raise RuntimeError('QP output ownership changed')
    if state.get('ik_request_mux') not in ((0,1) if allow_idle else (1,3)):raise RuntimeError('IK input ownership changed')
    if any(state.get('collision_'+k) is True for k in ('A','B')):raise RuntimeError('Collision event active')
    if now-state.get('backend_time',0)>.3:raise RuntimeError('Backend health stale')
    backend=state['backend']
    if not backend.get('ok') or not backend.get('data',{}).get('control_ready') or backend['data'].get('control_error'):raise RuntimeError('Backend control fault')
    if now-state.get('native_time',0)>2:raise RuntimeError('Native status stale')
    for name,v in zip(m.name,m.velocity):
        if abs(v)>(1.0 if name=='waist_joint_1' else 3.14)+1e-6:raise RuntimeError('Measured velocity limit exceeded: '+name)
def interrupted(*_):raise RuntimeError('Interrupted')
signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
try:
    if a.with_chassis:
        from chassis_replay import CoordinatedChassis
        if lerobot_plan:
            from lerobot_endpoint import fill_ik_request
            rows=[(int(t),[v[0],v[1],0.,0.,0.,v[2]]) for t,v in zip(lerobot_plan['chassis_offset_ns'],lerobot_plan['chassis_velocity'])]
            chassis=CoordinatedChassis(None,rows=rows)
            for i,stamp in enumerate(lerobot_plan['offset_ns']):
                target={k:np.asarray(v[i],float) for k,v in lerobot_plan['ik_reference'].items()}
                msg=IKRequest();fill_ik_request(msg,target,lerobot_plan['available'][i])
                timeline.append((int(stamp),'ik',msg))
            timeline.extend((stamp,'chassis',values) for stamp,values in chassis.rows)
            report.update(test_duration_s=lerobot_plan.get('test_duration_s'),source=lerobot_plan['source'],source_sha256=lerobot_plan['source_sha256'],raw_or_hdf5_read=False)
        else:
            chassis=CoordinatedChassis('/home/user/gento_replay_bags/'+bag)
            reader=rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri='/home/user/gento_replay_bags/'+bag,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
            while reader.has_next():
                topic,data,stamp=reader.read_next()
                if topic=='/tj/control/ik_request':timeline.append((stamp,'ik',deserialize_message(data,IKRequest)))
            del reader
            timeline.extend((stamp,'chassis',values) for stamp,values in chassis.rows)
        timeline.sort(key=lambda row:row[0])
        if not timeline or not any(k=='ik' for _,k,_ in timeline):raise RuntimeError('Missing original IK stream')
        origin=timeline[0][0];trajectory_duration=(timeline[-1][0]-origin)/1e9
        if not 0<trajectory_duration<35:raise RuntimeError('Unexpected recording duration')
        publisher=node.create_publisher(IKRequest,'/tj/control/ik_request/replay',10)
        report.update(mode='synchronized_LeRobot_IK_and_chassis' if lerobot_plan else 'synchronized_original_IK_and_chassis',source_ik_frames=sum(k=='ik' for _,k,_ in timeline),source_chassis_frames=len(chassis.rows),trajectory_duration_s=trajectory_duration)
    deadline=time.monotonic()+6
    while time.monotonic()<deadline:
        spin(.05)
        if all(k in state for k in ('joints','native','mode','vr','backend','ik_request_mux','joint_cmd_mux')):break
    check(allow_idle=True)
    initial_source=state['ik_request_mux'];report['initial_ik_source']=initial_source
    if chassis:chassis.ready()
    if state['native'].get('recording') or state['native'].get('playback_active') or state['native'].get('loop_playback'):raise RuntimeError('Native busy or looping')
    if initial_source not in (0,1):raise RuntimeError('Replay source already owned')
    if max(map(abs,state['joints'].velocity))>.02:raise RuntimeError('Robot moving')
    initial=dict(zip(state['joints'].name,state['joints'].position));report['initial']=initial
    errors={k:abs(initial[k]-v) for k,v in recorded.items()};report['recorded_start_errors']=errors
    if any(v>(.005 if k=='waist_joint_1' else .05) for k,v in errors.items()):raise RuntimeError('Too far from recorded start')
    if not a.execute:
        report['status']='preflight_passed'
    else:
        tree=ET.fromstring(Path('/opt/kernelmind/apex/install/marvin_description/share/marvin_description/mjcf/Gento_Sky/Gento_Sky.xml').read_text())
        order=[j.get('name') for j in tree.findall('.//worldbody//joint')]
        home=dict(zip(order,np.fromstring(tree.find("keyframe/key[@name='home']").get('qpos'),sep=' ')))
        if len(home)!=19 or any(abs(initial[k]-v)>(.005 if k=='waist_joint_1' else .05) for k,v in home.items()):
            raise RuntimeError('Too far from home for native initialization')
        # Idle is a valid stopped entry state; never change it during read-only preflight.
        check(allow_idle=True)
        if state['ik_request_mux']!=initial_source:raise RuntimeError('IK ownership changed during preflight')
        if initial_source==0:
            pc=node.create_client(GetParameters,'/tj/ik_request_mux/get_parameters')
            request=GetParameters.Request()
            request.names=['clear_target_cache_on_switch','publish_cached_on_switch','publish_disable_on_switch','source_topics']
            response=invoke(pc,request);node.destroy_client(pc)
            values=response.values
            if len(values)!=4 or [v.type for v in values]!=[1,1,1,9] or [v.bool_value for v in values[:3]]!=[True,False,True] or list(values[3].string_array_value)!=['','control/ik_request/vr','control/ik_request/vla','control/ik_request/replay']:
                raise RuntimeError('Unverified idle-to-home mux policy')
            check(allow_idle=True)
            if state['ik_request_mux']!=0:raise RuntimeError('Idle ownership changed')
            attempted=True;select(1);check()
            report['idle_initialization_source_selected']=True
        attempted=True;home_start=time.monotonic()
        hc=node.create_client(Trigger,'/tj/control/go_home');hr=invoke(hc,Trigger.Request());node.destroy_client(hc)
        report['home_response']=dict(success=hr.success,message=hr.message)
        match=re.search(r'duration=([0-9.]+)s',hr.message)
        if not hr.success or not match or not 0<float(match.group(1))<=4:raise RuntimeError('Unverified home initialization')
        duration=float(match.group(1));end=time.monotonic()+5;stable=None
        while time.monotonic()<end:
            spin(.01);check();now=time.monotonic();current=dict(zip(state['joints'].name,state['joints'].position))
            settled=now-home_start>=duration and max(map(abs,state['joints'].velocity))<.02
            settled &= all(abs(current[k]-v)<(.002 if k=='waist_joint_1' else .01) for k,v in home.items())
            for group,names in groups.items():
                if group not in commands or commands[group][0]<home_start:settled=False;continue
                vals=commands[group][1]
                settled &= len(vals)>=len(names) and all(abs(v-home[k])<(.002 if k=='waist_joint_1' else .01) for k,v in zip(names,vals))
            stable=(stable or now) if settled else None
            if stable and now-stable>.5:break
        else:raise RuntimeError('Home feedback/output confirmation failed')
        report['home_confirmed']=True
        if lerobot_plan:
            isolated=json.loads(Path('/home/user/codex-robot-test/native_qp_audit/isolated_summary.json').read_text())
            if not isolated.get('passed') or isolated.get('source_sha256')!=lerobot_plan['source_sha256']:raise RuntimeError('Matching full isolated validation missing')
        if chassis:
            chassis.ready();select(3);chassis.began=True
            report['start_calls']=1;report['ik_frames_sent']=0;report['max_schedule_lateness_s']=0
            held_velocity=[0.]*6
            print(json.dumps({'synchronized_replay_started':True,'duration_s':trajectory_duration}),flush=True)
        else:
            report['start_calls']=1;response=native(1);report['start_response']=response
            print(json.dumps({'start_response':response}),flush=True)
            if not response['success']:raise RuntimeError('Native playback rejected')
        launch=time.monotonic();active=False;bad_since=None;settle_start=None;settled_since=None;last_ik=None;next_hold=0;report['max_command_feedback_error']={};report['max_velocity']={}
        while time.monotonic()-launch<35:
            spin(.0005 if chassis else .005);check();now=time.monotonic();s=state['native']
            if chassis:
                chassis.check();chassis.tracking(held_velocity)
                if state['ik_request_mux']!=3 or s.get('playback_active'):raise RuntimeError('Coordinated replay ownership changed')
                if event_index<len(timeline):
                    stamp,kind,payload=timeline[event_index];due=launch+(stamp-origin)/1e9
                    if now>=due:
                        lateness=now-due
                        if lateness>.1:raise RuntimeError('Replay scheduling deadline exceeded 100ms')
                        report['max_schedule_lateness_s']=max(report['max_schedule_lateness_s'],lateness)
                        if kind=='ik':
                            payload.header.stamp=node.get_clock().now().to_msg();publisher.publish(payload);last_ik=payload;report['ik_frames_sent']+=1
                        else:
                            chassis.publish(payload,stamp);held_velocity=payload
                        event_index+=1
                if event_index==len(timeline):
                    if not report.get('test_duration_s'):report['status']='completed';break
                    settle_start=settle_start or now
                    if now>=next_hold:
                        last_ik.header.stamp=node.get_clock().now().to_msg();publisher.publish(last_ik)
                        chassis.publish([0.]*6);held_velocity=[0.]*6;next_hold=now+.02
                        report['settle_hold_messages']=report.get('settle_hold_messages',0)+1
                    stable=now-settle_start>=2 and max(map(abs,state['joints'].velocity))<.02
                    settled_since=(settled_since or now) if stable else None
                    if settled_since and now-settled_since>.3:
                        report['settle_duration_s']=now-settle_start;report['status']='completed';break
                    if now-settle_start>4:raise RuntimeError('Short segment final target did not settle within 4s')
            if s.get('playback_active'):
                active=True
                if Path(s.get('active_playback_bag_uri','')).name!=bag:raise RuntimeError('Wrong recording selected')
            current=dict(zip(state['joints'].name,state['joints'].position));bad=False
            for name,v in zip(state['joints'].name,state['joints'].velocity):report['max_velocity'][name]=max(report['max_velocity'].get(name,0),abs(v))
            for group,names in groups.items():
                if group not in commands or now-commands[group][0]>.2:
                    if state.get('replay_enabled') and now-state.get('enabled_since',now)>.3:
                        raise RuntimeError('Controller output missing during active replay: '+group)
                    continue
                vals=commands[group][1]
                if len(vals)!=dict(A=7,B=7,body=6,head=3)[group] or not all(math.isfinite(v) for v in vals):raise RuntimeError('Malformed command')
                if any(abs(v)>1e-10 for v in vals[len(names):]):raise RuntimeError('Nonzero reserved command slots')
                for name,v in zip(names,vals):
                    error=abs(current[name]-v);report['max_command_feedback_error'][name]=max(report['max_command_feedback_error'].get(name,0),error)
                    bad |= error>(.04 if name=='waist_joint_1' else .25)
            bad_since=(bad_since or now) if bad else None
            if bad_since and now-bad_since>.5:raise RuntimeError('Sustained tracking error >0.25rad/0.04m')
            if active and not s.get('playback_active'):report['status']='completed';break
            if not chassis and not active and now-launch>5:raise RuntimeError('No native playback-active transition')
        else:raise RuntimeError('Playback timeout')
except BaseException as exc:
    report['status']='failed';report['error']=str(exc)
finally:
    if attempted:
        try:
            if chassis:
                if chassis.began:chassis.publish([0.]*6)
                select(0)
            else:report['stop_response']=native(0);select(0)
            spin(.7)
            if time.monotonic()-state.get('feedback_time',0)>.2 or max(map(abs,state['joints'].velocity))>.02:raise RuntimeError('Stop not confirmed')
            report['stop_confirmed']=True
            if state.get('vr') is not False:raise RuntimeError('Cannot restore while VR connected')
            select(initial_source);report['original_source_restored']=True
        except BaseException as exc:report['stop_error']=str(exc);report['status']='failed'
    if chassis:
        try:
            report['chassis']=chassis.finish()
            if chassis.began and not report['chassis']['stop_confirmed']:raise RuntimeError('Chassis stop not confirmed')
            if report['status']=='completed' and not report.get('test_duration_s') and report['chassis'].get('measured_max_linear_m_s',0)<.01:raise RuntimeError('No measured chassis motion')
        except BaseException as exc:report['chassis_error']=str(exc);report['status']='failed'
    stop_watch.set();report['samples']=samples;report['events']=events
    report['replay_messages']=state.get('replay_messages',0);report['last_status']=state.get('native')
    report['final_ik_source']=state.get('ik_request_mux');report['duration_s']=time.monotonic()-start
    if 'initial' in report and samples:
        report['peak_displacement']={name:max(abs(dict(zip(s['names'],s['position']))[name]-v) for s in samples) for name,v in report['initial'].items()}
        report['measured_motion']=any(v>.01 for k,v in report['peak_displacement'].items() if k.startswith('Joint'))
    if report['status']=='completed' and not report.get('test_duration_s') and not report.get('measured_motion'):
        report['status']='failed';report['error']='Native stream ended without measured physical motion'
    path=Path('/home/user/codex-robot-test')/('native-ik-replay-'+str(report['start_unix_ns'])+'.json')
    path.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:({x:y for x,y in v.items() if x not in ('samples','sent','diagnostics','published_commands')} if k=='chassis' else v) for k,v in report.items() if k not in ('samples','events')},indent=2),flush=True)
    print('REPORT_PATH='+str(path),flush=True);node.destroy_node();rclpy.shutdown()
if report['status']=='failed':raise SystemExit(2)
