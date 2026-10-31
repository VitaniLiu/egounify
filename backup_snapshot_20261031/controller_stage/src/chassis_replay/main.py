"""Replay recorded physical chassis commands, never IK or gamepad intent.
Run on ms-thor with Apex ROS environment. Default: inspect only.
"""
import argparse,json,math,time,signal
from pathlib import Path

TOPIC='/recording/chassis/manual_move_cmd'
DEST='/move/ManualMoveCmd'
BASE=Path('/home/user/gento_replay_bags')


def validate(rows):
    if len(rows)<2:raise ValueError('No recorded chassis command stream')
    previous=None
    for stamp,v in rows:
        if type(stamp) is not int or len(v)!=6 or not all(math.isfinite(x) for x in v):raise ValueError('Malformed command')
        if previous is not None and not 0<stamp-previous<=100_000_000:raise ValueError('Nonmonotonic timestamps or >100ms recording gap')
        if abs(v[0])>.400001 or abs(v[5])>.500001:raise ValueError('Recorded command exceeds verified 0.4m/s / 0.5rad/s envelope')
        if any(abs(v[i])>1e-8 for i in (1,2,3,4)):raise ValueError('Unsupported lateral/other-axis command')
        previous=stamp
    if any(abs(v)>1e-8 for v in rows[-1][1]):raise ValueError('Recording must end with a stop command')
    if rows[-1][0]-rows[0][0]>120_000_000_000:raise ValueError('Replay exceeds bounded two-minute test')
    return rows


def load(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import TwistStamped
    r=rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
    types={t.name:t.type for t in r.get_all_topics_and_types()}
    if types.get(TOPIC)!='geometry_msgs/msg/TwistStamped':raise ValueError('Recorded physical manual-command topic missing')
    r.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]));rows=[]
    while r.has_next():
        _,blob,t=r.read_next();m=deserialize_message(blob,TwistStamped)
        if m.header.frame_id!='base_link':raise ValueError('Unverified command frame')
        v=m.twist;rows.append((int(t),[v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z]))
    return validate(rows)


def summary(rows):
    x=y=yaw=distance=0.
    for (t,v),(t2,_) in zip(rows,rows[1:]):
        dt=(t2-t)/1e9;turn=v[5]*dt
        # Integrate body-frame forward velocity under a held yaw rate.
        if abs(v[5])>1e-10:
            x+=v[0]/v[5]*(math.sin(yaw+turn)-math.sin(yaw))
            y+=v[0]/v[5]*(-math.cos(yaw+turn)+math.cos(yaw))
        else:x+=v[0]*dt*math.cos(yaw);y+=v[0]*dt*math.sin(yaw)
        yaw+=turn;distance+=abs(v[0])*dt
    return {'frames':len(rows),'duration_s':(rows[-1][0]-rows[0][0])/1e9,
            'nonzero_frames':sum(any(abs(x)>1e-6 for x in v) for t,v in rows),
            'max_linear_m_s':max(abs(v[0]) for t,v in rows),'max_yaw_rad_s':max(abs(v[5]) for t,v in rows),
            'command_integrated_relative_xy_yaw':[x,y,yaw],'command_integrated_path_m':distance}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag',required=True);p.add_argument('--execute',action='store_true');a=p.parse_args()
    bag=BASE/a.bag
    if bag.resolve().parent!=BASE or not bag.is_dir():raise ValueError('Bag must be a direct child of robot recording directory')
    rows=load(bag);report=summary(rows)
    report.update(bag=str(bag),source_topic=TOPIC,destination_topic=DEST,domain_id=0,mode='inspect',motion_commands_sent=0,
                  recording_has_original_chassis_feedback=False)
    if not a.execute:print(json.dumps(report,indent=2));return
    import rclpy
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
    from geometry_msgs.msg import TwistStamped
    from move.msg import State
    rclpy.init(domain_id=0);node=rclpy.create_node('recorded_chassis_replay')
    pub=node.create_publisher(TwistStamped,DEST,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
    state={};samples=[];own=set();foreign=[];sent=[];start=time.monotonic();began=False
    def feedback(m):
        state['message']=m;state['time']=time.monotonic()
        if not samples or state['time']-start-samples[-1]['t']>=.02:
            q=m.odometry.orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            samples.append({'t':state['time']-start,'vx':m.speed.linear.x,'vy':m.speed.linear.y,'wz':m.speed.angular.z,'x':m.odometry.position.x,'y':m.odometry.position.y,'yaw':yaw})
    def incoming(m):
        key=(m.header.stamp.sec,m.header.stamp.nanosec)
        if key not in own:foreign.append({'time':time.monotonic(),'vx':m.twist.linear.x,'wz':m.twist.angular.z})
    node.create_subscription(State,'/move/State',feedback,qos_profile_sensor_data)
    node.create_subscription(TwistStamped,DEST,incoming,qos_profile_sensor_data)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=min(.005,max(0,end-time.monotonic())))
    def check():
        now=time.monotonic()
        if now-state.get('time',0)>.2:raise RuntimeError('Chassis feedback stale >200ms')
        m=state['message']
        if m.is_soft_auto or not m.is_hard_auto or m.is_joystick_on or m.is_handle_on or m.is_parking_mode or m.is_avoid:
            raise RuntimeError('Chassis manual-control/obstacle/interlock state changed')
        vals=[m.speed.linear.x,m.speed.linear.y,m.speed.angular.z,m.odometry.position.x,m.odometry.position.y]
        if not all(math.isfinite(x) for x in vals):raise RuntimeError('Nonfinite chassis feedback')
        if abs(vals[0])>.55 or abs(vals[1])>.1 or abs(vals[2])>.65:raise RuntimeError('Measured chassis speed exceeds envelope')
        arm=Path('/home/user/gamepad_control/armed_until')
        if arm.exists() and float(arm.read_text())>now:raise RuntimeError('Gamepad motion output armed; refusing competing control')
        if foreign:raise RuntimeError('Another manual-command publisher is active')
    def publish(v):
        m=TwistStamped();m.header.stamp=node.get_clock().now().to_msg();m.header.frame_id='base_link'
        own.add((m.header.stamp.sec,m.header.stamp.nanosec))
        m.twist.linear.x=v[0];m.twist.angular.z=v[5];pub.publish(m)
    def interrupted(*_):raise KeyboardInterrupt('Replay interrupted')
    signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGINT,interrupted)
    failure=None
    try:
        spin(3);check()
        if pub.get_subscription_count()<2:raise RuntimeError('Physical receiver not discovered (recording bridge alone insufficient)')
        if abs(state['message'].speed.linear.x)>.01 or abs(state['message'].speed.angular.z)>.02:raise RuntimeError('Chassis is not initially stationary')
        report['initial_odom']=samples[-1];report['mode']='execute';began=True
        launch=time.monotonic();bad_since=None
        for i,(stamp,v) in enumerate(rows):
            deadline=launch+(stamp-rows[0][0])/1e9
            while time.monotonic()<deadline:
                spin(min(.005,deadline-time.monotonic()));check()
            check();delay=time.monotonic()-deadline
            if delay>.1:raise RuntimeError('Replay deadline missed >100ms; no catch-up burst')
            publish(v);sent.append({'index':i,'t':time.monotonic()-launch,'vx':v[0],'wz':v[5],'delay_ms':delay*1000})
            m=state['message'];bad=abs(m.speed.linear.x-v[0])>.25 or abs(m.speed.angular.z-v[5])>.35
            bad_since=(bad_since or time.monotonic()) if bad else None
            if bad_since and time.monotonic()-bad_since>.6:raise RuntimeError('Sustained measured speed tracking error')
        report['stream_completed']=True
    except BaseException as e:
        failure=str(e);report['stream_completed']=False;report['error']=failure
    finally:
        if began:
            for _ in range(25):publish([0.]*6);spin(.02)
            stop_deadline=time.monotonic()+3;stopped=False
            while time.monotonic()<stop_deadline:
                spin(.05)
                if time.monotonic()-state.get('time',0)<.2:
                    m=state['message']
                    if abs(m.speed.linear.x)<.01 and abs(m.speed.angular.z)<.02:stopped=True;break
            report['stop_feedback_confirmed']=stopped
        report['sent_frames']=len(sent);report['motion_commands_sent']=sum(abs(x['vx'])>1e-6 or abs(x['wz'])>1e-6 for x in sent)
        report['samples']=samples;report['sent']=sent;report['foreign_commands']=foreign
        if samples:
            report['final_odom']=samples[-1]
            report['measured_max_linear_m_s']=max(abs(x['vx']) for x in samples)
            report['measured_max_yaw_rad_s']=max(abs(x['wz']) for x in samples)
            report['measured_path_m']=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(samples,samples[1:]))
            report['measured_motion']=report['measured_max_linear_m_s']>.03 or report['measured_max_yaw_rad_s']>.03
        report['passed']=bool(report.get('stream_completed') and report.get('measured_motion') and report.get('stop_feedback_confirmed'))
        report['passed_scope']='command_stream_completed_motion_detected_and_stopped_only'
        report['original_trajectory_accuracy_verified']=False
        dest=Path('/home/user/codex-robot-test')/('chassis-replay-'+str(time.time_ns())+'.json')
        dest.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ['samples','sent','foreign_commands']},indent=2),flush=True)
        print('REPORT_PATH='+str(dest),flush=True)
        node.destroy_node();rclpy.shutdown()
    if failure or not report['passed']:raise SystemExit(1)





class ChassisTrackingGuard:
    """Bounded steering preparation followed by the original speed-error guard."""
    PREP_TIMEOUT = 1.0
    PROGRESS_TIMEOUT = .4
    ALIGN_TOLERANCE = .03
    def __init__(self):
        self.phase='tracking';self.bad_since=None;self.can_prepare=True
        self.good_since=None;self.events=[]
    def update(self, now, command, measured, steering_error, wheel_command, gyro, wheel_feedback=0.):
        vx,wz=command;mx,my,mw=measured
        if not all(math.isfinite(v) for v in [now,vx,wz,mx,my,mw,steering_error,wheel_command,gyro,wheel_feedback]):
            raise RuntimeError('Nonfinite chassis tracking input')
        stationary=abs(mx)<=.05 and abs(my)<=.05 and abs(mw)<=.1 and abs(gyro)<=.1
        active=abs(vx)>.001 or abs(wz)>.001
        if (self.phase=='tracking' and self.can_prepare and active
                and steering_error>.05 and wheel_command<=.005 and stationary):
            self.phase='preparing';self.can_prepare=False;self.started=now
            self.progress_time=now;self.best_error=steering_error;self.good_since=None
            self.prep_last=now;self.prep_distance=0.;self.prep_angle=0.
            self.prep_previous_speed=math.hypot(mx,my);self.prep_previous_yaw=max(abs(mw),abs(gyro))
            self.events.append(dict(t=now,phase='preparing',steering_error=steering_error))
        if self.phase=='preparing':
            dt=max(0,now-self.prep_last);self.prep_last=now
            self.prep_distance+=(self.prep_previous_speed+math.hypot(mx,my))*.5*dt
            self.prep_angle+=(self.prep_previous_yaw+max(abs(mw),abs(gyro)))*.5*dt
            self.prep_previous_speed=math.hypot(mx,my);self.prep_previous_yaw=max(abs(mw),abs(gyro))
            if self.prep_distance>.02 or self.prep_angle>.05:raise RuntimeError('Steering preparation motion budget exceeded')
            if now-self.started>self.PREP_TIMEOUT:raise RuntimeError('Steering preparation timeout >1s')
            if steering_error<=self.ALIGN_TOLERANCE:
                self.phase='tracking';self.bad_since=None
                self.events.append(dict(t=now,phase='tracking',preparation_s=now-self.started))
            else:
                if not stationary:raise RuntimeError('Unexpected motion during steering preparation')
                # The driver may creep while steering; bound it instead of assuming zero drive.
                if wheel_command>.05 or wheel_feedback>.08:raise RuntimeError('Wheel creep exceeded during steering preparation')
                if steering_error<self.best_error-.01:
                    self.best_error=steering_error;self.progress_time=now
                if now-self.progress_time>self.PROGRESS_TIMEOUT:raise RuntimeError('Steering failed to converge for 0.4s')
                return self.phase
        bad=abs(mx-vx)>.25 or abs(mw-wz)>.35
        if bad:
            if self.bad_since is None:self.bad_since=now
            self.good_since=None
        else:
            self.bad_since=None
            # Re-arm only after actual successful tracking/alignment, not a new command.
            if steering_error<=self.ALIGN_TOLERANCE:
                if self.good_since is None:self.good_since=now
                if now-self.good_since>=.2:self.can_prepare=True
            else:self.good_since=None
        if self.bad_since is not None and now-self.bad_since>.6:
            raise RuntimeError('Sustained chassis speed tracking error')
        return self.phase


class CoordinatedChassis:
    """Domain-0 transport/guards for a scheduler shared with original IK."""
    def __init__(self, bag, rows=None):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
        from geometry_msgs.msg import TwistStamped
        from move.msg import State
        import threading
        self.rows=load(bag) if rows is None else validate(rows); self.state={}; self.samples=[]; self.own=set()
        self.foreign=[]; self.sent=[]; self.published=[]; self.began=False; self.tracking_guard=ChassisTrackingGuard()
        self.TwistStamped=TwistStamped
        self.context=Context();rclpy.init(context=self.context,domain_id=0)
        self.node=rclpy.create_node('synchronized_raw_chassis',context=self.context)
        self.pub=self.node.create_publisher(TwistStamped,DEST,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
        def feedback(msg):
            now=time.monotonic();self.state.update(message=msg,time=now)
            self.samples.append(dict(t=now,vx=msg.speed.linear.x,vy=msg.speed.linear.y,wz=msg.speed.angular.z,
                                     x=msg.odometry.position.x,y=msg.odometry.position.y))
        def incoming(msg):
            if (msg.header.stamp.sec,msg.header.stamp.nanosec) not in self.own:
                self.foreign.append(time.monotonic())
        self.node.create_subscription(State,'/move/State',feedback,qos_profile_sensor_data)
        self.node.create_subscription(TwistStamped,DEST,incoming,qos_profile_sensor_data)
        # Read-only evidence for separating steering execution from state estimation.
        from rosidl_runtime_py.utilities import get_message
        from rosidl_runtime_py.convert import message_to_ordereddict
        self.diagnostics=[];self.diagnostic_times={};self.diagnostic_latest={}
        def diagnostic(msg, topic):
            now=time.monotonic()
            if now-self.diagnostic_times.get(topic,0)<.05:return
            self.diagnostic_times[topic]=now
            entry=dict(topic=topic,t=now,message=message_to_ordereddict(msg))
            self.diagnostics.append(entry);self.diagnostic_latest[topic]=entry
        for topic, typename in [('/driver/motor/Data','driver/msg/MotorsData'),
                                ('/driver/motor/SetMotorCmd','driver/msg/SetMotorCmd'),
                                ('/driver/shaft/motor/SetMotorCmd','driver/msg/SetShaftMotorCmd'),
                                ('/driver/imu/Data','sensor_msgs/msg/Imu'),
                                ('/driver/stm32/Data','driver/msg/Stm32Data'),
                                ('/move/AutoMoveCmd','geometry_msgs/msg/TwistStamped')]:
            self.node.create_subscription(get_message(typename),topic,
                lambda msg,topic=topic:diagnostic(msg,topic),qos_profile_sensor_data)
        self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.node)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()
    def check(self):
        now=time.monotonic()
        if now-self.state.get('time',0)>.2:raise RuntimeError('Chassis feedback stale >200ms')
        m=self.state['message']
        if m.is_soft_auto or not m.is_hard_auto or m.is_joystick_on or m.is_handle_on or m.is_parking_mode or m.is_avoid:
            raise RuntimeError('Chassis manual-control/obstacle/interlock changed')
        vals=[m.speed.linear.x,m.speed.linear.y,m.speed.angular.z,m.odometry.position.x,m.odometry.position.y]
        if not all(math.isfinite(v) for v in vals):raise RuntimeError('Nonfinite chassis feedback')
        if abs(vals[0])>.55 or abs(vals[1])>.1 or abs(vals[2])>.65:raise RuntimeError('Chassis measured velocity exceeded')
        arm=Path('/home/user/gamepad_control/armed_until')
        if arm.exists() and float(arm.read_text())>now:raise RuntimeError('Gamepad control armed')
        if self.foreign:raise RuntimeError('Competing chassis manual commands')
    def ready(self):
        self.check()
        if self.pub.get_subscription_count()<2:raise RuntimeError('Physical chassis receiver missing')
        m=self.state['message']
        if abs(m.speed.linear.x)>.01 or abs(m.speed.angular.z)>.02:raise RuntimeError('Chassis moving before replay')
    def publish(self, values, stamp=None):
        m=self.TwistStamped();m.header.stamp=self.node.get_clock().now().to_msg();m.header.frame_id='base_link'
        self.own.add((m.header.stamp.sec,m.header.stamp.nanosec))
        m.twist.linear.x=values[0];m.twist.angular.z=values[5];self.pub.publish(m)
        self.published.append(dict(t=time.monotonic(),vx=values[0],wz=values[5],source_ns=stamp))
        if stamp is not None:self.sent.append(dict(source_ns=stamp,t=time.monotonic(),vx=values[0],wz=values[5]))
    def tracking(self, values):
        now=time.monotonic();m=self.state['message']
        def fresh(topic):
            entry=self.diagnostic_latest.get(topic)
            if entry is None or now-entry['t']>.2:raise RuntimeError('Tracking diagnostic stale: '+topic)
            return entry['message']
        feedback=fresh('/driver/motor/Data');command=fresh('/driver/motor/SetMotorCmd')
        gyro=fresh('/driver/imu/Data')['angular_velocity']['z']
        stm=fresh('/driver/stm32/Data')
        if not stm['is_connected'] or stm['is_hard_scram'] or stm['is_soft_scram'] or stm['error_code']:
            raise RuntimeError('Chassis emergency/fault')
        for item in feedback['wheel_data']+feedback['steer_data']:
            if not item['is_enable'] or not item['is_connected'] or item['error_code']:raise RuntimeError('Motor fault')
        actual={v['id']:v['position'] for v in feedback['steer_data']}
        targets={v['id']:v['position'] for v in command['steer_data']}
        if len(actual)!=4 or actual.keys()!=targets.keys() or len(command['wheel_data'])!=4:
            raise RuntimeError('Incomplete chassis motor feedback/command')
        errors=[abs(math.atan2(math.sin(targets[k]-actual[k]),math.cos(targets[k]-actual[k]))) for k in actual]
        if not all(math.isfinite(v) for v in errors):raise RuntimeError('Invalid steering feedback')
        speeds=[abs(v['speed']) for v in command['wheel_data']]
        if not all(math.isfinite(v) for v in speeds):raise RuntimeError('Invalid wheel command')
        self.tracking_guard.update(now,(values[0],values[5]),
            (m.speed.linear.x,m.speed.linear.y,m.speed.angular.z),max(errors),max(speeds),gyro,max(abs(v['speed']) for v in feedback['wheel_data']))
    def finish(self):
        stopped=False
        if self.began:
            for _ in range(25):self.publish([0.]*6);time.sleep(.02)
            end=time.monotonic()+3
            while time.monotonic()<end:
                m=self.state.get('message')
                if m is not None and time.monotonic()-self.state.get('time',0)<.2 and abs(m.speed.linear.x)<.01 and abs(m.speed.angular.z)<.02:
                    stopped=True;break
                time.sleep(.02)
        result=dict(source=summary(self.rows),sent_frames=len(self.sent),
                    motion_commands_sent=sum(abs(v['vx'])>1e-6 or abs(v['wz'])>1e-6 for v in self.sent),
                    stop_confirmed=stopped if self.began else None,samples=self.samples,sent=self.sent,
                    tracking_policy=dict(schema='steering_then_speed_v2',preparation_timeout_s=1.,progress_timeout_s=.4,preparation_wheel_command_limit=.05,preparation_wheel_feedback_limit=.08,preparation_distance_limit_m=.02,preparation_angle_limit_rad=.05,alignment_tolerance_rad=.03,speed_error_duration_s=.6,linear_error_m_s=.25,yaw_error_rad_s=.35),
                    tracking_phases=self.tracking_guard.events,published_commands=self.published,diagnostics=self.diagnostics,diagnostic_topics_observed=list(self.diagnostic_times))
        if self.samples:
            result.update(measured_max_linear_m_s=max(abs(v['vx']) for v in self.samples),
                          measured_max_yaw_rad_s=max(abs(v['wz']) for v in self.samples),
                          measured_path_m=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(self.samples,self.samples[1:])))
        self.executor.shutdown();self.node.destroy_node();self.context.shutdown();self.thread.join(timeout=2)
        return result


def controlled_turn_test(transition=False):
    """Bounded positive-yaw diagnostic; no raw/LeRobot trajectory is loaded."""
    import sys,threading
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int32,Bool
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
    alive={'t':time.monotonic(),'eof':False}
    def heartbeat():
        for line in sys.stdin:alive['t']=time.monotonic()
        alive['eof']=True
    threading.Thread(target=heartbeat,daemon=True).start()
    def interrupted(*_):raise RuntimeError('Interrupted')
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    c=CoordinatedChassis(None,rows=[(0,[0.]*6),(20_000_000,[0.]*6)])
    rclpy.init(domain_id=29);node=rclpy.create_node('controlled_turn_stationary_guard');state={}
    node.create_subscription(JointState,'/tj/joint_states',lambda m:state.update(j=m,jt=time.monotonic()),qos_profile_sensor_data)
    node.create_subscription(Int32,'/tj/info/ik_request_mux/active_source',lambda m:state.update(ik=m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Bool,'/tj/info/vr_connected',lambda m:state.update(vr=m.data),qos_profile_sensor_data)
    report=dict(operation='controlled_positive_yaw',start_unix_ns=time.time_ns(),status='preflight',
                command_yaw_limit=.3,measured_yaw_limit=.65,angle_limit_rad=.35,duration_limit_s=3.,linear_drift_limit_m=.05)
    if transition:report.update(operation='straight_to_turn_transition',command_yaw_limit=.5,duration_limit_s=7.5,linear_drift_limit_m=.30,reverse_command_m_s=-.1,zero_interval_s=.75)
    report['phases']=[]
    try:
        end=time.monotonic()+6
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            if all(k in state for k in ('j','ik','vr')) and len(c.diagnostic_times)>=4:break
        def check():
            now=time.monotonic();c.check()
            if alive['eof'] or now-alive['t']>.5:raise RuntimeError('Host heartbeat lost')
            if now-state.get('jt',0)>.2 or state.get('ik')!=0 or state.get('vr') is not False:raise RuntimeError('Upper-body idle/feedback/VR guard')
            if len(state['j'].velocity)!=19 or not all(math.isfinite(v) for v in state['j'].velocity) or max(map(abs,state['j'].velocity))>.02:raise RuntimeError('Upper-body not stationary')
            latest={d['topic']:d for d in c.diagnostics}
            for topic in ('/driver/motor/Data','/driver/motor/SetMotorCmd','/driver/imu/Data','/driver/stm32/Data'):
                if topic not in latest or now-latest[topic]['t']>.2:raise RuntimeError('Missing/stale diagnostic '+topic)
            motors=latest['/driver/motor/Data']['message']
            for m in motors['wheel_data']+motors['steer_data']:
                if not m['is_connected'] or not m['is_enable'] or m['error_code']:raise RuntimeError('Motor fault')
            stm=latest['/driver/stm32/Data']['message']
            if stm['is_hard_scram'] or stm['is_soft_scram'] or not stm['is_connected'] or stm['error_code']:raise RuntimeError('Chassis emergency/fault')
            return latest
        check();c.ready();m=c.state['message'];x0=m.odometry.position.x;y0=m.odometry.position.y
        start=last=time.monotonic();yaw=imu_yaw=0.;no_turn_since=None;c.began=True
        phase='reverse' if transition else 'turn';phase_start=start
        report['phases'].append(dict(phase=phase,t=start))
        report['motion_start']=start;print('CONTROLLED_TURN_STARTED',flush=True)
        while True:
            rclpy.spin_once(node,timeout_sec=.005);latest=check();now=time.monotonic();dt=now-last;last=now
            m=c.state['message'];gyro=latest['/driver/imu/Data']['message']['angular_velocity']['z']
            if not math.isfinite(gyro):raise RuntimeError('Invalid IMU gyro')
            yaw+=m.speed.angular.z*dt;imu_yaw+=gyro*dt
            drift=math.hypot(m.odometry.position.x-x0,m.odometry.position.y-y0)
            if abs(m.speed.linear.x)>(.25 if transition else .1) or abs(m.speed.linear.y)>.05 or drift>(.30 if transition else .05):raise RuntimeError('Unexpected translation')
            if abs(gyro)>.65 or m.speed.angular.z<-.1 or gyro<-.1:raise RuntimeError('Unexpected turn speed/direction')
            elapsed=now-start;phase_elapsed=now-phase_start
            report.update(elapsed_s=elapsed,integrated_state_yaw=yaw,integrated_imu_yaw=imu_yaw,drift_m=drift,current_phase=phase)
            if elapsed>7.5:raise RuntimeError('Transition overall timeout')
            if max(abs(yaw),abs(imu_yaw))>=.35:
                report.update(status='completed',completion_reason='angle_stop_trigger');break
            if transition and phase=='reverse':
                if abs(gyro)>.1:raise RuntimeError('Rotation during straight setup')
                motors=latest['/driver/motor/Data']['message']
                aligned=all(abs(v['position'])<.05 for v in motors['steer_data'])
                if phase_elapsed>=1.0 and aligned and m.speed.linear.x<-.03:
                    phase='zero';phase_start=now;report['phases'].append(dict(phase=phase,t=now));values=[0.]*6
                elif phase_elapsed>3:raise RuntimeError('Straight-wheel setup not confirmed')
                else:values=[-min(.1,.2*phase_elapsed),0.,0.,0.,0.,0.]
            elif transition and phase=='zero':
                values=[0.]*6
                if phase_elapsed>=.75 and abs(m.speed.linear.x)<.02 and abs(m.speed.angular.z)<.02:
                    phase='turn';phase_start=now;report['phases'].append(dict(phase=phase,t=now))
                elif phase_elapsed>1.5:raise RuntimeError('Straight segment did not stop')
            else:
                if phase_elapsed>=(2 if transition else 3):
                    report.update(status='completed',completion_reason='duration_limit');break
                no_turn=phase_elapsed>1 and abs(m.speed.angular.z)<.03 and abs(gyro)<.03
                no_turn_since=(no_turn_since or now) if no_turn else None
                if no_turn_since and now-no_turn_since>.8:raise RuntimeError('No turning response for 0.8s after ramp')
                values=[0.,0.,0.,0.,0.,min(.5,2.5*phase_elapsed) if transition else min(.3,.5*phase_elapsed)]
            c.tracking(values);c.publish(values,int(elapsed*1e9));time.sleep(.01)
    except BaseException as exc:report.update(status='failed',error=str(exc))
    finally:
        report['chassis']=c.finish()
        if c.began and not report['chassis']['stop_confirmed']:report['status']='failed'
        path=Path('/home/user/codex-robot-test')/('controlled-turn-'+str(report['start_unix_ns'])+'.json')
        path.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({**{k:v for k,v in report.items() if k!='chassis'},'stop_confirmed':report['chassis']['stop_confirmed'],'report_path':str(path)}),flush=True)
        node.destroy_node();rclpy.shutdown()
    if report['status']!='completed':raise SystemExit(2)


if __name__ == "__main__":
    import sys
    if '--transition-test' in sys.argv:controlled_turn_test(transition=True)
    elif '--turn-test' in sys.argv:controlled_turn_test()
    else:main()
