"""Native ROS message/service adapters; laboratory profile enforced before ROS init."""
import json,math,time
from .api import DOMAIN,GROUPS,ROLES,HAND_NAMES,fill_ik,fill_hand,fill_chassis,check_hand_receipt,lab_guard,ROBOT_MODE_STATE


class RosAdapter:
    def __init__(self,role,namespace):
        lab_guard(namespace)
        import rclpy
        from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
        from std_msgs.msg import String,Int32,Int16MultiArray
        from std_srvs.srv import Trigger
        from sensor_msgs.msg import JointState
        from geometry_msgs.msg import TwistStamped
        self.ros=rclpy;rclpy.init(domain_id=DOMAIN);self.node=rclpy.create_node('unified_replay_'+role+'_agent',namespace=namespace)
        self.role=role;self.ns=namespace;self.pub={};self.types={};self.feedback={};self.pending={};self.receipts=[];self.outputs={};self.source=None;self.error=None;self.stopped=False;self.chassis_interlock=False;self.robot_mode=None
        self.stop_clients={};self.control_calls=[]
        self.fault_pub=self.node.create_publisher(String,namespace+'/fixture/fault',10)
        self.String=String
        def feedback(channel,value):self.feedback[channel]=(time.monotonic_ns(),value)
        if role=='robot':
            from marvin_msgs.msg import IKRequest,Jointfeedback,JointcmdArm,JointcmdBody,JointcmdHead
            from marvin_msgs.srv import Int
            from move.msg import State
            self.Int=Int
            self.pub['body']=self.node.create_publisher(IKRequest,namespace+'/input/ik',10);self.types['body']=IKRequest
            self.pub['chassis']=self.node.create_publisher(TwistStamped,namespace+'/move/ManualMoveCmd',10);self.types['chassis']=TwistStamped
            self.node.create_subscription(Jointfeedback,namespace+'/info/joint_feedback',lambda m:feedback('body',list(m.arm_positions)+list(m.body_positions[:3])+list(m.head_positions[:2])),qos_profile_sensor_data)
            def chassis_feedback(m):
                self.chassis_interlock=bool(m.is_hard_auto and not (m.is_soft_auto or m.is_joystick_on or m.is_handle_on or m.is_parking_mode or m.is_avoid))
                feedback('chassis',[m.speed.linear.x,m.speed.linear.y,m.speed.angular.z])
            self.node.create_subscription(State,namespace+'/move/State',chassis_feedback,qos_profile_sensor_data)
            self.node.create_subscription(Int16MultiArray,namespace+'/info/robot_state',lambda m:setattr(self,'robot_mode',(time.monotonic_ns(),list(m.data))),10)
            import xml.etree.ElementTree as ET
            from .fixtures import MODEL
            self.limits={j.get('name'):list(map(float,j.get('range').split())) for j in ET.fromstring(MODEL.read_text()).findall('.//worldbody//joint') if j.get('range')}
            for group,typ in [('A',JointcmdArm),('B',JointcmdArm),('body',JointcmdBody),('head',JointcmdHead)]:
                def output(m,g=group):
                    q=list(m.positions)
                    if not all(math.isfinite(v) for v in q):self.error='nonfinite_controller_output'
                    for joint,value in zip(GROUPS[g],q):
                        if joint in self.limits and not self.limits[joint][0]-1e-5<=value<=self.limits[joint][1]+1e-5:self.error='controller_joint_limit:'+joint
                    self.outputs.setdefault(g,[]).append({'time_ns':time.monotonic_ns(),'position':q})
                self.node.create_subscription(typ,namespace+'/output/joint_cmd_'+group,output,qos_profile_sensor_data)
            latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.node.create_subscription(Int32,namespace+'/info/ik_request_mux/active_source',lambda m:setattr(self,'source',m.data),latched)
            self.mux=self.node.create_client(Int,namespace+'/control/ik_request_mux/select')
            self.home=self.node.create_client(Trigger,namespace+'/control/go_home')
        else:
            for side in ('left','right'):
                channel=side+'_hand';self.types[channel]=JointState
                self.pub[channel]=self.node.create_publisher(JointState,namespace+'/sharpa/'+side+'/request',10)
                def on_state(m,c=channel,s=side):
                    names=[n.removeprefix(s+'_') for n in m.name]
                    if names!=HAND_NAMES:self.error='feedback_joint_order';return
                    feedback(c,list(m.position))
                def receipt(m,c=channel):
                    try:
                        r=json.loads(m.data);key=(c,r['command_stamp_ns']);expected=self.pending.pop(key,None)
                        if expected is None:raise ValueError('Unknown SDK receipt')
                        if r.get('side')!=c.removesuffix('_hand'):raise ValueError('Wrong receipt side')
                        check_hand_receipt(r,key[1],expected['payload'])
                        self.receipts.append({'channel':c,'event_index':expected['event_index'],'source_hash':expected['hash'],'status':r['status'],'received_ns':time.monotonic_ns()})
                    except Exception as e:self.error=c+':'+str(e)
                self.node.create_subscription(JointState,namespace+'/sharpa/'+side+'/joint_states',on_state,qos_profile_sensor_data)
                self.node.create_subscription(String,namespace+'/sharpa/'+side+'/command_result',receipt,10)
                self.stop_clients[channel]=self.node.create_client(Trigger,namespace+'/sharpa/'+side+'/stop')
        self.Trigger=Trigger

    def spin(self,seconds=0):
        until=time.monotonic()+seconds
        while True:
            self.ros.spin_once(self.node,timeout_sec=min(.001,max(0,until-time.monotonic())))
            if time.monotonic()>=until:return

    def call(self,client,request,timeout=2):
        if not client.wait_for_service(timeout_sec=timeout):raise TimeoutError('Service unavailable')
        future=client.call_async(request);until=time.monotonic()+timeout
        while not future.done() and time.monotonic()<until:self.spin(.002)
        if not future.done():raise RuntimeError('Control service timeout:'+client.srv_name)
        if not future.result().success:raise RuntimeError('Control service failed:'+client.srv_name+':'+future.result().message)
        self.control_calls.append({'service':client.srv_name,'time_ns':time.monotonic_ns(),'success':True})
        return future.result()

    def ready(self):
        for c in ROLES[self.role]:
            if c not in self.feedback or time.monotonic_ns()-self.feedback[c][0]>100_000_000:return False
            if self.pub[c].get_subscription_count()!=1:return False
        return True

    def preflight(self):
        if not self.ready():raise RuntimeError('Feedback/receiver unavailable')
        for c,p in self.pub.items():
            if self.node.count_publishers(p.topic_name)!=1:raise RuntimeError('Competing publisher:'+c)
        if self.role=='robot':
            if self.source not in (0,1):raise RuntimeError('Body source owned')
            if not self.chassis_interlock or max(map(abs,self.feedback['chassis'][1]))>.01:raise RuntimeError('Chassis not ready/stationary')
            if self.robot_mode is None or time.monotonic_ns()-self.robot_mode[0]>200_000_000 or self.robot_mode[1]!=ROBOT_MODE_STATE:raise RuntimeError('Lab robot mode invalid')
        return {c:v[1] for c,v in self.feedback.items()}

    def initialize_controller(self):
        if self.role!='robot':return
        if not self.home.wait_for_service(timeout_sec=5):raise RuntimeError('Native go_home unavailable')
        # Service discovery can precede the controller receiving its first state.
        # Feed the isolated controller several feedback/mode cycles before home.
        self.spin(.5);self.preflight()
        self.call(self.home,self.Trigger.Request());self.spin(2.5)
        if set(self.outputs)!=set(GROUPS):raise RuntimeError('Native controller did not produce all four groups')
        self.initial_outputs={k:len(v) for k,v in self.outputs.items()}

    def arm(self):
        self.preflight()
        if self.role=='robot':
            self.call(self.mux,self.Int.Request(data=3));self.spin(.02)
            if self.source!=3:raise RuntimeError('Mux selection not observed')
        self.armed_feedback={k:(t,list(q)) for k,(t,q) in self.feedback.items()}

    def check(self):
        if self.error:raise RuntimeError(self.error)
        now=time.monotonic_ns()
        for c,(t,q) in self.feedback.items():
            if now-t>100_000_000:raise RuntimeError(c+':feedback_timeout')
            if not all(math.isfinite(v) for v in q):raise RuntimeError(c+':nonfinite_feedback')
            if self.node.count_publishers(self.pub[c].topic_name)!=1:raise RuntimeError(c+':ownership_lost')
        if self.role=='robot':
            if self.source!=3:raise RuntimeError('Body ownership changed')
            if not self.chassis_interlock:raise RuntimeError('Chassis interlock changed')
            if self.robot_mode is None or now-self.robot_mode[0]>200_000_000 or self.robot_mode[1]!=ROBOT_MODE_STATE:raise RuntimeError('Lab robot mode changed')
        for (_,stamp),entry in self.pending.items():
            if now-entry['sent_ns']>150_000_000:raise RuntimeError('SDK receipt timeout')

    def dispatch(self,channel,event,index):
        if self.stopped:raise RuntimeError('Dispatch after STOP forbidden')
        self.check();stamp=self.node.get_clock().now().to_msg();m=self.types[channel]();payload=event['payload']
        if channel=='body':fill_ik(m,payload,stamp)
        elif channel=='chassis':fill_chassis(m,payload,stamp)
        else:
            fill_hand(m,payload,stamp,'unified_replay/'+str(index))
            ns=stamp.sec*10**9+stamp.nanosec
            self.pending[channel,ns]={'event_index':index,'payload':payload,'hash':event['payload_sha256'],'sent_ns':time.monotonic_ns()}
        self.pub[channel].publish(m)

    def inject(self,scenario):self.fault_pub.publish(self.String(data=scenario))

    def stop(self):
        self.stopped=True;result={}
        for c in ROLES[self.role]:
            try:
                if c=='body':
                    self.call(self.mux,self.Int.Request(data=0),.5);self.spin(.03)
                    if self.source!=0:raise RuntimeError('Body stop selection unconfirmed')
                    t,q=self.feedback[c];self.spin(.03);t2,q2=self.feedback[c]
                    if t2<=t or time.monotonic_ns()-t2>100_000_000 or max(abs(a-b) for a,b in zip(q,q2))>1e-5:raise RuntimeError('Body stopped feedback unavailable')
                    result[c]={'confirmed':True,'evidence':'mock mux source readback 0 and fresh stable ideal joint feedback'}
                elif c=='chassis':
                    m=self.types[c]();fill_chassis(m,[0.,0.,0.],self.node.get_clock().now().to_msg());self.pub[c].publish(m)
                    self.spin(.04);t,q=self.feedback[c]
                    if time.monotonic_ns()-t>100_000_000 or max(map(abs,q))>1e-6:raise RuntimeError('Chassis zero feedback unavailable')
                    result[c]={'confirmed':True,'evidence':'zero command and mock State speed readback'}
                else:
                    self.call(self.stop_clients[c],self.Trigger.Request(),.5)
                    result[c]={'confirmed':True,'evidence':'actual driver stop service; MockSharpaHand stopped'}
            except Exception as e:result[c]={'confirmed':False,'error':str(e)}
        return result

    def close(self):self.node.destroy_node();self.ros.shutdown()
