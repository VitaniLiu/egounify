"""Isolated ROS fixtures. Never create real hand/chassis hardware."""
import json,os,sys,time
from pathlib import Path
import xml.etree.ElementTree as ET
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from unified_replay.integration.api import DOMAIN,GROUPS,lab_guard,ROBOT_MODE_STATE
MODEL=Path('/opt/kernelmind/apex/install/marvin_description/share/marvin_description/mjcf/Gento_Sky/Gento_Sky.xml')


def model_home():
    tree=ET.fromstring(MODEL.read_text());names=[j.get('name') for j in tree.findall('.//worldbody//joint')]
    q=list(map(float,tree.find("keyframe/key[@name='home']").get('qpos').split()))
    return dict(zip(names,q))


def robot(ns):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
    from std_msgs.msg import Int32,Int16MultiArray,Bool,String
    from geometry_msgs.msg import TwistStamped
    from move.msg import State
    from marvin_msgs.msg import Jointfeedback,JointcmdArm,JointcmdBody,JointcmdHead
    from marvin_msgs.srv import Int
    rclpy.init(domain_id=DOMAIN);node=rclpy.create_node('fixture_robot',namespace=ns)
    home=model_home();latest={};plant={};source=1;velocity=[0.,0.,0.];commands=[];fault='';frozen=False
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    source_pub=node.create_publisher(Int32,ns+'/info/ik_request_mux/active_source',latched)
    source_pub.publish(Int32(data=source))
    def select(req,res):
        nonlocal source
        if req.data not in (0,1,3):res.success=False;return res
        source=int(req.data);source_pub.publish(Int32(data=source));res.success=True;res.error_code=0;res.message='isolated mock mux';return res
    node.create_service(Int,ns+'/control/ik_request_mux/select',select)
    for group,typ in [('A',JointcmdArm),('B',JointcmdArm),('body',JointcmdBody),('head',JointcmdHead)]:
        node.create_subscription(typ,ns+'/output/joint_cmd_'+group,lambda m,g=group:latest.update({g:list(m.positions)}),qos_profile_sensor_data)
    def command(m):
        nonlocal velocity
        velocity=[m.twist.linear.x,m.twist.linear.y,m.twist.angular.z];commands.append(velocity)
    node.create_subscription(TwistStamped,ns+'/move/ManualMoveCmd',command,10)
    def inject(m):
        nonlocal fault
        fault=m.data
    node.create_subscription(String,ns+'/fixture/fault',inject,10)
    fb=node.create_publisher(Jointfeedback,ns+'/info/joint_feedback',qos_profile_sensor_data)
    chassis=node.create_publisher(State,ns+'/move/State',qos_profile_sensor_data)
    state=node.create_publisher(Int16MultiArray,ns+'/info/robot_state',10)
    vr=node.create_publisher(Bool,ns+'/info/vr_connected',10)
    modes=[]
    for topic,value in [('info/joint_cmd_mux/active_source',0),('info/body_pose_mode',0),('info/teleop_motion_mode',0),('info/base_teleop/active_mode',0),('info/base_teleop/reanchor_seq',0)]:
        p=node.create_publisher(Int32,ns+'/'+topic,latched);p.publish(Int32(data=value));modes.append((p,value))
    def tick():
        msg=Jointfeedback();msg.header.stamp=node.get_clock().now().to_msg()
        if source!=0:plant.update({k:list(v) for k,v in latest.items()})
        msg.arm_positions=plant.get('A',[home[k] for k in GROUPS['A']])+plant.get('B',[home[k] for k in GROUPS['B']])
        msg.body_positions=plant.get('body',[home[k] for k in GROUPS['body']]+[0.,0.,0.])
        msg.head_positions=plant.get('head',[home[k] for k in GROUPS['head']]+[0.])
        if fault!='body_feedback_loss':fb.publish(msg)
        m=State();m.header.stamp=msg.header.stamp;m.speed.linear.x=velocity[0];m.speed.linear.y=velocity[1];m.speed.angular.z=velocity[2];m.is_hard_auto=True
        if fault!='chassis_feedback_loss':chassis.publish(m)
        state.publish(Int16MultiArray(data=ROBOT_MODE_STATE));vr.publish(Bool(data=False))
        for p,value in modes:p.publish(Int32(data=value))
    node.create_timer(.01,tick)
    try:rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def hands(ns):
    import rclpy
    from std_srvs.srv import Trigger
    from std_msgs.msg import String
    # Force the actual installed ROS driver to instantiate only fake actuators.
    import sharpa_driver.node as driver
    from litchi_hardware.hardware.sharpa.mock import MockSharpaHand
    from litchi_hardware.core.types import HandSide
    def fake_factory(config,*,use_fake_hardware=False):
        if use_fake_hardware is not True:raise PermissionError('Real Sharpa construction forbidden in integration')
        return MockSharpaHand(config)
    driver.create_hand=fake_factory
    args=['--ros-args','-r','__ns:='+ns,'-p','use_fake_hardware:=true','-p','enable_left:=true','-p','enable_right:=true',
          '-p','left_serial:=CE50903ECE53','-p','right_serial:=CC50933FCC53',
          '-p','publish_tactile:=false','-p','state_rate_hz:=100.0','-p','stop_on_command_timeout_s:=0.2']
    for side in ('left','right'):
        args+=['-p',side+'_command_topic:='+ns+'/sharpa/'+side+'/request','-p',side+'_state_topic:='+ns+'/sharpa/'+side+'/joint_states',
               '-r','/sharpa/'+side+'/command_result:='+ns+'/sharpa/'+side+'/command_result']
    rclpy.init(args=args,domain_id=DOMAIN)
    class ReplayDriver(driver.SharpaDriverNode):
        def __init__(self):
            self.latched=set();self.injected='';super().__init__()
            if not all(isinstance(hand,MockSharpaHand) for hand in self._hands.values()):raise PermissionError('Nonfake hand detected')
            for side in (HandSide.LEFT,HandSide.RIGHT):
                self.create_service(Trigger,ns+'/sharpa/'+side.value+'/stop',lambda req,res,s=side:self.stop_side(s,res))
            self.create_subscription(String,ns+'/fixture/fault',lambda m:setattr(self,'injected',m.data),10)
        def stop_side(self,side,res):
            self.latched.add(side);self._hands[side].stop();res.success=self._hands[side]._stopped;res.message='MockSharpaHand stop confirmed';return res
        def _on_command(self,side,msg):
            if side in self.latched:return
            if self.injected==side.value+'_sdk_reject':self._hands[side].stop()
            age=self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec)
            if not 0<=age<=250_000_000:raise RuntimeError('Stale request')
            if self.count_publishers(ns+'/sharpa/'+side.value+'/request')!=1:raise RuntimeError('Competing hand publisher')
            super()._on_command(side,msg)
    node=ReplayDriver()
    try:rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    role,ns=sys.argv[1:3];lab_guard(ns)
    if role=='robot':robot(ns)
    elif role=='hands':hands(ns)
    else:raise ValueError('Unknown fixture role')
