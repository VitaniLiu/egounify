import json,time
from types import SimpleNamespace as NS
from litchi_hardware.hardware.sharpa import create_hand,SharpaConfig,SHARPA_JOINT_NAMES
from litchi_hardware.core.types import HandSide
from sensor_msgs.msg import JointState
from sharpa_driver import node as mod

def test_receipt_publisher_does_not_request_reliable_backpressure():
    import rclpy
    from rclpy.qos import ReliabilityPolicy
    rclpy.init(args=['--ros-args','-p','use_fake_hardware:=true','-p','enable_left:=true'])
    node=None
    try:
        node=mod.SharpaDriverNode()
        assert node.get_parameter('use_fake_hardware').value is True
        assert node._result_publishers[HandSide.LEFT].qos_profile.reliability==ReliabilityPolicy.BEST_EFFORT
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()

def test_driver_outcome_distinguishes_sdk_return_from_rejected_and_has_identity():
    side=HandSide.LEFT;hand=create_hand(SharpaConfig(name='offline',side=side),use_fake_hardware=True);hand.connect()
    reports=[]
    obj=NS(_hands={side:hand},_last_command_at={},_has_received_command={},_timeout_stopped={},
           _result_publishers={side:NS(publish=lambda msg:reports.append(json.loads(msg.data)))},
           get_clock=lambda:NS(now=lambda:NS(nanoseconds=time.time_ns())),
           get_logger=lambda:NS(error=lambda text:None))
    try:
        for value in (0.,float('inf')):
            msg=JointState();msg.header.stamp.sec=10;msg.header.stamp.nanosec=3
            msg.name=['left_'+n for n in SHARPA_JOINT_NAMES];msg.position=[value]*22
            mod.SharpaDriverNode._on_command(obj,side,msg)
        assert len(reports)==2
        assert reports[0]['status']=='sdk_call_succeeded'
        assert reports[1]['status']=='rejected'
        assert 'finite' in reports[1]['reason']
        assert reports[0]['command_stamp_ns']==10_000_000_003
        assert reports[0]['side']=='left'
        assert reports[0]['execution_confirmed'] is False
        assert reports[1]['execution_confirmed'] is False
        assert hand.read_joint_state().position==(0.,)*22
    finally:hand.disconnect()

def test_reporting_failure_never_disables_or_rejects_successful_hand_command():
    side=HandSide.LEFT;hand=create_hand(SharpaConfig(name='offline',side=side),use_fake_hardware=True);hand.connect()
    def fail(msg):raise RuntimeError('reporting only')
    obj=NS(_hands={side:hand},_last_command_at={},_has_received_command={},_timeout_stopped={},
           _result_publishers={side:NS(publish=fail)},get_clock=lambda:NS(now=lambda:NS(nanoseconds=time.time_ns())),
           get_logger=lambda:NS(error=lambda text:None))
    msg=JointState();msg.header.stamp.sec=10;msg.position=[.1]*22
    try:
        mod.SharpaDriverNode._on_command(obj,side,msg)
        assert obj._has_received_command[side] is True
        assert hand.read_joint_state().position==(.1,)*22
    finally:hand.disconnect()
