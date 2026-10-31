import json,time
from types import SimpleNamespace as NS
import pytest
from sensor_msgs.msg import JointState
from litchi_hardware.core.types import HandSide
from litchi_hardware.hardware.sharpa import create_hand,SharpaConfig,SHARPA_JOINT_NAMES
from sharpa_driver.node import SharpaDriverNode

@pytest.fixture
def driver():
    side=HandSide.LEFT
    hand=create_hand(SharpaConfig(name='offline-clamp',side=side),use_fake_hardware=True)
    hand.connect();reports=[]
    node=NS(_hands={side:hand},_last_command_at={},_has_received_command={},_timeout_stopped={},
        _result_publishers={side:NS(publish=lambda m:reports.append(json.loads(m.data)))},
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=time.time_ns())),get_logger=lambda:NS(error=lambda _:None))
    yield node,hand,reports
    hand.disconnect()

def send(node,values):
    msg=JointState();msg.header.stamp.sec=10;msg.name=['left_'+n for n in SHARPA_JOINT_NAMES];msg.position=values
    SharpaDriverNode._on_command(node,HandSide.LEFT,msg)

def test_out_of_range_values_are_clamped_before_send_and_receipt_matches(driver):
    node,hand,reports=driver
    values=[.1]*22;values[5]=-1.;values[18]=2.
    send(node,values)
    expected=[.1]*22;expected[5]=-.17453293;expected[18]=1.5708
    assert list(hand.read_joint_state().position)==expected
    r=reports[0]
    assert r['schema']=='sharpa_command_result_v2'
    assert r['target']==expected and r['requested_target']==values
    assert r['clamped_joints']==['index_MCP_FE','pinky_MCP_FE']
    assert r['status']=='sdk_call_succeeded' and r['execution_confirmed'] is False

@pytest.mark.parametrize('value',[float('nan'),float('inf'),-float('inf')])
def test_nonfinite_is_rejected_not_clamped_to_a_limit(driver,value):
    node,hand,reports=driver;values=[.1]*22;values[5]=value;send(node,values)
    assert hand.read_joint_state().position==(0.,)*22
    assert reports[0]['status']=='rejected'
    assert 'target' not in reports[0]

def test_sdk_failure_after_clamping_is_not_reported_success(driver):
    node,hand,reports=driver
    def fail(_):raise RuntimeError('SDK transport failure')
    hand._write_joint_position=fail
    values=[.1]*22;values[5]=-1.;send(node,values)
    assert reports[0]['status']=='rejected'
    assert reports[0]['target'][5]==-.17453293
    assert 'SDK transport failure' in reports[0]['reason']
    assert not node._has_received_command
