import importlib.util
from pathlib import Path
import pytest
s=importlib.util.spec_from_file_location('chassis',Path(__file__).resolve().parents[1]/'src/chassis_replay/main.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
def test_source_and_integration():
    r=[(i*20_000_000,[.4 if i<50 else 0,0,0,0,0,0]) for i in range(51)]
    m.validate(r);assert m.TOPIC=='/recording/chassis/manual_move_cmd'
    assert m.summary(r)['command_integrated_path_m']==pytest.approx(.4)
@pytest.mark.parametrize('fault',['gap','reverse','overspeed','lateral','nan','no_stop'])
def test_invalid_replay_rejected(fault):
    r=[(0,[.1,0,0,0,0,0]),(20_000_000,[0]*6)]
    if fault=='gap':r[1]=(200_000_000,[0]*6)
    if fault=='reverse':r[1]=(0,[0]*6)
    if fault=='overspeed':r[0][1][0]=.41
    if fault=='lateral':r[0][1][1]=.1
    if fault=='nan':r[0][1][5]=float('nan')
    if fault=='no_stop':r[1][1][0]=.1
    with pytest.raises(ValueError):m.validate(r)


def update_guard(g,t,error=.8,wheel=0.,measured=(0.,0.,0.),command=(0.,.5),gyro=0.):
    return g.update(t,command,measured,error,wheel,gyro)


def test_prepare_then_original_tracking_deadline():
    g=m.ChassisTrackingGuard()
    assert update_guard(g,0.)=='preparing'
    update_guard(g,.2,error=.6)
    update_guard(g,.4,error=.1)
    assert update_guard(g,.45,error=.02,wheel=.01)=='tracking'
    update_guard(g,1.04,error=.02,wheel=.1)
    with pytest.raises(RuntimeError,match='Sustained chassis speed'):
        update_guard(g,1.06,error=.02,wheel=.1)


def test_stuck_steering_rejected_before_speed_tracking():
    g=m.ChassisTrackingGuard();update_guard(g,0)
    with pytest.raises(RuntimeError,match='failed to converge'):
        update_guard(g,.401)


def test_preparation_has_absolute_deadline_despite_progress():
    g=m.ChassisTrackingGuard()
    for t in [0,.2,.4,.6,.8,1.]:update_guard(g,t,error=.8-.4*t)
    with pytest.raises(RuntimeError,match='preparation timeout'):
        update_guard(g,1.01,error=.39)


@pytest.mark.parametrize('kwargs,reason',[
    ({'wheel':.051},'Wheel creep exceeded'),
    ({'measured':(.06,0,0)},'Unexpected motion'),
    ({'gyro':.11},'Unexpected motion'),
    ({'error':float('nan')},'Nonfinite')])
def test_preparation_preserves_other_protections(kwargs,reason):
    g=m.ChassisTrackingGuard();update_guard(g,0)
    with pytest.raises(RuntimeError,match=reason):update_guard(g,.1,**kwargs)


def test_cannot_reset_tracking_error_with_new_steering_target():
    g=m.ChassisTrackingGuard();update_guard(g,0)
    update_guard(g,.2,error=.02,wheel=.01)
    assert update_guard(g,.3,error=.8)=='tracking'
    with pytest.raises(RuntimeError,match='Sustained chassis speed'):
        update_guard(g,.81,error=.8)


def test_successful_tracking_rearms_next_preparation():
    g=m.ChassisTrackingGuard();update_guard(g,0)
    update_guard(g,.2,error=.02,wheel=.1,measured=(0,0,.3))
    update_guard(g,.41,error=.02,wheel=.1,measured=(0,0,.3))
    assert update_guard(g,.5,error=.8)=='preparing'


def test_aligned_start_keeps_original_error_deadline():
    g=m.ChassisTrackingGuard();update_guard(g,0,error=.01,wheel=.1)
    with pytest.raises(RuntimeError,match='Sustained chassis speed'):
        update_guard(g,.61,error=.01,wheel=.1)


def test_driver_creep_during_converging_steering_is_bounded():
    g=m.ChassisTrackingGuard();update_guard(g,0)
    update_guard(g,.16,error=.55,wheel=.006358624,measured=(.01,.01,.02),gyro=.01)
    update_guard(g,.3,error=.20,wheel=.02,measured=(.01,.01,.02),gyro=.01)
    assert update_guard(g,.45,error=.02,wheel=.04)=='tracking'


def test_preparation_translation_budget_cannot_accumulate_indefinitely():
    g=m.ChassisTrackingGuard();update_guard(g,0)
    update_guard(g,.2,error=.6,measured=(.04,0,0))
    update_guard(g,.4,error=.4,measured=(.04,0,0))
    update_guard(g,.59,error=.25,measured=(.04,0,0))
    with pytest.raises(RuntimeError,match='motion budget'):
        update_guard(g,.8,error=.1,measured=(.04,0,0))


def test_wheel_feedback_protection_during_preparation():
    g=m.ChassisTrackingGuard();update_guard(g,0)
    with pytest.raises(RuntimeError,match='Wheel creep'):
        g.update(.1,(0,.5),(0,0,0),.7,.01,0,wheel_feedback=.081)
