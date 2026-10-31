"""Independent hand process supervisor. Default is a no-connect plan."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from hands_common import ROOT, config, normalize, ensure_sdk_ports_free

def driver_main():
    import rclpy
    from sharpa_driver.node import SharpaDriverNode
    class GuardedDriver(SharpaDriverNode):
        def __init__(self):
            self.fault=None
            super().__init__()
        def _on_command(self,side,message):
            try:
                if self.fault:raise RuntimeError('Latched hand fault; restart required: '+self.fault)
                if self._timeout_stopped[side]:raise RuntimeError('Native command watchdog stopped hand; restart required')
                normalize(message.name,message.position,side.value)
                age=self.get_clock().now().nanoseconds-(message.header.stamp.sec*10**9+message.header.stamp.nanosec)
                if not 0<=age<=int(config()['max_input_age_s']*1e9):raise ValueError('Hand command stale or clock in future')
                topic=f'/sharpa/{side.value}/request'
                writers=self.get_publishers_info_by_topic(topic)
                if len(writers)!=1 or writers[0].node_name not in ('manus_sharpa_retarget','hands_replay'):
                    raise RuntimeError('Competing or unexpected command publisher')
                super()._on_command(side,message)
            except Exception as e:
                self.fault=str(e)
                for hand in self._hands.values():hand.stop()
                raise
    rclpy.init()
    node=None
    try:
        node=GuardedDriver();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

def main():
    if '--driver' in sys.argv:
        sys.argv.remove('--driver');driver_main();return
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true',help='Explicitly connect/enable real MANUS and Sharpa')
    p.add_argument('--fake',action='store_true',help='All hardware mocked; isolated local DDS; no SSH recorder')
    p.add_argument('--duration',type=float,default=0)
    p.add_argument('--replay',type=Path)
    a=p.parse_args();c=config()
    if a.execute and a.fake:p.error('Choose execute or fake')
    if not a.execute and not a.fake:
        print(json.dumps(dict(mode='plan-only',config=c,hardware_initialized=False),indent=2));return
    if a.replay:
        from replay import load_plan
        load_plan(a.replay)
    lock=(ROOT/'.hands.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if a.execute:ensure_sdk_ports_free()
    children=[];files=[];node=None;bridge=None;failure=None
    def stop_signal(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop_signal)
    if a.execute:
        # Only hand-owned files on the robot; no edits to original services/config.
        subprocess.run(['ssh','-o','BatchMode=yes',c['robot_ssh'],'mkdir -p /home/user/manus_sharpa'],check=True)
        subprocess.run(['scp',str(ROOT/'recording/robot_receiver.py'),c['robot_ssh']+':/home/user/manus_sharpa/robot_receiver.py'],check=True)
    from recording.bridge import RecordBridge
    import rclpy
    rclpy.init();node=rclpy.create_node('hands_supervisor')
    try:
        bridge=RecordBridge(node,fake=a.fake)
        if a.execute:
            deadline=time.monotonic()+10
            while bridge.last_ack==0 and time.monotonic()<deadline:time.sleep(.05)
            # Explicit readiness flag; failure precedes all device initialization.
            while not getattr(bridge,'ready',False) and time.monotonic()<deadline:
                if bridge.error:raise bridge.error
                time.sleep(.05)
            if not getattr(bridge,'ready',False):raise TimeoutError('Robot hand recorder not ready')
        def launch(name,cmd,first=False):
            f=(ROOT/'logs'/f'{time.time_ns()}-{name}.log').open('w');files.append(f)
            child=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
            if first:children.insert(0,child)
            else:children.append(child)
        fake='true' if a.fake else 'false'
        if not a.replay:
            launch('manus',['ros2','run','manus_driver','manus_driver','--ros-args','-p','use_fake_hardware:='+fake,
                           '-p','calibration_directory:='+str(ROOT/'portable_deps/litchi_hardware/calibration/metaglove')])
            launch('retarget',['ros2','run','manus_sharpa_teleop','retarget_node','--ros-args',
                              '-r','__node:=manus_sharpa_retarget','-p','enable_left:=true','-p','enable_right:=true',
                              '-p','backend:='+('analytic' if a.fake else 'v4'),
                              '-p','v4_worker_python:='+os.environ.get('LITCHI_RETARGET_V4_PYTHON',''),
                              '-p','left_command_topic:=/sharpa/left/request','-p','right_command_topic:=/sharpa/right/request',
                              '-p','max_input_age_s:='+str(c['max_input_age_s'])])
            # No actuator is connected during source initialization/optimizer warmup.
            deadline=time.monotonic()+20
            while not all(bridge.request_counts.get(s,0)>=10 and time.monotonic()-bridge.last_request.get(s,0)<.1 for s in ('left','right')):
                rclpy.spin_once(node,timeout_sec=.01)
                if bridge.error:raise bridge.error
                if any(ch.poll() is not None for ch in children):raise RuntimeError('MANUS/V4 warmup process exited')
                if time.monotonic()>deadline:raise TimeoutError('Dual-hand MANUS/V4 targets not ready; Sharpa not initialized')
            print('Dual MANUS/V4 targets ready; connecting Sharpa hands only',flush=True)
        args=['--ros-args','-p','use_fake_hardware:='+fake,'-p','enable_left:=true','-p','enable_right:=true',
              '-p','left_serial:='+c['serials']['left'],'-p','right_serial:='+c['serials']['right'],
              '-p','state_rate_hz:='+str(c['state_rate_hz']),'-p','stop_on_command_timeout_s:='+str(c['command_timeout_s']),'-p','publish_tactile:=false',
              '-p','speed_coefficient:='+str(c['speed_coefficient']),'-p','current_coefficient:='+str(c['current_coefficient']),
              '-p','left_command_topic:=/sharpa/left/request','-p','right_command_topic:=/sharpa/right/request']
        launch('driver',[sys.executable,str(ROOT/'runtime.py'),'--driver',*args],first=True)
        # Require actual feedback before a producer can publish commands.
        deadline=time.monotonic()+20
        while not all((s,'state') in bridge.last_data for s in ('left','right')):
            rclpy.spin_once(node,timeout_sec=.05)
            if bridge.error:raise bridge.error
            if any(ch.poll() is not None for ch in children):raise RuntimeError('Hand driver exited; see logs')
            if time.monotonic()>deadline:raise TimeoutError('Initial Sharpa feedback missing')
        if a.replay:
            launch('replay',[sys.executable,str(ROOT/'replay.py'),str(a.replay),'--publish'])
        started=time.monotonic()
        while not a.duration or time.monotonic()-started<a.duration:
            rclpy.spin_once(node,timeout_sec=.02)
            if bridge.error:raise bridge.error
            if not a.fake and time.monotonic()-bridge.last_ack>2:raise TimeoutError('Robot recording acknowledgement lost')
            for key,t in bridge.last_data.items():
                if key[1]=='state' and time.monotonic()-t>.5:raise RuntimeError('Sharpa feedback stale')
                if key[1]=='accepted' and time.monotonic()-t>.5:raise RuntimeError('Hand action stream stopped; restart required')
            if time.monotonic()-started>5 and not all((s,'accepted') in bridge.last_data for s in ('left','right')):
                raise TimeoutError('MANUS/replay produced no accepted dual-hand actions')
            exited=[(i,ch.returncode) for i,ch in enumerate(children) if ch.poll() is not None]
            if exited:
                if a.replay and exited==[(1,0)]:break
                raise RuntimeError('Hand subprocess exited: '+str(exited))
        if a.fake and not all((s,'accepted') in bridge.last_data for s in ('left','right')):
            raise RuntimeError('Fake smoke did not observe dual SDK receipts')
    except KeyboardInterrupt:pass
    except Exception as exc:
        failure=str(exc)
        raise
    finally:
        # Stop producers first, then let native watchdog disable, then close driver.
        for ch in reversed(children[1:]):
            if ch.poll() is None:os.killpg(ch.pid,signal.SIGINT)
        until=time.monotonic()+.3
        while node and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
        if children and children[0].poll() is None:os.killpg(children[0].pid,signal.SIGINT)
        for ch in children:
            try:ch.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(ch.pid,signal.SIGTERM);ch.wait(timeout=3)
        if bridge:bridge.close()
        if bridge:
            (ROOT/'reports'/f'runtime-{bridge.session}.json').write_text(json.dumps({
                'fake':a.fake,'session_id':bridge.session,'event_count':bridge.seq,
                'streams':sorted('/'.join(k) for k in bridge.last_data),'error':failure or (str(bridge.error) if bridge.error else None),
                'robot_control_commands_sent':0},indent=2))
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for f in files:f.close()

if __name__=='__main__':main()
