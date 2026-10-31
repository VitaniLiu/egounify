#!/usr/bin/env python3
"""Gento Record service coordinator; no playback/motor/enable/home calls."""
import fcntl
import json
import os
import sys
import threading
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from contract import Capture,Monitor,atomic_json
from audit import audit_bag

def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.callback_groups import ReentrantCallbackGroup,MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message
    from std_msgs.msg import String
    from marvin_msgs.srv import Int,GentoReplay,JsonCommand
    rclpy.init()
    node=Node('apex_record_sync')
    path=node.declare_parameter('config_path',str(Path(__file__).with_name('config.json'))).value
    cfg=json.loads(Path(path).read_text())
    domain=int(os.environ.get('ROS_DOMAIN_ID','0'))
    if cfg['diagnostic'] and domain!=199:raise RuntimeError('Diagnostic recorder must be isolated in domain199')
    if not cfg['diagnostic'] and domain!=29:raise RuntimeError('Production recorder must use robot domain29')
    lockpath=Path(cfg['state_root']);lockpath.mkdir(parents=True,exist_ok=True)
    singleton=(lockpath/'.coordinator.lock').open('a');fcntl.flock(singleton,fcntl.LOCK_EX|fcntl.LOCK_NB)
    group=ReentrantCallbackGroup();native_status={};native_at=[0.]
    monitor=Monitor(cfg)
    event_pub=node.create_publisher(String,cfg['event_topic'],1000)
    status_pub=node.create_publisher(String,'info/gento_replay/status',10)
    client=node.create_client(GentoReplay,'control/gento_replay/record_named_native',callback_group=group)
    class Native:
        def recording(self):
            if time.monotonic()-native_at[0]>2:raise RuntimeError('Native recorder status missing/stale')
            if native_status.get('playback_active'):raise RuntimeError('Native playback is active; recording denied')
            return bool(native_status.get('recording'))
        def command(self,data,name):
            if not client.wait_for_service(timeout_sec=2):raise RuntimeError('Native record service unavailable')
            requested_at=time.monotonic()
            req=GentoReplay.Request();req.data=data;req.name=name
            future=client.call_async(req);done=threading.Event();future.add_done_callback(lambda _:done.set())
            if not done.wait(10):raise TimeoutError('Native record RPC timed out')
            resp=future.result()
            if resp is None:raise RuntimeError('Native record RPC returned no response')
            if data==0 and resp.success:
                deadline=time.monotonic()+3
                while time.monotonic()<deadline:
                    status=dict(native_status)
                    if (native_at[0]>=requested_at and not status.get('recording',True)
                            and not status.get('camera_recording',True)
                            and status.get('write_queue_size')==0
                            and status.get('queued_messages')==status.get('written_messages')
                            and status.get('written_messages') is not None):
                        break
                    time.sleep(.02)
                else:
                    return {'success':False,'error_code':1,
                            'message':'Native stop lacks fresh idle/drained writer and camera status'}
            return {'success':bool(resp.success),'error_code':int(resp.error_code),'message':resp.message}
    def marker(e):
        # Native subscriptions are created by start RPC; allow DDS matching before
        # the one-shot boundary. Repeated identical markers are audited by capture ID.
        deadline=time.monotonic()+1
        while event_pub.get_subscription_count()<1 and time.monotonic()<deadline:time.sleep(.02)
        time.sleep(.1)
        message=String(data=json.dumps(e,allow_nan=False))
        for _ in range(3):event_pub.publish(message);time.sleep(.05)
    def writer_faults():
        faults=[]
        for key in ('dropped_messages','write_errors'):
            if int(native_status.get(key,0))>0:faults.append('native '+key+'='+str(native_status[key]))
        for key in ('camera_error','recording_error'):
            if native_status.get(key):faults.append('native '+key+': '+str(native_status[key]))
        return faults
    def certify(bag,profile,identity):
        result=audit_bag(bag,profile,identity)
        result['native_writer_status']=dict(native_status)
        result['issues'].extend(writer_faults())
        return result
    capture=Capture(cfg,monitor,Native(),marker,certify,clock=lambda:node.get_clock().now().nanoseconds)
    subscriptions=[]
    def native_callback(msg):
        try:native_status.clear();native_status.update(json.loads(msg.data));native_at[0]=time.monotonic()
        except Exception as e:node.get_logger().error(str(e))
    subscriptions.append(node.create_subscription(String,'info/gento_replay/native_status',native_callback,10,callback_group=group))
    for topic,typ in cfg['required_topics'].items():
        cls=get_message(typ)
        def observe(msg,t=topic,tp=typ):
            monitor.observe(t,tp)
            if t=='/recording/sharpa/context':
                try:monitor.hand_context(json.loads(msg.data))
                except Exception as e:node.get_logger().error('Invalid hand context: '+str(e))
        subscriptions.append(node.create_subscription(cls,topic,observe,qos_profile_sensor_data,callback_group=group))
    if cfg.get('camera_required'):
        topic=cfg['camera_topic'];subscriptions.append(node.create_subscription(get_message('sensor_msgs/msg/CompressedImage'),topic,
            lambda m:monitor.observe(topic,'sensor_msgs/msg/CompressedImage'),qos_profile_sensor_data,callback_group=group))
    def health_callback(msg):
        try:monitor.hand_health(json.loads(msg.data))
        except Exception as e:node.get_logger().error('Invalid hand health: '+str(e))
    subscriptions.append(node.create_subscription(String,cfg['health_topic'],health_callback,10,callback_group=group))
    service_group=ReentrantCallbackGroup()
    def record(req,resp):
        try:
            if req.data==1:value=capture.start(getattr(req,'name',''))
            elif req.data==0:value=capture.stop()
            else:raise ValueError('Record accepts only 1/start or 0/stop; playback is not routed here')
            resp.success=value.get('state',value.get('status')) not in ('FAILED','PARTIAL');resp.error_code=0 if resp.success else 1
            resp.message=json.dumps(value,ensure_ascii=False,allow_nan=False)
        except Exception as e:resp.success=False;resp.error_code=1;resp.message=str(e)
        return resp
    services=[node.create_service(Int,'control/gento_replay/record',record,callback_group=service_group),
              node.create_service(GentoReplay,'control/gento_replay/record_named',record,callback_group=service_group)]
    # Optional generic Apex API transport, without enabling any Pro playback routes.
    def json_control(req,resp):
        try:
            cmd=json.loads(req.command_json);action=cmd['action']
            if action=='start':v=capture.start(cmd.get('name',''))
            elif action=='stop':v=capture.stop()
            elif action=='status':v=capture.status()
            elif action=='get_topics':v={'required_topics':cfg['required_topics']}
            else:raise ValueError('Unsupported recording-only command')
            resp.success=v.get('state') not in ('FAILED','PARTIAL','STOP_FAILED');resp.result_json=json.dumps(v,ensure_ascii=False);resp.message=v.get('state','ok')
        except Exception as e:resp.success=False;resp.result_json=json.dumps({'status':'error','error_message':str(e)});resp.message=str(e)
        return resp
    services.append(node.create_service(JsonCommand,'recorder/control',json_control,callback_group=service_group))
    def tick():
        try:
            if capture.active and time.monotonic()-native_at[0]>2:capture.stop('Native recorder status lost')
            elif capture.active and writer_faults():capture.stop('; '.join(writer_faults()))
            elif capture.active and native_status.get('playback_active'):capture.stop('Native playback became active')
            else:capture.watchdog()
        except Exception as e:node.get_logger().error('Recording watchdog: '+str(e))
        status=dict(native_status);status['record_sync']=capture.status()
        status['recording']=bool(capture.active) or bool(native_status.get('recording'))
        if capture.last_result.get('failure_reason'):status['last_message']=capture.last_result['failure_reason']
        status_pub.publish(String(data=json.dumps(status,ensure_ascii=False,default=str)))
    node.create_timer(.2,tick,callback_group=MutuallyExclusiveCallbackGroup())
    executor=MultiThreadedExecutor(num_threads=6);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        if capture.active:
            try:capture.stop('Record coordinator shutting down')
            except Exception as e:node.get_logger().error(str(e))
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
