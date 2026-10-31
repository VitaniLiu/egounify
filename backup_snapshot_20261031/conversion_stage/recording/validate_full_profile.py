#!/usr/bin/env python3
"""Full numerical profile using synthetic publishers; domain199, no hardware."""
import json,os,subprocess,sys,threading,time,uuid
from pathlib import Path
import rclpy,yaml
from rclpy.executors import MultiThreadedExecutor
from marvin_msgs.srv import GentoReplay
from contract import HAND_NAMES,HAND_TOPICS,verify_completion

def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='199'
    root=Path(__file__).parent/'validation'/str(time.time_ns());root.mkdir(parents=True)
    cfg=json.loads(Path(__file__).with_name('config.json').read_text())
    cfg.update(diagnostic=True,record_root=str(root/'raw'),state_root=str(root/'state'),camera_required=False)
    Path(cfg['record_root']).mkdir()
    (root/'config.json').write_text(json.dumps(cfg))
    native={'/tj/gento_replay_node':{'ros__parameters':{'ros_namespace':'tj','record_dir':cfg['record_root'],
        'core_record_topics':['record_sync_unused'],'global_record_topics':list(cfg['required_topics'])+[cfg['health_topic'],cfg['event_topic']],
        'record_camera':False,'camera_required':False,'storage_id':'mcap','topic_scan_interval_sec':.1}}}
    (root/'native.yaml').write_text(yaml.safe_dump(native))
    processes=[];logs=[];running=threading.Event();running.set();feed_error=[]
    def launch(args,name):
        f=(root/(name+'.log')).open('w');logs.append(f)
        p=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=f,stderr=subprocess.STDOUT,text=True,bufsize=1)
        processes.append(p);return p
    nativeproc=launch(['/opt/kernelmind/apex/install/gento_replay/lib/gento_replay/gento_replay_node','--ros-args',
        '-r','__ns:=/tj','--params-file',str(root/'native.yaml'),
        '-r','control/gento_replay/record:=control/gento_replay/record_native',
        '-r','control/gento_replay/record_named:=control/gento_replay/record_named_native',
        '-r','info/gento_replay/status:=info/gento_replay/native_status'],'native')
    coordinator=launch([sys.executable,str(Path(__file__).with_name('record_node.py')),'--ros-args','-r','__ns:=/tj',
        '-p','config_path:='+str(root/'config.json')],'coordinator')
    receiver=launch([sys.executable,str(Path(__file__).with_name('robot_receiver_validation.py')),'--diagnostic'],'receiver')
    sid=uuid.uuid4().hex
    def feed():
        seq=0
        try:
            while running.is_set():
                for side in ('left','right'):
                    for kind in ('state','accepted'):
                        seq+=1;stamp=time.time_ns()
                        e=dict(schema='manus_sharpa_event_v1',session_id=sid,seq=seq,side=side,kind=kind,
                            live=False,serials=cfg['serials'],names=HAND_NAMES,position=[.01]*22,source_stamp_ns=stamp,
                            received_host_ns=stamp,source_monotonic_ns=time.monotonic_ns())
                        if kind=='accepted':e['sdk_result']={'status':'sdk_call_succeeded'}
                        receiver.stdin.write(json.dumps(e)+'\n')
                receiver.stdin.flush();time.sleep(.02)
        except Exception as e:feed_error.append(str(e))
    thread=threading.Thread(target=feed,daemon=True);thread.start()
    rclpy.init();node=rclpy.create_node('record_sync_validation');executor=MultiThreadedExecutor(2);executor.add_node(node)
    spinner=threading.Thread(target=executor.spin,daemon=True);spinner.start()
    from rosidl_runtime_py.utilities import get_message
    publishers=[]
    for topic,typename in cfg['required_topics'].items():
        if topic in HAND_TOPICS or topic=='/recording/sharpa/context':continue
        cls=get_message(typename)
        publishers.append((node.create_publisher(cls,topic,10),cls))
    def publish_numerical():
        if running.is_set():
            for pub,cls in publishers:pub.publish(cls())
    numerical_timer=node.create_timer(.02,publish_numerical)
    client=node.create_client(GentoReplay,'/tj/control/gento_replay/record_named')
    def rpc(data,name=''):
        assert client.wait_for_service(timeout_sec=10),'Coordinator unavailable'
        req=GentoReplay.Request();req.data=data;req.name=name
        f=client.call_async(req);end=time.monotonic()+25
        while not f.done() and time.monotonic()<end:time.sleep(.02)
        if not f.done():raise TimeoutError('Record RPC timeout')
        r=f.result();print(json.dumps({'data':data,'success':r.success,'message':r.message}),flush=True)
        return r
    try:
        time.sleep(4)
        result=rpc(1,'isolated_sync');assert result.success,result.message
        time.sleep(1.2)
        result=rpc(0);assert result.success,result.message
        m=json.loads(result.message);verified=verify_completion(m['bag'],allow_diagnostic=True)
        assert verified['audit']['verified_hand_samples']>100
        assert set(cfg['required_topics']).issubset(verified['audit']['streams'])
        try:verify_completion(m['bag'])
        except ValueError:pass
        else:raise AssertionError('Diagnostic capture admitted to production')
        result=rpc(1,'watchdog_sync');assert result.success,result.message
        time.sleep(.7);running.clear();thread.join(2);time.sleep(2)
        result=rpc(0);m=json.loads(result.message)
        assert not result.success and m['state']=='FAILED',m
        result=rpc(1,'stale_must_reject');assert not result.success
        report={'state':'PASS','domain':199,'root':str(root),'complete_bag':verified['bag'],
                'verified_hand_samples':verified['audit']['verified_hand_samples'],
                'fault_capture_state':m['state'],'checks':['native MCAP start/stop','same-bag evidence',
                'capture integrity hashes','diagnostic rejection in production','source dropout auto-stop','stale start rejection'],
                'hardware_initialized':False,'motion_services_started':False,'feed_errors':feed_error,
                'profile':'full_numerical_synthetic_camera_disabled',
                'required_topics':list(cfg['required_topics']),
                'stream_counts':{t:verified['audit']['streams'][t]['count'] for t in cfg['required_topics']}}
        (root/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
    finally:
        running.clear();thread.join(2)
        for p in reversed(processes):p.terminate()
        for p in reversed(processes):
            try:p.wait(5)
            except subprocess.TimeoutExpired:p.kill();p.wait()
        executor.shutdown();node.destroy_node();rclpy.shutdown()
        for f in logs:f.close()

if __name__=='__main__':main()
