"""Hand-only record bridge. Publishes recording copies, never robot control."""
import json
import os
import sys
import time
import math
import threading
from pathlib import Path

def main():
    import rclpy
    from sensor_msgs.msg import JointState
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    rclpy.init()
    node = rclpy.create_node('manus_sharpa_record_receiver')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=False)])
    qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
    pubs = {(s,k): node.create_publisher(JointState, f'/hand_{s}/joint_{k}', qos)
            for s in ('left','right') for k in ('commands','states')}
    from std_msgs.msg import String
    context_pub=node.create_publisher(String,'/recording/sharpa/context',qos)
    health_pub=node.create_publisher(String,'/recording/sharpa/health',10)
    import fcntl
    root = Path.home() / 'manus_sharpa/recordings'
    if '--diagnostic' in sys.argv:root=root/'diagnostic199'
    root.mkdir(parents=True, exist_ok=True)
    lock = (root / '.receiver.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    session = None; previous = 0; stream = None
    last={};counts={};serials={};source_live=False;fault=None;last_health=0.
    diagnostic='--diagnostic' in sys.argv
    if diagnostic and os.environ.get('ROS_DOMAIN_ID')!='199':raise RuntimeError('Diagnostic ingress requires domain199')
    def health():
        nonlocal last_health
        now=time.monotonic()
        if now-last_health<.1:return
        last_health=now
        roles=[(s,k) for s in ('left','right') for k in ('state','accepted')]
        ages={s+'/'+k:now-last.get((s,k),-1e9) for s,k in roles}
        ready=bool(session) and fault is None and all(age<.5 for age in ages.values())
        health_pub.publish(String(data=json.dumps({'schema':'apex_sharpa_health_v1','session_id':session,
            'serials':serials,'live':source_live,'ready':ready,'ages_s':ages,'counts':counts,'fault':fault},allow_nan=False)))
    print(json.dumps({'ready':True,'clock_domain':'robot_receive','control_commands_sent':0}), flush=True)
    try:
        for line in sys.stdin:
            e = json.loads(line)
            if e.get('kind') == 'heartbeat':
                health()
                print(json.dumps({'ack':True}), flush=True); continue
            if e.get('schema') != 'manus_sharpa_event_v1': raise ValueError('Event schema')
            sid=e['session_id']
            if len(sid)!=32 or any(c not in '0123456789abcdef' for c in sid): raise ValueError('Session')
            if session is None:
                session=sid
                stream=(root / (session+'.jsonl')).open('x', buffering=1)
                serials=e.get('serials',{});source_live=e.get('live') is True
            if e.get('serials')!=serials or (e.get('live') is not True and not diagnostic):raise ValueError('Device/live source changed')
            if sid != session or type(e['seq']) is not int or e['seq']!=previous+1: raise ValueError('Sequence gap/reordering')
            previous=e['seq']
            stamp=node.get_clock().now().nanoseconds
            e['received_robot_ns']=stamp
            e['clock_domain']='robot_receive'
            # Preserve original source header; canonical header identifies receipt event.
            stream.write(json.dumps(e,allow_nan=False)+'\n')
            context_pub.publish(String(data=json.dumps(e,allow_nan=False)))
            key=e.get('side','unknown')+'/'+e.get('kind','unknown');counts[key]=counts.get(key,0)+1
            if e['kind'] in ('state','accepted'):last[e['side'],e['kind']]=time.monotonic()
            if e['kind']=='rejected':fault='SDK rejected target'
            if e['kind'] in ('state','accepted'):
                if e['side'] not in ('left','right') or len(e['position'])!=22 or not all(math.isfinite(q) for q in e['position']): raise ValueError('Hand shape')
                suffix='states' if e['kind']=='state' else 'commands'
                # Gento declares idle replay publishers on these same topics. They are
                # allowed endpoints; every recorded sample still needs matching ingress evidence.
                endpoints=node.get_publishers_info_by_topic(f'/hand_{e["side"]}/joint_{suffix}')
                foreign=[p for p in endpoints if p.node_name not in ('manus_sharpa_record_receiver','gento_replay_node','_NODE_NAME_UNKNOWN_')]
                own=[p for p in endpoints if p.node_name=='manus_sharpa_record_receiver']
                if foreign or len(own)>1:raise RuntimeError('Competing hand recording publisher: '+str([(p.node_namespace,p.node_name) for p in endpoints]))
                msg=JointState();msg.header.stamp.sec=stamp//10**9;msg.header.stamp.nanosec=stamp%10**9
                msg.header.frame_id=f'manus_sharpa/{session}/{e["seq"]}'
                msg.name=e['names'];msg.position=e['position']
                pubs[e['side'],'states' if e['kind']=='state' else 'commands'].publish(msg)
            health()
            print(json.dumps({'ack':previous}),flush=True)
    finally:
        if stream: stream.flush();os.fsync(stream.fileno());stream.close()
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__': main()
