"""Hand-only record bridge. Publishes recording copies, never robot control."""
import json
import os
import sys
import time
import math
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
    import fcntl
    root = Path.home() / 'manus_sharpa/recordings'
    root.mkdir(parents=True, exist_ok=True)
    lock = (root / '.receiver.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    session = None; previous = 0; stream = None
    print(json.dumps({'ready':True,'clock_domain':'robot_receive','control_commands_sent':0}), flush=True)
    try:
        for line in sys.stdin:
            e = json.loads(line)
            if e.get('kind') == 'heartbeat':
                print(json.dumps({'ack':True}), flush=True); continue
            if e.get('schema') != 'manus_sharpa_event_v1': raise ValueError('Event schema')
            sid=e['session_id']
            if len(sid)!=32 or any(c not in '0123456789abcdef' for c in sid): raise ValueError('Session')
            if session is None:
                session=sid
                stream=(root / (session+'.jsonl')).open('x', buffering=1)
            if sid != session or type(e['seq']) is not int or e['seq']!=previous+1: raise ValueError('Sequence gap/reordering')
            previous=e['seq']
            stamp=node.get_clock().now().nanoseconds
            e['received_robot_ns']=stamp
            e['clock_domain']='robot_receive'
            # Preserve original source header; canonical header identifies receipt event.
            stream.write(json.dumps(e,allow_nan=False)+'\n')
            if e['kind'] in ('state','accepted'):
                if e['side'] not in ('left','right') or len(e['position'])!=22 or not all(math.isfinite(q) for q in e['position']): raise ValueError('Hand shape')
                suffix='states' if e['kind']=='state' else 'commands'
                if node.count_publishers(f'/hand_{e["side"]}/joint_{suffix}')>1:raise RuntimeError('Competing hand recording publisher')
                msg=JointState();msg.header.stamp.sec=stamp//10**9;msg.header.stamp.nanosec=stamp%10**9
                msg.header.frame_id=f'manus_sharpa/{session}/{e["seq"]}'
                msg.name=e['names'];msg.position=e['position']
                pubs[e['side'],'states' if e['kind']=='state' else 'commands'].publish(msg)
            print(json.dumps({'ack':previous}),flush=True)
    finally:
        if stream: stream.flush();os.fsync(stream.fileno());stream.close()
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__': main()
