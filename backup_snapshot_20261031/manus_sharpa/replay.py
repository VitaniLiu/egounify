"""Hand-only replay planner/publisher. Robot IK/chassis are never commanded."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
from hands_common import NAMES, config, digest

def load_plan(path):
    path=Path(path)
    if path.is_dir():path=path/'hands/replay.json'
    commit=json.loads(path.with_name('replay.commit.json').read_text())
    if digest(path)!=commit['sha256']:raise ValueError('Replay hash mismatch')
    p=json.loads(path.read_text())
    if p['schema']!='manus_sharpa_replay_v1' or p['clock_domain']!='robot_receive' or p['joint_names']!=list(NAMES):
        raise ValueError('Replay schema')
    if p['serials']!=config()['serials']:raise ValueError('Replay device binding')
    for side in ('left','right'):
        t=np.asarray(p['hands'][side]['offset_ns'],dtype=np.int64);q=np.asarray(p['hands'][side]['position'])
        if not len(t) or t[0]!=0 or np.any(np.diff(t)<=0) or q.shape!=(len(t),22) or not np.isfinite(q).all():raise ValueError('Replay values')
        if len(t)>1 and np.max(np.diff(t))>200_000_000:raise ValueError('Recorded action gap exceeds native watchdog')
    return p

def main():
    a=argparse.ArgumentParser(description=__doc__);a.add_argument('episode',type=Path);a.add_argument('--publish',action='store_true');args=a.parse_args()
    p=load_plan(args.episode)
    if not args.publish:
        print(json.dumps({'schema':p['schema'],'serials':p['serials'],'frames':{s:len(v['offset_ns']) for s,v in p['hands'].items()},
                          'hardware_initialized':False,'robot_ik_commands':0,'joint_chassis_sync':'not available without changing existing robot replay'}));return
    import rclpy
    from sensor_msgs.msg import JointState
    from rclpy.qos import qos_profile_sensor_data
    from litchi_hardware.hardware.sharpa.schema import SHARPA_WAVE_JOINT_SCHEMA
    rclpy.init();node=rclpy.create_node('hands_replay')
    feedback={};publishers={s:node.create_publisher(JointState,f'/sharpa/{s}/request',10) for s in ('left','right')}
    from hands_common import normalize
    subs=[node.create_subscription(JointState,f'/sharpa/{s}/joint_states',
          lambda m,s=s:feedback.update({s:(time.monotonic(),normalize(m.name,m.position,s))}),qos_profile_sensor_data) for s in publishers]
    try:
        deadline=time.monotonic()+5
        while len(feedback)<2 or not all(pub.get_subscription_count()==1 for pub in publishers.values()):
            rclpy.spin_once(node,timeout_sec=.02)
            if time.monotonic()>deadline:raise TimeoutError('Replay feedback/subscriber unavailable')
        limits=np.asarray(SHARPA_WAVE_JOINT_SCHEMA.limits)
        for side in publishers:
            q=np.asarray(p['hands'][side]['position'])
            if ((q<limits[:,0])|(q>limits[:,1])).any():raise ValueError('Replay outside SDK limits')
            if abs(q[0]-feedback[side][1]).max()>.1:raise ValueError('Start pose differs by more than 0.1rad')
        events=sorted((t,s,q) for s,v in p['hands'].items() for t,q in zip(v['offset_ns'],v['position']))
        start=time.monotonic_ns()+200_000_000
        for offset,side,q in events:
            while time.monotonic_ns()<start+offset:rclpy.spin_once(node,timeout_sec=.001)
            if time.monotonic_ns()-(start+offset)>50_000_000:raise TimeoutError('Replay scheduler late')
            if any(time.monotonic()-v[0]>.2 for v in feedback.values()):raise RuntimeError('Replay feedback stale')
            if node.count_publishers(f'/sharpa/{side}/request')!=1:raise RuntimeError('Competing hand writer')
            m=JointState();m.header.stamp=node.get_clock().now().to_msg();m.name=list(NAMES);m.position=q;publishers[side].publish(m)
            rclpy.spin_once(node,timeout_sec=0)
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
