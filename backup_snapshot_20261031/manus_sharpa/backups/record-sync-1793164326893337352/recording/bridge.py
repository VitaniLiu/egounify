"""Host subscriber sends evidence to the robot recording clock via SSH."""
import collections
import json
import queue
import shlex
import subprocess
import threading
import time
import uuid
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from hands_common import NAMES, config, normalize

class RecordBridge:
    def __init__(self, node, *, fake=False):
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from rclpy.qos import qos_profile_sensor_data
        self.node=node;self.fake=fake;self.session=uuid.uuid4().hex;self.seq=0
        self.error=None;self.stopping=False;self.q=queue.Queue(maxsize=4096)
        self.last_ack=time.monotonic();self.proc=None;self.ready=fake
        c=config()
        if not fake:
            command='source /etc/apex/apex_ros_env.sh; export ROS_DOMAIN_ID=29; exec /usr/bin/python3 -u /home/user/manus_sharpa/robot_receiver.py'
            self.proc=subprocess.Popen(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8',c['robot_ssh'],
                                        'bash -c '+shlex.quote(command)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
            self.reader=threading.Thread(target=self.read_ack,daemon=True);self.reader.start()
        self.worker=threading.Thread(target=self.send,daemon=True);self.worker.start()
        self.subs=[]
        for side in ('left','right'):
            self.subs.append(node.create_subscription(JointState,f'/sharpa/{side}/joint_states',
                 lambda m,s=side:self.joint(s,m,'state'),qos_profile_sensor_data))
            self.subs.append(node.create_subscription(JointState,f'/sharpa/{side}/request',
                 lambda m,s=side:self.joint(s,m,'request'),10))
            self.subs.append(node.create_subscription(String,f'/sharpa/{side}/command_result',
                 lambda m,s=side:self.result(s,m),10))
        self.last_data={};self.request_counts={};self.last_request={}

    def enqueue(self,e):
        age=time.time_ns()-e['source_stamp_ns']
        if not 0<=age<=500_000_000:
            self.error=ValueError('Source hand message stale or host clock changed');return
        self.seq+=1
        e.update(schema='manus_sharpa_event_v1',session_id=self.session,seq=self.seq,
                 received_host_ns=time.time_ns(),source_monotonic_ns=time.monotonic_ns(),serials=config()['serials'])
        if e.get('side') in ('left','right') and e['kind'] in ('state','accepted'):
            self.last_data[e['side'],e['kind']]=time.monotonic()
        if e.get('side') in ('left','right') and e['kind']=='request':
            self.request_counts[e['side']]=self.request_counts.get(e['side'],0)+1
            self.last_request[e['side']]=time.monotonic()
        try:self.q.put_nowait(e)
        except queue.Full:self.error=RuntimeError('Recording queue overflow; no silent drops')

    def joint(self,side,m,kind):
        try:self.enqueue(dict(side=side,kind=kind,names=list(NAMES),position=normalize(m.name,m.position,side).tolist(),
                             source_stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec))
        except Exception as e:self.error=e

    def result(self,side,m):
        try:
            r=json.loads(m.data)
            if r.get('schema')!='sharpa_command_result_v2' or r.get('side')!=side:raise ValueError('SDK receipt schema')
            accepted=r['status']=='sdk_call_succeeded'
            e=dict(side=side,kind='accepted' if accepted else 'rejected',source_stamp_ns=r['stamp_ns'],sdk_result=r)
            if accepted:e.update(names=list(NAMES),position=normalize(r['names'],r['target'],side).tolist())
            self.enqueue(e)
            if not accepted:self.error=RuntimeError('SDK rejected command: '+r.get('reason',''))
        except Exception as e:self.error=e

    def read_ack(self):
        try:
            for line in self.proc.stdout:
                r=json.loads(line)
                if r.get('ready'):self.ready=True
                if r.get('ready') or 'ack' in r:self.last_ack=time.monotonic()
            if not self.stopping:self.error=RuntimeError('Robot recording receiver disconnected')
        except Exception as e:self.error=e

    def send(self):
        try:
            while not self.stopping:
                try:e=self.q.get(timeout=.1)
                except queue.Empty:e={'kind':'heartbeat'}
                if e.get('source_monotonic_ns') and time.monotonic_ns()-e['source_monotonic_ns']>500_000_000:
                    raise RuntimeError('Recording transport backlog exceeds 0.5s')
                if not self.fake:
                    self.proc.stdin.write(json.dumps(e,allow_nan=False)+'\n');self.proc.stdin.flush()
                if not self.fake and time.monotonic()-self.last_ack>2:raise TimeoutError('Robot recording ACK timeout')
        except Exception as e:self.error=e

    def close(self):
        self.stopping=True
        if self.proc:
            try:self.proc.stdin.close();self.proc.wait(timeout=3)
            except Exception:self.proc.terminate()
