"""20 Hz LeRobot chassis-only replay. Default is OFFLINE: no SSH or ROS."""
import argparse
import base64
import hashlib
import json
import math
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parent))
from chassis_endpoint import validate_plan, require, STEP_NS, BACKEND_SHA256

DATA = Path('/mnt/apex-data/apex_n_sharpa/data/lerobot')
DEFAULT_TASK = 'robot_import_20261024_egoav_v4'
BACKEND = Path('/home/descfly/apex_n_sharpa/src/chassis_replay/main.py')


def make_plan(info, timeline, columns, seconds=None):
    import numpy as np
    n = info['total_frames']
    require(type(n) is int and n>=2 and info['fps']==20, 'Require 20 Hz dataset')
    for key in ('action.chassis','observation.chassis'):
        f=info['features'][key]
        require(f['shape']==[3] and f['names']==['vx_m_s','vy_m_s','wz_rad_s'], 'Wrong velocity layout/units')
    raw = timeline['timestamp_ns']
    require(len(raw)==n and all(type(t) is int and 0<t<2**63 for t in raw), 'Invalid integer timestamps')
    require(all(b-a==STEP_NS for a,b in zip(raw,raw[1:])), 'Require continuous 20 Hz timeline')
    offsets = [t-raw[0] for t in raw]
    require(columns['frame_index']==list(range(n)), 'Noncontiguous frame indices')
    require(np.allclose(columns['timestamp'],np.array(offsets)/1e9,atol=2e-6,rtol=0), 'Parquet timestamp mismatch')
    a = np.asarray(columns['action.chassis'],dtype=float)
    s = np.asarray(columns['observation.chassis'],dtype=float)
    require(a.shape==(n,3) and s.shape==(n,3) and np.isfinite(a).all() and np.isfinite(s).all(), 'Invalid chassis data')
    native = timeline.get('native_replay')
    require(isinstance(native,dict) and 'chassis_frame' in native and 'chassis_units' in native,
            'Missing verified chassis frame/units metadata; use episode 2 or re-export with metadata')
    require(native['chassis_frame']=='base_link' and native['chassis_units']==['m/s','m/s','rad/s'], 'Wrong chassis frame/units')
    chosen = timeline['aligned_sources']['actions/chassis']['timestamp_ns']
    require(len(chosen)==n and all(type(t) is int and 0<=g-t<=STEP_NS for g,t in zip(raw,chosen)), 'Noncausal or stale action alignment')
    # Reject unsupported input in the entire episode, not only a selected prefix.
    require((np.abs(a[:,0])<=.400001).all() and (np.abs(a[:,2])<=.500001).all(), 'Velocity exceeds existing envelope')
    require((a[:,1]==0).all(), 'Nonzero vy is not verified')
    count = n
    if seconds is not None:
        require(math.isfinite(seconds) and .1<=seconds<=120, 'seconds must be between 0.1 and 120')
        count = round(seconds*20)
        require(abs(count/20-seconds)<1e-8 and count<=n, 'seconds must fit the episode and 50ms grid')
    plan = {'schema':'chassis_parquet_20hz_v1','frame':'base_link','units':['m/s','m/s','rad/s'],
            'fps':20,'source_frames':n,'source_timestamp_ns':raw[:count],
            'offset_ns':offsets[:count],'end_ns':count*STEP_NS,'velocity':a[:count].tolist(),
            'recorded_state':s[:count].tolist(),'action_source':'Parquet action.chassis (not native_replay)',
            'last_sample_policy':'hold 50ms then separate stop; source unchanged'}
    validate_plan(plan)
    return plan


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
    return h.hexdigest()


def verify_episode(root):
    require(root.is_dir() and not root.is_symlink(), 'Invalid episode directory')
    manifest = root/'meta/completion.json'
    entries = json.loads(manifest.read_text())
    require(isinstance(entries,dict) and bool(entries), 'Missing completion file hashes')
    for rel, expected in entries.items():
        path = root/rel
        require(not Path(rel).is_absolute() and '..' not in Path(rel).parts, 'Unsafe completion path')
        require(path.resolve().is_relative_to(root.resolve()), 'Completion path escape')
        require(not any(p.is_symlink() for p in [path,*path.parents] if p!=root.parent), 'Symlink not allowed')
        require(path.is_file() and sha(path)==expected, f'Integrity mismatch: {rel}')
    files={str(p.relative_to(root)) for p in root.rglob('*') if p.is_file()}
    require(files==set(entries)|{'meta/completion.json'}, 'Episode file set changed')
    return sha(manifest)


def load_plan(root, seconds=None):
    import pyarrow.parquet as pq
    before = verify_episode(root)
    info=json.loads((root/'meta/info.json').read_text())
    timeline=json.loads((root/'meta/source_timestamps.json').read_text())
    files=list((root/'data').rglob('*.parquet'))
    require(len(files)==1, 'Require a single-episode dataset')
    columns=pq.read_table(files[0],columns=['timestamp','frame_index','action.chassis','observation.chassis']).to_pydict()
    plan=make_plan(info,timeline,columns,seconds)
    require(verify_episode(root)==before, 'Episode changed while loading')
    plan.update(source=str(root),completion_sha256=before,parquet_sha256=sha(files[0]))
    return plan


def finish_remote_result(result, returncode, execute):
    result=dict(result)
    if not result:
        result.update(status='failed',error='No final robot acknowledgement; inspect robot.log',stop_confirmed=None)
    if returncode!=0:
        result.update(status='failed',ssh_returncode=returncode)
    # Only an explicit robot acknowledgement can prove that motion never began.
    if execute and result.get('motion_started') is not False and result.get('stop_confirmed') is not True:
        result.update(status='failed',stop_warning='STOP_UNCONFIRMED: verify locally; use hardware emergency stop if needed')
    return result


def run_remote(plan, execute, run_dir):
    backend=BACKEND.read_text()
    require(hashlib.sha256(backend.encode()).hexdigest()==BACKEND_SHA256, 'Existing backend changed; review required')
    endpoint=Path(__file__).with_name('chassis_endpoint.py').read_bytes()
    boot='import base64; exec(compile(base64.b64decode('+repr(base64.b64encode(endpoint).decode())+'),"<chassis-test-endpoint>","exec"))'
    command='source /etc/apex/apex_ros_env.sh >/dev/null 2>&1; export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; exec python3 -u -B -c '+shlex.quote(boot)
    cancelled=threading.Event()
    previous={sig:signal.getsignal(sig) for sig in (signal.SIGINT,signal.SIGTERM)}
    for sig in previous: signal.signal(sig,lambda *_:cancelled.set())
    result={}; proc=None
    with (run_dir/'robot.log').open('w') as log:
        try:
            proc=subprocess.Popen(['ssh','-T','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes',
                '-o','ConnectTimeout=5','-o','ServerAliveInterval=1','-o','ServerAliveCountMax=3',
                'user@6.6.7.100',command],stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,text=True,bufsize=1,start_new_session=True)
            def output():
                for line in proc.stdout:
                    log.write(line); log.flush()
                    try: msg=json.loads(line)
                    except ValueError:
                        print(line.rstrip(),flush=True); continue
                    if 'result' in msg: result.update(msg['result'])
                    else: print(line.rstrip(),flush=True)
            reader=threading.Thread(target=output,daemon=True);reader.start()
            def feed():
                try:
                    proc.stdin.write(json.dumps({'plan':plan,'execute':execute,'backend':backend},allow_nan=False)+'\n')
                    proc.stdin.flush()
                    while proc.poll() is None:
                        proc.stdin.write('STOP\n' if cancelled.is_set() else 'PING\n');proc.stdin.flush()
                        if cancelled.wait(.1): break
                except (BrokenPipeError,OSError,ValueError): pass
                finally:
                    try: proc.stdin.close()
                    except (OSError,ValueError): pass
            feeder=threading.Thread(target=feed,daemon=True);feeder.start()
            deadline=time.monotonic()+plan['end_ns']/1e9+30
            cancel_at=None
            while proc.poll() is None:
                now=time.monotonic()
                if now>deadline: cancelled.set()
                if cancelled.is_set():
                    if cancel_at is None:
                        cancel_at=now
                        print('正在请求底盘停止并等待回执；重复 Ctrl+C 不会跳过清理。',flush=True)
                    if now-cancel_at>12:
                        proc.terminate();break
                time.sleep(.05)
            try: proc.wait(timeout=3)
            except subprocess.TimeoutExpired: proc.kill();proc.wait()
            reader.join(timeout=3)
            result=finish_remote_result(result,proc.returncode,execute)
        finally:
            cancelled.set()
            # Unexpected host-side failures also close the lease and allow the
            # robot's stop loop to finish; do not abandon an active SSH child.
            if proc is not None and proc.poll() is None:
                try: proc.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    proc.terminate()
                    try: proc.wait(timeout=3)
                    except subprocess.TimeoutExpired: proc.kill(); proc.wait()
            for sig,handler in previous.items(): signal.signal(sig,handler)
    (run_dir/'report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('samples','diagnostics','published')},ensure_ascii=False,indent=2))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',default=DEFAULT_TASK)
    p.add_argument('--episode',type=int,default=2)
    p.add_argument('--seconds',type=float,help='Optional prefix, multiple of 0.05s; default whole episode')
    mode=p.add_mutually_exclusive_group()
    mode.add_argument('--check',action='store_true',help='Robot read-only preflight; no velocity commands')
    mode.add_argument('--execute',action='store_true',help='MOVE CHASSIS using the 20Hz Parquet actions')
    a=p.parse_args()
    require(re.fullmatch(r'[\w-]+',a.task) is not None and 0<=a.episode<100_000_000,'Invalid task/episode')
    root=DATA/a.task/f'episode_{a.episode:08d}'
    plan=load_plan(root,a.seconds)
    run_dir=Path(__file__).resolve().parent/'runs'/(time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8])
    run_dir.mkdir(parents=True)
    (run_dir/'plan.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'mode':'execute' if a.execute else 'check' if a.check else 'offline',
        'source':str(root),'frames':len(plan['velocity']),'fps':20,
        'sample_span_s':plan['offset_ns'][-1]/1e9,'command_duration_s':plan['end_ns']/1e9,
        'frame':'base_link','order':['vx','vy','wz'],'vy_nonzero_supported':False,
        'run_dir':str(run_dir)},ensure_ascii=False,indent=2),flush=True)
    if not a.check and not a.execute:
        print('OFFLINE_PASS：没有连接机器人，没有发送运动命令。')
        return 0
    result=run_remote(plan,a.execute,run_dir)
    return 0 if result['status'] in ('completed','preflight_passed') else 1


if __name__=='__main__':
    try: raise SystemExit(main())
    except (ValueError,KeyError,OSError) as exc:
        print(f'拒绝启动: {exc}',file=sys.stderr)
        raise SystemExit(1)
