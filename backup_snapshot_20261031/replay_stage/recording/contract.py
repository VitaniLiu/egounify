"""Pure recording contract. No SDK, ROS, motion, enable or home commands."""
import hashlib
import json
import os
import re
import threading
import time
from pathlib import Path

HAND_NAMES = 'thumb_CMC_FE thumb_CMC_AA thumb_MCP_FE thumb_MCP_AA thumb_IP index_MCP_FE index_MCP_AA index_PIP index_DIP middle_MCP_FE middle_MCP_AA middle_PIP middle_DIP ring_MCP_FE ring_MCP_AA ring_PIP ring_DIP pinky_CMC pinky_MCP_FE pinky_MCP_AA pinky_PIP pinky_DIP'.split()
HAND_TOPICS = {f'/hand_{s}/joint_{suffix}': (s,k) for s in ('left','right') for suffix,k in [('states','state'),('commands','accepted')]}
CONTEXT='/recording/sharpa/context'
EVENT='/recording/capture_event'

def sha256(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

def atomic_json(path,obj):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_name(path.name+'.tmp')
    with tmp.open('w') as f:
        json.dump(obj,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
    os.replace(tmp,path)
    fd=os.open(path.parent,os.O_RDONLY)
    try:os.fsync(fd)
    finally:os.close(fd)

def verify_completion(root,allow_diagnostic=False):
    root=Path(root).resolve()
    c=json.loads((root/'capture_completion.json').read_text())
    if c.get('schema')!='apex_capture_completion_v1' or c.get('state')!='COMPLETE':raise ValueError('Capture is not COMPLETE')
    if c.get('diagnostic') and not allow_diagnostic:raise ValueError('Diagnostic capture cannot enter production conversion')
    if sha256(root/'capture_manifest.json')!=c['manifest_sha256']:raise ValueError('Manifest hash mismatch')
    m=json.loads((root/'capture_manifest.json').read_text())
    if m['capture_id']!=c['capture_id']:raise ValueError('Capture identity mismatch')
    if 'episode_index' in m and (c.get('episode_index')!=m['episode_index'] or m.get('episode_name')!=f"episode_{m['episode_index']:08d}"):raise ValueError('Episode identity mismatch')
    if m.get('schema')!='apex_capture_manifest_v1' or m.get('state')!='COMPLETE':raise ValueError('Manifest is not COMPLETE')
    if m.get('diagnostic')!=c.get('diagnostic'):raise ValueError('Diagnostic marker mismatch')
    if m.get('clock_domain')!='robot_receive' or not m.get('raw_files'):raise ValueError('Missing capture clock/files')
    for rel,e in m['raw_files'].items():
        p=(root/rel).resolve()
        if not p.is_relative_to(root) or p.is_symlink() or sha256(p)!=e['sha256'] or p.stat().st_size!=e['size']:
            raise ValueError('Raw file integrity mismatch: '+rel)
    return m

class Monitor:
    def __init__(self,cfg):
        from collections import deque
        self.cfg=cfg;self.last={};self.types={};self.health=None;self.health_at=0
        self.history=deque(maxlen=2048);self.lock=threading.RLock()
    def observe(self,topic,typ,now=None):
        with self.lock:self.last[topic]=time.monotonic() if now is None else now;self.types[topic]=typ
    def hand_context(self,e,now=None):
        with self.lock:
            self.history.append(dict(e));self.observe(CONTEXT,'std_msgs/msg/String',now)
    def hand_health(self,h,now=None):
        with self.lock:self.health=h;self.health_at=time.monotonic() if now is None else now
    def check(self,session=None,now=None):
        now=time.monotonic() if now is None else now
        with self.lock:
            issues=[]
            for t,typ in self.cfg['required_topics'].items():
                age=now-self.last.get(t,-1e9)
                if age<0 or age>self.cfg['freshness_s']:issues.append('missing/stale '+t)
                if t in self.types and self.types[t]!=typ:issues.append('type mismatch '+t)
            if self.cfg.get('camera_required') and now-self.last.get(self.cfg['camera_topic'],-1e9)>self.cfg['freshness_s']:
                issues.append('camera missing/stale')
            h=self.health or {}
            if now-self.health_at>self.cfg['freshness_s'] or h.get('schema')!='apex_sharpa_health_v1' or not h.get('ready'):
                issues.append('hands not READY')
            if not h.get('live') and not self.cfg['diagnostic']:issues.append('hand source is not live')
            if h.get('serials')!=self.cfg['serials']:issues.append('hand serial mismatch')
            if session and h.get('session_id')!=session:issues.append('hand session changed')
            return issues
    def snapshot(self):
        with self.lock:return {'health':dict(self.health or {}),'context_history':list(self.history)}

def reserve_episode(cfg):
    """One durable allocation per attempted recording; never reuse a failed ID."""
    import fcntl
    state=Path(cfg['state_root']);state.mkdir(parents=True,exist_ok=True)
    with (state/'episode_registry.lock').open('a') as guard:
        fcntl.flock(guard,fcntl.LOCK_EX)
        p=state/'episode_registry.json'
        registry=json.loads(p.read_text()) if p.exists() else {'schema':'apex_episode_allocator_v1','next_index':0}
        if registry.get('schema')!='apex_episode_allocator_v1' or type(registry.get('next_index')) is not int:raise ValueError('Invalid episode allocator')
        used=[]
        for directory in Path(cfg['record_root']).iterdir():
            match=re.fullmatch(r'episode_([0-9]{8})',directory.name)
            if match:used.append(int(match[1]))
            manifest=directory/'capture_manifest.json'
            if manifest.is_file():
                old=json.loads(manifest.read_text())
                if type(old.get('episode_index')) is int:used.append(old['episode_index'])
        idx=max(registry['next_index'],max(used,default=-1)+1)
        if not 0<=idx<100_000_000:raise ValueError('Episode index exhausted')
        registry['next_index']=idx+1;atomic_json(p,registry)
        return idx

class Capture:
    """One native writer. Hooks only call native record start/stop, never playback."""
    def __init__(self,cfg,monitor,native,marker,auditor,clock=time.time_ns):
        self.cfg=cfg;self.monitor=monitor;self.native=native;self.marker=marker;self.auditor=auditor;self.clock=clock
        self.lock=threading.Lock();self.active=None;self.state='IDLE';self.last_result={}
        Path(cfg['state_root']).mkdir(parents=True,exist_ok=True)
    def status(self):
        return {'state':self.state,'active':self.active,'last_result':self.last_result,'ready_issues':self.monitor.check()}
    def start(self,name=''):
        if not self.lock.acquire(False):raise RuntimeError('Record transaction busy')
        if self.active:
            self.lock.release();raise RuntimeError('Recorder already active')
        attempted=False
        try:
            if self.active or self.native.recording():raise RuntimeError('Recorder already active')
            issues=self.monitor.check()
            if issues:raise RuntimeError('; '.join(issues))
            if name and (len(name)>80 or not re.fullmatch(r'[\w-]+',name)):raise ValueError('Unsafe recording name')
            root=Path(self.cfg['record_root']);root.mkdir(parents=True,exist_ok=True)
            v=os.statvfs(root)
            if v.f_bavail*v.f_frsize<self.cfg['minimum_free_bytes']:raise RuntimeError('Insufficient recording disk space')
            import uuid
            cid=uuid.uuid4().hex
            self.state='PREPARING';snap=self.monitor.snapshot();session=snap['health']['session_id']
            before={p.name for p in root.iterdir() if p.is_dir()}
            idx=reserve_episode(self.cfg);episode_name=f'episode_{idx:08d}'
            request=self.clock();attempted=True;response=self.native.command(1,episode_name)
            if not response.get('success'):raise RuntimeError('Native recording start failed: '+str(response))
            # Save an intent before resolving the writer path, so any later failure is visible.
            self.active={'episode_index':idx,'episode_name':episode_name,'recording_label':name,'capture_id':cid,'source_session_id':session,'start_request_ns':request,'start_ack_ns':self.clock(),
                         'bag':None,'diagnostic':self.cfg['diagnostic'],'status':'RECORDING'}
            deadline=time.monotonic()+3
            while True:
                new=[p for p in root.iterdir() if p.is_dir() and p.name not in before and not p.is_symlink()]
                if len(new)==1:break
                if len(new)>1:raise RuntimeError('Ambiguous new recording folders')
                if time.monotonic()>deadline:raise RuntimeError('Native writer directory not found')
                time.sleep(.02)
            bag=new[0].resolve()
            if bag.parent!=root.resolve():raise RuntimeError('Native writer path escaped recording root')
            self.active['bag']=str(bag)
            from datetime import datetime,timezone
            atomic_json(bag/'capture_time.json',{'utc':datetime.fromtimestamp(request/1e9,timezone.utc).isoformat(),
                        'capture_id':cid,'episode_index':idx,'episode_name':episode_name,'clock_domain':'robot_receive'})
            atomic_json(bag/'capture_manifest.json',dict(schema='apex_capture_manifest_v1',state='RECORDING',**self.active))
            self.marker(dict(schema='apex_capture_event_v1',kind='start',**self.active,snapshot=self.monitor.snapshot()))
            self.state='RECORDING';self._save();return dict(self.active)
        except Exception as e:
            if self.active:
                try:self._stop_locked('start transaction failed: '+str(e))
                except Exception:pass
            elif attempted:
                try:self.native.command(0,'')
                except Exception:pass
            self.state='STOP_FAILED' if self.active else 'FAILED'
            if self.active:self.active['failure_reason']='start transaction failed: '+str(e)
            self.last_result={'state':'FAILED','error':str(e)};self._save();raise
        finally:self.lock.release()
    def stop(self,reason=None):
        if not self.lock.acquire(False):raise RuntimeError('Record transaction busy')
        try:
            if not self.active:
                if self.native.recording():
                    self.native.command(0,'')
                    raise RuntimeError('Stopped untracked native recording; cannot certify COMPLETE')
                return self.last_result or {'state':'IDLE'}
            return self._stop_locked(reason)
        except Exception as e:
            if self.active:
                self.state='STOP_FAILED'
                self.active['failure_reason']=self.active.get('failure_reason') or str(e)
                self.last_result={'state':'FAILED','failure_reason':str(e),'retry_required':True}
                try:self._save()
                except Exception:pass  # A disk failure must not erase the active transaction.
            raise
        finally:self.lock.release()
    def _stop_locked(self,reason):
        self.state='FINALIZING';a=dict(self.active)
        reason=reason or a.get('failure_reason')
        a['stop_request_ns']=self.clock()
        # Publish before native stop; audit requires this marker actually exists in the bag.
        try:
            self.marker(dict(schema='apex_capture_event_v1',kind='stop',**a,reason=reason))
            time.sleep(.05)
        except Exception as e:
            reason=(reason+'; ' if reason else '')+'stop marker failed: '+str(e)
        # Even a failed event publisher must never prevent stopping the writer.
        try:response=self.native.command(0,'')
        except Exception as e:response={'success':False,'message':str(e)}
        a['stop_ack_ns']=self.clock()
        if not response.get('success'):
            reason=(reason+'; ' if reason else '')+'native stop failed: '+str(response)
            self.active.update(failure_reason=reason,status='STOP_FAILED')
            self.state='STOP_FAILED'
            self.last_result=dict(state='FAILED',failure_reason=reason,retry_required=True,
                                  capture_id=a['capture_id'],bag=a.get('bag'))
            self._save()
            return dict(self.last_result)
        # Retain the transaction until audit, hashes and both completion files persist.
        if a.get('bag'):
            bag=Path(a['bag'])
            try:
                audit=self.auditor(bag,self.cfg,a)
                if audit.get('issues'):reason=(reason+'; ' if reason else '')+'; '.join(audit['issues'])
            except Exception as e:audit={'issues':[str(e)]};reason=(reason+'; ' if reason else '')+'audit failed: '+str(e)
            a.update(schema='apex_capture_manifest_v1',state='FAILED' if reason else 'COMPLETE',status='FAILED' if reason else 'COMPLETE',failure_reason=reason,
                     clock_domain='robot_receive',serials=self.cfg['serials'],required_topics=self.cfg['required_topics'],audit=audit)
            raw={}
            for p in sorted(bag.rglob('*')):
                if p.is_symlink():reason='Symlink in raw capture';a['state']='FAILED';a['failure_reason']=reason
                if p.is_file() and p.name not in ('capture_manifest.json','capture_completion.json'):
                    raw[str(p.relative_to(bag))]={'size':p.stat().st_size,'sha256':sha256(p)}
            a['raw_files']=raw
            atomic_json(bag/'capture_manifest.json',a)
            atomic_json(bag/'capture_completion.json',dict(schema='apex_capture_completion_v1',capture_id=a['capture_id'],state=a['state'],
                        diagnostic=self.cfg['diagnostic'],episode_index=a['episode_index'],manifest_sha256=sha256(bag/'capture_manifest.json')))
        else:a.update(state='FAILED',failure_reason=reason or 'No native writer directory')
        self.state=a['state'];self.last_result=a;self._save();self.active=None;self._save();return a
    def watchdog(self):
        if self.active and self.state=='STOP_FAILED':
            return self.stop(self.active.get('failure_reason') or 'Retrying failed recording stop')
        if self.active and self.state=='RECORDING':
            issues=self.monitor.check(self.active['source_session_id'])
            if issues:return self.stop('recording source fault: '+'; '.join(issues))
    def _save(self):atomic_json(Path(self.cfg['state_root'])/'record_sync_status.json',self.status())
