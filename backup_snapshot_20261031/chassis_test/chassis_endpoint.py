"""Chassis-only robot endpoint. No ROS import or hardware access on import."""
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time
import tempfile

BACKEND_SHA256 = 'd87b268260101a9cb1e02573e7ecaf4de44b6fc1ac1544bf0c112271412dea61'
STEP_NS = 50_000_000
MAX_LATENESS_S = .010


def json_safe(value):
    """Preserve nonfinite diagnostic evidence without emitting invalid JSON."""
    if isinstance(value, float) and not math.isfinite(value):
        return 'NaN' if math.isnan(value) else 'Infinity' if value > 0 else '-Infinity'
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


def save_report(path, plan, report):
    """Local robot evidence survives loss of the SSH stdout connection."""
    with Path(path).open('w') as f:
        json.dump(json_safe({'plan':plan,'result':report}),f,allow_nan=False)
        f.flush()
        os.fsync(f.fileno())


def require(ok, message):
    if not ok:
        raise ValueError(message)


def validate_plan(plan):
    require(plan.get('schema') == 'chassis_parquet_20hz_v1', 'Invalid plan schema')
    require(plan.get('frame') == 'base_link', 'Require base_link velocity')
    require(plan.get('units') == ['m/s', 'm/s', 'rad/s'], 'Invalid velocity units')
    offsets, values = plan['offset_ns'], plan['velocity']
    require(2 <= len(values) <= 2400 and len(offsets) == len(values), 'Invalid frame count')
    require(all(type(v) is int and v == i*STEP_NS for i,v in enumerate(offsets)), 'Require 20Hz offsets')
    require(type(plan['end_ns']) is int and plan['end_ns'] == len(values)*STEP_NS, 'Invalid stop deadline')
    for v in values:
        require(len(v) == 3 and all(type(x) in (int,float) and math.isfinite(x) for x in v), 'Invalid velocity')
        require(abs(v[0]) <= .400001 and abs(v[2]) <= .500001, 'Velocity exceeds existing envelope')
        require(v[1] == 0, 'Nonzero vy is not verified; refusing lateral command')


def run_schedule(plan, transport, guard, clock=time.monotonic, sleep=time.sleep):
    """Hold each original 20Hz sample for its slot; no backlog burst or resampling."""
    validate_plan(plan)
    report = {'status': 'running', 'sent_frames': 0, 'stop_confirmed': False,
              'max_lateness_ms': 0., 'trajectory_accuracy_verified': False}
    started = clock()
    previous = None
    try:
        for offset, v in zip(plan['offset_ns'], plan['velocity']):
            deadline = started + offset/1e9
            while clock() < deadline:
                guard(); transport.check()
                if previous is not None:
                    transport.tracking(previous)
                sleep(min(.005, max(0., deadline-clock())))
            guard(); transport.check()
            if previous is not None:
                transport.tracking(previous)
            delay = clock()-deadline
            if delay > MAX_LATENESS_S:
                raise RuntimeError('Replay late >10ms; no catch-up burst')
            previous = [v[0], v[1], 0., 0., 0., v[2]]
            transport.publish(previous, stamp=offset)
            report['sent_frames'] += 1
            report['max_lateness_ms'] = max(report['max_lateness_ms'], delay*1000)
        end = started + plan['end_ns']/1e9
        while clock() < end:
            guard(); transport.check(); transport.tracking(previous)
            sleep(min(.005, max(0., end-clock())))
        if clock()-end > MAX_LATENESS_S:
            raise RuntimeError('Replay end deadline missed')
        report['status'] = 'completed'
    except BaseException as exc:
        report.update(status='failed', error=f'{type(exc).__name__}: {exc}')
    finally:
        try:
            report.update(transport.stop())
        except BaseException as exc:
            report.update(status='failed', stop_confirmed=False, stop_error=str(exc))
        if report.get('stop_confirmed') is not True:
            report.update(status='failed', stop_error=report.get('stop_error', 'STOP_UNCONFIRMED'))
        report['elapsed_s'] = clock()-started
    return report


class HostLease:
    def __init__(self):
        self.last = time.monotonic()
        self.cancelled = threading.Event()
    def read(self, stream):
        try:
            for line in stream:
                if line.strip() != 'PING':
                    break
                self.last = time.monotonic()
        finally:
            self.cancelled.set()
    def check(self):
        if self.cancelled.is_set() or time.monotonic()-self.last > .5:
            raise RuntimeError('Host heartbeat lost or stop requested')


class RobotTransport:
    """Reuse the existing verified chassis guard, with stricter three-axis stop checks."""
    def __init__(self, backend, plan):
        # The legacy validator requires a trailing zero; this synthetic stop is
        # separate from source samples and never written back to the dataset.
        rows = [(t,[v[0],v[1],0.,0.,0.,v[2]]) for t,v in zip(plan['offset_ns'],plan['velocity'])]
        rows.append((plan['end_ns'],[0.]*6))
        self.c = backend['CoordinatedChassis'](None, rows=rows)
        self.started = False
    def check(self):
        self.c.check()
        if not self.c.thread.is_alive():
            raise RuntimeError('Chassis ROS executor stopped')
    def ready(self):
        self.check(); self.c.ready()
        if abs(self.c.state['message'].speed.linear.y) > .01:
            raise RuntimeError('Chassis lateral motion before start')
        self.c.tracking([0.]*6)  # includes motor/STM/IMU freshness and fault checks
    def tracking(self, values): self.c.tracking(values)
    def publish(self, values, stamp=None):
        self.started = True
        self.c.began = True
        self.c.publish(values, stamp=stamp)
    def stop(self):
        # Stop is independent of host lease / mode / tracking guards.
        confirmed = False
        good_since = None
        errors = []
        start = time.monotonic()
        while self.started and time.monotonic()-start < 4.:
            try:
                self.c.publish([0.]*6)
            except Exception as exc:
                errors.append(str(exc))
            now = time.monotonic(); state = self.c.state
            m = state.get('message')
            good = (m is not None and now-state.get('time',0) <= .2 and
                    all(math.isfinite(v) and abs(v)<limit for v,limit in
                        zip([m.speed.linear.x,m.speed.linear.y,m.speed.angular.z],[.01,.01,.02])))
            good_since = (now if good_since is None else good_since) if good else None
            if good_since is not None and now-good_since >= .2 and now-start >= .5:
                confirmed = True
                break
            time.sleep(.02)
        return {'stop_confirmed': confirmed, 'stop_publish_errors': errors,
                'samples': self.c.samples, 'published': self.c.published,
                'tracking_phases': self.c.tracking_guard.events,
                'diagnostics': self.c.diagnostics}
    def close(self):
        for executor,node,context,thread in [
            (self.c.executor,self.c.node,self.c.context,self.c.thread)]:
            try:
                if executor: executor.shutdown(timeout_sec=1.)
                if node: node.destroy_node()
                if context and context.ok(): context.shutdown()
                if thread: thread.join(timeout=1.)
            except Exception:
                pass


def main():
    # All input is validated before initializing ROS. No robot files installed.
    request = json.loads(sys.stdin.readline())
    plan = request['plan']; validate_plan(plan)
    require(type(request.get('execute')) is bool, 'Explicit execution mode required')
    source = request['backend']
    require(hashlib.sha256(source.encode()).hexdigest() == BACKEND_SHA256, 'Chassis backend changed; review required')
    backend = {'__name__':'chassis_readonly_import'}
    exec(compile(source,'<hash-pinned-chassis-backend>','exec'),backend)
    lease = HostLease()
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
        signal.signal(sig, lambda *_:lease.cancelled.set())
    threading.Thread(target=lease.read,args=(sys.stdin,),daemon=True).start()
    # Prevent concurrent invocations of this test; existing publishers guarded separately.
    import fcntl
    lock = open('/tmp/apex-chassis-parquet-test.lock','a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    tr = None
    report_path = None
    report = {'status':'failed','stop_confirmed':None,'sent_frames':0,'motion_started':False}
    try:
        tr = RobotTransport(backend,plan)
        deadline = time.monotonic()+6.
        last_error = None
        while time.monotonic()<deadline:
            lease.check()
            try:
                tr.ready()
                last_error = None
                break
            except RuntimeError as exc:
                last_error = exc
                time.sleep(.02)
        if last_error: raise last_error
        lease.check(); tr.ready()
        print(json.dumps({'event':'preflight_passed','execute':request['execute']}),flush=True)
        if request['execute']:
            report_path = Path(tempfile.mkdtemp(prefix='apex-chassis-parquet-'))/'report.json'
            save_report(report_path,plan,{'status':'starting','stop_confirmed':None})
            print(json.dumps({'event':'robot_report','path':str(report_path)}),flush=True)
            report = run_schedule(plan,tr,lease.check)
        else:
            report = {'status':'preflight_passed','sent_frames':0,'stop_confirmed':None}
    except BaseException as exc:
        report.update(status='failed',error=f'{type(exc).__name__}: {exc}')
    finally:
        report['motion_started'] = bool(tr and tr.started)
        if tr: tr.close()
        lock.close()
        if report_path:
            report['robot_report'] = str(report_path)
            try: save_report(report_path,plan,report)
            except Exception as exc:
                report.update(status='failed',report_save_error=str(exc))
    print(json.dumps(json_safe({'result':report}),allow_nan=False),flush=True)
    return 0 if report['status'] in ('completed','preflight_passed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
