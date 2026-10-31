"""Suspend only this module's fake producer; require watchdog stop and exit."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
ROOT=Path(__file__).resolve().parents[1]
with (ROOT/'logs/fake-source-loss.log').open('w') as log:
    proc=subprocess.Popen([str(ROOT/'scripts/start_hands.sh'),'--fake','--duration','20'],stdout=log,stderr=subprocess.STDOUT)
    target=None
    try:
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            rows=subprocess.check_output(['ps','-eo','pid,args'],text=True).splitlines()
            matches=[r.strip().split(None,1) for r in rows if str(ROOT)+'/portable_deps/' in r
                     and '/lib/manus_sharpa_teleop/retarget_node' in r and '__node:=manus_sharpa_retarget' in r]
            if matches:target=int(matches[0][0]);break
            time.sleep(.1)
        if target is None:raise RuntimeError('Isolated fake producer not found')
        time.sleep(1)
        os.kill(target,signal.SIGSTOP)
        time.sleep(1)
        os.kill(target,signal.SIGCONT)
        code=proc.wait(timeout=12)
        if code==0:raise AssertionError('Source loss did not fail closed')
        driverlog=max(ROOT.glob('logs/*driver.log'),key=lambda p:p.stat().st_mtime)
        text=driverlog.read_text()
        if 'stopped after command timeout' not in text:raise AssertionError('Native watchdog stop not observed')
        report={'fake':True,'source_suspended':True,'exit_code':code,'native_watchdog_stop':True,'hardware_initialized':False}
        (ROOT/'reports/fake-source-loss.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report))
    finally:
        if target:
            try:os.kill(target,signal.SIGCONT)
            except ProcessLookupError:pass
        if proc.poll() is None:proc.terminate();proc.wait(timeout=15)
