"""Real MANUS + V4 diagnostics only; never initializes Sharpa hardware."""
import json,time,sys,argparse
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from hands_common import ROOT
import numpy as np
from litchi_hardware.hardware.manus.native_driver import ManusNativeTracker
from litchi_hardware.hardware.manus.config import ManusNativeConfig
from litchi_hardware.core.types import HandSide
from manus_sharpa_teleop.v4_retargeter import SharpaV4Retargeter,V4RetargetingConfig

tracker=ManusNativeTracker(ManusNativeConfig(name='mapping_diagnostic',calibration_directory=str(ROOT/'portable_deps/litchi_hardware/calibration/metaglove')))
p=argparse.ArgumentParser();p.add_argument('--duration',type=float,default=0);args=p.parse_args()
r=None;rows=[]
try:
    r=SharpaV4Retargeter(V4RetargetingConfig())
    tracker.connect();time.sleep(2)
    ready={};deadline=time.monotonic()+12
    while len(ready)<2 and time.monotonic()<deadline:
        for side in HandSide:
            try:ready[side]=tracker.read_frame(side,.1)
            except TimeoutError:pass
    if len(ready)<2:raise RuntimeError('Both MANUS sides not ready within 12s')
    start_loop=time.monotonic();i=0
    while (time.monotonic()-start_loop<args.duration) if args.duration else i<8:
        i+=1
        for side in HandSide:
            try:frame=tracker.read_frame(side,.05)
            except TimeoutError:
                rows.append({'side':side.value,'error':'no MANUS frame'});continue
            pos=np.asarray([p.position for p in frame.keypoints])
            row={'side':side.value,'sequence':frame.sequence,'position_span':np.ptp(pos,axis=0).tolist(),'finite':bool(np.isfinite(pos).all())}
            start=time.monotonic()
            try:
                a=r.retarget(frame);q=np.asarray(a.features['joint_position'])
                row.update(seconds=time.monotonic()-start,finite_target=bool(np.isfinite(q).all()),target=q.tolist())
            except Exception as e:row.update(seconds=time.monotonic()-start,error=str(e))
            rows.append(row);print(json.dumps(row),flush=True)
finally:
    tracker.disconnect()
    if r:r.close()
    (ROOT/'reports/mapping-diagnostic.json').write_text(json.dumps(rows,indent=2))
