"""Read real MANUS frames only. Never creates a Sharpa device or publishes ROS."""
import fcntl
import json
from pathlib import Path
import sys
import time
import traceback
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from hands_common import ROOT

def main():
    from litchi_hardware.hardware.manus.native_driver import ManusNativeTracker
    from litchi_hardware.hardware.manus.config import ManusNativeConfig
    from litchi_hardware.core.types import HandSide
    lock=(ROOT/'.hands.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    # An empty calibration directory prevents applying the previous wearer's mcal.
    empty=ROOT/'data/manus_probe_no_calibration';empty.mkdir(parents=True,exist_ok=True)
    if any(empty.iterdir()):raise RuntimeError('Probe calibration directory must stay empty')
    tracker=ManusNativeTracker(ManusNativeConfig(name='read_only_manus_probe',native_host_discovery_s=3,
        calibration_directory=str(empty)))
    result={'sharpa_initialized':False,'robot_commands_sent':0,'calibration_applied':False,'sdk_connected':False,'hands':{}}
    records={s:[] for s in HandSide};error=None
    started=time.monotonic()
    try:
        tracker.connect();result['sdk_connected']=True
        print('MANUS SDK connected; reading left/right for 15s',flush=True)
        end=time.monotonic()+15
        while time.monotonic()<end:
            for side in HandSide:
                try:
                    f=tracker.read_frame(side,timeout_s=.05)
                    records[side].append((time.monotonic(),f.timestamp_s,f.sequence,len(f.keypoints)))
                except TimeoutError:pass
        import ctypes
        from litchi_hardware.hardware.manus.vendor import load_manus_library
        sdk=load_manus_library()
        count=ctypes.c_uint32()
        fn=sdk.CoreSdk_GetNumberOfAvailableGloves
        fn.argtypes=[ctypes.POINTER(ctypes.c_uint32)];fn.restype=ctypes.c_int
        rc=fn(ctypes.byref(count))
        result['glove_enumeration']={'sdk_return_code':rc,'count':count.value if rc==0 else None}
    except Exception as exc:
        error=str(exc);result['error']=error
    finally:
        tracker.disconnect()
    for side,rows in records.items():
        unique=len(set(r[2] for r in rows))
        result['hands'][side.value]={'frames':len(rows),'unique_sequences':unique,
            'observed_hz':round((len(rows)-1)/(rows[-1][0]-rows[0][0]),2) if len(rows)>1 else 0,
            'keypoint_counts':sorted(set(r[3] for r in rows)),
            'last_frame_age_s':round(time.monotonic()-rows[-1][0],3) if rows else None}
    result['both_streaming']=all(v['frames']>=10 and v['unique_sequences']>=10 and v['keypoint_counts']==[25] for v in result['hands'].values())
    result['elapsed_s']=round(time.monotonic()-started,2)
    (ROOT/'reports/manus-connection-check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
    return 0 if result['both_streaming'] and not error else 1

if __name__=='__main__':raise SystemExit(main())
