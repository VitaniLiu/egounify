"""Move selected Sharpa hands to SDK open pose; not reboot or encoder zeroing."""
import argparse
import fcntl
import json
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from hands_common import ROOT,config,NAMES,ensure_sdk_ports_free

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--side',choices=['left','right','both'],default='both')
    p.add_argument('--execute',action='store_true');p.add_argument('--fake',action='store_true')
    a=p.parse_args()
    if a.execute and a.fake:p.error('Choose execute or fake')
    c=config();sides=['left','right'] if a.side=='both' else [a.side]
    if not a.execute and not a.fake:
        print(json.dumps({'plan':'SDK open pose, 22 zero radians','hands':{s:c['serials'][s] for s in sides},'hardware_initialized':False}));return
    from litchi_hardware.hardware.sharpa import create_hand,SharpaConfig
    from litchi_hardware.core.types import HandSide
    import math
    lock=(ROOT/'.hands.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if a.execute:ensure_sdk_ports_free()
    hands={};result={'fake':a.fake,'operation':'open_pose','reboot':False,'hands':{}}
    failure=None
    try:
        for side in sides:
            hand=create_hand(SharpaConfig(name='sharpa_open_'+side,side=HandSide(side),serial=c['serials'][side],
                                         speed_coefficient=c['speed_coefficient'],current_coefficient=c['current_coefficient'],
                                         enable_tactile=False),use_fake_hardware=a.fake)
            hands[side]=hand;hand.connect()
        # Receiving new hardware packets must be established before sending a target.
        packets={};last_packet_at={}
        if not a.fake:
            deadline=time.monotonic()+2
            observed={s:0 for s in sides}
            while not all(v>=3 for v in observed.values()):
                for side,hand in hands.items():
                    raw=hand._wave.get_states();key=(int(raw.timestamp),int(raw.sequence))
                    if key[0]>0 and key!=packets.get(side):
                        packets[side]=key;observed[side]+=1;last_packet_at[side]=time.monotonic()
                if time.monotonic()>deadline:raise TimeoutError('Fresh Sharpa feedback unavailable before sending any open target')
                time.sleep(.02)
        for side,hand in hands.items():
            result['hands'][side]={'initial_position_rad':list(hand.read_joint_state().position)}
            if not a.fake:
                diagnostics={}
                for getter in ('get_enable_state','get_mcu_enable','get_control_source','get_control_mode'):
                    error,value=getattr(hand._wave,getter)()
                    hand._check_error(error,getter)
                    diagnostics[getter]=str(value)
                result['hands'][side]['settings_before_target']=diagnostics
                if diagnostics['get_enable_state']!='True' or diagnostics['get_mcu_enable']!='True':
                    raise RuntimeError('Hand or MCU not enabled before target: '+side)
            hand.open()
        start=time.monotonic();matches={s:0 for s in sides}
        while True:
            for side,hand in hands.items():
                fresh=True
                if a.fake:q=hand.read_joint_state().position
                else:
                    raw=hand._wave.get_states();q=list(raw.angles);key=(int(raw.timestamp),int(raw.sequence))
                    fresh=key!=packets.get(side)
                    if fresh:packets[side]=key;last_packet_at[side]=time.monotonic()
                    if time.monotonic()-last_packet_at[side]>.2:raise TimeoutError('Sharpa feedback packet stream stale: '+side)
                if len(q)!=22 or not all(math.isfinite(x) for x in q):raise RuntimeError('Invalid hand feedback')
                error=max(abs(x) for x in q)
                if fresh:matches[side]=matches[side]+1 if error<=.03 else 0
                result['hands'][side].update(serial=c['serials'][side],max_feedback_error_rad=error,
                    final_position_rad=list(q),out_of_tolerance=[{'joint':n,'position_rad':float(v)} for n,v in zip(NAMES,q) if abs(v)>.03])
            if time.monotonic()-start>=.2 and all(n>=3 for n in matches.values()):break
            if time.monotonic()-start>8:raise TimeoutError('Open pose feedback did not reach 0.03rad tolerance within 8s; see report for joint errors')
            time.sleep(.05)
        result['success']=True
    except Exception as e:
        failure=e;result.update(success=False,error=str(e))
    finally:
        failures=[]
        for hand in hands.values():
            try:hand.stop()
            except Exception as e:failures.append(str(e))
            try:hand.disconnect()
            except Exception as e:failures.append(str(e))
        result['cleanup_errors']=failures
        if failures and failure is None:failure=RuntimeError('Hand cleanup errors: '+'; '.join(failures))
        report='reset-hands-fake.json' if a.fake else 'reset-hands-latest.json'
        (ROOT/'reports'/report).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    if failure:raise failure

if __name__=='__main__':main()
