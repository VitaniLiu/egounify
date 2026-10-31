"""Offline target screening only; never imports ROS or connects to a robot."""
import argparse,json
from pathlib import Path
import numpy as np

def audit(p):
 t=np.asarray(p['offset_ns'])/1e9;dt=np.diff(t);gate=np.asarray(p['available'],bool)
 out={'frames':len(t),'duration_s':float(t[-1]-t[0]),'source_sha256':p['source_sha256'],'sends_robot_commands':False,'arms':{},'gate_transitions':np.flatnonzero(np.any(gate[1:]!=gate[:-1],axis=1)).__add__(1).tolist(),'scope':'Cartesian target screening, not controller IK, collision or tracking validation'}
 for side,slot in [('left',0),('right',1)]:
  a=np.asarray(p[side]);xyz=a[:,:3,3];step=np.linalg.norm(np.diff(xyz,axis=0),axis=1)
  r=a[:-1,:3,:3].transpose(0,2,1)@a[1:,:3,:3];angles=np.arccos(np.clip((np.trace(r,axis1=1,axis2=2)-1)/2,-1,1))
  mask=gate[:-1,slot]&gate[1:,slot];idx=np.flatnonzero(mask);speed=step/dt
  out['arms'][side]={'active_frames':int(gate[:,slot].sum()),'max_displacement_from_start_m':float(np.max(np.linalg.norm(xyz-xyz[0],axis=1))),'max_active_step_m':float(step[mask].max()),'max_active_target_speed_m_s':float(speed[mask].max()),'max_active_rotation_speed_rad_s':float((angles/dt)[mask].max()),'fastest_active_transition_frames':[int(idx[np.argmax(speed[mask])]),int(idx[np.argmax(speed[mask])]+1)],'prefix_displacement_m':{str(n):float(np.max(np.linalg.norm(xyz[:n]-xyz[0],axis=1))) for n in [6,10,20,len(t)]}}
 for name in ['body','head']:
  a=np.asarray(p[name]);out[name]={'min':a.min(axis=0).tolist(),'max':a.max(axis=0).tolist(),'max_abs_target_rate':np.max(np.abs(np.diff(a,axis=0))/dt[:,None],axis=0).tolist()}
 return out
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('plan',type=Path);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args();out=audit(json.loads(args.plan.read_text()));args.output.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
