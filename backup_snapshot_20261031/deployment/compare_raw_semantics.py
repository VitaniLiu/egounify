"""Read-only comparison of recorded VR targets, IK requests and LeRobot."""
import json
from pathlib import Path
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

ROOT=Path('/mnt/apex-data/apex_n_sharpa/data')
OUT=ROOT/'inspection/incremental_semantics_20261025'
OUT.mkdir(exist_ok=True)
bag=next((ROOT/'raw_data/apex_data/robot_import_20261024/episode_00000002/payload').glob('*.mcap'))
topics=['/tj/control/target_poseL','/tj/control/target_poseR','/tj/control/ik_request','/control/ik_request/vr','/tj/info/eef_left','/tj/info/eef_right']
rows={k:[] for k in topics}

def pose(p):
    q=p.orientation;v=np.array([q.w,q.x,q.y,q.z]);v=v/np.linalg.norm(v);w,x,y,z=v
    r=np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
    return np.array([p.position.x,p.position.y,p.position.z]),r

with bag.open('rb') as f:
    reader=make_reader(f,decoder_factories=[DecoderFactory()])
    for schema,ch,msg,m in reader.iter_decoded_messages(topics=topics):
        if 'ik_request' in ch.topic:
            rows[ch.topic].append((msg.log_time,m.header.frame_id,list(m.available),pose(m.left_hand_pose),pose(m.right_hand_pose)))
        else:rows[ch.topic].append((msg.log_time,m.header.frame_id,pose(m.pose)))

def stats(a):
    return {'max':float(np.max(a)),'median':float(np.median(a)),'p95':float(np.percentile(a,95))}

report={'bag':str(bag),'counts':{k:len(v) for k,v in rows.items()},'sides':{},'robot_commands_sent':0}
ik=rows['/tj/control/ik_request'];ti=np.array([x[0] for x in ik],np.int64)
report['ik_available_counts']=np.sum(np.array([x[2] for x in ik]),axis=0).tolist()
timeline=json.loads((ROOT/'lerobot/robot_import_20261024_egoav_v4/episode_00000002/meta/source_timestamps.json').read_text())
grid=np.array(timeline['timestamp_ns'],np.int64)
grid_ik=np.searchsorted(ti,grid,side='right')-1
report['lerobot_grid_ik_active_frames']=int(sum(ik[i][2][0] and ik[i][2][1] for i in grid_ik))
report['lerobot_grid_total_frames']=len(grid)
gate={'schema':'recorded_ik_availability_audit_v1','timestamp_ns':grid.tolist(),
      'ik_log_timestamp_ns':[int(ti[i]) for i in grid_ik],
      'available':[ik[i][2] for i in grid_ik],
      'sampling':'latest recorded IK at or before LeRobot frame; does not replace pose-validity mask',
      'source_topic':'/tj/control/ik_request','source_bag':str(bag),'robot_commands_sent':0}
(OUT/'recorded_ik_availability.json').write_text(json.dumps(gate,indent=2)+'\n')
flags=np.array([x[2][0] and x[2][1] for x in ik],bool)
edges=np.flatnonzero(np.diff(np.r_[False,flags,False].astype(int)))
report['active_intervals_ns']=[[int(ti[a]),int(ti[b-1])] for a,b in zip(edges[::2],edges[1::2])]
incremental=np.load(ROOT/'replay_incremental/robot_import_20261024_egoav_v4/episode_00000002/targets.npz',allow_pickle=False)
for side,letter,slot in [('left','L',0),('right','R',1)]:
    raw=rows['/tj/control/target_pose'+letter];tr=np.array([x[0] for x in raw],np.int64)
    # For each IK input, choose latest preceding raw target (controller sample-and-hold).
    ix=np.searchsorted(tr,ti,side='right')-1;good=ix>=0
    good &= (ti-tr[np.maximum(ix,0)])<30_000_000
    good &= np.array([x[2][slot] for x in ik],bool)
    indices=np.flatnonzero(good)
    a=np.array([raw[ix[i]][2][0] for i in indices]);b=np.array([ik[i][3+slot][0] for i in indices])
    ra=np.array([raw[ix[i]][2][1] for i in indices]);rb=np.array([ik[i][3+slot][1] for i in indices])
    if not len(a):report['sides'][side]={'active_pairs':0};continue
    d=b-a;first_delta_error=(b-b[0])-(a-a[0])
    left_offset=rb@ra.transpose(0,2,1);right_offset=ra.transpose(0,2,1)@rb
    rotation_error=lambda x,y:np.arccos(np.clip((np.trace(x.transpose(0,2,1)@y,axis1=1,axis2=2)-1)/2,-1,1))
    report['sides'][side]={'active_pairs':len(a),'frames_raw':sorted(set(x[1] for x in raw)),
        'frames_ik':sorted(set(x[1] for x in ik)),'first_position_offset_ik_minus_raw_m':d[0].tolist(),
        'constant_translation_offset_error_m':stats(np.linalg.norm(d-d[0],axis=1)),
        'episode_first_position_delta_error_m':stats(np.linalg.norm(first_delta_error,axis=1)),
        'constant_left_rotation_offset_error_rad':stats(rotation_error(np.repeat(left_offset[:1],len(a),axis=0),left_offset)),
        'constant_right_rotation_offset_error_rad':stats(rotation_error(np.repeat(right_offset[:1],len(a),axis=0),right_offset)),
        'first_rotation_offset_rad':float(rotation_error(ra[:1],rb[:1])[0]),
        'raw_motion_extent_m':stats(np.linalg.norm(a-a[0],axis=1)),
        'ik_motion_extent_m':stats(np.linalg.norm(b-b[0],axis=1))}
    # Refine asynchronous logging by looking only within +/-30ms, never a
    # global nearest-pose search. Estimate fixed offsets on first half only;
    # report second-half errors separately to avoid a fit-only conclusion.
    split=len(a)//2;translation=np.median(d[:split],axis=0)
    representative=int(np.argmin(np.linalg.norm(d[:split]-translation,axis=1)))
    orientation=right_offset[representative]
    rawp=np.array([x[2][0] for x in raw]);rawr=np.array([x[2][1] for x in raw])
    matched=[];position_errors=[];rotation_errors=[];timing=[]
    for k,i in enumerate(indices):
        lo=np.searchsorted(tr,ti[i]-30_000_000);hi=np.searchsorted(tr,ti[i]+30_000_000)
        if hi<=lo:continue
        j=lo+np.argmin(np.linalg.norm(rawp[lo:hi]+translation-b[k],axis=1))
        matched.append(j);position_errors.append(np.linalg.norm(rawp[j]+translation-b[k]))
        rotation_errors.append(rotation_error((rawr[j]@orientation)[None],rb[k:k+1])[0]);timing.append(abs(int(tr[j])-int(ti[i])))
    report['sides'][side]['time_bounded_match']={
        'method':'fixed translation and right-rotation offsets fitted on first half; closest raw position within +/-30ms',
        'translation_offset_m':translation.tolist(),'rotation_right_offset':orientation.tolist(),
        'position_error_m':stats(position_errors),'rotation_error_rad':stats(rotation_errors),
        'heldout_second_half_position_error_m':stats(position_errors[split:]),
        'heldout_second_half_rotation_error_rad':stats(rotation_errors[split:]),
        'exact_position_pairs_1e_8_m':int(np.sum(np.array(position_errors)<1e-8)),
        'exact_rotation_pairs_1e_6_rad':int(np.sum(np.array(rotation_errors)<1e-6)),
        'max_time_difference_ns':max(timing)}
    bad=np.flatnonzero((np.array(position_errors)>1e-8)|(np.array(rotation_errors)>1e-6))
    report['sides'][side]['unmatched_samples']=[{'ik_timestamp_ns':int(ti[indices[k]]),'position_error_m':float(position_errors[k]),'rotation_error_rad':float(rotation_errors[k])} for k in bad]
    raw_by_time={x[0]:x[2] for x in raw}
    selected=[raw_by_time[t] for t in timeline['aligned_sources']['actions/'+side]['timestamp_ns']]
    original=np.repeat(np.eye(4)[None],len(grid),axis=0)
    original[:,:3,3]=np.array([x[0] for x in selected]);original[:,:3,:3]=np.array([x[1] for x in selected])
    anchor=incremental['action_anchor_base_matrix'][slot];delta=incremental['action_'+side+'_delta_matrix']
    restored=delta.copy();restored[:,:3,3]+=anchor[:3,3];restored[:,:3,:3]=delta[:,:3,:3]@anchor[:3,:3]
    transform=restored[0]@np.linalg.inv(original[0])
    expected=transform@original
    report['sides'][side]['lerobot_raw_source_check']={
        'samples':len(grid),'timestamp_lookup':'exact recorded source log timestamp, not nearest matching',
        'constant_base_from_tracking':transform.tolist(),
        'max_matrix_error':float(np.max(np.abs(restored-expected))),
        'position_error_m':stats(np.linalg.norm(restored[:,:3,3]-expected[:,:3,3],axis=1)),
        'rotation_matrix_error':float(np.max(np.abs(restored[:,:3,:3]-expected[:,:3,:3])))}
    np.savez_compressed(OUT/(side+'_raw_ik_pairs.npz'),timestamp_ns=ti[indices],raw_p=a,ik_p=b,raw_R=ra,ik_R=rb)
(OUT/'raw_comparison.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
