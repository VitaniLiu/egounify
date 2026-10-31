"""Read-only mileage comparison of one replay report against original Raw MCAP."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


def require(ok, message):
    if not ok:
        raise ValueError(message)


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''): h.update(block)
    return h.hexdigest()


def check_rows(rows):
    a=np.asarray(rows,dtype=float)
    require(a.ndim==2 and a.shape[1]==3 and len(a)>=2 and np.isfinite(a).all(), 'Invalid odometry')
    require(np.all(np.diff(a[:,0])>0), 'Nonincreasing odometry times')
    return a


def window(rows, duration):
    a=check_rows(rows)
    require(a[0,0]<=0<=duration<=a[-1,0], 'Odometry does not cover the full comparison window')
    require(np.max(np.diff(a[:,0]))<=.1, 'Odometry gap >100ms; comparison refused')
    grid=np.arange(round(duration*20)+1,dtype=float)/20
    require(abs(grid[-1]-duration)<1e-8,'Duration must be on 20Hz grid')
    xy=np.column_stack([np.interp(grid,a[:,0],a[:,k]) for k in (1,2)])
    relative=xy-xy[0]
    native_t=np.concatenate(([0.],a[(a[:,0]>0)&(a[:,0]<duration),0],[duration]))
    native_xy=np.column_stack([np.interp(native_t,a[:,0],a[:,k]) for k in (1,2)])
    metrics={
        'duration_s':duration,'path_m_20hz':float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),
        'path_m_native':float(np.linalg.norm(np.diff(native_xy,axis=0),axis=1).sum()),
        'net_displacement_m':float(np.linalg.norm(relative[-1])),
        'max_distance_from_start_m':float(np.linalg.norm(relative,axis=1).max()),
        'local_odom_delta_xy_m':relative[-1].tolist(),
        'input_samples':len(a),'max_gap_s':float(np.diff(a[:,0]).max()),
        'input_hz':float((len(a)-1)/(a[-1,0]-a[0,0]))}
    return metrics,np.column_stack([grid,relative])


def forward_segments(plan, reference, measured):
    actions=np.asarray(plan['velocity'],dtype=float)
    forward=actions[:,0]>1e-6
    edges=np.diff(np.r_[False,forward,False].astype(int))
    result=[]
    for start,command_end in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)):
        end=command_end
        # Include the following zero-command deceleration, ending before the
        # next forward/reverse/turn command. Never include another action.
        while end<len(actions) and not np.any(np.abs(actions[end])>1e-6):
            end+=1
        item={'start_s':float(start/20),'forward_command_end_s':float(command_end/20),
              'measurement_end_s':float(end/20)}
        for name,data in [('raw',reference),('replay',measured)]:
            xy=data[start:end+1,1:3]
            item[name]={'path_m':float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),
                        'net_displacement_m':float(np.linalg.norm(xy[-1]-xy[0]))}
        delta=item['replay']['path_m']-item['raw']['path_m']
        item['difference_m']=delta
        item['difference_percent']=100*delta/item['raw']['path_m'] if item['raw']['path_m'] else None
        result.append(item)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    args=p.parse_args(); run=args.run.resolve()
    plan=json.loads((run/'plan.json').read_text())
    replay=json.loads((run/'report.json').read_text())
    require(replay['status']=='completed' and replay.get('stop_confirmed') is True,'Replay was not complete/stopped')
    require(replay['sent_frames']==len(plan['velocity']),'Frame count mismatch')
    source=Path(plan['source'])
    raw=Path('/mnt/apex-data/apex_n_sharpa/data/raw_data/apex_data')/source.parent.name/source.name
    manifest=json.loads((raw/'manifest.json').read_text())
    rows=[]; files=[]; topic='/recording/chassis/state'
    origin_ns=plan['source_timestamp_ns'][0]
    for entry in manifest['files']:
        rel=Path(entry['path'])
        if rel.suffix!='.mcap' or 'video' in rel.parts: continue
        require(not rel.is_absolute() and '..' not in rel.parts,'Unsafe Raw path')
        path=raw/'payload'/rel
        require(path.stat().st_size==entry['size'] and digest(path)==entry['sha256'],'Raw integrity mismatch')
        files.append({'path':str(path),'sha256':entry['sha256']})
        with path.open('rb') as f:
            for _,channel,record,msg in make_reader(f,decoder_factories=[DecoderFactory()]).iter_decoded_messages(topics=[topic]):
                rows.append(((int(record.log_time)-origin_ns)/1e9,msg.odometry.position.x,msg.odometry.position.y))
    require(bool(rows),'Raw odometry topic missing')
    rows.sort()
    commands=[v for v in replay['published'] if v.get('source_ns') is not None]
    require(commands[0]['source_ns']==0,'Replay start command missing')
    start=commands[0]['t']; duration=plan['end_ns']/1e9
    actual=[(s['t']-start,s['x'],s['y']) for s in replay['samples']]
    reference,ref_csv=window(rows,duration)
    measured,actual_csv=window(actual,duration)
    tail_duration=float(actual[-1][0])
    tail_rows=check_rows(actual)
    tail_xy=np.column_stack([np.interp([0.,tail_duration],tail_rows[:,0],tail_rows[:,k]) for k in (1,2)])
    delta=measured['path_m_20hz']-reference['path_m_20hz']
    result={
        'schema':'chassis_mileage_comparison_v1','run':str(run),'raw':str(raw),
        'raw_source':manifest['source'],'raw_topic':topic,'raw_files':files,
        'replay_complete':True,'sent_frames':replay['sent_frames'],'stop_confirmed':True,
        'measurement':'chassis odometry, not external ground truth',
        'comparison':'same 20Hz sampling and duration; separate origins; scalar distances are rotation invariant',
        'raw_full_duration_s':rows[-1][0]-rows[0][0],
        'selected_start_offset_in_raw_s':-rows[0][0],
        'raw_reference':reference,'replay_measured':measured,
        'difference':{'path_m':delta,'path_percent':100*delta/reference['path_m_20hz'] if reference['path_m_20hz'] else None,
            'net_displacement_m':measured['net_displacement_m']-reference['net_displacement_m']},
        'forward_segments_policy':'Positive vx command plus following zero-command deceleration; stop before next nonzero action; 20Hz odometry',
        'forward_segments':forward_segments(plan,ref_csv,actual_csv),
        'replay_including_stop':{'duration_s':tail_duration,'net_displacement_m':float(np.linalg.norm(tail_xy[-1]-tail_xy[0]))},
        'axis_note':'Raw and replay local odom axes have not been yaw-aligned; do not interpret XY difference as endpoint position error.'}
    for name,data in [('raw_odometry_20hz.csv',ref_csv),('replay_odometry_20hz.csv',actual_csv)]:
        with (run/name).open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['time_s','relative_odom_x_m','relative_odom_y_m']);w.writerows(data.tolist())
    (run/'mileage_comparison.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))


if __name__=='__main__': main()
