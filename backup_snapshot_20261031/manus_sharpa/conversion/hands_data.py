"""Hand-only MCAP -> HDF5 -> derived LeRobot slots. No original files edited."""
import argparse
import json
import heapq
from pathlib import Path
import shutil
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import h5py
import numpy as np
from hands_common import ROOT, NAMES, SLOTS, config, digest, normalize, match

def write_json(path,obj):
    Path(path).write_text(json.dumps(obj,ensure_ascii=False,indent=2,allow_nan=False)+'\n')

def output_path(path):
    path=Path(path).resolve()
    data=(ROOT/'data').resolve()
    if not path.is_relative_to(data):raise ValueError('Outputs must be under hand module data/, not original datasets')
    if path.exists():raise FileExistsError(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    return path

def extract(paths,journals,output,capture_id):
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    evidence={}
    for path in journals:
        for line in Path(path).read_text().splitlines():
            e=json.loads(line)
            if e.get('clock_domain')!='robot_receive':raise ValueError('Journal is not robot receiver evidence')
            key=(e['session_id'],e['seq'])
            if key in evidence:raise ValueError('Duplicate receiver event')
            evidence[key]=e
    streams={(s,k):[] for s in SLOTS for k in ('state','action')}
    topics={f'/hand_{s}/joint_{suffix}':(s,k) for s in SLOTS for k,suffix in [('state','states'),('action','commands')]}
    for path in paths:
        with open(path,'rb') as f:
            reader=make_reader(f,decoder_factories=[DecoderFactory()])
            for schema,channel,record,msg in reader.iter_decoded_messages(topics=list(topics)):
                side,kind=topics[channel.topic]
                frame=msg.header.frame_id.split('/')
                if len(frame)!=3 or frame[0]!='manus_sharpa':raise ValueError('Unverified command/state source')
                key=(frame[1],int(frame[2]));e=evidence[key]
                if e['side']!=side or e['kind']!=('accepted' if kind=='action' else 'state'):
                    raise ValueError('Receiver event mismatch')
                if e.get('serials')!=config()['serials']:raise ValueError('Hand device binding differs')
                if kind=='action' and e['sdk_result']['status']!='sdk_call_succeeded':raise ValueError('Unaccepted action')
                q=normalize(msg.name,msg.position,side)
                if not np.array_equal(q,np.asarray(e['position'])):raise ValueError('MCAP/evidence target differs')
                header=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                if header!=e['received_robot_ns']:raise ValueError('Receiver header differs')
                if abs(record.log_time-header)>500_000_000:raise ValueError('Recorder backlog exceeds 0.5s')
                streams[side,kind].append((record.log_time,e['source_stamp_ns'],header,q,key))
    sessions={r[4][0] for rows in streams.values() for r in rows}
    if len(sessions)!=1:raise ValueError('Exactly one hand session required per extraction')
    output=output_path(output)
    partial=output.with_name(output.name+'.partial')
    if partial.exists():raise FileExistsError(partial)
    with h5py.File(partial,'x') as h:
        h.attrs.update(schema='manus_sharpa_hdf5_v1',clock_domain='robot_receive',unit='rad',capture_id=capture_id,
                       serials=json.dumps(config()['serials']),session_id=next(iter(sessions)),complete=False)
        for (side,kind),rows in streams.items():
            rows.sort(key=lambda r:r[0]);ts=np.array([r[0] for r in rows],dtype=np.int64)
            if not len(rows) or np.any(np.diff(ts)<=0):raise ValueError('Missing/duplicate/non-increasing '+str((side,kind)))
            g=h.create_group(f'{kind}/{side}')
            g.create_dataset('timestamp_ns',data=ts)
            g.create_dataset('source_timestamp_ns',data=np.array([r[1] for r in rows],dtype=np.int64))
            g.create_dataset('receive_timestamp_ns',data=np.array([r[2] for r in rows],dtype=np.int64))
            g.create_dataset('position',data=np.array([r[3] for r in rows]))
            g.create_dataset('seq',data=np.array([r[4][1] for r in rows],dtype=np.int64))
            g.attrs['joint_names']=json.dumps(NAMES)
        h.attrs['provenance']=json.dumps({str(Path(p).resolve()):digest(p) for p in [*paths,*journals]})
        h.attrs['complete']=True
    partial.rename(output)
    write_json(str(output)+'.commit.json',{'sha256':digest(output),'capture_id':capture_id})
    return output

def stats(a):
    return dict(min=a.min(0).tolist(),max=a.max(0).tolist(),mean=a.mean(0).tolist(),std=a.std(0).tolist(),count=[len(a)])

def derive(source,hand_hdf5,output,capture_id):
    import pyarrow as pa
    import pyarrow.parquet as pq
    source=Path(source).resolve();output=output_path(output)
    if output.is_relative_to(source):raise ValueError('Derived output must not be inside source episode')
    if any(p.is_symlink() for p in source.rglob('*')):raise ValueError('Source episode cannot contain symlinks')
    completion=json.loads((source/'meta/completion.json').read_text())
    for rel,sha in completion.items():
        p=(source/rel).resolve()
        if not p.is_relative_to(source) or digest(p)!=sha:raise ValueError('Source completion mismatch: '+rel)
    srcinfo=json.loads((source/'meta/info.json').read_text())
    if srcinfo.get('fps')!=20:raise ValueError('Expected 20Hz source')
    stamps=json.loads((source/'meta/source_timestamps.json').read_text())
    grid=np.asarray(stamps['timestamp_ns'],dtype=np.int64)
    if len(grid)<2 or np.any(np.diff(grid)!=50_000_000):raise ValueError('Source grid must be original 20Hz robot nanoseconds')
    files=sorted((source/'data').rglob('*.parquet'))
    if len(files)!=1:raise ValueError('One episode per derived output required')
    table=pq.read_table(files[0]);old=table
    if table.num_rows!=len(grid):raise ValueError('Grid row count')
    for k in ('observation.state','action','validity'):
        if srcinfo['features'][k]['shape']!=[78]:raise ValueError('Expected 78 slots')
        for side,(start,end) in SLOTS.items():
            if srcinfo['features'][k]['names'][start:end]!=[side+'_hand_'+n for n in NAMES]:
                raise ValueError('Hand feature names differ from supplied sample')
    arrays={k:np.asarray(table[k].to_pylist()).copy() for k in ('observation.state','action','validity')}
    report={'schema':'manus_sharpa_derived_v1','capture_id':capture_id,'hdf5_sha256':digest(hand_hdf5),
            'source_completion_sha256':digest(source/'meta/completion.json'),'row_count_unchanged':len(grid),'streams':{}}
    aux={};plan={'schema':'manus_sharpa_replay_v1','clock_domain':'robot_receive','origin_ns':int(grid[0]),
                 'serials':config()['serials'],'joint_names':list(NAMES),'source_hdf5_sha256':digest(hand_hdf5),'hands':{}}
    commit=json.loads(Path(str(hand_hdf5)+'.commit.json').read_text())
    if digest(hand_hdf5)!=commit['sha256']:raise ValueError('Hand HDF5 integrity mismatch')
    with h5py.File(hand_hdf5) as h:
        if not h.attrs['complete'] or h.attrs['clock_domain']!='robot_receive' or h.attrs['capture_id']!=capture_id:
            raise ValueError('Hand HDF5 capture/clock mismatch')
        for side,(start,end) in SLOTS.items():
            for role,column in [('state','observation.state'),('action','action')]:
                g=h[f'{role}/{side}'];ts=g['timestamp_ns'][:];q=g['position'][:]
                if json.loads(g.attrs['joint_names'])!=list(NAMES):raise ValueError('HDF5 joint schema')
                ix,ok,error=match(ts,grid,causal=role=='action',tolerance_ns=int(config()['max_skew_ms']*1e6))
                if not ok.all():raise ValueError(f'{side}/{role}: {int((~ok).sum())} rows out of tolerance; no frames/other slots changed')
                arrays[column][:,start:end]=q[ix]
                report['streams'][f'{role}/{side}']={'max_abs_skew_ns':int(abs(error).max())}
                if role=='action':
                    selected=(ts>grid[0])&(ts<=grid[-1]);times=np.r_[grid[0],ts[selected]]
                    targets=np.vstack([q[ix[0]],q[selected]])
                    plan['hands'][side]={'offset_ns':(times-grid[0]).tolist(),'position':targets.tolist()}
            arrays['validity'][:,start:end]=1
    # Only the hand slices may change; metadata below only edits their entries.
    for k,a in arrays.items():
        index=table.schema.get_field_index(k);field=table.schema.field(index)
        table=table.set_column(index,field,pa.array(a.tolist(),type=field.type))
    nonhand=[i for i in range(78) if not (32<=i<54 or 55<=i<77)]
    for k in arrays:
        if not np.array_equal(np.asarray(old[k].to_pylist())[:,nonhand],arrays[k][:,nonhand]):raise AssertionError('Nonhand slot changed')
    for k in old.column_names:
        if k not in arrays and not old[k].equals(table[k]):raise AssertionError('Nonhand column changed')
    partial=output.with_name(output.name+'.partial')
    shutil.copytree(source,partial,symlinks=False)
    rel=files[0].relative_to(source);pq.write_table(table,partial/rel)
    info=json.loads(json.dumps(srcinfo))
    for side,(start,end) in SLOTS.items():
        for mask in ('state_mask','action_mask'):
            if mask in info:info[mask][start:end]=[True]*22
        if 'missing_data_mask' in info:info['missing_data_mask'][start:end]=[False]*22
        info.get('missing_data',{}).pop(side+'_hand',None)
    if 'action_mask' in info:info['valid_action_dim']=sum(info['action_mask'])
    if 'physical_action_dim' in info and 'state_mask' in info:info['physical_action_dim']=sum(info['state_mask'])
    info['hand_provenance']='hands/alignment_report.json'
    write_json(partial/'meta/info.json',info)
    count_path=partial/'meta/valid_sample_counts.json'
    if count_path.exists():
        counts=json.loads(count_path.read_text())
        for key in arrays:
            if key in counts and isinstance(counts[key],list) and len(counts[key])==78:
                for start,end in SLOTS.values():counts[key][start:end]=[len(grid)]*22
        write_json(count_path,counts)
    # Recompute only hand dimensions in existing numeric statistics.
    def update_stats(obj):
        if isinstance(obj,dict):
            for key,value in obj.items():
                if key in arrays and isinstance(value,dict):
                    newstats=stats(arrays[key])
                    for metric,values in newstats.items():
                        if metric=='count':continue
                        if metric in value and isinstance(value[metric],list) and len(value[metric])==78:
                            for start,end in SLOTS.values():value[metric][start:end]=values[start:end]
                else:update_stats(value)
        elif isinstance(obj,list):
            for value in obj:update_stats(value)
    for name in ('stats.json','episodes_stats.jsonl'):
        path=partial/'meta'/name
        if path.exists():
            if name.endswith('jsonl'):
                rows=[json.loads(l) for l in path.read_text().splitlines()];update_stats(rows)
                path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
            else:
                obj=json.loads(path.read_text());update_stats(obj);write_json(path,obj)
    (partial/'hands').mkdir(exist_ok=True)
    write_json(partial/'hands/alignment_report.json',report)
    write_json(partial/'hands/replay.json',plan)
    write_json(partial/'hands/replay.commit.json',{'sha256':digest(partial/'hands/replay.json')})
    shutil.copyfile(source/'meta/completion.json',partial/'hands/source_completion.json')
    complete={str(p.relative_to(partial)):digest(p) for p in sorted(partial.rglob('*')) if p.is_file() and p!=partial/'meta/completion.json'}
    write_json(partial/'meta/completion.json',complete)
    partial.rename(output)
    return output

def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    a=sub.add_parser('extract');a.add_argument('--mcap',type=Path,action='append',required=True)
    a.add_argument('--journal',type=Path,action='append',required=True);a.add_argument('--capture-id',required=True);a.add_argument('--output',type=Path,required=True)
    a=sub.add_parser('derive');a.add_argument('--source',type=Path,required=True);a.add_argument('--hdf5',type=Path,required=True)
    a.add_argument('--capture-id',required=True);a.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    print(extract(a.mcap,a.journal,a.output,a.capture_id) if a.command=='extract' else derive(a.source,a.hdf5,a.output,a.capture_id))

if __name__=='__main__':main()
