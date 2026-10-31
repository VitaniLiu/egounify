"""Independent read-only validation of regenerated HDF5/LeRobot outputs."""
from pathlib import Path
import json, hashlib, io
import numpy as np
import h5py
import pyarrow.parquet as pq
from PIL import Image

ROOT = Path('/mnt/apex-data/apex_n_sharpa/data')
TASK = 'robot_import_20261024_egoav_v4'
def read(p): return json.loads(p.read_text())
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
    return h.hexdigest()
def pose(v):
    x,y,z,w=(v[:,3:]/np.linalg.norm(v[:,3:],axis=1)[:,None]).T
    r=np.empty((len(v),3,3))
    r[:,0,0]=1-2*(y*y+z*z);r[:,0,1]=2*(x*y-z*w);r[:,0,2]=2*(x*z+y*w)
    r[:,1,0]=2*(x*y+z*w);r[:,1,1]=1-2*(x*x+z*z);r[:,1,2]=2*(y*z-x*w)
    r[:,2,0]=2*(x*z-y*w);r[:,2,1]=2*(y*z+x*w);r[:,2,2]=1-2*(x*x+y*y)
    return np.concatenate((v[:,:3],r[:,:,0],r[:,:,1]),axis=1)
result=[]
for i in range(3):
    ep=f'episode_{i:08d}'
    hp=ROOT/'hdf5'/TASK/f'hdf5_{i:08d}.hdf5'
    assert sha(hp)==read(hp.with_suffix('.commit.json'))['sha256']
    dest=ROOT/'lerobot'/TASK/ep;meta=dest/'meta'
    completion=read(meta/'completion.json')
    assert completion == {str(p.relative_to(dest)):sha(p) for p in dest.rglob('*')
                          if p.is_file() and p != meta/'completion.json'}
    info=read(meta/'info.json'); stamps=read(meta/'source_timestamps.json')
    report=read(meta/'cleaning_report.json')
    table=pq.read_table(dest/'data/chunk-000/episode_000000.parquet')
    a={k:np.array(table[k].to_pylist()) for k in ('observation.state','action','validity')}
    n=info['total_frames'];ts=np.array(stamps['timestamp_ns'],dtype=np.int64)
    assert n==len(table) and a['action'].shape==(n,27)
    assert np.all(np.diff(ts)==50000000) and np.all(a['validity']==1)
    assert info['coordinate_frame']=='chunk_first_frame_camera'
    assert report['retained_max_spread_ns']<=50000000
    anchors=read(meta/'chunk_frames.json')['chunks']; last=0;error=0.
    with h5py.File(hp,'r') as h:
        assert h.attrs['complete']
        assert h['actions/body_controller'].attrs['unit']=='m,rad,rad'
        assert h['actions/head_controller'].attrs['unit']=='rad,rad'
        for key,entry in stamps['aligned_sources'].items():
            st=np.array(entry['timestamp_ns'],dtype=np.int64)
            if key.startswith('actions/'): assert np.all(st<=ts),key
            if key in h:
                np.testing.assert_array_equal(st,h[key]['timestamp_ns'][:][entry['source_index']])
        for role,col in [('observations','observation.state'),('actions','action')]:
            assert np.isfinite(a[col]).all()
            base=np.empty_like(a[col],dtype=float)
            for entry in anchors:
                start,end=entry['start_frame'],entry['stop_frame']
                t=np.array(entry['base_from_chunk_camera']);r=t[:3,:3];p=t[:3,3]
                for offset in (0,9,18):
                    v=a[col][start:end,offset:offset+9]
                    base[start:end,offset:offset+9]=np.concatenate((v[:,:3]@r.T+p,v[:,3:6]@r.T,v[:,6:9]@r.T),axis=1)
            for side,offset in [('left',0),('right',9),('active_camera',18)]:
                key=f'{role}/{side}';ix=np.array(stamps['aligned_sources'][key]['source_index'])
                source=pose(h[key]['eef'][:][ix])
                error=max(error,float(np.abs(base[:,offset:offset+9]-source).max()))
                np.testing.assert_allclose(base[:,offset:offset+9],source,atol=2e-6,rtol=0)
        for col,path in [('observation.body_joints','observations/body_joints'),('action.body_controller','actions/body_controller'),('action.head_controller','actions/head_controller')]:
            ix=np.array(stamps['aligned_sources'][path]['source_index'])
            np.testing.assert_allclose(np.array(table[col].to_pylist()),h[path]['value'][:][ix],atol=1e-7,rtol=1e-6)
            assert info['features'][col]['unit']==h[path].attrs['unit']
    for entry in anchors:
        assert entry['start_frame']==last
        last=entry['stop_frame']
        np.testing.assert_allclose(a['observation.state'][entry['start_frame'],18:27],[0,0,0,1,0,0,0,1,0],atol=1e-6)
    assert last==n
    for key in table.column_names:
        if key.startswith('observation.images.'):
            for row in table[key].to_pylist():
                with Image.open(io.BytesIO(row['bytes'])) as im: im.verify()
    result.append({'episode':ep,'frames':n,'chunks':len(anchors),'max_roundtrip_error':error,'max_spread_ns':report['retained_max_spread_ns'],'hdf5_bytes':hp.stat().st_size,'lerobot_dimension':27})
out=ROOT/'inspection'/TASK/'reconversion_validation.json'
out.write_text(json.dumps({'episodes':result,'total_frames':sum(x['frames'] for x in result),'full_78_export':'blocked_by_uncollected_Sharpa','no_fabricated_hand_data':True},indent=2)+'\n')
print(out); print(json.dumps(result,indent=2))

