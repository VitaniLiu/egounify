"""End-to-end diagnostic pipeline on a real native-writer synthetic capture."""
import json,subprocess,sys,time,hashlib
from pathlib import Path
import numpy as np,h5py,pyarrow.parquet as pq
ROOT=Path(__file__).parent
DATA=Path('/mnt/apex-data/apex_n_sharpa/data')
record=json.loads(subprocess.check_output(['ssh','user@6.6.7.100','tail -1 /home/user/apex_record_sync/unified78-stage/conversion-raw-test.log'],text=True))
source=str(Path(record['complete_bag']).parent)
task='unified78_validation_'+str(time.time_ns());cfgdir=DATA/'config/unified_validation'/task;cfgdir.mkdir(parents=True)
configs={}
for kind in ('apex','lerobot'):
 c=json.loads((ROOT/'config'/f'{kind}_unified78.json').read_text());c['allow_diagnostic']=True
 if kind=='apex':
  for spec in c['streams'].values():
   if spec['kind']=='image':spec.pop('crop',None)
 cfg=cfgdir/(kind+'.json');cfg.write_text(json.dumps(c));configs[kind]=cfg
log=[]
def command(step,args,success=True):
 r=subprocess.run([sys.executable,str(ROOT/'src'/step/'main.py'),*args],text=True,capture_output=True)
 log.append({'step':step,'args':args,'returncode':r.returncode,'stdout':r.stdout[-2000:],'stderr':r.stderr[-4000:]})
 (ROOT/'validation-log.json').write_text(json.dumps(log,indent=2))
 if success and r.returncode:raise RuntimeError(step+': '+r.stderr)
 if not success and r.returncode==0:raise AssertionError('Expected rejection '+step)
 return r
base=['--unified','--task',task,'--episode','42','--apex-source',source]
reject=command('read_raw_data',base,False);assert 'Diagnostic' in reject.stderr
command('unified_pipeline',['--task',task,'--episode','42','--apex-source',source,'--allow-diagnostic',
                          '--apex-config',str(configs['apex']),'--lerobot-config',str(configs['lerobot'])])
raw=DATA/'raw_data/apex_data'/task/'episode_00000042'
hdf=DATA/'hdf5'/task/'hdf5_00000042.hdf5'
lr=DATA/'lerobot'/task/'episode_00000042'
table=pq.read_table(lr/'data/chunk-000/episode_000042.parquet')
state=np.array(table['observation.state'].to_pylist());action=np.array(table['action'].to_pylist());valid=np.array(table['validity'].to_pylist())
assert state.shape[1]==78 and action.shape==state.shape
assert np.all(valid[:,[54,77]]==0) and np.all(valid.sum(1)==76)
assert np.all(state[:,[54,77]]==0) and np.all(action[:,[54,77]]==0)
np.testing.assert_allclose(state[:,27:32],np.tile([.2,.1,.1,.1,.1],(len(state),1)))
np.testing.assert_allclose(action[:,27:32],np.tile([.21,.12,.13,.14,.15],(len(state),1)))
for lo,hi in ((32,54),(55,77)):
 np.testing.assert_allclose(state[:,lo:hi],.01);np.testing.assert_allclose(action[:,lo:hi],.03)
assert set(table['episode_index'].to_pylist())=={42}
info=json.loads((lr/'meta/info.json').read_text());assert info['source_episode_index']==42 and info['diagnostic']
stamps=json.loads((lr/'meta/source_timestamps.json').read_text());grid=np.array(stamps['timestamp_ns'],dtype='i8')
assert np.all(np.diff(grid)==50_000_000)
for key,e in stamps['aligned_sources'].items():
 if key.startswith(('action/','actions/')):assert np.all(np.array(e['timestamp_ns'],dtype='i8')<=grid)
plan=json.loads((lr/'meta/unified_replay.json').read_text());assert plan['episode_index']==42
with h5py.File(hdf,'r') as h:
 assert h.attrs['episode_index']==42 and h.attrs['capture_id']==plan['capture_id']
 for key,e in plan['streams'].items():
  np.testing.assert_array_equal(h[key+'/timestamp_ns'][:],e['timestamp_ns'])
  np.testing.assert_allclose(h[key+'/value'][:],e['value'])
  np.testing.assert_array_equal(np.array(e['timestamp_ns'],dtype='i8')-plan['origin_ns'],e['offset_ns'])
 assert 'evidence/sharpa/context_json' in h
 assert 'provenance/models' in h and len(h['provenance/models'])>0
 hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in lr.rglob('*') if p.is_file()}
# Idempotence, no re-numbering/overwrite.
command('unified_pipeline',['--task',task,'--episode','42','--apex-source',source,'--allow-diagnostic',
                          '--apex-config',str(configs['apex']),'--lerobot-config',str(configs['lerobot'])])
assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==sha for p,sha in hashes.items())
result={'state':'PASS','task':task,'episode_index':42,'frames':len(state),'dimensions':78,'valid_dimensions':76,
        'raw':str(raw),'hdf5':str(hdf),'lerobot':str(lr),'diagnostic':True,'hardware_initialized':False,
        'checks':['unified entrypoint end to end','native ROS MCAP import','same episode 42 at every stage','body5 measured/target values','left/right hand feedback vs accepted action',
        'gripper-only mask','20Hz integer grid','causal action alignment','full-rate replay stream equality','idempotent rerun','diagnostic default rejection']}
(ROOT/'validation-result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
