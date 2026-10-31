import hashlib,json,os,shutil,subprocess,time
from pathlib import Path
ROOT=Path('/home/user/apex_record_sync');STAGE=ROOT/'unified78-stage'
plan=json.loads((STAGE/'production-deploy-plan.json').read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(ROOT/'contract.py')==plan['before'],'Robot contract changed since review'
assert sha(STAGE/'contract.py')==plan['after'],'Robot staged contract mismatch'
assert not subprocess.run(['pgrep','-f','[p]ython.*record_node.py'],capture_output=True,text=True).stdout,'Coordinator currently active'
cfg=json.loads((ROOT/'config.json').read_text());state=Path(cfg['state_root']);registry=state/'episode_registry.json'
assert not registry.exists(),'Allocator already exists; inspect before initialization'
status_file=state/'record_sync_status.json'
if status_file.exists():assert not json.loads(status_file.read_text()).get('active'),'Capture is active'
backup=ROOT/'backups'/('unified78-'+str(time.time_ns()));backup.mkdir(parents=True)
shutil.copy2(ROOT/'contract.py',backup/'contract.py')
tmp=ROOT/'contract.py.unified78-new';shutil.copy2(STAGE/'contract.py',tmp);os.replace(tmp,ROOT/'contract.py')
import sys
sys.path.insert(0,str(ROOT))
from contract import atomic_json
# Seed after the highest existing legacy imported ID (0,1,2), while future
# allocations also scan native numbered folders and reserve monotonically.
atomic_json(registry,{'schema':'apex_episode_allocator_v1','next_index':plan['next_index'],
                     'initialization_reason':'Retain legacy imported episode IDs 0..2; no changes to legacy data'})
result={'state':'DEPLOYED','target':str(ROOT/'contract.py'),'before':plan['before'],'after':sha(ROOT/'contract.py'),
        'backup':str(backup/'contract.py'),'registry':str(registry),'next_index':plan['next_index'],
        'services_restarted':False,'effective':'next normal Teleop launch'}
(backup/'manifest.json').write_text(json.dumps(result,indent=2));(STAGE/'production-deployment-result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
