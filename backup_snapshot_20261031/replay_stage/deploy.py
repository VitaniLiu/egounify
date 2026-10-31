"""Install only new offline replay files, after hash checking tested stage."""
from pathlib import Path
import hashlib,json,os,shutil,time
root=Path('/home/descfly/apex_n_sharpa');stage=root/'.unified-replay-stage'
plan=json.loads((stage/'deployment-plan.json').read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for row in plan:
 assert not (root/row['source']).exists(),'Destination already exists: '+row['source']
 assert sha(stage/row['source'])==row['sha256'],'Stage differs from tested local version'
backup=root/'deployment'/('unified-replay-offline-'+str(time.time_ns()));backup.mkdir()
result={'state':'DEPLOYING','backup_root':str(backup),'files':[],
        'services_restarted':False,'robot_files_modified':False,'hardware_execution_supported':False}
for row in plan:
 target=root/row['source'];target.parent.mkdir(parents=True,exist_ok=True)
 temp=target.with_name(target.name+'.new-offline-replay');shutil.copyfile(stage/row['source'],temp)
 temp.chmod(row['mode']);os.replace(temp,target);assert sha(target)==row['sha256']
 result['files'].append(dict(row,target=str(target),rollback='Remove this newly added file; preserve datasets and reports'))
 (backup/'manifest.json').write_text(json.dumps(result,indent=2))
result['state']='DEPLOYED';(backup/'manifest.json').write_text(json.dumps(result,indent=2))
(stage/'deployment-result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
