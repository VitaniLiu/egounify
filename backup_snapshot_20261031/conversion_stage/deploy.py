"""Deploy only validated files after checking every source and baseline hash."""
import hashlib,json,os,shutil,time
from pathlib import Path
ROOT=Path('/home/descfly/apex_n_sharpa');STAGE=ROOT/'.unified78-stage'
rows=json.loads((STAGE/'deployment-plan.json').read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for r in rows:
 p=Path(r['target']);s=STAGE/r['source']
 assert sha(s)==r['after'],('Stage changed',str(s))
 actual=sha(p) if p.exists() else None
 assert actual==r['before'],('Deployment baseline changed',str(p),actual,r['before'])
backup=ROOT/'deployment'/('unified78-'+str(time.time_ns()));backup.mkdir()
report={'state':'DEPLOYING','backup_root':str(backup),'files':[],'services_restarted':False}
for n,r in enumerate(rows):
 p=Path(r['target']);old=backup/f'{n:02d}.before'
 if p.exists():shutil.copy2(p,old)
 else:old=None
 p.parent.mkdir(parents=True,exist_ok=True);tmp=p.with_name(p.name+'.unified78-new')
 shutil.copyfile(STAGE/r['source'],tmp);tmp.chmod(r['mode']);os.replace(tmp,p)
 assert sha(p)==r['after']
 report['files'].append(dict(r,backup=str(old) if old else None))
 (backup/'manifest.json').write_text(json.dumps(report,indent=2))
report['state']='DEPLOYED';(backup/'manifest.json').write_text(json.dumps(report,indent=2))
(STAGE/'deployment-result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
