import json,subprocess,time
from pathlib import Path
stage=Path('/home/descfly/apex_n_sharpa/.unified78-stage')
r=json.loads((stage/'validation-result.json').read_text());task=r['task']
source=json.loads(subprocess.check_output(['ssh','user@6.6.7.100','tail -1 /home/user/apex_record_sync/unified78-stage/conversion-raw-test.log'],text=True))['complete_bag']
cfg=Path('/mnt/apex-data/apex_n_sharpa/data/config/unified_validation')/task
cmd=['./convert_unified','--task',task,'--episode','42','--apex-source',str(Path(source).parent),'--apex-config',str(cfg/'apex.json'),'--lerobot-config',str(cfg/'lerobot.json'),'--allow-diagnostic']
p=subprocess.run(cmd,capture_output=True,text=True,cwd='/home/descfly/apex_n_sharpa');(stage/'installed-entry.log').write_text(p.stdout+p.stderr)
assert p.returncode==0,p.stderr
# Read clocks only. Round-trip bound is deliberately conservative.
t0=time.time_ns();robot=int(subprocess.check_output(['ssh','user@6.6.7.100','date +%s%N'],text=True));t1=time.time_ns()
clock={'robot_minus_av_estimate_ns':robot-(t0+t1)//2,'round_trip_ns':t1-t0,'uncertainty_bound_ns':(t1-t0)//2,'clock_changed':False}
report={'state':'PASS','entrypoint':'/home/descfly/apex_n_sharpa/convert_unified','task':task,'clock_observation':clock}
(stage/'installed-validation-result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
