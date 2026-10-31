import json,sys,time
from pathlib import Path
ROOT=Path(__file__).parent
sys.path.insert(0,str(ROOT/'src'))
from unified_replay.integration.coordinator import run
source=json.loads(Path('/home/descfly/apex_n_sharpa/.unified-replay-stage/simulation-final.log').read_text().splitlines()[-1])
plan=json.loads((Path(source['output'])/'plan.json').read_text())
scenario=sys.argv[1] if len(sys.argv)>1 else 'normal'
out=Path('/mnt/apex-data/apex_n_sharpa/data/replay_runs')/source['task']/'episode_00000042'/('controller-'+str(time.time_ns()))
r=run(plan,scenario,out)
summary={k:v for k,v in r.items() if k not in ('executors','ready')};summary['output']=str(out)
(ROOT/('integration-'+scenario+'-result.json')).write_text(json.dumps(summary,indent=2));print(json.dumps(summary))
raise SystemExit(0 if r['experiment_passed'] else 1)
