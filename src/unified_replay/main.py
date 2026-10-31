"""LeRobot-only offline checks and isolated four-process mocks. Hardware unavailable."""
import argparse
import json
import os
from pathlib import Path
import re
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src'))
from unified_replay.contract import build_plan, verify_episode, file_hash
from unified_replay.simulator import SCENARIOS, simulate


from unified_replay.isolation import install


def save(path, value):
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    os.replace(temporary,path)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',required=True)
    p.add_argument('--episode',type=int,required=True)
    p.add_argument('--episode-dir',type=Path,help='Explicit LeRobot episode directory for offline tests')
    p.add_argument('--output-dir',type=Path,help='New report directory; existing directories are never overwritten')
    mode=p.add_mutually_exclusive_group()
    mode.add_argument('--plan-only',action='store_true',help='Default: validate and generate offline plan')
    mode.add_argument('--simulate',action='store_true',help='Run local mock subprocesses, no ROS or SDK')
    p.add_argument('--scenario',choices=(*SCENARIOS,'all'),default='normal')
    p.add_argument('--allow-diagnostic',action='store_true',help='Explicitly permit diagnostic samples for offline/simulation only')
    p.add_argument('--execute',action='store_true',help='Unavailable: always rejected before any work')
    a=p.parse_args()
    if a.execute:p.error('Hardware execution is not implemented or authorized; use --plan-only or --simulate')
    if not re.fullmatch(r'[\w-]+',a.task) or not 0<=a.episode<100_000_000:p.error('Invalid task/episode')
    if not a.simulate and a.scenario!='normal':p.error('--scenario requires --simulate')
    install()
    data=Path('/mnt/apex-data/apex_n_sharpa/data')
    source=(a.episode_dir or data/'lerobot'/a.task/f'episode_{a.episode:08d}').absolute()
    out=(a.output_dir or data/'replay_runs'/a.task/f'episode_{a.episode:08d}'/('offline-'+str(time.time_ns())+'-'+uuid.uuid4().hex[:8])).absolute()
    if out.resolve().is_relative_to(source.resolve()):p.error('Reports must be outside the source episode')
    if out.exists():p.error('Output directory exists; refusing overwrite')
    out.mkdir(parents=True)
    report={'schema':'unified_replay_validation_v1','task':a.task,'episode_index':a.episode,
            'source':str(source),'output':str(out),'hardware_initialized':False,'hardware_commands_sent':0,
            'scope':'offline_and_mock_only','physical_execution_supported':False}
    try:
        plan=build_plan(source,a.episode,a.allow_diagnostic)
        save(out/'plan.json',plan)
        report.update(state='OFFLINE_PASS',plan_sha256=plan['plan_sha256'],capture_id=plan['capture_id'],
            diagnostic=plan['diagnostic'],duration_ns=plan['duration_ns'],
            events={k:len(v) for k,v in plan['channels'].items()},
            implementation={str(f.relative_to(ROOT)):file_hash(f) for f in (Path(__file__).parent).glob('*.py')},
            limitation=plan['unverified'])
        if a.simulate:
            scenarios=SCENARIOS if a.scenario=='all' else [a.scenario]
            results=[]
            for scenario in scenarios:
                result=simulate(plan,scenario)
                save(out/(scenario+'.json'),result)
                summary={k:result[k] for k in ('scenario','status','experiment_passed','trigger','mock_stop_confirmed',
                         'missing_stop_ack','start_spread_ns','stop_latency_ns','max_lateness_ns','mock_commands_emitted')}
                results.append(summary)
                print(json.dumps(summary),flush=True)
                report.update(scenarios=results,state='SIMULATION_RUNNING')
                save(out/'report.json',report)
            report['state']='SIMULATION_PASS' if all(r['experiment_passed'] for r in results) else 'SIMULATION_FAILED'
        if verify_episode(source)!=plan['source_completion_sha256']:
            raise ValueError('LeRobot source changed during validation')
        report['source_unchanged']=True
    except Exception as exc:
        report.update(state='FAILED',error=str(exc))
    save(out/'report.json',report)
    print(json.dumps(report,ensure_ascii=False),flush=True)
    return 0 if report['state'] in ('OFFLINE_PASS','SIMULATION_PASS') else 1


if __name__=='__main__':
    raise SystemExit(main())
