"""Import and convert certified unified episodes; never starts recording/hardware."""
import argparse,json,re,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'))
from read_raw_data.main import DATA,ensure_storage,safe,save

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',required=True)
    p.add_argument('--episode',type=int)
    p.add_argument('--apex-source',default='/home/user/gento_replay_bags')
    p.add_argument('--ssh',default='user@6.6.7.100')
    p.add_argument('--local-source',action='store_true')
    p.add_argument('--skip-import',action='store_true')
    p.add_argument('--apex-config',type=Path,help='Explicit unified78 config; copied immutably per task')
    p.add_argument('--lerobot-config',type=Path,help='Explicit unified78 config; copied immutably per task')
    p.add_argument('--allow-diagnostic',action='store_true',help='Synthetic validation only; propagated into every output')
    a=p.parse_args()
    if not re.fullmatch(r'[\w-]+',a.task):p.error('Invalid task name')
    if a.episode is not None and not 0<=a.episode<100_000_000:p.error('Invalid episode index')
    ensure_storage()
    selection=['--episode',str(a.episode)] if a.episode is not None else []
    def run(step,args):
        subprocess.run([sys.executable,str(ROOT/'src'/step/'main.py'),*args],check=True)
    configs={}
    for step in ('apex','lerobot'):
        source=getattr(a,step+'_config') or safe(DATA/'config'/f'{step}_unified78.json')
        value=json.loads(source.read_text())
        if value.get('profile')!='tianji_unified78':raise ValueError('Only unified78 configurations are allowed')
        value['allow_diagnostic']=a.allow_diagnostic
        path=safe(DATA/'config/unified_runs'/a.task/f'{step}.json')
        if path.exists() and json.loads(path.read_text())!=value:raise ValueError('Task configuration changed; use a new task instead of overwriting outputs')
        if not path.exists():save(path,value)
        configs[step]=path
    if not a.skip_import:
        run('read_raw_data',['--unified','--task',a.task,'--apex-source',a.apex_source,'--ssh',a.ssh,*selection,
                             *(['--local-source'] if a.local_source else []),*(['--allow-diagnostic'] if a.allow_diagnostic else [])])
    run('apex2hdf5',['--task',a.task,'--config',str(configs['apex']),*selection])
    run('hdf52lerobot',['--task',a.task,'--config',str(configs['lerobot']),*selection])
    print(json.dumps({'task':a.task,'episode_index':a.episode,'profile':'tianji_unified78',
        'diagnostic':a.allow_diagnostic,'raw_root':str(DATA/'raw_data/apex_data'/a.task),
        'hdf5_root':str(DATA/'hdf5'/a.task),'lerobot_root':str(DATA/'lerobot'/a.task)},ensure_ascii=False))
if __name__=='__main__':main()
