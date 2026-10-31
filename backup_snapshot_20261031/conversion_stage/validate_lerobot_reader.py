"""Read synthetic episode 42 through the pinned upstream loader and temporal adapter."""
import json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'src'))
from unified_lerobot import load_episode
import torch

def main():
 result=json.loads((ROOT/'validation-result.json').read_text());root=result['lerobot']
 try:load_episode(root)
 except ValueError as e:assert 'Diagnostic' in str(e)
 else:raise AssertionError('Diagnostic training not rejected')
 ds=load_episode(root,delta_timestamps={'action':[-.05,0,.05]},allow_diagnostic=True)
 assert len(ds)==result['frames'] and ds.num_episodes==1
 first,last=ds[0],ds[len(ds)-1]
 for row in (first,last):
  assert row['episode_index'].item()==42
  assert row['observation.state'].shape==(78,) and row['action'].shape==(3,78)
  assert row['validity'].sum().item()==76
 assert first['action_is_pad'].tolist()==[True,False,False]
 assert last['action_is_pad'].tolist()==[False,False,True]
 # Spawn validates that the adapter class can be imported by training workers.
 loader=torch.utils.data.DataLoader(ds,batch_size=2,num_workers=1,multiprocessing_context='spawn')
 batch=next(iter(loader));assert batch['episode_index'].tolist()==[42,42]
 report={'state':'PASS','lerobot_package':'0.3.3','format':'v2.1','episode_index':42,'frames':len(ds),
         'checks':['upstream image/state/action read','temporal padding at both boundaries','original episode ID retained','spawn DataLoader worker','diagnostic rejected by default']}
 (ROOT/'reader-validation-result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
if __name__=='__main__':main()
