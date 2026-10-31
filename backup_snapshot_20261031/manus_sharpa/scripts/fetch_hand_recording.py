"""Fetch an already finished hand-only receiver journal. Never starts recording."""
import argparse
import re
import subprocess
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from hands_common import ROOT,config
p=argparse.ArgumentParser(description=__doc__);p.add_argument('session');a=p.parse_args()
if not re.fullmatch('[0-9a-f]{32}',a.session):p.error('Expected 32-character session ID')
dest=ROOT/'data/journals';dest.mkdir(parents=True,exist_ok=True)
target=dest/(a.session+'.jsonl')
if target.exists():raise FileExistsError(target)
remote='/home/user/manus_sharpa/recordings/'+a.session+'.jsonl'
# A receiver lock must be free, otherwise the journal can still change.
check='flock -n /home/user/manus_sharpa/recordings/.receiver.lock stat -c %s '+remote
before=subprocess.check_output(['ssh','-o','BatchMode=yes',config()['robot_ssh'],check])
subprocess.run(['scp',config()['robot_ssh']+':'+remote,str(target)],check=True)
after=subprocess.check_output(['ssh','-o','BatchMode=yes',config()['robot_ssh'],check])
if before!=after or int(after)!=target.stat().st_size:raise RuntimeError('Journal changed during transfer; reject this copy')
print(target)
