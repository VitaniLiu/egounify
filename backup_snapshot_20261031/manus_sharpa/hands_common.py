"""Hand-only schema and time matching; never imports a hardware SDK."""
import hashlib
import json
from pathlib import Path
import numpy as np
import subprocess

ROOT = Path(__file__).resolve().parent
NAMES = tuple('thumb_CMC_FE thumb_CMC_AA thumb_MCP_FE thumb_MCP_AA thumb_IP index_MCP_FE index_MCP_AA index_PIP index_DIP middle_MCP_FE middle_MCP_AA middle_PIP middle_DIP ring_MCP_FE ring_MCP_AA ring_PIP ring_DIP pinky_CMC pinky_MCP_FE pinky_MCP_AA pinky_PIP pinky_DIP'.split())
SLOTS = {'left': (32, 54), 'right': (55, 77)}

def ensure_sdk_ports_free():
    """Sharpa apps do not share our flock; reject their occupied UDP ports."""
    result=subprocess.run(['ss','-H','-uapn'],capture_output=True,text=True,check=True)
    occupied=[]
    for line in result.stdout.splitlines():
        fields=line.split()
        if len(fields)>4 and fields[3].rsplit(':',1)[-1] in {'50000','50005','50010','50015','54321'}:
            occupied.append(line.strip())
    if occupied:
        raise RuntimeError('Sharpa SDK ports already occupied. Exit Sharpa Pilot/pilot_sdk and other hand controllers first. '+ '; '.join(occupied))

def config():
    c=json.loads((ROOT / 'config/hands.json').read_text())
    if len(set(c['serials'].values()))!=2 or set(c['serials'])!={'left','right'}:
        raise ValueError('Two explicitly bound, distinct hands required')
    for key,upper in [('command_timeout_s',.2),('max_input_age_s',.25),('max_skew_ms',50)]:
        if not 0<float(c[key])<=upper:raise ValueError('Hand protection limit invalid: '+key)
    if c['local_domain']!=198 or c['robot_domain']!=29 or c['clock_domain']!='robot_receive':
        raise ValueError('This hand adapter is pinned to isolated domain198 and robot recording domain29')
    return c

def digest(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''): h.update(chunk)
    return h.hexdigest()

def normalize(names, positions, side):
    if len(names) != 22 or len(set(names)) != 22 or len(positions) != 22:
        raise ValueError('22 unique named joints required')
    names = [n.removeprefix(side + '_hand_').removeprefix(side + '_') for n in names]
    if len(set(names)) != 22 or set(names) != set(NAMES):
        raise ValueError('Joint names differ from reference schema')
    q = np.asarray([positions[names.index(n)] for n in NAMES], dtype=np.float64)
    if not np.isfinite(q).all(): raise ValueError('Nonfinite hand position')
    return q

def match(times, targets, causal=False, tolerance_ns=50_000_000):
    times = np.asarray(times, dtype=np.int64)
    targets = np.asarray(targets, dtype=np.int64)
    if len(times) == 0 or np.any(np.diff(times) <= 0):
        raise ValueError('Source timestamps must strictly increase')
    r = np.searchsorted(times, targets, side='right') - 1
    valid = r >= 0
    if not causal:
        hi = np.clip(r + 1, 0, len(times)-1)
        lo = np.clip(r, 0, len(times)-1)
        r = np.where(abs(times[lo]-targets) <= abs(times[hi]-targets), lo, hi)
        valid = np.ones(len(targets), dtype=bool)
    r = np.clip(r, 0, len(times)-1)
    error = times[r] - targets
    valid &= abs(error) <= tolerance_ns
    return r, valid, error
