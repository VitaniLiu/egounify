"""Package reviewed raw-derived control metadata into a separate LeRobot copy.
Runtime replay subsequently needs only this copy, never raw recordings.
"""
import json,hashlib,shutil
from pathlib import Path
ROOT=Path('/mnt/apex-data/apex_n_sharpa/data')
source=ROOT/'lerobot/robot_import_20261024_egoav_v4/episode_00000002'
out=ROOT/'lerobot/robot_import_20261024_replay_v1/episode_00000002'
audit=ROOT/'inspection/incremental_semantics_20261025'
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads((source/'meta/completion.json').read_text())
assert manifest=={str(p.relative_to(source)):digest(p) for p in source.rglob('*') if p.is_file() and p!=source/'meta/completion.json'}
comparison=json.loads((audit/'raw_comparison.json').read_text())
gate=json.loads((audit/'recorded_ik_availability.json').read_text())
times=json.loads((source/'meta/source_timestamps.json').read_text())['timestamp_ns']
assert times==gate['timestamp_ns']
mapping={}
for side in ('left','right'):
    details=comparison['sides'][side];m=details['time_bounded_match']
    assert m['position_error_m']['max']<.005 and m['rotation_error_rad']['max']<.03
    assert details['lerobot_raw_source_check']['max_matrix_error']<2e-6
    mapping[side]={'translation_offset':m['translation_offset_m'],'right_rotation_offset':m['rotation_right_offset']}
contract={'schema':'skye_recorded_ik_mapping_v1','source_completion_sha256':digest(source/'meta/completion.json'),
    'mapping':mapping,'timestamp_ns':times,'available':gate['available'],
    'ik_log_timestamp_ns':gate['ik_log_timestamp_ns'],
    'mapping_formula':'p_ik=p_action+c; R_ik=R_action@C',
    'input_frame':'base_link','output_frame':'base_link',
    'tracking_to_base_verified_identity_tolerance':2e-6,
    'active_frames':comparison['lerobot_grid_ik_active_frames'],
    'raw_evidence_sha256':digest(audit/'raw_comparison.json'),
    'limitations':['One left-arm raw pair differs by 3.00mm/0.0245rad; within reviewed 5mm/0.03rad tolerance',
        '20Hz causally sampled replay is not original high-rate trajectory',
        'Body/head use recorded joint controller targets via FK; hands/grippers/chassis excluded'],
    'native_wire':'complete pose after upstream teleop mapping; not direct delta_matrix',
    'body_head_policy':'recorded_controller_FK'}
if out.exists():raise RuntimeError('Refusing to overwrite packaged data')
out.parent.mkdir(parents=True,exist_ok=True);tmp=out.with_name('.'+out.name+'.partial')
try:
    shutil.copytree(source,tmp)
    (tmp/'meta/replay_contract.json').write_text(json.dumps(contract,indent=2)+'\n')
    (tmp/'meta/completion.json').write_text(json.dumps({str(p.relative_to(tmp)):digest(p) for p in tmp.rglob('*') if p.is_file() and p!=tmp/'meta/completion.json'},indent=2)+'\n')
    tmp.rename(out)
except BaseException:
    if tmp.exists():shutil.rmtree(tmp)
    raise
print(json.dumps({'output':str(out),'active_frames':contract['active_frames']}))
