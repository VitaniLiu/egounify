"""Contract failure tests: no hardware and no production dataset mutations."""
import json,sys,hashlib,copy,importlib.util
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
import unified_capture as u

def write(path,value):path.write_text(json.dumps(value))

def fixture(root,diagnostic=False):
    (root/'raw.mcap').write_bytes(b'synthetic raw; parsing tested separately')
    m=dict(schema='apex_capture_manifest_v1',state='COMPLETE',diagnostic=diagnostic,
        capture_id='a'*32,episode_index=42,episode_name='episode_00000042',clock_domain='robot_receive',
        audit={'issues':[]},start_ack_ns=100,stop_request_ns=200,
        raw_files={'raw.mcap':{'size':len((root/'raw.mcap').read_bytes()),'sha256':u.digest(root/'raw.mcap')}})
    finish(root,m);return m

def finish(root,m):
    write(root/'capture_manifest.json',m)
    write(root/'capture_completion.json',dict(schema='apex_capture_completion_v1',state=m['state'],diagnostic=m['diagnostic'],
         capture_id=m['capture_id'],episode_index=m['episode_index'],manifest_sha256=u.digest(root/'capture_manifest.json')))

def test_explicit_episode_not_folder_order(tmp_path):
    fixture(tmp_path)
    assert u.verify_raw(tmp_path,expected=42)['episode_index']==42
    with pytest.raises(ValueError,match='Episode identity'):u.verify_raw(tmp_path,expected=0)

@pytest.mark.parametrize('value',[-1,100_000_000,True,None,'episode_42','42'])
def test_invalid_episode_id(value):
    with pytest.raises(ValueError):u.episode_index(value)

def test_diagnostic_needs_explicit_permission(tmp_path):
    fixture(tmp_path,True)
    with pytest.raises(ValueError,match='Diagnostic'):u.verify_raw(tmp_path)
    assert u.verify_raw(tmp_path,True)['diagnostic']

def test_raw_tamper_and_extra_file_rejected(tmp_path):
    fixture(tmp_path);(tmp_path/'raw.mcap').write_bytes(b'tampered')
    with pytest.raises(ValueError,match='hash/size'):u.verify_raw(tmp_path)
    fixture(tmp_path);(tmp_path/'unlisted.bin').write_bytes(b'x')
    with pytest.raises(ValueError,match='inventory'):u.verify_raw(tmp_path)

@pytest.mark.parametrize('change',[{'state':'FAILED'},{'clock_domain':'host_clock'},{'episode_name':'episode_00000041'},{'audit':{'issues':['dropout']}}])
def test_failed_or_conflicting_capture_rejected(tmp_path,change):
    m=fixture(tmp_path);m.update(change);finish(tmp_path,m)
    with pytest.raises(ValueError):u.verify_raw(tmp_path)

def test_completion_hash_and_index_rejected(tmp_path):
    fixture(tmp_path);p=tmp_path/'capture_completion.json';c=json.loads(p.read_text());c['episode_index']=41;write(p,c)
    with pytest.raises(ValueError,match='Completion episode'):u.verify_raw(tmp_path)
    c['episode_index']=42;c['manifest_sha256']='0'*64;write(p,c)
    with pytest.raises(ValueError,match='manifest hash'):u.verify_raw(tmp_path)

def test_unified_profile_forbids_masking_hands():
    spec=importlib.util.spec_from_file_location('lr_profile',ROOT/'src/hdf52lerobot/main.py');lr=importlib.util.module_from_spec(spec);spec.loader.exec_module(lr)
    with pytest.raises(ValueError,match='exactly two'):
        lr.build(None,None,None,{'profile':u.PROFILE,'uncollected_fields':['left_hand','right_hand']},
                 {'features':{'observation.state':{'names':[]}}},'00000042')

def test_episode_allocator_never_reuses_reserved_id(tmp_path):
    sys.path.insert(0,str(ROOT/'recording'))
    from contract import reserve_episode
    root=tmp_path/'raw';root.mkdir();(root/'episode_00000002').mkdir()
    cfg={'record_root':str(root),'state_root':str(tmp_path/'state')}
    assert reserve_episode(cfg)==3
    # Failed attempts have no new folder; reservation must still remain consumed.
    assert reserve_episode(cfg)==4
    (root/'episode_00000009').mkdir()
    assert reserve_episode(cfg)==10


def test_original_episode_temporal_lookup_keeps_id():
    from original_episode_index import OriginalEpisodeIndexMixin
    class DenseBase:
        def _get_query_indices(self, row, position):
            return row, position
    class Reader(OriginalEpisodeIndexMixin, DenseBase):
        episodes = [42, 1042]
    reader = Reader()
    assert reader._get_query_indices(8, 42) == (8, 0)
    assert reader._get_query_indices(38, 1042) == (38, 1)
    assert reader.episodes == [42, 1042]
    with pytest.raises(ValueError, match='Unknown original'):
        reader._get_query_indices(8, 0)
    with pytest.raises(RuntimeError, match='read-only'):
        reader.add_frame({})
