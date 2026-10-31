import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import h5py
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from hands_common import ROOT,NAMES,SLOTS,config,digest,match,normalize
from conversion.hands_data import derive,write_json,extract
from replay import load_plan
from unittest.mock import patch
from types import SimpleNamespace
from hands_common import ensure_sdk_ports_free

class HandTests(unittest.TestCase):
    def test_source_cache_is_synchronized_on_every_connection(self):
        vendor_src=ROOT/'portable_deps/litchi_hardware/src'
        sys.path.insert(0,str(vendor_src))
        from litchi_hardware.hardware.sharpa.sdk_driver import SharpaSdkHand
        from unittest.mock import Mock
        sdkhand=object.__new__(SharpaSdkHand)
        ok=SimpleNamespace(code=0)
        setter=Mock(return_value=ok)
        sdkhand._wave=SimpleNamespace(set_control_source=setter,get_control_source=lambda:(ok,'sdk'))
        sdkhand._ensure_enum_setting('source','sdk','sdk')
        setter.assert_called_once_with('sdk')
        with self.assertRaises(Exception):
            sdkhand._wave.get_control_source=lambda:(ok,'app')
            sdkhand._ensure_enum_setting('source','sdk','sdk')

    def test_competing_pilot_is_rejected_without_hardware(self):
        with patch('hands_common.subprocess.run',return_value=SimpleNamespace(stdout='UNCONN 0 0 0.0.0.0:50000 0.0.0.0:* users:(("pilot_sdk",pid=1,fd=4))\n')):
            with self.assertRaisesRegex(RuntimeError,'Exit Sharpa Pilot'):ensure_sdk_ports_free()
        with patch('hands_common.subprocess.run',return_value=SimpleNamespace(stdout='UNCONN 0 0 0.0.0.0:5353 0.0.0.0:*\n')):
            ensure_sdk_ports_free()
    def test_mcap_extract_requires_sdk_receipt_and_robot_clock(self):
        from mcap_ros2.writer import Writer
        (ROOT/'data').mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=ROOT/'data') as d:
            tmp=Path(d);mcap=tmp/'synthetic.mcap';journal=tmp/'receiver.jsonl'
            msgdef='''std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
'''
            sid='a'*32;events=[];seq=0;origin=1790000000000000000
            with Writer(str(mcap)) as writer:
                schema=writer.register_msgdef('sensor_msgs/msg/JointState',msgdef)
                for i in range(3):
                    for side in SLOTS:
                        for kind,suffix in [('state','states'),('accepted','commands')]:
                            seq+=1;stamp=origin+i*50_000_000+seq*100
                            e=dict(schema='manus_sharpa_event_v1',session_id=sid,seq=seq,side=side,kind=kind,
                                   clock_domain='robot_receive',received_robot_ns=stamp,source_stamp_ns=stamp-1000,
                                   serials=config()['serials'],position=[.1]*22,names=list(NAMES))
                            if kind=='accepted':e['sdk_result']={'status':'sdk_call_succeeded'}
                            events.append(e)
                            m={'header':{'stamp':{'sec':stamp//10**9,'nanosec':stamp%10**9},'frame_id':f'manus_sharpa/{sid}/{seq}'},
                               'name':list(NAMES),'position':[.1]*22,'velocity':[],'effort':[]}
                            writer.write_message(f'/hand_{side}/joint_{suffix}',schema,m,log_time=stamp+100,publish_time=stamp)
            journal.write_text(''.join(json.dumps(e)+'\n' for e in events))
            out=extract([mcap],[journal],tmp/'extracted.hdf5','synthetic-only')
            with h5py.File(out) as h:
                self.assertEqual(h['action/left/position'].shape,(3,22))
                self.assertEqual(int(h['action/left/timestamp_ns'][0]),events[1]['received_robot_ns']+100)
            events[1]['sdk_result']['status']='rejected'
            journal.write_text(''.join(json.dumps(e)+'\n' for e in events))
            with self.assertRaises(ValueError):extract([mcap],[journal],tmp/'bad.hdf5','synthetic-only')

    def test_named_order_and_nonfinite(self):
        q=normalize(['left_'+n for n in reversed(NAMES)],list(reversed(range(22))),'left')
        np.testing.assert_array_equal(q,np.arange(22))
        with self.assertRaises(ValueError):normalize(list(NAMES),[float('nan')]*22,'left')
        with self.assertRaises(ValueError):normalize([NAMES[0]]*22,[0]*22,'left')

    def test_causal_boundary_and_tolerance(self):
        ix,ok,e=match([100,200],[99,100,199,200,251],True,50)
        self.assertEqual(ok.tolist(),[False,True,False,True,False])
        self.assertEqual(ix.tolist(),[0,0,0,1,1])
        with self.assertRaises(ValueError):match([1,1],[1])

    def test_schema_matches_user_reference(self):
        reference=json.loads((ROOT/'reference/schema78.json').read_text())
        for side,(start,end) in SLOTS.items():
            self.assertEqual(reference['names'][start:end],[side+'_hand_'+n for n in NAMES])

    def fixture(self,tmp):
        source=tmp/'source';(source/'meta').mkdir(parents=True);(source/'data/chunk-000').mkdir(parents=True)
        n=5;origin=1790000000000000000;grid=origin+np.arange(n,dtype=np.int64)*50_000_000
        ref=json.loads((ROOT/'reference/info.json').read_text());ref['total_frames']=n
        for key in ('state_mask','action_mask'):
            for start,end in SLOTS.values():ref[key][start:end]=[False]*22
        write_json(source/'meta/info.json',ref);write_json(source/'meta/source_timestamps.json',{'timestamp_ns':grid.tolist()})
        q=np.arange(n*78,dtype=np.float32).reshape(n,78)/1000
        table=pa.table({'observation.state':pa.array(q.tolist(),type=pa.list_(pa.float32(),78)),
                        'action':pa.array((q+.001).tolist(),type=pa.list_(pa.float32(),78)),
                        'validity':pa.array([[0]*78]*n,type=pa.list_(pa.uint8(),78)),
                        'observation.images.head_color':pa.array([{'bytes':b'unchanged jpeg','path':None}]*n),
                        'timestamp':pa.array(np.arange(n,dtype=np.float32)/20),'frame_index':list(range(n))})
        pq.write_table(table,source/'data/chunk-000/episode_000000.parquet')
        write_json(source/'meta/completion.json',{str(p.relative_to(source)):digest(p) for p in source.rglob('*') if p.is_file()})
        hpath=tmp/'hands.hdf5'
        with h5py.File(hpath,'x') as h:
            h.attrs.update(complete=True,clock_domain='robot_receive',capture_id='synthetic-only')
            for side in SLOTS:
                for role in ('state','action'):
                    g=h.create_group(f'{role}/{side}');g.attrs['joint_names']=json.dumps(NAMES)
                    g['timestamp_ns']=grid;g['position']=np.ones((n,22))*(.1 if role=='state' else .12)
        write_json(str(hpath)+'.commit.json',{'sha256':digest(hpath)})
        return source,hpath,table

    def test_derive_preserves_all_nonhand_content_and_seals_replay(self):
        (ROOT/'data').mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=ROOT/'data') as d:
            tmp=Path(d);source,hpath,before=self.fixture(tmp);out=derive(source,hpath,tmp/'derived','synthetic-only')
            after=pq.read_table(out/'data/chunk-000/episode_000000.parquet')
            for name in before.column_names:
                if name not in ('observation.state','action','validity'):self.assertTrue(before[name].equals(after[name]))
            indices=[i for i in range(78) if not 32<=i<54 and not 55<=i<77]
            for name in ('observation.state','action','validity'):
                np.testing.assert_array_equal(np.array(before[name].to_pylist())[:,indices],np.array(after[name].to_pylist())[:,indices])
            for rel,sha in json.loads((out/'meta/completion.json').read_text()).items():self.assertEqual(digest(out/rel),sha)
            p=load_plan(out);self.assertEqual(len(p['hands']['left']['position']),5)
            (out/'hands/replay.json').write_text('{}')
            with self.assertRaises(ValueError):load_plan(out)

    def test_misaligned_or_mismatched_capture_rejected(self):
        (ROOT/'data').mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=ROOT/'data') as d:
            tmp=Path(d);source,hpath,_=self.fixture(tmp)
            with self.assertRaises(ValueError):derive(source,hpath,tmp/'bad','wrong-capture')
            with h5py.File(hpath,'r+') as h:h['action/left/timestamp_ns'][:]+=100_000_000
            write_json(str(hpath)+'.commit.json',{'sha256':digest(hpath)})
            with self.assertRaises(ValueError):derive(source,hpath,tmp/'bad2','synthetic-only')

if __name__=='__main__':unittest.main()
