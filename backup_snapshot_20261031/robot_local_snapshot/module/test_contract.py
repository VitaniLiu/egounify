import copy,json,tempfile,time,unittest
from pathlib import Path
from contract import Capture,Monitor,HAND_NAMES,HAND_TOPICS,CONTEXT,EVENT,verify_completion
from audit import audit_rows

class ContractTests(unittest.TestCase):
    def cfg(self,root):
        c=json.loads(Path(__file__).with_name('config.json').read_text())
        c.update(diagnostic=True,camera_required=False,required_topics={CONTEXT:'std_msgs/msg/String'},
                 record_root=str(root/'raw'),state_root=str(root/'state'),minimum_free_bytes=0)
        return c
    def ready(self,c):
        m=Monitor(c);m.observe(CONTEXT,'std_msgs/msg/String')
        m.hand_health(dict(schema='apex_sharpa_health_v1',session_id='a'*32,serials=c['serials'],live=False,ready=True))
        return m
    def test_capture_integrity_and_duplicate_start(self):
        with tempfile.TemporaryDirectory() as d:
            c=self.cfg(Path(d));m=self.ready(c)
            class Native:
                active=False
                def recording(self):return self.active
                def command(self,data,name):
                    self.active=bool(data)
                    if data:
                        p=Path(c['record_root'])/name;p.mkdir();(p/'raw.mcap').write_bytes(b'diagnostic-only')
                    return {'success':True}
            engine=Capture(c,m,Native(),lambda _:None,lambda *args:{'issues':[]})
            engine.start('test')
            with self.assertRaises(RuntimeError):engine.start('duplicate')
            # Duplicate start must not stop or discard an existing capture.
            self.assertIsNotNone(engine.active)
            result=engine.stop();self.assertEqual(result['state'],'COMPLETE')
            verify_completion(result['bag'],True)
            with self.assertRaises(ValueError):verify_completion(result['bag'])
            (Path(result['bag'])/'raw.mcap').write_bytes(b'tamper')
            with self.assertRaises(ValueError):verify_completion(result['bag'],True)
    def test_live_serial_and_freshness_gates(self):
        with tempfile.TemporaryDirectory() as d:
            c=self.cfg(Path(d));m=self.ready(c);self.assertFalse(m.check())
            c['diagnostic']=False;self.assertIn('hand source is not live',m.check())
            m.health['serials']={};self.assertIn('hand serial mismatch',m.check())
            self.assertTrue(any('stale' in x for x in m.check(now=time.monotonic()+2)))
    def test_evidence_and_action_audit(self):
        with tempfile.TemporaryDirectory() as d:
            c=self.cfg(Path(d));c['required_topics']={t:'sensor_msgs/msg/JointState' for t in HAND_TOPICS}
            capture=dict(capture_id='capture',source_session_id='a'*32,start_ack_ns=100,stop_request_ns=200)
            rows=[];seq=0
            for stamp in (100,200):
                for topic,(side,kind) in HAND_TOPICS.items():
                    seq+=1;e=dict(session_id='a'*32,seq=seq,serials=c['serials'],side=side,kind=kind,
                        live=False,clock_domain='robot_receive',position=[.1]*22,received_robot_ns=stamp,
                        sdk_result={'status':'sdk_call_succeeded'})
                    rows.extend([(CONTEXT,stamp,e),(topic,stamp,dict(frame_id=f'manus_sharpa/{"a"*32}/{seq}',names=HAND_NAMES,position=[.1]*22,header_ns=stamp))])
            rows.extend([(EVENT,100,dict(capture_id='capture',kind='start')),(EVENT,200,dict(capture_id='capture',kind='stop'))])
            rows.sort(key=lambda r:r[1]);self.assertFalse(audit_rows(rows,c,capture)['issues'])
            bad=copy.deepcopy(rows)
            next(e for t,_,e in bad if t==CONTEXT and e['kind']=='accepted')['sdk_result']['status']='rejected'
            self.assertIn('Unaccepted target recorded as action',audit_rows(bad,c,capture)['issues'])

if __name__=='__main__':unittest.main()
