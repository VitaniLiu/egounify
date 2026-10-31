"""Extract hand HDF5 from a certified unified capture; no external journal required."""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from hands_common import ROOT
from capture_contract import verify_completion,atomic_json
from hands_data import extract,output_path

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--raw',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--allow-diagnostic',action='store_true',help='Validation output only; never formal training')
    a=p.parse_args();m=verify_completion(a.raw,a.allow_diagnostic)
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    proofs={};paths=[]
    for rel in m['raw_files']:
        path=a.raw/rel
        if path.suffix!='.mcap' or 'video' in Path(rel).parts:continue
        paths.append(path)
        with path.open('rb') as f:
            reader=make_reader(f,decoder_factories=[DecoderFactory()])
            for _,channel,_,msg in reader.iter_decoded_messages(topics=['/recording/sharpa/context','/recording/capture_event']):
                e=json.loads(msg.data)
                events=[e] if channel.topic=='/recording/sharpa/context' else e.get('snapshot',{}).get('context_history',[])
                for event in events:
                    key=(event['session_id'],event['seq'])
                    if key in proofs and proofs[key]!=event:raise ValueError('Conflicting embedded hand context')
                    if event.get('session_id')!=m['source_session_id']:raise ValueError('Foreign hand session')
                    if event.get('serials')!=m['serials']:raise ValueError('Hand serial binding mismatch')
                    if not event.get('live') and not a.allow_diagnostic:raise ValueError('Simulated hand evidence')
                    proofs[key]=event
    if not paths or not proofs:raise ValueError('No numerical MCAP or embedded hand context')
    out=output_path(a.output);journal=out.with_name(out.name+'.embedded_context.jsonl')
    with journal.open('x') as f:
        for _,event in sorted(proofs.items()):f.write(json.dumps(event,allow_nan=False)+'\n')
    result=extract(paths,[journal],out,m['capture_id'])
    atomic_json(str(result)+'.capture.json',{'schema':'apex_capture_hand_extraction_v1',
        'capture_id':m['capture_id'],'diagnostic':m['diagnostic'],'raw_root':str(a.raw.resolve()),
        'clock_domain':'robot_receive','raw_manifest':m,'external_journal_required':False})
    print(result)

if __name__=='__main__':main()
