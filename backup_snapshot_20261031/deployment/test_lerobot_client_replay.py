"""LeRobot-only client contract, lossless transport, and timed replay test.

No raw/HDF5 reader, inference request, ROS publisher, or robot SDK is used.
"""
import sys
import json
import time
import socket
import threading
import importlib.util
import io
from pathlib import Path
import numpy as np
import pyarrow.parquet as pq
from PIL import Image

ROOT = Path('/mnt/apex-data/apex_n_sharpa/data')
OUT = ROOT/'inspection/lerobot_client_test_20261025'
TASK = ROOT/'lerobot/robot_import_20261024_egoav_v4'


def audit(event, args):
    if event == 'open' and isinstance(args[0], (str, bytes)):
        p = Path(args[0].decode() if isinstance(args[0], bytes) else args[0])
        if any(x in p.parts for x in ('raw_data', 'row_data', 'apex_data', 'sharpa_data', 'gento_replay_bags', 'hdf5', 'apex_hdf5')) or p.suffix in ('.hdf5', '.h5', '.mcap', '.db3'):
            raise PermissionError('Forbidden non-LeRobot source: '+str(p))


def main():
    sys.addaudithook(audit)
    spec = importlib.util.spec_from_file_location('replay_decoder', Path(__file__).parents[1]/'src/lerobot_replay/main.py')
    replay = importlib.util.module_from_spec(spec); spec.loader.exec_module(replay)
    sys.path.insert(0, str(OUT/'client_source'))
    from egoav78_contract import task_mask, validate_tasks, validate78
    from _ours_protocol import send_message, recv_message
    candidates = []
    for e in sorted(TASK.glob('episode_*')):
        times = json.loads((e/'meta/source_timestamps.json').read_text())['timestamp_ns']
        candidates.append((times[-1], e))
    assert len(candidates) == 3
    _, episode = max(candidates)
    fingerprint = replay.verify_episode(episode)
    read = lambda name: json.loads((episode/'meta'/name).read_text())
    info, timeline, chunks, cleaning = [read(x) for x in ('info.json','source_timestamps.json','chunk_frames.json','cleaning_report.json')]
    table = pq.read_table(episode/'data/chunk-000/episode_000000.parquet')
    cols = {k:np.array(table[k].to_pylist()) for k in ['action','observation.state','validity','frame_index','timestamp',*replay.AUX]}
    targets = replay.decode(info,timeline,chunks,cleaning,cols)
    # Independently encode decoded base targets back into each original chunk.
    max_error = 0.
    for chunk in chunks['chunks']:
        s,e = chunk['start_frame'],chunk['stop_frame']
        inverse = np.linalg.inv(np.asarray(chunk['base_from_chunk_camera']))
        for role,col in [('state','observation.state'),('action','action')]:
            for side,k in [('left',0),('right',9),('head',18)]:
                t = inverse @ targets[f'{role}_{side}_base_matrix'][s:e]
                encoded = np.concatenate([t[:,:3,3],t[:,:3,0],t[:,:3,1]],axis=1)
                max_error = max(max_error,float(np.max(np.abs(encoded-cols[col][s:e,k:k+9]))))
    assert max_error < 2e-5
    mask = cols['validity'][0].astype(bool)
    required = task_mask('g2').astype(bool)
    missing = np.flatnonzero(required & ~mask).tolist()
    # The numeric validator alone cannot distinguish missing zero placeholders.
    validate78(cols['observation.state'],'g2')
    try:
        validate_tasks({'recorded_episode':{'instruction':'Replay recorded episode', 'embodiment':'g2','validity':mask.astype(int).tolist()}})
    except ValueError as error:
        rejection = str(error)
    else:
        raise AssertionError('Client unexpectedly accepts missing-hand task mask')
    # Real client transport codec over a loopback socket, not a policy server.
    left,right = socket.socketpair();errors=[]
    def receive():
        try:
            for i in range(len(table)):
                msg=recv_message(right)
                assert np.array_equal(msg['state'],cols['observation.state'][i])
                assert np.array_equal(msg['validity'],cols['validity'][i])
                assert msg['frame_index']==i
                send_message(right,{'frame_index':i,'rgb_sha256':replay.hashlib.sha256(msg['head_rgb'].tobytes()).hexdigest()})
        except BaseException as e:errors.append(repr(e))
        finally:right.close()
    thread=threading.Thread(target=receive);thread.start()
    images=table['observation.images.head_color'].to_pylist()
    for i,entry in enumerate(images):
        assert entry.get('bytes'), 'Require embedded image; never follow external paths'
        rgb=np.array(Image.open(io.BytesIO(entry['bytes'])).convert('RGB'))
        assert rgb.shape==(540,960,3)
        send_message(left,{'frame_index':i,'state':cols['observation.state'][i],'validity':cols['validity'][i].astype(np.uint8),'head_rgb':rgb})
        ack=recv_message(left)
        assert ack=={'frame_index':i,'rgb_sha256':replay.hashlib.sha256(rgb.tobytes()).hexdigest()}
    left.close();thread.join();assert not errors,errors
    # Local sink exercises the whole recorded 20 Hz timeline, never robot IO.
    start=time.monotonic_ns();delays=[];consumed=[]
    for i,offset in enumerate(targets['offset_ns']):
        deadline=start+int(offset)
        while time.monotonic_ns()<deadline:
            time.sleep(min(.001,(deadline-time.monotonic_ns())/1e9) if deadline>time.monotonic_ns() else 0)
        delays.append((time.monotonic_ns()-deadline)/1e6)
        consumed.append(i)
    assert consumed==list(range(len(table)))
    assert replay.verify_episode(episode)==fingerprint
    np.savez_compressed(OUT/'latest_targets.npz',**targets)
    report=dict(source=str(episode),source_completion_sha256=fingerprint,
        selection='largest final timestamp among three LeRobot episodes',frames=len(table),fps=20,
        duration_s=int(targets['offset_ns'][-1])/1e9,raw_or_hdf5_read=False,
        replay_decode_passed=True,reencode_max_abs_error=max_error,
        client_transport_scope='actual client codec over local socketpair; no inference server',
        client_transport_frames=len(table),client_missing_required_indices=missing,
        client_task_mask_rejection=rejection,numeric_validator_alone_accepts_missing_zeros=True,
        inference_ready=False,inference_requests_sent=0,
        timed_replay_scope='local diagnostic sink, not physical execution',
        replay_consumed_frames=len(consumed),max_schedule_delay_ms=max(delays),
        robot_commands_sent=0,physical_replay_tested=False,
        limitations=['Current G2 client requires 44 measured hand dimensions absent from dataset',
        'Dataset has no selected trained task and explicitly declares wire incompatible',
        'Skye Cartesian execution needs verified IK, live start-pose transition and stop handling'])
    (OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
