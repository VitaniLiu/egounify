"""Offline MCAP certification. Imports ROS only inside the production reader."""
import json
import math
import time
from pathlib import Path
from contract import HAND_NAMES,HAND_TOPICS,CONTEXT,EVENT

def rows_from_rosbag(root,topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    reader=rosbag2_py.SequentialReader()
    uri=root if (root/'metadata.yaml').exists() else root/'data'
    reader.open(rosbag2_py.StorageOptions(uri=str(uri),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    try:
        while reader.has_next():
            topic,blob,stamp=reader.read_next()
            if topic in HAND_TOPICS:
                m=deserialize_message(blob,JointState)
                payload={'frame_id':m.header.frame_id,'names':list(m.name),'position':list(m.position),
                         'header_ns':m.header.stamp.sec*10**9+m.header.stamp.nanosec}
            elif topic in (CONTEXT,EVENT):payload=json.loads(deserialize_message(blob,String).data)
            else:payload=None
            yield topic,int(stamp),payload
    finally:del reader

def audit_rows(rows,cfg,capture):
    stats={};proof={};hands=[];markers=[];issues=[]
    for topic,stamp,payload in rows:
        s=stats.setdefault(topic,{'count':0,'first_log_ns':stamp,'last_log_ns':stamp,'max_gap_ns':0})
        if stamp<s['last_log_ns']:issues.append('Clock moved backwards: '+topic)
        s['max_gap_ns']=max(s['max_gap_ns'],stamp-s['last_log_ns']);s['last_log_ns']=stamp;s['count']+=1
        if topic==CONTEXT:
            key=(payload.get('session_id'),payload.get('seq'))
            if key in proof and proof[key]!=payload:issues.append('Conflicting hand context')
            proof[key]=payload
        elif topic==EVENT:
            if payload.get('capture_id')!=capture['capture_id']:issues.append('Foreign capture marker')
            markers.append(payload.get('kind'))
            for e in payload.get('snapshot',{}).get('context_history',[]):
                key=(e.get('session_id'),e.get('seq'))
                if key in proof and proof[key]!=e:issues.append('Conflicting snapshot event')
                proof[key]=e
        elif topic in HAND_TOPICS:hands.append((topic,stamp,payload))
    for topic in cfg['required_topics']:
        s=stats.get(topic,{})
        if s.get('count',0)<cfg['minimum_frames']:issues.append('Missing/insufficient recorded stream: '+topic)
        if s.get('max_gap_ns',0)>cfg['maximum_gap_s']*1e9:issues.append('Recorded gap exceeds profile: '+topic)
        if s and (s['first_log_ns']>capture['start_ack_ns']+cfg['maximum_gap_s']*1e9 or
                  s['last_log_ns']<capture['stop_request_ns']-cfg['maximum_gap_s']*1e9):
            issues.append('Recorded stream does not cover capture window: '+topic)
    if 'start' not in markers or 'stop' not in markers:issues.append('Capture start/stop marker not written')
    for topic,stamp,p in hands:
        side,kind=HAND_TOPICS[topic];parts=p['frame_id'].split('/')
        if len(parts)!=3 or parts[0]!='manus_sharpa':issues.append('Invalid hand frame identity');continue
        try:key=(parts[1],int(parts[2]));e=proof[key]
        except (KeyError,ValueError):issues.append('Hand sample lacks same-bag context');continue
        if e.get('session_id')!=capture['source_session_id'] or e.get('kind')!=kind or e.get('side')!=side:
            issues.append('Hand event role/session mismatch')
        if e.get('serials')!=cfg['serials']:issues.append('Hand event device mismatch')
        if e.get('clock_domain')!='robot_receive':issues.append('Hand clock domain mismatch')
        if not e.get('live') and not cfg['diagnostic']:issues.append('Simulated hand event in production capture')
        if p['names']!=HAND_NAMES or len(p['position'])!=22 or not all(math.isfinite(q) for q in p['position']):
            issues.append('Hand joint schema invalid')
        if p['position']!=e.get('position') or p['header_ns']!=e.get('received_robot_ns'):
            issues.append('Hand standard sample differs from context')
        if kind=='accepted' and e.get('sdk_result',{}).get('status')!='sdk_call_succeeded':
            issues.append('Unaccepted target recorded as action')
    return {'issues':sorted(set(issues)),'streams':stats,'markers':markers,'verified_hand_samples':len(hands),
            'clock_domain':'robot_receive','hardware_synchronized':False}

def audit_bag(root,cfg,capture):
    root=Path(root)
    # Native stop should close files; confirm stable sizes before reading/committing.
    previous=None;stable=0;deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        files={str(p):p.stat().st_size for p in root.rglob('*') if p.is_file() and p.name not in ('capture_manifest.json','capture_completion.json')}
        if files==previous:stable+=1
        else:stable=0;previous=files
        if stable>=3:break
        time.sleep(.15)
    if stable<3:raise RuntimeError('Recorder/video files still changing after stop')
    result=audit_rows(rows_from_rosbag(root,set(cfg['required_topics'])|{CONTEXT,EVENT}),cfg,capture)
    if cfg.get('camera_required'):
        videos=list((root/'video').rglob('*.mcap'))
        if not videos or not all(p.stat().st_size>0 for p in videos):result['issues'].append('Camera MCAP missing/empty')
        else:
            # Read actual video messages; existence alone is not coverage proof.
            import rosbag2_py
            times=[]
            for directory in sorted({p.parent for p in videos}):
                reader=rosbag2_py.SequentialReader()
                reader.open(rosbag2_py.StorageOptions(uri=str(directory),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
                reader.set_filter(rosbag2_py.StorageFilter(topics=[cfg['camera_topic']]))
                while reader.has_next():times.append(int(reader.read_next()[2]))
                del reader
            result['camera']={'count':len(times),'first_log_ns':min(times) if times else None,'last_log_ns':max(times) if times else None}
            if len(times)<2:result['issues'].append('Camera has insufficient real messages')
            elif result['streams']:
                start=min(s['first_log_ns'] for s in result['streams'].values())
                end=max(s['last_log_ns'] for s in result['streams'].values())
                if min(times)>start+100_000_000 or max(times)<end-100_000_000:result['issues'].append('Camera coverage does not span numerical recording within 100ms')
    return result
