#!/usr/bin/env python3
"""Install recording-only launch hooks; does not start/restart any ROS module."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import time

ROOT=Path(__file__).resolve().parent
LAUNCH=Path('/opt/kernelmind/apex/install/gento_replay/share/gento_replay/launch/gento_replay.launch.py')
YAML=Path('/opt/kernelmind/apex/install/gento_replay/share/gento_replay/config/gento_replay.yaml')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    if os.geteuid()!=0:raise SystemExit('Run sudo python3 install.py; this installer never starts robot services')
    import yaml
    if '--rollback' in sys.argv:
        m=json.loads((ROOT/'installation.json').read_text())
        for path,e in m['files'].items():
            p=Path(path)
            if sha(p)!=e['installed_sha256']:raise RuntimeError('Refuse rollback over independently changed file: '+path)
        for path,e in m['files'].items():shutil.copy2(e['backup'],path)
        print('Recording hooks rolled back. No services restarted.');return
    if (ROOT/'installation.json').exists():
        m=json.loads((ROOT/'installation.json').read_text())
        if all(sha(Path(p))==v['installed_sha256'] for p,v in m['files'].items()):
            print('Recording hooks already installed; no changes');return
        raise RuntimeError('Installed files changed; review before reinstalling')
    text=LAUNCH.read_text()
    needle="                executable='gento_replay_node',\n"
    if text.count(needle)!=1:raise RuntimeError('Unrecognized Gento launch; no blind patch')
    text=text.replace(needle,needle+'''                # APEX_RECORD_SYNC: preserve native writer and all playback paths.
                remappings=[
                    ('control/gento_replay/record', 'control/gento_replay/record_native'),
                    ('control/gento_replay/record_named', 'control/gento_replay/record_named_native'),
                    ('info/gento_replay/status', 'info/gento_replay/native_status'),
                ],
''')
    node="""            Node(
                executable='/usr/bin/python3',
                name='apex_record_sync',
                arguments=['/home/user/apex_record_sync/record_node.py'],
                parameters=[{'config_path': '/home/user/apex_record_sync/config.json'}],
                output='screen',
            ),
"""
    ui="            Node(\n                package='gento_replay',\n                executable='gento_replay_ui_node',"
    if text.count(ui)!=1:raise RuntimeError('Unrecognized Gento UI launch')
    text=text.replace(ui,node+ui)
    compile(text,str(LAUNCH),'exec')
    cfg=yaml.safe_load(YAML.read_text());params=cfg['/tj/gento_replay_node']['ros__parameters']
    for topic in ['/recording/sharpa/context','/recording/sharpa/health','/recording/capture_event']:
        if topic not in params['global_record_topics']:params['global_record_topics'].append(topic)
    params['camera_required']=True
    stamp=str(time.time_ns());backup=ROOT/'backups'/stamp;backup.mkdir(parents=True)
    manifest={'installed_at_ns':int(stamp),'motion_services_started':False,'files':{}}
    replacements=[(LAUNCH,text),(YAML,yaml.safe_dump(cfg,allow_unicode=True,sort_keys=False))]
    for p,new in replacements:
        target=backup/p.name;shutil.copy2(p,target)
        manifest['files'][str(p)]={'original_sha256':sha(p),'installed_sha256':hashlib.sha256(new.encode()).hexdigest(),'backup':str(target)}
    # Both originals are backed up before either installed file is replaced.
    (backup/'planned_installation.json').write_text(json.dumps(manifest,indent=2)+'\n')
    try:
        for p,new in replacements:
            tmp=p.with_name(p.name+'.record-sync-tmp');tmp.write_text(new);os.chmod(tmp,p.stat().st_mode);os.replace(tmp,p)
        (ROOT/'installation.json').write_text(json.dumps(manifest,indent=2)+'\n')
    except Exception:
        for p,_ in replacements:shutil.copy2(manifest['files'][str(p)]['backup'],p)
        raise
    print(json.dumps(manifest,indent=2));print('Installed. Applies on the next normal apex-teleop start; no robot services started/restarted.')

if __name__=='__main__':main()
