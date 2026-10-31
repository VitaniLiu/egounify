"""Strict import retaining recorder-assigned episode IDs; all data travels together."""
import hashlib,json,shutil,stat
from pathlib import Path
from unified_capture import validate_identity,verify_raw

def copy_unified(source,dest,save,safe,allow_diagnostic=False,selected=None):
    dest=safe(dest);dest.mkdir(parents=True,exist_ok=True)
    registry_path=dest/'episode_registry.json'
    registry=json.loads(registry_path.read_text()) if registry_path.exists() else {'schema':'unified_episode_registry_v1','episodes':{}}
    found={}
    for name in source.names(source.root):
        p=source.root+'/'+name
        if not stat.S_ISDIR(source.stat(p).st_mode):continue
        try:
            with source.open(p+'/capture_manifest.json') as f:blob=f.read()
            m=json.loads(blob)
            with source.open(p+'/capture_completion.json') as f:c=json.load(f)
        except FileNotFoundError:continue  # Legacy/in-progress folders never get re-numbered here.
        idx=validate_identity(m)
        if selected is not None and idx!=selected:continue
        if idx in found:raise ValueError('Duplicate source episode index: '+str(idx))
        if m.get('state')!='COMPLETE' or c.get('state')!='COMPLETE':raise ValueError('Episode is not COMPLETE: '+str(idx))
        if c.get('manifest_sha256')!=hashlib.sha256(blob).hexdigest() or c.get('capture_id')!=m['capture_id'] or c.get('episode_index')!=idx:raise ValueError('Remote completion identity/hash mismatch')
        if m.get('diagnostic') and not allow_diagnostic:raise ValueError('Diagnostic capture rejected for production')
        found[idx]=(p,m)
    if not found:raise ValueError('No certified numbered episodes; legacy recordings require legacy import')
    # Reject all conflicting IDs before copying any payload.
    for idx,(p,m) in found.items():
        key=f'episode_{idx:08d}';old=registry['episodes'].get(key)
        if old and old['capture_id']!=m['capture_id']:raise ValueError('Episode already bound to a different capture: '+key)
    for idx,(p,m) in sorted(found.items()):
        key=f'episode_{idx:08d}';out=dest/key
        if out.exists():
            old=verify_raw(out/'payload',allow_diagnostic,idx)
            if old['capture_id']!=m['capture_id'] or old!=m:raise ValueError('Existing episode differs; refusing overwrite')
            registry['episodes'][key]={'capture_id':m['capture_id'],'source':source.identity+':'+p,'status':'complete'}
            save(registry_path,registry);continue
        tmp=dest/('.'+key+'.partial')
        if tmp.exists():raise ValueError('Interrupted import exists; inspect before removing: '+str(tmp))
        tmp.mkdir();payload=tmp/'payload';payload.mkdir();copied=[]
        try:
            before=list(source.walk(p));directories=list(source.directories(p))
            for d in directories:(payload/d[len(p)+1:]).mkdir(parents=True,exist_ok=True)
            for src,s in before:
                rel=src[len(p)+1:];target=payload/rel;target.parent.mkdir(parents=True,exist_ok=True)
                h=hashlib.sha256()
                with source.open(src) as inp,target.open('xb') as output:
                    for b in iter(lambda:inp.read(1024*1024),b''):output.write(b);h.update(b)
                    output.flush();__import__('os').fsync(output.fileno())
                now=source.stat(src)
                if (s.st_size,s.st_mtime)!=(now.st_size,now.st_mtime):raise ValueError('Recording changed during import')
                copied.append({'path':rel,'size':s.st_size,'sha256':h.hexdigest()})
            final=list(source.walk(p))
            signature=lambda rows:[(f,s.st_size,s.st_mtime) for f,s in rows]
            if signature(before)!=signature(final) or directories!=list(source.directories(p)):raise ValueError('Recording inventory changed during import')
            verified=verify_raw(payload,allow_diagnostic,idx)
            if verified!=m:raise ValueError('Capture changed during import')
            save(tmp/'manifest.json',{'source':p,'episode_index':idx,'episode_name':key,'capture_id':m['capture_id'],
                'diagnostic':m['diagnostic'],'files':copied,'directories':[d[len(p)+1:] for d in directories],
                'classification':'certified unified raw; preserves original episode index'})
            tmp.rename(out)
            registry['episodes'][key]={'capture_id':m['capture_id'],'source':source.identity+':'+p,'status':'complete'}
            save(registry_path,registry)
            print('Imported '+key)
        except BaseException:
            # Preserve partial evidence for review, never publish it as a finished episode.
            raise
