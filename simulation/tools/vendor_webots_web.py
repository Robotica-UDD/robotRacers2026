#!/usr/bin/env python3
"""Cache the pinned official Webots web viewer and rewrite CDN paths locally.

No CDN is used by the running application. Resources remain under their
upstream licenses; the script records their URLs and SHA-256 checksums.
"""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path, PurePosixPath
import hashlib, json, os, re, urllib.parse, urllib.request

ROOT=Path(__file__).resolve().parents[2]
BASE='https://cyberbotics.com/wwi/R2025a/'
DEST=Path(os.environ.get('ROBOTRACERS_RUNTIME_DIR',ROOT/'simulation/.runtime'))/'wwi'
EXTENSIONS=('.js','.css','.wasm','.data','.png','.jpg','.jpeg','.svg','.woff','.woff2','.ttf','.ico','.gif')
EXTRA_RESOURCES={
    # Upstream uses this unversioned URL even in R2025a. Pin its bytes and cache
    # it at setup time so an unavailable project image never requires a CDN.
    'images/missing_texture.png':{
        'url':'https://cyberbotics.com/wwi/images/missing_texture.png',
        'sha256':'79165b59234c0600d697eb59076fabd6e36d3fe759d1616d01b1409bbadb6a8b',
    },
    'images/webots-logo.png':{
        'url':'https://cyberbotics.com/assets/images/webots.png',
        'sha256':'d5f10ace7034741f8646cb5978b82d78a2ef9ffb1771a757ca25fb185ccced0a',
    },
}


def resource_url(path):
    return EXTRA_RESOURCES.get(path,{}).get('url',BASE+path)


def validate_resource(path,data):
    expected=EXTRA_RESOURCES.get(path,{}).get('sha256')
    if expected and hashlib.sha256(data).hexdigest()!=expected:
        raise ValueError('El recurso '+path+' no coincide con su SHA-256 fijado.')


def patch_image_loader(text):
    """Resolve only this project's relative textures on the application's origin.

    Native Webots resolves these URLs relative to the PROTO/world. Its W3D
    stream exports the same string but resolves it at the streaming HTTP root,
    where ../../assets/ is no longer a valid project path. Keep native paths
    portable and remap this exact namespace only in the browser loader.
    """
    marker='// RobotRacers project textures: same-origin, independent of streaming port.'
    if marker not in text:
        anchor="    if (url.startsWith('webots://')) {"
        if anchor not in text:
            raise ValueError('Pinned Webots ImageLoader layout changed; review texture URL patch.')
        replacement="""    // RobotRacers project textures: same-origin, independent of streaming port.
    if (url.startsWith('../../assets/'))
      url = new URL('/simulation-assets/' + url.substring('../../assets/'.length), globalThis.location.href).href;

"""+anchor
        text=text.replace(anchor,replacement,1)
    return text.replace('https://cyberbotics.com/wwi/images/missing_texture.png',
                        '/webots/wwi/images/missing_texture.png')


def patch_logo_urls(text):
    """Keep the loading and information panels' exact upstream logos local."""
    for source in ('https://cyberbotics.com/assets/images/webots.png',
                   'https://raw.githubusercontent.com/cyberbotics/webots/master/resources/icons/core/webots.png'):
        text=text.replace(source,'/webots/wwi/images/webots-logo.png')
    return text


def fetch(path):
    target=DEST/path
    try:
        if target.is_file():
            data=target.read_bytes();validate_resource(path,data)
            return path,data,True
        request=urllib.request.Request(resource_url(path),headers={'User-Agent':'RobotRacers-local-setup/1.0'})
        with urllib.request.urlopen(request,timeout=45) as response:data=response.read()
        validate_resource(path,data)
        target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
        return path,data,False
    except Exception as exc:return path,None,str(exc)

def dependencies(path,data):
    if not path.endswith(('.js','.css')):return set()
    text=data.decode('utf-8',errors='replace')
    candidates=re.findall(r'''["']([^"'\n]{1,400})["']''',text)
    candidates+=re.findall(r'url\(([^)]+)\)',text)
    found=set()
    for item in candidates:
        item=item.strip(' "\'')
        if item.startswith(BASE):relative=item[len(BASE):]
        elif item.startswith(('https:','http:','data:','blob:','/','#')):continue
        else:relative=urllib.parse.urljoin(path,item)
        relative=relative.split('?')[0].split('#')[0]
        if relative.endswith(EXTENSIONS) and '..' not in PurePosixPath(relative).parts and not any(c in relative for c in '{}<>;=+\\'):
            found.add(relative)
    return found

def main():
    DEST.mkdir(parents=True,exist_ok=True)
    pending={'WebotsView.js','wrenjs.js','wrenjs.data','wrenjs.wasm','dependencies/assimpjs.wasm',
             'images/post_processing/smaa_area_texture.png','images/post_processing/smaa_search_texture.png',
             'images/post_processing/gtao_noise_texture.png'}
    pending.update(EXTRA_RESOURCES)
    visited=set();records=[];missing=[]
    with ThreadPoolExecutor(max_workers=12) as pool:
        while pending:
            batch=sorted(pending-visited);pending=set()
            if not batch:break
            for path,data,cached in pool.map(fetch,batch):
                visited.add(path)
                if data is None:missing.append({'path':path,'error':cached});continue
                record={'path':path,'url':resource_url(path),'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()}
                if path in EXTRA_RESOURCES:record['expected_sha256']=EXTRA_RESOURCES[path]['sha256']
                records.append(record)
                pending.update(dependencies(path,data)-visited)
                if path.endswith(('.js','.css')):
                    local=data.decode('utf-8',errors='replace').replace(BASE,'/webots/wwi/')
                    if path=='Stream.js':
                        # Product controls reconnect explicitly; upstream's English
                        # confirmation can reset the page and obscure connection loss.
                        local=re.sub(r"if \(window\.confirm\(`Streaming server error.*?window\.open\(window\.location\.href, '_self'\);", 
                                     '/* RobotRacers: onerror/onclose handle reconnect without a modal. */',local,flags=re.S)
                    if path=='W3dScene.js':
                        # Upstream pauses even broadcast-only viewers when loading
                        # a model; observers must not stop the camera/control clock.
                        local=local.replace("      view.stream.socket.send('pause');", "      if (!view.broadcast) view.stream.socket.send('pause');")
                        local=local.replace('const renderingMinTimeStep = 40;', 'const renderingMinTimeStep = 1000 / 60;')
                        if 'robotRacersRenderStats' not in local:
                            local=local.replace('    this.renderer.render();', '''    this.renderer.render();
    const stats = globalThis.robotRacersRenderStats ??= {total: 0, times: [], fps: 0};
    const stamp = performance.now();
    stats.total++; stats.times.push(stamp);
    while (stats.times.length > 1 && stats.times[0] < stamp - 1000) stats.times.shift();
    stats.fps = stats.times.length > 1 ? (stats.times.length - 1) * 1000 / (stamp - stats.times[0]) : 0;''',1)
                    if path=='WebotsView.js':
                        local=local.replace("          document.getElementsByTagName('webots-view')[0].toolbar.loadRobotWindows();", "          if (this.showRobotWindow !== false) document.getElementsByTagName('webots-view')[0].toolbar.loadRobotWindows();")
                    if path=='ImageLoader.js':
                        local=patch_image_loader(local)
                    if path in ('Progress.js','InformationPanel.js'):
                        local=patch_logo_urls(local)
                    (DEST/path).write_text(local)
            print(f'Visor local: {len(records)} recursos, {len(pending)} pendientes',flush=True)
    # Integrity is over the locally runnable bytes, after the documented
    # patches. Upstream URLs identify provenance, not unmodified file hashes.
    for record in records:
        data=(DEST/record['path']).read_bytes()
        record.update(bytes=len(data),sha256=hashlib.sha256(data).hexdigest())
    manifest={'version':'R2025a','source':BASE,'hash_scope':'local bytes after documented patches',
              'resources':records,'missing_optional':missing}
    (DEST.parent/'wwi-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    required=['WebotsView.js','webots.js','wrenjs.js','dependencies/assimpjs.js','dependencies/assimpjs.wasm',
              'images/missing_texture.png','images/webots-logo.png']
    absent=[p for p in required if not (DEST/p).is_file()]
    absent+= [row['path']for row in missing if row['path']in required and row['path']not in absent]
    if absent:raise SystemExit('Faltan recursos esenciales: '+','.join(absent))
    print('Visor preparado para uso local:',DEST)

if __name__=='__main__':main()
