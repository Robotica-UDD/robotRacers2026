#!/usr/bin/env python3
"""Verify the already installed local product without downloading anything."""
from pathlib import Path
import hashlib,json,re,time,urllib.request
from simulation.tools.validate_live import ROOT,BASE

def main():
    runtime=ROOT/'simulation/.runtime'
    manifest=json.loads((runtime/'wwi-manifest.json').read_text())
    failures=[];count=0
    for item in manifest['resources']:
        local=runtime/'wwi'/item['path']
        if not local.is_file() or hashlib.sha256(local.read_bytes()).hexdigest()!=item['sha256']:
            failures.append(item['path']);continue
        request=urllib.request.Request(BASE+'/webots/wwi/'+item['path'])
        with urllib.request.urlopen(request,timeout=5) as response:
            if hashlib.sha256(response.read()).hexdigest()!=item['sha256']:failures.append('HTTP:'+item['path'])
        count+=1
    with urllib.request.urlopen(BASE,timeout=5) as response:
        html=response.read().decode();policy=response.headers.get('Content-Security-Policy','')
    for name in re.findall(r'(?:src|href)="(/assets/[^\"]+)"',html):
        with urllib.request.urlopen(BASE+name,timeout=5) as response:assert response.status==200
    assert "default-src 'self'" in policy and 'https:' not in policy
    textures=['robot/pcb_top.png',*[f'markers/tag_{i}.png' for i in range(5)]]
    for name in textures:
        with urllib.request.urlopen(BASE+'/simulation-assets/'+name,timeout=5) as response:assert response.status==200
    report={'local_viewer_resources_verified':count,'project_textures_verified':len(textures),
        'hash_failures':failures,'external_document_resources_blocked':True,
        'scope':'Local hashes, HTTP resources and CSP. Fresh offline-proxy startup is recorded separately.',
        'passed':not failures}
    (ROOT/'simulation/validation/offline.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
