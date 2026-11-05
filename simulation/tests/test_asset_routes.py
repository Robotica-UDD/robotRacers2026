"""Project textures remain local without exposing files outside the assets tree."""
from pathlib import Path
import importlib
import hashlib
import json
import shutil
import subprocess

from fastapi.testclient import TestClient
import pytest

from simulation.service.runtime import RobotService
from simulation.tools.vendor_webots_web import patch_image_loader, patch_logo_urls, validate_resource

app_module=importlib.import_module('simulation.service.app')


@pytest.fixture
def asset_client(tmp_path,monkeypatch):
    service=RobotService(tmp_path/'runtime')
    simulation=tmp_path/'simulation'
    assets=simulation/'assets'
    (assets/'markers').mkdir(parents=True)
    (assets/'robot').mkdir()
    # The route streams bytes unchanged; it does not generate substitute images.
    payload=b'\x89PNG\r\n\x1a\nknown-texture'
    (assets/'markers'/'tag_0.png').write_bytes(payload)
    (assets/'robot'/'pcb_top.png').write_bytes(payload)
    (assets/'private.txt').write_text('not an image')
    (simulation/'outside.png').write_bytes(b'outside-assets')
    (assets/'markers'/'outside_link.png').symlink_to(simulation/'outside.png')
    monkeypatch.setattr(app_module,'SIMULATION_DIR',simulation)
    with TestClient(app_module.create_app(service,background=False)) as client:
        yield client,payload


@pytest.mark.parametrize('name',['markers/tag_0.png','robot/pcb_top.png'])
def test_assets_serve_exact_local_image(asset_client,name):
    client,payload=asset_client
    response=client.get('/simulation-assets/'+name)
    assert response.status_code==200
    assert response.content==payload
    assert response.headers['content-type']=='image/png'
    assert response.headers['cache-control']=='no-cache'


@pytest.mark.parametrize('name',[
    '%2e%2e%2foutside.png',
    '%2e%2e%2f%2e%2e%2foutside.png',
    '%2Fetc%2Fpasswd.png',
    'markers/outside_link.png',
    'markers/missing.png',
    'private.txt',
    'markers/%00.png',
    '%252e%252e%252foutside.png',
    '..%5Coutside.png',
])
def test_assets_reject_traversal_symlinks_and_non_images(asset_client,name):
    client,_=asset_client
    response=client.get('/simulation-assets/'+name)
    assert response.status_code==404
    assert b'outside-assets'not in response.content


def test_texture_patch_is_narrow_and_idempotent():
    original="""    if (url.startsWith('webots://')) {
      url = url.replace('webots://', 'upstream/');
    }
"""
    patched=patch_image_loader(original)
    assert patch_image_loader(patched)==patched
    assert patched.count('RobotRacers project textures:')==1
    assert patched.endswith(original)
    assert "url.startsWith('../../assets/')"in patched
    with pytest.raises(ValueError):patch_image_loader('upstream structure changed')


def test_missing_texture_fallback_is_local_even_when_namespace_already_patched():
    original="""    if (url.startsWith('webots://')) {
      img.src='https://cyberbotics.com/wwi/images/missing_texture.png';
    }
"""
    patched=patch_image_loader(original)
    assert "img.src='/webots/wwi/images/missing_texture.png'"in patched
    assert 'https://cyberbotics.com/wwi/images/missing_texture.png'not in patched
    assert patch_image_loader(patched)==patched


def test_fallback_download_rejects_unexpected_bytes(monkeypatch):
    vendor=importlib.import_module('simulation.tools.vendor_webots_web')
    good=b'known placeholder bytes'
    monkeypatch.setitem(vendor.EXTRA_RESOURCES,'test.png',{'url':'https://example.test/test.png','sha256':hashlib.sha256(good).hexdigest()})
    validate_resource('test.png',good)
    with pytest.raises(ValueError):validate_resource('test.png',b'changed bytes')


@pytest.mark.parametrize('source',[
    'https://cyberbotics.com/assets/images/webots.png',
    'https://raw.githubusercontent.com/cyberbotics/webots/master/resources/icons/core/webots.png',
])
def test_panel_logos_are_local_and_patch_is_idempotent(source):
    unrelated='https://example.test/logo.png'
    original=f'<img src="{source}"><img src="{unrelated}">'
    patched=patch_logo_urls(original)
    assert patched==f'<img src="/webots/wwi/images/webots-logo.png"><img src="{unrelated}">'
    assert patch_logo_urls(patched)==patched


def test_texture_patch_resolves_application_origin_in_actual_javascript():
    node=shutil.which('node')
    if node is None:pytest.skip('Node is required to execute the vendored loader URL regression.')
    original="""    if (url.startsWith('webots://')) {
      return url;
    }
    return url;
"""
    patched=patch_image_loader(original)
    source="globalThis.location={href:'http://127.0.0.1:8000/'};\nfunction resolve(url){\n"+patched+"}\n"
    source+="console.log(JSON.stringify(['../../assets/robot/pcb_top.png','../../assets/markers/tag_0.png','other.png','https://example.test/image.png','webots://projects/image.png'].map(resolve)));"
    result=subprocess.run([node,'--input-type=module','-e',source],check=True,capture_output=True,text=True)
    assert json.loads(result.stdout)==[
        'http://127.0.0.1:8000/simulation-assets/robot/pcb_top.png',
        'http://127.0.0.1:8000/simulation-assets/markers/tag_0.png',
        'other.png',
        'https://example.test/image.png',
        'webots://projects/image.png',
    ]
