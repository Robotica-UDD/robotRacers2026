"""Ajuste digital de cámara desde el panel, sin cámara ni movimiento reales."""
from pathlib import Path
import re
import shutil
import subprocess

import pytest


PAGE = Path(__file__).resolve().parents[1] / "webcam/static/index.html"
FUNCTIONS = (
    "illuminationValues", "renderIlluminationUI", "renderIllumination",
    "scheduleIllumination", "queueIllumination", "saveIllumination",
    "optimizeIllumination", "resetIllumination", "illuminationInput",
)
HARNESS = r"""
const assert = require('node:assert/strict');
class Element {
  constructor() { this.value=''; this.textContent=''; this.disabled=false; this.hidden=false; this.checked=false; }
}
const elements=new Map();
const $=id=>{ if(!elements.has(id))elements.set(id,new Element()); return elements.get(id); };
const number=new Intl.NumberFormat('es-CL',{maximumFractionDigits:1});
let connected=true;
const illuminationDefaults={enabled:false,automatic:true,brightness:0,contrast:1,gamma:1,local_contrast:true};
let illuminationState=null, illuminationPending=null, illuminationSending=null;
let illuminationInFlight=false, illuminationDebounce=null, illuminationAckGeneration=0, illuminationError='';
const timers=new Map(); let timerId=0;
const setTimeout=(callback,delay)=>{ const id=++timerId;timers.set(id,{callback,delay});return id; };
const clearTimeout=id=>timers.delete(id);
let api=async()=>{throw new Error('Solicitud inesperada');};
const initial=(revision=4,extra={})=>({...illuminationDefaults,revision,diagnostics:{mean:112,overexposed_pct:1,underexposed_pct:2,gamma_used:1},message:'Ajuste de imagen activo.',hardware_controls:false,...extra});
"""


def run(script):
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node permite comprobar los controles sin acceder al hardware.")
    html = PAGE.read_text()
    functions = []
    for name in FUNCTIONS:
        match = re.search(r"    (?:async )?function " + name + r"\([^\n]*\) \{.*?\n    \}", html, re.S)
        assert match, name
        functions.append(match.group())
    source = HARNESS + "\n".join(functions) + "\n(async()=>{\n" + script + "\n})().catch(error=>{console.error(error);process.exit(1)});"
    result = subprocess.run([node, "--input-type=commonjs"], input=source, text=True,
                            capture_output=True, timeout=10.)
    assert result.returncode == 0, result.stderr or result.stdout


def test_panel_has_precise_controls_and_explains_digital_adjustment():
    page = PAGE.read_text()
    assert "Optimizar iluminación" in page
    assert "Ajuste digital de brillo y contraste. No cambia la calibración." in page
    for identifier, low, high, step in (("brightness", "-60", "60", "2"),
                                       ("contrast", "0.5", "2.5", "0.05"),
                                       ("gamma", "0.5", "2", "0.05")):
        control = re.search(r'<input id="illumination-' + identifier + r'"[^>]*>', page).group()
        for value in (f'min="{low}"', f'max="{high}"', f'step="{step}"'):
            assert value in control
    assert "renderIllumination(next.illumination, 'state', illuminationGeneration)" in page
    assert "const illuminationGeneration = illuminationAckGeneration;" in page


def test_optimize_sends_one_authenticated_image_patch_and_no_other_action():
    run(r"""
renderIllumination(initial());
const requests=[];
api=async(path,options)=>{
  requests.push({path,options,body:JSON.parse(options.body)});
  return initial(5,{enabled:true});
};
assert.equal(await optimizeIllumination(),true);
assert.equal(requests.length,1);
assert.equal(requests[0].path,'/api/illumination');
assert.equal(requests[0].options.headers['X-Requested-With'],'XMLHttpRequest');
assert.equal(requests[0].options.headers['Content-Type'],'application/json');
assert.deepEqual(requests[0].body,{...illuminationDefaults,enabled:true,expected_revision:4});
assert.equal(illuminationState.enabled,true);
assert.equal(illuminationAckGeneration,1);
assert.equal(illuminationPending,null);
""")


def test_slider_changes_coalesce_and_wait_for_confirmed_revision():
    run(r"""
renderIllumination(initial());
const requests=[];let resolveFirst,resolveSecond;
api=(path,options)=>{
  requests.push({path,body:JSON.parse(options.body)});
  return new Promise(resolve=>{if(requests.length===1)resolveFirst=resolve;else resolveSecond=resolve;});
};
const first=optimizeIllumination();
await queueIllumination({enabled:true,automatic:false,brightness:16});
await queueIllumination({gamma:1.25});
await queueIllumination({brightness:22});
assert.equal(requests.length,1);
assert.equal(timers.size,1);
assert.equal([...timers.values()][0].delay,160);
assert.equal($('illumination-brightness').value,'22');
assert.equal(await saveIllumination(),false);
resolveFirst(initial(5,{enabled:true}));await first;
assert.equal($('illumination-brightness').value,'22');
const second=saveIllumination();
assert.equal(requests.length,2);
assert.deepEqual(requests[1].body,{enabled:true,automatic:false,brightness:22,gamma:1.25,expected_revision:5});
resolveSecond(initial(6,{enabled:true,automatic:false,brightness:22,gamma:1.25}));await second;
assert.equal(illuminationState.revision,6);
assert.equal($('illumination-gamma').value,'1.25');
assert.equal(illuminationInFlight,false);
assert.equal(timers.size,0);
""")


def test_old_state_read_cannot_replace_post_ack_or_pending_local_input():
    run(r"""
renderIllumination(initial());
const oldGeneration=illuminationAckGeneration;
api=async()=>initial(5,{enabled:true});
await optimizeIllumination();
assert.equal(renderIllumination(initial(4),'state',oldGeneration),false);
assert.equal(renderIllumination(initial(4),'state',illuminationAckGeneration),false);
assert.equal(illuminationState.enabled,true);
await queueIllumination({enabled:true,automatic:false,contrast:1.8});
assert.equal(renderIllumination(initial(5,{enabled:true})),true);
assert.equal($('illumination-contrast').value,'1.8');
assert.equal($('illumination-automatic').checked,false);
""")


def test_failed_save_restores_server_confirmed_values_without_retry():
    run(r"""
renderIllumination(initial(4,{brightness:8}));
const requests=[];
api=async(path)=>{requests.push(path);throw new Error('No se pudo guardar');};
await queueIllumination({brightness:30,enabled:true});
assert.equal(await saveIllumination(),false);
assert.equal($('illumination-brightness').value,'8');
assert.equal(illuminationState.brightness,8);
assert.equal(illuminationPending,null);
assert.equal(timers.size,0);
assert.equal($('illumination-error').hidden,false);
assert.equal($('illumination-error').textContent,'No se pudo guardar');
assert.equal(await saveIllumination(),false);
assert.deepEqual(requests,['/api/illumination']);
""")


def test_reset_restores_image_only_and_preserves_camera_map_and_calibration():
    run(r"""
renderIllumination(initial(10,{enabled:true,automatic:false,brightness:30,contrast:1.6,gamma:.8}));
const camera={camera_id:'usb',sequence:44,visible:false};
const calibration={center:[4,5],front:[6,7]},route=[[1,2],[3,4]],home=[9,8];
const before=JSON.stringify({camera,calibration,route,home});
const requests=[];
api=async(path,options)=>{requests.push([path,JSON.parse(options.body)]);return initial(11);};
assert.equal(await resetIllumination(),true);
assert.deepEqual(requests,[['/api/illumination',{...illuminationDefaults,expected_revision:10}]]);
assert.equal(JSON.stringify({camera,calibration,route,home}),before);
assert.equal(illuminationState.enabled,false);
assert.equal($('illumination-status').textContent,'Imagen original.');
""")


def test_controls_remain_available_without_marker_and_do_not_block_stop_buttons():
    run(r"""
renderIllumination(initial());
$('stop').disabled=false;$('robot-stop').disabled=false;
assert.equal($('illumination-optimize').disabled,false);
let release;
api=()=>new Promise(resolve=>{release=resolve});
const pending=optimizeIllumination();
assert.equal(illuminationInFlight,true);
assert.equal($('illumination-brightness').disabled,false);
assert.equal($('stop').disabled,false);
assert.equal($('robot-stop').disabled,false);
release(initial(5,{enabled:true}));await pending;
connected=false;renderIlluminationUI();
assert.equal($('illumination-optimize').disabled,true);
assert.equal($('illumination-brightness').disabled,true);
assert.equal($('stop').disabled,false);
assert.equal($('robot-stop').disabled,false);
""")


def test_invalid_slider_values_do_not_send_requests_or_replace_confirmed_values():
    run(r"""
renderIllumination(initial());let calls=0;
api=async()=>{calls++;throw new Error('No enviar');};
assert.equal(await queueIllumination({gamma:NaN}),false);
assert.equal(await queueIllumination({contrast:3}),false);
assert.equal(await queueIllumination({brightness:-61}),false);
assert.equal(await queueIllumination({unknown:true}),false);
assert.equal(calls,0);
assert.equal(illuminationPending,null);
assert.equal($('illumination-gamma').value,'1');
assert.equal($('illumination-error').hidden,false);
""")


def test_manual_slider_selects_manual_mode_and_overexposure_message_is_conditional():
    run(r"""
renderIllumination(initial(4,{diagnostics:{mean:201,overexposed_pct:9,underexposed_pct:1,gamma_used:.8}}));
assert.equal($('illumination-warning').hidden,false);
assert.match($('illumination-diagnostics').textContent,/201\/255/);
assert.equal(await illuminationInput({currentTarget:{value:'1.4',disabled:false}},'gamma'),true);
assert.deepEqual(illuminationPending,{enabled:true,automatic:false,gamma:1.4});
assert.equal($('illumination-automatic').checked,false);
renderIllumination(initial(4,{diagnostics:{mean:80,overexposed_pct:0}}));
assert.equal($('illumination-warning').hidden,true);
""")


def test_queued_change_waits_for_local_service_reconnection():
    run(r"""
renderIllumination(initial());
await queueIllumination({enabled:true,brightness:10});
connected=false;
const timer=[...timers.values()][0];timers.clear();await timer.callback();
assert.equal(illuminationInFlight,false);
assert.deepEqual(illuminationPending,{enabled:true,brightness:10});
connected=true;renderIllumination(initial());
assert.equal(timers.size,1);
api=async()=>initial(5,{enabled:true,brightness:10});
await saveIllumination();
assert.equal(illuminationState.brightness,10);
assert.equal(illuminationPending,null);
""")
