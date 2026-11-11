"""Casa se marca sin mover motores; la reacción a manos se activa explícitamente."""
from pathlib import Path
import re
import shutil
import subprocess

import pytest


PAGE = Path(__file__).resolve().parents[1] / "webcam/static/index.html"


def functions(*names):
    html = PAGE.read_text()
    found = []
    for name in names:
        match = re.search(r"    (?:async )?function " + name + r"\([^\n]*\) \{.*?\n    \}", html, re.S)
        assert match, name
        found.append(match.group())
    return "\n".join(found)


HARNESS = r"""
const assert = require('node:assert/strict');
class Element {
  constructor() { this.attributes = {}; this.textContent = ''; this.value = ''; this.hidden = false; this.disabled = false; this.dataset = {}; }
  setAttribute(k,v) { this.attributes[k] = String(v); }
  removeAttribute(k) { delete this.attributes[k]; }
}
const elements = new Map();
const $ = id => { if (!elements.has(id)) elements.set(id, new Element()); return elements.get(id); };
let state = {camera_id: 'usb', frame_width: 1280, frame_height: 720};
let homeState = {home_px: [400,300], source: {camera_id:'usb', frame_width:1280, frame_height:720},
  enabled:false, distance_mm:150, phase:'disabled', detector:{status:'ready'}};
let homeAvailable = true, homeBusy = '', homeReadBusy = false, homeReadVersion=0, homeAppliedVersion=0, homeActionVersion=0, homeDistanceEditing=false;
let calibrationStage='', navigationTool='points', navigationBusy='', robotAvailable=true;
let robotState={connected:true, ready:true, velocity_control:true, encoder_ready:true, moving:false};
let robotPulsePending=false, robotPulseSubmitting=false, robotStopBusy=false, robotConnectBusy=false;
let obstacleDraft=[], editing=true, calibrated=true, fresh=true, navigating=false;
const cameraVisionFresh=()=>fresh, navigationCalibrated=()=>calibrated, canEditNavigation=()=>editing;
const routePointsPending=()=>false, navigationOwnsRobot=()=>navigating;
let renders=0, overlays=0;
let api;
const updateNavigationUI=()=>{renders++}, drawNavigationOverlay=()=>{overlays++};
"""


COMMON = ("finitePoint", "setSvgVisible", "homePointInImage", "canMarkHome", "canArmHome",
          "setHomeError", "renderHomeUI", "renderHome", "readHomeState", "homePost",
          "markHomePoint", "toggleHomeReaction", "cancelHomeReturn", "saveHomeDistance", "drawHomeOverlay")


def run(script, *, prelude="", extra=()):
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node permite comprobar la interfaz sin cámara ni robot.")
    source = HARNESS + prelude + functions(*COMMON, *extra) + "\n(async()=>{\n" + script + "\n})().catch(error=>{console.error(error);process.exit(1)});"
    result = subprocess.run([node, "--input-type=commonjs"], input=source, text=True,
                            capture_output=True, timeout=10.)
    assert result.returncode == 0, result.stderr or result.stdout


def test_configuration_note_is_written_without_replacing_current_speed_values():
    html = PAGE.read_text()
    assert "Configuración probada: avance 240 mm/s · giro máximo 90°/s." in html
    run(r"""
const draftSpeeds={forward_mm_s:330, turn_deg_s:460};
renderHomeUI();
assert.deepEqual(draftSpeeds,{forward_mm_s:330, turn_deg_s:460});
assert.equal($('home-toggle').attributes['aria-pressed'],'false');
assert.equal($('home-cancel').hidden,true);
""")


def test_marking_home_only_saves_the_point_and_never_arms_or_starts_navigation():
    run(r"""
const requests=[];
api=async function(path,options){
  requests.push([path,JSON.parse(options.body),options.headers]);
  return {...homeState,home_px:[501,322]};
}
navigationTool='home';
assert.equal(await markHomePoint([501,322]),true);
assert.equal(requests.length,1);
assert.equal(requests[0][0],'/api/home/point');
assert.deepEqual(requests[0][1],{x:501,y:322,frame_width:1280,frame_height:720,camera_id:'usb'});
assert.equal(requests[0][2]['X-Requested-With'],'XMLHttpRequest');
assert.equal(homeState.enabled,false);
assert.equal(navigationTool,'points');
assert.equal(requests.some(([path])=>path.includes('/navigation/start')),false);
navigationTool='home';
assert.equal(await markHomePoint([NaN,2]),false);
assert.equal(requests.length,1);
""")


def test_reaction_requires_house_same_camera_calibration_link_and_ready_detector():
    run(r"""
assert.equal(canArmHome(),true);
for(const status of ['loading','missing_model','error']){
  homeState.detector.status=status;
  assert.equal(canArmHome(),false);
  renderHomeUI();assert.equal($('home-toggle').disabled,true);
}
homeState.detector.status='ready';
homeState.home_px=null;assert.equal(canArmHome(),false);
homeState.home_px=[400,300];state.camera_id='other';assert.equal(canArmHome(),false);
state.camera_id='usb';calibrated=false;assert.equal(canArmHome(),false);
calibrated=true;robotState.connected=false;assert.equal(canArmHome(),false);
robotState.connected=true;robotState.encoder_ready=false;assert.equal(canArmHome(),false);
robotState.encoder_ready=true;robotState.moving=true;assert.equal(canArmHome(),false);
navigating=true;assert.equal(canArmHome(),true);
""")


def test_reading_state_does_not_arm_and_toggle_is_explicit_even_during_navigation():
    run(r"""
const requests=[];
api=async function(path,options){
  requests.push([path,options?JSON.parse(options.body):null]);
  return options ? {...homeState,enabled:JSON.parse(options.body).enabled,phase:'watching'} : {...homeState};
}
await readHomeState();
assert.deepEqual(requests,[['/api/home/state',null]]);
assert.equal(homeState.enabled,false);
navigating=true;robotState.moving=true;
await toggleHomeReaction();
assert.deepEqual(requests[1],['/api/home/settings',{enabled:true}]);
assert.equal(homeState.enabled,true);
renderHomeUI();assert.equal($('home-toggle').attributes['aria-pressed'],'true');
fresh=false;robotState.connected=false;
await toggleHomeReaction();
assert.deepEqual(requests[2],['/api/home/settings',{enabled:false}]);
""")


def test_old_read_cannot_overwrite_an_explicit_arm_confirmation():
    run(r"""
let resolveRead;
api=async function(path,options){
  if(!options)return await new Promise(resolve=>{resolveRead=resolve});
  return {...homeState,enabled:true,phase:'watching'};
}
const pending=readHomeState();
await toggleHomeReaction();
assert.equal(homeState.enabled,true);
resolveRead({...homeState,enabled:false,phase:'disabled'});
await pending;
assert.equal(homeState.enabled,true);
""")


def test_house_marker_is_static_and_disappears_for_another_camera_or_clear():
    run(r"""
assert.equal(drawHomeOverlay(2),true);
assert.equal($('navigation-home').attributes.transform,'translate(400 300)');
assert.equal($('navigation-home-circle').attributes.r,'6');
assert.equal('hidden' in $('navigation-home').attributes,false);
state.camera_id='other';assert.equal(drawHomeOverlay(2),false);
assert.equal('hidden' in $('navigation-home').attributes,true);
state.camera_id='usb';homeState.home_px=null;assert.equal(drawHomeOverlay(2),false);
""")


def test_idle_hand_watch_allows_a_normal_route_but_not_an_open_loop_pulse():
    prelude = r"""
let navigationState={home_watch_armed:true,home_watch_idle:true,planning_status:'ready'};
let obstacleMarginPending=false,obstacleMarginEditing=false,speedPending=null,speedInFlight=false,robotPortsLoading=false;
const navigationWaypoints=()=>[[600,300]],navigationRouteRevision=()=>1,navigationObstacles=()=>[];
const robotSupportsSpeedSettings=()=>true,robotIsConfiguring=()=>false,navigationAllowsReconnect=()=>false;
"""
    run(r"""
homeState.enabled=true;robotState.navigation_active=true;
assert.equal(navigating,false);assert.equal(robotState.moving,false);
assert.equal(canStartNavigation(),true);
$('robot-port').value='usb';updateRobotButtons();
assert.equal($('robot-pulse').disabled,true);
assert.equal($('robot-stop').disabled,false);
homeState.phase='at_home';renderHomeUI();
assert.equal($('home-status').textContent,'En casa');
homeState.phase='waiting_vision';homeState.returning=true;renderHomeUI();
assert.equal($('home-status').textContent,'Esperando el marcador');
assert.equal($('home-cancel').hidden,false);
""", prelude=prelude, extra=("canStartNavigation", "updateRobotButtons"))


def test_proximity_is_validated_before_posting_and_cancel_button_uses_home_disarm():
    run(r"""
const requests=[];
api=async function(path,options){requests.push([path,JSON.parse(options.body)]);return {...homeState,distance_mm:200};}
homeDistanceEditing=true;$('home-distance').value='49';await saveHomeDistance();
assert.equal(requests.length,0);
assert.equal($('home-distance').value,150);
homeDistanceEditing=true;$('home-distance').value='200';await saveHomeDistance();
assert.deepEqual(requests,[['/api/home/settings',{distance_mm:200}]]);
""")
    html = PAGE.read_text()
    assert "$('home-cancel').addEventListener('click', cancelHomeReturn);" in html
    assert "$('navigation-home-mode').addEventListener('click', () => setNavigationTool('home'));" in html


def test_cancel_disarms_a_return_while_its_route_is_still_planning_or_link_is_lost():
    run(r"""
const requests=[];
api=async function(path,options){requests.push([path,JSON.parse(options.body)]);return {...homeState,enabled:false,returning:false,phase:'disabled'};};
homeState.enabled=true;homeState.returning=true;homeState.phase='waiting_connection';
robotState.connected=false;navigating=false;
renderHomeUI();
assert.equal($('home-cancel').hidden,false);
assert.equal($('home-cancel').disabled,false);
await cancelHomeReturn();
assert.deepEqual(requests,[['/api/home/settings',{enabled:false}]]);
assert.equal(homeState.enabled,false);
assert.equal(homeState.returning,false);
""")


def test_entire_page_script_has_valid_javascript_syntax():
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node permite comprobar el código JavaScript.")
    script = re.search(r"<script>(.*?)</script>", PAGE.read_text(), re.S).group(1)
    result = subprocess.run([node, "--check"], input=script, text=True, capture_output=True, timeout=10.)
    assert result.returncode == 0, result.stderr
