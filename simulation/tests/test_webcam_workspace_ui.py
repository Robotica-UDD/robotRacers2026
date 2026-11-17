"""Jerarquía de operación y navegación del panel, sin cámara ni robot reales."""
from dataclasses import dataclass, field
from html.parser import HTMLParser
from pathlib import Path
import re
import shutil
import subprocess
import tomllib

import pytest


REPO = Path(__file__).resolve().parents[2]
PAGE = REPO / "simulation/webcam/static/index.html"
TABS = ("route", "camera", "connection")


@dataclass(eq=False)
class Element:
    tag: str
    attrs: dict
    parents: tuple
    text: list = field(default_factory=list)


class Document(HTMLParser):
    def __init__(self, source):
        super().__init__()
        self.elements = []
        self.stack = []
        self.feed(source)

    def handle_starttag(self, tag, attrs):
        node = Element(tag, dict(attrs), tuple(self.stack))
        self.elements.append(node)
        if tag not in {"area", "base", "br", "col", "embed", "hr", "img", "input",
                       "link", "meta", "param", "source", "track", "wbr"}:
            self.stack.append(node)

    def handle_startendtag(self, tag, attrs):
        self.elements.append(Element(tag, dict(attrs), tuple(self.stack)))

    def handle_endtag(self, tag):
        for index in range(len(self.stack) - 1, -1, -1):
            if self.stack[index].tag == tag:
                del self.stack[index:]
                break

    def handle_data(self, data):
        for node in self.stack:
            node.text.append(data)

    def by_id(self, identifier):
        matches = [node for node in self.elements if node.attrs.get("id") == identifier]
        assert len(matches) == 1, (identifier, len(matches))
        return matches[0]


def test_workspace_has_unique_ids_and_one_accessible_tablist():
    document = Document(PAGE.read_text())
    identifiers = [node.attrs["id"] for node in document.elements if "id" in node.attrs]
    assert len(identifiers) == len(set(identifiers))
    tablists = [node for node in document.elements if node.attrs.get("role") == "tablist"]
    assert len(tablists) == 1
    tablist = tablists[0]
    assert tablist.attrs.get("aria-label") or tablist.attrs.get("aria-labelledby")
    selected = []
    for key in TABS:
        tab = document.by_id(f"workspace-tab-{key}")
        panel = document.by_id(f"workspace-panel-{key}")
        assert tab.tag == "button" and tab.attrs.get("type") == "button"
        assert tab.attrs.get("role") == "tab"
        assert tab.attrs.get("aria-controls") == panel.attrs["id"]
        assert tablist in tab.parents
        assert " ".join(tab.text).strip() or tab.attrs.get("aria-label")
        assert panel.attrs.get("role") == "tabpanel"
        assert panel.attrs.get("aria-labelledby") == tab.attrs["id"]
        active = tab.attrs.get("aria-selected") == "true"
        assert tab.attrs.get("tabindex") == ("0" if active else "-1")
        assert ("hidden" in panel.attrs) is not active
        if active:
            selected.append(key)
    assert selected == ["route"]


def test_primary_actions_are_unique_in_header_and_map_tools_stay_with_scene():
    document = Document(PAGE.read_text())
    start = document.by_id("navigation-start")
    stop = document.by_id("robot-stop")
    shared_headers = [node for node in start.parents if node.tag == "header" and node in stop.parents]
    assert shared_headers, "Iniciar y detener deben poder localizarse juntos en la cabecera."
    assert " ".join(stop.text).strip() or stop.attrs.get("aria-label")
    assert start.attrs.get("type") == stop.attrs.get("type") == "button"
    viewport = document.by_id("viewport")
    scene_sections = [node for node in viewport.parents if node.tag == "section"]
    for identifier in ("navigation-point-mode", "navigation-obstacle-mode",
                       "navigation-home-mode", "navigation-calibrate"):
        control = document.by_id(identifier)
        assert any(node in control.parents for node in scene_sections), identifier


def test_workspace_style_is_local_and_included_in_installable_package():
    document = Document(PAGE.read_text())
    stylesheets = [node.attrs.get("href") for node in document.elements
                   if node.tag == "link" and node.attrs.get("rel") == "stylesheet"]
    assert stylesheets.count("/assets/workspace.css") == 1
    stylesheet = REPO / "simulation/webcam/assets/workspace.css"
    assert stylesheet.is_file()
    assert "sticky" in stylesheet.read_text()
    configuration = tomllib.loads((REPO / "pyproject.toml").read_text())
    patterns = configuration["tool"]["setuptools"]["package-data"]["simulation.webcam"]
    assert any(stylesheet.match(pattern) for pattern in patterns)


def test_direct_dom_lookups_still_reference_existing_elements_after_reorganization():
    page = PAGE.read_text()
    document = Document(page)
    identifiers = {node.attrs["id"] for node in document.elements if "id" in node.attrs}
    script = re.search(r"<script>(.*?)</script>", page, re.S).group(1)
    referenced = set(re.findall(r"\$\('([A-Za-z][\w-]*)'\)", script))
    assert referenced <= identifiers, sorted(referenced - identifiers)


def functions(*names):
    page = PAGE.read_text()
    result = []
    for name in names:
        match = re.search(r"    (?:async )?function " + name + r"\([^\n]*\) \{.*?\n    \}", page, re.S)
        assert match, name
        result.append(match.group())
    return "\n".join(result)


def node(script, *, prelude="", names=()):
    executable = shutil.which("node")
    if executable is None:
        pytest.skip("Node permite revisar interacción sin activar hardware.")
    source = prelude + functions(*names) + "\n(async()=>{\n" + script + "\n})().catch(error=>{console.error(error);process.exit(1)});"
    result = subprocess.run([executable, "--input-type=commonjs"], input=source,
                            text=True, capture_output=True, timeout=10.)
    assert result.returncode == 0, result.stderr or result.stdout


TAB_HARNESS = r"""
const assert=require('node:assert/strict');
class Element {
  constructor(id) {this.id=id;this.hidden=false;this.disabled=false;this.textContent='';this.tabIndex=-1;this.attributes={};this.dataset={};this.focusCalls=0;this.scrollCalls=0}
  setAttribute(key,value) {this.attributes[key]=String(value)}
  removeAttribute(key) {delete this.attributes[key]}
  focus() {document.activeElement=this;this.focusCalls++}
  scrollIntoView() {this.scrollCalls++}
}
const elements=new Map();
const $=id=>{if(!elements.has(id))elements.set(id,new Element(id));return elements.get(id)};
const document={activeElement:null};
const requests=[];const api=(...args)=>{requests.push(args);throw new Error('Cambiar panel no debe solicitar acciones')};
let workspacePanel='route';
for(const key of ['route','camera','connection']){
  $(`workspace-tab-${key}`).setAttribute('aria-selected',String(key==='route'));
  $(`workspace-tab-${key}`).tabIndex=key==='route'?0:-1;
  $(`workspace-panel-${key}`).hidden=key!=='route';
}
let connected=true,robotStopBusy=false,calibrationStage='';
let state={running:true,visible:true},robotState={connected:true,ready:true,encoder_ready:true,velocity_control:true};
let navigationState={active:false,planning_status:'ready',waypoints_px:[[40,50]],home_watch_armed:false};
let homeState={enabled:false},fresh=true,calibrated=true,canStart=true;
let speedPending=null,speedInFlight=false,obstacleDraft=[],navigationTool='points',navigationBusy='';
let navigationAvailable=true,robotAvailable=true,homeAvailable=true;
const navigationOwnsRobot=()=>!!navigationState.active;
const cameraVisionFresh=()=>fresh;
const navigationCalibrated=()=>calibrated;
const navigationWaypoints=()=>navigationState.waypoints_px||[];
const canStartNavigation=()=>canStart;
const routePointsPending=()=>false;
const robotSupportsSpeedSettings=()=>true;
const beginCalibration=()=>{throw new Error('Una lectura de estado no inicia calibración')};
const setNavigationTool=()=>{throw new Error('Una lectura de estado no cambia herramienta')};
"""


def test_workspace_switch_changes_only_selected_tab_and_never_sends_requests():
    node(r"""
const source=JSON.stringify({state,robotState,navigationState,homeState});
assert.equal(setWorkspacePanel('camera'),true);
assert.equal(workspacePanel,'camera');
assert.equal($('workspace-tab-camera').attributes['aria-selected'],'true');
assert.equal($('workspace-tab-camera').tabIndex,0);
assert.equal($('workspace-panel-camera').hidden,false);
for(const key of ['route','connection']){
  assert.equal($(`workspace-tab-${key}`).attributes['aria-selected'],'false');
  assert.equal($(`workspace-tab-${key}`).tabIndex,-1);
  assert.equal($(`workspace-panel-${key}`).hidden,true);
}
assert.equal(document.activeElement,null);
assert.equal(setWorkspacePanel('invalid',true),false);
assert.equal(workspacePanel,'camera');
assert.deepEqual(requests,[]);
assert.equal(JSON.stringify({state,robotState,navigationState,homeState}),source);
""", prelude=TAB_HARNESS, names=("setWorkspacePanel",))


def test_workspace_tab_keys_wrap_and_move_focus_without_using_map_or_motor_actions():
    node(r"""
function key(current,name){let prevented=false;handleWorkspaceTabKeydown({currentTarget:$(`workspace-tab-${current}`),key:name,preventDefault(){prevented=true}});return prevented}
assert.equal(key('route','ArrowRight'),true);assert.equal(workspacePanel,'camera');
assert.equal(document.activeElement.id,'workspace-tab-camera');
assert.equal(key('camera','ArrowRight'),true);assert.equal(workspacePanel,'connection');
assert.equal(key('connection','ArrowRight'),true);assert.equal(workspacePanel,'route');
assert.equal(key('route','ArrowLeft'),true);assert.equal(workspacePanel,'connection');
assert.equal(key('connection','Home'),true);assert.equal(workspacePanel,'route');
assert.equal(key('route','End'),true);assert.equal(workspacePanel,'connection');
assert.equal(key('connection','Escape'),false);assert.equal(workspacePanel,'connection');
assert.equal(key('connection','Enter'),false);
let prevented=false;handleWorkspaceTabKeydown({currentTarget:{id:'robot-ssid'},key:'Home',preventDefault(){prevented=true}});
assert.equal(prevented,false);
assert.deepEqual(requests,[]);
""", prelude=TAB_HARNESS, names=("setWorkspacePanel", "handleWorkspaceTabKeydown"))


def test_status_updates_keep_selected_panel_and_explain_missing_vision_without_requests():
    node(r"""
setWorkspacePanel('connection',true);
fresh=false;updateWorkspaceChrome();
assert.equal(workspacePanel,'connection');
assert.equal($('workspace-panel-connection').hidden,false);
assert.equal(document.activeElement.id,'workspace-tab-connection');
assert.equal($('workspace-setup-action').dataset.action,'camera');
assert.match($('workspace-next-step').textContent,/marcador|iluminaci/i);
homeState={enabled:true,curious:true,phase:'curious'};updateWorkspaceChrome();
assert.match($('workspace-hand-status').textContent,/curiosidad/i);
robotStopBusy=true;updateWorkspaceChrome();
assert.match($('workspace-next-step').textContent,/deteniendo/i);
assert.equal($('workspace-setup-action').hidden,true);
assert.deepEqual(requests,[]);
""", prelude=TAB_HARNESS, names=("setWorkspacePanel", "updateWorkspaceChrome"))


@pytest.mark.parametrize("condition", ["blocked", "speed_pending", "encoder_unavailable"])
def test_workspace_never_announces_ready_when_start_conditions_are_not_met(condition):
    changes = {
        "blocked": "navigationState.planning_status='blocked';navigationState.planning_message='No existe una ruta libre';",
        "speed_pending": "speedPending={forward_mm_s:240};speedInFlight=true;",
        "encoder_unavailable": "robotState.encoder_ready=false;robotState.velocity_control=false;",
    }
    node(changes[condition] + r"""
canStart=false;
updateWorkspaceChrome();
assert.ok($('workspace-next-step').textContent.length>0);
assert.doesNotMatch($('workspace-next-step').textContent,/Recorrido preparado|pulsa Iniciar recorrido/i);
assert.deepEqual(requests,[]);
""", prelude=TAB_HARNESS, names=("updateWorkspaceChrome",))


def test_marker_size_selector_accepts_supported_patterns_and_change_only_marks_local_choice():
    page = PAGE.read_text()
    document = Document(page)
    selector = document.by_id("marker-size")
    options = [node.attrs.get("value") for node in document.elements
               if node.tag == "option" and selector in node.parents]
    assert options == ["40", "52"]
    handler = re.search(r"\$\('marker-size'\)\.addEventListener\('change', \(\) => \{([^}]+)\}\)", page)
    assert handler
    node(r"""
for(const [value,expected] of [['40',40],['52',52],['65',40],['NaN',40],['',40]]){
  $('marker-size').value=value;
  assert.equal(selectedMarkerSizeMm(),expected);
}
$('marker-size').value='52';
""" + handler.group(1) + r"""
assert.equal(markerSizeTouched,true);
assert.equal(selectedMarkerSizeMm(),52);
assert.deepEqual(requests,[]);
""", prelude=TAB_HARNESS + "\nlet markerSizeTouched=false;\n",
         names=("selectedMarkerSizeMm",))


def test_marker_size_render_preserves_manual_choice_and_locks_during_calibration_or_motion():
    node(r"""
navigationState.calibration_geometry={marker_size_m:.052};
$('marker-size').value='40';renderMarkerSize();
assert.equal($('marker-size').value,'52');
assert.equal($('marker-size').disabled,false);
markerSizeTouched=true;$('marker-size').value='40';renderMarkerSize();
assert.equal($('marker-size').value,'40');
calibrationStage='front';renderMarkerSize();assert.equal($('marker-size').disabled,true);
calibrationStage='';navigationState.active=true;renderMarkerSize();assert.equal($('marker-size').disabled,true);
navigationState.active=false;homeState.enabled=true;renderMarkerSize();assert.equal($('marker-size').disabled,true);
homeState.enabled=false;renderMarkerSize();assert.equal($('marker-size').disabled,false);
assert.equal($('marker-size').value,'40');
assert.deepEqual(requests,[]);
""", prelude=TAB_HARNESS + "\nlet markerSizeTouched=false;\n",
         names=("renderMarkerSize",))


def test_selected_marker_size_is_persisted_and_sent_only_after_both_calibration_clicks():
    prelude = TAB_HARNESS.replace(
        "const beginCalibration=()=>{throw new Error('Una lectura de estado no inicia calibración')};", ""
    ) + r"""
let markerSizeTouched=true,calibrationAnchor=null,calibrationSourceKey='';
let calibrationCenter=null,calibrationCenterTag=null,calibrationProjectionReady=false;
let obstacleDraftSource='',obstaclePointer=null,routeEditGeneration=0,routePointQueue=[];
let uiRenders=0,calibrationError='';
const canEditNavigation=()=>true;
const cancelObstacleDraft=()=>{obstacleDraft=[]};
const setNavigationError=value=>{calibrationError=value};
const updateNavigationUI=()=>{uiRenders++};
const viewport={getBoundingClientRect:()=>({left:0,top:0,width:640,height:480})};
const storage=new Map();
const sessionStorage={getItem:key=>storage.get(key)||null,setItem:(key,value)=>storage.set(key,value)};
const calibrationRequests=[];
const navigationPost=async(action,body)=>{calibrationRequests.push({action,body:JSON.parse(JSON.stringify(body))});return true};
state={running:true,visible:true,camera_id:'webcam-usb',frame_width:640,frame_height:480,
       heading_deg:0,marker_corners:[[280,200],[360,200],[360,280],[280,280]]};
"""
    node(r"""
$('marker-size').value='52';
beginCalibration();
assert.equal(calibrationStage,'center');
assert.equal(calibrationAnchor.metadata.marker_size_mm,52);
let saved=JSON.parse(sessionStorage.getItem('robotracers.calibration-drafts'));
assert.equal(saved[cameraKey()].metadata.marker_size_mm,52);
assert.deepEqual(calibrationRequests,[]);assert.deepEqual(requests,[]);
await selectNavigationPoint({clientX:320,clientY:240});
assert.equal(calibrationStage,'front');
assert.deepEqual(calibrationRequests,[]);
saved=JSON.parse(sessionStorage.getItem('robotracers.calibration-drafts'));
assert.equal(saved[cameraKey()].stage,'front');
assert.equal(saved[cameraKey()].metadata.marker_size_mm,52);
// Aunque un cambio externo altere el select, los dos clics usan el mismo anclaje.
$('marker-size').value='40';
await selectNavigationPoint({clientX:350,clientY:240});
assert.equal(calibrationRequests.length,1);
assert.ok(Math.abs(calibrationRequests[0].body.center_px[0]-320)<1e-8);
assert.ok(Math.abs(calibrationRequests[0].body.center_px[1]-240)<1e-8);
assert.deepEqual({...calibrationRequests[0],body:{...calibrationRequests[0].body,center_px:calibrationRequests[0].body.center_px.map(Math.round)}},{action:'calibrate',body:{center_px:[320,240],front_px:[350,240],
  frame_width:640,frame_height:480,camera_id:'webcam-usb',marker_size_mm:52}});
assert.equal(calibrationStage,'');
assert.deepEqual(JSON.parse(sessionStorage.getItem('robotracers.calibration-drafts')),{});
assert.deepEqual(requests,[]);
""", prelude=prelude, names=("selectedMarkerSizeMm", "finitePoint", "cameraKey", "markerPose",
        "calibrationDrafts", "persistCalibrationDraft", "synchronizeCalibrationSource", "mapQuadPoint",
        "checkCalibrationAnchor", "beginCalibration", "cancelCalibration", "containedImageGeometry",
        "nativeImagePoint", "selectNavigationPoint"))


STOP_HARNESS = r"""
const assert=require('node:assert/strict');
class Element {
  constructor() { this.disabled=false;this.hidden=false;this.value='';this.textContent='';this.attributes={};this.dataset={}; }
  setAttribute(key,value) {this.attributes[key]=String(value)}
  removeAttribute(key) {delete this.attributes[key]}
}
const elements=new Map();
const $=id=>{if(!elements.has(id))elements.set(id,new Element());return elements.get(id)};
let robotStopBusy=false,robotAvailable=true,navigationAvailable=true,homeAvailable=true;
let robotState={connected:true,moving:true},navigationState={active:true},homeState={enabled:true};
let robotConnectBusy=false,robotPulseSubmitting=false,robotPulsePending=true,robotPulseExpectedId='pulse',robotPulseResponseReceived=false;
let navigationStartPending=true,navigationBusy='',navigationTool='points',calibrationStage='';
let routeEditGeneration=2,routePointQueue=[{point:[1,2]}],obstacleDraft=[[3,4]];
let navigationActionVersion=3,navigationAppliedVersion=0,navigationReadVersion=4;
let robotAppliedVersion=0,robotReadVersion=5;
let homeActionVersion=0,homeAppliedVersion=0,homeReadVersion=0,homeBusy='';
let navReads=0,homeReads=0,robotRenders=0,navRenders=0,chromeUpdates=0,buttonUpdates=0;
let robotError='';
function setRobotError(message) {robotError=message;$('robot-error').textContent=message;$('robot-error').hidden=!message}
function cancelObstacleDraft() {obstacleDraft=[]}
function updateRobotButtons() {buttonUpdates++}
function updateNavigationUI() {}
function updateWorkspaceChrome() {chromeUpdates++}
function renderHomeUI() {}
function renderRobot(value) {robotState=value;robotRenders++}
function renderNavigation(value) {navigationState=value;navRenders++}
function renderHome(value) {homeState=value}
function readNavigationState() {navReads++;return Promise.resolve()}
function readHomeState() {homeReads++;return Promise.resolve()}
function navigationOwnsRobot() {return navigationState?.active||navigationStartPending}
let api;
"""


def test_global_stop_attempts_both_paths_in_parallel_and_invalidates_edits_immediately():
    node(r"""
const requests=[];let resolveNavigation,rejectRobot;
api=(path,options)=>{
  requests.push({path,options,body:JSON.parse(options.body)});
  return new Promise((resolve,reject)=>{if(path==='/api/navigation/stop')resolveNavigation=resolve;else if(path==='/api/robot/stop')rejectRobot=reject;else throw new Error(path)});
};
const operation=stopRobot();
assert.deepEqual(requests.map(x=>x.path).sort(),['/api/navigation/stop','/api/robot/stop']);
assert.deepEqual(routePointQueue,[]);assert.deepEqual(obstacleDraft,[]);
assert.ok(routeEditGeneration>2);assert.ok(navigationActionVersion>3);
assert.equal(robotStopBusy,true);
await stopRobot();assert.equal(requests.length,2);
for(const request of requests){assert.equal(request.options.method,'POST');assert.equal(request.options.headers['X-Requested-With'],'XMLHttpRequest');assert.deepEqual(request.body,{})}
resolveNavigation({active:false,phase:'stopped'});rejectRobot(new Error('Sin confirmación física'));
await operation;
assert.equal(robotStopBusy,false);assert.equal(navigationStartPending,false);
assert.ok(robotError.length>0,'La ausencia de confirmación física debe seguir visible.');
assert.ok(navReads>0);assert.ok(homeReads>0);assert.ok(buttonUpdates>0);
assert.equal(requests.some(x=>/\/start|\/pulse/.test(x.path)),false);
""", prelude=STOP_HARNESS, names=("stopRobot",))


def test_global_stop_keeps_robot_attempt_when_navigation_request_fails():
    node(r"""
const requests=[];
api=async(path)=>{requests.push(path);if(path==='/api/navigation/stop')throw new Error('Ruta no disponible');return {connected:true,moving:false,ready:true}};
await stopRobot();
assert.deepEqual(requests.sort(),['/api/navigation/stop','/api/robot/stop']);
assert.equal(robotStopBusy,false);assert.equal(robotPulsePending,false);
assert.ok(navReads>0);assert.ok(homeReads>0);
""", prelude=STOP_HARNESS, names=("stopRobot",))


def test_global_stop_cancels_local_reaction_even_when_camera_and_wifi_are_lost():
    node(r"""
robotAvailable=false;robotState={connected:false,moving:false};
navigationState={active:false,phase:'planning'};homeState={enabled:true,curious:true,phase:'curious'};
navigationStartPending=false;
const state={running:false,visible:false};
const requests=[];
api=async(path)=>{requests.push(path);if(path==='/api/robot/stop')throw new Error('Robot sin enlace');return {active:false,phase:'stopped'}};
await stopRobot();
assert.deepEqual(requests.sort(),['/api/navigation/stop','/api/robot/stop']);
assert.equal(robotStopBusy,false);assert.ok(robotError.length>0);
assert.ok(navReads>0);assert.ok(homeReads>0);
""", prelude=STOP_HARNESS, names=("stopRobot",))


def test_stop_remains_idempotent_when_disabled_and_reports_total_failure():
    node(r"""
const requests=[];
api=async(path)=>{requests.push(path);throw new Error('Servicio no disponible')};
$('robot-stop').disabled=true;await stopRobot();assert.deepEqual(requests,[]);
$('robot-stop').disabled=false;await stopRobot();
assert.deepEqual(requests.sort(),['/api/navigation/stop','/api/robot/stop']);
assert.equal(robotStopBusy,false);assert.ok(robotError.length>0);
assert.equal(navigationStartPending,false);
""", prelude=STOP_HARNESS, names=("stopRobot",))


def test_general_stop_stays_available_during_lost_link_hand_planning_and_pending_pulse():
    prelude = STOP_HARNESS.replace("function updateRobotButtons() {buttonUpdates++}", "") + r"""
let robotPortsLoading=false,illuminationInFlight=true,busy=true;
const state={running:false,visible:false};
const robotIsConfiguring=()=>false;
const navigationAllowsReconnect=()=>true;
"""
    node(r"""
robotAvailable=false;navigationAvailable=true;homeAvailable=false;
robotState={connected:false,ready:false,moving:false};
robotPulsePending=false;navigationStartPending=false;homeState={enabled:false};
navigationState={active:true,phase:'paused_connection'};
updateRobotButtons();assert.equal($('robot-stop').disabled,false);
navigationState={active:false};homeAvailable=true;
homeState={enabled:true,curious:true,phase:'curious'};
updateRobotButtons();assert.equal($('robot-stop').disabled,false);
homeState={enabled:false};homeAvailable=false;navigationAvailable=false;
robotPulsePending=true;
updateRobotButtons();assert.equal($('robot-stop').disabled,false);
robotStopBusy=true;updateRobotButtons();assert.equal($('robot-stop').disabled,true);
robotStopBusy=false;robotPulsePending=false;
updateRobotButtons();assert.equal($('robot-stop').disabled,true);
""", prelude=prelude, names=("updateRobotButtons",))


def test_turn_capability_uses_the_current_track_from_navigation():
    node(r"""
assert.equal(robotSupportsSpeedSettings(),true); // 90°/s exige128mm/s por rueda.
navigationState.track_width_mm=190;
assert.equal(robotSupportsSpeedSettings(),false); // La trocha anterior exigía150mm/s.
navigationState={};
assert.equal(robotSupportsSpeedSettings(),true); // Respaldo local de163mm.
""", prelude=r"""
const assert=require('node:assert/strict');
const robotState={max_velocity_mm_s:130};
const confirmedSpeeds={forward_mm_s:50,turn_deg_s:90};
let navigationState={track_width_mm:163};
""", names=("robotSupportsSpeedSettings",))
