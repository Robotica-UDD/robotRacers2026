"""Dibujo real del perímetro: el interior no se convierte en obstáculo."""
from simulation.tests.test_webcam_home_ui import PAGE, functions
from simulation.tests.test_webcam_route_overlay import run_javascript


HARNESS = r"""
const assert = require('node:assert/strict');
class Element {
  constructor(tag='g') { this.tag=tag;this.attributes={};this.children=[];this.dataset={};this.textContent='';this.hidden=false;this.disabled=false; }
  setAttribute(k,v) { this.attributes[k]=String(v); }
  removeAttribute(k) { delete this.attributes[k]; }
  appendChild(child) { this.children.push(child); }
  removeChild(child) { this.children.splice(this.children.indexOf(child),1); }
  get lastElementChild() { return this.children.at(-1); }
}
const elements=new Map();
const $=id=>{if(!elements.has(id))elements.set(id,new Element());return elements.get(id);};
const document={createElementNS:(_,tag)=>new Element(tag)};
const viewport={getBoundingClientRect:()=>({left:0,top:0,width:640,height:360})};
let state={camera_id:'usb',frame_width:1280,frame_height:720};
const cameraKey=()=>`${state.camera_id}:${state.frame_width}:${state.frame_height}`;
let navigationCameraKey=cameraKey(),navigationAvailable=true,navigationBusy='',homeBusy='';
let navigationState={obstacles_px:[],route_revision:7,planning_status:'empty',obstacle_settings:{safety_margin_mm:0}};
let navigationTool='obstacle',obstacleDraft=[],obstaclePointer=null,obstacleDraftSource='';
let routeEditGeneration=0,routePointQueue=[],updates=0,error='',editable=true;
let obstacleMarginEditing=false,obstacleMarginPending=false;
const canEditObstacles=()=>editable;
const navigationRouteRevision=()=>navigationState.route_revision;
const updateNavigationUI=()=>{updates++};
const setNavigationError=message=>{error=message};
const requests=[];
let accepted=true;
const navigationPost=async(action,payload)=>{
  requests.push([action,payload]);
  if(accepted)navigationState.obstacles_px=payload.polygons_px;
  return accepted;
};
"""


COMMON = ("finitePoint", "navigationObstacles", "cancelObstacleDraft", "undoObstaclePoint",
          "handleMapKeydown", "obstaclePerimeterValid", "obstacleCloseRequested",
          "appendObstaclePoint", "containedImageGeometry", "setSvgVisible",
          "drawPolygonGroup", "navigationFrameOverlayEnabled", "drawObstacleOverlay",
          "renderObstacleTools")


def run(script, *, extra=(), prelude=""):
    run_javascript(HARNESS + prelude + functions(*COMMON, *extra) +
                   "\n(async()=>{\n" + script +
                   "\n})().catch(error=>{console.error(error);process.exit(1)});")


def test_four_vertices_do_not_save_until_a_click_within_the_screen_radius_of_a():
    run(r"""
const previous=[[10,10],[30,10]];
navigationState.obstacles_px=[previous];
const vertices=[[100,100],[400,100],[400,400],[100,400]];
for(const point of vertices)await appendObstaclePoint(point);
assert.equal(requests.length,0);
assert.deepEqual(obstacleDraft,vertices);
assert.equal(obstacleCloseRequested([129,100]),false);
assert.equal(obstacleCloseRequested([125,100]),true);
await appendObstaclePoint([125,100]);
assert.equal(requests.length,1);
assert.equal(requests[0][0],'obstacles');
assert.deepEqual(requests[0][1],{
  polygons_px:[previous,[[100,100],[400,100]],[[400,100],[400,400]],[[400,400],[100,400]],[[100,400],[100,100]]],
  frame_width:1280,frame_height:720,camera_id:'usb',expected_route_revision:7
});
assert.equal(requests[0][1].polygons_px.every(edge=>edge.length===2),true);
assert.deepEqual(obstacleDraft,[]);
assert.equal(navigationTool,'points');
""")


def test_a_concave_perimeter_is_saved_as_its_edges_without_filling_the_interior():
    run(r"""
const vertices=[[100,100],[500,100],[500,500],[300,500],[300,300],[100,300]];
assert.equal(obstaclePerimeterValid(vertices),true);
for(const point of vertices)await appendObstaclePoint(point);
assert.equal(requests.length,0);
await appendObstaclePoint(vertices[0]);
assert.equal(requests[0][1].polygons_px.length,6);
assert.deepEqual(requests[0][1].polygons_px.at(-1),[vertices.at(-1),vertices[0]]);
assert.equal(obstaclePerimeterValid([[100,100],[400,400],[100,400],[400,100]]),false);
assert.equal(obstaclePerimeterValid([[100,100],[200,100]]),false);
assert.equal(obstaclePerimeterValid([[100,100],[200,100],[300,100]]),false);
""")


def test_self_crossing_close_keeps_the_draft_so_it_can_be_corrected():
    run(r"""
for(const point of [[100,100],[400,400],[100,400],[400,100]])await appendObstaclePoint(point);
await appendObstaclePoint([100,100]);
assert.equal(requests.length,0);
assert.equal(obstacleDraft.length,4);
assert.match(error,/se cruza/);
undoObstaclePoint();
assert.equal(obstacleDraft.length,3);
assert.equal(error,'');
await appendObstaclePoint([100,100]);
assert.equal(requests.length,1);
""")


def test_limits_count_existing_segments_and_allow_at_most_128_new_vertices():
    run(r"""
navigationState.obstacles_px=Array.from({length:509},()=>[[10,10],[30,10]]);
for(const point of [[100,100],[400,100],[400,400]])await appendObstaclePoint(point);
await appendObstaclePoint([100,400]);
assert.equal(obstacleDraft.length,3);
assert.equal(requests.length,0);
await appendObstaclePoint([100,100]);
assert.equal(requests[0][1].polygons_px.length,512);
navigationTool='obstacle';
await appendObstaclePoint([100,100]);
assert.equal(obstacleDraft.length,0);
assert.equal(requests.length,1);
navigationState.obstacles_px=[];
obstacleDraft=Array.from({length:128},(_,i)=>[640+300*Math.cos(i*2*Math.PI/128),360+300*Math.sin(i*2*Math.PI/128)]);
obstacleDraftSource=cameraKey();
await appendObstaclePoint([600,500]);
assert.equal(obstacleDraft.length,128);
await appendObstaclePoint(obstacleDraft[0]);
assert.equal(requests[1][1].polygons_px.length,128);
""")


def test_undo_and_escape_cancel_only_the_drawing_without_sending_any_robot_action():
    run(r"""
const saved=[[10,10],[30,10]];
navigationState.obstacles_px=[saved];
for(const point of [[100,100],[400,100],[400,400]])await appendObstaclePoint(point);
let prevented=0;
handleMapKeydown({key:'z',metaKey:true,target:{matches:()=>false},preventDefault:()=>prevented++});
assert.equal(obstacleDraft.length,2);
assert.equal(prevented,1);
handleMapKeydown({key:'Escape',preventDefault:()=>prevented++});
assert.deepEqual(obstacleDraft,[]);
assert.equal(obstaclePointer,null);
assert.equal(obstacleDraftSource,'');
assert.equal(navigationTool,'points');
assert.deepEqual(navigationObstacles(),[saved]);
assert.equal(requests.length,0);
assert.equal(prevented,2);
""")
    html = PAGE.read_text()
    assert "addEventListener('click', undoObstaclePoint)" in html
    assert "addEventListener('keydown', handleMapKeydown)" in html


def test_failed_save_keeps_every_vertex_and_waits_for_a_confirmed_map_revision():
    run(r"""
const vertices=[[100,100],[400,100],[400,400],[100,400]];
for(const point of vertices)await appendObstaclePoint(point);
navigationState.route_revision=null;
await appendObstaclePoint(vertices[0]);
assert.equal(requests.length,0);
assert.deepEqual(obstacleDraft,vertices);
navigationState.route_revision=8;accepted=false;
await appendObstaclePoint(vertices[0]);
assert.equal(requests.length,1);
assert.deepEqual(obstacleDraft,vertices);
assert.equal(navigationTool,'obstacle');
""")


def test_a_new_camera_drops_the_previous_draft_before_accepting_a_vertex():
    run(r"""
for(const point of [[100,100],[400,100],[400,400]])await appendObstaclePoint(point);
state.camera_id='other';
await appendObstaclePoint([600,600]);
assert.deepEqual(obstacleDraft,[[600,600]]);
assert.equal(obstacleDraftSource,cameraKey());
assert.equal(requests.length,0);
""")


def test_camera_source_synchronization_cancels_drawing_and_prevents_an_old_save():
    prelude = r"""
let calibrationSourceKey=cameraKey(),calibrationStage='',calibrationAnchor=null;
let calibrationCenter=null,calibrationCenterTag=null,calibrationProjectionReady=false;
const calibrationDrafts=()=>({}),persistCalibrationDraft=()=>{};
"""
    run(r"""
for(const point of [[100,100],[400,100],[400,400]])await appendObstaclePoint(point);
state.frame_width=1920;state.frame_height=1080;
synchronizeCalibrationSource();
assert.deepEqual(obstacleDraft,[]);
assert.equal(navigationTool,'points');
assert.equal(obstacleDraftSource,'');
assert.equal(routeEditGeneration,1);
assert.equal(requests.length,0);
""", prelude=prelude, extra=("synchronizeCalibrationSource",))


def test_segments_render_as_unfilled_lines_and_legacy_polygons_remain_compatible():
    run(r"""
navigationState.obstacles_px=[[[100,100],[400,100]],[[10,10],[30,10],[30,30],[10,30]]];
drawObstacleOverlay(.5);
const shapes=$('navigation-obstacles').children;
assert.equal(shapes.length,2);
assert.equal(shapes[0].tag,'polyline');
assert.match(shapes[0].attributes.class,/navigation-barrier/);
assert.equal(shapes[0].attributes.points,'100,100 400,100');
assert.equal(shapes[1].tag,'polygon');
navigationState.obstacles_px.reverse();
drawObstacleOverlay(.5);
assert.equal($('navigation-obstacles').children[0].tag,'polygon');
assert.equal($('navigation-obstacles').children[1].tag,'polyline');
""")
    html = PAGE.read_text()
    assert '.navigation-obstacle.navigation-barrier{fill:none;' in html
    assert '.navigation-obstacle-draft{fill:none;' in html
    assert '.navigation-obstacle-clearance{fill:none;' in html
    assert '.navigation-obstacle-draft{fill:#' not in html


def test_draft_remains_open_until_hovering_over_a_and_uses_a_screen_size_close_ring():
    run(r"""
obstacleDraft=[[100,100],[400,100],[400,400],[100,400]];
obstacleDraftSource=cameraKey();
drawObstacleOverlay(.5);
const line=$('navigation-obstacle-draft');
assert.equal(line.attributes.points,'100,100 400,100 400,400 100,400');
assert.equal($('navigation-obstacle-close-radius').attributes.r,'28');
assert.equal($('navigation-obstacle-start-label').textContent,'A');
obstaclePointer=[125,100];drawObstacleOverlay(.5);
assert.equal(line.attributes.points,'100,100 400,100 400,400 100,400 100,100');
assert.equal($('navigation-obstacle-close-radius').attributes['data-near'],'true');
assert.equal($('navigation-obstacle-start-label').textContent,'Cerrar en A');
state.camera_id='other';drawObstacleOverlay(.5);
assert.equal('hidden' in line.attributes,true);
assert.equal('hidden' in $('navigation-obstacle-start-label').attributes,true);
assert.equal('hidden' in $('navigation-obstacle-close-radius').attributes,true);
""")


def test_tool_instructions_explain_a_closure_instead_of_four_clicks():
    run(r"""
renderObstacleTools();
assert.match($('navigation-obstacle-hint').textContent,/Marca A/);
obstacleDraft=[[100,100],[400,100],[400,400]];
renderObstacleTools();
assert.match($('navigation-obstacle-hint').textContent,/3 vértices · Cerrar en A/);
navigationState.obstacles_px=Array.from({length:510},()=>[[10,10],[30,10]]);
renderObstacleTools();
assert.equal($('navigation-obstacle-mode').disabled,true);
""")
    html = PAGE.read_text()
    assert '>Dibujar perímetro</button>' in html
    assert '>Borrar perímetros</button>' in html
    assert 'con cuatro clics' not in html
