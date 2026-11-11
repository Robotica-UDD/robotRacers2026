"""Representación persistente de la ruta y centro calibrado, sin navegador ni hardware."""
from pathlib import Path
import re
import shutil
import subprocess

import pytest


HTML = (Path(__file__).parents[1] / "webcam/static/index.html").read_text()


def browser_functions(*names):
    functions = []
    for name in names:
        start = HTML.index(f"    function {name}(")
        tail = HTML[start:]
        end = re.search(r"\n    (?:async )?function ", tail[1:])
        functions.append(tail if end is None else tail[:end.start() + 1])
    return "\n".join(functions)


def run_javascript(script):
    node = shutil.which("node")
    if node is None:
        pytest.skip("La prueba del renderizado necesita Node.js.")
    result = subprocess.run([node, "--input-type=commonjs"], input=script, text=True,
                            capture_output=True, timeout=10)
    assert result.returncode == 0, result.stderr or result.stdout


DOM = """
const assert = require('node:assert/strict');
class Element {
  constructor() { this.attributes = {}; this.children = []; this.textContent = ''; }
  setAttribute(k, v) { this.attributes[k] = String(v); }
  removeAttribute(k) { delete this.attributes[k]; }
  appendChild(v) { this.children.push(v); }
  removeChild(v) { this.children.splice(this.children.indexOf(v), 1); }
  get lastElementChild() { return this.children.at(-1); }
}
const elements = new Map();
const $ = id => { if (!elements.has(id)) elements.set(id, new Element()); return elements.get(id); };
const document = {createElementNS: () => new Element()};
"""


def test_visual_route_survives_recalculation_updates_from_center_and_clears_explicitly():
    script = DOM + browser_functions("finitePoint", "setSvgVisible", "drawPolygonGroup",
                                    "navigationFrameOverlayEnabled", "drawObstacleOverlay") + """
let state = {navigation_overlay_in_frame: false};
let navigationCameraKey = 'same-camera';
const cameraKey = () => 'same-camera';
const navigationObstacles = () => navigationState.obstacles_px;
let obstacleDraftSource = '', obstacleDraft = [], obstaclePointer = null;
let navigationState = {
  planning_status: 'ready', collision_model: 'oriented_footprint',
  obstacle_settings: {safety_margin_mm: 0},
  obstacles_px: [[[100,100],[120,100],[120,120],[100,120]]],
  display_path_px: [[10,20],[50,80],[180,130]], planned_path_px: []
};
let layer = drawObstacleOverlay(1, [22,33]);
const line = $('navigation-planned-path');
assert.equal(line.attributes.points, '22,33 50,80 180,130');
assert.equal('hidden' in line.attributes, false);
assert.deepEqual(navigationState.display_path_px[0], [10,20]);
navigationState.planning_status = 'waiting';
layer = drawObstacleOverlay(1, [24,35]);
assert.equal(line.attributes.points, '24,35 50,80 180,130');
assert.equal('hidden' in line.attributes, false);
state.navigation_overlay_in_frame = true;
drawObstacleOverlay(1, [24,35]);
assert.equal('hidden' in line.attributes, true);
assert.equal($('navigation-path-dots').children.length, 0);
assert.equal($('navigation-obstacles').children.length, 1);
state.navigation_overlay_in_frame = false;
navigationState.display_path_px = [];
navigationState.planned_path_px = [[10,20],[99,99]];
drawObstacleOverlay(1, [24,35]);
assert.equal('hidden' in line.attributes, true);
navigationState.display_path_px = [[10,20],[50,80]];
navigationCameraKey = 'another-camera';
drawObstacleOverlay(1, [24,35]);
assert.equal('hidden' in line.attributes, true);
"""
    run_javascript(script)


def test_center_measurement_uses_calibration_and_never_falls_back_to_marker_when_calibrated():
    script = DOM + browser_functions("finitePoint", "mapQuadPoint",
                                    "calibratedCenterInImage", "renderCenterMeasurement") + """
let visible = true, calibrated = true, navigationAvailable = true;
const cameraVisionFresh = () => visible;
const navigationCalibrated = () => calibrated;
let state = {center_px: [300,300], marker_corners: [[280,280],[320,280],[320,320],[280,320]]};
let navigationState = {center_px: [260,300], calibration_geometry: {
  center_tag_m: [-.06,.02], marker_size_m: .04, front_angle_rad: 0
}};
renderCenterMeasurement();
assert.equal($('center-label').textContent, 'Centro del robot');
assert.equal($('center').textContent, '240, 280');
visible = false;
renderCenterMeasurement();
assert.equal($('center').textContent, '—');
visible = true; calibrated = false;
renderCenterMeasurement();
assert.equal($('center-label').textContent, 'Centro del marcador');
assert.equal($('center').textContent, '300, 300');
"""
    run_javascript(script)
