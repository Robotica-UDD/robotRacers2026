"""Ejecuta el dibujo SVG real con un DOM mínimo, sin abrir cámara ni motores."""
from pathlib import Path
import json
import re
import shutil
import subprocess

import pytest


PAGE = Path(__file__).resolve().parents[1] / "webcam" / "static" / "index.html"


def run_overlay(script):
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node permite comprobar el dibujo SVG de la interfaz.")
    html = PAGE.read_text()
    names = ("finitePoint", "setSvgVisible", "drawPolygonGroup", "navigationFrameOverlayEnabled",
             "drawRobotFootprintOverlay", "drawObstacleOverlay")
    functions = []
    for name in names:
        match = re.search(r"    function " + name + r"\([^\n]*\) \{.*?\n    \}", html, re.S)
        assert match, f"No se encontró {name} en la interfaz."
        functions.append(match.group())
    harness = r"""
class Element {
  constructor() { this.attributes = {}; this.children = []; }
  setAttribute(name, value) { this.attributes[name] = String(value); }
  removeAttribute(name) { delete this.attributes[name]; }
  appendChild(child) { this.children.push(child); }
  removeChild(child) { this.children.splice(this.children.indexOf(child), 1); }
  get lastElementChild() { return this.children[this.children.length - 1]; }
}
const elements = new Map();
const $ = id => { if (!elements.has(id)) elements.set(id, new Element()); return elements.get(id); };
const document = {createElementNS: () => new Element()};
let navigationCameraKey = 'recorded-camera';
const cameraKey = () => 'recorded-camera';
let navigationState = null;
let state = {};
const navigationObstacles = () => navigationState?.obstacles_px || [];
let obstacleDraftSource = '', obstacleDraft = [], obstaclePointer = null;
"""
    result = subprocess.run([node, "--input-type=module", "-e",
                             harness + "\n".join(functions) + script],
                            check=True, capture_output=True, text=True, timeout=10.)
    return json.loads(result.stdout)


def test_oriented_footprint_and_heading_are_drawn_then_removed_when_pose_is_lost():
    result = run_overlay(r"""
const polygon = [[20, 10], [50, 10], [60, 35], [20, 40]];
navigationState = {collision_model: 'oriented_footprint', robot_footprint_px: polygon, front_px: [40, 20]};
drawRobotFootprintOverlay([30, 20], 1);
const drawn = {count: $('navigation-robot-footprint').children.length,
  points: $('navigation-robot-footprint').children[0].attributes.points,
  heading: $('navigation-robot-front').attributes.d,
  headingHidden: 'hidden' in $('navigation-robot-front').attributes};
drawRobotFootprintOverlay(null, 1);
console.log(JSON.stringify({drawn, lostCount: $('navigation-robot-footprint').children.length,
  lostHeadingHidden: 'hidden' in $('navigation-robot-front').attributes}));
""")
    assert result["drawn"]["count"] == 1
    assert result["drawn"]["points"] == "20,10 50,10 60,35 20,40"
    assert result["drawn"]["heading"] == "M 33 24 L 40 20 L 33 16"
    assert not result["drawn"]["headingHidden"]
    assert result["lostCount"] == 0
    assert result["lostHeadingHidden"]


def test_footprint_rejects_nonfinite_vertices_and_mismatched_camera():
    result = run_overlay(r"""
navigationState = {collision_model: 'oriented_footprint', robot_footprint_px: [[1, 1], [2, 1], [2, NaN]], front_px: [3, 2]};
const invalid = drawRobotFootprintOverlay([2, 2], 1);
navigationState.robot_footprint_px = [[1, 1], [2, 1], [2, 3]];
navigationCameraKey = 'previous-camera';
const wrongCamera = drawRobotFootprintOverlay([2, 2], 1);
console.log(JSON.stringify({invalid, wrongCamera, count: $('navigation-robot-footprint').children.length}));
""")
    assert result == {"invalid": False, "wrongCamera": False, "count": 0}


def test_zero_margin_pose_mode_hides_inflated_zones_and_keeps_the_planned_route():
    result = run_overlay(r"""
navigationState = {collision_model: 'oriented_footprint', obstacle_settings: {safety_margin_mm: 0},
  obstacles_px: [[[1, 1], [5, 1], [5, 5], [1, 5]]],
  inflated_obstacles_px: [[[0, 0], [6, 0], [6, 6], [0, 6]]],
  planning_status: 'ready', planned_path_px: [[10, 10], [20, 20], [30, 20]]};
drawObstacleOverlay(1);
const zero = {zones: $('navigation-obstacle-clearances').children.length,
  obstacles: $('navigation-obstacles').children.length,
  route: $('navigation-planned-path').attributes.points};
navigationState.obstacle_settings.safety_margin_mm = 10;
drawObstacleOverlay(1);
const extraMarginZones = $('navigation-obstacle-clearances').children.length;
navigationState.obstacle_settings.safety_margin_mm = 0;
navigationState.collision_model = 'circle';
drawObstacleOverlay(1);
console.log(JSON.stringify({zero, extraMarginZones,
  legacyZones: $('navigation-obstacle-clearances').children.length}));
""")
    assert result["zero"] == {"zones": 0, "obstacles": 1, "route": "10,10 20,20 30,20"}
    assert result["extraMarginZones"] == 1
    assert result["legacyZones"] == 1


def test_same_frame_video_overlay_prevents_a_second_delayed_svg_footprint():
    result = run_overlay(r"""
navigationState = {collision_model: 'oriented_footprint',
  robot_footprint_px: [[20,10],[50,10],[60,35],[20,40]], front_px: [40,20]};
state.navigation_overlay_in_frame = true;
const duplicated = drawRobotFootprintOverlay([30,20], 1);
console.log(JSON.stringify({duplicated, count: $('navigation-robot-footprint').children.length,
  headingHidden: 'hidden' in $('navigation-robot-front').attributes}));
""")
    assert result == {"duplicated": False, "count": 0, "headingHidden": True}
