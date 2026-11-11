"""El origen de la ruta y de las correcciones es el centro calibrado.

Marcador deliberadamente desplazado respecto al centro del robot. Las pruebas
no abren la webcam, no arman sesiones ni envían consignas a motores.
"""
from __future__ import annotations

import math
import json
from pathlib import Path
import re
import shutil
import subprocess

import numpy as np
import pytest

from simulation.tests.test_webcam_point_navigation import FakeCamera, FakeRobot
from simulation.webcam.navigation import Navigation
from simulation.webcam.navigation_geometry import (Calibration, _homographies, _project,
                                                   calibrate_robot, observe_robot)


class InspectFollower:
    def update(self, local, *_args, **_kwargs):
        self.local = local
        return {"left": 0., "right": 0., "target_local_m": [.1, 0.]}


@pytest.mark.parametrize("heading", [0., math.pi / 2., math.pi, -math.pi / 2.])
def test_offset_tag_never_replaces_the_calibrated_route_origin(heading):
    camera, robot = FakeCamera(), FakeRobot()
    camera.pose(x=620., y=400., heading=heading, scale=800.)
    calibration = Calibration((.075, -.040), .42, "usb-camera", 1280, 960)
    frame = camera.state()
    observation = observe_robot(frame["marker_corners"], calibration)
    _, backward, _ = _homographies(frame["marker_corners"], .04, 1280, 960)
    c, s = math.cos(.42), math.sin(.42)
    rotation = np.asarray([[c, -s], [s, c]])
    goal = _project(backward, np.asarray(calibration.center_tag_m) + rotation @ [.2, .04],
                    np.zeros(2)).tolist()
    nav = Navigation(camera, robot)
    try:
        result = nav._calculate_plan(frame, calibration, [goal], 0, [],
                                     nav._obstacle_settings(), nav._route_generation)
        assert result["status"] == "ready"
        assert result["path"][0] == pytest.approx(observation["center_px"], abs=1e-6)
        # El centro elegido está a68px del marcador: confundirlos sería visible.
        assert np.linalg.norm(np.asarray(result["path"][0]) - [620., 400.]) > 60.
        follower = InspectFollower()
        command = nav._continuous_command(frame, calibration, result["path"], follower,
                                           {"forward_mm_s": 200, "turn_deg_s": 100}, 1.)
        assert follower.local[0] == pytest.approx([0., 0.], abs=1e-9)
        assert follower.local[1] == pytest.approx([.2, .04], abs=1e-9)
        expected = _project(backward,
                            np.asarray(calibration.center_tag_m) + rotation @ [.1, 0.],
                            np.zeros(2))
        assert command["target_px"] == pytest.approx(expected, abs=1e-6)
        assert robot.starts == 0 and robot.drives == []
    finally:
        nav.close()


def browser_calibration_projection(center, first_corners, next_corners):
    """Ejecuta las transformaciones reales que conservan el primer clic."""
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node permite ejecutar la geometría del navegador.")
    page = (Path(__file__).resolve().parents[1] / "webcam/static/index.html").read_text()
    functions = []
    for name in ("finitePoint", "mapQuadPoint"):
        match = re.search(r"    function " + name + r"\([^\n]*\) \{.*?\n    \}", page, re.S)
        assert match, name
        functions.append(match.group())
    data = json.dumps([center, first_corners.tolist(), next_corners.tolist()])
    script = "\n".join(functions) + f"""
const [center, first, next] = {data};
const unit = [[0,0], [1,0], [1,1], [0,1]];
const local = mapQuadPoint(center, first, unit);
console.log(JSON.stringify(mapQuadPoint(local, unit, next)));
"""
    result = subprocess.run([node, "--input-type=module", "-e", script],
                            check=True, capture_output=True, text=True, timeout=5.)
    return json.loads(result.stdout)


@pytest.mark.parametrize("scale", [850., 3000.])
@pytest.mark.parametrize("angle", [0., 1.2])
def test_browser_keeps_the_selected_center_with_the_same_model_as_python(scale, angle):
    def corners_at(rotation, translation):
        c, s = math.cos(rotation), math.sin(rotation)
        matrix = np.asarray([[scale*c, -scale*s, translation[0]],
                             [-scale*s, -scale*c, translation[1]], [0., 0., 1.]])
        corners = np.asarray([_project(matrix, point, np.zeros(2)) for point in
                              [(-.02,.02), (.02,.02), (.02,-.02), (-.02,-.02)]])
        return matrix, corners

    matrix, first = corners_at(angle, (620., 400.))
    first += np.asarray([[.21, -.13], [-.19, .28], [.32, -.17], [-.23, -.10]])
    _, next_corners = corners_at(angle + .2, (650., 380.))
    next_corners += np.asarray([[-.15, .24], [.26, -.17], [-.29, .12], [.10, -.21]])
    center = _project(matrix, [.075, -.04], np.zeros(2)).tolist()
    front = _project(matrix, [.115, -.04], np.zeros(2)).tolist()
    calibration = calibrate_robot(first, center, front, camera_id="usb-camera", width=1280, height=960)
    expected = observe_robot(next_corners, calibration)["center_px"]
    actual = browser_calibration_projection(center, first, next_corners)
    # El modelo afín de un tag pequeño y la homografía de un tag grande deben
    # coincidir en ambas mitades de la calibración, también lejos del marcador.
    assert actual == pytest.approx(expected, abs=.001)
