"""Regresión de proyección y recorrido reales, con cámara y robot ficticios.

Las variaciones de esquinas son fracciones de píxel de un marcador de 35 px.
No se abre cámara, puerto USB, transporte Wi-Fi ni conexión a motores.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from simulation.tests.test_webcam_point_navigation import FakeCamera, FakeRobot
import simulation.webcam.navigation as navigation_module
from simulation.webcam.navigation import Navigation
from simulation.webcam.navigation_geometry import Calibration, _homographies, _project, observe_robot
from simulation.webcam.obstacle_planner import _PlaneMap
from simulation.webcam.pose_obstacle_planner import check_swept_motion


SCENE = json.loads((Path(__file__).with_name("fixtures") / "webcam_projection_scene.json").read_text())
CALIBRATION = Calibration.from_dict(SCENE["calibration"])


def perturbed_corners():
    corners = np.asarray(SCENE["marker_corners"])
    yield corners
    yield np.asarray(SCENE["subpixel_failure_corners"])
    random = np.random.default_rng(42)
    for uncertainty in (.25, .50):
        for _ in range(64):
            yield corners + random.normal(0., uncertainty, corners.shape)


def test_small_corner_uncertainty_keeps_the_entire_visible_plane_projectable():
    image_points = np.asarray([[0., 0.], [1279., 0.], [1279., 719.], [0., 719.],
                               *SCENE["waypoints_px"]])
    previous_center = None
    for corners in perturbed_corners():
        forward, backward, reference = _homographies(corners, .04, 1280, 720)
        world = np.asarray([_project(forward, point, reference) for point in image_points])
        assert np.all(np.isfinite(world))
        restored = np.asarray([_project(backward, point, np.zeros(2)) for point in world])
        assert restored == pytest.approx(image_points, abs=1e-5)
        plane = _PlaneMap.build(corners, CALIBRATION, SCENE["obstacles_px"])
        assert np.all(np.isfinite(plane.frame))
        # This frame covers about 1.4 m; subpixel noise must not invent a horizon.
        assert np.max(np.ptp(plane.frame, axis=0)) < 3.
        observation = observe_robot(corners, CALIBRATION, SCENE["waypoints_px"][0])
        assert .15 < observation["distance_m"] < .40
        distant_observation = observe_robot(corners, CALIBRATION, SCENE["waypoints_px"][2])
        assert .80 < distant_observation["distance_m"] < 1.0
        center = np.asarray(observation["center_px"])
        if previous_center is not None:
            assert np.linalg.norm(center - previous_center) < 8.
        previous_center = center


def test_crossing_half_pixel_fit_residual_does_not_jump_the_distant_target():
    corners = np.asarray(SCENE["residual_threshold_corners"])
    plane = np.asarray([[-.02, .02], [.02, .02], [.02, -.02], [-.02, -.02]])
    design = np.column_stack((plane, np.ones(4)))
    fitted = design @ np.linalg.lstsq(design, corners, rcond=None)[0]
    residual = corners - fitted
    magnitude = np.max(np.linalg.norm(residual, axis=1))
    below = fitted + residual * (.499999 / magnitude)
    above = fitted + residual * (.500001 / magnitude)
    assert np.max(np.abs(below - above)) < .000003
    first = observe_robot(below, CALIBRATION, SCENE["waypoints_px"][2])
    second = observe_robot(above, CALIBRATION, SCENE["waypoints_px"][2])
    # An earlier mode switch changed this stationary target from 876 to724 mm.
    assert .80 < first["distance_m"] < 1.0
    assert .80 < second["distance_m"] < 1.0
    assert abs(first["distance_m"] - second["distance_m"]) < .001


@pytest.mark.parametrize("corner_key", ["marker_corners", "subpixel_failure_corners"])
def test_recorded_five_point_route_remains_forward_and_collision_checked(corner_key):
    corners = SCENE[corner_key]
    camera, robot = FakeCamera(), FakeRobot()
    camera.update(camera_id="recorded-camera", frame_width=1280, frame_height=720,
                  marker_corners=corners, running=True, opening=False)
    nav = Navigation(camera, robot)
    try:
        result = nav._calculate_plan(camera.state(), CALIBRATION, SCENE["waypoints_px"], 0,
                                     SCENE["obstacles_px"], nav._obstacle_settings(),
                                     nav._route_generation)
        assert result["status"] == "ready", result["message"]
        assert len(result["pose_legs"]) == len(SCENE["waypoints_px"])
        assert np.all(np.isfinite(result["path"]))
        center, heading = result["center"], result["heading"]
        for goal, steps in zip(SCENE["waypoints_px"], result["pose_legs"]):
            assert steps[-1]["target_px"] == pytest.approx(goal, abs=.002)
            for step in steps:
                if step["kind"] == "drive":
                    assert step["direction"] == 1
                sweep = check_swept_motion(corners, CALIBRATION, center, heading,
                    step["target_px"], step["heading_rad"], SCENE["obstacles_px"],
                    allow_start_overlap=step["kind"] == "escape")
                assert sweep["clear"], sweep["reason"]
                center, heading = step["target_px"], step["heading_rad"]
        assert robot.starts == 0 and robot.drives == []
    finally:
        nav.close()


def test_failed_cached_projection_retries_without_requiring_robot_motion(monkeypatch):
    camera, robot = FakeCamera(), FakeRobot()
    camera.update(camera_id="recorded-camera", frame_width=1280, frame_height=720,
                  marker_corners=SCENE["subpixel_failure_corners"], running=True, opening=False)
    nav = Navigation(camera, robot)
    nav._calibration = CALIBRATION
    nav._waypoints = [point[:] for point in SCENE["waypoints_px"]]
    nav._obstacles = [[point[:] for point in polygon] for polygon in SCENE["obstacles_px"]]
    clock = [100.]
    monkeypatch.setattr(navigation_module.time, "monotonic", lambda: clock[0])
    calculate = nav._calculate_plan
    attempts = []

    def initially_failed_projection(frame, calibration, points, index, obstacles, settings, generation):
        attempts.append(frame["sequence"])
        if len(attempts) == 1:
            return {"status": "blocked", "message": "No se pudo calcular la proyección.",
                    "path": [], "inflated": [], "legs": [],
                    "center": observe_robot(frame["marker_corners"], calibration)["center_px"]}
        return calculate(frame, calibration, points, index, obstacles, settings, generation)

    monkeypatch.setattr(nav, "_calculate_plan", initially_failed_projection)
    try:
        first = nav._refresh_plan(camera.state())
        assert first["status"] == "blocked"
        assert nav._plan_attempted_at == 100.
        assert len(attempts) == 1

        # Nueva detección del mismo robot quieto: conserva el fallo solo hasta
        # agotar la espera, en vez de convertirlo en un bloqueo permanente.
        camera.update(marker_corners=SCENE["marker_corners"])
        clock[0] = 100.5
        assert nav._refresh_plan(camera.state()) is None
        assert nav._planning_status == "blocked" and len(attempts) == 1

        clock[0] = 101.01
        recovered = nav._refresh_plan(camera.state())
        assert recovered["status"] == "ready", recovered["message"]
        assert len(recovered["pose_legs"]) == 5
        assert len(attempts) == 2 and attempts[1] != attempts[0]
        assert nav._plan_attempted_at == 101.01

        # Una ruta válida permanece en caché aunque transcurra otro intervalo.
        camera.update(marker_corners=SCENE["marker_corners"])
        clock[0] = 200.
        assert nav._refresh_plan(camera.state()) is None
        assert nav._planning_status == "ready" and len(attempts) == 2
        assert robot.starts == 0 and robot.drives == []
    finally:
        nav.close()
