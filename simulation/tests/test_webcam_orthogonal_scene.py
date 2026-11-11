"""Recorrido ortogonal y centrado en la escena dibujada por el usuario.

La fixture contiene únicamente geometría anónima. No abre cámaras ni envía
órdenes: se comprueban las poses y el barrido completo de cada maniobra.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import time

import numpy as np
import pytest

from simulation.webcam.navigation_geometry import Calibration, observe_robot
from simulation.webcam.pose_obstacle_planner import (
    _Scene, check_swept_motion, plan_pose_path,
)
from simulation.webcam.robot_footprint import ROBOT_FOOTPRINT_MM


SCENE = json.loads((Path(__file__).with_name("fixtures") / "webcam_orthogonal_scene.json").read_text())
CALIBRATION = Calibration.from_dict(SCENE["calibration"])
CORNERS = SCENE["marker_corners"]
OBSTACLES = SCENE["obstacles_px"]
GOALS = SCENE["waypoints_px"]


def world_scene():
    return _Scene(CORNERS, CALIBRATION, OBSTACLES, ROBOT_FOOTPRINT_MM)


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def image_bearing(first, last):
    return math.atan2(first[1] - last[1], last[0] - first[0])


def translation_runs(result, start, heading):
    """Combina solo las subdivisiones colineales del controlador local."""
    scene = world_scene()
    previous = scene.pose(start, heading)
    runs = []
    for step in result["steps"]:
        pose = scene.pose(step["target_px"], step["heading_rad"])
        distance = float(np.linalg.norm(pose.point - previous.point))
        if step["kind"] == "drive" and distance > 1e-6:
            yaw = math.atan2(pose.y - previous.y, pose.x - previous.x)
            if runs and abs(wrap(yaw - runs[-1]["yaw"])) < math.radians(.1):
                runs[-1]["end"] = pose.point
                runs[-1]["length"] += distance
            else:
                runs.append({"start": previous.point, "end": pose.point,
                             "length": distance, "yaw": yaw})
        previous = pose
    return runs


def validate_route(start, heading, goal):
    started = time.monotonic()
    result = plan_pose_path(CORNERS, CALIBRATION, start, goal, OBSTACLES,
                            start_heading_rad=heading, max_seconds=3.)
    assert time.monotonic() - started < 3.25
    assert result["path_px"][0] == pytest.approx(start, abs=.003)
    assert result["path_px"][-1] == pytest.approx(goal, abs=.003)
    assert np.all(np.isfinite(result["path_px"]))
    previous = start
    for step in result["steps"]:
        if step["kind"] == "drive":
            assert step["direction"] == 1
        report = check_swept_motion(CORNERS, CALIBRATION, previous, heading,
            step["target_px"], step["heading_rad"], OBSTACLES,
            allow_start_overlap=step.get("escaping", False))
        assert report["clear"], (step, report)
        previous, heading = step["target_px"], step["heading_rad"]
    return result, heading


def assert_main_legs_are_orthogonal(runs):
    # Un enlace corto para alcanzar un punto exacto puede ajustar unos grados.
    # Los tramos largos no deben formar el zigzag diagonal de la ruta anterior.
    long_runs = [run for run in runs if run["length"] >= .05]
    for first, last in zip(long_runs, long_runs[1:]):
        angle = abs(wrap(last["yaw"] - first["yaw"]))
        assert min(angle, abs(angle - math.pi / 2), abs(angle - math.pi)) <= math.radians(6.)


def test_recorded_return_from_four_to_one_uses_a_simple_orthogonal_detour():
    start = SCENE["center_px"]
    heading = math.radians(observe_robot(CORNERS, CALIBRATION)["heading_deg"])
    result, _ = validate_route(start, heading, GOALS[0])
    runs = translation_runs(result, start, heading)
    assert len(runs) <= 4, [(run["length"], math.degrees(run["yaw"])) for run in runs]
    assert_main_legs_are_orthogonal(runs)
    # El gran obstáculo se rodea por debajo, sin buscar un paso junto al borde
    # superior de la imagen; el destino queda a su izquierda.
    points = [world_scene().plane.world(point) for point in result["path_px"]]
    assert max(point[1] for point in points) > .52
    assert min(point[0] for point in points) < .36


def vertical_polygon_interval(polygon, x):
    values = []
    for first, last in zip(polygon, np.roll(polygon, -1, axis=0)):
        if min(first[0], last[0]) <= x <= max(first[0], last[0]) and abs(last[0] - first[0]) > 1e-9:
            t = (x - first[0]) / (last[0] - first[0])
            values.append(first[1] + t * (last[1] - first[1]))
    return min(values), max(values)


def test_recorded_two_to_three_uses_a_centered_long_corridor_without_a_zigzag():
    heading = image_bearing(GOALS[0], GOALS[1])
    result, _ = validate_route(GOALS[1], heading, GOALS[2])
    runs = translation_runs(result, GOALS[1], heading)
    assert len(runs) <= 3, [(run["length"], math.degrees(run["yaw"])) for run in runs]
    assert_main_legs_are_orthogonal(runs)
    scene = world_scene()
    errors = []
    for run in runs:
        for ratio in np.linspace(0., 1., 31):
            point = run["start"] * (1. - ratio) + run["end"] * ratio
            if .56 <= point[0] <= .90:
                upper = vertical_polygon_interval(scene.plane.obstacles[0], point[0])[1]
                lower = vertical_polygon_interval(scene.plane.obstacles[1], point[0])[0]
                errors.append(abs(point[1] - (upper + lower) / 2.))
    assert errors
    assert float(np.median(errors)) < .020
    assert max(errors) < .030


def test_recorded_four_point_mission_keeps_turns_simple_and_all_sweeps_free():
    start = SCENE["center_px"]
    heading = math.radians(observe_robot(CORNERS, CALIBRATION)["heading_deg"])
    for goal in GOALS:
        initial_heading = heading
        result, heading = validate_route(start, heading, goal)
        runs = translation_runs(result, start, initial_heading)
        assert len(runs) <= 4
        assert_main_legs_are_orthogonal(runs)
        start = goal
