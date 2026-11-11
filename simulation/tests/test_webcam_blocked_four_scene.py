"""Salida del contacto estimado al pasar del punto 3 al 4.

La fixture contiene solo geometría anónima. Las comprobaciones no abren
webcam, USB, conexiones de red ni transporte hacia los motores físicos.
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
    _Scene, check_pose, check_swept_motion, check_wheel_motion, plan_pose_path,
)
from simulation.webcam.robot_footprint import ROBOT_FOOTPRINT_MM


SCENE = json.loads((Path(__file__).with_name("fixtures") / "webcam_blocked_four_scene.json").read_text())
CALIBRATION = Calibration.from_dict(SCENE["calibration"])
OBSTACLES = SCENE["obstacles_px"]
GOAL = SCENE["goal_px"]


def validate_escape_route(corners, *, prefer_orthogonal=True):
    observed = observe_robot(corners, CALIBRATION, GOAL)
    start = observed["center_px"]
    heading = math.radians(observed["heading_deg"])
    report = check_pose(corners, CALIBRATION, start, heading, OBSTACLES)
    assert 10. < report["penetration_mm"] < 17.
    assert report["penetrations_mm"][0] == 0.
    initial_depths = np.asarray(report["penetrations_mm"])

    started = time.monotonic()
    result = plan_pose_path(corners, CALIBRATION, start, GOAL, OBSTACLES,
                            start_heading_rad=heading, max_seconds=3.,
                            allow_start_overlap=True, prefer_orthogonal=prefer_orthogonal)
    assert time.monotonic() - started < 3.25
    assert np.all(np.isfinite(result["path_px"]))
    assert result["path_px"][0] == pytest.approx(start, abs=.003)
    assert result["path_px"][-1] == pytest.approx(GOAL, abs=.003)
    assert result["steps"]

    previous, previous_depths = start, initial_depths
    previous_local = np.zeros(2)
    ordinary_drive_seen = False
    reverse_distance = 0.
    for step in result["steps"]:
        escaping = step.get("escaping", False)
        local = np.asarray(observe_robot(corners, CALIBRATION, step["target_px"])["target_local_m"])
        if step["kind"] == "drive":
            assert step["direction"] == 1
            ordinary_drive_seen = True
        if step["direction"] < 0:
            assert step["kind"] == "escape" and not ordinary_drive_seen
            reverse_distance += float(np.linalg.norm(local - previous_local))
        swept = check_swept_motion(corners, CALIBRATION, previous, heading,
            step["target_px"], step["heading_rad"], OBSTACLES,
            allow_start_overlap=escaping, max_penetration_mm=step.get("max_penetration_mm"))
        assert swept["clear"], (step, swept)
        end = check_pose(corners, CALIBRATION, step["target_px"], step["heading_rad"], OBSTACLES)
        depths = np.asarray(end["penetrations_mm"])
        if escaping:
            # El giro necesita atravesar una cresta numérica de 0,103 mm. La
            # concesión de 0,25 mm pertenece al contacto inicial y no se puede
            # acumular como otra concesión nueva en cada paso.
            assert np.all(depths <= previous_depths + .05)
            assert np.all(depths <= initial_depths + .30)
            assert step.get("max_penetration_mm") == pytest.approx(float(np.max(initial_depths)), abs=.002)
            assert swept["penetration_mm"] <= float(np.max(initial_depths)) + .30
            assert depths[0] <= .002  # El obstáculo superior no estaba tocado.
        else:
            assert np.max(depths) <= .002
        previous, heading, previous_depths = step["target_px"], step["heading_rad"], depths
        previous_local = local
    assert np.max(previous_depths) <= .002
    assert reverse_distance <= .08001
    assert ordinary_drive_seen
    return result


def test_recorded_contact_releases_then_reaches_point_four_instead_of_reporting_no_sequence():
    result = validate_escape_route(SCENE["marker_corners"])
    assert any(step.get("escaping", False) for step in result["steps"])


@pytest.mark.parametrize("seed", [1, 7, 19])
def test_subpixel_variations_do_not_reintroduce_the_blocked_fourth_destination(seed):
    random = np.random.default_rng(seed)
    corners = np.asarray(SCENE["marker_corners"]) + random.normal(0., .15, (4, 2))
    validate_escape_route(corners.tolist())


def test_contact_release_also_works_when_the_orthogonal_preference_is_disabled():
    validate_escape_route(SCENE["marker_corners"], prefer_orthogonal=False)


@pytest.mark.parametrize("sign", [-1, 1])
def test_wheel_guard_accepts_the_first_escape_turn_with_the_same_global_contact_baseline(sign):
    corners = SCENE["marker_corners"]
    observed = observe_robot(corners, CALIBRATION)
    initial = check_pose(corners, CALIBRATION, observed["center_px"],
                         math.radians(observed["heading_deg"]), OBSTACLES)
    baseline = initial["penetration_mm"]
    report = check_wheel_motion(corners, CALIBRATION, sign * 20., -sign * 20., .25,
        obstacles_px=OBSTACLES, allow_start_overlap=True, max_penetration_mm=baseline)
    assert report["clear"], report
    assert report["penetration_mm"] <= baseline + .30
    assert report["predicted_center_px"] == pytest.approx(observed["center_px"], abs=.003)
    assert not check_wheel_motion(corners, CALIBRATION, sign * 20., -sign * 20., .25,
        obstacles_px=OBSTACLES, allow_start_overlap=True, max_penetration_mm=baseline - 1.)["clear"]


def test_existing_contact_allowance_never_authorizes_hitting_a_new_obstacle():
    corners = SCENE["marker_corners"]
    scene = _Scene(corners, CALIBRATION, OBSTACLES, ROBOT_FOOTPRINT_MM)
    observed = observe_robot(corners, CALIBRATION)
    heading = math.radians(observed["heading_deg"])
    pose = scene.pose(observed["center_px"], heading)
    # Una caja 12 mm por delante del frente inicial: avanzar 20 mm la tocaría.
    # Su profundidad sería menor que el contacto inicial de 13 mm, de modo que
    # un único techo escalar incorrecto podría confundir ambos obstáculos.
    center = pose.point + .095 * np.asarray([math.cos(pose.yaw), math.sin(pose.yaw)])
    new_obstacle = [scene.image_point(center + offset)
                    for offset in np.asarray([[-.003, -.003], [.003, -.003],
                                              [.003, .003], [-.003, .003]])]
    obstacles = [*OBSTACLES, new_obstacle]
    initial = check_pose(corners, CALIBRATION, observed["center_px"], heading, obstacles)
    assert initial["penetrations_mm"][-1] == 0.
    free_motion = check_wheel_motion(corners, CALIBRATION, 80., 80., .25,
        obstacles_px=OBSTACLES, allow_start_overlap=True,
        max_penetration_mm=initial["penetration_mm"])
    assert free_motion["clear"], free_motion
    blocked = check_wheel_motion(corners, CALIBRATION, 80., 80., .25,
        obstacles_px=obstacles, allow_start_overlap=True,
        max_penetration_mm=initial["penetration_mm"])
    assert not blocked["clear"]
