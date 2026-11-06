"""Regresión de la escena reportada, anonimizada y sin red durante las pruebas.

Las coordenadas de imagen proceden de una instantánea local del robot detenido.
No se almacenan identidad de cámara, red, credenciales ni archivos de ejecución.
Los cambios de pose se proyectan mediante la misma cámara fija.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from simulation.webcam.navigation_geometry import calibrate_robot, _homographies, _project
from simulation.webcam.obstacle_planner import (plan_path, segment_is_clear, point_clearances_mm,
                                               camera_visibility_margin_mm)
from simulation.webcam.robot_footprint import ROBOT_RADIUS_MM

CORNERS = [[916.770630, 537.728333], [973.447632, 569.325195],
           [947.815491, 620.861938], [890.407410, 588.225159]]
CENTER = [892.004547, 502.343002]
FRONT = [919.201428, 518.745517]
OBSTACLES = [[[904.147844, 128.377823], [908.090349, 312.361396],
              [1053.963039, 293.963039], [1005.338809, 91.581109]]]
GOAL = [643.942505, 129.691992]


def calibration():
    return calibrate_robot(CORNERS, CENTER, FRONT, camera_id='recorded-camera',
                           width=1280, height=720, marker_size_m=.04)


def route(corners=CORNERS, center=CENTER, goal=GOAL, *, radius=None):
    cal = calibration()
    options = {} if radius is None else {'robot_radius_mm': radius}
    return plan_path(corners, cal, center, goal, OBSTACLES,
                     safety_margin_mm=0, tracking_margin_mm=0,
                     camera_clearance_mm=camera_visibility_margin_mm(cal, extra_margin_mm=0), **options)


def assert_route_free(result, corners, center, goal):
    cal = calibration()
    view = camera_visibility_margin_mm(cal, extra_margin_mm=0)
    assert result['path_px'][0] == pytest.approx(center, abs=.01)
    assert result['path_px'][-1] == pytest.approx(goal, abs=.01)
    assert result['clearance_mm'] == pytest.approx(ROBOT_RADIUS_MM)
    for first, last in zip(result['path_px'], result['path_px'][1:]):
        assert segment_is_clear(corners, cal, first, last, OBSTACLES,
                                clearance_mm=ROBOT_RADIUS_MM, camera_clearance_mm=view)


def test_actual_footprint_finds_left_detour_where_old_150mm_circle_rejected_start():
    cal = calibration()
    distances = point_clearances_mm(CORNERS, cal, CENTER, OBSTACLES)
    assert ROBOT_RADIUS_MM == pytest.approx(116.1)
    assert 140 < distances['obstacle_mm'] < 150  # Libre con la envolvente real.
    with pytest.raises(ValueError, match='El robot.*obstáculo'):
        route(radius=150.)
    result = route()
    assert len(result['path_px']) == 3
    assert result['path_length_m'] == pytest.approx(.338, abs=.015)
    assert result['path_px'][1][0] < CENTER[0] - 20  # Primer rodeo hacia la izquierda.
    assert_route_free(result, CORNERS, CENTER, GOAL)


@pytest.mark.parametrize('goal', [GOAL, [720., 160.], [600., 210.], [560., 360.]])
def test_real_scene_supports_several_free_left_targets(goal):
    result = route(goal=goal)
    assert result['path_length_m'] > 0
    assert_route_free(result, CORNERS, CENTER, goal)


def moved_pose(angle_deg, displacement):
    cal = calibration()
    forward, backward, reference = _homographies(CORNERS, .04, 1280, 720)
    angle = math.radians(angle_deg)
    rotation = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
    center_world = np.array(cal.center_tag_m)
    moved_center = center_world + np.asarray(displacement)
    origin = moved_center - rotation @ center_world
    # Girar el conjunto alrededor del centro entre ruedas mueve el marcador
    # desplazado, pero no desplaza arbitrariamente el centro de colisión.
    points = [origin + rotation @ np.array(point)
              for point in [(-.02, .02), (.02, .02), (.02, -.02), (-.02, -.02)]]
    corners = [_project(backward, point, np.zeros(2)).tolist() for point in points]
    center = _project(backward, moved_center, np.zeros(2)).tolist()
    return corners, center


@pytest.mark.parametrize('angle', [-45., 0., 45., 90.])
@pytest.mark.parametrize('displacement', [(-.005, 0.), (0., .005), (.005, -.005)])
def test_same_camera_scene_remains_navigable_when_robot_pose_is_perturbed(angle, displacement):
    corners, center = moved_pose(angle, displacement)
    result = route(corners=corners, center=center)
    assert_route_free(result, corners, center, GOAL)
    assert result['path_length_m'] == pytest.approx(.338, abs=.035)
    assert result['path_px'][1][0] < center[0]


def test_true_body_collision_is_still_rejected_with_zero_extra_margin():
    # El centro situado dentro del cuadrilátero nunca es un inicio válido.
    occupied = np.mean(np.array(OBSTACLES[0]), axis=0).tolist()
    with pytest.raises(ValueError, match='El robot.*obstáculo'):
        route(center=occupied)
    # Tampoco un objetivo demasiado próximo al borde físico del obstáculo.
    with pytest.raises(ValueError, match='El destino.*obstáculo'):
        route(goal=[910., 350.])
