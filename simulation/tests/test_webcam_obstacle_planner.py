"""Planificador geométrico sobre imágenes sintéticas, sin cámara ni robot."""
from __future__ import annotations

import math
import time

import numpy as np
import pytest

from simulation.webcam.navigation_geometry import Calibration
from simulation.webcam.obstacle_planner import (plan_path, segment_is_clear, validate_obstacles,
                                               point_clearance_mm, point_clearances_mm,
                                               camera_visibility_margin_mm)


def scene(scale=600., perspective=False):
    matrix = np.array([[scale, 0., 640.], [0., -scale, 480.], [0., 0., 1.]])
    if perspective:
        matrix[2, :2] = [.12, -.10]
    def project(point):
        value = matrix @ np.array([*point, 1.])
        return (value[:2] / value[2]).tolist()
    corners = [project(point) for point in [(-.02, .02), (.02, .02), (.02, -.02), (-.02, -.02)]]
    return corners, Calibration((0., 0.), 0., 'camera', 1280, 960), project


def rectangle(project, x1, y1, x2, y2):
    return [project(point) for point in [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]]


def assert_safe(result, corners, calibration, obstacles, clearance=220.):
    assert len(result['path_px']) >= 2
    for a, b in zip(result['path_px'], result['path_px'][1:]):
        assert segment_is_clear(corners, calibration, a, b, obstacles, clearance_mm=clearance)


def test_direct_path_and_maximum_segment_length():
    corners, calibration, project = scene()
    start, goal = project((-.65, 0)), project((.65, 0))
    result = plan_path(corners, calibration, start, goal, [], max_segment_m=.3)
    assert result['path_px'][0] == pytest.approx(start)
    assert result['path_px'][-1] == pytest.approx(goal)
    assert result['path_length_m'] == pytest.approx(1.3, abs=1e-6)
    assert result['clearance_mm'] == pytest.approx(220.)
    for a, b in zip(result['path_px'], result['path_px'][1:]):
        assert np.linalg.norm(np.array(a) - b) / 600. <= .300001
    assert_safe(result, corners, calibration, [])


@pytest.mark.parametrize('perspective', [False, True])
def test_routes_around_rectangle_with_continuous_clearance(perspective):
    corners, calibration, project = scene(perspective=perspective)
    obstacles = [rectangle(project, -.08, -.16, .08, .16)]
    start, goal = project((-.65, 0)), project((.65, 0))
    assert not segment_is_clear(corners, calibration, start, goal, obstacles, clearance_mm=220)
    result = plan_path(corners, calibration, start, goal, obstacles)
    assert result['path_length_m'] > 1.3
    assert any(abs(point[1] - 480.) > 100 for point in result['path_px'])
    assert result['inflated_obstacles_px'] and len(result['inflated_obstacles_px'][0]) > 4
    assert_safe(result, corners, calibration, obstacles)


def test_multiple_obstacles_and_a_concave_quadrilateral():
    corners, calibration, project = scene(scale=500.)
    concave = [project(p) for p in [(-.3, -.1), (-.1, .1), (-.28, .02), (-.4, .15)]]
    obstacles = [concave, rectangle(project, .2, -.3, .3, -.1)]
    result = plan_path(corners, calibration, project((-.85, -.1)), project((.85, .1)), obstacles)
    assert_safe(result, corners, calibration, obstacles)
    assert len(validate_obstacles(obstacles, 1280, 960)) == 2


def test_safety_margin_forbids_corridor_too_narrow():
    corners, calibration, project = scene()
    obstacles = [rectangle(project, -.1, -.79, .1, -.10), rectangle(project, -.1, .10, .1, .79)]
    with pytest.raises(ValueError, match='camino libre'):
        plan_path(corners, calibration, project((-.65, 0)), project((.65, 0)), obstacles)


@pytest.mark.parametrize('which', ['start', 'goal'])
def test_occupied_start_or_goal_never_returns_direct_fallback(which):
    corners, calibration, project = scene()
    obstacles = [rectangle(project, -.1, -.1, .1, .1)]
    start, goal = project((-.65, 0)), project((.65, 0))
    if which == 'start':
        start = project((-.2, 0))
    else:
        goal = project((.2, 0))
    with pytest.raises(ValueError, match='margen'):
        plan_path(corners, calibration, start, goal, obstacles)


def test_tangency_is_blocked_but_clearance_just_above_it_is_free():
    corners, calibration, project = scene()
    obstacles = [rectangle(project, -.1, -.1, .1, .1)]
    assert not segment_is_clear(corners, calibration, project((-.5, .32)), project((.5, .32)),
                                obstacles, clearance_mm=220)
    assert segment_is_clear(corners, calibration, project((-.5, .321)), project((.5, .321)),
                            obstacles, clearance_mm=220)


def test_diagonal_cannot_cut_corner_of_two_obstacles():
    corners, calibration, project = scene()
    obstacles = [rectangle(project, -.30, -.1, -.01, .20), rectangle(project, .01, -.20, .30, .1)]
    start, goal = project((-.6, -.4)), project((.6, .4))
    result = plan_path(corners, calibration, start, goal, obstacles, robot_radius_mm=40,
                       safety_margin_mm=15, tracking_margin_mm=10)
    assert_safe(result, corners, calibration, obstacles, clearance=65)
    assert result['path_length_m'] > math.hypot(1.2, .8)


@pytest.mark.parametrize('scale', [400., 800., 1200.])
def test_same_metric_route_length_at_different_image_scales(scale):
    corners, calibration, project = scene(scale=scale)
    result = plan_path(corners, calibration, project((-.18, .0)), project((.18, .0)), [],
                       robot_radius_mm=40, safety_margin_mm=15, tracking_margin_mm=10)
    assert result['path_length_m'] == pytest.approx(.36, abs=1e-6)
    assert_safe(result, corners, calibration, [], clearance=65)


@pytest.mark.parametrize('bad', [None, {}, [[[1, 1], [2, 2], [3, 3]]],
    [[[20, 20], [80, 80], [80, 20], [20, 80]]],
    [[[20, 20], [20, 20], [80, 80], [20, 80]]],
    [[[20, 20], [40, 20], [80, 20], [20, 80]]],
    [[[20, 20], [80, 20], [80, 80], [-1, 80]]],
    [[[True, 20], [80, 20], [80, 80], [20, 80]]],
    [[[float('nan'), 20], [80, 20], [80, 80], [20, 80]]],
    [[[20, 20], [80, 20], [80, 80], [20, 80]]] * 33])
def test_invalid_polygons_are_rejected(bad):
    with pytest.raises(ValueError):
        validate_obstacles(bad, 1280, 960)


def test_clockwise_and_counterclockwise_polygons_are_equivalent():
    corners, calibration, project = scene()
    polygon = rectangle(project, -.08, -.16, .08, .16)
    first = plan_path(corners, calibration, project((-.65, 0)), project((.65, 0)), [polygon])
    second = plan_path(corners, calibration, project((-.65, 0)), project((.65, 0)), [polygon[::-1]])
    assert first['path_px'] == second['path_px']


def test_planning_is_deterministic_and_does_not_mutate_inputs():
    corners, calibration, project = scene()
    obstacles = [rectangle(project, -.08, -.16, .08, .16)]
    original = [[point[:] for point in poly] for poly in obstacles]
    args = (corners, calibration, project((-.65, 0)), project((.65, 0)), obstacles)
    assert plan_path(*args)['path_px'] == plan_path(*args)['path_px']
    assert obstacles == original


def test_invalid_projection_horizon_is_rejected_without_allocating_unbounded_grid():
    corners, calibration, _ = scene()
    # Muy pocos píxeles por marcador implican un plano visible enorme.
    tiny = [[636., 476.], [644., 476.], [644., 484.], [636., 484.]]
    started = time.monotonic()
    with pytest.raises(ValueError, match='grande'):
        plan_path(tiny, calibration, [500, 480], [800, 480], [])
    assert time.monotonic() - started < 1.


def test_point_clearance_obstacle_border_and_occupied_position():
    corners, calibration, project = scene()
    obstacles = [rectangle(project, -.1, -.1, .1, .1)]
    assert point_clearance_mm(corners, calibration, project((.3, 0)), obstacles) == pytest.approx(200., abs=.001)
    assert point_clearance_mm(corners, calibration, project((0., 0)), obstacles) == 0
    assert point_clearance_mm(corners, calibration, [60, 480], []) == pytest.approx(100., abs=.001)
    assert point_clearance_mm(corners, calibration, [-1, 480], []) == 0


def test_horizon_crossing_the_image_is_rejected():
    matrix = np.array([[600., 0., 640.], [0., -600., 480.], [2., 0., 1.]])
    def project(point):
        value = matrix @ np.array([*point, 1.])
        return (value[:2] / value[2]).tolist()
    corners = [project(p) for p in [(-.02, .02), (.02, .02), (.02, -.02), (-.02, -.02)]]
    calibration = Calibration((0., 0.), 0., 'camera', 1280, 960)
    with pytest.raises(ValueError, match='proyección'):
        plan_path(corners, calibration, [500, 480], [800, 480], [])


def test_planning_time_budget_rejects_work_without_a_direct_fallback(monkeypatch):
    from simulation.webcam import obstacle_planner
    corners, calibration, project = scene()
    obstacles = [rectangle(project, -.08, -.16, .08, .16)]
    monkeypatch.setattr(obstacle_planner, 'MAX_PLANNING_SECONDS', -1.)
    with pytest.raises(ValueError, match='cálculo'):
        plan_path(corners, calibration, project((-.65, 0)), project((.65, 0)), obstacles)


def marker_image_corners(center, angle_deg, side_px=24.):
    angle = math.radians(angle_deg)
    rotation = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
    offsets = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]], dtype=float) * side_px / 2
    return (offsets @ rotation.T + np.asarray(center)).tolist()


@pytest.mark.parametrize('angle', [-45., 0., 45., 90.])
@pytest.mark.parametrize('center', [(472., 480.), (808., 600.), (640., 250.)])
def test_fixed_camera_route_does_not_change_side_when_marker_rotates_or_moves(angle, center):
    _, calibration, _ = scene()
    obstacles = [[[592., 384.], [688., 384.], [688., 576.], [592., 576.]]]
    start, goal = [250., 480.], [1030., 480.]
    baseline_corners = marker_image_corners((472., 480.), 0.)
    changed_corners = marker_image_corners(center, angle)
    baseline = plan_path(baseline_corners, calibration, start, goal, obstacles)
    changed = plan_path(changed_corners, calibration, start, goal, obstacles)
    # El mapa sigue unido a la cámara; solo giró/cambió de sitio el marcador.
    assert np.mean(np.array(changed['path_px'])[1:-1, 1] - 480.) * np.mean(
        np.array(baseline['path_px'])[1:-1, 1] - 480.) > 0
    assert len(changed['path_px']) == len(baseline['path_px'])
    assert np.max(np.linalg.norm(np.array(changed['path_px']) - baseline['path_px'], axis=1)) / 600. < .01
    assert changed['path_length_m'] == pytest.approx(baseline['path_length_m'], abs=.01)
    assert_safe(changed, changed_corners, calibration, obstacles)


def test_perspective_camera_keeps_route_side_for_different_marker_poses():
    _, calibration, project = scene(perspective=True)
    obstacles = [rectangle(project, -.08, -.16, .08, .16)]
    start, goal = project((-.65, 0)), project((.65, 0))
    results = []
    for center, angle_deg in [((-.3, 0.), 0.), ((.2, .15), 45.), ((0., -.25), 90.)]:
        angle = math.radians(angle_deg)
        rotation = np.array([[math.cos(angle), -math.sin(angle)],
                             [math.sin(angle), math.cos(angle)]])
        corners = [project(np.array(center) + rotation @ np.array(point))
                   for point in [(-.02, .02), (.02, .02), (.02, -.02), (-.02, -.02)]]
        result = plan_path(corners, calibration, start, goal, obstacles)
        assert_safe(result, corners, calibration, obstacles)
        results.append(result)
    for result in results[1:]:
        assert len(result['path_px']) == len(results[0]['path_px'])
        assert np.max(np.linalg.norm(np.array(result['path_px']) - results[0]['path_px'], axis=1)) < 6.


def test_reported_live_scene_is_clear_of_obstacle_but_near_the_camera_border():
    corners = [[234.193, 325.179], [178.574, 299.417],
               [208.61, 248.28], [263.85, 273.21]]
    calibration = Calibration((.05536, 0.), 0., 'reported-camera', 1280, 720)
    obstacles = [[[850., 188.], [1025., 188.], [1025., 374.], [850., 374.]]]
    start, goal = [258., 359.], [450., 359.]
    distances = point_clearances_mm(corners, calibration, start, obstacles)
    assert distances['obstacle_mm'] == pytest.approx(354.56, abs=.05)
    assert distances['camera_mm'] == pytest.approx(161.92, abs=.05)
    assert distances['obstacle_mm'] > 210 > distances['camera_mm']
    with pytest.raises(ValueError, match=r'El robot.*borde de la imagen.*162 mm.*210 mm'):
        plan_path(corners, calibration, start, goal, obstacles, safety_margin_mm=20)
    camera_margin = camera_visibility_margin_mm(calibration, tracking_margin_mm=40)
    assert camera_margin == pytest.approx(143.64, abs=.05)
    result = plan_path(corners, calibration, start, goal, obstacles, safety_margin_mm=20,
                       camera_clearance_mm=camera_margin)
    assert result['clearance_mm'] == pytest.approx(210.)
    assert result['camera_clearance_mm'] == pytest.approx(camera_margin)
    for a, b in zip(result['path_px'], result['path_px'][1:]):
        assert segment_is_clear(corners, calibration, a, b, obstacles, clearance_mm=210,
                                camera_clearance_mm=camera_margin)
    # El margen de obstáculos sigue bloqueando un destino que queda a206 mm.
    with pytest.raises(ValueError, match=r'El destino.*obstáculo.*206 mm.*210 mm'):
        plan_path(corners, calibration, start, [500., 359.], obstacles, safety_margin_mm=20,
                   camera_clearance_mm=camera_margin)


@pytest.mark.parametrize('angle_deg', [0., 45., 90., -135.])
@pytest.mark.parametrize('tracking', [0., 40.])
def test_visibility_margin_uses_offset_norm_and_marker_semidiagonal(angle_deg, tracking):
    theta = math.radians(angle_deg)
    offset = (.05 * math.cos(theta), .05 * math.sin(theta))
    calibration = Calibration(offset, theta, 'camera', 1280, 960, .04)
    value = camera_visibility_margin_mm(calibration, tracking_margin_mm=tracking)
    assert value == pytest.approx(50. + math.sqrt(2.) * 20. + 20. + tracking)
    assert camera_visibility_margin_mm(Calibration((0., 0.), theta, 'camera', 1280, 960)) == pytest.approx(
        math.sqrt(2.) * 20. + 20.)


def test_reduced_camera_margin_preserves_legacy_none_and_obstacle_radius():
    corners, calibration, project = scene()
    start, goal = project((-.95, 0)), project((-.5, 0))
    with pytest.raises(ValueError, match='borde de la imagen'):
        plan_path(corners, calibration, start, goal, [])
    assert not segment_is_clear(corners, calibration, start, goal, [], clearance_mm=220)
    result = plan_path(corners, calibration, start, goal, [], camera_clearance_mm=60)
    assert result['clearance_mm'] == 220 and result['camera_clearance_mm'] == 60
    assert segment_is_clear(corners, calibration, start, goal, [], clearance_mm=220,
                            camera_clearance_mm=60)


def test_smaller_camera_margin_does_not_open_a_narrow_obstacle_corridor():
    corners, calibration, project = scene()
    obstacles = [rectangle(project, -.1, -.79, .1, -.10),
                 rectangle(project, -.1, .10, .1, .79)]
    with pytest.raises(ValueError, match='camino libre'):
        plan_path(corners, calibration, project((-.65, 0)), project((.65, 0)), obstacles,
                   camera_clearance_mm=60)


def test_astar_uses_separate_camera_margin_and_still_clears_every_obstacle():
    corners, calibration, project = scene()
    obstacles = [rectangle(project, -.08, -.4, .08, .4)]
    start, goal = project((-.65, 0)), project((.65, 0))
    with pytest.raises(ValueError, match='camino libre'):
        plan_path(corners, calibration, start, goal, obstacles)
    result = plan_path(corners, calibration, start, goal, obstacles, camera_clearance_mm=60)
    assert result['path_length_m'] > 1.3
    for a, b in zip(result['path_px'], result['path_px'][1:]):
        assert segment_is_clear(corners, calibration, a, b, obstacles, clearance_mm=220,
                                camera_clearance_mm=60)


def test_point_distances_remain_separate_for_braking_and_legacy_minimum():
    corners, calibration, project = scene()
    point = project((-.95, 0))
    distances = point_clearances_mm(corners, calibration, point, [])
    assert distances['obstacle_mm'] == math.inf
    assert distances['camera_mm'] == pytest.approx(116.6667, abs=.001)
    assert point_clearance_mm(corners, calibration, point, []) == distances['camera_mm']
    obstacles = [rectangle(project, -.1, -.1, .1, .1)]
    occupied = point_clearances_mm(corners, calibration, project((0., 0)), obstacles)
    assert occupied['obstacle_mm'] == 0 and occupied['camera_mm'] > 700
    assert point_clearance_mm(corners, calibration, project((0., 0)), obstacles) == 0
    assert point_clearances_mm(corners, calibration, [-1., 480.], [])['camera_mm'] == 0


def test_destination_at_visibility_edge_names_camera_instead_of_obstacle():
    corners, calibration, project = scene()
    with pytest.raises(ValueError, match=r'El destino.*borde de la imagen.*47 mm.*60 mm'):
        plan_path(corners, calibration, project((-.65, 0)), project((-1.02, 0)), [],
                   camera_clearance_mm=60)


@pytest.mark.parametrize('bad', [True, -1., float('nan'), float('inf'), '60'])
def test_invalid_separate_camera_margin_is_rejected(bad):
    corners, calibration, project = scene()
    with pytest.raises(ValueError):
        plan_path(corners, calibration, project((-.65, 0)), project((.65, 0)), [],
                   camera_clearance_mm=bad)
    with pytest.raises(ValueError):
        segment_is_clear(corners, calibration, project((-.65, 0)), project((.65, 0)), [],
                         camera_clearance_mm=bad)
    with pytest.raises(ValueError):
        camera_visibility_margin_mm(calibration, bad)
