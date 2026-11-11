"""Huella orientada y maniobras, con escenas sintéticas y sin dispositivos."""
from __future__ import annotations

import math
import time

import numpy as np
import pytest

from simulation.webcam.navigation_geometry import Calibration, observe_robot
from simulation.webcam.obstacle_planner import segment_is_clear
from simulation.webcam.pose_obstacle_planner import (
    check_pose, check_swept_motion, check_wheel_motion, heading_error_rad, plan_pose_path)


def scene(center_offset=(0., 0.), front_angle=0.):
    corners = [[628., 468.], [652., 468.], [652., 492.], [628., 492.]]
    calibration = Calibration(center_offset, front_angle, 'synthetic-camera', 1280, 960)
    return corners, calibration, lambda x, y: [640. + 600. * x, 480. - 600. * y]


def rectangle(project, x1, y1, x2, y2):
    return [project(x, y) for x, y in [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]]


def validate_steps(corners, calibration, start, heading, obstacles, result):
    previous, yaw = start, heading
    reverse_distance = 0.
    normal_started = False
    for step in result['steps']:
        assert step['kind'] in {'drive', 'turn', 'escape'} and step['direction'] in (-1, 1)
        if step['kind'] == 'drive':
            assert step['direction'] == 1
            normal_started = True
        if step['direction'] < 0:
            assert step['kind'] == 'escape' and not normal_started
            reverse_distance += np.linalg.norm(np.array(step['target_px']) - previous) / 600.
        if step['kind'] == 'turn':
            assert step['target_px'] == pytest.approx(previous, abs=.01)
        else:
            assert np.linalg.norm(np.array(step['target_px']) - previous) / 600. <= .45001
        report = check_swept_motion(corners, calibration, previous, yaw,
                                    step['target_px'], step['heading_rad'], obstacles,
                                    allow_start_overlap=step.get('escaping', False))
        assert report['clear'], (step, report)
        previous, yaw = step['target_px'], step['heading_rad']
    assert reverse_distance <= .080001
    return previous


def test_oriented_hull_fits_where_circle_would_falsely_reject():
    corners, cal, p = scene()
    obstacle = [rectangle(p, .1, -.04, .2, .04)]
    assert not segment_is_clear(corners, cal, p(0., 0.), p(0., 0.), obstacle,
                                clearance_mm=116.1, camera_clearance_mm=0)
    report = check_pose(corners, cal, p(0., 0.), 0., obstacle)
    assert report['clear'] and report['penetration_mm'] == 0
    assert report['clearance_mm'] > 15
    assert len(report['footprint_px']) >= 4


def test_rotating_changes_real_hull_collision_without_circle_test():
    corners, cal, p = scene()
    obstacle = [rectangle(p, .1, -.04, .2, .04)]
    assert check_pose(corners, cal, p(0., 0.), 0., obstacle)['clear']
    assert not check_pose(corners, cal, p(0., 0.), math.pi / 2., obstacle)['clear']
    assert not check_swept_motion(corners, cal, p(0., 0.), 0., p(0., 0.), math.pi / 2., obstacle)['clear']


def test_direct_drive_is_merged_and_no_motors_are_needed():
    corners, cal, p = scene()
    start, goal = p(-.3, 0.), p(.3, 0.)
    result = plan_pose_path(corners, cal, start, goal, [], start_heading_rad=0.)
    assert result['path_length_m'] == pytest.approx(.6, abs=.0001)
    assert len(result['steps']) == 2
    assert all(step['kind'] == 'drive' and step['direction'] == 1 for step in result['steps'])
    assert validate_steps(corners, cal, start, 0., [], result) == pytest.approx(goal, abs=.01)


def test_goal_behind_requires_validated_half_turn_then_forward_on_same_centerline():
    corners, cal, p = scene()
    start, goal = p(0., 0.), p(-.25, 0.)
    result = plan_pose_path(corners, cal, start, goal, [], start_heading_rad=0.)
    assert [step['kind'] for step in result['steps']] == ['turn', 'turn', 'drive']
    assert all(step['direction'] == 1 for step in result['steps'])
    assert result['steps'][0]['target_px'] == pytest.approx(start, abs=.01)
    assert result['steps'][1]['target_px'] == pytest.approx(start, abs=.01)
    assert abs(result['steps'][0]['heading_rad']) == pytest.approx(math.pi / 2., abs=1e-6)
    assert abs(result['steps'][1]['heading_rad']) == pytest.approx(math.pi, abs=1e-6)
    assert np.asarray(result['path_px']) == pytest.approx(np.asarray([start, goal]), abs=.01)
    validate_steps(corners, cal, start, 0., [], result)


def test_rectangle_detour_is_fast_and_all_sweeps_are_valid():
    corners, cal, p = scene()
    obstacle = [rectangle(p, -.05, -.133333, .05, .133333)]
    start, goal = p(-.4, 0.), p(.266666, 0.)
    began = time.monotonic()
    result = plan_pose_path(corners, cal, start, goal, obstacle, start_heading_rad=0.)
    assert time.monotonic() - began < 3.
    assert result['path_length_m'] > .666
    assert len(result['steps']) < 15
    assert all(step['direction'] == 1 for step in result['steps'] if step['kind'] == 'drive')
    validate_steps(corners, cal, start, 0., obstacle, result)


@pytest.mark.parametrize('jitter_m', [-.0002, 0., .0002])
def test_free_corner_recovery_can_join_an_orthogonal_detour(jitter_m):
    corners, cal, p = scene()
    obstacles = [rectangle(p, -.18, .1, .18, .25)]
    start = p(-.01475246 + jitter_m, -.01242430)
    goal, heading = p(-.4, .3), math.radians(159.329)
    assert check_pose(corners, cal, start, heading, obstacles)['clear']
    began = time.monotonic()
    result = plan_pose_path(corners, cal, start, goal, obstacles,
                            start_heading_rad=heading)
    assert time.monotonic() - began < 1.
    assert any(step['kind'] == 'escape' and step['direction'] < 0
               for step in result['steps'])
    assert validate_steps(corners, cal, start, heading, obstacles, result) == pytest.approx(goal, abs=.01)
    # No existía contacto inicial: tampoco puede introducirlo el prefijo de
    # recuperación ni su unión con la ruta que rodea la barrera de frente.
    previous = start
    for step in result['steps']:
        assert check_swept_motion(corners, cal, previous, heading, step['target_px'],
                                  step['heading_rad'], obstacles)['clear']
        previous, heading = step['target_px'], step['heading_rad']


def test_marker_visibility_is_its_real_oriented_quad_not_an_84mm_circle():
    # El centro está a80mm del borde. El marcador va desplazado hacia dentro,
    # por lo que queda completo aunque el antiguo radio84mm lo rechazase.
    corners, cal, p = scene(center_offset=(-.045, .03))
    center = [48., 480.]
    report = check_pose(corners, cal, center, 0., [])
    assert report['marker_visible'] and report['clear']
    assert not check_pose(corners, cal, center, math.radians(225), [])['marker_visible']


def test_goal_80mm_from_border_can_be_reached_with_a_valid_final_heading():
    corners, cal, p = scene(center_offset=(-.045, .03))
    start, goal = [250., 480.], [48., 480.]
    result = plan_pose_path(corners, cal, start, goal, [], start_heading_rad=0.)
    validate_steps(corners, cal, start, 0., [], result)
    assert result['steps'][-1]['direction'] == 1
    assert all(step['direction'] == 1 for step in result['steps'] if step['kind'] == 'drive')


def test_real_calibrated_52mm_marker_carrier_is_part_of_collision_hull():
    corners, cal, p = scene(center_offset=(-.065, .055))
    # El portador queda fuera del cuerpo simulado y debe seguir siendo físico.
    report = check_pose(corners, cal, p(0., 0.), 0., [])
    points = np.array(report['footprint_px'])
    assert points[:, 0].max() >= 640. + (.065 + .026) * 600. - .01


def test_initial_20mm_contact_escapes_by_decreasing_penetration_in_reverse():
    corners, cal, p = scene()
    obstacle = [rectangle(p, .063, -.04, .2, .04)]
    start, goal = p(0., 0.), p(-.4, 0.)
    initial = check_pose(corners, cal, start, 0., obstacle)
    assert initial['penetration_mm'] > 20.
    result = plan_pose_path(corners, cal, start, goal, obstacle, start_heading_rad=0.)
    assert result['steps'][0]['kind'] == 'escape'
    assert result['steps'][0]['direction'] == -1
    validate_steps(corners, cal, start, 0., obstacle, result)
    assert check_pose(corners, cal, result['steps'][-1]['target_px'],
                      result['steps'][-1]['heading_rad'], obstacle)['clear']


def test_escape_never_allows_contact_to_deepen_or_a_new_object_to_be_crossed():
    corners, cal, p = scene()
    obstacle = [rectangle(p, .063, -.04, .2, .04)]
    start = p(0., 0.)
    assert check_swept_motion(corners, cal, start, 0., p(-.03, 0.), 0., obstacle,
                              allow_start_overlap=True)['clear']
    assert not check_swept_motion(corners, cal, start, 0., p(.03, 0.), 0., obstacle,
                                  allow_start_overlap=True)['clear']
    assert not check_swept_motion(corners, cal, start, 0., p(-.03, 0.), 0., obstacle)['clear']
    assert not check_swept_motion(corners, cal, start, 0., p(-.03, 0.), 0., obstacle,
                                  allow_start_overlap=True, max_penetration_mm=10)['clear']


def test_swept_translation_cannot_jump_over_a_thin_obstacle():
    corners, cal, p = scene()
    obstacle = [rectangle(p, -.002, -.02, .002, .02)]
    assert check_pose(corners, cal, p(-.2, 0.), 0., obstacle)['clear']
    assert check_pose(corners, cal, p(.2, 0.), 0., obstacle)['clear']
    assert not check_swept_motion(corners, cal, p(-.2, 0.), 0., p(.2, 0.), 0., obstacle)['clear']


def test_heading_error_uses_physical_angle_and_expected_sign():
    corners, cal, p = scene()
    assert heading_error_rad(corners, cal, math.pi / 2.) == pytest.approx(math.pi / 2., abs=1e-6)
    assert heading_error_rad(corners, cal, -math.pi / 2.) == pytest.approx(-math.pi / 2., abs=1e-6)
    assert heading_error_rad(corners, cal, 0.) == pytest.approx(0., abs=1e-6)


def test_wheel_motion_integrates_straight_and_turning_with_correct_sign():
    corners, cal, p = scene()
    straight = check_wheel_motion(corners, cal, 100., 100., .12, obstacles_px=[])
    assert straight['clear']
    assert straight['predicted_center_px'] == pytest.approx(p(.012, 0.), abs=.01)
    arc = check_wheel_motion(corners, cal, 50., 150., .12, obstacles_px=[])
    assert arc['clear']
    assert arc['predicted_center_px'][0] > 640
    assert arc['predicted_center_px'][1] < 480  # Más rueda derecha: gira a la izquierda.


def test_wheel_guard_rejects_forward_contact_but_permits_reverse_escape():
    corners, cal, p = scene()
    obstacle = [rectangle(p, .063, -.04, .2, .04)]
    assert not check_wheel_motion(corners, cal, 100., 100., obstacles_px=obstacle,
                                  allow_start_overlap=True)['clear']
    assert check_wheel_motion(corners, cal, -100., -100., obstacles_px=obstacle,
                              allow_start_overlap=True)['clear']


@pytest.mark.parametrize('bad', [True, -1., float('nan'), float('inf'), '3'])
def test_invalid_compute_budget_is_rejected(bad):
    corners, cal, p = scene()
    with pytest.raises(ValueError):
        plan_pose_path(corners, cal, p(0., 0.), p(.2, 0.), [], max_seconds=bad)


def test_collision_goal_is_not_accepted_as_an_escape():
    corners, cal, p = scene()
    obstacle = [rectangle(p, .15, -.1, .3, .1)]
    with pytest.raises(ValueError, match='destino'):
        plan_pose_path(corners, cal, p(0., 0.), p(.2, 0.), obstacle, start_heading_rad=0.)


@pytest.mark.parametrize('wall_y', [.100, .085, .080])
def test_head_on_close_wall_uses_short_reverse_before_detouring_in_under_one_second(wall_y):
    scale = 800.
    p = lambda x, y: [640. + scale * x, 480. - scale * y]
    corners = [p(-.02, .02), p(.02, .02), p(.02, -.02), p(-.02, -.02)]
    # Frente hacia arriba. La calibración expresa ese mismo rumbo en la base
    # del marcador, sin falsear su relación con el eje de las ruedas.
    cal = Calibration((0., 0.), math.pi / 2., 'synthetic-camera', 1280, 960)
    obstacle = [rectangle(p, -.18, wall_y, .18, .25)]
    start, goal = p(0., 0.), p(-.4, .3)
    initial = check_pose(corners, cal, start, math.pi / 2., obstacle)
    began = time.monotonic()
    result = plan_pose_path(corners, cal, start, goal, obstacle)
    assert time.monotonic() - began < 1.
    assert result['steps'][0]['direction'] == -1
    assert result['steps'][0]['kind'] == 'escape'  # Contacto o espacio insuficiente para girar.
    assert result['steps'][0]['target_px'][1] > start[1]
    previous, heading = start, math.pi / 2.
    reverse_distance = 0.
    normal_motion_seen = False
    for step in result['steps']:
        if step['kind'] == 'drive':
            normal_motion_seen = True
            assert step['direction'] == 1
        if step['direction'] < 0:
            assert not normal_motion_seen and step['kind'] == 'escape'
            reverse_distance += np.linalg.norm(np.array(step['target_px']) - previous) / scale
        checked = check_swept_motion(corners, cal, previous, heading, step['target_px'],
                                     step['heading_rad'], obstacle,
                                     allow_start_overlap=step.get('escaping', False))
        assert checked['clear'], checked
        previous, heading = step['target_px'], step['heading_rad']
    assert reverse_distance <= .080001
    assert previous == pytest.approx(goal, abs=.01)
    assert check_pose(corners, cal, previous, heading, obstacle)['clear']
