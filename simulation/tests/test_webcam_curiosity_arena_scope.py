"""Alcance de curiosidad en un recinto extenso, sin cámara, red ni motores.

La proyección sintética mantiene el marcador por encima del mínimo geométrico
de 4 píxeles. Estas pruebas separan el alcance métrico del comportamiento de
la resolución y fiabilidad reales del detector de imagen.
"""
import cv2
import numpy as np
import pytest

from simulation.tests.test_webcam_curiosity_behavior import CuriousNavigation
from simulation.tests.test_webcam_home_behavior import Clock
from simulation.tests.test_webcam_perimeter_geometry import perimeter
from simulation.tests.test_webcam_point_navigation import FakeCamera
from simulation.webcam.home_behavior import HomeBehavior
from simulation.webcam.interest_goal import choose_interest_point
from simulation.webcam.navigation_geometry import Calibration
from simulation.webcam.pose_obstacle_planner import (
    _Scene, check_pose, check_swept_motion, plan_pose_path,
)
from simulation.webcam.robot_footprint import ROBOT_FOOTPRINT_MM


WIDTH, HEIGHT, SCALE = 4096, 2160, 240.
ARENA = [(-1.5, -4.), (12.5, -4.), (12.5, 4.), (-1.5, 4.)]


def arena_scene():
    def project(point):
        return [512. + SCALE * point[0], 1080. - SCALE * point[1]]
    corners = [project(point) for point in
               [(-.02, .02), (.02, .02), (.02, -.02), (-.02, -.02)]]
    calibration = Calibration((0., 0.), 0., 'usb-camera', WIDTH, HEIGHT)
    walls = perimeter(project, ARENA)
    return corners, calibration, project, walls


def hand_at(project, x, y=0., *, half_width=.04, palm=None):
    upper_left = project((x - half_width, y + .04))
    lower_right = project((x + half_width, y - .04))
    return {'bbox_px': [*upper_left, *lower_right],
            'palm_px': project((x, y) if palm is None else palm), 'confidence': 1.}


@pytest.fixture
def arena_rig():
    corners, calibration, project, walls = arena_scene()
    camera, clock = FakeCamera(), Clock()
    camera.update(running=True, opening=False, frame_width=WIDTH, frame_height=HEIGHT,
                  marker_corners=corners)
    navigation = CuriousNavigation(camera)
    navigation._calibration = calibration
    navigation._obstacles = walls
    navigation.value['obstacles_px'] = walls
    behavior = HomeBehavior(camera, navigation, clock=clock)
    behavior.set_home(*project((-.5, 0.)), WIDTH, HEIGHT, 'usb-camera')
    behavior.set_enabled(True)
    yield camera, navigation, behavior, clock, project, walls
    behavior.close()


def observe_for_two_seconds(rig, hand):
    for sequence in range(1, 12):
        if sequence > 1:
            rig[3].advance(.2)
        state = rig[2].observe({'ready': True, 'status': 'ready', 'model': 'MediaPipe Hands',
            'error': None, 'captured_at': rig[3].now, 'sequence': sequence,
            'frame_width': WIDTH, 'frame_height': HEIGHT, 'camera_id': 'usb-camera',
            'hands': [hand]})
    return state


@pytest.mark.parametrize('distance_m', [3., 5., 10.])
def test_visible_hand_inside_large_arena_confirms_after_two_seconds_without_distance_ceiling(arena_rig, distance_m):
    hand = hand_at(arena_rig[4], distance_m)
    state = observe_for_two_seconds(arena_rig, hand)
    assert distance_m * 1000. - 200. < state['nearest_hand_mm'] < distance_m * 1000.
    assert state['far_hand_seconds'] == 2. and state['curious']
    assert state['curiosity_attempt_count'] == state['curiosity_count'] == 1
    assert len(arena_rig[1].approaches) == 1 and not arena_rig[1].returns
    assert arena_rig[1].approaches[0][2] == {
        'frame_width': WIDTH, 'frame_height': HEIGHT, 'camera_id': 'usb-camera'}
    # El doble recibe la intención; no se abre ninguna sesión física.
    assert arena_rig[1].starts == 0 and not arena_rig[1].robot.drives


@pytest.mark.parametrize('distance_m', [3., 5., 10.])
def test_far_hand_gets_a_nearby_interior_goal_with_a_checked_path(distance_m):
    corners, calibration, project, walls = arena_scene()
    hand = hand_at(project, distance_m)
    result = choose_interest_point(corners, calibration, hand['bbox_px'], walls, 100.)
    scene = _Scene(corners, calibration, walls, ROBOT_FOOTPRINT_MM)
    target = result['target_px']
    origin = scene.plane.world(project((0., 0.)))
    world_target = scene.plane.world(target) - origin
    assert distance_m - .5 < world_target[0] < distance_m
    assert cv2.pointPolygonTest(np.asarray(ARENA, np.float32),
                               tuple(map(float, world_target)), False) > 0
    assert check_pose(corners, calibration, target, 0., walls)['clear']
    assert check_swept_motion(corners, calibration, project((0., 0.)), 0.,
                             target, 0., walls)['clear']
    plan = plan_pose_path(corners, calibration, project((0., 0.)), target, walls,
                          start_heading_rad=0., max_seconds=.8)
    assert plan['path_px'][-1] == pytest.approx(target)
    assert all(cv2.pointPolygonTest(np.asarray(ARENA, np.float32),
        tuple(map(float, scene.plane.world(point) - origin)), False) > 0 for point in plan['path_px'])


def test_closed_arena_rejects_crossing_the_boundary_even_when_endpoint_is_visible():
    corners, calibration, project, walls = arena_scene()
    outside = project((13.2, 0.))
    assert 0. < outside[0] < WIDTH and 0. < outside[1] < HEIGHT
    assert check_pose(corners, calibration, outside, 0., walls)['clear']
    assert not check_swept_motion(corners, calibration, project((0., 0.)), 0.,
                                 outside, 0., walls)['clear']


@pytest.mark.parametrize('distance_m,half_width', [(13.2, .04), (12.52, .08)])
def test_hand_palm_outside_robot_arena_never_starts_or_accumulates_curiosity(arena_rig, distance_m, half_width):
    state = observe_for_two_seconds(arena_rig,
        hand_at(arena_rig[4], distance_m, half_width=half_width))
    assert state['enabled'] and not state['near_hand']
    assert not state['curious'] and state['far_hand_seconds'] == 0.
    assert state['curiosity_status'] == 'outside_arena'
    assert state['curiosity_attempt_count'] == state['curiosity_count'] == 0
    assert not arena_rig[1].approaches and not arena_rig[1].returns


@pytest.mark.parametrize('box_center,palm', [(12.48, None), (12.52, (12.48, 0.))])
def test_hand_palm_inside_is_eligible_even_when_its_box_overlaps_the_boundary(arena_rig, box_center, palm):
    hand = hand_at(arena_rig[4], box_center, half_width=.08, palm=palm)
    assert hand['bbox_px'][2] > arena_rig[4]((12.5, 0.))[0]
    state = observe_for_two_seconds(arena_rig, hand)
    assert state['curious'] and state['far_hand_seconds'] == 2.
    assert state['curiosity_attempt_count'] == state['curiosity_count'] == 1
    assert len(arena_rig[1].approaches) == 1


def test_without_perimeter_a_hand_elsewhere_in_the_image_remains_eligible(arena_rig):
    arena_rig[1]._obstacles = []
    arena_rig[1].value['obstacles_px'] = []
    state = observe_for_two_seconds(arena_rig, hand_at(arena_rig[4], 13.2))
    assert state['curious'] and state['far_hand_seconds'] == 2.
    assert len(arena_rig[1].approaches) == 1


def test_nearby_hand_outside_perimeter_still_has_fear_priority(arena_rig):
    project = arena_rig[4]
    walls = perimeter(project, [(-.13, -4.), (12.5, -4.), (12.5, 4.), (-.13, 4.)])
    arena_rig[1]._obstacles = walls
    arena_rig[1].value['obstacles_px'] = walls
    arena_rig[2].set_home(*project((.5, 0.)), WIDTH, HEIGHT, 'usb-camera')
    arena_rig[2].set_enabled(True)
    state = observe_for_two_seconds(arena_rig, hand_at(project, -.20))
    assert state['near_hand'] and state['returning']
    assert len(arena_rig[1].returns) == 1 and not arena_rig[1].approaches
    assert state['curiosity_count'] == 0


@pytest.mark.parametrize('fear_mm,hand_x', [(150., .29), (450., .62)])
def test_curiosity_has_no_extra_distance_gap_beyond_the_fear_zone(arena_rig, fear_mm, hand_x):
    arena_rig[2].set_distance(fear_mm)
    state = observe_for_two_seconds(arena_rig, hand_at(arena_rig[4], hand_x))
    assert fear_mm < state['nearest_hand_mm'] < fear_mm + 50.
    assert state['curiosity_min_distance_mm'] == state['curiosity_distance_mm'] == fear_mm
    assert state['curiosity_max_distance_mm'] is None
    assert state['far_hand_seconds'] == 2. and state['curious']
    assert len(arena_rig[1].approaches) == 1 and not arena_rig[1].returns


def test_manual_button_stop_prevents_even_a_valid_far_hand_from_rearming(arena_rig):
    arena_rig[1].robot.update(stop_reason='button_stop', resume_blocked=True)
    state = observe_for_two_seconds(arena_rig, hand_at(arena_rig[4], 10.))
    assert not state['enabled'] and not state['curious']
    assert state['curiosity_count'] == state['curiosity_attempt_count'] == 0
    assert not arena_rig[1].approaches and not arena_rig[1].returns


@pytest.mark.parametrize('point,expected', [([80., 80.], True), ([80., 50.], False)])
def test_other_arm_of_a_concave_perimeter_remains_the_same_arena(point, expected):
    from simulation.webcam.perimeter_scope import same_perimeter_region
    c_shape = [[10., 10.], [90., 10.], [90., 30.], [30., 30.],
               [30., 70.], [90., 70.], [90., 90.], [10., 90.]]
    walls = [[first, last] for first, last in zip(c_shape, c_shape[1:] + c_shape[:1])]
    assert same_perimeter_region(point, [20., 20.], walls) is expected


@pytest.mark.parametrize('nested,point,expected', [
    (False, [250., 50.], False),
    (False, [80., 80.], True),
    (True, [50., 50.], False),
    (True, [80., 80.], True),
])
def test_disjoint_and_nested_perimeters_do_not_merge_separate_regions(nested, point, expected):
    from simulation.webcam.perimeter_scope import same_perimeter_region
    outer = [[0., 0.], [100., 0.], [100., 100.], [0., 100.]]
    other = ([[30., 30.], [70., 30.], [70., 70.], [30., 70.]] if nested else
             [[200., 0.], [300., 0.], [300., 100.], [200., 100.]])
    walls = [[first, last] for ring in (outer, other)
             for first, last in zip(ring, ring[1:] + ring[:1])]
    assert same_perimeter_region(point, [20., 20.], walls) is expected


def test_perimeter_edge_is_eligible_and_open_walls_do_not_invent_a_closed_arena():
    from simulation.webcam.perimeter_scope import same_perimeter_region
    ring = [[0., 0.], [100., 0.], [100., 100.], [0., 100.]]
    walls = [[first, last] for first, last in zip(ring, ring[1:] + ring[:1])]
    assert same_perimeter_region([100., 50.], [20., 20.], walls)
    assert same_perimeter_region([150., 50.], [20., 20.], walls[:-1])
