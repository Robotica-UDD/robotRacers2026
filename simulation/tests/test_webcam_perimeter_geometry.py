"""Paredes lineales cerradas con interior libre, sin cámara ni robot reales."""
import math
import time

import numpy as np
import pytest

from simulation.tests.test_webcam_obstacle_planner import scene, rectangle
from simulation.tests.test_webcam_pose_obstacle_planner import validate_steps
from simulation.webcam.obstacle_planner import (
    _PlaneMap, _grid, plan_path, point_clearances_mm, segment_is_clear,
    validate_obstacles,
)
from simulation.webcam.pose_obstacle_planner import (
    check_pose, check_swept_motion, plan_pose_path,
)


CONCAVE_PERIMETER = [(-.7, -.5), (.7, -.5), (.7, .5), (.15, .5),
                     (.15, .05), (-.15, .05), (-.15, .5), (-.7, .5)]


def perimeter(project, vertices=CONCAVE_PERIMETER):
    return [[project(first), project(last)]
            for first, last in zip(vertices, vertices[1:] + vertices[:1])]


def test_lines_are_normalized_without_filling_or_mutating_the_perimeter():
    _, calibration, project = scene()
    walls = perimeter(project)
    original = [[point[:] for point in line] for line in walls]
    result = validate_obstacles(walls, calibration.width, calibration.height)
    assert result == original and walls == original
    assert all(len(line) == 2 for line in result)
    result[0][0][0] += 1
    assert walls == original


def test_line_and_legacy_solid_limits_remain_independent():
    _, _, project = scene()
    line = [project((-.4, 0.)), project((.4, 0.))]
    solid = rectangle(project, -.1, -.1, .1, .1)
    assert len(validate_obstacles([line] * 512, 1280, 960)) == 512
    assert len(validate_obstacles([solid] * 32 + [line] * 480, 1280, 960)) == 512
    with pytest.raises(ValueError, match="512"):
        validate_obstacles([line] * 513, 1280, 960)
    with pytest.raises(ValueError, match="32"):
        validate_obstacles([solid] * 33, 1280, 960)


@pytest.mark.parametrize("line", [
    [[10., 10.], [10., 10.]],
    [[-1., 10.], [20., 10.]],
    [[10., 10.], [1280., 10.]],
    [[True, 10.], [20., 10.]],
    [[float("nan"), 10.], [20., 10.]],
])
def test_invalid_line_endpoints_are_rejected(line):
    with pytest.raises(ValueError):
        validate_obstacles([line], 1280, 960)


@pytest.mark.parametrize("scale,perspective", [(400., False), (800., False), (600., True)])
def test_line_thickness_is_one_metric_millimetre_with_square_joint_caps(scale, perspective):
    corners, calibration, project = scene(scale=scale, perspective=perspective)
    walls = [[project((-.4, 0.)), project((.4, 0.))]]
    plane = _PlaneMap.build(corners, calibration, walls)
    polygon = plane.obstacles[0]
    lengths = sorted(np.linalg.norm(np.roll(polygon, -1, axis=0) - polygon, axis=1))
    assert lengths[:2] == pytest.approx([.001, .001], abs=1e-9)
    first, last = (plane.world(point) for point in walls[0])
    assert lengths[2:] == pytest.approx([np.linalg.norm(last - first) + .001] * 2, abs=1e-9)
    delta, offset = last - first, plane.world(project((0., .1))) - first
    centerline_distance = abs(delta[0] * offset[1] - delta[1] * offset[0]) / np.linalg.norm(delta)
    assert point_clearances_mm(corners, calibration, project((0., .1)), walls)[
        "obstacle_mm"] == pytest.approx(centerline_distance * 1000. - .5, abs=.01)
    assert len(plane.line_segments) == 1
    assert plane.line_segments[0] == pytest.approx(np.asarray([first, last]))


def test_concave_boundary_keeps_interior_and_exterior_free_as_separate_regions():
    corners, calibration, project = scene()
    walls = perimeter(project)
    for point in ((-.45, .2), (.45, .2), (0., -.25), (0., .3)):
        assert point_clearances_mm(corners, calibration, project(point), walls)["obstacle_mm"] > 100.
    # Cruzar la pared que separa el brazo izquierdo y la concavidad exterior
    # queda bloqueado, aunque ambos extremos sean puntos libres.
    assert not segment_is_clear(corners, calibration, project((-.45, .2)),
        project((0., .2)), walls, clearance_mm=0, camera_clearance_mm=0)


def test_every_boundary_segment_blocks_crossing_including_the_closing_edge():
    corners, calibration, project = scene()
    walls = perimeter(project)
    for first, last in zip(CONCAVE_PERIMETER, CONCAVE_PERIMETER[1:] + CONCAVE_PERIMETER[:1]):
        first, last = np.asarray(first), np.asarray(last)
        delta = last - first
        normal = np.array([-delta[1], delta[0]]) / np.linalg.norm(delta)
        middle = (first + last) / 2.
        assert not segment_is_clear(corners, calibration,
            project(middle + normal * .03), project(middle - normal * .03), walls,
            clearance_mm=0, camera_clearance_mm=0)


def test_robot_can_follow_a_valid_path_inside_a_concave_perimeter():
    corners, calibration, project = scene()
    walls = perimeter(project)
    start, goal = project((-.45, .2)), project((.45, .2))
    result = plan_pose_path(corners, calibration, start, goal, walls, start_heading_rad=0.)
    assert result["path_length_m"] > .9
    assert validate_steps(corners, calibration, start, 0., walls, result) == pytest.approx(goal, abs=.01)
    assert any(point[1] > 480. for point in result["path_px"])


def test_planning_cannot_escape_a_closed_perimeter_into_its_free_exterior():
    corners, calibration, project = scene()
    walls = perimeter(project)
    with pytest.raises(ValueError, match="camino libre"):
        plan_path(corners, calibration, project((-.45, .2)), project((0., .3)), walls,
            robot_radius_mm=60, safety_margin_mm=0, tracking_margin_mm=0,
            camera_clearance_mm=0)


def test_swept_robot_cannot_jump_over_the_closing_wall():
    corners, calibration, project = scene()
    vertices = [(-.7, -.45), (.7, -.45), (.7, .45), (-.7, .45)]
    walls = perimeter(project, vertices)
    start, end = project((-.5, 0.)), project((-.9, 0.))
    assert check_pose(corners, calibration, start, math.pi, walls)["clear"]
    assert check_pose(corners, calibration, end, math.pi, walls)["clear"]
    assert not check_swept_motion(corners, calibration, start, math.pi,
        end, math.pi, walls)["clear"]


def test_grid_with_512_short_walls_stays_within_the_planning_budget():
    corners, calibration, project = scene()
    vertices = [(.4 * math.cos(i * 2. * math.pi / 512),
                 .4 * math.sin(i * 2. * math.pi / 512)) for i in range(512)]
    plane = _PlaneMap.build(corners, calibration, perimeter(project, vertices))
    blocked, minimum, step, _ = _grid(plane, .06, time.monotonic() + 2., 0.)
    center = np.rint((plane.world(project((0., 0.))) - minimum) / step).astype(int)
    assert not blocked[center[1], center[0]]
