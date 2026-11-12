"""Presupuesto del acercamiento geométrico, sin cámara ni robot reales."""
import math
import time
from types import SimpleNamespace

import pytest

from simulation.tests.test_webcam_curiosity_navigation import HAND, STAND_OFF_MM
from simulation.tests.test_webcam_obstacle_planner import scene
from simulation.tests.test_webcam_perimeter_geometry import perimeter
from simulation.tests.test_webcam_pose_navigation import projected_obstacles, synthetic_scene
from simulation.webcam import interest_goal
from simulation.webcam.pose_obstacle_planner import _Scene, check_pose, check_swept_motion, plan_pose_path
from simulation.webcam.robot_footprint import ROBOT_FOOTPRINT_MM


def test_expired_geometry_budget_does_not_publish_an_unchecked_target(monkeypatch):
    ticks = iter([100.])
    clock = SimpleNamespace(monotonic=lambda: next(ticks, 100.81))
    barrier = [[(.100, -.060), (.140, -.060), (.140, .060), (.100, .060)]]
    with synthetic_scene(start=(0., 0., 0.), polygons=[]) as (camera, _, navigation):
        monkeypatch.setattr(interest_goal, "time", clock)
        with pytest.raises(ValueError, match="acercamiento|punto libre"):
            interest_goal.choose_interest_point(camera.state()["marker_corners"],
                navigation._calibration, HAND, projected_obstacles(barrier), STAND_OFF_MM)


def test_hand_outside_a_closed_perimeter_chooses_a_reachable_point_inside():
    corners, calibration, project = scene()
    walls = perimeter(project, [(-.35, -.5), (.35, -.5), (.35, .5), (-.35, .5)])
    first, last = project((.85, .03)), project((.95, -.03))
    box = [first[0], first[1], last[0], last[1]]
    result = interest_goal.choose_interest_point(corners, calibration, box, walls, 100.)
    # Los primeros anillos están fuera de la región del robot. No gastar los
    # 800 ms completos en ellos: elegir el acercamiento alcanzable más próximo
    # de los anillos restantes, conservando la pared y la huella del robot.
    assert project((.20, 0.))[0] < result["target_px"][0] < project((.35, 0.))[0]
    # La selección del extremo no sustituye esta aprobación independiente de
    # la trayectoria que efectúa Navigation antes de permitir movimiento.
    target = result["target_px"]
    plan = plan_pose_path(corners, calibration, project((0., 0.)), target, walls,
                          start_heading_rad=0., max_seconds=.5)
    assert check_pose(corners, calibration, target,
        plan["steps"][-1]["heading_rad"], walls)["clear"]


def test_perimeter_search_can_use_more_than_350_mm_of_additional_separation():
    corners, calibration, project = scene(scale=400.)
    walls = perimeter(project, [(-.35, -.5), (.35, -.5), (.35, .5), (-.35, .5)])
    first, last = project((1.4, .03)), project((1.5, -.03))
    result = interest_goal.choose_interest_point(corners, calibration,
        [first[0], first[1], last[0], last[1]], walls, 100.)
    # La mano queda más de un metro fuera de la pared. El destino debe seguir
    # acercando al robot dentro de su región, sin un radio extra fijo de 350 mm.
    assert project((.2, 0.))[0] < result["target_px"][0] < project((.35, 0.))[0]
    assert result["stand_off_mm"] == 100.


def test_target_more_than_one_metre_from_the_robot_has_no_fixed_distance_limit():
    corners, calibration, project = scene(scale=400.)
    first, last = project((1.4, .03)), project((1.5, -.03))
    result = interest_goal.choose_interest_point(corners, calibration,
        [first[0], first[1], last[0], last[1]], [], 100.)
    assert result["target_px"][0] - project((0., 0.))[0] > 400.


def test_endpoint_with_turning_space_is_preferred_to_a_reverse_only_slot():
    corners, calibration, project = scene()
    walls = perimeter(project, [(-.35, -.5), (.35, -.5), (.35, .5), (-.35, .5)])
    first, last = project((.62, .05)), project((.70, -.05))
    result = interest_goal.choose_interest_point(corners, calibration,
        [first[0], first[1], last[0], last[1]], walls, 100.)
    target = result["target_px"]
    world = _Scene(corners, calibration, walls, ROBOT_FOOTPRINT_MM)
    point = world.plane.world(target)
    assert world.plane.point_clearances(point)[1] >= world.radius
    assert check_swept_motion(corners, calibration, target, 0.,
        target, math.pi, walls)["clear"]
    # La primera ranura orientada estaba en x≈279 mm y solo admitía un rumbo
    # hacia atrás. La alternativa conserva el progreso, con espacio al llegar.
    assert project((.15, 0.))[0] < target[0] < project((.25, 0.))[0]


def test_turning_circle_is_a_preference_and_never_excludes_a_narrow_corridor():
    corners, calibration, project = scene()
    walls = perimeter(project, [(-.95, -.09), (.95, -.09), (.95, .09), (-.95, .09)])
    first, last = project((.62, .05)), project((.70, -.05))
    result = interest_goal.choose_interest_point(corners, calibration,
        [first[0], first[1], last[0], last[1]], walls, 100.)
    target = result["target_px"]
    world = _Scene(corners, calibration, walls, ROBOT_FOOTPRINT_MM)
    point = world.plane.world(target)
    assert world.plane.point_clearances(point)[1] < world.radius
    assert check_pose(corners, calibration, target, math.pi / 2., walls)["clear"]


def test_component_filter_can_anchor_a_center_close_to_a_wall_without_crossing_it():
    corners, calibration, project = scene()
    walls = perimeter(project, [(-.35, -.5), (.35, -.5), (.35, .5), (-.35, .5)])
    world = _Scene(corners, calibration, walls, ROBOT_FOOTPRINT_MM)
    reachable = interest_goal._reachable_component(world, project((.349, 0.)), time.monotonic() + 1.)
    assert callable(reachable)
    assert reachable(project((.2, 0.)))
    assert not reachable(project((.5, 0.)))
    # Un punto sobre la pared no se rechaza por falta de ancla de cuadrícula;
    # seguirá teniendo que superar la comprobación exacta de poses.
    assert reachable(project((.35, 0.)))


def test_component_filter_leaves_an_unanchorable_start_to_the_exact_planner():
    corners, calibration, project = scene()
    walls = perimeter(project, [(-.35, -.5), (.35, -.5), (.35, .5), (-.35, .5)])
    world = _Scene(corners, calibration, walls, ROBOT_FOOTPRINT_MM)
    assert interest_goal._reachable_component(world, project((.35, 0.)), time.monotonic() + 1.) is None
