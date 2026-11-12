"""Presupuesto del acercamiento geométrico, sin cámara ni robot reales."""
import time
from types import SimpleNamespace

import pytest

from simulation.tests.test_webcam_curiosity_navigation import HAND, STAND_OFF_MM
from simulation.tests.test_webcam_obstacle_planner import scene
from simulation.tests.test_webcam_perimeter_geometry import perimeter
from simulation.tests.test_webcam_pose_navigation import projected_obstacles, synthetic_scene
from simulation.webcam import interest_goal
from simulation.webcam.pose_obstacle_planner import _Scene, check_pose
from simulation.webcam.robot_footprint import ROBOT_FOOTPRINT_MM


def test_nearly_spent_budget_never_requests_an_invalid_planner_window(monkeypatch):
    # El planificador admite como mínimo 50 ms. Con solo 40 ms restantes no
    # debe recibir una duración inválida ni consumir otro presupuesto completo.
    ticks = iter([100.])
    clock = SimpleNamespace(monotonic=lambda: next(ticks, 100.76))
    calls = []

    def planner(*args, **kwargs):
        calls.append(kwargs["max_seconds"])
        assert kwargs["max_seconds"] >= .05
        return {}

    barrier = [[(.100, -.060), (.140, -.060), (.140, .060), (.100, .060)]]
    with synthetic_scene(start=(0., 0., 0.), polygons=[]) as (camera, _, navigation):
        monkeypatch.setattr(interest_goal, "time", clock)
        monkeypatch.setattr(interest_goal, "plan_pose_path", planner)
        with pytest.raises(ValueError, match="acercamiento|punto libre"):
            interest_goal.choose_interest_point(camera.state()["marker_corners"],
                navigation._calibration, HAND, projected_obstacles(barrier), STAND_OFF_MM)
        assert calls == []


def test_hand_outside_a_closed_perimeter_chooses_a_reachable_point_inside(monkeypatch):
    corners, calibration, project = scene()
    walls = perimeter(project, [(-.35, -.5), (.35, -.5), (.35, .5), (-.35, .5)])
    first, last = project((.85, .03)), project((.95, -.03))
    box = [first[0], first[1], last[0], last[1]]
    plans = []
    planner = interest_goal.plan_pose_path

    def recorded(*args, **kwargs):
        result = planner(*args, **kwargs)
        plans.append((list(args[3]), result))
        return result

    monkeypatch.setattr(interest_goal, "plan_pose_path", recorded)
    result = interest_goal.choose_interest_point(corners, calibration, box, walls, 100.)
    # Los primeros anillos están fuera de la región del robot. No gastar los
    # 800 ms completos en ellos: elegir el acercamiento alcanzable más próximo
    # de los anillos restantes, conservando la pared y la huella del robot.
    assert project((.20, 0.))[0] < result["target_px"][0] < project((.35, 0.))[0]
    assert 1 <= len(plans) <= 2
    target, plan = plans[-1]
    assert target == pytest.approx(result["target_px"])
    assert check_pose(corners, calibration, target,
        plan["steps"][-1]["heading_rad"], walls)["clear"]


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
